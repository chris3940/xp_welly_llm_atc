/*
 * xp_wellys_atc - AI-powered ATC voice communication for X-Plane 12
 * Copyright (C) 2026 thWelly & Claude (Anthropic)
 * Copyright (C) 2026 Christopher P. Potter (Linux port + IFR extensions)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

#include "data/openair_db.hpp"

#include "data/airspace_db.hpp"
#include "core/logging.hpp"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace openair_db {

namespace {

struct Entry {
  std::string name;
  AirspaceClass ac_class = AirspaceClass::OTHER;
  int floor_ft = 0;
  int ceiling_ft = 0;
  std::uint32_t freq_khz = 0; // openair "AF" field (overlay-carried controller freq)
  // Bounding box for fast rejection.
  double bbox_min_lat = 0.0, bbox_max_lat = 0.0;
  double bbox_min_lon = 0.0, bbox_max_lon = 0.0;
  double bbox_area = 0.0; // (lat span) * (lon span) — proxy for polygon size
  std::vector<std::pair<double, double>> polygon; // (lat, lon) pairs
};

// Point-in-polygon via ray casting.
static bool
point_in_polygon(double lat, double lon,
                 const std::vector<std::pair<double, double>> &poly) {
  if (poly.size() < 3)
    return false;
  bool inside = false;
  std::size_t n = poly.size();
  for (std::size_t i = 0, j = n - 1; i < n; j = i++) {
    double xi = poly[i].second, yi = poly[i].first;
    double xj = poly[j].second, yj = poly[j].first;
    if (((yi > lat) != (yj > lat)) &&
        (lon < (xj - xi) * (lat - yi) / (yj - yi) + xi))
      inside = !inside;
  }
  return inside;
}

// Parse "DD:MM:SS N DDD:MM:SS E" or "DD:MM:SS S DDD:MM:SS W".
static bool parse_dp(const char *s, double &lat, double &lon) {
  int latd, latm, lond, lonm;
  double lats, lons;
  char latdir[4] = {}, londir[4] = {};
  // Fixed-format coordinate parse; the returned field count (8) is checked.
  // NOLINTNEXTLINE(bugprone-unchecked-string-to-number-conversion)
  if (std::sscanf(s, " %d:%d:%lf %3s %d:%d:%lf %3s", &latd, &latm, &lats,
                  latdir, &lond, &lonm, &lons, londir) != 8)
    return false;
  lat = latd + latm / 60.0 + lats / 3600.0;
  if (latdir[0] == 'S')
    lat = -lat;
  lon = lond + lonm / 60.0 + lons / 3600.0;
  if (londir[0] == 'W')
    lon = -lon;
  return true;
}

// Parse "AH/AL <value>" — supports "FL095", "4000 MSL", "2500 AGL",
// "GND", "SFC", "UNLIM", plain integers.
// Note: "AGL" values are stored as-is (MSL not known at parse time);
// callers that need MSL must add airport elevation themselves.
static int parse_alt(const char *val) {
  while (*val == ' ')
    ++val;
  if (std::strncmp(val, "FL", 2) == 0)
    return static_cast<int>(std::strtol(val + 2, nullptr, 10)) * 100;
  if (std::strncmp(val, "GND", 3) == 0 || std::strncmp(val, "SFC", 3) == 0)
    return 0;
  if (std::strncmp(val, "UNLIM", 5) == 0)
    return 99999;
  return static_cast<int>(std::strtol(val, nullptr, 10));
}

static AirspaceClass parse_class(const char *s) {
  if (std::strcmp(s, "CTR") == 0)
    return AirspaceClass::CTR;
  if (std::strcmp(s, "TMA") == 0)
    return AirspaceClass::TMA;
  if (std::strcmp(s, "CTA") == 0)
    return AirspaceClass::CTA;
  if (std::strcmp(s, "FIR") == 0)
    return AirspaceClass::FIR;
  if (std::strcmp(s, "UIR") == 0)
    return AirspaceClass::UIR;
  // ICAO CLASS LETTER. Most European exports put the class in AC and the type in
  // the name, and until now only the name was consulted -- so any controlled
  // volume whose name carries no type word was invisible. Measured on the user's
  // export: `FREE RT ASPC E` (Reims UIR over Nancy) and `DORTMUND` / `DORTMUND
  // SECTOR B` at EDLW are all controlled airspace and none were indexed, which
  // is what let a terminal-area descent fire 186 NM out on 2026-08-14.
  //
  // A-D are controlled; E/F/G are not indexed (E is advisory-to-uncontrolled
  // depending on the state, and neither F nor G carries an ATC clearance
  // obligation). Classified CTA -- the enroute default -- so a name keyword can
  // still refine it to CTR/TMA/FIR in the AN handler below.
  //
  // R / P / Q / W (restricted, prohibited, danger, warning) deliberately fall
  // through to OTHER: they are not control airspace and must never resolve to a
  // controller.
  if (std::strcmp(s, "A") == 0 || std::strcmp(s, "B") == 0 ||
      std::strcmp(s, "C") == 0 || std::strcmp(s, "D") == 0)
    return AirspaceClass::CTA;
  return AirspaceClass::OTHER;
}

// Only index these classes — skip restricted/prohibited/danger areas.
static bool is_indexed(AirspaceClass c) {
  return c == AirspaceClass::CTR || c == AirspaceClass::TMA ||
         c == AirspaceClass::CTA || c == AirspaceClass::FIR ||
         c == AirspaceClass::UIR;
}

std::vector<Entry> s_entries;
std::atomic<bool> s_ready{false};

static std::vector<Entry> load_file(const std::string &path) {
  std::vector<Entry> entries;
  FILE *f = std::fopen(path.c_str(), "r");
  if (!f) {
    logging::info("openair_db: file not found (%s)", path.c_str());
    return entries;
  }

  bool active = false;
  // True when cur.ac_class came from an ICAO class LETTER (AC C / AC D / ...)
  // rather than an explicit type token (AC TMA / AC CTR / ...). A letter says
  // how the airspace is REGULATED, not what it IS, so the name is still allowed
  // to refine it below. Without this distinction, teaching parse_class to read
  // class letters silently demoted every letter-named TMA to a plain CTA --
  // terminal_tma() then returned nothing and the whole terminal-descent path
  // went dead. Caught by tests/test_openair_db.cpp before it ever flew.
  bool class_from_letter = false;
  Entry cur;

  char line[512];
  while (std::fgets(line, sizeof(line), f)) {
    std::size_t len = std::strlen(line);
    while (len > 0 && (line[len - 1] == '\n' || line[len - 1] == '\r' ||
                       line[len - 1] == ' '))
      line[--len] = '\0';
    if (len == 0 || line[0] == '*')
      continue;

    if (std::strncmp(line, "AC ", 3) == 0) {
      if (active && cur.polygon.size() >= 3) {
        cur.bbox_area = (cur.bbox_max_lat - cur.bbox_min_lat) *
                        (cur.bbox_max_lon - cur.bbox_min_lon);
        entries.push_back(std::move(cur));
      }
      cur = Entry{};
      const char *ac = line + 3;
      cur.ac_class = parse_class(ac);
      class_from_letter = (std::strlen(ac) == 1);
      active = is_indexed(cur.ac_class);
      continue;
    }

    // AN: always process so we can upgrade letter-class entries (AC C, AC D,
    // etc.) to the correct type when the name contains "TMA", "CTR", or "CTA".
    // Many European airspace files use the ICAO class letter in AC rather than
    // the airspace type — e.g. French TMA zones appear as "AC D AN ... TMA ...".
    if (std::strncmp(line, "AN ", 3) == 0) {
      cur.name = line + 3;
      if (cur.ac_class == AirspaceClass::OTHER || class_from_letter) {
        const std::string &n = cur.name;
        // The last branch assigns the same class as the "CTA" branch, but the two
        // MUST stay separate and in this order: a delegation polygon whose name
        // also carries a type word ("DELEGATED BY LIMM TO LJLA FIR/UIR") has to be
        // classified by that word, so the FIR test has to run first. Merging the
        // conditions to satisfy bugprone-branch-clone would silently reclassify
        // such overlays as CTA. Suppressed rather than "fixed".
        // NOLINTBEGIN(bugprone-branch-clone)
        if (n.find("CTR") != std::string::npos)
          cur.ac_class = AirspaceClass::CTR;
        else if (n.find("TMA") != std::string::npos)
          cur.ac_class = AirspaceClass::TMA;
        else if (n.find("CTA") != std::string::npos)
          cur.ac_class = AirspaceClass::CTA;
        else if (n.find("FIR") != std::string::npos)
          cur.ac_class = AirspaceClass::FIR;
        // Cross-border delegation polygon (overlay): named by delegation marker,
        // not by "CTA"/"FIR" type, and often "AC C" (ICAO class C). Index as an
        // enroute CTA so find_enclosing returns it and resolve_sector_controller
        // routes it to the delegated ACC.
        else if (n.find("DELEGATED") != std::string::npos ||
                 n.find("SKYGUIDE") != std::string::npos ||
                 n.find("MUAC") != std::string::npos)
          cur.ac_class = AirspaceClass::CTA;
        // NOLINTEND(bugprone-branch-clone)
        active = is_indexed(cur.ac_class);
      }
      continue;
    }

    if (!active)
      continue;

    if (std::strncmp(line, "AH ", 3) == 0) {
      cur.ceiling_ft = parse_alt(line + 3);
    } else if (std::strncmp(line, "AL ", 3) == 0) {
      cur.floor_ft = parse_alt(line + 3);
    } else if (std::strncmp(line, "AF ", 3) == 0) {
      // "AF <MHz>" -- controller frequency carried by the overlay (e.g.
      // "AF 125.155"). Standard OpenAir has no ATC-freq field; this lets a
      // hand-maintained sector (missing from the vendor export, e.g. LYON TMA
      // SECTOR 2.2) name its OWN frequency so resolve_sector_controller speaks it
      // directly instead of falling back to the atc.dat nearest ACC (Marseille).
      // [C. P. Potter]
      cur.freq_khz = static_cast<std::uint32_t>(
          std::lround(std::atof(line + 3) * 1000.0));
    } else if (std::strncmp(line, "DP ", 3) == 0) {
      double lat, lon;
      if (parse_dp(line + 3, lat, lon)) {
        if (cur.polygon.empty()) {
          cur.bbox_min_lat = cur.bbox_max_lat = lat;
          cur.bbox_min_lon = cur.bbox_max_lon = lon;
        } else {
          if (lat < cur.bbox_min_lat)
            cur.bbox_min_lat = lat;
          if (lat > cur.bbox_max_lat)
            cur.bbox_max_lat = lat;
          if (lon < cur.bbox_min_lon)
            cur.bbox_min_lon = lon;
          if (lon > cur.bbox_max_lon)
            cur.bbox_max_lon = lon;
        }
        cur.polygon.emplace_back(lat, lon);
      }
    }
    // DA / DB arc records skipped — surrounding DP points approximate arcs.
  }
  if (active && cur.polygon.size() >= 3) {
    cur.bbox_area = (cur.bbox_max_lat - cur.bbox_min_lat) *
                    (cur.bbox_max_lon - cur.bbox_min_lon);
    entries.push_back(std::move(cur));
  }

  std::fclose(f);
  logging::info("openair_db: parsed %zu entries from %s", entries.size(),
                path.c_str());
  return entries;
}

// Compose base + optional overlay. Overlay entries are appended and WIN on a
// same-name collision (a hand-maintained polygon replaces the vendor one).
static void load(const std::string &base, const std::string &overlay) {
  std::vector<Entry> entries = load_file(base);
  if (!overlay.empty()) {
    std::vector<Entry> ov = load_file(overlay);
    if (!ov.empty()) {
      for (const auto &o : ov)
        entries.erase(
            std::remove_if(entries.begin(), entries.end(),
                           [&](const Entry &e) { return e.name == o.name; }),
            entries.end());
      for (auto &o : ov)
        entries.push_back(std::move(o));
      logging::info(
          "openair_db: +%zu overlay entries from %s (overlay wins on name)",
          ov.size(), overlay.c_str());
    }
  }
  s_entries = std::move(entries);
  logging::info("openair_db: %zu airspace entries indexed", s_entries.size());
  s_ready = true;
}

std::thread s_thread;

} // namespace

void init(std::string path, std::string overlay_path) {
  if (path.empty()) {
    s_ready = true;
    return;
  }
  s_thread = std::thread(
      [b = std::move(path), o = std::move(overlay_path)]() { load(b, o); });
}

void stop() {
  if (s_thread.joinable())
    s_thread.join();
  s_entries.clear();
  s_ready = false;
}

bool ready() { return s_ready.load(); }


// ── atc.dat fallback ──────────────────────────────────────────────────────
// The OpenAir airspace file ships with a PAID navdata subscription; atc.dat comes with
// a standard X-Plane install. Without a fallback, a user without that subscription has
// no volumes at all -- no sector handoffs, no TMA logic, no descend-to-enter -- which
// is the entire IFR core of the plugin (user, 2026-08-15).
//
// Precedence is strict and one-way: OpenAir stays AUTHORITATIVE wherever it answers.
// atc.dat is consulted ONLY when OpenAir is absent or silent for that point, so an
// installed subscription behaves exactly as before, byte for byte.
//
// atc.dat carries the same shelved geometry (the Duesseldorf tracon alone: 24 polygons,
// floors 1500-8500 under one 10000 ft ceiling), so the mapping is a role translation
// plus the ring that actually contains the point. [C. P. Potter]
namespace {

AirspaceClass class_from_role(airspace_db::ControllerRole r) {
  switch (r) {
  case airspace_db::ControllerRole::TWR:
    return AirspaceClass::CTR;
  case airspace_db::ControllerRole::TRACON:
    return AirspaceClass::TMA;
  case airspace_db::ControllerRole::CTR:
    return AirspaceClass::CTA;
  default:
    return AirspaceClass::OTHER;
  }
}

double bbox_area_of(const airspace_db::Controller &c) {
  if (!c.has_bbox)
    return 1e18; // no box -> never wins the innermost test
  return (c.bbox_max_lat - c.bbox_min_lat) * (c.bbox_max_lon - c.bbox_min_lon);
}

// Innermost enclosing atc.dat volume, mapped to an AirspaceEntry. want_tma restricts
// the search to TRACON records (the terminal_tma() query). Empty entry when nothing
// encloses the point.
AirspaceEntry atc_dat_entry(double lat, double lon, int alt_ft, bool want_tma) {
  if (!airspace_db::ready())
    return {};
  const auto hits =
      airspace_db::find_enclosing(lat, lon, static_cast<float>(alt_ft));
  const airspace_db::Controller *best = nullptr;
  double best_area = 1e18;
  for (const auto *c : hits) {
    if (c == nullptr)
      continue;
    if (want_tma && c->role != airspace_db::ControllerRole::TRACON)
      continue;
    const double a = bbox_area_of(*c);
    if (best == nullptr || a < best_area) {
      best = c;
      best_area = a;
    }
  }
  if (best == nullptr)
    return {};
  AirspaceEntry out;
  out.name = best->name;
  out.ac_class = class_from_role(best->role);
  out.floor_ft = best->floor_ft;
  out.ceiling_ft = best->ceiling_ft;
  int f = 0, ceil = 0;
  if (airspace_db::enclosing_ring_extent(*best, lat, lon,
                                         static_cast<float>(alt_ft), &f, &ceil)) {
    out.floor_ft = f;      // the shelf the aircraft is actually in, not the
    out.ceiling_ft = ceil; // record's first polygon
  }
  if (!best->freqs_khz.empty())
    out.freq_khz = best->freqs_khz.front();
  return out;
}

} // namespace

AirspaceEntry find_enclosing(double lat, double lon, int alt_ft) {
  if (!s_ready)
    return atc_dat_entry(lat, lon, alt_ft, /*want_tma=*/false);
  const Entry *best = nullptr;
  for (const auto &e : s_entries) {
    // Bounding-box fast reject.
    if (lat < e.bbox_min_lat || lat > e.bbox_max_lat)
      continue;
    if (lon < e.bbox_min_lon || lon > e.bbox_max_lon)
      continue;
    // Altitude range check.
    if (alt_ft < e.floor_ft)
      continue;
    if (e.ceiling_ft > 0 && alt_ft > e.ceiling_ft)
      continue;
    // Full polygon check.
    if (!point_in_polygon(lat, lon, e.polygon))
      continue;
    // Pick the innermost (smallest bbox area).
    if (!best || e.bbox_area < best->bbox_area)
      best = &e;
  }
  if (!best)
    return atc_dat_entry(lat, lon, alt_ft, /*want_tma=*/false);
  return {best->name, best->ac_class, best->floor_ft, best->ceiling_ft,
          best->freq_khz};
}

std::vector<AirspaceEntry> find_all_enclosing(double lat, double lon,
                                               int alt_ft) {
  std::vector<AirspaceEntry> result;
  if (!s_ready)
    return result;
  for (const auto &e : s_entries) {
    if (lat < e.bbox_min_lat || lat > e.bbox_max_lat)
      continue;
    if (lon < e.bbox_min_lon || lon > e.bbox_max_lon)
      continue;
    if (alt_ft < e.floor_ft)
      continue;
    if (e.ceiling_ft > 0 && alt_ft > e.ceiling_ft)
      continue;
    if (!point_in_polygon(lat, lon, e.polygon))
      continue;
    result.push_back({e.name, e.ac_class, e.floor_ft, e.ceiling_ft, e.freq_khz});
  }
  return result;
}

int ctr_ceiling_ft(double lat, double lon) {
  if (!s_ready)
    return 0;
  for (const auto &e : s_entries) {
    if (e.ac_class != AirspaceClass::CTR)
      continue;
    if (lat < e.bbox_min_lat || lat > e.bbox_max_lat)
      continue;
    if (lon < e.bbox_min_lon || lon > e.bbox_max_lon)
      continue;
    if (point_in_polygon(lat, lon, e.polygon))
      return e.ceiling_ft;
  }
  return 0;
}

// NOTE: terminal_tma() deliberately does NOT fall back to atc.dat. Measured on the
// DIK -> EDLW replay (2026-08-15): dest_terminal_tma_below() probes the AIRCRAFT's
// position as well as the destination's, and with a fallback atc.dat answered there
// with a NEIGHBOURING terminal area (ceiling 10000 ft) which the guard then mistook
// for the destination's -- firing "descend-to-enter terminal area -> FL090" 63 NM
// out, on top of the ladder's FL100 twenty seconds earlier. Silence is the correct
// answer for that guard: it reads "no volume" as "can't tell" and stays permissive.
// find_enclosing() keeps the fallback, which is where the value is (sector handoffs
// and the enclosing-volume queries). [C. P. Potter]
AirspaceEntry terminal_tma(double lat, double lon) {
  if (!s_ready)
    return {};
  int best_floor = 1000000000; // base (lowest-floor) TMA over the point
  const Entry *best = nullptr; // ties on floor break to the higher ceiling
  for (const auto &e : s_entries) {
    if (e.ac_class != AirspaceClass::TMA)
      continue;
    if (lat < e.bbox_min_lat || lat > e.bbox_max_lat)
      continue;
    if (lon < e.bbox_min_lon || lon > e.bbox_max_lon)
      continue;
    if (e.floor_ft > best_floor) // can't be a lower base -- skip the poly test
      continue;
    if (!point_in_polygon(lat, lon, e.polygon))
      continue;
    if (best == nullptr || e.floor_ft < best_floor ||
        (e.floor_ft == best_floor && e.ceiling_ft > best->ceiling_ft)) {
      best_floor = e.floor_ft;
      best = &e;
    }
  }
  if (best == nullptr)
    return {}; // see the note above terminal_tma(): no atc.dat fallback here
  return {best->name, best->ac_class, best->floor_ft, best->ceiling_ft,
          best->freq_khz};
}

int terminal_tma_ceiling(double lat, double lon) {
  return terminal_tma(lat, lon).ceiling_ft;
}

int highest_tma_ceiling(double lat, double lon) {
  if (!s_ready)
    return 0;
  int max_ceil = 0;
  for (const auto &e : s_entries) {
    if (e.ac_class != AirspaceClass::TMA)
      continue;
    if (lat < e.bbox_min_lat || lat > e.bbox_max_lat)
      continue;
    if (lon < e.bbox_min_lon || lon > e.bbox_max_lon)
      continue;
    if (e.ceiling_ft <= max_ceil) // can't raise the max -- skip the poly test
      continue;
    if (!point_in_polygon(lat, lon, e.polygon))
      continue;
    max_ceil = e.ceiling_ft;
  }
  return max_ceil;
}

int descend_to_enter_ceiling(double acft_lat, double acft_lon, double dest_lat,
                             double dest_lon) {
  if (!s_ready)
    return 0;
  const int ref_ceil = terminal_tma_ceiling(dest_lat, dest_lon);
  if (ref_ceil <= 0)
    return 0; // no terminal TMA at the destination (AFIS / CTR-only field)
  // Aircraft laterally inside a TMA whose ceiling matches the destination's
  // terminal ceiling -> it is over the destination terminal area.
  for (const auto &e : s_entries) {
    if (e.ac_class != AirspaceClass::TMA || e.ceiling_ft != ref_ceil)
      continue;
    if (acft_lat < e.bbox_min_lat || acft_lat > e.bbox_max_lat)
      continue;
    if (acft_lon < e.bbox_min_lon || acft_lon > e.bbox_max_lon)
      continue;
    if (point_in_polygon(acft_lat, acft_lon, e.polygon))
      return ref_ceil;
  }
  return 0;
}

} // namespace openair_db
