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

#include "data/airport_overrides.hpp"
#include "core/logging.hpp"

#include <json.hpp>

#include <cctype>
#include <cmath>
#include <fstream>
#include <unordered_map>
#include <vector>

namespace airport_overrides {

namespace {

struct ApproachRule {
  std::string runway;    // "" = any
  std::string designator;
  float min_vis_m = -1.0f, max_vis_m = -1.0f;
  float min_ceiling_ft = -1.0f, max_ceiling_ft = -1.0f;
  float min_rvr_m = -1.0f; // touchdown RVR floor (metres) for this approach
};

struct RunwayRule {
  std::string arrival, departure;
  float max_tailwind_kt = -1.0f;
  float wind_min_dir = -1.0f, wind_max_dir = -1.0f;
};

std::unordered_map<std::string, std::vector<ApproachRule>> s_approaches;
std::unordered_map<std::string, std::vector<RunwayRule>> s_runways;
bool s_ready = false;

// Tailwind component (kt, +ve = tailwind) on a runway given the wind. Runway heading
// from the leading digits (04L -> 040). tailwind = -headwind = -V*cos(wind_from - hdg).
float tailwind_kt(const std::string &rwy, float wind_dir, float wind_speed) {
  int num = 0;
  for (char c : rwy) {
    if (std::isdigit(static_cast<unsigned char>(c)))
      num = num * 10 + (c - '0');
    else
      break;
  }
  const float hdg = static_cast<float>(num * 10);
  const float d = (wind_dir - hdg) * 3.14159265358979f / 180.0f;
  return -wind_speed * std::cos(d);
}

// wind_dir within [lo, hi] on the compass, wrap-aware (e.g. 340..020 spans north).
bool wind_in_range(float wdir, float lo, float hi) {
  wdir = std::fmod(wdir + 360.0f, 360.0f);
  lo = std::fmod(lo + 360.0f, 360.0f);
  hi = std::fmod(hi + 360.0f, 360.0f);
  return (lo <= hi) ? (wdir >= lo && wdir <= hi) : (wdir >= lo || wdir <= hi);
}

// Does `rule` allow `rwy` for the current wind? Wind-range gate + tailwind cap on THIS
// runway; a rule with neither gate always passes (the unconditional fallback).
bool rule_allows(const RunwayRule &r, const std::string &rwy, float wdir,
                 float wspd) {
  if (rwy.empty())
    return false;
  if (r.wind_min_dir >= 0.0f && r.wind_max_dir >= 0.0f &&
      !wind_in_range(wdir, r.wind_min_dir, r.wind_max_dir))
    return false;
  if (r.max_tailwind_kt >= 0.0f &&
      tailwind_kt(rwy, wdir, wspd) > r.max_tailwind_kt)
    return false;
  return true;
}

std::string upper(std::string s) {
  for (char &c : s)
    c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
  return s;
}

// Read a numeric field if present; leaves `out` untouched when absent/not a number.
void read_num(const nlohmann::json &obj, const char *key, float &out) {
  auto it = obj.find(key);
  if (it != obj.end() && it->is_number())
    out = it->get<float>();
}

} // namespace

void init(std::string path) {
  s_approaches.clear();
  s_runways.clear();
  s_ready = false;
  if (path.empty())
    return;

  std::ifstream in(path);
  if (!in.good()) {
    logging::info("airport_overrides: file not found (%s)", path.c_str());
    s_ready = true; // "loaded, empty" -- lookups just return ""
    return;
  }

  nlohmann::json root;
  try {
    in >> root;
  } catch (const std::exception &e) {
    logging::info("airport_overrides: JSON parse error (%s): %s", path.c_str(),
                  e.what());
    s_ready = true;
    return;
  }

  int n_appr = 0, n_rwy = 0;
  for (auto it = root.begin(); it != root.end(); ++it) {
    const std::string key = it.key();
    if (key.empty() || key[0] == '_' || !it->is_object())
      continue; // skip _comment / _schema
    const std::string icao = upper(key);

    // approaches (weather-gated preferred approach designator)
    if (auto ap = it->find("approaches"); ap != it->end() && ap->is_array()) {
      std::vector<ApproachRule> rules;
      for (const auto &entry : *ap) {
        if (!entry.is_object())
          continue;
        ApproachRule r;
        if (auto d = entry.find("designator");
            d != entry.end() && d->is_string())
          r.designator = d->get<std::string>();
        if (r.designator.empty())
          continue; // a rule with no designator is useless
        if (auto rw = entry.find("runway"); rw != entry.end() && rw->is_string())
          r.runway = upper(rw->get<std::string>());
        read_num(entry, "min_vis_m", r.min_vis_m);
        read_num(entry, "max_vis_m", r.max_vis_m);
        read_num(entry, "min_ceiling_ft", r.min_ceiling_ft);
        read_num(entry, "max_ceiling_ft", r.max_ceiling_ft);
        read_num(entry, "min_rvr_m", r.min_rvr_m);
        rules.push_back(std::move(r));
        ++n_appr;
      }
      if (!rules.empty())
        s_approaches[icao] = std::move(rules);
    }

    // runway_config (wind-gated arrival/departure runway, incl. L/R split)
    if (auto rc = it->find("runway_config"); rc != it->end() && rc->is_array()) {
      std::vector<RunwayRule> rules;
      for (const auto &entry : *rc) {
        if (!entry.is_object())
          continue;
        RunwayRule r;
        if (auto a = entry.find("arrival"); a != entry.end() && a->is_string())
          r.arrival = upper(a->get<std::string>());
        if (auto dp = entry.find("departure"); dp != entry.end() && dp->is_string())
          r.departure = upper(dp->get<std::string>());
        if (r.arrival.empty() && r.departure.empty())
          continue;
        read_num(entry, "max_tailwind_kt", r.max_tailwind_kt);
        read_num(entry, "wind_min_dir", r.wind_min_dir);
        read_num(entry, "wind_max_dir", r.wind_max_dir);
        rules.push_back(std::move(r));
        ++n_rwy;
      }
      if (!rules.empty())
        s_runways[icao] = std::move(rules);
    }
  }

  logging::info(
      "airport_overrides: %d approach + %d runway rules (%zu / %zu airports) from %s",
      n_appr, n_rwy, s_approaches.size(), s_runways.size(), path.c_str());
  s_ready = true;
}

void stop() {
  s_approaches.clear();
  s_runways.clear();
  s_ready = false;
}

bool ready() { return s_ready; }

float rvr_for_runway(const std::string &metar, const std::string &runway) {
  if (metar.empty() || runway.empty())
    return -1.0f;
  const std::string rwy = upper(runway);
  // METAR RVR group: "R<rwy>/<val>" e.g. R04L/0600, R25/P2000, R04L/M0050,
  // R04L/0550V0800, optional trailing trend (U/D/N) and/or "FT" (US, feet).
  size_t pos = metar.find("R" + rwy + "/");
  if (pos == std::string::npos && std::isalpha((unsigned char)rwy.back()))
    pos = metar.find("R" + rwy.substr(0, rwy.size() - 1) + "/"); // drop L/C/R
  if (pos == std::string::npos)
    return -1.0f;
  const size_t slash = metar.find('/', pos);
  const size_t end = metar.find(' ', slash);
  const std::string tok =
      metar.substr(slash + 1, (end == std::string::npos ? metar.size() : end) -
                                  slash - 1);
  const bool feet = tok.find("FT") != std::string::npos;
  auto parse_num = [](const std::string &s) -> int {
    size_t i = 0;
    if (i < s.size() && (s[i] == 'P' || s[i] == 'M'))
      ++i; // "more/less than" prefix
    int v = 0;
    bool any = false;
    for (; i < s.size() && std::isdigit((unsigned char)s[i]); ++i) {
      v = v * 10 + (s[i] - '0');
      any = true;
    }
    return any ? v : -1;
  };
  int val;
  const size_t vpos = tok.find('V'); // variable range -> take the lower value
  if (vpos != std::string::npos) {
    const int a = parse_num(tok.substr(0, vpos));
    const int b = parse_num(tok.substr(vpos + 1));
    val = (a >= 0 && b >= 0) ? (a < b ? a : b) : (a >= 0 ? a : b);
  } else {
    val = parse_num(tok);
  }
  if (val < 0)
    return -1.0f;
  return feet ? static_cast<float>(val) * 0.3048f : static_cast<float>(val);
}

std::string preferred_approach(const std::string &icao, const std::string &runway,
                               float visibility_m, float ceiling_ft) {
  if (!s_ready || icao.empty())
    return {};
  auto it = s_approaches.find(upper(icao));
  if (it == s_approaches.end())
    return {};
  const std::string rwy = upper(runway);
  // Change-guarded diagnostic: log the weather-gated approach decision ONCE per
  // outcome change (this is called every descent frame). So the pilot can see WHY
  // a preferred approach (e.g. LFMN 04L RNAV Alpha) was ruled out -- vis or ceiling
  // -- rather than silently getting the fallback (Zulu). (LFMN 2026-07-19.)
  static std::string s_last_key;
  const std::string key = upper(icao) + "|" + rwy;
  for (const ApproachRule &r : it->second) {
    if (!r.runway.empty() && !rwy.empty() && r.runway != rwy)
      continue;
    const char *why = nullptr;
    if (r.min_vis_m >= 0.0f && visibility_m < r.min_vis_m)
      why = "vis-below-min";
    else if (r.max_vis_m >= 0.0f && visibility_m > r.max_vis_m)
      why = "vis-above-max";
    else if (r.min_ceiling_ft >= 0.0f && ceiling_ft < r.min_ceiling_ft)
      why = "ceiling-below-min";
    else if (r.max_ceiling_ft >= 0.0f && ceiling_ft > r.max_ceiling_ft)
      why = "ceiling-above-max";
    if (why) {
      const std::string log_key = key + "|out|" + r.designator + "|" + why;
      if (log_key != s_last_key) {
        s_last_key = log_key;
        logging::info("airport_overrides: %s %s -> %s RULED OUT (%s: vis=%.0fm "
                      "[min %.0f], ceiling=%.0fft [min %.0f])",
                      upper(icao).c_str(), rwy.c_str(), r.designator.c_str(), why,
                      visibility_m, r.min_vis_m, ceiling_ft, r.min_ceiling_ft);
      }
      continue;
    }
    const std::string log_key = key + "|use|" + r.designator;
    if (log_key != s_last_key) {
      s_last_key = log_key;
      logging::info("airport_overrides: %s %s -> %s SELECTED (vis=%.0fm, "
                    "ceiling=%.0fft)",
                    upper(icao).c_str(), rwy.c_str(), r.designator.c_str(),
                    visibility_m, ceiling_ft);
    }
    return r.designator; // first match wins
  }
  return {};
}

std::string arrival_runway(const std::string &icao, float wind_dir,
                           float wind_speed) {
  if (!s_ready || icao.empty())
    return {};
  auto it = s_runways.find(upper(icao));
  if (it == s_runways.end())
    return {};
  for (const RunwayRule &r : it->second)
    if (!r.arrival.empty() && rule_allows(r, r.arrival, wind_dir, wind_speed))
      return r.arrival;
  return {};
}

std::string departure_runway(const std::string &icao, float wind_dir,
                             float wind_speed) {
  if (!s_ready || icao.empty())
    return {};
  auto it = s_runways.find(upper(icao));
  if (it == s_runways.end())
    return {};
  for (const RunwayRule &r : it->second)
    if (!r.departure.empty() && rule_allows(r, r.departure, wind_dir, wind_speed))
      return r.departure;
  return {};
}

} // namespace airport_overrides
