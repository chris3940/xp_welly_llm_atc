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

// Per-airport SID climb hold: a local procedure NOT derivable from CIFP/airspace
// (the initial-climb alt comes from the CIFP; these are the FL/distance the field
// keeps you low under an adjacent TMA). hold_alt_ft held until release_nm
// (great-circle from departure), then the general climb rules take over
// (hand off to the upper controller, who clears the next step, step2_alt_ft).
// Matched by the SID's terminating fix (match_fixes; empty = any SID here).
struct DepartureHold {
  std::vector<std::string> match_fixes; // SID last-fix idents; empty = any
  int   hold_alt_ft  = 0;
  float release_nm   = 0.0f;
  int   step2_alt_ft = 0; // 0 = no explicit second step
};

// Published SID initial-climb clearance altitude (a CHART annotation, NOT in the CIFP):
// the altitude ATC clears the aircraft to in the pre-departure clearance ("climb FLxxx").
// jet_alt_ft / prop_alt_ft optionally split by aircraft type; alt_ft = single value.
struct SidInitialClimb {
  std::vector<std::string> match_sids; // SID names (UPPER); empty = any SID from this field
  int alt_ft = 0;       // single value for all types
  int jet_alt_ft = 0;   // optional jet override (takes precedence for jets)
  int prop_alt_ft = 0;  // optional turboprop/prop override
};

// Delegated / override controller: a facility that atc.dat / apt.dat cannot
// resolve correctly (absent, or wrong frequency). role is normalised UPPER.
struct ControllerOverride {
  std::string role; // "APPROACH","DEPARTURE","TOWER","GROUND","DELIVERY","ATIS","INFO"
  std::string name; // spoken label, e.g. "Milan Radar"
  float freq_mhz = 0.0f;
};

std::unordered_map<std::string, std::vector<ApproachRule>> s_approaches;
std::unordered_map<std::string, std::vector<RunwayRule>> s_runways;
std::unordered_map<std::string, std::vector<DepartureHold>> s_dep_holds;
std::unordered_map<std::string, std::vector<ControllerOverride>> s_controllers;
// Per-approach override of the Approach->Tower handoff trigger fix (replaces the
// FAF). icao -> designator(UPPER) -> handoff-fix ident(UPPER). For curved RNP
// approaches whose FAF sits far out (LOWI RNP 08: FAF WI749 ~28 NM from RW08), so
// the aircraft is "established on final" only at a late last-turn fix; the FAF-based
// Tower handoff would fire far too early. [C. P. Potter]
std::unordered_map<std::string, std::unordered_map<std::string, std::string>>
    s_tower_handoff_fixes;
std::unordered_map<std::string, std::vector<SidInitialClimb>> s_sid_initial_climb;
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

void init(const std::string &path) {
  s_approaches.clear();
  s_runways.clear();
  s_dep_holds.clear();
  s_controllers.clear();
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
    const std::string &key = it.key();
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

    // departure_holds (SID climb: hold hold_alt_ft until release_nm, then general)
    if (auto dh = it->find("departure_holds"); dh != it->end() && dh->is_array()) {
      std::vector<DepartureHold> holds;
      for (const auto &entry : *dh) {
        if (!entry.is_object())
          continue;
        DepartureHold h;
        float alt = 0.0f, rel = 0.0f, s2 = 0.0f;
        read_num(entry, "hold_alt_ft", alt);
        read_num(entry, "release_nm", rel);
        read_num(entry, "step2_alt_ft", s2);
        h.hold_alt_ft = static_cast<int>(alt);
        h.release_nm = rel;
        h.step2_alt_ft = static_cast<int>(s2);
        if (auto mf = entry.find("match_fixes"); mf != entry.end() && mf->is_array())
          for (const auto &f : *mf)
            if (f.is_string())
              h.match_fixes.push_back(upper(f.get<std::string>()));
        if (h.hold_alt_ft > 0)
          holds.push_back(std::move(h));
      }
      if (!holds.empty())
        s_dep_holds[icao] = std::move(holds);
    }

    // controllers (delegated/override facility name + freq, e.g. LIMF -> Milan Radar)
    if (auto cs = it->find("controllers"); cs != it->end() && cs->is_array()) {
      std::vector<ControllerOverride> ctrls;
      for (const auto &entry : *cs) {
        if (!entry.is_object())
          continue;
        ControllerOverride c;
        if (auto r = entry.find("role"); r != entry.end() && r->is_string())
          c.role = upper(r->get<std::string>());
        if (auto nm = entry.find("name"); nm != entry.end() && nm->is_string())
          c.name = nm->get<std::string>();
        float f = 0.0f;
        read_num(entry, "freq_mhz", f);
        c.freq_mhz = f;
        if (!c.role.empty() && !c.name.empty())
          ctrls.push_back(std::move(c));
      }
      if (!ctrls.empty())
        s_controllers[icao] = std::move(ctrls);
    }

    // tower_handoff_fixes (per-designator override of the Approach->Tower handoff
    // trigger, replacing the FAF -- for curved RNP finals; LOWI RNP 08 -> WI754).
    if (auto th = it->find("tower_handoff_fixes");
        th != it->end() && th->is_object()) {
      std::unordered_map<std::string, std::string> m;
      for (auto jt = th->begin(); jt != th->end(); ++jt) {
        if (jt.key().empty() || jt.key()[0] == '_') // skip _comment
          continue;
        if (jt.value().is_string() && !jt.value().get<std::string>().empty())
          m[upper(jt.key())] = upper(jt.value().get<std::string>());
      }
      if (!m.empty())
        s_tower_handoff_fixes[icao] = std::move(m);
    }

    // sid_initial_climb (published initial-climb clearance alt, a chart annotation)
    if (auto sic = it->find("sid_initial_climb");
        sic != it->end() && sic->is_array()) {
      std::vector<SidInitialClimb> rules;
      for (const auto &entry : *sic) {
        if (!entry.is_object())
          continue;
        SidInitialClimb r;
        float a = 0.0f, j = 0.0f, p = 0.0f;
        read_num(entry, "alt_ft", a);
        read_num(entry, "jet_alt_ft", j);
        read_num(entry, "prop_alt_ft", p);
        r.alt_ft = static_cast<int>(a);
        r.jet_alt_ft = static_cast<int>(j);
        r.prop_alt_ft = static_cast<int>(p);
        if (auto ms = entry.find("match_sids"); ms != entry.end() && ms->is_array())
          for (const auto &s : *ms)
            if (s.is_string())
              r.match_sids.push_back(upper(s.get<std::string>()));
        if (r.alt_ft > 0 || r.jet_alt_ft > 0 || r.prop_alt_ft > 0)
          rules.push_back(std::move(r));
      }
      if (!rules.empty())
        s_sid_initial_climb[icao] = std::move(rules);
    }
  }

  logging::info("airport_overrides: %d approach + %d runway rules + %zu dep-hold "
                "airports (%zu / %zu appr/rwy airports) from %s",
                n_appr, n_rwy, s_dep_holds.size(), s_approaches.size(),
                s_runways.size(), path.c_str());
  s_ready = true;
}

void stop() {
  s_approaches.clear();
  s_runways.clear();
  s_dep_holds.clear();
  s_controllers.clear();
  s_tower_handoff_fixes.clear();
  s_sid_initial_climb.clear();
  s_ready = false;
}

// Published SID initial-climb clearance altitude (feet) for icao + sid_name, chosen by
// aircraft type: is_jet -> jet_alt_ft; else prop_alt_ft; else the single alt_ft (with the
// set jet/prop value as a last-resort fallback). First rule whose match_sids contains
// sid_name wins (empty match_sids = any SID). 0 = no override -> caller keeps the CIFP /
// generic initial climb. [C. P. Potter]
int sid_initial_climb_ft(const std::string &icao, const std::string &sid_name,
                         bool is_jet) {
  if (!s_ready || icao.empty())
    return 0;
  auto it = s_sid_initial_climb.find(upper(icao));
  if (it == s_sid_initial_climb.end())
    return 0;
  const std::string want = upper(sid_name);
  for (const SidInitialClimb &r : it->second) {
    bool match = r.match_sids.empty();
    for (const auto &m : r.match_sids)
      if (m == want) { match = true; break; }
    if (!match)
      continue;
    if (is_jet && r.jet_alt_ft > 0)
      return r.jet_alt_ft;
    if (!is_jet && r.prop_alt_ft > 0)
      return r.prop_alt_ft;
    if (r.alt_ft > 0)
      return r.alt_ft;
    if (r.jet_alt_ft > 0)
      return r.jet_alt_ft; // only jet set -> use it
    if (r.prop_alt_ft > 0)
      return r.prop_alt_ft;
  }
  return 0;
}

// Per-approach Tower-handoff override fix (replaces the FAF trigger). Empty when
// no override -> caller keeps the FAF-based handoff. [C. P. Potter]
std::string tower_handoff_fix(const std::string &icao,
                              const std::string &designator) {
  if (!s_ready || icao.empty() || designator.empty())
    return {};
  auto it = s_tower_handoff_fixes.find(upper(icao));
  if (it == s_tower_handoff_fixes.end())
    return {};
  auto jt = it->second.find(upper(designator));
  return jt == it->second.end() ? std::string{} : jt->second;
}

bool controller(const std::string &icao, const std::string &role,
                std::string *out_name, float *out_freq_mhz) {
  if (!s_ready || icao.empty() || role.empty())
    return false;
  auto it = s_controllers.find(upper(icao));
  if (it == s_controllers.end())
    return false;
  const std::string want = upper(role);
  for (const ControllerOverride &c : it->second) {
    if (c.role != want)
      continue;
    if (out_name)
      *out_name = c.name;
    if (out_freq_mhz)
      *out_freq_mhz = c.freq_mhz;
    return true;
  }
  return false;
}

bool ready() { return s_ready; }

bool departure_hold(const std::string &icao, const std::string &sid_last_fix,
                    int *hold_alt_ft, float *release_nm, int *step2_alt_ft) {
  if (!s_ready || icao.empty())
    return false;
  auto it = s_dep_holds.find(upper(icao));
  if (it == s_dep_holds.end())
    return false;
  const std::string fix = upper(sid_last_fix);
  for (const DepartureHold &h : it->second) {
    if (!h.match_fixes.empty()) {
      bool matched = false;
      for (const auto &f : h.match_fixes)
        if (f == fix) {
          matched = true;
          break;
        }
      if (!matched)
        continue; // rule is fix-scoped and this SID's last fix is not listed
    }
    if (hold_alt_ft && h.hold_alt_ft > 0)
      *hold_alt_ft = h.hold_alt_ft;
    if (release_nm && h.release_nm > 0.0f)
      *release_nm = h.release_nm;
    if (step2_alt_ft && h.step2_alt_ft > 0)
      *step2_alt_ft = h.step2_alt_ft;
    logging::info("airport_overrides: %s SID last-fix '%s' -> dep-hold FL%d "
                  "release %.0f NM step2 FL%d",
                  upper(icao).c_str(), fix.c_str(), h.hold_alt_ft / 100,
                  h.release_nm, h.step2_alt_ft / 100);
    return true;
  }
  return false;
}

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
