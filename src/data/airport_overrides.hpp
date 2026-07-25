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

#ifndef DATA_AIRPORT_OVERRIDES_HPP
#define DATA_AIRPORT_OVERRIDES_HPP

#include <string>

// Per-airport, hand-maintained overrides loaded from
// <plugin>/Resources/airport+.json. Supplements Navigraph/apt.dat/atc.dat with
// data those sources cannot express: a weather-gated PREFERRED APPROACH (e.g.
// LFMN 04L -> R04LA when visibility is good, else R04LZ), the runway-in-use
// config, and delegated controllers. Only the preferred-approach lookup is wired
// so far (v4.4.0); runway_config + controllers are parsed-but-unused for now.
namespace airport_overrides {

// Load and parse airport+.json. Empty path or missing file -> disabled (every
// lookup returns empty). Runs on the caller thread (the file is tiny).
void init(std::string path);
void stop();
bool ready();

// Preferred approach DESIGNATOR (e.g. "R04LA") for icao + landing runway under the
// current weather, or "" when no override matches (caller then falls back to its
// own heuristic). Rules are evaluated in file order; the first whose runway matches
// (case-insensitive; an empty rule-runway matches any) AND whose weather gates all
// hold wins. Gates (any subset): min_vis_m / max_vis_m (metres),
// min_ceiling_ft / max_ceiling_ft. A gate that is absent is not tested.
std::string preferred_approach(const std::string &icao, const std::string &runway,
                               float visibility_m, float ceiling_ft);

// Touchdown RVR (metres) for `runway` parsed from a raw METAR, or -1 when the
// METAR reports no RVR group for that runway. RVR is SEPARATE from prevailing
// visibility -- it is NOT used for the preferred-approach gate (that keys on
// visibility); this is the data source for approach minima and the future
// "ATC reads the RVR in low visibility" phraseology. Accepts runway with or
// without the L/C/R suffix; handles P/M prefixes, V (variable, lower value used),
// and "FT" (US, feet).
float rvr_for_runway(const std::string &metar, const std::string &runway);

// Arrival / departure runway-in-use for icao under the current wind, from the
// runway_config list. Rules are evaluated in file order; the first whose gates hold
// wins -- a wind-direction range (wind_min_dir..wind_max_dir, with wrap) AND a
// tailwind cap (max_tailwind_kt) ON THE RELEVANT RUNWAY (arrival vs departure judged
// INDEPENDENTLY). A rule with no gate always matches (the fallback). Returns "" when
// there is no config or no match -> the caller keeps its own wind-based pick.
// wind_dir in degrees (FROM), wind_speed in kt. This resolves the L/R split too
// (e.g. LFMN arrive 04L / depart 04R) which wind alone cannot.
std::string arrival_runway(const std::string &icao, float wind_dir,
                           float wind_speed);
std::string departure_runway(const std::string &icao, float wind_dir,
                             float wind_speed);

// SID climb hold override for `icao` whose SID terminates at `sid_last_fix`.
// Returns true and fills the (non-null) out-params from the first departure_holds
// rule whose match_fixes contains sid_last_fix (empty match_fixes = any SID from
// this field). A rule that omits release_nm / step2_alt_ft leaves those out-params
// untouched. Returns false when no override matches -> the caller keeps its generic
// data-driven / FL110 default. hold_alt_ft = step-1 hold level (feet); release_nm =
// great-circle hold distance; step2_alt_ft = optional explicit second step (feet).
// These are LOCAL procedures not derivable from CIFP/airspace (Annecy: FL110 held
// 30 NM under the Chambery/Geneva TMAs); everything else stays generic.
bool departure_hold(const std::string &icao, const std::string &sid_last_fix,
                    int *hold_alt_ft, float *release_nm, int *step2_alt_ft);

} // namespace airport_overrides

#endif // DATA_AIRPORT_OVERRIDES_HPP
