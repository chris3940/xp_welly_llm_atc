// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Christopher P. Potter
//
// route_shortcut — generic "cut direct to the nearest worthwhile fix" helper.
//
// Phase-agnostic core shared by every ATC-initiated direct-to shortcut (SID exit
// fix, en-route navlog fix, STAR/approach IAF). The caller supplies the aircraft
// position + current altitude and a list of candidate fixes, each annotated with:
//   - its position (lat/lon),
//   - the routed (leg-by-leg) distance the aircraft would otherwise fly to reach
//     it along the CURRENT phase's points (SID / STAR / en-route) — used to prove
//     the direct actually saves track,
//   - an optional published crossing altitude at the fix (for the descent-angle
//     feasibility gate on arrivals),
//   - the route-tracker index to jump to if the shortcut is issued.
//
// The helper returns the candidate GEOGRAPHICALLY CLOSEST to the aircraft that
// (a) materially shortens the route and (b) stays flyable (descent within the
// caller's max angle). Purely geometric, SDK-free, header-only, unit-testable.

#pragma once

#include <cmath>
#include <string>
#include <vector>

#include "data/traffic_geometry.hpp"

namespace route_shortcut {

struct Candidate {
  std::string ident;
  double lat = 0.0, lon = 0.0;
  double routed_nm = 0.0;   // leg-by-leg aircraft->...->this fix (0 = skip saving gate)
  int cross_alt_ft = 0;     // published at/for altitude at the fix (0 = no descent gate)
  int route_idx = -1;       // tracker index to jump to when issued
};

struct Pick {
  bool ok = false;
  int which = -1;           // index into the candidates vector, -1 = none
  std::string ident;
  int route_idx = -1;
  double direct_nm = 0.0;   // direct aircraft->fix distance
  double saved_nm = 0.0;    // routed_nm - direct_nm (0 if no routed_nm supplied)
  double descent_deg = 0.0; // required descent angle to the fix's cross altitude
};

// Feet per nautical mile — for the descent-angle computation.
inline constexpr double kFtPerNm = 6076.12;

// Pick the aircraft-nearest candidate that is a worthwhile + flyable shortcut.
//   min_save_frac   : require saved >= frac * routed_nm (<=0 disables the gate).
//                     A candidate with routed_nm <= 1 NM is skipped when the gate
//                     is on (nothing meaningful to save).
//   max_descent_deg : reject a candidate whose descent to cross_alt_ft would be
//                     steeper than this (<=0 disables; 3.0 typical for arrivals).
//                     Candidates already at/below their cross altitude always pass
//                     the descent gate (nothing to lose).
inline Pick pick_nearest(double ac_lat, double ac_lon, int ac_alt_ft,
                         const std::vector<Candidate> &cands,
                         double min_save_frac, double max_descent_deg) {
  Pick best;
  for (int i = 0; i < static_cast<int>(cands.size()); ++i) {
    const Candidate &c = cands[i];
    if (c.lat == 0.0 && c.lon == 0.0)
      continue; // no position -> cannot evaluate
    const double direct =
        traffic_geometry::distance_nm(ac_lat, ac_lon, c.lat, c.lon);
    if (direct <= 0.0)
      continue;

    // Saving gate: the direct must shorten the routed track by min_save_frac.
    if (min_save_frac > 0.0) {
      if (c.routed_nm <= 1.0)
        continue;
      if ((c.routed_nm - direct) < min_save_frac * c.routed_nm)
        continue;
    }

    // Descent-angle feasibility gate (arrivals): the descent to the fix's
    // published crossing altitude, flown over the DIRECT distance, must not
    // exceed max_descent_deg. Skipped when no cross altitude is supplied.
    double descent_deg = 0.0;
    if (max_descent_deg > 0.0 && c.cross_alt_ft > 0) {
      const double lose =
          static_cast<double>(ac_alt_ft) - static_cast<double>(c.cross_alt_ft);
      if (lose > 0.0) {
        descent_deg = std::atan2(lose, direct * kFtPerNm) * 180.0 / M_PI;
        if (descent_deg > max_descent_deg)
          continue;
      }
    }

    // Nearest-to-aircraft wins (user rule 2026-08-02: the closest IAF, even if
    // it changes the approach transition).
    if (!best.ok || direct < best.direct_nm) {
      best.ok = true;
      best.which = i;
      best.ident = c.ident;
      best.route_idx = c.route_idx;
      best.direct_nm = direct;
      best.saved_nm = c.routed_nm > 0.0 ? (c.routed_nm - direct) : 0.0;
      best.descent_deg = descent_deg;
    }
  }
  return best;
}

} // namespace route_shortcut
