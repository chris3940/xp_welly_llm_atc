/*
 * xp_wellys_atc - AI-powered ATC voice communication for X-Plane 12
 * Copyright (C) 2026 thWelly & Claude (Anthropic)
 * Copyright (C) 2026 Christopher P. Potter (Linux port + IFR extensions)
 *
 * Licensed under the GNU GPL-3.0-or-later. See LICENSE.
 */

#ifndef DATA_SIMBRIEF_OFP_HPP
#define DATA_SIMBRIEF_OFP_HPP

#include <string>
#include <vector>

// SDK-free shared cache for the last successfully fetched SimBrief OFP.
// Written by simbrief_client (plugin-only) after a successful fetch;
// read by xplane_context_runtime to populate XPlaneContext IFR fields.
namespace simbrief_ofp {

// One entry from the SimBrief navlog.fix array.
// Fields kept minimal — only what the ATC engine and UI actually consume.
struct NavlogFix {
  std::string ident;      // waypoint identifier, e.g. "ODIK", "BORDI"
  std::string via_airway; // airway to this fix, e.g. "UM728", "DCT"
  double lat = 0.0;
  double lon = 0.0;
  int alt_ft = 0;           // planned altitude at this fix (feet)
  bool is_sid_star = false; // true when part of SID or STAR
  // Flight-phase classification from SimBrief's own optimizer:
  //   "CLB" - climb from origin to top-of-climb
  //   "CRZ" - level cruise (may include filed step-downs)
  //   "DSC" - descent from TOD to destination
  // Empty when SimBrief did not populate the field.  Lets consumers
  // distinguish mid-climb altitude artifacts from genuine mid-cruise
  // step-downs — e.g. KUKEV=19600ft CLB (skip) vs GOLEB=21000ft CRZ
  // (honor as filed step-down from FL220 to FL210).
  std::string stage;
};

// One explicit step marker parsed from the filed ICAO route string
// (general.route). Format in the raw route: "<FIX>/N<spd>F<FL>", e.g.
// "BANKO/N0307F210" = at BANKO, step to 307 kts / FL210.  ATC drives the
// enroute FL clearances from this list — not from the SimBrief-expanded
// navlog altitudes, which include intermediate airway fixes and per-fix
// vertical-profile altitudes that don't correspond to filed ATC step points.
struct RouteStep {
  std::string ident; // fix where the step becomes effective, e.g. "BANKO"
  int cruise_fl = 0; // FL after this step, e.g. 210
};

struct OfpData {
  std::string origin_icao;      // e.g. "LFLP"
  std::string destination_icao; // e.g. "LFMN"
  std::string
      destination_name; // e.g. "Nice" (short airport name, empty if unknown)
  std::string sid_name; // e.g. "MOBE2D" (empty if none filed)
  std::string fpl_first_fix; // first waypoint after departure = last fix of
                             // SID, e.g. "AMIKI"
  int cruise_alt_ft = 0;     // cruise altitude in feet (display only)
  std::string aircraft_reg;  // e.g. "N900SB"
  std::string aircraft_type; // ICAO type code, e.g. "TBM9"
  long long sched_off = 0;   // scheduled takeoff Unix timestamp (0 = unknown)
  // Full navlog waypoint list (origin apt → ... → destination apt).
  // Empty until a successful fetch. Used by the IFR tab for FPL display,
  // and by Phase 3/4 for cross-track deviation detection and direct-to
  // shortcuts.
  std::vector<NavlogFix> navlog;
  // Raw ICAO route string as returned by SimBrief in general.route
  // (e.g. "DCT KUKEV L50 BANKO/N0307F210 Y52 SALEV DCT"). Empty when
  // absent from the OFP. Retained verbatim so the enroute step parser
  // and log diagnostics can reference the exact filed sequence.
  std::string raw_route;
  // Explicit FL step markers extracted from raw_route. Empty for
  // single-cruise-FL flights (no /F markers filed) — in that case
  // poll_enroute falls back to navlog-driven behavior.
  std::vector<RouteStep> route_steps;
  // Optional: force a specific approach procedure (e.g. "I04LZ") instead of
  // letting best_approach() pick one by visibility priority.  Used by the
  // atc_ifr_repl test harness and by future UI "preferred approach" overrides.
  std::string preferred_approach_designator;
  bool valid = false;
};

void set(const OfpData &ofp); // called from simbrief_client after fetch
OfpData get();                // called from xplane_context_runtime
void clear();                 // call on new flight / user request

// Parse the raw ICAO route string (general.route) for explicit FL step
// markers ("<FIX>/N<spd>F<FL>", "<FIX>/M<mach>F<FL>", "<FIX>/K<kmh>F<FL>").
// Returns one RouteStep per step marker in filed order. SDK-free — used
// from simbrief_client (populating OfpData) and directly from unit tests.
std::vector<RouteStep> parse_route_steps(const std::string &raw_route);

} // namespace simbrief_ofp

#endif // DATA_SIMBRIEF_OFP_HPP
