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

#include "engine.hpp"

#include "atc/atc_state_machine.hpp"
#include "atc/atc_templates.hpp"
#include "atc/flight_phase.hpp"
#include "atc/intent_rules.hpp"
#include "atc/landing_sequence.hpp"
#include "atc/sector_picker.hpp"
#include "atc/traffic_advisor.hpp"
#include "atc/traffic_dialog.hpp"
#include "backends/manager.hpp"
#include "core/logging.hpp"
#include "data/airspace_db.hpp"
#include "data/airport_overrides.hpp"
#include "data/cifp_reader.hpp"
#include "data/openair_db.hpp"
#include "data/simbrief_ofp.hpp"
#include "data/traffic_context.hpp"
#include "data/traffic_geometry.hpp"
#include "persistence/settings.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <optional>
#include <regex>
#include <unordered_map>
#include <unordered_set>

namespace engine {

static int profanity_warnings_ = 0;
static int lm_inferences_ = 0;
static traffic_advisor::AdvisoryHistory advisory_history_;
// Counts back-to-back unintelligible transmissions. Reset whenever the
// pilot lands a valid intent. Drives the escalation from "garbled" to
// "use standard phraseology" so a controller-style nudge follows the
// pilot's repeated unclear calls.
static int unclear_streak_ = 0;

// Phase-4 go-around throttle. Last monotonic clock at which a
// go-around was emitted; -1e9 = never. Keeps the trigger from re-
// firing every frame while the runway stays occupied.
static double last_go_around_emit_secs_ = -1e9;
constexpr double kGoAroundCooldownSec = 60.0;
constexpr double kGoAroundTriggerDistanceNm = 1.0;

// IFR departure handoff timer. Accumulates seconds while in
// IFR_DEPARTURE_CLEARED + CLIMB. Reset whenever the state is not
// IFR_DEPARTURE_CLEARED so a re-entry starts fresh.
static float s_departure_handoff_timer = 0.0f;
static std::string
    s_current_controller_label; // last handoff target (for transcript)
// Departure label stored when the takeoff clearance is built (ground phase).
// Activated into s_current_controller_label by poll_departure_handoff() so
// the label never appears in ground-phase transcript entries.
static std::string s_pending_departure_label;
// Controller label the pilot is being handed off TO but hasn't reached yet.
// Set when a mid-flight sector-change handoff is issued (SID climb TMA-exit,
// enroute sub-phase 1.5 sector change, poll_approach sector change).
// Stays populated while s_sector_checkin_pending is true; swapped into
// s_current_controller_label the moment the pilot's active COM matches
// s_pending_handoff_freq_mhz (i.e., pilot actually switched).  Prevents the
// transcript from showing the NEW controller's name in responses that are
// actually still being emitted by the PREVIOUS controller (e.g. a wrong-freq
// reminder on the old sector's frequency).
static std::string s_pending_controller_label;
// Frequency (MHz) the pilot was last asked to switch to. Used by
// check_handoff_reissue() to re-state the instruction when the pilot calls
// back on the old frequency.
static float s_pending_handoff_freq_mhz = 0.0f;
// Frequency the pilot should tune after a training JUMP (the target phase's
// controller freq). A jump sets ATC state only, never the radio, so the UI
// surfaces this via jump_switch_freq_mhz() as a "Switch COM to X" popup. 0 when
// unknown (no airspace/airport freq data).
static float s_jump_switch_freq_mhz = 0.0f;
// True after ATC issues a sector frequency-change instruction.
// Cleared the first time the pilot transmits on the new sector frequency
// (process_transcript detects active COM ≈ s_pending_handoff_freq_mhz).
// Gates all proactive poll_enroute messages so ATC never speaks before
// the pilot's check-in call on the new frequency.
static bool s_sector_checkin_pending = false;

// IFR en-route management (IFR_ENROUTE_CRUISE state).
static float s_enroute_timer = 0.0f; // accumulates while in IFR_ENROUTE_CRUISE
static bool s_enroute_direct_issued = false;
static float s_enroute_direct_delay_sec =
    0.0f; // pseudo-random 90-120 s, set on first entry
static bool s_enroute_descent_issued = false;
static float s_enroute_deviation_cooldown_sec =
    0.0f; // countdown between deviation warnings
// Sector frequency monitoring: detect when the aircraft crosses into a
// different ACC/FIR sector (e.g. Marseille Nord → Marseille Sud).
static uint32_t s_enroute_sector_freq_khz = 0;  // 0 = not yet initialised
// Sectors already handed off to during the enroute cruise phase.  Same
// rule as its s_approach_visited_sector_freqs sibling: ATC never hands
// backward, so the picker excludes any candidate whose freq is already
// in this list.  Reset on state exit.
static std::vector<uint32_t> s_enroute_visited_sector_freqs;
static float s_enroute_sector_check_sec = 0.0f; // countdown; fires at 0

// ACC-to-ACC (CTR/FIR/UIR) sector handoff during DESCENT + ARRIVAL, e.g.
// Milan -> France (UIR, >FL195) -> Marseille (FIR, <FL195) as the aircraft
// crosses the Italy/France boundary ~6 NM after BANKO. Dedicated statics: the
// enroute set above is zeroed every frame by poll_enroute outside cruise, so
// it cannot carry state through descent. TRACON (Approach) handoffs are owned
// by poll_arrival / poll_approach; this helper only advances CTR sectors.
static uint32_t s_acc_sector_freq_khz = 0;
static std::vector<uint32_t> s_acc_visited_sector_freqs;
static float s_acc_sector_check_sec = 0.0f;
// Altitude deviation monitoring during cruise.
// RVSM (>=FL290): threshold 200 ft. Below FL290: 300 ft.
static int s_enroute_cleared_alt_ft =
    0; // ATC-cleared cruise altitude (0 = unknown)
static float s_enroute_alt_warn_cooldown =
    0.0f; // countdown between altitude warnings
// Step-1 altitude verification: fires 30-60 s after a descent/climb clearance
// when the aircraft has NOT started moving toward the new target (vertical
// speed ~0). Comes before the step-2 "check altitude" deviation warning.
// Reset by every clearance that sets s_enroute_alt_warn_cooldown = 180.
static bool  s_enroute_verify_query_sent = false;
static int   s_enroute_verify_target_ft  = 0; // altitude at time of last clearance
// Altitude-compliance monitor for the DESCENT + ARRIVAL phases (the gap where
// the en-route 2.4 verify and the approach verify-descending do not run).
static int   s_alt_comp_target_ft = 0;   // assigned level being monitored
static float s_alt_comp_arm_sec   = 0.0f; // seconds since that assignment
static bool  s_alt_comp_sent      = false; // fired once for this assignment
// Descend-to-enter-terminal-area clearance (poll_descent): the last TMA-ceiling
// level we cleared the aircraft down to, so the clearance fires only once per
// distinct target instead of every 1 Hz tick.
static int   s_descent_tma_target_ft = 0;
// CIFP STAR crossing-altitude corrective (poll_descent): the last crossing
// target we cleared to, so "descend FLxxx" for a STAR fix fires once per fix.
static int   s_descent_cifp_target_ft = 0;
// Set to true when the pilot says "request descent" in IFR_ENROUTE_CRUISE.
// Consumed by poll_enroute on the next frame to issue the descent clearance.
static bool s_pilot_requested_descent = false;
// Requested FL in feet extracted from the pilot's transcript (0 = not parsed).
static int  s_pilot_requested_fl_ft   = 0;
// Set when the proactive step-up climb (cleared_alt < cruise_alt) has fired.
static bool s_cruise_stepup_issued = false;
// Set when ATC issues the pre-TOD "advise when ready to descend" prompt.
static bool s_enroute_descent_prompt_issued = false;
// Set when the Approach frequency handoff ("contact Approach on X.XXX") is issued.
static bool s_enroute_approach_handoff_issued = false;
// Controller label of the last ARRIVAL-phase frequency handoff (Stage A of the
// arrival handoff -- "contact Geneva Approach" while still flying the STAR). Used
// to dedup so the same controller is not re-announced every second, and so the
// later APPROACH-phase handoff (Stage B, near the IAF) stays SILENT when it
// resolves to the same controller already tuned. Empty = none yet.
static std::string s_arrival_freq_handoff_label;
// EUROCONTROL / DGAC phraseology: QNH is stated with every altitude instruction
// below the transition level — but only ONCE per pilot–sector interaction.
// If a navlog step-down clearance already transmitted "descend X feet, QNH XXXX",
// build_descent_clearance suppresses the QNH to avoid repeating the same value
// 20–30 seconds later in the approach clearance.  Reset when a new enroute
// phase begins (new flight, sector entry).
static bool s_qnh_stated = false;
// True after a training jump DIRECTLY into ARRIVAL/APPROACH (no en-route phase),
// until the first Approach check-in ack consumes it. Makes that first descent
// clearance say "descend FLxx" (fresh) rather than "continue descent to FLxx"
// -- there was no prior en-route/Centre clearance to CONTINUE (LFMN jump-to-APP
// 2026-07-21).
static bool s_jump_no_enroute_descent = false;
static float s_enroute_app_check_sec = 0.0f; // throttle TMA-entry poll to 1 Hz
static float s_descent_timer        = 0.0f; // time spent in IFR_DESCENT (guards 50 NM fallback)
static float s_descent_arrival_check_sec = 0.0f; // throttle DESCENT->ARRIVAL poll to 1 Hz
static float s_arrival_timer        = 0.0f; // time spent in IFR_ARRIVAL (guards 50 NM fallback)
static float s_enroute_approach_freq_mhz = 0.0f; // set by build_approach_handoff
// Navlog altitude step tracking: index into OFP navlog for the next fix whose
// planned alt_ft may require a climb or descent clearance during cruise.
// Used only as a fallback when ofp.route_steps is empty (single-cruise-FL
// flights with no filed /F markers).
static int s_navlog_alt_step_idx = 0;
// Filed FL step marker tracking: index into ofp.route_steps for the next
// filed step whose FL may require a clearance. Preferred over the navlog
// walker when ofp.route_steps is non-empty — driven by the ICAO route
// string's explicit "<FIX>/N<spd>F<FL>" markers rather than SimBrief's
// per-fix computed altitudes (which include intermediate airway fixes and
// vertical-profile-optimised altitudes that don't match filed ATC steps).
static int s_route_step_idx = 0;
// Speed restriction: set once when the 250 kt / FL100 advisory fires; cleared
// when the aircraft climbs back above FL100 so the advisory re-fires on the
// next descent through FL100.
static bool s_speed_250_warned = false;
// Last speed limit (kt) we evaluated, so a NEWLY tighter cap (e.g. 250 -> 210
// approaching a terminal fix) re-arms the advisory even if it had already
// fired for the looser limit. 0 = none evaluated yet.
static int s_last_speed_limit_kt = 0;

static int round_to_fl(int feet); // defined near poll_sid_climb
static void init_route_fixes(const xplane_context::XPlaneContext &ctx); // defined near poll_approach
static void build_sid_route_table(const xplane_context::XPlaneContext &ctx); // departure half of the route table
static std::string controller_label_for(const airspace_db::Controller *ctrl); // defined near handoff helpers
static std::string openair_sector_label(const std::string &name); // openair NAME -> label ("MARSEILLE CTA..." -> "Marseille")
static bool resolve_sector_controller(const openair_db::AirspaceEntry &enc,
                                      bool terminal, std::string *out_label,
                                      float *out_mhz); // unified handoff resolver

// Arrival runway-in-use: the per-airport runway_config (airport+.json) wins when it
// resolves one -- it pins the direction AND the L/R split that wind alone cannot
// (LFLP arrive 04, LFMN arrive 04L) -- else the CIFP wind-based pick. This is the
// runway fed into the preferred-approach lookup, so the approach keys on the correct
// runway (user 2026-07-19).
static std::string pick_arrival_runway(const xplane_context::XPlaneContext &ctx,
                                       const std::string &dest) {
  const std::string cfg = airport_overrides::arrival_runway(
      dest, ctx.wind_direction_deg, ctx.wind_speed_kt);
  if (!cfg.empty())
    return cfg;
  return cifp_reader::best_runway_for_approach(
      ctx.cifp_dir, dest, ctx.wind_direction_deg, ctx.visibility_m);
}

// Weather for the preferred-approach gate: prefer the DESTINATION airport's METAR
// (parsed in xplane_context_runtime) so the RNAV Alpha/Zulu choice keys on the
// arrival field's reported vis/ceiling, not the region value sampled at the
// aircraft en route (which flipped "expect Alpha" -> "cleared Zulu"; LFMN
// 2026-07-19). Fall back to the region DataRef when no METAR is available.
static float approach_gate_vis_m(const xplane_context::XPlaneContext &ctx) {
  return ctx.dest_metar_visibility_m >= 0.0f ? ctx.dest_metar_visibility_m
                                             : ctx.visibility_m;
}
static float approach_gate_ceiling_ft(const xplane_context::XPlaneContext &ctx) {
  return ctx.dest_metar_ceiling_ft >= 0.0f ? ctx.dest_metar_ceiling_ft
                                           : ctx.cloud_base_ft_msl;
}

// Effective altitude (ft) for OPENAIR airspace lookups (find_enclosing etc.).
// Navigraph's openair writes FLIGHT-LEVEL ceilings (e.g. FL195 -> "19500 MSL",
// Chambery TMA FL095 -> "9500 MSL") as feet labelled "MSL" and NEVER emits "FL".
// So above the transition altitude the boundary means a FLIGHT LEVEL and must be
// compared against PRESSURE altitude; below it, boundaries are true AMSL -> MSL.
// Using MSL against an FL ceiling was wrong by the QNH offset (~800 ft at QNH
// 1026) and hid the aircraft's entry into Chambery TMA (user 2026-07-19).
// *** On standard pressure (QNH 1013) pressure_alt == MSL, so this is a NO-OP --
// standard-day flights are byte-identical; only non-standard QNH changes (and was
// wrong before). *** Only for openair_db; airspace_db (atc.dat) is untouched.
static int openair_alt(const xplane_context::XPlaneContext &ctx) {
  const float ta = ctx.transition_alt_ft > 0.0f ? ctx.transition_alt_ft : 5000.0f;
  return static_cast<int>(ctx.altitude_ft_msl > ta ? ctx.pressure_alt_ft
                                                    : ctx.altitude_ft_msl);
}

// Spoken (plain-language) form of a SID/STAR designator per ICAO/EUROCONTROL
// phraseology (Annex 11 App.3 / Doc 8168): the FULL significant-point name, the
// validity indicator as a NUMBER WORD, and the route indicator as a NATO letter.
// ICAO examples: coded "TBO6S" -> "TARBES SIX SIERRA", "LUGEN1N" -> "LUGEN ONE
// NOVEMBER". ARINC-424 truncates a 5-letter fix to 4 chars to fit its 6-char
// field ("ABDIL" -> coded "ABDI8R"); `full_point` (the STAR's entry/naming fix,
// e.g. "ABDIL") restores it when it starts with the coded prefix. So "ABDI8R" +
// "ABDIL" -> "ABDIL EIGHT ROMEO", "SALEV3P" + "SALEV" -> "SALEV THREE PAPA".
// Falls back to the coded prefix (VOR idents like TBO need a navaid-name DB we do
// not read here) and returns the name unchanged if it isn't <letters><digits>...
// (user 2026-07-19, EUROCONTROL check).
static std::string spoken_procedure_name(const std::string &name,
                                         const std::string &full_point = "") {
  static const char *kNato[] = {
      "Alpha", "Bravo",  "Charlie", "Delta",   "Echo",   "Foxtrot", "Golf",
      "Hotel", "India",  "Juliet",  "Kilo",    "Lima",   "Mike",    "November",
      "Oscar", "Papa",   "Quebec",  "Romeo",   "Sierra", "Tango",   "Uniform",
      "Victor","Whiskey","X-ray",   "Yankee",  "Zulu"};
  static const char *kNum[] = {"Zero", "One",  "Two",   "Three", "Four",
                               "Five", "Six",  "Seven", "Eight", "Nine"};
  size_t i = 0;
  while (i < name.size() && std::isalpha(static_cast<unsigned char>(name[i])))
    ++i;
  size_t j = i;
  while (j < name.size() && std::isdigit(static_cast<unsigned char>(name[j])))
    ++j;
  if (i == 0 || j == i)
    return name; // not <letters><digits>[letters] -- leave as-is
  const std::string prefix = name.substr(0, i);
  const std::string digits = name.substr(i, j - i);
  const std::string rev = name.substr(j);
  // Basic indicator: full point name when it starts with the coded prefix.
  std::string basic = prefix;
  if (full_point.size() >= prefix.size()) {
    bool is_prefix = true;
    for (size_t k = 0; k < prefix.size(); ++k)
      if (std::toupper(static_cast<unsigned char>(full_point[k])) !=
          std::toupper(static_cast<unsigned char>(prefix[k]))) {
        is_prefix = false;
        break;
      }
    if (is_prefix && !full_point.empty())
      basic = full_point;
  }
  std::string out = basic;
  for (char c : digits)
    if (std::isdigit(static_cast<unsigned char>(c))) {
      out += ' ';
      out += kNum[c - '0'];
    }
  for (char c : rev)
    if (std::isalpha(static_cast<unsigned char>(c))) {
      const int idx = std::toupper(static_cast<unsigned char>(c)) - 'A';
      if (idx >= 0 && idx < 26) {
        out += ' ';
        out += kNato[idx];
      }
    }
  return out;
}

// IFR approach STAR constraint tracking (IFR_APPROACH_CONTACT / IFR_APPROACH_DESCENT).
static std::string s_assigned_star_name;             // set by build_descent_clearance
static std::string s_assigned_star_entry_fix;        // full naming fix (ABDIL) for the spoken designator
static std::string s_assigned_dest_icao;             // set by build_descent_clearance
static std::string s_assigned_approach_designator;   // set by build_descent_clearance
// One-shot informational note drained by atc_session into the transcript as a
// System line (e.g. "preferred approach downgraded by weather"). Set by the
// engine, read+cleared via engine::take_pending_transcript_note().
static std::string s_pending_transcript_note;
static std::string s_assigned_landing_runway;        // set at APPROACH_CONTACT from CIFP
static std::string s_no_star_direct_iaf;             // IAF ident issued in no-STAR direct clearance
static std::vector<cifp_reader::StarWaypoint> s_approach_waypoints;
static int   s_approach_waypoint_idx   = 0;   // next constraint to issue
static float s_approach_timer          = 0.0f;
static int   s_approach_initial_fl     = 0;    // FL issued at Approach check-in
static bool              s_approach_final_issued     = false; // final altitude + QNH issued
static bool              s_approach_cleared_issued   = false; // "cleared <appr> runway <rwy>" issued once at the IAF
static bool              s_approach_tower_handed_off = false; // "contact Tower, report established"
static cifp_reader::FafFix s_approach_faf;                   // FAF from CIFP + earth_fix.dat
// Sector-boundary handoff during approach descent: tracks the enclosing
// TRACON/CTR frequency baseline so a sector exit (e.g. leaving Melun TMA)
// triggers a handoff to the destination INFO/Tower — same mechanism as
// poll_enroute() sub-phase 1.5.
static uint32_t s_approach_sector_freq_khz  = 0;
static float    s_approach_sector_check_sec = 0.0f;
// openair ceiling (ft) of the current approach sector's TMA -- so the forward
// handoff only advances to a MORE-TERMINAL (lower-ceiling) inner TMA and never
// backward/sideways (Chambery->Lyon->Marseille, LIMF->LFLP 2026-07-14).
static int      s_approach_sector_ceiling_ft = 0;
// Sectors already handed off to during this approach phase.  Real ATC only
// hands forward: once you're on Chambery, you don't get sent back to
// Geneva even if the Geneva polygon re-envelops the aircraft later.  The
// approach sector picker skips any candidate whose freq is in this set
// (except the currently-active one).  Reset when leaving IFR_APPROACH_*.
static std::vector<uint32_t> s_approach_visited_sector_freqs;
// Expedite-descent monitor: proactive warning when required VS > current VS * 1.5.
// Fires only AFTER a step-down clearance has been issued (s_expedite_last_cleared_ft > 0).
// Distinct from s_enroute_deviation_cooldown_sec (airway/sector off-track, en-route only).
static float s_expedite_cooldown       = 0.0f;  // counts down; fires when <= 0
static int   s_expedite_last_cleared_ft = 0;    // altitude of last issued step-down
// Lateral-deviation monitor (after FAF, Tower state): cross-track from runway centerline.
static float s_alignment_cooldown = 0.0f;
// DirectMonitor course-deviation cooldowns (en-route + approach sites). SID has
// its own inline check (s_sid_deviation_cooldown_sec). Firing gates TBD.
static float s_enroute_course_cooldown  = 0.0f;
static float s_approach_course_cooldown = 0.0f;

// Route fix tracker — the COMPLETE ordered arrival sequence (enroute navlog +
// ALL STAR + ALL approach fixes, constrained or not). Fixes are dropped from
// the sequence ONLY when ATC issues a direct-to (user rule 2026-07-11), never
// silently. Each fix carries its CIFP altitude/FL + speed constraint (0/empty
// = none) so the phase/speed/altitude logic can read one authoritative table.
struct RouteFix {
  std::string ident;
  double lat = 0.0;
  double lon = 0.0;
  cifp_reader::CifpAlt alt{};       // feet + is_fl; feet==0 = no altitude constraint.
                                    // For a "B" block this is the CEILING (upper).
  bool is_ceiling = false;          // at-or-below ("-" / "B")
  bool is_floor = false;            // at-or-above ("+")
  int floor_ft = 0;                 // block "B" lower bound (0 = not a block)
  int speed_kt = 0;                 // 0 = no speed constraint
  bool is_approach_proc = false;    // from the APPCH transition (vs STAR)
  bool is_map = false;              // Missed Approach Point
};
static std::vector<RouteFix> s_route_fixes;
static int   s_route_fix_idx      = 0;
static float s_route_tracker_tick = 0.0f; // seconds since last distance check
// Pending ATC-direct event from poll_approach — returned by poll_route_tracker
// so atc_session picks it up via the existing System transcript push.
static std::string s_pending_route_direct;
// Step-down trigger: fires when route tracker passes the last-cleared fix index.
// Falls back to 3-min timer when no step-down has been issued yet (idx == -1).
static int  s_last_cleared_route_idx   = -1;
static int  s_faf_route_idx            = -1; // route idx of FAF (Tower handoff trigger)
static int  s_iaf_route_idx            = -1; // route idx of IAF (guards against outbound false FAF)
static int  s_faf_ap_idx               = -1; // FAF index in s_approach_waypoints
static int  s_map_ap_idx               = -1; // MAP index (post-MAP = GO_AROUND territory)
static bool s_approach_has_visual_final = false; // MDA approach: offset final, "runway in sight"

// IFR SID climb management (IFR_RADAR_CONTACT state).
static bool s_sid_direct_issued = false;
static bool s_sid_step1_issued = false;
static bool s_sid_cruise_issued = false;
static bool s_sid_radar_handoff_issued = false;
static bool s_sid_initialized = false; // guards one-time init block
// Intermediate TRACON handoffs already fired during the SID climb — one
// entry per (kHz) frequency. Prevents retriggering when the aircraft
// re-enters the same TRACON polygon or crosses back through a stacked
// sub-polygon boundary. Used only by the "intermediate TRACON entry"
// check in poll_sid_climb; the final "exited all TMAs" handoff has its
// own s_sid_radar_handoff_issued flag.
static std::unordered_set<uint32_t> s_sid_intermediate_tracon_khz_seen;
// True once the aircraft has been detected INSIDE a CTR or TMA at least once.
// The TMA-exit handoff is only issued when this transitions from true → false,
// preventing a spurious handoff when the departure altitude is below the TMA
// floor (aircraft was never inside the TMA to begin with).
static bool s_sid_was_in_tma = false;
static float s_sid_tma_check_sec = 0.0f; // throttle openair_db TMA-exit poll to 1 Hz
static float s_sid_pos_log_sec  = 0.0f; // throttle periodic position log to 1/60 s
static float s_sid_climb_timer = 0.0f;
static int s_sid_step1_alt_ft = 0; // computed once on first entry
// While > 0, the step-1 level is HELD -- neither the cruise climb nor the radar
// handoff fires until the aircraft is this many NM (great-circle) from the
// departure airport. 0 = no hold (normal continuous climb). Set only for local
// procedures that keep an aircraft low under an adjacent TMA (LFLP west SIDs:
// FL110 until ~30 NM out -> avoid a pointless Geneva-APP shuffle). Overridden
// when the SID's own minimum crossing altitude is above step-1 (the procedure
// itself demands a higher climb).
static float s_sid_hold_release_nm = 0.0f;
static float s_sid_deviation_cooldown_sec = 0.0f;
// Aircraft position when ATC issued the direct-to clearance.
// Used to build the direct leg (origin → fix) for post-direct deviation check.
static double s_sid_direct_origin_lat = 0.0;
static double s_sid_direct_origin_lon = 0.0;
// Seconds elapsed since ATC issued the direct-to clearance. Used by the
// post-direct heading-vs-bearing check (gives the FMS ~3 min to intercept
// before we start comparing course to bearing_to_fix).
static float s_sid_direct_elapsed_sec = 0.0f;
// Departure airport captured at radar-contact entry.
// Kept here so nearest_airport_id cannot drift as the aircraft flies away.
static std::string s_departure_apt_id;
static double s_departure_apt_lat = 0.0;
static double s_departure_apt_lon = 0.0;

// The airport all AIRBORNE IFR lookups (controller, TA, QNH, CIFP) must key on:
// the DESTINATION once a descent clearance has bound it (s_assigned_dest_icao),
// the DEPARTURE field before that, and only nearest as a ground/pre-departure
// fallback. ctx.nearest_airport_id is a VFR-native concept that drifts to
// whatever field is closest and is actively wrong en-route (the "climb FL80"
// small-field regression). See [[feedback_nearest_airport_ifr]].
static std::string current_flight_airport(
    const xplane_context::XPlaneContext &ctx) {
  if (!s_assigned_dest_icao.empty())
    return s_assigned_dest_icao;   // descent and beyond
  if (!s_departure_apt_id.empty())
    return s_departure_apt_id;     // airborne, pre-descent
  return ctx.nearest_airport_id;   // on the ground / fallback
}

// Ground runway-change detection: ATC must announce when active runway changes
// while on the ground.
static std::string s_ground_last_announced_runway; // last runway ATC announced on ground

void reset() {
  profanity_warnings_ = 0;
  lm_inferences_ = 0;
  unclear_streak_ = 0;
  advisory_history_ = traffic_advisor::AdvisoryHistory{};
  last_go_around_emit_secs_ = -1e9;
  s_departure_handoff_timer = 0.0f;
  s_current_controller_label.clear();
  s_pending_departure_label.clear();
  s_pending_controller_label.clear();
  s_pending_handoff_freq_mhz = 0.0f;
  s_enroute_timer = 0.0f;
  s_sector_checkin_pending = false;
  s_enroute_direct_issued = false;
  s_enroute_direct_delay_sec = 0.0f;
  s_enroute_descent_issued = false;
  s_enroute_descent_prompt_issued = false;
  s_pilot_requested_descent = false;
  s_pilot_requested_fl_ft   = 0;
  s_enroute_approach_handoff_issued = false;
  s_enroute_approach_freq_mhz = 0.0f;
  s_enroute_deviation_cooldown_sec = 0.0f;
  s_cruise_stepup_issued = false;
  s_qnh_stated = false;
  s_jump_no_enroute_descent = false;
  s_descent_timer = 0.0f;
  s_enroute_sector_freq_khz = 0;
  s_enroute_visited_sector_freqs.clear();
  s_enroute_sector_check_sec = 0.0f;
  s_acc_sector_freq_khz = 0;
  s_acc_visited_sector_freqs.clear();
  s_acc_sector_check_sec = 0.0f;
  s_enroute_cleared_alt_ft = 0;
  s_enroute_alt_warn_cooldown = 0.0f;
  s_navlog_alt_step_idx = 0;
  s_route_step_idx = 0;
  s_sid_direct_issued = false;
  s_sid_step1_issued = false;
  s_sid_cruise_issued = false;
  s_sid_radar_handoff_issued = false;
  s_sid_was_in_tma = false;
  s_sid_tma_check_sec = 0.0f;
  s_sid_pos_log_sec  = 0.0f;
  s_sid_climb_timer = 0.0f;
  s_sid_step1_alt_ft = 0;
  s_sid_hold_release_nm = 0.0f;
  s_sid_initialized = false;
  s_sid_deviation_cooldown_sec = 0.0f;
  s_sid_direct_origin_lat = 0.0;
  s_sid_direct_origin_lon = 0.0;
  s_sid_direct_elapsed_sec = 0.0f;
  s_departure_apt_id.clear();
  s_departure_apt_lat = 0.0;
  s_departure_apt_lon = 0.0;
  s_speed_250_warned = false;
  s_last_speed_limit_kt = 0;
  s_enroute_course_cooldown = 0.0f;
  s_approach_course_cooldown = 0.0f;
  s_alt_comp_target_ft = 0;
  s_alt_comp_arm_sec = 0.0f;
  s_alt_comp_sent = false;
  s_descent_tma_target_ft = 0;
  s_descent_cifp_target_ft = 0;
  s_ground_last_announced_runway.clear();
  s_assigned_star_name.clear();
  s_assigned_dest_icao.clear();
  s_assigned_approach_designator.clear();
  s_approach_waypoints.clear();
  s_approach_waypoint_idx = 0;
  s_approach_timer = 0.0f;
  s_approach_initial_fl = 0;
  s_approach_final_issued = false;
  s_approach_cleared_issued = false;
  s_approach_tower_handed_off = false;
  s_approach_faf = {};
  s_last_cleared_route_idx    = -1;
  s_faf_route_idx             = -1;
  s_iaf_route_idx             = -1;
  s_faf_ap_idx                = -1;
  s_map_ap_idx                = -1;
  s_approach_has_visual_final = false;
  s_assigned_landing_runway.clear();
  s_no_star_direct_iaf.clear();
  s_route_fixes.clear();
  s_route_fix_idx = 0;
  s_route_tracker_tick = 0.0f;
  s_pending_route_direct.clear();
  traffic_dialog::reset();
}

void training_jump_enroute(int cleared_alt_ft) {
  // Aircraft is already at cruise altitude — skip phases that have already passed.
  s_enroute_direct_issued = true;    // skip "direct X, when able" shortcut
  s_cruise_stepup_issued = true;     // already at cruise, no FL step-up needed
  s_enroute_timer = 0.0f;
  s_enroute_sector_freq_khz = 0;
  s_enroute_visited_sector_freqs.clear();
  s_enroute_sector_check_sec = 15.0f;
  s_acc_sector_freq_khz = 0;
  s_acc_visited_sector_freqs.clear();
  s_acc_sector_check_sec = 0.0f;
  s_enroute_cleared_alt_ft = cleared_alt_ft > 0 ? cleared_alt_ft : 0;
  s_enroute_descent_issued = false;
  s_enroute_descent_prompt_issued = false;
  s_pilot_requested_descent = false;
  s_pilot_requested_fl_ft   = 0;
  s_enroute_approach_handoff_issued = false;
  s_enroute_deviation_cooldown_sec = 0.0f;
  s_navlog_alt_step_idx = 0;
  s_route_step_idx = 0;
  s_qnh_stated = false;
  s_jump_no_enroute_descent = false;
  s_descent_timer = 0.0f;

  // Hardening 1: seed the controller label from the enclosing CTR sector at
  // the aircraft's current 3-D position, so the first clearance is spoken by
  // the real sector (e.g. "Milan") rather than the generic "Control"
  // fallback that poll_enroute would otherwise show until the sector
  // resolves. Mirrors the lazy seeding in poll_enroute sub-phase 1.5.
  {
    const auto &ctx = xplane_context::get();
    s_current_controller_label.clear();
    s_jump_switch_freq_mhz = 0.0f;
    const auto sectors = airspace_db::find_enclosing(
        ctx.latitude, ctx.longitude, ctx.altitude_ft_msl);
    for (const auto *s : sectors) {
      if (s && s->role == airspace_db::ControllerRole::CTR &&
          !s->freqs_khz.empty()) {
        s_current_controller_label = controller_label_for(s);
        s_jump_switch_freq_mhz =
            static_cast<float>(s->freqs_khz.front()) / 1000.0f;
        break;
      }
    }
    if (s_current_controller_label.empty())
      s_current_controller_label = "Control";
    // Prefer the accurate openair sector NAME for the label (e.g. "Marseille" over
    // the broad atc.dat "France") while keeping the atc.dat frequency above
    // (openair = geometry + name, atc.dat = frequency; LFMN 2026-07-20).
    if (openair_db::ready()) {
      const std::string oa = openair_sector_label(
          openair_db::find_enclosing(ctx.latitude, ctx.longitude,
                                     openair_alt(ctx))
              .name);
      if (!oa.empty())
        s_current_controller_label = oa;
    }
  }

  // Hardening 2: an IFR training jump has no destination / STAR / approach
  // context of its own — those come from the loaded SimBrief OFP + CIFP when
  // build_descent_clearance fires. Without an OFP the descent/approach flow
  // has no destination and silently misbehaves. Warn loudly so the cause is
  // obvious in Log.txt.
  {
    const auto ofp = simbrief_ofp::get();
    if (!ofp.valid || ofp.destination_icao.empty())
      logging::info("WARN training_jump_enroute: no valid OFP loaded -- "
                    "descent/approach will have no destination context. "
                    "Load a SimBrief OFP before jumping to ENR.");
    else
      logging::info("training_jump_enroute: dest=%s cruise=%dft cleared=%dft "
                    "controller=%s",
                    ofp.destination_icao.c_str(), ofp.cruise_alt_ft,
                    s_enroute_cleared_alt_ft,
                    s_current_controller_label.c_str());
  }

  // Hardening 3: lock the session callsign now. The jump bypasses the
  // initial-call flow that normally locks it, so it would otherwise stay
  // empty and the first mid-flight transcript yielding a callsign token
  // would hijack it — LIMF -> LFLP 2026-07-10 locked "Lima Papa" from the
  // RNAV IAF readback "Direct LP403" ("Lima Papa 403"), and ATC then
  // addressed the aircraft as "Lima Papa" for the rest of the flight.
  atc_state_machine::set_session_callsign(settings::pilot_callsign());

  atc_state_machine::set_state(atc_state_machine::ATCState::IFR_ENROUTE_CRUISE);
}

void training_jump_approach() {
  // Skip en-route phase; pilot will call Approach to begin.
  s_enroute_descent_issued = true;
  s_enroute_approach_handoff_issued = true;
  s_enroute_approach_freq_mhz = 0.0f; // unknown at training jump — accept any frequency
  s_jump_switch_freq_mhz = 0.0f;      // APP accepts any freq -> no single "switch to"
  // Populate dest ICAO from OFP so poll_approach can load STAR waypoints once
  // s_assigned_star_name is set via the normal descent-clearance path.
  auto ofp = simbrief_ofp::get();
  // Hardening 2 (see training_jump_enroute): approach jump needs the OFP for
  // destination / STAR / approach. Warn loudly if none is loaded.
  if (!ofp.valid || ofp.destination_icao.empty())
    logging::info("WARN training_jump_approach: no valid OFP loaded -- "
                  "no destination/STAR/approach context. Load a SimBrief OFP "
                  "before jumping to APP.");
  s_assigned_dest_icao = ofp.destination_icao;
  s_assigned_star_name.clear();
  s_assigned_approach_designator.clear();
  s_assigned_landing_runway.clear();
  s_approach_waypoints.clear();
  s_approach_waypoint_idx = 0;
  s_approach_timer = 0.0f;
  s_approach_initial_fl = 0;
  s_approach_final_issued = false;
  s_approach_cleared_issued = false;
  s_approach_tower_handed_off = false;
  s_approach_faf = {};
  s_last_cleared_route_idx    = -1;
  s_faf_route_idx             = -1;
  s_iaf_route_idx             = -1;
  s_faf_ap_idx                = -1;
  s_map_ap_idx                = -1;
  s_approach_has_visual_final = false;
  s_no_star_direct_iaf.clear();
  s_route_fixes.clear();
  s_route_fix_idx = 0;
  s_route_tracker_tick = 0.0f;
  s_pending_route_direct.clear();
  s_sector_checkin_pending = false;
  // Hardening 3 (see training_jump_enroute): lock the callsign so an RNAV
  // IAF ident readback ("Lima Papa 403") can't hijack it mid-approach.
  atc_state_machine::set_session_callsign(settings::pilot_callsign());
  s_jump_no_enroute_descent = true; // no en-route descent -> first ack "descend"
  atc_state_machine::set_state(atc_state_machine::ATCState::IFR_APPROACH_CONTACT);
  // Set a temporary approach label so the transcript doesn't fall back to
  // the nearest airport name during check-in (training jump skips handoff).
  s_current_controller_label =
      s_assigned_dest_icao.empty() ? "Approach" : (s_assigned_dest_icao + " Approach");
}

void training_jump_arrival() {
  // On the STAR, descending, under ACC -- before the TMA/approach handoff.
  // Position the aircraft at/after the STAR entry fix (e.g. SALEV for SALE3P),
  // descending, on the ACC/sector frequency, well before the IAF. The plugin
  // then drives the full arrival chain from here: ACC sector handoffs
  // (poll_acc_sector_change) -> approach handoff at the TMA (build_approach_
  // handoff) -> "cleared <appr> approach" at the IAF -> Tower at the FAF.
  auto ofp = simbrief_ofp::get();
  if (!ofp.valid || ofp.destination_icao.empty())
    logging::info("WARN training_jump_arrival: no valid OFP loaded -- no "
                  "destination/STAR/approach context. Load a SimBrief OFP "
                  "before jumping to ARR.");
  s_assigned_dest_icao = ofp.destination_icao;
  // Descent already issued (past TOD); the approach handoff has NOT fired yet
  // -- poll_arrival() must be free to issue it at the TMA/IAF.
  s_enroute_descent_issued          = true;
  s_enroute_approach_handoff_issued = false;
  s_enroute_approach_freq_mhz       = 0.0f;
  s_enroute_cleared_alt_ft          = 0; // set when the approach clearance fires
  // STAR / approach derived later: the approach handoff (CIFP + dest) and the
  // check-in handler both handle the training-jump case (engine.cpp ~1190).
  s_assigned_star_name.clear();
  s_assigned_approach_designator.clear();
  s_assigned_landing_runway.clear();
  // Reset approach + route trackers (mirror training_jump_approach).
  s_approach_waypoints.clear();
  s_approach_waypoint_idx     = 0;
  s_approach_timer            = 0.0f;
  s_approach_initial_fl       = 0;
  s_approach_final_issued     = false;
  s_approach_cleared_issued   = false;
  s_approach_tower_handed_off = false;
  s_approach_faf              = {};
  s_last_cleared_route_idx    = -1;
  s_faf_route_idx             = -1;
  s_iaf_route_idx             = -1;
  s_faf_ap_idx                = -1;
  s_map_ap_idx                = -1;
  s_approach_has_visual_final = false;
  s_no_star_direct_iaf.clear();
  s_route_fixes.clear();
  s_route_fix_idx             = 0;
  s_route_tracker_tick        = 0.0f;
  s_pending_route_direct.clear();
  s_sector_checkin_pending    = false;
  s_pending_controller_label.clear();
  s_pending_handoff_freq_mhz  = 0.0f;
  // ACC sector baseline: reseed so poll_acc_sector_change() (called by
  // poll_arrival) seeds silently to the current sector, then hands off on the
  // next boundary crossing.
  s_acc_sector_freq_khz       = 0;
  s_acc_visited_sector_freqs.clear();
  s_acc_sector_check_sec      = 0.0f;
  // Seed the controller label from the enclosing CTR sector (ACC) at the
  // current 3-D position -- the aircraft is under ACC on the STAR, not yet
  // Approach. Mirrors training_jump_enroute's Hardening 1.
  {
    const auto &ctx = xplane_context::get();
    s_current_controller_label.clear();
    s_jump_switch_freq_mhz = 0.0f;
    const auto sectors = airspace_db::find_enclosing(
        ctx.latitude, ctx.longitude, ctx.altitude_ft_msl);
    for (const auto *s : sectors) {
      if (s && s->role == airspace_db::ControllerRole::CTR &&
          !s->freqs_khz.empty()) {
        s_current_controller_label = controller_label_for(s);
        s_jump_switch_freq_mhz =
            static_cast<float>(s->freqs_khz.front()) / 1000.0f;
        break;
      }
    }
    if (s_current_controller_label.empty())
      s_current_controller_label = "Control";
    // Accurate openair sector NAME for the label (e.g. "Marseille" over the broad
    // atc.dat "France"), keeping the atc.dat frequency (LFMN 2026-07-20).
    if (openair_db::ready()) {
      const std::string oa = openair_sector_label(
          openair_db::find_enclosing(ctx.latitude, ctx.longitude,
                                     openair_alt(ctx))
              .name);
      if (!oa.empty())
        s_current_controller_label = oa;
    }
  }
  atc_state_machine::set_session_callsign(settings::pilot_callsign());
  s_jump_no_enroute_descent = true; // no en-route descent -> first ack "descend"
  atc_state_machine::set_state(atc_state_machine::ATCState::IFR_ARRIVAL);
  logging::info("training_jump_arrival: dest=%s controller=%s (position on the "
                "STAR, descending, under ACC, before the TMA)",
                s_assigned_dest_icao.c_str(),
                s_current_controller_label.c_str());
}

void training_jump_predep() {
  // Pre-departure clearance is on Delivery (or Ground if the field has no
  // Delivery frequency) -- surface it for the "Switch COM to X" popup.
  const auto &ctx = xplane_context::get();
  using FT = xplane_context::FrequencyType;
  float f = ctx.airport_freqs.first_mhz(FT::DELIVERY);
  if (f < 100.0f)
    f = ctx.airport_freqs.first_mhz(FT::GROUND);
  s_jump_switch_freq_mhz = f;
  atc_state_machine::set_state(atc_state_machine::ATCState::IFR_PREDEP_CLEARANCE);
}

int unclear_streak() { return unclear_streak_; }

int lm_inferences() { return lm_inferences_; }

// Lower-case copy used for keyword scanning. ASCII only — Whisper
// transcripts don't contain anything else.
static std::string to_lower_copy(const std::string &s) {
  std::string out = s;
  std::transform(out.begin(), out.end(), out.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  return out;
}

// Extract the multiset of digits-only tokens from a transcript. Used by
// the LM-repair validator below: a 3B model occasionally invents runway
// numbers / frequencies / altitudes that were never in the pilot's input
// (the example pattern in the prompt has been observed leaking into
// inputs that contain no number at all). If the repair carries a
// numeric token that the original lacks, we discard the repair and fall
// back to the raw Whisper transcript. Letters and "9er" → "9" mappings
// are deliberately ignored — only contiguous digit runs are compared.
static std::vector<std::string> extract_digit_tokens(const std::string &s) {
  std::vector<std::string> out;
  std::string cur;
  for (char c : s) {
    if (std::isdigit(static_cast<unsigned char>(c))) {
      cur += c;
    } else if (!cur.empty()) {
      out.push_back(cur);
      cur.clear();
    }
  }
  if (!cur.empty())
    out.push_back(cur);
  return out;
}

// True when `repaired` contains a digit token that is absent from
// `original`. The check is multiset-based so a repeated runway is fine
// as long as both sides have it. Catches the canonical hallucination:
//   original: "Clear for takeoff Delta Chari Hotel"  (no digits)
//   repaired: "Cleared for takeoff runway 06, ..."   (introduces "06")
static bool repair_invents_digits(const std::string &original,
                                  const std::string &repaired) {
  auto orig_digits = extract_digit_tokens(original);
  auto rep_digits = extract_digit_tokens(repaired);
  for (const auto &d : rep_digits) {
    auto it = std::find(orig_digits.begin(), orig_digits.end(), d);
    if (it == orig_digits.end())
      return true;
    orig_digits.erase(it);
  }
  return false;
}

// Plausibility guard against post-landing repair hallucinations. The
// 3B local LM occasionally rewrites "runway 06 located" (Whisper
// mishearing of "vacated") into "Cleared for takeoff runway 06" when
// it loses track of the just-landed context. Even with the prompt
// updated to forbid that, the model sometimes drifts; this hard check
// is a deterministic safety net.
//
// When `just_landed_flag` is true, any repair containing a tokenised
// takeoff/departure phrase is rejected outright. Caller falls back to
// the raw Whisper transcript.
static bool repair_violates_history(const std::string &repaired,
                                    bool just_landed_flag) {
  if (!just_landed_flag || repaired.empty())
    return false;
  std::string lower = to_lower_copy(repaired);
  static const char *kForbidden[] = {
      "cleared for takeoff", "clear for takeoff", "ready for departure",
      "ready for take",      "line up",
  };
  for (const char *needle : kForbidden) {
    if (lower.find(needle) != std::string::npos)
      return true;
  }
  return false;
}

// True if the transcript carries at least one identifiable ATC element
// (callsign extracted, runway extracted, or any of a handful of
// unambiguous EU/ICAO keywords). Used to distinguish a partially-
// understood transmission ("Tower ... runway 14 ...") from total
// noise. The set is deliberately small: words common across pilot
// requests AND readbacks, picked so a single match means the pilot
// was using radio phraseology even if Whisper killed a key word.
static bool has_recognisable_elements(const intent_parser::PilotMessage &msg) {
  if (!msg.callsign.empty())
    return true;
  if (!msg.runway.empty())
    return true;
  std::string t = to_lower_copy(msg.raw_transcript);
  static const char *kKeywords[] = {
      "tower",    "ground",    "approach",    "runway",  "request", "ready",
      "downwind", "base",      "final",       "holding", "qnh",     "wilco",
      "roger",    "departure", "information", "inbound", "vacated",
  };
  for (const char *kw : kKeywords) {
    if (t.find(kw) != std::string::npos)
      return true;
  }
  return false;
}

// Three-tier "I didn't get that" response. Increments unclear_streak_;
// the caller resets it when a valid intent finally lands. EU/ICAO
// phraseology (Doc 4444 / EU 2020/469):
//   - elements recognised        -> "garbled, say again"
//   - nothing recognised         -> "say again"
//   - 2nd unclear in a row       -> "say again, use standard phraseology"
static std::string
build_unclear_response(const intent_parser::PilotMessage &msg,
                       const std::string &fallback_cs) {
  // Benign acknowledgment -- a "direct <fix>" reply to a "confirm direct" query,
  // or a plain wilco/affirm/roger -- is NOT an unclear transmission. Answering it
  // with "say again, use standard phraseology" produced a loop even for a correct
  // "Direct SALEV" (LFLP 2026-07-17: the reply fell to the LM -> _INVALID). Treat
  // these as acknowledged: a simple "roger", and do not bump the unclear streak.
  {
    std::string t = msg.raw_transcript;
    for (char &c : t)
      c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    for (const char *k : {"direct", "wilco", "affirm", "roger"})
      if (t.find(k) != std::string::npos) {
        const std::string &scs = atc_state_machine::session_callsign();
        const std::string cs2 = !scs.empty() ? scs : fallback_cs;
        return cs2.empty() ? std::string("Roger.") : (cs2 + ", roger.");
      }
  }
  ++unclear_streak_;
  // Prefer the session-locked callsign so a mistranscribed utterance
  // ("Delta ...") cannot hijack the tower's salutation mid-session.
  const std::string &session_cs = atc_state_machine::session_callsign();
  std::string cs;
  if (!session_cs.empty())
    cs = session_cs;
  else if (!msg.callsign.empty())
    cs = msg.callsign;
  else
    cs = fallback_cs;
  std::string prefix = cs.empty() ? std::string{} : cs + ", ";

  if (unclear_streak_ >= 2)
    return prefix +
           atc_templates::lookup_fallback("say_again_use_standard_phraseology",
                                          "say again, use standard "
                                          "phraseology.");
  if (has_recognisable_elements(msg))
    return prefix + atc_templates::lookup_fallback(
                        "garbled_say_again",
                        "your transmission was garbled, say again.");
  return prefix + atc_templates::lookup_fallback("say_again", "say again.");
}

// Convenience for the quality-rejection path which has no parsed
// PilotMessage yet — only the raw transcript and a probably-empty
// callsign hint from the cockpit settings.
static std::string build_unclear_response_raw(const std::string &transcript,
                                              const std::string &fallback_cs) {
  intent_parser::PilotMessage stub;
  stub.raw_transcript = transcript;
  return build_unclear_response(stub, fallback_cs);
}

// Reset the back-to-back unclear counter. Called whenever a meaningful
// reply (template-rendered, traffic dialog, profanity etc.) is about to
// be returned to the pilot.
static void mark_clear() { unclear_streak_ = 0; }

static std::string build_profanity_response(int warning_number,
                                            const std::string &callsign) {
  if (warning_number == 1) {
    return callsign + ", maintain proper radio discipline. Use standard "
                      "phraseology on this frequency.";
  }
  if (warning_number == 2) {
    return callsign + ", this is your final warning. Continued inappropriate "
                      "language on this frequency will be reported to the "
                      "civil aviation authority. Use standard phraseology.";
  }
  return callsign + ", your conduct has been noted and will be reported to "
                    "the aviation authority. Maintain radio discipline "
                    "immediately.";
}

// Side-channel: when traffic_dialog is awaiting a pilot ack, route the
// transcript there first. Returns true if traffic_dialog handled it
// (the caller should skip the main flow). Updates advisory_history_'s
// visual-ack lockout when the pilot reported visual contact.
static bool try_traffic_dialog(const intent_parser::PilotMessage &msg,
                               const xplane_context::XPlaneContext &ctx,
                               double now_secs, Output &out) {
  if (!traffic_dialog::is_awaiting_ack())
    return false;

  uint32_t target_id = traffic_dialog::pending_target_id();
  auto reply = traffic_dialog::handle_pilot(msg, ctx);
  if (!reply.handled)
    return false;

  if (reply.acknowledged_with_visual)
    traffic_advisor::mark_acknowledged_visual(advisory_history_, target_id,
                                              now_secs);

  if (settings::debug_logging())
    logging::debug("Traffic dialog reply: %s",
                   reply.text.empty() ? "(silent)" : reply.text.c_str());
  out.parsed = msg;
  out.response_text = std::move(reply.text);
  // Pilot landed an intelligible TRAFFIC_* reply — break any in-flight
  // "say again" escalation.
  mark_clear();
  return true;
}

static Output run_state_machine(const intent_parser::PilotMessage &msg,
                                const xplane_context::XPlaneContext &ctx_now,
                                double now_secs) {
  auto atc_resp = atc_state_machine::process(msg, ctx_now, now_secs);
  if (settings::debug_logging())
    logging::debug("ATC response text: %s",
                   atc_resp.text.empty() ? "(silent)" : atc_resp.text.c_str());
  // A landed intent (rule parser or LM both produce non-UNKNOWN) means
  // the pilot was understood — even if the state machine subsequently
  // rejected the request via _INVALID/phase guard. Break the streak so
  // the next garbled call still starts at the friendly "garbled, say
  // again" tier rather than the escalation.
  if (msg.intent != intent_parser::PilotIntent::UNKNOWN)
    mark_clear();
  Output out;
  out.parsed = msg;
  out.response_text = atc_resp.text;
  return out;
}

// Is `mhz` a frequency the aircraft could plausibly reach RIGHT NOW -- the current
// IFR controller / a pending handoff, a published frequency of the nearest airport,
// or a controller of an airspace the aircraft is physically inside? Used to keep
// ATC SILENT when the pilot transmits on a frequency that exists nowhere near the
// aircraft (a typo, a non-existent freq, or a distant field) -- a real controller
// simply doesn't hear it (user 2026-07-19). Deliberately a WIDE net so a legitimate
// check-in is never silenced; only a truly-unreachable frequency draws silence.
static bool is_reachable_frequency(const xplane_context::XPlaneContext &ctx,
                                   float mhz) {
  if (mhz < 100.0f)
    return true; // no/again-invalid radio reading -> don't gate
  auto near = [mhz](float f) {
    return f > 100.0f && std::fabs(f - mhz) < 0.010f;
  };
  auto near_khz = [&](uint32_t khz) {
    return khz != 0 && near(static_cast<float>(khz) / 1000.0f);
  };
  // Current IFR controller / pending handoff trackers.
  if (near(s_enroute_approach_freq_mhz) || near(s_pending_handoff_freq_mhz) ||
      near_khz(s_acc_sector_freq_khz) || near_khz(s_approach_sector_freq_khz))
    return true;
  // Nearest airport's published frequencies (ATIS/Delivery/Ground/Tower/Approach).
  for (const auto &af : ctx.airport_freqs.all)
    if (near_khz(af.freq_khz))
      return true;
  // Controllers of the airspaces the aircraft is physically inside (atc.dat).
  for (const auto *c : ctx.enclosing_airspaces)
    if (c)
      for (uint32_t fk : c->freqs_khz)
        if (near_khz(fk))
          return true;
  return false;
}

void process_transcript(Input in, Done done) {
  if (settings::debug_logging())
    logging::debug("STT response (quality=%.2f): \"%s\"", in.quality,
                   in.transcript.c_str());

  // Poor transcript quality — likely noise or engine sounds. Even at
  // very low quality the transcript may still contain a recognised
  // ATC keyword, so route via the unclear-response builder instead of
  // the fixed "say again". Never the moment to land a valid intent,
  // so the streak counter advances normally.
  if (in.quality < 0.3f) {
    logging::info("Transcript quality too low, requesting say again");
    Output out;
    out.response_text =
        build_unclear_response_raw(in.transcript, in.pilot_callsign);
    done(std::move(out));
    return;
  }

  const auto &ctx = *in.ctx;

  // Deferred controller-label swap: the mid-flight handoff sites store the
  // NEW controller in s_pending_controller_label and keep the previous one
  // in s_current_controller_label until the pilot actually switches. Once
  // the pilot's active COM matches the pending handoff freq, the new
  // controller is the speaker — swap so the response + transcript show it
  // (e.g. "Milan", not "Torino Approach"). This runs for EVERY transmission
  // on the new freq, so it also covers the INITIAL_CALL_CENTER check-in
  // during SID climb, which bypasses the sector-checkin block below
  // (LIMF -> LFLP 2026-07-09: Milan check-in was labelled "Torino Approach").
  if (!s_pending_controller_label.empty() &&
      s_pending_handoff_freq_mhz > 100.0f) {
    const float active_lbl =
        ctx.active_com == 2 ? ctx.com2_freq_mhz : ctx.com1_freq_mhz;
    if (std::fabs(active_lbl - s_pending_handoff_freq_mhz) < 0.005f) {
      s_current_controller_label = s_pending_controller_label;
      s_pending_controller_label.clear();
      logging::info("Controller label -> %s (pilot reached handoff freq %.3f)",
                    s_current_controller_label.c_str(),
                    s_pending_handoff_freq_mhz);
    }
  }

  // Frequency guard: only process pilot transmissions on the correct frequency
  // for the current ATC state. A call on the wrong radio is silently ignored —
  // the pilot must retune and call again.
  {
    using AS = atc_state_machine::ATCState;
    using FT = xplane_context::FrequencyType;
    const auto state = atc_state_machine::get_state();
    const auto freq_t = ctx.frequency_type;
    bool wrong_freq = false;

    // Pending-handoff bypass: if the plugin just told the pilot to switch
    // to X.YYY, then the pilot's active COM matching X.YYY IS THE correct
    // frequency, regardless of what frequency_type derivation says.
    // Covers the CTR/CTA case (Milan Radar 118.675 sits in atc.dat with
    // CTR role; frequency_type stays UNKNOWN because the standard mapping
    // only recognises TRACON as APPROACH — a poll_enroute suppression
    // convention we do not want to break by widening that mapping).
    // Without this bypass, the plugin issues "contact Milan on 118.675"
    // then silently drops every pilot check-in on that same frequency.
    // See feedback_approach_freq_defines_intent: frequency the plugin just
    // handed off to is authoritative.
    const float active_com_ck =
        ctx.active_com == 2 ? ctx.com2_freq_mhz : ctx.com1_freq_mhz;
    const bool matches_pending_handoff =
        s_pending_handoff_freq_mhz > 100.0f &&
        std::fabs(active_com_ck - s_pending_handoff_freq_mhz) < 0.005f;

    // IFR airborne states that require APPROACH or DEPARTURE: pilot has been
    // handed off and must check in on the departure/approach frequency.
    if (matches_pending_handoff) {
      wrong_freq = false;  // pilot is on the exact freq we handed them off to
    } else if (s_sector_checkin_pending && s_pending_handoff_freq_mhz > 100.0f) {
      // Pending handoff active and pilot's freq doesn't match the target.
      // Torino 121.100 and Milan 118.675 are BOTH classified as APPROACH,
      // so the old (freq_t != APPROACH && freq_t != DEPARTURE) check was
      // too permissive — it let Torino-freq transmissions through as if
      // the pilot had switched to Milan. The freq the plugin just handed
      // off TO is the only correct freq until the pilot switches.
      // Non-READBACK / non-LEAVING_FREQUENCY intents on the old freq are
      // treated as wrong-freq; the reminder path below emits a spoken
      // "still on my frequency" call from the previous controller.
      const auto pi = in.pre_classified_intent;
      // The FIRST readback of the handoff is the expected acknowledgment
      // (readback still pending) -- accept it silently on the old freq. But if
      // the pilot READS BACK AGAIN / calls again on the old freq after that
      // (readback already consumed), they failed to switch -> reminder fires
      // ("negative, contact X"). LEAVING_FREQUENCY / UNABLE are always exempt.
      //
      // Gate on is_readback_pending() ALONE, not pi==READBACK: the rule parser
      // often scores a handoff readback UNKNOWN (garbled callsign/freq) and it
      // is only promoted to READBACK later by the "readback pending override".
      // pre_classified_intent is still UNKNOWN here, so the old pi==READBACK
      // check missed and fired "negative, contact France" on the pilot's very
      // FIRST correct readback, then looped (LIMF->LFLP 2026-07-11 Log 2117/2136).
      // While the readback is pending the transmission is routed to the readback
      // path (which consumes it); once consumed, pending is false and a repeat
      // call on the old freq correctly draws the reminder.
      // A handoff ACKNOWLEDGMENT on the old freq ("contact <X> on <freq>") is the
      // pilot READING BACK the handoff -- accept it silently even when the ACC
      // sector handoff never armed is_readback_pending() (the poll handoffs set
      // out_requires_readback but nothing wires that to readback_pending state, so
      // a correct readback of a Milan/Swiss/Geneva sector handoff drew "you are
      // still with X" on the very first read-back; LIMF->LFLP Swiss Radar
      // 2026-07-19). "contact" is the unambiguous handoff-readback marker here.
      const bool handoff_ack = [&] {
        std::string t = in.transcript;
        for (char &c : t)
          c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
        if (t.find("contact") != std::string::npos)
          return true;
        // A readback that NAMES the pending target (freq or controller) is a
        // handoff acknowledgment even without the word "contact" -- e.g.
        // "Chambery approach on 121.205" (approach handoff echo; LFLP 2026-07-20).
        if (s_pending_handoff_freq_mhz > 100.0f) {
          char fbuf[16];
          std::snprintf(fbuf, sizeof(fbuf), "%.3f", s_pending_handoff_freq_mhz);
          if (t.find(fbuf) != std::string::npos)
            return true;
        }
        if (!s_pending_controller_label.empty()) {
          std::string lbl = s_pending_controller_label;
          for (char &c : lbl)
            c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
          const auto sp = lbl.find(' '); // leading name token ("chambery")
          const std::string name = (sp == std::string::npos) ? lbl : lbl.substr(0, sp);
          if (name.size() >= 4 && t.find(name) != std::string::npos)
            return true;
        }
        return false;
      }();
      // A handoff READBACK on the OLD freq -> accept SILENTLY and STOP. Clearing
      // wrong_freq alone is not enough: the transmission would fall through to the
      // state machine, and an INITIAL_CALL_APPROACH readback there is processed as
      // the terminal CHECK-IN -- answered "radar contact, continue descent" spoken
      // by the OUTGOING controller (Geneva) inside the destination TMA (LFLP
      // 2026-07-20). The real check-in fires when the pilot is actually on the new
      // freq (matches_pending_handoff / sector-checkin detection), where the richer
      // approach handler responds. Silence here is correct -- the pilot is switching.
      if (handoff_ack) {
        logging::info("Handoff readback on old freq -- accepted silently (pending "
                      "%s %.3f)",
                      s_pending_controller_label.c_str(),
                      static_cast<double>(s_pending_handoff_freq_mhz));
        done(Output{});
        return;
      }
      const bool readback_or_leaving =
          atc_state_machine::is_readback_pending() ||
          pi == intent_parser::PilotIntent::LEAVING_FREQUENCY ||
          pi == intent_parser::PilotIntent::UNABLE;
      wrong_freq = !readback_or_leaving;
    } else if (state == AS::IFR_EN_ROUTE || state == AS::IFR_RADAR_CONTACT) {
      wrong_freq = (freq_t != FT::APPROACH && freq_t != FT::DEPARTURE);
    }
    // Ground/tower states: APPROACH and DEPARTURE are wrong.
    // Excluded from this guard:
    //   EN_ROUTE / APPROACH_CONTACT — VFR cross-country, unguarded (pilot may
    //   be on
    //     Tower or Approach depending on whether flight following has been
    //     established).
    //   IFR_DEPARTURE_CLEARED / IFR_FREQ_HANDOFF — IFR post-clearance; pilot
    //   may
    //     already have switched to the departure/approach frequency.
    else if (state != AS::UNICOM_ACTIVE && state != AS::IDLE &&
             state != AS::EN_ROUTE && state != AS::APPROACH_CONTACT &&
             state != AS::IFR_DEPARTURE_CLEARED &&
             state != AS::IFR_FREQ_HANDOFF && state != AS::IFR_ENROUTE_CRUISE &&
             state != AS::IFR_DESCENT && state != AS::IFR_ARRIVAL &&
             state != AS::IFR_APPROACH_CONTACT &&
             state != AS::IFR_APPROACH_DESCENT &&
             state != AS::IFR_APPROACH_TOWER &&
             state != AS::IFR_LANDING_CLEARED) {
      wrong_freq = (freq_t == FT::APPROACH || freq_t == FT::DEPARTURE ||
                    freq_t == FT::ATIS);
    }

    // Unreachable-frequency silence (airborne IFR states): if no handoff is
    // pending and the pilot's frequency exists NOWHERE near the aircraft (a typo,
    // a non-existent freq, or a distant field), the controller cannot hear it --
    // stay SILENT instead of answering on a frequency no one is on (user
    // 2026-07-19). The reachability net is wide (current controller / airport /
    // enclosing sector) so a legitimate check-in is never silenced.
    // NOT while a handoff is pending: the pilot is legitimately still on the OLD
    // controller's freq, which drops out of the reachable set once the handoff
    // moved the tracked sector freq to the new controller. The branch above already
    // decided that case (handoff readback accepted). Without this guard the
    // unreachable check flipped wrong_freq back to true and re-fired "you are still
    // with X" on the pilot's correct handoff READBACK (LIMF->LFLP Swiss/Geneva
    // 2026-07-20: the two alpha-69 changes -- handoff-ack accept + unreachable
    // silence -- collided; user "i am just reading back").
    if (!wrong_freq && !matches_pending_handoff && !s_sector_checkin_pending &&
        (state == AS::IFR_ENROUTE_CRUISE || state == AS::IFR_DESCENT ||
         state == AS::IFR_ARRIVAL || state == AS::IFR_APPROACH_CONTACT ||
         state == AS::IFR_APPROACH_DESCENT || state == AS::IFR_APPROACH_TOWER ||
         state == AS::IFR_LANDING_CLEARED || state == AS::IFR_EN_ROUTE ||
         state == AS::IFR_RADAR_CONTACT) &&
        !is_reachable_frequency(ctx, active_com_ck)) {
      wrong_freq = true;
      logging::info("Unreachable frequency %.3f MHz for state %s -- silent",
                    static_cast<double>(active_com_ck),
                    atc_state_machine::state_name(state));
    }

    if (wrong_freq) {
      // Reminder path: when a handoff is pending and the pilot transmits
      // on the OLD frequency, the previous controller re-issues the
      // handoff instead of going silent.  Real ATC never leaves a pilot
      // hanging on a frequency they should have already left.  Silence
      // was the old behaviour; the reminder replaces it because pilots
      // otherwise repeat the call three or four times wondering why
      // ATC isn't answering (see feedback_sector_checkin_ack and the
      // 2026-07-09 LIMF -> LFLP retest where the plugin instead
      // *impersonated* Milan on 121.100 due to the too-permissive
      // APPROACH classification check).
      if (s_sector_checkin_pending && s_pending_handoff_freq_mhz > 100.0f) {
        const std::string &sess_cs = atc_state_machine::session_callsign();
        const std::string &cs = sess_cs.empty() ? settings::pilot_callsign() : sess_cs;
        // Target of the reminder = pending controller (Milan).  The
        // speaker of this transmission is still the current controller
        // (Torino) — the transcript labels the message with
        // s_current_controller_label, not the target.
        const std::string &target_label =
            s_pending_controller_label.empty() ? "the next controller"
                                               : s_pending_controller_label;
        // "you are still with <current>, contact <target> on <freq>" -- a natural
        // "you haven't switched yet" nudge, not a "negative" that wrongly implies
        // the readback was incorrect (the readback content is usually fine; the
        // pilot simply hasn't changed frequency).
        const std::string &cur_label =
            s_current_controller_label.empty() ? "me" : s_current_controller_label;
        Output r;
        char buf[192];
        std::snprintf(buf, sizeof(buf),
                      "%s, you are still with %s, contact %s on %.3f.",
                      cs.c_str(), cur_label.c_str(), target_label.c_str(),
                      s_pending_handoff_freq_mhz);
        r.response_text = buf;
        r.is_warning    = true;
        logging::info("Wrong frequency (%s) for state %s -- reminder: contact %s on %.3f",
                      xplane_context::frequency_type_name(freq_t),
                      atc_state_machine::state_name(state),
                      target_label.c_str(), s_pending_handoff_freq_mhz);
        done(std::move(r));
        return;
      }
      logging::info("Wrong frequency (%s) for state %s -- ignoring",
                    xplane_context::frequency_type_name(freq_t),
                    atc_state_machine::state_name(state));
      done(Output{});
      return;
    }
  }

  // Sector check-in detection: the first pilot transmission on the new sector
  // frequency after a handoff instruction. Two responsibilities:
  //   (1) clear s_sector_checkin_pending so poll_* loops resume proactive
  //       messages (descent step-downs, sector handoffs, etc.);
  //   (2) MUST emit an acknowledgment — real ATC never lets a check-in on a
  //       new frequency go unanswered. Silent clears in the past led to the
  //       pilot repeating the call three or four times with no reply
  //       (see feedback_sector_checkin_ack).
  //
  // Exception: the initial Approach check-in (IFR_APPROACH_CONTACT) and the
  // Tower/AFIS check-in (IFR_APPROACH_TOWER) have their own richer handlers
  // below that produce a full clearance ("radar contact, identified, direct
  // X, RNAV approach RWY Y, descend Z feet, QNH W" / "runway X, cleared to
  // land"). For those states, only clear the flag here and let the specific
  // handler emit the response.
  bool sector_checkin_just_fired = false;
  // A course-correction READBACK ("confirm direct SALEV", "direct PIRUV") on the
  // current frequency must NOT be mistaken for a sector check-in -- the bare "radar
  // contact" ack is wrong for a readback (LFLP 2026-07-16). Skip the check-in
  // treatment for readback-like transmissions; normal processing handles them.
  const bool readback_like = [&] {
    std::string t = in.transcript;
    for (char &c : t)
      c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    // Readback / acknowledgement markers -- never a fresh sector check-in. Covers
    // course ("confirm direct"), speed ("reduce speed / knots"), and plain acks
    // (wilco/roger). A readback echoing one of these must not get a bare "radar
    // contact" (LFLP 2026-07-17).
    for (const char *k : {"direct", "confirm", "knots", "reduce speed", "wilco",
                          "roger"})
      if (t.find(k) != std::string::npos)
        return true;
    return false;
  }();
  if (s_sector_checkin_pending && s_pending_handoff_freq_mhz > 0.0f &&
      !readback_like) {
    const float active = ctx.active_com == 2 ? ctx.com2_freq_mhz : ctx.com1_freq_mhz;
    if (std::fabs(active - s_pending_handoff_freq_mhz) < 0.005f) {
      s_sector_checkin_pending = false;
      sector_checkin_just_fired = true;
      // Pilot actually switched to the new frequency — swap the pending
      // controller label into current so the response to this check-in
      // (and every subsequent transmission) is labelled as the new
      // controller in transcript.log.
      if (!s_pending_controller_label.empty()) {
        s_current_controller_label = s_pending_controller_label;
        s_pending_controller_label.clear();
      }
      logging::info("Sector checkin: pilot first call on %.3f MHz -- resuming proactive messages",
                    active);
      using AS = atc_state_machine::ATCState;
      const auto ck_state = atc_state_machine::get_state();
      // Defer to the richer approach/tower clearance handler ONLY once the
      // approach phase has actually been entered: IFR_APPROACH_CONTACT (set by
      // poll_arrival Stage B at/near the terminating IAF, PIRUV) or the Tower leg.
      // An intermediate-TMA check-in -- IFR_ARRIVAL / IFR_DESCENT on the approach
      // freq, e.g. Geneva far out on the STAR -- must NOT get the approach
      // clearance (Geneva has no authority to clear the approach from there;
      // several TMAs are crossed on a STAR and none of them is the approach). It
      // gets the bare "radar contact" sector ack below and stays in ARRIVAL; the
      // terminal controller (Chambery) issues the clearance near the IAF.
      const bool defer_to_richer_handler =
          ck_state == AS::IFR_APPROACH_CONTACT ||
          ck_state == AS::IFR_APPROACH_TOWER;
      if (!defer_to_richer_handler) {
        const std::string &cs_ref_ck = atc_state_machine::session_callsign();
        const std::string &cs_ck =
            cs_ref_ck.empty() ? in.pilot_callsign : cs_ref_ck;
        // Verify the pilot's stated level against the ASSIGNED cleared level and
        // re-state it on the handoff check-in if they differ -- the assigned FL/alt
        // carries across controllers, so a check-in with the wrong level
        // ("descending FL100" when cleared FL090) must be corrected, not silently
        // "radar contact"-ed (LFLP 2026-07-17). Re-stating the assigned level is
        // never wrong (it IS the clearance), so this is safe even when the pilot was
        // merely reporting their current altitude mid-descent.
        std::string restate;
        const int assigned_ft = current_cleared_alt_ft();
        if (assigned_ft > 0) {
          int stated_ft = 0;
          std::string t = in.transcript;
          for (char &ch : t)
            ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
          std::smatch mm;
          static const std::regex kFlRe(R"((?:flight level|fl)\s*(\d{2,3}))");
          static const std::regex kFtRe(R"((\d{3,5})\s*fe?e?t)");
          if (std::regex_search(t, mm, kFlRe))
            stated_ft = std::stoi(mm[1].str()) * 100;
          else if (std::regex_search(t, mm, kFtRe))
            stated_ft = std::stoi(mm[1].str());
          if (stated_ft > 0 && std::abs(stated_ft - assigned_ft) > 500) {
            const int ta_ck =
                ctx.transition_alt_ft > 0 ? ctx.transition_alt_ft : 5000;
            // Inline FL-vs-feet (format_alt_clearance is defined later in the TU):
            // FL above the transition altitude, feet below.
            char lvlbuf[32];
            if (assigned_ft >= ta_ck)
              std::snprintf(lvlbuf, sizeof(lvlbuf), "flight level %d",
                            assigned_ft / 100);
            else
              std::snprintf(lvlbuf, sizeof(lvlbuf), "%d feet", assigned_ft);
            const std::string lvl = lvlbuf;
            const int pa_i = static_cast<int>(ctx.pressure_alt_ft);
            const char *verb = (assigned_ft < pa_i - 200)   ? "descend"
                               : (assigned_ft > pa_i + 200) ? "climb"
                                                            : "maintain";
            restate = std::string(", ") + verb + " " + lvl;
            logging::info("Sector checkin: stated %d ft vs assigned %d ft -> re-state",
                          stated_ft, assigned_ft);
          }
        }
        char ack_buf[192];
        std::snprintf(ack_buf, sizeof(ack_buf), "%s, radar contact%s.",
                      cs_ck.c_str(), restate.c_str());
        logging::info("Sector checkin ack: %s (state=%s)",
                      ack_buf, atc_state_machine::state_name(ck_state));
        Output out_ck;
        out_ck.response_text = ack_buf;
        done(std::move(out_ck));
        return;
      }
    }
  }
  (void)sector_checkin_just_fired;

  // Parse intent
  auto parsed = intent_parser::parse(in.transcript, ctx);

  // LM re-entry: override rule-based result with pre-classified intent so
  // IFR-specific handlers in this function fire for LM-classified intents.
  if (in.pre_classified_intent != intent_parser::PilotIntent::UNKNOWN) {
    parsed.intent     = in.pre_classified_intent;
    parsed.confidence = in.pre_classified_conf;
  }

  // Frequency-based ATC type promotion: the ATC type is determined by the
  // frequency the pilot is transmitting on, NOT by the spoken facility name.
  // If the pilot is on an Approach (or Departure) frequency while en-route,
  // any check-in-family intent is promoted to INITIAL_CALL_APPROACH regardless
  // of what words were spoken (e.g. Voxtral mishears "Approach" as "Information",
  // the pilot says "Paris information" but the frequency is 118.050 Melun APP).
  // This runs only on the first pass; LM re-entry already has the correct intent.
  if (in.pre_classified_intent == intent_parser::PilotIntent::UNKNOWN) {
    using FT  = xplane_context::FrequencyType;
    using PI2 = intent_parser::PilotIntent;
    using AS2 = atc_state_machine::ATCState;
    const auto ft  = ctx.frequency_type;
    const auto ast = atc_state_machine::get_state();
    if ((ft == FT::APPROACH || ft == FT::DEPARTURE) &&
        (ast == AS2::IFR_ENROUTE_CRUISE ||
         ast == AS2::IFR_DESCENT        ||
         ast == AS2::IFR_ARRIVAL        ||
         ast == AS2::IFR_APPROACH_CONTACT)) {
      const bool is_explicit_call =
          parsed.intent == PI2::INITIAL_CALL         ||
          parsed.intent == PI2::INITIAL_CALL_INBOUND ||
          parsed.intent == PI2::INITIAL_CALL_TOWER   ||
          parsed.intent == PI2::INITIAL_CALL_CENTER  ||
          parsed.intent == PI2::RADIO_CHECK;
      // A bare UNKNOWN on the approach freq is only a check-in when one is actually
      // EXPECTED (a handoff is pending, or we are in APPROACH_CONTACT). Otherwise
      // it is almost always a garbled READBACK -- reclassifying it to
      // INITIAL_CALL_APPROACH turned course/speed readbacks into a "radar contact"
      // check-in ack (LFLP 2026-07-17). Never override while a readback is pending.
      const bool checkin_expected =
          s_sector_checkin_pending || ast == AS2::IFR_APPROACH_CONTACT;
      const bool unknown_is_checkin =
          parsed.intent == PI2::UNKNOWN && checkin_expected &&
          !atc_state_machine::is_readback_pending();
      if (is_explicit_call || unknown_is_checkin) {
        logging::info("Freq override: %s on %s -> INITIAL_CALL_APPROACH",
                      intent_parser::intent_name(parsed.intent),
                      xplane_context::frequency_type_name(ft));
        parsed.intent     = PI2::INITIAL_CALL_APPROACH;
        parsed.confidence = std::max(parsed.confidence, 0.85f);
      }
    }
  }

  if (settings::debug_logging())
    logging::debug("Intent: %s (confidence=%.2f), callsign=%s",
                   intent_parser::intent_name(parsed.intent), parsed.confidence,
                   parsed.callsign.empty() ? "(none)"
                                           : parsed.callsign.c_str());

  // Traffic dialog short-circuit. When the controller is awaiting a
  // pilot acknowledgement of a traffic advisory and the pilot just
  // matched a TRAFFIC_* intent at high confidence, route directly there
  // and skip the main flow + LM disambig.
  if (traffic_dialog::is_awaiting_ack() &&
      (parsed.intent == intent_parser::PilotIntent::TRAFFIC_IN_SIGHT ||
       parsed.intent == intent_parser::PilotIntent::TRAFFIC_NEGATIVE_CONTACT ||
       parsed.intent == intent_parser::PilotIntent::TRAFFIC_LOOKING) &&
      parsed.confidence >= 0.7f) {
    Output out;
    if (try_traffic_dialog(parsed, ctx, in.now_secs, out)) {
      done(std::move(out));
      return;
    }
  }

  // Inappropriate language — intercept before state machine.
  // Does NOT change ATC state, pilot can continue normally after.
  if (parsed.intent == intent_parser::PilotIntent::INAPPROPRIATE_LANGUAGE) {
    ++profanity_warnings_;
    const std::string &session_cs = atc_state_machine::session_callsign();
    std::string cs;
    if (!session_cs.empty())
      cs = session_cs;
    else if (!parsed.callsign.empty())
      cs = parsed.callsign;
    else
      cs = in.pilot_callsign;
    logging::info("Radio discipline warning #%d", profanity_warnings_);
    Output out;
    out.parsed = parsed;
    out.response_text = build_profanity_response(profanity_warnings_, cs);
    out.is_warning = true;
    // Coherent (if rude) utterance — no "say again" loop carries over.
    mark_clear();
    done(std::move(out));
    return;
  }

  using PI = intent_parser::PilotIntent;

  // IFR en-route descent request: set flag so poll_enroute fires the
  // clearance on the next frame. No state-machine response here.
  if (parsed.intent == PI::REQUEST_DESCENT &&
      atc_state_machine::get_state() ==
          atc_state_machine::ATCState::IFR_ENROUTE_CRUISE) {
    s_pilot_requested_descent = true;
    // Extract the requested FL so poll_enroute can match it against the
    // navlog before deciding whether this is an en-route step or the TOD.
    {
      static const std::regex kFlRe(R"((?:flight\s+level|fl)\s+(\d{2,3}))",
                                    std::regex_constants::icase);
      std::smatch m;
      if (std::regex_search(in.transcript, m, kFlRe))
        s_pilot_requested_fl_ft = std::stoi(m[1].str()) * 100;
      else
        s_pilot_requested_fl_ft = 0;
    }
    done(Output{});
    return;
  }

  // IFR en-route climb request: pilot asks for a higher FL.
  // Issue cruise FL clearance if aircraft is below cruise altitude;
  // otherwise maintain current level.
  if (parsed.intent == PI::REQUEST_HIGHER &&
      atc_state_machine::get_state() ==
          atc_state_machine::ATCState::IFR_ENROUTE_CRUISE) {
    const std::string &cs_r = atc_state_machine::session_callsign();
    const std::string cs_h = cs_r.empty() ? in.pilot_callsign : cs_r;
    const xplane_context::XPlaneContext &ctx_h = *in.ctx;
    int cruise_fl = 0;
    if (ctx_h.ifr_cruise_alt_ft > 0)
      cruise_fl = round_to_fl(ctx_h.ifr_cruise_alt_ft);
    else if (s_enroute_cleared_alt_ft > 0)
      cruise_fl = round_to_fl(s_enroute_cleared_alt_ft + 2000);
    Output out_h;
    if (cruise_fl > 0 &&
        cruise_fl * 100 >
            static_cast<int>(ctx_h.altitude_ft_msl) + 500) {
      s_enroute_cleared_alt_ft = cruise_fl * 100;
      char buf[128];
      std::snprintf(buf, sizeof(buf), "%s, climb flight level %d.",
                    cs_h.c_str(), cruise_fl);
      out_h.response_text = buf;
      logging::info("IFR en-route: REQUEST_HIGHER -> FL%d", cruise_fl);
    } else {
      // Already at or above cruise altitude
      int fl_now = round_to_fl(static_cast<int>(ctx_h.altitude_ft_msl));
      char buf[128];
      std::snprintf(buf, sizeof(buf), "%s, maintain flight level %d.",
                    cs_h.c_str(), fl_now);
      out_h.response_text = buf;
    }
    done(std::move(out_h));
    return;
  }

  // (removed) "Early approach call while still en-route" promotion.
  // Reading the pilot's intent word ("Radar" / "Approach") to jump state
  // to IFR_APPROACH_CONTACT is an anti-pattern: it lets any en-route
  // ACC/Center check-in ("Milano Radar", "Marseille Control") trigger
  // the DESTINATION approach clearance mid-cruise. See
  // feedback_approach_freq_defines_intent — state transitions must be
  // frequency- and geometry-driven, never phraseology-driven. Approach
  // state is entered ONLY when the plugin itself issued a "contact
  // Approach on X" handoff (poll_enroute / poll_descent), setting state
  // and s_enroute_approach_freq_mhz. The check-in handler below then
  // fires the full clearance gated on frequency match, not on words.

  // IFR Approach check-in: intercept INITIAL_CALL_APPROACH in APPROACH_CONTACT
  // to issue "identified, descend FL[initial]" directly (template cannot hold
  // the dynamic FL). Transitions state to IFR_APPROACH_DESCENT.
  // Gate on the pilot being on the approach frequency — prevents a readback
  // on the previous Centre frequency from triggering the check-in.
  const bool on_approach_freq =
      // "Unknown approach freq" defaults TRUE only when no ACC sector handoff
      // is in progress. While the aircraft is being handed Milan -> France ->
      // Marseille (poll_acc_sector_change active, s_acc_sector_freq_khz != 0)
      // the real Approach freq is not set yet; without this guard the ACC
      // handoff readback ("Contact France on 118.030") tripped the approach
      // check-in and promoted to IFR/APPROACH_DESCENT before the STAR entry
      // (LIMF -> LFLP 2026-07-11). Once build_approach_handoff sets the real
      // Approach freq, the match branch below fires normally.
      (s_enroute_approach_freq_mhz < 100.0f && s_acc_sector_freq_khz == 0) ||
      std::fabs((ctx.active_com == 1 ? ctx.com1_freq_mhz : ctx.com2_freq_mhz) -
                s_enroute_approach_freq_mhz) < 0.010f;
  // Any initial-call variant OR unknown/unrecognised call on the approach frequency
  // while waiting for check-in fires the handler. The state + frequency gates are
  // tight enough that a false positive is not possible here.
  const bool is_initial_call_any =
      parsed.intent == PI::INITIAL_CALL_APPROACH ||
      parsed.intent == PI::INITIAL_CALL_CENTER    ||
      parsed.intent == PI::INITIAL_CALL           ||
      parsed.intent == PI::INITIAL_CALL_INBOUND   ||
      parsed.intent == PI::UNKNOWN;
  // State gate: normally the Approach check-in fires in IFR_APPROACH_CONTACT
  // (the state the plugin sets after issuing "contact Approach"). Also
  // accepted: IFR_DESCENT — when the aircraft is still descending under
  // Centre when it crosses into the Approach sector, the pilot may check in
  // on the Approach freq while the plugin's state hasn't auto-advanced yet.
  // The frequency gate above already confirms the pilot IS on the Approach
  // freq, so firing the full clearance here is correct.
  // Fire ONLY in IFR_APPROACH_CONTACT -- the state poll_arrival Stage B sets at/
  // near the terminating IAF (PIRUV). IFR_DESCENT / IFR_ARRIVAL are deliberately
  // excluded: crossing a TMA on the STAR (Geneva, far from the IAF) is NOT the
  // approach phase, so a check-in there gets the bare sector ack, never the
  // approach clearance + init_route_fixes (which was jumping the tracker past the
  // STAR to the approach fixes -- LIMF->LFLP 2026-07-14).
  const auto ck_st = atc_state_machine::get_state();
  const bool checkin_state_ok =
      ck_st == atc_state_machine::ATCState::IFR_APPROACH_CONTACT;
  // Suppress the approach check-in when the pilot owes a readback of an
  // ALTITUDE clearance (descend/climb/flight level). That transmission is
  // the readback — often echoing "expect RNAV Zulu approach" — not a fresh
  // check-in. Without this, a FL140 descent readback was misclassified as
  // INITIAL_CALL_APPROACH and fired a premature "continue descent FL090"
  // 29 s after the FL140 clearance, promoting to APPROACH_DESCENT while
  // still with the previous controller (LIMF -> LFLP 2026-07-10). Real
  // ATC rule: the previous FL clearance stands across handoff; a lower FL
  // comes only from a proactive step-down at the constraint or a pilot
  // request. A pending FREQUENCY-handoff readback ("contact Approach on
  // X") is EXEMPT — checking in on the new freq IS the acknowledgment.
  bool alt_readback_pending = false;
  if (atc_state_machine::is_readback_pending()) {
    const std::string &cl = atc_state_machine::last_clearance_text();
    auto has = [&](const char *k) { return cl.find(k) != std::string::npos; };
    const bool is_handoff = has("contact") || has("Contact");
    const bool is_alt = has("descend") || has("climb") || has("flight level");
    alt_readback_pending = is_alt && !is_handoff;
  }
  if (is_initial_call_any &&
      on_approach_freq &&
      checkin_state_ok &&
      !alt_readback_pending) {
    using AS = atc_state_machine::ATCState;
    const std::string &cs_ref = atc_state_machine::session_callsign();
    const std::string cs = cs_ref.empty() ? in.pilot_callsign : cs_ref;

    // Early-approach / training: build_descent_clearance() may not have run yet,
    // leaving s_assigned_dest_icao empty. Seed it from the OFP now so the
    // STAR-derivation, early-approach, and no-STAR blocks below can fire.
    if (s_assigned_dest_icao.empty()) {
      const auto &ofp_seed = simbrief_ofp::get();
      if (!ofp_seed.destination_icao.empty()) {
        s_assigned_dest_icao = ofp_seed.destination_icao;
        logging::info("[approach] seeded dest ICAO from OFP: %s",
                      s_assigned_dest_icao.c_str());
      }
    }

    // Training jump: s_assigned_star_name not set — derive from OFP last fix.
    // The last navlog fix before the destination is the STAR entry point;
    // CIFP maps (entry_fix, dest_runway) -> STAR name.
    if (s_assigned_star_name.empty() && !s_assigned_dest_icao.empty() &&
        !ctx.cifp_dir.empty()) {
      const auto &ofp_tj = simbrief_ofp::get();
      std::string entry_fix;
      for (int i = static_cast<int>(ofp_tj.navlog.size()) - 1; i >= 0; --i) {
        const auto &f = ofp_tj.navlog[i];
        if (!f.ident.empty() && f.ident != s_assigned_dest_icao) {
          entry_fix = f.ident;
          break;
        }
      }
      if (!entry_fix.empty()) {
        const std::string dest_rwy = pick_arrival_runway(ctx, s_assigned_dest_icao);
        s_assigned_star_name = cifp_reader::star_name_for_entry_fix(
            ctx.cifp_dir, s_assigned_dest_icao, dest_rwy, entry_fix);
        if (s_assigned_star_name.empty())
          s_assigned_star_name = cifp_reader::star_name_for_entry_fix(
              ctx.cifp_dir, s_assigned_dest_icao, "", entry_fix);
      }
      if (s_assigned_star_name.empty()) {
        const std::string dest_rwy = pick_arrival_runway(ctx, s_assigned_dest_icao);
        if (!dest_rwy.empty())
          s_assigned_star_name = cifp_reader::first_star_for_runway(
              ctx.cifp_dir, s_assigned_dest_icao, dest_rwy);
      }
    }

    // Load STAR waypoints now so poll_approach can use them.
    if (!s_assigned_star_name.empty() && !s_assigned_dest_icao.empty() &&
        s_approach_waypoints.empty()) {
      s_approach_waypoints = cifp_reader::star_waypoints(
          ctx.cifp_dir, s_assigned_dest_icao, s_assigned_star_name);
    }

    // Confirm approach type and append IAF-transition waypoints.
    // Must run before altitude selection so all constrained waypoints are loaded.
    std::string approach_confirm;
    if (!s_assigned_star_name.empty() && !s_assigned_dest_icao.empty() &&
        !ctx.cifp_dir.empty()) {
      std::string rwy = cifp_reader::runway_for_star(
          ctx.cifp_dir, s_assigned_dest_icao, s_assigned_star_name);
      // STAR may serve all runways — use wind-favoured runway in that case.
      if (rwy.empty())
        rwy = pick_arrival_runway(ctx, s_assigned_dest_icao);
      if (!rwy.empty()) {
        const auto &ofp_ac = simbrief_ofp::get();
        cifp_reader::ApproachInfo appr;
        // Consistency lock (LFMN 2026-07-19): the approach was already CHOSEN and
        // SPOKEN at the descent "expect <appr> approach" briefing and stored in
        // s_assigned_approach_designator. Re-running preferred_approach() here would
        // re-evaluate the weather gate on LOCAL aircraft weather, which differs
        // between the far-out briefing point (clear aloft -> R04LA "Alpha") and the
        // IAF (marine layer -> R04LZ "Zulu") -- so the pilot heard "expect Alpha"
        // then "cleared Zulu". ATC never silently swaps the approach at the IAF:
        // reuse the designator locked at the briefing. Empty (no STAR briefing yet)
        // falls through to the normal OFP/airport+/best_approach resolution below.
        if (!s_assigned_approach_designator.empty())
          appr = cifp_reader::approach_by_designator(
              ctx.cifp_dir, s_assigned_dest_icao, s_assigned_approach_designator);
        if (appr.type_str.empty() && !ofp_ac.preferred_approach_designator.empty())
          appr = cifp_reader::approach_by_designator(
              ctx.cifp_dir, s_assigned_dest_icao,
              ofp_ac.preferred_approach_designator);
        if (appr.type_str.empty()) {
          // Per-airport preferred approach (airport+.json), weather-gated:
          // e.g. LFMN 04L -> R04LA when vis>=10km & ceiling>=2500ft, else R04LZ.
          const std::string pref = airport_overrides::preferred_approach(
              s_assigned_dest_icao, rwy, approach_gate_vis_m(ctx),
              approach_gate_ceiling_ft(ctx));
          if (!pref.empty())
            appr = cifp_reader::approach_by_designator(
                ctx.cifp_dir, s_assigned_dest_icao, pref);
        }
        if (appr.type_str.empty())
          appr = cifp_reader::best_approach(
              ctx.cifp_dir, s_assigned_dest_icao, rwy, approach_gate_vis_m(ctx));
        if (!appr.type_str.empty()) {
          // Persist the CIFP runway so Tower uses the correct landing runway
          // regardless of which airport ctx.active_runway points to.
          s_assigned_landing_runway = appr.runway;
          // Also push into the state machine so template-lookup {runway}
          // resolves to the assigned CIFP runway, not ctx.active_runway
          // (which can be wind-selected for the wrong end, e.g. RWY 07 approach
          // vs RWY 29 wind-favoured on calm-wind days).
          atc_state_machine::set_assigned_runway(appr.runway);
          // Look up FAF position so poll_approach() can trigger Tower handoff.
          if (!appr.designator.empty()) {
            s_approach_faf = cifp_reader::approach_faf(
                ctx.cifp_dir, s_assigned_dest_icao, appr.designator);
            s_assigned_approach_designator = appr.designator;
            // Append IAF-transition waypoints (skip FM vectoring + IF entry).
            const std::string iaf =
                cifp_reader::star_last_fix(ctx.cifp_dir, s_assigned_dest_icao,
                                           s_assigned_star_name);
            if (!iaf.empty()) {
              auto proc = cifp_reader::approach_procedure_waypoints(
                  ctx.cifp_dir, s_assigned_dest_icao, appr.designator, iaf);
              if (!proc.empty()) {
                for (auto &w : proc)
                  s_approach_waypoints.push_back(w);
                s_approach_final_issued = true;
                // Locate FAF and MAP in the waypoint array once, so
                // poll_approach can skip GO_AROUND territory efficiently.
                s_faf_ap_idx = -1;
                s_map_ap_idx = -1;
                for (int i = 0; i < static_cast<int>(s_approach_waypoints.size()); ++i) {
                  const auto &w = s_approach_waypoints[i];
                  if (s_faf_ap_idx < 0 && w.is_approach_proc &&
                      w.ident == s_approach_faf.ident)
                    s_faf_ap_idx = i;
                  if (s_map_ap_idx < 0 && w.is_approach_proc && w.is_map)
                    s_map_ap_idx = i;
                }
                logging::info("[route] FAF ap_idx=%d MAP ap_idx=%d",
                              s_faf_ap_idx, s_map_ap_idx);
              }
            }
          }
          // Dedup: the check-in ack no longer restates the approach identity
          // ("radar contact, identified, RNAV Zulu approach runway 04, ...").
          // The single authoritative approach clearance is issued once at the
          // IAF by poll_approach (see project_arrival_announcement_model). The
          // ack stays informational: "radar contact, identified, descend ...".
          // (STAR path -- approach_confirm intentionally left empty here; all
          // the FAF / waypoint / runway SETUP above is retained.)
        }
      }
    }

    // Early-approach path: pilot called Approach before poll_descent() ran, so
    // s_assigned_approach_designator is still empty. Assign an approach now so
    // that:
    //   (a) the no-STAR block below loads s_approach_faf + s_approach_waypoints,
    //   (b) s_approach_final_issued is set → poll_approach() FAF gate opens →
    //       INFO/Tower handoff fires,
    //   (c) initial_ft comes from real waypoint altitudes instead of defaults_ft.
    if (s_assigned_star_name.empty() && s_assigned_approach_designator.empty() &&
        !s_assigned_dest_icao.empty() && !ctx.cifp_dir.empty()) {
      std::string dest_rwy = pick_arrival_runway(ctx, s_assigned_dest_icao);
      if (!dest_rwy.empty()) {
        const auto &ofp_early = simbrief_ofp::get();
        cifp_reader::ApproachInfo appr_early;
        if (!ofp_early.preferred_approach_designator.empty())
          appr_early = cifp_reader::approach_by_designator(
              ctx.cifp_dir, s_assigned_dest_icao,
              ofp_early.preferred_approach_designator);
        if (appr_early.type_str.empty()) {
          const std::string pref = airport_overrides::preferred_approach(
              s_assigned_dest_icao, dest_rwy, approach_gate_vis_m(ctx),
              approach_gate_ceiling_ft(ctx));
          if (!pref.empty())
            appr_early = cifp_reader::approach_by_designator(
                ctx.cifp_dir, s_assigned_dest_icao, pref);
        }
        if (appr_early.type_str.empty())
          appr_early = cifp_reader::best_approach(
              ctx.cifp_dir, s_assigned_dest_icao, dest_rwy, approach_gate_vis_m(ctx));
        if (!appr_early.type_str.empty()) {
          s_assigned_approach_designator = appr_early.designator;
          s_assigned_landing_runway      = appr_early.runway;
          logging::info("[approach] early check-in: no designator — assigned %s rwy %s",
                        appr_early.designator.c_str(), appr_early.runway.c_str());
        }
      }
    }

    // No-STAR path: approach designator was set by descent clearance but no
    // STAR was found or assigned (e.g. LFQA, small airports).  Load the
    // approach fix data so poll_approach() can trigger the Tower/INFO handoff
    // at the FAF — without this s_approach_final_issued stays false and the
    // handoff never fires.
    if (s_assigned_star_name.empty() && !s_assigned_approach_designator.empty() &&
        !s_assigned_dest_icao.empty() && !ctx.cifp_dir.empty() &&
        approach_confirm.empty()) {
      cifp_reader::ApproachInfo appr_ns = cifp_reader::approach_by_designator(
          ctx.cifp_dir, s_assigned_dest_icao, s_assigned_approach_designator);
      if (!appr_ns.type_str.empty()) {
        s_assigned_landing_runway = appr_ns.runway;
        atc_state_machine::set_assigned_runway(appr_ns.runway);
        if (s_approach_faf.ident.empty())
          s_approach_faf = cifp_reader::approach_faf(
              ctx.cifp_dir, s_assigned_dest_icao, appr_ns.designator);
        std::string iaf_ns; // IAF closest to aircraft; hoisted for approach_confirm
        if (s_approach_waypoints.empty()) {
          auto iaf_ids = cifp_reader::approach_transition_idents(
              ctx.cifp_dir, s_assigned_dest_icao, appr_ns.designator);
          // Pick the IAF closest to the aircraft (avoids directing to a fix
          // already behind the aircraft). Same logic as build_descent_clearance.
          if (!iaf_ids.empty()) {
            if (iaf_ids.size() == 1) {
              iaf_ns = iaf_ids[0];
            } else {
              auto iaf_pos = cifp_reader::lookup_fix_positions(
                  ctx.cifp_dir, iaf_ids, s_assigned_dest_icao);
              double best_d = 1e9;
              for (const auto &id : iaf_ids) {
                auto it = iaf_pos.find(id);
                if (it == iaf_pos.end()) continue;
                double d = traffic_geometry::distance_nm(
                    ctx.latitude, ctx.longitude,
                    it->second.first, it->second.second);
                if (d < best_d) { best_d = d; iaf_ns = id; }
              }
              if (iaf_ns.empty()) iaf_ns = iaf_ids[0];
            }
          }
          auto proc_ns = cifp_reader::approach_procedure_waypoints(
              ctx.cifp_dir, s_assigned_dest_icao, appr_ns.designator, iaf_ns);
          s_faf_ap_idx = -1; s_map_ap_idx = -1;
          for (auto &w : proc_ns) {
            int widx = static_cast<int>(s_approach_waypoints.size());
            s_approach_waypoints.push_back(w);
            if (s_faf_ap_idx < 0 && w.is_approach_proc &&
                w.ident == s_approach_faf.ident)
              s_faf_ap_idx = widx;
            if (s_map_ap_idx < 0 && w.is_approach_proc && w.is_map)
              s_map_ap_idx = widx;
          }
        }
        // Dedup: the check-in ack no longer restates the approach identity
        // (the single approach clearance is issued at the IAF by poll_approach;
        // see project_arrival_announcement_model). Only the direct-to-IAF is
        // kept -- a direct to an IAF IS a legitimate shortcut (cancels the
        // STAR, enters the approach). No-STAR: if the en-route descent never
        // issued the direct-to-IAF (Condition A/B/C unmet), issue it now.
        if (s_no_star_direct_iaf.empty() && !iaf_ns.empty()) {
          approach_confirm = ", direct " + iaf_ns;
          s_no_star_direct_iaf = iaf_ns;
        }
        s_approach_final_issued = true;
        logging::info("[approach] no-STAR setup: appr=%s rwy=%s FAF=%s wpts=%d",
                      appr_ns.designator.c_str(), appr_ns.runway.c_str(),
                      s_approach_faf.ident.c_str(),
                      static_cast<int>(s_approach_waypoints.size()));
      }
    }

    // Initial descent altitude: find the STAR/approach waypoint closest to
    // the aircraft. Handles mid-STAR training jumps (pilot may have already
    // passed the STAR entry fix).
    // Fallback priority: (1) waypoints, (2) Centre's cleared altitude
    // (s_enroute_cleared_alt_ft — maintain previous clearance, do not override
    // with approach_entry_alt_ft which may be 8000 ft / FL080 regardless of
    // aircraft altitude), (3) approach_entry_alt_ft.
    const int defaults_ft = flight_phase::get_ifr_defaults().approach_entry_alt_ft;
    int initial_ft = defaults_ft;
    bool initial_ft_from_waypoints = false;
    // Transition altitude: FL assignments compare against pressure_alt_ft;
    // feet (QNH) assignments compare against altitude_ft_msl.
    const int ta_check = (ctx.transition_alt_ft > 0) ? ctx.transition_alt_ft : 5000;
    if (!s_approach_waypoints.empty()) {
      // Build ident->position map from OFP navlog for distance lookups.
      const auto &ofp_pos = simbrief_ofp::get();
      std::unordered_map<std::string, std::pair<double, double>> fix_pos;
      for (const auto &nf : ofp_pos.navlog)
        if (!nf.ident.empty())
          fix_pos[nf.ident] = {nf.lat, nf.lon};

      float best_dist = -1.0f;
      int pos_ft = 0;
      for (const auto &wp : s_approach_waypoints) {
        // Skip floor-only (at-or-above) constraints and no-altitude entries.
        if (wp.alt.feet <= 0 || (wp.is_floor && !wp.is_ceiling))
          continue;
        // Only issue a descent, never a climb.
        {
          const float ref = (wp.alt.feet > ta_check) ? ctx.pressure_alt_ft : ctx.altitude_ft_msl;
          if (wp.alt.feet >= static_cast<int>(ref)) continue;
        }
        auto it = fix_pos.find(wp.ident);
        if (it == fix_pos.end())
          continue;
        auto d = static_cast<float>(traffic_geometry::distance_nm(
            ctx.latitude, ctx.longitude, it->second.first, it->second.second));
        if (best_dist < 0.0f || d < best_dist) {
          best_dist = d;
          pos_ft = wp.alt.feet;
        }
      }
      if (pos_ft > 0) {
        initial_ft = pos_ft;
        initial_ft_from_waypoints = true;
      } else {
        // No OFP navlog coordinates for any constrained fix (typical for no-STAR
        // approach-procedure fixes like QA503 which are not in the navlog).
        // Accept any constrained fix — including floor-only constraints (e.g.
        // "at or above 2700 ft") since they still define the target altitude.
        for (const auto &wp : s_approach_waypoints) {
          const float ref2 = (wp.alt.feet > ta_check) ? ctx.pressure_alt_ft : ctx.altitude_ft_msl;
          if (wp.alt.feet > 0 &&
              wp.alt.feet < static_cast<int>(ref2)) {
            initial_ft = wp.alt.feet;
            initial_ft_from_waypoints = true;
            break;
          }
        }
      }
    }
    // Fallback (2): use Centre's last cleared altitude rather than the generic
    // approach_entry_alt_ft default. This keeps the Approach check-in consistent
    // with what Centre already issued (e.g. "descend 4500 ft" → Approach says
    // "radar contact, descend 4500 ft" not "descend FL080").
    if (!initial_ft_from_waypoints && s_enroute_cleared_alt_ft > 0) {
      initial_ft = s_enroute_cleared_alt_ft;
      logging::info("[approach] check-in: no waypoints, using Centre clearance %d ft",
                    initial_ft);
    }
    // Floor-only constraints (e.g. MUS FL080+) must be respected: never issue
    // an initial clearance below the constraint altitude at a floor-only fix
    // that the aircraft has not yet passed.
    for (const auto &wp : s_approach_waypoints) {
      if (wp.is_floor && !wp.is_ceiling && wp.alt.feet > initial_ft)
        initial_ft = wp.alt.feet;
    }
    s_approach_initial_fl = initial_ft;
    // Also seed s_enroute_cleared_alt_ft with the initial approach FL so
    // engine::current_cleared_alt_ft() (used by STT context_bias) reads
    // the fresh approach clearance instead of the stale cruise FL from
    // the previous en-route phase. Subsequent STAR / approach step-downs
    // update s_enroute_cleared_alt_ft again so the context_bias tracks
    // the current cleared altitude through descent.
    s_enroute_cleared_alt_ft = initial_ft;

    // Safety: if initial_ft >= current altitude the aircraft has already
    // reached or passed the target — issuing "descend X" would be a climb.
    // FL assignments are pressure-referenced; feet (QNH) use altitude_ft_msl.
    const float no_desc_ref = (initial_ft > ta_check) ? ctx.pressure_alt_ft : ctx.altitude_ft_msl;
    // Tolerance band: treat the aircraft as already AT the cleared level when within
    // ~200 ft of it. Without this a few feet of drift flips the phrasing wrongly --
    // LFMN 2026-07-19: pressure_alt 12006 vs FL120 target 12000 (a 6 ft gap) produced
    // "continue descent to flight level 120" while level at FL120.
    constexpr int kLevelTolFt = 200;
    const bool no_descent_needed =
        (initial_ft >= static_cast<int>(no_desc_ref) - kLevelTolFt);

    // Skip waypoints already covered by the initial descent clearance.
    // Rules:
    //   (a) Ceiling, exact, or block constraints at or above initial_ft
    //       are cleared — pilot descends through them.
    //   (b) Floor-only (at-or-above) constraints the aircraft already
    //       meets are trivially satisfied — no instruction needed.
    // The loop continues past (b) so (a) constraints that follow are
    // also reached and skipped (e.g. MN261 "B" block after MUS floor).
    while (s_approach_waypoint_idx <
           static_cast<int>(s_approach_waypoints.size())) {
      const auto &skip_wp = s_approach_waypoints[s_approach_waypoint_idx];
      // (a) not floor-only AND altitude at or above initial clearance
      if (skip_wp.alt.feet > 0 &&
          !(skip_wp.is_floor && !skip_wp.is_ceiling) &&
          skip_wp.alt.feet >= initial_ft) {
        s_approach_waypoint_idx++;
        continue;
      }
      // (b) floor-only constraint already satisfied by current altitude
      const float ref_b = (skip_wp.alt.feet > ta_check) ? ctx.pressure_alt_ft : ctx.altitude_ft_msl;
      if (skip_wp.is_floor && !skip_wp.is_ceiling &&
          skip_wp.alt.feet > 0 &&
          ref_b >= static_cast<float>(skip_wp.alt.feet)) {
        s_approach_waypoint_idx++;
        continue;
      }
      break;
    }

    // Build the check-in response. Approach confirm precedes the descent
    // instruction (ICAO order). QNH appended when clearance is in feet, not FL.
    // "Descend X" = new Approach constraint (waypoints).
    // "Continue descent to X" = maintaining previous Centre clearance.
    // "Continue descent" = aircraft already at/below target, no new altitude.
    // "Continue descent to X" = Centre already cleared this altitude; pilot is descending.
    // "Descend X" = Approach issues a new (lower) target not previously cleared.
    // A jump directly into ARRIVAL/APPROACH had no spoken en-route descent, so
    // "continue descent" is wrong here -- force a fresh "descend" (LFMN jump-to-APP
    // 2026-07-21). One-shot: consumed at this first check-in.
    const bool initial_ft_from_centre =
        s_enroute_cleared_alt_ft > 0 && initial_ft == s_enroute_cleared_alt_ft &&
        !s_jump_no_enroute_descent;
    s_jump_no_enroute_descent = false;
    char buf[240];
    if (no_descent_needed) {
      // Already level at (within tolerance of) the cleared altitude -> MAINTAIN,
      // not "continue descent" (LFMN 2026-07-19: was told to "continue descent"
      // while level at the correct FL). Lower step-downs, if any, come later from
      // poll_approach as the aircraft reaches the constrained fixes.
      char alt_buf_lvl[64];
      if (initial_ft > ta_check)
        std::snprintf(alt_buf_lvl, sizeof(alt_buf_lvl), "maintain flight level %d",
                      initial_ft / 100);
      else
        std::snprintf(alt_buf_lvl, sizeof(alt_buf_lvl), "maintain %d feet, QNH %d",
                      initial_ft, ctx.qnh_hpa);
      std::snprintf(buf, sizeof(buf), "%s, radar contact, identified%s, %s.",
                    cs.c_str(), approach_confirm.c_str(), alt_buf_lvl);
      logging::info("[approach] check-in: initial_ft=%d ~>= alt=%.0f (tol %d), maintain",
                    initial_ft, no_desc_ref, kLevelTolFt);
    } else {
      const int ta_ic = ta_check;
      char alt_buf_ic[64];
      if (initial_ft > ta_ic)
        std::snprintf(alt_buf_ic, sizeof(alt_buf_ic), "flight level %d", initial_ft / 100);
      else
        std::snprintf(alt_buf_ic, sizeof(alt_buf_ic), "%d feet, QNH %d",
                      initial_ft, ctx.qnh_hpa);
      if (initial_ft_from_centre)
        std::snprintf(buf, sizeof(buf),
                      "%s, radar contact, identified%s, continue descent to %s.",
                      cs.c_str(), approach_confirm.c_str(), alt_buf_ic);
      else
        std::snprintf(buf, sizeof(buf), "%s, radar contact, identified%s, descend %s.",
                      cs.c_str(), approach_confirm.c_str(), alt_buf_ic);
    }

    atc_state_machine::set_state(AS::IFR_APPROACH_DESCENT);

    // Arm the descent/maintain read-back for this check-in clearance. This Path-B
    // ack previously armed NOTHING, so when the cleared-approach (runway) arrived
    // moments later the pilot's descent read-back was verified against the runway
    // ("negative, runway zero four"). With the multi-item queue this alt item
    // coexists with the approach runway item -- read them back together in one
    // transmission OR one at a time, each clears independently (LFLP 2026-07-20).
    atc_state_machine::arm_readback(buf);

    // Build route fix list now that STAR + approach waypoints are complete.
    init_route_fixes(ctx);
    if (!s_approach_faf.ident.empty()) {
      for (int i = 0; i < static_cast<int>(s_route_fixes.size()); ++i) {
        if (s_route_fixes[i].ident == s_approach_faf.ident) {
          s_faf_route_idx = i;
          logging::info("[route] FAF %s at route idx=%d", s_approach_faf.ident.c_str(), i);
          break;
        }
      }
    }
    if (s_iaf_route_idx < 0 && !s_no_star_direct_iaf.empty()) {
      for (int i = 0; i < static_cast<int>(s_route_fixes.size()); ++i) {
        if (s_route_fixes[i].ident == s_no_star_direct_iaf) {
          s_iaf_route_idx = i;
          logging::info("[route] IAF %s at route idx=%d",
                        s_no_star_direct_iaf.c_str(), i);
          break;
        }
      }
    }

    Output out;
    out.parsed = parsed;
    out.response_text = buf;
    done(std::move(out));
    return;
  }

  // IFR Tower/AFIS check-in: pilot contacts Tower (or Information/AFIS at
  // non-towered destinations, e.g. LFQA "Reims Prunay Information") after
  // Approach hands off. Any initial-call variant OR unknown/unrecognised
  // transmission in IFR_APPROACH_TOWER on the tower freq fires the handler —
  // state + frequency gates are tight enough to prevent false positives
  // (covers "Reims Information" and "Prunay Information" facility names
  // which don't match any INITIAL_CALL_* rule, plus Voxtral garbles like
  // "Pass information").
  const bool is_tower_call_any =
      parsed.intent == PI::INITIAL_CALL_TOWER   ||
      parsed.intent == PI::INITIAL_CALL         ||
      parsed.intent == PI::INITIAL_CALL_APPROACH ||
      parsed.intent == PI::INITIAL_CALL_INBOUND ||
      // Visual-final (MDA) approaches: Tower said "report runway in sight",
      // so the pilot's "runway in sight" reply -- scored TRAFFIC_IN_SIGHT by the
      // "in sight" keyword -- IS the expected Tower report here, not a traffic
      // ack (LFMN R04LA 2026-07-12: it fell through to "say again"). Tight gate
      // (IFR_APPROACH_TOWER on the Tower freq) makes this unambiguous.
      parsed.intent == PI::TRAFFIC_IN_SIGHT     ||
      parsed.intent == PI::UNKNOWN;
  if (is_tower_call_any &&
      atc_state_machine::get_state() ==
          atc_state_machine::ATCState::IFR_APPROACH_TOWER) {
    // Frequency guard: readbacks on the Approach freq mis-classified as
    // INITIAL_CALL must not trigger the landing clearance prematurely.
    const float tower_freq_mhz =
        xplane_context::tower_mhz_for(s_assigned_dest_icao);
    const float active_com_mhz =
        (ctx.active_com == 1) ? ctx.com1_freq_mhz : ctx.com2_freq_mhz;
    const bool on_tower_freq =
        tower_freq_mhz < 100.0f || // unknown (no OFP / training) — allow any
        std::fabs(active_com_mhz - tower_freq_mhz) < 0.010f;
    if (on_tower_freq) {
      using ASt = atc_state_machine::ATCState;
      const std::string &cs_at_ref = atc_state_machine::session_callsign();
      const std::string cs_at =
          cs_at_ref.empty() ? in.pilot_callsign : cs_at_ref;
      // Use the CIFP-derived approach runway (set at Approach check-in).
      // ctx.active_runway can belong to a different nearby airport when the
      // aircraft is still on approach, causing "runway JCA"-style mistakes.
      const std::string rwy_at =
          !s_assigned_landing_runway.empty() ? s_assigned_landing_runway :
          !ctx.active_runway.empty()         ? ctx.active_runway :
                                               ctx.nearest_airport_id;
      int wind_dir = static_cast<int>(std::round(ctx.wind_direction_deg));
      int wind_kt  = static_cast<int>(std::round(ctx.wind_speed_kt));
      char buf_at[160];
      std::snprintf(buf_at, sizeof(buf_at),
                    "%s, runway %s, cleared to land, wind %03d degrees %02d knots.",
                    cs_at.c_str(), rwy_at.c_str(), wind_dir, wind_kt);
      // Preserve the label set by the sector-exit handoff (e.g. "Reims
      // Prunay Information" for AFIS destinations). Only fall back to a
      // generic "Tower" label if nothing was set upstream.
      if (s_current_controller_label.empty())
        s_current_controller_label =
            s_assigned_dest_icao.empty() ? "Tower" : (s_assigned_dest_icao + " Tower");
      atc_state_machine::set_state(ASt::IFR_LANDING_CLEARED);
      Output out_at;
      out_at.parsed = parsed;
      out_at.response_text = buf_at;
      done(std::move(out_at));
      return;
    }
    // Pilot still on Approach freq — suppress; they need to switch to Tower.
    // Without this return the VFR machine fires "you are already airborne".
    Output out_ap;
    out_ap.parsed = parsed;
    out_ap.response_text = "";  // silent ack — pilot is reading back the handoff
    done(std::move(out_ap));
    return;
  }

  // IFR flight-plan closure at destination.
  // Trigger: state IFR/LANDING_CLEARED (i.e. aircraft has landed IFR and
  // cleared the runway) + pilot's transmission signals arrival at
  // parking / leaving the frequency. Distinguish AFIS vs towered:
  //   - AFIS (no Ground freq at destination) → AFIS cannot close IFR;
  //     the pilot must call the ARO by telephone. Reply "leaving frequency
  //     approved, contact <airport> by telephone to close IFR flight plan."
  //   - Towered → controller closes the flight plan automatically on
  //     landing. Reply "IFR flight plan closed at HH:MM, good day."
  // Fires for LEAVING_FREQUENCY, REPORT_POSITION, REQUEST_TAXI_PARKING (a
  // misclassification post-landing at parking), or UNKNOWN utterances that
  // contain a "parking / at stand / leaving" keyword.
  // State gate covers two arrival situations:
  //   (1) IFR_LANDING_CLEARED — pilot reports parking/leaving before the
  //       RUNWAY_VACATED call has dropped the state.
  //   (2) post-arrival IDLE — RUNWAY_VACATED transitions IFR_LANDING_CLEARED
  //       -> IDLE immediately (+ "contact ground"), so by the time the pilot
  //       taxis in and reports "at the stand" the state is IDLE. Without this
  //       branch the closure never fired and the at-stand call fell to
  //       INITIAL_CALL_GROUND -> a bogus pre-departure greeting (LIMF -> LFLP
  //       2026-07-10: "Annecy Ground, information current, runway 22, QNH").
  //       Gated on was_airborne() so a genuine cold-start departure is never
  //       mistaken for an arrival.
  const auto cl_state = atc_state_machine::get_state();
  const bool in_landing_cleared =
      cl_state == atc_state_machine::ATCState::IFR_LANDING_CLEARED;
  const bool post_arrival_idle =
      atc_state_machine::was_airborne() &&
      cl_state == atc_state_machine::ATCState::IDLE;
  if ((in_landing_cleared || post_arrival_idle) && ctx.on_ground) {
    const std::string &tx = in.transcript;
    auto tx_contains = [&](const char *needle) {
      if (!needle) return false;
      const std::string n = needle;
      auto lower = tx;
      for (auto &c : lower)
        c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
      return lower.find(n) != std::string::npos;
    };
    // A closure is an ARRIVAL-AT-STAND / leaving-to-close report — never a
    // service or taxi REQUEST. Any transmission containing "request" (taxi
    // to parking, request Doxy, ...) wants something, so it can't close the
    // flight plan. This guard prevents false closures on departure/taxi
    // calls that merely mention "parking" ("at parking request taxi",
    // "request taxi to general aviation parking", garbled "at the parking
    // position request ...").
    const bool is_request = tx_contains("request");
    // Intent-based trigger only applies in IFR_LANDING_CLEARED (and only for
    // non-request transmissions via the is_request guard below).
    const bool closure_intent =
        in_landing_cleared &&
        (parsed.intent == PI::LEAVING_FREQUENCY ||
         parsed.intent == PI::REPORT_POSITION   ||
         parsed.intent == PI::REQUEST_TAXI_PARKING);
    // Keyword trigger requires an unambiguous at-STAND report or an explicit
    // leaving/close phrase. Bare "at parking" / "parking" is deliberately
    // NOT here — it matches taxi-to-parking and "at the parking position"
    // garble. The real arrival call is "at the (parking) stand".
    // A taxi-clearance READBACK echoes "report on stand" / "taxi to <parking>" --
    // that is the instruction coming back, NOT an at-stand arrival report, so it
    // must not fire the IFR closure while the aircraft is still taxiing in (LFMN
    // 2026-07-20: closure fired on the taxi readback, before the stand). The real
    // arrival call is a bare "on/at the stand" (no "report", no "taxi to").
    const bool taxi_readback_echo =
        tx_contains("report on stand") || tx_contains("taxi to");
    const bool at_stand_report =
        !taxi_readback_echo &&
        (tx_contains("at the stand") || tx_contains("at stand") ||
         tx_contains("on stand") || tx_contains("on the stand") ||
         tx_contains("parking stand"));
    const bool closure_keyword =
        at_stand_report ||
        tx_contains("leaving frequency") || tx_contains("leave the frequency") ||
        tx_contains("leave frequency") || tx_contains("shut down") ||
        tx_contains("close ifr") || tx_contains("close flight plan");
    if (!is_request && (closure_intent || closure_keyword)) {
      const std::string &cs_ifr_ref = atc_state_machine::session_callsign();
      const std::string cs_ifr =
          cs_ifr_ref.empty() ? in.pilot_callsign : cs_ifr_ref;
      // Destination classification: no Ground freq → AFIS/Information.
      const std::string dest =
          !s_assigned_dest_icao.empty() ? s_assigned_dest_icao
                                        : ctx.nearest_airport_id;
      const bool is_afis =
          !xplane_context::has_ground_freq_for(dest);
      char buf_cl[256];
      if (is_afis) {
        // Prefer airport name over ICAO in the spoken phrase.
        std::string apt =
            !dest.empty() ? xplane_context::airport_name_for(dest) : "";
        if (apt.empty()) apt = dest.empty() ? "the ARO" : dest;
        std::snprintf(buf_cl, sizeof(buf_cl),
                      "%s, leaving frequency approved, contact %s "
                      "by telephone to close IFR flight plan, good day.",
                      cs_ifr.c_str(), apt.c_str());
      } else {
        // Towered: controller closes the FPL automatically on landing.
        // Use current sim UTC time for the closure stamp.
        std::time_t now_t = std::time(nullptr);
        std::tm *utc = std::gmtime(&now_t);
        char hhmm[8] = "";
        if (utc)
          std::snprintf(hhmm, sizeof(hhmm), "%02d%02d",
                        utc->tm_hour, utc->tm_min);
        if (hhmm[0])
          std::snprintf(buf_cl, sizeof(buf_cl),
                        "%s, IFR flight plan closed at %s, good day.",
                        cs_ifr.c_str(), hhmm);
        else
          std::snprintf(buf_cl, sizeof(buf_cl),
                        "%s, IFR flight plan closed, good day.",
                        cs_ifr.c_str());
      }
      atc_state_machine::set_state(atc_state_machine::ATCState::IDLE);
      // Mark the arrival complete so a repeated "at the stand" call can't
      // re-fire the closure (the post_arrival_idle gate keys on was_airborne).
      atc_state_machine::set_was_airborne(false);
      logging::info("IFR closure at %s (%s): %s", dest.c_str(),
                    is_afis ? "AFIS" : "towered", buf_cl);
      Output out_cl;
      out_cl.parsed = parsed;
      out_cl.response_text = buf_cl;
      done(std::move(out_cl));
      return;
    }
  }

  // ── LM-not-ready fast path ────────────────────────────────────────
  // Headless tools, scenario tests, and the brief window between
  // plugin start and "models verified" all hit this path. The
  // rule-based parser is authoritative here — same behaviour as
  // before always-on classification was introduced.
  if (!backends::lm_ready()) {
    if (parsed.intent == PI::UNKNOWN) {
      Output out;
      out.parsed = parsed;
      out.response_text = build_unclear_response(parsed, in.pilot_callsign);
      logging::info("ATC (LM unavailable, UNKNOWN): %s",
                    out.response_text.c_str());
      done(std::move(out));
      return;
    }
    done(run_state_machine(parsed, ctx, in.now_secs));
    return;
  }

  // ── Readback-pending guard ────────────────────────────────────────
  // When ATC is waiting for a readback, the pilot is on the same
  // frequency and reading back the last clearance.  If the clearance
  // text contained the word "approach" (runway, STAR, etc.) the rule
  // parser can misclassify the readback as INITIAL_CALL_APPROACH.
  // Override any INITIAL_CALL_* back to READBACK so the readback
  // verifier fires correctly instead of triggering a bogus new check-in.
  //
  // Exception: if the pending readback is a frequency-change clearance and
  // the pilot is already transmitting on that frequency, they have implicitly
  // acknowledged the handoff by switching.  Auto-clear the readback and let
  // the initial contact proceed normally (EUROCONTROL: tuning the new freq
  // is the pilot's operational confirmation of the transfer instruction).
  if (atc_state_machine::is_readback_pending()) {
    // A readback is pending, so the pilot's transmission is almost always
    // the readback itself.  Escape intents are the exceptions where the
    // pilot is deliberately NOT reading back (refuse / ask-repeat /
    // go-around / handoff sign-off / new request) — let those through to
    // normal processing.  Everything else is forced to READBACK so it
    // reaches the field-matching verifier instead of fresh classification.
    //
    // Previously this override was gated on a whitelist of INITIAL_CALL_*
    // / READBACK intents only.  A long IFR descent-clearance readback
    // ("descend FL170, cleared via SALE3P arrival, expect RNAV Zulu
    // approach runway 04") was mis-scored RUNWAY_VACATED 0.30 by the rule
    // parser (it contains "runway 04" + "clear"), fell outside the
    // whitelist, then the LM returned _INVALID -> "say again, use standard
    // phraseology" x3 (LIMF -> LFLP 2026-07-09).  Broadening the override
    // to "anything that isn't an escape intent" routes it to the verifier.
    const bool escape_intent =
        parsed.intent == PI::UNABLE           ||
        parsed.intent == PI::REQUEST_REPEAT   ||
        parsed.intent == PI::GO_AROUND        ||
        parsed.intent == PI::LEAVING_FREQUENCY ||
        parsed.intent == PI::REQUEST_DESCENT;
    if (!escape_intent) {
      // Extract expected frequency from the pending clearance text.
      bool auto_cleared = false;
      {
        static const std::regex kFreqRe(R"(\b(\d{3})\.(\d{3})\b)");
        const std::string &cl = atc_state_machine::last_clearance_text();
        std::smatch fm;
        if (std::regex_search(cl, fm, kFreqRe)) {
          float cl_freq = std::stof(fm[1].str() + "." + fm[2].str());
          float active  = ctx.active_com == 2 ? ctx.com2_freq_mhz
                                               : ctx.com1_freq_mhz;
          if (std::fabs(active - cl_freq) < 0.005f) {
            logging::info(
                "Readback auto-cleared: pilot on expected freq %.3f (handoff)",
                cl_freq);
            atc_state_machine::cancel_readback();
            auto_cleared = true;
          }
        }
      }
      // Also auto-clear when a SECTOR HANDOFF is in progress and the pilot has
      // switched to the new sector's frequency, even if the pending clearance has
      // NO freq in its text. A readback owed to the PREVIOUS controller (e.g. Geneva
      // issued "descend 6500" just before handing off) is moot after the switch --
      // if left pending it forces the new-sector check-in to READBACK and loops
      // "negative 6500 readback", blocking the entire arrival (LFLP 2026-07-17). The
      // cleared level persists in state; the new controller re-issues if needed.
      if (!auto_cleared && s_sector_checkin_pending &&
          s_pending_handoff_freq_mhz > 100.0f) {
        const float active = ctx.active_com == 2 ? ctx.com2_freq_mhz
                                                 : ctx.com1_freq_mhz;
        if (std::fabs(active - s_pending_handoff_freq_mhz) < 0.010f) {
          logging::info("Readback auto-cleared: sector handoff, pilot on new "
                        "freq %.3f (stale readback dropped)",
                        s_pending_handoff_freq_mhz);
          atc_state_machine::cancel_readback();
          auto_cleared = true;
        }
      }
      // A fresh sector CHECK-IN (INITIAL_CALL_*) is never a readback -- the pilot
      // has just arrived on a new controller's frequency. Any readback still
      // pending is stale relative to this check-in (e.g. the approach clearance
      // fired on the new freq BEFORE the pilot checked in, arming a runway
      // readback). Overriding the check-in to READBACK rejected it as "negative,
      // runway zero four, readback" (LFLP Chambery 2026-07-19). Drop the stale
      // readback and let the check-in be handled as a check-in; the controller
      // re-issues anything outstanding, and the cleared values persist in state.
      if (!auto_cleared &&
          (parsed.intent == PI::INITIAL_CALL_APPROACH ||
           parsed.intent == PI::INITIAL_CALL ||
           parsed.intent == PI::INITIAL_CALL_GROUND ||
           parsed.intent == PI::INITIAL_CALL_TOWER ||
           parsed.intent == PI::INITIAL_CALL_INBOUND ||
           parsed.intent == PI::INITIAL_CALL_INBOUND_VRP)) {
        logging::info("Readback auto-cleared: fresh check-in (%s), stale readback "
                      "not applied",
                      intent_parser::intent_name(parsed.intent));
        atc_state_machine::cancel_readback();
        auto_cleared = true;
      }
      if (!auto_cleared) {
        logging::info("Readback pending: overriding %s -> READBACK",
                      intent_parser::intent_name(parsed.intent));
        parsed.intent     = PI::READBACK;
        parsed.confidence = 0.80f;
      }
    }
  }

  // ── LM as fallback only ───────────────────────────────────────────
  // The rule-based parser (data-driven matchers in intent_rules.json
  // + state-history-aware adjustments such as just_landed) is
  // authoritative. The local LM only fires when the rule parser is
  // genuinely unsure (UNKNOWN or confidence < 0.7).
  //
  // Field measurement on Apple Silicon: even with Metal flash-
  // attention and QOS_UTILITY workers, every Llama 3.2 3B classify
  // call costs visible FPS in X-Plane. At conf >= 0.7 the rule
  // parser was empirically right in nearly every observed case
  // (see LSZG circuit log 2026-05-04: REQUEST_TAXI / READBACK /
  // RUNWAY_VACATED / REPORT_POSITION_* all classified correctly at
  // 0.90, while the LM frequently disagreed wrongly or returned
  // _INVALID and was overridden by safety nets).
  if (parsed.confidence >= 0.7f && parsed.intent != PI::UNKNOWN) {
    if (settings::debug_logging())
      logging::debug("Rule-based path: %s (conf=%.2f) — skip LM",
                     intent_parser::intent_name(parsed.intent),
                     parsed.confidence);
    done(run_state_machine(parsed, ctx, in.now_secs));
    return;
  }

  // ── Always-on LM classification with constrained JSON output ──────
  // The LM gets the rule-based parser's intent as a low-priority
  // hint, the valid_intents enum for the current state (grammar-
  // enforced — model literally cannot return anything else), and the
  // flight context. It returns {intent, repaired_transcript,
  // whisper_fix}. Whisper-artifact repair is the LM's job; pilot
  // phraseology errors fall through to the state machine which still
  // reacts realistically (frequency guards, phase guards, _INVALID
  // templates).
  using FT = xplane_context::FrequencyType;
  bool is_towered = ctx.is_towered_airport &&
                    ctx.frequency_type != FT::UNICOM &&
                    ctx.frequency_type != FT::CTAF;

  std::string state_str =
      atc_state_machine::state_name(atc_state_machine::get_state());

  // IFR states live exclusively in the "towered" template section —
  // valid_intents must look there even when the nearest airport is uncontrolled
  // (e.g. en-route over rural airspace far from any towered field).
  if (state_str.rfind("IFR/", 0) == 0)
    is_towered = true;

  std::string previous_state_str =
      atc_state_machine::state_name(atc_state_machine::previous_state());
  std::string state_history_csv = atc_state_machine::history_csv();
  bool just_landed_flag = atc_state_machine::just_landed(in.now_secs);
  auto valid = atc_templates::valid_intents(is_towered, state_str);

  // Always include the traffic-acknowledgement intents — they are
  // valid any time the controller has just issued a traffic advisory,
  // regardless of which ATC state we're in.
  for (const char *t :
       {"TRAFFIC_IN_SIGHT", "TRAFFIC_NEGATIVE_CONTACT", "TRAFFIC_LOOKING"}) {
    if (std::find(valid.begin(), valid.end(), t) == valid.end())
      valid.emplace_back(t);
  }

  std::string valid_list;
  for (const auto &v : valid) {
    if (!valid_list.empty())
      valid_list += ", ";
    valid_list += v;
  }

  std::string sys_prompt = atc_templates::get_prompt("gpt_classify_prompt");
  if (sys_prompt.empty()) {
    sys_prompt = "You are an ATC intent classifier. State: {state}. "
                 "Valid intents: {valid_intents}. Hint: {hint_intent}. "
                 "Transcript: \"{transcript}\". Respond with strict JSON "
                 "{\"intent\":\"...\",\"repaired\":\"...\",\"whisper_fix\":"
                 "false}.";
  }
  sys_prompt = atc_templates::fill(
      sys_prompt,
      {{"state", state_str},
       {"previous_state", previous_state_str},
       {"state_history_csv", state_history_csv},
       {"just_landed", just_landed_flag ? "true" : "false"},
       {"valid_intents", valid_list},
       {"transcript", in.transcript},
       {"frequency_type",
        xplane_context::frequency_type_name(ctx.frequency_type)},
       {"on_ground", ctx.on_ground ? "true" : "false"},
       {"altitude_ft", std::to_string(static_cast<int>(ctx.altitude_ft_msl))},
       {"groundspeed_kts",
        std::to_string(static_cast<int>(ctx.groundspeed_kts))},
       {"airport", ctx.nearest_airport_id},
       {"hint_intent", intent_parser::intent_name(parsed.intent)}});

  if (settings::debug_logging())
    logging::debug("Routing to local LM classify_with_repair (rule hint=%s "
                   "conf=%.2f)",
                   intent_parser::intent_name(parsed.intent),
                   parsed.confidence);

  // Snapshot ctx + transcript so the async callback sees the state at
  // the moment the pilot spoke, not whatever ctx contains when the LM
  // responds.
  xplane_context::XPlaneContext ctx_snapshot = ctx;
  double now_secs = in.now_secs;
  std::string fallback_cs = in.pilot_callsign;
  std::string original_transcript = in.transcript;
  // Apply the same JSON `normalize` table (eu/us intent_rules.json) to the
  // LM's input as the rule parser already got. Without this, Voxtral
  // mishearings that the normalize table fixes for rule-based
  // classification still trip up the LM. Concrete case that motivated
  // this: "VFR squawk 3076" (Voxtral for "verify squawk 3076"). The rule
  // parser sees "verify squawk" after normalize and skips READY_FOR_*
  // rules; the LM used to see raw "VFR squawk" and classified the taxi
  // readback as READY_FOR_DEPARTURE_VFR — plugin then rejected on Ground
  // freq with "contact Tower when ready for departure", making it look
  // like a spurious handoff. See project_voxtral_stt_errors.md.
  std::string lm_input = in.transcript;
  std::transform(lm_input.begin(), lm_input.end(), lm_input.begin(),
                 [](unsigned char c) { return std::tolower(c); });
  lm_input = intent_rules::preprocess(lm_input);
  // Snapshot the just-landed flag too — the async callback may fire
  // after the state machine has moved on, but the post-landing
  // plausibility decision must reflect the moment the pilot spoke.
  bool just_landed_snapshot = just_landed_flag;
  ++lm_inferences_;
  backends::lm::classify_with_repair_async(
      lm_input, sys_prompt, valid,
      // NOLINTNEXTLINE(bugprone-exception-escape)
      [parsed, ctx_snapshot, now_secs, fallback_cs, original_transcript,
       just_landed_snapshot, done = std::move(done)](
          const backends::lm::ClassifyResult &result) mutable {
        std::string intent_key =
            result.success ? result.intent_name : std::string("_INVALID");

        if (settings::debug_logging()) {
          logging::debug(
              "LM classified: intent=%s whisper_fix=%d repaired=\"%s\"",
              intent_key.c_str(), result.whisper_fix ? 1 : 0,
              result.repaired_transcript.c_str());
        }

        // Telemetry: log when LM and rule-based parser disagree.
        // Helps decide whether the 3B model is good enough or we need
        // a bigger one.
        auto rule_intent = parsed.intent;
        auto lm_intent = intent_parser::intent_from_key(intent_key);
        if (rule_intent != intent_parser::PilotIntent::UNKNOWN &&
            lm_intent != rule_intent && intent_key != "_INVALID") {
          logging::info("LM/rule disagree: rule=%s (conf=%.2f) llm=%s",
                        intent_parser::intent_name(rule_intent),
                        parsed.confidence, intent_key.c_str());
        }

        // Readback safety net: trust rule=READBACK whenever the rule
        // parser is confident (>=0.90), regardless of whether
        // readback_pending is currently armed. Two cases this catches:
        //   1) Mid-clearance readbacks where readback_pending=true.
        //      LM occasionally hallucinates TRAFFIC_IN_SIGHT or
        //      READY_FOR_DEPARTURE for a taxi readback whose Whisper
        //      transcription was garbled.
        //   2) Closing readbacks AFTER state→IDLE has already cleared
        //      readback_pending (e.g. post-landing "general aviation
        //      parking via Alpha, good day"). Without this widened
        //      check, LM=REQUEST_TAXI wins and triggers a brand-new
        //      departure cycle (TAXI_CLEARED → TOWER_CONTACT auto-
        //      advance), turning the parking-arrival readback into a
        //      bogus takeoff briefing.
        // The rule parser's READBACK matchers are keyword-anchored
        // (wilco/roger/good day/holding point/cleared+takeoff/qnh/
        // hold short/runway-suffix endings), so false positives are
        // rare. Letting the LM override these consistently produces
        // wrong ATC chatter at moments ICAO requires silence.
        if (rule_intent == intent_parser::PilotIntent::READBACK &&
            parsed.confidence >= 0.90f &&
            lm_intent != intent_parser::PilotIntent::READBACK) {
          logging::info("Readback safety net: keeping rule=READBACK over "
                        "LM=%s (rule_conf=%.2f, readback_pending=%s)",
                        intent_key.c_str(), parsed.confidence,
                        atc_state_machine::is_readback_pending() ? "true"
                                                                 : "false");
          intent_key = "READBACK";
          lm_intent = intent_parser::PilotIntent::READBACK;
        }

        // Validate the repair before letting it influence anything
        // downstream. If the LM invented digits that weren't in the
        // original (a runway number, a frequency, an altitude), drop
        // the repair and keep the raw Whisper text. Logged at info so
        // the rejection is visible without debug-mode.
        bool repair_accepted =
            result.whisper_fix && !result.repaired_transcript.empty();
        if (repair_accepted &&
            repair_invents_digits(original_transcript,
                                  result.repaired_transcript)) {
          logging::info("STT repair rejected (invented digits): "
                        "\"%s\" -> \"%s\"",
                        original_transcript.c_str(),
                        result.repaired_transcript.c_str());
          repair_accepted = false;
        } else if (repair_accepted &&
                   repair_violates_history(result.repaired_transcript,
                                           just_landed_snapshot)) {
          logging::info("STT repair rejected (post-landing context): "
                        "\"%s\" -> \"%s\"",
                        original_transcript.c_str(),
                        result.repaired_transcript.c_str());
          repair_accepted = false;
        } else if (repair_accepted) {
          logging::info("STT repair: \"%s\" -> \"%s\"",
                        original_transcript.c_str(),
                        result.repaired_transcript.c_str());
        }

        // _INVALID: controller asks for say-again. Tier picks itself
        // based on whether anything in the transcript was recognisable.
        if (intent_key == "_INVALID") {
          Output out;
          out.parsed = parsed;
          out.response_text = build_unclear_response(parsed, fallback_cs);
          logging::info("ATC (LM _INVALID): %s", out.response_text.c_str());
          done(std::move(out));
          return;
        }

        // Build a PilotMessage with the LM-classified intent. Keep
        // the rule-based callsign / runway / VRP extraction — those
        // are deterministic and don't benefit from LM interpretation.
        auto lm_msg = parsed;
        lm_msg.intent = lm_intent;
        lm_msg.confidence = 0.85f;
        if (repair_accepted) {
          // Replace the raw transcript with the repaired one so the
          // UI history shows what the controller acted on. The
          // confidence stays at 0.85 — repair doesn't make us more
          // certain about intent classification.
          lm_msg.raw_transcript = result.repaired_transcript;
        }

        // Traffic dialog short-circuit. The rule parser frequently
        // misses softer phrasings ("looking", "have the traffic") and
        // only the LM lands them on TRAFFIC_*.
        Output out;
        if (try_traffic_dialog(lm_msg, ctx_snapshot, now_secs, out)) {
          done(std::move(out));
          return;
        }

        // IFR early-approach re-entry: INITIAL_CALL_APPROACH in IFR en-route
        // or approach states requires the IFR-specific handlers in
        // process_transcript (early-approach-call promotion, APPROACH_CONTACT
        // check-in with descent clearance). Those handlers check parsed.intent
        // BEFORE the LM path runs, so they never fire via this callback.
        // Re-invoke process_transcript with the LM-classified intent injected
        // so the full IFR handler chain runs on this second pass.
        // The second pass sees pre_classified_conf=0.85 ≥ 0.7 → takes the
        // rule-based path immediately (no further LM call → no recursion).
        {
          using PI = intent_parser::PilotIntent;
          using AS = atc_state_machine::ATCState;
          const auto cur_st = atc_state_machine::get_state();
          if (lm_msg.intent == PI::INITIAL_CALL_APPROACH &&
              (cur_st == AS::IFR_ENROUTE_CRUISE ||
               cur_st == AS::IFR_DESCENT        ||
               cur_st == AS::IFR_ARRIVAL        ||
               cur_st == AS::IFR_APPROACH_CONTACT)) {
            Input reinput;
            reinput.transcript =
                repair_accepted ? result.repaired_transcript : original_transcript;
            reinput.quality    = 1.0f;
            reinput.ctx        = &ctx_snapshot;
            reinput.pilot_callsign = fallback_cs;
            reinput.now_secs   = now_secs;
            reinput.pre_classified_intent = PI::INITIAL_CALL_APPROACH;
            reinput.pre_classified_conf   = 0.85f;
            process_transcript(std::move(reinput), std::move(done));
            return;
          }
        }

        auto atc_resp =
            atc_state_machine::process(lm_msg, ctx_snapshot, now_secs);

        if (settings::debug_logging())
          logging::debug("ATC response text: %s", atc_resp.text.empty()
                                                      ? "(silent)"
                                                      : atc_resp.text.c_str());

        // LM produced a concrete intent — pilot was understood, even
        // if the state machine subsequently rejected the request.
        if (lm_msg.intent != intent_parser::PilotIntent::UNKNOWN)
          mark_clear();
        out.parsed = lm_msg;
        out.response_text = atc_resp.text;
        done(std::move(out));
      });
}

namespace {

// Resolve the active landing runway from a XPlaneContext snapshot.
// Mirrors pattern_flow::resolve_active_runway but kept local so the
// frame-driven go-around path does not have to link pattern_flow's
// internal anonymous namespace.
std::optional<landing_sequence::ActiveRunway>
resolve_active_runway_for_go_around(const xplane_context::XPlaneContext &ctx) {
  if (ctx.active_runway.empty() || ctx.runways.empty())
    return std::nullopt;
  for (const auto &rw : ctx.runways) {
    const xplane_context::RunwayEnd *end = nullptr;
    double heading = 0.0;
    if (rw.end1.number == ctx.active_runway) {
      end = &rw.end1;
      heading = static_cast<double>(rw.end1.heading_deg);
    } else if (rw.end2.number == ctx.active_runway) {
      end = &rw.end2;
      heading = static_cast<double>(rw.end2.heading_deg);
    }
    if (!end)
      continue;
    landing_sequence::ActiveRunway out;
    out.threshold_lat = end->lat;
    out.threshold_lon = end->lon;
    out.heading_deg = heading;
    out.length_m = static_cast<double>(rw.length_m);
    if (out.length_m < 500.0)
      out.length_m = 2500.0;
    out.designator = end->number;
    return out;
  }
  return std::nullopt;
}

} // namespace

bool poll_go_around(const xplane_context::XPlaneContext &ctx, double now_secs,
                    std::string *out_text) {
  // Gate 1: user is on a granted landing clearance.
  if (atc_state_machine::get_state() !=
      atc_state_machine::ATCState::LANDING_CLEARED)
    return false;

  // Gate 2: cooldown — never fire two go-arounds inside 60 s.
  if (now_secs - last_go_around_emit_secs_ < kGoAroundCooldownSec)
    return false;

  // Gate 3: active runway must resolve to a concrete threshold.
  auto rwy_opt = resolve_active_runway_for_go_around(ctx);
  if (!rwy_opt.has_value())
    return false;

  // Gate 4: user within 1 NM of the threshold.
  const double user_dist_nm = traffic_geometry::distance_nm(
      rwy_opt->threshold_lat, rwy_opt->threshold_lon, ctx.latitude,
      ctx.longitude);
  if (user_dist_nm > kGoAroundTriggerDistanceNm)
    return false;

  // Gate 5: runway-occupied scan via the same sequencing primitive
  // pattern_flow uses for the "continue approach" overlay. We can't
  // cheap-out to a single-target scan — the occupant may be the
  // second-nearest target rather than the first.
  const auto &traffic = traffic_context::current();
  landing_sequence::UserPosition user{ctx.latitude, ctx.longitude};
  auto seq =
      landing_sequence::compute_landing_sequence(traffic, user, *rwy_opt);
  if (!seq.runway_occupied)
    return false;

  // All gates passed — render the unsolicited go-around call. No state
  // change, no traffic_dialog ack hook: this is a controller flight
  // command, the pilot's reaction is to fly, not to speak.
  std::string text = atc_state_machine::render_traffic_advisory(
      {}, ctx, "go_around_traffic_runway");
  last_go_around_emit_secs_ = now_secs;
  if (out_text)
    *out_text = std::move(text);
  logging::info("Engine emitted go-around (user dist=%.2f NM, occupant id=%u)",
                user_dist_nm,
                seq.occupant.has_value() ? seq.occupant->modeS_id : 0u);
  return true;
}

bool poll_readback_reminder(const xplane_context::XPlaneContext &ctx,
                            double now_secs, std::string *out_text) {
  std::string template_key =
      atc_state_machine::consume_readback_reminder(now_secs);
  if (template_key.empty())
    return false;
  // render_traffic_advisory pulls callsign + airport from the live
  // context — identical pipeline used by traffic advisories, go-around
  // call etc. The {callsign} placeholder is filled with session_callsign
  // when set (so reminders address the same callsign Tower used in the
  // clearance).
  std::string text =
      atc_state_machine::render_traffic_advisory({}, ctx, template_key);
  if (out_text)
    *out_text = std::move(text);
  return true;
}

bool poll_traffic_advisory(const xplane_context::XPlaneContext &ctx,
                           double now_secs, std::string *out_text) {
  using FT = xplane_context::FrequencyType;

  // Don't fire fresh advisories while the previous one hasn't been
  // acknowledged yet — the dialog is the gate, not the main ATCState.
  if (traffic_dialog::is_awaiting_ack())
    return false;

  traffic_advisor::UserState user;
  user.atc_state = atc_state_machine::get_state();
  user.on_active_atc_freq = ctx.frequency_type == FT::TOWER ||
                            ctx.frequency_type == FT::GROUND ||
                            ctx.frequency_type == FT::APPROACH;
  user.lat = ctx.latitude;
  user.lon = ctx.longitude;
  user.alt_msl_ft = static_cast<double>(ctx.altitude_ft_msl);
  user.heading_deg = static_cast<double>(ctx.heading_true);
  // Ground track == heading_true is a small simplification (no wind
  // crab) but matches the precision the advisory geometry needs (clock
  // positions are rounded to the hour).
  user.track_deg = static_cast<double>(ctx.heading_true);
  user.groundspeed_kts = static_cast<double>(ctx.groundspeed_kts);
  user.on_ground = ctx.on_ground;
  user.target_has_mode_c_default = true;
  user.user_taxiing = flight_phase::get() == flight_phase::FlightPhase::TAXI;

  const auto &traffic = traffic_context::current();

  auto adv =
      traffic_advisor::evaluate(traffic, user, advisory_history_, now_secs);
  if (!adv.has_value())
    return false;

  std::string text = atc_state_machine::render_traffic_advisory(
      adv->vars, ctx, adv->template_key);
  traffic_advisor::mark_emitted(advisory_history_, adv->modeS_id, now_secs);
  // Ground-conflict advisories don't expect a voice ack — the pilot
  // reacts by stopping / giving way. Skip the dialog side-channel so
  // the next pilot transcript still flows through the normal ATC
  // pipeline.
  if (adv->requires_ack)
    traffic_dialog::on_advisory_emitted(adv->modeS_id);

  if (out_text)
    *out_text = std::move(text);

  logging::info("Engine emitted traffic advisory (target_id=%u, template=%s)",
                adv->modeS_id, adv->template_key.c_str());
  return true;
}

// Strip known controller-role suffixes from a raw apt.dat name and title-case
// the remainder. "CHAMBERY APP" -> "Chambery", "ZURICH DEP" -> "Zurich".
static std::string controller_location(const std::string &raw) {
  static const char *kSuffixes[] = {
      // Long-form first so they match before their 3-letter abbreviations.
      " APPROACH", " DEPARTURE", " ARRIVAL",  " CONTROL",
      " CENTRE",   " CENTER",    " DIRECTOR",
      // 3-letter ATC.dat abbreviations.
      " APP", " DEP", " CTR", " GND", " TWR", " DLV", " DEL", " FSS",
      nullptr};
  std::string loc = raw;
  // Case-insensitive suffix match: atc.dat names are uppercase
  // ("GENEVA APPROACH") but apt.dat frequency names are mixed-case
  // ("Geneva Approach").  A case-sensitive compare against the uppercase
  // suffix list failed to strip the mixed-case form, leaving the caller
  // to append a second " Approach" -> "Geneva Approach Approach"
  // (LIMF -> LFLP 2026-07-09). Compare on an uppercased copy, trim the
  // original by the matched length.
  std::string upper = loc;
  for (char &c : upper)
    c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
  for (int i = 0; kSuffixes[i]; ++i) {
    std::string suf(kSuffixes[i]);
    if (upper.size() >= suf.size() &&
        upper.compare(upper.size() - suf.size(), suf.size(), suf) == 0) {
      loc = loc.substr(0, loc.size() - suf.size());
      break;
    }
  }
  if (loc.empty())
    return loc;
  bool cap = true;
  for (char &c : loc) {
    if (c == ' ') {
      cap = true;
    } else if (cap) {
      c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
      cap = false;
    } else {
      c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    }
  }
  return loc;
}

// Return a human-readable ATC location name for a controller, preferring the
// facility airport city (e.g. LFLB → "Chambery") over the abstract org name
// stored in NAME (e.g. "LYON"), which may cover multiple cities.
static std::string delegated_callsign(const std::string &org); // defined near resolve_acc_controller

// Derive a spoken controller label from an openair sector NAME (design principle:
// openair = geometry + NAME, atc.dat = frequency). "MARSEILLE CTA SECTOR 2" ->
// "Marseille". Used when the aircraft is inside a NAMED enroute openair sector whose
// own controller/frequency is not in atc.dat: keep the accurate openair name for the
// label while taking the frequency from the atc.dat CTR fallback -- otherwise a
// MARSEILLE sector is announced as the broad atc.dat "France" even though the IFR
// tab (openair) resolves MARSEILLE (LFMN 2026-07-20).
static std::string openair_sector_label(const std::string &name) {
  std::string n = name;
  for (const char *kw :
       {" CTA", " TMA", " CTR", " FIR", " UIR", " SECTOR", " SEC"}) {
    const auto p = n.find(kw);
    if (p != std::string::npos) {
      n = n.substr(0, p);
      break;
    }
  }
  while (!n.empty() && n.back() == ' ')
    n.pop_back();
  if (n.empty())
    return "";
  n[0] = static_cast<char>(std::toupper(static_cast<unsigned char>(n[0])));
  for (std::size_t i = 1; i < n.size(); ++i)
    n[i] = static_cast<char>(std::tolower(static_cast<unsigned char>(n[i])));
  return n;
}

static std::string controller_label_for(const airspace_db::Controller *ctrl) {
  if (!ctrl)
    return "Area Control";
  // Delegated/renamed ACC radio callsign (atc.dat "SWITZERLAND"/LSAS -> "Swiss
  // Radar"): keep the CURRENT-controller label consistent with the handoff label
  // (resolve_acc_controller) and the STT context. Without this the raw "SWITZERLAND"
  // set at the poll_enroute/poll_descent sector sites leaked into the Voxtral bias and
  // competed with "Swiss Radar" -> readback garble "SwissRoda" (LFLP 2026-07-18).
  const std::string cs = delegated_callsign(ctrl->facility_id);
  if (!cs.empty())
    return cs;
  if (!ctrl->facility_id.empty()) {
    const std::string apt = xplane_context::airport_name_for(ctrl->facility_id);
    if (!apt.empty()) {
      // First token of the airport name is the city; stop at a space OR a '/'
      // so "Nice/Cote d'Azur" -> "Nice" (not "Nice/Cote"), "Chambery Savoie" ->
      // "Chambery", "Reims" -> "Reims".
      auto sp = apt.find_first_of(" /");
      std::string city = (sp == std::string::npos) ? apt : apt.substr(0, sp);
      if (!city.empty()) {
        city[0] = static_cast<char>(std::toupper(static_cast<unsigned char>(city[0])));
        for (std::size_t i = 1; i < city.size(); ++i)
          city[i] = static_cast<char>(std::tolower(static_cast<unsigned char>(city[i])));
        return city;
      }
    }
  }
  return controller_location(ctrl->name);
}

bool poll_departure_handoff(const xplane_context::XPlaneContext &ctx,
                            float /*dt*/, std::string *out_text) {
  using AS = atc_state_machine::ATCState;
  using FP = flight_phase::FlightPhase;
  using FT = xplane_context::FrequencyType;

  if (atc_state_machine::get_state() != AS::IFR_DEPARTURE_CLEARED)
    return false;

  auto phase = flight_phase::get();
  if (phase != FP::CLIMB && phase != FP::CRUISE)
    return false;

  // If the takeoff clearance already embedded the departure instruction via
  // {ifr_departure_contact} (e.g. "passing 3000ft, contact Chambery Approach
  // on 121.205"), the pilot already knows where to go — advance state silently
  // without issuing a duplicate or conflicting "contact X" message.
  if (!s_pending_departure_label.empty()) {
    s_current_controller_label = s_pending_departure_label;
    atc_state_machine::set_state(AS::IFR_FREQ_HANDOFF);
    s_departure_handoff_timer = 0.0f;
    logging::info("IFR departure handoff: silent (frequency already in clearance: %s)",
                  s_pending_departure_label.c_str());
    return false; // no second message — clearance already gave the instruction
  }

  // Takeoff clearance had no departure contact. Fire an explicit "contact X"
  // when the aircraft has left the CTR. Use find_enclosing() for 3-D check;
  // fall back to an AGL threshold when OpenAir data is absent or incomplete.
  openair_db::AirspaceEntry enc;
  {
    enc = openair_db::find_enclosing(
        ctx.latitude, ctx.longitude, openair_alt(ctx));
    logging::info(
        "IFR departure handoff: openair enc='%s' class=%d floor=%dft ceil=%dft"
        " at %.0fft MSL pos=%.4f,%.4f",
        enc.name.c_str(), static_cast<int>(enc.ac_class),
        enc.floor_ft, enc.ceiling_ft,
        ctx.altitude_ft_msl, ctx.latitude, ctx.longitude);
    if (enc.ac_class == openair_db::AirspaceClass::CTR)
      return false; // still inside CTR — wait

    if (enc.ac_class == openair_db::AirspaceClass::OTHER) {
      // OpenAir file absent OR this airport's CTR not in the dataset.
      // Use AGL threshold so the handoff doesn't fire at ground level.
      float airport_elev_ft = ctx.altitude_ft_msl - ctx.height_agl_ft;
      int ctr_msl =
          openair_db::ctr_ceiling_ft(ctx.airport_lat, ctx.airport_lon);
      float threshold_agl =
          ctr_msl > 0 ? static_cast<float>(ctr_msl) - airport_elev_ft : 2500.0f;
      if (ctx.height_agl_ft < threshold_agl)
        return false;
    }
    // TMA / CTA / FIR / UIR → aircraft has left the CTR, proceed.
  }

  std::string controller_label;
  float freq = 0.0f;

  // Primary: use the openair TMA name to identify the correct TRACON in
  // atc.dat by name.  This is geometrically exact: openair has altitude-aware
  // polygons (e.g. CHAMBERY TMA 1000-9500ft vs GENEVA TMA FL095-FL195), so
  // the aircraft can never be handed off to a controller whose airspace starts
  // above its current altitude.
  //
  // "CHAMBERY TMA SECTOR 1" → city fragment "CHAMBERY"
  // → find atc.dat TRACON with NAME containing "CHAMBERY" → Chambery APP.
  // Unified resolver (step 1c, 2026-07-18): was an inline duplicate of
  // resolve_tma_controller. resolve_sector_controller(terminal=true) resolves a
  // TMA/CTA name -> atc.dat TRACON, label "<City> Approach" -- identical output, plus
  // the shared TWR-facility fallback (so a few names that previously fell to P2 now
  // resolve at P1). Non-TMA/CTA classes return false -> the P2/P3 fallbacks below.
  {
    std::string lbl;
    float mhz = 0.0f;
    if (resolve_sector_controller(enc, /*terminal=*/true, &lbl, &mhz)) {
      controller_label = lbl;
      freq = mhz;
      logging::info("IFR departure handoff: [P1-openair] '%s' -> %s %.3f",
                    enc.name.c_str(), controller_label.c_str(), freq);
    } else {
      logging::info(
          "IFR departure handoff: [P1-openair] '%s' -> no match, falling back",
          enc.name.c_str());
    }
  }

  // Fallback 1: departure airport's own DEPARTURE / APPROACH frequency from
  // apt.dat.  Used when openair has no named TMA (OTHER) or name matching
  // failed.
  if (freq < 100.0f) {
    const std::string fallback = ctx.nearest_airport_name.empty()
                                     ? ctx.nearest_airport_id
                                     : ctx.nearest_airport_name;
    float dep_freq = ctx.airport_freqs.first_mhz(FT::DEPARTURE);
    float app_freq = ctx.airport_freqs.first_mhz(FT::APPROACH);
    if (dep_freq >= 100.0f) {
      std::string raw = ctx.airport_freqs.first_name(FT::DEPARTURE);
      controller_label = raw.empty() ? (fallback + " Departure")
                                     : controller_location(raw) + " Departure";
      if (!raw.empty() && (raw.find("RADAR") != std::string::npos ||
                           raw.find("CONTROL") != std::string::npos ||
                           raw.find("CTL") != std::string::npos))
        controller_label = controller_location(raw);
      freq = dep_freq;
      logging::info("IFR departure handoff: [P2-apt.dat DEP] %s %.3f",
                    controller_label.c_str(), freq);
    } else if (app_freq >= 100.0f) {
      std::string raw = ctx.airport_freqs.first_name(FT::APPROACH);
      controller_label = raw.empty() ? (fallback + " Approach")
                                     : controller_location(raw) + " Approach";
      if (!raw.empty() && (raw.find("RADAR") != std::string::npos ||
                           raw.find("CONTROL") != std::string::npos ||
                           raw.find("CTL") != std::string::npos))
        controller_label = controller_location(raw);
      freq = app_freq;
      logging::info("IFR departure handoff: [P2-apt.dat APP] %s %.3f",
                    controller_label.c_str(), freq);
    } else {
      logging::info(
          "IFR departure handoff: [P2-apt.dat] no DEP/APP freq for %s",
          ctx.nearest_airport_id.c_str());
    }
  }

  // Fallback 2: nearest TRACON in atc.dat, queried at the DEPARTURE AIRPORT
  // position (not the aircraft's current position).  Querying at the airport
  // avoids selecting a controller that is geographically closer to the aircraft
  // but belongs to a different airport's TMA.
  if (freq < 100.0f) {
    const airspace_db::Controller *tracon = airspace_db::find_by_role_near(
        airspace_db::ControllerRole::TRACON,
        ctx.airport_lat, ctx.airport_lon,
        ctx.altitude_ft_msl, /*prefer_largest_area=*/true);
    if (tracon && !tracon->freqs_khz.empty()) {
      freq = static_cast<float>(tracon->freqs_khz.front()) / 1000.0f;
      controller_label = controller_label_for(tracon);
      logging::info(
          "IFR departure handoff: [P3-atc.dat at airport %.4f,%.4f] %s %.3f",
          ctx.airport_lat, ctx.airport_lon,
          controller_label.c_str(), freq);
    } else {
      logging::info(
          "IFR departure handoff: [P3-atc.dat] no TRACON near airport %.4f,%.4f",
          ctx.airport_lat, ctx.airport_lon);
    }
  }

  // Transition to IFR_FREQ_HANDOFF: pilot must read back the frequency before
  // advancing to IFR_EN_ROUTE. Even with no frequency we advance so the state
  // doesn't get stuck in IFR_DEPARTURE_CLEARED forever.
  if (!controller_label.empty())
    s_current_controller_label = controller_label;
  s_pending_handoff_freq_mhz = freq;
  logging::debug("[DBG] pending_handoff_freq=%.3f [dept-freq-handoff ctrl=%s]",
                 freq, controller_label.c_str());
  atc_state_machine::set_state(AS::IFR_FREQ_HANDOFF);
  s_departure_handoff_timer = 0.0f;

  if (controller_label.empty())
    return false; // uncontrolled airspace — silent transition, nothing to speak

  const std::string &cs = atc_state_machine::session_callsign();
  const std::string &callsign = cs.empty() ? settings::pilot_callsign() : cs;

  if (out_text) {
    char buf[160];
    std::snprintf(buf, sizeof(buf), "%s, contact %s on %.3f, good day.",
                  callsign.c_str(), controller_label.c_str(), freq);
    *out_text = buf;
  }

  logging::info("IFR departure handoff: contact %s %.3f",
                controller_label.c_str(), freq);
  return true;
}

// Helper: round feet to the nearest FL boundary (500 ft increments) and
// return the FL number as an integer (e.g. 19000 ft → 190).
static int round_to_fl(int feet) {
  // Round to the nearest 1000 ft — IFR flight levels ALWAYS end in a
  // round thousand (FL110, FL210, FL220, ...). The +500 midpoint gives
  // conventional "half up" rounding. Returns FL as units of 100 ft
  // (e.g. 21000 ft -> 210).
  // Bug fix (v4.3.1): previously rounded up to nearest 500 ft, which
  // produced VFR-level outputs like FL215 (from 21500 ft) and FL205
  // (from 20500 ft) when the caller passed a mid-climb SimBrief
  // predicted altitude. Real IFR ATC never assigns x500 levels — those
  // are VFR-only. See feedback / reference_ifr_vfr_flight_levels
  // (round thousands = IFR, +500 = VFR).
  int fl_units_1000 = (feet + 500) / 1000;
  return fl_units_1000 * 10;
}

// Returns the Transition Level in feet (pressure altitude reference).
// ICAO formula: TL_pressure_alt = TA + (1013 - QNH) * 27 ft, rounded UP
// to the nearest 500-ft FL boundary.  Guarantees at least 500 ft above TA.
// Use to decide FL vs. altitude notation in ATC clearances: if the assigned
// pressure altitude is below the returned value, express as "X feet, QNH Y";
// at or above, express as "flight level X".
static int compute_tl_ft(int ta_ft, int qnh_hpa) {
  int pa = ta_ft + (1013 - qnh_hpa) * 27;
  int tl = ((pa + 499) / 500) * 500;
  return tl > ta_ft ? tl : ta_ft + 500;
}

static double
procedure_deviation_nm(const xplane_context::XPlaneContext &ctx,
                       const std::vector<simbrief_ofp::NavlogFix> &navlog,
                       bool sid_star_only, const std::string &direct_fix,
                       double direct_from_lat, double direct_from_lon);

// True while the step-1 hold is active -- a hold distance is set and the aircraft
// is still within it (great-circle from the departure airport) AND the SID does
// not itself demand a climb above step-1. Suppresses both the cruise climb and the
// radar handoff so the aircraft stays at step-1 until clear (LFLP west SIDs).
static bool sid_step1_hold_active(const xplane_context::XPlaneContext &ctx) {
  if (s_sid_hold_release_nm <= 0.0f)
    return false;
  if (ctx.ifr_sid_min_alt_ft > s_sid_step1_alt_ft)
    return false; // SID's own minimum demands a higher climb -> do not hold
  if (s_departure_apt_lat == 0.0 && s_departure_apt_lon == 0.0)
    return false; // departure fix not captured -> cannot measure -> do not hold
  const double d = traffic_geometry::distance_nm(
      ctx.latitude, ctx.longitude, s_departure_apt_lat, s_departure_apt_lon);
  return d < static_cast<double>(s_sid_hold_release_nm);
}

bool poll_sid_climb(const xplane_context::XPlaneContext &ctx, float dt,
                    std::string *out_text) {
  using AS = atc_state_machine::ATCState;
  using FP = flight_phase::FlightPhase;

  if (atc_state_machine::get_state() != AS::IFR_RADAR_CONTACT) {
    s_sid_direct_issued = false;
    s_sid_step1_issued = false;
    s_sid_cruise_issued = false;
    s_sid_radar_handoff_issued = false;
    s_sid_intermediate_tracon_khz_seen.clear();
    s_sid_was_in_tma = false;
    s_sid_tma_check_sec = 0.0f;
    s_sid_initialized = false;
    s_sid_climb_timer = 0.0f;
    s_sid_step1_alt_ft = 0;
    s_sid_hold_release_nm = 0.0f;
    s_sid_deviation_cooldown_sec = 0.0f;
    s_sid_direct_origin_lat = 0.0;
    s_sid_direct_origin_lon = 0.0;
    s_sid_direct_elapsed_sec = 0.0f;
    s_departure_apt_id.clear();
    s_departure_apt_lat = 0.0;
    s_departure_apt_lon = 0.0;
    return false;
  }

  auto phase = flight_phase::get();
  if (phase == FP::PARKED || phase == FP::TAXI) {
    // Aircraft back on ground — auto_correction in flight_rules.json handles
    // the state reset; don't fire climb clearances.
    return false;
  }

  s_sid_climb_timer += dt;

  const auto &defaults = flight_phase::get_ifr_defaults();
  const std::string &cs = atc_state_machine::session_callsign();
  const std::string &callsign = cs.empty() ? settings::pilot_callsign() : cs;

  // One-time initialisation on first entry to IFR_RADAR_CONTACT.
  if (!s_sid_initialized) {
    s_sid_initialized = true;
    s_departure_apt_id  = ctx.nearest_airport_id; // still the departure field here
    s_departure_apt_lat = ctx.airport_lat;
    s_departure_apt_lon = ctx.airport_lon;
    // Build the DEPARTURE half of the unified route table (SID + enroute
    // navlog) so the compliance monitor / speed enforcement work during the
    // climb, exactly as the arrival table serves approach. The arrival re-init
    // (init_route_fixes) later rebuilds it for the STAR/approach.
    build_sid_route_table(ctx);

    // Step1 altitude: LFLP RW04 westbound → FL110 (clear of Geneva TMA
    // after SOCOF). All other cases: midpoint between SID minimum and
    // cruise, rounded to the nearest FL.
    bool lflp_west = false;
    // The west-departure SIDs (ROMA2A/LSE2A/LTP2A ...) are RW22 SIDs but can also
    // be joined off runway 04 under vis/ceiling limits (user 2026-07-21), so the
    // FL110 (clear-of-Geneva) case must key on the WEST FIX, not the runway. The
    // gate previously required "04", so a normal ROMA2A off 22 fell through to the
    // generic FL120 step (LFLP->LFMD 2026-07-21).
    if (ctx.nearest_airport_id == "LFLP") {
      const std::string &fix = ctx.ifr_sid_last_fix.empty()
                                   ? ctx.ifr_fpl_first_fix
                                   : ctx.ifr_sid_last_fix;
      static const char *kWestFixes[] = {"LSE", "LTP", "ROMAM", nullptr};
      for (int i = 0; kWestFixes[i]; ++i)
        if (fix == kWestFixes[i]) {
          lflp_west = true;
          break;
        }
    }
    if (lflp_west) {
      s_sid_step1_alt_ft = 11000; // FL110 — clear of Geneva TMA after SOCOF
      // Hold FL110 until 30 NM (great-circle) from LFLP before releasing the
      // cruise climb + radar handoff: Chambery keeps west departures low until
      // past ~TOLNA to avoid a pointless short handoff to Geneva APP (user
      // 2026-07-21). Great-circle from the field is a good-enough proxy for the
      // fix here. FUTURE: move this whole west-SID case (hold_alt + release) into
      // airport+.json as a per-airport "departure_holds" section so it is not
      // engine-hardcoded -- the loader (airport_overrides) already exists.
      s_sid_hold_release_nm = 30.0f;
    } else {
      int floor_ft = ctx.ifr_sid_min_alt_ft > 0 ? ctx.ifr_sid_min_alt_ft : 5000;
      int cruise_ft =
          ctx.ifr_cruise_alt_ft > 0 ? ctx.ifr_cruise_alt_ft : floor_ft + 8000;
      // When the SID minimum exceeds the filed cruise FL the midpoint would be
      // above cruise — meaningless. Issue the SID minimum directly so the
      // message is at least correct ("climb FL150" vs a phantom "climb FL120").
      int mid_ft = floor_ft >= cruise_ft ? floor_ft : (floor_ft + cruise_ft) / 2;
      s_sid_step1_alt_ft = round_to_fl(mid_ft) * 100;
      logging::info("IFR SID step1 calc: sid_min=%dft cruise=%dft -> FL%d%s",
                    floor_ft, cruise_ft, round_to_fl(mid_ft),
                    floor_ft >= cruise_ft
                        ? " [SID min ABOVE cruise - FPL too low]" : "");
    }
  }

  // ── Phase 2.8: intermediate APP -> Radar/Center handoff during SID climb
  // Fires the "contact Milan on 118.675" / "contact Lyon on 123.700" call
  // mid-climb instead of holding the pilot on the departure Approach
  // controller all the way to the top of the CTA stack (FL195+ in EU).
  //
  // Scenario this fixes: LIMF -> LFLP. Torino APP is the departure
  // controller (freq 121.100 from apt.dat). Above FL100-FL120 real ATC
  // hands the pilot to Milan Radar / Milan ACC. Without this check, the
  // pilot stayed on Torino APP all the way to FL220 (the "exited all
  // TMAs" Phase 3 primary handoff), which is unrealistic.
  //
  // Handoff target: innermost TRACON if any exists in ctx.enclosing_airspaces
  // (matches sector-change preference); falls back to innermost CTR when
  // no TRACON polygon covers the position. MILAN RADAR's atc.dat TRACON
  // polygons are geographically limited to the Milan hub area — for
  // LIMF -> LFLP the aircraft's climb corridor is NOT inside them, but
  // MILAN CTR (blanket 0-60000) IS enclosing. Without the CTR fallback,
  // Phase 2.8 finds no TRACON and never fires. The CTR fallback picks up
  // MILAN CTR (freq 118.675, different from Torino APP 121.100) and the
  // handoff fires correctly.
  //
  // Guards (order matters):
  //   1. State = IFR_RADAR_CONTACT + SID init complete (existing block).
  //   2. Altitude >= sid_handoff_min_alt_ft (ifr_defaults):
  //      EU default 12000 ft (FL120), US default 10000 ft (FL100).
  //      Prevents firing during the initial climb where Approach still
  //      works the aircraft. Struct default 10000; EU JSON overrides to
  //      12000 via data/atc_profiles/eu/ifr/flight_rules.json.
  //   3. Best controller's freq differs from s_pending_handoff_freq_mhz.
  //   4. Fire-once per controller freq (kHz key) via
  //      s_sid_intermediate_tracon_khz_seen — prevents retrigger on
  //      polygon re-entry or stacked sub-polygon boundaries.
  //
  // Does NOT replace the "exited all TMAs" primary handoff below (Phase 3);
  // this only ADDS an intermediate handoff between Tower/APP and Centre.
  if (!s_sid_radar_handoff_issued && s_sid_initialized) {
    const int handoff_min_ft = defaults.sid_handoff_min_alt_ft > 0
                                   ? defaults.sid_handoff_min_alt_ft
                                   : 10000;
    if (static_cast<int>(ctx.altitude_ft_msl) >= handoff_min_ft) {
      using CR = airspace_db::ControllerRole;
      // Innermost TRACON first (matches sector-change logic); CTR fallback
      // when no TRACON polygon covers the position.
      const airspace_db::Controller *best = nullptr;
      for (const auto *c : ctx.enclosing_airspaces) {
        if (!c || c->freqs_khz.empty()) continue;
        if (c->role == CR::TRACON) {
          if (!best || best->role != CR::TRACON ||
              c->floor_ft > best->floor_ft)
            best = c;
        } else if (c->role == CR::CTR) {
          if (!best || (best->role != CR::TRACON &&
                        c->floor_ft > best->floor_ft))
            best = c;
        }
      }
      if (best) {
        uint32_t new_khz = best->freqs_khz.front();
        float new_mhz = static_cast<float>(new_khz) / 1000.0f;
        const bool same_as_pending =
            s_pending_handoff_freq_mhz > 100.0f &&
            std::fabs(new_mhz - s_pending_handoff_freq_mhz) < 0.005f;
        const bool already_seen =
            s_sid_intermediate_tracon_khz_seen.count(new_khz) > 0;
        if (!same_as_pending && !already_seen) {
          std::string new_label = controller_label_for(best);
          if (new_label.empty())
            new_label = (best->role == CR::TRACON) ? "Radar" : "Control";
          const std::string &cs_ir = atc_state_machine::session_callsign();
          const std::string &callsign_ir =
              cs_ir.empty() ? settings::pilot_callsign() : cs_ir;
          if (out_text) {
            char buf[160];
            std::snprintf(buf, sizeof(buf), "%s, contact %s on %.3f.",
                          callsign_ir.c_str(), new_label.c_str(), new_mhz);
            *out_text = buf;
          }
          s_sid_intermediate_tracon_khz_seen.insert(new_khz);
          // Defer the label switch: previous controller (e.g. Torino) is
          // still the speaker until the pilot actually moves to the new
          // frequency.  Stash the new label as pending; the sector-checkin
          // block will swap it into current when active COM matches.
          s_pending_controller_label = new_label;
          s_pending_handoff_freq_mhz = new_mhz;
          s_sector_checkin_pending = true;
          logging::info(
              "IFR SID climb: intermediate handoff -> %s %.3f MHz "
              "at %.0f ft MSL (role=%s)",
              new_label.c_str(), new_mhz, ctx.altitude_ft_msl,
              best->role == CR::TRACON ? "TRACON" : "CTR");
          // Coarseness diagnostic: warn when the picked controller is a
          // wide-blanket FIR-level entry (e.g. LIMM CTR spanning 0-FL600
          // across all of northern Italy) with a flat freq list. Selected
          // freq comes from freqs_khz.front() — arbitrary and unlikely to
          // match the real-world sub-CTA sector working freq. atc.dat
          // simply does not have per-sub-CTA granularity; proper fix
          // requires the airspace+.txt overlay + polygon-name-to-freq
          // map planned for v4.4.0.
          {
            const double bbox_lat_span =
                best->has_bbox ? (best->bbox_max_lat - best->bbox_min_lat) : 0.0;
            const double bbox_lon_span =
                best->has_bbox ? (best->bbox_max_lon - best->bbox_min_lon) : 0.0;
            const int alt_span = best->ceiling_ft - best->floor_ft;
            const size_t nfreqs = best->freqs_khz.size();
            const bool coarse =
                alt_span >= 40000 ||        // >= FL400 span (FIR-blanket signature)
                bbox_lat_span > 1.5 ||      // >~90 NM lat span
                bbox_lon_span > 1.5 ||      // >~65 NM lon span at mid-latitude
                nfreqs >= 5;                // flat pool of many freqs
            if (coarse) {
              logging::info(
                  "IFR SID climb: WARN coarse controller data -- %s covers "
                  "alt %d-%d ft, bbox %.2f x %.2f deg, %zu freqs; picked "
                  "freqs_khz.front()=%.3f which may not match real sub-CTA "
                  "sector freq (data-source limitation, v4.4.0 fix planned)",
                  best->name.c_str(), best->floor_ft, best->ceiling_ft,
                  bbox_lat_span, bbox_lon_span, nfreqs, new_mhz);
            }
          }
          return true;
        }
      }
    }
  }

  // ── Phase 3: radar handoff — fires when aircraft exits the TMA ───────
  // Requires step1 already issued so we never hand off before the first
  // climb clearance. Use find_enclosing() on the aircraft's 3-D position:
  // while still inside a CTR or TMA we hold; once in CTA/FIR/uncontrolled
  // we hand off to Area Control / Radar.
  // Fall back to a configured or computed altitude when airspace.txt is absent.
  if (!s_sid_radar_handoff_issued && s_sid_step1_issued) {
    // Don't hand off until the aircraft is approaching step1 altitude.
    // This prevents an immediate handoff right after step1 is issued
    // when the aircraft is still far below (e.g. at 6700 ft for FL170 step1).
    if (static_cast<int>(ctx.altitude_ft_msl) < s_sid_step1_alt_ft - 2000)
      goto skip_tma_check;

    // Compute the altitude-based fallback threshold regardless of openair_db.
    // Used when openair_db is absent OR when it is present but never detected
    // the aircraft inside a CTR/TMA (data gap — e.g. Chambery TMA not in atc.dat).
    int handoff_fallback_ft =
        defaults.radar_handoff_alt_ft > 0
            ? defaults.radar_handoff_alt_ft
            : (ctx.ifr_cruise_alt_ft > 2000 ? ctx.ifr_cruise_alt_ft - 2000
                                            : 14000);
    // step1+1000 guard prevents firing before the aircraft has climbed past
    // step1. Cap at cruise-500 so it stays reachable when the filed cruise FL
    // is below the SID minimum (step1 > cruise in that case).
    {
      const int step1_guard = ctx.ifr_cruise_alt_ft > 2000
          ? std::min(s_sid_step1_alt_ft + 1000, ctx.ifr_cruise_alt_ft - 500)
          : s_sid_step1_alt_ft + 1000;
      handoff_fallback_ft = std::max(handoff_fallback_ft, step1_guard);
    }
    bool exited_tma = false;
    s_sid_tma_check_sec -= dt;
    if (s_sid_tma_check_sec <= 0.0f) {
      s_sid_tma_check_sec = 1.0f; // 1 Hz — TMA boundary at cruise speed ~4 NM/min
      logging::debug("[DBG] SID handoff: fallback=%dft cruise=%dft step1=%dft timer=%.0fs",
                     handoff_fallback_ft, ctx.ifr_cruise_alt_ft,
                     s_sid_step1_alt_ft, s_sid_climb_timer);
      if (openair_db::ready()) {
        // Check ALL enclosing airspaces — a large background CTR (e.g. CTR
        // MARSEILLE covering a whole FIR) must not mask smaller CTAs inside it.
        auto all = openair_db::find_all_enclosing(
            ctx.latitude, ctx.longitude, openair_alt(ctx));
        bool in_tma_now = false;
        std::string tma_names;
        std::string all_zones; // all enclosing zones with floor/ceiling for pos log
        for (const auto &e : all) {
          // Build full zone string with altitude bands for every enclosing zone.
          char zbuf[128];
          std::snprintf(zbuf, sizeof(zbuf), "%s(%d-%dft)",
                        e.name.c_str(), e.floor_ft, e.ceiling_ft);
          if (!all_zones.empty()) all_zones += ", ";
          all_zones += zbuf;

          if (e.ac_class == openair_db::AirspaceClass::CTR ||
              e.ac_class == openair_db::AirspaceClass::TMA) {
            in_tma_now = true;
            if (!tma_names.empty()) tma_names += ", ";
            tma_names += zbuf; // include altitude in TMA check log too
          }
        }
        // Only log on state change to avoid filling Log.txt during long SID climbs.
        if (in_tma_now != s_sid_was_in_tma || (!tma_names.empty() && !s_sid_was_in_tma))
          logging::info("IFR SID TMA check: in_tma=%d was_in=%d zones=[%s] alt=%.0fft",
                        in_tma_now, s_sid_was_in_tma,
                        tma_names.empty() ? (all.empty() ? "none" : "CTA/FIR only")
                                          : tma_names.c_str(),
                        ctx.altitude_ft_msl);
        // Periodic position log (every 60 s) — lets us replay the trajectory
        // from Log.txt without needing a live sim session.
        s_sid_pos_log_sec -= 1.0f; // decremented at 1 Hz
        if (s_sid_pos_log_sec <= 0.0f) {
          s_sid_pos_log_sec = 60.0f;
          logging::info(
              "IFR pos: lat=%.4f lon=%.4f alt=%.0fft hdg=%.0f gs=%.0fkt "
              "zones=[%s] timer=%.0fs",
              ctx.latitude, ctx.longitude, ctx.altitude_ft_msl,
              ctx.heading_true, ctx.groundspeed_kts,
              all_zones.empty() ? "none" : all_zones.c_str(),
              s_sid_climb_timer);
        }
        if (in_tma_now)
          s_sid_was_in_tma = true;
        // Primary: all CTR/TMA polygons exited (requires prior entry to prevent
        // false fires when the departure altitude is below the TMA floor).
        if (s_sid_was_in_tma && !in_tma_now) {
          exited_tma = true;
        } else if (!s_sid_was_in_tma) {
          // openair_db present but this airport's CTR/TMA is not in the dataset
          // (or the aircraft passed through the CTR ceiling before Phase 3 began
          // tracking). Require the same stuck timer as Path 3 so we don't fire
          // before the aircraft has left the SID exit fix.
          const bool stuck_no_tma =
              s_sid_climb_timer > defaults.radar_handoff_stuck_timer_sec;
          exited_tma = stuck_no_tma &&
                       static_cast<int>(ctx.altitude_ft_msl) >= handoff_fallback_ft;
        } else {
          // Still inside a CTR/TMA. Safety net: fire after configured stuck
          // timer (default 3 min) once the aircraft has reached handoff altitude.
          const bool stuck =
              s_sid_climb_timer > defaults.radar_handoff_stuck_timer_sec;
          exited_tma =
              stuck && static_cast<int>(ctx.altitude_ft_msl) >= handoff_fallback_ft;
        }
      } else {
        exited_tma =
            static_cast<int>(ctx.altitude_ft_msl) >= handoff_fallback_ft;
      }
    }

    if (exited_tma && !sid_step1_hold_active(ctx)) {
      // Look up Centre controller — use ctx.enclosing_airspaces (polygon
      // containment, same source as the sector check and the EN ROUTE tab UI)
      // so the SID handoff and the sector check always agree.
      // find_by_role_near (proximity) was previously used here but can pick a
      // nearby airport's TRACON (e.g. LFLY near LFLL) instead of the
      // geographically enclosing one, leading to stale pending_freq that
      // triggers false "I say again" messages.
      std::string centre_label;
      float centre_freq = 0.0f;
      {
        using CR = airspace_db::ControllerRole;
        const airspace_db::Controller *best = nullptr;
        for (const auto *c : ctx.enclosing_airspaces) {
          if (c->freqs_khz.empty()) continue;
          if (c->role == CR::TRACON) {
            if (!best || best->role != CR::TRACON || c->floor_ft > best->floor_ft)
              best = c;
          } else if (c->role == CR::CTR) {
            if (!best || (best->role != CR::TRACON && c->floor_ft > best->floor_ft))
              best = c;
          }
        }
        if (!best) {
          // No enclosing sector — fall back to proximity lookup.
          best = airspace_db::find_by_role_near(
              CR::TRACON, ctx.latitude, ctx.longitude,
              ctx.altitude_ft_msl, /*prefer_largest_area=*/false);
          if (!best)
            best = airspace_db::find_by_role_near(
                CR::CTR, ctx.latitude, ctx.longitude,
                ctx.altitude_ft_msl, /*prefer_largest_area=*/true);
          if (best)
            logging::debug("[DBG] sid-radar: no enclosing sector, fallback "
                           "find_by_role_near -> %s", best->name.c_str());
        }
        if (best && !best->freqs_khz.empty()) {
          centre_label = controller_label_for(best);
          centre_freq = static_cast<float>(best->freqs_khz.front()) / 1000.0f;
        }
      }
      if (centre_label.empty())
        centre_label = "Area Control";
      s_current_controller_label = centre_label;
      s_pending_handoff_freq_mhz = centre_freq;
      logging::debug("[DBG] pending_handoff_freq=%.3f [sid-radar ctrl=%s]",
                     centre_freq, centre_label.c_str());
      s_sid_radar_handoff_issued = true;
      if (s_enroute_cleared_alt_ft == 0) {
        if (ctx.ifr_cruise_alt_ft > 0)
          s_enroute_cleared_alt_ft = round_to_fl(ctx.ifr_cruise_alt_ft) * 100;
        else if (s_sid_step1_alt_ft > 0)
          s_enroute_cleared_alt_ft = s_sid_step1_alt_ft;
      }
      atc_state_machine::set_state(AS::IFR_ENROUTE_CRUISE);

      // If the pilot is already on the handoff frequency (e.g. Tower already
      // handed them to the TRACON Chambery sector on 121.205 and Phase 3 finds
      // the same Lyon TRACON on 121.205), advance state silently — announcing
      // "contact Lyon on 121.205" while already on that frequency is confusing.
      const float active_com_freq_mhz =
          (ctx.active_com == 2) ? ctx.com2_freq_mhz : ctx.com1_freq_mhz;
      const bool same_freq =
          centre_freq >= 100.0f &&
          std::abs(centre_freq - active_com_freq_mhz) < 0.010f;
      if (same_freq) {
        s_sid_cruise_issued = true;
        logging::info(
            "IFR SID climb: silent state advance to ENROUTE_CRUISE (already on "
            "%s %.3f) at %.0f ft MSL",
            centre_label.c_str(), centre_freq, ctx.altitude_ft_msl);
        return false;
      }

      if (!s_sid_cruise_issued) {
        // Phase 2 (near step1) hasn't fired yet — combine cruise clearance
        // and radar handoff in a single transmission.  Issuing them as two
        // rapid consecutive messages (previous two-frame split) caused pilots
        // to miss the altitude change while reacting to the freq change.
        s_sid_cruise_issued = true;
        int cruise_fl =
            round_to_fl(ctx.ifr_cruise_alt_ft > 0 ? ctx.ifr_cruise_alt_ft
                                                  : s_sid_step1_alt_ft + 4000);
        s_enroute_cleared_alt_ft = cruise_fl * 100;
        // Omit "climb FL..." when aircraft is already at or above cruise FL
        // to avoid issuing a step-down instruction.
        const bool already_at_cruise =
            static_cast<int>(ctx.altitude_ft_msl) >= cruise_fl * 100 - 500;
        if (out_text) {
          char buf[200];
          if (already_at_cruise) {
            if (centre_freq >= 100.0f)
              std::snprintf(buf, sizeof(buf), "%s, contact %s on %.3f, good day.",
                            callsign.c_str(), centre_label.c_str(), centre_freq);
            else
              std::snprintf(buf, sizeof(buf), "%s, contact %s, good day.",
                            callsign.c_str(), centre_label.c_str());
          } else if (centre_freq >= 100.0f)
            std::snprintf(buf, sizeof(buf),
                          "%s, climb flight level %d, contact %s on %.3f, "
                          "good day.",
                          callsign.c_str(), cruise_fl, centre_label.c_str(),
                          centre_freq);
          else
            std::snprintf(buf, sizeof(buf),
                          "%s, climb flight level %d, contact %s, good day.",
                          callsign.c_str(), cruise_fl, centre_label.c_str());
          *out_text = buf;
        }
        if (already_at_cruise)
          logging::info(
              "IFR SID climb: at cruise + handoff to %s at %.0f ft MSL (combined)",
              centre_label.c_str(), ctx.altitude_ft_msl);
        else
          logging::info(
              "IFR SID climb: FL%d + handoff to %s at %.0f ft MSL (TMA exit, combined)",
              cruise_fl, centre_label.c_str(), ctx.altitude_ft_msl);
        return true;
      }

      // Phase 2 already issued the cruise clearance — just hand off.
      if (out_text) {
        char buf[160];
        if (centre_freq >= 100.0f)
          std::snprintf(buf, sizeof(buf), "%s, contact %s on %.3f, good day.",
                        callsign.c_str(), centre_label.c_str(), centre_freq);
        else
          std::snprintf(buf, sizeof(buf), "%s, contact %s, good day.",
                        callsign.c_str(), centre_label.c_str());
        *out_text = buf;
      }
      logging::info("IFR SID climb: radar handoff at %.0f ft MSL (exited TMA/CTR)",
                    ctx.altitude_ft_msl);
      return true;
    }
  }
skip_tma_check:;

  // ── Phase 2: climb to cruise FL when near step1 altitude ──────────────
  if (s_sid_step1_issued && !s_sid_cruise_issued) {
    int cruise_fl =
        round_to_fl(ctx.ifr_cruise_alt_ft > 0 ? ctx.ifr_cruise_alt_ft
                                              : s_sid_step1_alt_ft + 4000);
    bool near_step1 = std::abs(static_cast<int>(ctx.altitude_ft_msl) -
                               s_sid_step1_alt_ft) < 500;
    bool timeout = s_sid_climb_timer > 900.0f; // 15-min safety net
    if ((near_step1 || timeout) && !sid_step1_hold_active(ctx)) {
      s_sid_cruise_issued = true;
      s_enroute_cleared_alt_ft =
          cruise_fl * 100; // record for en-route altitude monitoring
      if (out_text) {
        char buf[64];
        std::snprintf(buf, sizeof(buf), "%s, climb flight level %d.",
                      callsign.c_str(), cruise_fl);
        *out_text = buf;
      }
      logging::info("IFR SID climb: FL%d (cruise clearance)", cruise_fl);
      return true;
    }
  }

  // ── Phase 1: direct-to shortcut + initial step climb ──────────────────
  // Fire when the aircraft is ≥10 NM from the departure airport.
  // The 600 s fallback catches cases where airport_lat/lon were not captured
  // (e.g. apt.dat parse still in progress when radar contact was established).
  {
    double dist_nm = (s_departure_apt_lat != 0.0 || s_departure_apt_lon != 0.0)
                         ? traffic_geometry::distance_nm(
                               ctx.latitude, ctx.longitude, s_departure_apt_lat,
                               s_departure_apt_lon)
                         : 0.0;
    bool far_enough = dist_nm >= 10.0;
    bool fallback = s_sid_climb_timer > 600.0f;
    if (!s_sid_step1_issued && (far_enough || fallback)) {
      // 20 % probability: fire the "direct <SID last fix>, climb FL X"
      // shortcut. 80 % of the time issue a plain "climb FL X" without the
      // direct-to — models real ATC variability (many controllers let the
      // aircraft follow the full SID silently). Only gate if we actually
      // have a valid SID last fix to direct-to; otherwise we always
      // issue the plain climb (no shortcut possible).
      const bool have_last_fix = !ctx.ifr_sid_last_fix.empty();
      const bool fire_direct = have_last_fix && ((std::rand() % 5) == 0);
      s_sid_step1_issued = true;
      s_sid_direct_issued = fire_direct;
      if (fire_direct) {
        s_sid_direct_origin_lat = ctx.latitude;
        s_sid_direct_origin_lon = ctx.longitude;
        s_sid_direct_elapsed_sec = 0.0f;
        // Advance route tracker to the direct fix — mirrors the STAR /
        // approach direct-to pattern (engine.cpp ~5102) so downstream
        // "next fix" reporting reflects the actual routing. Without this
        // the tracker keeps advancing through intermediate SID waypoints
        // that the aircraft is now bypassing (e.g. MF702, MF418 for LIMF
        // KUKE1X → direct KUKEV).
        const std::string &target = ctx.ifr_sid_last_fix;
        for (int ri = s_route_fix_idx;
             ri < static_cast<int>(s_route_fixes.size()); ++ri) {
          if (s_route_fixes[ri].ident == target) {
            s_route_fix_idx = ri;
            s_pending_route_direct = "ATC direct: " + target;
            logging::info("[route] ATC direct: %s (idx=%d, SID)",
                          target.c_str(), ri);
            break;
          }
        }
      }
      // Give the FMS time to intercept the new direct track before the
      // cross-track deviation check can fire.
      s_sid_deviation_cooldown_sec = defaults.sid_deviation_cooldown_sec;
      int step1_fl = round_to_fl(s_sid_step1_alt_ft);
      const int cruise_ft = ctx.ifr_cruise_alt_ft > 0
                                ? ctx.ifr_cruise_alt_ft
                                : (s_sid_step1_alt_ft + 4000);
      // Aircraft already above step1 — issuing "climb FL%d" would be a
      // step-down. Mark step1 done silently and let Phase 2/3 proceed next
      // frame. Also pre-mark cruise done if already at cruise altitude so
      // Phase 3 won't repeat the climb.
      if (static_cast<int>(ctx.altitude_ft_msl) >= s_sid_step1_alt_ft) {
        if (static_cast<int>(ctx.altitude_ft_msl) >= cruise_ft - 1000) {
          s_sid_cruise_issued = true;
          s_enroute_cleared_alt_ft = round_to_fl(cruise_ft) * 100;
        }
        logging::info("IFR SID climb: skipping step1 FL%d (already at %.0f ft)",
                      step1_fl, ctx.altitude_ft_msl);
        return false;
      }
      if (out_text) {
        const std::string &last_fix = ctx.ifr_sid_last_fix;
        char buf[128];
        // Only speak "direct FIX" when the 20% probability gate above
        // (fire_direct) actually fired. Without this second check the
        // clearance always said "direct FIX, climb FL X" while only the
        // internal s_sid_direct_issued flag respected the gate.
        if (s_sid_direct_issued && !last_fix.empty()) {
          std::snprintf(buf, sizeof(buf),
                        "%s, direct %s, climb flight level %d.",
                        callsign.c_str(), last_fix.c_str(), step1_fl);
        } else {
          std::snprintf(buf, sizeof(buf), "%s, climb flight level %d.",
                        callsign.c_str(), step1_fl);
        }
        *out_text = buf;
      }
      // Log matches what the pilot actually heard: only say "direct FIX"
      // in the log if the 20 % gate above actually fired the direct-to
      // shortcut. Previously this always said " direct" whenever
      // ifr_sid_last_fix was set, misleading users into thinking the gate
      // was firing 100 % of flights when it was silently going the 80 %
      // (no-direct) branch.
      logging::info("IFR SID climb: FL%d%s (step1)", step1_fl,
                    s_sid_direct_issued && !ctx.ifr_sid_last_fix.empty()
                        ? " direct"
                        : "");
      return true;
    }
  }

  // ── SID deviation warning ──────────────────────────────────────────────
  // Two modes:
  //   Before direct: cross-track vs SID legs in the navlog (2 NM tolerance,
  //   2-minute cooldown). SID legs are dense and lateral offset is a
  //   meaningful indicator that the pilot is not following the SID.
  //   After direct: heading-vs-bearing to the assigned fix, checked
  //   starting 180 s after the "direct FIX" was issued. Real ATC does
  //   NOT measure cross-track from the origin — they look at whether the
  //   aircraft is on a converging course. A large lateral offset with
  //   correct heading is a normal parallel-intercept manoeuvre and
  //   should not warn (LIMF-KUKEV false-positive fix).
  s_sid_deviation_cooldown_sec =
      std::max(0.0f, s_sid_deviation_cooldown_sec - dt);
  if (s_sid_direct_issued)
    s_sid_direct_elapsed_sec += dt;

  if (s_sid_deviation_cooldown_sec <= 0.0f) {
    if (s_sid_direct_issued) {
      // Heading-vs-bearing check, 180 s post-direct.
      const std::string &direct_fix = ctx.ifr_sid_last_fix;
      if (!direct_fix.empty() && s_sid_direct_elapsed_sec >= 180.0f) {
        double fix_lat = 0.0, fix_lon = 0.0;
        bool have_fix = false;
        for (const auto &rf : s_route_fixes) {
          if (rf.ident == direct_fix) {
            fix_lat = rf.lat;
            fix_lon = rf.lon;
            have_fix = true;
            break;
          }
        }
        if (have_fix) {
          double brg = traffic_geometry::bearing_deg(ctx.latitude, ctx.longitude,
                                                    fix_lat, fix_lon);
          double diff = std::fabs(brg - ctx.heading_true);
          if (diff > 180.0) diff = 360.0 - diff;
          // 25 deg = 10 deg intercept + 10 deg wind correction + 5 deg slop.
          if (diff > 25.0) {
            s_sid_deviation_cooldown_sec = defaults.sid_deviation_cooldown_sec;
            if (out_text) {
              char buf[160];
              std::snprintf(buf, sizeof(buf),
                            "%s, confirm direct %s, "
                            "you appear tracking heading %.0f, expected %.0f.",
                            callsign.c_str(), direct_fix.c_str(),
                            static_cast<double>(ctx.heading_true), brg);
              *out_text = buf;
            }
            logging::info("IFR SID: heading %.0f vs bearing %.0f to %s "
                          "(diff %.0f, post-direct %.0f s)",
                          static_cast<double>(ctx.heading_true), brg,
                          direct_fix.c_str(), diff,
                          static_cast<double>(s_sid_direct_elapsed_sec));
            return true;
          }
        }
      }
    } else {
      // Pre-direct: original cross-track check vs SID legs.
      auto ofp = simbrief_ofp::get();
      if (ofp.valid && !ofp.navlog.empty()) {
        double xt_nm = procedure_deviation_nm(ctx, ofp.navlog,
                                              /*sid_star_only=*/true,
                                              /*direct_fix=*/std::string{},
                                              0.0, 0.0);
        if (xt_nm > 2.0 && xt_nm < 1e8) {
          s_sid_deviation_cooldown_sec = defaults.sid_deviation_cooldown_sec;
          if (out_text) {
            char buf[160];
            std::snprintf(buf, sizeof(buf),
                          "%s, confirm SID routing, you appear %.0f NM off track.",
                          callsign.c_str(), xt_nm);
            *out_text = buf;
          }
          logging::info("IFR SID: cross-track deviation %.1f NM (pre-direct)",
                        xt_nm);
          return true;
        }
      }
    }
  }

  return false;
}

// ── Helpers for poll_enroute ──────────────────────────────────────────────

// Canonical FL-vs-feet formatter — the single source of truth for every spoken
// altitude phrase (replaces three rival rules that previously diverged).
//   AltHint::Auto        -> Transition-Level threshold (altitudes with no CIFP
//                           is_fl flag: cruise, climb targets, maintain-current)
//   AltHint::Feet / FlightLevel -> force the format the CIFP constraint publishes
//                           (a plate "06500" is feet even above the TL, "FL080"
//                           is a level even below it — the old TL guess spoke
//                           LP403's 6500 ft as "flight level 65", LIMF->LFLP).
// QNH is appended only in feet form, never with a flight level.
enum class AltHint { Auto, Feet, FlightLevel };
static std::string format_alt_clearance(int alt_ft, AltHint hint,
                                        int qnh_hpa = 1013, int ta_ft = 5000) {
  bool as_fl;
  switch (hint) {
  case AltHint::Feet:        as_fl = false; break;
  case AltHint::FlightLevel: as_fl = true;  break;
  default:
    as_fl = (alt_ft >= compute_tl_ft(ta_ft > 0 ? ta_ft : 5000, qnh_hpa));
    break;
  }
  char buf[48];
  if (as_fl)
    std::snprintf(buf, sizeof(buf), "flight level %d", alt_ft / 100);
  else
    std::snprintf(buf, sizeof(buf), "%d feet, QNH %d", alt_ft, qnh_hpa);
  return buf;
}

// Convenience for the TL-threshold path (no CIFP is_fl available).
// Default args replicate the old 5000 ft threshold (neutral QNH, EU TA).
static std::string format_alt(int alt_ft, int ta_ft = 5000, int qnh_hpa = 1013) {
  return format_alt_clearance(alt_ft, AltHint::Auto, qnh_hpa, ta_ft);
}

// Pick the format hint for a CIFP-sourced constraint: honor its is_fl flag when
// it has a real altitude, else fall back to the TL threshold.
static AltHint alt_hint_for(const cifp_reader::CifpAlt &a) {
  if (a.feet <= 0)
    return AltHint::Auto;
  return a.is_fl ? AltHint::FlightLevel : AltHint::Feet;
}

// Cross-track distance (NM) from point P to the great-circle leg A→B.
// Positive = right of track, negative = left. Returns large value when
// the leg has zero length.
static double cross_track_nm(double lat_p, double lon_p, double lat_a,
                             double lon_a, double lat_b, double lon_b) {
  constexpr double kRnm = 3440.065; // Earth radius in NM
  double d_ap = traffic_geometry::distance_nm(lat_a, lon_a, lat_p, lon_p);
  if (d_ap < 0.001)
    return 0.0;
  double theta_ab = traffic_geometry::bearing_deg(lat_a, lon_a, lat_b, lon_b);
  double theta_ap = traffic_geometry::bearing_deg(lat_a, lon_a, lat_p, lon_p);
  double ang_diff = (theta_ap - theta_ab) * (3.14159265358979323846 / 180.0);
  double xt = std::asin(std::sin(d_ap / kRnm) * std::sin(ang_diff)) * kRnm;
  return xt;
}

// Find the minimum absolute cross-track error (NM) from the aircraft to any
// navlog leg. Returns a large value when the navlog is empty.
static double
min_cross_track_nm(const xplane_context::XPlaneContext &ctx,
                   const std::vector<simbrief_ofp::NavlogFix> &navlog) {
  if (navlog.size() < 2)
    return 1e9;
  double min_xt = 1e9;
  for (size_t i = 0; i + 1 < navlog.size(); ++i) {
    double xt =
        cross_track_nm(ctx.latitude, ctx.longitude, navlog[i].lat,
                       navlog[i].lon, navlog[i + 1].lat, navlog[i + 1].lon);
    if (std::abs(xt) < std::abs(min_xt))
      min_xt = xt;
  }
  return min_xt;
}

// Cross-track deviation for a procedure (SID or STAR).
// Two modes:
//   direct_fix empty  → check vs navlog legs where is_sid_star matches
//   sid_star_only. direct_fix set    → check vs a single leg (direct_from →
//   direct_fix position in navlog).
// Returns absolute deviation in NM, or 1e9 when no usable data.
static double
procedure_deviation_nm(const xplane_context::XPlaneContext &ctx,
                       const std::vector<simbrief_ofp::NavlogFix> &navlog,
                       bool sid_star_only, const std::string &direct_fix,
                       double direct_from_lat, double direct_from_lon) {

  if (direct_fix.empty()) {
    // Normal procedure legs: filter by SID/STAR flag.
    std::vector<simbrief_ofp::NavlogFix> legs;
    legs.reserve(navlog.size());
    for (const auto &f : navlog)
      if (f.is_sid_star == sid_star_only)
        legs.push_back(f);
    return std::abs(min_cross_track_nm(ctx, legs));
  }

  // Direct-to mode: find the target fix in the navlog.
  for (const auto &f : navlog) {
    if (f.ident != direct_fix)
      continue;
    double xt = cross_track_nm(ctx.latitude, ctx.longitude, direct_from_lat,
                               direct_from_lon, f.lat, f.lon);
    return std::abs(xt);
  }
  return 1e9; // fix not found
}

struct StarEntryResult {
  std::string ident;
  std::string star_name;  // empty if CIFP has no matching STAR
  double lat = 0.0;
  double lon = 0.0;
  int entry_alt_ft = 0;  // 0 = use defaults; set only for non-ceiling CIFP constraint
};

// Finds the STAR entry fix for the OFP destination.
// Per FPL convention: the last navlog fix before the destination ICAO is always
// the STAR entry (FPL filed as ROMAM...ABDIL...LFMN without explicit SID/STAR).
// Primary path uses is_sid_star flags (explicit STAR in SimBrief OFP).
// star_name may be empty when CIFP has no matching STAR (lat/lon still usable).
static bool find_star_entry(const std::string &cifp_dir,
                            const simbrief_ofp::OfpData &ofp,
                            StarEntryResult &out) {
  if (ofp.destination_icao.empty() || ofp.navlog.empty())
    return false;
  // Cache: OFP is static per flight — result never changes once computed.
  static bool cached = false, cache_valid = false;
  static std::string cache_dest;
  static StarEntryResult cache_result;
  if (cached && cache_dest == ofp.destination_icao) {
    if (cache_valid) out = cache_result;
    return cache_valid;
  }

  // Primary: track first fix of each is_sid_star group; last group = STAR.
  std::string star_entry_ident;
  bool in_group = false;
  for (const auto &fix : ofp.navlog) {
    if (fix.is_sid_star && !in_group &&
        !fix.ident.empty() && fix.ident != ofp.destination_icao)
      star_entry_ident = fix.ident;
    in_group = fix.is_sid_star;
  }

  // Fallback: last navlog fix before destination (standard FPL convention).
  if (star_entry_ident.empty()) {
    for (int i = (int)ofp.navlog.size() - 1; i >= 0; --i) {
      const auto &fix = ofp.navlog[i];
      if (!fix.ident.empty() && fix.ident != ofp.destination_icao) {
        star_entry_ident = fix.ident;
        break;
      }
    }
  }
  if (star_entry_ident.empty())
    return false;

  logging::debug("[DBG] find_star_entry: dest=%s entry_fix=%s cifp=%s",
                 ofp.destination_icao.c_str(), star_entry_ident.c_str(),
                 cifp_dir.empty() ? "(empty)" : "ok");

  // Locate lat/lon from navlog.
  for (const auto &fix : ofp.navlog) {
    if (fix.ident != star_entry_ident)
      continue;
    out.ident = star_entry_ident;
    out.lat   = fix.lat;
    out.lon   = fix.lon;
    // Optional CIFP STAR name + altitude constraint.
    if (!cifp_dir.empty()) {
      out.star_name = cifp_reader::star_name_for_entry_fix(
          cifp_dir, ofp.destination_icao, "", star_entry_ident);
      if (!out.star_name.empty()) {
        auto entry = cifp_reader::star_entry_fix(
            cifp_dir, ofp.destination_icao, out.star_name);
        if (entry.alt.feet > 0 && !entry.is_ceiling)
          out.entry_alt_ft = entry.alt.feet;
      }
    }
    logging::debug("[DBG] find_star_entry: star_name=%s entry_alt_ft=%d",
                   out.star_name.empty() ? "(none)" : out.star_name.c_str(),
                   out.entry_alt_ft);
    cached = true; cache_valid = true;
    cache_dest = ofp.destination_icao; cache_result = out;
    return true;
  }
  cached = true; cache_valid = false; cache_dest = ofp.destination_icao;
  return false;
}

static bool is_pseudo_fix(const std::string &ident); // defined in poll_enroute

// Build Phase 3 descent clearance: STAR entry altitude + STAR name + expected
// approach type.  Does NOT issue the Approach frequency handoff — that comes
// later via build_approach_handoff() when the aircraft reaches the CTA boundary.
// Sets s_enroute_descent_issued and transitions to IFR_DESCENT.
// Returns false when already issued.
static bool build_descent_clearance(const xplane_context::XPlaneContext &ctx,
                                    const std::string &callsign,
                                    const flight_phase::IfrDefaults &defaults,
                                    std::string *out_text) {
  if (s_enroute_descent_issued)
    return false;

  auto ofp = simbrief_ofp::get();

  // ── 1. Descent altitude ───────────────────────────────────────────────
  // Baseline: the higher of the profile default (FL110) and cruise-5000
  // (so an FL195 turboprop gets FL140, not FL110 — a more realistic first step).
  // At-or-below ceilings (e.g. ABDIL <= FL190) are upper bounds, not targets.
  // Exact / at-or-above CIFP constraints override when they are below cruise.
  // Use the OFP cruise FL so navlog step-downs don't undercut the approach
  // entry altitude.  Fall back to the last cleared level only when no cruise
  // FL was filed (e.g. training / debug scenarios).
  int cruise_ref_dc = ctx.ifr_cruise_alt_ft > 0 ? ctx.ifr_cruise_alt_ft
                      : (s_enroute_cleared_alt_ft > 0 ? s_enroute_cleared_alt_ft : 0);
  // Initial STAR-entry descent target when CIFP has no crossing constraint
  // at the entry fix. Proportional (cruise * 0.66) rather than a fixed
  // -5000 offset: a fixed subtraction barely descends a high-cruise jet
  // (FL350 -> FL300), whereas the fraction scales — FL220 -> FL140 (the
  // typical real STAR entry), FL350 -> FL230. Still a heuristic; the
  // correct fix is a STAR-waypoint lookahead to the first hard constraint
  // (e.g. LUVOB FL090 on LFLP SALE3P), which is the P0 v4.4.0 item
  // [[project_star_entry_alt_heuristic]]. Rounded to whole thousands.
  int star_alt_ft = std::max(defaults.star_entry_alt_ft,
                             (cruise_ref_dc * 66 / 100) / 1000 * 1000);
  // Never issue a descent TO an altitude above (or equal to) the current
  // cleared level — that would be a climb instruction disguised as a descent.
  // Cap at one FL below cruise (e.g. FL090 cruise → max target FL080).
  if (star_alt_ft >= cruise_ref_dc)
    star_alt_ft = (cruise_ref_dc / 1000 - 1) * 1000;
  std::string star_name;
  std::string star_entry_fix; // full naming-fix name for spoken designator (ABDIL)
  std::string dest_runway;

  {
    StarEntryResult se;
    if (find_star_entry(ctx.cifp_dir, ofp, se)) {
      star_name = se.star_name;
      star_entry_fix = se.ident;
      s_assigned_star_entry_fix = se.ident; // for engine::assigned_star_spoken()
      if (se.entry_alt_ft > 0 && se.entry_alt_ft < cruise_ref_dc)
        star_alt_ft = se.entry_alt_ft;
      if (!star_name.empty())
        dest_runway = cifp_reader::runway_for_star(
            ctx.cifp_dir, ofp.destination_icao, star_name);
    }
  }

  // ── 2. Expected approach type ─────────────────────────────────────────
  // When STAR serves ALL runways, dest_runway is empty — pick the best runway
  // using wind alignment and L-over-R preference.
  if (dest_runway.empty() && !ctx.cifp_dir.empty() && !ofp.destination_icao.empty())
    dest_runway = pick_arrival_runway(ctx, ofp.destination_icao);

  std::string approach_phrase;
  if (!dest_runway.empty() && !ctx.cifp_dir.empty() &&
      !ofp.destination_icao.empty()) {
    cifp_reader::ApproachInfo appr;
    if (!ofp.preferred_approach_designator.empty())
      appr = cifp_reader::approach_by_designator(ctx.cifp_dir, ofp.destination_icao,
                                                 ofp.preferred_approach_designator);
    if (appr.type_str.empty()) {
      const std::string pref = airport_overrides::preferred_approach(
          ofp.destination_icao, dest_runway, approach_gate_vis_m(ctx),
          approach_gate_ceiling_ft(ctx));
      if (!pref.empty())
        appr = cifp_reader::approach_by_designator(ctx.cifp_dir,
                                                   ofp.destination_icao, pref);
    }
    if (appr.type_str.empty())
      appr = cifp_reader::best_approach(ctx.cifp_dir, ofp.destination_icao,
                                        dest_runway, approach_gate_vis_m(ctx));
    if (!appr.type_str.empty()) {
      // Variant letter via cifp_reader::approach_suffix — safely handles
      // both "R04LZ" and dash-form "R04-Y" (LFLP-style).  Emitted as
      // full NATO word for TTS clarity: "expect RNAV Zulu approach ...".
      static const char *nato[] = {
          "Alpha","Bravo","Charlie","Delta","Echo","Foxtrot","Golf",
          "Hotel","India","Juliet","Kilo","Lima","Mike","November",
          "Oscar","Papa","Quebec","Romeo","Sierra","Tango","Uniform",
          "Victor","Whiskey","X-ray","Yankee","Zulu"};
      char suf = cifp_reader::approach_suffix(appr.designator);
      std::string variant_word;
      if (suf) {
        int idx = std::toupper(static_cast<unsigned char>(suf)) - 'A';
        if (idx >= 0 && idx < 26)
          variant_word = std::string(" ") + nato[idx];
      }
      approach_phrase = ", expect " + appr.type_str + variant_word +
                        " approach runway " + appr.runway;
      s_assigned_approach_designator = appr.designator;
      // Transcript note when the pilot gets a NON-preferred approach because the
      // destination weather ruled out the airport+.json first choice (e.g. LFMN
      // 04L: RNAV Alpha needs >=10 km & >=2500 ft, else Zulu). Only when the
      // choice came from the weather gate (not a pilot-filed approach): compare
      // the selected designator against the ideal (same gate under perfect
      // weather). (user 2026-07-19: "log when another app is selected instead of
      // preferred".) Log.txt already carries the reason via airport_overrides.
      if (ofp.preferred_approach_designator.empty()) {
        const std::string ideal = airport_overrides::preferred_approach(
            ofp.destination_icao, dest_runway, 1.0e9f, 1.0e9f);
        if (!ideal.empty() && ideal != appr.designator) {
          char note[200];
          std::snprintf(note, sizeof(note),
                        "Note: preferred approach %s unavailable at %s (dest METAR "
                        "vis %.0f m, ceiling %.0f ft) -- %s in use",
                        ideal.c_str(), ofp.destination_icao.c_str(),
                        approach_gate_vis_m(ctx), approach_gate_ceiling_ft(ctx),
                        appr.designator.c_str());
          s_pending_transcript_note = note;
        }
      }
      // Lock the ARRIVAL runway now, at the "expect approach runway NN"
      // briefing -- not at the (much later) approach check-in. Until this,
      // assigned_runway() still held the DEPARTURE runway set on the ground at
      // the origin (e.g. 09), so the STT context bias injected "runway 09 /
      // R-NAV 09 / RNAV 09" throughout the descent and actively pushed Voxtral
      // toward the wrong runway while ATC was briefing runway 04 (LFLP)
      // (LIMx->LFLP 2026-07-11). set_assigned_runway also feeds build_vars
      // {runway} and s_assigned_landing_runway keeps Tower consistent.
      atc_state_machine::set_assigned_runway(appr.runway);
      s_assigned_landing_runway = appr.runway;
    }
  }

  // ── 2b. CIFP fallback: no STAR from navlog → use first STAR for runway ──
  // When the pilot's FPL doesn't include a STAR entry fix (e.g. filed without
  // ABDIL), navlog lookup returns no star_name.  Fall back to the alphabetically
  // first STAR that serves the active destination runway from CIFP data alone.
  if (star_name.empty() && !dest_runway.empty() && !ctx.cifp_dir.empty() &&
      !ofp.destination_icao.empty()) {
    star_name = cifp_reader::first_star_for_runway(ctx.cifp_dir, ofp.destination_icao,
                                                   dest_runway);
    if (!star_name.empty()) {
      auto entry = cifp_reader::star_entry_fix(ctx.cifp_dir, ofp.destination_icao, star_name);
      if (entry.alt.feet > 0 && !entry.is_ceiling && entry.alt.feet < cruise_ref_dc)
        star_alt_ft = entry.alt.feet;
    }
  }

  // ── 2b-P0: STAR-lookahead — clamp the initial descent, don't collapse ──
  // The initial descent target is the proportional cruise*0.66 intermediate
  // computed above (FL220 -> FL140). We do NOT pull it all the way down to
  // the first STAR ceiling: that would collapse a naturally staged descent
  // (FL220 -> FL140 -> FL090 -> 6000) into one 13000 ft clearance and drop
  // the realistic intermediate step. The poll_approach walker (P0-B, now
  // fires reliably on every hard crossing constraint) issues the FL090 step
  // at LUVOB and the 6000 step at PIRUV as the aircraft reaches them.
  //
  // The lookahead only matters as a SAFETY CLAMP: if the cruise*0.66
  // intermediate happens to sit BELOW the first at-or-below constraint
  // there's nothing to do (a ceiling is a max, being under it is fine);
  // but if it sits ABOVE the FINAL/lowest STAR floor we must not clear
  // above where the STAR ultimately wants us. In practice cruise*0.66 is
  // always a valid intermediate, so this block now only logs the first
  // constraint for diagnostics and leaves star_alt_ft at the intermediate.
  // See [[project_star_entry_alt_heuristic]].
  if (!star_name.empty() && !ctx.cifp_dir.empty() &&
      !ofp.destination_icao.empty()) {
    auto star_wps = cifp_reader::star_waypoints(ctx.cifp_dir,
                                                ofp.destination_icao, star_name);
    for (const auto &w : star_wps) {
      if (w.is_ceiling && w.alt.feet > 0) {
        logging::info("[approach] STAR first at-or-below %d ft at %s on %s; "
                      "initial descent kept at intermediate %d ft "
                      "(walker steps down to constraints)",
                      w.alt.feet, w.ident.c_str(), star_name.c_str(),
                      star_alt_ft);
        break;
      }
    }
  }

  // ── 2c. No-STAR direct-to IAF ─────────────────────────────────────────
  // Two conditions allow issuing a direct:
  //   A) Aircraft has passed the last non-STAR FPL fix (bearing check).
  //   B) Last FPL fix is closer to the destination than the aircraft is to
  //      the nearest IAF — the fix is "inside" the approach environment and
  //      routing through it before the IAF would be backwards geometry.
  //      Handles REQUEST_DESCENT issued before the aircraft reaches the fix.
  std::string direct_iaf;
  if (star_name.empty() && !s_assigned_approach_designator.empty() &&
      !ctx.cifp_dir.empty() && !ofp.destination_icao.empty()) {
    // Find last non-STAR/non-pseudo FPL fix.
    double last_lat = 0.0, last_lon = 0.0;
    bool last_fix_found = false;
    for (int i = static_cast<int>(ofp.navlog.size()) - 1; i >= 0; --i) {
      const auto &nf = ofp.navlog[i];
      if (!nf.is_sid_star && !nf.ident.empty() && !is_pseudo_fix(nf.ident)) {
        last_lat = nf.lat; last_lon = nf.lon;
        last_fix_found = true;
        break;
      }
    }

    // Get IAF idents first so we can compute dist(aircraft → IAF) before
    // applying the bearing check.
    auto iaf_idents = cifp_reader::approach_transition_idents(
        ctx.cifp_dir, ofp.destination_icao, s_assigned_approach_designator);

    if (!iaf_idents.empty()) {
      // Select best IAF and record distance from aircraft to that IAF.
      std::string best_iaf;
      double dist_aircraft_to_iaf = 1e9;

      if (iaf_idents.size() == 1) {
        best_iaf = iaf_idents[0];
        // Position likely absent from earth_fix.dat (terminal fix) — use
        // distance to the destination as a conservative proxy.
        dist_aircraft_to_iaf = traffic_geometry::distance_nm(
            ctx.latitude, ctx.longitude, ctx.airport_lat, ctx.airport_lon);
      } else {
        auto iaf_pos = cifp_reader::lookup_fix_positions(
            ctx.cifp_dir, iaf_idents, ofp.destination_icao);
        double best_dist_ac = 1e9;
        for (const auto &id : iaf_idents) {
          auto it = iaf_pos.find(id);
          if (it == iaf_pos.end()) continue;
          // Pick the IAF closest to the aircraft — most relevant for a direct clearance.
          double d_ac = traffic_geometry::distance_nm(
              ctx.latitude, ctx.longitude, it->second.first, it->second.second);
          if (d_ac < best_dist_ac) {
            best_dist_ac = d_ac;
            best_iaf = id;
            dist_aircraft_to_iaf = d_ac;
          }
        }
        if (best_iaf.empty()) {
          best_iaf = iaf_idents[0];
          dist_aircraft_to_iaf = traffic_geometry::distance_nm(
              ctx.latitude, ctx.longitude, ctx.airport_lat, ctx.airport_lon);
          logging::info("IFR descent (no STAR): IAF pos lookup failed, using first: %s",
                        best_iaf.c_str());
        }
      }

      // Condition A: aircraft has passed the last FPL fix (bearing / proximity).
      bool past_last_fix = false;
      if (last_fix_found) {
        double dist_to_last = traffic_geometry::distance_nm(
            ctx.latitude, ctx.longitude, last_lat, last_lon);
        if (dist_to_last < 3.0) {
          past_last_fix = true;
        } else {
          double dlat = last_lat - ctx.latitude;
          double dlon = (last_lon - ctx.longitude) *
                        std::cos(ctx.latitude * M_PI / 180.0);
          double bdeg = std::atan2(dlon, dlat) * 180.0 / M_PI;
          double diff = std::abs(bdeg - static_cast<double>(ctx.heading_true));
          if (diff > 180.0) diff = 360.0 - diff;
          past_last_fix = (diff > 90.0);
        }
      }

      // Condition B: last FPL fix is closer to destination than the aircraft
      // is to the IAF — the fix is inside the approach area, so a direct to
      // the IAF is safe and makes sense geometrically.
      bool fix_inside_approach = false;
      if (last_fix_found && dist_aircraft_to_iaf < 1e8) {
        double dist_last_to_dest = traffic_geometry::distance_nm(
            last_lat, last_lon, ctx.airport_lat, ctx.airport_lon);
        fix_inside_approach = (dist_last_to_dest < dist_aircraft_to_iaf);
      }

      // Condition C: within 20 NM of the IAF, issue direct regardless of
      // FPL fix geometry. Prevents the fix-ahead check from suppressing the
      // IAF clearance when the aircraft is already in the approach environment.
      const bool within_iaf_gate = (dist_aircraft_to_iaf <= 20.0);

      if (past_last_fix || fix_inside_approach || within_iaf_gate) {
        direct_iaf = best_iaf;
        s_no_star_direct_iaf = best_iaf;
        logging::info(
            "IFR descent (no STAR): direct IAF=%s (past_fix=%d inside=%d "
            "within20=%d dist_iaf=%.0f NM)",
            direct_iaf.c_str(), past_last_fix ? 1 : 0,
            fix_inside_approach ? 1 : 0, within_iaf_gate ? 1 : 0,
            dist_aircraft_to_iaf);
      } else {
        logging::info(
            "IFR descent (no STAR): last FPL fix still ahead and outside "
            "approach, not within 20 NM (dist_iaf=%.0f NM) -- no direct IAF",
            dist_aircraft_to_iaf);
      }
    }
  }

  // ── 3. STAR phrase ────────────────────────────────────────────────────
  // For direct-to IAF, look up the IAF floor altitude from CIFP (e.g. 2700 ft at QA503)
  // so the clearance reads "direct QA503, descend 2700 feet" rather than altitude-less.
  int direct_iaf_alt_ft = 0;
  if (!direct_iaf.empty() && !ctx.cifp_dir.empty() && !ofp.destination_icao.empty()) {
    auto iaf_wpts = cifp_reader::approach_procedure_waypoints(
        ctx.cifp_dir, ofp.destination_icao, s_assigned_approach_designator, direct_iaf);
    if (!iaf_wpts.empty() && iaf_wpts[0].alt.feet > 0)
      direct_iaf_alt_ft = iaf_wpts[0].alt.feet;
  }
  std::string star_phrase;
  // EUROCONTROL / DGAC: QNH need not be repeated if it was already transmitted
  // in this sector (e.g. a navlog step-down clearance already said "descend
  // 4500 feet, QNH 1025" moments earlier).  s_qnh_stated tracks this.
  // The suppression applies only to the spoken text; logging always includes QNH.
  const bool qnh_omit = s_qnh_stated;

  if (!star_name.empty())
    star_phrase = ", cleared via " +
                  spoken_procedure_name(star_name, star_entry_fix) + " arrival";
  else if (!direct_iaf.empty()) {
    int current_cleared_dc = s_enroute_cleared_alt_ft > 0 ? s_enroute_cleared_alt_ft
                                                           : ctx.ifr_cruise_alt_ft;
    if (direct_iaf_alt_ft > 0 && direct_iaf_alt_ft < current_cleared_dc) {
      // Build spoken altitude: omit QNH if already stated earlier this sector.
      const int tl_dc = compute_tl_ft(
          ctx.transition_alt_ft > 0 ? ctx.transition_alt_ft : 5000, ctx.qnh_hpa);
      char iaf_alt_str[32];
      if (direct_iaf_alt_ft >= tl_dc)
        std::snprintf(iaf_alt_str, sizeof(iaf_alt_str),
                      "flight level %d", direct_iaf_alt_ft / 100);
      else if (qnh_omit)
        std::snprintf(iaf_alt_str, sizeof(iaf_alt_str),
                      "%d feet", direct_iaf_alt_ft);
      else
        std::snprintf(iaf_alt_str, sizeof(iaf_alt_str),
                      "%d feet, QNH %d", direct_iaf_alt_ft, ctx.qnh_hpa);
      star_phrase = std::string(", direct ") + direct_iaf + ", descend " + iaf_alt_str;
    } else {
      star_phrase = ", direct " + direct_iaf;
    }
  }

  // ── 4. Commit ─────────────────────────────────────────────────────────
  s_enroute_descent_issued = true;
  atc_state_machine::set_state(atc_state_machine::ATCState::IFR_DESCENT);
  // Store for poll_approach() to load STAR waypoints at Approach check-in.
  s_assigned_star_name = star_name;
  s_assigned_dest_icao = ofp.destination_icao;

  // Decide whether to SPEAK a "descend" step: only when star_alt_ft is a genuine
  // step below the PRIOR cleared level (or the prior level is unknown -- an
  // external jump). Computed against the prior value BEFORE recording the new
  // one, otherwise emit_alt always sees its own write and the spoken descent
  // vanishes (alpha-11 regression: "cleared via ABDI8R" with no "descend FL120").
  const int prior_cleared = s_enroute_cleared_alt_ft > 0 ? s_enroute_cleared_alt_ft
                                                         : ctx.ifr_cruise_alt_ft;
  const bool emit_alt =
      star_alt_ft > 0 && (prior_cleared <= 0 || star_alt_ft < prior_cleared);

  // Record the cleared altitude so current_cleared_alt_ft() (and EVERY downstream
  // altitude gate -- descend-to-enter, poll_altitude_compliance, the CIFP crossing
  // corrective) reflects it. build_descent_clearance previously only READ this,
  // never wrote it -- so on an external jump it stayed 0, current_cleared_alt_ft()
  // returned 0, and SILENTLY disabled all descent enforcement (LFMN ABDI8R
  // 2026-07-13). Seed to cruise when no step is spoken so it is never left at 0.
  if (emit_alt)
    s_enroute_cleared_alt_ft = star_alt_ft;
  else if (s_enroute_cleared_alt_ft <= 0 && ctx.ifr_cruise_alt_ft > 0)
    s_enroute_cleared_alt_ft = ctx.ifr_cruise_alt_ft;
  logging::info("IFR descent: cleared alt recorded = %d ft (star=%d emit_alt=%d "
                "prior=%d cruise=%d)",
                s_enroute_cleared_alt_ft, star_alt_ft, emit_alt ? 1 : 0,
                prior_cleared, ctx.ifr_cruise_alt_ft);

  // Load the STAR + approach fix chain into the route table NOW, at the descent
  // clearance -- not only at the Approach check-in. ATC has already named both
  // the STAR (s_assigned_star_name) and the expected approach
  // (s_assigned_approach_designator, set above), so the full arrival sequence
  // (e.g. GIROL/AMFOU/TIPIK/MUS + the RNAV 22L fixes on LFMN ABDI8R) is known.
  // Without this the route table jumps STAR-entry -> destination during the
  // whole descent, so "direct <STAR fix>" readbacks are not biased (AMFOU ->
  // "I'm full", 2026-07-13), check_course compares against the airport, and the
  // STAR block-floor clamp has no floors to read. init_route_fixes clears +
  // rebuilds and re-syncs the tracker to the aircraft position (idempotent --
  // the Approach check-in still calls it again to fold in any late changes).
  init_route_fixes(ctx);

  if (out_text) {
    char buf[240];
    // emit_alt was decided above (against the PRIOR cleared level). When false,
    // navlog step-downs already had us below the STAR entry altitude, so give
    // routing + approach info only.
    std::string routing = star_phrase + approach_phrase;
    if (!emit_alt && !routing.empty()) {
      if (routing.size() >= 2 && routing[0] == ',')
        routing = routing.substr(2); // strip leading ", "
      std::snprintf(buf, sizeof(buf), "%s, %s.", callsign.c_str(), routing.c_str());
    } else {
      // Build spoken altitude: omit QNH if already stated earlier this sector.
      const int tl_s = compute_tl_ft(
          ctx.transition_alt_ft > 0 ? ctx.transition_alt_ft : 5000, ctx.qnh_hpa);
      char alt_str[32];
      if (star_alt_ft >= tl_s)
        std::snprintf(alt_str, sizeof(alt_str), "flight level %d", star_alt_ft / 100);
      else if (qnh_omit)
        std::snprintf(alt_str, sizeof(alt_str), "%d feet", star_alt_ft);
      else
        std::snprintf(alt_str, sizeof(alt_str), "%d feet, QNH %d", star_alt_ft, ctx.qnh_hpa);
      std::snprintf(buf, sizeof(buf), "%s, descend %s%s%s.",
                    callsign.c_str(), alt_str,
                    star_phrase.c_str(), approach_phrase.c_str());
    }
    *out_text = buf;
  }
  logging::info("IFR en-route: descent -> %s, STAR=%s, rwy=%s",
                format_alt(star_alt_ft, ctx.transition_alt_ft, ctx.qnh_hpa).c_str(),
                star_name.empty() ? "(none)" : star_name.c_str(),
                dest_runway.empty() ? "(none)" : dest_runway.c_str());
  return true;
}

// Issue the Approach frequency handoff ("contact Nice Approach on X.XXX").
// Called when the aircraft crosses the CTA/TMA boundary during descent.
// Sets s_enroute_approach_handoff_issued and transitions to IFR_APPROACH_CONTACT.
// Returns false when already issued.
// enc: the openair airspace the aircraft just entered (TMA/CTR), or an OTHER
// entry when the 30 NM distance fallback fired (no openair TMA data).
// Resolve an openair TMA/CTA name (e.g. "CHAMBERY TMA SECTOR 2") to its terminal
// approach controller + frequency from atc.dat. Order: TRACON whose name matches
// the fragment (GENEVA, NICE); else the field's tower's FACILITY -> that
// facility's TRACON (CHAMBERY -> facility LFLB -> the LFLB tracon labelled "LYON",
// 123.70) keeping the spoken label as the field ("Chambery Approach", not "Lyon");
// else the tower itself. Returns false if nothing resolves. Shared by the arrival
// handoff and the boundary sector-change so both resolve TMAs identically.
// Core terminal resolution: an openair TMA/CTA name -> the atc.dat controller to
// TUNE (freq source, returned) + the controller to NAME (*out_label_ctrl). Strips
// the name to a city fragment, finds the TRACON, else follows the TWR->facility link
// to that field's TRACON, else the tower itself. Returns nullptr if unresolved.
// SHARED so resolve_tma_controller (label+freq) and poll_approach's forward handoff
// (which needs the Controller* for its visited-guard/ceiling state) use ONE copy.
static const airspace_db::Controller *
resolve_terminal_ctrl(const std::string &name,
                      const airspace_db::Controller **out_label_ctrl) {
  std::string fragment = name;
  for (const char *kw : {"TMA", "CTA", "FIR", "UIR", " SECTOR", " SEC"}) {
    auto pos = fragment.find(kw);
    if (pos != std::string::npos) {
      fragment = fragment.substr(0, pos);
      break;
    }
  }
  while (!fragment.empty() && fragment.back() == ' ')
    fragment.pop_back();
  if (fragment.empty())
    return nullptr;
  const airspace_db::Controller *ctrl = airspace_db::find_by_role_name_contains(
      airspace_db::ControllerRole::TRACON, fragment);
  const airspace_db::Controller *label_ctrl = ctrl;
  if (!ctrl || ctrl->freqs_khz.empty()) {
    const airspace_db::Controller *named = airspace_db::find_by_role_name_contains(
        airspace_db::ControllerRole::TWR, fragment);
    if (named && !named->facility_id.empty()) {
      const airspace_db::Controller *fac = airspace_db::find_by_role_facility(
          airspace_db::ControllerRole::TRACON, named->facility_id);
      if (fac && !fac->freqs_khz.empty()) {
        ctrl = fac;
        label_ctrl = named;
      }
    }
    if ((!ctrl || ctrl->freqs_khz.empty()) && named && !named->freqs_khz.empty()) {
      ctrl = named;
      label_ctrl = named;
    }
  }
  if (!ctrl || ctrl->freqs_khz.empty())
    return nullptr;
  if (out_label_ctrl)
    *out_label_ctrl = label_ctrl ? label_ctrl : ctrl;
  return ctrl;
}

static bool resolve_tma_controller(const std::string &tma_name,
                                   std::string *out_label, float *out_freq_mhz) {
  const airspace_db::Controller *label_ctrl = nullptr;
  const airspace_db::Controller *ctrl =
      resolve_terminal_ctrl(tma_name, &label_ctrl);
  if (!ctrl)
    return false;
  if (out_freq_mhz)
    *out_freq_mhz = static_cast<float>(ctrl->freqs_khz.front()) / 1000.0f;
  if (out_label)
    *out_label = controller_label_for(label_ctrl) + " Approach";
  return true;
}

// Delegation markers embedded in an openair polygon NAME. The vendor file marks
// cross-border operational delegation in the name rather than as a separate
// controller; map the marker to the ICAO of the ACC that actually works the
// airspace (Navigraph 2026-07-07: "... SKYGUIDE ..." over the LFFF->LSAS
// Mont-Blanc corridor; "... MUAC ..." = Maastricht UAC). Returns the delegated
// org ICAO, or "" when the name carries no delegation marker.
static std::string delegated_org(const std::string &name) {
  if (name.find("SKYGUIDE") != std::string::npos)
    return "LSAS";
  if (name.find("MUAC") != std::string::npos)
    return "EDYY";
  static const std::regex kDel(R"(DELEGATED BY \w+ TO (\w+))");
  std::smatch m;
  if (std::regex_search(name, m, kDel))
    return m[1].str();
  return {};
}

// Radio callsign for a delegated ACC. atc.dat NAMEs the Swiss centre "SWITZERLAND",
// but its real/IVAO radio callsign for ALL positions (Zurich FIR + Geneva FIR) is
// "Swiss Radar" (IVAO Switzerland Div Swiss Radar OM, 2018: "for all positions,
// Swiss Radar"). This corrects only the SPOKEN callsign -- the frequency still comes
// from atc.dat/Navigraph. Empty -> fall back to controller_label_for.
static std::string delegated_callsign(const std::string &org) {
  if (org == "LSAS")
    return "Swiss Radar";
  if (org == "EDYY")
    return "Maastricht"; // Maastricht UAC (EUROCONTROL MUAC)
  return {};
}

// Enroute analog of resolve_tma_controller for CROSS-BORDER DELEGATION polygons:
// map a delegation-marked openair polygon NAME to the atc.dat ACC (CTR) that
// actually works it, by the delegated org ICAO (SKYGUIDE/DELEGATED -> LSAS ->
// atc.dat "SWITZERLAND" CTR 119.175). openair supplies the fine GEOMETRY + the
// delegation marker; atc.dat supplies the FREQUENCY.
//
// SCOPE (step 1, 2026-07-18): ONLY delegation-marked polygons resolve here. A
// non-delegated base CTA/FIR returns false so the caller falls back to the
// UNCHANGED atc.dat geometric CTR picker -- avoids fragile name-substring
// matching (e.g. "FRANCE" would hit "FRANCEVILLE") and guarantees zero enroute
// regression. Named sub-CTA resolution (Milan BRERA/LIGURIA) is the next
// increment, once those overlay polygons exist and can be validated.
static bool resolve_acc_controller(const std::string &name,
                                   std::string *out_label, float *out_mhz) {
  const std::string org = delegated_org(name);
  if (org.empty())
    return false;
  const airspace_db::Controller *c =
      airspace_db::find_by_role_facility(airspace_db::ControllerRole::CTR, org);
  if (!c || c->freqs_khz.empty())
    c = airspace_db::find_by_role_name_contains(
        airspace_db::ControllerRole::CTR, org);
  if (!c || c->freqs_khz.empty())
    return false;
  if (out_mhz)
    *out_mhz = static_cast<float>(c->freqs_khz.front()) / 1000.0f;
  if (out_label) {
    const std::string cs = delegated_callsign(org);
    *out_label = cs.empty() ? controller_label_for(c) : cs;
  }
  return true;
}

// ONE resolver for every handoff phase (SID / ENROUTE / STAR): given the openair
// airspace the aircraft is in, return the controller label + frequency. The
// class picks the atc.dat role -- a terminal TMA/CTR resolves to the field's
// TRACON ("... Approach", via resolve_tma_controller); an enroute CTA/FIR/UIR
// resolves to the ACC (via resolve_acc_controller). Callers keep their own
// trigger geometry; only the who-do-I-contact decision is centralised here.
// `terminal` = this is a TERMINAL handoff (SID departure / STAR arrival), where a
// CTA is part of the terminal control area and resolves to the field's TRACON like a
// TMA. En-route (terminal=false) a CTA is an ACC/delegation volume. This one bool is
// the only phase-context the resolver needs; everything else (which fix/geometry
// triggers, the phraseology) stays in the caller.
static bool resolve_sector_controller(const openair_db::AirspaceEntry &enc,
                                      bool terminal, std::string *out_label,
                                      float *out_mhz) {
  switch (enc.ac_class) {
  case openair_db::AirspaceClass::TMA:
  case openair_db::AirspaceClass::CTR:
    return resolve_tma_controller(enc.name, out_label, out_mhz);
  case openair_db::AirspaceClass::CTA:
    return terminal ? resolve_tma_controller(enc.name, out_label, out_mhz)
                    : resolve_acc_controller(enc.name, out_label, out_mhz);
  case openair_db::AirspaceClass::FIR:
  case openair_db::AirspaceClass::UIR:
    return resolve_acc_controller(enc.name, out_label, out_mhz);
  default:
    return false;
  }
}

// Option B (LFLP 2026-07-15): only the TERMINAL controller -- the unit that owns
// the destination's approach -- may issue the approach clearance, never an outer
// TMA (Geneva has no authority over the LFLP RNAV choice). The terminal TMA is the
// innermost TMA enclosing the DESTINATION; the clearance is allowed only once the
// aircraft is in that SAME terminal TMA (i.e. on Chambery, after the handoff).
// Returns true when openair / destination data is unavailable so it never blocks a
// field that has no nested TMAs (single-TMA arrivals like LFMN, AFIS fields).
static bool on_destination_terminal(const xplane_context::XPlaneContext &ctx) {
  if (!openair_db::ready() || s_assigned_dest_icao.empty())
    return true;
  const auto dpos = xplane_context::airport_pos_for(s_assigned_dest_icao);
  if (dpos.first == 0.0 && dpos.second == 0.0)
    return true;
  // Probe the destination just UNDER its base terminal-TMA ceiling so we get the
  // approach TMA, NOT the field's low CTR. A low probe (3000 ft) returned ANNECY
  // CTR (0-4000) as the innermost over LFLP -- its fragment "ANNECY" != the
  // approach TMA "CHAMBERY", which false-blocked the clearance in-sim (LFLP
  // 2026-07-17). terminal_tma_ceiling() is TMA-class only (skips the CTR).
  const int tma_ceil = openair_db::terminal_tma_ceiling(dpos.first, dpos.second);
  const int probe = (tma_ceil > 1500) ? tma_ceil - 500 : 3000;
  const openair_db::AirspaceEntry dest_tma =
      openair_db::find_enclosing(dpos.first, dpos.second, probe);
  const openair_db::AirspaceEntry acft_tma = openair_db::find_enclosing(
      ctx.latitude, ctx.longitude, openair_alt(ctx));
  if (dest_tma.name.empty() || acft_tma.name.empty())
    return true; // can't tell -> don't block
  // Compare CONTROLLER FRAGMENTS (strip TMA/CTA/CTR/FIR/UIR/SECTOR + trailing) so a
  // different sub-sector -- or CTR-vs-TMA -- over the SAME field still counts as
  // on-terminal: "CHAMBERY TMA SECTOR 2" and "CHAMBERY CTR" both -> "CHAMBERY".
  // Exact-name + freq-resolve was too fragile and FALSE-BLOCKED the approach
  // clearance in-sim (LFLP 2026-07-17: the aircraft was in CHAMBERY TMA SECTOR 2
  // but the dest probe at 3000 ft resolved a different Chambery volume, and
  // resolve_tma_controller couldn't map "... CTR" -> the clearance never fired).
  // Still blocks Geneva ("GENEVA" != "CHAMBERY") so Option B holds.
  auto frag = [](std::string n) {
    for (const char *kw : {" TMA", " CTA", " CTR", " FIR", " UIR", " SECTOR", " SEC"}) {
      auto p = n.find(kw);
      if (p != std::string::npos) { n = n.substr(0, p); break; }
    }
    while (!n.empty() && n.back() == ' ')
      n.pop_back();
    return n;
  };
  return frag(acft_tma.name) == frag(dest_tma.name);
}

// ICAO Doc 4444 / EUROCONTROL coordination: a controller may descend an aircraft
// only within its OWN area of responsibility. Over a nested/stacked TMA the
// effective floor is the ceiling of the inner TMA sitting beneath the aircraft
// when that inner TMA is owned by a DIFFERENT unit -- everything below that
// ceiling belongs to the inner controller, issued only after the handoff.
// Returns that inner ceiling (ft) when a descent to target_ft would enter a
// lower, differently-controlled TMA; 0 when there is no such boundary (same
// controlling unit, no inner TMA, or outside all airspace / ACC). Guards Geneva
// from clearing "descend 6500" into the Chambery TMA (LFLP 2026-07-15).
static int sector_transfer_floor_ft(const xplane_context::XPlaneContext &ctx,
                                    int target_ft) {
  if (!openair_db::ready() || target_ft <= 0)
    return 0;
  const int cur_alt = openair_alt(ctx); // FL-aware (openair ceilings are FLs)
  const openair_db::AirspaceEntry cur =
      openair_db::find_enclosing(ctx.latitude, ctx.longitude, cur_alt);
  if (cur.name.empty())
    return 0; // outside all TMAs (ACC/FIR) -- no inner-boundary clamp here
  const openair_db::AirspaceEntry lower =
      openair_db::find_enclosing(ctx.latitude, ctx.longitude, target_ft);
  if (lower.name.empty() || lower.name == cur.name)
    return 0; // target level is in the same volume -- no boundary crossed
  // Same controlling unit (sub-sectors of one TMA)? Then no transfer is needed.
  float cf = 0.0f, lf = 0.0f;
  if (resolve_tma_controller(cur.name, nullptr, &cf) &&
      resolve_tma_controller(lower.name, nullptr, &lf) &&
      std::fabs(cf - lf) < 0.005f)
    return 0;
  return lower.ceiling_ft; // inner controller owns everything below this
}

// enter_approach: true (Stage B) = also flip to IFR_APPROACH_CONTACT and latch the
// one-shot lock (legacy behaviour). false (Stage A) = issue the "contact X" call
// only and STAY in IFR_ARRIVAL (an intermediate TMA frequency handoff while flying
// the STAR); deduped by controller label via s_arrival_freq_handoff_label.
static bool build_approach_handoff(const xplane_context::XPlaneContext &ctx,
                                   const std::string &callsign,
                                   std::string *out_text,
                                   const openair_db::AirspaceEntry &enc,
                                   bool enter_approach = true) {
  using AS = atc_state_machine::ATCState;
  using FT = xplane_context::FrequencyType;
  if (enter_approach && s_enroute_approach_handoff_issued)
    return false;

  auto ofp = simbrief_ofp::get();

  std::string app_label;
  float app_freq = 0.0f;

  logging::info(
      "IFR arrival handoff: openair enc='%s' class=%d floor=%dft ceil=%dft"
      " at %.0fft MSL pos=%.4f,%.4f",
      enc.name.c_str(), static_cast<int>(enc.ac_class),
      enc.floor_ft, enc.ceiling_ft,
      ctx.altitude_ft_msl, ctx.latitude, ctx.longitude);

  // Primary: use the openair TMA name to find the correct TRACON by name
  // (same logic as departure handoff — altitude-correct, not centroid-distance).
  // "CHAMBERY TMA SECTOR 1" → "CHAMBERY" → Chambery TRACON.
  if (enc.ac_class == openair_db::AirspaceClass::TMA ||
      enc.ac_class == openair_db::AirspaceClass::CTA) {
    std::string lbl;
    float f = 0.0f;
    // Unified resolver (step 1c): terminal=true so a CTA resolves to the TRACON
    // like a TMA (same as the previous direct resolve_tma_controller call).
    if (resolve_sector_controller(enc, /*terminal=*/true, &lbl, &f)) {
      app_label = lbl;
      app_freq  = f;
      logging::info("IFR arrival handoff: [P1-openair] '%s' -> %s %.3f",
                    enc.name.c_str(), app_label.c_str(), app_freq);
    } else {
      logging::info(
          "IFR arrival handoff: [P1-openair] '%s' -> no controller match, falling back",
          enc.name.c_str());
    }
  }

  // Fallback 1: destination airport's own APPROACH frequency from apt.dat.
  // ctx.airport_freqs is the NEAREST airport's freq list — only valid as
  // the destination's approach freq once nearest_airport_id has actually
  // switched to the destination. Guard on that equality: without it, a
  // handoff issued while still ~50+ NM out picks a random nearer airport's
  // approach freq (LIMF -> LFLP 2026-07-09: nearest=LFLI gave 136.250
  // instead of the correct Geneva sector 119.530). When nearest != dest,
  // skip P2 and let the sector-based handoff (openair/atc.dat) or the
  // later poll_approach local handoff serve the correct frequency.
  // NOTE: minimal call-site guard for v4.3.1; the full nearest_airport ->
  // s_assigned_dest_icao refactor is deferred to v4.4.0
  // (see [[feedback_nearest_airport_ifr]]).
  const bool nearest_is_dest =
      !s_assigned_dest_icao.empty() &&
      ctx.nearest_airport_id == s_assigned_dest_icao;
  if (app_label.empty() && nearest_is_dest) {
    float arr_app_freq = ctx.airport_freqs.first_mhz(FT::APPROACH);
    float arr_dep_freq = ctx.airport_freqs.first_mhz(FT::DEPARTURE);
    float arr_freq = arr_app_freq >= 100.0f ? arr_app_freq : arr_dep_freq;
    if (arr_freq >= 100.0f) {
      FT ft = arr_app_freq >= 100.0f ? FT::APPROACH : FT::DEPARTURE;
      std::string raw = ctx.airport_freqs.first_name(ft);
      app_label = raw.empty()
                      ? (ctx.nearest_airport_id + " Approach")
                      : controller_location(raw) + " Approach";
      app_freq = arr_freq;
      logging::info("IFR arrival handoff: [P2-apt.dat] %s %.3f (nearest=%s)",
                    app_label.c_str(), app_freq,
                    ctx.nearest_airport_id.c_str());
    } else {
      logging::info(
          "IFR arrival handoff: [P2-apt.dat] no APP/DEP freq for nearest=%s",
          ctx.nearest_airport_id.c_str());
    }
  }

  // Fallback 3 (arrival): atc.dat TRACON whose polygon ENCLOSES the destination
  // airport. Stricter than the departure P3's find_by_role_near (nearest): the
  // TRACON must actually contain the destination, so a STAR field with a
  // delegated approach (LFLP -> Geneva/Chambery) resolves, while a
  // non-controlled AFIS field that sits inside no Approach TRACON polygon
  // (LFQA) correctly finds nothing and falls through to the silent path.
  // The enclosure filter is what makes restoring P3 safe -- the old P3 was
  // removed precisely because "nearest" returned Paris Approach for LFQA
  // (see project_p3_tracon_removal). Queried at the DESTINATION position, so
  // it is immune to the aircraft-nearest drift that also blocks P2.
  if (app_label.empty() && !s_assigned_dest_icao.empty() &&
      airspace_db::enabled()) {
    auto dpos = xplane_context::airport_pos_for(s_assigned_dest_icao);
    if (dpos.first != 0.0 || dpos.second != 0.0) {
      // Probe at a representative low TMA/approach altitude (MSL). EU TMA
      // floors are typically a few thousand feet; 6000 ft sits inside the low
      // band while staying above most field elevations.
      constexpr float kApproachProbeFtMsl = 6000.0f;
      auto enc_ctrls = airspace_db::find_enclosing(dpos.first, dpos.second,
                                                   kApproachProbeFtMsl);
      const airspace_db::Controller *best = nullptr;
      double best_dist = 1e18;
      for (const auto *c : enc_ctrls) {
        if (!c || c->freqs_khz.empty())
          continue;
        if (c->role != airspace_db::ControllerRole::TRACON)
          continue;
        // Multiple TRACON polygons can enclose one field at the same floor
        // (LFLP sits inside Geneva/LSGG, Chambery/LFLB AND Lyon/LFLL at floor
        // 1000). Prefer the higher floor (tighter/more-local, as sector_picker
        // does), then break ties by distance from the destination to the
        // TRACON's FACILITY airport -- i.e. which field this approach actually
        // serves. That ranks Geneva (~18 NM) and Chambery (~20 NM) over the
        // larger Lyon sector (~44 NM) that merely overlaps LFLP. Bbox-centroid
        // distance does NOT discriminate here (Lyon's centroid is marginally
        // nearer). Falls back to the centroid when the facility position is
        // unavailable (e.g. headless, where airport_pos_for is a stub).
        std::pair<double, double> fpos{0.0, 0.0};
        if (!c->facility_id.empty())
          fpos = xplane_context::airport_pos_for(c->facility_id);
        const double d =
            (fpos.first != 0.0 || fpos.second != 0.0)
                ? traffic_geometry::distance_nm(dpos.first, dpos.second,
                                                fpos.first, fpos.second)
                : traffic_geometry::distance_nm(
                      dpos.first, dpos.second,
                      (c->bbox_min_lat + c->bbox_max_lat) * 0.5,
                      (c->bbox_min_lon + c->bbox_max_lon) * 0.5);
        const bool better =
            !best || c->floor_ft > best->floor_ft ||
            (c->floor_ft == best->floor_ft && d < best_dist);
        if (better) {
          best = c;
          best_dist = d;
        }
      }
      if (best) {
        app_freq = static_cast<float>(best->freqs_khz.front()) / 1000.0f;
        app_label = controller_label_for(best) + " Approach";
        logging::info(
            "IFR arrival handoff: [P3-atc.dat encloses dest %s @%.4f,%.4f] "
            "TRACON '%s' -> %s %.3f",
            s_assigned_dest_icao.c_str(), dpos.first, dpos.second,
            best->name.c_str(), app_label.c_str(), app_freq);
      } else {
        logging::info(
            "IFR arrival handoff: [P3-atc.dat] no TRACON encloses dest %s",
            s_assigned_dest_icao.c_str());
      }
    }
  }

  if (app_label.empty()) {
    logging::info("IFR arrival handoff: no Approach controller (P1+P2+P3 failed) -- silent");
    return false;
  }

  // The handoff is SPOKEN by the CURRENT controller (e.g. Marseille on 119.755),
  // so s_current_controller_label must stay until the pilot switches. Defer the
  // target to the PENDING slot: this call is labelled with the current controller
  // ("Marseille"), and process_transcript promotes the pending label to the
  // speaker once the pilot's COM reaches app_freq. Overwriting s_current here
  // mislabeled the handoff call itself as the target ("Geneva Approach: contact
  // Geneva Approach ..." while still on Marseille) -- LIMx->LFLP 2026-07-12.
  auto speak_contact = [&]() {
    s_pending_controller_label = app_label;
    s_pending_handoff_freq_mhz = app_freq;
    if (out_text) {
      char buf[200];
      if (app_freq >= 100.0f)
        std::snprintf(buf, sizeof(buf), "%s, contact %s on %.3f.",
                      callsign.c_str(), app_label.c_str(), app_freq);
      else
        std::snprintf(buf, sizeof(buf), "%s, contact %s.", callsign.c_str(),
                      app_label.c_str());
      *out_text = buf;
    }
  };

  // Stage A: intermediate TMA frequency handoff while flying the STAR. Issue
  // "contact <controller>" and gate the check-in on app_freq, but STAY in
  // IFR_ARRIVAL (no walker, no tracker jump). Dedup by controller label so a
  // multi-sector TMA (GENEVA TMA SECTOR 7/8/...) only announces once.
  if (!enter_approach) {
    const float active_com_arr =
        (ctx.active_com == 2) ? ctx.com2_freq_mhz : ctx.com1_freq_mhz;
    if (app_label == s_arrival_freq_handoff_label ||
        std::fabs(active_com_arr - app_freq) < 0.005f)
      return false; // already tuned to this controller / frequency
    s_arrival_freq_handoff_label = app_label;
    s_enroute_approach_freq_mhz = app_freq; // gate check-in on correct frequency
    // Treat this like an ACC sector handoff: arm the generic sector check-in so
    // the pilot's call on the new freq gets a bare "radar contact" ack and
    // proactive messages pause until then -- NOT the richer approach clearance
    // (which must wait for the IAF / APPROACH_CONTACT).
    s_sector_checkin_pending = true;
    speak_contact();
    logging::info("IFR arrival: intermediate freq handoff -> %s %.3f MHz (stay ARRIVAL)",
                  app_label.c_str(), app_freq);
    return true;
  }

  // Stage B: enter the APPROACH phase (starts the poll_approach walker).
  s_enroute_approach_handoff_issued = true;
  s_enroute_approach_freq_mhz = app_freq;
  atc_state_machine::set_state(AS::IFR_APPROACH_CONTACT);
  // Flip the phase SILENTLY (no "contact X") when the pilot is already talking to
  // the resolved approach controller -- either because Stage A handed them there
  // (single-TMA field like Nice) OR because they are already on that FREQUENCY via
  // an upstream ACC handoff. At LFLP the Marseille->Geneva ACC handoff already put
  // the pilot on Geneva 119.530; at APPROACH entry (still above the Chambery
  // ceiling) find_enclosing resolves Geneva again, so a spoken handoff would be
  // "contact Geneva on 119.530" while already on 119.530 -- a dead lock on a
  // check-in that can never differ (LFLP 2026-07-15). The real Geneva->Chambery
  // terminal handoff fires later from poll_approach's sector-change when the
  // aircraft descends into the Chambery TMA. Only speak when the controller
  // genuinely differs from the one currently tuned.
  const float active_com_app =
      (ctx.active_com == 2) ? ctx.com2_freq_mhz : ctx.com1_freq_mhz;
  if (app_label == s_arrival_freq_handoff_label ||
      std::fabs(active_com_app - app_freq) < 0.005f) {
    // Already on the terminal controller (the pilot checked in via the sector ack
    // BEFORE APPROACH entry) -> go straight to APPROACH_DESCENT. Staying in
    // APPROACH_CONTACT would wait for an "approach check-in" that already happened,
    // so the pilot's NEXT readback (a speed or course readback) was consumed as the
    // check-in and wrongly answered "radar contact, identified, continue descent"
    // (LFLP 2026-07-17). poll_approach's cleared-approach fires in DESCENT.
    atc_state_machine::set_state(AS::IFR_APPROACH_DESCENT);
    s_sector_checkin_pending = false;
    logging::info("IFR arrival: APPROACH (already on %s %.3f) -- silent flip -> APPROACH_DESCENT",
                  app_label.c_str(), app_freq);
    return true; // out_text stays empty -> caller falls through to poll_approach
  }
  // Arm the sector check-in (like Stage A at 4837 and the ACC sector handoff).
  // Without it, s_sector_checkin_pending stays false after the approach handoff, so
  // the pilot's readback on the OLD freq ("Chambery approach on 121.205" while still
  // on Geneva 119.530) bypassed the handoff-pending branch in process_transcript and
  // was consumed as the Chambery CHECK-IN -- answered "radar contact, continue
  // descent" spoken by the OUTGOING controller (Geneva), inside the destination TMA
  // (LFLP 2026-07-20). With the flag set, the old-freq readback is accepted silently
  // (handoff-ack) and the check-in is only honoured once the pilot is on the terminal
  // freq, where the check-in detection defers to the richer approach handler.
  s_sector_checkin_pending = true;
  speak_contact();
  logging::info("IFR en-route: approach handoff -> %s %.3f MHz",
                app_label.c_str(), app_freq);
  return true;
}

// Pick the first non-SID/STAR navlog fix that is ahead of the aircraft
// (distance > 20 NM) for the en-route direct-to shortcut.
// SimBrief pseudo-fix identifiers that are not real nav fixes.
static bool is_pseudo_fix(const std::string &ident) {
  static const char *kPseudo[] = {"TOC", "TOD", "BOC", "BOD", "SOSTA", nullptr};
  for (int i = 0; kPseudo[i]; ++i)
    if (ident == kPseudo[i])
      return true;
  return false;
}

static std::string
pick_direct_fix(const xplane_context::XPlaneContext &ctx,
                const std::vector<simbrief_ofp::NavlogFix> &navlog) {
  for (const auto &fix : navlog) {
    if (fix.is_sid_star)
      continue;
    if (fix.ident.empty() || is_pseudo_fix(fix.ident))
      continue;
    double dist = traffic_geometry::distance_nm(ctx.latitude, ctx.longitude,
                                                fix.lat, fix.lon);
    if (dist < 20.0 || dist >= 500.0)
      continue;
    // Skip fixes that are behind the aircraft (bearing > 90 deg off heading).
    double dlat = fix.lat - ctx.latitude;
    double dlon = (fix.lon - ctx.longitude) * std::cos(ctx.latitude * M_PI / 180.0);
    double bearing_deg = std::atan2(dlon, dlat) * 180.0 / M_PI;
    double hdg = static_cast<double>(ctx.heading_true);
    double diff = std::abs(bearing_deg - hdg);
    if (diff > 180.0) diff = 360.0 - diff;
    if (diff > 90.0)
      continue; // fix is behind — skip
    return fix.ident;
  }
  return {};
}

// Nearest upcoming SID/STAR/approach speed cap (kt) from the unified route
// table, scanning a short look-ahead window from the current tracker index.
// 0 = no cap in range (or the table isn't populated — e.g. SID climb today,
// which falls back to the pure ICAO 250 kt rule). Reads s_route_fixes so SID
// and STAR/approach share one rule the moment the table carries the fix.
// Highest block-"B" FLOOR among route fixes NOT yet passed (positional: from
// the current tracker index forward). A descent may never be cleared below this
// until the block fix is physically behind the aircraft. 0 = no active block.
// This is the authoritative block gate: even if the walker's queue skips a
// block fix on altitude, the descent target is still clamped up to this floor
// (LFMN R22LZ: at MUS/FL080 the walker grabbed SOTOX FL070, below MN261's
// FL080 block floor). Positional, so the floor releases once MN261 is passed.
static int active_block_floor_ft() {
  int floor = 0;
  for (int i = std::max(0, s_route_fix_idx);
       i < static_cast<int>(s_route_fixes.size()); ++i)
    if (s_route_fixes[i].floor_ft > floor)
      floor = s_route_fixes[i].floor_ft;
  return floor;
}

// Compliance of the NEXT constrained route fix with the aircraft's current
// altitude/speed. This is the enforcement primitive behind the corrective ATC
// model: the pilot flies the published SID/STAR/approach profile IMPLICITLY,
// and ATC issues a statement ONLY when the fix is within `lead_seconds` flying
// time AND the aircraft will bust its constraint. The trigger is TIME, not a
// fixed distance -- a jet at 300 kt and a turboprop at 150 kt get the same
// warning lead. Phase-agnostic: it reads s_route_fixes, so the same monitor
// serves SID, en-route, and approach (the roadmap enforcement-symmetry goal).
struct FixCompliance {
  bool        valid = false;   // a positioned, constrained fix exists ahead
  std::string ident;
  double      dist_nm = 1e9;
  double      eta_sec = 1e9;   // estimated flying time to the fix
  bool        near = false;    // within lead_seconds of that fix
  bool        alt_bust = false;
  int         alt_target_ft = 0;
  bool        alt_is_fl = false;
  bool        spd_bust = false;
  int         spd_target_kt = 0;
};

// Routed "distance to FLY" (NM) from the aircraft to route fix `target_idx`: the
// leg from the aircraft to the current tracked fix, plus each subsequent leg up to
// the target. Use for TOD / ETA / descent planning -- NOT straight-line, which
// under-reads on a dog-legged STAR and fires descents far too early (LFLP: FL090
// cleared 53 NM out because great-circle acft->LUVOB was 53 NM while the routed STAR
// path is longer). Straight-line fallback when the target is behind the tracker,
// out of range, or position-less. Proximity/capture checks keep using straight-line
// (see [[project_distance_limitation]] -- distance-to-fly vs distance-now).
static double routed_distance_to_fix_idx(const xplane_context::XPlaneContext &ctx,
                                         int target_idx) {
  const int n = static_cast<int>(s_route_fixes.size());
  if (target_idx < 0 || target_idx >= n)
    return 0.0;
  int start = std::max(0, s_route_fix_idx);
  const auto &tgt = s_route_fixes[target_idx];
  if (target_idx < start || (tgt.lat == 0.0 && tgt.lon == 0.0))
    return traffic_geometry::distance_nm(ctx.latitude, ctx.longitude, tgt.lat,
                                         tgt.lon);
  // Skip fixes at/after `start` that are BEHIND the aircraft. The route tracker
  // lags between resyncs (it advances only within ~2 NM of a fix), so fix[start]
  // can be behind the aircraft; summing the backward leg aircraft->fix[start]
  // INFLATES the routed distance and fires the descent late/steep -- FL090 was
  // held until the resync corrected the index (LFLP 2026-07-17). Advance to the
  // first fix that is ahead (bearing within 100 deg of the nose).
  while (start < target_idx) {
    const auto &f = s_route_fixes[start];
    if (f.lat == 0.0 && f.lon == 0.0) { ++start; continue; }
    double brg = traffic_geometry::bearing_deg(ctx.latitude, ctx.longitude,
                                               f.lat, f.lon);
    double off = std::fabs(brg - static_cast<double>(ctx.heading_true));
    if (off > 180.0) off = 360.0 - off;
    if (off <= 100.0) break; // fix is ahead -> start summing here
    ++start;                 // fix is behind -> skip it
  }
  double total = 0.0;
  double plat = ctx.latitude, plon = ctx.longitude;
  for (int i = start; i <= target_idx; ++i) {
    const auto &f = s_route_fixes[i];
    if (f.lat == 0.0 && f.lon == 0.0)
      continue; // skip position-less fix, keep summing the chain
    total += traffic_geometry::distance_nm(plat, plon, f.lat, f.lon);
    plat = f.lat;
    plon = f.lon;
  }
  return total;
}

static FixCompliance check_next_fix(const xplane_context::XPlaneContext &ctx,
                                    double lead_seconds) {
  using AS = atc_state_machine::ATCState;
  FixCompliance c;
  const int ta = ctx.transition_alt_ft > 0 ? ctx.transition_alt_ft : 5000;
  const float pa = ctx.pressure_alt_ft;
  // Phase gate: APPROACH-procedure fixes (LP403 etc.) are enforced ONLY once in an
  // approach phase. On the STAR (DESCENT/ARRIVAL) they must be ignored -- otherwise
  // an APP constraint leaks into the STAR (LFLP 2026-07-14: LP403's 200 kt / 6500 ft,
  // an APP fix that loops back geographically near COLLO, was issued during the STAR
  // near COLLO, skipping LUVOB/GOVNA's 210 kt). The STAR uses only STAR fixes.
  const AS st = atc_state_machine::get_state();
  const bool in_approach =
      st == AS::IFR_APPROACH_CONTACT || st == AS::IFR_APPROACH_DESCENT ||
      st == AS::IFR_APPROACH_TOWER || st == AS::IFR_LANDING_CLEARED;
  for (int i = std::max(0, s_route_fix_idx);
       i < static_cast<int>(s_route_fixes.size()); ++i) {
    const auto &f = s_route_fixes[i];
    if (f.is_approach_proc && !in_approach)
      continue; // APP fix but still on the STAR -> not yet enforceable
    const bool has_alt = (f.alt.feet > 0) || (f.floor_ft > 0);
    const bool has_spd = (f.speed_kt > 0);
    if (!has_alt && !has_spd)
      continue; // unconstrained fix — flown implicitly, nothing to enforce
    if (f.lat == 0.0 && f.lon == 0.0)
      continue; // no position — cannot range-gate
    c.valid   = true;
    c.ident   = f.ident;
    // Routed distance-to-fly (sum of legs), not straight-line -- the TOD trigger
    // reads this, and great-circle under-read fired FL090 53 NM out (LFLP).
    c.dist_nm = routed_distance_to_fix_idx(ctx, i);
    // Time to the fix = distance / groundspeed (floored so we don't divide by a
    // near-zero GS on the ground / in a hold).
    const double gs = ctx.groundspeed_kts > 40.0f
                          ? static_cast<double>(ctx.groundspeed_kts)
                          : 120.0;
    c.eta_sec = c.dist_nm / gs * 3600.0;
    c.near    = (c.eta_sec <= lead_seconds);
    // Altitude bust: the aircraft will not satisfy the fix's altitude band.
    if (f.floor_ft > 0) { // block "B": must be within [floor, ceiling]
      if (pa > static_cast<float>(f.alt.feet) + 200.0f ||
          pa < static_cast<float>(f.floor_ft) - 200.0f) {
        c.alt_bust = true;
        c.alt_target_ft = f.floor_ft; // descend to (or climb to) the block floor
        c.alt_is_fl = (f.floor_ft >= compute_tl_ft(ta, ctx.qnh_hpa));
      }
    } else if (f.is_floor) { // at-or-above: bust only if BELOW
      if (pa < static_cast<float>(f.alt.feet) - 200.0f) {
        c.alt_bust = true; c.alt_target_ft = f.alt.feet; c.alt_is_fl = f.alt.is_fl;
      }
    } else if (f.alt.feet > 0) { // ceiling / "at": bust if ABOVE (descent case)
      if (pa > static_cast<float>(f.alt.feet) + 200.0f) {
        c.alt_bust = true; c.alt_target_ft = f.alt.feet; c.alt_is_fl = f.alt.is_fl;
      }
    }
    // Speed bust: faster than the fix's cap (5 kt hysteresis).
    if (has_spd && ctx.indicated_airspeed_kts > static_cast<float>(f.speed_kt) + 5.0f) {
      c.spd_bust = true; c.spd_target_kt = f.speed_kt;
    }
    return c; // first constrained fix ahead is the one that governs
  }
  return c;
}

// Lateral analog of check_next_fix (the "DirectMonitor" primitive): course
// compliance to the ACTIVE next route fix. off_course = the aircraft's heading
// diverges from the bearing to the next positioned fix by more than
// threshold_deg. Drives "confirm direct <fix>" corrections at SID / en-route /
// approach. Firing gates (cooldown, distance guard, threshold, post-turn grace)
// are decided per phase at the call site -- wired now, thresholds TBD.
struct CourseCheck {
  bool        valid = false;
  std::string ident;
  double      dist_nm = 1e9;
  double      bearing_deg = 0.0;
  double      diff_deg = 0.0;
  bool        off_course = false;
};

static CourseCheck check_course(const xplane_context::XPlaneContext &ctx,
                                double threshold_deg) {
  CourseCheck c;
  for (int i = std::max(0, s_route_fix_idx);
       i < static_cast<int>(s_route_fixes.size()); ++i) {
    const auto &f = s_route_fixes[i];
    if (f.lat == 0.0 && f.lon == 0.0)
      continue; // no position -> cannot compute a bearing
    const double bearing = traffic_geometry::bearing_deg(
        ctx.latitude, ctx.longitude, f.lat, f.lon);
    const double dist = traffic_geometry::distance_nm(ctx.latitude,
                                                      ctx.longitude, f.lat, f.lon);
    double diff = std::fabs(bearing - static_cast<double>(ctx.heading_true));
    if (diff > 180.0) diff = 360.0 - diff;
    // Skip a fix that is clearly BEHIND the aircraft (bearing >120 deg off the
    // nose) and not adjacent: it was bypassed, so "fly direct to it" is wrong --
    // evaluate the next fix instead. Belt-and-suspenders with the tracker's own
    // bypass-advance; keeps a stale index from firing "confirm direct <behind
    // fix>" (LFMN ABDI8R 2026-07-13, diff 179 to ABDIL).
    if (diff > 120.0 && dist > 1.5) continue;
    c.valid       = true;
    c.ident       = f.ident;
    c.dist_nm     = dist;
    c.bearing_deg = bearing;
    c.diff_deg    = diff;
    c.off_course  = (diff > threshold_deg);
    // Turn-anticipation suppression: if the aircraft is still within the turn-settle
    // radius of the PREVIOUS route fix, a large diff to THIS fix is just the upcoming
    // turn, not an off-course -- the pilot flies the current leg, then turns at the
    // fix. LFLP 2026-07-18: 1.4 NM before BIVLO, heading 273 on the inbound leg,
    // bearing 234 to the NEXT fix SALEV -> false "confirm direct SALEV". Once
    // established on the new leg (past the settle radius) a real deviation still fires.
    constexpr double kTurnAnticipationNm = 2.5;
    if (c.off_course && i > 0) {
      const auto &prev = s_route_fixes[i - 1];
      if (!(prev.lat == 0.0 && prev.lon == 0.0) &&
          traffic_geometry::distance_nm(ctx.latitude, ctx.longitude, prev.lat,
                                        prev.lon) < kTurnAnticipationNm)
        c.off_course = false;
    }
    return c; // first positioned fix ahead is the active leg target
  }
  return c;
}

// Descent-planning tunables (shared by the STAR-crossing TOD trigger and the
// enroute pre-TOD step alert). Two independent levers:
//   - SLOPE: 265 ft/NM ~= 2.5 deg (a true 2.5deg = tan(2.5)*6076 = 265). Was
//     300 ft/NM (~2.83 deg). A shallower slope makes TOD fire earlier and the
//     descent more gradual, and it scales with the size of the descent.
//   - REACTION buffer: a flat extra distance added before the geometric TOD so the
//     call comes with room to start down. Bump to 10.0 for "+5 NM more" (user
//     2026-07-17). NOTE: 3 deg (ILS glideslope / the pilot 3:1 rule) is a planning
//     convention, not an ATC rule -- safe to tune.
static constexpr double kDescentSlopeFtPerNm = 265.0; // 2.5 deg
static constexpr double kDescentReactionNm   = 8.0;   // flat pre-TOD buffer (NM)

// ── poll_profile_crossing ─────────────────────────────────────────────────
// Extracted from poll_descent: the CIFP crossing-altitude corrective, now shared
// by every airborne IFR phase via poll_profile_enforcement. Fires once per target
// (s_descent_cifp_target_ft dedup) when the next constrained fix will be busted
// within ~90 s of flying time AND its target is genuinely below what is already
// cleared. Records the target in s_enroute_cleared_alt_ft so the poll_approach
// walker and descend-to-enter (which read current_cleared_alt_ft) don't re-issue
// the same descent -- the `>= cleared` guard is the coordination point. Altitude
// reference is correct per constraint (check_next_fix judges FL vs feet, TL-aware).
static bool poll_profile_crossing(const xplane_context::XPlaneContext &ctx,
                                  std::string *out_text) {
  if (!out_text)
    return false;
  // Once "cleared for the approach", the pilot flies the PUBLISHED vertical profile
  // (the charted step-down altitudes); ATC does NOT issue each step-down descent --
  // ICAO Doc 4444 / EUROCONTROL "descend with the procedure" (confirmed 2026-07-17,
  // continental EU). So stop the crossing correctives once the approach is cleared;
  // the pilot descends PIRUV/LP403/LP402/IP04Z/FP04Z per the chart. Tower handoff +
  // landing still fire.
  if (s_approach_cleared_issued)
    return false;
  // Do not issue a descent while a SECTOR HANDOFF is in progress (the pilot is
  // switching controllers). This is the RIGHT gate -- it's brief (only until the
  // pilot checks in on the new freq) and prevents the OLD controller from issuing a
  // new descent mid-handoff: Geneva issued "descend 6500" (Chambery's) right after
  // "contact Chambery", stranding its readback across the switch (LFLP 2026-07-17).
  // NOTE: an earlier gate on is_readback_pending was WRONG -- it blocked the
  // time-critical descent for ANY unrelated readback, so FL090 fired 12 NM late and
  // steep. s_sector_checkin_pending is false when FL090 fires (no handoff then), so
  // this gate does not delay it.
  if (s_sector_checkin_pending)
    return false;
  // Don't clear a descent INTO a terminal TMA the aircraft has ENTERED but is NOT
  // YET handed off to. As it descends across the inner-TMA boundary, find_enclosing
  // already returns the INNER TMA (e.g. Chambery) while the pilot is still on the
  // OUTER controller (Geneva) -- the handoff poll (15 s) lags the crossing poll, so
  // the outer controller gets a cycle in and clears a descent DEEP into the inner
  // controller's airspace right before the handoff (LFLP 2026-07-18: Geneva "descend
  // 6500" 8 s before "contact Chambery"). The transfer-floor guard misses this (it
  // only suppresses while still in the OUTER sector) and s_sector_checkin_pending is
  // still false (handoff not issued yet). Suppress when the enclosing TMA resolves to
  // a controller on a DIFFERENT freq than the pilot's active COM -> the handoff fires
  // first, then the INNER controller issues the step-downs.
  if (openair_db::ready()) {
    const openair_db::AirspaceEntry enc = openair_db::find_enclosing(
        ctx.latitude, ctx.longitude, openair_alt(ctx));
    if (enc.ac_class == openair_db::AirspaceClass::TMA) {
      std::string tlbl;
      float tfreq = 0.0f;
      if (resolve_sector_controller(enc, /*terminal=*/true, &tlbl, &tfreq) &&
          tfreq > 0.0f) {
        const float acom =
            (ctx.active_com == 2) ? ctx.com2_freq_mhz : ctx.com1_freq_mhz;
        if (std::fabs(acom - tfreq) >= 0.005f)
          return false; // in an un-handed-off terminal TMA -> let the handoff win
      }
    }
  }
  // Wide window: check_next_fix returns the governing constrained fix regardless
  // of proximity; the REAL trigger is the top-of-descent distance below.
  const FixCompliance fc = check_next_fix(ctx, 900.0);
  if (!(fc.valid && fc.alt_bust && fc.alt_target_ft > 0 &&
        fc.alt_target_ft != s_descent_cifp_target_ft))
    return false;
  const int cleared = engine::current_cleared_alt_ft();
  if (fc.alt_target_ft >= cleared)
    return false; // only a genuine DOWN step below what is already cleared
  // ICAO/EUROCONTROL: the current controller may not descend the aircraft into a
  // lower, differently-controlled TMA. If this crossing's target lies below the
  // ceiling of such an inner TMA, the descent belongs to the inner controller --
  // suppress it here. The descend-to-enter clearance brings the aircraft to the
  // boundary, the handoff transfers control, and the inner controller then issues
  // it (Geneva must not clear "descend 6500" into the Chambery TMA -- LFLP
  // 2026-07-15). Once handed off and inside the inner TMA, cur == inner volume and
  // this returns 0, so the inner controller's crossings fire normally.
  const int xfer_floor = sector_transfer_floor_ft(ctx, fc.alt_target_ft);
  if (xfer_floor > 0 && fc.alt_target_ft < xfer_floor) {
    static int s_last_suppressed_target = 0; // dedup the per-frame diagnostic
    if (fc.alt_target_ft != s_last_suppressed_target) {
      s_last_suppressed_target = fc.alt_target_ft;
      if (settings::debug_logging())
        logging::info("[dbg prof] crossing %s -> %d ft SUPPRESSED: below inner-TMA "
                      "transfer floor %d ft (belongs to inner controller)",
                      fc.ident.c_str(), fc.alt_target_ft, xfer_floor);
    }
    return false;
  }
  // Fire at the TOP OF DESCENT for this crossing: within the distance needed to
  // lose the altitude at kDescentSlopeFtPerNm + kDescentReactionNm buffer. A fixed 90 s
  // (~5 NM) window was physically impossible for a big step -- FL140->FL090 by
  // LUVOB needs ~17 NM, so the aircraft stayed high, missed the crossing AND
  // stayed above the inner (Chambery) TMA so the handoff never fired
  // (LFLP 2026-07-14). Altitude reference is correct per constraint (FL vs feet).
  const float alt_now = fc.alt_is_fl ? ctx.pressure_alt_ft : ctx.altitude_ft_msl;
  const double alt_to_lose = static_cast<double>(alt_now) - fc.alt_target_ft;
  if (alt_to_lose <= 200.0)
    return false; // essentially at the level already
  const double tod_dist = alt_to_lose / kDescentSlopeFtPerNm + kDescentReactionNm;
  if (fc.dist_nm > tod_dist)
    return false; // not yet at the top of descent for this crossing
  s_descent_cifp_target_ft = fc.alt_target_ft;
  s_enroute_cleared_alt_ft = fc.alt_target_ft; // coordinates with the walker
  const std::string &cs = atc_state_machine::session_callsign();
  const std::string &callsign = cs.empty() ? settings::pilot_callsign() : cs;
  const int ta = (ctx.transition_alt_ft > 0) ? ctx.transition_alt_ft : 5000;
  const AltHint hint = fc.alt_is_fl ? AltHint::FlightLevel : AltHint::Auto;
  const std::string clr = format_alt_clearance(fc.alt_target_ft, hint, ctx.qnh_hpa, ta);
  *out_text = callsign + ", descend " + clr + ".";
  if (settings::debug_logging())
    logging::info("[dbg prof] crossing %s -> descend %s @ PA %.0f (%.1f NM, TOD %.1f NM, "
                  "lose %.0f ft, st=%d)",
                  fc.ident.c_str(), clr.c_str(),
                  static_cast<double>(ctx.pressure_alt_ft), fc.dist_nm, tod_dist,
                  alt_to_lose, static_cast<int>(atc_state_machine::get_state()));
  return true;
}

// ── poll_speed_restriction ────────────────────────────────────────────────
// Effective speed limit = min(ICAO 250 kt < FL100, nearest active SID/STAR/
// approach waypoint cap). Continuously enforced: fires "reduce speed, N knots
// or less" whenever IAS exceeds the limit + 5 kt (hysteresis). The flag is held
// until the pilot complies (IAS <= limit - 5 kt); a subsequent overspeed, or a
// newly tighter limit, re-fires the advisory.
bool poll_speed_restriction(const xplane_context::XPlaneContext &ctx,
                            std::string *out_text) {
  using AS = atc_state_machine::ATCState;
  auto state = atc_state_machine::get_state();
  if (state != AS::IFR_RADAR_CONTACT   &&
      state != AS::IFR_ENROUTE_CRUISE  &&
      state != AS::IFR_DESCENT          &&
      state != AS::IFR_ARRIVAL          &&
      state != AS::IFR_APPROACH_CONTACT &&
      state != AS::IFR_APPROACH_DESCENT &&
      state != AS::IFR_APPROACH_TOWER)
    return false;
  // Don't issue a speed restriction mid-handoff (see poll_profile_crossing): brief,
  // avoids the old controller issuing a clearance the pilot must read back on the
  // new freq. Gated on the handoff, NOT on any readback (which delayed the profile).
  if (s_sector_checkin_pending)
    return false;
  // Outgoing sector goes QUIET in the DESTINATION's terminal TMA. Once the aircraft
  // is inside the destination's own terminal control area (Chambery TMA for LFLP),
  // the terminal controller owns speed control -- the PREVIOUS sector (Geneva) must
  // not issue a new restriction there, even in the window BEFORE the handoff fires
  // (s_sector_checkin_pending is still false then, so the gate above doesn't cover
  // it). Geneva was giving "reduce speed 210" inside Chambery TMA Sector 3 while the
  // handoff waited for the IAF sequence gate (user 2026-07-19). Only in the PRE-
  // approach states (still with the outgoing sector); once in IFR_APPROACH_* the
  // pilot IS on the terminal controller and its speed calls are correct. Guarded on
  // openair-ready + dest known (same as the poll_acc_sector_change defer) so the
  // helper's permissive true-when-unavailable never suppresses enroute restrictions.
  if ((state == AS::IFR_RADAR_CONTACT || state == AS::IFR_ENROUTE_CRUISE ||
       state == AS::IFR_DESCENT || state == AS::IFR_ARRIVAL) &&
      !s_assigned_dest_icao.empty() && openair_db::ready() &&
      on_destination_terminal(ctx))
    return false;

  // Effective speed limit in force now: the tighter of the ICAO 250 kt < FL100
  // rule and any active SID/STAR/approach waypoint cap from the unified route
  // table (e.g. 210 kt at a terminal fix). 0 = no restriction.
  // FL100 is a FLIGHT LEVEL (always above the transition level), so the boundary
  // is pressure altitude, not QNH/MSL -- with a low QNH the aircraft crosses
  // FL100 while still above 10000 ft MSL, and the 250 kt limit applies from FL100.
  int limit = (ctx.pressure_alt_ft < 10000.0f) ? 250 : 0;
  // Procedure speed cap is enforced CORRECTIVELY: the pilot flies the published
  // cap implicitly, and ATC only calls it when the fix is within ~60 s flying
  // time AND the aircraft is still too fast (so NANAX's 200 kt fires approaching
  // NANAX, not back at MUS -- LFMN R22LZ 2026-07-12). Shared compliance monitor.
  const FixCompliance fc = check_next_fix(ctx, 60.0);
  const int proc = (fc.valid && fc.near && fc.spd_bust) ? fc.spd_target_kt : 0;
  if (proc > 0)
    limit = (limit == 0) ? proc : std::min(limit, proc);

  // A newly tighter limit re-arms the advisory (e.g. 250 -> 210 approaching a
  // terminal fix) even if it had already fired for the looser limit.
  if (limit > 0 && s_last_speed_limit_kt > 0 && limit < s_last_speed_limit_kt)
    s_speed_250_warned = false;
  s_last_speed_limit_kt = limit;

  if (limit == 0) { // no restriction — reset (re-arms next descent) and stay silent
    s_speed_250_warned = false;
    return false;
  }
  // Compliance / hysteresis: 5 kt band around the effective limit.
  if (ctx.indicated_airspeed_kts <= static_cast<float>(limit) - 5.0f)
    s_speed_250_warned = false;
  if (s_speed_250_warned)
    return false;
  if (ctx.indicated_airspeed_kts <= static_cast<float>(limit) + 5.0f)
    return false;

  s_speed_250_warned = true;
  const std::string &cs = atc_state_machine::session_callsign();
  const std::string &callsign = cs.empty() ? settings::pilot_callsign() : cs;
  // DIAGNOSTIC (alpha-20): which fix drove the cap + the tracker index, to pin the
  // premature-200kt bug (LP403 issued at COLLO). Remove once the LFLP looping-RNAV
  // enforcement is fixed.
  if (settings::debug_logging()) {
    const FixCompliance dfc = check_next_fix(ctx, 60.0);
    logging::info("[dbg spd] issuing %d kt @ PA %.0f ft -- check_next_fix: %s idx=%d "
                  "dist=%.1f near=%d spd_tgt=%d (route_idx=%d/%d)",
                  limit, static_cast<double>(ctx.pressure_alt_ft),
                  dfc.valid ? dfc.ident.c_str() : "(none)", s_route_fix_idx,
                  dfc.dist_nm, dfc.near ? 1 : 0, dfc.spd_target_kt,
                  s_route_fix_idx, static_cast<int>(s_route_fixes.size()));
  }
  logging::info("IFR speed: %.0f ft IAS %.0f kts -- issuing %d kt restriction",
                ctx.altitude_ft_msl, ctx.indicated_airspeed_kts, limit);
  if (out_text) {
    char buf[128];
    std::snprintf(buf, sizeof(buf),
                  "%s, reduce speed, %d knots or less.",
                  callsign.c_str(), limit);
    *out_text = buf;
  }
  return true;
}

// ── poll_profile_enforcement ──────────────────────────────────────────────
// Unified in-front-profile enforcement, dispatched once per frame BEFORE the
// per-phase poll_* handlers. It runs the same three checks in EVERY airborne IFR
// phase, so no phase is silently unmonitored -- the "no altitude warning at
// LP402/LP403 in the approach" bug came from each check being wired to a
// different, non-overlapping phase set (speed skipped DESCENT/ARRIVAL; the
// crossing corrective was DESCENT-only; the compliance nag was DESCENT/ARRIVAL-
// only). Priority order, one utterance/frame:
//   1. altitude crossing   -- "descend X" for the next constrained fix that will
//      be busted within ~90 s. Mandatory readback. (DESCENT/ARRIVAL/APPROACH.)
//   2. speed restriction    -- min(ICAO 250 kt < FL100, next fix cap). Readback.
//   3. cleared-level nag     -- "confirm descending X". Advisory, no readback.
// All three coordinate with the poll_approach walker + descend-to-enter through
// the shared s_enroute_cleared_alt_ft, so nothing double-fires. The altitude
// reference is correct per constraint (pressure-alt vs FL, QNH/feet vs feet,
// TL-aware) because every check routes through check_next_fix / current_cleared.
bool poll_profile_enforcement(const xplane_context::XPlaneContext &ctx, float dt,
                              std::string *out_text,
                              bool *out_requires_readback) {
  using AS = atc_state_machine::ATCState;
  const AS st = atc_state_machine::get_state();
  const bool airborne_ifr =
      st == AS::IFR_RADAR_CONTACT || st == AS::IFR_ENROUTE_CRUISE ||
      st == AS::IFR_DESCENT || st == AS::IFR_ARRIVAL ||
      st == AS::IFR_APPROACH_CONTACT || st == AS::IFR_APPROACH_DESCENT ||
      st == AS::IFR_APPROACH_TOWER;
  if (out_requires_readback)
    *out_requires_readback = false;
  if (!airborne_ifr)
    return false;

  // Tower-handoff priority near the FAF: once the approach is cleared and the
  // aircraft is within ~2 NM of the FAF (Tower not yet contacted), stop issuing
  // profile enforcement so poll_approach's "contact Tower, report established"
  // wins the frame. On final the pilot flies the published vertical path; ATC
  // issues no further step-downs. Without this, LFLP's tightly-packed step-downs
  // (LP402/IP04Z within ~2 NM of FP04Z) can win the one-utterance-per-frame budget
  // and push the Tower handoff past the FAF (LFLP 2026-07-15).
  if (s_approach_final_issued && !s_approach_tower_handed_off &&
      (s_approach_faf.lat != 0.0 || s_approach_faf.lon != 0.0)) {
    const double faf_nm = traffic_geometry::distance_nm(
        ctx.latitude, ctx.longitude, s_approach_faf.lat, s_approach_faf.lon);
    if (faf_nm < 2.0) {
      logging::info("[approach] profile enforcement yielded to Tower handoff "
                    "(%.1f NM from FAF %s)",
                    faf_nm, s_approach_faf.ident.c_str());
      return false;
    }
  }

  // 1. Altitude crossing -- descent / arrival / approach only (climb is owned by
  //    poll_sid_climb; the final segment by the glidepath, not a crossing).
  const bool alt_phase =
      st == AS::IFR_DESCENT || st == AS::IFR_ARRIVAL ||
      st == AS::IFR_APPROACH_CONTACT || st == AS::IFR_APPROACH_DESCENT;
  if (alt_phase && poll_profile_crossing(ctx, out_text)) {
    if (out_requires_readback)
      *out_requires_readback = true;
    return true;
  }
  // 2. Speed restriction (mandatory readback item).
  if (poll_speed_restriction(ctx, out_text)) {
    if (out_requires_readback)
      *out_requires_readback = true;
    return true;
  }
  // 3. Cleared-level compliance courtesy prompt (advisory only).
  if (poll_altitude_compliance(ctx, dt, out_text))
    return true;
  return false;
}

bool poll_enroute(const xplane_context::XPlaneContext &ctx, float dt,
                  std::string *out_text,
                  bool *out_requires_readback) {
  auto rb = [&](bool v) { if (out_requires_readback) *out_requires_readback = v; };
  rb(false); // default: advisories don't require readback
  using AS = atc_state_machine::ATCState;
  using FP = flight_phase::FlightPhase;

  if (atc_state_machine::get_state() != AS::IFR_ENROUTE_CRUISE) {
    // Reset all flags when not in target state.
    s_enroute_timer = 0.0f;
    // NOTE: do NOT reset s_sector_checkin_pending here. This block fires every
    // frame the state is not IFR_ENROUTE_CRUISE -- including IFR_DESCENT and
    // IFR_ARRIVAL, where poll_acc_sector_change() legitimately sets the flag
    // for a Milan->France->Marseille ACC handoff. Clearing it per-frame here
    // stomped that flag one frame after the handoff, so the pilot's first
    // check-in on the new sector freq fell through the sector-checkin ack
    // (engine.cpp ~line 883) and only a verbatim "France, N... " that the LM
    // happened to classify as INITIAL_CALL_CENTER got a reply (a plain call
    // classified as READBACK was silently dropped). LIMF -> LFLP 2026-07-11
    // log lines 2063/2068: garbled "November 750XR Papa descending FL140" on
    // the correct France freq 118.030 got no ack. The full IFR lifecycle
    // reset (reset_state) and the sector-checkin handler both clear it.
    s_enroute_direct_issued = false;
    s_enroute_direct_delay_sec = 0.0f;
    s_enroute_descent_issued = false;
    s_pilot_requested_descent = false;
    s_enroute_descent_prompt_issued = false;
    s_enroute_approach_handoff_issued = false;
    s_enroute_app_check_sec = 0.0f;
    s_enroute_deviation_cooldown_sec = 0.0f;
    s_enroute_sector_freq_khz = 0;
  s_enroute_visited_sector_freqs.clear();
    s_enroute_sector_check_sec = 15.0f;
    // Do NOT wipe the cleared altitude here for the airborne IFR continuation
    // phases. poll_enroute runs EVERY frame and this block fires for ANY
    // non-ENROUTE_CRUISE state -- so unconditionally zeroing s_enroute_cleared_alt_ft
    // erased the "descend FL120" that build_descent_clearance had just set, ONE
    // FRAME later. current_cleared_alt_ft() then returned 0 for the whole descent,
    // silently killing descend-to-enter, poll_altitude_compliance AND the CIFP
    // crossing corrective (LFMN ABDI8R 2026-07-13: [dbg dte] showed cleared=0 at
    // every tick while over the NICE TMA). Reset it only on a true lifecycle
    // reset to IDLE; DESCENT/ARRIVAL/APPROACH must keep the assigned level.
    if (atc_state_machine::get_state() == AS::IDLE)
      s_enroute_cleared_alt_ft = 0;
    s_enroute_alt_warn_cooldown = 0.0f;
    s_cruise_stepup_issued = false;
    s_navlog_alt_step_idx = 0;
  s_route_step_idx = 0;
    return false;
  }

  auto phase = flight_phase::get();
  if (phase == FP::PARKED || phase == FP::TAXI)
    return false; // auto_correction handles ground reset

  // Re-seed cleared altitude if it was wiped by a prior readback-timeout reset
  // to IDLE (poll_enroute zeroes everything when state != ENROUTE_CRUISE, then
  // the pilot re-checks in and re-enters ENROUTE_CRUISE with a blank baseline).
  // Without a non-zero s_enroute_cleared_alt_ft, sub-phase 1.7 (navlog altitude
  // step monitoring) is permanently gated and descent clearances never fire.
  if (s_enroute_cleared_alt_ft == 0) {
    if (ctx.ifr_cruise_alt_ft > 0)
      s_enroute_cleared_alt_ft = round_to_fl(ctx.ifr_cruise_alt_ft) * 100;
    else if (ctx.altitude_ft_msl > 1000.0f)
      s_enroute_cleared_alt_ft =
          (static_cast<int>(ctx.altitude_ft_msl) / 1000) * 1000;
  }

  // Route table may be empty if the SID phase was skipped (JUMP-to-ENR, airborne
  // start, reposition): build_sid_route_table only runs at SID-init, and
  // init_route_fixes only at approach. Without route fixes the route tracker AND
  // the course monitor (check_course) have nothing to work with -- the reason a
  // jumped-in flight got zero course enforcement (LFLP->LFMN 2026-07-13). Build
  // it lazily here so en-route/descent monitoring works regardless of how the
  // aircraft became airborne.
  if (s_route_fixes.empty()) {
    const auto &ofp_lazy = simbrief_ofp::get();
    if (ofp_lazy.valid && !ofp_lazy.navlog.empty())
      build_sid_route_table(ctx);
  }

  // Training jump: s_current_controller_label not set. Find the CTR sector
  // that geometrically encloses the aircraft at its current FL.
  if (s_current_controller_label.empty()) {
    const auto sectors = airspace_db::find_enclosing(
        ctx.latitude, ctx.longitude, ctx.altitude_ft_msl);
    for (const auto *s : sectors) {
      if (s && s->role == airspace_db::ControllerRole::CTR &&
          !s->freqs_khz.empty()) {
        s_current_controller_label = controller_label_for(s);
        break;
      }
    }
    if (s_current_controller_label.empty())
      s_current_controller_label = "Control";
  }

  // ── Sub-phase 1.5: en-route sector / FIR frequency change ────────────
  // Runs unconditionally — even while the pilot is on an APP/TRACON frequency.
  // In France (and much of Europe) pilots stay on the Approach frequency for
  // the entire cruise leg; there is no separate "Centre" they check in with.
  // The frequency guard below would suppress this check entirely otherwise.
  // Suppressed once descent clearance issued (Approach then takes over).
  if (!s_enroute_descent_issued && airspace_db::enabled()) {
    s_enroute_sector_check_sec -= dt;
    if (s_enroute_sector_check_sec <= 0.0f) {
      s_enroute_sector_check_sec = 15.0f;

      const airspace_db::Controller *best = sector_picker::pick_next(
          ctx.enclosing_airspaces, s_enroute_visited_sector_freqs);

      if (best) {
        uint32_t new_freq_khz = best->freqs_khz.front();
        if (s_enroute_sector_freq_khz == 0) {
          // First check: Phase 2/3 already gave the pilot the correct APP freq
          // (from apt.dat, e.g. 121.205). Seed silently so the first real sector
          // change is announced, not the initial baseline.
          s_enroute_sector_freq_khz = new_freq_khz;
          // Log all enclosing sectors at seed time so unexpected handoffs are diagnosable.
          std::string all_enc;
          for (const auto *ec : ctx.enclosing_airspaces) {
            if (!all_enc.empty()) all_enc += ", ";
            all_enc += ec->name;
          }
          logging::info("IFR en-route: sector baseline %s %.3f MHz floor=%dft (silent; enclosing=[%s])",
                        best->name.c_str(),
                        static_cast<float>(new_freq_khz) / 1000.0f,
                        best->floor_ft,
                        all_enc.c_str());
        } else if (new_freq_khz != s_enroute_sector_freq_khz) {
          // Sector changed — issue handoff and wait for pilot to check in.
          // Record the OUTGOING freq so this sector cannot be re-elected
          // later in the same cruise phase (block backward handoff).
          s_enroute_visited_sector_freqs.push_back(s_enroute_sector_freq_khz);
          s_enroute_sector_freq_khz = new_freq_khz;
          std::string new_label = controller_label_for(best);
          // Defer label switch — see s_pending_controller_label comment.
          // Previous sector controller stays as speaker until the pilot
          // actually moves to the new frequency.
          s_pending_controller_label = new_label;
          float new_freq_mhz = static_cast<float>(new_freq_khz) / 1000.0f;
          s_pending_handoff_freq_mhz = new_freq_mhz;
          // When the new sector is a TRACON (Approach controller), update
          // the "active approach freq" gate so the check-in handler only
          // fires when the pilot actually switches. Without this, the
          // fallback (s_enroute_approach_freq_mhz < 100 → accept any freq)
          // makes the check-in fire on the OLD frequency the pilot never
          // left.
          if (best->role == airspace_db::ControllerRole::TRACON)
            s_enroute_approach_freq_mhz = new_freq_mhz;
          // Suppress announcement if the pilot is already on the new
          // sector's frequency. Common after the aircraft re-enters an
          // earlier sector (e.g. drifts out of Melun TMA back into Paris
          // CTR) whose frequency happens to match the pilot's active COM:
          // no switch is needed, so the "contact X on Y" phrase would be
          // spoken while the pilot is already there.
          const float active_com_now =
              (ctx.active_com == 2) ? ctx.com2_freq_mhz : ctx.com1_freq_mhz;
          const bool already_on_new_freq =
              std::fabs(active_com_now - new_freq_mhz) < 0.005f;
          if (already_on_new_freq) {
            s_sector_checkin_pending = false;
            logging::info("IFR en-route: sector change -> %s %.3f MHz (pilot already on freq -- silent)",
                          new_label.c_str(), new_freq_mhz);
            return false;  // no announcement, no state change
          }
          s_sector_checkin_pending = true;
          if (out_text) {
            const std::string &cs_s = atc_state_machine::session_callsign();
            const std::string &cs_callsign =
                cs_s.empty() ? settings::pilot_callsign() : cs_s;
            char buf[160];
            std::snprintf(buf, sizeof(buf), "%s, contact %s on %.3f.",
                          cs_callsign.c_str(), new_label.c_str(), new_freq_mhz);
            *out_text = buf;
          }
          std::string all_enc2;
          for (const auto *ec : ctx.enclosing_airspaces) {
            if (!all_enc2.empty()) all_enc2 += ", ";
            all_enc2 += ec->name;
          }
          logging::info("IFR en-route: sector change -> %s %.3f MHz floor=%dft (enclosing=[%s])",
                        new_label.c_str(), new_freq_mhz, best->floor_ft, all_enc2.c_str());
          rb(true);
          return true;
        }
      }
    }
  }

  // Don't issue proactive messages (direct-to, step-up, pre-TOD, etc.) while
  // the pilot is still on the departure/approach frequency — they haven't
  // checked in on Centre yet.  Timer only counts while on Centre so the
  // 90-120 s delays are relative to actual check-in, not the handoff issue.
  using FT = xplane_context::FrequencyType;
  if (ctx.frequency_type != FT::UNKNOWN)
    return false;

  s_enroute_timer += dt;
  s_enroute_deviation_cooldown_sec =
      std::max(0.0f, s_enroute_deviation_cooldown_sec - dt);

  // Block all proactive messages until the pilot has checked in on the new
  // sector frequency.  The flag is cleared in process_transcript() the moment
  // the pilot transmits while their active COM matches s_pending_handoff_freq_mhz.
  if (s_sector_checkin_pending)
    return false;

  // Fallback: only used when the aircraft entered IFR_ENROUTE_CRUISE without
  // going through the normal SID-climb sequence (e.g. loaded mid-flight,
  // resumed session, or skipped departure phase). The step1 seed above covers
  // the normal case, so this only fires in the edge-case where neither
  // step1 nor cruise clearance was recorded — and even then the deviation
  // warning is suppressed until after the 60-second grace period.
  if (s_enroute_cleared_alt_ft == 0 && ctx.ifr_cruise_alt_ft > 0) {
    s_enroute_cleared_alt_ft = round_to_fl(ctx.ifr_cruise_alt_ft) * 100;
    logging::info("IFR en-route: cleared alt seeded from OFP cruise (%d ft)",
                  s_enroute_cleared_alt_ft);
  }

  // Step-up: proactively issue cruise FL climb if ATC handed off to Centre
  // while the aircraft is still below cruise altitude (e.g. TMA exit fired the
  // handoff before cruise clearance reached the pilot). Fires once, ≥30 s
  // after Centre check-in, only when cleared_alt < cruise_alt.
  if (!s_cruise_stepup_issued && s_enroute_timer >= 30.0f &&
      s_enroute_cleared_alt_ft > 0 && ctx.ifr_cruise_alt_ft > 0 &&
      ctx.ifr_cruise_alt_ft > s_enroute_cleared_alt_ft + 1000) {
    s_cruise_stepup_issued = true;
    int fl = round_to_fl(ctx.ifr_cruise_alt_ft);
    s_enroute_cleared_alt_ft = fl * 100;
    if (out_text) {
      const std::string &cs2 = atc_state_machine::session_callsign();
      const std::string &callsign2 =
          cs2.empty() ? settings::pilot_callsign() : cs2;
      char buf[64];
      std::snprintf(buf, sizeof(buf), "%s, climb flight level %d.",
                    callsign2.c_str(), fl);
      *out_text = buf;
    }
    logging::info("IFR en-route: step-up FL%d (cleared %d ft < cruise %d ft)",
                  fl, s_enroute_cleared_alt_ft / 100,
                  ctx.ifr_cruise_alt_ft / 100);
    rb(true);
    return true;
  }

  const auto &defaults = flight_phase::get_ifr_defaults();
  const std::string &cs = atc_state_machine::session_callsign();
  const std::string &callsign = cs.empty() ? settings::pilot_callsign() : cs;

  // ── Sub-phase 1.7: filed FL step changes ───────────────────────────────
  // When the FPL contains explicit "<FIX>/N<spd>F<FL>" step markers,
  // treat them as the authoritative ATC clearance script — one clearance
  // per marker, no clearances at intermediate airway fixes.  This matches
  // real ATC behaviour: controllers step aircraft down at the filed step
  // points, not at every waypoint the flight-planner computed.
  // Falls back to the navlog-driven walker below when no /F markers are
  // filed (single-cruise-FL flights).
  if (!s_enroute_descent_issued && s_enroute_cleared_alt_ft > 0) {
    auto ofp_step = simbrief_ofp::get();
    if (ofp_step.valid && !ofp_step.route_steps.empty() &&
        !ofp_step.navlog.empty()) {
      float gs_step = ctx.groundspeed_kts > 80.0f ? ctx.groundspeed_kts : 250.0f;
      const int steps_sz = static_cast<int>(ofp_step.route_steps.size());

      // Advance past any filed step whose fix is behind the aircraft or
      // absent from the navlog (unlikely — sanity guard).
      auto find_nav_pos = [&](const std::string &ident,
                              double *lat, double *lon) -> bool {
        for (const auto &f : ofp_step.navlog) {
          if (f.ident == ident) {
            *lat = f.lat; *lon = f.lon; return true;
          }
        }
        return false;
      };

      while (s_route_step_idx < steps_sz) {
        const auto &step = ofp_step.route_steps[s_route_step_idx];
        double lat = 0.0, lon = 0.0;
        if (!find_nav_pos(step.ident, &lat, &lon)) {
          ++s_route_step_idx;
          continue;
        }
        double dist_nm = traffic_geometry::distance_nm(
            ctx.latitude, ctx.longitude, lat, lon);
        // Prefer the ROUTED (leg-by-leg) distance when this step fix is on the
        // tracked route: sum the filed legs from the aircraft through the
        // intermediate fixes to it, instead of cutting straight across a dogleg, so
        // the enroute step / pre-TOD alert fires at the true distance-to-fly to the
        // STAR entry (user 2026-07-19). Straight-line stays the fallback for a fix
        // not (yet) in s_route_fixes. routed_distance_to_fix_idx carries the
        // skip-behind and (0,0)-skip guards, so a bad fix can't inflate the sum.
        for (int ri = std::max(0, s_route_fix_idx);
             ri < static_cast<int>(s_route_fixes.size()); ++ri) {
          if (s_route_fixes[ri].ident == step.ident) {
            const double rd = routed_distance_to_fix_idx(ctx, ri);
            if (rd > 0.0) {
              // [dbg dist] throttled routed-vs-straight readout so the leg-by-leg
              // sum can be verified in-flight (user 2026-07-19). Remove after
              // validation.
              static float s_dbg_dist_sec = 0.0f;
              s_dbg_dist_sec -= dt;
              if (s_dbg_dist_sec <= 0.0f) {
                s_dbg_dist_sec = 15.0f;
                if (settings::debug_logging())
                  logging::info("[dbg dist] to %s: routed %.1f NM (leg-by-leg) vs "
                                "straight %.1f NM (route idx %d, tracker %d)",
                                step.ident.c_str(), rd, dist_nm, ri, s_route_fix_idx);
              }
              dist_nm = rd;
            }
            break;
          }
        }
        if (dist_nm < 2.0) {
          ++s_route_step_idx; // already at/past this filed step
          continue;
        }
        double dlat = lat - ctx.latitude;
        double dlon = (lon - ctx.longitude) *
                      std::cos(ctx.latitude * M_PI / 180.0);
        double bdeg = std::atan2(dlon, dlat) * 180.0 / M_PI;
        double diff = std::abs(bdeg - static_cast<double>(ctx.heading_true));
        if (diff > 180.0) diff = 360.0 - diff;
        if (diff > 90.0) {
          ++s_route_step_idx; // step fix is behind us
          continue;
        }

        // Ahead. Compare cleared FL against filed FL.
        const int step_target_ft = step.cruise_fl * 100;
        const int step_diff = step_target_ft - s_enroute_cleared_alt_ft;
        if (std::abs(step_diff) < 500) {
          ++s_route_step_idx; // no meaningful change at this filed step
          break;
        }
        float alt_comp = static_cast<float>(std::abs(step_diff)) /
                         static_cast<float>(kDescentSlopeFtPerNm);
        float alert_nm_step =
            std::max(15.0f, std::min(80.0f, alt_comp + gs_step / 20.0f));
        if (dist_nm > static_cast<double>(alert_nm_step))
          break; // wait until closer

        ++s_route_step_idx;
        s_enroute_cleared_alt_ft = step_target_ft;
        if (out_text) {
          char buf[128];
          const int ta = ctx.transition_alt_ft > 0 ? ctx.transition_alt_ft : 5000;
          const int tl = compute_tl_ft(ta, ctx.qnh_hpa);
          const bool below_tl = s_enroute_cleared_alt_ft < tl;
          if (below_tl) {
            std::snprintf(buf, sizeof(buf),
                          step_diff > 0 ? "%s, climb to %d feet, QNH %d."
                                        : "%s, descend to %d feet, QNH %d.",
                          callsign.c_str(), s_enroute_cleared_alt_ft, ctx.qnh_hpa);
            s_qnh_stated = true;
          } else {
            std::snprintf(buf, sizeof(buf),
                          step_diff > 0 ? "%s, climb flight level %d."
                                        : "%s, descend flight level %d.",
                          callsign.c_str(), step.cruise_fl);
          }
          *out_text = buf;
        }
        {
          const int ta = ctx.transition_alt_ft > 0 ? ctx.transition_alt_ft : 5000;
          const int tl = compute_tl_ft(ta, ctx.qnh_hpa);
          logging::info(
              "IFR en-route: filed step %s FL%d (fix %s, dist %.0f NM, TL=%d)",
              step_diff > 0 ? "climb" : "descent", step.cruise_fl,
              step.ident.c_str(), dist_nm, tl / 100);
        }
        s_enroute_alt_warn_cooldown = 180.0f;
        s_enroute_verify_query_sent = false;
        s_enroute_verify_target_ft = s_enroute_cleared_alt_ft;
        rb(true);
        return true;
      }
      // Filed-steps path handled (or waited); do not fall through.
    }
    // Deliberately no `else` fallback: when the OFP has no explicit
    // "<FIX>/N<spd>F<FL>" step markers we do NOT synthesise clearances
    // from SimBrief's per-fix altitude column.  Those values are
    // SimBrief's own vertical-profile guesses (mid-climb, mid-descent,
    // pre-computed TOD stepping) and treating them as ATC clearances
    // produced false step-downs like the LIMF->LFLP KUKEV=19600ft
    // artifact seen 2026-07-09.  Enroute FL stays at the cruise seed
    // until:
    //   - the pilot requests descent (poll_enroute sub-phase 2), OR
    //   - the pre-TOD prompt fires and build_descent_clearance takes
    //     over the descent phase.
  }

  // One-time initialisation of pseudo-random direct-to delay (90-120 s).
  if (s_enroute_direct_delay_sec < 1.0f) {
    unsigned hash = 0;
    for (char c : callsign)
      hash = hash * 31u + static_cast<unsigned char>(c);
    s_enroute_direct_delay_sec =
        90.0f + static_cast<float>(hash % 31u); // [90, 120]
  }

  // ── Sub-phase 1: en-route direct-to shortcut ─────────────────────────
  // Fires once, ~90-120 s after Centre check-in. Requires navlog with at
  // least one non-SID/STAR fix still ahead.
  if (!s_enroute_direct_issued &&
      s_enroute_timer >= s_enroute_direct_delay_sec) {
    s_enroute_direct_issued = true;
    auto ofp = simbrief_ofp::get();
    if (ofp.valid && !ofp.navlog.empty()) {
      std::string fix = pick_direct_fix(ctx, ofp.navlog);
      if (!fix.empty()) {
        // A direct-to shortcut supersedes any outstanding clearance
        // readback (e.g. descent clearance with runway field still pending).
        // Cancel it so the pilot isn't stuck reading back "runway 07" for
        // a "direct DJL, when able" transmission.
        atc_state_machine::cancel_readback();
        if (out_text) {
          char buf[128];
          std::snprintf(buf, sizeof(buf), "%s, direct %s, when able.",
                        callsign.c_str(), fix.c_str());
          *out_text = buf;
        }
        logging::info("IFR en-route: direct %s shortcut", fix.c_str());
        return true;
      }
    }
    // No navlog or no fix — don't speak, but mark issued so we don't retry.
  }

  // ── Sub-phase 2: pre-TOD prompt → pilot confirms → descent clearance ────
  //
  // Normal flow:
  //   a) At tod_nm + 15 NM: ATC prompts "advise when ready to descend."
  //   b) Pilot replies REQUEST_DESCENT → ATC issues descent + STAR + approach.
  //   c) If pilot requests descent BEFORE the prompt: skip prompt, issue directly.
  //
  // Fallbacks (no OFP or pilot never responds):
  //   - Actual TOD reached (dist <= tod_nm) without pilot response → issue directly.
  //   - 25 min safety net → issue directly (no OFP / unexpected corner cases).
  if (!s_enroute_descent_issued) {
    // Pilot requested descent.  First try to match against a forward navlog
    // step — if the requested FL is a planned en-route altitude change, issue
    // a simple step clearance (no STAR/runway) and stay in ENROUTE_CRUISE.
    // Only fall through to build_descent_clearance() (TOD/approach logic)
    // when no navlog step matches (e.g. pilot responds to the pre-TOD prompt).
    if (s_pilot_requested_descent) {
      s_pilot_requested_descent = false;
      int req_ft = s_pilot_requested_fl_ft;
      s_pilot_requested_fl_ft = 0;

      bool issued_step = false;
      if (req_ft > 0 && s_enroute_cleared_alt_ft > 0) {
        auto ofp_req = simbrief_ofp::get();
        if (ofp_req.valid && !ofp_req.navlog.empty()) {
          const int navlog_sz_req = static_cast<int>(ofp_req.navlog.size());
          int idx = s_navlog_alt_step_idx;
          while (idx < navlog_sz_req) {
            const auto &fix = ofp_req.navlog[idx];
            if (fix.is_sid_star || fix.ident.empty() || fix.alt_ft <= 0) {
              ++idx; continue;
            }
            double d = traffic_geometry::distance_nm(
                ctx.latitude, ctx.longitude, fix.lat, fix.lon);
            if (d < 2.0) { ++idx; continue; }
            double dlat2 = fix.lat - ctx.latitude;
            double dlon2 = (fix.lon - ctx.longitude) *
                           std::cos(ctx.latitude * M_PI / 180.0);
            double bdeg2 = std::atan2(dlon2, dlat2) * 180.0 / M_PI;
            double diff2 = std::abs(bdeg2 - ctx.heading_true);
            if (diff2 > 180.0) diff2 = 360.0 - diff2;
            if (diff2 > 90.0) { ++idx; continue; }
            // This fix is ahead.  Accept if its planned FL is within 500 ft of
            // the pilot's request and the step is significant (>= 500 ft change).
            int step_diff = fix.alt_ft - s_enroute_cleared_alt_ft;
            if (std::abs(step_diff) >= 500 && std::abs(fix.alt_ft - req_ft) <= 500) {
              int step_fl = round_to_fl(fix.alt_ft);
              s_enroute_cleared_alt_ft = step_fl * 100;
              s_navlog_alt_step_idx = idx + 1;
              if (out_text) {
                char buf[128];
                const int ta2  = ctx.transition_alt_ft > 0 ? ctx.transition_alt_ft : 5000;
                const int tl2  = compute_tl_ft(ta2, ctx.qnh_hpa);
                const bool below_tl2 = s_enroute_cleared_alt_ft < tl2;
                if (below_tl2) {
                  std::snprintf(buf, sizeof(buf),
                                step_diff > 0 ? "%s, climb to %d feet, QNH %d."
                                              : "%s, descend to %d feet, QNH %d.",
                                callsign.c_str(), s_enroute_cleared_alt_ft, ctx.qnh_hpa);
                } else {
                  std::snprintf(buf, sizeof(buf),
                                step_diff > 0 ? "%s, climb flight level %d."
                                              : "%s, descend flight level %d.",
                                callsign.c_str(), step_fl);
                }
                *out_text = buf;
              }
              {
                const int ta2 = ctx.transition_alt_ft > 0 ? ctx.transition_alt_ft : 5000;
                const int tl2 = compute_tl_ft(ta2, ctx.qnh_hpa);
                logging::info(
                    "IFR en-route: pilot requested FL%d matched navlog step %s %s%d (dist %.0f NM, TL=%d)",
                    req_ft / 100, fix.ident.c_str(),
                    s_enroute_cleared_alt_ft < tl2 ? "ALT" : "FL",
                    step_fl, d, tl2 / 100);
              }
              s_enroute_alt_warn_cooldown = 180.0f;
              s_enroute_verify_query_sent = false;
              s_enroute_verify_target_ft  = s_enroute_cleared_alt_ft;
              rb(true);
              issued_step = true;
            }
            break;
          }
        }
      }
      if (!issued_step) {
        if (build_descent_clearance(ctx, callsign, defaults, out_text)) {
          rb(true);
          return true;
        }
      } else {
        return true;
      }
    }

    // Compute distance to STAR entry fix and alert threshold.
    //
    // Alert distance = altitude component + speed component:
    //   altitude:  cruise_alt_ft / 1200  (FL350 → 29 NM, FL195 → 16 NM)
    //   speed:     groundspeed_kts / 30   (480 kts → 16 NM, 180 kts → 6 NM)
    //   result:    clamped [15, 80] NM
    // Examples: FL350 at 300 kts → 39 NM, FL195 at 180 kts → 22 NM.
    //
    // Reference fix: only use a CIFP-confirmed STAR entry (star_name non-empty).
    // If no CIFP match (e.g., last FPL fix is not a LFMN STAR entry), fall back
    // to destination fix so the prompt does not fire prematurely.
    double dist_nm = 1e9;
    float alert_nm = 25.0f; // fallback when groundspeed unavailable
    float tod_alt_component = 0.0f; // hoisted for pre-TOD gate
    {
      auto ofp = simbrief_ofp::get();
      if (ofp.valid && !ofp.navlog.empty()) {
        int cruise_ref = s_enroute_cleared_alt_ft > 0 ? s_enroute_cleared_alt_ft
                                                       : ctx.ifr_cruise_alt_ft;
        float gs = ctx.groundspeed_kts > 80.0f ? ctx.groundspeed_kts : 250.0f;
        // Descent target mirrors build_descent_clearance so the alert fires at
        // exactly the distance needed for a comfortable 3-degree descent.
        // Cap below cruise so a low-FL flight doesn't compute a zero-altitude delta.
        // Keep this formula IDENTICAL to build_descent_clearance's star_alt_ft
        // (proportional cruise * 0.66) or the pre-TOD alert distance will be
        // computed for a different target than what actually gets cleared.
        int descent_target =
            std::max(defaults.star_entry_alt_ft,
                     (cruise_ref * 66 / 100) / 1000 * 1000);
        if (descent_target >= cruise_ref)
          descent_target = (cruise_ref / 1000 - 1) * 1000;
        // One CIFP lookup covers both the descent-target override and the
        // reference-fix distance — avoids calling find_star_entry() twice
        // per frame which floods debug logs.
        StarEntryResult se;
        bool se_valid = find_star_entry(ctx.cifp_dir, ofp, se);
        if (se_valid && se.entry_alt_ft > 0 && se.entry_alt_ft < cruise_ref)
          descent_target = se.entry_alt_ft;
        // alert = NM needed for 3-deg descent  +  clearance exchange buffer (gs/20)
        float alt_component =
            static_cast<float>(std::max(0, cruise_ref - descent_target)) / 300.0f;
        tod_alt_component = alt_component;
        float spd_component = gs / 20.0f;
        alert_nm = std::max(15.0f, std::min(80.0f, alt_component + spd_component));

        if (se_valid && !se.star_name.empty()) {
          // CIFP-confirmed STAR entry: measure to that fix.
          dist_nm = traffic_geometry::distance_nm(
              ctx.latitude, ctx.longitude, se.lat, se.lon);
        } else {
          // No CIFP match — measure to destination so prompt fires at correct time.
          const auto &dest_fix = ofp.navlog.back();
          dist_nm = traffic_geometry::distance_nm(
              ctx.latitude, ctx.longitude, dest_fix.lat, dest_fix.lon);
        }
      }
    }

    // Pre-TOD clearance: fire once, ~5 min before STAR entry.
    // In IFR controlled airspace ATC gives the STAR + expected approach
    // proactively before TOD — no "advise when ready" intermediate step.
    // Suppressed while a sector-handoff readback is pending (pilot must
    // acknowledge the frequency first) and when the cleared altitude is
    // already within 3000 ft of the approach target (navlog steps covered
    // the descent; forced clearance at 10 NM handles the final segment).
    // tod_alt_component threshold: minimum 3 NM so the alert fires even when
    // navlog step-downs already brought the aircraft to a low altitude
    // (e.g. 4500 ft heading to an IAF at 2700 ft → only 6 NM of descent needed,
    // which was previously below the 10 NM guard and suppressed the clearance).
    if (!s_enroute_descent_prompt_issued &&
        dist_nm <= static_cast<double>(alert_nm) &&
        tod_alt_component >= 3.0f &&
        !atc_state_machine::is_readback_pending()) {
      s_enroute_descent_prompt_issued = true;
      logging::info("IFR en-route: pre-TOD descent (%.1f NM to STAR entry, alert=%.0f NM)",
                    dist_nm, alert_nm);
      if (build_descent_clearance(ctx, callsign, defaults, out_text)) {
        rb(true);
        return true;
      }
      // build_descent_clearance returned false (no CIFP/OFP data yet) —
      // fall back to the advisory prompt so the pilot knows descent is coming.
      if (out_text) {
        char buf[120];
        std::snprintf(buf, sizeof(buf), "%s, expect descent shortly.",
                      callsign.c_str());
        *out_text = buf;
      }
      return true;
    }

    // Forced clearance when aircraft is very close to STAR entry (40% of alert
    // distance, but at most 10 NM hard cutoff).  Fires regardless of whether the
    // pre-TOD prompt was issued — navlog steps may have already stepped the aircraft
    // down without ever setting s_enroute_descent_prompt_issued, so the prompt gate
    // must not block the final approach clearance.
    if (dist_nm <= static_cast<double>(std::min(alert_nm * 0.4f, 10.0f))) {
      logging::info("IFR en-route: forced descent (%.1f NM, prompt_issued=%d)",
                    dist_nm, s_enroute_descent_prompt_issued ? 1 : 0);
      if (build_descent_clearance(ctx, callsign, defaults, out_text)) {
        rb(true);
        return true;
      }
    }

    // Safety net: 25 min elapsed with no OFP or no TOD ever computed.
    if (s_enroute_timer > 25.0f * 60.0f) {
      logging::info("IFR en-route: 25 min safety net -- issuing descent");
      if (build_descent_clearance(ctx, callsign, defaults, out_text)) {
        rb(true);
        return true;
      }
    }
  }

  // Sub-phase 2.5 (CTA boundary → Approach handoff) has moved to poll_descent().
  // poll_enroute() only runs in IFR_ENROUTE_CRUISE; poll_descent() takes over
  // in IFR_DESCENT once build_descent_clearance() has fired.

  // ── Sub-phase 2.4: altitude verification query (soft) ──────────────────
  // Fires 45-100 s after a descent/climb clearance when the aircraft has
  // NOT started moving toward the target (|vs| < 200 fpm) and is still
  // >= 500 ft off the assigned altitude but under the hard-deviation
  // threshold. Standard EUROCONTROL courtesy prompt used before the harder
  // "check altitude" warning — catches missed clearances (STT drop, pilot
  // distraction) before they escalate. Fires once per clearance.
  s_enroute_alt_warn_cooldown =
      std::max(0.0f, s_enroute_alt_warn_cooldown - dt);
  if (!s_enroute_verify_query_sent && s_enroute_verify_target_ft > 0 &&
      s_enroute_alt_warn_cooldown > 80.0f &&
      s_enroute_alt_warn_cooldown < 135.0f &&
      !atc_state_machine::is_readback_pending()) {
    const int ta_v = (ctx.transition_alt_ft > 0) ? ctx.transition_alt_ft : 5000;
    const bool tgt_is_fl = (s_enroute_verify_target_ft > ta_v);
    const int actual_v = tgt_is_fl ? static_cast<int>(ctx.pressure_alt_ft)
                                    : static_cast<int>(ctx.altitude_ft_msl);
    const int diff_v = actual_v - s_enroute_verify_target_ft;
    if (std::abs(diff_v) >= 500 && std::abs(diff_v) < 800 &&
        std::abs(ctx.vertical_speed_fpm) < 200.0f) {
      s_enroute_verify_query_sent = true;
      if (out_text) {
        char buf[160];
        const char *verb = (diff_v > 0) ? "descending" : "climbing";
        if (tgt_is_fl)
          std::snprintf(buf, sizeof(buf),
                        "%s, confirm %s flight level %d.",
                        callsign.c_str(), verb,
                        s_enroute_verify_target_ft / 100);
        else
          std::snprintf(buf, sizeof(buf),
                        "%s, confirm %s %d feet.",
                        callsign.c_str(), verb,
                        s_enroute_verify_target_ft);
        *out_text = buf;
      }
      logging::info("IFR en-route: verify query -> target=%d diff=%+d VS=%.0f fpm",
                    s_enroute_verify_target_ft, diff_v, ctx.vertical_speed_fpm);
      return true;
    }
  }

  // ── Sub-phase 2.5: cruise altitude deviation warning ──────────────────
  // RVSM (FL290+): threshold 200 ft. Below FL290: 300 ft (ICAO standard).
  // 2-minute cooldown between warnings. Grace period of 60 s after check-in
  // so the aircraft has time to level off before monitoring begins.
  // Suppressed once descent clearance has been issued.
  if (!s_enroute_descent_issued && s_enroute_cleared_alt_ft > 0 &&
      s_enroute_timer >= 60.0f && s_enroute_alt_warn_cooldown <= 0.0f &&
      !atc_state_machine::is_readback_pending()) {
    // FL clearances (above TA) use pressure altitude (1013.25 hPa reference);
    // feet clearances (below TA) use QNH MSL altitude. Same rule as the pilot's
    // altimeter — comparing pressure vs QNH would produce a phantom deviation
    // equal to the QNH-standard offset (~740 ft at QNH 1025).
    const int ta_dev = (ctx.transition_alt_ft > 0) ? ctx.transition_alt_ft : 5000;
    const bool cleared_is_fl = (s_enroute_cleared_alt_ft > ta_dev);
    const int actual_ft = cleared_is_fl
                              ? static_cast<int>(ctx.pressure_alt_ft)
                              : static_cast<int>(ctx.altitude_ft_msl);
    int deviation_ft = actual_ft - s_enroute_cleared_alt_ft;
    int threshold_ft = (s_enroute_cleared_alt_ft >= 29000) ? 200 : 300;
    if (std::abs(deviation_ft) >= threshold_ft) {
      s_enroute_alt_warn_cooldown = 120.0f;
      if (out_text) {
        char buf[160];
        if (cleared_is_fl) {
          std::snprintf(
              buf, sizeof(buf),
              "%s, check altitude, you are %d feet %s assigned flight level %d.",
              callsign.c_str(), std::abs(deviation_ft),
              deviation_ft > 0 ? "above" : "below",
              s_enroute_cleared_alt_ft / 100);
        } else {
          std::snprintf(
              buf, sizeof(buf),
              "%s, check altitude, you are %d feet %s assigned altitude %d feet.",
              callsign.c_str(), std::abs(deviation_ft),
              deviation_ft > 0 ? "above" : "below",
              s_enroute_cleared_alt_ft);
        }
        *out_text = buf;
      }
      logging::info("IFR en-route: altitude deviation %+d ft from %s%d (ref=%s)",
                    deviation_ft, cleared_is_fl ? "FL" : "",
                    cleared_is_fl ? s_enroute_cleared_alt_ft / 100
                                  : s_enroute_cleared_alt_ft,
                    cleared_is_fl ? "pressure" : "MSL");
      return true;
    }
  }

  // ── Sub-phase 3: cross-track deviation warning ────────────────────────
  // Fires when the aircraft is more than 5 NM off the filed route.
  // 3-minute cooldown between warnings.
  // Suppressed after a direct-to has been issued: the original navlog legs are
  // superseded by the direct routing ATC just cleared the aircraft on.
  // Only warn after the direct-to window has opened. Sub-phase 1 runs first
  // in this function so direct-to fires (and sets s_enroute_direct_issued) at
  // the same threshold — this guard therefore only triggers when the direct-to
  // itself was suppressed (e.g. no OFP fix ahead), preventing early false
  // positives while still on the SID.
  if (!s_enroute_direct_issued && s_enroute_deviation_cooldown_sec <= 0.0f &&
      s_enroute_timer >= s_enroute_direct_delay_sec) {
    auto ofp = simbrief_ofp::get();
    if (ofp.valid && ofp.navlog.size() >= 2) {
      double xt_nm = std::abs(min_cross_track_nm(ctx, ofp.navlog));
      if (xt_nm > 5.0) {
        s_enroute_deviation_cooldown_sec = 180.0f;
        if (out_text) {
          char buf[160];
          std::snprintf(buf, sizeof(buf),
                        "%s, confirm routing, you appear off track.",
                        callsign.c_str());
          *out_text = buf;
        }
        logging::info("IFR en-route: cross-track deviation %.1f NM", xt_nm);
        return true;
      }
    }
  }

  // DirectMonitor (en-route course): "confirm direct <fix>" when the aircraft is
  // off the leg to the ACTIVE next fix. Lowest priority -- only reached when no
  // higher event fired this frame. WIRED with placeholder gates (25 deg / 3 NM
  // guard / 180 s cooldown); firing conditions to be tuned. Distinct from the
  // coarse 5 NM cross-track check above (heading-vs-bearing, not offset).
  s_enroute_course_cooldown = std::max(0.0f, s_enroute_course_cooldown - dt);
  if (s_enroute_course_cooldown <= 0.0f) {
    const CourseCheck cc = check_course(ctx, 25.0);
    if (cc.valid && cc.off_course && cc.dist_nm > 3.0) {
      s_enroute_course_cooldown = 180.0f;
      if (out_text) {
        char buf[176];
        std::snprintf(buf, sizeof(buf),
                      "%s, confirm direct %s, you appear tracking heading %.0f, "
                      "expected %.0f.",
                      callsign.c_str(), cc.ident.c_str(),
                      static_cast<double>(ctx.heading_true), cc.bearing_deg);
        *out_text = buf;
      }
      logging::info("IFR en-route: course deviation hdg %.0f vs brg %.0f to %s (diff %.0f)",
                    static_cast<double>(ctx.heading_true), cc.bearing_deg,
                    cc.ident.c_str(), cc.diff_deg);
      rb(false);
      return true;
    }
  }

  return false;
}

// ── poll_descent ──────────────────────────────────────────────────────────────
// Runs in IFR_DESCENT state (after build_descent_clearance fired).
// Sole responsibility: detect TMA/CTR entry and hand off to Approach.
// Uses the same statics as poll_enroute()'s former sub-phase 2.5; they are
// reset by poll_enroute()'s guard block when ENROUTE_CRUISE → DESCENT transitions.
// ACC/FIR sector handoff for the descent + arrival phases (both are "under
// ACC", before the Approach TRACON handoff). Advances CTR sectors only, e.g.
// Milan -> France (UIR, >FL195) -> Marseille (FIR, <FL195) at the Italy/France
// boundary ~6 NM after BANKO. Validated against atc.dat (LIMF->LFLP): MILAN
// 118.670 -> FRANCE 118.030 -> MARSEILLE 119.755. Returns true when it issues a
// "contact X on Y" handoff. Uses dedicated s_acc_* statics (the enroute set is
// zeroed outside cruise). TRACON/Approach handoffs are NOT done here.
static bool poll_acc_sector_change(const xplane_context::XPlaneContext &ctx,
                                   float dt, std::string *out_text,
                                   bool *out_requires_readback) {
  auto rb = [&](bool v) { if (out_requires_readback) *out_requires_readback = v; };
  if (!airspace_db::enabled())
    return false;
  s_acc_sector_check_sec -= dt;
  if (s_acc_sector_check_sec > 0.0f)
    return false;
  s_acc_sector_check_sec = 15.0f;

  // Controlling sector = the innermost / most-terminal enclosing volume,
  // boundary-driven. An openair TMA (GENEVA / CHAMBERY) OVERRIDES the broad
  // atc.dat CTR when the aircraft is inside it (lateral + altitude-gated) -- so
  // TMA handoffs fire when you CROSS the boundary (enter the TMA, or descend into
  // a lower one such as GENEVA -> CHAMBERY below FL095), not at a route point
  // (LFLP 2026-07-15). CTR sectors (Milan / France / Marseille) are the fallback
  // when outside any terminal TMA.
  // The DESTINATION's terminal approach handoff (Chambery for LFLP) is owned by
  // poll_arrival/build_approach_handoff. Once the aircraft is inside the destination
  // terminal TMA, defer -- otherwise poll_acc_sector_change ALSO resolves that TMA
  // and the pilot gets a DOUBLE "contact <dest> Approach" (LFLP 2026-07-18).
  // Non-destination TMAs (Geneva) and enroute ACC sectors are unaffected
  // (on_destination_terminal is false there). Guarded on dest known so the helper's
  // permissive true-when-unknown never disables enroute handoffs.
  if (!s_assigned_dest_icao.empty() && openair_db::ready() &&
      on_destination_terminal(ctx))
    return false;

  std::string new_label;
  float new_mhz = 0.0f;
  openair_db::AirspaceEntry enc; // hoisted: the atc.dat fallback reuses its NAME
  if (openair_db::ready()) {
    enc = openair_db::find_enclosing(ctx.latitude, ctx.longitude,
                                     openair_alt(ctx));
    // openair geometry drives the controller for BOTH terminal TMAs (Geneva /
    // Chambery) AND enroute sub-sectors / cross-border delegation (Milan
    // sub-CTAs, LFFF->LSAS) via the unified resolver. The atc.dat CTR picker
    // below is the fallback when no openair sector resolves a frequency.
    std::string lbl;
    float f = 0.0f;
    if (resolve_sector_controller(enc, /*terminal=*/false, &lbl, &f)) {
      new_label = lbl;
      new_mhz = f;
    }
  }
  if (new_mhz <= 0.0f) {
    // atc.dat CTR (ACC/FIR/UIR) sectors; drop oceanic / global-junk polygons.
    std::vector<const airspace_db::Controller *> ctrs;
    for (const auto *c : ctx.enclosing_airspaces) {
      if (!c || c->role != airspace_db::ControllerRole::CTR ||
          c->freqs_khz.empty())
        continue;
      if ((c->bbox_max_lat - c->bbox_min_lat) > 40.0 ||
          (c->bbox_max_lon - c->bbox_min_lon) > 40.0 ||
          c->name.find("OCEANIC") != std::string::npos)
        continue;
      ctrs.push_back(c);
    }
    const airspace_db::Controller *best =
        sector_picker::pick_next(ctrs, s_acc_visited_sector_freqs);
    if (!best)
      return false;
    new_mhz = static_cast<float>(best->freqs_khz.front()) / 1000.0f;
    // Label from the openair sector NAME when the aircraft is inside a named
    // enroute openair sector (e.g. MARSEILLE CTA) that had no own atc.dat freq --
    // take the atc.dat CTR FREQUENCY here but keep the accurate openair name for
    // the label, else a MARSEILLE sector is announced as the broad atc.dat "France"
    // (LFMN 2026-07-20). Falls back to the atc.dat label when openair is absent.
    const std::string oa_label = openair_sector_label(enc.name);
    new_label = oa_label.empty() ? controller_label_for(best) : oa_label;
  }
  if (new_mhz <= 0.0f)
    return false;

  const uint32_t new_freq_khz =
      static_cast<uint32_t>(std::lround(new_mhz * 1000.0));
  if (s_acc_sector_freq_khz == 0) {
    // Seed to the pilot's ACTUAL freq, NOT the resolved sector. If the aircraft is
    // already over the NEXT sector's airspace at the first poll (e.g. over the
    // SKYGUIDE/Switzerland volume above FL195 while still on Milan), seeding the
    // resolved sector SWALLOWS that handoff -- the Milan->Switzerland step was never
    // spoken (LFLP 2026-07-18; same class as the alpha-31 Chambery seed bug). Seeding
    // the pilot's freq makes the difference fire the real "contact <sector>" next poll.
    const float acom =
        (ctx.active_com == 2) ? ctx.com2_freq_mhz : ctx.com1_freq_mhz;
    const uint32_t acom_khz = static_cast<uint32_t>(std::lround(acom * 1000.0));
    s_acc_sector_freq_khz = (acom_khz > 0) ? acom_khz : new_freq_khz;
    logging::info(
        "IFR sector baseline: pilot on %.3f, sector resolves %s %.3f (silent)",
        acom, new_label.c_str(), new_mhz);
    return false;
  }
  if (new_freq_khz == s_acc_sector_freq_khz)
    return false;
  // Never hand back to a sector already left (prevents flicker at a boundary).
  for (uint32_t v : s_acc_visited_sector_freqs)
    if (v == new_freq_khz)
      return false;

  // Sector changed -> hand off; block backward handoff via the visited guard.
  s_acc_visited_sector_freqs.push_back(s_acc_sector_freq_khz);
  s_acc_sector_freq_khz = new_freq_khz;
  s_pending_controller_label = new_label;
  s_pending_handoff_freq_mhz = new_mhz;
  const float active_com_now =
      (ctx.active_com == 2) ? ctx.com2_freq_mhz : ctx.com1_freq_mhz;
  if (std::fabs(active_com_now - new_mhz) < 0.005f) {
    s_sector_checkin_pending = false;
    logging::info("IFR ACC: sector change -> %s %.3f MHz (already on freq -- silent)",
                  new_label.c_str(), new_mhz);
    return false;
  }
  s_sector_checkin_pending = true;
  if (out_text) {
    const std::string &cs = atc_state_machine::session_callsign();
    const std::string &callsign = cs.empty() ? settings::pilot_callsign() : cs;
    char buf[160];
    std::snprintf(buf, sizeof(buf), "%s, contact %s on %.3f.", callsign.c_str(),
                  new_label.c_str(), new_mhz);
    *out_text = buf;
  }
  logging::info("IFR ACC: sector change -> %s %.3f MHz", new_label.c_str(),
                new_mhz);
  rb(true);
  return true;
}

// Runs in IFR_DESCENT. Advances the phase to IFR_ARRIVAL when the aircraft
// reaches the STAR entry fix (procedure-anchored boundary). The real Approach
// handoff now lives in poll_arrival() -- poll_descent no longer transitions to
// IFR_APPROACH_CONTACT.
// Post-clearance altitude-compliance courtesy prompt for the DESCENT + ARRIVAL
// phases -- the gap between the TOD "descend FLxx" clearance and the approach
// phase, where poll_enroute's 2.4 verify and poll_approach's verify-descending
// do NOT run. Same shape as the en-route 2.4 verify: arm on the assigned level;
// if ~45 s later the aircraft is >=500 ft off it and NOT moving toward it
// (small |VS| or wrong direction), prompt "confirm descending/climbing <level>".
// Once per assignment. Firing gates (45 s grace, 500 ft, 200 fpm) are
// placeholders mirroring 2.4 -- tune in-sim. Runs every frame (real dt).
bool poll_altitude_compliance(const xplane_context::XPlaneContext &ctx, float dt,
                              std::string *out_text) {
  using AS = atc_state_machine::ATCState;
  const AS st = atc_state_machine::get_state();
  if (st != AS::IFR_DESCENT && st != AS::IFR_ARRIVAL &&
      st != AS::IFR_APPROACH_CONTACT && st != AS::IFR_APPROACH_DESCENT) {
    s_alt_comp_target_ft = 0;
    s_alt_comp_arm_sec = 0.0f;
    s_alt_comp_sent = false;
    return false;
  }
  // Post-clearance: the last assigned level (current_cleared_alt_ft) is SUPERSEDED
  // by the published approach profile -- the aircraft is meant to descend below it,
  // so this monitor would false-fire "confirm climbing <old level>" as it correctly
  // descends (LFLP: "confirm climbing FL090"/"6500"). Silence it once the approach
  // is cleared. The published-constraint compliance query ("confirm <alt> at <FIX>")
  // is a deliberate deferred follow-up (option A, user 2026-07-17).
  if (s_approach_cleared_issued) {
    s_alt_comp_target_ft = 0;
    s_alt_comp_arm_sec = 0.0f;
    s_alt_comp_sent = false;
    return false;
  }
  const int target = engine::current_cleared_alt_ft();
  if (target <= 0) {
    s_alt_comp_target_ft = 0;
    s_alt_comp_arm_sec = 0.0f;
    s_alt_comp_sent = false;
    return false;
  }
  if (target != s_alt_comp_target_ft) { // new level assigned -> re-arm grace
    s_alt_comp_target_ft = target;
    s_alt_comp_arm_sec = 0.0f;
    s_alt_comp_sent = false;
    return false;
  }
  s_alt_comp_arm_sec += dt;
  if (s_alt_comp_sent || atc_state_machine::is_readback_pending())
    return false;
  if (s_alt_comp_arm_sec < 45.0f) // grace for the pilot to start toward it
    return false;
  const int ta = (ctx.transition_alt_ft > 0) ? ctx.transition_alt_ft : 5000;
  const bool tgt_is_fl = (target > ta);
  const int actual = tgt_is_fl ? static_cast<int>(ctx.pressure_alt_ft)
                               : static_cast<int>(ctx.altitude_ft_msl);
  const int diff = actual - target; // + above assigned (needs descent)
  if (std::abs(diff) < 500) // effectively at / reaching the level
    return false;
  const bool needs_descent = diff > 0;
  const float vs = ctx.vertical_speed_fpm; // + up, - down
  const bool moving_ok = needs_descent ? (vs < -200.0f) : (vs > 200.0f);
  if (moving_ok) // actively moving toward the assigned level
    return false;
  s_alt_comp_sent = true;
  if (out_text) {
    const std::string &cs = atc_state_machine::session_callsign();
    const std::string &callsign = cs.empty() ? settings::pilot_callsign() : cs;
    const char *verb = needs_descent ? "descending" : "climbing";
    char buf[160];
    if (tgt_is_fl)
      std::snprintf(buf, sizeof(buf), "%s, confirm %s flight level %d.",
                    callsign.c_str(), verb, target / 100);
    else
      std::snprintf(buf, sizeof(buf), "%s, confirm %s %d feet.",
                    callsign.c_str(), verb, target);
    *out_text = buf;
  }
  logging::info("IFR descent/arrival: alt-compliance target=%d diff=%+d VS=%.0f -> confirm %s",
                target, diff, static_cast<double>(ctx.vertical_speed_fpm),
                needs_descent ? "descending" : "climbing");
  return true;
}

bool poll_descent(const xplane_context::XPlaneContext &ctx, float dt,
                  std::string *out_text,
                  bool *out_requires_readback) {
  using AS = atc_state_machine::ATCState;

  if (atc_state_machine::get_state() != AS::IFR_DESCENT) {
    s_descent_timer = 0.0f;
    s_descent_arrival_check_sec = 0.0f;
    return false;
  }

  // ACC/FIR handoff first (Milan -> France -> Marseille), before the STAR-entry
  // phase advance -- the aircraft is still under ACC here.
  if (poll_acc_sector_change(ctx, dt, out_text, out_requires_readback))
    return true;

  // DirectMonitor (descent course): extend course enforcement into DESCENT (was
  // en-route + approach only). Runs every frame (real dt), before the 1 Hz
  // throttle below. Shares s_enroute_course_cooldown (en-route/descent are
  // mutually exclusive states). ARRIVAL is intentionally NOT covered yet -- the
  // STAR is curved and direct-bearing-vs-heading would false-fire on the turns;
  // it needs the leg-track refinement (see the consolidation roadmap).
  s_enroute_course_cooldown = std::max(0.0f, s_enroute_course_cooldown - dt);
  if (s_enroute_course_cooldown <= 0.0f) {
    const CourseCheck cc = check_course(ctx, 25.0);
    if (cc.valid && cc.off_course && cc.dist_nm > 3.0) {
      s_enroute_course_cooldown = 180.0f;
      if (out_text) {
        const std::string &cs = atc_state_machine::session_callsign();
        const std::string &callsign = cs.empty() ? settings::pilot_callsign() : cs;
        char buf[176];
        std::snprintf(buf, sizeof(buf),
                      "%s, confirm direct %s, you appear tracking heading %.0f, "
                      "expected %.0f.",
                      callsign.c_str(), cc.ident.c_str(),
                      static_cast<double>(ctx.heading_true), cc.bearing_deg);
        *out_text = buf;
      }
      if (out_requires_readback)
        *out_requires_readback = false;
      logging::info("IFR descent: course deviation hdg %.0f vs brg %.0f to %s (diff %.0f)",
                    static_cast<double>(ctx.heading_true), cc.bearing_deg,
                    cc.ident.c_str(), cc.diff_deg);
      return true;
    }
  }

  s_descent_timer += dt;

  // DESCENT -> ARRIVAL: the aircraft has reached the STAR entry fix (first fix
  // of the arrival). This is the procedure-anchored phase boundary -- from here
  // the aircraft is flying the STAR under ACC/arrival control until the real
  // Approach handoff (poll_arrival). Silent: the same controller keeps working
  // the aircraft; only the internal phase advances. No-STAR fields:
  // find_star_entry falls back to the last enroute fix (still a valid anchor);
  // a terminal-area proximity fallback covers routes with no resolvable entry.
  // Polled at 1 Hz.
  s_descent_arrival_check_sec -= dt;
  if (s_descent_arrival_check_sec > 0.0f)
    return false;
  s_descent_arrival_check_sec = 1.0f;

  auto ofp = simbrief_ofp::get();
  if (!ofp.valid || ofp.navlog.empty())
    return false;

  const double d_dest = traffic_geometry::distance_nm(
      ctx.latitude, ctx.longitude, ofp.navlog.back().lat, ofp.navlog.back().lon);

  // Descend-to-enter the terminal area. If the aircraft is still cleared ABOVE
  // the destination TMA while over it, clear it down to a level INSIDE the TMA
  // so it can actually enter -- where Approach + the STAR step-downs take over.
  // Without this, an aircraft cleared to a level above a low-ceilinged TMA
  // (NICE and CHAMBERY both top at FL115) can never enter it and stalls above
  // the terminal area (LFLP->LFMN jump-to-ENR 2026-07-13: sat at FL120 over the
  // NICE TMA with no instruction). Geometry-based, so it resolves correctly for
  // a field controlled by a differently-named unit (LFLP/Annecy under the
  // CHAMBERY TMA). Fires once per distinct target; the descent it issues then
  // re-arms poll_altitude_compliance (both read s_enroute_cleared_alt_ft). This
  // is the primary "get into the TMA" path; the buffer-based TMA-entry advance
  // below is now the fallback for when no ceiling resolves (no openair).
  if (out_text && openair_db::ready()) {
    // Fire when the aircraft is laterally OVER a terminal TMA (polygon test),
    // above its ceiling. Use terminal_tma_ceiling(AIRCRAFT) -- the base TMA the
    // aircraft is actually over -- NOT descend_to_enter_ceiling(...,dest): the
    // latter depended on ofp.navlog.back() coords, which are unreliable in-sim
    // and silently returned 0 (LFMN ABDI8R 2026-07-13: over NICE TMA at FL177,
    // no descent). Base-TMA logic still ignores a stacked overlying TMA (GENEVA
    // over CHAMBERY -> 9500). We are already gated to DESCENT near the arrival.
    const int tma_ceil =
        openair_db::terminal_tma_ceiling(ctx.latitude, ctx.longitude);
    // Highest full-thousand flight level strictly below the TMA ceiling.
    int target_ft = (tma_ceil > 1000) ? ((tma_ceil - 100) / 1000) * 1000 : 0;
    // Respect STAR block-constraint floors: NEVER clear below an active block
    // floor ahead (e.g. LFMN MN261 block FL080/FL120). If a floor sits above
    // the level we'd otherwise pick, clamp up to it; if that clamp pushes the
    // target at/above the TMA ceiling, the aircraft can't legally enter the TMA
    // yet (it must stay above the floor), so the target_ft < tma_ceil guard
    // below suppresses the clearance until the block fix is behind us. ("+"/
    // at-or-above crossings fold in via the effective-constraint merge later.)
    const int block_floor = active_block_floor_ft();
    if (block_floor > 0 && target_ft < block_floor)
      target_ft = block_floor;
    const int cleared = engine::current_cleared_alt_ft();
    // Diagnostic: 1 Hz snapshot of the descend-to-enter gate inputs, so a flight
    // log shows exactly why it fires or not (remove once validated in-sim).
    if (tma_ceil > 0 && settings::debug_logging())
      logging::info("[dbg dte] tma_ceil=%d target=%d alt=%.0f cleared=%d floor=%d "
                    "last=%d -> %s",
                    tma_ceil, target_ft, static_cast<double>(ctx.altitude_ft_msl),
                    cleared, block_floor, s_descent_tma_target_ft,
                    (target_ft > 0 && target_ft < tma_ceil &&
                     static_cast<int>(ctx.altitude_ft_msl) > tma_ceil + 200 &&
                     cleared > target_ft + 100 && s_descent_tma_target_ft != target_ft)
                        ? "FIRE" : "hold");
    if (target_ft > 0 && target_ft < tma_ceil &&
        static_cast<int>(ctx.altitude_ft_msl) > tma_ceil + 200 &&
        cleared > target_ft + 100 && s_descent_tma_target_ft != target_ft) {
      s_descent_tma_target_ft = target_ft;
      s_enroute_cleared_alt_ft = target_ft; // re-arms poll_altitude_compliance
      const std::string &cs = atc_state_machine::session_callsign();
      const std::string &callsign = cs.empty() ? settings::pilot_callsign() : cs;
      const int ta = (ctx.transition_alt_ft > 0) ? ctx.transition_alt_ft : 5000;
      const std::string clr =
          format_alt_clearance(target_ft, AltHint::Auto, ctx.qnh_hpa, ta);
      *out_text = callsign + ", descend " + clr + ".";
      if (out_requires_readback)
        *out_requires_readback = true;
      logging::info("IFR descent: descend-to-enter terminal area, TMA ceil %d -> "
                    "%s (%.1f NM from dest)",
                    tma_ceil, clr.c_str(), d_dest);
      return true;
    }
  }

  // NOTE: the CIFP STAR crossing-altitude corrective that used to live here now
  // runs for EVERY airborne IFR phase in poll_profile_enforcement (dispatched
  // before this handler), so a STAR/approach crossing is enforced uniformly in
  // descent, arrival AND the approach -- not just here. See poll_profile_crossing.

  bool reached_arrival = false;
  std::string anchor_id;
  double anchor_lat = 0.0, anchor_lon = 0.0;

  // Anchor = the FIRST STAR fix (the STAR entry fix, which is the last en-route /
  // FPL fix). ARRIVAL must begin when the aircraft reaches the first STAR fix --
  // NOT earlier at the top-of-descent fix and NOT at a TMA boundary. Several TMAs
  // are crossed while flying a STAR and none of them is the arrival (user
  // 2026-07-14); this mirrors APPROACH firing at the IAF, for consistency. The
  // CIFP STAR entry fix is authoritative.
  {
    StarEntryResult se;
    if (find_star_entry(ctx.cifp_dir, ofp, se) && (se.lat != 0.0 || se.lon != 0.0)) {
      anchor_id  = se.ident;
      anchor_lat = se.lat;
      anchor_lon = se.lon;
    }
  }
  // Fallback: first navlog fix planned as descent (DSC) when no STAR entry
  // resolves (e.g. no STAR assigned / direct arrival).
  if (anchor_id.empty()) {
    for (const auto &f : ofp.navlog) {
      if (f.stage == "DSC" && (f.lat != 0.0 || f.lon != 0.0)) {
        anchor_id  = f.ident;
        anchor_lat = f.lat;
        anchor_lon = f.lon;
        break;
      }
    }
  }

  if (!anchor_id.empty()) {
    const double d_anchor = traffic_geometry::distance_nm(
        ctx.latitude, ctx.longitude, anchor_lat, anchor_lon);
    const double anchor_to_dest = traffic_geometry::distance_nm(
        anchor_lat, anchor_lon, ofp.navlog.back().lat, ofp.navlog.back().lon);
    // Reached the arrival entry (within 5 NM) or already past it (now closer to
    // the destination than the entry fix is).
    if (d_anchor <= 5.0 || d_dest < anchor_to_dest) {
      reached_arrival = true;
      logging::info("IFR descent -> arrival: at arrival entry %s (%.1f NM)",
                    anchor_id.c_str(), d_anchor);
    }
  } else if (s_descent_timer > 60.0f && d_dest <= 40.0) {
    // No resolvable entry fix -- begin the arrival on terminal-area proximity.
    reached_arrival = true;
    logging::info(
        "IFR descent -> arrival: terminal-area fallback (%.1f NM, no entry fix)",
        d_dest);
  }

  // (Removed the close-in TMA safety net: it fired at ~18 NM to dest and
  // preempted the STAR-entry anchor, setting ARRIVAL ~10 NM before SALEV
  // (LFLP 2026-07-14). The STAR-entry anchor above -- reached within 5 NM of the
  // entry fix OR once closer to dest than the entry fix is -- is the sole
  // trigger; the 40 NM no-anchor fallback covers the no-STAR case.)

  // NOTE: DESCENT->ARRIVAL and the Approach handoff both stay ALTITUDE-GATED
  // (find_enclosing above / in poll_arrival). Correct ATC model (user 2026-07-13):
  // the CURRENT controller (e.g. Marseille ACC) clears the aircraft DOWN to the
  // TMA ceiling (the descend-to-enter block above) to force it into the TMA;
  // only ONCE INSIDE the TMA does the handoff to the terminal unit (Nice
  // Approach) fire. So we must NOT flip/hand off while still above the ceiling.

  if (reached_arrival)
    atc_state_machine::set_state(AS::IFR_ARRIVAL);

  return false;
}

// Runs in IFR_ARRIVAL. The aircraft is flying the STAR under ACC/arrival
// control. Fires the real Approach handoff at the TMA/CTR boundary (openair) or
// the 50 NM fallback: a resolved controller -> spoken "contact X Approach" and
// IFR_APPROACH_CONTACT; no controller (AFIS / no-STAR field like LFQA) -> silent
// transition to IFR_APPROACH_CONTACT so poll_approach() drives the local
// INFO/Tower handoff at the FAF. (STAR step-downs under ACC are a future
// refinement -- today poll_approach issues them after the handoff.)
bool poll_arrival(const xplane_context::XPlaneContext &ctx, float dt,
                  std::string *out_text,
                  bool *out_requires_readback) {
  auto rb = [&](bool v) { if (out_requires_readback) *out_requires_readback = v; };
  rb(false);
  using AS = atc_state_machine::ATCState;

  if (atc_state_machine::get_state() != AS::IFR_ARRIVAL) {
    s_enroute_approach_handoff_issued = false;
    s_arrival_freq_handoff_label.clear();
    s_enroute_app_check_sec = 0.0f;
    s_arrival_timer = 0.0f;
    return false;
  }

  if (s_enroute_approach_handoff_issued)
    return false;

  // ACC/FIR handoff continues in ARRIVAL (Milan -> France -> Marseille) until
  // the Approach TRACON handoff below takes over. Runs first.
  if (poll_acc_sector_change(ctx, dt, out_text, out_requires_readback))
    return true;

  s_arrival_timer += dt;

  const std::string &cs = atc_state_machine::session_callsign();
  const std::string &callsign = cs.empty() ? settings::pilot_callsign() : cs;

  // ── TMA/CTR boundary → Approach frequency handoff ────────────────────
  // PRIMARY: openair_db TMA/CTR boundary crossing (exact airspace geometry
  // from Custom Data/airspaces/airspace.txt -- covers e.g. CHAMBERY TMA,
  // LYON TMA). The handoff fires as the aircraft descends INTO the arrival
  // TMA (inside laterally AND below the ceiling), which is why the old
  // premature FL207 handoff saw class=OTHER -- it was above every TMA.
  // When airspace data is present we rely on this real boundary and do NOT
  // use the crude distance rule (which is always true in the terminal area).
  // A close-in safety net (<=12 NM) covers sparse data where no TMA is
  // indexed. Without airspace data, fall back to the classic 50 NM handoff.
  // A 20 s minimum dwell lets the descent-clearance readback complete first.
  openair_db::AirspaceEntry enc_arrival;
  bool inside_tma = false;
  // Destination field's OWN stacked CTA (e.g. NICE CTA SECTOR 2, 11500-14500,
  // sitting above the Nice TMA) -> the accurate "you are now in <field>'s
  // airspace" cue, EARLIER than the IAF-eta gate. See the risk note at the
  // enter_approach decision below.
  bool enc_is_dest_cta = false;
  openair_db::AirspaceEntry dest_cta_enc;
  s_enroute_app_check_sec -= dt;
  if (s_enroute_app_check_sec <= 0.0f) {
    s_enroute_app_check_sec = 1.0f;
    const bool have_airspace = openair_db::ready();
    if (have_airspace) {
      // Query openair 1000 ft LOWER so a TMA the aircraft is about to drop into
      // still matches (mirrors poll_descent). find_enclosing returns the
      // INNERMOST enclosing volume, so near the IAF it resolves the terminal TMA
      // (CHAMBERY) rather than the big outer one (GENEVA) -- which is exactly what
      // makes Stage B below hand off to the correct terminal controller.
      constexpr int kArrivalCeilingBufferFt = 1000;
      enc_arrival = openair_db::find_enclosing(
          ctx.latitude, ctx.longitude,
          openair_alt(ctx) - kArrivalCeilingBufferFt);
      inside_tma = (enc_arrival.ac_class == openair_db::AirspaceClass::TMA ||
                    enc_arrival.ac_class == openair_db::AirspaceClass::CTR);
      // Destination CTA check at the ACTUAL altitude (no -1000 buffer -- a CTA
      // floored at the TMA top, e.g. 11500 ft, would fall out of a buffered
      // query). Only a CTA that resolves to the DESTINATION's own approach
      // facility counts (NICE CTA SECTOR 2 -> NICE APPROACH, facility LFMN ==
      // dest). This is what makes it SALEV3P-immune: see the note below.
      if (!inside_tma) {
        dest_cta_enc = openair_db::find_enclosing(
            ctx.latitude, ctx.longitude, openair_alt(ctx));
        if (dest_cta_enc.ac_class == openair_db::AirspaceClass::CTA) {
          const airspace_db::Controller *lc = nullptr;
          const airspace_db::Controller *oc =
              resolve_terminal_ctrl(dest_cta_enc.name, &lc);
          if (oc && !oc->freqs_khz.empty() && !s_assigned_dest_icao.empty() &&
              oc->facility_id == s_assigned_dest_icao)
            enc_is_dest_cta = true;
        }
      }
    }

    // ETA to the STAR-terminating IAF (e.g. PIRUV) -- the gate for ENTERING the
    // APPROACH phase. Same IAF selection as the cleared-approach trigger: the
    // no-STAR direct IAF, else the STAR's last fix. <0 = not resolvable
    // (AFIS / no-STAR field -> fall back to the legacy TMA/distance trigger).
    double iaf_eta_s = -1.0;
    int iaf_route_idx_local = -1;
    {
      std::string iaf = s_no_star_direct_iaf;
      if (iaf.empty())
        iaf = cifp_reader::star_last_fix(ctx.cifp_dir, s_assigned_dest_icao,
                                         s_assigned_star_name);
      if (!iaf.empty()) {
        for (int i = std::max(0, s_route_fix_idx);
             i < static_cast<int>(s_route_fixes.size()); ++i) {
          const auto &f = s_route_fixes[i];
          if (f.ident == iaf && (f.lat != 0.0 || f.lon != 0.0)) {
            const double gs = ctx.groundspeed_kts > 40.0f
                                  ? static_cast<double>(ctx.groundspeed_kts)
                                  : 120.0;
            iaf_eta_s = traffic_geometry::distance_nm(ctx.latitude, ctx.longitude,
                                                      f.lat, f.lon) /
                        gs * 3600.0;
            iaf_route_idx_local = i;
            break;
          }
        }
      }
    }

    // Stage B condition: enter APPROACH within ~90 s of the IAF. Without a
    // resolvable IAF (AFIS / no-STAR) fall back to the legacy trigger (TMA entry,
    // or the 12/50 NM distance net) so those fields still transition.
    bool enter_approach = false;
    if (enc_is_dest_cta) {
      // *** SALEV3P REGRESSION RISK (noted per user 2026-07-19) ***
      // This enters APPROACH on crossing into the destination field's own CTA --
      // EARLIER than the deliberately index-based IAF-eta gate below (which exists
      // to stop the looping SALE3P firing APPROACH ~4 fixes early at COLLO, see
      // 2026-07-16). It is placed FIRST but is structurally SALEV3P-IMMUNE: it
      // fires ONLY when the innermost CTA resolves to the DESTINATION's OWN
      // approach facility (facility_id == s_assigned_dest_icao). On the LIMF->LFLP
      // SALEV3P arrival NO CTA resolves to facility LFLP (Milan CTA -> LIMx;
      // Chambery, the terminal, is a TMA with facility LFLB != LFLP), so
      // enc_is_dest_cta is ALWAYS false there -> that flow is byte-for-byte
      // unchanged. It fires only for fields whose OWN centre owns a stacked CTA
      // above their TMA (LFMN: Nice CTA Sector 2, hand off to Nice on entry,
      // between AMFOU and TIPIK -- user 2026-07-19). *** If a SALEV3P arrival ever
      // regresses (premature APPROACH entry / early descent), REVERT THIS BRANCH
      // FIRST. ***
      enter_approach = true;
      enc_arrival = dest_cta_enc; // resolve the handoff controller from the CTA
    } else if (iaf_eta_s >= 0.0) {
      // Enter only when the route TRACKER has sequenced to within 2 fixes of the
      // IAF AND it is straight-line close. The guard MUST be tracker-index based,
      // NOT distance: on the looping SALE3P, COLLO (idx 8, an IAF overflown
      // mid-STAR) sits ~2 NM from PIRUV (idx 12) geographically, so a DISTANCE
      // "within 2 fixes" test fired APPROACH at COLLO -- 4 fixes early (LFLP
      // 2026-07-16). Only the SEQUENCE distinguishes them. The tracker is kept
      // reliable through wide/direct flying by the resync in poll_route_tracker.
      enter_approach = (iaf_eta_s <= 90.0 && iaf_route_idx_local >= 0 &&
                        s_route_fix_idx >= iaf_route_idx_local - 2);
    } else if (inside_tma) {
      enter_approach = true;
    } else if (s_arrival_timer > 20.0f) {
      auto ofp = simbrief_ofp::get();
      if (ofp.valid && !ofp.navlog.empty()) {
        const double dist_nm = traffic_geometry::distance_nm(
            ctx.latitude, ctx.longitude, ofp.navlog.back().lat,
            ofp.navlog.back().lon);
        enter_approach = dist_nm <= (have_airspace ? 12.0 : 50.0);
      }
    }

    // ── Early terminal frequency handoff (decoupled from the APPROACH phase) ──
    // As soon as the aircraft enters the DESTINATION's controlling terminal TMA
    // (e.g. CHAMBERY TMA, which works LFLP -- Annecy has no approach of its own),
    // hand the frequency to that approach controller. poll_acc_sector_change()
    // DEFERS while on_destination_terminal (it owns non-dest sectors), so the
    // dest-terminal handoff is OURS and there is no double "contact". This fires
    // Stage A: speak "contact <approach>" + arm the sector check-in but STAY in
    // IFR_ARRIVAL -- no walker, no clearance. The pilot talks to the terminal
    // controller while flying the STAR; the cleared-approach still waits for the
    // IAF-eta gate below (Stage B), which then silent-flips to APPROACH_DESCENT
    // (deduped by controller label via s_arrival_freq_handoff_label). Robust
    // across airports: keyed on on_destination_terminal (same terminal controller
    // as the dest), NOT facility==dest. ICAO Doc 4444: transfer of control at the
    // TMA boundary, approach clearance at/approaching the IAF -- so hand off
    // early, clear for the approach at the IAF. Fixes the ~40 s spent on the
    // previous (ACC) controller while already inside the terminal TMA (LFLP
    // 2026-07-20). Stage A deduplicates, so it fires once on entry then falls
    // through until the IAF gate.
    if (!enter_approach && inside_tma && !s_assigned_dest_icao.empty() &&
        openair_db::ready() && on_destination_terminal(ctx)) {
      if (build_approach_handoff(ctx, callsign, out_text, enc_arrival,
                                 /*enter_approach=*/false)) {
        rb(false);
        return true;
      }
    }

    if (enter_approach) {
      logging::info("IFR arrival: entering APPROACH (IAF eta=%.0fs, enc='%s')",
                    iaf_eta_s, enc_arrival.name.c_str());
      if (build_approach_handoff(ctx, callsign, out_text, enc_arrival,
                                 /*enter_approach=*/true)) {
        rb(false);
        return true; // caller only speaks when out_text is non-empty
      }
      // No dedicated Approach controller -- silent transition so poll_approach
      // drives the local INFO/Tower/AFIS handoff at the FAF.
      s_enroute_approach_handoff_issued = true;
      s_enroute_approach_freq_mhz = 0.0f;
      atc_state_machine::set_state(AS::IFR_APPROACH_CONTACT);
      logging::info("IFR arrival: no Approach controller -- current sector handles approach to established");
      return false;
    }

    // Frequency handoffs are boundary-driven: enroute + non-dest TMA sectors
    // (GENEVA, Milan sub-CTAs) by poll_acc_sector_change() on boundary crossing;
    // the DESTINATION's terminal TMA (CHAMBERY for LFLP) by the Stage A block
    // ABOVE on TMA entry (poll_acc_sector_change defers there via
    // on_destination_terminal). Both fire on CROSSING a boundary, never at a
    // route point (LFLP 2026-07-15/20). The approach CLEARANCE is separate and
    // waits for the IAF-eta gate (Stage B).
  }

  // ARRIVAL course enforcement. Previously OMITTED (the STAR is curved and
  // direct-bearing-vs-heading false-fired at the turns) -- now safe because
  // check_course suppresses within the turn-anticipation radius of the fix just
  // passed. Catches a pilot who leaves the STAR track, e.g. flies DIRECT to the
  // field after SALEV (LFLP 2026-07-18, "no course enforcement during STAR").
  // Lowest priority (after the approach handoff); shares s_enroute_course_cooldown
  // (ARRIVAL is mutually exclusive with en-route/descent).
  s_enroute_course_cooldown = std::max(0.0f, s_enroute_course_cooldown - dt);
  if (s_enroute_course_cooldown <= 0.0f) {
    const CourseCheck cc = check_course(ctx, 25.0);
    if (cc.valid && cc.off_course && cc.dist_nm > 3.0) {
      s_enroute_course_cooldown = 180.0f;
      if (out_text) {
        const std::string &cs = atc_state_machine::session_callsign();
        const std::string &callsign =
            cs.empty() ? settings::pilot_callsign() : cs;
        char buf[188];
        std::snprintf(buf, sizeof(buf),
                      "%s, confirm routing, you appear tracking heading %.0f, "
                      "expected %.0f to %s.",
                      callsign.c_str(), static_cast<double>(ctx.heading_true),
                      cc.bearing_deg, cc.ident.c_str());
        *out_text = buf;
      }
      rb(false);
      logging::info(
          "IFR arrival: course deviation hdg %.0f vs brg %.0f to %s (diff %.0f)",
          static_cast<double>(ctx.heading_true), cc.bearing_deg,
          cc.ident.c_str(), cc.diff_deg);
      return true;
    }
  }

  return false;
}

const std::string &current_controller_label() {
  return s_current_controller_label;
}

// Frequency (MHz) the pilot should tune after the most recent training jump, or
// 0.0 if unknown. The jumps set ATC state only; the UI shows this as a
// "Switch COM to X" hint popup.
float jump_switch_freq_mhz() { return s_jump_switch_freq_mhz; }

// CIFP-assigned landing runway, set at the approach clearance and retained
// through landing + taxi-in (cleared only at the full IFR reset), unlike
// atc_state_machine::assigned_runway() which is cleared post-landing. The STT
// context bias uses this so "runway 22L / R-NAV 22L" stays anchored from the
// approach clearance through the vacated / ground calls (LFMN 2026-07-12: once
// it dropped from the bias post-landing, "runway 22L" was heard "runway to toL").
const std::string &assigned_landing_runway() { return s_assigned_landing_runway; }

// The controller the pilot is being handed TO (set on a handoff, before the
// pilot switches frequency). Biased into the STT context so the readback of
// "contact <X>" transcribes the target name correctly.
const std::string &pending_controller_label() {
  return s_pending_controller_label;
}

void set_controller_label(const std::string &label) {
  if (!label.empty())
    s_current_controller_label = label;
}

void set_pending_departure_label(const std::string &label) {
  if (!label.empty())
    s_pending_departure_label = label;
}

const std::string &pending_departure_label() {
  return s_pending_departure_label;
}

const std::string &assigned_star_name() { return s_assigned_star_name; }

// Spoken (plain-language) form of the assigned STAR for the STT context bias, so
// the pilot's readback of what ATC actually SAID ("SALEV THREE PAPA") is
// recognised -- the coded form alone ("SALEV3P") made Voxtral hear "side of 3
// Papa" (user 2026-07-19). Computed from the coded name + naming fix; empty when
// no STAR is assigned.
std::string assigned_star_spoken() {
  if (s_assigned_star_name.empty())
    return {};
  return spoken_procedure_name(s_assigned_star_name, s_assigned_star_entry_fix);
}

std::vector<std::string> upcoming_route_fix_idents() {
  std::vector<std::string> out;
  for (int i = std::max(0, s_route_fix_idx);
       i < static_cast<int>(s_route_fixes.size()); ++i) {
    const std::string &id = s_route_fixes[i].ident;
    if (!id.empty())
      out.push_back(id);
  }
  return out;
}

int current_cleared_alt_ft() {
  // Precedence: freshest write wins. s_enroute_cleared_alt_ft is updated
  // by SID climb, en-route step-ups, en-route step-downs, approach
  // check-in (seeds it from s_approach_initial_fl), STAR step-downs, and
  // approach final descent. So it always holds the most current cleared
  // altitude when in an airborne IFR phase. s_approach_initial_fl kept
  // as a fallback in case check-in code path failed to seed enroute.
  // s_sid_step1_alt_ft is a last-resort fallback for very early climb.
  if (s_enroute_cleared_alt_ft > 0) return s_enroute_cleared_alt_ft;
  if (s_approach_initial_fl > 0) return s_approach_initial_fl;
  if (s_sid_step1_alt_ft > 0) return s_sid_step1_alt_ft;
  return 0;
}

int current_speed_restriction_kt() { return s_last_speed_limit_kt; }

// ── Helpers for poll_approach ─────────────────────────────────────────────

// Build a STAR constraint clearance string for one waypoint.
// Format: "[cs], direct [fix], descend [alt]."
// QNH is appended when alt is in feet (below transition altitude).
static std::string build_star_constraint(
    const std::string &cs,
    const cifp_reader::StarWaypoint &wp,
    int cleared_ft,
    int qnh_hpa,
    int ta_ft
) {
  const int ta = (ta_ft > 0) ? ta_ft : 5000;
  std::string msg = cs;
  // FL-vs-feet from the CIFP constraint's is_fl flag (feet vs FL as published
  // on the plate); TL threshold only when not tied to a real CIFP altitude.
  // For a block "B", cleared_ft is the FLOOR (a plain feet value, e.g. GOVNA
  // 6500) — its own is_fl is not the ceiling's, so decide by the TL threshold.
  const AltHint hint =
      (wp.floor_ft > 0) ? AltHint::Auto : alt_hint_for(wp.alt);
  msg += ", descend " + format_alt_clearance(cleared_ft, hint, qnh_hpa, ta);
  if (wp.speed_kt > 0) {
    char spd[40];
    std::snprintf(spd, sizeof(spd), ", speed %d knots or less", wp.speed_kt);
    msg += spd;
  }
  msg += ".";
  return msg;
}

// Build an approach step-down clearance.
// Format: "[cs], direct [fix], descend [alt]."  (fix_ident may be empty)
// QNH appended when alt is below transition altitude.
static std::string build_approach_final_alt(const std::string &cs,
                                             const std::string &fix_ident,
                                             int alt_ft,
                                             int qnh_hpa,
                                             int ta_ft,
                                             bool is_fl) {
  // FL-vs-feet is taken from the CIFP constraint's is_fl flag, NOT re-derived
  // from a transition-altitude comparison. Approach/STAR crossing altitudes
  // are published in feet (QNH) below the transition level and as FL above;
  // the chart designer already encoded that in the CIFP field ("06500" =
  // 6500 ft is_fl=false, "FL080" = FL080 is_fl=true). The old "alt_ft > ta"
  // guess spoke LP403's 6500 ft as "flight level 65" (LIMF -> LFLP
  // 2026-07-10) — wrong; it's a QNH altitude on the plate.
  std::string msg = cs;
  if (!fix_ident.empty())
    msg += ", direct " + fix_ident;
  msg += ", descend " +
         format_alt_clearance(alt_ft, is_fl ? AltHint::FlightLevel : AltHint::Feet,
                              qnh_hpa, ta_ft) +
         ".";
  return msg;
}

// Build the spoken approach identity phrase, no verb -- e.g.
// "RNAV Zulu approach runway 04". Empty when the assigned approach/runway is
// unknown. Shared by the TOD "expect <phrase>" advisory and the single
// "cleared <phrase>" clearance fired once at the IAF (see
// project_arrival_announcement_model: ONE expect + ONE cleared-approach).
static std::string approach_clearance_phrase(
    const xplane_context::XPlaneContext &ctx) {
  if (ctx.cifp_dir.empty() || s_assigned_dest_icao.empty() ||
      s_assigned_approach_designator.empty())
    return "";
  cifp_reader::ApproachInfo appr = cifp_reader::approach_by_designator(
      ctx.cifp_dir, s_assigned_dest_icao, s_assigned_approach_designator);
  if (appr.type_str.empty())
    return "";
  static const char *nato[] = {
      "Alpha","Bravo","Charlie","Delta","Echo","Foxtrot","Golf",
      "Hotel","India","Juliet","Kilo","Lima","Mike","November",
      "Oscar","Papa","Quebec","Romeo","Sierra","Tango","Uniform",
      "Victor","Whiskey","X-ray","Yankee","Zulu"};
  std::string variant_word;
  char suf = cifp_reader::approach_suffix(appr.designator);
  if (suf) {
    int idx = std::toupper(static_cast<unsigned char>(suf)) - 'A';
    if (idx >= 0 && idx < 26)
      variant_word = std::string(" ") + nato[idx];
  }
  const std::string rwy = !s_assigned_landing_runway.empty()
                              ? s_assigned_landing_runway
                              : appr.runway;
  return appr.type_str + variant_word + " approach runway " + rwy;
}

// Public wrapper: spoken approach identity ("RNAV Zulu approach runway 04") for
// the STT context bias -- ATC speaks the NATO variant word, so bias it or the
// pilot's readback garbles ("Zulu" -> "zero", "04" -> "zero for"; user
// 2026-07-19). Empty when no approach is assigned.
std::string assigned_approach_spoken(const xplane_context::XPlaneContext &ctx) {
  return approach_clearance_phrase(ctx);
}

// ── Route fix tracker ────────────────────────────────────────────────────
// Logging only — no ATC speech, no state change.

// Build the ordered route fix list from OFP navlog + STAR/APP waypoints.
// Looks up lat/lon for STAR/APP idents from earth_fix.dat.
// Called once when STAR/approach waypoints are loaded.
// Build the DEPARTURE portion of the unified route table: enroute navlog fixes
// enriched with the assigned SID's CIFP climb constraints (floor / block /
// speed). Lets the shared compliance monitor + poll_speed_restriction enforce
// SID limits during the climb, exactly as the arrival table serves approach.
// Transient -- init_route_fixes rebuilds the table for the arrival. v1 enriches
// fixes already present in the OFP navlog; a SID fix absent from the filed route
// carries no constraint (acceptable first cut). See [[project_release_4_4_0]].
static void build_sid_route_table(const xplane_context::XPlaneContext &ctx) {
  s_route_fixes.clear();
  s_route_fix_idx = 0;

  const std::string dep =
      s_departure_apt_id.empty() ? ctx.nearest_airport_id : s_departure_apt_id;
  const std::string &sid = ctx.ifr_sid;
  std::unordered_set<std::string> seen;

  // 1. SID fixes FROM CIFP (idents + climb constraints), positioned via
  //    earth_fix.dat -- the same pattern init_route_fixes uses for STAR/
  //    approach. Sourcing from CIFP (not the OFP navlog) means every SID fix
  //    is present with its constraint even when the filed route omits it.
  if (!ctx.cifp_dir.empty() && !dep.empty() && !sid.empty()) {
    const auto sw = cifp_reader::sid_waypoints(ctx.cifp_dir, dep, sid,
                                               ctx.active_runway,
                                               /*constrained_only=*/false);
    std::vector<std::string> idents;
    for (const auto &wp : sw)
      if (!wp.ident.empty())
        idents.push_back(wp.ident);
    const auto pos = cifp_reader::lookup_fix_positions(ctx.cifp_dir, idents, dep);
    for (const auto &wp : sw) {
      if (wp.ident.empty() || seen.count(wp.ident))
        continue;
      seen.insert(wp.ident);
      double lat = 0.0, lon = 0.0;
      auto it = pos.find(wp.ident);
      if (it != pos.end()) { lat = it->second.first; lon = it->second.second; }
      RouteFix rf;
      rf.ident      = wp.ident;
      rf.lat        = lat;
      rf.lon        = lon;
      rf.alt        = wp.alt;
      rf.is_ceiling = wp.is_ceiling;
      rf.is_floor   = wp.is_floor;
      rf.floor_ft   = wp.floor_ft;
      rf.speed_kt   = wp.speed_kt;
      s_route_fixes.push_back(rf);
    }
  }

  // 2. Enroute navlog fixes AFTER the SID (skip any already added from the SID,
  //    plus the SimBrief pseudo-fixes). These carry the OFP lat/lon.
  const auto &ofp = simbrief_ofp::get();
  for (const auto &nf : ofp.navlog) {
    if (nf.ident.empty() || nf.ident == "TOC" || nf.ident == "TOD")
      continue;
    if (seen.count(nf.ident))
      continue;
    seen.insert(nf.ident);
    s_route_fixes.push_back({nf.ident, nf.lat, nf.lon});
  }

  logging::info("[route] SID table: %d fixes (SID %s from CIFP + navlog, dep=%s rwy=%s)",
                static_cast<int>(s_route_fixes.size()),
                sid.empty() ? "(none)" : sid.c_str(), dep.c_str(),
                ctx.active_runway.c_str());

  // Advance past fixes already behind the aircraft (proximity + bearing),
  // same rule as init_route_fixes.
  while (s_route_fix_idx < static_cast<int>(s_route_fixes.size())) {
    const auto &rf = s_route_fixes[s_route_fix_idx];
    if (rf.lat == 0.0 && rf.lon == 0.0) { s_route_fix_idx++; continue; }
    const float d = static_cast<float>(traffic_geometry::distance_nm(
        ctx.latitude, ctx.longitude, rf.lat, rf.lon));
    if (d < 1.5f) { s_route_fix_idx++; continue; }
    double brg = traffic_geometry::bearing_deg(ctx.latitude, ctx.longitude,
                                               rf.lat, rf.lon);
    double diff = std::abs(brg - static_cast<double>(ctx.heading_true));
    if (diff > 180.0) diff = 360.0 - diff;
    if (diff > 90.0) { s_route_fix_idx++; continue; }
    break;
  }
}

static void init_route_fixes(const xplane_context::XPlaneContext &ctx) {
  s_route_fixes.clear();
  s_route_fix_idx = 0;

  // 1. OFP navlog fixes (already have lat/lon).
  // TOC and TOD are SimBrief pseudo-fixes with no airspace significance.
  const auto &ofp = simbrief_ofp::get();
  std::unordered_set<std::string> seen;
  for (const auto &nf : ofp.navlog) {
    if (nf.ident.empty() || nf.ident == s_assigned_dest_icao) continue;
    if (nf.ident == "TOC" || nf.ident == "TOD") continue;
    if (seen.count(nf.ident)) continue;
    seen.insert(nf.ident);
    s_route_fixes.push_back({nf.ident, nf.lat, nf.lon});
  }

  // 2. COMPLETE STAR + approach fix sequence (ALL fixes, not just the
  //    constrained subset) so the tracker follows every fix; a fix leaves the
  //    sequence only via an explicit ATC direct-to. Each carries its CIFP
  //    alt/FL + speed constraint. A fix already present from the navlog is
  //    ENRICHED with its constraint rather than duplicated or left bare.
  if (!ctx.cifp_dir.empty() && !s_assigned_dest_icao.empty()) {
    std::vector<cifp_reader::StarWaypoint> arr;
    if (!s_assigned_star_name.empty())
      arr = cifp_reader::star_waypoints(ctx.cifp_dir, s_assigned_dest_icao,
                                        s_assigned_star_name,
                                        /*constrained_only=*/false);
    if (!s_assigned_approach_designator.empty()) {
      const std::string iaf =
          s_assigned_star_name.empty()
              ? s_no_star_direct_iaf
              : cifp_reader::star_last_fix(ctx.cifp_dir, s_assigned_dest_icao,
                                           s_assigned_star_name);
      auto ap = cifp_reader::approach_procedure_waypoints(
          ctx.cifp_dir, s_assigned_dest_icao, s_assigned_approach_designator,
          iaf, /*constrained_only=*/false);
      for (auto &w : ap)
        arr.push_back(w);
    }
    // Never regress to fewer fixes: if the full lists came back empty, fall
    // back to the already-loaded constrained set.
    if (arr.empty())
      arr = s_approach_waypoints;

    std::vector<std::string> idents;
    for (const auto &wp : arr)
      if (!wp.ident.empty())
        idents.push_back(wp.ident);
    const auto pos_map = cifp_reader::lookup_fix_positions(
        ctx.cifp_dir, idents, s_assigned_dest_icao);
    // Destination position for the implausibility guard below: a CIFP STAR/approach
    // fix is always in the destination terminal area, so a resolved coord far from
    // the dest is a mis-resolution (duplicate-ident collision).
    const auto dpos = s_assigned_dest_icao.empty()
                          ? std::pair<double, double>{0.0, 0.0}
                          : xplane_context::airport_pos_for(s_assigned_dest_icao);

    for (const auto &wp : arr) {
      if (wp.ident.empty())
        continue;
      // Enrich an existing (navlog) entry with its CIFP constraint.
      RouteFix *existing = nullptr;
      for (auto &rf : s_route_fixes)
        if (rf.ident == wp.ident) { existing = &rf; break; }
      if (existing) {
        existing->alt = wp.alt;
        existing->is_ceiling = wp.is_ceiling;
        existing->is_floor = wp.is_floor;
        existing->floor_ft = wp.floor_ft;
        existing->speed_kt = wp.speed_kt;
        existing->is_approach_proc = wp.is_approach_proc;
        existing->is_map = wp.is_map;
        continue;
      }
      double lat = 0.0, lon = 0.0;
      auto it = pos_map.find(wp.ident);
      if (it != pos_map.end()) { lat = it->second.first; lon = it->second.second; }
      // Plausibility guard + root-cause diagnostic (LFLP 2026-07-18: PINOT resolved
      // to BEKOK's China coord (31.45,122.45), 7000+ NM from LFLP, so the tracker
      // could never reach it -> "skipped"). A CIFP STAR/approach fix >300 NM from the
      // destination is a mis-resolution: drop the coord (lat=lon=0 -> the tracker and
      // check_course ignore 0/0 fixes) so it can't poison the leg geometry. Log the
      // ident, coord, distance AND cifp_dir so the next flight pins WHY pos_map
      // returned it (lookup_fix_positions resolves PINOT correctly in isolation, so
      // the runtime input/data source is the suspect).
      if ((lat != 0.0 || lon != 0.0) &&
          (dpos.first != 0.0 || dpos.second != 0.0)) {
        const double d =
            traffic_geometry::distance_nm(lat, lon, dpos.first, dpos.second);
        if (d > 300.0) {
          logging::info(
              "[route] IMPLAUSIBLE fix %s (%.4f,%.4f) is %.0f NM from %s -- dropping "
              "coord (via pos_map, cifp_dir=%s)",
              wp.ident.c_str(), lat, lon, d, s_assigned_dest_icao.c_str(),
              ctx.cifp_dir.c_str());
          lat = 0.0;
          lon = 0.0;
        }
      }
      RouteFix rf;
      rf.ident = wp.ident;
      rf.lat = lat;
      rf.lon = lon;
      rf.alt = wp.alt;
      rf.is_ceiling = wp.is_ceiling;
      rf.is_floor = wp.is_floor;
      rf.floor_ft = wp.floor_ft;
      rf.speed_kt = wp.speed_kt;
      rf.is_approach_proc = wp.is_approach_proc;
      rf.is_map = wp.is_map;
      s_route_fixes.push_back(rf);
    }
  }

  // 3. Advance past fixes that are already behind the aircraft.
  // Two conditions to skip: (a) within 3 NM — too close, already past;
  // (b) behind the aircraft — bearing to fix differs from heading by > 90°.
  // Condition (b) is necessary when calling in mid-STAR: the navlog starts
  // at the origin airport (far behind), so without heading-based skipping
  // s_route_fix_idx would remain at 0 forever.
  while (s_route_fix_idx < static_cast<int>(s_route_fixes.size())) {
    const auto &rf = s_route_fixes[s_route_fix_idx];
    if (rf.lat == 0.0 && rf.lon == 0.0) { s_route_fix_idx++; continue; }
    float d = static_cast<float>(traffic_geometry::distance_nm(
        ctx.latitude, ctx.longitude, rf.lat, rf.lon));
    if (d < 1.5f) { s_route_fix_idx++; continue; }  // already very close
    double brg = traffic_geometry::bearing_deg(
        ctx.latitude, ctx.longitude, rf.lat, rf.lon);
    double diff = std::abs(brg - static_cast<double>(ctx.heading_true));
    if (diff > 180.0) diff = 360.0 - diff;
    if (diff > 90.0) { s_route_fix_idx++; continue; }  // fix is behind heading
    break;
  }

  // 3b. Overshoot guard. The heading-based skip above walks PAST any fix whose
  // bearing differs from the current heading by >90deg. On a curved STAR entered
  // from a mid-air training jump the ahead fixes can all read "behind heading",
  // so the loop skips the ENTIRE route to the end (LFMN jump-to-APP 2026-07-21:
  // "tracker init: 17 fixes, start idx=17", past the FAF at idx 12 -> the Tower
  // handoff fired instantly). Snap back to the geographically NEAREST fix: the
  // aircraft cannot be past a fix it is still far from. No-op on a normal flight
  // (there the skip stops at the real position, so idx ~= nearest).
  {
    int nearest = -1;
    double best_nm = 1e18;
    for (int i = 0; i < static_cast<int>(s_route_fixes.size()); ++i) {
      const auto &rf = s_route_fixes[i];
      if (rf.lat == 0.0 && rf.lon == 0.0)
        continue;
      const double d = traffic_geometry::distance_nm(ctx.latitude, ctx.longitude,
                                                     rf.lat, rf.lon);
      if (d < best_nm) {
        best_nm = d;
        nearest = i;
      }
    }
    if (nearest >= 0 && s_route_fix_idx > nearest + 1)
      s_route_fix_idx = nearest;
  }

  // 4. If a direct-to-IAF was issued before init (e.g. descent clearance "direct QA503"),
  // jump the tracker to that fix so intermediate skipped waypoints don't stall it.
  if (!s_no_star_direct_iaf.empty()) {
    for (int i = s_route_fix_idx; i < static_cast<int>(s_route_fixes.size()); ++i) {
      if (s_route_fixes[i].ident != s_no_star_direct_iaf) continue;
      s_route_fix_idx = i;
      const auto &rf = s_route_fixes[i];
      if (rf.lat != 0.0 || rf.lon != 0.0) {
        float d = static_cast<float>(
            traffic_geometry::distance_nm(ctx.latitude, ctx.longitude, rf.lat, rf.lon));
        double brg = traffic_geometry::bearing_deg(
            ctx.latitude, ctx.longitude, rf.lat, rf.lon);
        double diff = std::abs(brg - static_cast<double>(ctx.heading_true));
        if (diff > 180.0) diff = 360.0 - diff;
        if (d < 1.5f || diff > 90.0) s_route_fix_idx = i + 1; // already past
      }
      logging::info("[route] direct-to %s: tracker idx=%d",
                    s_no_star_direct_iaf.c_str(), s_route_fix_idx);
      break;
    }
  }

  logging::info("[route] tracker init: %d fixes, start idx=%d",
               static_cast<int>(s_route_fixes.size()), s_route_fix_idx);
  for (int i = s_route_fix_idx;
       i < static_cast<int>(s_route_fixes.size()) && i < s_route_fix_idx + 20;
       ++i) {
    const auto &rf = s_route_fixes[i];
    char cons[48] = "";
    if (rf.alt.feet > 0) {
      const char *d = rf.is_ceiling ? "-" : rf.is_floor ? "+" : "@";
      if (rf.alt.is_fl)
        std::snprintf(cons, sizeof(cons), " %sFL%d", d, rf.alt.feet / 100);
      else
        std::snprintf(cons, sizeof(cons), " %s%dft", d, rf.alt.feet);
    }
    char spd[24] = "";
    if (rf.speed_kt > 0)
      std::snprintf(spd, sizeof(spd), " %dkt", rf.speed_kt);
    logging::info("[route]   [%d] %s (%.4f,%.4f)%s%s%s", i, rf.ident.c_str(),
                  rf.lat, rf.lon, cons, spd, rf.is_approach_proc ? " [APP]" : "");
  }
}

std::string take_pending_transcript_note() {
  std::string n;
  n.swap(s_pending_transcript_note);
  return n;
}

std::string poll_route_tracker(const xplane_context::XPlaneContext &ctx) {
  // Priority: return any pending ATC-direct event before the rate-limited
  // proximity check so atc_session sees it on the very next frame.
  if (!s_pending_route_direct.empty()) {
    std::string ev = s_pending_route_direct;
    s_pending_route_direct.clear();
    return ev;
  }

  if (s_route_fixes.empty()) return {};
  if (s_route_fix_idx >= static_cast<int>(s_route_fixes.size())) return {};

  // Rate-limit to 1 Hz — distance check is not time-critical.
  // atc_session::update() is called at ~60 FPS; we accumulate real dt.
  // Use a simple flight-loop frame counter approximation via a static.
  s_route_tracker_tick += 1.0f / 60.0f; // approximate 60 FPS
  if (s_route_tracker_tick < 1.0f) return {};
  s_route_tracker_tick = 0.0f;

  // RESYNC (user pb2, LFLP 2026-07-16): if the aircraft flew WIDE of the current
  // fix (never entered its 1.5 NM capture zone) and then rejoined the STAR at a
  // LATER fix, jump the tracker forward to whatever route fix it is now flying over
  // (within ~2 NM) -- do not stay frozen on the skipped fix. The tracker froze at
  // PINOT after a post-SALEV divergence, so the whole ARRIVAL->APPROACH gate (which
  // reads s_route_fix_idx) stalled and Chambery never cleared the approach. Scan
  // from the current index forward and take the FURTHEST fix within capture so all
  // skipped fixes are passed at once.
  {
    int resync_idx = -1;
    for (int i = s_route_fix_idx; i < static_cast<int>(s_route_fixes.size()); ++i) {
      const auto &fi = s_route_fixes[i];
      if (fi.lat == 0.0 && fi.lon == 0.0)
        continue;
      // Do NOT let a geographically-clustered APPROACH fix hijack the resync while
      // still on the STAR: on the LFLP RNAV, LP403 sits ~2 NM from COLLO, so the
      // resync jumped the tracker PAST the IAF (PIRUV) -- which broke the STAR
      // descent (targeted LP402 5000 instead of the FL090 step) AND the approach-
      // clearance anchor (LFLP 2026-07-16 regression). Approach-proc fixes only
      // become resync targets once the approach is actually cleared.
      if (fi.is_approach_proc && !s_approach_cleared_issued)
        continue;
      if (traffic_geometry::distance_nm(ctx.latitude, ctx.longitude, fi.lat,
                                        fi.lon) <= 2.0)
        resync_idx = i; // keep the furthest fix currently within capture
    }
    if (resync_idx > s_route_fix_idx) {
      const auto &rf = s_route_fixes[resync_idx];
      const int skipped = resync_idx - s_route_fix_idx;
      const int nxt = resync_idx + 1;
      std::string ni = (nxt < static_cast<int>(s_route_fixes.size()))
                           ? s_route_fixes[nxt].ident
                           : "end of route";
      char rbuf[176];
      std::snprintf(rbuf, sizeof(rbuf),
                    "Track: resync to %s (skipped %d), next: %s",
                    rf.ident.c_str(), skipped, ni.c_str());
      logging::info("[route] %s", rbuf);
      s_route_fix_idx = resync_idx + 1; // advance past the overflown fix
      return rbuf;
    }
  }

  const auto &fix = s_route_fixes[s_route_fix_idx];

  // Skip fixes whose position is unknown.
  if (fix.lat == 0.0 && fix.lon == 0.0) {
    s_route_fix_idx++;
    return {};
  }

  const float dist = static_cast<float>(traffic_geometry::distance_nm(
      ctx.latitude, ctx.longitude, fix.lat, fix.lon));

  if (dist > 1.5f) {
    // The fix is within a few NM but never entered the 1.5 NM capture zone and is
    // now clearly BEHIND the aircraft (bearing >100 deg off the nose) -- it was
    // overflown wide. Advance so the tracker (and check_course) move on instead
    // of forever flagging "off course" to a fix behind us (LFMN ABDI8R 2026-07-13:
    // deviated wide of ABDIL, tracker stuck, repeated "confirm direct ABDIL").
    //
    // UPPER DISTANCE CAP (kNearBypass) is essential: a fix that is FAR ahead on a
    // CURVED path (e.g. the FAF at 10 NM before the final turn) has a wide
    // bearing-off-nose yet is NOT bypassed -- the aircraft simply hasn't turned
    // toward it. Without the cap, heading-vs-bearing wrongly "bypassed" FP04Z at
    // 10.1 NM (brg 98 vs hdg 221) and fired the Tower handoff 10 NM early
    // (LIMF->LFLP 2026-07-14). You can't overfly a fix 10 NM away.
    constexpr float kNearBypass = 4.0f;
    const double brg = traffic_geometry::bearing_deg(ctx.latitude, ctx.longitude,
                                                     fix.lat, fix.lon);
    double behind = std::fabs(brg - static_cast<double>(ctx.heading_true));
    if (behind > 180.0) behind = 360.0 - behind;
    if (behind > 100.0 && dist < kNearBypass) {
      logging::info("[route] bypassed %s (%.1f NM, brg %.0f behind hdg %.0f) -- advancing",
                    fix.ident.c_str(), dist, brg,
                    static_cast<double>(ctx.heading_true));
      s_route_fix_idx++;
    }
    return {};
  }

  // Entered 1.5 NM zone around this fix — log and advance.
  const int next_idx = s_route_fix_idx + 1;
  std::string next_ident = "end of route";
  if (next_idx < static_cast<int>(s_route_fixes.size()))
    next_ident = s_route_fixes[next_idx].ident;

  char buf[160];
  std::snprintf(buf, sizeof(buf), "Track: near %s (%.1f NM), next: %s",
                fix.ident.c_str(), dist, next_ident.c_str());

  logging::info("[route] %s", buf);
  s_route_fix_idx++;
  return buf;
}

// ── poll_approach ─────────────────────────────────────────────────────────

bool poll_approach(const xplane_context::XPlaneContext &ctx, float dt,
                   std::string *out_text,
                   bool *out_requires_readback) {
  auto rb = [&](bool v) { if (out_requires_readback) *out_requires_readback = v; };
  rb(false);
  using AS = atc_state_machine::ATCState;

  const AS state = atc_state_machine::get_state();
  if (state != AS::IFR_APPROACH_CONTACT && state != AS::IFR_APPROACH_DESCENT) {
    s_approach_waypoints.clear();
    s_approach_waypoint_idx = 0;
    s_approach_timer = 0.0f;
    s_approach_initial_fl = 0;
    s_approach_final_issued = false;
    s_approach_cleared_issued = false;
    s_approach_tower_handed_off = false;
    s_approach_faf = {};
    s_last_cleared_route_idx    = -1;
    s_faf_route_idx             = -1;
    s_iaf_route_idx             = -1;
    s_faf_ap_idx                = -1;
    s_map_ap_idx                = -1;
    s_approach_has_visual_final = false;
    s_expedite_cooldown        = 0.0f;
    s_expedite_last_cleared_ft = 0;
    s_pending_route_direct.clear();
    s_approach_sector_freq_khz  = 0;
    s_approach_sector_ceiling_ft = 0;
    s_approach_sector_check_sec = 0.0f;
    s_approach_visited_sector_freqs.clear();
    return false;
  }

  // Load STAR + approach procedure waypoints on first entry (fallback if not
  // loaded at APPROACH_CONTACT, e.g. training_jump_approach).
  if (s_approach_waypoints.empty() && s_approach_waypoint_idx == 0 &&
      !s_assigned_star_name.empty() && !s_assigned_dest_icao.empty()) {
    s_approach_waypoints = cifp_reader::star_waypoints(
        ctx.cifp_dir, s_assigned_dest_icao, s_assigned_star_name);
    logging::info("IFR approach: loaded %d constrained STAR waypoints for %s",
                  static_cast<int>(s_approach_waypoints.size()),
                  s_assigned_star_name.c_str());
    if (!s_assigned_approach_designator.empty()) {
      const std::string iaf = cifp_reader::star_last_fix(
          ctx.cifp_dir, s_assigned_dest_icao, s_assigned_star_name);
      if (!iaf.empty()) {
        auto proc = cifp_reader::approach_procedure_waypoints(
            ctx.cifp_dir, s_assigned_dest_icao,
            s_assigned_approach_designator, iaf);
        if (!proc.empty()) {
          for (auto &w : proc)
            s_approach_waypoints.push_back(w);
          s_approach_final_issued = true;
          logging::info("IFR approach: appended %d IAF-transition waypoints (%s)",
                        static_cast<int>(proc.size()),
                        s_assigned_approach_designator.c_str());
          s_faf_ap_idx = -1;
          s_map_ap_idx = -1;
          for (int i = 0; i < static_cast<int>(s_approach_waypoints.size()); ++i) {
            const auto &w = s_approach_waypoints[i];
            if (s_faf_ap_idx < 0 && w.is_approach_proc &&
                w.ident == s_approach_faf.ident)
              s_faf_ap_idx = i;
            if (s_map_ap_idx < 0 && w.is_approach_proc && w.is_map)
              s_map_ap_idx = i;
          }
          logging::info("[route] FAF ap_idx=%d MAP ap_idx=%d (lazy)",
                        s_faf_ap_idx, s_map_ap_idx);
        }
      }
    }
    // Route tracker init (lazy path: training jump, waypoints loaded here).
    if (s_route_fixes.empty())
      init_route_fixes(ctx);
    if (s_faf_route_idx < 0 && !s_approach_faf.ident.empty()) {
      for (int i = 0; i < static_cast<int>(s_route_fixes.size()); ++i) {
        if (s_route_fixes[i].ident == s_approach_faf.ident) {
          s_faf_route_idx = i;
          break;
        }
      }
    }
    if (s_iaf_route_idx < 0 && !s_no_star_direct_iaf.empty()) {
      for (int i = 0; i < static_cast<int>(s_route_fixes.size()); ++i) {
        if (s_route_fixes[i].ident == s_no_star_direct_iaf) {
          s_iaf_route_idx = i;
          break;
        }
      }
    }
  }

  // No-STAR direct approach: load FAF + procedure waypoints on first entry.
  // Fires when there is no STAR (e.g. LFQA) so poll_approach can drive the
  // INFO/Tower handoff at the FAF without requiring an explicit check-in on
  // the approach frequency (the pilot may stay on the FIS frequency).
  if (s_approach_waypoints.empty() && s_approach_waypoint_idx == 0 &&
      s_assigned_star_name.empty() && !s_assigned_approach_designator.empty() &&
      !s_assigned_dest_icao.empty() && !ctx.cifp_dir.empty()) {
    if (s_approach_faf.ident.empty())
      s_approach_faf = cifp_reader::approach_faf(
          ctx.cifp_dir, s_assigned_dest_icao, s_assigned_approach_designator);
    auto iaf_ids = cifp_reader::approach_transition_idents(
        ctx.cifp_dir, s_assigned_dest_icao, s_assigned_approach_designator);
    std::string best_iaf_poll;
    if (!iaf_ids.empty()) {
      if (iaf_ids.size() == 1) {
        best_iaf_poll = iaf_ids[0];
      } else {
        auto iaf_pos = cifp_reader::lookup_fix_positions(
            ctx.cifp_dir, iaf_ids, s_assigned_dest_icao);
        double best_d = 1e9;
        for (const auto &id : iaf_ids) {
          auto it = iaf_pos.find(id);
          if (it == iaf_pos.end()) continue;
          double d = traffic_geometry::distance_nm(
              ctx.latitude, ctx.longitude,
              it->second.first, it->second.second);
          if (d < best_d) { best_d = d; best_iaf_poll = id; }
        }
        if (best_iaf_poll.empty()) best_iaf_poll = iaf_ids[0];
      }
    }
    const std::string iaf_ns = best_iaf_poll;
    auto proc_ns = cifp_reader::approach_procedure_waypoints(
        ctx.cifp_dir, s_assigned_dest_icao, s_assigned_approach_designator, iaf_ns);
    s_faf_ap_idx = -1; s_map_ap_idx = -1;
    for (auto &w : proc_ns) {
      int widx = static_cast<int>(s_approach_waypoints.size());
      s_approach_waypoints.push_back(w);
      if (s_faf_ap_idx < 0 && w.is_approach_proc && w.ident == s_approach_faf.ident)
        s_faf_ap_idx = widx;
      if (s_map_ap_idx < 0 && w.is_approach_proc && w.is_map)
        s_map_ap_idx = widx;
    }
    if (!s_approach_faf.ident.empty())
      s_approach_final_issued = true;
    if (s_route_fixes.empty())
      init_route_fixes(ctx);
    if (s_faf_route_idx < 0 && !s_approach_faf.ident.empty()) {
      for (int i = 0; i < static_cast<int>(s_route_fixes.size()); ++i) {
        if (s_route_fixes[i].ident == s_approach_faf.ident) {
          s_faf_route_idx = i;
          break;
        }
      }
    }
    if (s_iaf_route_idx < 0 && !iaf_ns.empty()) {
      for (int i = 0; i < static_cast<int>(s_route_fixes.size()); ++i) {
        if (s_route_fixes[i].ident == iaf_ns) {
          s_iaf_route_idx = i;
          logging::info("[route] IAF %s at route idx=%d (poll)", iaf_ns.c_str(), i);
          break;
        }
      }
    }
    logging::info("[approach] no-STAR poll load: FAF=%s faf_idx=%d wpts=%d",
                  s_approach_faf.ident.c_str(), s_faf_ap_idx,
                  static_cast<int>(s_approach_waypoints.size()));
  }

  // In APPROACH_CONTACT: wait for pilot INITIAL_CALL_APPROACH check-in before
  // issuing any proactive messages. ATC must never call an aircraft on a new
  // frequency — the aircraft always initiates. Exception: non-towered AFIS
  // airports (e.g. LFQA) with no STAR have no formal Approach to call, so the
  // engine proceeds directly to FAF handoff tracking without waiting.
  // Timer only counts while in APPROACH_DESCENT (or no-STAR APPROACH_CONTACT)
  // so 60-s guards are relative to actual entry, not the waiting period.
  // Use dest ICAO for the towered check — nearest airport may be a phantom.
  const bool dest_is_afis =
      !xplane_context::has_ground_freq_for(current_flight_airport(ctx));
  if (state == AS::IFR_APPROACH_CONTACT &&
      !(s_assigned_star_name.empty() && s_approach_final_issued && dest_is_afis))
    return false;

  // ── Sector-boundary handoff (IFR_APPROACH_DESCENT) ─────────────────────────
  // When the enclosing TRACON/CTR changes or disappears while in approach
  // descent, the current controller (e.g. Melun) hands off to the next sector
  // controller (e.g. Paris FIR Information) or to the destination TOWER/AFIS.
  // Uses the same 15-second polling as en-route sector-change sub-phase 1.5.
  if (state == AS::IFR_APPROACH_DESCENT && !s_approach_tower_handed_off &&
      airspace_db::enabled()) {
    s_approach_sector_check_sec -= dt;
    if (s_approach_sector_check_sec <= 0.0f) {
      s_approach_sector_check_sec = 15.0f;

      // Approach-phase pick: lower-ceiling tiebreak drives Geneva -> Chambery
      // as the aircraft descends below FL115 (both floor 1000; plain pick_next
      // would stay on Geneva). dest position resolves the facility tiebreak.
      auto dpos = s_assigned_dest_icao.empty()
                      ? std::pair<double, double>{0.0, 0.0}
                      : xplane_context::airport_pos_for(s_assigned_dest_icao);
      const airspace_db::Controller *best = sector_picker::pick_next_approach(
          ctx.enclosing_airspaces, s_approach_visited_sector_freqs,
          dpos.first, dpos.second);

      // openair-driven forward handoff. atc.dat (ctx.enclosing_airspaces) is
      // X-Plane's coarse ATC model and can model a field's controller as a small
      // TWR zone (Navigraph atc.dat: CHAMBERY = twr / 118.30 / <=3500 ft), so
      // pick_next_approach never sees the inner terminal TMA. Resolve the INNER
      // (lowest) enclosing TMA from openair -- the Navigraph airspace.txt, which
      // is authoritative for TMAs -- and hand FORWARD to its controller. This is
      // the Geneva->Chambery case (LIMF->LFLP 2026-07-14): at LUVOB the aircraft
      // leaves GENEVA TMA and enters CHAMBERY TMA (1000-9500). Freq via the same
      // fragment->controller path as the initial handoff, trying TRACON then TWR
      // (Chambery has no TRACON in the data -> resolves to Chambery TWR 118.30).
      bool force_forward = false;
      std::string forced_label; // spoken label when freq comes via the facility link
      bool stay = false;      // openair says: still in current sector -> no handoff
      int  new_ceiling = 0;   // openair ceiling of the sector we hand off to
      if (openair_db::ready()) {
        const openair_db::AirspaceEntry inner = openair_db::find_enclosing(
            ctx.latitude, ctx.longitude, openair_alt(ctx));
        if (inner.ac_class == openair_db::AirspaceClass::TMA &&
            inner.ceiling_ft > 0) {
          // Shared terminal resolver (step 1c): openair inner-TMA name -> the atc.dat
          // controller to tune (freq source) + label source. Freq via the FACILITY
          // link so a field whose TRACON is labelled differently still resolves:
          // CHAMBERY (twr) -> facility LFLB -> LFLB TRACON 121.205; spoken label kept
          // as the field ("Chambery Approach"), never the tracon's name ("Lyon")
          // (LFLP 2026-07-15). Was an inline duplicate of resolve_tma_controller.
          const airspace_db::Controller *label_ctrl = nullptr;
          const airspace_db::Controller *oc =
              resolve_terminal_ctrl(inner.name, &label_ctrl);
          if (oc)
            forced_label = controller_label_for(label_ctrl) + " Approach";
          if (oc && !oc->freqs_khz.empty()) {
            const uint32_t of = oc->freqs_khz.front();
            if (of == s_approach_sector_freq_khz) {
              stay = true; // still in the current sector's own TMA -> hold
            } else {
              bool visited = false;
              for (uint32_t f : s_approach_visited_sector_freqs)
                if (f == of) { visited = true; break; }
              // FORWARD only: hand off to the inner TMA solely when it is MORE
              // terminal (lower openair ceiling) than the current sector and not
              // already visited. Otherwise HOLD -- never go to a larger/higher
              // overlapping sector (Chambery->Lyon) or backward.
              const bool more_terminal =
                  (s_approach_sector_ceiling_ft == 0 ||
                   inner.ceiling_ft < s_approach_sector_ceiling_ft);
              if (!visited && more_terminal) {
                best = oc;
                force_forward = true;
                new_ceiling = inner.ceiling_ft;
              } else {
                stay = true;
              }
            }
          } else if (s_approach_sector_freq_khz != 0) {
            // Over a TMA whose controller can't be resolved -> keep current;
            // do NOT fall through to the atc.dat picker or the Tower fallback.
            stay = true;
          }
        } else if (inner.ac_class == openair_db::AirspaceClass::CTA &&
                   inner.ceiling_ft > 0) {
          // openair authoritative for CTA too, but STAY-ONLY (never force a
          // handoff). Above the terminal TMA ceiling the innermost ACCURATE volume
          // is a sectorised, altitude-banded CTA: at LFMN, NICE CTA SECTOR 2
          // (11500-14500 MSL) is NICE, and Marseille only owns the layer ABOVE
          // FL145. X-Plane's coarse atc.dat model has no such sectorisation -- one
          // Marseille controller blankets the whole area at all altitudes -- so
          // pick_next_approach(ctx.enclosing_airspaces) invents a BACKWARD handoff
          // to Marseille after the approach is already cleared (LFMN 2026-07-19,
          // held at FL120 over MUS, ~1400 ft above the Nice TMA). Trust openair: if
          // the inner CTA resolves to the controller the aircraft is ALREADY on,
          // HOLD. Stay-only by design -- a CTA can only SUPPRESS the spurious
          // handoff, never create one -- so LFLP's TMA-driven forward handoffs
          // (Geneva->Chambery, SALEV3P) are provably unchanged.
          const airspace_db::Controller *lc = nullptr;
          const airspace_db::Controller *oc =
              resolve_terminal_ctrl(inner.name, &lc);
          // SCOPE GUARD (SALEV3P cross-check 2026-07-19): fire ONLY for the
          // DESTINATION's own stacked CTA. NICE CTA SECTOR 2 -> NICE APPROACH ->
          // facility LFMN == dest, so a stay is correct. But an ENROUTE CTA can
          // also be the innermost volume mid-descent -- verified on the LIMF->LFLP
          // SALEV3P arrival, MILAN CTA ZONE 9/24 is innermost at 8-17 kft (facility
          // LIMx != dest). Requiring oc->facility_id == dest excludes every enroute
          // CTA, so enroute/ACC handoffs (Milan->Geneva->Chambery) are provably
          // untouched -- the CTA branch can only keep the aircraft on the arrival
          // field's OWN approach when it is briefly above that field's TMA.
          if (oc && !oc->freqs_khz.empty() &&
              !s_assigned_dest_icao.empty() &&
              oc->facility_id == s_assigned_dest_icao) {
            const uint32_t of = oc->freqs_khz.front();
            const float acom =
                (ctx.active_com == 2) ? ctx.com2_freq_mhz : ctx.com1_freq_mhz;
            const uint32_t acom_khz =
                static_cast<uint32_t>(std::lround(acom * 1000.0f));
            if (of == s_approach_sector_freq_khz || of == acom_khz)
              stay = true;
          }
        }
      }
      if (stay)
        best = nullptr; // suppress this cycle's handoff entirely

      if (best) {
        uint32_t new_freq_khz = best->freqs_khz.front();
        const float acom_seed =
            (ctx.active_com == 2) ? ctx.com2_freq_mhz : ctx.com1_freq_mhz;
        if (s_approach_sector_freq_khz == 0) {
          // Baseline = the sector the pilot is ACTUALLY on (their active COM), NOT
          // the resolved inner TMA. After the silent ARRIVAL->APPROACH flip the
          // pilot may still be on the OUTER sector (Geneva) while find_enclosing
          // already resolves the INNER TMA (Chambery). Seeding the inner freq here
          // marked the Geneva->Chambery handoff "done" without it happening,
          // leaving the pilot stuck on Geneva which then wrongly cleared the
          // approach (LFLP 2026-07-16). Seeding the current freq lets the
          // change-detection below fire the real handoff on the next poll.
          if (std::fabs(acom_seed - static_cast<float>(new_freq_khz) / 1000.0f) <
              0.005f) {
            s_approach_sector_freq_khz = new_freq_khz;
            s_approach_sector_ceiling_ft = new_ceiling;
            logging::info("[approach] sector baseline: %s %.3f MHz floor=%dft",
                          best->name.c_str(),
                          static_cast<float>(new_freq_khz) / 1000.0f,
                          best->floor_ft);
          } else {
            s_approach_sector_freq_khz =
                static_cast<uint32_t>(std::lround(acom_seed * 1000.0f));
            s_approach_sector_ceiling_ft = 0; // outer sector -> inner always wins
            logging::info("[approach] sector baseline = current freq %.3f MHz "
                          "(inner %s %.3f pending handoff)",
                          acom_seed, best->name.c_str(),
                          static_cast<float>(new_freq_khz) / 1000.0f);
          }
        } else if (new_freq_khz != s_approach_sector_freq_khz &&
                   (force_forward || [&] {
                     // "Lowest enclosing volume in charge": do NOT hand off
                     // while the CURRENT controller's volume still encloses the
                     // aircraft, UNLESS the candidate is MORE TERMINAL (a
                     // descent into a tighter sector -- the legit Geneva ->
                     // Chambery case). pick_next_approach hides the current
                     // controller via the visited-freq guard, so without this
                     // the picker hands a still-enclosed aircraft off to the
                     // next non-visited enclosing sector -- a LARGER overlapping
                     // CTR (Chambery -> Lyon) -- even though the aircraft never
                     // left Chambery's volume (LIMx->LFLP 2026-07-12, near GOVNA
                     // 19 NM from the FAF). If the current controller no longer
                     // encloses the aircraft (left its volume), hand off freely.
                     const airspace_db::Controller *current = nullptr;
                     for (const auto *c : ctx.enclosing_airspaces) {
                       if (c && !c->freqs_khz.empty() &&
                           c->freqs_khz.front() == s_approach_sector_freq_khz) {
                         current = c;
                         break;
                       }
                     }
                     if (current && !sector_picker::more_terminal(best, current)) {
                       logging::info(
                           "[approach] keep %s: still enclosing, %s not more terminal -- no handoff",
                           s_current_controller_label.c_str(), best->name.c_str());
                       return false;
                     }
                     return true;
                   }())) {
          // Record the OUTGOING freq so this sector cannot be re-elected
          // later in the same approach phase (block backward handoff).
          s_approach_visited_sector_freqs.push_back(s_approach_sector_freq_khz);
          // Sector changed to another controller (e.g. Melun → Paris FIR Info).
          s_approach_sector_freq_khz = new_freq_khz;
          s_approach_sector_ceiling_ft = new_ceiling; // track for the next more-terminal test
          // Use the facility-link label ("Chambery Approach") when set, so the
          // handoff isn't spoken as the differently-named tracon ("Lyon").
          std::string new_label =
              !forced_label.empty() ? forced_label : controller_label_for(best);
          float new_mhz = static_cast<float>(new_freq_khz) / 1000.0f;
          // Defer label switch — see s_pending_controller_label comment.
          s_pending_controller_label = new_label;
          s_pending_handoff_freq_mhz = new_mhz;
          // Also update the "active approach freq" gate so the check-in
          // handler (engine.cpp line ~820) doesn't accept a call on the
          // OLD sector's freq. Without this the check-in fires while the
          // pilot is still on the previous sector's frequency.
          s_enroute_approach_freq_mhz = new_mhz;
          // Suppress "contact X on Y" when the pilot is already on that
          // frequency (same rationale as the en-route sector-change guard).
          const float active_com_ac =
              (ctx.active_com == 2) ? ctx.com2_freq_mhz : ctx.com1_freq_mhz;
          if (std::fabs(active_com_ac - new_mhz) < 0.005f) {
            s_sector_checkin_pending = false;
            logging::info("[approach] sector change -> %s %.3f MHz (pilot already on freq -- silent)",
                          new_label.c_str(), new_mhz);
            return false;
          }
          s_sector_checkin_pending = true;
          const std::string &cs_s2 = atc_state_machine::session_callsign();
          const std::string &cs2 = cs_s2.empty() ? settings::pilot_callsign() : cs_s2;
          if (out_text) {
            char buf[160];
            std::snprintf(buf, sizeof(buf), "%s, contact %s on %.3f.",
                          cs2.c_str(), new_label.c_str(), new_mhz);
            *out_text = buf;
          }
          logging::info("[approach] sector change -> %s %.3f MHz",
                        new_label.c_str(), new_mhz);
          rb(true);
          return true;
        }
      } else if (!stay && s_approach_sector_freq_khz != 0 && out_text) {
        // No TRACON/CTR remaining: aircraft has left the approach sector
        // (e.g. left Melun TMA). Hand off to destination TOWER or INFO/AFIS.
        // FAF gate: a small TMA (e.g. Chambery) can be exited 10+ NM before the
        // FAF; Approach keeps working the aircraft (radar) until ~the FAF, so
        // the "contact Tower, report established" call must not fire that early
        // (LIMF -> LFLP 2026-07-11: Tower given at 13.9 NM on Chambery exit).
        // When the FAF is known, defer until within 4 NM (or past it); the
        // FAF-proximity handoff below (< 2 NM) also covers it. FAF unknown
        // (AFIS field with no CIFP FAF) keeps the on-exit behaviour.
        if (s_approach_faf.lat != 0.0 || s_approach_faf.lon != 0.0) {
          const double faf_nm = traffic_geometry::distance_nm(
              ctx.latitude, ctx.longitude, s_approach_faf.lat, s_approach_faf.lon);
          const bool faf_passed =
              (s_faf_route_idx >= 0 && s_route_fix_idx > s_faf_route_idx);
          if (faf_nm > 4.0 && !faf_passed)
            return false; // too early -- stay with Approach until near the FAF
        }
        float tower_mhz = 0.0f;
        if (!s_assigned_dest_icao.empty())
          tower_mhz = xplane_context::tower_mhz_for(s_assigned_dest_icao);
        if (tower_mhz <= 100.0f)
          tower_mhz = ctx.airport_freqs.first_mhz(xplane_context::FrequencyType::TOWER);
        bool is_info_svc = false;
        if (tower_mhz <= 100.0f) {
          tower_mhz = ctx.airport_freqs.first_mhz(xplane_context::FrequencyType::ATIS);
          if (tower_mhz > 100.0f)
            is_info_svc = true;
        } else if (!xplane_context::has_ground_freq_for(
                       current_flight_airport(ctx))) {
          is_info_svc = true;
        }
        s_approach_tower_handed_off = true;
        // Hold the alignment "confirm established" nag for a bit after the Tower
        // handoff: without this it fires on the Tower freq BEFORE the pilot's first
        // Tower call, spoken with the stale Approach label ("Nice Approach: confirm
        // established" while the pilot is on 118.700 about to check in -- LFMN
        // 2026-07-20, "weird, I just called in"). By the time it elapses the pilot
        // has normally checked in + been cleared (-> LANDING_CLEARED, poll stops),
        // so it only ever fires for a genuinely-late, off-centerline aircraft.
        s_alignment_cooldown = 45.0f;
        const std::string &cs_s2 = atc_state_machine::session_callsign();
        const std::string &cs2 = cs_s2.empty() ? settings::pilot_callsign() : cs_s2;
        char buf[128];
        if (tower_mhz > 100.0f) {
          int khz = static_cast<int>(std::round(tower_mhz * 1000.0f));
          std::string ctrl_label;
          if (is_info_svc) {
            using FT = xplane_context::FrequencyType;
            std::string raw = ctx.airport_freqs.first_name(FT::TOWER);
            if (raw.empty())
              raw = ctx.airport_freqs.first_name(FT::ATIS);
            if (!raw.empty()) {
              bool cap = true;
              for (char c2 : raw) {
                ctrl_label +=
                    cap ? static_cast<char>(std::toupper(static_cast<unsigned char>(c2)))
                        : static_cast<char>(std::tolower(static_cast<unsigned char>(c2)));
                cap = (c2 == ' ');
              }
            } else {
              // Prefer the destination airport NAME over the ICAO code —
              // ICAO codes are never spoken over the radio.
              std::string apt_name2;
              if (!s_assigned_dest_icao.empty())
                apt_name2 = xplane_context::airport_name_for(s_assigned_dest_icao);
              if (!apt_name2.empty())
                ctrl_label = apt_name2 + " Information";
              else
                ctrl_label = s_assigned_dest_icao.empty()
                                 ? "Information"
                                 : (s_assigned_dest_icao + " Information");
            }
          } else {
            ctrl_label = "Tower";
          }
          std::snprintf(buf, sizeof(buf), "%s, contact %s on %d.%03d.",
                        cs2.c_str(), ctrl_label.c_str(), khz / 1000, khz % 1000);
          // Update the current controller label so the next TTS speaker prefix
          // matches the new facility (e.g. "Reims Prunay Information:" instead
          // of the previous sector's "Paris:").
          s_current_controller_label = ctrl_label;
          s_pending_handoff_freq_mhz = tower_mhz;
        } else {
          std::snprintf(buf, sizeof(buf), "%s, contact Tower.", cs2.c_str());
        }
        *out_text = buf;
        atc_state_machine::set_state(AS::IFR_APPROACH_TOWER);
        logging::info("[approach] sector exit: dest Tower/Info %.3f MHz (sector was %u kHz)",
                      tower_mhz, s_approach_sector_freq_khz);
        rb(tower_mhz > 100.0f);
        return true;
      }
    }
  }

  // A sector handoff is pending ("contact X on Y"): the current controller has
  // handed the aircraft off and must stay SILENT until the pilot checks in on
  // the new frequency. Without this the step-down walker kept issuing
  // "direct LP402, descend 5000" on the OLD (Geneva) frequency right after
  // "contact Chambery" (LIMF -> LFLP 2026-07-11). Cleared by the check-in on
  // the new freq (engine.cpp ~877), then the walker resumes under Chambery.
  if (s_sector_checkin_pending)
    return false;

  s_approach_timer += dt;

  // IFR_APPROACH_DESCENT: step through constrained waypoints.
  // Trigger: aircraft has descended within 10% above the constraint altitude
  // (or 3-minute fallback after previous clearance).
  const std::string &cs_ref2 = atc_state_machine::session_callsign();
  const std::string cs =
      cs_ref2.empty() ? settings::pilot_callsign() : cs_ref2;

  // Approach clearance timing: confirm the approach as the aircraft NEARS the
  // IAF (~60 s out), not one fix later. The IAF is the last STAR fix = the route
  // fix immediately before the first approach-transition fix; its own CIFP leg
  // (IF/FM) is skipped, so keying the clearance on "reaching the first
  // approach-proc fix" fired it one fix PAST the IAF (LFMN R22LZ: at MN261
  // instead of approaching MUS). Time-based (eta), so it scales with GS. A
  // descent is folded in only if the next fix would otherwise be busted.
  // FREQUENCY gate (LFLP 2026-07-17 regression): the clearance must be spoken by
  // the controller the pilot is ACTUALLY on -- not just resolved from the airspace.
  // on_destination_terminal() confirms the aircraft is in the terminal TMA
  // (Chambery), but the aircraft descends INTO that airspace while still on the
  // OUTER freq (Geneva) between the "contact Chambery" handoff and the pilot's
  // switch -- so Geneva was speaking "cleared RNAV approach" on 119.530. Require
  // the active COM to match the terminal approach freq (set by the handoff). When
  // no approach freq is latched yet (< 100), stay permissive.
  const float active_com_clr =
      (ctx.active_com == 2) ? ctx.com2_freq_mhz : ctx.com1_freq_mhz;
  const bool on_terminal_freq =
      s_enroute_approach_freq_mhz < 100.0f ||
      std::fabs(active_com_clr - s_enroute_approach_freq_mhz) < 0.010f;
  if (!s_approach_cleared_issued && !dest_is_afis && out_text &&
      !s_route_fixes.empty() && on_destination_terminal(ctx) && on_terminal_freq) {
    // Option B: only fire once the aircraft is on the destination's TERMINAL
    // controller (in the terminal TMA -- e.g. Chambery), never from an outer TMA
    // (Geneva). Combined with the on-terminal gate, the eta window below issues it
    // shortly after the terminal check-in, >=3 NM before the IAF (LFLP 2026-07-15).
    // Anchor the "cleared approach" at the SELECTED IAF -- the STAR-terminating
    // IAF (star_last_fix) or the no-STAR direct IAF -- NOT the first approach-proc
    // route fix. On an approach with several IAFs (LFLP R04-Z: COLLO/PIRUV/TOLNA)
    // every IAF/transition fix is flagged is_approach_proc, so "first approach-
    // proc" landed on COLLO instead of the chosen IAF PIRUV, firing the clearance
    // (and shifting the whole FAF/Tower timing) far too early (LIMF->LFLP
    // 2026-07-14). The STAR always terminates at its IAF, so star_last_fix is the
    // authoritative entry for a STAR arrival.
    std::string sel_iaf = s_no_star_direct_iaf;
    if (sel_iaf.empty())
      sel_iaf = cifp_reader::star_last_fix(ctx.cifp_dir, s_assigned_dest_icao,
                                           s_assigned_star_name);
    int iaf_idx = -1;
    if (!sel_iaf.empty()) {
      for (int i = std::max(0, s_route_fix_idx);
           i < static_cast<int>(s_route_fixes.size()); ++i)
        if (s_route_fixes[i].ident == sel_iaf) { iaf_idx = i; break; }
    }
    if (iaf_idx < 0) { // fallback: the fix before the first approach-proc waypoint
      int first_ap = -1;
      for (int i = std::max(0, s_route_fix_idx);
           i < static_cast<int>(s_route_fixes.size()); ++i)
        if (s_route_fixes[i].is_approach_proc) { first_ap = i; break; }
      iaf_idx = (first_ap > 0) ? first_ap - 1 : first_ap;
    }
    if (iaf_idx >= 0 && iaf_idx < static_cast<int>(s_route_fixes.size())) {
      const auto &iaf = s_route_fixes[iaf_idx];
      if (iaf.lat != 0.0 || iaf.lon != 0.0) {
        const double gs = ctx.groundspeed_kts > 40.0f
                              ? static_cast<double>(ctx.groundspeed_kts)
                              : 120.0;
        const double eta = traffic_geometry::distance_nm(
                               ctx.latitude, ctx.longitude, iaf.lat, iaf.lon) /
                           gs * 3600.0;
        // On-terminal already gates this to the Chambery TMA; fire across a wider
        // window (<=180 s, ~10 NM) so it lands right after the terminal check-in
        // rather than waiting to ~3 NM (Option B). Still >=3 NM before the IAF.
        // Sequence guard (same as the ARRIVAL->APPROACH trigger): the route TRACKER
        // must be within 2 fixes of the IAF -- SEQUENCE, not distance. On the
        // looping SALE3P, COLLO sits ~2 NM from PIRUV but 4 fixes earlier, so a
        // distance test fired the clearance at COLLO (LFLP 2026-07-16). The resync
        // keeps the tracker reliable through wide flying.
        if (settings::debug_logging())
          logging::info("[dbg appclr] eta=%.0f route_idx=%d iaf_idx=%d (need>=%d) "
                        "on_term=1 checkin_pending=%d", eta, s_route_fix_idx,
                        iaf_idx, iaf_idx - 2, s_sector_checkin_pending ? 1 : 0);
        // Wait until the pilot has CHECKED IN on the terminal (approach)
        // controller before issuing the clearance. Firing while a handoff is still
        // pending clears the approach on the NEW freq before the pilot arrives --
        // spoken with the OLD controller's stale label ("Geneva" on Chambery's
        // 121.205) AND it arms a runway readback that then eats the pilot's
        // check-in as "negative, runway zero four, readback" (LFLP Chambery
        // 2026-07-19). s_sector_checkin_pending clears on the first call on the new
        // freq, so the clearance fires right after the check-in ack, correctly
        // labelled.
        if (eta <= 180.0 && s_route_fix_idx >= iaf_idx - 2 &&
            !s_sector_checkin_pending) {
          const std::string phrase = approach_clearance_phrase(ctx);
          if (!phrase.empty()) {
            s_approach_cleared_issued = true;
            std::string msg = cs + ", cleared " + phrase;
            const FixCompliance fc = check_next_fix(ctx, 60.0);
            if (fc.valid && fc.near && fc.alt_bust) {
              msg += ", descend " +
                     format_alt_clearance(fc.alt_target_ft,
                                          fc.alt_is_fl ? AltHint::FlightLevel
                                                       : AltHint::Feet,
                                          ctx.qnh_hpa, ctx.transition_alt_ft);
              s_enroute_cleared_alt_ft = fc.alt_target_ft;
            }
            // QNH with the approach clearance: the pilot flies the approach /
            // descends to minima against this QNH, so ICAO passes it with the
            // clearance (user 2026-07-20). Append only when a descend-to-feet
            // clause (which already carries QNH via format_alt_clearance) was not
            // added, to avoid stating it twice.
            if (ctx.qnh_hpa > 0 && msg.find("QNH") == std::string::npos) {
              char qbuf[24];
              std::snprintf(qbuf, sizeof(qbuf), ", QNH %d", ctx.qnh_hpa);
              msg += qbuf;
              s_qnh_stated = true;
            }
            msg += ".";
            *out_text = msg;
            logging::info(
                "[approach] cleared approach approaching IAF %s (eta %.0f s)",
                iaf.ident.c_str(), eta);
            rb(true);
            return true;
          }
        }
      }
    }
  }

  // Skip waypoints the aircraft is already in compliance with — no instruction
  // needed for a constraint the aircraft already meets.
  // Also skip unconstrained STAR routing waypoints (no altitude, no speed) —
  // these are plain route fixes with no ATC action; skip to the next constrained
  // fix so the clearance names the real target (e.g. "direct BISBO" not MN141).
  while (s_approach_waypoint_idx < static_cast<int>(s_approach_waypoints.size())) {
    const auto &wp = s_approach_waypoints[s_approach_waypoint_idx];
    // Don't silently skip MAP or post-MAP via already_compliant —
    // the step-down block handles them explicitly.
    if (wp.is_approach_proc &&
        (wp.is_map || (s_map_ap_idx >= 0 && s_approach_waypoint_idx > s_map_ap_idx)))
      break;
    // Unconstrained routing fix (no altitude, no speed) — skip silently,
    // keep route tracker in sync. Applies to both STAR and approach-proc
    // fixes (e.g. MAP/NERAS which has no altitude constraint but blocks
    // the Tower handoff if left in the queue).
    if (wp.alt.feet == 0 && wp.speed_kt == 0) {
      if (!wp.ident.empty()) {
        for (int ri = s_route_fix_idx;
             ri < static_cast<int>(s_route_fixes.size()); ++ri) {
          if (s_route_fixes[ri].ident == wp.ident) {
            s_route_fix_idx = ri;
            break;
          }
        }
      }
      s_approach_waypoint_idx++;
      continue;
    }
    if (wp.alt.feet > 0) {
      const float wp_ft = static_cast<float>(wp.alt.feet);
      // 200 ft tolerance for ceiling constraints: pressure altimeter error
      // and residual QNH offsets mean PA can be slightly above the cleared FL.
      bool already_compliant;
      if (wp.floor_ft > 0) {
        // Block "B" (e.g. MN261 FL120/FL080): the operative lower limit is the
        // FLOOR. Not compliant until at/below the floor — otherwise we skip the
        // fix and clear the next (lower) fix BELOW the block floor
        // (LFMN MN261 -> SOTOX FL070). Cleared to the floor in the issue loop.
        already_compliant =
            ctx.pressure_alt_ft <= static_cast<float>(wp.floor_ft) + 200.0f;
      } else {
        already_compliant =
            (wp.is_ceiling  && ctx.pressure_alt_ft <= wp_ft + 200.0f) ||
            (wp.is_floor    && ctx.pressure_alt_ft >= wp_ft) ||
            (!wp.is_ceiling && !wp.is_floor && ctx.pressure_alt_ft <= wp_ft + 200.0f);
      }
      if (already_compliant) {
        s_approach_waypoint_idx++;
        s_approach_timer = 0.0f;
        continue;
      }
    }
    break;
  }

  // ── Approach → Tower handoff: "contact Tower when established" ──────────
  // Checked BEFORE the waypoint loop so it fires regardless of any remaining
  // queued waypoints (e.g. MN04A constraint not yet cleared).
  // APP hands off to Tower when the aircraft is established on final (at FAF).
  if (s_approach_final_issued && !s_approach_tower_handed_off) {
    bool at_faf = false;
    if (s_faf_route_idx >= 0) {
      // Primary: route tracker has passed the FAF fix.
      // Suppressed when the dual-use IAF/MAP-hold fix (s_iaf_route_idx) sits after
      // the FAF in s_route_fixes: the IAF itself is skipped by approach_procedure_waypoints
      // (IF path_term), but the same fix reappears as the missed-approach hold (DF path_term).
      // After a direct-to-IAF, step 4 advances route_idx past that hold (idx>iaf_idx), which
      // would fire primary immediately at the IAF position. Use distance-only in that case.
      if (s_iaf_route_idx < 0 || s_iaf_route_idx <= s_faf_route_idx)
        at_faf = (s_route_fix_idx > s_faf_route_idx);
      // Fallback: direct distance to FAF — always valid; only trigger when guard clears.
      if (!at_faf && (s_approach_faf.lat != 0.0 || s_approach_faf.lon != 0.0)) {
        double dist_nm = traffic_geometry::distance_nm(
            ctx.latitude, ctx.longitude, s_approach_faf.lat, s_approach_faf.lon);
        at_faf = (dist_nm < 2.0);
      }
    } else if (s_approach_faf.lat != 0.0 || s_approach_faf.lon != 0.0) {
      double dist_nm = traffic_geometry::distance_nm(
          ctx.latitude, ctx.longitude, s_approach_faf.lat, s_approach_faf.lon);
      at_faf = (dist_nm < 2.0);
    } else if (s_approach_faf.alt_ft > 0) {
      at_faf = (ctx.pressure_alt_ft <=
                static_cast<float>(s_approach_faf.alt_ft) * 1.1f);
    } else {
      double dist_to_apt = traffic_geometry::distance_nm(
          ctx.latitude, ctx.longitude, ctx.airport_lat, ctx.airport_lon);
      at_faf = (ctx.height_agl_ft < 3500.0f && dist_to_apt < 12.0);
    }

    // Guard: when CIFP body records precede the IAF transition (s_iaf_route_idx >
    // s_faf_route_idx), the IAF lands AFTER the FAF in s_route_fixes. The aircraft
    // flying outbound to the IAF will pass over the FAF lat/lon and trigger at_faf
    // prematurely. Suppress until the route tracker has actually passed the IAF.
    if (at_faf && s_iaf_route_idx > s_faf_route_idx && s_faf_route_idx >= 0 &&
        s_route_fix_idx <= s_iaf_route_idx) {
      logging::debug("[approach] at_faf suppressed (IAF not yet passed): "
                     "route_idx=%d iaf_idx=%d faf_idx=%d",
                     s_route_fix_idx, s_iaf_route_idx, s_faf_route_idx);
      at_faf = false;
    }

    if (at_faf) {
      s_approach_tower_handed_off = true;
      // Visual-final (MDA) vs instrument (DA) -> "report runway in sight" vs
      // "report established". CIFP-derived: an approach that terminates at the
      // runway threshold (RWxx leg) is a straight-in instrument approach to a
      // DA; one with no runway leg (ends at fixes + missed-approach hold, e.g.
      // LFMN R04LA/R22LD) is flown visually on the last segment (MDA). Replaces
      // the old track-vs-runway-heading > 30deg guess, which wrongly flagged
      // CURVED (RF) instrument finals like R22LZ as visual (LFMN 2026-07-12:
      // R22LZ has a DA + runway leg, but its curved final tripped the guess ->
      // bogus "report runway in sight"). Pilot-requested visual approaches are
      // a separate future feature.
      s_approach_has_visual_final =
          !s_assigned_approach_designator.empty() &&
          !cifp_reader::approach_terminates_at_runway(
              ctx.cifp_dir, s_assigned_dest_icao, s_assigned_approach_designator);
      logging::info("[approach] Tower: designator=%s rwy=%s visual-final(MDA)=%d",
                    s_assigned_approach_designator.c_str(),
                    s_assigned_landing_runway.c_str(),
                    s_approach_has_visual_final ? 1 : 0);
      float tower_mhz = 0.0f;
      if (!s_assigned_dest_icao.empty())
        tower_mhz = xplane_context::tower_mhz_for(s_assigned_dest_icao);
      if (tower_mhz <= 100.0f)
        tower_mhz = ctx.airport_freqs.first_mhz(
            xplane_context::FrequencyType::TOWER);
      // No tower: fall back to ATIS/INFO service (e.g. "Reims-Prunay Information" at LFQA).
      bool is_info_svc = false;
      if (tower_mhz <= 100.0f) {
        tower_mhz =
            ctx.airport_freqs.first_mhz(xplane_context::FrequencyType::ATIS);
        if (tower_mhz > 100.0f)
          is_info_svc = true;
      } else if (!xplane_context::has_ground_freq_for(
                     current_flight_airport(ctx))) {
        // No Ground freq for the destination → AFIS/Information service, not a
        // real Tower controller. current_flight_airport() returns the bound
        // destination here so a phantom airport near it can't corrupt the check.
        is_info_svc = true;
      }
      if (out_text) {
        char buf[128];
        // EUROCONTROL: visual-final approaches (e.g. LFMN RNAV 04L Alpha, whose
        // last segment is flown visually) are handed to Tower with "report
        // runway in sight"; instrument straight-in with "report established".
        const char *final_call = s_approach_has_visual_final
                                     ? "report runway in sight"
                                     : "report established";
        if (tower_mhz > 100.0f) {
          int khz = static_cast<int>(std::round(tower_mhz * 1000.0f));
          std::string ctrl_label;
          if (is_info_svc) {
            // Use the apt.dat name for the INFO/AFIS frequency (title-cased).
            // Try TOWER name first (AFIS stored as Tower type), then ATIS name.
            using FT = xplane_context::FrequencyType;
            std::string raw = ctx.airport_freqs.first_name(FT::TOWER);
            if (raw.empty())
              raw = ctx.airport_freqs.first_name(FT::ATIS);
            if (!raw.empty()) {
              // Title-case: "REIMS PRUNAY INFORMATION" → "Reims Prunay Information"
              bool cap = true;
              for (char c : raw) {
                ctrl_label += cap ? static_cast<char>(std::toupper(static_cast<unsigned char>(c)))
                                  : static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
                cap = (c == ' ');
              }
            } else {
              // Fallback: use the destination's airport NAME (e.g. "Reims
              // Prunay") rather than the ICAO code. ICAO codes are printed
              // in flight planning but never spoken over the radio ("contact
              // LFQA Information" is unnatural). Only fall back to ICAO if
              // the airport name is also unavailable.
              std::string apt_name;
              if (!s_assigned_dest_icao.empty())
                apt_name = xplane_context::airport_name_for(s_assigned_dest_icao);
              if (!apt_name.empty())
                ctrl_label = apt_name + " Information";
              else
                ctrl_label = s_assigned_dest_icao.empty() ? "Information"
                                                          : (s_assigned_dest_icao + " Information");
            }
          } else {
            ctrl_label = "Tower";
          }
          std::snprintf(buf, sizeof(buf),
                        "%s, contact %s on %d.%03d, %s.",
                        cs.c_str(), ctrl_label.c_str(), khz / 1000, khz % 1000, final_call);
          // Set the label as PENDING (not current): the handoff is spoken by the
          // CURRENT controller (Chambery Approach) and the label swaps to Tower only
          // when the pilot actually reaches 118.200 (deferred-swap promoter). Setting
          // s_current here made the handoff read "Tower:" while still on 121.205, and
          // the wrong-freq reminder said "you are still with Tower, contact the next
          // controller on 118.200" instead of "still with Chambery Approach, contact
          // Tower on 118.200" (LFLP 2026-07-17).
          s_pending_controller_label = ctrl_label;
          s_pending_handoff_freq_mhz = tower_mhz;
          s_sector_checkin_pending   = true;
        } else {
          std::snprintf(buf, sizeof(buf),
                        "%s, contact Tower, %s.", cs.c_str(), final_call);
        }
        *out_text = buf;
        atc_state_machine::set_state(AS::IFR_APPROACH_TOWER);
        rb(tower_mhz > 100.0f); // only arm readback when a frequency was given
        return true;
      }
    }
  }

  if (s_approach_waypoint_idx < static_cast<int>(s_approach_waypoints.size())) {
    const auto &wp = s_approach_waypoints[s_approach_waypoint_idx];

    // Route-tracker fix trigger: fires when the aircraft passes the last-cleared
    // fix (route tracker advances past s_last_cleared_route_idx). This ensures
    // the next step-down fires as the aircraft reaches the previous cleared fix,
    // not when it happens to descend through an altitude band prematurely.
    // Falls back to 3-minute timer when no step-down has been issued yet.
    bool fix_trigger  = (s_last_cleared_route_idx >= 0 &&
                         s_route_fix_idx > s_last_cleared_route_idx);
    bool time_trigger = (s_approach_timer > 180.0f);

    if (!fix_trigger && !time_trigger)
      return false;

    // Cleared FL: for ceiling constraints, clear to that FL.
    // For floor constraints (at-or-above), descend to the floor value
    // so the crew meets the constraint.
    int cleared_ft = wp.alt.feet;
    // Block "B": clear to the FLOOR (lower bound), not the ceiling in wp.alt —
    // descend to the bottom of the block, never below it before the fix.
    if (wp.floor_ft > 0)
      cleared_ft = wp.floor_ft;
    if (cleared_ft == 0 && wp.speed_kt > 0)
      cleared_ft = static_cast<int>(ctx.pressure_alt_ft / 100) * 100; // maintain current

    if (cleared_ft > 0 && out_text) {
      if (wp.is_approach_proc) {
        // MAP: no ATC clearance — crew follows the chart from here.
        // Post-MAP: GO_AROUND territory — skip unless a GO_AROUND was fired.
        if (wp.is_map || (s_map_ap_idx >= 0 && s_approach_waypoint_idx > s_map_ap_idx)) {
          s_approach_waypoint_idx++;
          s_approach_timer = 0.0f;
          return false;
        }
        // Sequential next-fix walk (the ++ below advances to target+1). No
        // shortcut is ever fabricated: real ATC does not shortcut to an
        // arbitrary intermediate STAR/approach fix; a direct-to is issued only
        // to an IAF, which the no-STAR path handles. The walk here is silent on
        // fix names (see the announcement block below) -- it only drives the
        // procedure-tied descents + the single IAF approach clearance.
        const int target_idx = s_approach_waypoint_idx;
        const auto &twp = s_approach_waypoints[target_idx];
        // Honor the CIFP constraint's is_fl WHENEVER the target waypoint has a
        // published altitude -- including the normal (non-shortcut) case where
        // target_idx == s_approach_waypoint_idx. The old
        // "target_idx != s_approach_waypoint_idx" guard meant a direct-to the
        // CURRENT fix fell back to the TA threshold and spoke LP403's 06500
        // (is_fl=false) as "flight level 65" instead of "6500 feet"
        // (LIMF -> LFLP 2026-07-11).
        const bool twp_has_alt = (twp.alt.feet > 0);
        const int ta_c = ctx.transition_alt_ft > 0 ? ctx.transition_alt_ft : 5000;
        int  tft = twp_has_alt ? twp.alt.feet : cleared_ft;
        bool tfl = twp_has_alt ? twp.alt.is_fl : (tft > ta_c);
        // Block "B" on an approach fix (e.g. LFMN R22LZ MN261 FL120/FL080):
        // clear to the FLOOR. The floor is a plain feet value, so decide FL vs
        // feet by the transition level rather than the ceiling's is_fl flag.
        if (twp.floor_ft > 0) {
          tft = twp.floor_ft;
          tfl = (tft >= compute_tl_ft(ta_c, ctx.qnh_hpa));
        }
        // Positional block gate: never clear below the nearest UNPASSED block
        // floor -- even when the walker skipped the block fix on altitude
        // (LFMN R22LZ: at MUS/FL080 the walker grabbed SOTOX's FL070, below
        // MN261's FL080 block floor). Releases once the block fix is passed.
        const int block_floor_ap = active_block_floor_ft();
        if (block_floor_ap > 0 && tft < block_floor_ap) {
          tft = block_floor_ap;
          tfl = (tft >= compute_tl_ft(ta_c, ctx.qnh_hpa));
        }
        // A descent is "needed" only when the clamped target is actually below
        // the altitude already cleared. At MUS, already at the FL080 block
        // floor, the clamp lifts SOTOX's FL070 back to FL080 -> no descent
        // (user 2026-07-12: "at MUS no descent order can be issued").
        const int cur_cl_ap = engine::current_cleared_alt_ft();
        const bool descent_needed_ap = (cur_cl_ap <= 0) || (tft < cur_cl_ap - 100);
        // Post-clearance implicit profile: once the approach has been CLEARED
        // at a controlled field (the clearance carries the first approach-fix
        // altitude, e.g. "cleared RNAV Zulu approach runway 04, descend 6500
        // feet"), ATC stays SILENT on the remaining published step-downs (5000,
        // 3500, ...). The pilot flies the charted approach vertical profile
        // implicitly, exactly as with the lateral path (user 2026-07-12: "should
        // not give after 6500" -- no 5000/3500). Advance the tracker silently,
        // no ATC message. AFIS fields (no clearance issued) keep the per-fix
        // descent behaviour unchanged.
        if (s_approach_cleared_issued && !dest_is_afis) {
          if (!twp.ident.empty()) {
            for (int ri = s_route_fix_idx;
                 ri < static_cast<int>(s_route_fixes.size()); ++ri) {
              if (s_route_fixes[ri].ident == twp.ident) {
                s_route_fix_idx = ri;
                break;
              }
            }
          }
          logging::info("[approach] implicit step-down at %s (silent, post-clearance)",
                        twp.ident.c_str());
          s_last_cleared_route_idx = s_route_fix_idx;
          s_approach_waypoint_idx  = target_idx + 1;
          s_approach_timer         = 0.0f;
          return false;
        }
        // No-shortcut arrivals: ATC does NOT announce each approach/transition
        // fix ("direct LP403 ...") -- the pilot flies the published procedure
        // implicitly. Only the procedure-tied descent is spoken here, plus the
        // single approach clearance issued ONCE as the aircraft enters the
        // approach transition (~at the IAF, e.g. PIRUV for SALE3P). A spoken
        // "direct FIX" is reserved for a genuine shortcut, which the no-STAR
        // path issues to an IAF only. See project_arrival_announcement_model.
        std::string lead = cs;
        // AFIS/Information fields (e.g. LFQA) have no Approach control and
        // never issue an approach clearance -- the pilot self-announces. Only
        // controlled destinations get "cleared <appr>".
        if (!s_approach_cleared_issued && !dest_is_afis) {
          const std::string appr_phrase = approach_clearance_phrase(ctx);
          if (!appr_phrase.empty()) {
            lead += ", cleared " + appr_phrase;
            s_approach_cleared_issued = true;
            logging::info("[approach] cleared approach at IAF entry: %s",
                          appr_phrase.c_str());
          }
        }
        if (descent_needed_ap) {
          *out_text = build_approach_final_alt(lead, /*fix_ident=*/"", tft,
                                               ctx.qnh_hpa,
                                               ctx.transition_alt_ft, tfl);
          // Keep engine::current_cleared_alt_ft() (used by STT context_bias)
          // in sync with the freshly-issued descent step-down.
          s_enroute_cleared_alt_ft = tft;
        } else if (lead != cs) {
          // Approach clearance to voice, but the block floor means no descent
          // (already at/above it) -- "cleared RNAV Zulu approach runway 22L."
          *out_text = lead + ".";
        } else {
          // Nothing to say (no clearance phrase, no descent) -> advance silently.
          if (!twp.ident.empty())
            for (int ri = s_route_fix_idx;
                 ri < static_cast<int>(s_route_fixes.size()); ++ri)
              if (s_route_fixes[ri].ident == twp.ident) { s_route_fix_idx = ri; break; }
          s_last_cleared_route_idx = s_route_fix_idx;
          s_approach_waypoint_idx  = target_idx + 1;
          s_approach_timer         = 0.0f;
          return false;
        }
        // Advance the route tracker SILENTLY -- no "direct" is spoken, the pilot
        // follows the procedure implicitly (no s_pending_route_direct event).
        if (!twp.ident.empty()) {
          for (int ri = s_route_fix_idx;
               ri < static_cast<int>(s_route_fixes.size()); ++ri) {
            if (s_route_fixes[ri].ident == twp.ident) {
              s_route_fix_idx = ri;
              logging::info("[route] passing %s (idx=%d, implicit)",
                            twp.ident.c_str(), ri);
              break;
            }
          }
        }
        s_expedite_last_cleared_ft = tft;
        s_expedite_cooldown        = 60.0f;
        s_approach_waypoint_idx = target_idx; // ++ below lands on target+1
      } else {
        // STAR crossing constraint. Real ATC ALWAYS issues a hard crossing
        // restriction — the previous 80% silent-skip dropped it four times
        // out of five, so LUVOB FL090 on SALE3P never got voiced and the
        // aircraft stayed high (LIMF -> LFLP 2026-07-09). Fire whenever the
        // constraint requires a descent below the current cleared altitude;
        // skip only when the aircraft is already cleared at or below it
        // (constraint already satisfied — e.g. the STAR-lookahead initial
        // descent in build_descent_clearance already cleared to this fix's
        // FL). See [[project_star_walker_80_20]].
        // Positional block gate: never clear below the nearest unpassed block
        // floor (same rule as the approach branch).
        const int block_floor_star = active_block_floor_ft();
        if (block_floor_star > 0 && cleared_ft < block_floor_star)
          cleared_ft = block_floor_star;
        const int cur_cleared = engine::current_cleared_alt_ft();
        if (cur_cleared > 0 && cleared_ft >= cur_cleared) {
          s_approach_waypoint_idx++;
          s_approach_timer = 0.0f;
          return false; // no descent needed at this constraint
        }
        *out_text = build_star_constraint(cs, wp, cleared_ft,
                                          ctx.qnh_hpa, ctx.transition_alt_ft);
        // Keep engine::current_cleared_alt_ft() (used by STT context_bias)
        // in sync with the freshly-issued STAR step-down.
        s_enroute_cleared_alt_ft = cleared_ft;
        // Advance route tracker to this STAR fix.
        if (!wp.ident.empty()) {
          for (int ri = s_route_fix_idx;
               ri < static_cast<int>(s_route_fixes.size()); ++ri) {
            if (s_route_fixes[ri].ident == wp.ident) {
              s_route_fix_idx = ri;
              logging::info("[route] ATC direct: %s (idx=%d)", wp.ident.c_str(), ri);
              s_pending_route_direct = "ATC direct: " + wp.ident;
              break;
            }
          }
        }
      }
      s_expedite_last_cleared_ft = cleared_ft;
      s_expedite_cooldown        = 60.0f;
      s_last_cleared_route_idx   = s_route_fix_idx; // arm fix_trigger for next step-down
      s_approach_waypoint_idx++;
      s_approach_timer = 0.0f;
      rb(true);
      return true;
    }
    s_approach_waypoint_idx++;
    return false;
  }

  // ── Expedite-descent monitor ─────────────────────────────────────────────
  // Fires AFTER a step-down clearance has been issued (s_expedite_last_cleared_ft > 0)
  // when the aircraft is clearly not descending fast enough to meet it.
  // Uses the last-cleared altitude so it always references an altitude the pilot
  // was actually given, never a future waypoint constraint.
  if (s_expedite_last_cleared_ft > 0 &&
      ctx.pressure_alt_ft > static_cast<float>(s_expedite_last_cleared_ft) + 300.0f) {
    s_expedite_cooldown -= dt;
    if (s_expedite_cooldown <= 0.0f) {
      const double dist_apt = traffic_geometry::distance_nm(
          ctx.latitude, ctx.longitude, ctx.airport_lat, ctx.airport_lon);
      if (dist_apt < 60.0) {
        const float gs = ctx.groundspeed_kts > 60.0f ? ctx.groundspeed_kts : 200.0f;
        const float alt_diff =
            ctx.pressure_alt_ft - static_cast<float>(s_expedite_last_cleared_ft);
        // Approximate time to destination airport as proxy for time to cleared fix.
        const float time_min = static_cast<float>(dist_apt) / gs * 60.0f;
        if (time_min > 0.5f) {
          const float required_rate = alt_diff / time_min;
          const float current_rate  = -ctx.vertical_speed_fpm; // + = descending
          const bool nearly_level = current_rate < 300.0f && required_rate > 800.0f;
          const bool too_slow     = required_rate > current_rate * 1.5f &&
                                    required_rate > 800.0f;
          if (nearly_level || too_slow) {
            const int ta = ctx.transition_alt_ft > 0 ? ctx.transition_alt_ft : 5000;
            char buf[160];
            if (s_expedite_last_cleared_ft > ta)
              std::snprintf(buf, sizeof(buf),
                            "%s, expedite descent to flight level %d.",
                            cs.c_str(), s_expedite_last_cleared_ft / 100);
            else
              std::snprintf(buf, sizeof(buf),
                            "%s, expedite descent to %d feet, QNH %d.",
                            cs.c_str(), s_expedite_last_cleared_ft, ctx.qnh_hpa);
            if (out_text) {
              *out_text = buf;
              s_expedite_cooldown = 90.0f;
              return true;
            }
          }
        }
      }
    }
  }

  // All STAR constraints issued — issue final altitude below transition altitude.
  // final_alt_ft comes from ifr_defaults.approach_entry_alt_ft (flight_rules.json).
  const auto &ifrdef = flight_phase::get_ifr_defaults();
  // Fire final altitude when aircraft is below 1.5× the approach entry altitude.
  if (!s_approach_final_issued && s_approach_timer > 60.0f &&
      ctx.pressure_alt_ft < static_cast<float>(ifrdef.approach_entry_alt_ft) * 1.5f) {
    s_approach_final_issued = true;
    int final_alt_ft = ifrdef.approach_entry_alt_ft;
    // Safety: never issue a "descend to X" when X >= current altitude (would be
    // a climb). Suppress silently — the approach continues without a new clearance.
    if (final_alt_ft >= static_cast<int>(ctx.pressure_alt_ft)) {
      logging::info("[approach] final alt %d ft >= current %.0f ft — suppressed",
                    final_alt_ft, ctx.pressure_alt_ft);
      return false;
    }
    if (out_text) {
      // Fallback path (no approach-proc fix carried an altitude, e.g. AFIS /
      // sparse CIFP): still issue the single approach clearance here if it
      // never fired at the IAF-transition entry above.
      std::string lead = cs;
      if (!s_approach_cleared_issued && !dest_is_afis) {
        const std::string appr_phrase = approach_clearance_phrase(ctx);
        if (!appr_phrase.empty()) {
          lead += ", cleared " + appr_phrase;
          s_approach_cleared_issued = true;
          logging::info("[approach] cleared approach at final-alt fallback: %s",
                        appr_phrase.c_str());
        }
      }
      // Approach entry altitude is a low QNH altitude (below TL) — always feet.
      *out_text = build_approach_final_alt(lead, "", final_alt_ft, ctx.qnh_hpa,
                                           ctx.transition_alt_ft, /*is_fl=*/false);
      // Sync engine::current_cleared_alt_ft() for STT context_bias.
      s_enroute_cleared_alt_ft = final_alt_ft;
      rb(true);
      return true;
    }
  }

  // DirectMonitor (approach course): lowest-priority "confirm direct <fix>" when
  // off the leg to the ACTIVE next fix. Skipped post-FAF (IFR_APPROACH_TOWER /
  // LANDING_CLEARED) where poll_approach_alignment owns lateral (centerline).
  // WIRED with placeholder gates (25 deg / 2 NM guard / 90 s cooldown) -- tune later.
  if (state != AS::IFR_APPROACH_TOWER && state != AS::IFR_LANDING_CLEARED) {
    s_approach_course_cooldown = std::max(0.0f, s_approach_course_cooldown - dt);
    if (s_approach_course_cooldown <= 0.0f) {
      const CourseCheck cc = check_course(ctx, 25.0);
      if (cc.valid && cc.off_course && cc.dist_nm > 2.0) {
        s_approach_course_cooldown = 90.0f;
        if (out_text) {
          char buf[176];
          std::snprintf(buf, sizeof(buf),
                        "%s, confirm direct %s, you appear tracking heading %.0f, "
                        "expected %.0f.",
                        cs.c_str(), cc.ident.c_str(),
                        static_cast<double>(ctx.heading_true), cc.bearing_deg);
          *out_text = buf;
        }
        logging::info("[approach] course deviation hdg %.0f vs brg %.0f to %s (diff %.0f)",
                      static_cast<double>(ctx.heading_true), cc.bearing_deg,
                      cc.ident.c_str(), cc.diff_deg);
        rb(false);
        return true;
      }
    }
  }

  return false;
}

// ── poll_approach_alignment ───────────────────────────────────────────────
// Fires after the FAF (IFR_APPROACH_TOWER state) when the aircraft is more
// than 0.5 NM off the extended runway centerline.
// Distinct from s_enroute_deviation_cooldown_sec (airway off-track, en-route).

bool poll_approach_alignment(const xplane_context::XPlaneContext &ctx, float dt,
                             std::string *out_text) {
  using AS = atc_state_machine::ATCState;
  // Fire only AFTER the pilot is cleared to land (LANDING_CLEARED), never in
  // APPROACH_TOWER before the Tower check-in -- otherwise the "confirm
  // established" nag fires on the Tower freq while the pilot is still turning
  // onto final and has not called Tower yet, spoken with the stale Approach
  // label (LFMN 2026-07-20/21: fired at ~2:40 after the handoff, before check-in;
  // the earlier 45 s cooldown was too short). Post-clearance it is a genuine
  // "you are cleared but drifting off centerline" warning.
  if (atc_state_machine::get_state() != AS::IFR_LANDING_CLEARED)
    return false;

  // Only once the pilot has actually switched to the Tower frequency. The
  // state flips to IFR_APPROACH_TOWER at the "contact Tower" handoff, but the
  // pilot is still on the previous Approach freq for a few transmissions;
  // firing here would transmit "confirm established" on that old frequency
  // (LIMF -> LFLP 2026-07-11: spoken on Chambery 121.205 before the switch).
  {
    float tower_mhz = s_assigned_dest_icao.empty()
                          ? 0.0f
                          : xplane_context::tower_mhz_for(s_assigned_dest_icao);
    if (tower_mhz <= 100.0f)
      tower_mhz = ctx.airport_freqs.first_mhz(xplane_context::FrequencyType::TOWER);
    const float active =
        (ctx.active_com == 2) ? ctx.com2_freq_mhz : ctx.com1_freq_mhz;
    if (tower_mhz > 100.0f && std::fabs(active - tower_mhz) > 0.010f)
      return false; // not on the Tower frequency yet
  }

  s_alignment_cooldown -= dt;
  if (s_alignment_cooldown > 0.0f)
    return false;

  if (s_assigned_landing_runway.empty())
    return false;

  // Only check when within 8 NM of airport and below 3000 ft AGL.
  const double dist_apt = traffic_geometry::distance_nm(
      ctx.latitude, ctx.longitude, ctx.airport_lat, ctx.airport_lon);
  if (dist_apt > 8.0 || ctx.height_agl_ft > 3000.0f)
    return false;

  // Find landing-runway threshold (matching s_assigned_landing_runway).
  double rwy_lat = 0.0, rwy_lon = 0.0;
  float  rwy_hdg = -1.0f;
  for (const auto &rwy : ctx.runways) {
    if (rwy.end1.number == s_assigned_landing_runway) {
      rwy_lat = rwy.end1.lat; rwy_lon = rwy.end1.lon;
      rwy_hdg = rwy.end1.heading_deg;
      break;
    }
    if (rwy.end2.number == s_assigned_landing_runway) {
      rwy_lat = rwy.end2.lat; rwy_lon = rwy.end2.lon;
      rwy_hdg = rwy.end2.heading_deg;
      break;
    }
  }
  if (rwy_hdg < 0.0f)
    return false;

  // Cross-track error from extended centerline.
  // Approach course from threshold (outbound = rwy_hdg + 180°).
  const double approach_course = std::fmod(static_cast<double>(rwy_hdg) + 180.0, 360.0);
  const double bearing_to_acft = traffic_geometry::bearing_deg(
      rwy_lat, rwy_lon, ctx.latitude, ctx.longitude);
  double bearing_diff = bearing_to_acft - approach_course;
  if (bearing_diff >  180.0) bearing_diff -= 360.0;
  if (bearing_diff < -180.0) bearing_diff += 360.0;
  const double dist_nm = traffic_geometry::distance_nm(
      ctx.latitude, ctx.longitude, rwy_lat, rwy_lon);
  const double cross_track_nm =
      std::sin(bearing_diff * M_PI / 180.0) * dist_nm;

  if (std::fabs(cross_track_nm) < 0.5)
    return false;

  const std::string &cs_ref = atc_state_machine::session_callsign();
  const std::string cs = cs_ref.empty() ? settings::pilot_callsign() : cs_ref;

  char buf[160];
  std::snprintf(buf, sizeof(buf),
                "%s, confirm established on the approach, runway %s.",
                cs.c_str(), s_assigned_landing_runway.c_str());
  if (out_text) {
    *out_text = buf;
    s_alignment_cooldown = 60.0f;
    return true;
  }
  return false;
}

bool poll_ground_runway_change(const xplane_context::XPlaneContext &ctx,
                               std::string *out_text) {
  if (!ctx.on_ground || ctx.active_runway.empty())
    return false;

  using AS = atc_state_machine::ATCState;
  AS state = atc_state_machine::get_state();

  // In IDLE: silently track the runway so we don't announce a change that
  // happened before the pilot was engaged.
  if (state == AS::IDLE) {
    s_ground_last_announced_runway = ctx.active_runway;
    return false;
  }

  bool active_ground_state =
      (state == AS::GROUND_CONTACT || state == AS::TAXI_CLEARED ||
       state == AS::TOWER_CONTACT || state == AS::IFR_PREDEP_CLEARANCE ||
       state == AS::IFR_CLEARED);
  if (!active_ground_state)
    return false;

  // Don't interrupt a pending readback — the pilot is mid-clearance.
  if (atc_state_machine::is_readback_pending())
    return false;

  // Seed on first entry into an active state.
  if (s_ground_last_announced_runway.empty()) {
    s_ground_last_announced_runway = ctx.active_runway;
    return false;
  }

  if (ctx.active_runway == s_ground_last_announced_runway)
    return false;

  // Runway changed — update tracking and sync the assigned runway so
  // get_runway() in build_vars uses the new runway for holding point
  // phrases and lineup instructions.
  s_ground_last_announced_runway = ctx.active_runway;
  atc_state_machine::set_assigned_runway(ctx.active_runway);

  if (!out_text)
    return true;

  static const char *kPhonetic[] = {
      "Alpha",   "Bravo",  "Charlie", "Delta",   "Echo",    "Foxtrot",
      "Golf",    "Hotel",  "India",   "Juliet",  "Kilo",    "Lima",
      "Mike",    "November", "Oscar", "Papa",    "Quebec",  "Romeo",
      "Sierra",  "Tango",  "Uniform", "Victor",  "Whiskey", "X-ray",
      "Yankee",  "Zulu"};

  std::string hp_phrase = "runway " + ctx.active_runway;
  auto hp_it = ctx.runway_holding_points.find(ctx.active_runway);
  if (hp_it != ctx.runway_holding_points.end() && !hp_it->second.empty()) {
    const std::string &hp = hp_it->second;
    std::string name;
    if (hp.size() == 1 && hp[0] >= 'A' && hp[0] <= 'Z')
      name = kPhonetic[hp[0] - 'A'];
    else if (hp.size() == 1 && hp[0] >= 'a' && hp[0] <= 'z')
      name = kPhonetic[hp[0] - 'a'];
    else
      name = hp;
    hp_phrase = "holding point " + name + ", runway " + ctx.active_runway;
  }

  const std::string &cs = atc_state_machine::session_callsign();
  const std::string &callsign = cs.empty() ? settings::pilot_callsign() : cs;
  char buf[192];
  std::snprintf(buf, sizeof(buf),
                "%s, be advised, active runway is now runway %s, taxi to %s.",
                callsign.c_str(), ctx.active_runway.c_str(),
                hp_phrase.c_str());
  *out_text = buf;
  logging::info("Ground: active runway changed to %s", ctx.active_runway.c_str());
  return true;
}

void set_pending_handoff_freq(float mhz) {
  if (mhz >= 100.0f) {
    s_pending_handoff_freq_mhz = mhz;
    logging::debug("[DBG] pending_handoff_freq=%.3f [set_api]", mhz);
  }
}

float pending_handoff_freq() { return s_pending_handoff_freq_mhz; }

} // namespace engine
