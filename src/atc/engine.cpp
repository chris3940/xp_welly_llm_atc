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
#include "atc/phonetic.hpp"
#include "atc/intent_rules.hpp"
#include "atc/landing_sequence.hpp"
#include "atc/route_shortcut.hpp"
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
// Report-then-transfer coupling: the departure handoff must be the RESPONSE to the
// pilot's "reaching N feet" report, not an unsolicited push. Set true when the pilot
// transmits in IFR_DEPARTURE_CLEARED while airborne; the poll then fires the handoff.
// s_departure_at_alt_sec is the grace timer -- if the pilot never reports, the poll
// hands off anyway after it elapses (they level at the initial climb and would
// otherwise sit on Tower forever). Both reset on leaving IFR_DEPARTURE_CLEARED.
static bool s_departure_level_reported = false;
static float s_departure_at_alt_sec = 0.0f;
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
// Stepped high-cruise descent: from a very high cruise (e.g. FL450) a single
// "descend FLxx" is a 25-33k ft clearance. build_descent_clearance caps the
// FIRST step at FL200 and stashes the ultimate STAR-entry target here; a SECOND
// clearance (poll_descent_second_step) issues it once the aircraft nears FL200.
// 0 = no deferral pending (cruise was already <= FL200, or single-step). The
// freq change is DECOUPLED -- poll_acc_sector_change fires at the real boundary.
static int  s_descent_final_target_ft = 0;    // ultimate STAR-entry target when stepped
static int  s_descent_first_step_ft   = 0;    // FL of the first (TMA-top) step
static bool s_descent_second_step_issued = false;
// Connector-STAR "direct <IAF>": when the arrival uses a connector STAR NOT in the
// pilot's FMS (LOWI NANI2A->RTT bridged to the R08-Z IAF ELMEM by RTT1B), ATC issues
// an explicit "direct <IAF>" as the aircraft reaches the end of the FILED STAR (RTT),
// well BEFORE the approach clearance (which comes near the IAF). One-shot. [C.P.Potter]
static bool s_connector_direct_issued = false;
static float s_arrival_timer        = 0.0f; // time spent in IFR_ARRIVAL (guards 50 NM fallback)
static float s_enroute_approach_freq_mhz = 0.0f; // set by build_approach_handoff
// TOD estimate for the IFR tab, updated each frame in poll_enroute (routed distance
// to the STAR entry + the alert distance where the pre-TOD descent fires). -1 = N/A.
static float s_tod_dist_nm  = -1.0f;
static float s_tod_alert_nm = -1.0f;
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

// A destination is CONTROLLED (not AFIS/Information) when airport+.json overrides a
// local Tower, Approach or Ground controller for it. Some real towers -- LOWI
// Innsbruck 120.100 -- carry neither a separate Ground nor an Approach freq in the
// user's apt.dat, so the raw "no Ground AND no Approach = AFIS" test mislabels them
// "Information", suppresses the Tower handoff phrasing, and (worse) keeps issuing
// per-fix step-downs after the approach is cleared (bogus "descend flight level 135"
// on a speed-only fix, real vol 2026-08-02). The airport+.json override is the
// authority the raw freq tables lack. user 2026-08-02: "Innsbruck Kranebitten
// Information => c'est la Tour, il faut surcharger dans airport+.json". [C. P. Potter]
static bool dest_ctrl_override_present(const std::string &icao) {
  if (icao.empty()) return false;
  std::string n;
  float f = 0.0f;
  return airport_overrides::controller(icao, "tower", &n, &f) ||
         airport_overrides::controller(icao, "approach", &n, &f) ||
         airport_overrides::controller(icao, "ground", &n, &f);
}
// Resolve the approach transition IAF for the assigned STAR + approach, chaining
// through a connector STAR when the STAR terminus is not itself an IAF of the
// approach (LOWI RTT->ELMEM via RTT1B). *out_connector, if non-null, receives
// the connector STAR name (empty = direct / none). [[project_star_chaining]]
static std::string resolve_approach_iaf(const xplane_context::XPlaneContext &ctx,
                                        const std::string &star_name,
                                        const std::string &approach_designator,
                                        std::string *out_connector = nullptr);
static void build_sid_route_table(const xplane_context::XPlaneContext &ctx); // departure half of the route table
static std::string approach_clearance_phrase(
    const xplane_context::XPlaneContext &ctx); // "RNAV Zulu approach runway 08"; defined near poll_approach
static double routed_distance_to_fix_idx(const xplane_context::XPlaneContext &ctx,
                                         int target_idx); // defined before check_next_fix
static std::string controller_label_for(const airspace_db::Controller *ctrl); // defined near handoff helpers
static std::string openair_sector_label(const std::string &name); // openair NAME -> label ("MARSEILLE CTA..." -> "Marseille")
// Spoken facility name from an ICAO: airport name with any "/..." suffix dropped
// ("Nice/Cote d'Azur" -> "Nice") so a slash never garbles the radio call
// ("NICE/COTE"). Multi-word names without a slash are kept ("Reims Prunay").
static std::string spoken_airport_name(const std::string &icao);
static bool resolve_sector_controller(
    const openair_db::AirspaceEntry &enc, bool terminal, std::string *out_label,
    float *out_mhz,
    std::uint32_t avoid_freq_khz = 0); // unified handoff resolver

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
// --- STAR direct-to-IAF shortcut (poll_star_shortcut) ----------------------
// On a STAR arrival, ATC may (20% roll, or 100% with settings::shortcut_always)
// clear "direct <IAF>" to the approach IAF geographically NEAREST the aircraft --
// cutting the remaining STAR track when it stays flyable (descent <= 3 deg to the
// IAF's at-or-below altitude). Fires once, in IFR_ARRIVAL (== past the 1st STAR
// fix). The pilot may refuse (UNABLE) -> revert to the filed STAR. [C. P. Potter]
static bool s_star_shortcut_offered  = false; // one-shot: the roll happened this arrival
static bool s_star_shortcut_pending  = false; // direct issued, awaiting accept/refuse
static int  s_star_shortcut_prev_idx = -1;    // route idx before the jump (UNABLE restore)
// --- Vector-to-intercept (radar vectors to final) --------------------------
// Bypass-IAF radar vectoring: when the arrival needs a large turn/reversal to join
// the final approach course (LOWI R08-Z: direct ELMEM heading ~270, final 082 ->
// ~172 deg), ATC issues up to 3 SEPARATE vectors that turn the aircraft onto an
// intercept heading (final +/- 30) INSTEAD of overflying the IAF + procedure turn
// (user 2026-07-31: bypass, established+timer cadence, magnetic headings). The last
// vector carries the approach clearance ("...maintain <alt> until established on the
// final approach course, cleared <appr>") and latches s_approach_cleared_issued so
// the normal cleared-at-IAF gate no-ops. Generic by (course, start ref) -> reusable
// at the FAF later. [C. P. Potter]
struct VecInterceptPlan {
  std::vector<int> vectors;      // teardrop: [open_hdg, reverse1_hdg] (reverse2 = live inbound intercept)
  bool turn_left = false;        // REVERSE turn direction
  bool open_left = false;        // OPEN turn direction (opposite of the reverse)
  bool needed = false;
  int  maintain_ft = 0;          // "... until established" = IAF platform alt
  bool maintain_is_fl = false;   // IAF constraint expressed as FL (else feet/QNH)
  double iaf_lat = 0.0, iaf_lon = 0.0; // outbound-distance gate + direct-to-IAF heading
  double inbound_mag = 0.0;      // final approach course at the IAF (magnetic) -- the
                                 // reverse2 vector INTERCEPTS this axis (not direct-to-fix)
};
static VecInterceptPlan s_vec_plan;
static int   s_vec_step  = -1;    // -1 = not started; else index of the NEXT vector to issue
static float s_vec_timer = 0.0f;  // seconds on the current vector (timer-fallback cadence)
static bool  s_vec_done  = false; // whole sequence issued (latch, once per arrival)
// Vector-compliance monitor state: the currently-assigned vector heading (magnetic;
// -1 = none), its turn-direction word, the time the pilot has NOT been following it, and
// whether the T+12s nudge already fired. [C. P. Potter]
static double s_vec_assigned_hdg  = -1.0;
static bool   s_vec_assigned_left = false;
static float  s_vec_follow_timer  = 0.0f;
static bool   s_vec_nudged        = false;
static bool   s_vec_expect_issued = false; // "expect vectors" heads-up sent once/arrival

// --- Published hold at a STAR fix (random, once per arrival) ----------------
// As the aircraft nears a STAR fix that has a PUBLISHED hold (earth_hold.dat), ATC
// may (random roll, once per arrival) issue "hold at <FIX> as published, maintain
// <alt>, expect further clearance in <N> minutes" -- for flow/sequencing. After a
// random 2-6 min the pilot is cleared to continue. Altitude is clamped to the hold's
// published [min,max] band. STAR fixes ONLY for now (user 2026-07-31). Generic by
// fix -> reusable at an IAF / en-route fix later. [C. P. Potter]
static cifp_reader::HoldSpec s_hold;
static int   s_hold_state    = 0;     // 0 = none, 1 = holding, 2 = done (released)
static float s_hold_secs     = 0.0f;  // seconds elapsed since entering the hold
static float s_hold_efc_secs = 0.0f;  // random hold duration (expect-further-clearance)
static float s_hold_efc_zulu = 0.0f;  // EFC as a ZULU clock time (sec since midnight) --
                                      // release keyed on this so the spoken time == exit
static int   s_hold_alt_ft   = 0;     // altitude held at (clamped to the band)
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
// Post-direct settle: seconds of grace after a "direct <fix>" before the DirectMonitor
// may flag a course deviation to that fix -- the pilot/FMS needs time to TURN onto the
// new leg. Without it a "direct ELMEM" at hdg 026 (bearing ~262) tripped "confirm direct,
// you appear tracking heading 026, expected 262" ~6 s later, before any turn (real vol
// LOWI R08-Z 2026-08-02). A ~130 deg turn at standard rate is ~45 s; 90 s covers turn +
// intercept. [C. P. Potter]
static constexpr float kDirectSettleSecs = 90.0f;

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
// Runway the pilot has already been cleared to CROSS at the current departure (avoids
// repeating the crossing clearance while still holding short of it). Reset per flight.
static std::string s_crossing_cleared_runway;
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
// Optional SECOND intermediate climb level (feet) before cruise, e.g. Annecy west
// SIDs: FL110 (under the FL115 TMA) -> FL140 (under the FL145 CTA) -> cruise. 0 =
// no second step (FL110 -> cruise directly). Issued once past the step-1 hold, and
// only when genuinely below the cruise FL (user 2026-07-22).
static int  s_sid_step2_alt_ft = 0;
static bool s_sid_step2_issued = false;
// Handoff-driven climb ladder: the level (feet) the NEXT controller will clear on
// the pilot's check-in after a SID handoff. > 0 while a handoff has fired but the
// pilot has not yet checked in on the new frequency; the sector-checkin ack reads
// it, announces "radar contact, climb flight level N", records it as the cleared
// level, then zeroes it. 0 = no climb pending on the next check-in. This is what
// makes each higher climb come from the higher controller AFTER its handoff, not
// from the departure Approach directly (user 2026-07-24).
static int  s_sid_pending_climb_ft = 0;
// Seconds the aircraft has dwelt AT the second step (FL140) since reaching it. The
// cruise clearance / FIR handoff is withheld until this passes a short grace, so the
// aircraft actually LEVELS at FL140 instead of being re-cleared on the way up
// (user 2026-07-24 -- "how long will it stay at FL140?").
static float s_sid_step2_dwell_sec = 0.0f;
// While > 0, the step-1 level is HELD -- neither the cruise climb nor the radar
// handoff fires until the aircraft is this many NM (great-circle) from the
// departure airport. 0 = no hold (normal continuous climb). Set only for local
// procedures that keep an aircraft low under an adjacent TMA (LFLP west SIDs:
// FL110 until ~20 NM out -> avoid a pointless Geneva-APP shuffle). Overridden
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
  s_departure_level_reported = false;
  s_departure_at_alt_sec = 0.0f;
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
  s_descent_final_target_ft = 0;
  s_descent_first_step_ft = 0;
  s_descent_second_step_issued = false;
  s_connector_direct_issued = false;
  s_tod_dist_nm = -1.0f;
  s_tod_alert_nm = -1.0f;
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
  s_sid_step2_issued = false;
  s_sid_step2_alt_ft = 0;
  s_sid_pending_climb_ft = 0;
  s_sid_step2_dwell_sec = 0.0f;
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
  s_crossing_cleared_runway.clear();
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
  s_star_shortcut_offered = false;
  s_star_shortcut_pending = false;
  s_star_shortcut_prev_idx = -1;
  s_approach_faf = {};
  s_vec_plan = {};
  s_vec_step = -1;
  s_vec_timer = 0.0f;
  s_vec_done = false;
  s_vec_assigned_hdg = -1.0;
  s_vec_follow_timer = 0.0f;
  s_vec_nudged = false;
  s_vec_expect_issued = false;
  s_hold = {};
  s_hold_state = 0;
  s_hold_secs = 0.0f;
  s_hold_efc_secs = 0.0f;
  s_hold_alt_ft = 0;
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
  // Statics NOT covered by the manual list above -- must be cleared so a jump (or a
  // repeated jump) does not inherit stale values from a previous flight. TODO: the
  // jump functions do a MANUAL partial reset; a full engine::reset() first would be
  // more robust but currently also clears s_route_fixes that the en-route jump relies
  // on -- revisit. (user 2026-07-30)
  s_descent_final_target_ft = 0;
  s_descent_first_step_ft = 0;
  s_descent_second_step_issued = false;
  s_connector_direct_issued = false;
  s_vec_plan = {};
  s_vec_step = -1;
  s_vec_timer = 0.0f;
  s_vec_done = false;
  s_vec_assigned_hdg = -1.0;
  s_vec_follow_timer = 0.0f;
  s_vec_nudged = false;
  s_vec_expect_issued = false;
  s_hold = {};
  s_hold_state = 0;
  s_hold_secs = 0.0f;
  s_hold_efc_secs = 0.0f;
  s_hold_alt_ft = 0;
  s_tod_dist_nm = -1.0f;
  s_tod_alert_nm = -1.0f;
  s_speed_250_warned = false;
  // No clearance was actually read back before a mid-flight jump: clear any pending
  // readback inherited from a previous flight so the fresh jump starts clean. The FL
  // "last clearance" IS set (s_enroute_cleared_alt_ft = cruise, above); speed has no
  // restriction at cruise.
  atc_state_machine::cancel_readback();

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
  s_star_shortcut_offered = false;
  s_star_shortcut_pending = false;
  s_star_shortcut_prev_idx = -1;
  s_approach_faf = {};
  s_vec_plan = {};
  s_vec_step = -1;
  s_vec_timer = 0.0f;
  s_vec_done = false;
  s_vec_assigned_hdg = -1.0;
  s_vec_follow_timer = 0.0f;
  s_vec_nudged = false;
  s_vec_expect_issued = false;
  s_hold = {};
  s_hold_state = 0;
  s_hold_secs = 0.0f;
  s_hold_efc_secs = 0.0f;
  s_hold_alt_ft = 0;
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
  s_star_shortcut_offered = false;
  s_star_shortcut_pending = false;
  s_star_shortcut_prev_idx = -1;
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
  // Benign acknowledgment -- a "direct <fix>" reply to a "confirm direct" query, or a
  // plain wilco/affirm/roger -- is NOT an unclear transmission. Real ATC does NOT reply
  // to a correct readback/ack (silence = accepted); it only speaks on a wrong readback or
  // to add something (user 2026-08-02: "j'ai confirme le direct, l'ATC doit-il repondre?"
  // -> no). So stay SILENT (empty response) -- this also avoids the "say again" loop that
  // a correct "Direct SALEV" once triggered (LFLP 2026-07-17). Do NOT bump the unclear
  // streak (return before the ++ below). [C. P. Potter]
  {
    std::string t = msg.raw_transcript;
    for (char &c : t)
      c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    for (const char *k : {"direct", "wilco", "affirm", "roger"})
      if (t.find(k) != std::string::npos)
        return std::string(); // silent: correct readback / benign ack
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

// The departure level-report altitude, in MSL. The config default
// (tower_report_alt_ft) is CAPPED at the SID initial-climb level-off the aircraft
// actually reaches (LIMF RW36 KUKE1Z: 2000, not the 3000 config it never sees).
// Shared by poll_departure_handoff (the transfer gate) and process_transcript (the
// off-altitude report challenge) so both agree on the figure. Optionally reports the
// phraseology VERB via is_reaching (true = "reaching" at a level-off, false =
// "passing" a config altitude climbed THROUGH) -- MUST mirror ground_operations
// {ifr_departure_contact} so the challenge matches the clearance the pilot heard
// (Annecy "passing 3000" vs LIMF "reaching 2000"). Returns 0 when report-then-
// transfer is off. [C. P. Potter] (declared in engine.hpp -- also drives the STT bias)
int departure_report_alt_ft(const xplane_context::XPlaneContext &ctx,
                            bool *is_reaching) {
  if (is_reaching)
    *is_reaching = false;
  int report_alt = flight_phase::get_ifr_defaults().tower_report_alt_ft;
  if (report_alt > 0) {
    const auto init = cifp_reader::initial_altitude(
        ctx.cifp_dir, ctx.nearest_airport_id, ctx.active_runway);
    const int field_ft =
        static_cast<int>(ctx.altitude_ft_msl - ctx.height_agl_ft);
    if (init.feet > 0 && !init.is_fl &&
        (init.feet <= report_alt || report_alt <= field_ft + 300)) {
      report_alt = init.feet;
      if (is_reaching)
        *is_reaching = true;
    }
  }
  return report_alt;
}

// Tolerance below the report altitude within which the pilot's level report is
// accepted rather than challenged. LARGER for a "passing" report -- a continuous
// climb-THROUGH where the pilot anticipates the crossing and STT/pipeline latency
// shifts the sampled altitude -- than for a "reaching" report where the aircraft has
// SETTLED at the level-off (a few tens of feet of altimeter wobble). Shared by the
// poll gate and the process_transcript challenge. [C. P. Potter]
static int report_alt_tolerance_ft(bool is_reaching) {
  return is_reaching ? 200 : 400;
}

// Runway that must be CROSSED at the current ground position, per ICAO Doc 4444: the
// aircraft is stopped/slow on the ground within ~250 m of a runway threshold whose end
// is NOT the assigned departure runway -- a parallel runway between the aircraft and the
// departure runway (LFMN: depart 04R, the taxi crosses 04L at A1). Returns the crossing
// runway ("04L") or "" when none. (user 2026-07-29) [C. P. Potter]
static std::string
crossing_runway_at_position(const xplane_context::XPlaneContext &ctx,
                            const std::string &dep_rwy) {
  if (!ctx.on_ground || ctx.groundspeed_kts > 10.0f || dep_rwy.empty())
    return {};
  // AFIS (Information) field (airport+.json "info" role, e.g. LFLU Valence
  // Information) issues NO clearances -- never offer a runway crossing there
  // (real vol 2026-08-02: bogus "cross runway 01L"). [C. P. Potter]
  {
    std::string n;
    float f = 0.0f;
    if (airport_overrides::controller(ctx.nearest_airport_id, "info", &n, &f))
      return {};
  }
  constexpr double kHoldShortNm = 0.135; // ~250 m
  for (const auto &rwy : ctx.runways) {
    for (const auto *end : {&rwy.end1, &rwy.end2}) {
      if (end->number.empty() || end->number == dep_rwy)
        continue; // the departure runway itself is never a crossing
      const double d = traffic_geometry::distance_nm(ctx.latitude, ctx.longitude,
                                                     end->lat, end->lon);
      if (d >= kHoldShortNm)
        continue;
      // A crossing is offered ONLY when the aircraft is holding short FACING ACROSS
      // that runway -- i.e. its heading is roughly TRANSVERSE to the runway axis
      // (within 45 deg of perpendicular). When the heading is PARALLEL to the
      // runway (aligned or reciprocal) the aircraft is ALONGSIDE / backtracking it,
      // not crossing: LFLU rwy 19 lining up next to the parallel grass 19L (~250 m
      // abeam) was wrongly told "cross runway 19L, report vacated" (real vol
      // 2026-08-02) -- a false positive that also bites TOWERED fields with
      // parallel runways. The legitimate LFMN case (holding on taxiway A1,
      // transverse to the parallel 04L, about to cross it toward 04R) still fires.
      // [C. P. Potter]
      const auto *other = (end == &rwy.end1) ? &rwy.end2 : &rwy.end1;
      const double axis = traffic_geometry::bearing_deg(end->lat, end->lon,
                                                        other->lat, other->lon);
      double off = std::fabs(static_cast<double>(ctx.heading_true) - axis);
      if (off > 180.0)
        off = 360.0 - off;
      if (std::fabs(off - 90.0) > 45.0)
        continue; // heading too parallel to this runway -> alongside, not crossing
      return end->number;
    }
  }
  return {};
}

// True when the pilot's ACTIVE COM frequency actually has a controller/station in
// this area: it matches the nearest airport's frequency DB, the current handoff /
// assigned-approach frequency (incl. airport+.json overrides), or an atc.dat
// controller whose polygon encloses the aircraft on that frequency. Used to gate
// the radio-check reply -- on a frequency with no station there is only silence,
// as in real ops (user 2026-07-30). [C. P. Potter]
static bool active_freq_has_controller(const xplane_context::XPlaneContext &ctx) {
  const float acom = (ctx.active_com == 2) ? ctx.com2_freq_mhz : ctx.com1_freq_mhz;
  if (acom < 100.0f)
    return false;
  if (ctx.frequency_type != xplane_context::FrequencyType::UNKNOWN)
    return true; // ATIS/Ground/Tower/Approach of the nearest airport (apt.dat)
  if (s_pending_handoff_freq_mhz >= 100.0f &&
      std::fabs(acom - s_pending_handoff_freq_mhz) < 0.02f)
    return true; // freq we just handed the pilot to (incl. airport+.json overrides)
  if (s_enroute_approach_freq_mhz >= 100.0f &&
      std::fabs(acom - s_enroute_approach_freq_mhz) < 0.02f)
    return true;
  if (airspace_db::enabled()) {
    const int ta = ctx.transition_alt_ft > 0 ? ctx.transition_alt_ft : 5000;
    const float alt = (ctx.altitude_ft_msl > static_cast<float>(ta))
                          ? ctx.pressure_alt_ft
                          : ctx.altitude_ft_msl;
    const auto khz = static_cast<std::uint32_t>(std::lround(acom * 1000.0));
    if (airspace_db::lookup_by_freq(khz, ctx.latitude, ctx.longitude, alt))
      return true; // atc.dat controller enclosing the position on this freq
  }
  return false;
}

// A transcript is "readback-like" -- an acknowledgement / readback of a clearance,
// never a fresh sector check-in. Covers course ("direct", "confirm direct"), radar
// VECTORS ("turn left/right", "heading", "vectors"), speed ("reduce speed", "knots"),
// and plain acks ("wilco", "roger"). Used to (a) skip the sector check-in ack, and
// (b) stop a "direct <fix>" / "turn <hdg>" readback from being classified as an
// INITIAL_CALL_APPROACH check-in and answered "radar contact" -- the pilot reading back
// the connector-direct + the teardrop vectors kept getting a spurious "radar contact"
// (real vol LOWI R08-Z 2026-08-02, Log.txt line 3300-3309). [C. P. Potter]
static bool transcript_is_readback_like(const std::string &transcript) {
  std::string t = transcript;
  for (char &c : t)
    c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  // A level-report CHECK-IN ("Lyon, passing 3000 feet, direct ROME") is NOT a
  // readback even when it states the cleared routing ("direct <fix>"): the
  // "passing/leaving ... feet" (or "with you") marks a fresh check-in on a new
  // controller. Without this, an airborne AFIS departure check-in on Lyon was
  // reclassified READBACK -> answered "unable" (real vol LFLU 2026-08-02, Log.txt
  // 1621-1629). [C. P. Potter]
  for (const char *ci : {"passing", "with you", "climbing through",
                         "descending through"})
    if (t.find(ci) != std::string::npos)
      return false;
  for (const char *k : {"direct", "confirm", "heading", "turn left", "turn right",
                        "vectors", "knots", "reduce speed", "wilco", "roger"})
    if (t.find(k) != std::string::npos)
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

  // STAR direct-to-IAF shortcut: resolve the outstanding "direct <IAF>, when
  // able" offer. "unable"/"negative" reverts to the filed STAR (restore the route
  // tracker jump); any other transmission is taken as acceptance and only clears
  // the pending flag, letting the readback be handled normally (silent benign
  // ack). One-shot -- s_star_shortcut_offered stays set so it never re-fires.
  if (s_star_shortcut_pending) {
    const std::string low = to_lower_copy(in.transcript);
    const bool refused = low.find("unable") != std::string::npos ||
                         low.find("negative") != std::string::npos;
    s_star_shortcut_pending = false;
    if (refused) {
      if (s_star_shortcut_prev_idx >= 0)
        s_route_fix_idx = s_star_shortcut_prev_idx;
      logging::info("IFR STAR shortcut: pilot UNABLE -> reverting to STAR "
                    "(tracker idx %d)",
                    s_star_shortcut_prev_idx);
      Output out;
      const std::string &cs = in.pilot_callsign.empty()
                                  ? settings::pilot_callsign()
                                  : in.pilot_callsign;
      out.response_text = cs + ", roger, continue on the arrival.";
      done(std::move(out));
      return;
    }
    logging::info("IFR STAR shortcut: direct accepted (readback follows)");
  }

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

  // Report-then-transfer coupling: in the departure-clearance window the pilot's
  // airborne transmission IS the "reaching N feet" level report the takeoff
  // clearance asked for. Arm the departure handoff so poll_departure_handoff fires
  // it as the RESPONSE to this call instead of spontaneously (user 2026-07-27:
  // "an App Approach call coming from nowhere ... should be the pilot calling").
  if (atc_state_machine::get_state() ==
          atc_state_machine::ATCState::IFR_DEPARTURE_CLEARED &&
      !ctx.on_ground) {
    bool reaching = false;
    const int report_alt = departure_report_alt_ft(ctx, &reaching);
    // Off-altitude report: the pilot called the level but is well BELOW it (beyond
    // the tolerance -- STT garble or an early call). CHALLENGE it ("confirm
    // reaching/passing N feet?") instead of arming the handoff on a bogus report, and
    // never leave the pilot with silence (user 2026-07-27). Verb mirrors the
    // clearance (Annecy "passing 3000" vs LIMF "reaching 2000"). Do NOT set
    // s_departure_level_reported -- the real report at the level arms the transfer.
    if (report_alt > 0 &&
        static_cast<int>(ctx.altitude_ft_msl) <
            report_alt - report_alt_tolerance_ft(reaching)) {
      Output out;
      char buf[96];
      std::snprintf(buf, sizeof(buf), "%s, confirm %s %d feet?",
                    in.pilot_callsign.c_str(), reaching ? "reaching" : "passing",
                    report_alt);
      out.response_text = buf;
      out.is_warning = true; // query only -- no state change
      logging::info("IFR departure: off-altitude level report at %.0fft (report_alt="
                    "%d) -> challenge",
                    ctx.altitude_ft_msl, report_alt);
      done(std::move(out));
      return;
    }
    s_departure_level_reported = true;
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
  const bool readback_like = transcript_is_readback_like(in.transcript);
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
        // First departure handoff check-in (Torino Tower -> Departure/Radar): the
        // readback on the old freq was accepted silently; advance FREQ_HANDOFF ->
        // RADAR_CONTACT now, on the pilot's real call on the NEW freq -- this used to
        // happen on the old-freq readback via INITIAL_CALL_CENTER (user 2026-07-27).
        if (ck_state == AS::IFR_FREQ_HANDOFF)
          atc_state_machine::set_state(AS::IFR_RADAR_CONTACT);
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
        // Handoff-driven SID climb ladder (user 2026-07-24, Eurocontrol): the climb
        // above the terminal airspace is issued by the controller the pilot was
        // handed off to, on their check-in -- NOT by the departure APP. When a climb
        // is queued (poll_sid_climb set s_sid_pending_climb_ft before the handoff),
        // this new controller clears it now. Overrides the plain "radar contact" ack.
        if (s_sid_pending_climb_ft > 0) {
          const int fl = s_sid_pending_climb_ft / 100;
          char ack_climb[192];
          std::snprintf(ack_climb, sizeof(ack_climb),
                        "%s, radar contact, climb flight level %d.", cs_ck.c_str(),
                        fl);
          s_enroute_cleared_alt_ft = s_sid_pending_climb_ft;
          logging::info("Sector checkin ack: %s (SID ladder climb, state=%s)",
                        ack_climb, atc_state_machine::state_name(ck_state));
          s_sid_pending_climb_ft = 0;
          // If this was the FINAL (FIR/cruise) handoff -- Phase 2.9 set
          // s_sid_radar_handoff_issued -- the SID climb is done: advance to en-route
          // now that the FIR has the aircraft and cruise is cleared.
          if (s_sid_radar_handoff_issued)
            atc_state_machine::set_state(AS::IFR_ENROUTE_CRUISE);
          Output out_climb;
          out_climb.response_text = ack_climb;
          done(std::move(out_climb));
          return;
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

  // IFR "leaving frequency" guard (user 2026-07-28): an IFR flight NEVER self-leaves a
  // controller's frequency in the air -- it is HANDED OFF (contact X on F), CANCELS IFR
  // explicitly, or CLOSES the flight plan at the destination (IFR_LANDING_CLEARED). So a
  // bare LEAVING_FREQUENCY in any other IFR state is spurious -- almost always a Voxtral
  // garble on a readback ("climb flight level 110 ... bye-bye", LFLP ROMA2A Log 1957).
  // Left alone it JUMPS the state (e.g. RADAR_CONTACT -> EN_ROUTE), skipping the
  // intermediate handoff/climb states and stranding the poll_* logic bound to the state
  // it left (the FL110 -> [30 NM hold] -> Lyon -> FL140 ladder went silent). Treat it as
  // the readback it is. IFR_LANDING_CLEARED is EXCLUDED -- there LEAVING_FREQUENCY
  // legitimately closes the flight plan. [C. P. Potter]
  if (parsed.intent == intent_parser::PilotIntent::LEAVING_FREQUENCY) {
    using AS2 = atc_state_machine::ATCState;
    const auto st_lv = atc_state_machine::get_state();
    const bool ifr_airborne_no_close =
        st_lv == AS2::IFR_FREQ_HANDOFF || st_lv == AS2::IFR_EN_ROUTE ||
        st_lv == AS2::IFR_RADAR_CONTACT || st_lv == AS2::IFR_ENROUTE_CRUISE ||
        st_lv == AS2::IFR_DESCENT || st_lv == AS2::IFR_ARRIVAL ||
        st_lv == AS2::IFR_APPROACH_CONTACT || st_lv == AS2::IFR_APPROACH_DESCENT ||
        st_lv == AS2::IFR_APPROACH_TOWER;
    if (ifr_airborne_no_close) {
      logging::info("Suppressed spurious LEAVING_FREQUENCY in IFR state %s -> READBACK "
                    "(IFR leaves only via handoff / cancel / landing closure)",
                    atc_state_machine::state_name(st_lv));
      parsed.intent = intent_parser::PilotIntent::READBACK;
    }
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
          !atc_state_machine::is_readback_pending() && !readback_like;
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

  // Proactive runway-CROSSING clearance (ICAO Doc 4444): once Ground has handed the
  // pilot to Tower AND the pilot has CHECKED IN on the Tower frequency at the holding
  // point of a runway that is NOT the assigned departure runway (a parallel runway to be
  // crossed to reach it -- LFMN depart 04R crosses 04L at A1), the Tower CLEARS the
  // crossing ("cross runway 04 Left, report vacated") instead of line-up (user
  // 2026-07-29). Fires on the Tower check-in in the pre-departure ground states, once per
  // crossing runway; the aircraft then crosses, taxis to the departure holding point, and
  // the normal line-up flow resumes (crossing_runway_at_position returns "" there). [CPP]
  {
    using AS3 = atc_state_machine::ATCState;
    const auto st3 = atc_state_machine::get_state();
    const bool pre_departure_on_tower =
        (st3 == AS3::TOWER_CONTACT || st3 == AS3::IFR_LINE_UP_AND_WAIT) &&
        ctx.frequency_type == xplane_context::FrequencyType::TOWER;
    if (pre_departure_on_tower) {
      const std::string dep_rwy = atc_state_machine::effective_runway(ctx);
      const std::string cross = crossing_runway_at_position(ctx, dep_rwy);
      if (!cross.empty()) {
        if (cross != s_crossing_cleared_runway) {
          // First call at this crossing hold-short: ISSUE the crossing clearance.
          s_crossing_cleared_runway = cross;
          const std::string &cs_ref = atc_state_machine::session_callsign();
          const std::string &cs = cs_ref.empty() ? in.pilot_callsign : cs_ref;
          Output out;
          char buf[128];
          std::snprintf(buf, sizeof(buf), "%s, cross runway %s, report vacated.",
                        cs.c_str(), cross.c_str());
          out.response_text = buf;
          logging::info("IFR Tower: proactive crossing clearance -> cross runway %s "
                        "(departure runway %s)",
                        cross.c_str(), dep_rwy.c_str());
          done(std::move(out));
          return;
        }
        // Already cleared to cross this runway: the pilot's call here is the crossing
        // READBACK or the "runway X vacated" report -- accept SILENTLY (ICAO: no ATC
        // response to a crossing readback). Without this the readback fell through to
        // the state machine and drew "say again" x3 (user 2026-07-29). The normal
        // line-up flow resumes once the aircraft reaches the departure holding point,
        // where crossing_runway_at_position() returns "".
        logging::info("IFR Tower: crossing readback/vacated accepted silently "
                      "(runway %s)",
                      cross.c_str());
        done(Output{});
        return;
      }
    }
  }

  // Radio check (IFR): reply with an ICAO readability report -- but ONLY when the
  // active frequency actually has a controller/station in this area. On a frequency
  // with no station a radio check draws SILENCE, as in real ops (user 2026-07-30).
  // ICAO Annex 10 Vol II: reply is a readability 1-5 ("readability five"), NOT the
  // US-military "five by five". IFR only; VFR keeps its own template. [C. P. Potter]
  {
    using PIR = intent_parser::PilotIntent;
    const auto rc_state = atc_state_machine::get_state();
    const bool ifr_state =
        std::string(atc_state_machine::state_name(rc_state)).rfind("IFR", 0) == 0;
    if (parsed.intent == PIR::RADIO_CHECK && ifr_state) {
      const std::string &cs_ref = atc_state_machine::session_callsign();
      const std::string &cs = cs_ref.empty() ? in.pilot_callsign : cs_ref;
      if (active_freq_has_controller(ctx)) {
        Output out;
        out.response_text =
            s_current_controller_label.empty()
                ? (cs + ", readability five.")
                : (cs + ", " + s_current_controller_label + ", readability five.");
        logging::info("IFR radio check -> readability five (controller '%s')",
                      s_current_controller_label.c_str());
        done(std::move(out));
        return;
      }
      // No controller on this frequency -> silence.
      logging::info("IFR radio check on a frequency with no station -- silent");
      done(Output{});
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
            // Chains through a connector STAR when needed. [[project_star_chaining]]
            const std::string iaf =
                resolve_approach_iaf(ctx, s_assigned_star_name, appr.designator);
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
      // Destination classification: AFIS = neither a Ground NOR an Approach freq
      // (a real Tower with no separate Ground still has Approach -- LOWI).
      const std::string dest =
          !s_assigned_dest_icao.empty() ? s_assigned_dest_icao
                                        : ctx.nearest_airport_id;
      const bool is_afis =
          !xplane_context::has_ground_freq_for(dest) &&
          !xplane_context::has_approach_freq_for(dest) &&
          !dest_ctrl_override_present(dest);
      char buf_cl[256];
      if (is_afis) {
        // Prefer airport name over ICAO in the spoken phrase.
        std::string apt =
            !dest.empty() ? spoken_airport_name(dest) : "";
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
          // A "direct <fix>" / radar-vector ("turn left/right heading NNN") readback
          // that the LM guessed as ANY check-in-family intent must NOT be answered
          // "radar contact" -- a direct/vector is issued in EVERY airborne phase (SID
          // climb "direct <last SID fix>", en-route shortcut, arrival connector-direct,
          // approach vectors), and the aircraft is already identified, so the readback
          // is never a fresh check-in (real vol LOWI R08-Z 2026-08-02, user: "un direct
          // concerne toutes les phases"). Treat it as a plain READBACK instead. [CPP]
          const bool checkin_family =
              lm_msg.intent == PI::INITIAL_CALL_APPROACH ||
              lm_msg.intent == PI::INITIAL_CALL_CENTER   ||
              lm_msg.intent == PI::INITIAL_CALL          ||
              lm_msg.intent == PI::INITIAL_CALL_TOWER    ||
              lm_msg.intent == PI::INITIAL_CALL_INBOUND;
          if (checkin_family &&
              transcript_is_readback_like(repair_accepted ? result.repaired_transcript
                                                          : original_transcript)) {
            logging::info("LM %s reclassified -> READBACK (readback-like: direct/vector "
                          "ack in any phase, not a check-in)",
                          intent_parser::intent_name(lm_msg.intent));
            lm_msg.intent = PI::READBACK;
          } else if (lm_msg.intent == PI::INITIAL_CALL_APPROACH &&
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
  // Reject GENERIC sector names that carry no place: a bare airspace type as the
  // FIRST token ("CTA C", "TMA 1", "SECTOR C" -- LOWI/Austria) has no city and
  // yields a meaningless spoken label ("Cta c"). Return "" so the caller falls
  // back to the atc.dat controller's proper name (VIENNA -> "Vienna"). Real
  // labels put the city BEFORE the type keyword ("MARSEILLE CTA" -> "Marseille").
  {
    std::string first = n;
    const auto sp = first.find(' ');
    if (sp != std::string::npos)
      first = first.substr(0, sp);
    for (char &c : first)
      c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
    for (const char *g : {"CTA", "TMA", "CTR", "FIR", "UIR", "SECTOR", "SEC",
                          "ACC", "APP", "AREA"})
      if (first == g)
        return "";
  }
  if (n.size() <= 2)
    return ""; // too short to be a place name (e.g. a bare sector letter "C")
  n[0] = static_cast<char>(std::toupper(static_cast<unsigned char>(n[0])));
  for (std::size_t i = 1; i < n.size(); ++i)
    n[i] = static_cast<char>(std::tolower(static_cast<unsigned char>(n[i])));
  return n;
}

static std::string spoken_airport_name(const std::string &icao) {
  if (icao.empty())
    return {};
  std::string apt = xplane_context::airport_name_for(icao);
  const auto slash = apt.find('/');
  if (slash != std::string::npos)
    apt = apt.substr(0, slash); // "Nice/Cote d'Azur" -> "Nice"
  while (!apt.empty() && apt.back() == ' ')
    apt.pop_back();
  return apt;
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
                            float dt, std::string *out_text) {
  using AS = atc_state_machine::ATCState;
  using FP = flight_phase::FlightPhase;
  using FT = xplane_context::FrequencyType;

  if (atc_state_machine::get_state() != AS::IFR_DEPARTURE_CLEARED) {
    // Not in the departure-clearance window -> keep the report-coupling flags
    // clear so the next takeoff starts fresh (before takeoff, and after handoff).
    s_departure_level_reported = false;
    s_departure_at_alt_sec = 0.0f;
    return false;
  }

  // Must be AIRBORNE and climbing out -- but NOT strictly CLIMB/CRUISE. The
  // geometric phase detector routinely mislabels a low, turning IFR departure
  // as PATTERN or even FINAL_APPROACH (LIMF RW36 climb-out: TAKEOFF_ROLL ->
  // PATTERN -> FINAL_APPROACH -> ... -> CRUISE, user 2026-07-26). The old strict
  // guard then BLOCKED the handoff for ~1 min while the aircraft climbed through
  // the report altitude, so the pilot's repeated "reaching 2000 feet" reports met
  // silence and the handoff fired late. In IFR_DEPARTURE_CLEARED there is no real
  // traffic pattern -- any airborne phase is the departure climb-out -- so accept
  // all of them and let the altitude gate below be the real trigger. [C. P. Potter]
  auto phase = flight_phase::get();
  if (ctx.on_ground || phase == FP::TAKEOFF_ROLL || phase == FP::LANDING_ROLL)
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

  // Takeoff clearance had no embedded departure contact. Fire an explicit
  // "contact <Approach/Departure> on F". Two trigger modes (user 2026-07-24):
  //  - report_alt > 0 (EUROCONTROL report-then-transfer): the Tower KEEPS the
  //    aircraft until it climbs past report_alt ft AGL, THEN hands off -- the takeoff
  //    clearance said "report passing N feet". AGL so it works at any field elevation.
  //  - report_alt == 0 (legacy): hand off once the aircraft has left the CTR (3-D
  //    find_enclosing; AGL fallback when OpenAir data is absent).
  // enc (openair volume + NAME) is resolved in BOTH modes -- it feeds the controller
  // name/freq resolution below.
  // Report altitude (MSL), capped at the SID initial-climb level-off. See
  // departure_report_alt_ft -- shared with the process_transcript report challenge.
  bool report_is_reaching = false;
  int report_alt = departure_report_alt_ft(ctx, &report_is_reaching);
  const int report_tol_ft = report_alt_tolerance_ft(report_is_reaching);
  openair_db::AirspaceEntry enc =
      openair_db::find_enclosing(ctx.latitude, ctx.longitude, openair_alt(ctx));
  if (report_alt > 0) {
    // Tolerance band: the pilot calls "reaching N feet" as the altimeter settles a
    // few tens of feet either side of the level-off (observed 1990/1995/1996 for a
    // 2000 ft initial climb). Without it the strict MSL gate ignored three
    // consecutive level reports and the aircraft sat on Tower in silence until it
    // physically crossed the exact figure -- the report path never fired because
    // its own trigger altitude was unreachable by rounding (user 2026-07-27:
    // "no ATC reply ... to 2000 feet"). The report/grace gate below still controls
    // WHEN the handoff fires; this only opens the altitude window. [C. P. Potter]
    if (static_cast<int>(ctx.altitude_ft_msl) < report_alt - report_tol_ft) {
      s_departure_at_alt_sec = 0.0f;
      return false; // still climbing to the report altitude -> stay on Tower
    }
    // At/above the report altitude. The takeoff clearance said "report reaching
    // N feet", so the transfer is normally the RESPONSE to the pilot's level report,
    // NOT an unsolicited push (user 2026-07-27: "an App Approach call coming from
    // nowhere ... should be the pilot calling"). Wait for the report -- EXCEPT:
    //  - the aircraft CLIMBS THROUGH the UPPER band (report_alt + tol) without ever
    //    reporting (a "passing" climb-out that blew past the point): the controller
    //    then AUTHORITATIVELY hands off (APP / Departure / Radar per airport, resolved
    //    below) rather than let it keep climbing on Tower (user 2026-07-27); OR
    //  - the grace timer expires (a "reaching" aircraft levelled off that never
    //    reports would otherwise sit on Tower forever).
    s_departure_at_alt_sec += dt;
    constexpr float kReportGraceSec = 25.0f;
    const bool crossed_upper_band =
        static_cast<int>(ctx.altitude_ft_msl) >= report_alt + report_tol_ft;
    if (!s_departure_level_reported && !crossed_upper_band &&
        s_departure_at_alt_sec < kReportGraceSec)
      return false;
    if (!s_departure_level_reported && crossed_upper_band)
      logging::info("IFR departure handoff: AUTHORITATIVE (climbed through %dft "
                    "without a level report)",
                    report_alt + report_tol_ft);
  } else {
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

  // Log once here (past all the altitude / report / grace gates) instead of every
  // frame from rotation -- the poll runs continuously in IFR_DEPARTURE_CLEARED.
  logging::info(
      "IFR departure handoff: openair enc='%s' class=%d floor=%dft ceil=%dft at "
      "%.0fft MSL pos=%.4f,%.4f (report_alt=%d MSL, reported=%d)",
      enc.name.c_str(), static_cast<int>(enc.ac_class), enc.floor_ft,
      enc.ceiling_ft, ctx.altitude_ft_msl, ctx.latitude, ctx.longitude, report_alt,
      s_departure_level_reported ? 1 : 0);

  std::string controller_label;
  float freq = 0.0f;

  // P0: explicit airport+.json controller override -- GENERIC (data-driven, no
  // per-airport code): any field may declare a "departure"/"approach" controller
  // whose frequency the sim's atc.dat/apt.dat get wrong. LIMF's Milan Radar freq in
  // atc.dat is stale vs Navigraph (129.275), so airport+.json carries the correct
  // value. The default remains the volume-change re-probe below (a CTR departure is
  // just a volume change: resolve the controller of the volume STACKED above, like
  // every other climb/descent transition); this override only supplies a correct
  // frequency where the data files are wrong. "departure" role first, then
  // "approach". (user 2026-07-27) [C. P. Potter]
  {
    std::string oname;
    float ofreq = 0.0f;
    // AFIS field (airport+.json "info" role): there is no Tower/Departure -- the
    // overlying ACC in the "delivery" role (LFLU -> Lyon Control 125.155) works the
    // departure too. So after takeoff the pilot contacts THAT ACC, not a
    // (non-existent) Departure. Use "delivery" for the handoff only when AFIS.
    std::string ainfo;
    float ainfo_f = 0.0f;
    const bool is_afis = airport_overrides::controller(ctx.nearest_airport_id,
                                                       "info", &ainfo, &ainfo_f);
    if (airport_overrides::controller(ctx.nearest_airport_id, "departure", &oname,
                                      &ofreq) ||
        airport_overrides::controller(ctx.nearest_airport_id, "approach", &oname,
                                      &ofreq) ||
        (is_afis && airport_overrides::controller(ctx.nearest_airport_id,
                                                  "delivery", &oname, &ofreq))) {
      controller_label = oname;
      freq = ofreq;
      logging::info("IFR departure handoff: [P0-airport+.json%s] %s %.3f",
                    is_afis ? "/AFIS" : "", controller_label.c_str(), freq);
    }
  }

  // Primary: use the openair TMA name to identify the correct TRACON in
  // atc.dat by name.  This is geometrically exact: openair has altitude-aware
  // polygons (e.g. CHAMBERY TMA 1000-9500ft vs GENEVA TMA FL095-FL195), so
  // the aircraft can never be handed off to a controller whose airspace starts
  // above its current altitude.
  //
  // "CHAMBERY TMA SECTOR 1" → city fragment "CHAMBERY"
  // → find atc.dat TRACON with NAME containing "CHAMBERY" → Chambery APP.
  // resolve_sector_controller(terminal=true) resolves a TMA/CTA name -> atc.dat
  // TRACON, label "<City> Approach". Non-TMA/CTA classes return false -> P2/P3.
  //
  // CTR re-probe: when the aircraft is still in the departure CTR at the report
  // altitude (it levels at the low SID initial climb, e.g. LIMF 2000 ft under the
  // 3500 ft TORINO CTR), find_enclosing returns only the CTR, whose name atc.dat
  // cannot map. Re-probe airspace.txt just above the CTR ceiling for the volume
  // STACKED over it (MILAN CTA ZONE 24, floor 3500) and resolve from that --
  // airspace.txt stays authoritative for the volume. (user 2026-07-27)
  if (freq < 100.0f) {
    openair_db::AirspaceEntry ctrl_vol = enc;
    if (enc.ac_class == openair_db::AirspaceClass::CTR && enc.ceiling_ft > 0) {
      const openair_db::AirspaceEntry above = openair_db::find_enclosing(
          ctx.latitude, ctx.longitude, enc.ceiling_ft + 200);
      if (!above.name.empty() &&
          above.ac_class != openair_db::AirspaceClass::CTR)
        ctrl_vol = above;
    }
    std::string lbl;
    float mhz = 0.0f;
    if (resolve_sector_controller(ctrl_vol, /*terminal=*/true, &lbl, &mhz)) {
      controller_label = lbl;
      freq = mhz;
      logging::info("IFR departure handoff: [P1-openair] '%s' -> %s %.3f",
                    ctrl_vol.name.c_str(), controller_label.c_str(), freq);
    } else {
      logging::info(
          "IFR departure handoff: [P1-openair] '%s' -> no match, falling back",
          ctrl_vol.name.c_str());
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
  // Arm the sector check-in so the pilot's handoff READBACK on the OLD (Tower) freq is
  // accepted SILENTLY -- the new controller must say "radar contact" only when the
  // pilot actually CALLS on the new freq, exactly like the mid-flight sector handoffs
  // (Milan Approach 125.630 in the LIMF->LFLP test). Without this the readback was
  // processed as an immediate INITIAL_CALL_CENTER check-in and the new controller
  // spoke "radar contact, climb ..." on the OLD freq before the pilot ever switched;
  // it also kept answering when the pilot mistuned instead of nudging them to the
  // handoff freq (user 2026-07-27). [C. P. Potter]
  if (freq >= 100.0f)
    s_sector_checkin_pending = true;

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

// A SID's published binding minimum counts as a real early departure floor ONLY
// when it is at an INTERMEDIATE fix. When it sits at the SID's EXIT fix it is the
// altitude to reach when LEAVING the SID -- it tracks the enroute climb / cruise
// (e.g. KUKEV at FL200 on LIMF KUKE1Z with a FL220 cruise), NOT a departure floor
// -- so it must not drive the progressive first climb step nor defeat the step-1
// hold (which would collapse the ladder into an immediate "climb FL200" straight
// out of the 2000 ft initial level-off). (user 2026-07-27) [C. P. Potter]
static bool sid_min_at_exit_fix(const xplane_context::XPlaneContext &ctx) {
  return !ctx.ifr_sid_min_waypoint.empty() &&
         ctx.ifr_sid_min_waypoint == ctx.ifr_sid_last_fix;
}

// The SID minimum that acts as an EARLY-CLIMB FLOOR for the intermediate steps
// (step1 / step2). An exit-fix binding (LIMF KUKE1Z: KUKEV FL200, cruise FL220)
// is an enroute-climb target, NOT a floor -- treating it as one clamps step1 up
// AND zeroes step2 (FL140 < FL200 fails the ">= sid_min" usable test), collapsing
// the FL110 -> FL140 -> cruise ladder into a single "climb FL220". Returns 0 in
// that case so the intermediate steps survive. (user 2026-07-27) [C. P. Potter]
static int sid_climb_floor_ft(const xplane_context::XPlaneContext &ctx) {
  return sid_min_at_exit_fix(ctx) ? 0 : ctx.ifr_sid_min_alt_ft;
}

// True while the step-1 hold is active -- a hold distance is set (default 10 NM,
// generic for every departure) and the aircraft is still within it (great-circle
// from the departure airport) AND the SID does not itself demand a climb above
// step-1. Suppresses both the cruise climb and the radar handoff so the aircraft
// stays at step-1 until clear of the terminal area.
static bool sid_step1_hold_active(const xplane_context::XPlaneContext &ctx) {
  if (s_sid_hold_release_nm <= 0.0f)
    return false;
  if (ctx.ifr_sid_min_alt_ft > s_sid_step1_alt_ft && !sid_min_at_exit_fix(ctx))
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
    s_sid_step2_issued = false;
    s_sid_step2_alt_ft = 0;
    s_sid_pending_climb_ft = 0;
    s_sid_step2_dwell_sec = 0.0f;
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
    // NOTE: s_departure_apt_id / lat / lon are intentionally NOT cleared here.
    // They are the PERSISTENT departure field (current_flight_airport, the 20 NM
    // hold, build_sid_route_table, the lflp_west gate all key on them). Clearing
    // them on every RADAR_CONTACT exit let a state bounce (e.g. the 131.315
    // wrong-freq mess: RADAR_CONTACT -> EN_ROUTE -> RADAR_CONTACT) re-capture the
    // DRIFTED nearest airport (LFLY) on re-init -> lflp_west failed -> generic
    // FL120 instead of FL110 (user 2026-07-22). They are cleared only by the full
    // flight reset (new flight / IDLE).
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
    // Capture the departure field ONCE and keep it across state bounces, so a
    // re-init (e.g. after a wrong-freq state bounce) never overwrites it with the
    // drifted nearest airport (LFLY) -- which broke the lflp_west FL110 gate and
    // build_sid_route_table (user 2026-07-22).
    if (s_departure_apt_id.empty()) {
      s_departure_apt_id  = ctx.nearest_airport_id; // still the departure field here
      s_departure_apt_lat = ctx.airport_lat;
      s_departure_apt_lon = ctx.airport_lon;
    }
    // Build the DEPARTURE half of the unified route table (SID + enroute
    // navlog) so the compliance monitor / speed enforcement work during the
    // climb, exactly as the arrival table serves approach. The arrival re-init
    // (init_route_fixes) later rebuilds it for the STAR/approach.
    build_sid_route_table(ctx);

    // Step-1 climb shaping, DATA-DRIVEN from the airspace the aircraft actually
    // climbs THROUGH -- no hardcoded ICAO or flight level. Critically, the terminal
    // TMA ceiling is sampled ~10 NM OUT (at the first route fix beyond 5 NM), NOT at
    // the field: at LFLP the field sits under Chambery TMA Sector 1 (top FL95) but
    // ~10 NM out the sector tops FL115 -> FL110. Sampling AT the field gave FL90
    // (wrong) and stranded the climb (user 2026-07-23).
    //   step1 = highest full-thousand FL strictly below that ~10-NM TMA ceiling,
    //           HELD 10 NM (great-circle) from departure before releasing the cruise
    //           climb + radar handoff. Overwritten UP by the SID's published minimum
    //           crossing altitude (never clear below a fix min). No hold if cruise is
    //           at/below step1 (climb straight to cruise).
    //   Fallback (no route fix / no openair / implausible ceiling): FL110, the safe
    //           default intermediate. FL110/10 NM and any per-airport first SID step
    //           are a FUTURE airport+.json "departure_holds" override (parse stubbed
    //           in airport_overrides). LIMITATION: the FL110 fallback can sit above
    //           the origin's FIRST TMA volume ceiling (e.g. FL95) -- accepted for the
    //           default; the data-driven probe avoids this whenever openair resolves.
    constexpr int kDefaultDepStep1Ft = 11000; // FL110 fallback default
    constexpr float kDefaultDepHoldNm = 10.0f; // 10 NM fallback hold
    const int sid_min_ft = ctx.ifr_sid_min_alt_ft > 0 ? ctx.ifr_sid_min_alt_ft : 0;
    const int cruise_ft =
        ctx.ifr_cruise_alt_ft > 0 ? ctx.ifr_cruise_alt_ft : sid_min_ft + 8000;
    const float dep_field_elev_ft = ctx.altitude_ft_msl - ctx.height_agl_ft;
    auto fl_below = [](int ceil) { return ((ceil - 100) / 1000) * 1000; };

    // Probe the vertical airspace STACK ~10 NM out: the FIRST route fix beyond 5 NM
    // from the field (within the ~40 NM terminal area). At LFLP that point is under
    // the FL115 TMA, not the FL95 field sector. BOTH intermediate steps follow ONE
    // rule -- highest FL just below the NEXT airspace ceiling (user 2026-07-24):
    //   step1 = highest FL below the TERMINAL TMA ceiling  (~FL115 -> FL110);
    //   step2 = highest FL below the CTA stacked above it   (~FL145 -> FL140), i.e.
    //           the last level below the FIR/Radar floor. Both data-driven, mirrored.
    int probe_ceil = 0; // terminal TMA ceiling at the probe point (ft)
    int cta_ceil = 0;   // ceiling of the volume stacked directly above it (ft)
    if (openair_db::ready() &&
        (s_departure_apt_lat != 0.0 || s_departure_apt_lon != 0.0)) {
      for (const auto &rf : s_route_fixes) {
        if (rf.lat == 0.0 && rf.lon == 0.0)
          continue;
        const double d = traffic_geometry::distance_nm(
            s_departure_apt_lat, s_departure_apt_lon, rf.lat, rf.lon);
        if (d < 5.0)
          continue; // still the low field CTR/TMA sector
        if (d > 40.0)
          break; // past the terminal area -> no useful terminal ceiling
        probe_ceil = openair_db::terminal_tma_ceiling(rf.lat, rf.lon);
        if (probe_ceil > 1500) {
          // Volume stacked directly ABOVE the TMA (the "high" CTA): sample just
          // above the TMA ceiling. Its ceiling is the FIR/Radar floor (~FL145).
          const openair_db::AirspaceEntry above =
              openair_db::find_enclosing(rf.lat, rf.lon, probe_ceil + 500);
          if (above.ceiling_ft > probe_ceil)
            cta_ceil = above.ceiling_ft;
        }
        break; // first fix beyond 5 NM is the ~10 NM probe point
      }
    }

    int step1 = kDefaultDepStep1Ft; // FL110 fallback
    bool from_probe = false;
    if (probe_ceil > 1500) {
      const int fl = fl_below(probe_ceil);
      // Plausible only if above the field and below cruise.
      if (fl > static_cast<int>(dep_field_elev_ft) && fl < cruise_ft &&
          fl >= 3000) {
        step1 = fl;
        from_probe = true;
      }
    }
    // SECOND step: highest FL just below the CTA ceiling (the FIR/Radar floor),
    // data-driven like step1. Falls back to generic FL140 (a "high" CTA usually tops
    // at FL145) when no CTA resolves. Clamped below cruise / >= SID min further down.
    constexpr int kDefaultDepStep2Ft = 14000; // FL140 generic fallback second step
    int step2 = kDefaultDepStep2Ft;
    bool step2_from_probe = false;
    if (cta_ceil > 1500) {
      const int fl2 = fl_below(cta_ceil);
      if (fl2 > step1 && fl2 < cruise_ft) {
        step2 = fl2;
        step2_from_probe = true;
      }
    }
    float hold_nm = kDefaultDepHoldNm;

    // Per-airport airport+.json override (LOCAL procedure, e.g. Annecy FL110 held
    // 30 NM under the Chambery/Geneva TMAs). Overrides step1 / hold distance / step2
    // ONLY for the fields present in the JSON; the rest keep the generic values.
    const bool has_override = airport_overrides::departure_hold(
        s_departure_apt_id, ctx.ifr_sid_last_fix, &step1, &hold_nm, &step2);

    // Never clear below the SID's published minimum crossing altitude -- UNLESS
    // that minimum sits at the SID's EXIT fix (an enroute-climb altitude, not an
    // early departure floor; see sid_min_at_exit_fix). Otherwise a SID whose only
    // binding is at a high exit fix (LIMF KUKE1Z: KUKEV FL200) would force step1
    // straight to FL200 out of the 2000 ft level-off instead of the progressive
    // FL110 -> FL140 -> cruise ladder. (user 2026-07-27) [C. P. Potter]
    if (sid_min_ft > step1 && !sid_min_at_exit_fix(ctx))
      step1 = sid_min_ft;

    if (step1 >= cruise_ft) {
      // Cruise at/below step1 -> no hold, climb straight to cruise, no second step.
      s_sid_step1_alt_ft = round_to_fl(cruise_ft) * 100;
      s_sid_hold_release_nm = 0.0f;
      s_sid_step2_alt_ft = 0;
    } else {
      s_sid_step1_alt_ft = step1;
      s_sid_hold_release_nm = hold_nm;
      // step2 usable only strictly between step1 and cruise, and >= the SID min
      // FLOOR (0 when the only binding is at the SID exit fix -- see
      // sid_climb_floor_ft; otherwise FL140 collapses under a FL200 exit binding).
      s_sid_step2_alt_ft = (step2 > step1 && step2 < cruise_ft &&
                            step2 >= sid_climb_floor_ft(ctx))
                               ? step2
                               : 0;
    }
    logging::info("IFR SID climb: probe10nm tma_ceil=%d cta_ceil=%d (step1 %s / "
                  "step2 %s%s) dep=%s -> step1 FL%d step2 FL%d hold %.0f NM "
                  "(sid_min=%d cruise=%d)",
                  probe_ceil, cta_ceil,
                  from_probe ? "data-driven" : "FL110-fallback",
                  step2_from_probe ? "data-driven" : "FL140-fallback",
                  has_override ? "+airport+.json" : "", s_departure_apt_id.c_str(),
                  s_sid_step1_alt_ft / 100, s_sid_step2_alt_ft / 100,
                  s_sid_hold_release_nm, sid_min_ft, cruise_ft);
  }

  // ── Phase 2.8: post-hold handoff to the controller ABOVE the departure TMA ──
  // AIRSPACE.TXT-DRIVEN (user 2026-07-24): after the step-1 hold, at step1 (just
  // below the TMA ceiling), hand off to the controller of the volume STACKED ABOVE
  // the departure TMA. The volume and its NAME come from openair (airspace.txt) via
  // find_enclosing; resolve_sector_controller maps that name to the atc.dat
  // FREQUENCY (openair carries no frequencies). The upper controller then clears
  // step2 (FL140) on the pilot's check-in (s_sid_pending_climb_ft), not the departure
  // APP. Fires only when a controller DISTINCT from the pilot's current one resolves;
  // otherwise the fallback Phase 2a clears FL140 in-block so the climb never stalls.
  const int sid2_cruise_fl_ph28 =
      round_to_fl(ctx.ifr_cruise_alt_ft > 0 ? ctx.ifr_cruise_alt_ft
                                            : s_sid_step1_alt_ft + 4000);
  const bool step2_usable_ph28 =
      s_sid_step2_alt_ft > 0 && s_sid_step2_alt_ft < sid2_cruise_fl_ph28 * 100 &&
      s_sid_step2_alt_ft >= sid_climb_floor_ft(ctx);
  if (!s_sid_radar_handoff_issued && s_sid_initialized && s_sid_step1_issued &&
      !s_sid_step2_issued && step2_usable_ph28 && s_sid_pending_climb_ft == 0 &&
      !sid_step1_hold_active(ctx) && openair_db::ready() &&
      static_cast<int>(ctx.altitude_ft_msl) >= s_sid_step1_alt_ft - 500) {
    // Volume just ABOVE the departure TMA at the aircraft's current position (the
    // "high" CTA). step1 is the highest FL below the TMA ceiling, so +1500 clears it.
    const openair_db::AirspaceEntry above = openair_db::find_enclosing(
        ctx.latitude, ctx.longitude, s_sid_step1_alt_ft + 1500);
    const float active_com_freq_mhz =
        (ctx.active_com == 2) ? ctx.com2_freq_mhz : ctx.com1_freq_mhz;
    // Never resolve the handoff to the freq we're already on: two same-named
    // controllers (LYON = LFLB 121.205 current vs LFLL 120.230 Sector 4) are
    // disambiguated by avoiding the current COM (verified atc.dat, 2026-07-24).
    const std::uint32_t avoid_khz =
        static_cast<std::uint32_t>(std::lround(active_com_freq_mhz * 1000.0));
    std::string lbl;
    float mhz = 0.0f;
    if (resolve_sector_controller(above, /*terminal=*/true, &lbl, &mhz,
                                  avoid_khz) &&
        mhz >= 100.0f) {
      const bool distinct = std::fabs(mhz - active_com_freq_mhz) >= 0.010f;
      if (distinct) {
        const std::string &cs_ir = atc_state_machine::session_callsign();
        const std::string &callsign_ir =
            cs_ir.empty() ? settings::pilot_callsign() : cs_ir;
        if (out_text) {
          char buf[160];
          std::snprintf(buf, sizeof(buf), "%s, contact %s on %.3f.",
                        callsign_ir.c_str(), lbl.c_str(), mhz);
          *out_text = buf;
        }
        s_pending_controller_label = lbl;
        s_pending_handoff_freq_mhz = mhz;
        s_sector_checkin_pending = true;
        s_sid_pending_climb_ft = s_sid_step2_alt_ft; // FL140, cleared on check-in
        s_sid_step2_issued = true;                   // queued -> blocks Phase 2a
        logging::info("IFR SID climb: post-hold handoff -> %s %.3f (openair '%s') "
                      "at %.0f ft MSL; FL%d queued for pilot check-in",
                      lbl.c_str(), mhz, above.name.c_str(), ctx.altitude_ft_msl,
                      s_sid_step2_alt_ft / 100);
        return true;
      }
    }
  }

  // Dwell at the second step (FL140) before cruise / the FIR handoff, so the aircraft
  // actually LEVELS off at FL140 rather than being re-cleared on the way up
  // (user 2026-07-24). Accumulates once the aircraft is at/near step2.
  constexpr float kSidStep2DwellSec = 60.0f; // ~4-5 NM at climb speed
  if (s_sid_step2_issued && !s_sid_cruise_issued && s_sid_step2_alt_ft > 0 &&
      static_cast<int>(ctx.altitude_ft_msl) >= s_sid_step2_alt_ft - 200)
    s_sid_step2_dwell_sec += dt;

  // ── Phase 2.9: FL140 -> FIR/Radar -> cruise (AIRSPACE.TXT-DRIVEN) ─────
  // After the FL140 dwell, hand off to the controller of the volume ABOVE the CTA
  // (the FIR/Radar). The volume + NAME come from openair (airspace.txt) via
  // find_enclosing just above step2 (into the FIR); resolve_sector_controller
  // (terminal=false -> a CTA/FIR/UIR resolves to the ACC) maps the name to the
  // atc.dat frequency. The FIR clears CRUISE on the pilot's check-in -- the climb
  // above the CTA comes from the FIR, in their airspace (user 2026-07-24,
  // Eurocontrol). Because openair carries the FINE FL145 boundary, this resolves the
  // FIR even where atc.dat lumps it under a wide TMA blob. Fires only when a DISTINCT
  // controller resolves; otherwise Phase 2b clears cruise in-block.
  const int sid_cruise_ft_ph29 =
      round_to_fl(ctx.ifr_cruise_alt_ft > 0 ? ctx.ifr_cruise_alt_ft
                                            : s_sid_step1_alt_ft + 4000) *
      100;
  if (!s_sid_radar_handoff_issued && s_sid_step2_issued && !s_sid_cruise_issued &&
      s_sid_pending_climb_ft == 0 && sid_cruise_ft_ph29 > s_sid_step2_alt_ft &&
      s_sid_step2_dwell_sec >= kSidStep2DwellSec && openair_db::ready()) {
    // Volume just ABOVE the CTA at the aircraft's current position (the FIR/Radar).
    // step2 is the highest FL below the CTA ceiling, so +1500 clears it into the FIR.
    const openair_db::AirspaceEntry fir = openair_db::find_enclosing(
        ctx.latitude, ctx.longitude, s_sid_step2_alt_ft + 1500);
    const float active_com_freq_mhz =
        (ctx.active_com == 2) ? ctx.com2_freq_mhz : ctx.com1_freq_mhz;
    const std::uint32_t avoid_khz =
        static_cast<std::uint32_t>(std::lround(active_com_freq_mhz * 1000.0));
    std::string lbl;
    float mhz = 0.0f;
    // Delegation FIRs (SKYGUIDE/MUAC) resolve here; a plain area CTA (MARSEILLE CTA)
    // does not (resolve_acc_controller is delegation-only), so fall back to the
    // atc.dat CTR matched by the openair sector NAME -- openair volume+name, atc.dat
    // freq, avoiding the current COM (per [[coding_airspace_txt_authoritative]]).
    if (!(resolve_sector_controller(fir, /*terminal=*/false, &lbl, &mhz,
                                    avoid_khz) &&
          mhz >= 100.0f)) {
      const std::string frag = openair_sector_label(fir.name);
      const airspace_db::Controller *ctr =
          frag.empty() ? nullptr
                       : airspace_db::find_by_role_name_contains(
                             airspace_db::ControllerRole::CTR, frag, avoid_khz);
      if (ctr && !ctr->freqs_khz.empty()) {
        mhz = static_cast<float>(ctr->freqs_khz.front()) / 1000.0f;
        lbl = frag; // openair name for the spoken label (e.g. "Marseille")
      }
    }
    if (mhz >= 100.0f) {
      const bool distinct = std::fabs(mhz - active_com_freq_mhz) >= 0.010f;
      if (distinct) {
        const std::string &cs_fir = atc_state_machine::session_callsign();
        const std::string &callsign_fir =
            cs_fir.empty() ? settings::pilot_callsign() : cs_fir;
        if (out_text) {
          char buf[160];
          std::snprintf(buf, sizeof(buf), "%s, contact %s on %.3f.",
                        callsign_fir.c_str(), lbl.c_str(), mhz);
          *out_text = buf;
        }
        s_pending_controller_label = lbl;
        s_pending_handoff_freq_mhz = mhz;
        s_sector_checkin_pending = true;
        s_sid_pending_climb_ft = sid_cruise_ft_ph29; // FIR clears cruise on check-in
        s_sid_cruise_issued = true;        // queued -> blocks the Phase 2b fallback
        s_sid_radar_handoff_issued = true; // this IS the final SID/enroute handoff
        logging::info("IFR SID climb: FIR handoff -> %s %.3f (openair '%s') at "
                      "%.0f ft MSL; cruise FL%d queued for pilot check-in",
                      lbl.c_str(), mhz, fir.name.c_str(), ctx.altitude_ft_msl,
                      sid_cruise_ft_ph29 / 100);
        return true;
      }
    }
  }

  // ── Phase 3: radar handoff — fires when aircraft exits the TMA ───────
  // Requires step1 already issued so we never hand off before the first
  // climb clearance. Use find_enclosing() on the aircraft's 3-D position:
  // while still inside a CTR or TMA we hold; once in CTA/FIR/uncontrolled
  // we hand off to Area Control / Radar.
  // Fall back to a configured or computed altitude when airspace.txt is absent.
  //
  // GUARD (user 2026-07-24, in-sim): do NOT fire while the FL140->cruise ladder is
  // still mid-progress (step2 cleared/queued but cruise not yet). LYON CTA is class
  // CTA, so the "exited all TMAs" test below thinks the aircraft has left terminal
  // airspace at ~FL115 and silently jumps to EN_ROUTE -- which preempted Phase 2.9
  // (FL140 dwell -> Marseille -> cruise) and stopped the ladder at FL140. Once the
  // ladder finishes (Phase 2.9 sets radar_handoff_issued, or Phase 2b sets
  // cruise_issued), this guard opens and Phase 3 resumes normally.
  const bool sid_ladder_in_progress =
      s_sid_step2_issued && !s_sid_cruise_issued;
  if (!s_sid_radar_handoff_issued && s_sid_step1_issued &&
      !sid_ladder_in_progress) {
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

  const int sid_cruise_fl =
      round_to_fl(ctx.ifr_cruise_alt_ft > 0 ? ctx.ifr_cruise_alt_ft
                                            : s_sid_step1_alt_ft + 4000);
  // A configured second step is USABLE only when it sits strictly below cruise AND
  // is not below the SID's published minimum crossing altitude (never clear below a
  // SID fix min -- user 2026-07-22). Otherwise skip straight to cruise.
  const bool step2_usable = s_sid_step2_alt_ft > 0 &&
                            s_sid_step2_alt_ft < sid_cruise_fl * 100 &&
                            s_sid_step2_alt_ft >= sid_climb_floor_ft(ctx);

  // ── Phase 2a: second intermediate step (Annecy FL110 -> FL140) ────────
  // FALLBACK only: Phase 2.8 normally hands the aircraft to the upper controller and
  // queues FL140 for the check-in (setting s_sid_step2_issued). This fires FL140
  // directly ONLY when Phase 2.8 found no upper controller to hand off to (data gap),
  // so the climb never stalls. Never while a check-in climb is pending.
  if (s_sid_step1_issued && step2_usable && !s_sid_step2_issued &&
      !s_sid_cruise_issued && s_sid_pending_climb_ft == 0) {
    // Reached OR passed the step (not a symmetric +/-500 window): a climb that
    // overshoots the step must still release the next clearance, never strand.
    const bool near_step1 =
        static_cast<int>(ctx.altitude_ft_msl) >= s_sid_step1_alt_ft - 500;
    if (near_step1 && !sid_step1_hold_active(ctx)) {
      s_sid_step2_issued = true;
      s_enroute_cleared_alt_ft = s_sid_step2_alt_ft;
      const int fl2 = s_sid_step2_alt_ft / 100;
      if (out_text) {
        char buf[64];
        std::snprintf(buf, sizeof(buf), "%s, climb flight level %d.",
                      callsign.c_str(), fl2);
        *out_text = buf;
      }
      logging::info("IFR SID climb: FL%d (step2)", fl2);
      return true;
    }
  }

  // ── Phase 2b: climb to cruise FL when near the last intermediate step ──
  // In-block cruise FALLBACK: fires only when no check-in climb is pending and
  // Phase 2.9 did not hand off to a distinct FIR (coarse atc.dat), so cruise is
  // cleared by the current controller (correct when cruise is within its block).
  if (s_sid_step1_issued && !s_sid_cruise_issued && s_sid_pending_climb_ft == 0) {
    // Defer while a usable second step is still pending (Phase 2a owns it).
    const bool step2_pending = step2_usable && !s_sid_step2_issued;
    const int near_target = (step2_usable && s_sid_step2_issued)
                                ? s_sid_step2_alt_ft
                                : s_sid_step1_alt_ft;
    // Reached OR passed the target (see Phase 2a) -- overshoot must not strand.
    const bool near =
        static_cast<int>(ctx.altitude_ft_msl) >= near_target - 500;
    const bool timeout = s_sid_climb_timer > 900.0f; // 15-min safety net
    // When stepping FL140 -> cruise, respect the FL140 dwell so the aircraft levels
    // off first; no dwell for the plain step1 -> cruise case (no second step).
    const bool step2_dwell_ok = !(step2_usable && s_sid_step2_issued) ||
                                s_sid_step2_dwell_sec >= kSidStep2DwellSec;
    if (!step2_pending && step2_dwell_ok && (near || timeout) &&
        !sid_step1_hold_active(ctx)) {
      s_sid_cruise_issued = true;
      s_enroute_cleared_alt_ft =
          sid_cruise_fl * 100; // record for en-route altitude monitoring
      if (out_text) {
        char buf[64];
        std::snprintf(buf, sizeof(buf), "%s, climb flight level %d.",
                      callsign.c_str(), sid_cruise_fl);
        *out_text = buf;
      }
      logging::info("IFR SID climb: FL%d (cruise clearance)", sid_cruise_fl);
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
    // Issue step1 (FL110) IMMEDIATELY on radar contact -- NOT at 10 NM. A fast
    // climber (TBM) reaches FL110 before 10 NM, so a distance-gated step1 was
    // skipped and the climb jumped straight to cruise, killing the hold + ladder
    // (user 2026-07-24). The direct-to shortcut still waits until >=10 NM.
    const bool far_enough = dist_nm >= 10.0; // gates only the direct-to shortcut
    if (!s_sid_step1_issued) {
      // 20 % probability (only once >=10 NM out): fire the "direct <SID last fix>,
      // climb FL X" shortcut. 80 % of the time -- and always closer in -- issue a
      // plain "climb FL X" without the direct-to. Only gate if we actually have a
      // valid SID last fix to direct-to; otherwise always the plain climb.
      const bool have_last_fix = !ctx.ifr_sid_last_fix.empty();
      const bool fire_direct =
          have_last_fix && far_enough && ((std::rand() % 5) == 0);
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
      s_enroute_cleared_alt_ft = s_sid_step1_alt_ft; // step1 is now the cleared level
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

  // STAR entry resolution -- the MIRROR of the SID (SID = first FPL fix + runway ->
  // SID from CIFP; STAR = last FPL fix + runway -> STAR from CIFP). The STAR is
  // ATC/CIFP-decided; SimBrief's filed SID/STAR is IGNORED. So instead of trusting
  // SimBrief's is_sid_star classification (which files the STAR's own entry fix in
  // the ENROUTE portion for e.g. ROMA3P -> ROMAM, breaking the old "first
  // is_sid_star fix" heuristic that picked LSE), scan the filed route fixes and ask
  // the CIFP which one is a STAR ENTRY. Only a STAR's FIRST fix matches
  // star_name_for_entry_fix, so mid-STAR fixes (LSE/GOVNA/PIRUV) never false-match;
  // the nearest-departure entry wins. Handles ROMA3P (entry ROMAM, filed enroute)
  // AND SALEV3P/ABDIL (entry filed in the STAR group) uniformly. [C. P. Potter]
  std::string star_entry_ident;
  if (!cifp_dir.empty()) {
    for (int i = (int)ofp.navlog.size() - 1; i >= 0; --i) {
      const auto &fix = ofp.navlog[i];
      if (fix.ident.empty() || fix.ident == ofp.destination_icao ||
          fix.ident == ofp.origin_icao || fix.ident == "TOC" ||
          fix.ident == "TOD")
        continue;
      if (!cifp_reader::star_name_for_entry_fix(cifp_dir, ofp.destination_icao, "",
                                                fix.ident)
               .empty())
        star_entry_ident = fix.ident; // keep scanning: nearest-departure entry wins
    }
  }

  // Fallback (CIFP has no matching STAR / no CIFP): first fix of the first
  // is_sid_star group, else the last navlog fix before destination.
  if (star_entry_ident.empty()) {
    bool in_group = false;
    for (const auto &fix : ofp.navlog) {
      if (fix.is_sid_star && !in_group && !fix.ident.empty() &&
          fix.ident != ofp.destination_icao)
        star_entry_ident = fix.ident;
      in_group = fix.is_sid_star;
    }
  }
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

  // ── 2b-P1: STEP a very-high-cruise descent ────────────────────────────
  // From a very high cruise (e.g. FL450) a single "descend FLxx" is a 25-33k
  // ft clearance -- unrealistic. Cap the FIRST descent at FL200 (a standard
  // intermediate near the common FL195 UIR/FIR band) and defer the remainder
  // to a SECOND clearance issued by poll_descent_second_step once the aircraft
  // nears FL200. Only when cruise is above FL200 AND the ultimate target is
  // meaningfully below FL180 (else a single step is already realistic). The
  // freq change is DECOUPLED -- poll_acc_sector_change fires at the real sector
  // boundary, NOT tied to this altitude step (the Ljubljana FIR runs to FL660,
  // so there is no guaranteed FL195 freq cut). [[project_stepped_descent]]
  s_descent_final_target_ft = 0;
  s_descent_first_step_ft = 0;
  s_descent_second_step_issued = false;
  // Restricted to a real STAR arrival (star_name set): the no-STAR direct-to-IAF
  // path (2c below) builds its own "direct X, descend <iaf alt>" phrase, so
  // capping star_alt_ft there would double-speak the descent.
  if (!star_name.empty() && cruise_ref_dc > 24000 && star_alt_ft > 0 &&
      star_alt_ft < 18000) {
    s_descent_final_target_ft = star_alt_ft; // remember the ultimate target
    // First step: FL200 by default, but never BELOW the top of the highest TMA
    // currently overflown -- stay ABOVE an enroute TMA (LOWI/DOLSKO tops FL245
    // -> first step FL250) instead of diving into it. Rounds up to the next FL
    // above the ceiling. [[project_stepped_descent]]
    int first_step = 20000;
    const int hi_ceil = openair_db::ready()
                            ? openair_db::highest_tma_ceiling(ctx.latitude,
                                                              ctx.longitude)
                            : 0;
    if (hi_ceil > 0) {
      const int above = ((hi_ceil / 1000) + 1) * 1000; // FL245 -> FL250
      if (above > first_step)
        first_step = above;
    }
    star_alt_ft = first_step;
    s_descent_first_step_ft = first_step;
    logging::info(
        "IFR descent: stepping high cruise %d ft -> FL%d first (TMA top %d), then %d ft (deferred)",
        cruise_ref_dc, first_step / 100, hi_ceil, s_descent_final_target_ft);
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

// STAR-clearance safety net. The arrival clearance (STAR + expect-approach, via
// build_descent_clearance) is normally issued around the TOD inside poll_enroute,
// which runs ONLY in IFR_ENROUTE_CRUISE. On a SHORT flight the aircraft can reach
// the STAR entry BEFORE cruise/TOD -- LFLU->LFLP filed FL140 crosses ROMAM (the
// ROMA3P entry) still in the CLIMB (IFR_RADAR_CONTACT) -- so the clearance is never
// issued at the STAR. This fires build_descent_clearance a bit BEFORE the STAR
// entry when it hasn't been issued yet (user rule 2026-08-03). build_descent_clearance
// itself picks the altitude: if the STAR profile is below the current cleared level
// it says "descend FLxxx" (stop the climb -- correct for the compressed profile),
// else routing only. Runs ONLY in IFR_RADAR_CONTACT (climb) + IFR_ENROUTE_CRUISE
// (backup); IFR_DESCENT is entered BY build_descent_clearance so the clearance is
// already given there -- no descent scenario (user confirmed). [C. P. Potter]
static constexpr float kStarClearanceLeadNm =
    12.0f; // issue the arrival clearance this far before the STAR entry
bool poll_star_clearance_safety_net(const xplane_context::XPlaneContext &ctx,
                                    std::string *out_text,
                                    bool *out_requires_readback) {
  using AS = atc_state_machine::ATCState;
  const AS st = atc_state_machine::get_state();
  if (st != AS::IFR_RADAR_CONTACT && st != AS::IFR_ENROUTE_CRUISE)
    return false;
  if (s_enroute_descent_issued || !s_assigned_star_name.empty())
    return false; // arrival clearance already issued
  if (ctx.cifp_dir.empty())
    return false;
  auto ofp = simbrief_ofp::get();
  if (!ofp.valid)
    return false;
  StarEntryResult se;
  if (!find_star_entry(ctx.cifp_dir, ofp, se) ||
      (se.lat == 0.0 && se.lon == 0.0))
    return false; // no STAR (AFIS / no-STAR arrival) -> nothing to pre-empt
  const double d = traffic_geometry::distance_nm(ctx.latitude, ctx.longitude,
                                                 se.lat, se.lon);
  if (d > kStarClearanceLeadNm)
    return false; // not yet within lead distance of the STAR entry
  const std::string &cs = atc_state_machine::session_callsign();
  const std::string &callsign = cs.empty() ? settings::pilot_callsign() : cs;
  const auto &defaults = flight_phase::get_ifr_defaults();
  logging::info("IFR STAR safety-net: %.0f NM from STAR entry %s, arrival clearance "
                "not yet issued -> issuing now (state %s)",
                d, se.ident.c_str(), atc_state_machine::state_name(st));
  if (build_descent_clearance(ctx, callsign, defaults, out_text)) {
    if (out_requires_readback)
      *out_requires_readback = true;
    return true;
  }
  return false;
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
                      const airspace_db::Controller **out_label_ctrl,
                      std::uint32_t avoid_freq_khz = 0) {
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
  // avoid_freq_khz: when this resolves a HANDOFF target, never pick the frequency
  // the pilot is already on -- disambiguates two same-named controllers (LYON =
  // LFLB 121.205 current vs LFLL 120.230 handoff). 0 = legacy first-match.
  const airspace_db::Controller *ctrl = airspace_db::find_by_role_name_contains(
      airspace_db::ControllerRole::TRACON, fragment, avoid_freq_khz);
  const airspace_db::Controller *label_ctrl = ctrl;
  if (!ctrl || ctrl->freqs_khz.empty()) {
    const airspace_db::Controller *named = airspace_db::find_by_role_name_contains(
        airspace_db::ControllerRole::TWR, fragment, avoid_freq_khz);
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
                                   std::string *out_label, float *out_freq_mhz,
                                   std::uint32_t avoid_freq_khz = 0) {
  const airspace_db::Controller *label_ctrl = nullptr;
  const airspace_db::Controller *ctrl =
      resolve_terminal_ctrl(tma_name, &label_ctrl, avoid_freq_khz);
  if (!ctrl)
    return false;
  if (out_freq_mhz)
    *out_freq_mhz = static_cast<float>(ctrl->freqs_khz.front()) / 1000.0f;
  if (out_label) {
    // Satellite-facility guard: an atc.dat TRACON can be NAMED after a big area
    // but tied to a SMALL satellite field ("MILAN RADAR" -> facility LILN =
    // "Venegono"), so controller_label_for() gives the WRONG spoken name. When the
    // facility name does NOT match the openair AREA name, prefer the openair name
    // ("MILAN CTA..." -> "Milan"). Chambery matches (facility LFLB -> "Chambery"
    // ~ area "CHAMBERY") so it is UNAFFECTED (user 2026-07-26).
    const std::string facility_label = controller_label_for(label_ctrl);
    const std::string area_label = openair_sector_label(tma_name);
    auto lc = [](std::string s) {
      for (char &c : s)
        c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
      return s;
    };
    std::string base = facility_label;
    if (facility_label.empty()) {
      base = area_label;
    } else if (!area_label.empty()) {
      const std::string fl = lc(facility_label), al = lc(area_label);
      if (fl.find(al) == std::string::npos && al.find(fl) == std::string::npos)
        base = area_label; // mismatch -> openair area name (Milan, not Venegono)
    }
    *out_label = base + " Approach";
  }
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
                                   std::string *out_label, float *out_mhz,
                                   std::uint32_t avoid_freq_khz = 0) {
  const std::string org = delegated_org(name);
  if (org.empty())
    return false;
  const airspace_db::Controller *c =
      airspace_db::find_by_role_facility(airspace_db::ControllerRole::CTR, org);
  if (!c || c->freqs_khz.empty())
    c = airspace_db::find_by_role_name_contains(
        airspace_db::ControllerRole::CTR, org, avoid_freq_khz);
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
                                      float *out_mhz,
                                      std::uint32_t avoid_freq_khz) {
  switch (enc.ac_class) {
  case openair_db::AirspaceClass::TMA:
  case openair_db::AirspaceClass::CTR:
    return resolve_tma_controller(enc.name, out_label, out_mhz, avoid_freq_khz);
  case openair_db::AirspaceClass::CTA:
    return terminal
               ? resolve_tma_controller(enc.name, out_label, out_mhz, avoid_freq_khz)
               : resolve_acc_controller(enc.name, out_label, out_mhz, avoid_freq_khz);
  case openair_db::AirspaceClass::FIR:
  case openair_db::AirspaceClass::UIR:
    return resolve_acc_controller(enc.name, out_label, out_mhz, avoid_freq_khz);
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

  // P0: explicit airport+.json controller override for the destination approach.
  // Data-driven freq correction where atc.dat has no TRACON for the field and the
  // TMA therefore resolves to the Tower instead (LOWI: Innsbruck TMA -> only an
  // atc.dat Tower 120.10; the real Approach 119.275 lives in apt.dat/airport+.json).
  // The VOLUME is still the openair Innsbruck TMA that triggered this handoff --
  // this only supplies the correct frequency. Mirrors the departure handoff P0.
  // [C. P. Potter]
  {
    const std::string dest = !s_assigned_dest_icao.empty()
                                 ? s_assigned_dest_icao
                                 : ctx.nearest_airport_id;
    std::string oname;
    float ofreq = 0.0f;
    if (airport_overrides::controller(dest, "approach", &oname, &ofreq) &&
        ofreq >= 100.0f) {
      app_label = oname;
      app_freq = ofreq;
      logging::info("IFR arrival handoff: [P0-airport+.json] %s %.3f",
                    app_label.c_str(), app_freq);
    }
  }

  logging::info(
      "IFR arrival handoff: openair enc='%s' class=%d floor=%dft ceil=%dft"
      " at %.0fft MSL pos=%.4f,%.4f",
      enc.name.c_str(), static_cast<int>(enc.ac_class),
      enc.floor_ft, enc.ceiling_ft,
      ctx.altitude_ft_msl, ctx.latitude, ctx.longitude);

  // Primary: use the openair TMA name to find the correct TRACON by name
  // (same logic as departure handoff — altitude-correct, not centroid-distance).
  // "CHAMBERY TMA SECTOR 1" → "CHAMBERY" → Chambery TRACON.
  // Skipped when P0 (airport+.json) already supplied the frequency.
  if (app_freq < 100.0f &&
      (enc.ac_class == openair_db::AirspaceClass::TMA ||
       enc.ac_class == openair_db::AirspaceClass::CTA)) {
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

// Heading-compliance primitive (sibling of check_course in the "DirectMonitor" family):
// absolute error (deg) between the aircraft's magnetic heading and an ASSIGNED heading
// (an ATC vector) -- distinct from check_course, which tracks the next ROUTE-fix bearing.
// The caller sets the threshold + reaction; the vector-compliance monitor uses it, and it
// folds into the unified monitor later (feedback_refactor_unify #2). [C. P. Potter]
static double heading_error_deg(double heading_mag, double assigned_mag) {
  double d = std::fabs(heading_mag - assigned_mag);
  if (d > 180.0) d = 360.0 - d;
  return d;
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

      // Controlling sector: OPENAIR FIR/sector geometry FIRST (airspace.txt carries the
      // fine FIR boundaries -- Milan / Ljubljana / Wien -- that the COARSE atc.dat
      // polygons miss at cruise: Milan's atc.dat FIR overflows into Slovenia so the
      // boundary never fired and the Milan->Ljubljana->Wien handoffs were silently
      // skipped (user 2026-07-29, LFMN->LOWI). atc.dat sector-picker is the fallback.
      // Mirrors poll_acc_sector_change so CRUISE and DESCENT use the SAME resolution;
      // resolve_sector_controller(terminal=false) yields the enroute ACC/FIR. [CPP]
      // Controlling sector: OPENAIR FIRST (airspace.txt has the FINE TMA/CTA/CTR geometry
      // -- e.g. entering DOLSKO TMA FL195-245 on descent), then the atc.dat CONTROLLER
      // POLYGON as fallback. The "atc.dat = freq only" rule holds for TMAs but CANNOT
      // apply to enroute FIRs/UIRs: airspace.txt does NOT contain FIR polygons at all
      // (Slovenia's highest openair volume is DOLSKO TMA at FL245), so at cruise FL450
      // find_enclosing(openair) returns NOTHING. The FIR geometry lives ONLY in atc.dat
      // as the controller's own AIRSPACE_POLYGON (LJUBLJANA 0-66000 ft encloses KUBUD at
      // FL450, verified hors-sim 2026-07-29). So atc.dat supplies BOTH geometry AND freq
      // for enroute FIRs -- unavoidable, openair simply lacks them. [CPP]
      uint32_t new_freq_khz = 0;
      std::string new_label;
      int sector_floor_ft = 0;
      if (openair_db::ready()) {
        const openair_db::AirspaceEntry enc = openair_db::find_enclosing(
            ctx.latitude, ctx.longitude, openair_alt(ctx));
        std::string lbl;
        float f = 0.0f;
        if (resolve_sector_controller(enc, /*terminal=*/false, &lbl, &f) &&
            f > 0.0f) {
          new_freq_khz = static_cast<uint32_t>(std::lround(f * 1000.0));
          new_label = lbl;
          sector_floor_ft = enc.floor_ft;
        }
      }
      if (new_freq_khz == 0) { // no openair sector (e.g. enroute FIR) -> atc.dat polygons
        const airspace_db::Controller *best = sector_picker::pick_next(
            ctx.enclosing_airspaces, s_enroute_visited_sector_freqs);
        if (best) {
          new_freq_khz = best->freqs_khz.front();
          new_label = controller_label_for(best);
          sector_floor_ft = best->floor_ft;
        }
      }

      if (new_freq_khz != 0) {
        if (s_enroute_sector_freq_khz == 0) {
          // Seed silently: Phase 2/3 already gave the pilot the correct freq; the
          // FIRST real sector change is announced, not the baseline.
          s_enroute_sector_freq_khz = new_freq_khz;
          logging::info("IFR en-route: sector baseline %s %.3f MHz floor=%dft (silent)",
                        new_label.c_str(),
                        static_cast<float>(new_freq_khz) / 1000.0f, sector_floor_ft);
        } else if (new_freq_khz != s_enroute_sector_freq_khz) {
          // Never hand back to a sector already left (openair has no visited filter of
          // its own, unlike pick_next; prevents flicker at a boundary).
          bool already_visited = false;
          for (uint32_t v : s_enroute_visited_sector_freqs)
            if (v == new_freq_khz) { already_visited = true; break; }
          if (!already_visited) {
            // Sector changed -> issue handoff, wait for the pilot to check in.
            s_enroute_visited_sector_freqs.push_back(s_enroute_sector_freq_khz);
            s_enroute_sector_freq_khz = new_freq_khz;
            s_pending_controller_label = new_label; // deferred label switch
            const float new_freq_mhz = static_cast<float>(new_freq_khz) / 1000.0f;
            s_pending_handoff_freq_mhz = new_freq_mhz;
            const float active_com_now =
                (ctx.active_com == 2) ? ctx.com2_freq_mhz : ctx.com1_freq_mhz;
            if (std::fabs(active_com_now - new_freq_mhz) < 0.005f) {
              s_sector_checkin_pending = false;
              logging::info("IFR en-route: sector change -> %s %.3f MHz (already on freq -- silent)",
                            new_label.c_str(), new_freq_mhz);
              return false;
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
            logging::info("IFR en-route: sector change -> %s %.3f MHz floor=%dft",
                          new_label.c_str(), new_freq_mhz, sector_floor_ft);
            rb(true);
            return true;
          }
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
        // VERB from the aircraft's CURRENT altitude, NOT the previously-cleared FL:
        // a filed step DOWN (e.g. FL220->FL210) issued while the aircraft is still
        // CLIMBING and BELOW the step target must not say "descend" (physically
        // impossible). Below target -> climb/maintain; above -> descend (user
        // 2026-07-26: cleared FL220, climbing through FL180, step FL210 said
        // "descend FL210"). "maintain" when the aircraft is already at the level
        // (the step just caps a higher previous clearance = "stop climb").
        const int cur_ft = static_cast<int>(ctx.pressure_alt_ft);
        const int verb_diff = step_target_ft - cur_ft;
        const char *verb =
            (verb_diff > 200) ? "climb" : (verb_diff < -200) ? "descend"
                                                             : "maintain";
        if (out_text) {
          char buf[128];
          const int ta = ctx.transition_alt_ft > 0 ? ctx.transition_alt_ft : 5000;
          const int tl = compute_tl_ft(ta, ctx.qnh_hpa);
          const bool below_tl = s_enroute_cleared_alt_ft < tl;
          if (below_tl) {
            std::snprintf(buf, sizeof(buf), "%s, %s %d feet, QNH %d.",
                          callsign.c_str(), verb, s_enroute_cleared_alt_ft,
                          ctx.qnh_hpa);
            s_qnh_stated = true;
          } else {
            std::snprintf(buf, sizeof(buf), "%s, %s flight level %d.",
                          callsign.c_str(), verb, step.cruise_fl);
          }
          *out_text = buf;
        }
        {
          const int ta = ctx.transition_alt_ft > 0 ? ctx.transition_alt_ft : 5000;
          const int tl = compute_tl_ft(ta, ctx.qnh_hpa);
          logging::info(
              "IFR en-route: filed step %s FL%d (fix %s, dist %.0f NM, cur FL%d, TL=%d)",
              verb, step.cruise_fl, step.ident.c_str(), dist_nm, cur_ft / 100,
              tl / 100);
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
      int fix_idx = -1;
      if (!fix.empty())
        for (int i = std::max(0, s_route_fix_idx);
             i < static_cast<int>(s_route_fixes.size()); ++i)
          if (s_route_fixes[i].ident == fix) { fix_idx = i; break; }
      // Only issue a direct that MATERIALLY shortens the route: it must save >= 5%
      // of the remaining leg-by-leg distance, and then only 20% of the time (ATC
      // variability -- most of the time the aircraft just flies the full route).
      // savings = (leg-by-leg aircraft->...->fix) - (direct aircraft->fix); the
      // legs after `fix` are unchanged so they cancel (user 2026-07-22).
      bool worth_it = false;
      if (fix_idx >= 0 && (s_route_fixes[fix_idx].lat != 0.0 ||
                           s_route_fixes[fix_idx].lon != 0.0)) {
        const double routed_to_fix = routed_distance_to_fix_idx(ctx, fix_idx);
        const double direct_to_fix = traffic_geometry::distance_nm(
            ctx.latitude, ctx.longitude, s_route_fixes[fix_idx].lat,
            s_route_fixes[fix_idx].lon);
        const double total = routed_distance_to_fix_idx(
            ctx, static_cast<int>(s_route_fixes.size()) - 1);
        worth_it = total > 1.0 && (routed_to_fix - direct_to_fix) >= 0.05 * total;
      }
      if (worth_it && (std::rand() % 5) == 0) {
        // Jump the route tracker to the direct-to fix so routed_distance_to_fix_idx
        // (TOD / ETA / handoff gates) shortens to the direct leg + remaining legs.
        s_route_fix_idx = fix_idx;
        // A direct-to supersedes any outstanding clearance readback (e.g. a descent
        // clearance with the runway field still pending) -- cancel it so the pilot
        // isn't stuck reading back "runway 07" for a "direct DJL, when able".
        atc_state_machine::cancel_readback();
        if (out_text) {
          char buf[128];
          std::snprintf(buf, sizeof(buf), "%s, direct %s, when able.",
                        callsign.c_str(), fix.c_str());
          *out_text = buf;
        }
        logging::info("IFR en-route: direct %s shortcut (>=5%% saved, 20%% roll)",
                      fix.c_str());
        return true;
      }
    }
    // No navlog / no fix / not worth it / 80% no-direct -- mark issued, don't retry.
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
      // Too far from the TOD -> DENY and hold cruise (user 2026-07-30: > ~5 min from
      // the routed TOD). A planned navlog step (matched above) is exempt -- it is a
      // filed cruise step, not an early arrival descent. Uses the routed TOD estimate
      // (s_tod_dist_nm/alert, refreshed each frame below); if not yet known, allow.
      if (!issued_step) {
        float min_to_tod = -1.0f;
        if (s_tod_dist_nm > 0.0f && s_tod_alert_nm > 0.0f &&
            ctx.groundspeed_kts > 40.0f)
          min_to_tod =
              (s_tod_dist_nm - s_tod_alert_nm) / ctx.groundspeed_kts * 60.0f;
        if (min_to_tod > 5.0f) {
          const int cur = s_enroute_cleared_alt_ft > 0 ? s_enroute_cleared_alt_ft
                                                       : ctx.ifr_cruise_alt_ft;
          if (out_text) {
            char buf[144];
            std::snprintf(
                buf, sizeof(buf),
                "%s, maintain flight level %d, expect descent in %d minutes.",
                callsign.c_str(), round_to_fl(cur),
                static_cast<int>(std::lround(min_to_tod)));
            *out_text = buf;
          }
          logging::info("IFR en-route: descent request DENIED -- %.0f min from TOD, "
                        "maintain cruise",
                        static_cast<double>(min_to_tod));
          rb(false);
          return true;
        }
      }

      // Near the TOD (or unknown), ATC gives the descent clearance -- it decides the
      // FL, the pilot does not request a level. build_descent_clearance issues the
      // arrival descent + STAR + expected approach.
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
        // alert = NM to lose the altitude at the shared descent slope
        // (kDescentSlopeFtPerNm = 265 ft/NM = 2.5 deg, matching the crossing/navlog-step
        // calcs and a typical VNAV) + clearance-exchange buffer (gs/20). PROPORTIONAL to
        // the altitude to lose: the cap is a sanity limit (160 NM), NOT a value that cuts
        // a high-cruise descent -- the old 80 NM cap forced FL450->FL150 (~113 NM) into a
        // ~3.5 deg steep descent (user 2026-07-31).
        float alt_component = static_cast<float>(std::max(0, cruise_ref - descent_target)) /
                              static_cast<float>(kDescentSlopeFtPerNm);
        tod_alt_component = alt_component;
        float spd_component = gs / 20.0f;
        alert_nm = std::max(15.0f, std::min(160.0f, alt_component + spd_component));

        if (se_valid && !se.star_name.empty()) {
          // ROUTED (fix-by-fix) distance to the STAR entry -- sums every leg so a
          // dogleg before the entry is counted, unlike great-circle which
          // underestimates and fires the TOD late/steep. Reuses the SALEV3P
          // routed helper; the STAR entry is a filed navlog fix so it is already in
          // s_route_fixes en route. Falls back to great-circle if not found.
          // (user 2026-07-30)
          int se_idx = -1;
          for (int i = 0; i < static_cast<int>(s_route_fixes.size()); ++i)
            if (s_route_fixes[i].ident == se.ident) { se_idx = i; break; }
          dist_nm = (se_idx >= 0)
                        ? routed_distance_to_fix_idx(ctx, se_idx)
                        : traffic_geometry::distance_nm(ctx.latitude,
                                                        ctx.longitude, se.lat,
                                                        se.lon);
        } else {
          // No CIFP match — measure to destination so prompt fires at correct time.
          const auto &dest_fix = ofp.navlog.back();
          dist_nm = traffic_geometry::distance_nm(
              ctx.latitude, ctx.longitude, dest_fix.lat, dest_fix.lon);
        }
        // Expose the TOD estimate to the IFR tab (routed dist + alert distance).
        s_tod_dist_nm = static_cast<float>(dist_nm);
        s_tod_alert_nm = alert_nm;
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
    // Suppress while the aircraft is CLOSING on the assigned level -- i.e. a normal
    // climb/descent toward it (below + climbing, or above + descending). Without this
    // guard a routine climb to the cruise FL (e.g. through FL163 toward a just-cleared
    // FL190) fired a false "check altitude, 3243 ft below FL190" (LFLP 2026-07-25).
    // The warning still fires when the aircraft is LEVEL but off (|VS| small -- a
    // wrong-FL level-off) or moving AWAY from the assigned level (a runaway).
    const float vs = ctx.vertical_speed_fpm;
    const bool closing = (deviation_ft < 0 && vs > 200.0f) ||
                         (deviation_ft > 0 && vs < -200.0f);
    if (!closing && std::abs(deviation_ft) >= threshold_ft) {
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
  // (on_destination_terminal is false there).
  // NOTE (build 165): on_destination_terminal() returns its permissive "can't tell
  // -> true" whenever the aircraft is ABOVE every openair sector (acft_tma empty).
  // At enroute high altitude that WRONGLY DEFERRED this whole sector-change -- incl.
  // the atc.dat FIR/CTR fallback below -- so no enroute handoff fired until the
  // aircraft descended into an openair volume: Milan->Vienna (LOWI arrival) never
  // fired at ~FL270 and only Glockner (openair CTA) took over at FL245; same cause
  // as the pre-overlay Slovenia miss at KUBUD FL397 (real vols 2026-07-31). Only
  // honour the dest-terminal defer when the aircraft is ACTUALLY inside an openair
  // volume -- otherwise it is enroute (above all TMAs) and the fallback must run.
  // [C. P. Potter]
  bool acft_in_openair_vol = false;
  if (openair_db::ready()) {
    const auto acft_enc = openair_db::find_enclosing(
        ctx.latitude, ctx.longitude, openair_alt(ctx));
    acft_in_openair_vol = !acft_enc.name.empty();
  }
  if (!s_assigned_dest_icao.empty() && openair_db::ready() &&
      acft_in_openair_vol && on_destination_terminal(ctx))
    return false;
  // Latch: once the destination APPROACH handoff has fired, the aircraft is
  // committed to the approach controller -- never hand it back to an ACC/enroute
  // sector, even if an RNP approach momentarily takes it OUT of the narrow
  // destination TMA corridor into an overlying ACC sector. At LOWI the RTT->ELMEM
  // leg re-crosses Vienna's CTA C, which without this latch flickers "Innsbruck
  // Approach <-> Vienna". Enroute/descent sector changes (Milan->Ljubljana->Vienna)
  // are unaffected: the flag is false until the terminal handoff. (user 2026-07-30)
  // [C. P. Potter]
  if (s_enroute_approach_handoff_issued)
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
    logging::info("[acc] atc.dat fallback resolved %s (%s) %.3f MHz "
                  "(openair empty at %.4f,%.4f %dft)",
                  new_label.c_str(), best->facility_id.c_str(), new_mhz,
                  ctx.latitude, ctx.longitude, openair_alt(ctx));
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
  // Suppressed while the aircraft is in a published hold: the pilot is flying the
  // pattern (altitude excursions in the turns are normal, and the hold altitude may
  // differ from the descent-profile target), NOT tracking a descent -- so a "confirm
  // climbing/descending <level>" nag here is spurious (user 2026-07-31). Resumes on
  // release (s_hold_state != 1).
  if (s_hold_state == 1) {
    s_alt_comp_arm_sec = 0.0f; // re-arm the grace timer for after the hold
    s_alt_comp_sent = false;
    return false;
  }
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

// Descend-to-enter-terminal-area: when laterally over a terminal TMA but above its
// ceiling, step the aircraft down to the highest full-thousand FL strictly below
// the ceiling. Respects STAR block floors (clamp UP to an active floor; if that
// reaches/exceeds the ceiling the aircraft cannot enter the TMA yet, so the
// target_ft < tma_ceil guard suppresses the clearance until the block fix is
// behind -- Caveat B: respect the fix minimum, defer). Runs in BOTH descent AND
// arrival so the aircraft is stepped INTO the dest terminal TMA before the approach
// handoff, not handed off while still above the ceiling (user 2026-07-22, LFLP->
// LFMD: at FL120 over the FL115 NICE TMA it was handed to Nice while above the
// ceiling because this ran only in IFR_DESCENT).
static bool poll_descend_to_enter_tma(const xplane_context::XPlaneContext &ctx,
                                      std::string *out_text,
                                      bool *out_requires_readback) {
  if (!out_text || !openair_db::ready())
    return false;
  // Only descend INTO the DESTINATION's terminal area. Over an ENROUTE TMA
  // (LOWI/DOLSKO tops FL245 while transiting at FL450) this must NOT fire a
  // "descend into it" clearance -- the stepped descent keeps the aircraft ABOVE
  // the enroute TMA. Geometry-based dest check (works for a field under a
  // differently-named TMA). [[project_stepped_descent]]
  if (!s_assigned_dest_icao.empty() && !on_destination_terminal(ctx))
    return false;
  const int tma_ceil =
      openair_db::terminal_tma_ceiling(ctx.latitude, ctx.longitude);
  int target_ft = (tma_ceil > 1000) ? ((tma_ceil - 100) / 1000) * 1000 : 0;
  const int block_floor = active_block_floor_ft();
  if (block_floor > 0 && target_ft < block_floor)
    target_ft = block_floor; // Caveat A/B: never below an active block floor
  const int cleared = engine::current_cleared_alt_ft();
  const bool fire = target_ft > 0 && target_ft < tma_ceil &&
                    static_cast<int>(ctx.altitude_ft_msl) > tma_ceil + 200 &&
                    cleared > target_ft + 100 &&
                    s_descent_tma_target_ft != target_ft;
  if (tma_ceil > 0 && settings::debug_logging())
    logging::info("[dbg dte] tma_ceil=%d target=%d alt=%.0f cleared=%d floor=%d "
                  "last=%d -> %s",
                  tma_ceil, target_ft, static_cast<double>(ctx.altitude_ft_msl),
                  cleared, block_floor, s_descent_tma_target_ft,
                  fire ? "FIRE" : "hold");
  if (!fire)
    return false;
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
  logging::info("IFR descent: descend-to-enter terminal area, TMA ceil %d -> %s",
                tma_ceil, clr.c_str());
  return true;
}

// GENERAL RULE for ANY ATC direct-to: every route fix BETWEEN the aircraft and the
// direct target is SKIPPED. Advance the route tracker to that fix so the intermediate
// fixes sit BEHIND the index (removed from ATC tracking -- the DirectMonitor, the
// ARRIVAL->APPROACH eta gate and routed_distance_to_fix_idx all measure to the target,
// not a skipped fix), and cancel any pending readback (a direct supersedes it). The
// en-route direct shortcut does the same inline (~6837). Shared so every direct-to
// (en-route / connector IAF / no-STAR IAF) follows the rule identically. (user
// 2026-07-31) [C. P. Potter]
static void apply_direct_to(const std::string &fix_ident) {
  for (int i = std::max(0, s_route_fix_idx);
       i < static_cast<int>(s_route_fixes.size()); ++i)
    if (s_route_fixes[i].ident == fix_ident) {
      s_route_fix_idx = i;
      break;
    }
  // Post-direct settle: hold the DirectMonitor off for kDirectSettleSecs so the pilot has
  // time to TURN onto the new leg before ATC flags "confirm direct <fix>, you appear
  // tracking ...". [C. P. Potter]
  s_enroute_course_cooldown  = kDirectSettleSecs;
  s_approach_course_cooldown = kDirectSettleSecs;
  atc_state_machine::cancel_readback();
}

// GENERIC (not LOWI-specific): when the arrival uses a CONNECTOR STAR that is not in
// the pilot's FMS -- the filed STAR terminates at a fix that is not an IAF of the
// selected approach, and a linking STAR bridges it (resolve_approach_iaf's
// out_connector, e.g. LOWI NANI2A->RTT + RTT1B: RTT->ELMEM) -- the FMS has a
// discontinuity at the filed-STAR terminus. ATC issues an explicit "direct <IAF>"
// as the aircraft reaches that terminus, so the pilot is guided across the gap
// rather than left to deduce it. This is EARLY and SEPARATE from the approach
// clearance (which still fires near the IAF via the eta gate). One-shot. Any airport
// with the same STAR/approach gap benefits -- no per-field exception. [C. P. Potter]
static bool poll_connector_direct(const xplane_context::XPlaneContext &ctx,
                                  std::string *out_text, bool *out_rb) {
  if (s_connector_direct_issued || ctx.cifp_dir.empty() ||
      s_assigned_star_name.empty() || s_assigned_approach_designator.empty() ||
      s_assigned_dest_icao.empty())
    return false;
  std::string connector;
  const std::string iaf = resolve_approach_iaf(
      ctx, s_assigned_star_name, s_assigned_approach_designator, &connector);
  if (connector.empty() || iaf.empty())
    return false; // no connector -> normal arrival, nothing to guide
  // Connector-start = the terminating fix of the FILED STAR (RTT for NANI2A).
  const std::string start = cifp_reader::star_last_fix(ctx.cifp_dir,
                                                       s_assigned_dest_icao,
                                                       s_assigned_star_name);
  if (start.empty())
    return false;
  int start_idx = -1;
  for (int i = 0; i < static_cast<int>(s_route_fixes.size()); ++i)
    if (s_route_fixes[i].ident == start) { start_idx = i; break; }
  if (start_idx < 0 || s_route_fix_idx < start_idx)
    return false; // tracker not yet at the filed-STAR terminus
  s_connector_direct_issued = true;
  // Direct-to rule: skip the intermediate fixes (RTT) -- advance the tracker to the
  // IAF so the DirectMonitor and the ARRIVAL->APPROACH eta gate measure to ELMEM, not
  // stall at RTT (which looped the approach handoff, stuck at cruise -- LOWI
  // 2026-07-31, the connector STAR RTT1B was never reflected in the tracker).
  apply_direct_to(iaf);
  if (out_text) {
    const std::string &cs_ref = atc_state_machine::session_callsign();
    const std::string &cs = cs_ref.empty() ? settings::pilot_callsign() : cs_ref;
    *out_text = cs + ", direct " + iaf + ".";
  }
  if (out_rb)
    *out_rb = false;
  logging::info("IFR arrival: connector direct -> direct %s (filed-STAR end %s, "
                "connector %s)",
                iaf.c_str(), start.c_str(), connector.c_str());
  return true;
}

// --- Vector-to-intercept (radar vectors to final) --------------------------
static double vec_norm360(double d) { while (d < 0.0) d += 360.0; while (d >= 360.0) d -= 360.0; return d; }
static double vec_norm180(double d) { while (d < -180.0) d += 360.0; while (d > 180.0) d -= 360.0; return d; }

// Bypass-IAF radar vectoring to the final approach course. Called from poll_arrival
// AND poll_approach; one-shot per arrival (s_vec_done). CONTAINED by a large-turn
// gate (kVectorMinTurnDeg): an arrival whose STAR already lines up with final turns
// less than the gate -> immediate no-op, nothing changes (LFLP/LFMN/SALEV3P stay on
// the normal cleared-at-IAF path). Only a genuine reversal (LOWI R08-Z: direct ELMEM
// heading ~270 -> final 082) arms it. Issues up to 3 SEPARATE vectors (established-
// on-previous OR timer cadence); the last carries "maintain <alt> until established
// ... cleared <appr>". From the first vector it latches s_approach_cleared_issued so
// the normal cleared-at-IAF gate never double-issues mid-sequence. [C. P. Potter]
static constexpr double kVectorArmNm      = 2.5;   // arm once the IAF is this near (routed)
static constexpr double kVectorMinTurnDeg = 100.0; // only a reversal; smaller -> own-nav
static constexpr double kVectorOpenDeg    = 45.0;  // opening turn off the arrival heading
static constexpr double kVectorOutboundNm = 5.0;   // fly this far outbound before reversing
static constexpr float  kVectorMaxSecs    = 20.0f; // timer cap (reverse1 -> reverse2)
static constexpr double kVectorEstabDeg   = 6.0;   // "established" tolerance on the heading
// Vector-compliance monitor: the pilot is "following" the assigned vector within this
// heading error; if not, NUDGE at kVectorNudgeSecs, then ADAPT (disregard + clear the
// approach) at kVectorAdaptSecs -- an RNAV approach flown in LNAV ignores the vector and
// flies the coded procedure toward the fix, so ATC must notice and clear the approach
// (user 2026-08-02). Getting back within kVectorFollowDeg resets the timer -> a brief
// deviation is forgiven. [C. P. Potter]
static constexpr double kVectorFollowDeg  = 20.0;
static constexpr float  kVectorNudgeSecs  = 12.0f;
static constexpr float  kVectorAdaptSecs  = 30.0f;

// Does the assigned approach need a REVERSAL vector at the IAF? True when the arrival
// heading is ~opposite the IAF's published course (IAF -> next fix) -- the SAME large-turn
// test the teardrop vector uses (kVectorMinTurnDeg). Used to (a) DEFER the straight
// eta-based approach clearance so the vector (poll_vector) owns it -- otherwise the
// straight clearance fires at ~7 NM and advances the tracker PAST the IAF, killing the
// vector (real vol LOWI R08-Z 2026-08-02); and (b) drive the early "expect vectors"
// heads-up. IAF-CENTRED (reversal to rejoin at the IAF); the FAF "vectors to final" case
// is separate/more usual. Self-contained (inline normalisation). [C. P. Potter]
static bool approach_needs_reversal_vector(const xplane_context::XPlaneContext &ctx) {
  if (s_vec_done) return false; // vectoring already resolved/skipped this arrival
  if (s_assigned_approach_designator.empty() || s_route_fixes.empty()) return false;
  std::string sel_iaf = s_no_star_direct_iaf;
  if (sel_iaf.empty())
    sel_iaf = resolve_approach_iaf(ctx, s_assigned_star_name,
                                   s_assigned_approach_designator);
  if (sel_iaf.empty()) return false;
  int iaf_idx = -1;
  for (int i = 0; i < static_cast<int>(s_route_fixes.size()); ++i)
    if (s_route_fixes[i].ident == sel_iaf) { iaf_idx = i; break; }
  if (iaf_idx < 0) return false;
  const RouteFix &iaf_fix = s_route_fixes[iaf_idx];
  const RouteFix *nxt = nullptr;
  for (int i = iaf_idx + 1; i < static_cast<int>(s_route_fixes.size()); ++i)
    if (s_route_fixes[i].lat != 0.0 || s_route_fixes[i].lon != 0.0) {
      nxt = &s_route_fixes[i];
      break;
    }
  if (nxt == nullptr || (iaf_fix.lat == 0.0 && iaf_fix.lon == 0.0)) return false;
  const double course_true =
      traffic_geometry::bearing_deg(iaf_fix.lat, iaf_fix.lon, nxt->lat, nxt->lon);
  const double magvar =
      static_cast<double>(ctx.heading_true) - static_cast<double>(ctx.heading_mag);
  double inbound = std::fmod(course_true - magvar + 360.0, 360.0);
  double turn = std::fabs(inbound - static_cast<double>(ctx.heading_mag));
  if (turn > 180.0) turn = 360.0 - turn;
  return turn >= kVectorMinTurnDeg;
}

static bool poll_vector_to_intercept(const xplane_context::XPlaneContext &ctx, float dt,
                                     std::string *out_text, bool *out_rb) {
  // Diagnostic: log WHY the vector-to-intercept declines to arm, once per changed
  // reason (never armed on the LOWI R08-Z ELMEM reversal in-sim -- real vol
  // 2026-07-31/08-01 -- and poll_vector logged nothing, so the bail cause was
  // invisible). Throttled by reason string so it never spams. [C. P. Potter]
  static std::string s_vec_diag;
  auto bail = [&](const std::string &why) {
    if (s_vec_diag != why) {
      s_vec_diag = why;
      logging::info("IFR vector: not arming -- %s", why.c_str());
    }
    return false;
  };

  if (s_vec_done || out_text == nullptr) return false;
  if (s_assigned_dest_icao.empty() || s_assigned_approach_designator.empty() ||
      s_approach_faf.ident.empty() || s_route_fixes.empty())
    return bail(std::string("precondition empty:") +
                (s_assigned_dest_icao.empty() ? " dest" : "") +
                (s_assigned_approach_designator.empty() ? " designator" : "") +
                (s_approach_faf.ident.empty() ? " faf" : "") +
                (s_route_fixes.empty() ? " route" : ""));

  const double cur_mag = static_cast<double>(ctx.heading_mag);

  if (s_vec_step < 0) {
    // Locate the selected IAF; start once within kVectorStartNm (routed distance).
    std::string sel_iaf = s_no_star_direct_iaf;
    if (sel_iaf.empty())
      sel_iaf = resolve_approach_iaf(ctx, s_assigned_star_name,
                                     s_assigned_approach_designator);
    if (sel_iaf.empty()) return bail("no IAF resolved");
    int iaf_idx = -1;
    for (int i = std::max(0, s_route_fix_idx);
         i < static_cast<int>(s_route_fixes.size()); ++i)
      if (s_route_fixes[i].ident == sel_iaf) { iaf_idx = i; break; }
    if (iaf_idx < 0)
      return bail("IAF " + sel_iaf + " not ahead of tracker (idx " +
                  std::to_string(s_route_fix_idx) + ")");
    const double iaf_d = routed_distance_to_fix_idx(ctx, iaf_idx);
    if (iaf_d > kVectorArmNm)
      return bail("IAF " + sel_iaf + " far (" + std::to_string((int)iaf_d) + " NM)");

    // Reference course = the PUBLISHED HEADING OF THE APPROACH AT THE IAF = the true
    // bearing from the IAF to the NEXT approach fix (the first leg out of the IAF),
    // converted to MAGNETIC with the live magvar (heading_true-heading_mag). This is
    // the reference for BOTH the reversal decision AND the intercept target -- the
    // aircraft is vectored to rejoin the published path ALIGNED at the IAF. Correct
    // for an RNP approach with curved RF legs: we never intercept a curved final, only
    // the IAF entry heading (user 2026-07-31: "on arrive a l'inverse du heading publie
    // de la RNAV a l'IAF"). LOWI R08-Z: ELMEM->WI749 ~087; arrival via RTT1B ~260 ->
    // ~173 deg reversal -> vector; a runway-08 STAR (XEBI1B) reaches ELMEM ~aligned ->
    // no reversal -> no vectoring (fly the procedure).
    const RouteFix &iaf_fix = s_route_fixes[iaf_idx];
    const RouteFix *nxt = nullptr;
    for (int i = iaf_idx + 1; i < static_cast<int>(s_route_fixes.size()); ++i)
      if (s_route_fixes[i].lat != 0.0 || s_route_fixes[i].lon != 0.0) {
        nxt = &s_route_fixes[i];
        break;
      }
    if (nxt == nullptr || (iaf_fix.lat == 0.0 && iaf_fix.lon == 0.0)) {
      s_vec_done = true; // no next-fix geometry -> cannot derive the IAF course
      bail("no next-fix geometry after IAF " + sel_iaf);
      return false;
    }
    const double course_true =
        traffic_geometry::bearing_deg(iaf_fix.lat, iaf_fix.lon, nxt->lat, nxt->lon);
    const double magvar = static_cast<double>(ctx.heading_true) -
                          static_cast<double>(ctx.heading_mag);
    const double inbound = vec_norm360(course_true - magvar); // published course, magnetic

    // Reversal only: vector when the arrival heading is ~opposite the IAF's published
    // course; an aligned arrival just flies the procedure.
    const double turn = std::fabs(vec_norm180(inbound - cur_mag));
    if (turn < kVectorMinTurnDeg) {
      s_vec_done = true;
      bail("turn " + std::to_string((int)turn) + " deg < " +
           std::to_string((int)kVectorMinTurnDeg) + " (inbound " +
           std::to_string((int)inbound) + " vs hdg " +
           std::to_string((int)cur_mag) + ")");
      return false;
    }

    // TEARDROP reversal (user 2026-08-01 "option B"; 2026-08-02 refinement): OPEN ~45
    // deg off the APPROACH AXIS -- the first segment OUT OF THE IAF (ELMEM->WI749 for
    // the LOWI RNAV 08), i.e. the reciprocal of `inbound` -- NOT off the arrival heading.
    // This makes the teardrop width defined by the PROCEDURE and independent of how the
    // aircraft happens to arrive. The old cur_mag reference gave a too-tight open (247)
    // when the arrival heading already sat near the reciprocal; axis-referenced it opens
    // the proper 216 -- "plus a gauche" (user 2026-08-02: "sur l'axe d'approche du 1er
    // segment partant de ELMEM pour la RNAV 08"). Open to the side of the reciprocal
    // OPPOSITE the current heading so the outbound leg crosses the extended centreline
    // (a true teardrop), then reverse the OTHER way, the last vector a heading DIRECT to
    // the IAF so the aircraft re-crosses it aligned to fly the approach. [C. P. Potter]
    const double inbound_recip = vec_norm360(inbound + 180.0);
    const int open_side =
        (vec_norm180(cur_mag - inbound_recip) >= 0.0) ? -1 : +1; // -1 left, +1 right
    const double open_hdg = vec_norm360(inbound_recip + kVectorOpenDeg * open_side);
    const double rev1 = vec_norm360(open_hdg + 90.0 * (-open_side));

    s_vec_plan = {};
    s_vec_plan.needed = true;
    s_vec_plan.open_left = (open_side < 0);
    s_vec_plan.turn_left = (-open_side < 0); // reverse direction = opposite the open
    s_vec_plan.iaf_lat = iaf_fix.lat;
    s_vec_plan.iaf_lon = iaf_fix.lon;
    s_vec_plan.inbound_mag = inbound; // final approach course -> reverse2 intercepts it
    s_vec_plan.vectors = {static_cast<int>(std::lround(open_hdg)),
                          static_cast<int>(std::lround(rev1))};
    // Maintain/descend altitude = the IAF platform constraint (LOWI ELMEM +13000 QNH;
    // for a BLOCK the ceiling alt.feet = minimum-compliant descent). FAF alt fallback.
    if (iaf_fix.alt.feet > 0) {
      s_vec_plan.maintain_ft    = iaf_fix.alt.feet;
      s_vec_plan.maintain_is_fl = iaf_fix.alt.is_fl;
    } else if (s_approach_faf.alt_ft > 0) {
      s_vec_plan.maintain_ft    = s_approach_faf.alt_ft;
      s_vec_plan.maintain_is_fl = false;
    }
    s_vec_step = 0;
    s_vec_timer = 0.0f;
    logging::info("IFR arrival: vector-to-intercept armed (teardrop) -- open %s %03d, "
                  "outbound %.0f NM, reverse %s -> intercept inbound %03d (via %s)",
                  s_vec_plan.open_left ? "left" : "right",
                  static_cast<int>(std::lround(open_hdg)), kVectorOutboundNm,
                  s_vec_plan.turn_left ? "left" : "right",
                  static_cast<int>(std::lround(inbound)), sel_iaf.c_str());
  }

  // Cadence gates. step 0 = OPEN (fires at the IAF); step 1 = reverse1 (after the fixed
  // OUTBOUND distance -- speed-independent); step 2 = reverse2 = DIRECT to the IAF +
  // cleared approach (once established on reverse1, or the timer cap).
  s_vec_timer += dt;
  const double iaf_dist_nm = traffic_geometry::distance_nm(
      ctx.latitude, ctx.longitude, s_vec_plan.iaf_lat, s_vec_plan.iaf_lon);

  // ── Vector-compliance monitor (user 2026-08-02, option 1: generic heading detection
  //    via heading_error_deg + vector-specific reaction) ─────────────────────────────
  // The pilot may not follow the assigned vector -- an RNAV approach flown in LNAV
  // sequences ELMEM->WI749 and turns toward the FIX, not the ATC heading (real vol LOWI
  // R08-Z: told "turn left 216" at hdg 311, the aircraft turned RIGHT to 001). Detect the
  // sustained non-compliance; NUDGE at T+kVectorNudgeSecs, then ADAPT at kVectorAdaptSecs
  // -- disregard vectors + clear the approach so the FMS flies the coded procedure.
  // Getting back within kVectorFollowDeg resets the timer (a brief deviation is forgiven).
  // [C. P. Potter]
  if (s_vec_assigned_hdg >= 0.0) {
    s_vec_follow_timer += dt;
    const double herr =
        heading_error_deg(static_cast<double>(cur_mag), s_vec_assigned_hdg);
    if (herr <= kVectorFollowDeg) {
      s_vec_follow_timer = 0.0f; // converging on the assigned heading -> compliant
      s_vec_nudged = false;
    } else if (s_vec_follow_timer >= kVectorAdaptSecs) {
      const std::string &cs_a = atc_state_machine::session_callsign();
      const std::string &csa = cs_a.empty() ? settings::pilot_callsign() : cs_a;
      const std::string phrase = approach_clearance_phrase(ctx);
      char vb[224];
      std::snprintf(vb, sizeof(vb),
                    "%s, disregard vectors, cleared %s, fly the published procedure.",
                    csa.c_str(), phrase.empty() ? "the approach" : phrase.c_str());
      *out_text = vb;
      s_approach_cleared_issued = true;
      s_approach_final_issued   = true; // open the FAF/Tower handoff gate
      s_vec_done = true;
      s_vec_step = -1;
      s_vec_assigned_hdg = -1.0;
      logging::info("IFR vector: NOT followed (hdg err %.0f deg) -> disregard + cleared %s",
                    herr, phrase.empty() ? "approach" : phrase.c_str());
      if (out_rb) *out_rb = true;
      return true;
    } else if (!s_vec_nudged && s_vec_follow_timer >= kVectorNudgeSecs) {
      s_vec_nudged = true;
      const std::string &cs_n = atc_state_machine::session_callsign();
      const std::string &csn = cs_n.empty() ? settings::pilot_callsign() : cs_n;
      char vb[160];
      std::snprintf(vb, sizeof(vb), "%s, confirm turning %s heading %03d.",
                    csn.c_str(), s_vec_assigned_left ? "left" : "right",
                    static_cast<int>(std::lround(s_vec_assigned_hdg)));
      *out_text = vb;
      logging::info("IFR vector: not followed (hdg err %.0f, T+%.0fs) -> nudge", herr,
                    static_cast<double>(s_vec_follow_timer));
      if (out_rb) *out_rb = false;
      return true;
    }
  }

  if (s_vec_step == 1) {
    if (iaf_dist_nm < kVectorOutboundNm) return false; // still flying outbound
  } else if (s_vec_step == 2) {
    const double off = std::fabs(vec_norm180(cur_mag - s_vec_plan.vectors[1]));
    if (off > kVectorEstabDeg && s_vec_timer < kVectorMaxSecs) return false;
  }

  // From the first vector on, the vectoring OWNS the approach clearance: latch the
  // gate flag so poll_approach's cleared-at-IAF never double-issues mid-sequence.
  s_approach_cleared_issued = true;

  const bool last = (s_vec_step == 2);
  const std::string &cs_ref = atc_state_machine::session_callsign();
  const std::string &cs = cs_ref.empty() ? settings::pilot_callsign() : cs_ref;
  const double magvar2 = static_cast<double>(ctx.heading_true) -
                         static_cast<double>(ctx.heading_mag);
  int h;
  const char *lr;
  if (s_vec_step == 0) {
    h = s_vec_plan.vectors[0]; // OPEN
    lr = s_vec_plan.open_left ? "left" : "right";
  } else if (s_vec_step == 1) {
    h = s_vec_plan.vectors[1]; // reverse1
    lr = s_vec_plan.turn_left ? "left" : "right";
  } else { // reverse2 = INTERCEPT the inbound (final approach) course, not direct-to-fix
    // Anchor the last vector on the INBOUND axis (ELMEM->WI749, magnetic), so the
    // aircraft crosses the IAF ALIGNED to fly the approach -- a direct-to-fix heading
    // pointed AT the IAF and crossed it at the teardrop angle, ~30-45 deg off the
    // course (user 2026-08-02: "le dernier vecteur n'etait pas exact ... reverse sur
    // le cap de l'avion"). Standard radar intercept: turn onto inbound +/- ~30 deg
    // toward the side the aircraft sits on, collapsing to the inbound course once
    // within ~0.5 NM cross-track of the extended final course line. [C. P. Potter]
    const double inbound_mag = s_vec_plan.inbound_mag;
    const double inbound_true = vec_norm360(inbound_mag + magvar2);
    // Signed cross-track (NM) of the aircraft from the final course line through the
    // IAF: >0 = right of the inbound direction (turn onto inbound - intercept), <0 =
    // left (inbound + intercept).
    const double brg_true = traffic_geometry::bearing_deg(
        s_vec_plan.iaf_lat, s_vec_plan.iaf_lon, ctx.latitude, ctx.longitude);
    const double theta = vec_norm180(brg_true - inbound_true) * M_PI / 180.0;
    const double xte = std::sin(theta) * iaf_dist_nm;
    const double intercept = (std::fabs(xte) < 0.5) ? 0.0 : (xte >= 0.0 ? -30.0 : 30.0);
    h = static_cast<int>(std::lround(vec_norm360(inbound_mag + intercept)));
    // Announce the turn in the sense that actually reaches h from the current heading.
    lr = (vec_norm180(static_cast<double>(h) - cur_mag) >= 0.0) ? "right" : "left";
  }
  // Arm the vector-compliance monitor on this freshly-issued heading: the pilot has the
  // nudge/adapt cadence to start the turn before ATC notices non-compliance. [CPP]
  s_vec_assigned_hdg  = static_cast<double>(h);
  s_vec_assigned_left = (lr[0] == 'l');
  s_vec_follow_timer  = 0.0f;
  s_vec_nudged        = false;
  char buf[224];
  if (last) {
    const int maintain_ft = s_vec_plan.maintain_ft;
    const AltHint hint =
        s_vec_plan.maintain_is_fl ? AltHint::FlightLevel : AltHint::Feet;
    std::string alt_str =
        maintain_ft > 0 ? format_alt_clearance(maintain_ft, hint, ctx.qnh_hpa,
                                               ctx.transition_alt_ft)
                        : std::string("present altitude");
    // "descend to <alt>" when the aircraft is above the IAF platform (the normal
    // case on a vectored arrival), else "maintain <alt>" (user 2026-07-31).
    const char *alt_verb =
        (maintain_ft > 0 &&
         ctx.altitude_ft_msl > static_cast<float>(maintain_ft) + 200.0f)
            ? "descend to"
            : "maintain";
    const std::string phrase = approach_clearance_phrase(ctx);
    std::snprintf(buf, sizeof(buf),
                  "%s, turn %s heading %03d, %s %s until established on the "
                  "approach, cleared %s.",
                  cs.c_str(), lr, h, alt_verb, alt_str.c_str(),
                  phrase.empty() ? "the approach" : phrase.c_str());
    s_approach_final_issued = true; // open the FAF/Tower handoff gate
    // Keep the cleared-alt tracker in sync with "descend to <alt> until established"
    // (STT bias + altitude monitor + approach step-down all read it) -- the vector
    // final clearance previously set only s_approach_cleared_issued, leaving the
    // tracker at the stale enroute level. [C. P. Potter]
    if (maintain_ft > 0)
      s_enroute_cleared_alt_ft = maintain_ft;
    s_vec_done = true;
    s_vec_step = -1;
    logging::info("IFR arrival: vector-to-intercept -- final vector %03d + cleared "
                  "approach (%s %s)", h, alt_verb, alt_str.c_str());
  } else {
    std::snprintf(buf, sizeof(buf),
                  "%s, turn %s heading %03d, vectors for the approach.",
                  cs.c_str(), lr, h);
    ++s_vec_step;
  }
  s_vec_timer = 0.0f;
  *out_text = buf;
  if (out_rb) *out_rb = false;
  return true;
}

// Published hold at a STAR fix (random, once per arrival). See the static block near
// s_hold. Runs in poll_descent + poll_arrival. STAR fixes ONLY: the eligible fix must
// belong to the assigned STAR (star_waypoints) and not be an approach-proc fix. When
// holding, ticks the random EFC timer and clears the aircraft to continue. Altitude is
// clamped to the published [min,max] band. [C. P. Potter]
// Zulu time (sim sim/time/zulu_time_sec) -> HHMM spoken digit-by-digit, e.g. 14:25 ->
// "one four two five". For the holding-clearance EFC (real ATC gives a clock time, not a
// relative "in N minutes" -- user 2026-08-01). [C. P. Potter]
static std::string zulu_hhmm_spoken(float zulu_sec) {
  static const char *d[] = {"zero", "one", "two",   "three", "four",
                            "five", "six", "seven", "eight", "nine"};
  int total_min = static_cast<int>(zulu_sec / 60.0f);
  int hh = (total_min / 60) % 24;
  int mm = total_min % 60;
  const int dig[4] = {hh / 10, hh % 10, mm / 10, mm % 10};
  std::string s;
  for (int i = 0; i < 4; ++i) {
    if (i) s += " ";
    s += d[dig[i]];
  }
  return s;
}

static constexpr double kHoldArmNm     = 8.0;    // roll when within this (routed) of the fix
static constexpr int    kHoldChancePct = 100;    // probability %  (TEST=100; realistic ~35)
static constexpr float  kHoldMinSecs   = 180.0f; // 3 min (user 2026-08-01: +1 min)
static constexpr float  kHoldMaxSecs   = 420.0f; // 7 min
static bool poll_hold(const xplane_context::XPlaneContext &ctx, float dt,
                      std::string *out_text, bool *out_rb) {
  if (out_text == nullptr) return false;

  // HOLDING: wait out the random EFC, then clear the aircraft to continue.
  if (s_hold_state == 1) {
    s_hold_secs += dt;
    // Release when the sim ZULU clock reaches the announced EFC time -- NOT after N
    // dt-seconds (which drifts from the clock under sim time acceleration; real vol
    // 2026-08-01: EFC announced 16:45 but exit 16:48). Wrap-safe across midnight.
    // [C. P. Potter]
    float remaining = s_hold_efc_zulu - ctx.zulu_time_sec;
    if (remaining < -43200.0f) remaining += 86400.0f; // EFC just after midnight
    if (remaining > 0.0f) {
      // Still in the pattern: OWN the frame (empty text) so NO other clearance --
      // approach, vectoring, step-down -- fires while the aircraft is holding. The
      // caller returns true; the consumer skips the empty text (no message, no state
      // advance). poll_hold runs first in poll_descent/arrival/approach.
      out_text->clear();
      if (out_rb) *out_rb = false;
      return true;
    }
    s_hold_state = 2; // released -> one hold per arrival
    const std::string &cs_ref = atc_state_machine::session_callsign();
    const std::string &cs = cs_ref.empty() ? settings::pilot_callsign() : cs_ref;
    *out_text = cs + ", cleared to leave the hold, continue via the arrival.";
    if (out_rb) *out_rb = false;
    logging::info("IFR hold: released at %s after %.0f s", s_hold.fix.c_str(),
                  static_cast<double>(s_hold_secs));
    return true;
  }
  if (s_hold_state != 0) return false; // 2 = already held/decided this arrival

  if (s_assigned_dest_icao.empty() || s_assigned_star_name.empty() ||
      ctx.cifp_dir.empty() || s_route_fixes.empty())
    return false;

  // STAR fixes only: the eligible fix must belong to the assigned STAR.
  const auto star_wps = cifp_reader::star_waypoints(
      ctx.cifp_dir, s_assigned_dest_icao, s_assigned_star_name, /*constrained_only=*/false);
  if (star_wps.empty()) return false;
  std::unordered_set<std::string> star_idents;
  for (const auto &w : star_wps) star_idents.insert(w.ident);

  // First STAR fix ahead of the tracker, within the arm distance, that has a
  // published hold. One decision per arrival: roll at that fix and latch either way.
  for (int i = std::max(0, s_route_fix_idx);
       i < static_cast<int>(s_route_fixes.size()); ++i) {
    const RouteFix &f = s_route_fixes[i];
    if (f.is_approach_proc) continue;                             // out of STAR scope
    if (star_idents.find(f.ident) == star_idents.end()) continue; // not a STAR fix
    if (routed_distance_to_fix_idx(ctx, i) > kHoldArmNm)
      return false; // nearest STAR fix ahead not reached yet
    cifp_reader::HoldSpec h = cifp_reader::published_hold(
        ctx.cifp_dir, f.ident, static_cast<int>(ctx.altitude_ft_msl));
    if (!h.valid) continue; // no published hold at this fix -> consider the next one

    static bool s_hold_seeded = false;
    if (!s_hold_seeded) {
      std::srand(static_cast<unsigned>(std::time(nullptr)));
      s_hold_seeded = true;
    }
    // User switch (IFR tab): holds can be turned off entirely. Latch state 2
    // ("no hold this arrival") so the decision is made once, like a roll miss.
    // [C. P. Potter]
    if (!settings::hold_enabled()) {
      s_hold_state = 2;
      logging::info("IFR hold: disabled in Settings -- no hold this arrival (at %s)",
                    f.ident.c_str());
      return false;
    }
    const bool hit = (std::rand() % 100) < kHoldChancePct;
    s_hold_state = hit ? 1 : 2; // latch -> one decision per arrival
    if (!hit) {
      logging::info("IFR hold: no hold this arrival (roll miss at %s)", f.ident.c_str());
      return false;
    }
    s_hold = h;
    s_hold_secs = 0.0f;
    // EFC clock time = ETA at the fix + the in-hold duration. The clearance is issued
    // ~8 NM out, so add the time to REACH the fix (routed distance / groundspeed) to the
    // random hold time. s_hold_efc_secs (the release threshold, ticked from arm) covers
    // the SAME total, so the spoken clock time equals the release moment. [C. P. Potter]
    const float hold_dur =
        kHoldMinSecs +
        static_cast<float>(std::rand() %
                           static_cast<int>(kHoldMaxSecs - kHoldMinSecs + 1));
    const double gs_kt = std::max(60.0, static_cast<double>(ctx.groundspeed_kts));
    const float time_to_fix =
        static_cast<float>(routed_distance_to_fix_idx(ctx, i) / gs_kt * 3600.0);
    s_hold_efc_secs = time_to_fix + hold_dur;
    // Altitude = current cleared level, clamped to the published [min,max] band.
    int alt = s_enroute_cleared_alt_ft > 0 ? s_enroute_cleared_alt_ft
                                           : static_cast<int>(ctx.altitude_ft_msl);
    if (h.min_alt_ft > 0 && alt < h.min_alt_ft) alt = h.min_alt_ft;
    if (h.max_alt_ft > 0 && alt > h.max_alt_ft) alt = h.max_alt_ft;
    s_hold_alt_ft = alt;
    const std::string &cs_ref = atc_state_machine::session_callsign();
    const std::string &cs = cs_ref.empty() ? settings::pilot_callsign() : cs_ref;
    const std::string alt_str = format_alt_clearance(
        alt, AltHint::Auto, ctx.qnh_hpa, ctx.transition_alt_ft);
    s_hold_efc_zulu =
        std::fmod(ctx.zulu_time_sec + s_hold_efc_secs, 86400.0f); // release keys on this
    const std::string efc_clock = zulu_hhmm_spoken(s_hold_efc_zulu);
    char buf[224];
    std::snprintf(buf, sizeof(buf),
                  "%s, hold at %s as published, maintain %s, expect further clearance "
                  "at %s.",
                  cs.c_str(), f.ident.c_str(), alt_str.c_str(), efc_clock.c_str());
    *out_text = buf;
    if (out_rb) *out_rb = false;
    logging::info("IFR hold: HOLD at %s as published (inbound %03d, %s turns, %.1f min "
                  "legs), maintain %d ft, EFC at %s (fix ETA %.0fs + hold %.0fs)",
                  f.ident.c_str(), h.inbound_course_deg,
                  h.turn_right ? "right" : "left", h.leg_time_min, alt,
                  efc_clock.c_str(), static_cast<double>(time_to_fix),
                  static_cast<double>(hold_dur));
    return true;
  }
  return false;
}

// Second step of a stepped high-cruise descent. build_descent_clearance capped
// the first descent at FL200 and stashed the ultimate STAR-entry target in
// s_descent_final_target_ft. Once the aircraft nears FL200 (within ~2000 ft),
// clear it the rest of the way so it keeps descending instead of levelling off
// and waiting. One-shot, decoupled from any frequency change. Runs every frame.
static bool poll_descent_second_step(const xplane_context::XPlaneContext &ctx,
                                     std::string *out_text,
                                     bool *out_requires_readback) {
  if (s_descent_final_target_ft <= 0 || s_descent_second_step_issued)
    return false;
  // Fire only once the aircraft (a) has descended to near the first (TMA-top)
  // step AND (b) is clear of any tall TMA -- i.e. descending to the final target
  // will no longer dive into an overflown TMA. Over DOLSKO (top FL245) the
  // highest ceiling exceeds the final target -> hold at the first step; past it
  // the ceiling drops -> continue down. [[project_stepped_descent]]
  const int first_step = s_descent_first_step_ft > 0 ? s_descent_first_step_ft : 20000;
  if (static_cast<int>(ctx.altitude_ft_msl) > first_step + 1500)
    return false; // not yet down to the first step
  const int hi_ceil = openair_db::ready()
                          ? openair_db::highest_tma_ceiling(ctx.latitude,
                                                            ctx.longitude)
                          : 0;
  if (hi_ceil > s_descent_final_target_ft + 500 && !on_destination_terminal(ctx))
    return false; // still over a tall enroute TMA -- stay above it
  const int target = s_descent_final_target_ft;
  s_descent_second_step_issued = true;
  s_descent_final_target_ft = 0;      // consumed
  s_enroute_cleared_alt_ft = target;  // re-arms poll_altitude_compliance
  if (out_text) {
    const std::string &cs = atc_state_machine::session_callsign();
    const std::string &callsign = cs.empty() ? settings::pilot_callsign() : cs;
    const int ta = (ctx.transition_alt_ft > 0) ? ctx.transition_alt_ft : 5000;
    const std::string clr =
        format_alt_clearance(target, AltHint::Auto, ctx.qnh_hpa, ta);
    *out_text = callsign + ", descend " + clr + ".";
  }
  if (out_requires_readback)
    *out_requires_readback = true;
  logging::info("IFR descent: second step -> %s",
                format_alt(target, ctx.transition_alt_ft, ctx.qnh_hpa).c_str());
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

  // Second step of a stepped high-cruise descent (FL200 -> STAR entry). Fires
  // once the aircraft nears the FL200 first step, so it keeps descending.
  if (poll_descent_second_step(ctx, out_text, out_requires_readback))
    return true;

  // Published hold at a STAR fix (random, once per arrival). Runs early so it owns the
  // frame while holding (issues the hold, ticks the EFC, then the release).
  if (poll_hold(ctx, dt, out_text, out_requires_readback))
    return true;

  // Connector-STAR "direct <IAF>" (FMS discontinuity at the filed-STAR terminus).
  if (poll_connector_direct(ctx, out_text, out_requires_readback))
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
  // Descend-to-enter the terminal TMA (now a shared helper -- also runs in
  // poll_arrival so the aircraft is stepped INTO the TMA before the handoff).
  if (poll_descend_to_enter_tma(ctx, out_text, out_requires_readback))
    return true;

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

// STAR direct-to-IAF shortcut. Runs in IFR_ARRIVAL (which by design begins at the
// 1st STAR fix), so reaching here == "past the first STAR point". One-shot per
// arrival: ATC may clear "direct <IAF>" to the approach IAF GEOGRAPHICALLY NEAREST
// the aircraft, cutting the remaining STAR track -- but only when the direct still
// (a) saves >= 5% of the point-by-point routed distance to the natural approach
// join, and (b) stays flyable: descent <= 3 deg to the IAF's AT-OR-BELOW altitude
// (the block CEILING, never the floor -- so a block-constrained IAF like COLLO on
// SALEV3P is not wrongly judged too steep). 20% roll, or 100% with
// settings::shortcut_always(). The pilot may refuse (UNABLE) -> revert to the STAR.
// The candidate set is the expected approach's IAFs; the generic nearest-pick lives
// in route_shortcut::pick_nearest so other STAR strategies can reuse it. [C. P. Potter]
static bool poll_star_shortcut(const xplane_context::XPlaneContext &ctx,
                               std::string *out_text,
                               bool *out_requires_readback) {
  if (s_star_shortcut_offered)         return false; // one-shot per arrival
  if (s_approach_cleared_issued)       return false; // already cleared for approach
  if (!s_no_star_direct_iaf.empty())   return false; // no-STAR path already directs
  if (s_assigned_star_name.empty())    return false; // STAR arrivals only
  if (s_assigned_approach_designator.empty()) return false;
  if (ctx.cifp_dir.empty() || s_route_fixes.empty()) return false;
  const std::string dest = !s_assigned_dest_icao.empty()
                               ? s_assigned_dest_icao
                               : simbrief_ofp::get().destination_icao;
  if (dest.empty()) return false;

  // The roll happens exactly once per arrival, regardless of outcome.
  s_star_shortcut_offered = true;
  const bool always = settings::shortcut_always();
  if (!always && (std::rand() % 5) != 0)
    return false; // 80% of arrivals: no shortcut

  auto iaf_idents = cifp_reader::approach_transition_idents(
      ctx.cifp_dir, dest, s_assigned_approach_designator);
  if (iaf_idents.empty())
    return false;
  auto iaf_pos = cifp_reader::lookup_fix_positions(ctx.cifp_dir, iaf_idents, dest);

  // IAF at-or-below altitude for the descent gate: scan the STAR fixes (a
  // block-constrained IAF like COLLO carries its block there) then the route
  // fixes. Use the CEILING (block upper / '-' / exact); a pure at-or-above
  // (is_floor only) forces no descent -> 0 disables the gate for that candidate.
  auto star_wps = cifp_reader::star_waypoints(ctx.cifp_dir, dest,
                                              s_assigned_star_name, false);
  auto iaf_cross_alt = [&](const std::string &id) -> int {
    for (const auto &w : star_wps)
      if (w.ident == id && w.alt.feet > 0)
        return (w.is_floor && !w.is_ceiling) ? 0 : w.alt.feet;
    for (const auto &rf : s_route_fixes)
      if (rf.ident == id && rf.alt.feet > 0)
        return (rf.is_floor && !rf.is_ceiling) ? 0 : rf.alt.feet;
    return 0;
  };

  // Reference = point-by-point routed distance to the natural approach join (the
  // STAR terminus = last non-approach-proc route fix ahead). Every candidate's
  // saving is measured against this single baseline.
  int join_idx = -1;
  for (int i = static_cast<int>(s_route_fixes.size()) - 1;
       i >= std::max(0, s_route_fix_idx); --i)
    if (!s_route_fixes[i].is_approach_proc) { join_idx = i; break; }
  if (join_idx < 0)
    join_idx = static_cast<int>(s_route_fixes.size()) - 1;
  const double routed_to_join = routed_distance_to_fix_idx(ctx, join_idx);
  if (routed_to_join <= 1.0)
    return false;

  auto route_idx_of = [&](const std::string &id) -> int {
    for (int i = std::max(0, s_route_fix_idx);
         i < static_cast<int>(s_route_fixes.size()); ++i)
      if (s_route_fixes[i].ident == id)
        return i;
    return -1;
  };

  std::vector<route_shortcut::Candidate> cands;
  for (const auto &id : iaf_idents) {
    auto it = iaf_pos.find(id);
    if (it == iaf_pos.end())
      continue;
    route_shortcut::Candidate c;
    c.ident = id;
    c.lat = it->second.first;
    c.lon = it->second.second;
    c.routed_nm = routed_to_join; // shared point-by-point baseline
    c.cross_alt_ft = iaf_cross_alt(id);
    c.route_idx = route_idx_of(id);
    cands.push_back(std::move(c));
  }
  if (cands.empty())
    return false;

  route_shortcut::Pick pick = route_shortcut::pick_nearest(
      ctx.latitude, ctx.longitude, static_cast<int>(ctx.pressure_alt_ft), cands,
      /*min_save_frac*/ 0.05, /*max_descent_deg*/ 3.0);
  if (!pick.ok) {
    logging::info(
        "IFR STAR shortcut: roll passed but no worthwhile/flyable IAF");
    return false;
  }

  // Issue the direct. Jump the route tracker to the IAF when it is on the route
  // (save the previous idx so an UNABLE can restore the STAR routing).
  s_star_shortcut_prev_idx = s_route_fix_idx;
  if (pick.route_idx >= 0) {
    s_route_fix_idx = pick.route_idx;
    s_pending_route_direct = "ATC direct: " + pick.ident;
  }
  s_star_shortcut_pending = true;

  const std::string &cs = atc_state_machine::session_callsign();
  const std::string &callsign = cs.empty() ? settings::pilot_callsign() : cs;
  if (out_text)
    *out_text = callsign + ", direct " + pick.ident + ", when able.";
  if (out_requires_readback)
    *out_requires_readback = false;
  logging::info("IFR STAR shortcut: direct %s (nearest IAF, saved %.0f NM, "
                "descent %.1f deg, %s)",
                pick.ident.c_str(), pick.saved_nm, pick.descent_deg,
                always ? "forced" : "roll");
  return true;
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

  // Published hold at a STAR fix (random, once per arrival). Runs early so it owns the
  // frame while holding.
  if (poll_hold(ctx, dt, out_text, out_requires_readback))
    return true;

  // Connector-STAR "direct <IAF>" (FMS discontinuity at the filed-STAR terminus).
  if (poll_connector_direct(ctx, out_text, out_requires_readback))
    return true;

  // Bypass-IAF radar vectors to final (reversal onto the approach course, e.g. LOWI
  // R08-Z). One-shot, large-turn-gated -> no-op for normal arrivals. Runs here in
  // case the approach phase has not been entered yet at ~12 NM from the IAF.
  if (poll_vector_to_intercept(ctx, dt, out_text, out_requires_readback))
    return true;

  // STAR direct-to-IAF shortcut (nearest IAF, one-shot, 20% / 100%). Runs after
  // the vector/connector directs so it never competes with a reversal vectoring.
  if (poll_star_shortcut(ctx, out_text, out_requires_readback))
    return true;

  s_arrival_timer += dt;

  // FIR step-down INTO the dest terminal TMA BEFORE the approach handoff (user
  // 2026-07-22): the same descend-to-enter as DESCENT, run here so the aircraft
  // reaches the highest FL below the TMA ceiling before the IAF, instead of being
  // handed to Approach while still above the ceiling (LFLP->LFMD: FL120 over the
  // FL115 NICE TMA). Respects block floors (defers when a fix min holds it high).
  if (poll_descend_to_enter_tma(ctx, out_text, out_requires_readback))
    return true;

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
      // Query at the ACTUAL altitude (no early buffer): openair_alt is pressure
      // alt above the transition, so this is FL-vs-FL. At FL120 against a FL115
      // ceiling the aircraft is correctly ABOVE the TMA -> inside_tma=false, and it
      // is NOT handed off until the FIR step-down (poll_descend_to_enter_tma) puts
      // it inside. The old -1000 ft "about to drop in" buffer probed FL110 and
      // matched a FL115-ceiling TMA while still 500 ft above it -> premature
      // Nice-APP handoff at FL120 (LFLP->LFMD near NEKIP, user 2026-07-22).
      // find_enclosing still returns the INNERMOST volume (terminal TMA, not the
      // outer one) once actually inside.
      enc_arrival = openair_db::find_enclosing(ctx.latitude, ctx.longitude,
                                               openair_alt(ctx));
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
        iaf = resolve_approach_iaf(ctx, s_assigned_star_name,
                                   s_assigned_approach_designator);
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

    // Do NOT hand off to Approach while still ABOVE the destination terminal TMA:
    // the FIR steps the aircraft down into it first (poll_descend_to_enter_tma),
    // and the APP handoff fires on actual TMA ENTRY (user 2026-07-22, LFLP->LFMD:
    // was handed to Nice at FL120, above the FL115 NICE TMA). Skips the dest's-OWN
    // stacked-CTA path (enc_is_dest_cta), which deliberately hands off on CTA entry
    // for a field whose own centre owns the CTA (LFMN). openair_alt is pressure alt
    // above the transition, so this is a flight-level-vs-flight-level test.
    if (enter_approach && !enc_is_dest_cta && openair_db::ready()) {
      const int dest_tma_ceil =
          openair_db::terminal_tma_ceiling(ctx.latitude, ctx.longitude);
      if (dest_tma_ceil > 1000 &&
          static_cast<int>(openair_alt(ctx)) > dest_tma_ceil + 100) {
        enter_approach = false; // above the TMA -> wait for step-down + entry
        logging::info("IFR arrival: APP handoff deferred -- above dest TMA "
                      "(ceil %d, alt %d) pending step-down",
                      dest_tma_ceil, static_cast<int>(openair_alt(ctx)));
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

// Approach FAF / IAF idents for the STT context bias: the pilot reports
// "established" at the FAF and is cleared to the approach at the IAF, so these
// two fix names are worth anchoring in the approach phase. Empty until assigned.
const std::string &approach_faf_ident() { return s_approach_faf.ident; }
const std::string &approach_iaf_ident() { return s_no_star_direct_iaf; }

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

static std::string resolve_approach_iaf(const xplane_context::XPlaneContext &ctx,
                                        const std::string &star_name,
                                        const std::string &approach_designator,
                                        std::string *out_connector) {
  if (out_connector)
    out_connector->clear();
  if (star_name.empty() || approach_designator.empty() || ctx.cifp_dir.empty() ||
      s_assigned_dest_icao.empty())
    return {};
  const std::string star_end = cifp_reader::star_last_fix(
      ctx.cifp_dir, s_assigned_dest_icao, star_name);
  const auto trans = cifp_reader::approach_transition_idents(
      ctx.cifp_dir, s_assigned_dest_icao, approach_designator);
  if (trans.empty())
    return star_end; // approach has no named transitions -> use STAR end as-is
  for (const auto &t : trans)
    if (t == star_end)
      return star_end; // STAR terminus IS an approach IAF (normal case)
  // STAR ends at a fix that is not this approach's IAF -> bridge via a connector
  // STAR (LOWI NANI2A->RTT, R08-Z begins at ELMEM, RTT1B: RTT->ELMEM).
  const std::string link = cifp_reader::connector_star(
      ctx.cifp_dir, s_assigned_dest_icao, star_end, trans);
  if (link.empty())
    return star_end; // no bridge -> fall back (route may be discontinuous)
  if (out_connector)
    *out_connector = link;
  return cifp_reader::star_last_fix(ctx.cifp_dir, s_assigned_dest_icao, link);
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
      std::string iaf;
      if (s_assigned_star_name.empty()) {
        iaf = s_no_star_direct_iaf;
      } else {
        // Splice in a connector STAR (LOWI RTT1B: RTT->ELMEM) when the assigned
        // STAR does not terminate at an IAF of the selected approach, so the
        // arrival route stays continuous. [[project_star_chaining]]
        std::string connector;
        iaf = resolve_approach_iaf(ctx, s_assigned_star_name,
                                   s_assigned_approach_designator, &connector);
        if (!connector.empty()) {
          auto link_wps = cifp_reader::star_waypoints(
              ctx.cifp_dir, s_assigned_dest_icao, connector,
              /*constrained_only=*/false);
          for (auto &w : link_wps)
            arr.push_back(w);
          logging::info(
              "[route] STAR chain: %s -> connector %s -> approach IAF %s",
              s_assigned_star_name.c_str(), connector.c_str(), iaf.c_str());
        }
      }
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

    // Drop missed-approach fixes (everything AFTER the MAP) from the FORWARD route
    // table: they are the go-around procedure (climb-out + hold), not the arrival,
    // and otherwise leak into the tracker and the STT bias (LOWI R08-Z: WI103 /
    // WI002 / RTT after the RW08 MAP). The MAP itself (runway threshold) is kept as
    // the last route fix. The FULL missed-approach sequence stays in
    // s_approach_waypoints (with s_map_ap_idx) for GO_AROUND handling.
    for (size_t i = 0; i < arr.size(); ++i) {
      if (arr[i].is_map) {
        arr.resize(i + 1); // keep up to and including the MAP
        break;
      }
    }

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

  // FREEZE the walker while actively holding: the aircraft is orbiting the hold fix,
  // so the proximity check (and worse, the 2 NM RESYNC scan below) would keep advancing
  // s_route_fix_idx past the hold fix -- and could even jump forward to a later fix that
  // an orbit leg wanders within 2 NM of. On the real vol (LIMF->LOWI 2026-07-31) the
  // tracker logged "near NANIT, next: RTT" mid-hold. Pin the route position until the
  // hold is released (s_hold_state != 1); the arrival then resumes cleanly from the hold
  // fix. [C. P. Potter]
  if (s_hold_state == 1) return {};

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
    s_vec_plan = {};
    s_vec_step = -1;
    s_vec_timer = 0.0f;
    s_vec_done = false;
  s_vec_assigned_hdg = -1.0;
  s_vec_follow_timer = 0.0f;
  s_vec_nudged = false;
  s_vec_expect_issued = false;
    // NOTE: the hold statics (s_hold*) are deliberately NOT reset here. The hold is
    // descent/arrival-scoped -- armed by poll_hold from poll_descent/poll_arrival, i.e.
    // BEFORE the aircraft ever reaches IFR_APPROACH_*. Because this not-in-approach
    // branch runs every frame while still in descent, resetting the hold here wiped the
    // s_hold_state==1 latch each frame, so poll_hold re-armed and re-issued the hold call
    // repeatedly with a freshly-rolled EFC (real vol LIMF->LOWI 2026-07-31: 7x "hold at
    // NANIT ... EFC 5/5/4/4/2/4/3 min"). The hold is reset by the three per-flight paths
    // (engine::reset, training_jump_enroute, training_jump_approach) and self-latches to
    // state 2 (done) on release -- one hold per arrival. [C. P. Potter]
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
      const std::string iaf = resolve_approach_iaf(
          ctx, s_assigned_star_name, s_assigned_approach_designator);
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
  // AFIS = no ATC position at all: neither a Ground NOR an Approach frequency.
  // A towered field with no separate Ground still has one of them (LOWI: TWR +
  // APP, no Ground) and must NOT be treated as AFIS/Information.
  const std::string dest_ap = current_flight_airport(ctx);
  const bool dest_is_afis = !xplane_context::has_ground_freq_for(dest_ap) &&
                            !xplane_context::has_approach_freq_for(dest_ap) &&
                            !dest_ctrl_override_present(dest_ap);
  if (state == AS::IFR_APPROACH_CONTACT &&
      !(s_assigned_star_name.empty() && s_approach_final_issued && dest_is_afis))
    return false;

  // Radar vectors to final take PRIORITY over the sector-boundary handoff below.
  // A reversal-vectoring (LOWI R08-Z) MUST fire from the CURRENT approach controller
  // (Innsbruck Radar) BEFORE any terminal sector handoff -- otherwise the sector block
  // fires "contact Innsbruck Approach 120.100" first, arms s_sector_checkin_pending, and
  // the "if (s_sector_checkin_pending) return false" gate further down then starves
  // poll_vector for the rest of the approach -> the vectoring never fires (real vol
  // 2026-08-01: no vectoring at ELMEM). [C. P. Potter]
  //
  // Ensure the FAF is resolved BEFORE the vector arm: for a WITH-STAR arrival the FAF is
  // set on the descent clearance (~engine.cpp:2214), but the No-STAR block below sets it
  // only when s_assigned_star_name is empty. If poll_vector's precondition sees an empty
  // s_approach_faf (real vol 2026-08-01: LOWI NANI2A -> R08-Z, FAF never populated here),
  // it bails "precondition empty: faf" and the reversal never arms. Resolve it now,
  // independent of the STAR, so both paths have it. [C. P. Potter]
  if (s_approach_faf.ident.empty() && !s_assigned_approach_designator.empty() &&
      !s_assigned_dest_icao.empty() && !ctx.cifp_dir.empty()) {
    s_approach_faf = cifp_reader::approach_faf(ctx.cifp_dir, s_assigned_dest_icao,
                                               s_assigned_approach_designator);
    if (!s_approach_faf.ident.empty())
      logging::info("IFR approach: FAF resolved = %s (fallback for vectoring)",
                    s_approach_faf.ident.c_str());
  }
  if (poll_vector_to_intercept(ctx, dt, out_text, out_requires_readback))
    return true;

  // ── Sector-boundary handoff (IFR_APPROACH_DESCENT) ─────────────────────────
  // When the enclosing TRACON/CTR changes or disappears while in approach
  // descent, the current controller (e.g. Melun) hands off to the next sector
  // controller (e.g. Paris FIR Information) or to the destination TOWER/AFIS.
  // Uses the same 15-second polling as en-route sector-change sub-phase 1.5.
  if (state == AS::IFR_APPROACH_DESCENT && !s_approach_tower_handed_off &&
      !(s_vec_step >= 0 && !s_vec_done) && // hold while radar-vectoring to final
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
          // STAY unconditionally (build 165). This block runs ONLY in
          // IFR_APPROACH_DESCENT, where the aircraft is committed to the dest
          // approach -- forward terminal handoffs go through the TMA branch above
          // (force_forward, e.g. Geneva->Chambery), never here. So an innermost CTA
          // can only SUPPRESS a spurious handoff, never create one: the dest's own
          // stacked CTA (LFMN NICE CTA SECTOR 2), the SALEV3P backward-to-Marseille
          // case, AND an ENROUTE CTA the approach path re-crosses -- the LOWI R08-Z
          // RTT->ELMEM reversal into VIENNA CTA C (real vol 2026-07-31: a spurious
          // "contact Vienna" mid-approach that then suppressed the vectoring +
          // cleared-approach). The earlier dest-facility restriction let that enroute
          // CTA through; keying on APPROACH_DESCENT (this block's gate) instead makes
          // the broad stay safe -- enroute Milan->Geneva->Chambery handoffs live in
          // poll_acc_sector_change / the TMA branch, not this CTA branch. Replaces the
          // s_enroute_approach_handoff_issued latch attempt (that flag is reset every
          // frame by poll_enroute outside CRUISE, so it was always false here).
          // [C. P. Potter]
          stay = true;
        }
      }
      if (stay)
        best = nullptr; // suppress this cycle's handoff entirely

      // The DESTINATION's own terminal chain -- its approach controller (already
      // established, e.g. Innsbruck Radar 128.975 via airport+.json) and the FAF Tower
      // handoff (the at_faf logic below) -- is NOT this enroute sector block's job.
      // Only a FORWARD handoff to a DIFFERENT facility's terminal TMA belongs here
      // (LFLP worked by Chambery LFLB). Suppress everything else: (a) the dest's OWN
      // facility -- at LOWI the block resolves the Innsbruck TMA -> atc.dat Innsbruck
      // 120.100 = the TOWER, mislabelled "Innsbruck Approach", firing a premature/
      // redundant handoff while already on Innsbruck Radar (real vol 2026-08-01); and
      // (b) any non-terminal enroute sector reached via the atc.dat picker (Vienna/
      // Munich on the R08-Z reversal). [C. P. Potter]
      if (best && !(force_forward && !s_assigned_dest_icao.empty() &&
                    best->facility_id != s_assigned_dest_icao)) {
        logging::info("[approach] sector handoff suppressed -- %s (%s) not a "
                      "non-dest terminal (force_forward=%d dest=%s)",
                      best->name.c_str(), best->facility_id.c_str(),
                      force_forward ? 1 : 0, s_assigned_dest_icao.c_str());
        best = nullptr;
      }

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
                       current_flight_airport(ctx)) &&
                   !xplane_context::has_approach_freq_for(
                       current_flight_airport(ctx)) &&
                   !dest_ctrl_override_present(current_flight_airport(ctx))) {
          is_info_svc = true; // has a Tower freq but no Ground AND no Approach = AFIS
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
                apt_name2 = spoken_airport_name(s_assigned_dest_icao);
              if (!apt_name2.empty())
                ctrl_label = apt_name2 + " Information";
              else
                ctrl_label = s_assigned_dest_icao.empty()
                                 ? "Information"
                                 : (s_assigned_dest_icao + " Information");
            }
          } else {
            {
              std::string on;
              float of = 0.0f;
              ctrl_label = (airport_overrides::controller(
                                current_flight_airport(ctx), "tower", &on, &of) &&
                            !on.empty())
                               ? on
                               : "Tower";
            }
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

  // Published hold at a STAR fix (random, once per arrival). Safe here: only fires on
  // a STAR fix still AHEAD of the tracker (past the STAR -> no eligible fix -> no-op).
  if (poll_hold(ctx, dt, out_text, out_requires_readback))
    return true;

  // NOTE: bypass-IAF radar vectors to final (poll_vector_to_intercept, LOWI R08-Z
  // reversal) now run EARLIER -- before the sector-boundary block and the
  // s_sector_checkin_pending gate -- so a terminal sector handoff can't starve them
  // (real vol 2026-08-01). From its first vector it latches s_approach_cleared_issued
  // so the cleared-at-IAF gate below no-ops; large-turn-gated -> no-op for normal
  // arrivals. See the poll_vector_to_intercept call above this block.

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
      sel_iaf = resolve_approach_iaf(ctx, s_assigned_star_name,
                                     s_assigned_approach_designator);
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
        // ROUTED distance (fix-by-fix along the tracked route) to the IAF, not
        // straight-line (user 2026-07-31: do it now, with the vectoring). On a
        // dog-legged / looping STAR (SALEV3P) the great-circle acft->IAF under-reads
        // -- COLLO sits ~2 NM from PIRUV straight-line but 4 fixes of loop away -- so
        // the routed sum measures the real track remaining and stops the clearance
        // firing early. routed_distance_to_fix_idx already falls back to straight-line
        // when the IAF is behind the tracker, and skips backward-lagging fixes. The
        // <=2-fix sequence guard below stays as belt-and-suspenders. Once the
        // connector-direct jumps the tracker to ELMEM, the routed sum == direct, so
        // LOWI gets "direct ELMEM" then the clearance a few NM later, as intended.
        const double eta = routed_distance_to_fix_idx(ctx, iaf_idx) / gs * 3600.0;
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
        // Reversal-vector case (IAF-centred): warn the pilot EARLY with "expect vectors"
        // (so he does not wonder why there is no clearance while flying toward the IAF --
        // FAA issues "expect vectors" ahead of time), and DEFER the straight eta-based
        // clearance so the VECTOR owns it. Without the defer the straight clearance fires
        // at ~7 NM and advances the tracker PAST the IAF, killing the vector (real vol
        // LOWI R08-Z 2026-08-02). The FAF "vectors to final" case is separate. [CPP]
        const bool reversal = approach_needs_reversal_vector(ctx);
        if (reversal && !s_vec_expect_issued && !s_sector_checkin_pending &&
            s_route_fix_idx >= iaf_idx - 2 &&
            routed_distance_to_fix_idx(ctx, iaf_idx) > 5.0) {
          s_vec_expect_issued = true;
          const std::string ph = approach_clearance_phrase(ctx);
          *out_text = cs + ", expect vectors for " +
                      (ph.empty() ? std::string("the approach") : ph) + ".";
          logging::info("[approach] expect-vectors heads-up (reversal at IAF %s)",
                        iaf.ident.c_str());
          rb(false);
          return true;
        }
        if (!reversal && eta <= 180.0 && s_route_fix_idx >= iaf_idx - 2 &&
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
    // Per-approach OVERRIDE of the handoff trigger (airport+.json
    // tower_handoff_fixes): a curved RNP final has its FAF far out (LOWI RNP 08:
    // FAF WI749 ~28 NM from RW08), so the FAF-based handoff fires far too early --
    // the pilot is nowhere near "established" (real vol 2026-08-02, Info kept asking
    // "confirm established"). When the destination approach names a last-turn fix,
    // hand to Tower only when the tracker passes THAT fix (established on the
    // straight-in). LOWI R08-Z -> WI754, R26-Z -> WI103. [C. P. Potter]
    int ho_idx = -1;
    {
      const std::string ho_fix = airport_overrides::tower_handoff_fix(
          current_flight_airport(ctx), s_assigned_approach_designator);
      if (!ho_fix.empty())
        for (int i = 0; i < static_cast<int>(s_route_fixes.size()); ++i)
          if (s_route_fixes[i].ident == ho_fix) {
            ho_idx = i;
            break;
          }
    }
    if (ho_idx >= 0) {
      at_faf = (s_route_fix_idx > ho_idx);
      if (!at_faf && s_route_fix_idx >= ho_idx) {
        const RouteFix &hf = s_route_fixes[ho_idx];
        if (hf.lat != 0.0 || hf.lon != 0.0)
          at_faf = (traffic_geometry::distance_nm(ctx.latitude, ctx.longitude,
                                                  hf.lat, hf.lon) < 2.0);
      }
    } else if (s_faf_route_idx >= 0) {
      // Primary: route tracker has passed the FAF fix.
      // Suppressed when the dual-use IAF/MAP-hold fix (s_iaf_route_idx) sits after
      // the FAF in s_route_fixes: the IAF itself is skipped by approach_procedure_waypoints
      // (IF path_term), but the same fix reappears as the missed-approach hold (DF path_term).
      // After a direct-to-IAF, step 4 advances route_idx past that hold (idx>iaf_idx), which
      // would fire primary immediately at the IAF position. Use distance-only in that case.
      if (s_iaf_route_idx < 0 || s_iaf_route_idx <= s_faf_route_idx)
        at_faf = (s_route_fix_idx > s_faf_route_idx);
      // Fallback: direct distance to the FAF -- but ONLY once the route tracker has
      // reached the FAF in route order (s_route_fix_idx >= s_faf_route_idx), i.e. every
      // approach fix BEFORE the FAF is overflown. "You cannot be at the FAF while points
      // before it are not overflown" (user 2026-08-01). Without this gate the reversal
      // geometry trips it early: LOWI R08-Z has the FAF WI749 EAST of the IAF ELMEM, so an
      // aircraft flying direct to ELMEM (heading west) crosses within 2 NM of WI749 ~4 NM
      // BEFORE reaching ELMEM -> a premature Tower handoff that killed the vectoring
      // (real vol 2026-08-01, ENR-jump-near-STAR). [C. P. Potter]
      if (!at_faf && s_route_fix_idx >= s_faf_route_idx &&
          (s_approach_faf.lat != 0.0 || s_approach_faf.lon != 0.0)) {
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

    // Cannot hand off to Tower before the aircraft has flown the approach fixes
    // PRECEDING the FAF (user 2026-08-01: "on ne peut pas passer sur la tour avant
    // d'etre passe sur les points precedant le FAF"). Two ways the distance-only
    // at_faf trips early on a reversal: (a) the vector-to-intercept is still turning
    // the aircraft onto the approach -- it is NOT established; (b) LOWI R08-Z, where
    // the IAF (ELMEM) sits WEST of the FAF (WI749), so an east->west arrival crosses
    // WI749 BEFORE reaching ELMEM (real vol 2026-07-31/08-01: Tower fired at the IAF
    // and killed the vectoring). Suppress the Tower handoff while vectoring, and until
    // the route tracker has actually passed the IAF. [C. P. Potter]
    // Throttle the "held" reasons: log a reason only when it CHANGES, so a held
    // handoff does not spam Log.txt every frame (real vol 2026-08-01: ~130 lines of
    // "vector-to-intercept in progress"). [C. P. Potter]
    static std::string s_tower_held_diag;
    auto hold_log = [&](const std::string &key, const char *msg) {
      if (s_tower_held_diag != key) {
        s_tower_held_diag = key;
        logging::info("[approach] Tower handoff held -- %s", msg);
      }
    };
    if (at_faf && s_vec_step >= 0 && !s_vec_done) {
      hold_log("vec", "vector-to-intercept in progress");
      at_faf = false;
    }
    if (at_faf && s_iaf_route_idx >= 0 && s_route_fix_idx <= s_iaf_route_idx) {
      char hb[96];
      std::snprintf(hb, sizeof(hb), "IAF not yet passed (route_idx=%d iaf_idx=%d)",
                    s_route_fix_idx, s_iaf_route_idx);
      hold_log("iaf", hb);
      at_faf = false;
    }
    // HARD IFR rule (user 2026-08-01): "en IFR on passe Tour APRES le FAF, report
    // established (ou runway in sight) suivant l'approche". A pilot can only report
    // "established" on an approach he was actually CLEARED for. If "cleared <appr>
    // runway <rwy>" was never issued (s_approach_cleared_issued == false), the
    // aircraft is not on the approach -- it is still under Approach/Radar and must
    // NOT be handed to Tower. This is precisely the LIMF->LOWI R26-Z failure
    // (real vol 2026-08-01): the route-tracker jumped past the IAF, cleared-approach
    // never fired, yet the aircraft reached the FAF at FL150 and was wrongly handed
    // to Tower ("survol terrain"). No clearance -> stay with Approach. [C. P. Potter]
    if (at_faf && !s_approach_cleared_issued) {
      char hb[128];
      std::snprintf(hb, sizeof(hb),
                    "approach not cleared (no 'cleared approach' issued; not "
                    "established) route_idx=%d faf_idx=%d iaf_idx=%d",
                    s_route_fix_idx, s_faf_route_idx, s_iaf_route_idx);
      hold_log("nocleared", hb);
      at_faf = false;
    }

    if (at_faf) {
      // Diagnostic: WHY the Tower fires -- indices + distances + vectoring state, so a
      // premature reversal Tower is traceable without guessing the tracker (2026-08-01).
      double d_faf = (s_approach_faf.lat != 0.0 || s_approach_faf.lon != 0.0)
                         ? traffic_geometry::distance_nm(ctx.latitude, ctx.longitude,
                                                         s_approach_faf.lat,
                                                         s_approach_faf.lon)
                         : -1.0;
      double d_iaf = -1.0;
      if (s_iaf_route_idx >= 0 &&
          s_iaf_route_idx < static_cast<int>(s_route_fixes.size()))
        d_iaf = traffic_geometry::distance_nm(
            ctx.latitude, ctx.longitude, s_route_fixes[s_iaf_route_idx].lat,
            s_route_fixes[s_iaf_route_idx].lon);
      logging::info("[approach] Tower FIRING: route_idx=%d faf_idx=%d iaf_idx=%d "
                    "dFAF=%.1f dIAF=%.1f vec_step=%d vec_done=%d pos=%.4f,%.4f",
                    s_route_fix_idx, s_faf_route_idx, s_iaf_route_idx, d_faf, d_iaf,
                    s_vec_step, s_vec_done ? 1 : 0, ctx.latitude, ctx.longitude);
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
      // MDA/visual ONLY when the approach neither terminates at the runway NOR
      // publishes a vertical angle. RNAV DA approaches (LFMD R35-Z = 3.5 deg,
      // LFMN R04LA) end at a fix + missed-approach hold (no runway leg) yet carry
      // a vertical angle -> they are DA/instrument -> "report established" (user
      // 2026-07-22: RNAV 35 Z was wrongly read as MDA -> "report runway in sight").
      s_approach_has_visual_final =
          !s_assigned_approach_designator.empty() &&
          !cifp_reader::approach_terminates_at_runway(
              ctx.cifp_dir, s_assigned_dest_icao, s_assigned_approach_designator) &&
          !cifp_reader::approach_has_vertical_guidance(
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
                     current_flight_airport(ctx)) &&
                 !xplane_context::has_approach_freq_for(
                     current_flight_airport(ctx)) &&
                 !dest_ctrl_override_present(current_flight_airport(ctx))) {
        // No Ground AND no Approach freq → AFIS/Information service, not a real
        // Tower controller. A towered field with no separate Ground still has
        // Approach (LOWI: TWR + APP). current_flight_airport() returns the bound
        // destination so a phantom airport near it can't corrupt the check.
        // airport+.json may override a local Tower (LOWI 120.100) -> not AFIS.
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
                apt_name = spoken_airport_name(s_assigned_dest_icao);
              if (!apt_name.empty())
                ctrl_label = apt_name + " Information";
              else
                ctrl_label = s_assigned_dest_icao.empty() ? "Information"
                                                          : (s_assigned_dest_icao + " Information");
            }
          } else {
            {
              std::string on;
              float of = 0.0f;
              ctrl_label = (airport_overrides::controller(
                                current_flight_airport(ctx), "tower", &on, &of) &&
                            !on.empty())
                               ? on
                               : "Tower";
            }
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
          // Trace the proactive approach step-down at INFO -- these were previously
          // logged NOWHERE (only in transcript.log via TTS), so a spurious descent
          // (bogus "descend flight level 135" at speed-only WI751, real vol
          // 2026-08-02) was invisible in Log.txt. Now every issued step-down names
          // its target fix + altitude + the cleared-alt it steps down from. [CPP]
          logging::info("[approach] step-down at %s -> %s%d (from cleared %d, "
                        "cleared_appr=%d afis=%d)",
                        twp.ident.c_str(), tfl ? "FL" : "", tfl ? tft / 100 : tft,
                        cur_cl_ap, s_approach_cleared_issued ? 1 : 0,
                        dest_is_afis ? 1 : 0);
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

  // Reference axis for the cross-track check = the FINAL APPROACH SEGMENT, not the
  // extended runway centreline. General case: FAF -> threshold. For a curved RNP
  // final whose straight-in starts at a last-turn fix (airport+.json
  // tower_handoff_fixes: LOWI R08-Z WI754 / R26-Z WI103), use THAT fix -> threshold,
  // so a correctly-flown offset/curved final is not read as a deviation
  // (user 2026-08-02). Falls back to the runway centreline when no fix geometry is
  // available. The anchor is the fix -> threshold OUTBOUND bearing. [C. P. Potter]
  double anchor_lat = 0.0, anchor_lon = 0.0;
  bool anchor_valid = false;
  {
    const std::string ho_fix = airport_overrides::tower_handoff_fix(
        current_flight_airport(ctx), s_assigned_approach_designator);
    if (!ho_fix.empty())
      for (const auto &rf : s_route_fixes)
        if (rf.ident == ho_fix && (rf.lat != 0.0 || rf.lon != 0.0)) {
          anchor_lat = rf.lat;
          anchor_lon = rf.lon;
          anchor_valid = true;
          break;
        }
    if (!anchor_valid &&
        (s_approach_faf.lat != 0.0 || s_approach_faf.lon != 0.0)) {
      anchor_lat = s_approach_faf.lat;
      anchor_lon = s_approach_faf.lon;
      anchor_valid = true;
    }
  }
  // Approach course from threshold (outbound toward the final-segment anchor;
  // fallback = extended runway centreline rwy_hdg + 180).
  const double approach_course =
      anchor_valid ? traffic_geometry::bearing_deg(rwy_lat, rwy_lon, anchor_lat,
                                                   anchor_lon)
                   : std::fmod(static_cast<double>(rwy_hdg) + 180.0, 360.0);
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

  std::string hp_phrase = "runway " + ctx.active_runway;
  auto hp_it = ctx.runway_holding_points.find(ctx.active_runway);
  if (hp_it != ctx.runway_holding_points.end() && !hp_it->second.empty()) {
    hp_phrase = "holding point " +
                atc_phonetic::spell_holding_point(hp_it->second) + ", runway " +
                ctx.active_runway;
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

bool tod_to_go(float groundspeed_kts, float *out_nm, float *out_min) {
  if (s_tod_dist_nm < 0.0f || s_tod_alert_nm < 0.0f)
    return false;
  const float nm = s_tod_dist_nm - s_tod_alert_nm; // >0 before TOD, <=0 at/after
  if (out_nm)
    *out_nm = nm;
  if (out_min)
    *out_min = (groundspeed_kts > 40.0f) ? (nm / groundspeed_kts * 60.0f) : -1.0f;
  return true;
}

} // namespace engine
