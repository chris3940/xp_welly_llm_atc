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

#ifndef ENGINE_ENGINE_HPP
#define ENGINE_ENGINE_HPP

#include "atc/intent_parser.hpp"
#include "core/xplane_context.hpp"

#include <functional>
#include <string>

namespace engine {

struct Input {
  std::string transcript;
  float quality = 1.0f; // Whisper quality; 1.0 for text-only tests
  // Non-owning pointer to the current XPlaneContext. The plugin passes
  // &xplane_context::get(); the CLI will pass a scenario-built context.
  // Lifetime must cover the duration of process_transcript + any async
  // callbacks it spawns.
  const xplane_context::XPlaneContext *ctx = nullptr;
  std::string pilot_callsign;
  // Monotonic clock used by traffic_advisor cooldowns when the pilot's
  // utterance is a TRAFFIC_* acknowledgement. Plugin passes
  // XPLMGetElapsedTime; the headless CLI / tests pass a deterministic
  // counter. Defaults to 0 — fine for code paths that never enter the
  // traffic dialog.
  double now_secs = 0.0;
  // LM re-entry: when the LM callback needs the IFR-specific handlers that
  // live in process_transcript (e.g. early-approach-call, APPROACH_CONTACT
  // check-in), it re-invokes process_transcript with this field set.
  // process_transcript overrides the rule-based result with this intent.
  // UNKNOWN = not set (normal path).
  intent_parser::PilotIntent pre_classified_intent =
      intent_parser::PilotIntent::UNKNOWN;
  float pre_classified_conf = 0.0f;
};

struct Output {
  // Empty = silent transition (state changed but no response to speak).
  std::string response_text;
  intent_parser::PilotMessage parsed;
  // True for radio-discipline warnings — caller uses this if it needs to
  // distinguish "ATC clearance" from "ATC correction". State is unchanged
  // when is_warning is true.
  bool is_warning = false;
};

using Done = std::function<void(Output)>;

// Reset internal counters (profanity warnings, LLM call count). Call from
// plugin init / re-enable. Separate from the per-call flow so engine has
// no "stop" phase.
void reset();

// Training quick-start: skip normal flight phases and jump directly to the
// target state. Position the aircraft in X-Plane first; the plugin picks up
// normal proactive messaging from the new state on the next flight-loop tick.
void training_jump_enroute(int cleared_alt_ft); // IFR_ENROUTE_CRUISE, cleared FL in feet
void training_jump_arrival();                   // IFR_ARRIVAL (on the STAR, descending, under ACC)
void training_jump_approach();                  // IFR_APPROACH_CONTACT (pilot calls in)
void training_jump_predep();                    // IFR_PREDEP_CLEARANCE (ground, pre-startup)

// Frequency (MHz) the pilot should tune after the most recent training jump
// (the target phase's controller freq), or 0.0 if unknown. Jumps set ATC state
// only, never the radio; the UI surfaces this as a "Switch COM to X" popup.
float jump_switch_freq_mhz();

// CIFP-assigned landing runway (e.g. "22L"), set at the approach clearance and
// retained through landing + taxi-in. Empty before an approach is assigned.
// The STT context bias uses this (persistent) instead of the state machine's
// assigned_runway(), which is cleared post-landing.
const std::string &assigned_landing_runway();

// Number of LLM inferences kicked off by the engine since last reset()
// (intent classification, sub-variant disambiguation). Callers that
// maintain an aggregate inference counter (STT + TTS + LM) add this in.
int lm_inferences();

// Count of consecutive unintelligible pilot transmissions since the
// last successful intent. Reset by reset() and by any clear pilot
// reply. Exposed for tests / instrumentation; the engine drives the
// "say again, use standard phraseology" escalation off this internally.
int unclear_streak();

// Process a pilot transcript end-to-end:
//   - quality check (low quality -> say again)
//   - rule-based intent parse
//   - INAPPROPRIATE_LANGUAGE interception (escalating warnings)
//   - departure sub-variant disambiguation via local LLM (if loaded)
//   - state machine invocation with two-stage (direct vs. LLM) routing
//
// `done` is always called exactly once. On the sync path it runs before
// process_transcript returns; on the LLM-async path it runs later on the
// thread that the LLM callback is dispatched on (main thread, via the
// plugin's callback drain).
void process_transcript(Input in, Done done);

// Per-tick traffic-advisory poll. SDK-free: takes the current
// XPlaneContext, reads traffic_context::current() for the live
// snapshot, and runs traffic_advisor::evaluate(). On a positive
// decision, renders the advisory text via
// atc_state_machine::render_traffic_advisory() and notifies
// traffic_dialog so the next pilot transcript is routed there for
// acknowledgement. Returns true iff an advisory was emitted (caller is
// responsible for routing the text to TTS / transcript display).
//
// `now_secs` is the monotonic clock the cooldown logic compares
// against. In the plugin this is XPLMGetElapsedTime; in the headless
// CLI / tests the caller passes a deterministic counter.
bool poll_traffic_advisory(const xplane_context::XPlaneContext &ctx,
                           double now_secs, std::string *out_text);

// Phase-4 unsolicited go-around trigger. Frame-driven, render-only:
//   - user is in Pattern/LANDING_CLEARED
//   - user is within 1 NM of the active-runway threshold
//   - a ground-phase target sits on the active runway centerline
//   - no go-around has been emitted in the last 60 s
//
// On a positive decision, renders the `go_around_traffic_runway`
// template via atc_state_machine::render_traffic_advisory() and
// returns true. Does NOT change ATCState — go-around is a flight
// command, not a dialog turn (no readback, no ack channel).
//
// `now_secs` is the same monotonic clock poll_traffic_advisory uses.
bool poll_go_around(const xplane_context::XPlaneContext &ctx, double now_secs,
                    std::string *out_text);

// Per-tick readback-reminder poll. When a pending readback has gone
// unanswered for the configured cadence (20 s first, 25 s for
// repeats, max 3 nudges before the clearance is cancelled), renders
// the appropriate TRAFFIC_DIALOG entry ("readback_reminder" or, on
// the final timeout, "readback_cancel") and returns true. On a
// cancel, atc_state_machine has already moved state to IDLE and
// cleared readback_pending_ before this returns. Uses the same
// monotonic clock as poll_traffic_advisory / poll_go_around.
bool poll_readback_reminder(const xplane_context::XPlaneContext &ctx,
                            double now_secs, std::string *out_text);

// IFR departure handoff: fires ~10 s into CLIMB after IFR_DEPARTURE_CLEARED.
// Tells the pilot to contact Departure (large airport) or Approach (small).
// Transitions to IFR_EN_ROUTE; returns true when the handoff fired.
// out_text is empty when neither frequency exists (silent transition).
bool poll_departure_handoff(const xplane_context::XPlaneContext &ctx, float dt,
                            std::string *out_text);

// IFR SID climb management: fires ATC-initiated step climbs and an optional
// direct-to shortcut after the pilot checks in with Departure
// (IFR_RADAR_CONTACT state). Three phases:
//   1. ~20-40 s after check-in: "direct {last_fix}, climb FL{step1}" or
//      "climb FL{step1}" if no last fix.
//   2. When aircraft is within 500 ft of step1 alt (or 40 s timeout):
//      "climb FL{cruise}".
//   3. When altitude_ft_msl >= radar_handoff_alt_ft:
//      "contact Area Control, good day" → transitions to IFR_EN_ROUTE.
// Returns true when the message was fired this frame.
bool poll_sid_climb(const xplane_context::XPlaneContext &ctx, float dt,
                    std::string *out_text);

// ICAO speed restriction: issues "reduce speed to 250 knots" when the aircraft
// is below FL100 and IAS > 255 kt (5 kt hysteresis). Fires once per descent
// through FL100; resets automatically when the aircraft climbs back above FL100.
// Active in IFR_RADAR_CONTACT, IFR_ENROUTE_CRUISE, and IFR_APPROACH_CONTACT.
// No readback required.
bool poll_speed_restriction(const xplane_context::XPlaneContext &ctx,
                            std::string *out_text);

// Unified in-front-profile enforcement, dispatched once per frame before the
// per-phase poll_* handlers. Runs the same three checks -- altitude crossing
// (readback), speed restriction (readback), cleared-level compliance nag
// (advisory) -- in EVERY airborne IFR phase, so no phase is silently
// unmonitored. Coordinates with the poll_approach walker + descend-to-enter via
// the shared cleared-altitude state; see engine.cpp for the priority order and
// per-check phase applicability. out_requires_readback is set true only for the
// altitude/speed clearances, false for the advisory nag.
bool poll_profile_enforcement(const xplane_context::XPlaneContext &ctx, float dt,
                              std::string *out_text,
                              bool *out_requires_readback = nullptr);

// IFR en-route management: fires while in IFR_ENROUTE_CRUISE (on Centre).
// Three sub-functions:
//   1. ~90-120 s after Centre check-in: "direct {fix}, when able." (navlog
//   shortcut)
//   2. When openair_db detects entry into destination TMA: Centre issues
//   descent
//      clearance + Approach handoff → transitions to IFR_APPROACH_CONTACT.
//      Fired proactively — ATC does NOT wait for the pilot to request descent.
//   3. Cross-track deviation > 5 NM vs navlog: "confirm routing, you appear off
//   track."
//      (3-minute cooldown between warnings)
// Returns true when a message was emitted (caller routes to TTS + transcript).
bool poll_enroute(const xplane_context::XPlaneContext &ctx, float dt,
                  std::string *out_text,
                  bool *out_requires_readback = nullptr);

// IFR descent phase (IFR_DESCENT state): advances to IFR_ARRIVAL when the
// aircraft reaches the STAR entry fix. Runs after build_descent_clearance fires.
// Altitude-compliance courtesy prompt for IFR_DESCENT + IFR_ARRIVAL (the gap
// where the en-route verify and approach verify-descending don't run). Advisory
// only (no readback). See engine.cpp for the firing gates.
bool poll_altitude_compliance(const xplane_context::XPlaneContext &ctx, float dt,
                              std::string *out_text);

bool poll_descent(const xplane_context::XPlaneContext &ctx, float dt,
                  std::string *out_text,
                  bool *out_requires_readback = nullptr);

// IFR arrival phase (IFR_ARRIVAL state): aircraft flying the STAR under
// ACC/arrival control. Fires the real Approach handoff (TMA/CTR boundary or
// 50 NM fallback) -> IFR_APPROACH_CONTACT.
bool poll_arrival(const xplane_context::XPlaneContext &ctx, float dt,
                  std::string *out_text,
                  bool *out_requires_readback = nullptr);

// IFR Approach STAR constraint management (IFR_APPROACH_DESCENT state).
// Issues altitude/speed constraints fix by fix along the STAR as the aircraft
// descends, then issues the final altitude + QNH below transition altitude.
// Returns true when a message was emitted.
bool poll_approach(const xplane_context::XPlaneContext &ctx, float dt,
                   std::string *out_text,
                   bool *out_requires_readback = nullptr);

// After-FAF lateral deviation monitor (IFR_APPROACH_TOWER). Fires when
// cross-track error from extended runway centerline exceeds 0.5 NM.
bool poll_approach_alignment(const xplane_context::XPlaneContext &ctx, float dt,
                              std::string *out_text);

// Label of the last controller the pilot was handed off to (e.g. "Lyon",
// "Marseille"). Empty until the first IFR departure handoff fires.
// Used by atc_ui to show the correct controller name in the transcript.
const std::string &current_controller_label();
const std::string &pending_controller_label();

// Store a controller label without triggering a full handoff (used by
// ground_operations when the departure contact is embedded in the takeoff
// clearance — so the label is set before poll_departure_handoff() runs).
void set_controller_label(const std::string &label);

// Store the departure controller label when the takeoff clearance is built
// (ground phase). poll_departure_handoff() activates it into
// current_controller_label() so it never appears in ground-phase transcript
// entries.
void set_pending_departure_label(const std::string &label);
const std::string &pending_departure_label();

// ATC-assigned STAR name (set by build_descent_clearance, empty before then).
// Exposed so atc_session can include it in the STT pre-context on each PTT.
const std::string &assigned_star_name();

// Spoken plain-language form of the assigned STAR ("SALEV THREE PAPA") for the STT
// context bias -- matches what ATC speaks so the pilot's readback is recognised.
// Empty when no STAR is assigned.
std::string assigned_star_spoken();

// Approach IAF / FAF idents for the STT context bias (the pilot is cleared to the
// IAF and reports "established" at the FAF). Empty until an approach is assigned.
const std::string &approach_faf_ident();
const std::string &approach_iaf_ident();

// Spoken approach identity ("RNAV Zulu approach runway 04") for the STT context
// bias -- matches what ATC speaks (NATO variant word + runway). Empty when no
// approach is assigned.
std::string assigned_approach_spoken(const xplane_context::XPlaneContext &ctx);

// Idents of the UPCOMING route fixes (from the current tracker position to the
// end of the route table). This includes the CIFP STAR + approach procedure
// waypoints (e.g. GIROL, AMFOU, TIPIK, MUS on LFMN ABDI8R) that are NOT in the
// filed SimBrief navlog -- so atc_session can add them to the STT context_bias.
// Without this, a "direct <STAR fix>" readback garbles (AMFOU -> "I'm full",
// LFMN 2026-07-13). Empty until a route table is built. Ordered nearest-first.
std::vector<std::string> upcoming_route_fix_idents();

// Most recent ATC-assigned altitude in feet MSL, or 0 when none is active.
// Precedence: approach initial FL (once Approach has issued a target) >
// en-route cleared altitude (covers cruise + step-ups / step-downs) >
// SID step1 altitude (early climb). Used by atc_session to seed the STT
// context_bias with the specific "N feet" / "flight level N" tokens the
// pilot is most likely to read back — nudges Voxtral away from off-by-15
// number-conversion errors ("2000 feet" → digit token "2015").
int current_cleared_alt_ft();

// Active speed restriction (kt) in force, for the STT context bias so the pilot's
// readback digits are anchored ("210" was misheard as "110"). 0 = no restriction.
int current_speed_restriction_kt();

// Frequency the pilot was last instructed to switch to (MHz).
// Set whenever a handoff is issued (departure, TMA exit, en-route).
// Used by check_handoff_reissue() to re-state the instruction if the pilot
// calls back on the wrong frequency.
void set_pending_handoff_freq(float mhz);
float pending_handoff_freq();

// Ground runway-change poll: fires when the active runway changes while the
// aircraft is on the ground and the dialog is in an active ground state
// (GROUND_CONTACT, TAXI_CLEARED, TOWER_CONTACT, IFR_PREDEP_CLEARANCE,
// IFR_CLEARED). Announces the new runway and its holding point to the pilot.
// Returns true and writes to *out_text when an announcement was generated.
bool poll_ground_runway_change(const xplane_context::XPlaneContext &ctx,
                               std::string *out_text);

// Route fix tracker — for debugging and awareness only, no ATC speech.
// Combines OFP navlog fixes with STAR/approach waypoints.
// Each time the aircraft enters the 5 NM radius around the next fix in the
// ordered list, returns a log string: "Track: near FIX (dist NM), next: FIX2"
// Returns empty string when no new event has occurred.
// Must be called every frame from atc_session::update().
std::string poll_route_tracker(const xplane_context::XPlaneContext &ctx);

// One-shot informational note for the transcript (System line), e.g. when the
// destination weather forced a non-preferred approach. Returns the note and
// clears it; empty when there is nothing pending. Drain every frame from
// atc_session::update() alongside poll_route_tracker().
std::string take_pending_transcript_note();

} // namespace engine

#endif
