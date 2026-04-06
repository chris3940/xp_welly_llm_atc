/*
 * xp_wellys_atc - AI-powered ATC voice communication for X-Plane 12
 * Copyright (C) 2026 thWelly & Claude (Anthropic)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <https://www.gnu.org/licenses/>.
 */

#include "atc_state_machine.hpp"
#include "atc_templates.hpp"
#include "settings.hpp"

#include <XPLMUtilities.h>

#include <cmath>
#include <cstdio>

namespace atc_state_machine {

static ATCState state_ = ATCState::IDLE;

// Helper: get callsign from message or settings fallback
static std::string get_callsign(const intent_parser::PilotMessage &msg) {
  if (!msg.callsign.empty())
    return msg.callsign;
  return settings::pilot_callsign();
}

// Helper: get runway from message or fallback
static std::string get_runway(const intent_parser::PilotMessage &msg) {
  if (!msg.runway.empty())
    return msg.runway;
  return "28"; // default placeholder
}

// Helper: format QNH from inHg
static std::string format_qnh(float inhg) {
  int hpa = static_cast<int>(std::round(inhg * 33.8639f));
  char buf[32];
  std::snprintf(buf, sizeof(buf), "%d", hpa);
  return buf;
}

// Helper: format wind
static std::string format_wind(float dir, float spd) {
  char buf[64];
  std::snprintf(buf, sizeof(buf), "%03.0f degrees %02.0f knots", dir, spd);
  return buf;
}

// Helper: airport name from ICAO (just return the ID for now)
static std::string airport_name(const xplane_context::XPlaneContext &ctx) {
  if (!ctx.nearest_airport_id.empty())
    return ctx.nearest_airport_id;
  return "Airport";
}

void init() { state_ = ATCState::IDLE; }

void stop() { state_ = ATCState::IDLE; }

void reset() {
  state_ = ATCState::IDLE;
  XPLMDebugString("[xp_wellys_atc] ATC state machine reset to IDLE\n");
}

ATCState get_state() { return state_; }

const char *state_name(ATCState state) {
  switch (state) {
  case ATCState::IDLE:
    return "IDLE";
  case ATCState::GROUND_CONTACT:
    return "GROUND_CONTACT";
  case ATCState::TAXI_CLEARED:
    return "TAXI_CLEARED";
  case ATCState::TOWER_CONTACT:
    return "TOWER_CONTACT";
  case ATCState::DEPARTURE_CLEARED:
    return "DEPARTURE_CLEARED";
  case ATCState::PATTERN_ENTRY:
    return "PATTERN_ENTRY";
  case ATCState::LANDING_CLEARED:
    return "LANDING_CLEARED";
  case ATCState::UNICOM_ACTIVE:
    return "UNICOM_ACTIVE";
  }
  return "UNKNOWN";
}

ATCState state_from_name(const std::string &name) {
  if (name == "IDLE")
    return ATCState::IDLE;
  if (name == "GROUND_CONTACT")
    return ATCState::GROUND_CONTACT;
  if (name == "TAXI_CLEARED")
    return ATCState::TAXI_CLEARED;
  if (name == "TOWER_CONTACT")
    return ATCState::TOWER_CONTACT;
  if (name == "DEPARTURE_CLEARED")
    return ATCState::DEPARTURE_CLEARED;
  if (name == "PATTERN_ENTRY")
    return ATCState::PATTERN_ENTRY;
  if (name == "LANDING_CLEARED")
    return ATCState::LANDING_CLEARED;
  if (name == "UNICOM_ACTIVE")
    return ATCState::UNICOM_ACTIVE;
  return ATCState::IDLE;
}

void set_state(ATCState state) {
  char log[256];
  std::snprintf(log, sizeof(log),
                "[xp_wellys_atc] ATC state (external): %s -> %s\n",
                state_name(state_), state_name(state));
  XPLMDebugString(log);
  state_ = state;
}

static std::string extract_position(const intent_parser::PilotMessage &msg,
                                    const xplane_context::XPlaneContext &ctx) {
  std::string rwy = get_runway(msg);
  std::string apt = airport_name(ctx);

  if (ctx.on_ground)
    return "on the ground at " + apt;

  std::string lower = msg.raw_transcript;
  for (auto &c : lower)
    c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));

  if (lower.find("downwind") != std::string::npos)
    return "downwind runway " + rwy;
  if (lower.find("base") != std::string::npos)
    return "base runway " + rwy;
  if (lower.find("final") != std::string::npos)
    return "final runway " + rwy;
  if (lower.find("crosswind") != std::string::npos)
    return "crosswind runway " + rwy;
  if (lower.find("upwind") != std::string::npos)
    return "upwind runway " + rwy;
  return "in the pattern at " + apt;
}

std::map<std::string, std::string>
build_vars(const intent_parser::PilotMessage &msg,
           const xplane_context::XPlaneContext &ctx) {
  return {
      {"callsign", get_callsign(msg)},
      {"airport", airport_name(ctx)},
      {"runway", get_runway(msg)},
      {"wind", format_wind(ctx.wind_direction_deg, ctx.wind_speed_kt)},
      {"qnh", format_qnh(ctx.qnh_inhg)},
      {"frequency", "121.9"}, // TODO: derive from xplane_context
      {"position", extract_position(msg, ctx)},
  };
}

ATCResponse process(const intent_parser::PilotMessage &msg,
                    const xplane_context::XPlaneContext &ctx) {
  ATCResponse resp;

  using FT = xplane_context::FrequencyType;

  // Non-towered airport OR Unicom/CTAF frequency: force UNICOM flow
  bool unicom_flow = !ctx.is_towered_airport ||
                     ctx.frequency_type == FT::UNICOM ||
                     ctx.frequency_type == FT::CTAF;

  if (unicom_flow) {
    auto vars = build_vars(msg, ctx);
    std::string intent_key = intent_parser::intent_template_key(msg.intent);
    auto tmpl = atc_templates::lookup(false, "IDLE", intent_key);

    resp.text = atc_templates::fill(tmpl.response_template, vars);
    resp.next_state = ATCState::IDLE;
    state_ = ATCState::IDLE;

    XPLMDebugString("[xp_wellys_atc] ATC state: UNICOM_ACTIVE -> IDLE "
                    "(non-towered/CTAF)\n");
    return resp;
  }

  // Frequency-based state validation at towered airports
  if (ctx.frequency_type == FT::GROUND && state_ != ATCState::IDLE &&
      state_ != ATCState::GROUND_CONTACT && state_ != ATCState::TAXI_CLEARED) {
    state_ = ATCState::IDLE;
  }
  if (ctx.frequency_type == FT::TOWER && state_ != ATCState::IDLE &&
      state_ != ATCState::TOWER_CONTACT &&
      state_ != ATCState::DEPARTURE_CLEARED &&
      state_ != ATCState::PATTERN_ENTRY &&
      state_ != ATCState::LANDING_CLEARED) {
    state_ = ATCState::IDLE;
  }

  // Template-based response lookup
  auto vars = build_vars(msg, ctx);
  std::string intent_key = intent_parser::intent_template_key(msg.intent);
  std::string state_str = state_name(state_);

  auto tmpl = atc_templates::lookup(true, state_str, intent_key);
  resp.text = atc_templates::fill(tmpl.response_template, vars);
  resp.next_state = state_from_name(tmpl.next_state);
  resp.requires_readback = tmpl.requires_readback;

  // Apply state transition if we have a response
  if (!resp.text.empty()) {
    char log[256];
    std::snprintf(log, sizeof(log), "[xp_wellys_atc] ATC state: %s -> %s\n",
                  state_name(state_), state_name(resp.next_state));
    XPLMDebugString(log);
    state_ = resp.next_state;
  }

  return resp;
}

} // namespace atc_state_machine
