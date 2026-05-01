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

#ifndef ATC_STATE_MACHINE_HPP
#define ATC_STATE_MACHINE_HPP

#include "atc/flight_phase.hpp"
#include "atc/intent_parser.hpp"
#include "core/xplane_context.hpp"

#include <cstdint>
#include <map>
#include <string>

namespace atc_state_machine {

enum class ATCState {
  IDLE,
  GROUND_CONTACT,
  TAXI_CLEARED,
  TOWER_CONTACT,
  DEPARTURE_CLEARED,
  PATTERN_ENTRY,
  LANDING_CLEARED,
  TOUCH_AND_GO_CLEARED,
  UNICOM_ACTIVE,
  EN_ROUTE,
  APPROACH_CONTACT,
  // Phase-2 peer state. Entered when the controller issues a traffic
  // advisory (synthetic, not pilot-driven). On pilot acknowledgement
  // (TRAFFIC_IN_SIGHT / TRAFFIC_NEGATIVE_CONTACT / TRAFFIC_LOOKING) the
  // state restores `previous_state_`. See traffic_advisor.hpp.
  TRAFFIC_ADVISORY_PENDING,
};

struct ATCResponse {
  std::string text;
  ATCState next_state = ATCState::IDLE;
  bool requires_readback = false;
};

void init();
void stop();
void reset();

ATCState get_state();
const char *state_name(ATCState state);
bool is_readback_pending();

// Departure intent (set when entering DEPARTURE_CLEARED).
// Returns "PATTERN" or "CROSS_COUNTRY".
const char *get_departure_type_name();

ATCState state_from_name(const std::string &name);
void set_state(ATCState state);

std::map<std::string, std::string>
build_vars(const intent_parser::PilotMessage &msg,
           const xplane_context::XPlaneContext &ctx);

ATCResponse process(const intent_parser::PilotMessage &msg,
                    const xplane_context::XPlaneContext &ctx);

// Check and apply auto-corrections based on flight phase mismatches.
// Call every frame from atc_session::update(). Uses dt for delay timers.
void check_auto_correction(flight_phase::FlightPhase phase, float dt);

// Synthetic dispatch: enter TRAFFIC_ADVISORY_PENDING and render the
// initial advisory text. Used by the per-tick advisor to push an
// advisory through the same template/render path a pilot intent would
// take, but without going through intent_parser. `target_modeS_id` is
// stashed so a later TRAFFIC_NEGATIVE_CONTACT / TRAFFIC_LOOKING reply
// can re-issue with updated geometry. Returns the rendered text.
std::string
emit_traffic_advisory(uint32_t target_modeS_id,
                      const std::map<std::string, std::string> &advisory_vars,
                      const xplane_context::XPlaneContext &ctx);

} // namespace atc_state_machine

#endif // ATC_STATE_MACHINE_HPP
