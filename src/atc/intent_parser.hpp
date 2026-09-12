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

#ifndef INTENT_PARSER_HPP
#define INTENT_PARSER_HPP

#include "core/xplane_context.hpp"

#include <string>

namespace intent_parser {

enum class PilotIntent {
  UNKNOWN,
  RADIO_CHECK,
  INITIAL_CALL,
  INITIAL_CALL_GROUND,
  INITIAL_CALL_TOWER,
  INITIAL_CALL_INBOUND,
  INITIAL_CALL_INBOUND_VRP,
  INITIAL_CALL_APPROACH,
  REQUEST_TAXI,
  REQUEST_TAXI_PARKING,
  // "backtrack runway 19" -- routine at a field whose only taxiway is the runway
  // itself. It drew "unable" at Valence (2026-09-08) because no intent existed.
  REQUEST_BACKTRACK,
  REPORT_TAKING_OFF,
  READY_FOR_DEPARTURE,
  READY_FOR_DEPARTURE_VFR,
  REPORT_POSITION,
  REPORT_POSITION_DOWNWIND,
  REPORT_POSITION_BASE,
  REPORT_POSITION_FINAL,
  REQUEST_LANDING,
  REQUEST_TOUCH_AND_GO,
  GO_AROUND,
  RUNWAY_VACATED,
  READBACK,
  REQUEST_FREQUENCY,
  LEAVING_FREQUENCY,
  UNABLE,
  SELF_ANNOUNCE,
  REQUEST_FLIGHT_FOLLOWING,
  INAPPROPRIATE_LANGUAGE,
  NEGATIVE_CORRECTION,
  TRAFFIC_IN_SIGHT,
  TRAFFIC_NEGATIVE_CONTACT,
  TRAFFIC_LOOKING,
  REQUEST_REPEAT, // "SAY AGAIN"
  // IFR Phase 4
  REQUEST_IFR_CLEARANCE, // "request IFR clearance to [destination]"
  REQUEST_STARTUP,       // "request startup [and pushback]"
  REPORT_HOLDING_SHORT,  // "holding short runway X" / "at holding point" — IFR
                         // Tower call
  INITIAL_CALL_CENTER,   // en-route initial call to Area Control / UIR Centre
                         // ("Control", "Centre")
  REQUEST_DESCENT,       // "request descent" / "ready to descend" — IFR en-route
  REQUEST_HIGHER,        // "request higher" / "for higher" — IFR en-route climb request
};

struct PilotMessage {
  std::string raw_transcript;
  PilotIntent intent = PilotIntent::UNKNOWN;
  float confidence = 0.0f;
  std::string callsign;
  std::string runway;
  std::string vrp_name;      // canonical VRP name if detected ("Whiskey")
  bool has_position = false; // pilot reported position (e.g. "on parking")
};

void init();
void stop();

PilotMessage parse(const std::string &transcript,
                   const xplane_context::XPlaneContext &ctx);

const char *intent_name(PilotIntent intent);
const char *intent_template_key(PilotIntent intent);
PilotIntent intent_from_key(const std::string &key);

// Collapse any SPOKEN frequency inside `text` to the compact digit form so the
// digit-based rules recognise every read-back style: digits ("125.630"), spelled
// digit-by-digit ("one two five decimal six three zero"), cardinal ("one hundred
// twenty five decimal two hundred"), and the pilot-abbreviated forms that DROP the
// leading "1" of the 1xx MHz band ("two five decimal ...", "twenty five decimal
// ...") -- a whole part below 100 is lifted into the 118-136 band. Anchored on the
// word "decimal", which appears only in a frequency, so spelled callsign digits
// ("November Seven Five Zero ...") are never touched. Non-frequency text passes
// through unchanged. Exposed for unit testing. (user 2026-07-26)
std::string normalize_spoken_frequency(const std::string &text);

} // namespace intent_parser

#endif // INTENT_PARSER_HPP
