/*
 * xp_wellys_atc - AI-powered ATC voice communication for X-Plane 12
 * Copyright (C) 2026 thWelly & Claude (Anthropic)
 * Copyright (C) 2026 Christopher P. Potter (Linux port + IFR extensions)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

// EVERY INTENT WAS REFUSED ON AN AFIS FREQUENCY.
//
// The frequency guard tables were written for towered fields: GROUND, TOWER,
// DELIVERY, APPROACH. The AFIS "Information" service was added long after, and
// INFO was never put in a single one of the 32 allowed-lists. The result, at
// LFLU Valence on 2026-09-09: the pilot tunes Valence Information, reads back
// correctly, and hears "November Romeo Charlie, unable" -- the log naming the
// cause outright, "Frequency guard: READBACK blocked on freq_type 9".
//
// At a field with no ATC there is one frequency and one officer holding every
// position, so INFO now satisfies a GROUND or TOWER requirement. [C. P. Potter]

#include <catch2/catch_amalgamated.hpp>

#include "atc/flight_phase.hpp"
#include "core/xplane_context.hpp"

using FT = xplane_context::FrequencyType;

TEST_CASE("an AFIS Information frequency satisfies the GROUND/TOWER guards",
          "[afis][frequency][guard]") {
  flight_phase::init();

  // The exact refusal from the flight: a read-back on Valence Information.
  REQUIRE(flight_phase::check_frequency_precondition("READBACK", FT::INFO)
              .empty());

  // The whole AFIS departure sequence runs on that one frequency.
  for (const char *k : {"REQUEST_IFR_CLEARANCE", "REQUEST_STARTUP",
                        "REQUEST_TAXI", "REPORT_HOLDING_SHORT",
                        "READY_FOR_DEPARTURE", "RADIO_CHECK",
                        "REQUEST_TAXI_PARKING", "RUNWAY_VACATED",
                        "REQUEST_LANDING", "REQUEST_FREQUENCY"}) {
    INFO(k);
    REQUIRE(flight_phase::check_frequency_precondition(k, FT::INFO).empty());
  }

  // GUARD: the substitution does not reach UNICOM/CTAF intents -- self-announce
  // on an AFIS frequency is a different thing and nobody answers it.
  REQUIRE_FALSE(
      flight_phase::check_frequency_precondition("SELF_ANNOUNCE", FT::INFO)
          .empty());

  // GUARD: a genuinely wrong frequency is still refused. A taxi request on the
  // ATIS frequency is an error at any field.
  REQUIRE_FALSE(
      flight_phase::check_frequency_precondition("REQUEST_TAXI", FT::ATIS)
          .empty());
}

// AN UNIDENTIFIED FREQUENCY IS NOT A WRONG ONE.
//
// An en-route centre belongs to no airport (Lyon Control 125.155 matches
// nothing in LFLU's list), and a field with no scenery pack has no frequency
// list at all. Both land on freq_type UNKNOWN, and the guard refused: a
// correct read-back of the Lyon handoff drew "unable" (user 2026-09-10, log:
// "Frequency guard: READBACK blocked on freq_type 0"). [C. P. Potter]
TEST_CASE("an unidentified frequency does not refuse the transmission",
          "[frequency][guard]") {
  flight_phase::init();

  for (const char *k : {"READBACK", "REQUEST_IFR_CLEARANCE", "RADIO_CHECK",
                        "REQUEST_TAXI", "READY_FOR_DEPARTURE",
                        "REQUEST_FREQUENCY", "LEAVING_FREQUENCY"}) {
    INFO(k);
    REQUIRE(flight_phase::check_frequency_precondition(k, FT::UNKNOWN).empty());
  }

  // GUARD: an IDENTIFIED but wrong frequency is still refused -- the guard is
  // narrowed to ignorance, not disabled.
  REQUIRE_FALSE(
      flight_phase::check_frequency_precondition("REQUEST_TAXI", FT::ATIS)
          .empty());
  REQUIRE_FALSE(
      flight_phase::check_frequency_precondition("SELF_ANNOUNCE", FT::TOWER)
          .empty());
}
