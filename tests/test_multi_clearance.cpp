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

// Unit tests for the multi-item ("compound") readback queue in
// atc_state_machine + readback_verifier::fields_present. Covers the LFLP
// (alpha-75) failure: ATC issues "descend 6500" then "cleared approach runway
// 04" moments apart; the pilot must be able to read them back TOGETHER or
// piecemeal, each item clearing independently, instead of the runway readback
// superseding the descent ("negative, runway zero four" on a correct descent
// read-back).

#include "atc/atc_state_machine.hpp"
#include "atc/intent_parser.hpp"
#include "atc/readback_verifier.hpp"
#include "core/xplane_context.hpp"

#include <algorithm>
#include <catch2/catch_amalgamated.hpp>
#include <string>
#include <vector>

using atc_state_machine::ATCState;
using intent_parser::PilotIntent;
using intent_parser::PilotMessage;

namespace {

xplane_context::XPlaneContext ctx_airborne() {
  xplane_context::XPlaneContext ctx;
  ctx.on_ground = false;
  ctx.is_towered_airport = true;
  return ctx;
}

PilotMessage readback(const std::string &t) {
  PilotMessage m;
  m.intent = PilotIntent::READBACK;
  m.raw_transcript = t;
  m.confidence = 1.0f;
  return m;
}

bool has(const std::vector<std::string> &v, const char *k) {
  return std::find(v.begin(), v.end(), std::string(k)) != v.end();
}

bool cache_has(const char *needle) {
  return atc_state_machine::last_clearance_text().find(needle) !=
         std::string::npos;
}

} // namespace

TEST_CASE("multi-clearance: fields_present detects each verifiable field",
          "[multi_clearance]") {
  using readback_verifier::fields_present;
  const auto d = fields_present("continue descent to 6500 feet, QNH 1019");
  REQUIRE(has(d, "alt"));
  REQUIRE_FALSE(has(d, "runway"));

  const auto r = fields_present("cleared RNAV Zulu approach runway 04");
  REQUIRE(has(r, "runway"));

  const auto s = fields_present("reduce speed, 210 knots or less");
  REQUIRE(has(s, "speed"));
}

TEST_CASE("multi-clearance: different-field clearances coexist (no supersede)",
          "[multi_clearance]") {
  atc_state_machine::init();
  atc_state_machine::set_state(ATCState::IFR_APPROACH_DESCENT);

  atc_state_machine::arm_readback("November One, descend 6500 feet, QNH 1019.");
  REQUIRE(atc_state_machine::is_readback_pending());

  // The runway clearance must NOT wipe the pending descent (the alpha-75 bug).
  atc_state_machine::arm_readback(
      "November One, cleared RNAV Zulu approach runway 04.");
  REQUIRE(atc_state_machine::is_readback_pending());
  REQUIRE(cache_has("6500"));
  REQUIRE(cache_has("runway 04"));
}

TEST_CASE("multi-clearance: same-field clearance replaces (latest wins)",
          "[multi_clearance]") {
  atc_state_machine::init();
  atc_state_machine::set_state(ATCState::IFR_APPROACH_DESCENT);

  atc_state_machine::arm_readback("November One, descend flight level 140.");
  atc_state_machine::arm_readback("November One, descend 6500 feet, QNH 1019.");
  // Only the latest altitude survives -- the FL140 item was replaced.
  REQUIRE(cache_has("6500"));
  REQUIRE_FALSE(cache_has("140"));
}

TEST_CASE("multi-clearance: piecemeal readback clears items independently",
          "[multi_clearance]") {
  auto ctx = ctx_airborne();
  atc_state_machine::init();
  atc_state_machine::set_state(ATCState::IFR_APPROACH_DESCENT);
  atc_state_machine::arm_readback("November One, descend 6500 feet, QNH 1019.");
  atc_state_machine::arm_readback(
      "November One, cleared RNAV Zulu approach runway 04.");

  // Read back the descent only -> alt clears, runway still owed.
  atc_state_machine::process(readback("descend 6500 feet"), ctx, 10.0);
  REQUIRE(atc_state_machine::is_readback_pending());
  REQUIRE(cache_has("runway 04"));
  REQUIRE_FALSE(cache_has("6500"));

  // Then read back the runway -> fully cleared.
  atc_state_machine::process(readback("runway 04"), ctx, 20.0);
  REQUIRE_FALSE(atc_state_machine::is_readback_pending());
}

TEST_CASE("multi-clearance: both items read back in one transmission",
          "[multi_clearance]") {
  auto ctx = ctx_airborne();
  atc_state_machine::init();
  atc_state_machine::set_state(ATCState::IFR_APPROACH_DESCENT);
  atc_state_machine::arm_readback("November One, descend 6500 feet, QNH 1019.");
  atc_state_machine::arm_readback(
      "November One, cleared RNAV Zulu approach runway 04.");

  atc_state_machine::process(readback("descend 6500 feet, runway 04"), ctx, 10.0);
  REQUIRE_FALSE(atc_state_machine::is_readback_pending());
}
