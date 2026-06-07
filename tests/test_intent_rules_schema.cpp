/*
 * xp_wellys_atc - AI-powered ATC voice communication for X-Plane 12
 * Copyright (C) 2026 thWelly & Claude (Anthropic)
 *
 * Licensed under GPL-3.0-or-later. See LICENSE.
 */

// Schema-level coverage for the new adjustment condition
// `current_state_in` / `current_state_not_in` (intent_rules.cpp).
// The EU profile uses it to demote INITIAL_CALL_* intents outside the
// initial-contact window. These tests exercise it through the EU JSON
// so we don't have to ship a synthetic JSON loader path.

#include "atc/atc_state_machine.hpp"
#include "atc/intent_parser.hpp"
#include "atc/intent_rules.hpp"
#include "core/xplane_context.hpp"
#include "persistence/settings.hpp"

#include <catch2/catch_amalgamated.hpp>

#include <string>

using atc_state_machine::ATCState;
using intent_parser::parse;
using intent_parser::PilotIntent;

namespace {

struct EuRegionGuard {
    std::string saved_profile;
    std::string saved_callsign;
    EuRegionGuard()
        : saved_profile(settings::atc_profile()),
          saved_callsign(settings::pilot_callsign()) {
        settings::set_atc_profile("EU");
        settings::set_pilot_callsign_raw("November One Seven Two Sierra Papa");
        intent_rules::reload();
    }
    ~EuRegionGuard() {
        settings::set_atc_profile(saved_profile);
        settings::set_pilot_callsign_raw(saved_callsign);
        intent_rules::reload();
    }
};

xplane_context::XPlaneContext ground_ctx() {
    xplane_context::XPlaneContext ctx;
    ctx.on_ground = true;
    ctx.is_towered_airport = true;
    return ctx;
}

} // namespace

// ── current_state_not_in: demotion fires when outside whitelist ────────

TEST_CASE("schema: INITIAL_CALL_TOWER confidence demoted outside the "
          "initial-contact window (PATTERN_ENTRY)",
          "[intent][schema][current_state_not_in]") {
    EuRegionGuard g;
    atc_state_machine::init();
    atc_state_machine::set_state(ATCState::PATTERN_ENTRY);
    auto ctx = ground_ctx();
    ctx.on_ground = false;

    auto m = parse("Bern Tower November One Seven Two Sierra Papa", ctx);
    REQUIRE(m.intent == PilotIntent::INITIAL_CALL_TOWER);
    // Base rule confidence is 0.85 — adjustment knocks it down to 0.3.
    REQUIRE(m.confidence == Catch::Approx(0.3f));
}

// ── current_state_not_in: demotion silent inside whitelist ─────────────

TEST_CASE("schema: INITIAL_CALL_TOWER confidence preserved inside the "
          "initial-contact window (IDLE)",
          "[intent][schema][current_state_not_in]") {
    EuRegionGuard g;
    atc_state_machine::init();
    atc_state_machine::set_state(ATCState::IDLE);
    auto ctx = ground_ctx();

    auto m = parse("Bern Tower November One Seven Two Sierra Papa", ctx);
    REQUIRE(m.intent == PilotIntent::INITIAL_CALL_TOWER);
    REQUIRE(m.confidence == Catch::Approx(0.85f));
}

// ── current_state_not_in: also covers TAXI_CLEARED hop ─────────────────

TEST_CASE("schema: INITIAL_CALL_TOWER confidence preserved during the "
          "tower_only auto-advance hop (TAXI_CLEARED)",
          "[intent][schema][current_state_not_in]") {
    EuRegionGuard g;
    atc_state_machine::init();
    atc_state_machine::set_state(ATCState::TAXI_CLEARED);
    auto ctx = ground_ctx();

    auto m = parse("Bern Tower November One Seven Two Sierra Papa", ctx);
    REQUIRE(m.intent == PilotIntent::INITIAL_CALL_TOWER);
    REQUIRE(m.confidence == Catch::Approx(0.85f));
}
