/*
 * xp_wellys_atc - AI-powered ATC voice communication for X-Plane 12
 * Copyright (C) 2026 thWelly & Claude (Anthropic)
 *
 * Licensed under GPL-3.0-or-later. See LICENSE.
 */

// EU-profile intent classification fixes for the TOWER_CONTACT
// deadlock observed at LSZB (log 2026-06-07): pilot's "Bern Tower
// N172SP ready for departure" was misclassified as INITIAL_CALL_TOWER
// because the rule was a bare has_facility("tower") match and Whisper
// occasionally drops a letter ("depature" instead of "departure"),
// blinding the READY_FOR_DEPARTURE rule. Three layers of defence:
//   1. normalize patches the typo
//   2. INITIAL_CALL_TOWER none-clause vetoes follow-up tokens
//   3. current_state_not_in adjustment drops INITIAL_CALL_* confidence
//      outside the initial-contact window (IDLE/GROUND_CONTACT/TAXI_CLEARED)

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

// RAII guard: switches atc_profile to "EU" + sets a EU-style pilot
// callsign + forces intent_rules to reload from the EU JSON for the
// test case, then restores both. Mirrors DeRegionGuard in
// test_intent_parser_de.cpp / test_intent_rules_runway_vacated_veto.cpp.
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

// ── 1. The LSZB deadlock case ───────────────────────────────────────

TEST_CASE("eu-tower-contact: 'Bern Tower ... ready for departure' classifies "
          "as READY_FOR_DEPARTURE, not INITIAL_CALL_TOWER",
          "[intent][eu][tower_contact][ready_for_departure]") {
    EuRegionGuard g;
    atc_state_machine::init();
    atc_state_machine::set_state(ATCState::TOWER_CONTACT);
    auto ctx = ground_ctx();

    auto m = parse(
        "Bern Tower November One Seven Two Sierra Papa runway 14 ready for departure",
        ctx);
    REQUIRE((m.intent == PilotIntent::READY_FOR_DEPARTURE ||
             m.intent == PilotIntent::READY_FOR_DEPARTURE_VFR));
    REQUIRE(m.intent != PilotIntent::INITIAL_CALL_TOWER);
}

TEST_CASE("eu-tower-contact: Whisper typo 'depature' is normalized to "
          "'departure' before rule matching",
          "[intent][eu][normalize]") {
    EuRegionGuard g;
    atc_state_machine::init();
    atc_state_machine::set_state(ATCState::TOWER_CONTACT);
    auto ctx = ground_ctx();

    // Verbatim transcript from LSZB log 2026-06-07.
    auto m = parse(
        "Bern Tower November One Seven Two Sierra Papa short runway 14 ready for depature",
        ctx);
    REQUIRE((m.intent == PilotIntent::READY_FOR_DEPARTURE ||
             m.intent == PilotIntent::READY_FOR_DEPARTURE_VFR));
}

// ── 2. Regression checks (must stay green) ──────────────────────────

TEST_CASE("eu-initial-contact: pure 'Bern Tower N172SP' in IDLE still "
          "classifies as INITIAL_CALL_TOWER",
          "[intent][eu][initial_call]") {
    EuRegionGuard g;
    atc_state_machine::init();
    atc_state_machine::set_state(ATCState::IDLE);
    auto ctx = ground_ctx();

    auto m = parse("Bern Tower November One Seven Two Sierra Papa", ctx);
    REQUIRE(m.intent == PilotIntent::INITIAL_CALL_TOWER);
}

TEST_CASE("eu-initial-contact: inbound call still routes to INITIAL_CALL_INBOUND "
          "(order against new INITIAL_CALL_TOWER none-clause)",
          "[intent][eu][initial_call_inbound]") {
    EuRegionGuard g;
    atc_state_machine::init();
    atc_state_machine::set_state(ATCState::IDLE);
    auto ctx = ground_ctx();
    ctx.on_ground = false; // inbound = airborne

    auto m = parse(
        "Bern Tower November One Seven Two Sierra Papa inbound for landing", ctx);
    REQUIRE(m.intent == PilotIntent::INITIAL_CALL_INBOUND);
}
