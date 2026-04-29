#include "atc/intent_parser.hpp"

#include <catch2/catch_amalgamated.hpp>

#include <string>

using intent_parser::intent_from_key;
using intent_parser::intent_name;
using intent_parser::intent_template_key;
using intent_parser::PilotIntent;

// ── Enum ↔ string mappings ───────────────────────────────────────────────────

TEST_CASE("intent_name: known intents return their canonical key", "[intent][name]")
{
    REQUIRE(std::string(intent_name(PilotIntent::UNKNOWN)) == "UNKNOWN");
    REQUIRE(std::string(intent_name(PilotIntent::RADIO_CHECK)) == "RADIO_CHECK");
    REQUIRE(std::string(intent_name(PilotIntent::REQUEST_TAXI)) == "REQUEST_TAXI");
    REQUIRE(std::string(intent_name(PilotIntent::REPORT_POSITION_FINAL)) == "REPORT_POSITION_FINAL");
}

// intent_template_key collapses the generic INITIAL_CALL onto the tower variant
// so JSON template lookup always resolves to a concrete template.
TEST_CASE("intent_template_key: generic INITIAL_CALL maps to tower variant", "[intent][template]")
{
    REQUIRE(std::string(intent_template_key(PilotIntent::INITIAL_CALL)) == "INITIAL_CALL_TOWER");
}

TEST_CASE("intent_template_key: sub-variants pass through unchanged", "[intent][template]")
{
    REQUIRE(std::string(intent_template_key(PilotIntent::INITIAL_CALL_GROUND)) == "INITIAL_CALL_GROUND");
    REQUIRE(std::string(intent_template_key(PilotIntent::REPORT_POSITION_DOWNWIND)) == "REPORT_POSITION_DOWNWIND");
}

TEST_CASE("intent_from_key: round-trips intent_name for known keys", "[intent][from_key]")
{
    REQUIRE(intent_from_key("RADIO_CHECK") == PilotIntent::RADIO_CHECK);
    REQUIRE(intent_from_key("REQUEST_TAXI") == PilotIntent::REQUEST_TAXI);
    REQUIRE(intent_from_key("GO_AROUND") == PilotIntent::GO_AROUND);
}

TEST_CASE("intent_from_key: unknown keys fall back to UNKNOWN", "[intent][from_key]")
{
    REQUIRE(intent_from_key("") == PilotIntent::UNKNOWN);
    REQUIRE(intent_from_key("NOT_A_REAL_INTENT") == PilotIntent::UNKNOWN);
}
