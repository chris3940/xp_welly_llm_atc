#include "atc/intent_parser.hpp"
#include "atc/intent_rules.hpp"
#include "core/xplane_context.hpp"

#include <catch2/catch_amalgamated.hpp>

#include <string>

using intent_parser::intent_from_key;
using intent_parser::intent_name;
using intent_parser::intent_template_key;
using intent_parser::parse;
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

// Helper: minimal airborne ctx (no airport context required for facility-keyword
// classification tests).
static xplane_context::XPlaneContext airborne_ctx()
{
    xplane_context::XPlaneContext ctx;
    ctx.on_ground = false;
    ctx.is_towered_airport = true;
    return ctx;
}

// Whisper occasionally prefixes the first utterance with a dash ("-Tower, ...").
// `match_request_landing` excludes transcripts that mention "tower" so the
// initial inbound call routes to INITIAL_CALL_INBOUND. The exclusion broke
// when `has_facility_keyword` only handled clean word boundaries — leading
// punctuation made "tower" invisible to the guard, and the call was
// misclassified as REQUEST_LANDING (which then hits _INVALID from IDLE).
TEST_CASE("parse: leading-dash 'Tower' still routes to INITIAL_CALL_INBOUND", "[intent][parse]")
{
    auto ctx = airborne_ctx();

    auto m1 = parse("-Tower, Hotel Bravo Delta Charlie Hotel inbound runway 06, request full stop landing.", ctx);
    REQUIRE(m1.intent == PilotIntent::INITIAL_CALL_INBOUND);

    auto m2 = parse("Tower, Hotel Bravo Delta Charlie Hotel inbound runway 06, request full stop landing.", ctx);
    REQUIRE(m2.intent == PilotIntent::INITIAL_CALL_INBOUND);

    // Without facility keyword the same body should route to REQUEST_LANDING.
    auto m3 = parse("Hotel Bravo Delta Charlie Hotel inbound runway 06, request full stop landing.", ctx);
    REQUIRE(m3.intent == PilotIntent::REQUEST_LANDING);
}

TEST_CASE("parse: facility keyword is punctuation-tolerant", "[intent][parse]")
{
    auto ctx = airborne_ctx();
    ctx.on_ground = true;

    // Leading-comma artifact still detects "ground" as facility.
    auto m = parse(",Ground, Hotel Bravo Delta Charlie Hotel at parking, request taxi runway 14.", ctx);
    REQUIRE((m.intent == PilotIntent::INITIAL_CALL_GROUND ||
             m.intent == PilotIntent::REQUEST_TAXI));
}

// Pilot's clearance readback after Approach/Tower issues "cleared into the
// control zone, runway X, joining instructions ..." must classify as
// READBACK rather than UNKNOWN. Whisper sometimes drops the leading word
// (logged as "-Control-Zone runway 32 Delta Charlie Hotel") and without
// these patterns the rule parser landed on UNKNOWN, letting the LM
// classify the readback as REPORT_POSITION_DOWNWIND.
TEST_CASE("parse: control zone clearance readback classifies as READBACK", "[intent][parse][readback]")
{
    auto ctx = airborne_ctx();

    auto m1 = parse("Cleared into the control zone runway 32 Delta Charlie Hotel", ctx);
    REQUIRE(m1.intent == PilotIntent::READBACK);

    auto m2 = parse("-Control-Zone runway 32 Delta Charlie Hotel", ctx);
    REQUIRE(m2.intent == PilotIntent::READBACK);

    auto m3 = parse("Joining instructions, runway 32, QNH 1013, Delta Charlie Hotel", ctx);
    REQUIRE(m3.intent == PilotIntent::READBACK);
}

TEST_CASE("normalize_spoken_frequency: all read-back styles collapse to 125.630",
          "[intent][frequency]") {
  using intent_parser::normalize_spoken_frequency;
  auto only = [](const std::string &t) {
    // return the collapsed frequency token embedded in the phrase
    return normalize_spoken_frequency(t);
  };
  // Digits pass through untouched (no "decimal" word).
  REQUIRE(only("contact milan on 125.630") == "contact milan on 125.630");
  // Spelled digit-by-digit.
  REQUIRE(only("contact milan on one two five decimal six three zero") ==
          "contact milan on 125.630");
  // Cardinal.
  REQUIRE(only("contact milan on one hundred twenty five decimal six three zero") ==
          "contact milan on 125.630");
  REQUIRE(only("one hundred twenty decimal two hundred") == "120.200");
  // Dropped leading "1" (1xx band) -- digit-by-digit and cardinal.
  REQUIRE(only("two five decimal six three zero") == "125.630");
  REQUIRE(only("twenty five decimal six three zero") == "125.630");
  // Frac leading zero preserved (digit-by-digit).
  REQUIRE(only("one one eight decimal zero five zero") == "118.050");
  // A spelled callsign is NEVER touched (no "decimal").
  REQUIRE(only("november seven five zero x-ray papa") ==
          "november seven five zero x-ray papa");
  // A non-frequency number + decimal outside the VHF band is left alone.
  REQUIRE(only("flight level two three zero") == "flight level two three zero");
  // FULLY hyphenated, INCLUDING the separator, and with the trailing comma the
  // transcription leaves on the last word. Exactly as flown on 2026-08-25:
  // "And over on one-two-zero-decimal-eight-six-zero, November, Romeo, Charlie."
  // -- a correct readback of 120.860 that the verifier reported as
  // `field=freq expected=120.860 stated=(missing)` and answered "negative".
  REQUIRE(only("and over on one-two-zero-decimal-eight-six-zero, november, "
               "romeo, charlie.").find("120.860") != std::string::npos);
  // HYPHENATED digit strings. Voxtral punctuates them arbitrarily -- the same
  // pilot said the same frequency both ways on one flight, and only the spaced
  // form was understood, so a correct readback drew "negative, I say again"
  // (real flight 2026-08-17).
  REQUIRE(only("contact langen on one-two-five decimal-zero-zero-zero") ==
          "contact langen on 125.000");
  REQUIRE(only("one-two-five decimal zero zero zero") == "125.000");
  REQUIRE(only("one two five decimal-six-three-zero") == "125.630");
  // ... but a hyphen that is NOT joining number-words must survive untouched:
  // splitting it would turn "x-ray" into "x ray" and corrupt every callsign.
  REQUIRE(only("november seven five zero x-ray papa") ==
          "november seven five zero x-ray papa");
  REQUIRE(only("cleared for the r-nav approach") == "cleared for the r-nav approach");
  // US "point" / "period" separators also collapse; "holding point" is NOT a freq.
  REQUIRE(only("one two five point six three zero") == "125.630");
  REQUIRE(only("one two five period six three zero") == "125.630");
  REQUIRE(only("taxi to holding point charlie") == "taxi to holding point charlie");
}

// Every transmission of the LFMN -> LOWI flight of 2026-08-28, run back through
// the rules. Two things it found. [C. P. Potter]
TEST_CASE("a parallel-runway designator is not a vacated report",
          "[intent][runway_vacated]") {
  intent_parser::init();
  xplane_context::XPlaneContext ctx;
  ctx.nearest_airport_id = "LFMN";
  ctx.active_runway = "04R";
  ctx.on_ground = true;

  // The rule matched the two loose words "left" + "runway", so at a field with
  // parallel runways EVERY transmission naming one fired RUNWAY_VACATED at 0.90
  // and bypassed the LM. The real flight read back its taxi clearance and was
  // recorded as having vacated a runway it had not yet reached.
  REQUIRE(intent_parser::parse(
              "taxi to holding point Charlie 1 November runway zero four left "
              "November Romeo Charlie",
              ctx)
              .intent != intent_parser::PilotIntent::RUNWAY_VACATED);
  REQUIRE(intent_parser::parse(
              "Lined up runway zero four left, November Romeo Charlie.", ctx)
              .intent != intent_parser::PilotIntent::RUNWAY_VACATED);

  // A real vacated report still fires -- it says the word.
  REQUIRE(intent_parser::parse(
              "November Romeo Charlie, runway zero four left vacated.", ctx)
              .intent == intent_parser::PilotIntent::RUNWAY_VACATED);
  // And so does the phrase the branch actually exists for.
  REQUIRE(intent_parser::parse("we have left runway 22, November Romeo Charlie",
                               ctx)
              .intent == intent_parser::PilotIntent::RUNWAY_VACATED);
}

TEST_CASE("Voxtral mishearings of 2026-08-28 are repaired", "[intent][stt]") {
  intent_parser::init();
  struct Case { const char *heard, *want; };
  const Case cases[] = {
      {"november room, charlie", "november romeo charlie"},
      {"climb flight level 450 november, rome, charlie.", "romeo charlie"},
      // read as TRAFFIC_NEGATIVE_CONTACT by the LM, silencing the reply
      {"en route to rtt november, no charlie.", "november romeo charlie"},
      {"direct hit cap, when able", "itcap"},
      {"descend via nedit 2 alpha rival to flight level 190", "nanit"},
      {"descend via nedit 2 alpha rival to flight level 190", "arrival to"},
      {"expect r90 approach runway 26", "rnav approach"},
      {"climb being to flight level 100", "climbing to"},
      {"one one eight decimal four'eight zero", "four eight zero"},
  };
  for (const auto &c : cases) {
    const std::string out = intent_rules::preprocess(c.heard);
    INFO(c.heard << "  ->  " << out);
    REQUIRE(out.find(c.want) != std::string::npos);
  }
}

// The single most common IFR transmission must not need a network round-trip.
//
// Flight of 2026-08-29: the cloud LM answered HTTP 403 "tier_not_allowed" on
// every call, and because no RULE covered a bare level readback, five textbook
// readbacks of "climb flight level 110" in a row drew "say again, use standard
// phraseology" while the aircraft climbed correctly. [C. P. Potter]
TEST_CASE("a bare level readback is recognised without the LM",
          "[intent][readback][level]") {
  intent_parser::init();
  xplane_context::XPlaneContext ctx;
  ctx.nearest_airport_id = "LFMN";
  ctx.active_runway = "04R";
  ctx.on_ground = false;
  using I = intent_parser::PilotIntent;

  // Every form the pilot actually used, in order, before giving up.
  for (const char *t : {"Climbing flight level 110, November Charlie.",
                        "Climbing flight level 110, November, Romeo, Charlie.",
                        "Climb flight level 110 November, Romeo Charlie.",
                        "Climb flight level 110 November"}) {
    const auto m = intent_parser::parse(t, ctx);
    INFO(t);
    REQUIRE(m.intent == I::READBACK);
    REQUIRE(m.confidence >= 0.9f);
  }
  // The other levels, and feet with a QNH.
  REQUIRE(intent_parser::parse("Descend flight level 150, November Romeo Charlie.", ctx)
              .intent == I::READBACK);
  REQUIRE(intent_parser::parse("Maintain flight level 190, November Romeo Charlie.", ctx)
              .intent == I::READBACK);
  REQUIRE(intent_parser::parse("Descend 3000 feet, QNH 1013, November Romeo Charlie.", ctx)
              .intent == I::READBACK);
}

TEST_CASE("the level branch does not swallow requests or reports",
          "[intent][readback][level]") {
  intent_parser::init();
  xplane_context::XPlaneContext ctx;
  ctx.nearest_airport_id = "LFMN";
  ctx.on_ground = false;
  using I = intent_parser::PilotIntent;

  // A request is not a readback.
  REQUIRE(intent_parser::parse("Request descent, November Romeo Charlie.", ctx)
              .intent != I::READBACK);
  REQUIRE(intent_parser::parse("Request flight level 200, November Romeo Charlie.", ctx)
              .intent != I::READBACK);
  // "passing" / "reaching" are REPORTS the controller asked for.
  REQUIRE(intent_parser::parse("November Romeo Charlie passing 3000 feet.", ctx)
              .intent != I::READBACK);
  REQUIRE(intent_parser::parse("November Romeo Charlie reaching flight level 110.", ctx)
              .intent != I::READBACK);
  // "expect FL190" is part of an arrival clearance being previewed, not an echo.
  REQUIRE(intent_parser::parse("Expect flight level 190, November Romeo Charlie.", ctx)
              .intent != I::READBACK);

  // The participle form is a sector CHECK-IN when nothing is awaiting readback.
  // It is handed to INITIAL_CALL_CENTER outright, NOT merely demoted: a demotion
  // routes it to the LM, and a transmission must always end somewhere -- a
  // readback (silence) or a check-in (ack), never in the gap between them where
  // an LM outage turns it into "say again".
  const auto ci =
      intent_parser::parse("Climbing to flight level 110, November Romeo Charlie.", ctx);
  REQUIRE(ci.intent == I::INITIAL_CALL_CENTER);
  REQUIRE(ci.confidence >= 0.7f); // never routed to the LM
}
