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

// A FREQUENCY READBACK IS THE SAME READBACK IN THREE SHAPES.
//
// The bare-frequency rule wants the compact form "125.155", and only the fully
// SPELLED shape was ever collapsed to it. Voxtral just as often writes the
// numeric one. On the ground at Valence (2026-09-08), with the LM answering 403,
// "125, decimal 155, November Romeo Charlie" -- a perfectly correct readback of
// the handoff to Lyon -- drew "garbled, say again" three times in a row.
// [C. P. Potter]
TEST_CASE("a frequency readback is recognised whatever its shape",
          "[intent][readback][freq]") {
  intent_parser::init();
  xplane_context::XPlaneContext ctx;
  ctx.nearest_airport_id = "LFLU";
  ctx.on_ground = true;
  using I = intent_parser::PilotIntent;

  for (const char *t : {"one two five decimal one five five, November Romeo Charlie",
                        "125, decimal 155, November, Romeo, Charlie.",
                        "125 decimal 155",
                        "125.155 November Romeo Charlie"}) {
    const auto m = intent_parser::parse(t, ctx);
    INFO(t);
    REQUIRE(m.intent == I::READBACK);
    REQUIRE(m.confidence >= 0.9f);
  }
}

TEST_CASE("the numeric frequency collapse pads the fraction", "[intent][freq]") {
  // A pilot who says "one two five decimal one five" means 125.150.
  REQUIRE(intent_parser::normalize_spoken_frequency("125 decimal 15") == "125.150");
  REQUIRE(intent_parser::normalize_spoken_frequency("125, decimal 155") == "125.155");
  // "to" is "two" immediately after "decimal", and nowhere else.
  REQUIRE(intent_parser::normalize_spoken_frequency("120 decimal to 30") == "120.230");
  REQUIRE(intent_parser::normalize_spoken_frequency("taxi to holding point alpha") ==
          "taxi to holding point alpha");
}

// THE MHz PART ARRIVES AS SEPARATE DIGITS.
//
// The pilot spells the frequency out loud, digit by digit, and Voxtral writes
// it the same way: "1 2 5 decimal 1 5 5". Neither path recognised that shape --
// the numeric collapse wants three CONTIGUOUS digits before "decimal", and the
// spelled-word path throws every digit away (fq_lc_alpha keeps letters only),
// so a perfectly correct frequency read-back was refused with "unable" on the
// very first call of the flight (user 2026-09-09).
// Voxtral mixes the two shapes inside one transmission, so digits are now
// first-class number tokens everywhere in the normaliser. [C. P. Potter]
TEST_CASE("a frequency spelled as separate digits collapses",
          "[intent][frequency]") {
  using intent_parser::normalize_spoken_frequency;
  // The reported failure, verbatim.
  REQUIRE(normalize_spoken_frequency("1 2 5 decimal 1 5 5") == "125.155");
  // In a full read-back sentence.
  REQUIRE(normalize_spoken_frequency(
              "contact lyon on 1 2 5 decimal 1 5 5 november romeo charlie") ==
          "contact lyon on 125.155 november romeo charlie");
  // Hyphenated, which Voxtral alternates with spaces arbitrarily.
  REQUIRE(normalize_spoken_frequency("1-2-5 decimal 1-5-5") == "125.155");
  // Mixed shapes in one transmission: spelled whole part, numeric fraction.
  REQUIRE(normalize_spoken_frequency("one two five decimal 1 5 5") == "125.155");
  REQUIRE(normalize_spoken_frequency("1 2 5 decimal six three zero") ==
          "125.630");
  // Short fraction pads to three digits, as the numeric path already does.
  REQUIRE(normalize_spoken_frequency("1 2 5 decimal 1 5") == "125.150");
  // Dropped leading "1" of the 1xx band, in digit form.
  REQUIRE(normalize_spoken_frequency("2 5 decimal 6 3 0") == "125.630");
  // GUARD: a digit run that is not a plausible VHF frequency is left alone.
  REQUIRE(normalize_spoken_frequency("4 5 6 point 7 8 9") == "4 5 6 point 7 8 9");
  // GUARD: "point" as a taxiway word is still not a decimal separator.
  REQUIRE(normalize_spoken_frequency("hold short at holding point alpha") ==
          "hold short at holding point alpha");
}

// THE BARE FREQUENCY IS A COMPLETE READ-BACK.
//
// A pilot acknowledging a handoff often says the frequency and nothing else --
// no "contact", no facility, sometimes not even the callsign. That is correct
// phraseology, and the rule table recognises it through starts_with_vhf_freq.
// It only ever saw the compact "125.155" form, so the spelled-digit shape was
// classified as something else entirely and the guards refused it with
// "unable" (user 2026-09-09). End-to-end here, not just the normaliser, because
// the defect lived in the seam between the two. [C. P. Potter]
TEST_CASE("a bare frequency read-back is a READBACK, spelled or compact",
          "[intent][frequency][readback]") {
  intent_parser::init();
  xplane_context::XPlaneContext ctx;
  ctx.nearest_airport_id = "LFLU";
  using I = intent_parser::PilotIntent;

  for (const char *t : {"125.155",
                        "1 2 5 decimal 1 5 5",
                        "one two five decimal one five five",
                        "125, decimal 155",
                        "1 2 5 decimal 1 5 5, November Romeo Charlie"}) {
    INFO(t);
    REQUIRE(intent_parser::parse(t, ctx).intent == I::READBACK);
  }
}

// BACKTRACKING IS ROUTINE WHERE THE RUNWAY IS THE TAXIWAY.
//
// No intent existed for it, so "November Romeo Charlie, tracking back runway 19"
// at Valence was answered "unable" -- a refusal of a manoeuvre that, at a field
// with no Tower, is nobody's to refuse (2026-09-08). Voxtral writes "backtrack"
// as "tracking back" and even "tracking pack". [C. P. Potter]
TEST_CASE("a backtrack request is recognised, however Voxtral spells it",
          "[intent][backtrack]") {
  intent_parser::init();
  xplane_context::XPlaneContext ctx;
  ctx.nearest_airport_id = "LFLU";
  ctx.on_ground = true;
  ctx.active_runway = "19";
  using I = intent_parser::PilotIntent;

  for (const char *t : {"November Romeo Charlie, request backtrack runway 19",
                        "November, Romeo Charlie tracking back runway 19",
                        "November, Romeo Charlie tracking pack runway OneNine",
                        "backtrack runway 19, November Romeo Charlie"}) {
    INFO(t);
    REQUIRE(intent_parser::parse(t, ctx).intent == I::REQUEST_BACKTRACK);
  }
  // "back on track" is a position report, not a backtrack.
  REQUIRE(intent_parser::parse(
              "November Romeo Charlie, we are back on track to ROMAM", ctx)
              .intent != I::REQUEST_BACKTRACK);
}


// THREE CORRECT TRANSMISSIONS ANSWERED "GARBLED" IN ONE FLIGHT.
//
// LFLU -> LFLP, 2026-09-10, with the LM returning 403 on every call: three
// transmissions the rules did not cover fell through to a language model that
// was not there, and the pilot heard "your transmission was garbled, say
// again" for phraseology that was correct.
//
//   1. The AFIS acknowledgement read back -- "runway 01 in use, QNH 1016, no
//      reported traffic". The bare word "traffic" was the ENTIRE predicate of
//      the SELF_ANNOUNCE rule, so the controller's own words about traffic
//      matched it, were demoted to 0.30 at a towered field, and went to the LM.
//   2. "taking off runway 01" -- at a field with no Tower this is the whole of
//      the departure procedure, and no intent existed for it.
//   3. The closure acknowledgement -- "flight plan closed at 2013".
//
// [C. P. Potter]
TEST_CASE("the AFIS calls that drew 'garbled' are recognised",
          "[intent][afis][garbled]") {
  intent_parser::init();
  xplane_context::XPlaneContext ctx;
  ctx.nearest_airport_id = "LFLU";
  ctx.on_ground = true;
  ctx.active_runway = "01";
  using I = intent_parser::PilotIntent;
  auto in = [&](const char *t) { return intent_parser::parse(t, ctx).intent; };

  // 1. Reading back what the AFIS station just said.
  REQUIRE(in("Runway Zero One in use, QNH One Zero One Six, no reported "
             "traffic, November Romeo Charlie.") == I::READBACK);
  REQUIRE(in("runway 01 in use, wind 347 degrees 09 knots, no reported "
             "traffic, November Romeo Charlie") == I::READBACK);

  // 2. Announcing his own departure -- the AFIS field has nobody to clear it.
  REQUIRE(in("November, Romeo Charlie taking off runway 01.") ==
          I::REPORT_TAKING_OFF);
  REQUIRE(in("November Romeo Charlie, now rolling runway 01") ==
          I::REPORT_TAKING_OFF);

  // 3. Acknowledging the closure.
  REQUIRE(in("I found the flight plan closed at 2013 November, Romeo Charlie. "
             "Bye bye.") == I::READBACK);

  // GUARD: at a TOWERED field the take-off CLEARANCE read-back is a readback,
  // never a self-announcement -- the pilot is repeating an instruction.
  REQUIRE(in("cleared for takeoff runway 01, November Romeo Charlie") ==
          I::READBACK);
  // GUARD: asking is not announcing.
  REQUIRE(in("November Romeo Charlie, ready for departure runway 01") !=
          I::REPORT_TAKING_OFF);
}

// A GENUINE SELF-ANNOUNCEMENT STILL WORKS.
//
// The SELF_ANNOUNCE guard above must exclude the CONTROLLER's words about
// traffic without disarming the intent itself, which is what a pilot at an
// uncontrolled field broadcasts on the common frequency.
TEST_CASE("self-announce survives the traffic guard", "[intent][selfannounce]") {
  intent_parser::init();
  xplane_context::XPlaneContext ctx;
  ctx.nearest_airport_id = "LFLU";
  ctx.is_towered_airport = false;
  using I = intent_parser::PilotIntent;
  REQUIRE(intent_parser::parse(
              "Valence traffic, November Romeo Charlie, downwind runway 01",
              ctx)
              .intent == I::SELF_ANNOUNCE);
}
