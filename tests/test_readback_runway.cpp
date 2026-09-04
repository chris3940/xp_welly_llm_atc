// Runway readback: the FULL designator, side included.
//
// The flight of 2026-08-28 out of Nice drew "negative, I say again, runway zero
// four" from an aircraft lined up on 04R. "Zero four" is not a runway
// designator, and at a field with parallel runways the side is the one word
// that carries the meaning. extract_runway() returned an int and threw the
// side away, so every correction built from it was truncated and 04L/04R were
// indistinguishable to the verifier. [C. P. Potter]
#include "atc/readback_verifier.hpp"
#include <catch2/catch_amalgamated.hpp>

using readback_verifier::check;
using readback_verifier::matched_fields;

static bool has_field(const std::vector<readback_verifier::Mismatch> &v,
                      const std::string &f) {
  for (const auto &m : v)
    if (m.field == f)
      return true;
  return false;
}

static std::string correction_for(
    const std::vector<readback_verifier::Mismatch> &v, const std::string &f) {
  for (const auto &m : v)
    if (m.field == f)
      return m.correction;
  return "";
}

TEST_CASE("the correction speaks the side, not just the number",
          "[readback][runway]") {
  const auto mm = check("runway 04R, line up and wait", "line up runway 22");
  REQUIRE(has_field(mm, "runway"));
  REQUIRE(correction_for(mm, "runway") ==
          "negative, I say again, runway zero four right");
}

TEST_CASE("a spoken side is accepted", "[readback][runway]") {
  const auto ok =
      matched_fields("runway 04R, line up and wait",
                     "line up runway zero four right November Romeo Charlie");
  bool found = false;
  for (const auto &f : ok)
    if (f == "runway")
      found = true;
  REQUIRE(found);
  REQUIRE(check("runway 04R, line up and wait",
                "line up runway zero four right November Romeo Charlie")
              .empty());
}

TEST_CASE("a written side is accepted", "[readback][runway]") {
  REQUIRE(check("runway 04R, cleared for takeoff",
                "cleared for takeoff runway 04R")
              .empty());
}

TEST_CASE("the wrong side is a mismatch", "[readback][runway]") {
  const auto mm =
      check("runway 04R, line up and wait", "line up runway zero four left");
  REQUIRE(has_field(mm, "runway"));
  REQUIRE(correction_for(mm, "runway") ==
          "negative, I say again, runway zero four right");
}

TEST_CASE("a missing side is an incomplete readback", "[readback][runway]") {
  // ICAO Doc 4444: the runway in use is read back in full. At Nice, "runway
  // zero four" could be either of two parallel runways.
  REQUIRE(has_field(check("runway 04R, line up and wait",
                          "line up runway zero four"),
                    "runway"));
}

TEST_CASE("a runway with no side is unaffected", "[readback][runway]") {
  REQUIRE(check("runway 22, cleared to land", "cleared to land runway 22")
              .empty());
  const auto mm = check("runway 22, cleared to land", "cleared to land runway 04");
  REQUIRE(correction_for(mm, "runway") ==
          "negative, I say again, runway two two");
}

// "negative, I say again" is for a readback that was WRONG. An item never
// mentioned is MISSING, and ICAO Doc 4444 says "READ BACK (items)".
//
// LOWI arrival of 2026-08-29: the pilot correctly read back the arrival
// clearance while a level clearance issued nine seconds later was still
// outstanding, and was answered "negative, I say again, flight level one five
// zero" -- accused of an error he had not made. [C. P. Potter]
TEST_CASE("a missing item is asked for, not negated", "[readback][missing]") {
  // The level was never mentioned: ask for it.
  const auto missing = check("descend flight level 150",
                             "cleared via NANIT Two Alpha arrival, expect RNAV "
                             "Zulu approach runway 08, November Romeo Charlie");
  REQUIRE(has_field(missing, "fl"));
  REQUIRE(correction_for(missing, "fl") == "read back flight level one five zero");

  // A level read back WRONG is still negated.
  const auto wrong = check("descend flight level 150",
                           "descend flight level 130, November Romeo Charlie");
  REQUIRE(has_field(wrong, "fl"));
  REQUIRE(correction_for(wrong, "fl") ==
          "negative, I say again, flight level one five zero");

  // Same rule for the runway.
  const auto rwy_missing = check("runway 04R, cleared for takeoff",
                                 "cleared for takeoff, November Romeo Charlie");
  REQUIRE(correction_for(rwy_missing, "runway") ==
          "read back runway zero four right");
}
