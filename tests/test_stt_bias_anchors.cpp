// Every keyword the pilot reads back must be in the STT bias, in the form ATC
// spoke it. The stock phrase list cannot anchor the part that changes -- the fix
// in "direct ELMEM", the procedure in "expect RNAV Zulu approach" -- and that is
// what the pilot reads back.
//
// LOWI arrival of 2026-08-29, measured against the five bias sets of that flight:
//   ATC "direct ELMEM"              -> heard back "Direct LMEM"   (ELMEM absent)
//   ATC "RNAV Zulu approach rwy 08" -> "Arnazul approaching"      (never present)
// [C. P. Potter]
#include "atc/phonetic.hpp"
#include <catch2/catch_amalgamated.hpp>
#include <algorithm>
#include <string>
#include <vector>

static bool has(const std::vector<std::string> &v, const std::string &w) {
  return std::find(v.begin(), v.end(), w) != v.end();
}

TEST_CASE("the fix of a direct-to is anchored", "[bias]") {
  const auto a = atc_phonetic::readback_anchors(
      "November One One One Romeo Charlie, direct ELMEM.");
  REQUIRE(has(a, "direct ELMEM"));
  REQUIRE(has(a, "ELMEM"));      // the bare ident, as it was misheard
}

TEST_CASE("the approach procedure is anchored as spoken", "[bias]") {
  const auto a = atc_phonetic::readback_anchors(
      "November Romeo Charlie, cleared via NANIT Two Alpha arrival, expect "
      "RNAV Zulu approach runway 08.");
  REQUIRE(has(a, "RNAV Zulu approach"));
  REQUIRE(has(a, "RNAV Zulu"));
}

TEST_CASE("an ILS with no letter still anchors", "[bias]") {
  const auto a = atc_phonetic::readback_anchors(
      "November Romeo Charlie, cleared ILS approach runway 06.");
  REQUIRE(has(a, "ILS approach"));
}

TEST_CASE("a distant 'approach' is not glued to a procedure type", "[bias]") {
  // "contact Approach" is a facility, not a procedure: the type and the word
  // must sit in the same clause.
  const auto a = atc_phonetic::readback_anchors(
      "November Romeo Charlie, established on the ILS, report field in sight, "
      "and afterwards contact approach on one two eight decimal nine.");
  for (const auto &e : a)
    INFO("anchor: " << e);
  REQUIRE_FALSE(has(a, "ILS, report field in sight, and afterwards contact approach"));
}

TEST_CASE("nothing is invented from a transmission with neither", "[bias]") {
  REQUIRE(atc_phonetic::readback_anchors(
              "November Romeo Charlie, descend flight level 150.")
              .empty());
  REQUIRE(atc_phonetic::readback_anchors("").empty());
}

TEST_CASE("two directs in one transmission both anchor", "[bias]") {
  const auto a = atc_phonetic::readback_anchors(
      "November Romeo Charlie, direct RTT, then direct ELMEM.");
  REQUIRE(has(a, "direct RTT"));
  REQUIRE(has(a, "direct ELMEM"));
}
