// SPDX-License-Identifier: GPL-3.0-or-later
// IFR TTS phraseology tests. Copyright (C) 2026 Christopher P. Potter.

// A FREQUENCY IS NEVER A CARDINAL NUMBER.
//
// The TTS chain expanded squawk, flight levels, runways and navigation fixes,
// but handed frequencies to the voice as bare numerals. A neural voice reads
// "121.730" however it feels like on the day -- the user heard "one hundred
// twenty one decimal seven hundred thirty zero" on 2026-09-10, and it is
// intermittent, which is worse than always wrong: the pilot reads back what he
// heard and the verifier rejects it. ICAO Doc 4444: digit by digit, "decimal".

#include "atc/phonetic.hpp"

#include <catch2/catch_amalgamated.hpp>

#include <string>

using atc_phonetic::speak_frequencies;

TEST_CASE("frequencies are spoken digit by digit", "[tts][frequency]") {
  // The exact line from the flight.
  REQUIRE(speak_frequencies("contact ground on 121.730") ==
          "contact ground on one two one decimal seven three zero");

  // Every handoff of that flight, trailing full stop included.
  REQUIRE(speak_frequencies("contact Lyon Control on 125.155.") ==
          "contact Lyon Control on one two five decimal one five five.");
  REQUIRE(speak_frequencies("contact Chambery Approach on 121.205.") ==
          "contact Chambery Approach on one two one decimal two zero five.");
  REQUIRE(speak_frequencies("contact Tower on 118.200") ==
          "contact Tower on one one eight decimal two zero zero");

  // Two frequencies in one transmission.
  REQUIRE(speak_frequencies("118.200 or 121.730") ==
          "one one eight decimal two zero zero or one two one decimal seven "
          "three zero");

  // Band edges.
  REQUIRE(speak_frequencies("108.15") == "one zero eight decimal one five");
  REQUIRE(speak_frequencies("137.99") == "one three seven decimal nine nine");
}

TEST_CASE("only a plausible VHF frequency is rewritten", "[tts][frequency]") {
  // GUARD: out of the aviation band.
  REQUIRE(speak_frequencies("107.900") == "107.900");
  REQUIRE(speak_frequencies("138.000") == "138.000");

  // GUARD: no dot -- QNH, squawk and altitudes keep their own expanders.
  REQUIRE(speak_frequencies("QNH 1013, squawk 6334, climb 3000 feet") ==
          "QNH 1013, squawk 6334, climb 3000 feet");

  // GUARD: a single decimal is a distance, not a frequency.
  REQUIRE(speak_frequencies("125.5 NM to run") == "125.5 NM to run");

  // GUARD: four digits before the dot, or digits glued to letters.
  REQUIRE(speak_frequencies("1210.730") == "1210.730");
  REQUIRE(speak_frequencies("R118.200") == "R118.200");

  // Empty and plain text pass through untouched.
  REQUIRE(speak_frequencies("").empty());
  REQUIRE(speak_frequencies("November Romeo Charlie, roger.") ==
          "November Romeo Charlie, roger.");
}

TEST_CASE("the spelled frequency matches what the STT bias injects",
          "[tts][frequency][bias]") {
  // The bias and the spoken line must be the SAME string, or the pilot reads
  // back a phrase that was never biased. [[coding_bias_covers_readback]]
  REQUIRE(atc_phonetic::spell_frequency("120.230") ==
          "one two zero decimal two three zero");
  REQUIRE(atc_phonetic::spell_digits("730") == "seven three zero");
}
