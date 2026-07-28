// SPDX-License-Identifier: GPL-3.0-or-later
// IFR ground phraseology helper. Copyright (C) 2026 Christopher P. Potter.
#pragma once

#include <cctype>
#include <string>

namespace atc_phonetic {

// Expand a taxiway / holding-point identifier to its SPOKEN ICAO form: each LETTER
// -> NATO word, each DIGIT -> digit word, joined by spaces.
//   "D"  -> "Delta"            (small fields)
//   "C1" -> "Charlie One"      (big airports, user 2026-07-28 -- LFMN uses C1/A3/W3)
//   "A3" -> "Alpha Three"      "EB" -> "Echo Bravo"
// Separators (slash etc.) are dropped from the spoken form. Falls back to the raw
// identifier if nothing expandable. Shared by the ATC SPOKEN output (ground_operations
// {holding_point}, engine runway-change) AND the STT context bias (atc_session) so the
// biased phrase matches what the controller says and what the pilot reads back.
inline std::string spell_holding_point(const std::string &hp) {
  static const char *kNato[] = {
      "Alpha",   "Bravo",  "Charlie", "Delta",   "Echo",    "Foxtrot", "Golf",
      "Hotel",   "India",  "Juliet",  "Kilo",    "Lima",    "Mike",    "November",
      "Oscar",   "Papa",   "Quebec",  "Romeo",   "Sierra",  "Tango",   "Uniform",
      "Victor",  "Whiskey","X-ray",   "Yankee",  "Zulu"};
  static const char *kDigit[] = {"Zero", "One", "Two",   "Three", "Four",
                                 "Five", "Six", "Seven", "Eight", "Nine"};
  std::string out;
  for (char c : hp) {
    const unsigned char uc = static_cast<unsigned char>(c);
    std::string tok;
    if (std::isalpha(uc))
      tok = kNato[std::toupper(uc) - 'A'];
    else if (std::isdigit(uc))
      tok = kDigit[uc - '0'];
    else
      continue; // drop separators from the spoken form
    if (!out.empty())
      out += ' ';
    out += tok;
  }
  return out.empty() ? hp : out;
}

} // namespace atc_phonetic
