// SPDX-License-Identifier: GPL-3.0-or-later
// IFR ground phraseology helper. Copyright (C) 2026 Christopher P. Potter.
#pragma once

#include <cctype>
#include <cstddef>
#include <string>
#include <vector>

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

// The VARIABLE content of an ATC transmission, for the STT context bias.
//
// A fixed phrase list can anchor "cleared to land" or "report established". It
// cannot anchor the part that changes every time -- the fix in "direct ELMEM",
// the procedure in "expect RNAV Zulu approach runway 08" -- and that is exactly
// what the pilot has to read back. Measured on the LOWI arrival of 2026-08-29:
// "direct ELMEM" came back as "LMEM", "RNAV Zulu approach" as "Arnazul
// approaching". ELMEM had been in the bias earlier in the flight and had dropped
// out of the set in force when it was spoken; "RNAV Zulu" was in none of that
// flight's five bias sets at all.
//
// Mining the transmission itself is the only construction that cannot drift out
// of step with what was just said. Returns entries in the ORIGINAL casing.
// See [[coding_bias_covers_readback]]. [C. P. Potter]
inline std::vector<std::string> readback_anchors(const std::string &atc_text) {
  std::vector<std::string> out;
  auto push = [&out](std::string v) {
    while (!v.empty() && (v.back() == ' ' || v.back() == ',' || v.back() == '.'))
      v.pop_back();
    if (v.empty())
      return;
    for (const auto &e : out)
      if (e == v)
        return;
    out.push_back(std::move(v));
  };
  std::string lc = atc_text;
  for (char &c : lc)
    c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));

  // "direct <FIX>" -- the phrase and the bare ident.
  for (std::size_t p = lc.find("direct "); p != std::string::npos;
       p = lc.find("direct ", p + 1)) {
    std::size_t b = p + 7, e = b;
    while (e < lc.size() && std::isalnum(static_cast<unsigned char>(lc[e])))
      ++e;
    if (e > b) {
      const std::string fix = atc_text.substr(b, e - b);
      push("direct " + fix);
      push(fix);
    }
  }

  // "<type> [letter] approach" -- "RNAV Zulu approach", "ILS approach".
  // The 24-character window keeps the type and the word "approach" in the same
  // clause, so "RNAV Zulu approach" matches and "... ILS ... contact approach"
  // does not.
  static const char *kTypes[] = {"rnav", "ils",       "rnp",   "vor",
                                 "ndb",  "localizer", "visual", nullptr};
  for (const char **t = kTypes; *t; ++t) {
    const std::size_t q = lc.find(*t);
    if (q == std::string::npos)
      continue;
    const std::size_t a = lc.find("approach", q);
    if (a == std::string::npos || a <= q || a - q > 24)
      continue;
    push(atc_text.substr(q, a - q + 8)); // "RNAV Zulu approach"
    push(atc_text.substr(q, a - q));     // "RNAV Zulu"
  }
  return out;
}

} // namespace atc_phonetic
