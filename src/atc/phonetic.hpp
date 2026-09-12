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


// Spell a run of digits the way ATC says them: "730" -> "seven three zero".
// Non-digits are skipped. Shared by the TTS expanders and the STT bias so the
// spoken form and the biased form are the same string. See
// coding_atc_number_spelling.
inline std::string spell_digits(const std::string &num) {
  static const char *kDigit[] = {"zero", "one", "two",   "three", "four",
                                 "five", "six", "seven", "eight", "nine"};
  std::string out;
  for (char c : num) {
    if (!std::isdigit(static_cast<unsigned char>(c)))
      continue;
    if (!out.empty())
      out += ' ';
    out += kDigit[c - '0'];
  }
  return out;
}

// Spell a VHF frequency digit-by-digit with "decimal" for the dot (ICAO/EU):
// "120.230" -> "one two zero decimal two three zero".
inline std::string spell_frequency(const std::string &mhz) {
  const auto dot = mhz.find('.');
  if (dot == std::string::npos)
    return spell_digits(mhz);
  const std::string whole = spell_digits(mhz.substr(0, dot));
  const std::string frac = spell_digits(mhz.substr(dot + 1));
  if (whole.empty() && frac.empty())
    return {};
  std::string s = whole;
  s += (s.empty() ? "" : " ") + std::string("decimal");
  if (!frac.empty())
    s += " " + frac;
  return s;
}

// A FREQUENCY IS NEVER A CARDINAL NUMBER.
//
// The TTS chain already expands squawk, flight levels, runways and navigation
// fixes, but frequencies were handed to the voice as bare numerals -- and a
// neural voice reads "121.730" however it feels like on the day: sometimes
// "one twenty one point seven thirty", sometimes the outright wrong "one
// hundred twenty one decimal seven hundred thirty" the user heard on
// 2026-09-10. The pilot then reads back what he heard, and the read-back
// verifier rejects it. ICAO Doc 4444 is unambiguous: frequencies are spoken
// digit by digit, with "decimal".
//
// Only a plausible VHF aviation frequency is rewritten -- three digits in the
// 108-137 MHz band, a dot, then two or three digits. That shape does not occur
// anywhere else in ATC speech: QNH and squawk carry no dot, altitudes and
// speeds are cardinals without one, and a distance in that band with two
// decimals is not something a controller says. [C. P. Potter]
inline std::string speak_frequencies(const std::string &text) {
  std::string out;
  out.reserve(text.size() + 32);
  size_t i = 0;
  while (i < text.size()) {
    const bool left_ok =
        (i == 0) || (!std::isalnum(static_cast<unsigned char>(text[i - 1])) &&
                     text[i - 1] != '.');
    size_t e = i;
    while (e < text.size() && std::isdigit(static_cast<unsigned char>(text[e])))
      ++e;
    const size_t whole_len = e - i;
    bool matched = false;
    if (left_ok && whole_len == 3 && e < text.size() && text[e] == '.') {
      size_t f = e + 1;
      while (f < text.size() && std::isdigit(static_cast<unsigned char>(text[f])))
        ++f;
      const size_t frac_len = f - (e + 1);
      const bool right_ok =
          f >= text.size() || !std::isalnum(static_cast<unsigned char>(text[f]));
      const int mhz = (text[i] - '0') * 100 + (text[i + 1] - '0') * 10 +
                      (text[i + 2] - '0');
      if ((frac_len == 2 || frac_len == 3) && right_ok && mhz >= 108 &&
          mhz <= 137) {
        out += spell_frequency(text.substr(i, f - i));
        i = f;
        matched = true;
      }
    }
    if (!matched) {
      out += text[i];
      ++i;
    }
  }
  return out;
}

} // namespace atc_phonetic
