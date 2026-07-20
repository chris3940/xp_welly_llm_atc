/*
 * xp_wellys_atc - AI-powered ATC voice communication for X-Plane 12
 * Copyright (C) 2026 thWelly & Claude (Anthropic)
 * Copyright (C) 2026 Christopher P. Potter (Linux port + IFR extensions)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

#include "atc/readback_verifier.hpp"

#include <algorithm>
#include <cctype>
#include <regex>
#include <string>

namespace readback_verifier {

// ── Normalisation helpers ─────────────────────────────────────────────────

static std::string to_lower(std::string s) {
  std::transform(s.begin(), s.end(), s.begin(), ::tolower);
  return s;
}

// Replace all whole-word occurrences of `from` with `to` in `s`.
static void replace_word(std::string &s, const std::string &from,
                         const std::string &to) {
  std::string::size_type pos = 0;
  while ((pos = s.find(from, pos)) != std::string::npos) {
    bool left  = (pos == 0 || !std::isalpha(static_cast<unsigned char>(s[pos - 1])));
    bool right = (pos + from.size() >= s.size() ||
                  !std::isalpha(static_cast<unsigned char>(s[pos + from.size()])));
    if (left && right) {
      s.replace(pos, from.size(), to);
      pos += to.size();
    } else {
      pos += 1;
    }
  }
}

// Convert NATO phonetic digits and "decimal"/"point" to ASCII equivalents.
// "niner zero" → "9 0",  "decimal" → ".",  "fife" → "5", etc.
static std::string normalize_phonetics(const std::string &raw) {
  std::string s = to_lower(raw);
  // Strip commas so a thousands-separated number reads back correctly: Voxtral
  // renders "5000" as "5,000", which split the digits and made the alt extractor
  // report (missing) -> false "negative, 5000 feet" on a CORRECT readback (LFLP
  // 2026-07-17). Removing the ',' char is safe -- commas carry no meaning here.
  s.erase(std::remove(s.begin(), s.end(), ','), s.end());
  // Order matters: longer words first to avoid partial matches.
  static const std::pair<const char *, const char *> kMap[] = {
      {"niner",   "9"}, {"seven",   "7"}, {"eight",   "8"},
      {"three",   "3"}, {"decimal", "."}, {"point",   "."},
      {"zero",    "0"}, {"one",     "1"}, {"two",     "2"},
      {"four",    "4"}, {"fife",    "5"}, {"five",    "5"},
      {"six",     "6"}, {"nine",    "9"},
      {"to",      "2"}, // STT mishears "two" as "to" (e.g. "118.200" → "118 decimal to 00")
  };
  for (const auto &[from, to] : kMap)
    replace_word(s, from, to);
  return s;
}

// Compact isolated single digits separated by a space: "9 0" → "90",
// "2 2" → "22", "1 6 6 1" → "1661".  Applied repeatedly until stable.
// Word-boundary anchors ensure we don't collapse digits inside multi-digit
// tokens (e.g. "FL 90" → only the "9 0" part compacts, not "FL90").
static std::string compact_digit_spaces(const std::string &s) {
  static const std::regex kRe(R"((\b\d) (\d\b))");
  std::string prev;
  std::string cur = s;
  do {
    prev = cur;
    cur  = std::regex_replace(cur, kRe, "$1$2");
  } while (cur != prev);
  return cur;
}

// Merge hyphen-separated digits: "7-0" -> "70". Voxtral renders spoken
// "seven zero" as "Seven-0", and after normalize_phonetics that is "7-0"; the
// hyphen would otherwise split the number in every extractor (FL70 read as
// FL7, causing a "negative, flight level seven zero, readback" loop --
// LFLP->LFMN 2026-07-12). Applied repeatedly so "1-1-0" -> "110".
static std::string merge_digit_hyphens(const std::string &s) {
  static const std::regex kRe(R"((\d)-(\d))");
  std::string prev;
  std::string cur = s;
  do {
    prev = cur;
    cur  = std::regex_replace(cur, kRe, "$1$2");
  } while (cur != prev);
  return cur;
}

// Full normalisation pipeline for a readback transcript.
static std::string normalise(const std::string &text) {
  return compact_digit_spaces(merge_digit_hyphens(normalize_phonetics(text)));
}

// ── Value extractors ──────────────────────────────────────────────────────

// Returns runway number as int (e.g. 22) or -1 if absent.
static int extract_runway(const std::string &norm) {
  static const std::regex kRe(R"(\brunway\s*(\d{1,2})[lLrRcC]?\b)",
                               std::regex_constants::icase);
  std::smatch m;
  if (std::regex_search(norm, m, kRe))
    return std::stoi(m[1]);
  // Runway implicit in the approach clearance: "RNAV Zulu approach 04",
  // "ILS approach 04" -> runway 04. Pilots routinely drop the word "runway"
  // when reading back the approach clearance ("expect RNAV Zulu approach 04"),
  // where the number after "approach" (optionally "approach runway") IS the
  // landing runway. Without this the readback verifier flags runway=missing
  // and rejects an otherwise-correct readback (LIMF -> LFLP 2026-07-11).
  static const std::regex kReAppr(
      R"(\bapproach\s*(?:runway\s*)?(\d{1,2})[lLrRcC]?\b)",
      std::regex_constants::icase);
  if (std::regex_search(norm, m, kReAppr))
    return std::stoi(m[1]);
  return -1;
}

// Returns flight level as int (90 for FL90 / "flight level 90" / "FL 090")
// or 0 if absent.
static int extract_fl(const std::string &norm) {
  // The digit run may contain internal spaces that compact_digit_spaces did
  // not merge. Voxtral mishears "two one zero" (210) as "to 10", which
  // normalize_phonetics turns into "2 10" -- compact_digit_spaces only merges
  // single-digit+single-digit ("(\b\d) (\d\b)"), so "2 10" survives and the
  // old "(\d{1,3})" grabbed just the leading "2" -> FL2, a false mismatch
  // (LIMF -> LFLP 2026-07-11: "flight level 210" verified as FL2). Match up to
  // three digits separated by optional single spaces, then strip the spaces.
  static const std::regex kRe(
      R"((?:fl\s*|flight\s+level\s+)(\d(?:\s?\d){0,2})\b)",
      std::regex_constants::icase);
  std::smatch m;
  if (std::regex_search(norm, m, kRe)) {
    std::string digits = m[1];
    digits.erase(std::remove(digits.begin(), digits.end(), ' '), digits.end());
    return std::stoi(digits);  // stoi("090") == 90
  }
  return 0;
}

// Returns altitude in feet (e.g. 6500) or 0 if absent.
static int extract_alt_ft(const std::string &norm) {
  static const std::regex kRe(R"((\d{3,5})\s*(?:feet|ft)\b)",
                               std::regex_constants::icase);
  std::smatch m;
  if (std::regex_search(norm, m, kRe))
    return std::stoi(m[1]);
  // Spoken word form: pilots read altitudes as "five thousand feet" (5000),
  // "five thousand five hundred" (5500), "thirty-five hundred" (3500).
  // Phonetic digits are already numerals here ("five"->"5"), so match
  // "<n> thousand [<m> hundred]" and compute. "thousand"/"hundred" are
  // altitude-specific (FL uses "flight level"), so "feet" may be dropped.
  static const std::regex kReTh(
      R"((\d{1,2})\s*thousand(?:\s*(\d{1,2})\s*hundred)?)",
      std::regex_constants::icase);
  if (std::regex_search(norm, m, kReTh)) {
    int v = std::stoi(m[1]) * 1000;
    if (m[2].matched)
      v += std::stoi(m[2]) * 100;
    return v;
  }
  static const std::regex kReH(R"((\d{1,3})\s*hundred)",
                               std::regex_constants::icase);
  if (std::regex_search(norm, m, kReH))
    return std::stoi(m[1]) * 100;
  return 0;
}

// Returns frequency as "NNN.NNN" string or "" if absent.
// Handles "121.205", "121 decimal 205" (after phonetics), and spaced
// digit triplets "1 2 1 . 2 0 5".
static std::string extract_freq(const std::string &norm) {
  // Standard format after normalisation: "121.205"
  static const std::regex kStd(R"(\b(\d{3})\.(\d{3})\b)");
  std::smatch m;
  if (std::regex_search(norm, m, kStd))
    return m[1].str() + "." + m[2].str();

  // Spaced variant: digits may have spaces — "1 2 1.2 0 5"
  static const std::regex kSpaced(
      R"((\d[\s]?\d[\s]?\d)\s*[.,]\s*(\d[\s]?\d[\s]?\d))");
  if (std::regex_search(norm, m, kSpaced)) {
    std::string p1 = m[1].str();
    std::string p2 = m[2].str();
    p1.erase(std::remove(p1.begin(), p1.end(), ' '), p1.end());
    p2.erase(std::remove(p2.begin(), p2.end(), ' '), p2.end());
    if (p1.size() == 3 && p2.size() == 3)
      return p1 + "." + p2;
  }
  return {};
}

// Returns squawk code as 4-char string or "" if absent.
static std::string extract_squawk(const std::string &norm) {
  static const std::regex kRe(R"(\bsquawk\s*(\d{4})\b)",
                               std::regex_constants::icase);
  std::smatch m;
  if (std::regex_search(norm, m, kRe))
    return m[1].str();
  return {};
}

// ── Formatting helpers ────────────────────────────────────────────────────

// Format a runway number for ICAO speech ("22" → "two two", "09" → "zero niner").
static std::string runway_to_speech(int rwy) {
  static const char *kDigit[] = {
      "zero", "one", "two", "three", "four",
      "five", "six", "seven", "eight", "niner",
  };
  char buf[4];
  std::snprintf(buf, sizeof(buf), "%02d", rwy);
  std::string out;
  for (char c : std::string(buf)) {
    if (!out.empty()) out += ' ';
    out += kDigit[c - '0'];
  }
  return out;  // "two two", "zero niner"
}

// Format a flight level for ICAO speech (90 → "niner zero", 120 → "one two zero").
static std::string fl_to_speech(int fl) {
  static const char *kDigit[] = {
      "zero", "one", "two", "three", "four",
      "five", "six", "seven", "eight", "niner",
  };
  char buf[16];
  std::snprintf(buf, sizeof(buf), "%d", fl);
  // Pad to at least 2 digits for single-digit levels (unlikely but safe)
  std::string s(buf);
  std::string out;
  for (char c : s) {
    if (!out.empty()) out += ' ';
    out += kDigit[c - '0'];
  }
  return out;  // 90 → "niner zero", 120 → "one two zero"
}

// ── Public API ────────────────────────────────────────────────────────────

// Spelled tens/teens -> value, for the "N hundred TEN" word form Voxtral emits when
// the STT context biases spoken numbers (LFLP 2026-07-18). Returns -1 if not a word.
static int spelled_low(const std::string &w) {
  static const std::pair<const char *, int> kLow[] = {
      {"ten", 10},      {"eleven", 11},  {"twelve", 12},   {"thirteen", 13},
      {"fourteen", 14}, {"fifteen", 15}, {"sixteen", 16},  {"seventeen", 17},
      {"eighteen", 18}, {"nineteen", 19}, {"twenty", 20},  {"thirty", 30},
      {"forty", 40},    {"fifty", 50},   {"sixty", 60},    {"seventy", 70},
      {"eighty", 80},   {"ninety", 90}};
  for (const auto &p : kLow)
    if (w == p.first)
      return p.second;
  return -1;
}

// Returns the assigned speed in knots (e.g. 250) or -1 if the text carries no
// speed restriction. Anchored on "speed" so a wind read-out ("... 01 knots")
// never matches; the digits may follow "speed" after filler ("speed, 250",
// "reduce speed to 250 knots").
static int extract_speed(const std::string &norm) {
  std::smatch m;
  // Primary: anchored on "speed" ("reduce speed, 250", "speed 210 knots").
  static const std::regex kRe(R"(speed[a-z, ]*?(\d{2,3}))",
                              std::regex_constants::icase);
  if (std::regex_search(norm, m, kRe))
    return std::stoi(m[1]);
  // Remaining spoken forms are trusted ONLY when "knot(s)" is present, so a
  // wind read-out or an altitude can never be mistaken for a speed. Pilots
  // naturally drop the word "speed" when reading a restriction back
  // ("200 knots or less"), which the anchored pattern missed -- causing an
  // endless "negative, N knots, readback" loop on a CORRECT read-back, in every
  // spoken form of 200 (LFMN R22LZ 2026-07-12).
  if (norm.find("knot") == std::string::npos)
    return -1;
  // "200 knots" -- digits, incl. "two zero zero" which normalise compacts to
  // "200".  Require >= 100 kt (real IAS restriction) vs a wind's "15 knots".
  static const std::regex kKnots(R"((\d{2,3})\s*knots?)",
                                 std::regex_constants::icase);
  if (std::regex_search(norm, m, kKnots)) {
    const int v = std::stoi(m[1]);
    if (v >= 100)
      return v;
  }
  // "two hundred [and] ten" / "two hundred 10" -- COMPOUND spoken speed. Capture the
  // tens remainder after "hundred" so 210 verifies (not just 200). The remainder is a
  // digit run ("2 hundred 10") OR a spelled tens/teens ("two hundred ten"), the forms
  // Voxtral emits once the STT context biases the spoken value (LFLP 2026-07-18,
  // "reduce speed 210 knots" read back as "to 110"/"two hundred ten"). Listed
  // explicitly so "two hundred knots" (no remainder) falls through to the plain rule.
  static const std::regex kHundredCompound(
      R"((\d)\s*hundred(?:\s+and)?\s+(\d{1,2}|ten|eleven|twelve|thirteen|fourteen|fifteen|sixteen|seventeen|eighteen|nineteen|twenty|thirty|forty|fifty|sixty|seventy|eighty|ninety))",
      std::regex_constants::icase);
  if (std::regex_search(norm, m, kHundredCompound)) {
    const std::string t = m[2].str();
    const int rem = std::isdigit(static_cast<unsigned char>(t[0]))
                        ? std::stoi(t)
                        : spelled_low(t);
    if (rem >= 0 && rem < 100) {
      const int v = std::stoi(m[1]) * 100 + rem;
      if (v >= 100)
        return v;
    }
  }
  // "two hundred knots" -- normalize_phonetics maps the digit word ("two"->"2")
  // but NOT "hundred", so expand "<n> hundred" -> n*100 here.
  static const std::regex kHundred(R"((\d)\s*hundred)",
                                   std::regex_constants::icase);
  if (std::regex_search(norm, m, kHundred)) {
    const int v = std::stoi(m[1]) * 100;
    if (v >= 100)
      return v;
  }
  return -1;
}

// Collapse a spoken compound "<hundreds> hundred <tens>" that Voxtral renders in
// pure-digit form -- "two hundred ten" -> "200, 10" -> normalised "200 10".
// A human controller hears 210; the digit-space merge below would instead splice
// it into "20010" and never match 210 (LFLP 2026-07-18, "reduce speed 210 knots"
// read back as "200, 10 knots"). Rewrite "(\d)00 <1-2 digits>" -> hundreds*100+tens
// so "200 10"->"210", "100 20"->"120", "200 5"->"205". Only fires when the trailing
// group is < 100, so a genuine "5000 100" style pair is left untouched.
static std::string collapse_compound_hundreds(const std::string &s) {
  static const std::regex kCompound(R"((\d)00\s+(\d{1,2})\b)");
  std::string out;
  std::smatch m;
  auto begin = s.cbegin();
  while (std::regex_search(begin, s.cend(), m, kCompound)) {
    out.append(m.prefix().first, m.prefix().second);
    const int v = std::stoi(m[1]) * 100 + std::stoi(m[2]);
    out += std::to_string(v);
    begin = m[0].second;
  }
  out.append(begin, s.cend());
  return out;
}

// Does the normalised readback CONTAIN the expected numeric value? Merges spaces
// between digits first ("2 1 0"/"2 10" -> "210") so a spaced digit run still
// matches. Used as a fallback when the field EXTRACTOR fails on a garbled prefix:
// Voxtral renders "flight level" as fit/flat/plate/table level or "flat of L", which
// breaks extraction even though the NUMBER ("210") is right there -> false "negative"
// (LFLP 2026-07-17). CONTAINS the expected value is robust to any prefix garble.
// Also tries a compound-hundreds collapse ("200 10" -> "210") before merging, for
// the "two hundred ten" spoken form (LFLP 2026-07-18).
static bool readback_contains(const std::string &norm, int value) {
  const std::string target = std::to_string(value);
  for (const std::string &variant : {norm, collapse_compound_hundreds(norm)}) {
    std::string s;
    for (size_t i = 0; i < variant.size(); ++i) {
      if (variant[i] == ' ' && i > 0 && i + 1 < variant.size() &&
          std::isdigit(static_cast<unsigned char>(variant[i - 1])) &&
          std::isdigit(static_cast<unsigned char>(variant[i + 1])))
        continue; // drop a space sitting between two digits
      s += variant[i];
    }
    if (s.find(target) != std::string::npos)
      return true;
  }
  return false;
}

std::vector<Mismatch> check(const std::string &clearance_text,
                            const std::string &readback_text) {
  std::vector<Mismatch> out;
  if (clearance_text.empty() || readback_text.empty())
    return out;

  // Normalise both texts: clearance is clean ATC text; readback is raw STT.
  const std::string cl = normalise(clearance_text);
  const std::string rb = normalise(readback_text);

  // ── Runway ─────────────────────────────────────────────────────────────
  int cl_rwy = extract_runway(cl);
  if (cl_rwy >= 0) {
    int rb_rwy = extract_runway(rb);
    if (rb_rwy < 0 || rb_rwy != cl_rwy) {
      Mismatch m;
      m.field    = "runway";
      m.expected = std::to_string(cl_rwy);
      m.stated   = rb_rwy >= 0 ? std::to_string(rb_rwy) : "";
      m.correction = "negative, runway " + runway_to_speech(cl_rwy) +
                     ", readback";
      out.push_back(std::move(m));
    }
  }

  // ── Flight level ───────────────────────────────────────────────────────
  int cl_fl = extract_fl(cl);
  if (cl_fl > 0) {
    int rb_fl = extract_fl(rb);
    // Accept when the readback CONTAINS the expected FL even if the "flight level"
    // prefix was garbled (fit/flat/plate level) so the extractor missed it.
    if ((rb_fl <= 0 || rb_fl != cl_fl) && !readback_contains(rb, cl_fl)) {
      Mismatch m;
      m.field    = "fl";
      m.expected = std::to_string(cl_fl);
      m.stated   = rb_fl > 0 ? std::to_string(rb_fl) : "";
      m.correction = "negative, flight level " + fl_to_speech(cl_fl) +
                     ", readback";
      out.push_back(std::move(m));
    }
  }

  // ── Altitude in feet (when no FL assigned) ─────────────────────────────
  if (cl_fl == 0) {
    int cl_alt = extract_alt_ft(cl);
    if (cl_alt > 0) {
      int rb_alt = extract_alt_ft(rb);
      // Allow ±100 ft tolerance; also accept when the readback CONTAINS the value
      // (garbled "feet"/prefix but the number is present).
      if ((rb_alt <= 0 || std::abs(rb_alt - cl_alt) > 100) &&
          !readback_contains(rb, cl_alt)) {
        Mismatch m;
        m.field    = "alt";
        m.expected = std::to_string(cl_alt);
        m.stated   = rb_alt > 0 ? std::to_string(rb_alt) : "";
        char buf[32];
        std::snprintf(buf, sizeof(buf), "%d feet", cl_alt);
        m.correction = std::string("negative, ") + buf + ", readback";
        out.push_back(std::move(m));
      }
    }
  }

  // ── Frequency ──────────────────────────────────────────────────────────
  std::string cl_freq = extract_freq(cl);
  if (!cl_freq.empty()) {
    std::string rb_freq = extract_freq(rb);
    if (rb_freq.empty() || rb_freq != cl_freq) {
      Mismatch m;
      m.field      = "freq";
      m.expected   = cl_freq;
      m.stated     = rb_freq;
      m.correction = "negative, " + cl_freq + ", readback";
      out.push_back(std::move(m));
    }
  }

  // ── Squawk ─────────────────────────────────────────────────────────────
  std::string cl_sq = extract_squawk(cl);
  if (!cl_sq.empty()) {
    std::string rb_sq = extract_squawk(rb);
    if (rb_sq.empty() || rb_sq != cl_sq) {
      Mismatch m;
      m.field      = "squawk";
      m.expected   = cl_sq;
      m.stated     = rb_sq;
      m.correction = "negative, squawk " + cl_sq + ", readback";
      out.push_back(std::move(m));
    }
  }

  // ── Speed ──────────────────────────────────────────────────────────────
  // Voxtral mishears the value ("250" -> "150"); speed was previously
  // unverified so the wrong readback was silently accepted (LIMF -> LFLP).
  int cl_spd = extract_speed(cl);
  if (cl_spd > 0) {
    int rb_spd = extract_speed(rb);
    if ((rb_spd <= 0 || rb_spd != cl_spd) && !readback_contains(rb, cl_spd)) {
      Mismatch m;
      m.field    = "speed";
      m.expected = std::to_string(cl_spd);
      m.stated   = rb_spd > 0 ? std::to_string(rb_spd) : "";
      char buf[40];
      std::snprintf(buf, sizeof(buf), "%d knots or less", cl_spd);
      m.correction = std::string("negative, ") + buf + ", readback";
      out.push_back(std::move(m));
    }
  }

  return out;
}

std::vector<std::string> matched_fields(const std::string &clearance_text,
                                        const std::string &readback_text) {
  std::vector<std::string> ok;
  if (clearance_text.empty() || readback_text.empty())
    return ok;

  const std::string cl = normalise(clearance_text);
  const std::string rb = normalise(readback_text);

  int cl_rwy = extract_runway(cl);
  if (cl_rwy >= 0 && extract_runway(rb) == cl_rwy)
    ok.push_back("runway");

  int cl_fl = extract_fl(cl);
  if (cl_fl > 0 && extract_fl(rb) == cl_fl)
    ok.push_back("fl");

  if (cl_fl == 0) {
    int cl_alt = extract_alt_ft(cl);
    if (cl_alt > 0) {
      int rb_alt = extract_alt_ft(rb);
      if (rb_alt > 0 && std::abs(rb_alt - cl_alt) <= 100)
        ok.push_back("alt");
    }
  }

  std::string cl_freq = extract_freq(cl);
  if (!cl_freq.empty() && extract_freq(rb) == cl_freq)
    ok.push_back("freq");

  std::string cl_sq = extract_squawk(cl);
  if (!cl_sq.empty() && extract_squawk(rb) == cl_sq)
    ok.push_back("squawk");

  int cl_spd = extract_speed(cl);
  if (cl_spd > 0 && extract_speed(rb) == cl_spd)
    ok.push_back("speed");

  return ok;
}

} // namespace readback_verifier
