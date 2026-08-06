/*
 * xp_wellys_atc - AI-powered ATC voice communication for X-Plane 12
 * Copyright (C) 2026 thWelly & Claude (Anthropic)
 * Copyright (C) 2026 Christopher P. Potter (Linux port + IFR extensions)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <https://www.gnu.org/licenses/>.
 */

#include "data/cifp_reader.hpp"

#include "core/logging.hpp"

#include <algorithm>
#include <cctype>
#include <climits>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <mutex>
#include <set>
#include <sstream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace cifp_reader {

namespace {

// Per-airport+runway result cache keyed on "ICAO:runway" (e.g. "LFLP:04").
// Avoids re-reading the CIFP .dat file on every flight-loop iteration.
// Populated on first query; cleared on cifp_reader::clear_cache().
static std::unordered_map<std::string, CifpAlt> g_alt_cache;
static std::unordered_map<std::string, std::string> g_sid_name_cache;
static std::unordered_map<std::string, std::string> g_last_fix_cache;
static std::unordered_map<std::string, CifpBindingAlt> g_binding_alt_cache;
static std::mutex g_alt_cache_mutex;

std::vector<std::string> split_csv(const std::string &line) {
  std::vector<std::string> out;
  std::istringstream ss(line);
  std::string tok;
  while (std::getline(ss, tok, ','))
    out.push_back(tok);
  return out;
}

std::string trim(const std::string &s) {
  const std::string ws = " \t\r\n;";
  size_t start = s.find_first_not_of(ws);
  if (start == std::string::npos)
    return "";
  size_t end = s.find_last_not_of(ws);
  return s.substr(start, end - start + 1);
}

// Parse an altitude field from the CIFP record.
// "06500"  → {6500, false}  (feet AMSL — express as "6500 feet" in clearance)
// "FL130"  → {13000, true}  (flight level — express as "FL130" in clearance)
// ""       → {0, false}     (field empty, no constraint)
CifpAlt parse_alt(const std::string &raw) {
  std::string s = trim(raw);
  if (s.empty())
    return {};

  if (s.size() >= 3 && s[0] == 'F' && s[1] == 'L') {
    // Malformed flight level: fall through to {} below.
    try {
      int fl = std::stoi(s.substr(2));
      return {fl * 100, true};
    } catch (...) { // NOLINT(bugprone-empty-catch)
    }
    return {};
  }

  // Malformed altitude: fall through to {} below.
  try {
    int ft = std::stoi(s);
    if (ft > 0)
      return {ft, false};
  } catch (...) { // NOLINT(bugprone-empty-catch)
  }
  return {};
}

} // namespace

CifpAlt initial_altitude(const std::string &cifp_dir, const std::string &icao,
                         const std::string &active_runway) {
  if (cifp_dir.empty() || icao.empty() || active_runway.empty())
    return {};

  // Cache lookup — avoids re-reading the file every flight-loop frame.
  std::string cache_key = icao + ":" + active_runway;
  {
    std::lock_guard<std::mutex> lk(g_alt_cache_mutex);
    auto it = g_alt_cache.find(cache_key);
    if (it != g_alt_cache.end())
      return it->second;
  }

  std::string dir = cifp_dir;
  if (dir.back() != '/')
    dir += '/';

  // CIFP files are named by uppercase ICAO, e.g. LFLP.dat
  std::string icao_upper = icao;
  for (char &c : icao_upper)
    c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));

  std::string path = dir + icao_upper + ".dat";
  std::ifstream in(path);
  if (!in.good()) {
    logging::info("[cifp] file not found: %s (icao=%s rwy=%s) -> fallback",
                  path.c_str(), icao_upper.c_str(), active_runway.c_str());
    std::lock_guard<std::mutex> lk(g_alt_cache_mutex);
    g_alt_cache[cache_key] = {};  // cache the failure to suppress per-frame log spam
    return {};
  }

  // Runway match string: prepend "RW", e.g. "22" → "RW22", "09L" → "RW09L"
  std::string rwy_match = "RW" + active_runway;

  int best_seq = INT_MAX;
  CifpAlt best;

  std::string line;
  while (std::getline(in, line)) {
    // Only SID records start with "SID:"
    if (line.size() < 4 || line.compare(0, 4, "SID:") != 0)
      continue;

    auto f = split_csv(line);
    if (f.size() < 26)
      continue;

    // Field[3] = runway designator (e.g. "RW22")
    if (trim(f[3]) != rwy_match)
      continue;

    // Field[0] = "SID:{seq}", sequence in steps of 10 (010, 020, ...)
    std::string seq_str = trim(f[0]);
    if (seq_str.size() <= 4)
      continue;
    int seq = 0;
    try {
      seq = std::stoi(seq_str.substr(4));
    } catch (...) {
      continue;
    }

    // Field[11] = path terminator
    std::string pterm = trim(f[11]);

    CifpAlt alt;
    if (pterm == "CF" && f.size() > 25) {
      alt = parse_alt(f[25]);
    } else if ((pterm == "DF" || pterm == "TF") && f.size() > 23) {
      alt = parse_alt(f[23]);
    }

    if (alt.feet > 0 && seq < best_seq) {
      best_seq = seq;
      best = alt;
    }
  }

  if (best.feet > 0) {
    logging::info("[cifp] %s rwy %s initial alt -> %d ft (is_fl=%d)",
                  icao_upper.c_str(), active_runway.c_str(), best.feet,
                  best.is_fl ? 1 : 0);
    std::lock_guard<std::mutex> lk(g_alt_cache_mutex);
    g_alt_cache[cache_key] = best;
    return best;
  }

  // No SID found for the requested runway.  Try the reciprocal end as a
  // fallback (calm-wind selection may have picked the non-procedure end).
  logging::info("[cifp] %s rwy %s -> no SID found, trying reciprocal",
                icao_upper.c_str(), active_runway.c_str());

  // Compute reciprocal: strip optional L/R/C suffix, flip number by ±18.
  if (!active_runway.empty()) {
    std::string digits = active_runway;
    char suffix = 0;
    if (!digits.empty() && (digits.back() == 'L' || digits.back() == 'R' ||
                            digits.back() == 'C')) {
      suffix = digits.back();
      digits.pop_back();
    }
    try {
      int num = std::stoi(digits);
      int recip_num = (num <= 18) ? num + 18 : num - 18;
      char recip_buf[16];
      std::snprintf(recip_buf, sizeof(recip_buf), "%02d", recip_num);
      std::string recip = recip_buf;
      if (suffix == 'L')
        recip += 'R';
      else if (suffix == 'R')
        recip += 'L';
      else if (suffix == 'C')
        recip += 'C';

      // Re-read from the start of the file for the reciprocal runway.
      in.clear();
      in.seekg(0);
      std::string recip_match = "RW" + recip;
      int recip_best_seq = INT_MAX;
      CifpAlt recip_best;

      while (std::getline(in, line)) {
        if (line.size() < 4 || line.compare(0, 4, "SID:") != 0)
          continue;
        auto f2 = split_csv(line);
        if (f2.size() < 26)
          continue;
        if (trim(f2[3]) != recip_match)
          continue;
        std::string seq_str2 = trim(f2[0]);
        if (seq_str2.size() <= 4)
          continue;
        int seq2 = 0;
        try {
          seq2 = std::stoi(seq_str2.substr(4));
        } catch (...) {
          continue;
        }
        std::string pterm2 = trim(f2[11]);
        CifpAlt alt2;
        if (pterm2 == "CF" && f2.size() > 25)
          alt2 = parse_alt(f2[25]);
        else if ((pterm2 == "DF" || pterm2 == "TF") && f2.size() > 23)
          alt2 = parse_alt(f2[23]);
        if (alt2.feet > 0 && seq2 < recip_best_seq) {
          recip_best_seq = seq2;
          recip_best = alt2;
        }
      }

      if (recip_best.feet > 0) {
        logging::info(
            "[cifp] %s reciprocal rwy %s initial alt -> %d ft (is_fl=%d)",
            icao_upper.c_str(), recip.c_str(), recip_best.feet,
            recip_best.is_fl ? 1 : 0);
        std::lock_guard<std::mutex> lk(g_alt_cache_mutex);
        g_alt_cache[cache_key] = recip_best;
        return recip_best;
      }
      // CIFP parse failure: fall through and try the next source.
    } catch (...) { // NOLINT(bugprone-empty-catch)
    }
  }

  logging::info(
      "[cifp] %s rwy %s -> no initial alt found (no SID on either end)",
      icao_upper.c_str(), active_runway.c_str());
  // Cache the negative result too so the file isn't re-read on every frame.
  {
    std::lock_guard<std::mutex> lk(g_alt_cache_mutex);
    g_alt_cache[cache_key] = {};
  }
  return {};
}

// ── Shared CIFP file opener + ICAO upper-caser ──────────────────────────

static std::string make_cifp_path(const std::string &cifp_dir,
                                  const std::string &icao) {
  std::string dir = cifp_dir;
  if (dir.back() != '/')
    dir += '/';
  std::string upper = icao;
  for (char &c : upper)
    c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
  return dir + upper + ".dat";
}

// ── Approach helpers ───────────────────────────────────────────────────

// Parse "I04LY" → type_char='I', runway="04L", suffix='Y'.  Suffix is the
// trailing variant letter (Z, Y, X, W, ...) — set to 0 when the designator
// has no variant letter (e.g. "I04L").  ICAO convention: multiple approaches
// of the same type on the same runway are published Z first, then Y, X, ...
// so a Z variant is the primary / preferred one — best_approach uses this
// to tie-break same-type-same-runway candidates.
static bool parse_approach_designator(const std::string &des,
                                       char &type_char,
                                       std::string &runway,
                                       char &suffix) {
  if (des.size() < 3) return false;
  type_char = des[0];
  size_t i = 1;
  while (i < des.size() && std::isdigit(static_cast<unsigned char>(des[i]))) ++i;
  if (i < des.size() &&
      (des[i] == 'L' || des[i] == 'R' || des[i] == 'C')) ++i;
  runway = des.substr(1, i - 1);
  // Optional dash separator between runway and variant letter — some
  // AIRAC vendors emit "R04-Y" instead of "R04Y".  Skip a single dash
  // before scanning for the trailing variant letter.
  if (i < des.size() && des[i] == '-') ++i;
  suffix = static_cast<char>(
      (i < des.size() && std::isalpha(static_cast<unsigned char>(des[i])))
          ? des[i]
          : 0);
  return !runway.empty();
}
// 3-arg overload for call sites that don't care about the variant letter.
static bool parse_approach_designator(const std::string &des,
                                       char &type_char,
                                       std::string &runway) {
  char unused = 0;
  return parse_approach_designator(des, type_char, runway, unused);
}

// Visibility-driven priority: RNAV preferred in normal conditions,
// ILS preferred below 800 m (LVP). Fallback chain is unchanged.
static int approach_priority(char t, float vis_m) {
  const bool lvp = vis_m < 800.0f;
  switch (t) {
    case 'I': case 'S': return lvp ? 5 : 4; // ILS: top in LVP, second otherwise
    case 'R':           return lvp ? 4 : 5; // RNAV/RNP: top in VMC, second in LVP
    case 'L': case 'B': return 3;            // Localizer
    case 'D':           return 2;            // VOR/DME
    case 'V':           return 1;            // VOR
    default:            return 0;            // NDB, TACAN, etc.
  }
}

static const char *approach_type_str(char t) {
  switch (t) {
    case 'I': case 'S': return "ILS";
    case 'R':           return "RNAV";
    case 'L':           return "Localizer";
    case 'B':           return "Localizer back-course";
    case 'D':           return "VOR DME";
    case 'V':           return "VOR";
    case 'N': case 'Q': return "NDB";
    default:            return "approach";
  }
}

static std::unordered_map<std::string, ApproachInfo> g_approach_cache;
static std::unordered_map<std::string, FafFix>      g_faf_cache;

// ── sid_last_fix ───────────────────────────────────────────────────────

std::string sid_last_fix(const std::string &cifp_dir, const std::string &icao,
                         const std::string &sid_name) {
  if (cifp_dir.empty() || icao.empty() || sid_name.empty())
    return {};

  std::string cache_key = icao + ":" + sid_name + ":LAST";
  {
    std::lock_guard<std::mutex> lk(g_alt_cache_mutex);
    auto it = g_last_fix_cache.find(cache_key);
    if (it != g_last_fix_cache.end())
      return it->second;
  }

  std::ifstream in(make_cifp_path(cifp_dir, icao));
  if (!in.good()) {
    std::lock_guard<std::mutex> lk(g_alt_cache_mutex);
    g_last_fix_cache[cache_key] = {};
    return {};
  }

  int best_seq = -1;
  std::string best_wpt;

  std::string line;
  while (std::getline(in, line)) {
    if (line.size() < 4 || line.compare(0, 4, "SID:") != 0)
      continue;
    auto f = split_csv(line);
    if (f.size() < 5)
      continue;
    if (trim(f[2]) != sid_name)
      continue;

    std::string seq_str = trim(f[0]);
    if (seq_str.size() <= 4)
      continue;
    int seq = 0;
    try {
      seq = std::stoi(seq_str.substr(4));
    } catch (...) {
      continue;
    }

    if (seq > best_seq) {
      best_seq = seq;
      best_wpt = trim(f[4]);
    }
  }

  logging::debug("[cifp] %s SID %s last fix -> %s", icao.c_str(),
                 sid_name.c_str(),
                 best_wpt.empty() ? "(none)" : best_wpt.c_str());
  {
    std::lock_guard<std::mutex> lk(g_alt_cache_mutex);
    g_last_fix_cache[cache_key] = best_wpt;
  }
  return best_wpt;
}

// True when a SID/STAR runway-transition field (ARINC 424 f[3], e.g. "RW04B",
// "RW22R", "RW04") applies to the given active runway ("04R"). Honours the "B" = BOTH
// parallels suffix and a bare "RWxx" (no L/R/C) = all parallels of that number, so both
// "RW04B" and "RW04" match "04R" (user 2026-07-28: RW04B / RW22R exact-compare misses).
static bool rwy_transition_matches(const std::string &field3,
                                   const std::string &active_runway) {
  if (active_runway.empty() || field3.size() < 3 ||
      field3.compare(0, 2, "RW") != 0)
    return false;
  const std::string tr = field3.substr(2); // "04B", "22R", "04"
  if (tr == active_runway)
    return true;
  auto split_num_suffix = [](const std::string &s, std::string &num, char &suf) {
    num = s;
    suf = 0;
    if (!num.empty()) {
      const char c = num.back();
      if (c == 'L' || c == 'R' || c == 'C' || c == 'B') {
        suf = c;
        num.pop_back();
      }
    }
  };
  std::string tn, an;
  char ts = 0, as = 0;
  split_num_suffix(tr, tn, ts);
  split_num_suffix(active_runway, an, as);
  if (tn != an)
    return false; // different runway number
  // Transition serves BOTH / all parallels, or exactly this side.
  return ts == 'B' || ts == 0 || ts == as;
}

// ── sid_name_for_last_fix ──────────────────────────────────────────────

std::string sid_name_for_last_fix(const std::string &cifp_dir,
                                  const std::string &icao,
                                  const std::string &active_runway,
                                  const std::string &fpl_first_fix) {
  if (cifp_dir.empty() || icao.empty() || fpl_first_fix.empty())
    return {};

  std::string cache_key = icao + ":" + active_runway + ":FIX:" + fpl_first_fix;
  {
    std::lock_guard<std::mutex> lk(g_alt_cache_mutex);
    auto it = g_sid_name_cache.find(cache_key);
    if (it != g_sid_name_cache.end())
      return it->second;
  }

  std::ifstream in(make_cifp_path(cifp_dir, icao));
  if (!in.good()) {
    std::lock_guard<std::mutex> lk(g_alt_cache_mutex);
    g_sid_name_cache[cache_key] = {};
    return {};
  }

  // Pass 1: scan ALL SID rows (do NOT pre-filter by runway). A SID's TERMINATING fix
  // lives in its COMMON-route rows (empty f[3]) or a "RWxxB" both-parallels row, NOT the
  // per-runway RWxx transition rows. Pre-filtering by "RW"+runway dropped the exit fix:
  // BASI8X's BASIP sits in a common row, so a 22R lookup saw only MN221/MN223 and
  // returned (none), then a runway-independent fallback picked the 04 SID BASI8A (user
  // 2026-07-28). Per SID track: max-sequence terminating waypoint across ALL rows, plus
  // whether it has ANY runway transition and one MATCHING the active runway (via
  // rwy_transition_matches -> honours "B"=both and bare RWxx).
  struct SidInfo {
    int max_seq = -1;
    std::string wpt;
    bool has_any_rwy_transition = false;
    bool serves_runway = false;
  };
  std::unordered_map<std::string, SidInfo> sid_map;

  std::string line;
  while (std::getline(in, line)) {
    if (line.size() < 4 || line.compare(0, 4, "SID:") != 0)
      continue;
    auto f = split_csv(line);
    if (f.size() < 5)
      continue;
    std::string sid = trim(f[2]);
    if (sid.empty())
      continue;

    std::string seq_str = trim(f[0]);
    if (seq_str.size() <= 4)
      continue;
    int seq = 0;
    try {
      seq = std::stoi(seq_str.substr(4));
    } catch (...) {
      continue;
    }

    auto &info = sid_map[sid];
    const std::string tr = trim(f[3]);
    if (tr.size() >= 2 && tr.compare(0, 2, "RW") == 0) {
      info.has_any_rwy_transition = true;
      if (rwy_transition_matches(tr, active_runway))
        info.serves_runway = true;
    }
    const std::string wpt = trim(f[4]);
    if (seq > info.max_seq) {
      info.max_seq = seq;
      info.wpt = wpt;
    }
  }

  // Pass 2: the SID whose terminating waypoint == fpl_first_fix AND that SERVES the
  // active runway -- it has a matching RW transition, OR has NO runway transitions at
  // all (a runway-independent SID). Empty active_runway = any-runway search.
  std::string result;
  for (auto &kv : sid_map) {
    const SidInfo &s = kv.second;
    if (s.wpt != fpl_first_fix)
      continue;
    const bool serves =
        active_runway.empty() || s.serves_runway || !s.has_any_rwy_transition;
    if (serves) {
      result = kv.first;
      break;
    }
  }

  logging::info("[cifp] %s rwy %s last_fix=%s -> SID=%s", icao.c_str(),
                active_runway.c_str(), fpl_first_fix.c_str(),
                result.empty() ? "(none)" : result.c_str());

  {
    std::lock_guard<std::mutex> lk(g_alt_cache_mutex);
    g_sid_name_cache[cache_key] = result;
  }
  return result;
}

// ── sid_name_for_runway ────────────────────────────────────────────────

std::string sid_name_for_runway(const std::string &cifp_dir,
                                const std::string &icao,
                                const std::string &active_runway) {
  if (cifp_dir.empty() || icao.empty() || active_runway.empty())
    return {};

  std::string cache_key = icao + ":" + active_runway + ":NAME";
  {
    std::lock_guard<std::mutex> lk(g_alt_cache_mutex);
    auto it = g_sid_name_cache.find(cache_key);
    if (it != g_sid_name_cache.end())
      return it->second;
  }

  std::ifstream in(make_cifp_path(cifp_dir, icao));
  if (!in.good()) {
    std::lock_guard<std::mutex> lk(g_alt_cache_mutex);
    g_sid_name_cache[cache_key] = {};
    return {};
  }

  std::string rwy_match = "RW" + active_runway;
  std::string best_sid;

  std::string line;
  while (std::getline(in, line)) {
    if (line.size() < 4 || line.compare(0, 4, "SID:") != 0)
      continue;
    auto f = split_csv(line);
    if (f.size() < 5)
      continue;
    if (trim(f[3]) != rwy_match)
      continue;
    std::string sid = trim(f[2]);
    if (sid.empty())
      continue;
    // Alphabetically first SID is the representative ATC-assigned SID.
    if (best_sid.empty() || sid < best_sid)
      best_sid = sid;
  }

  logging::info("[cifp] %s rwy %s SID name -> %s", icao.c_str(),
                active_runway.c_str(),
                best_sid.empty() ? "(none)" : best_sid.c_str());
  {
    std::lock_guard<std::mutex> lk(g_alt_cache_mutex);
    g_sid_name_cache[cache_key] = best_sid;
  }
  return best_sid;
}

// ── sid_name_for_fix_prefix ────────────────────────────────────────────

std::string sid_name_for_fix_prefix(const std::string &cifp_dir,
                                    const std::string &icao,
                                    const std::string &prefix) {
  if (cifp_dir.empty() || icao.empty() || prefix.empty())
    return {};

  std::string cache_key = icao + ":PREFIX:" + prefix;
  {
    std::lock_guard<std::mutex> lk(g_alt_cache_mutex);
    auto it = g_sid_name_cache.find(cache_key);
    if (it != g_sid_name_cache.end())
      return it->second;
  }

  std::ifstream in(make_cifp_path(cifp_dir, icao));
  if (!in.good()) {
    std::lock_guard<std::mutex> lk(g_alt_cache_mutex);
    g_sid_name_cache[cache_key] = {};
    return {};
  }

  // Find the SID whose name starts with prefix (case-sensitive, exact prefix
  // match on first prefix.size() chars). Pick the alphabetically lowest match.
  std::string best_sid;
  std::string line;
  while (std::getline(in, line)) {
    if (line.size() < 4 || line.compare(0, 4, "SID:") != 0)
      continue;
    auto f = split_csv(line);
    if (f.size() < 3)
      continue;
    std::string sid = trim(f[2]);
    if (sid.size() < prefix.size())
      continue;
    if (sid.compare(0, prefix.size(), prefix) != 0)
      continue;
    if (best_sid.empty() || sid < best_sid)
      best_sid = sid;
  }

  logging::info("[cifp] %s prefix=%s -> SID=%s", icao.c_str(), prefix.c_str(),
                best_sid.empty() ? "(none)" : best_sid.c_str());
  {
    std::lock_guard<std::mutex> lk(g_alt_cache_mutex);
    g_sid_name_cache[cache_key] = best_sid;
  }
  return best_sid;
}

// ── sid_binding_altitude ────────────────────────────────────────────────

CifpBindingAlt sid_binding_altitude(const std::string &cifp_dir,
                                    const std::string &icao,
                                    const std::string &active_runway,
                                    const std::string &sid_name) {
  if (cifp_dir.empty() || icao.empty() || active_runway.empty())
    return {};

  std::string cache_key = icao + ":" + active_runway + ":" +
                          (sid_name.empty() ? "ALL" : sid_name) + ":BIND";
  {
    std::lock_guard<std::mutex> lk(g_alt_cache_mutex);
    auto it = g_binding_alt_cache.find(cache_key);
    if (it != g_binding_alt_cache.end())
      return it->second;
  }

  std::ifstream in(make_cifp_path(cifp_dir, icao));
  if (!in.good()) {
    std::lock_guard<std::mutex> lk(g_alt_cache_mutex);
    g_binding_alt_cache[cache_key] = {};
    return {};
  }

  std::string rwy_match = "RW" + active_runway;
  CifpBindingAlt best;

  std::string line;
  while (std::getline(in, line)) {
    if (line.size() < 4 || line.compare(0, 4, "SID:") != 0)
      continue;
    auto f = split_csv(line);
    if (f.size() < 24)
      continue;
    if (trim(f[3]) != rwy_match)
      continue;
    if (!sid_name.empty() && trim(f[2]) != sid_name)
      continue;

    std::string pterm = trim(f[11]);
    if (pterm != "DF" && pterm != "TF")
      continue; // only leg-end waypoints

    // f[22] = altitude descriptor: '+' = at or above (minimum constraint).
    // ' ' / '' with a non-empty f[23] = "at" altitude (also a hard minimum).
    std::string alt_desc = trim(f[22]);
    bool is_minimum =
        (alt_desc == "+") || (alt_desc.empty() && !trim(f[23]).empty());
    if (!is_minimum)
      continue;

    CifpAlt candidate = parse_alt(f[23]);
    if (candidate.feet <= 0)
      continue;

    if (candidate.feet > best.alt.feet) {
      best.alt = candidate;
      best.waypoint = trim(f[4]);
      best.sid = trim(f[2]);
    }
  }

  if (best.alt.feet > 0) {
    logging::debug(
        "[cifp] %s rwy %s binding min -> %d ft (is_fl=%d) at %s (%s)",
        icao.c_str(), active_runway.c_str(), best.alt.feet,
        best.alt.is_fl ? 1 : 0, best.waypoint.c_str(), best.sid.c_str());
  }
  {
    std::lock_guard<std::mutex> lk(g_alt_cache_mutex);
    g_binding_alt_cache[cache_key] = best;
  }
  return best;
}

// ── is_sid_valid_for_runway ────────────────────────────────────────────

bool is_sid_valid_for_runway(const std::string &cifp_dir,
                             const std::string &icao,
                             const std::string &sid_name,
                             const std::string &active_runway) {
  if (cifp_dir.empty() || icao.empty() || sid_name.empty() ||
      active_runway.empty())
    return false;

  std::ifstream in(make_cifp_path(cifp_dir, icao));
  if (!in.good())
    return false; // file absent — cannot validate

  std::string rwy_match = "RW" + active_runway;
  std::string line;
  while (std::getline(in, line)) {
    if (line.size() < 4 || line.compare(0, 4, "SID:") != 0)
      continue;
    auto f = split_csv(line);
    if (f.size() < 4)
      continue;
    if (trim(f[2]) == sid_name && trim(f[3]) == rwy_match)
      return true;
  }
  return false;
}

// ── sid_waypoints ─────────────────────────────────────────────────────────

// Own cache (g_star_waypoints_cache is defined further down, next to
// star_waypoints, so it isn't in scope here). Guarded by g_alt_cache_mutex.
static std::unordered_map<std::string, std::vector<StarWaypoint>>
    g_sid_waypoints_cache;

std::vector<StarWaypoint> sid_waypoints(const std::string &cifp_dir,
                                        const std::string &icao,
                                        const std::string &sid_name,
                                        const std::string &active_runway,
                                        bool constrained_only) {
  if (cifp_dir.empty() || icao.empty() || sid_name.empty())
    return {};

  const std::string cache_key = icao + ":SIDWPTS:" + sid_name + ":" +
                                (active_runway.empty() ? "ALL" : active_runway) +
                                (constrained_only ? ":C" : ":ALL");
  {
    std::lock_guard<std::mutex> lk(g_alt_cache_mutex);
    auto it = g_sid_waypoints_cache.find(cache_key);
    if (it != g_sid_waypoints_cache.end())
      return it->second;
  }

  std::ifstream in(make_cifp_path(cifp_dir, icao));
  std::vector<StarWaypoint> result;
  if (!in.good()) {
    std::lock_guard<std::mutex> lk(g_alt_cache_mutex);
    g_sid_waypoints_cache[cache_key] = result;
    return result;
  }

  const std::string rwy_match =
      active_runway.empty() ? "" : ("RW" + active_runway);

  std::string line;
  while (std::getline(in, line)) {
    if (line.size() < 4 || line.compare(0, 4, "SID:") != 0)
      continue;
    auto f = split_csv(line);
    if (f.size() < 28) continue;
    if (trim(f[2]) != sid_name) continue;

    // Keep our runway transition + the common route (f[3] not "RW.."); drop
    // other runways' transitions so we don't pull a foreign leg's constraint.
    const std::string rwy = trim(f[3]);
    const bool is_rw = (rwy.size() > 2 && rwy.compare(0, 2, "RW") == 0);
    if (is_rw && !rwy_match.empty() && rwy != rwy_match)
      continue;

    std::string seq_str = trim(f[0]);
    if (seq_str.size() <= 4) continue;
    int seq = 0;
    try { seq = std::stoi(seq_str.substr(4)); } catch (...) { continue; }

    std::string wpt = trim(f[4]);
    if (wpt.empty()) continue;

    const std::string pterm    = trim(f[11]);
    const std::string alt_desc = trim(f[22]);
    // Altitude column is path-term dependent (see initial_altitude): CF legs
    // carry the climb altitude at f[25], TF/DF at f[23].
    CifpAlt alt = (pterm == "CF" && f.size() > 25) ? parse_alt(f[25])
                                                   : parse_alt(f[23]);

    std::string spd_desc = trim(f[26]);
    int         speed_kt = 0;
    if (!spd_desc.empty() && f.size() > 27) {
      std::string sv = trim(f[27]);
      try { if (!sv.empty()) speed_kt = std::stoi(sv); }
      catch (...) { speed_kt = 0; } // NOLINT(bugprone-empty-catch)
    }

    if (constrained_only && alt.feet == 0 && speed_kt == 0)
      continue;

    StarWaypoint wp;
    wp.ident      = wpt;
    wp.alt        = alt;
    wp.is_ceiling = (alt_desc == "-");
    // "+" or a bare "at" altitude (empty descriptor with a value) is a climb
    // minimum on a SID — the aircraft must be at or above it.
    wp.is_floor   = (alt_desc == "+") || (alt_desc.empty() && alt.feet > 0);
    if (alt_desc == "B" && f.size() > 24) {
      wp.is_ceiling = true;
      wp.floor_ft   = parse_alt(f[24]).feet;
    }
    wp.speed_kt   = speed_kt;
    wp.seq        = seq;
    result.push_back(wp);
  }

  std::sort(result.begin(), result.end(),
            [](const StarWaypoint &a, const StarWaypoint &b) {
              return a.seq < b.seq;
            });

  logging::info("[cifp] %s SID %s rwy %s -> %d waypoints",
                icao.c_str(), sid_name.c_str(),
                active_runway.empty() ? "(any)" : active_runway.c_str(),
                static_cast<int>(result.size()));

  std::lock_guard<std::mutex> lk(g_alt_cache_mutex);
  g_sid_waypoints_cache[cache_key] = result;
  return result;
}

// ── best_approach ──────────────────────────────────────────────────────

ApproachInfo best_approach(const std::string &cifp_dir,
                            const std::string &icao,
                            const std::string &dest_runway,
                            float visibility_m) {
  if (cifp_dir.empty() || icao.empty() || dest_runway.empty())
    return {};

  std::string cache_key = icao + ":APP:" + dest_runway +
                          ":" + std::to_string(static_cast<int>(visibility_m));
  {
    std::lock_guard<std::mutex> lk(g_alt_cache_mutex);
    auto it = g_approach_cache.find(cache_key);
    if (it != g_approach_cache.end())
      return it->second;
  }

  std::ifstream in(make_cifp_path(cifp_dir, icao));
  if (!in.good()) {
    std::lock_guard<std::mutex> lk(g_alt_cache_mutex);
    g_approach_cache[cache_key] = {};
    return {};
  }

  int best_score = -1;
  ApproachInfo best;

  std::string line;
  while (std::getline(in, line)) {
    if (line.size() < 6 || line.compare(0, 6, "APPCH:") != 0)
      continue;
    auto f = split_csv(line);
    if (f.size() < 3) continue;

    std::string des = trim(f[2]);
    char type_char = 0, suffix = 0;
    std::string rwy;
    if (!parse_approach_designator(des, type_char, rwy, suffix))
      continue;
    if (rwy != dest_runway)
      continue;

    int prio = approach_priority(type_char, visibility_m);
    // Composite score: type priority dominates; variant letter breaks
    // ties within same type+runway.  ICAO convention is Z-first-published
    // so Z is the primary/preferred variant — Z > Y > X > ... > (none).
    int suffix_score =
        suffix ? std::toupper(static_cast<unsigned char>(suffix)) - 'A' + 1 : 0;
    int score = prio * 100 + suffix_score;
    if (score > best_score) {
      best_score       = score;
      best.type_str    = approach_type_str(type_char);
      best.runway      = rwy;
      best.designator  = des;
    }
  }

  logging::info("[cifp] %s rwy %s vis=%.0fm best approach -> %s (%s)",
                icao.c_str(), dest_runway.c_str(), visibility_m,
                best.designator.empty() ? "(none)" : best.designator.c_str(),
                best.type_str.empty()   ? "(none)" : best.type_str.c_str());

  std::lock_guard<std::mutex> lk(g_alt_cache_mutex);
  g_approach_cache[cache_key] = best;
  return best;
}

// ── approach_suffix ─────────────────────────────────────────────────────

// Public accessor: parses the designator and returns the variant letter,
// or 0 when the designator has no variant.  Handles the "R04-Y" dash form
// via parse_approach_designator's suffix scan (dash is skipped).
char approach_suffix(const std::string &designator) {
  char type_char = 0, suffix = 0;
  std::string rwy;
  parse_approach_designator(designator, type_char, rwy, suffix);
  return suffix;
}

// ── approach_by_designator ──────────────────────────────────────────────

ApproachInfo approach_by_designator(const std::string &cifp_dir,
                                     const std::string &icao,
                                     const std::string &designator) {
  if (cifp_dir.empty() || icao.empty() || designator.empty())
    return {};

  std::ifstream in(make_cifp_path(cifp_dir, icao));
  if (!in.good())
    return {};

  std::string line;
  while (std::getline(in, line)) {
    if (line.size() < 6 || line.compare(0, 6, "APPCH:") != 0)
      continue;
    auto f = split_csv(line);
    if (f.size() < 3) continue;
    if (trim(f[2]) != designator) continue;

    char type_char = 0;
    std::string rwy;
    if (!parse_approach_designator(designator, type_char, rwy))
      continue;

    ApproachInfo info;
    info.designator = designator;
    info.runway     = rwy;
    info.type_str   = approach_type_str(type_char);
    logging::info("[cifp] %s approach_by_designator %s -> %s rwy %s",
                  icao.c_str(), designator.c_str(),
                  info.type_str.c_str(), rwy.c_str());
    return info;
  }
  logging::info("[cifp] %s approach_by_designator %s -> not found",
                icao.c_str(), designator.c_str());
  return {};
}

// ── approach_terminates_at_runway ───────────────────────────────────────

// True if the approach's legs include a leg to the runway threshold (fix
// "RWxx" or subsection "G"). Straight-in instrument approaches with vertical
// guidance to a DA terminate at the runway; MDA / visual-final approaches
// (e.g. LFMN R04LA, R22LD) end at fixes + a missed-approach hold and never
// reach the runway -- the pilot flies the last segment visually. The CIFP
// carries no explicit DA/MDA value, so runway-leg presence is the reliable
// structural proxy. Used to pick "report established" (true = DA/instrument)
// vs "report runway in sight" (false = MDA/visual). Defaults to true
// (instrument) when data is unavailable -- the conservative choice.
bool approach_terminates_at_runway(const std::string &cifp_dir,
                                   const std::string &icao,
                                   const std::string &designator) {
  if (cifp_dir.empty() || icao.empty() || designator.empty())
    return true;
  std::ifstream in(make_cifp_path(cifp_dir, icao));
  if (!in.good())
    return true;
  std::string line;
  while (std::getline(in, line)) {
    if (line.size() < 6 || line.compare(0, 6, "APPCH:") != 0)
      continue;
    auto f = split_csv(line);
    if (f.size() < 8) continue;
    if (trim(f[2]) != designator) continue;
    if (trim(f[4]).rfind("RW", 0) == 0) return true; // leg to runway threshold
    if (trim(f[7]) == "G") return true;              // subsection G = runway
  }
  return false; // no runway leg -> visual / MDA last segment
}

// True if any leg of the approach carries a coded VERTICAL ANGLE (ARINC-424
// field f[28], e.g. "-350" = 3.50 deg) -- i.e. the approach publishes vertical
// guidance (ILS / LOC+GP / LPV / LNAV+VNAV) and is flown to a DECISION ALTITUDE.
// This is a more reliable DA/MDA signal than runway-leg presence: RNAV DA
// approaches (LFMD R35-Y/Z, LFMN R04LA) end at a fix + missed-approach hold in the
// CIFP yet still publish a vertical angle. Combined with
// approach_terminates_at_runway to pick "report established" (DA) vs "report
// runway in sight" (MDA/visual) -- user 2026-07-22 (LFMD RNAV 35 Z read as MDA).
bool approach_has_vertical_guidance(const std::string &cifp_dir,
                                    const std::string &icao,
                                    const std::string &designator) {
  if (cifp_dir.empty() || icao.empty() || designator.empty())
    return false;
  std::ifstream in(make_cifp_path(cifp_dir, icao));
  if (!in.good())
    return false;
  std::string line;
  while (std::getline(in, line)) {
    if (line.size() < 6 || line.compare(0, 6, "APPCH:") != 0)
      continue;
    auto f = split_csv(line);
    if (f.size() <= 28)
      continue;
    if (trim(f[2]) != designator)
      continue;
    const std::string va = trim(f[28]); // vertical angle, e.g. "-350" (3.50 deg)
    for (char c : va)
      if (std::isdigit(static_cast<unsigned char>(c)))
        return true; // any digit present -> a real angle is coded
  }
  return false;
}

// ── best_runway_for_approach ────────────────────────────────────────────

// Headwind alignment score [0, 100].  runway e.g. "04L", wind_from in true deg.
static int rwy_wind_score(const std::string &rwy, float wind_from_true) {
  if (rwy.size() < 2 || wind_from_true < 0.0f) return 50;
  int rwy_num = 0;
  for (int i = 0; i < 2; ++i) {
    if (!std::isdigit(static_cast<unsigned char>(rwy[i]))) return 50;
    rwy_num = rwy_num * 10 + (rwy[i] - '0');
  }
  if (rwy_num == 0 || rwy_num > 36) return 50;
  float diff = static_cast<float>(rwy_num) * 10.0f - wind_from_true;
  while (diff >  180.0f) diff -= 360.0f;
  while (diff < -180.0f) diff += 360.0f;
  float rad = diff * 3.14159265f / 180.0f;
  return static_cast<int>((::cos(static_cast<double>(rad)) + 1.0) * 50.0);
}

std::string best_runway_for_approach(const std::string &cifp_dir,
                                      const std::string &icao,
                                      float wind_dir_true,
                                      float visibility_m) {
  if (cifp_dir.empty() || icao.empty()) return {};

  std::ifstream in(make_cifp_path(cifp_dir, icao));
  if (!in.good()) return {};

  // Per runway: keep highest approach priority seen.
  std::unordered_map<std::string, int> rwy_prio;
  std::string line;
  while (std::getline(in, line)) {
    if (line.size() < 6 || line.compare(0, 6, "APPCH:") != 0) continue;
    auto f = split_csv(line);
    if (f.size() < 3) continue;
    char type_char = 0;
    std::string rwy;
    if (!parse_approach_designator(trim(f[2]), type_char, rwy)) continue;
    int prio = approach_priority(type_char, visibility_m);
    auto it = rwy_prio.find(rwy);
    if (it == rwy_prio.end() || prio > it->second)
      rwy_prio[rwy] = prio;
  }
  if (rwy_prio.empty()) return {};

  // Score each runway: approach_priority * 10000 + wind_score * 10 + L_bonus.
  // Highest score wins.  L_bonus prefers parallel L over R when wind is equal.
  std::string best_rwy;
  int best_score = -1;
  for (const auto &kv : rwy_prio) {
    const std::string &rwy = kv.first;
    int l_bonus = (!rwy.empty() && rwy.back() == 'L') ? 1 : 0;
    int score = kv.second * 10000
              + rwy_wind_score(rwy, wind_dir_true) * 10
              + l_bonus;
    if (score > best_score || (score == best_score && rwy < best_rwy)) {
      best_score = score;
      best_rwy   = rwy;
    }
  }

  logging::info("[cifp] %s wind=%.0f best runway -> %s",
                icao.c_str(), wind_dir_true, best_rwy.c_str());
  return best_rwy;
}

// ── star_entry_fix ─────────────────────────────────────────────────────

static std::unordered_map<std::string, StarEntryFix> g_star_entry_cache;

StarEntryFix star_entry_fix(const std::string &cifp_dir,
                             const std::string &icao,
                             const std::string &star_name) {
  if (cifp_dir.empty() || icao.empty() || star_name.empty())
    return {};

  std::string cache_key = icao + ":STAR:" + star_name;
  {
    std::lock_guard<std::mutex> lk(g_alt_cache_mutex);
    auto it = g_star_entry_cache.find(cache_key);
    if (it != g_star_entry_cache.end())
      return it->second;
  }

  std::ifstream in(make_cifp_path(cifp_dir, icao));
  if (!in.good()) {
    std::lock_guard<std::mutex> lk(g_alt_cache_mutex);
    g_star_entry_cache[cache_key] = {};
    return {};
  }

  int best_seq = INT_MAX;
  StarEntryFix best;
  best.star_name = star_name;

  std::string line;
  while (std::getline(in, line)) {
    if (line.size() < 5 || line.compare(0, 5, "STAR:") != 0)
      continue;
    auto f = split_csv(line);
    if (f.size() < 24) continue;
    if (trim(f[2]) != star_name) continue;

    std::string seq_str = trim(f[0]);
    if (seq_str.size() <= 5) continue;
    int seq = 0;
    try { seq = std::stoi(seq_str.substr(5)); } catch (...) { continue; }

    if (seq >= best_seq) continue;

    std::string wpt = trim(f[4]);
    if (wpt.empty()) continue;

    std::string alt_desc = trim(f[22]);
    CifpAlt     alt      = parse_alt(f[23]);

    best_seq       = seq;
    best.ident     = wpt;
    best.alt       = alt;
    best.is_ceiling = (alt_desc == "-");
  }

  if (!best.ident.empty()) {
    logging::info("[cifp] %s STAR %s entry fix -> %s alt=%d fl=%d ceiling=%d",
                  icao.c_str(), star_name.c_str(), best.ident.c_str(),
                  best.alt.feet, best.alt.is_fl ? 1 : 0,
                  best.is_ceiling ? 1 : 0);
  } else {
    logging::info("[cifp] %s STAR %s -> entry fix not found",
                  icao.c_str(), star_name.c_str());
  }

  std::lock_guard<std::mutex> lk(g_alt_cache_mutex);
  g_star_entry_cache[cache_key] = best;
  return best;
}

// ── star_name_for_entry_fix ─────────────────────────────────────────────

std::string star_name_for_entry_fix(const std::string &cifp_dir,
                                     const std::string &icao,
                                     const std::string &dest_runway,
                                     const std::string &entry_fix_ident) {
  if (cifp_dir.empty() || icao.empty() || entry_fix_ident.empty())
    return {};

  std::string cache_key = icao + ":STARENTRY:" + dest_runway + ":" + entry_fix_ident;
  {
    std::lock_guard<std::mutex> lk(g_alt_cache_mutex);
    auto it = g_sid_name_cache.find(cache_key);
    if (it != g_sid_name_cache.end())
      return it->second;
  }

  std::ifstream in(make_cifp_path(cifp_dir, icao));
  if (!in.good()) {
    std::lock_guard<std::mutex> lk(g_alt_cache_mutex);
    g_sid_name_cache[cache_key] = {};
    return {};
  }

  // Runway match: "ALL" always matches; specific runway matches dest_runway.
  std::string rwy_match = dest_runway.empty() ? "" : "RW" + dest_runway;

  // For each STAR, track lowest-sequence fix ident and the runway it serves.
  struct StarInfo { int min_seq = INT_MAX; std::string wpt; std::string rwy; };
  std::unordered_map<std::string, StarInfo> star_map;

  std::string line;
  while (std::getline(in, line)) {
    if (line.size() < 5 || line.compare(0, 5, "STAR:") != 0)
      continue;
    auto f = split_csv(line);
    if (f.size() < 5) continue;

    std::string rwy = trim(f[3]);
    if (!rwy_match.empty() && rwy != "ALL" && rwy != rwy_match)
      continue;

    std::string star = trim(f[2]);
    if (star.empty()) continue;

    std::string seq_str = trim(f[0]);
    if (seq_str.size() <= 5) continue;
    int seq = 0;
    try { seq = std::stoi(seq_str.substr(5)); } catch (...) { continue; }

    auto &info = star_map[star];
    if (seq < info.min_seq) {
      info.min_seq = seq;
      info.wpt     = trim(f[4]);
      info.rwy     = rwy;
    }
  }

  // Collect all candidates that have the matching entry fix, sorted by name
  // for determinism.  Among candidates, prefer a runway ending in 'L'
  // (left runway = typically the main runway for parallel pairs like 04L/04R).
  // STAR name suffixes (8T, 8R, etc.) are sequential codes unrelated to runway.
  std::vector<std::pair<std::string, std::string>> candidates; // (star_name, rwy)
  for (auto &kv : star_map) {
    if (kv.second.wpt == entry_fix_ident)
      candidates.push_back({kv.first, kv.second.rwy});
  }
  std::sort(candidates.begin(), candidates.end());

  std::string result;
  for (const auto &c : candidates) {
    if (!c.second.empty() && c.second.back() == 'L') {
      result = c.first; // L-runway STAR wins; first alphabetically among L-runways
      break;
    }
    if (result.empty())
      result = c.first; // fallback: first alphabetically among non-L candidates
  }

  logging::info("[cifp] %s rwy %s STAR entry_fix=%s -> STAR=%s",
                icao.c_str(), dest_runway.c_str(),
                entry_fix_ident.c_str(),
                result.empty() ? "(none)" : result.c_str());

  std::lock_guard<std::mutex> lk(g_alt_cache_mutex);
  g_sid_name_cache[cache_key] = result;
  return result;
}

// ── first_star_for_runway ────────────────────────────────────────────────

std::string first_star_for_runway(const std::string &cifp_dir,
                                   const std::string &icao,
                                   const std::string &dest_runway) {
  if (cifp_dir.empty() || icao.empty())
    return {};

  std::string cache_key = icao + ":FIRSTSTAR:" + dest_runway;
  {
    std::lock_guard<std::mutex> lk(g_alt_cache_mutex);
    auto it = g_sid_name_cache.find(cache_key);
    if (it != g_sid_name_cache.end())
      return it->second;
  }

  std::ifstream in(make_cifp_path(cifp_dir, icao));
  if (!in.good()) {
    std::lock_guard<std::mutex> lk(g_alt_cache_mutex);
    g_sid_name_cache[cache_key] = {};
    return {};
  }

  std::string rwy_match = dest_runway.empty() ? "" : "RW" + dest_runway;

  // Collect all STARs (no entry-fix filter), just runway filter.
  std::set<std::string> rwy_specific; // serve exactly this runway
  std::set<std::string> all_rwys;     // serve ALL runways

  std::string line;
  while (std::getline(in, line)) {
    if (line.size() < 5 || line.compare(0, 5, "STAR:") != 0)
      continue;
    auto f = split_csv(line);
    if (f.size() < 5) continue;
    std::string rwy  = trim(f[3]);
    std::string star = trim(f[2]);
    if (star.empty()) continue;
    if (rwy == "ALL")
      all_rwys.insert(star);
    else if (!rwy_match.empty() && rwy == rwy_match)
      rwy_specific.insert(star);
  }

  // Prefer runway-specific STARs; fall back to ALL.
  std::string result;
  if (!rwy_specific.empty())
    result = *rwy_specific.begin(); // alphabetically first
  else if (!all_rwys.empty())
    result = *all_rwys.begin();

  logging::info("[cifp] %s rwy %s first_star -> %s",
                icao.c_str(), dest_runway.c_str(),
                result.empty() ? "(none)" : result.c_str());

  std::lock_guard<std::mutex> lk(g_alt_cache_mutex);
  g_sid_name_cache[cache_key] = result;
  return result;
}

// ── runway_for_star ─────────────────────────────────────────────────────

std::string runway_for_star(const std::string &cifp_dir,
                             const std::string &icao,
                             const std::string &star_name) {
  if (cifp_dir.empty() || icao.empty() || star_name.empty())
    return {};

  std::string cache_key = icao + ":STARRUN:" + star_name;
  {
    std::lock_guard<std::mutex> lk(g_alt_cache_mutex);
    auto it = g_sid_name_cache.find(cache_key);
    if (it != g_sid_name_cache.end())
      return it->second;
  }

  std::ifstream in(make_cifp_path(cifp_dir, icao));
  if (!in.good()) {
    std::lock_guard<std::mutex> lk(g_alt_cache_mutex);
    g_sid_name_cache[cache_key] = {};
    return {};
  }

  std::string result;
  std::string line;
  while (std::getline(in, line)) {
    if (line.size() < 5 || line.compare(0, 5, "STAR:") != 0)
      continue;
    auto f = split_csv(line);
    if (f.size() < 4) continue;
    if (trim(f[2]) != star_name) continue;
    std::string rwy = trim(f[3]);
    if (rwy == "ALL" || rwy.empty()) continue;
    // Strip "RW" prefix (e.g. "RW04L" -> "04L")
    if (rwy.size() > 2 && rwy.compare(0, 2, "RW") == 0)
      rwy = rwy.substr(2);
    result = rwy;
    break;
  }

  logging::info("[cifp] %s STAR=%s -> runway=%s",
                icao.c_str(), star_name.c_str(),
                result.empty() ? "(ALL/none)" : result.c_str());

  std::lock_guard<std::mutex> lk(g_alt_cache_mutex);
  g_sid_name_cache[cache_key] = result;
  return result;
}

// ── star_waypoints ────────────────────────────────────────────────────────

static std::unordered_map<std::string, std::vector<StarWaypoint>>
    g_star_waypoints_cache;

std::vector<StarWaypoint> star_waypoints(const std::string &cifp_dir,
                                          const std::string &icao,
                                          const std::string &star_name,
                                          bool constrained_only) {
  if (cifp_dir.empty() || icao.empty() || star_name.empty())
    return {};

  std::string cache_key =
      icao + ":STARWPTS:" + star_name + (constrained_only ? ":C" : ":ALL");
  {
    std::lock_guard<std::mutex> lk(g_alt_cache_mutex);
    auto it = g_star_waypoints_cache.find(cache_key);
    if (it != g_star_waypoints_cache.end())
      return it->second;
  }

  std::ifstream in(make_cifp_path(cifp_dir, icao));
  std::vector<StarWaypoint> result;

  if (!in.good()) {
    std::lock_guard<std::mutex> lk(g_alt_cache_mutex);
    g_star_waypoints_cache[cache_key] = result;
    return result;
  }

  std::string line;
  while (std::getline(in, line)) {
    if (line.size() < 5 || line.compare(0, 5, "STAR:") != 0)
      continue;
    auto f = split_csv(line);
    if (f.size() < 28) continue;
    if (trim(f[2]) != star_name) continue;

    std::string seq_str = trim(f[0]);
    if (seq_str.size() <= 5) continue;
    int seq = 0;
    try { seq = std::stoi(seq_str.substr(5)); } catch (...) { continue; }

    std::string wpt = trim(f[4]);
    if (wpt.empty()) continue;

    std::string alt_desc  = trim(f[22]);
    CifpAlt     alt       = parse_alt(f[23]);
    std::string spd_desc  = trim(f[26]);
    int         speed_kt  = 0;
    if (!spd_desc.empty() && f.size() > 27) {
      std::string sv = trim(f[27]);
      try { if (!sv.empty()) speed_kt = std::stoi(sv); }
      catch (...) { speed_kt = 0; }
    }

    if (constrained_only && alt.feet == 0 && speed_kt == 0)
      continue; // no constraint — skip (full route table keeps it)

    StarWaypoint wp;
    wp.ident      = wpt;
    wp.alt        = alt;
    // "B" = between (block altitude): f[23] is the ceiling (in wp.alt),
    // f[24] is the floor. We keep BOTH: the ceiling lets the aircraft
    // descend into the block, the floor stops ATC clearing below it (e.g.
    // AMVA3P GOVNA "B FL090/6500" — descend to 6500 ft, not lower, until
    // the fix is behind). Discarding the floor caused the arrival to be
    // cleared below the block (LFMN MN261 -> SOTOX FL070).
    wp.is_ceiling = (alt_desc == "-" || alt_desc == "B");
    wp.is_floor   = (alt_desc == "+");
    if (alt_desc == "B" && f.size() > 24)
      wp.floor_ft = parse_alt(f[24]).feet; // 0 when the floor field is blank
    wp.speed_kt   = speed_kt;
    wp.seq        = seq;
    result.push_back(wp);
  }

  std::sort(result.begin(), result.end(),
            [](const StarWaypoint &a, const StarWaypoint &b) {
              return a.seq < b.seq;
            });

  logging::info("[cifp] %s STAR %s -> %d constrained waypoints",
                icao.c_str(), star_name.c_str(),
                static_cast<int>(result.size()));

  std::lock_guard<std::mutex> lk(g_alt_cache_mutex);
  g_star_waypoints_cache[cache_key] = result;
  return result;
}

// ── star_last_fix ─────────────────────────────────────────────────────────

std::string star_last_fix(const std::string &cifp_dir,
                           const std::string &icao,
                           const std::string &star_name) {
  if (cifp_dir.empty() || icao.empty() || star_name.empty())
    return {};

  const std::string cache_key = icao + ":STARLAST:" + star_name;
  {
    std::lock_guard<std::mutex> lk(g_alt_cache_mutex);
    auto it = g_last_fix_cache.find(cache_key);
    if (it != g_last_fix_cache.end())
      return it->second;
  }

  std::ifstream in(make_cifp_path(cifp_dir, icao));
  if (!in.good()) {
    std::lock_guard<std::mutex> lk(g_alt_cache_mutex);
    g_last_fix_cache[cache_key] = {};
    return {};
  }

  int best_seq = -1;
  std::string best_wpt;

  std::string line;
  while (std::getline(in, line)) {
    if (line.size() < 5 || line.compare(0, 5, "STAR:") != 0)
      continue;
    auto f = split_csv(line);
    if (f.size() < 5) continue;
    if (trim(f[2]) != star_name) continue;

    std::string seq_str = trim(f[0]);
    if (seq_str.size() <= 5) continue;
    int seq = 0;
    try { seq = std::stoi(seq_str.substr(5)); } catch (...) { continue; }

    std::string wpt = trim(f[4]);
    if (wpt.empty()) continue;

    if (seq > best_seq) {
      best_seq = seq;
      best_wpt = wpt;
    }
  }

  logging::info("[cifp] %s STAR %s last fix -> %s",
                icao.c_str(), star_name.c_str(),
                best_wpt.empty() ? "(none)" : best_wpt.c_str());

  std::lock_guard<std::mutex> lk(g_alt_cache_mutex);
  g_last_fix_cache[cache_key] = best_wpt;
  return best_wpt;
}

// ── connector_star ─────────────────────────────────────────────────────────

std::string connector_star(const std::string &cifp_dir,
                           const std::string &icao,
                           const std::string &from_fix,
                           const std::vector<std::string> &to_fixes) {
  if (cifp_dir.empty() || icao.empty() || from_fix.empty() || to_fixes.empty())
    return {};

  std::ifstream in(make_cifp_path(cifp_dir, icao));
  if (!in.good())
    return {};

  // Per STAR: entry fix (lowest seq) + terminating fix (highest seq).
  struct Ends {
    int min_seq = INT_MAX; std::string first;
    int max_seq = -1;      std::string last;
  };
  std::unordered_map<std::string, Ends> star_map;

  std::string line;
  while (std::getline(in, line)) {
    if (line.size() < 5 || line.compare(0, 5, "STAR:") != 0)
      continue;
    auto f = split_csv(line);
    if (f.size() < 5) continue;
    std::string star = trim(f[2]);
    if (star.empty()) continue;
    std::string seq_str = trim(f[0]);
    if (seq_str.size() <= 5) continue;
    int seq = 0;
    try { seq = std::stoi(seq_str.substr(5)); } catch (...) { continue; }
    std::string wpt = trim(f[4]);
    if (wpt.empty()) continue;
    auto &e = star_map[star];
    if (seq < e.min_seq) { e.min_seq = seq; e.first = wpt; }
    if (seq > e.max_seq) { e.max_seq = seq; e.last  = wpt; }
  }

  // A connector enters at from_fix and terminates at one of the approach IAFs.
  std::vector<std::string> matches;
  for (const auto &kv : star_map) {
    if (kv.second.first != from_fix)
      continue;
    for (const auto &t : to_fixes) {
      if (kv.second.last == t) { matches.push_back(kv.first); break; }
    }
  }
  std::sort(matches.begin(), matches.end());
  std::string result = matches.empty() ? std::string{} : matches.front();

  logging::info("[cifp] %s connector STAR from %s to approach IAF -> %s",
                icao.c_str(), from_fix.c_str(),
                result.empty() ? "(none)" : result.c_str());
  return result;
}

// ── approach_procedure_waypoints ─────────────────────────────────────────

std::vector<StarWaypoint> approach_procedure_waypoints(
    const std::string &cifp_dir,
    const std::string &icao,
    const std::string &approach_designator,
    const std::string &transition_ident,
    bool constrained_only) {

  if (cifp_dir.empty() || icao.empty() || approach_designator.empty() ||
      transition_ident.empty())
    return {};

  const std::string cache_key = icao + ":APPCHWPTS:" + approach_designator +
                                ":" + transition_ident +
                                (constrained_only ? ":C" : ":ALL");
  {
    std::lock_guard<std::mutex> lk(g_alt_cache_mutex);
    auto it = g_star_waypoints_cache.find(cache_key);
    if (it != g_star_waypoints_cache.end())
      return it->second;
  }

  std::ifstream in(make_cifp_path(cifp_dir, icao));
  std::vector<StarWaypoint> result;

  if (!in.good()) {
    std::lock_guard<std::mutex> lk(g_alt_cache_mutex);
    g_star_waypoints_cache[cache_key] = result;
    return result;
  }

  // Holding path terms — missed-approach patterns, not step-down fixes.
  auto is_holding = [](const std::string &pt) {
    return pt == "HM" || pt == "HA" || pt == "HF" || pt == "HX";
  };

  std::string line;
  while (std::getline(in, line)) {
    if (line.size() < 6 || line.compare(0, 6, "APPCH:") != 0) continue;
    auto f = split_csv(line);
    if (f.size() < 24) continue;
    if (trim(f[2]) != approach_designator) continue;

    const std::string rt        = trim(f[1]);
    const std::string path_term = trim(f[11]);

    if (rt == "A") {
      // IAF transition records: must match the requested transition ident.
      if (trim(f[3]) != transition_ident) continue;
      if (path_term == "FM") continue; // Fix-to-Manual = radar vector, no fixed endpoint
      if (path_term == "IF") continue; // IAF entry, already covered by STAR
    } else if (rt == "R") {
      // Final approach body: blank transition ident (main approach, not a named transition).
      if (!trim(f[3]).empty()) continue;
      if (path_term == "IF") continue; // IF entry = BISBO, already in transition above
      if (is_holding(path_term)) continue;
    } else {
      continue;
    }

    std::string wpt = trim(f[4]);
    if (wpt.empty()) continue;

    std::string seq_str = trim(f[0]);
    if (seq_str.size() <= 6) continue;
    int seq = 0;
    try { seq = std::stoi(seq_str.substr(6)); } catch (...) { continue; }

    // Distinguish A/R records by an offset so they sort correctly: R records
    // always come after the last A record in the procedure sequence.
    if (rt == "R") seq += 10000;

    std::string alt_desc = trim(f[22]);
    CifpAlt     alt      = parse_alt(f[23]);

    int speed_kt = 0;
    if (f.size() > 27) {
      std::string sv = trim(f[27]);
      try { if (!sv.empty()) speed_kt = std::stoi(sv); }
      catch (...) { speed_kt = 0; } // NOLINT(bugprone-empty-catch)
    }

    // constrained_only (default): drop fixes with neither alt nor speed
    // (matches star_waypoints; the old test dropped speed-only fixes too).
    // Full route table (false) keeps every fix.
    if (constrained_only && alt.feet == 0 && speed_kt == 0)
      continue;

    const std::string wpt_desc8 = f.size() > 8 ? trim(f[8]) : "";
    StarWaypoint wp;
    wp.ident            = wpt;
    wp.alt              = alt;
    wp.is_ceiling       = (alt_desc == "-" || alt_desc == "B");
    wp.is_floor         = (alt_desc == "+");
    if (alt_desc == "B" && f.size() > 24)
      wp.floor_ft = parse_alt(f[24]).feet; // block lower bound (0 if blank)
    wp.speed_kt         = speed_kt;
    wp.seq              = seq;
    wp.is_approach_proc = true;
    wp.is_map           = (wpt_desc8.size() >= 4 && wpt_desc8[3] == 'M');
    result.push_back(wp);
  }

  std::sort(result.begin(), result.end(),
            [](const StarWaypoint &a, const StarWaypoint &b) {
              return a.seq < b.seq;
            });

  logging::info("[cifp] %s %s IAF %s -> %d approach waypoints",
                icao.c_str(), approach_designator.c_str(),
                transition_ident.c_str(),
                static_cast<int>(result.size()));

  std::lock_guard<std::mutex> lk(g_alt_cache_mutex);
  g_star_waypoints_cache[cache_key] = result;
  return result;
}

// ── approach_iaf_fix ──────────────────────────────────────────────────────
// The IAF's OWN constraint, from the IF (Initial Fix) leg that
// approach_procedure_waypoints() skips at "if (path_term == \"IF\") continue;".
StarEntryFix approach_iaf_fix(const std::string &cifp_dir, const std::string &icao,
                              const std::string &approach_designator,
                              const std::string &iaf_ident) {
  StarEntryFix out;
  if (cifp_dir.empty() || icao.empty() || approach_designator.empty() ||
      iaf_ident.empty())
    return out;

  std::ifstream in(make_cifp_path(cifp_dir, icao));
  if (!in.good())
    return out;

  std::string line;
  while (std::getline(in, line)) {
    if (line.size() < 6 || line.compare(0, 6, "APPCH:") != 0) continue;
    auto f = split_csv(line);
    if (f.size() < 24) continue;
    if (trim(f[1]) != "A") continue;                 // named transition record
    if (trim(f[2]) != approach_designator) continue; // this approach
    if (trim(f[3]) != iaf_ident) continue;           // this IAF's transition
    if (trim(f[11]) != "IF") continue;               // the Initial Fix leg
    if (trim(f[4]) != iaf_ident) continue;           // the IAF fix itself
    const std::string alt_desc = trim(f[22]);
    out.ident      = iaf_ident;
    out.alt        = parse_alt(f[23]);
    out.is_ceiling = (alt_desc == "-" || alt_desc == "B");
    return out;
  }
  return out;
}

// ── approach_transition_idents ────────────────────────────────────────────

std::vector<std::string>
approach_transition_idents(const std::string &cifp_dir,
                            const std::string &icao,
                            const std::string &approach_designator) {
  if (cifp_dir.empty() || icao.empty() || approach_designator.empty())
    return {};

  std::ifstream in(make_cifp_path(cifp_dir, icao));
  if (!in.good()) return {};

  std::vector<std::string> result;
  std::string line;
  while (std::getline(in, line)) {
    if (line.size() < 6 || line.compare(0, 6, "APPCH:") != 0) continue;
    auto f = split_csv(line);
    if (f.size() < 5) continue;
    if (trim(f[1]) != "A") continue;
    if (trim(f[2]) != approach_designator) continue;
    std::string ident = trim(f[3]);
    if (ident.empty()) continue;
    if (std::find(result.begin(), result.end(), ident) == result.end())
      result.push_back(ident);
  }
  return result;
}

// ── approach_faf ─────────────────────────────────────────────────────────

FafFix approach_faf(const std::string &cifp_dir,
                    const std::string &icao,
                    const std::string &approach_designator) {
  if (cifp_dir.empty() || icao.empty() || approach_designator.empty())
    return {};

  const std::string cache_key = icao + ":FAF:" + approach_designator;
  {
    std::lock_guard<std::mutex> lk(g_alt_cache_mutex);
    auto it = g_faf_cache.find(cache_key);
    if (it != g_faf_cache.end())
      return it->second;
  }

  // ── Step 1: find FAF ident + altitude in CIFP ──────────────────────────
  // FAF is in the final-approach segment (route_type field f[1] == "I").
  // The 4th character of the waypoint description (f[8]) is 'F' for FAF.
  std::string faf_ident;
  int         faf_alt_ft    = 0;
  int         faf_track_deg = 0; // final approach track (field 20: e.g. "3530" = 353deg)

  {
    std::ifstream cifp_in(make_cifp_path(cifp_dir, icao));
    if (cifp_in.good()) {
      std::string line;
      while (std::getline(cifp_in, line)) {
        if (line.size() < 6 || line.compare(0, 6, "APPCH:") != 0)
          continue;
        auto f = split_csv(line);
        if (f.size() < 24)
          continue;
        const std::string rt = trim(f[1]);
        if (rt != "I" && rt != "R")           // final segment: ILS ("I") or RNAV ("R")
          continue;
        if (trim(f[2]) != approach_designator)
          continue;
        const std::string wpt_desc = trim(f[8]);
        if (wpt_desc.size() >= 4 && wpt_desc[3] == 'F') {
          faf_ident  = trim(f[4]);
          faf_alt_ft = parse_alt(f[23]).feet; // altitude1
          // Field 20: final approach track in 1/10 degree ("3530" = 353deg).
          if (f.size() > 20) {
            const std::string ts = trim(f[20]);
            if (!ts.empty() && std::isdigit(static_cast<unsigned char>(ts[0])))
              faf_track_deg = std::atoi(ts.c_str()) / 10;
          }
          break;
        }
      }
    }
  }

  if (faf_ident.empty()) {
    logging::info("[cifp] %s %s: FAF not found in CIFP",
                  icao.c_str(), approach_designator.c_str());
    std::lock_guard<std::mutex> lk(g_alt_cache_mutex);
    g_faf_cache[cache_key] = {};
    return {};
  }

  // ── Step 2: resolve lat/lon from earth_fix.dat ─────────────────────────
  // earth_fix.dat lives one directory above the CIFP directory.
  // Format (space-separated): lat lon ident airport_icao region ...
  double faf_lat = 0.0, faf_lon = 0.0;

  {
    // Strip trailing separators then remove the last path component (CIFP/).
    std::string navdata_dir = cifp_dir;
    while (!navdata_dir.empty() &&
           (navdata_dir.back() == '/' || navdata_dir.back() == '\\'))
      navdata_dir.pop_back();
    const auto slash     = navdata_dir.rfind('/');
    const auto backslash = navdata_dir.rfind('\\');
    const size_t last =
        (slash != std::string::npos && backslash != std::string::npos)
            ? std::max(slash, backslash)
            : (slash != std::string::npos ? slash : backslash);
    if (last != std::string::npos)
      navdata_dir = navdata_dir.substr(0, last + 1);
    else
      navdata_dir += '/';

    std::ifstream fix_in(navdata_dir + "earth_fix.dat");
    if (fix_in.good()) {
      std::string line;
      while (std::getline(fix_in, line)) {
        if (line.empty()) continue;
        std::istringstream ss(line);
        double lat, lon;
        std::string ident, airport, region;
        if (!(ss >> lat >> lon >> ident >> airport >> region))
          continue;
        if (ident == faf_ident && airport == icao) {
          faf_lat = lat;
          faf_lon = lon;
          break;
        }
      }
    }
  }

  FafFix result;
  result.ident          = faf_ident;
  result.lat            = faf_lat;
  result.lon            = faf_lon;
  result.alt_ft         = faf_alt_ft;
  result.final_track_deg = faf_track_deg;

  logging::info("[cifp] %s %s FAF: ident=%s lat=%.5f lon=%.5f alt=%dft track=%d",
                icao.c_str(), approach_designator.c_str(),
                faf_ident.c_str(), faf_lat, faf_lon, faf_alt_ft, faf_track_deg);

  // Only cache successful lookups — a lat=0/lon=0 result means earth_fix.dat
  // parsing failed (e.g. Navigraph leading-space format) and should be retried.
  if (faf_lat != 0.0 || faf_lon != 0.0) {
    std::lock_guard<std::mutex> lk(g_alt_cache_mutex);
    g_faf_cache[cache_key] = result;
  }
  return result;
}

// ────────────────────────────────────────────────────────────────────────

HoldSpec published_hold(const std::string &cifp_dir, const std::string &fix,
                        int alt_ft) {
  HoldSpec result;
  if (cifp_dir.empty() || fix.empty())
    return result;

  // earth_hold.dat lives one directory above the CIFP directory (sibling of
  // earth_fix.dat). Strip trailing separators then drop the last path component.
  std::string navdata_dir = cifp_dir;
  while (!navdata_dir.empty() &&
         (navdata_dir.back() == '/' || navdata_dir.back() == '\\'))
    navdata_dir.pop_back();
  const auto slash     = navdata_dir.rfind('/');
  const auto backslash = navdata_dir.rfind('\\');
  const size_t last =
      (slash != std::string::npos && backslash != std::string::npos)
          ? std::max(slash, backslash)
          : (slash != std::string::npos ? slash : backslash);
  if (last != std::string::npos)
    navdata_dir = navdata_dir.substr(0, last + 1);
  else
    navdata_dir += '/';

  std::ifstream in(navdata_dir + "earth_hold.dat");
  if (!in.good())
    return result;

  // Columns: FIX REGION TYPE ? INBOUND LEG_TIME LEG_DIST TURN MIN_ALT MAX_ALT SPEED
  // Keep the FIRST matching row as a fallback; prefer a row whose [min,max] band
  // contains alt_ft (max 0 = open ceiling).
  // Header lines ("I", "1140 Version - ...") and the "99" footer fail the token
  // parse below (non-numeric where a number is expected) or simply don't match
  // `fix`, so they are skipped without a special case.
  HoldSpec first_match;
  std::string line;
  while (std::getline(in, line)) {
    if (line.empty())
      continue;
    std::istringstream ss(line);
    std::string ident, region, type, dummy4, turn;
    double inbound = 0, leg_t = 0, leg_d = 0;
    int minA = 0, maxA = 0, spd = 0;
    if (!(ss >> ident >> region >> type >> dummy4 >> inbound >> leg_t >> leg_d >>
          turn >> minA >> maxA >> spd))
      continue;
    if (ident != fix)
      continue;
    HoldSpec h;
    h.fix               = ident;
    h.inbound_course_deg = static_cast<int>(std::lround(inbound));
    h.turn_right        = (turn != "L"); // L = left, anything else = right
    h.leg_time_min      = leg_t;
    h.leg_dist_nm       = leg_d;
    h.min_alt_ft        = minA;
    h.max_alt_ft        = maxA;
    h.max_speed_kt      = spd;
    h.valid             = true;
    if (!first_match.valid)
      first_match = h;
    const bool in_band = (minA == 0 || alt_ft >= minA) &&
                         (maxA == 0 || alt_ft <= maxA);
    if (in_band)
      return h; // best match for this altitude band
  }
  return first_match; // no band matched -> first row (valid=false if none)
}

// ────────────────────────────────────────────────────────────────────────

void clear_cache() {
  std::lock_guard<std::mutex> lk(g_alt_cache_mutex);
  g_alt_cache.clear();
  g_sid_name_cache.clear();
  g_binding_alt_cache.clear();
  g_star_entry_cache.clear();
  g_approach_cache.clear();
  g_star_waypoints_cache.clear();
  g_sid_waypoints_cache.clear();
  g_faf_cache.clear();
}

std::string preferred_departure_runway(const std::string &cifp_dir,
                                       const std::string &icao) {
  if (cifp_dir.empty() || icao.empty())
    return {};

  std::string dir = cifp_dir;
  if (dir.back() != '/')
    dir += '/';

  std::string icao_upper = icao;
  for (char &c : icao_upper)
    c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));

  std::ifstream in(dir + icao_upper + ".dat");
  if (!in.good())
    return {};

  // Count SID records per runway designator, return the one with most entries.
  std::unordered_map<std::string, int> counts;
  std::string line;
  while (std::getline(in, line)) {
    if (line.size() < 4 || line.compare(0, 4, "SID:") != 0)
      continue;
    auto f = split_csv(line);
    if (f.size() < 4)
      continue;
    std::string rwy = trim(f[3]);
    if (rwy.size() > 2 && rwy[0] == 'R' && rwy[1] == 'W')
      counts[rwy.substr(2)]++;
  }

  std::string best;
  int best_count = 0;
  for (auto &kv : counts) {
    if (kv.second > best_count) {
      best_count = kv.second;
      best = kv.first;
    }
  }
  return best;
}

// ── lookup_fix_positions ──────────────────────────────────────────────────

std::unordered_map<std::string, std::pair<double, double>>
lookup_fix_positions(const std::string &cifp_dir,
                     const std::vector<std::string> &idents,
                     const std::string &preferred_icao, double dest_lat,
                     double dest_lon) {
  if (cifp_dir.empty() || idents.empty())
    return {};

  std::unordered_set<std::string> wanted(idents.begin(), idents.end());

  // Derive earth_fix.dat path (one directory above CIFP/).
  std::string navdata_dir = cifp_dir;
  while (!navdata_dir.empty() &&
         (navdata_dir.back() == '/' || navdata_dir.back() == '\\'))
    navdata_dir.pop_back();
  const auto slash     = navdata_dir.rfind('/');
  const auto backslash = navdata_dir.rfind('\\');
  const size_t last =
      (slash != std::string::npos && backslash != std::string::npos)
          ? std::max(slash, backslash)
          : (slash != std::string::npos ? slash : backslash);
  if (last != std::string::npos)
    navdata_dir = navdata_dir.substr(0, last + 1);
  else
    navdata_dir += '/';

  const std::string country_pref =
      preferred_icao.size() >= 2 ? preferred_icao.substr(0, 2) : "";
  const bool have_dest = (dest_lat != 0.0 || dest_lon != 0.0);

  // preferred: airport-specific hit (airport field == preferred_icao) -- always wins.
  // cand: EVERY homonym (earth_fix.dat + earth_nav.dat) so proximity can disambiguate.
  // country_hit: first candidate whose region/country matches the dest country prefix
  // (legacy tie-breaker when no dest position is given).
  std::unordered_map<std::string, std::pair<double, double>> preferred;
  std::unordered_map<std::string, std::vector<std::pair<double, double>>> cand;
  std::unordered_map<std::string, std::pair<double, double>> country_hit;

  std::ifstream fix_in(navdata_dir + "earth_fix.dat");
  if (!fix_in.good())
    return {};
  std::string line;
  while (std::getline(fix_in, line)) {
    if (line.empty()) continue;
    std::istringstream ss(line);
    double lat, lon;
    std::string ident, airport, region;
    if (!(ss >> lat >> lon >> ident >> airport >> region))
      continue;
    if (!wanted.count(ident))
      continue;
    if (airport == preferred_icao)
      preferred[ident] = {lat, lon};
    cand[ident].push_back({lat, lon});
    if (!country_pref.empty() && region == country_pref && !country_hit.count(ident))
      country_hit[ident] = {lat, lon};
  }

  // earth_nav.dat: a VOR/NDB/DME is now ALWAYS a candidate (not only when the ident is
  // absent from earth_fix.dat) -- e.g. CBY is the CHAMBERY VOR here, while earth_fix.dat
  // only has a same-named Sydney racecourse fix. Types 2=NDB 3=VOR 12=DME 13=TACAN.
  {
    std::ifstream nav_in(navdata_dir + "earth_nav.dat");
    if (nav_in.good()) {
      std::string nline;
      while (std::getline(nav_in, nline)) {
        if (nline.empty()) continue;
        std::istringstream ss(nline);
        int type;
        if (!(ss >> type)) continue;
        if (type != 2 && type != 3 && type != 12 && type != 13) continue;
        double lat, lon;
        int elev, freq, range;
        double var;
        std::string ident, region, country;
        if (!(ss >> lat >> lon >> elev >> freq >> range >> var >> ident >> region >>
              country))
          continue;
        if (!wanted.count(ident)) continue;
        cand[ident].push_back({lat, lon});
        if (!country_pref.empty() && country == country_pref &&
            !country_hit.count(ident))
          country_hit[ident] = {lat, lon};
      }
    }
  }

  auto approx_nm2 = [](double la, double lo, double lb, double ob) {
    const double dlat = la - lb;
    const double dlon = (lo - ob) * std::cos(la * M_PI / 180.0);
    return dlat * dlat + dlon * dlon; // squared, monotone in real distance
  };

  std::unordered_map<std::string, std::pair<double, double>> result;
  for (const auto &id : idents) {
    // 1) airport-specific hit always wins.
    auto pit = preferred.find(id);
    if (pit != preferred.end()) { result[id] = pit->second; continue; }
    auto cit = cand.find(id);
    if (cit == cand.end() || cit->second.empty()) continue;
    // 2) nearest homonym to the destination when a position is known.
    if (have_dest) {
      const auto &cs = cit->second;
      size_t best = 0;
      double bestd = approx_nm2(cs[0].first, cs[0].second, dest_lat, dest_lon);
      for (size_t i = 1; i < cs.size(); ++i) {
        const double d = approx_nm2(cs[i].first, cs[i].second, dest_lat, dest_lon);
        if (d < bestd) { bestd = d; best = i; }
      }
      result[id] = cs[best];
      continue;
    }
    // 3) legacy: dest country match, else the first candidate.
    auto chit = country_hit.find(id);
    result[id] = (chit != country_hit.end()) ? chit->second : cit->second.front();
  }
  return result;
}

} // namespace cifp_reader
