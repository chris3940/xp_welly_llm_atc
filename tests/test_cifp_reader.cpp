/*
 * Unit tests for cifp_reader — SID selection, initial altitude, last-fix lookup.
 * Uses a minimal CIFP fixture at tests/fixtures/cifp/LFLP.dat containing:
 *   BULO2A (last fix: BULOS), ODIK2A (last fix: ODIKI),
 *   LTP2A  (last fix: LTPNO), ROMA2A (last fix: ROMAM)
 *   All on RW22. Initial altitude 6500 ft (CF record at seq 010).
 */

#include "data/cifp_reader.hpp"

#include <catch2/catch_amalgamated.hpp>
#include <string>

// Fixture directory injected by CMake.
#ifndef XP_WELLYS_ATC_TEST_FIXTURES_DIR
#define XP_WELLYS_ATC_TEST_FIXTURES_DIR "tests/fixtures"
#endif

static const std::string kCifpDir =
    std::string(XP_WELLYS_ATC_TEST_FIXTURES_DIR) + "/cifp";

// Clear the reader cache between test cases so each test is independent.
static void reset() { cifp_reader::clear_cache(); }

// ── initial_altitude ──────────────────────────────────────────────────

TEST_CASE("cifp: initial_altitude returns 6500 ft for LFLP RW22", "[cifp][lflp]") {
  reset();
  auto alt = cifp_reader::initial_altitude(kCifpDir, "LFLP", "22");
  REQUIRE(alt.feet == 6500);
  REQUIRE(!alt.is_fl);
}

TEST_CASE("cifp: initial_altitude returns cached result on second call", "[cifp][lflp]") {
  reset();
  auto a1 = cifp_reader::initial_altitude(kCifpDir, "LFLP", "22");
  auto a2 = cifp_reader::initial_altitude(kCifpDir, "LFLP", "22");
  REQUIRE(a1.feet == a2.feet);
}

TEST_CASE("cifp: initial_altitude returns 0 for unknown airport", "[cifp]") {
  reset();
  auto alt = cifp_reader::initial_altitude(kCifpDir, "ZZZZ", "22");
  REQUIRE(alt.feet == 0);
}

TEST_CASE("cifp: initial_altitude returns 0 for empty inputs", "[cifp]") {
  reset();
  REQUIRE(cifp_reader::initial_altitude("", "LFLP", "22").feet == 0);
  REQUIRE(cifp_reader::initial_altitude(kCifpDir, "", "22").feet == 0);
  REQUIRE(cifp_reader::initial_altitude(kCifpDir, "LFLP", "").feet == 0);
}

// ── sid_name_for_runway (alphabetically first) ────────────────────────

TEST_CASE("cifp: sid_name_for_runway returns alphabetically first SID", "[cifp][lflp]") {
  reset();
  // BULO2A < LTP2A < ODIK2A < ROMA2A
  std::string sid = cifp_reader::sid_name_for_runway(kCifpDir, "LFLP", "22");
  REQUIRE(sid == "BULO2A");
}

TEST_CASE("cifp: sid_name_for_runway returns empty for unknown runway", "[cifp]") {
  reset();
  std::string sid = cifp_reader::sid_name_for_runway(kCifpDir, "LFLP", "04");
  REQUIRE(sid.empty());
}

// ── sid_name_for_last_fix ─────────────────────────────────────────────

TEST_CASE("cifp: sid_name_for_last_fix finds ODIK2A when last fix is ODIKI", "[cifp][lflp]") {
  reset();
  std::string sid =
      cifp_reader::sid_name_for_last_fix(kCifpDir, "LFLP", "22", "ODIKI");
  REQUIRE(sid == "ODIK2A");
}

TEST_CASE("cifp: sid_name_for_last_fix finds LTP2A when last fix is LTPNO", "[cifp][lflp]") {
  reset();
  std::string sid =
      cifp_reader::sid_name_for_last_fix(kCifpDir, "LFLP", "22", "LTPNO");
  REQUIRE(sid == "LTP2A");
}

TEST_CASE("cifp: sid_name_for_last_fix finds ROMA2A when last fix is ROMAM", "[cifp][lflp]") {
  reset();
  std::string sid =
      cifp_reader::sid_name_for_last_fix(kCifpDir, "LFLP", "22", "ROMAM");
  REQUIRE(sid == "ROMA2A");
}

TEST_CASE("cifp: sid_name_for_last_fix returns empty for unknown last fix", "[cifp][lflp]") {
  reset();
  std::string sid =
      cifp_reader::sid_name_for_last_fix(kCifpDir, "LFLP", "22", "AMBET");
  REQUIRE(sid.empty());
}

TEST_CASE("cifp: sid_name_for_last_fix returns empty for unknown airport", "[cifp]") {
  reset();
  std::string sid =
      cifp_reader::sid_name_for_last_fix(kCifpDir, "ZZZZ", "22", "ODIKI");
  REQUIRE(sid.empty());
}

TEST_CASE("cifp: sid_name_for_last_fix returns empty for empty inputs", "[cifp]") {
  reset();
  REQUIRE(cifp_reader::sid_name_for_last_fix("", "LFLP", "22", "ODIKI").empty());
  REQUIRE(cifp_reader::sid_name_for_last_fix(kCifpDir, "", "22", "ODIKI").empty());
  // Note: an empty active_runway is NOT an empty input -- it triggers an
  // any-runway search (see "empty runway searches all runways" below).
  REQUIRE(cifp_reader::sid_name_for_last_fix(kCifpDir, "LFLP", "22", "").empty());
}

TEST_CASE("cifp: empty runway searches all runways", "[cifp]") {
  reset();
  // active_runway="" means "search every runway at the airport" (used by the
  // SimBrief navlog path before the active runway is known). ODIKI is the last
  // fix of SID ODIK2A, so it must still resolve.
  REQUIRE(cifp_reader::sid_name_for_last_fix(kCifpDir, "LFLP", "", "ODIKI") ==
          "ODIK2A");
}

// ── is_sid_valid_for_runway ───────────────────────────────────────────

TEST_CASE("cifp: is_sid_valid_for_runway accepts known SID", "[cifp][lflp]") {
  reset();
  REQUIRE(cifp_reader::is_sid_valid_for_runway(kCifpDir, "LFLP", "ODIK2A", "22"));
  REQUIRE(cifp_reader::is_sid_valid_for_runway(kCifpDir, "LFLP", "LTP2A",  "22"));
}

TEST_CASE("cifp: is_sid_valid_for_runway rejects SID not in CIFP", "[cifp][lflp]") {
  reset();
  REQUIRE(!cifp_reader::is_sid_valid_for_runway(kCifpDir, "LFLP", "AMBET2A", "22"));
}

TEST_CASE("cifp: is_sid_valid_for_runway returns false for unknown airport", "[cifp]") {
  reset();
  REQUIRE(!cifp_reader::is_sid_valid_for_runway(kCifpDir, "ZZZZ", "ODIK2A", "22"));
}

// ── approach_suffix ──────────────────────────────────────────────────
// Extracts the trailing variant letter (Z, Y, X, ...) from an approach
// designator. Must handle both the compact form ("I04LZ") and the
// dash-separated form some AIRAC vendors emit for LFLP ("R04-Y").
// Returns 0 when no variant letter is present.

TEST_CASE("cifp: approach_suffix extracts trailing letter", "[cifp][approach]") {
  CHECK(cifp_reader::approach_suffix("I04LZ") == 'Z');
  CHECK(cifp_reader::approach_suffix("I04LY") == 'Y');
  CHECK(cifp_reader::approach_suffix("R04-Y") == 'Y');
  CHECK(cifp_reader::approach_suffix("R04-Z") == 'Z');
  CHECK(cifp_reader::approach_suffix("R04Z")  == 'Z');
  CHECK(cifp_reader::approach_suffix("I04L")  == 0);     // no variant
  CHECK(cifp_reader::approach_suffix("R22")   == 0);
  CHECK(cifp_reader::approach_suffix("")      == 0);
}

// ── ils_approach: backs the "force ILS if available" setting ──────────
// The setting short-circuits the whole approach-selection chain when the
// ARRIVAL RUNWAY has an ILS published, and must stay out of the way when it
// does not. Fixture: LFMN.dat, 04L with ILS + RNAV, 22L with RNAV/VOR only.

TEST_CASE("cifp: ils_approach picks the Z variant on a runway with an ILS",
          "[cifp][approach][ils]") {
  reset();
  const auto a = cifp_reader::ils_approach(kCifpDir, "LFMN", "04L");
  CHECK(a.designator == "I04LZ"); // Z beats Y, same tie-break as best_approach
  CHECK(a.type_str == "ILS");
  CHECK(a.runway == "04L");
}

TEST_CASE("cifp: ils_approach returns nothing on a runway with no ILS",
          "[cifp][approach][ils]") {
  reset();
  // 22L publishes RNAV (R22LD/R22LZ) and VOR/DME (D22LB) only. Force-ILS must
  // fall through here rather than dragging in the OTHER end's ILS -- landing
  // 22L on the 04L localizer would be the worst possible failure mode.
  const auto a = cifp_reader::ils_approach(kCifpDir, "LFMN", "22L");
  CHECK(a.designator.empty());
  CHECK(a.type_str.empty());
}

TEST_CASE("cifp: ils_approach is empty for unknown airport / empty inputs",
          "[cifp][approach][ils]") {
  reset();
  CHECK(cifp_reader::ils_approach(kCifpDir, "ZZZZ", "04L").type_str.empty());
  CHECK(cifp_reader::ils_approach(kCifpDir, "LFMN", "").type_str.empty());
  CHECK(cifp_reader::ils_approach("", "LFMN", "04L").type_str.empty());
}

// ── approach_procedure_waypoints: the final body of a NON-RNAV approach ──
// The route type of an APPCH record is the APPROACH TYPE letter (I = ILS,
// R = RNAV, D = VOR/DME, L = LOC, N = NDB), not a fixed "R". The reader used to
// accept only "R", so every non-RNAV approach silently lost its whole final
// segment: EDLW ILS 06 yielded just the two transition legs, the FAF index came
// out -1, and the Tower handoff -- which triggers at the FAF -- could never fire
// (real vol 2026-08-14). RNAV arrivals hid it because they were the only match.

TEST_CASE("cifp: ILS approach keeps its final segment, not just the transition",
          "[cifp][approach][ils]") {
  reset();
  const auto wps = cifp_reader::approach_procedure_waypoints(
      kCifpDir, "LFMN", "I04LZ", "MUS");
  // Transition (IF04L via the MUS transition) plus the final body. Without the
  // fix only the transition legs survived.
  REQUIRE(wps.size() >= 3);
  std::vector<std::string> idents;
  for (const auto &w : wps)
    idents.push_back(w.ident);
  const auto has = [&](const std::string &s) {
    return std::find(idents.begin(), idents.end(), s) != idents.end();
  };
  CHECK(has("FN04L")); // the FAF -- descriptor "E  F"
  CHECK(has("RW04L")); // the MAP  -- descriptor "G  M"
}

TEST_CASE("cifp: the ILS final body sorts after the transition",
          "[cifp][approach][ils]") {
  reset();
  const auto wps = cifp_reader::approach_procedure_waypoints(
      kCifpDir, "LFMN", "I04LZ", "MUS");
  REQUIRE(wps.size() >= 3);
  // The +10000 sequence offset must key on "not a transition", not on "R", or
  // an ILS final body interleaves with its own transition. Expected order:
  // MUS, IF04L (transition) then FN04L, RW04L (final body).
  CHECK(wps.front().ident == "MUS");
  CHECK(wps.back().ident == "RW04L");
}

TEST_CASE("cifp: the MAP is flagged on a non-RNAV approach",
          "[cifp][approach][ils]") {
  reset();
  const auto wps = cifp_reader::approach_procedure_waypoints(
      kCifpDir, "LFMN", "I04LZ", "MUS");
  bool map_seen = false;
  for (const auto &w : wps)
    if (w.ident == "RW04L")
      map_seen = w.is_map;
  CHECK(map_seen);
}

// ── star_waypoints: STAR-lookahead constraint scan (P0-A) ─────────────
// build_descent_clearance's STAR-lookahead clears the initial descent to
// the first "at or below" constraint on the STAR (LUVOB FL090 on SALE3P)
// instead of the cruise*0.66 fallback. This pins the data source: the
// first is_ceiling waypoint must be LUVOB at FL090.

TEST_CASE("cifp: star_waypoints SALE3P first at-or-below is LUVOB FL090",
          "[cifp][lflp][star]") {
  reset();
  auto wps = cifp_reader::star_waypoints(kCifpDir, "LFLP", "SALE3P");
  // Only constrained waypoints are returned (LUVOB, GOVNA, PIRUV).
  REQUIRE(wps.size() >= 3);
  // First "at or below" (is_ceiling) constraint governs the initial descent.
  const cifp_reader::StarWaypoint *first_ceiling = nullptr;
  for (const auto &w : wps) {
    if (w.is_ceiling && w.alt.feet > 0) {
      first_ceiling = &w;
      break;
    }
  }
  REQUIRE(first_ceiling != nullptr);
  CHECK(first_ceiling->ident == "LUVOB");
  CHECK(first_ceiling->alt.feet == 9000);
  CHECK(first_ceiling->alt.is_fl);
}

// ── best_approach: Zulu tie-break ────────────────────────────────────
// LFLP fixture has both R04-Y and R04-Z with identical RNAV type on
// runway 04. Before the tie-break fix, the file-order-first entry won
// (Y). ICAO convention is Z = first published = primary, so best_approach
// must return R04-Z regardless of file order.

TEST_CASE("cifp: best_approach prefers Zulu variant on same-type tie",
          "[cifp][lflp][approach]") {
  reset();
  auto appr = cifp_reader::best_approach(kCifpDir, "LFLP", "04");
  REQUIRE(!appr.designator.empty());
  CHECK(appr.designator == "R04-Z");
  CHECK(appr.runway == "04");
  CHECK(appr.type_str == "RNAV");
}
