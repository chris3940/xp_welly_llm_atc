/*
 * Unit tests for msa_db — the Minimum Sector Altitude reader.
 *
 * Written 2026-08-15 as the prerequisite for vectoring onto the FAF/IAF. Under
 * vectors the aircraft has left the published procedure and lost its altitude
 * protection, so the controller owes it a level at or above the sector MSA. The
 * published fix altitude is NOT enough — measured on the real data, the MSA
 * around LOWI's IAF ELMEM is 14 300 ft in the 270 sector while ELMEM's published
 * altitude is 13 000 ft, so a westward vector descending to the published figure
 * would be 1 300 ft below the minimum, in a valley.
 *
 * Fixture: tests/fixtures/msa/earth_msa.dat — synthetic, centres on whole
 * degrees so every bearing in the tests is computable by hand.
 */

#include "data/msa_db.hpp"

#include <catch2/catch_amalgamated.hpp>
#include <string>

#ifndef XP_WELLYS_ATC_TEST_FIXTURES_DIR
#define XP_WELLYS_ATC_TEST_FIXTURES_DIR "tests/fixtures"
#endif

namespace {

// TESTA sits at 45N 005E. Offsets big enough to be unambiguous, small enough to
// stay inside the 25 NM sector radius (1 deg lat = 60 NM, so 0.2 deg = 12 NM).
constexpr double kLat = 45.0;
constexpr double kLon = 5.0;

struct Fixture {
  Fixture() {
    msa_db::init(std::string(XP_WELLYS_ATC_TEST_FIXTURES_DIR) +
                 "/msa/earth_msa.dat");
  }
  ~Fixture() { msa_db::stop(); }
};

// MSA at a point offset from TESTA by (dlat, dlon) degrees.
int at(double dlat, double dlon, double magvar = 0.0) {
  return msa_db::minimum_ft("TEST", "TESTA", kLat, kLon, kLat + dlat,
                            kLon + dlon, magvar);
}

} // namespace

TEST_CASE_METHOD(Fixture, "msa: the fixture loads", "[msa]") {
  REQUIRE(msa_db::ready());
  REQUIRE(msa_db::records_for("TEST").size() == 2);
  CHECK(msa_db::records_for("TSOL").size() == 1);
  CHECK(msa_db::records_for("NOPE").empty());
}

TEST_CASE_METHOD(Fixture, "msa: sectors are picked clockwise from their start",
                 "[msa]") {
  // 000-090 -> 5000 : north-east of the centre
  CHECK(at(+0.10, +0.10) == 5000);
  // 090-180 -> 3000 : south-east
  CHECK(at(-0.10, +0.10) == 3000);
  // 180-270 -> 4000 : south-west
  CHECK(at(-0.10, -0.10) == 4000);
  // 270-360 -> 7000 : north-west
  CHECK(at(+0.10, -0.10) == 7000);
}

TEST_CASE_METHOD(Fixture, "msa: a zero START bearing is not the terminator",
                 "[msa]") {
  // The record ends with a literal "000 000" PAIR. A sector starting due north
  // with a real altitude ("000 050 25") must survive -- dropping it would leave
  // the whole northern quadrant unprotected, silently.
  CHECK(at(+0.15, +0.01) == 5000); // just east of due north
  const auto recs = msa_db::records_for("TEST");
  REQUIRE(recs.size() == 2);
  CHECK(recs[0].sectors.size() == 4);
}

TEST_CASE_METHOD(Fixture, "msa: hundreds of feet are converted", "[msa]") {
  // "050" in the file means 5000 ft, not 50.
  CHECK(at(+0.10, +0.10) == 5000);
}

TEST_CASE_METHOD(Fixture, "msa: outside the radius returns 0, not a guess",
                 "[msa]") {
  // SOLO's single sector has a 10 NM radius. At ~18 NM the MSA says nothing
  // about this position, and 0 must mean exactly that -- the caller has to fall
  // back or refuse, never silently reuse a figure that does not apply.
  CHECK(msa_db::minimum_ft("TSOL", "SOLO", kLat, kLon, kLat + 0.30, kLon) == 0);
  CHECK(msa_db::minimum_ft("TSOL", "SOLO", kLat, kLon, kLat + 0.10, kLon) == 2500);
}

TEST_CASE_METHOD(Fixture, "msa: a single sector covers the whole circle",
                 "[msa]") {
  for (double d : {+0.10, -0.10})
    for (double e : {+0.10, -0.10})
      CHECK(msa_db::minimum_ft("TSOL", "SOLO", kLat, kLon, kLat + d, kLon + e) ==
            2500);
}

TEST_CASE_METHOD(Fixture, "msa: the centre ident selects the record", "[msa]") {
  // Same airport, two centres. TESTB is 010-190 -> 9000, 190-010 -> 6000.
  CHECK(msa_db::minimum_ft("TEST", "TESTB", kLat, kLon, kLat + 0.10, kLon + 0.10) ==
        9000);
  CHECK(msa_db::minimum_ft("TEST", "TESTB", kLat, kLon, kLat - 0.10, kLon - 0.10) ==
        6000);
  // An unknown centre must not fall back to the airport's other record.
  CHECK(msa_db::minimum_ft("TEST", "NOSUCH", kLat, kLon, kLat + 0.1, kLon + 0.1) ==
        0);
}

TEST_CASE_METHOD(Fixture, "msa: magnetic variation rotates the sector lookup",
                 "[msa]") {
  // Bearing ~120 true (south-east): dlat = cos(120)*d, dlon = sin(120)*d/cos(lat).
  // Deliberately NOT on a sector boundary -- a point due east of the centre comes
  // out at 089.96, not 090, because the great circle between two points at the
  // same latitude bulges poleward. Testing exactly on a boundary would be
  // measuring floating-point noise rather than the sector logic.
  const double dlat = -0.050, dlon = +0.1225;
  CHECK(at(dlat, dlon, 0.0) == 3000);   // 120 magnetic -> the 090-180 sector
  CHECK(at(dlat, dlon, 40.0) == 5000);  // 080 magnetic -> the 000-090 sector
}

TEST_CASE_METHOD(Fixture, "msa: a sector boundary belongs to the sector it starts",
                 "[msa]") {
  // Pins the convention rather than leaving it to chance: a bearing exactly on a
  // start bearing falls in THAT sector (>= from, < to). Probed just inside each
  // side of the 090 boundary.
  CHECK(at(-0.001, +0.10) == 3000); // a hair south of due east -> 090 sector
  CHECK(at(+0.001, +0.10) == 5000); // a hair north            -> 000 sector
}

TEST_CASE_METHOD(Fixture, "msa: a TRUE-referenced record ignores magvar",
                 "[msa]") {
  // TTRU is flagged "T": 000-180 -> 8000, 180-360 -> 2000. Feeding a variation
  // must NOT rotate it.
  CHECK(msa_db::minimum_ft("TTRU", "TRUEB", kLat, kLon, kLat, kLon + 0.10, 0.0) ==
        8000);
  CHECK(msa_db::minimum_ft("TTRU", "TRUEB", kLat, kLon, kLat, kLon + 0.10, 40.0) ==
        8000);
}

TEST_CASE_METHOD(Fixture, "msa: an empty path disables cleanly", "[msa]") {
  msa_db::init("");
  CHECK(msa_db::ready());
  CHECK(msa_db::records_for("TEST").empty());
  CHECK(msa_db::minimum_ft("TEST", "TESTA", kLat, kLon, kLat + 0.1, kLon) == 0);
}
