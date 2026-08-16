/*
 * Unit tests for mora_db — the grid MORA reader.
 *
 * Written 2026-08-16 as the second prerequisite for FORCE APP VECTORING (see
 * docs/force-app-vectoring.md). msa_db answers the altitude question around a
 * procedure's centre point but only INSIDE the sector radius and only where the
 * airport publishes sectors; outside it correctly returns 0, meaning "I have
 * nothing to say". Grid MORA is the second net: worldwide, coarse, always there.
 *
 * The measured numbers this reader has to reproduce, from the reference data:
 *
 *     LOWI  MORA cell 14000, neighbourhood 15700, MSA around ELMEM 14300,
 *           IAF ELMEM published 13000  -> cannot descend into a vectoring
 *                                          pattern: vectors to the IAF
 *     EDLW  MORA cell  3700, neighbourhood  4400, platform 3000
 *                                       -> pattern at 5000 clears it:
 *                                          vectors to final
 *
 * Fixture: tests/fixtures/mora/earth_mora.dat — synthetic, on the same shape as
 * the real file (1-degree bands, 30-cell lines on 30-degree boundaries), with
 * the LOWI figures reproduced at 47N so the neighbourhood query is checked
 * against numbers that mean something.
 */

#include "data/mora_db.hpp"

#include <catch2/catch_amalgamated.hpp>
#include <string>

#ifndef XP_WELLYS_ATC_TEST_FIXTURES_DIR
#define XP_WELLYS_ATC_TEST_FIXTURES_DIR "tests/fixtures"
#endif

namespace {

struct Fixture {
  Fixture() {
    mora_db::init(std::string(XP_WELLYS_ATC_TEST_FIXTURES_DIR) +
                  "/mora/earth_mora.dat");
  }
  ~Fixture() { mora_db::stop(); }
};

} // namespace

TEST_CASE_METHOD(Fixture, "mora: the fixture loads", "[mora]") {
  REQUIRE(mora_db::ready());
  CHECK(mora_db::cell_count() > 0);
}

TEST_CASE_METHOD(Fixture, "mora: hundreds of feet are converted", "[mora]") {
  // "143" in the file means 14300 ft, not 143.
  CHECK(mora_db::minimum_ft(47.5, 10.5) == 14300);
}

TEST_CASE_METHOD(Fixture, "mora: a cell covers its whole 1x1 degree square",
                 "[mora]") {
  // Anywhere in [47,48) x [10,11) is the same cell. A 1-degree cell is ~60 NM
  // tall, which is exactly why MORA is a fallback and not a substitute for MSA.
  CHECK(mora_db::minimum_ft(47.01, 10.01) == 14300);
  CHECK(mora_db::minimum_ft(47.99, 10.99) == 14300);
  // One degree east is a different cell.
  CHECK(mora_db::minimum_ft(47.5, 11.5) == 15700);
}

TEST_CASE_METHOD(Fixture, "mora: 000 means no figure, not zero feet", "[mora]") {
  // An unsurveyed / open-water cell reports 0, and the caller must treat that as
  // "no value available" -- never as permission to descend.
  CHECK(mora_db::minimum_ft(47.5, 12.5) == 0);
}

TEST_CASE_METHOD(Fixture, "mora: the neighbourhood query takes the worst cell",
                 "[mora]") {
  // A vectoring pattern spans tens of miles, so the cell the aircraft sits in is
  // not enough: the leg can cross into a higher one. At 47.5N/10.5E the cell is
  // 14300 but the neighbour east is 15700 -- the pattern must clear 15700.
  CHECK(mora_db::minimum_ft(47.5, 10.5) == 14300);
  CHECK(mora_db::minimum_ft_around(47.5, 10.5) == 15700);
}

TEST_CASE_METHOD(Fixture, "mora: an empty neighbour does not lower the answer",
                 "[mora]") {
  // 47N/12E carries no figure. Asking around 47.5/11.5 must still return the
  // 15700 of the cell itself rather than being dragged down by the 0.
  CHECK(mora_db::minimum_ft_around(47.5, 11.5) == 15700);
}

TEST_CASE_METHOD(Fixture, "mora: negative latitude and longitude", "[mora]") {
  // The file writes both fields signed (-10 / -030); getting the sign wrong
  // would silently mirror the whole southern or western hemisphere.
  CHECK(mora_db::minimum_ft(-9.5, -29.5) == 5500);
  CHECK(mora_db::minimum_ft(-9.5, -28.5) == 5000);
}

TEST_CASE_METHOD(Fixture, "mora: the neighbourhood wraps the antimeridian",
                 "[mora]") {
  // 0N/179E and 0N/180W are adjacent on the ground. Without the wrap, a pattern
  // flown across the antimeridian silently loses half its neighbours.
  CHECK(mora_db::minimum_ft(0.5, 179.5) == 6600);
  CHECK(mora_db::minimum_ft(0.5, -179.5) == 7700);
  CHECK(mora_db::minimum_ft_around(0.5, 179.5) == 7700);
}

TEST_CASE_METHOD(Fixture, "mora: out of range returns 0", "[mora]") {
  CHECK(mora_db::minimum_ft(91.0, 10.0) == 0);
  CHECK(mora_db::minimum_ft(-91.0, 10.0) == 0);
}

TEST_CASE_METHOD(Fixture, "mora: longitude outside [-180,180) is normalised",
                 "[mora]") {
  // 190 E is 170 W. Normalising rather than rejecting keeps callers from having
  // to sanitise a wrapped great-circle computation.
  CHECK(mora_db::minimum_ft(47.5, 10.5 + 360.0) == 14300);
  CHECK(mora_db::minimum_ft(47.5, 10.5 - 360.0) == 14300);
}

TEST_CASE("mora: an empty path disables cleanly", "[mora]") {
  mora_db::init("");
  CHECK(mora_db::ready());
  CHECK(mora_db::cell_count() == 0);
  CHECK(mora_db::minimum_ft(47.5, 10.5) == 0);
  mora_db::stop();
}
