/*
 * Unit tests for openair_db — CLASSIFICATION and the geometry queries built on it.
 *
 * Written 2026-08-14 after the LFLP -> EDLW flight, where a terminal-area descent
 * fired 186 NM from the destination. The root cause was classification, not
 * geometry: openair_db decides what an airspace IS from a keyword in its NAME
 * (CTR / TMA / CTA / FIR), because most European exports carry the ICAO class
 * LETTER in the AC record rather than the airspace type. Anything whose name has
 * no such keyword is classified OTHER and never indexed, so find_enclosing()
 * returns nothing over it — which silently opened a "can't tell -> don't block"
 * guard upstream.
 *
 * These tests pin the CURRENT behaviour, gaps included, so that changing the
 * classifier to key on the ICAO class letter is a measurable diff rather than a
 * leap of faith. The two cases marked GAP are the ones that would flip.
 *
 * Fixture: tests/fixtures/openair/classify.txt — synthetic, one 1-degree square
 * per case on a lon grid at lat 40..41, so every probe point is unambiguous.
 */

#include "data/openair_db.hpp"

#include <catch2/catch_amalgamated.hpp>
#include <chrono>
#include <string>
#include <thread>

#ifndef XP_WELLYS_ATC_TEST_FIXTURES_DIR
#define XP_WELLYS_ATC_TEST_FIXTURES_DIR "tests/fixtures"
#endif

namespace {

using AC = openair_db::AirspaceClass;

// Centre of the square occupying [lon_deg, lon_deg + 1].
constexpr double kLat = 40.5;
double lon_of(int square) { return static_cast<double>(square) + 0.5; }

struct Fixture {
  Fixture() {
    openair_db::init(std::string(XP_WELLYS_ATC_TEST_FIXTURES_DIR) +
                     "/openair/classify.txt");
    // init() parses on a background thread; wait for it rather than racing.
    for (int i = 0; i < 200 && !openair_db::ready(); ++i)
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  ~Fixture() { openair_db::stop(); }
};

} // namespace

TEST_CASE_METHOD(Fixture, "openair: the fixture loads", "[openair]") {
  REQUIRE(openair_db::ready());
}

// ── Classification ────────────────────────────────────────────────────

TEST_CASE_METHOD(Fixture, "openair: AC CTR is a control zone", "[openair]") {
  const auto e = openair_db::find_enclosing(kLat, lon_of(0), 1000);
  CHECK(e.name == "ALPHA CTR");
  CHECK(e.ac_class == AC::CTR);
}

TEST_CASE_METHOD(Fixture,
                 "openair: a class LETTER is upgraded by the name keyword",
                 "[openair]") {
  // The AC record says "D" / "C" / "A" -- the ICAO class, not the type. The
  // type comes from the name. This is the mechanism the whole index rests on.
  const auto tma = openair_db::find_enclosing(kLat, lon_of(1), 5000);
  CHECK(tma.name == "BRAVO TMA");
  CHECK(tma.ac_class == AC::TMA);

  const auto cta = openair_db::find_enclosing(kLat, lon_of(2), 12000);
  CHECK(cta.name == "CHARLIE CTA");
  CHECK(cta.ac_class == AC::CTA);

  const auto fir = openair_db::find_enclosing(kLat, lon_of(3), 30000);
  CHECK(fir.name == "DELTA FIR");
  CHECK(fir.ac_class == AC::FIR);
}

TEST_CASE_METHOD(Fixture, "openair: a delegation polygon indexes as CTA",
                 "[openair]") {
  // Named by its delegation marker rather than a type word; must still be
  // indexed so the enroute resolver can route it to the delegated ACC.
  const auto e = openair_db::find_enclosing(kLat, lon_of(7), 30000);
  CHECK(e.name == "DELEGATED BY AAAA TO BBBB");
  CHECK(e.ac_class == AC::CTA);
}

TEST_CASE_METHOD(Fixture, "openair: restricted areas are never indexed",
                 "[openair]") {
  // R / P / Q must stay out: they are not control airspace and must never
  // resolve to a controller. This one must NOT flip if the classifier changes.
  const auto e = openair_db::find_enclosing(kLat, lon_of(6), 10000);
  CHECK(e.name.empty());
}

// ── The two gaps that caused the 2026-08-14 misfire ───────────────────
// GAP: both squares below are real control airspace whose name carries no type
// keyword, so today they are invisible. Pinned deliberately -- if the classifier
// is ever changed to key on the ICAO class letter, THESE are the assertions that
// must be inverted, and their inversion is the whole point of that change.

TEST_CASE_METHOD(Fixture,
                 "openair: GAP -- a Free Route block is not indexed today",
                 "[openair][gap]") {
  // Real-world instance: `FREE RT ASPC E` covers Nancy at FL280 (Reims UIR), and
  // find_enclosing returns nothing there -- logged as "openair empty" on the
  // LFLP -> EDLW flight.
  const auto e = openair_db::find_enclosing(kLat, lon_of(4), 30000);
  CHECK(e.name.empty());
}

TEST_CASE_METHOD(Fixture,
                 "openair: GAP -- a bare class-letter volume is not indexed today",
                 "[openair][gap]") {
  // Real-world instance: `DORTMUND` (2000-4500) and `DORTMUND SECTOR B` carry no
  // keyword, so only `DORTMUND CTR` survives at EDLW.
  const auto e = openair_db::find_enclosing(kLat, lon_of(5), 3000);
  CHECK(e.name.empty());
}

// ── Geometry queries built on the index ───────────────────────────────

TEST_CASE_METHOD(Fixture, "openair: altitude band is respected", "[openair]") {
  // BRAVO TMA is 2500-9500. Below and above it, nothing else covers that square.
  CHECK(openair_db::find_enclosing(kLat, lon_of(1), 2000).name.empty());
  CHECK(openair_db::find_enclosing(kLat, lon_of(1), 5000).name == "BRAVO TMA");
  CHECK(openair_db::find_enclosing(kLat, lon_of(1), 12000).name.empty());
}

TEST_CASE_METHOD(Fixture, "openair: a point outside every polygon resolves to nothing",
                 "[openair]") {
  const auto e = openair_db::find_enclosing(50.0, 50.0, 10000);
  CHECK(e.name.empty());
  CHECK(e.ac_class == AC::OTHER);
}

TEST_CASE_METHOD(Fixture, "openair: terminal_tma picks the LOWEST-floor TMA",
                 "[openair]") {
  // GOLF TMA is stacked 1000-8500 / 8500-19500. The terminal query must return
  // the base block -- that is the shelf an arrival descends INTO -- regardless
  // of where the aircraft currently is.
  const auto t = openair_db::terminal_tma(kLat, lon_of(8));
  CHECK(t.name == "GOLF TMA SECTOR 1");
  CHECK(t.floor_ft == 1000);
  CHECK(t.ceiling_ft == 8500);
  CHECK(openair_db::terminal_tma_ceiling(kLat, lon_of(8)) == 8500);
}

TEST_CASE_METHOD(Fixture, "openair: terminal_tma ignores CTR and CTA",
                 "[openair]") {
  // Only TMA-class volumes are terminal areas: a CTR underneath (ALPHA CTR) and
  // an enroute CTA (CHARLIE CTA) must not be returned.
  CHECK(openair_db::terminal_tma(kLat, lon_of(0)).name.empty());
  CHECK(openair_db::terminal_tma_ceiling(kLat, lon_of(0)) == 0);
  CHECK(openair_db::terminal_tma(kLat, lon_of(2)).name.empty());
}

TEST_CASE_METHOD(Fixture, "openair: highest_tma_ceiling takes the top of the stack",
                 "[openair]") {
  // Counterpart to terminal_tma: an enroute descent must stay ABOVE an overflown
  // TMA, so this one reports the TOP (19500), not the base block's 8500.
  CHECK(openair_db::highest_tma_ceiling(kLat, lon_of(8)) == 19500);
}

TEST_CASE_METHOD(Fixture, "openair: ctr_ceiling_ft reports the control zone top",
                 "[openair]") {
  CHECK(openair_db::ctr_ceiling_ft(kLat, lon_of(0)) == 2500);
  CHECK(openair_db::ctr_ceiling_ft(kLat, lon_of(1)) == 0); // TMA, not a CTR
}

TEST_CASE_METHOD(Fixture, "openair: find_all_enclosing returns the whole stack",
                 "[openair]") {
  // At 10000 ft over the GOLF square only the upper block contains the point;
  // the query must not collapse a stack to its first hit.
  const auto all = openair_db::find_all_enclosing(kLat, lon_of(8), 10000);
  REQUIRE(all.size() == 1);
  CHECK(all[0].name == "GOLF TMA SECTOR 2");

  const auto low = openair_db::find_all_enclosing(kLat, lon_of(8), 4000);
  REQUIRE(low.size() == 1);
  CHECK(low[0].name == "GOLF TMA SECTOR 1");
}
