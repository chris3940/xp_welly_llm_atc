/*
 * Unit tests for the openair_db -> atc.dat fallback.
 *
 * Why it exists (user, 2026-08-15): the OpenAir airspace file ships with a PAID
 * navdata subscription, while atc.dat comes with a standard X-Plane install.
 * Without a fallback, a user without that subscription has NO volumes at all --
 * no sector handoffs, no TMA logic, no descend-to-enter -- which is the entire
 * IFR core of the plugin. atc.dat carries the same shelved geometry, so it can
 * stand in.
 *
 * The contract these tests pin is the PRECEDENCE, because getting it backwards
 * would silently degrade every subscriber:
 *
 *   1. OpenAir answers        -> OpenAir wins, atc.dat is never consulted.
 *   2. OpenAir is silent here -> atc.dat fills in (this also patches genuine
 *                                holes for subscribers: the class E blanket over
 *                                Dortmund leaves 4500-10000 ft uncovered, and
 *                                the export carries no class E at all).
 *   3. OpenAir absent         -> everything comes from atc.dat.
 *
 * Fixtures: tests/fixtures/atcdat/atc.dat is laid out on the SAME lat 40..41 grid
 * as tests/fixtures/openair/classify.txt, so both sources can be probed at the
 * same points and their precedence observed directly. Square 0 is deliberately
 * present in BOTH; square 9 exists only in atc.dat.
 */

#include "data/airspace_db.hpp"
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

constexpr double kLat = 40.5;
double lon_of(int square) { return static_cast<double>(square) + 0.5; }

std::string fixtures() { return std::string(XP_WELLYS_ATC_TEST_FIXTURES_DIR); }

void wait_openair() {
  for (int i = 0; i < 200 && !openair_db::ready(); ++i)
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
}

// Both loaders parse on a background thread. Waiting only on OpenAir is not enough:
// init("") makes it ready instantly, so a "no subscription" case would race atc.dat
// and see an empty index rather than the fallback.
void wait_airspace() {
  for (int i = 0; i < 200 && !airspace_db::ready(); ++i)
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
}

// atc.dat always loaded; OpenAir loaded only when with_openair is set, so the
// "no subscription" case is a real absence rather than an empty file.
struct Fixture {
  explicit Fixture(bool with_openair) {
    airspace_db::init(fixtures() + "/atcdat/atc.dat");
    if (with_openair) {
      openair_db::init(fixtures() + "/openair/classify.txt");
      wait_openair();
    } else {
      openair_db::init("");
      wait_openair();
    }
    wait_airspace();
  }
  ~Fixture() {
    openair_db::stop();
    airspace_db::stop();
  }
};

struct WithOpenAir : Fixture {
  WithOpenAir() : Fixture(true) {}
};
struct WithoutOpenAir : Fixture {
  WithoutOpenAir() : Fixture(false) {}
};

} // namespace

TEST_CASE_METHOD(WithOpenAir, "fallback: both sources load", "[openair][atcdat]") {
  REQUIRE(openair_db::ready());
  REQUIRE(airspace_db::ready());
  REQUIRE(airspace_db::controller_count() == 3);
}

// ── 1. OpenAir wins wherever it answers ───────────────────────────────

TEST_CASE_METHOD(WithOpenAir, "fallback: OpenAir wins on an overlap",
                 "[openair][atcdat]") {
  // Square 0 is ALPHA CTR in OpenAir and FALLBACK CTR in atc.dat, both covering
  // 1000 ft. A subscriber must keep the OpenAir answer byte for byte -- if this
  // ever flips, every installed subscription silently changes behaviour.
  const auto e = openair_db::find_enclosing(kLat, lon_of(0), 1000);
  CHECK(e.name == "ALPHA CTR");
  CHECK(e.ac_class == AC::CTR);
}

// ── 2. OpenAir silent -> atc.dat fills the hole ───────────────────────

TEST_CASE_METHOD(WithOpenAir, "fallback: a hole in OpenAir is filled by atc.dat",
                 "[openair][atcdat]") {
  // Square 9 exists ONLY in atc.dat. This is the Dortmund case in miniature: the
  // subscriber has a file, it simply says nothing here.
  const auto e = openair_db::find_enclosing(kLat, lon_of(9), 5000);
  REQUIRE_FALSE(e.name.empty());
  CHECK(e.name == "LONE TRACON");
}

TEST_CASE_METHOD(WithOpenAir, "fallback: the innermost atc.dat volume wins",
                 "[openair][atcdat]") {
  // LONE TRACON (1 deg square) sits inside LONE CENTER (3 deg square); both
  // contain the point at 5000 ft. Innermost must win, exactly as OpenAir picks
  // the smallest bbox.
  const auto e = openair_db::find_enclosing(kLat, lon_of(9), 5000);
  CHECK(e.name == "LONE TRACON");
  // Above the tracon ceiling only the centre remains.
  const auto above = openair_db::find_enclosing(kLat, lon_of(9), 15000);
  CHECK(above.name == "LONE CENTER");
}

TEST_CASE_METHOD(WithOpenAir, "fallback: the ring's own floor and ceiling are used",
                 "[openair][atcdat]") {
  // A controller is a stack of shelves; the caller needs the shelf the aircraft
  // is in, not the record's first polygon.
  const auto e = openair_db::find_enclosing(kLat, lon_of(9), 5000);
  CHECK(e.floor_ft == 1500);
  CHECK(e.ceiling_ft == 10000);
}

TEST_CASE_METHOD(WithOpenAir, "fallback: below the floor, no volume is invented",
                 "[openair][atcdat]") {
  // 500 ft is under LONE TRACON's 1500 ft floor. The centre (floor 0) still
  // covers it -- what must NOT happen is the tracon being returned anyway.
  const auto e = openair_db::find_enclosing(kLat, lon_of(9), 500);
  CHECK(e.name == "LONE CENTER");
}

// ── 3. No OpenAir at all (no subscription) ────────────────────────────

TEST_CASE_METHOD(WithoutOpenAir, "fallback: without OpenAir everything comes from atc.dat",
                 "[openair][atcdat]") {
  const auto e = openair_db::find_enclosing(kLat, lon_of(9), 5000);
  REQUIRE_FALSE(e.name.empty());
  CHECK(e.name == "LONE TRACON");
  // The square that OpenAir would have owned now answers from atc.dat.
  const auto z = openair_db::find_enclosing(kLat, lon_of(0), 1000);
  CHECK(z.name == "FALLBACK CTR");
}

// ── Role -> class mapping ─────────────────────────────────────────────

TEST_CASE_METHOD(WithoutOpenAir, "fallback: roles map to airspace classes",
                 "[openair][atcdat]") {
  CHECK(openair_db::find_enclosing(kLat, lon_of(0), 1000).ac_class == AC::CTR);
  CHECK(openair_db::find_enclosing(kLat, lon_of(9), 5000).ac_class == AC::TMA);
  CHECK(openair_db::find_enclosing(kLat, lon_of(9), 15000).ac_class == AC::CTA);
}

TEST_CASE_METHOD(WithoutOpenAir, "fallback: terminal_tma does NOT fall back",
                 "[openair][atcdat]") {
  // Deliberate asymmetry, and the reason is worth keeping: dest_terminal_tma_below()
  // probes the AIRCRAFT's position as well as the destination's, so a fallback made
  // atc.dat answer with a NEIGHBOURING terminal area which the guard mistook for the
  // destination's -- "descend-to-enter terminal area -> FL090" fired 63 NM out on the
  // DIK -> EDLW replay (2026-08-15). Silence is what that guard needs: it reads "no
  // volume" as "can't tell" and stays permissive. find_enclosing() keeps the fallback.
  CHECK(openair_db::terminal_tma(kLat, lon_of(9)).name.empty());
  CHECK(openair_db::terminal_tma_ceiling(kLat, lon_of(9)) == 0);
  // ... while the enclosing query at the very same point still answers.
  CHECK(openair_db::find_enclosing(kLat, lon_of(9), 5000).name == "LONE TRACON");
}

TEST_CASE_METHOD(WithoutOpenAir, "fallback: outside everything stays empty",
                 "[openair][atcdat]") {
  // The honest answer when neither source knows the point. Inventing a volume
  // here would be worse than silence: upstream guards read "no volume" as
  // "can't tell" and stay permissive on purpose.
  const auto e = openair_db::find_enclosing(kLat, 100.0, 5000);
  CHECK(e.name.empty());
  CHECK(openair_db::terminal_tma_ceiling(kLat, 100.0) == 0);
}
