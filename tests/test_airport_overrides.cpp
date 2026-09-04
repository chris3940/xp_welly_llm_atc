// Coverage for the hand-maintained airport+.json overrides.
//
// WRITTEN AFTER THE FACT, and that is the point. The SID initial-climb table
// carries seventeen rules for LFMN alone, was shipped as "reader wired", and had
// no test of any kind -- so nobody noticed that a real departure never reached
// it: the clearance announced "initial climb to 5000 feet" for BASI8A, which
// publishes FL100 for jets and FL070 for props (real flight 2026-08-26, user:
// "c'est fou que tu ne testes pas plus tes dev"). A table this size with no
// assertion is a promise, not a feature. [C. P. Potter]

#include <catch2/catch_amalgamated.hpp>

#include "data/airport_overrides.hpp"

#include <cstdlib>
#include <string>

namespace {
// init() takes the FILE, not the directory -- it opens the path verbatim and
// stays silent when it cannot, which is how a wrong argument reads exactly like
// an empty table.
std::string overrides_file() {
  const char *home = std::getenv("HOME");
  return std::string(home ? home : "")
         + "/Apps/Scripts/PERSO/AI-atc-XP12-plugin/repo/Resources/airport+.json";
}
} // namespace

TEST_CASE("airport+.json: LFMN publishes a SID initial climb per departure",
          "[overrides][sid]") {
  airport_overrides::init(overrides_file());

  // BASI8A -- the departure flown on 2026-08-26. Jet and prop differ.
  REQUIRE(airport_overrides::sid_initial_climb_ft("LFMN", "BASI8A", true) == 10000);
  REQUIRE(airport_overrides::sid_initial_climb_ft("LFMN", "BASI8A", false) == 7000);

  // A single-value rule answers the same whatever the aircraft.
  REQUIRE(airport_overrides::sid_initial_climb_ft("LFMN", "PERU8A", true) == 13000);
  REQUIRE(airport_overrides::sid_initial_climb_ft("LFMN", "PERU8A", false) == 13000);

  // The jet/prop split exists on more than one departure.
  REQUIRE(airport_overrides::sid_initial_climb_ft("LFMN", "EPOL8A", true) == 10000);
  REQUIRE(airport_overrides::sid_initial_climb_ft("LFMN", "EPOL8B", true) == 7000);

  // No rule -> 0, so the caller keeps the CIFP value rather than inventing one.
  REQUIRE(airport_overrides::sid_initial_climb_ft("LFMN", "ZZZZ9Z", true) == 0);
  REQUIRE(airport_overrides::sid_initial_climb_ft("EDLW", "ANYTHING", true) == 0);
}

// The overlay must be able to name its own frequencies. The plugin hands the
// pilot 128.975 at Innsbruck -- a frequency that exists nowhere but this file --
// and then has to recognise it when the pilot tunes it, or the whole arrival
// classifies as UNKNOWN and no clearance is ever due (real flight 2026-08-28).
// [C. P. Potter]
TEST_CASE("airport+.json can classify its own frequencies",
          "[airport_overrides][freq]") {
  std::string role, name;

  SECTION("the primary approach frequency") {
    REQUIRE(airport_overrides::role_for_freq("LOWI", 128.975f, &role, &name));
    REQUIRE(role == "APPROACH");
    REQUIRE(name == "Innsbruck Radar");
  }
  SECTION("an alt_freqs_mhz alternate, which nothing read before") {
    REQUIRE(airport_overrides::role_for_freq("LOWI", 119.275f, &role, &name));
    REQUIRE(role == "APPROACH");
  }
  SECTION("the tower") {
    REQUIRE(airport_overrides::role_for_freq("LOWI", 120.100f, &role, &name));
    REQUIRE(role == "TOWER");
    REQUIRE(name == "Innsbruck Tower");
  }
  SECTION("an 8.33 kHz channel designator names the same channel") {
    REQUIRE(airport_overrides::role_for_freq("LFLP", 121.205f, &role, &name));
    REQUIRE(role == "APPROACH");
  }
  SECTION("a frequency belonging to no overlay controller") {
    REQUIRE_FALSE(airport_overrides::role_for_freq("LOWI", 121.500f, &role, &name));
  }
  SECTION("an airport with no overlay at all") {
    REQUIRE_FALSE(airport_overrides::role_for_freq("EDDM", 128.975f, &role, &name));
  }
}
