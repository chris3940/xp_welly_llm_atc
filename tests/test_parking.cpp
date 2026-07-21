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

// Unit tests for the general-aviation stand matcher (pick_ga_stand) +
// icao_size_code_for_wingspan. Fixture mirrors the LFLP (Annecy) apt.dat GA
// stands: all size A/B, no size C. Verifies a Falcon 7X (Code C jet) falls back
// to the largest B jet stand, and a TBM (Code A turboprop) gets a size-A stand.

#include "core/xplane_context.hpp"

#include <catch2/catch_amalgamated.hpp>
#include <string>
#include <vector>

using xplane_context::EngineKind;
using xplane_context::icao_size_code_for_wingspan;
using xplane_context::ParkingStand;
using xplane_context::pick_ga_stand;

namespace {

// LFLP GA stands (from the real apt.dat). Coords identical so distance ties fall
// through to the size rule (what we are testing).
std::vector<ParkingStand> lflp_stands() {
  auto mk = [](const char *n, char sz, bool j, bool t, bool p) {
    ParkingStand s;
    s.name = n;
    s.lat = 45.929;
    s.lon = 6.099;
    s.size_code = sz;
    s.general_aviation = true;
    s.jets = j;
    s.turboprops = t;
    s.props = p;
    return s;
  };
  return {
      mk("GA-B size Ramp Start 12", 'B', true, true, false),
      mk("GA Ramp Start 22", 'B', true, true, true),
      mk("GA Ramp Start FL250", 'A', false, true, true),
      mk("GA Ramp Start Avialpes", 'A', false, false, true),
      mk("GA Ramp Start 74", 'A', false, false, true),
      mk("GA Ramp 32", 'A', true, false, true),
      mk("GA Ramp 111", 'B', true, false, true),
  };
}

} // namespace

TEST_CASE("ga-stand: wingspan -> ICAO size code", "[parking]") {
  REQUIRE(icao_size_code_for_wingspan(12.8f) == 'A'); // TBM
  REQUIRE(icao_size_code_for_wingspan(20.0f) == 'B');
  REQUIRE(icao_size_code_for_wingspan(26.2f) == 'C'); // Falcon 7X
  REQUIRE(icao_size_code_for_wingspan(60.0f) == 'E');
}

TEST_CASE("ga-stand: Falcon 7X (Code C jet) -> largest B jet stand", "[parking]") {
  const auto s = lflp_stands();
  const std::string pick = pick_ga_stand(
      s, icao_size_code_for_wingspan(26.2f), EngineKind::Jet, 45.929, 6.099);
  REQUIRE_FALSE(pick.empty());
  // No Code-C stand exists -> fall back to a size-B jet stand.
  const bool b_jet = pick == "GA-B size Ramp Start 12" ||
                     pick == "GA Ramp Start 22" || pick == "GA Ramp 111";
  REQUIRE(b_jet);
}

TEST_CASE("ga-stand: TBM (Code A turboprop) -> a size-A stand", "[parking]") {
  const auto s = lflp_stands();
  const std::string pick =
      pick_ga_stand(s, icao_size_code_for_wingspan(12.8f),
                    EngineKind::Turboprop, 45.929, 6.099);
  REQUIRE_FALSE(pick.empty());
  const bool a_stand =
      pick == "GA Ramp Start FL250" || pick == "GA Ramp Start Avialpes" ||
      pick == "GA Ramp Start 74" || pick == "GA Ramp 32";
  REQUIRE(a_stand);
}

TEST_CASE("ga-stand: no stand / airline-only -> empty", "[parking]") {
  std::vector<ParkingStand> none;
  REQUIRE(pick_ga_stand(none, 'A', EngineKind::Prop, 0.0, 0.0).empty());
  // An explicit AIRLINE gate is excluded even when it's the only stand.
  ParkingStand airline;
  airline.name = "Gate 10";
  airline.size_code = 'C';
  airline.jets = true;
  airline.op_airline = true;
  REQUIRE(pick_ga_stand({airline}, 'A', EngineKind::Jet, 0.0, 0.0).empty());
}

TEST_CASE("ga-stand: untagged (no 1301) stand used as non-airline fallback",
          "[parking]") {
  // Custom scenery ramp start without operation metadata: not general_aviation,
  // not airline -> should still be picked via the non-airline fallback pass.
  ParkingStand ramp;
  ramp.name = "Ramp 5";
  ramp.size_code = 'B';
  ramp.jets = true;
  ramp.props = true;
  REQUIRE(pick_ga_stand({ramp}, 'A', EngineKind::Jet, 0.0, 0.0) == "Ramp 5");
}
