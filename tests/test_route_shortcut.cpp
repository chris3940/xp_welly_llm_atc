#include "atc/route_shortcut.hpp"

#include <catch2/catch_amalgamated.hpp>

using Catch::Approx;

// A small cluster of fixes around the LFLP (Annecy) arrival area, ~1 deg lat
// spacing so distances are order-tens-of-NM and easy to reason about.
//   AC at (46.00, 6.00), FL200 (20000 ft), heading toward the fixes.
//   NEAR  (46.30, 6.00)  ~18 NM north
//   FAR   (46.80, 6.00)  ~48 NM north
static constexpr double kAcLat = 46.00, kAcLon = 6.00;

TEST_CASE("route_shortcut::pick_nearest picks the closest candidate",
          "[shortcut]") {
  std::vector<route_shortcut::Candidate> cands;
  cands.push_back({"NEAR", 46.30, 6.00, /*routed*/ 60.0, /*cross*/ 0, /*idx*/ 5});
  cands.push_back({"FAR", 46.80, 6.00, /*routed*/ 90.0, /*cross*/ 0, /*idx*/ 8});

  auto p = route_shortcut::pick_nearest(kAcLat, kAcLon, 20000, cands,
                                        /*min_save*/ 0.05, /*max_deg*/ 0.0);
  REQUIRE(p.ok);
  REQUIRE(p.ident == "NEAR");
  REQUIRE(p.route_idx == 5);
  REQUIRE(p.direct_nm == Approx(18.0).margin(1.5));
}

TEST_CASE("route_shortcut::pick_nearest saving gate rejects non-shortening",
          "[shortcut]") {
  // routed_nm barely above direct -> saves < 5% -> rejected.
  std::vector<route_shortcut::Candidate> cands;
  cands.push_back({"NEAR", 46.30, 6.00, /*routed*/ 18.2, 0, 5});
  auto p = route_shortcut::pick_nearest(kAcLat, kAcLon, 20000, cands, 0.05, 0.0);
  REQUIRE_FALSE(p.ok);
}

TEST_CASE("route_shortcut::pick_nearest descent gate rejects too-steep",
          "[shortcut]") {
  // NEAR ~18 NM; must lose 20000-5000 = 15000 ft over 18 NM =>
  // gradient ~833 ft/NM => ~7.8 deg, well over 3 deg -> rejected.
  std::vector<route_shortcut::Candidate> cands;
  cands.push_back({"NEAR", 46.30, 6.00, /*routed*/ 60.0, /*cross*/ 5000, 5});
  auto p = route_shortcut::pick_nearest(kAcLat, kAcLon, 20000, cands, 0.05, 3.0);
  REQUIRE_FALSE(p.ok);
}

TEST_CASE("route_shortcut::pick_nearest descent gate accepts shallow",
          "[shortcut]") {
  // FAR ~48 NM; lose 20000-8000 = 12000 ft over 48 NM => ~250 ft/NM =>
  // ~2.35 deg, under 3 deg -> accepted (and it's the only candidate).
  std::vector<route_shortcut::Candidate> cands;
  cands.push_back({"FAR", 46.80, 6.00, /*routed*/ 90.0, /*cross*/ 8000, 8});
  auto p = route_shortcut::pick_nearest(kAcLat, kAcLon, 20000, cands, 0.05, 3.0);
  REQUIRE(p.ok);
  REQUIRE(p.ident == "FAR");
  REQUIRE(p.descent_deg < 3.0);
  REQUIRE(p.descent_deg > 1.5);
}

TEST_CASE("route_shortcut::pick_nearest already-low candidate passes descent gate",
          "[shortcut]") {
  // Aircraft at 6000 ft, fix crossing 8000 ft -> nothing to lose -> passes.
  std::vector<route_shortcut::Candidate> cands;
  cands.push_back({"NEAR", 46.30, 6.00, /*routed*/ 60.0, /*cross*/ 8000, 5});
  auto p = route_shortcut::pick_nearest(kAcLat, kAcLon, 6000, cands, 0.05, 3.0);
  REQUIRE(p.ok);
  REQUIRE(p.descent_deg == Approx(0.0));
}

TEST_CASE("route_shortcut::pick_nearest ignores position-less candidates",
          "[shortcut]") {
  std::vector<route_shortcut::Candidate> cands;
  cands.push_back({"NOPOS", 0.0, 0.0, 60.0, 0, 3});
  auto p = route_shortcut::pick_nearest(kAcLat, kAcLon, 20000, cands, 0.0, 0.0);
  REQUIRE_FALSE(p.ok);
}
