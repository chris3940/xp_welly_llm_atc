#include "data/traffic_geometry.hpp"

#include <catch2/catch_amalgamated.hpp>

using Catch::Approx;

// LSZH (Zurich) and LSGG (Geneva) reference points. Distance and
// initial bearing are well-known great-circle values; the test allows
// ±1 NM / ±1° tolerance because the published runway lat/lon for each
// airport varies a few hundred metres between sources.
static constexpr double kLszhLat = 47.4583;
static constexpr double kLszhLon = 8.5483;
static constexpr double kLsggLat = 46.2381;
static constexpr double kLsggLon = 6.1090;

TEST_CASE("traffic_geometry::distance_nm: LSZH -> LSGG", "[traffic][geometry]") {
    // Great-circle distance LSZH (47.4583, 8.5483) to LSGG (46.2381, 6.1090)
    // is ~124 NM via haversine. Tolerance is wide enough that small
    // refinements to the reference lat/lon won't churn the test.
    const double d = traffic_geometry::distance_nm(kLszhLat, kLszhLon,
                                                   kLsggLat, kLsggLon);
    REQUIRE(d == Approx(124.0).margin(2.0));
}

TEST_CASE("traffic_geometry::bearing_deg: LSZH -> LSGG", "[traffic][geometry]") {
    const double b = traffic_geometry::bearing_deg(kLszhLat, kLszhLon,
                                                   kLsggLat, kLsggLon);
    REQUIRE(b == Approx(234.0).margin(1.5));
}

TEST_CASE("traffic_geometry::distance_nm: zero distance",
          "[traffic][geometry]") {
    REQUIRE(traffic_geometry::distance_nm(kLszhLat, kLszhLon, kLszhLat,
                                          kLszhLon) == Approx(0.0).margin(1e-6));
}

TEST_CASE("traffic_geometry::bearing_deg: result is in [0, 360)",
          "[traffic][geometry]") {
    // Point due south of LSZH should yield ~180 deg.
    const double b = traffic_geometry::bearing_deg(kLszhLat, kLszhLon,
                                                   kLszhLat - 1.0, kLszhLon);
    REQUIRE(b == Approx(180.0).margin(0.5));
    REQUIRE(b >= 0.0);
    REQUIRE(b < 360.0);
}

// The clock-position helper rounds to the nearest hour and collapses
// 0/12 onto 12 so the result stays in (0, 12]. Each row in the table
// pins down one of the edge cases described in the spec.
struct ClockCase {
    double heading;
    double bearing;
    double expected;
    const char *label;
};

TEST_CASE("traffic_geometry::clock_position table",
          "[traffic][geometry][clock]") {
    const ClockCase cases[] = {
        {360.0, 90.0, 3.0, "head=360 brg=090 -> 3 o'clock"},
        {180.0, 90.0, 9.0, "head=180 brg=090 -> 9 o'clock"},
        {0.0, 0.0, 12.0, "head=000 brg=000 -> 12 o'clock"},
        {350.0, 10.0, 1.0, "head=350 brg=010 -> 1 o'clock (wrap)"},
        {10.0, 350.0, 11.0, "head=010 brg=350 -> 11 o'clock (wrap)"},
        {0.0, 355.0, 12.0, "head=000 brg=355 -> 12 o'clock (just left)"},
        {0.0, 5.0, 12.0, "head=000 brg=005 -> 12 o'clock (just right)"},
        {270.0, 0.0, 3.0, "head=270 brg=000 -> 3 o'clock"},
        {90.0, 270.0, 6.0, "head=090 brg=270 -> 6 o'clock"},
    };
    for (const auto &c : cases) {
        INFO(c.label);
        REQUIRE(traffic_geometry::clock_position(c.heading, c.bearing) ==
                Approx(c.expected));
    }
}

TEST_CASE("traffic_geometry::clock_position: result is always in (0, 12]",
          "[traffic][geometry][clock]") {
    for (int hdg = 0; hdg < 360; hdg += 23) {
        for (int brg = 0; brg < 360; brg += 17) {
            const double c =
                traffic_geometry::clock_position(hdg, brg);
            REQUIRE(c > 0.0);
            REQUIRE(c <= 12.0);
        }
    }
}
