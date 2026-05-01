/*
 * xp_wellys_atc - AI-powered ATC voice communication for X-Plane 12
 * Copyright (C) 2026 thWelly & Claude (Anthropic)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

#include "data/traffic_geometry.hpp"

#include <cmath>

namespace traffic_geometry {

namespace {
constexpr double kDeg2Rad = M_PI / 180.0;
constexpr double kEarthRadiusM = 6371000.0;
constexpr double kMetersPerNm = 1852.0;
} // namespace

double bearing_deg(double lat1, double lon1, double lat2, double lon2) {
  double lat1r = lat1 * kDeg2Rad;
  double lat2r = lat2 * kDeg2Rad;
  double dlon = (lon2 - lon1) * kDeg2Rad;
  double y = std::sin(dlon) * std::cos(lat2r);
  double x = std::cos(lat1r) * std::sin(lat2r) -
             std::sin(lat1r) * std::cos(lat2r) * std::cos(dlon);
  double bearing = std::atan2(y, x) / kDeg2Rad;
  return std::fmod(bearing + 360.0, 360.0);
}

double distance_nm(double lat1, double lon1, double lat2, double lon2) {
  double dlat = (lat2 - lat1) * kDeg2Rad;
  double dlon = (lon2 - lon1) * kDeg2Rad;
  double a = std::sin(dlat / 2.0) * std::sin(dlat / 2.0) +
             std::cos(lat1 * kDeg2Rad) * std::cos(lat2 * kDeg2Rad) *
                 std::sin(dlon / 2.0) * std::sin(dlon / 2.0);
  double dist_m =
      kEarthRadiusM * 2.0 * std::atan2(std::sqrt(a), std::sqrt(1.0 - a));
  return dist_m / kMetersPerNm;
}

double clock_position(double user_heading_deg, double target_bearing_deg) {
  double rel = std::fmod(target_bearing_deg - user_heading_deg + 720.0, 360.0);
  double hours = std::round(rel / 30.0);
  // 0 and 12 both refer to dead-ahead; collapse onto 12 so the value
  // stays in (0, 12] as documented.
  if (hours <= 0.0 || hours >= 12.0)
    return 12.0;
  return hours;
}

} // namespace traffic_geometry
