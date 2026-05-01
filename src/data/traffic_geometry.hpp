/*
 * xp_wellys_atc - AI-powered ATC voice communication for X-Plane 12
 * Copyright (C) 2026 thWelly & Claude (Anthropic)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef TRAFFIC_GEOMETRY_HPP
#define TRAFFIC_GEOMETRY_HPP

namespace traffic_geometry {

// Initial bearing from (lat1,lon1) to (lat2,lon2) in degrees true,
// normalised to [0, 360). Pure haversine math, no SDK calls.
double bearing_deg(double lat1, double lon1, double lat2, double lon2);

// Great-circle distance in nautical miles (haversine, R = 6371 km).
double distance_nm(double lat1, double lon1, double lat2, double lon2);

// Clock position of a target as seen from the user. Result is in
// (0.0, 12.0]: dead-ahead returns 12.0, abeam right 3.0, six o'clock
// 6.0, abeam left 9.0. The result is rounded to the nearest hour clock
// position so callers can use it as a phraseology token directly.
double clock_position(double user_heading_deg, double target_bearing_deg);

} // namespace traffic_geometry

#endif // TRAFFIC_GEOMETRY_HPP
