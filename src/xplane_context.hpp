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

#ifndef XPLANE_CONTEXT_HPP
#define XPLANE_CONTEXT_HPP

#include <string>

namespace xplane_context {

enum class FrequencyType {
  UNKNOWN,
  DELIVERY,
  GROUND,
  TOWER,
  APPROACH,
  UNICOM,
  CTAF,
  ATIS,
};

const char *frequency_type_name(FrequencyType ft);

struct XPlaneContext {
  double latitude = 0.0;
  double longitude = 0.0;
  float altitude_ft_msl = 0.0f;
  float groundspeed_kts = 0.0f;
  float indicated_airspeed_kts = 0.0f;
  float vertical_speed_fpm = 0.0f;
  float heading_true = 0.0f;
  bool on_ground = true;
  bool engines_running = false;
  float com1_freq_mhz = 0.0f;
  float com2_freq_mhz = 0.0f;
  int active_com = 1;
  std::string aircraft_icao;
  std::string nearest_airport_id;
  bool is_towered_airport = false;
  FrequencyType frequency_type = FrequencyType::UNKNOWN;
  bool avionics_on = false;
  float qnh_inhg = 29.92f;
  float wind_direction_deg = 0.0f;
  float wind_speed_kt = 0.0f;
};

void init();
void stop();
void update();

const XPlaneContext &get();

} // namespace xplane_context

#endif // XPLANE_CONTEXT_HPP
