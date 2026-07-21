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

#include <cstdint>
#include <map>
#include <string>
#include <vector>

namespace airspace_db {
struct Controller;
}

namespace xplane_context {

struct RunwayEnd {
  std::string number; // e.g. "09", "27L"
  double lat = 0.0;
  double lon = 0.0;
  float heading_deg = 0.0f; // computed from lat/lon of both ends
};

struct RunwayInfo {
  RunwayEnd end1;
  RunwayEnd end2;
  float width_m = 0.0f;
  float length_m = 0.0f; // computed via haversine
  int surface_code = 0;  // 1=asphalt, 2=concrete, etc.
};

enum class EngineKind { Prop = 0, Turboprop = 1, Jet = 2 };

// A parking position from apt.dat: row 1300 (lat/lon + equipment + name) plus its
// row 1301 (ICAO size code + operation type). Used to taxi an arriving aircraft to
// a size/type-appropriate general-aviation stand by name.
struct ParkingStand {
  std::string name; // e.g. "GA Ramp Start 12"
  double lat = 0.0;
  double lon = 0.0;
  char size_code = 'A';           // ICAO width code A..F (row 1301)
  bool general_aviation = false;  // row 1301 operation == general_aviation
  bool op_airline = false;        // row 1301 operation == airline (exclude for GA)
  bool jets = false;              // row 1300 equipment flags
  bool turboprops = false;
  bool props = false;
  bool helos = false;
};

// ICAO wingspan size code from wingspan in metres: A<15, B<24, C<36, D<52, E<65,
// else F. Derives the aircraft's size code from its wingspan.
char icao_size_code_for_wingspan(float wingspan_m);

// Pick the best-fit general-aviation stand for an aircraft: the SMALLEST GA stand
// whose size >= ac_size_code AND whose equipment accepts ac_engine, tie-broken by
// nearest to (ac_lat, ac_lon). Falls back to the largest engine-compatible GA stand
// when none is big enough (e.g. a Code-C bizjet where the biggest GA stand is B).
// Returns the stand name, or "" when the airport has no GA stand. SDK-free.
std::string pick_ga_stand(const std::vector<ParkingStand> &stands,
                          char ac_size_code, EngineKind ac_engine, double ac_lat,
                          double ac_lon);

enum class FrequencyType {
  UNKNOWN,
  DELIVERY,
  GROUND,
  TOWER,
  APPROACH,  // apt.dat code 55/1055 — APP/DEP at regional airports
  DEPARTURE, // apt.dat code 56/1056 — dedicated Departure at large airports
  UNICOM,
  CTAF,
  ATIS,
};

const char *frequency_type_name(FrequencyType ft);

struct AirportFrequency {
  uint32_t freq_khz =
      0; // e.g. 121900 for 121.900 MHz (exact integer, no float)
  FrequencyType type = FrequencyType::UNKNOWN;
  std::string
      name; // raw apt.dat label, e.g. "CHAMBERY APP" (empty when unknown)
};

struct AirportFrequencies {
  std::vector<AirportFrequency> all;

  // Has at least one frequency of the given type?
  bool has(FrequencyType ft) const;
  // First frequency of given type as MHz (0.0f if none)
  float first_mhz(FrequencyType ft) const;
  // Raw apt.dat name of the first frequency of given type ("" if none/unnamed)
  std::string first_name(FrequencyType ft) const;
  // Match a COM frequency (MHz) to a FrequencyType (UNKNOWN if no match)
  FrequencyType lookup(float freq_mhz) const;
  // Convenience: has(GROUND)
  bool has_ground() const;
};

struct XPlaneContext {
  double latitude = 0.0;
  double longitude = 0.0;
  float altitude_ft_msl = 0.0f;   // true (GPS) altitude in feet MSL
  float pressure_alt_ft = 0.0f;  // pressure altitude (altimeter at 1013.25 hPa)
  float groundspeed_kts = 0.0f;
  float indicated_airspeed_kts = 0.0f;
  float vertical_speed_fpm = 0.0f;
  float heading_true = 0.0f;
  float height_agl_ft = 0.0f;
  bool on_ground = true;
  float com1_freq_mhz = 0.0f;
  float com2_freq_mhz = 0.0f;
  float com1_standby_mhz = 0.0f;
  float com2_standby_mhz = 0.0f;
  int active_com = 1;
  std::string aircraft_icao;       // ICAO type code, e.g. "C172"
  std::string aircraft_tail_number; // aircraft registration, e.g. "F-HABC"
  std::string ifr_destination; // filed destination ICAO (empty if no plan)
  std::string ifr_sid; // departure SID name (SimBrief, display only fallback)
  std::string
      ifr_fpl_first_fix; // first FPL waypoint after departure (= SID last fix)
  bool ifr_simbrief_valid = false; // true when a SimBrief OFP is loaded
  // CIFP-derived SID data for the active departure runway.
  // Updated whenever active_runway changes. Populated from cifp_reader.
  std::string ifr_cifp_sid; // ATC-assigned SID name from CIFP (e.g. "ODIK2A")
  int ifr_sid_min_alt_ft =
      0; // highest "at or above" minimum on SID (0 = no constraint)
  bool ifr_sid_min_is_fl = false; // true when the minimum is expressed as FL
  std::string
      ifr_sid_min_waypoint;     // waypoint at which the binding minimum occurs
  std::string ifr_sid_last_fix; // last waypoint on the assigned SID (for
                                // direct-to shortcut)
  int ifr_cruise_alt_ft =
      0; // cruise altitude from SimBrief OFP (0 when no plan)
  std::string nearest_airport_id;   // active airport (may be frequency-tuned)
  std::string geometric_nearest_id; // raw geographic nearest from XPLM
  std::string nearest_airport_name; // from apt.dat, e.g. "Grenchen"
  bool is_towered_airport = false;
  FrequencyType frequency_type = FrequencyType::UNKNOWN;
  bool avionics_on = false;
  bool com_radio_powered = false;
  float qnh_inhg = 29.92f; // kept for US altimeter display and pressure-alt correction
  int   qnh_hpa  = 1013;   // Pa / 100, rounded — used for all QNH broadcasts
  float wind_direction_deg = 0.0f;
  float wind_speed_kt = 0.0f;
  float visibility_m = 9999.0f;
  // Destination-airport METAR visibility (metres), parsed from the filed IFR
  // destination's METAR (XPLMGetMETARForAirport). -1 = unavailable (custom
  // weather / not yet downloaded). Used for the approach-selection weather gate
  // so the RNAV Alpha/Zulu choice keys on the ARRIVAL field's reported vis, not
  // the region value sampled at the aircraft en route. (LFMN 2026-07-19.)
  float dest_metar_visibility_m = -1.0f;
  float cloud_base_ft_msl = 99999.0f;
  // Destination-airport METAR ceiling (feet MSL) = lowest BKN/OVC/VV layer +
  // field elevation. Large value (>=99999) = no ceiling (CAVOK/NSC/SKC). -1 =
  // unavailable. Pairs with dest_metar_visibility_m for the approach gate.
  float dest_metar_ceiling_ft = -1.0f;
  // Raw destination METAR string (for per-runway RVR parsing at the approach
  // gate, where the arrival runway is known). Empty = unavailable.
  std::string dest_metar;
  int cloud_type = 0; // 0=clear,1=few,2=scattered,3=broken,4=overcast
  float temperature_c = 15.0f;
  float dewpoint_c = 10.0f;
  float atis_freq_mhz = 0.0f;
  AirportFrequencies airport_freqs; // all frequencies for nearest airport
  bool tower_only = false;          // towered but no separate ground freq
  double airport_lat = 0.0;         // airport position (for range checks)
  double airport_lon = 0.0;
  std::vector<RunwayInfo> runways;
  // General-aviation parking stands at the nearest airport (apt.dat 1300/1301),
  // for the post-landing taxi-to-parking clearance. Empty until parsed.
  std::vector<ParkingStand> airport_parking;
  // Aircraft profile for stand selection (read from DataRefs each frame).
  float aircraft_wingspan_m = 0.0f;
  EngineKind aircraft_engine_kind = EngineKind::Prop;
  std::string active_runway;               // wind-determined, e.g. "28", "09L"
  std::string active_runway_holding_point; // apt.dat 1201 node name at
                                           // hold-short, e.g. "A3"
  std::map<std::string, std::string> runway_holding_points; // runway → hold-short node name for all runways at this airport
  int transition_alt_ft = 0; // apt.dat 1302 transition_alt, 0 if absent
  // Controllers (TWR/TRACON/CTR from atc.dat) whose polygon + altitude band
  // enclose the current aircraft position. Refreshed once per second.
  // Empty if airspace_db is disabled (atc.dat missing).
  std::vector<const airspace_db::Controller *> enclosing_airspaces;
  // Monotonic clock at the time this context snapshot was taken. Plugin
  // populates from XPLMGetElapsedTime(); CLI / scenarios use a synthetic
  // step counter. Consumers that need a "how recent is X" check (e.g.
  // intent rules with a require_just_landed flag) read this instead of
  // taking the timestamp as a separate parameter.
  double now_secs = 0.0;
  // Transponder state — read from sim/cockpit/radios/transponder_code and
  // sim/cockpit2/radios/actuators/transponder_mode (0=OFF,1=STBY,2=ALT).
  int transponder_code = 0;
  int transponder_mode = 0;
  // Path to the X-Plane CIFP directory, e.g. "/path/to/X-Plane 12/Custom
  // Data/CIFP". Set once at plugin init from xplane_system_path(); empty in
  // headless builds.
  std::string cifp_dir;
};

void init();
void stop();
void update();

const XPlaneContext &get();

// X-Plane system root path with trailing slash, e.g. "/home/user/X-Plane 12/".
// Empty in headless builds.
const std::string &system_path();

// Write a frequency (in kHz, e.g. 121900) to the active COM's standby slot
void set_standby_freq(uint32_t freq_khz);

// ── Airport picker / lock ────────────────────────────────────────
// Force `nearest_airport_id` (and all derived fields) to a specific ICAO.
// Overrides both geometric-nearest and frequency-match logic.
// No-op if ICAO is not in the airport cache.
void lock_airport(const std::string &icao);
void unlock_airport();
const std::string &locked_airport() noexcept;

struct NearbyAirport {
  std::string icao;
  std::string name;
  double distance_nm = 0.0;
  bool has_atis = false;
  bool has_ground = false;
  bool has_tower = false;
  bool has_approach = false;
};

// Return up to `max_count` airports within `max_nm` of the aircraft,
// sorted ascending by distance. Empty until the airport cache is ready.
std::vector<NearbyAirport> find_nearby_airports(double max_nm,
                                                size_t max_count);

// Field elevation in feet for the given ICAO. Returns 0.0f if the cache
// is not ready or the airport is unknown — callers that need to gate on
// "have we got real data" should check `airport_elevation_known()` too.
float airport_elevation_ft(const std::string &icao);
bool airport_elevation_known(const std::string &icao);

// Returns the apt.dat name for the given ICAO (e.g. "Nice Cote d Azur" for
// "LFMN"). Empty string if not in the parsed cache (airport not in apt.dat
// or apt.dat not yet parsed).
std::string airport_name_for(const std::string &icao);

// Returns the reference lat/lon for the given ICAO from the apt.dat position
// cache. Returns {0,0} if unknown. Used for airspace lookups at remote airports.
std::pair<double, double> airport_pos_for(const std::string &icao);

// Returns the Tower frequency in MHz for the given ICAO from the apt.dat
// freq cache. Returns 0.0f if the airport is unknown or has no Tower freq.
float tower_mhz_for(const std::string &icao);

// Returns true if the airport has a dedicated Ground frequency. Use this to
// distinguish a real Tower from an AFIS/Information service (AFIS airports
// have a Tower-type freq but no Ground freq, e.g. LFQA 134.925 AFIS).
bool has_ground_freq_for(const std::string &icao);

// Returns a ready-to-use phrase for the nearest taxiway to the given position:
// "via Alpha", "via Bravo", ... or "to the apron" when no taxiway is found.
// Used in the RUNWAY_VACATED_TOWER_ONLY template as {nearest_taxiway}.
std::string nearest_taxiway_phrase(const std::string &icao,
                                   double lat, double lon);

} // namespace xplane_context

#endif // XPLANE_CONTEXT_HPP
