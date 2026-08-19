/*
 * xp_wellys_atc - headless CLI
 *
 * Engine modules link against xplane_context::get() (declared in
 * xplane_context.hpp). In the plugin, xplane_context_runtime.cpp
 * provides it; in the CLI we expose a single mutable ctx that the
 * REPL main primes before each process_transcript call.
 */

#include <map>
#include "core/xplane_context.hpp"

#include <cstdint>

namespace xplane_context {

// Definition lives here (CLI-only); plugin has its own in the runtime.
XPlaneContext g_cli_ctx;

const XPlaneContext &get() { return g_cli_ctx; }

void set_standby_freq(uint32_t) {}

void lock_airport(const std::string &) {}
void unlock_airport() {}
const std::string &locked_airport() noexcept {
  static const std::string empty;
  return empty;
}

std::vector<NearbyAirport> find_nearby_airports(double, size_t) { return {}; }

// Aerodrome elevations, injectable from the REPL. The real values come from
// apt.dat, parsed in xplane_context_runtime.cpp, which is plugin-only -- so
// headless every field sat at 0 ft. That is not a cosmetic gap: the vectoring
// mode decision is "lowest approach-sector MSA MINUS FIELD ELEVATION", so with
// the elevation at zero the terrain test judged every airport by its raw MSA.
// LSGG (MSA 7000, field 1411) came out 7000 ft above the field and was refused
// vectors; the true figure is 5589, which passes. [C. P. Potter]
static std::map<std::string, float> g_airport_elev_ft;

void set_airport_elevation_ft(const std::string &icao, float ft) {
  g_airport_elev_ft[icao] = ft;
}

float airport_elevation_ft(const std::string &icao) {
  auto it = g_airport_elev_ft.find(icao);
  return it == g_airport_elev_ft.end() ? 0.0f : it->second;
}
bool airport_elevation_known(const std::string &icao) {
  return g_airport_elev_ft.count(icao) != 0;
}

void init() {}
void stop() {}
void update() {}

} // namespace xplane_context
