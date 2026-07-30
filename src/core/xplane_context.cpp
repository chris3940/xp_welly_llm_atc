/*
 * xp_wellys_atc - AI-powered ATC voice communication for X-Plane 12
 * Copyright (C) 2026 thWelly & Claude (Anthropic)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * ── POD helpers only. All X-Plane SDK / apt.dat runtime code lives in
 *    xplane_context_runtime.cpp, which is only linked into the plugin.
 */

#include "core/xplane_context.hpp"

#include <cmath>
#include <functional>

namespace xplane_context {

namespace {
// Priority ranking for resolving frequency type when multiple apt.dat entries
// share the same frequency (part-time towers like KVRB list 126.300 as both
// TOWER and UNICOM). Higher rank wins.
int priority_rank(FrequencyType t) {
  switch (t) {
  case FrequencyType::TOWER:
    return 7;
  case FrequencyType::GROUND:
    return 6;
  case FrequencyType::DELIVERY:
    return 5;
  case FrequencyType::APPROACH:
  case FrequencyType::DEPARTURE:
    return 4;
  case FrequencyType::ATIS:
    return 3;
  case FrequencyType::CTAF:
    return 2;
  case FrequencyType::UNICOM:
    return 1;
  case FrequencyType::UNKNOWN:
    return 0;
  }
  return 0;
}
} // namespace

bool AirportFrequencies::has(FrequencyType ft) const {
  for (const auto &f : all)
    if (f.type == ft)
      return true;
  return false;
}

float AirportFrequencies::first_mhz(FrequencyType ft) const {
  for (const auto &f : all)
    if (f.type == ft)
      return static_cast<float>(f.freq_khz) / 1000.0f;
  return 0.0f;
}

std::string AirportFrequencies::first_name(FrequencyType ft) const {
  for (const auto &f : all)
    if (f.type == ft)
      return f.name;
  return {};
}

FrequencyType AirportFrequencies::lookup(float freq_mhz) const {
  const uint32_t target = static_cast<uint32_t>(std::round(freq_mhz * 1000.0f));
  FrequencyType best = FrequencyType::UNKNOWN;
  int best_rank = -1;
  for (const auto &f : all) {
    const uint32_t diff =
        (target > f.freq_khz) ? target - f.freq_khz : f.freq_khz - target;
    if (diff > 1)
      continue;
    const int rank = priority_rank(f.type);
    if (rank > best_rank) {
      best_rank = rank;
      best = f.type;
    }
  }
  return best;
}

bool AirportFrequencies::has_ground() const {
  return has(FrequencyType::GROUND);
}

const char *frequency_type_name(FrequencyType ft) {
  switch (ft) {
  case FrequencyType::UNKNOWN:
    return "Unknown";
  case FrequencyType::DELIVERY:
    return "Delivery";
  case FrequencyType::GROUND:
    return "Ground";
  case FrequencyType::TOWER:
    return "Tower";
  case FrequencyType::APPROACH:
    return "Approach";
  case FrequencyType::DEPARTURE:
    return "Departure";
  case FrequencyType::UNICOM:
    return "Unicom";
  case FrequencyType::CTAF:
    return "CTAF";
  case FrequencyType::ATIS:
    return "ATIS";
  }
  return "Unknown";
}

// Weak stubs: atc_repl and headless tests don't link xplane_context_runtime.cpp.
// The plugin binary links both TUs; the strong definitions in runtime win.
// MSVC has no weak-symbol equivalent, and the Windows build only ever produces
// the plugin (which always links the strong runtime definitions), so the stubs
// are simply omitted there to avoid a duplicate-symbol link error.
#if !defined(_WIN32)
__attribute__((weak)) float tower_mhz_for(const std::string &) { return 0.0f; }
__attribute__((weak)) bool has_ground_freq_for(const std::string &) { return false; }
__attribute__((weak)) bool has_approach_freq_for(const std::string &) { return false; }
__attribute__((weak)) std::string airport_name_for(const std::string &) {
  return {};
}
__attribute__((weak)) std::string nearest_taxiway_phrase(const std::string &,
                                                         double, double) {
  return "to the apron";
}
__attribute__((weak)) std::pair<double, double>
airport_pos_for(const std::string &) {
  return {0.0, 0.0};
}
#endif

char icao_size_code_for_wingspan(float w) {
  if (w < 15.0f)
    return 'A';
  if (w < 24.0f)
    return 'B';
  if (w < 36.0f)
    return 'C';
  if (w < 52.0f)
    return 'D';
  if (w < 65.0f)
    return 'E';
  return 'F';
}

std::string pick_ga_stand(const std::vector<ParkingStand> &stands,
                          char ac_size_code, EngineKind ac_engine, double ac_lat,
                          double ac_lon) {
  auto eq_ok = [&](const ParkingStand &s) {
    switch (ac_engine) {
    case EngineKind::Jet:
      return s.jets;
    case EngineKind::Turboprop:
      return s.turboprops || s.props;
    case EngineKind::Prop:
    default:
      return s.props || s.turboprops;
    }
  };
  const double kdeg2rad = 0.017453292519943295;
  const double coslat = std::cos(ac_lat * kdeg2rad);
  auto d2 = [&](const ParkingStand &s) {
    const double dlat = s.lat - ac_lat;
    const double dlon = (s.lon - ac_lon) * coslat;
    return dlat * dlat + dlon * dlon;
  };
  // One pass over the stands that pass `eligible`: smallest fitting size (>= the
  // aircraft) nearest wins; else the largest engine-compatible stand (nearest).
  auto run = [&](const std::function<bool(const ParkingStand &)> &eligible) {
    const ParkingStand *best_fit = nullptr;      // smallest size >= ac, nearest
    const ParkingStand *best_fallback = nullptr; // largest eq-ok, nearest
    for (const auto &s : stands) {
      if (!eligible(s) || !eq_ok(s))
        continue;
      if (!best_fallback || s.size_code > best_fallback->size_code ||
          (s.size_code == best_fallback->size_code &&
           d2(s) < d2(*best_fallback)))
        best_fallback = &s;
      if (s.size_code < ac_size_code)
        continue; // too small for this aircraft
      if (!best_fit || s.size_code < best_fit->size_code ||
          (s.size_code == best_fit->size_code && d2(s) < d2(*best_fit)))
        best_fit = &s;
    }
    const ParkingStand *pick = best_fit ? best_fit : best_fallback;
    return pick ? pick->name : std::string();
  };
  // Prefer stands explicitly tagged general_aviation (basic XP12 apt.dat). Fall
  // back to any NON-airline stand -- custom sceneries often have ramp starts
  // (row 1300) without the row-1301 metadata, so nothing is tagged
  // general_aviation and the GA-only pass finds nothing (LFMN_JustSim, user
  // 2026-07-21). Airline gates stay excluded.
  std::string r = run([](const ParkingStand &s) { return s.general_aviation; });
  if (r.empty())
    r = run([](const ParkingStand &s) { return !s.op_airline; });
  return r;
}

} // namespace xplane_context
