/*
 * xp_wellys_atc - headless IFR test CLI
 * Copyright (C) 2026 thWelly & Claude (Anthropic)
 * Copyright (C) 2026 Christopher P. Potter (Linux port + IFR extensions)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Usage:
 *   atc_ifr_repl              — starts with LFMN default context
 *   atc_ifr_repl <cifp_dir>   — override CIFP directory on the command line
 *
 * Default LFMN scenario:
 *   Aircraft near ABDIL STAR entry, FL115, heading 103, GS 200 kt, VS -800 fpm.
 *   Destination: LFMN, QNH 1024, wind 165/05 (favours runway 04L).
 *   CIFP: auto-detected from XP_CIFP_DIR env, or ~/X-Plane 12/Custom Data/CIFP.
 *
 * The REPL exposes poll_approach, fly, goto, jump_*, and say so the full
 * IFR approach sequence can be simulated without X-Plane running.
 */

#include "ifr_repl.hpp"

#include "atc/atc_state_machine.hpp"
#include "atc/atc_templates.hpp"
#include "atc/flight_phase.hpp"
#include "data/airport_vrps.hpp"
#include "data/airspace_db.hpp"
#include "data/openair_db.hpp"
#include "data/simbrief_ofp.hpp"
#include "core/xplane_context.hpp"

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <utility>
#include <thread>

// Test-harness STRONG override of the weak SDK-free stub (xplane_context.cpp).
// In the plugin this reads the apt.dat frequency DB; the headless harness has no
// apt.dat, so the weak stub returns false -> every towered destination reads as
// AFIS (dest_is_afis=true), which gates out the terminal approach-clearance path.
// Returning true for any non-empty ICAO lets the replay exercise "cleared <appr>
// approach" for a towered field like LFLP. Harness-only; does not affect the plugin.
namespace xplane_context {
bool has_ground_freq_for(const std::string &icao) { return !icao.empty(); }
// Real dest position so on_destination_terminal() actually runs (the weak stub
// returns 0,0 which the function treats as "unknown -> permissive", masking the
// in-sim behaviour). LFLP = Annecy.
std::pair<double, double> airport_pos_for(const std::string &icao) {
  if (icao == "LFLP")
    return {45.929, 6.099};
  if (icao == "LOWI")
    return {47.260, 11.344};
  return {0.0, 0.0};
}
} // namespace xplane_context

// Try to locate the X-Plane 12 CIFP directory automatically.
// Priority: XP_CIFP_DIR env var > ~/X-Plane 12/Custom Data/CIFP.
static std::string detect_cifp_dir() {
  if (const char *v = std::getenv("XP_CIFP_DIR"))
    return v;
  if (const char *home = std::getenv("HOME")) {
    std::string candidate = std::string(home) + "/X-Plane 12/Custom Data/CIFP";
    return candidate;
  }
  return "";
}

static xplane_context::XPlaneContext lfmn_context(const std::string &cifp_dir) {
  using xplane_context::FrequencyType;

  xplane_context::XPlaneContext ctx;

  // Aircraft position: near ABDIL (ABDI8R STAR entry for LFMN), west of Nice.
  ctx.latitude         = 43.65;
  ctx.longitude        = 6.20;
  ctx.altitude_ft_msl  = 11500.0f;
  ctx.pressure_alt_ft  = 11500.0f;
  ctx.heading_true     = 103.0f;  // east toward LFMN
  ctx.groundspeed_kts  = 200.0f;
  ctx.vertical_speed_fpm = -800.0f;
  ctx.height_agl_ft    = 11000.0f;
  ctx.on_ground        = false;
  ctx.avionics_on      = true;
  ctx.com_radio_powered = true;

  // Radio — LFMN Approach
  ctx.com1_freq_mhz  = 120.160f;
  ctx.active_com     = 1;
  ctx.frequency_type = FrequencyType::APPROACH;

  // Weather: QNH 1024, wind 350/10 kt (northerly Tramontane — favours runway 04L at LFMN)
  ctx.qnh_hpa            = 1024;
  ctx.qnh_inhg           = 30.24f;
  ctx.wind_direction_deg = 350.0f;
  ctx.wind_speed_kt      = 10.0f;
  ctx.visibility_m       = 9999.0f;
  ctx.temperature_c      = 20.0f;
  ctx.dewpoint_c         = 10.0f;

  // IFR destination
  ctx.nearest_airport_id = "LFMN";
  ctx.is_towered_airport = true;
  ctx.ifr_destination    = "LFMN";
  ctx.aircraft_icao      = "B738";
  ctx.cifp_dir           = cifp_dir;

  return ctx;
}

int main(int argc, char **argv) {
  // Init engine modules (same order as atc_repl).
  atc_templates::init();
  flight_phase::init();
  atc_state_machine::init();
  airport_vrps::init();

  std::string cifp_dir = detect_cifp_dir();
  if (argc >= 2) cifp_dir = argv[1]; // explicit override

  // Load the real Navigraph openair airspace so the arrival/approach handoffs
  // (find_enclosing -> TMA -> TRACON) and the close-in ARRIVAL safety net work in
  // the replay. Env override XP_AIRSPACE; default ~/X-Plane 12/Custom Data/...
  {
    std::string ap;
    if (const char *v = std::getenv("XP_AIRSPACE")) ap = v;
    else if (const char *home = std::getenv("HOME"))
      ap = std::string(home) + "/X-Plane 12/Custom Data/airspaces/airspace.txt";
    if (!ap.empty()) {
      openair_db::init(ap); // async loader thread
      for (int i = 0; i < 300 && !openair_db::ready(); ++i)
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // wait <=30s
      std::fprintf(stderr, "openair: %s (ready=%d)\n", ap.c_str(),
                   openair_db::ready() ? 1 : 0);
    }
  }

  // atc.dat (airspace_db) -> controller/handoff resolution (Geneva/Chambery).
  {
    std::string ad;
    if (const char *v = std::getenv("XP_ATCDAT")) ad = v;
    else if (const char *home = std::getenv("HOME")) {
      std::string h(home);
      ad = h + "/X-Plane 12/Custom Data/1200 atc data/Earth nav data/atc.dat";
      if (FILE *f = std::fopen(ad.c_str(), "r")) std::fclose(f);
      else ad = h + "/X-Plane 12/Custom Data/Earth nav data/atc.dat";
    }
    if (!ad.empty()) {
      airspace_db::init(ad);
      for (int i = 0; i < 300 && !airspace_db::enabled(); ++i)
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      std::fprintf(stderr, "airspace_db: %s (enabled=%d)\n", ad.c_str(),
                   airspace_db::enabled() ? 1 : 0);
    }
  }

  auto ctx = lfmn_context(cifp_dir);

  // Pre-populate OFP with LFMN as destination and ABDIL as the last navlog fix
  // so training_jump_approach picks up the correct STAR via star_name_for_entry_fix.
  {
    simbrief_ofp::OfpData ofp;
    ofp.destination_icao  = "LFMN";
    ofp.destination_name  = "Nice";
    ofp.origin_icao       = "LFLP";
    ofp.valid             = true;
    // ABDIL is the ABDI8R STAR entry fix for LFMN 04L.
    simbrief_ofp::NavlogFix abdil;
    abdil.ident       = "ABDIL";
    abdil.via_airway  = "DCT";
    abdil.is_sid_star = true;
    ofp.navlog.push_back(abdil);
    simbrief_ofp::set(ofp);
  }

  if (cifp_dir.empty()) {
    std::fprintf(stderr,
        "Warning: CIFP directory not found.\n"
        "  Set XP_CIFP_DIR env var or pass it as an argument:\n"
        "  atc_ifr_repl \"/path/to/X-Plane 12/Custom Data/CIFP\"\n\n");
  } else {
    std::fprintf(stderr, "CIFP: %s\n", cifp_dir.c_str());
  }

  int rc = ifr_repl::run(std::move(ctx), "November Romeo Charlie");
  openair_db::stop(); // join the loader thread so exit doesn't std::terminate
  airspace_db::stop();
  return rc;
}
