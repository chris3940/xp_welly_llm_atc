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
 * REPL for IFR approach simulation. The aircraft starts near the STAR
 * entry and the user can fly it step by step (fly/goto/poll) while
 * watching the ATC step-down clearances fire in sequence.
 *
 * Context lives in xplane_context::g_cli_ctx (same stub as atc_repl).
 * A global now_secs counter advances with every poll/fly call so the
 * engine's cooldown logic behaves as in the live plugin.
 */

#include "ifr_repl.hpp"

#include "atc/atc_state_machine.hpp"
#include "atc/atc_templates.hpp"
#include "atc/engine.hpp"
#include "atc/flight_phase.hpp"
#include "atc/intent_parser.hpp"
#include "core/xplane_context.hpp"
#include "data/airspace_db.hpp"
#include "data/cifp_reader.hpp"
#include "data/openair_db.hpp"
#include "backends/simbrief_client.hpp"
#include "data/simbrief_ofp.hpp"
#include "persistence/settings.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <exception>
#include <iostream>
#include <sstream>
#include <string>
#include <unordered_map>

namespace xplane_context {
// Defined in tools/atc_repl/xplane_context_stub.cpp -- lets a scenario inject
// the real aerodrome elevation, which the headless build otherwise has no
// source for (apt.dat is parsed plugin-side only).
void set_airport_elevation_ft(const std::string &icao, float ft);
// Defined in tools/atc_ifr_repl/main.cpp -- without it the homonym
// disambiguation in lookup_fix_positions() is disabled and a STAR fix can
// resolve to the other side of the world.
void set_airport_pos(const std::string &icao, double lat, double lon);
void set_tower_mhz(const std::string &icao, float mhz);
} // namespace xplane_context

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace xplane_context {
extern XPlaneContext g_cli_ctx;
}

namespace ifr_repl {
namespace {

using xplane_context::FrequencyType;

// Running simulated time (seconds). Advanced by poll/fly commands.
static double g_now_secs = 0.0;


// ── String helpers ────────────────────────────────────────────────────

std::string trim(std::string s) {
  auto notsp = [](unsigned char c) { return !std::isspace(c); };
  s.erase(s.begin(), std::find_if(s.begin(), s.end(), notsp));
  s.erase(std::find_if(s.rbegin(), s.rend(), notsp).base(), s.end());
  return s;
}

std::pair<std::string, std::string> split_first(const std::string &line) {
  auto sp = line.find_first_of(" \t");
  if (sp == std::string::npos) return {line, ""};
  return {line.substr(0, sp), trim(line.substr(sp + 1))};
}

bool parse_freq_type(const std::string &s, FrequencyType &out) {
  static const std::unordered_map<std::string, FrequencyType> kMap{
      {"UNKNOWN", FrequencyType::UNKNOWN},
      {"DELIVERY", FrequencyType::DELIVERY},
      {"GROUND", FrequencyType::GROUND},
      {"TOWER", FrequencyType::TOWER},
      {"APPROACH", FrequencyType::APPROACH},
      {"DEPARTURE", FrequencyType::DEPARTURE},
      {"UNICOM", FrequencyType::UNICOM},
      {"ATIS", FrequencyType::ATIS},
  };
  std::string up = s;
  std::transform(up.begin(), up.end(), up.begin(),
                 [](unsigned char c) { return std::toupper(c); });
  auto it = kMap.find(up);
  if (it == kMap.end()) return false;
  out = it->second;
  return true;
}

// ── Position math (flat-earth, good to ~200 NM) ─────────────────────

void advance_position(xplane_context::XPlaneContext &ctx, float nm) {
  double hdg_rad = ctx.heading_true * M_PI / 180.0;
  double lat_rad = ctx.latitude * M_PI / 180.0;
  ctx.latitude  += nm * std::cos(hdg_rad) / 60.0;
  ctx.longitude += nm * std::sin(hdg_rad) / (60.0 * std::cos(lat_rad));
}

// Populate ctx.enclosing_airspaces from the real atc.dat controller index at the
// current position -- exactly like xplane_context_runtime.cpp does each frame in the
// plugin (pressure alt above the transition altitude, MSL below). Without this the
// REPL cannot exercise the enroute/approach SECTOR-CHANGE handoffs
// (poll_acc_sector_change / poll_approach's sector block), which iterate
// enclosing_airspaces -- so those handoffs were previously untestable headless and
// bugs (Vienna hysteresis, high-altitude defer) only surfaced in-sim. [C. P. Potter]
void refresh_enclosing(xplane_context::XPlaneContext &ctx) {
  if (!airspace_db::enabled()) {
    ctx.enclosing_airspaces.clear();
    return;
  }
  const int trans_alt_ft = ctx.transition_alt_ft > 0 ? ctx.transition_alt_ft : 5000;
  const float airspace_alt_ft =
      (ctx.altitude_ft_msl > static_cast<float>(trans_alt_ft)) ? ctx.pressure_alt_ft
                                                               : ctx.altitude_ft_msl;
  ctx.enclosing_airspaces =
      airspace_db::find_enclosing(ctx.latitude, ctx.longitude, airspace_alt_ft);
}

// ── Poll helper: run all frame-driven IFR polls for one dt step ──────

void run_polls(float dt) {
  auto &ctx = xplane_context::g_cli_ctx;
  g_now_secs += dt;
  ctx.now_secs = g_now_secs;
  refresh_enclosing(ctx); // atc.dat enclosing sectors, as the plugin does per frame

  auto emit = [](const char *tag, const std::string &text) {
    if (!text.empty())
      std::printf("ATC [%s]: %s\n", tag, text.c_str());
  };

  // Route tracker must run BEFORE poll_approach so s_route_fix_idx is
  // up-to-date when poll_approach checks (s_route_fix_idx > s_faf_route_idx)
  // for the Tower handoff. This matches the order in atc_session::update().
  //
  // The tracker's internal tick accumulates 1/60 per call and fires at 1.0
  // (i.e., ~once per second at 60 FPS). Simulate dt seconds worth of frames
  // so proximity events fire correctly even in large poll steps.
  {
    int rt_calls = std::max(60, static_cast<int>(std::ceil(dt * 60.0f)));
    for (int i = 0; i < rt_calls; ++i) {
      std::string t = engine::poll_route_tracker(ctx);
      if (!t.empty())
        std::printf("-- %s --\n", t.c_str());
    }
  }

  // Log every ATC state change with position so the replay shows exactly WHERE
  // DESCENT->ARRIVAL->APPROACH fired (relative to the STAR fixes / IAF).
  static std::string s_last_state;
  auto log_state = [&]() {
    const char *nm = atc_state_machine::state_name(atc_state_machine::get_state());
    if (nm && s_last_state != nm) {
      std::printf(">> STATE %-20s -> %-22s @ lat=%.4f lon=%.4f pa=%.0fft t=%.0fs\n",
                  s_last_state.empty() ? "(init)" : s_last_state.c_str(), nm,
                  ctx.latitude, ctx.longitude,
                  static_cast<double>(ctx.pressure_alt_ft), g_now_secs);
      s_last_state = nm;
    }
  };
  log_state();

  // Order mirrors atc_session::update(): sid -> profile enforcement -> enroute
  // -> descent -> arrival -> approach.
  std::string out;
  bool rb = false;
  // The plugin runs this every frame (main.cpp). Without it the REPL's phase stays
  // PARKED forever, and every poll that opens with a PARKED/TAXI guard -- most
  // importantly poll_sid_climb -- returned false before doing anything, so the whole
  // SID climb ladder was structurally untestable headless. [C. P. Potter]
  flight_phase::update(ctx, dt);
  if (engine::poll_departure_handoff(ctx, dt, &out)) { emit("dep", out); out.clear(); }
  if (engine::poll_sid_climb(ctx, dt, &out))         { emit("sid", out); out.clear(); }
  if (engine::poll_profile_enforcement(ctx, dt, &out, &rb)) { emit("profile", out); out.clear(); }
  if (engine::poll_enroute(ctx, dt, &out))           { emit("enroute", out); out.clear(); }
  if (engine::poll_star_clearance_safety_net(ctx, &out, &rb)) { emit("star-net", out); out.clear(); }
  if (engine::poll_descent(ctx, dt, &out, &rb))      { emit("descent", out); out.clear(); }
  if (engine::poll_arrival(ctx, dt, &out, &rb))      { emit("arrival", out); out.clear(); }
  if (engine::poll_vector_to_final(ctx, dt, &out, &rb)) { emit("vector", out); out.clear(); }
  if (engine::poll_approach(ctx, dt, &out))          { emit("approach", out); out.clear(); }
  if (engine::poll_approach_alignment(ctx, dt, &out)){ emit("align", out); out.clear(); }
  if (engine::poll_readback_reminder(ctx, g_now_secs, &out)) { emit("readback", out); out.clear(); }
  if (engine::poll_go_around(ctx, g_now_secs, &out)) { emit("go_around", out); out.clear(); }
  log_state();
}

// ── Command handlers ──────────────────────────────────────────────────

void cmd_say(const std::string &callsign, const std::string &text) {
  if (text.empty()) {
    std::fprintf(stderr, "Usage: say <transcript>\n");
    return;
  }
  auto &ctx = xplane_context::g_cli_ctx;
  ctx.now_secs = g_now_secs;
  engine::Input in{text, 1.0f, &ctx, callsign, g_now_secs};
  engine::process_transcript(std::move(in), [](engine::Output out) {
    std::printf("PILOT : %s\n", out.parsed.raw_transcript.c_str());
    std::printf("INTENT: %s (%.2f)\n",
                intent_parser::intent_name(out.parsed.intent),
                out.parsed.confidence);
    std::printf("ATC   : %s\n",
                out.response_text.empty() ? "(silent)" : out.response_text.c_str());
    std::printf("STATE : %s\n",
                atc_state_machine::state_name(atc_state_machine::get_state()));
  });
}

void cmd_set(std::string &callsign, const std::string &rest) {
  auto [field, value] = split_first(rest);
  if (field.empty() || value.empty()) {
    std::fprintf(stderr, "Usage: set <field> <value>  (try 'help')\n");
    return;
  }
  auto &ctx = xplane_context::g_cli_ctx;
  try {
    if (field == "lat") {
      ctx.latitude = std::stod(value);
    } else if (field == "lon") {
      ctx.longitude = std::stod(value);
    } else if (field == "alt") {
      ctx.altitude_ft_msl = std::stof(value);
      ctx.pressure_alt_ft = std::stof(value); // sync both unless pa given separately
    } else if (field == "pa") {
      ctx.pressure_alt_ft = std::stof(value);
    } else if (field == "heading") {
      ctx.heading_true = std::stof(value);
      ctx.heading_mag = ctx.heading_true; // harness: magvar=0 so spoken vectors == true
    } else if (field == "gs") {
      ctx.groundspeed_kts = std::stof(value);
      // The harness flies in still air, so IAS == groundspeed. Without this
      // ctx.indicated_airspeed_kts stayed 0 and every rule that reads it -- the
      // vectoring speed instructions among them -- was silently untestable.
      ctx.indicated_airspeed_kts = ctx.groundspeed_kts;
    } else if (field == "vs") {
      ctx.vertical_speed_fpm = std::stof(value);
    } else if (field == "cruise") {
      ctx.ifr_cruise_alt_ft = static_cast<int>(std::stof(value));
    } else if (field == "airport_lat") {
      // Departure/nearest field position. The plugin fills these from apt.dat; the
      // REPL cannot, and without them sid_step1_hold_active() bails out on its
      // "departure fix not captured" guard, so the whole distance-based departure
      // hold (Annecy FL110 / 30 NM) was silently inert headless. [C. P. Potter]
      ctx.airport_lat = std::stod(value);
    } else if (field == "airport_lon") {
      ctx.airport_lon = std::stod(value);
    } else if (field == "cifp_dir") {
      ctx.cifp_dir = value;
    } else if (field == "dest") {
      std::string up = value;
      std::transform(up.begin(), up.end(), up.begin(),
                     [](unsigned char c) { return std::toupper(c); });
      ctx.ifr_destination = up;
      // Mirror to OFP so training_jump_approach can pick up dest ICAO.
      auto ofp = simbrief_ofp::get();
      ofp.destination_icao = up;
      ofp.valid = true;
      simbrief_ofp::set(ofp);
    } else if (field == "zulu") {
      // HHMM (e.g. 1420) -> seconds since midnight for ctx.zulu_time_sec.
      int hhmm = static_cast<int>(std::stof(value));
      ctx.zulu_time_sec =
          static_cast<float>((hhmm / 100) * 3600 + (hhmm % 100) * 60);
    } else if (field == "qnh") {
      ctx.qnh_hpa = static_cast<int>(std::stof(value));
    } else if (field == "wind_dir") {
      ctx.wind_direction_deg = std::stof(value);
    } else if (field == "wind_kt") {
      ctx.wind_speed_kt = std::stof(value);
    } else if (field == "visibility") {
      ctx.visibility_m = std::stof(value);
    } else if (field == "airport") {
      std::string up = value;
      std::transform(up.begin(), up.end(), up.begin(),
                     [](unsigned char c) { return std::toupper(c); });
      ctx.nearest_airport_id = up;
    } else if (field == "com") {
      ctx.com1_freq_mhz = std::stof(value);
      ctx.active_com = 1;
    } else if (field == "freq_type") {
      FrequencyType ft;
      if (!parse_freq_type(value, ft))
        throw std::runtime_error("unknown freq_type");
      ctx.frequency_type = ft;
    } else if (field == "runway") {
      ctx.active_runway = value;
    } else if (field == "callsign") {
      callsign = value;
      settings::set_pilot_callsign_raw(value);
    } else if (field == "region") {
      std::string up = value;
      std::transform(up.begin(), up.end(), up.begin(),
                     [](unsigned char c) { return std::toupper(c); });
      settings::set_atc_profile(up);
      atc_templates::reload();
      flight_phase::reload();
    } else if (field == "navlog_clear") {
      auto ofp = simbrief_ofp::get();
      ofp.navlog.clear();
      simbrief_ofp::set(ofp);
    } else if (field == "navlog_fix") {
      // Rich form (replay): set navlog_fix <ident> <lat> <lon> <alt_ft> <stage> <sidstar0|1>
      // Minimal form still works: set navlog_fix ABDIL
      auto ofp = simbrief_ofp::get();
      simbrief_ofp::NavlogFix f;
      std::istringstream iss(value);
      std::string stage;
      int sidstar = 0;
      iss >> f.ident;
      if (iss >> f.lat >> f.lon >> f.alt_ft >> stage >> sidstar) {
        f.stage = stage;
        f.is_sid_star = (sidstar != 0);
      } else {
        f.is_sid_star = true;
      }
      ofp.navlog.push_back(f);
      simbrief_ofp::set(ofp);
    } else if (field == "approach_desig") {
      // Force a specific approach procedure, e.g. "set approach_desig I04LZ"
      auto ofp = simbrief_ofp::get();
      std::string up = value;
      std::transform(up.begin(), up.end(), up.begin(),
                     [](unsigned char c) { return std::toupper(c); });
      ofp.preferred_approach_designator = up;
      simbrief_ofp::set(ofp);
    } else if (field == "state") {
      atc_state_machine::set_state(atc_state_machine::state_from_name(value));
    } else if (field == "on_ground") {
      ctx.on_ground = (value == "1" || value == "true");
    } else if (field == "agl") {
      ctx.height_agl_ft = std::stof(value);
    } else if (field == "ifr_sid") {
      ctx.ifr_cifp_sid = value; // ATC-assigned CIFP SID (empty = omni departure)
      // Mirror what xplane_context_runtime does when the runway/SID changes:
      // resolve the SID's published minima from the REAL CIFP. Without this the
      // climb-floor logic (sid_climb_floor_ft) had no data headless and every
      // SID looked unconstrained. Needs cifp_dir + airport + runway set first.
      // [C. P. Potter]
      if (!ctx.cifp_dir.empty() && !ctx.nearest_airport_id.empty() &&
          !ctx.active_runway.empty()) {
        auto bind = cifp_reader::sid_binding_altitude(
            ctx.cifp_dir, ctx.nearest_airport_id, ctx.active_runway, value);
        ctx.ifr_sid_min_alt_ft = bind.alt.feet;
        ctx.ifr_sid_min_is_fl = bind.alt.is_fl;
        ctx.ifr_sid_min_waypoint = bind.waypoint;
        ctx.ifr_sid_floor_alt_ft = bind.floor_alt.feet;
        ctx.ifr_sid_floor_waypoint = bind.floor_waypoint;
        if (ctx.ifr_sid_last_fix.empty())
          ctx.ifr_sid_last_fix =
              cifp_reader::sid_last_fix(ctx.cifp_dir, ctx.nearest_airport_id,
                                        value);
        std::fprintf(stderr,
                     "  [cifp] %s: min %d ft @%s, floor %d ft @%s, exit %s\n",
                     value.c_str(), ctx.ifr_sid_min_alt_ft,
                     ctx.ifr_sid_min_waypoint.c_str(), ctx.ifr_sid_floor_alt_ft,
                     ctx.ifr_sid_floor_waypoint.c_str(),
                     ctx.ifr_sid_last_fix.c_str());
      }
    } else if (field == "ifr_sid_last_fix") {
      ctx.ifr_sid_last_fix = value;
    } else {
      std::fprintf(stderr, "Unknown field '%s' (try 'help')\n", field.c_str());
      return;
    }
    std::fprintf(stderr, "set %s = %s\n", field.c_str(), value.c_str());
  } catch (const std::exception &e) {
    std::fprintf(stderr, "Error setting %s: %s\n", field.c_str(), e.what());
  }
}

void cmd_poll(const std::string &rest) {
  float dt = 5.0f;
  if (!rest.empty()) {
    try { dt = std::stof(rest); } catch (...) {}
  }
  run_polls(dt);
  std::printf("[t=%.0fs] lat=%.4f lon=%.4f alt=%.0fft pa=%.0fft\n",
              g_now_secs,
              xplane_context::g_cli_ctx.latitude,
              xplane_context::g_cli_ctx.longitude,
              xplane_context::g_cli_ctx.altitude_ft_msl,
              xplane_context::g_cli_ctx.pressure_alt_ft);
}

void cmd_fly(const std::string &rest) {
  if (rest.empty()) {
    std::fprintf(stderr, "Usage: fly <nm>\n");
    return;
  }
  auto &ctx = xplane_context::g_cli_ctx;
  float total_nm;
  try { total_nm = std::stof(rest); } catch (...) {
    std::fprintf(stderr, "Invalid distance\n"); return;
  }
  float gs = std::max(ctx.groundspeed_kts, 1.0f);
  float dt = 5.0f; // seconds per simulation step
  float d_nm = gs * dt / 3600.0f;
  int steps = std::max(1, static_cast<int>(std::ceil(total_nm / d_nm)));

  std::fprintf(stderr, "Flying %.1f NM (hdg=%.0f gs=%.0f kt, %d steps of %.1f s)...\n",
               total_nm, ctx.heading_true, gs, steps, dt);

  float flown = 0.0f;
  for (int i = 0; i < steps; ++i) {
    float step = std::min(d_nm, total_nm - flown);
    advance_position(ctx, step);
    float alt_change = ctx.vertical_speed_fpm * dt / 60.0f;
    ctx.altitude_ft_msl += alt_change;
    ctx.pressure_alt_ft += alt_change;
    flown += step;
    run_polls(dt);
  }
  std::printf("[t=%.0fs] flew %.1f NM -> lat=%.4f lon=%.4f alt=%.0fft pa=%.0fft\n",
              g_now_secs, total_nm,
              ctx.latitude, ctx.longitude,
              ctx.altitude_ft_msl, ctx.pressure_alt_ft);
}

void cmd_goto(const std::string &fix) {
  if (fix.empty()) {
    std::fprintf(stderr, "Usage: goto <FIXNAME>\n");
    return;
  }
  auto &ctx = xplane_context::g_cli_ctx;
  if (ctx.cifp_dir.empty()) {
    std::fprintf(stderr, "cifp_dir not set — use 'set cifp_dir <path>'\n");
    return;
  }
  std::string up = fix;
  std::transform(up.begin(), up.end(), up.begin(),
                 [](unsigned char c) { return std::toupper(c); });
  auto pos = cifp_reader::lookup_fix_positions(ctx.cifp_dir, {up},
                                               ctx.nearest_airport_id);
  auto it = pos.find(up);
  if (it == pos.end()) {
    std::fprintf(stderr, "Fix '%s' not found in earth_fix.dat\n", up.c_str());
    return;
  }
  ctx.latitude  = it->second.first;
  ctx.longitude = it->second.second;
  std::printf("Teleported to %s: lat=%.4f lon=%.4f\n",
              up.c_str(), ctx.latitude, ctx.longitude);
}

void reset_track_vs(); // defined with the track state below

void cmd_jump(const std::string &rest) {
  auto [sub, arg] = split_first(rest);
  if (sub == "approach") {
    engine::training_jump_approach();
    std::printf("Jumped to IFR_APPROACH_CONTACT (dest=%s)\n",
                xplane_context::g_cli_ctx.ifr_destination.c_str());
    std::printf("STATE : %s\n",
                atc_state_machine::state_name(atc_state_machine::get_state()));
  } else if (sub == "enroute") {
    int alt = 0;
    try { alt = std::stoi(arg); } catch (...) {}
    if (alt <= 0) {
      std::fprintf(stderr, "Usage: jump enroute <alt_ft>\n");
      return;
    }
    engine::training_jump_enroute(alt);
    std::printf("Jumped to IFR_ENROUTE_CRUISE at %d ft\n", alt);
    std::printf("STATE : %s\n",
                atc_state_machine::state_name(atc_state_machine::get_state()));
  } else if (sub == "predep") {
    engine::training_jump_predep();
    std::printf("Jumped to IFR_PREDEP_CLEARANCE\n");
    std::printf("STATE : %s\n",
                atc_state_machine::state_name(atc_state_machine::get_state()));
  } else if (sub == "descent") {
    // Enroute jump (sets dest / controller baseline / cleared alt) then force the
    // DESCENT state so poll_acc_sector_change runs (it fires only in DESCENT/ARRIVAL,
    // never CRUISE). Lets the harness replay enroute SECTOR handoffs without a full
    // TOD/STAR setup. [C. P. Potter]
    int alt = 0;
    try { alt = std::stoi(arg); } catch (...) {}
    if (alt <= 0) { std::fprintf(stderr, "Usage: jump descent <alt_ft>\n"); return; }
    engine::training_jump_enroute(alt);
    atc_state_machine::set_state(atc_state_machine::ATCState::IFR_DESCENT);
    std::printf("Jumped to IFR_DESCENT at %d ft (dest=%s)\n", alt,
                xplane_context::g_cli_ctx.ifr_destination.c_str());
    std::printf("STATE : %s\n",
                atc_state_machine::state_name(atc_state_machine::get_state()));
  } else {
    std::fprintf(stderr, "Usage: jump approach|enroute <alt_ft>|descent <alt_ft>|predep\n");
  }
  reset_track_vs();
}

void cmd_enc() {
  auto &ctx = xplane_context::g_cli_ctx;
  const int alt = static_cast<int>(ctx.altitude_ft_msl);
  auto inner = openair_db::find_enclosing(ctx.latitude, ctx.longitude, alt);
  std::printf("find_enclosing @ %.4f,%.4f %d ft -> INNERMOST '%s' class=%d floor=%d ceil=%d\n",
              ctx.latitude, ctx.longitude, alt, inner.name.c_str(),
              static_cast<int>(inner.ac_class), inner.floor_ft, inner.ceiling_ft);
  auto all = openair_db::find_all_enclosing(ctx.latitude, ctx.longitude, alt);
  for (const auto &e : all)
    std::printf("   enclosing: '%s' class=%d floor=%d ceil=%d\n", e.name.c_str(),
                static_cast<int>(e.ac_class), e.floor_ft, e.ceiling_ft);
  // Terminal-area queries, both answers side by side. The name-based lookup is
  // what every arrival uses today; the stack walk is the fallback for exports
  // that omit the type word (Germany, and Turin locally). Printing both makes
  // the difference visible at a glance when flying an ED**/LI** arrival.
  const auto t_named = openair_db::terminal_tma(ctx.latitude, ctx.longitude);
  const auto t_walk =
      openair_db::terminal_stack_shelf(ctx.latitude, ctx.longitude);
  std::printf("terminal by NAME : %s\n",
              t_named.name.empty()
                  ? "(none)"
                  : (t_named.name + " " + std::to_string(t_named.floor_ft) +
                     "-" + std::to_string(t_named.ceiling_ft))
                        .c_str());
  std::printf("terminal by WALK : %s\n",
              t_walk.name.empty()
                  ? "(none)"
                  : (t_walk.name + " " + std::to_string(t_walk.floor_ft) + "-" +
                     std::to_string(t_walk.ceiling_ft))
                        .c_str());
  // atc.dat enclosing controllers -- what the SECTOR-CHANGE handoffs actually see.
  refresh_enclosing(ctx);
  std::printf("atc.dat enclosing (%zu):\n", ctx.enclosing_airspaces.size());
  for (const auto *c : ctx.enclosing_airspaces) {
    if (!c) continue;
    std::printf("   CTR '%s' (%s) role=%d freqs=%zu\n", c->name.c_str(),
                c->facility_id.c_str(), static_cast<int>(c->role),
                c->freqs_khz.size());
  }
}

// Replay a single track point: teleport to lat/lon/alt, refresh enclosing, run one
// poll step. Scriptable line-by-line to walk a real flight path and watch the
// handoff chain fire. Usage: track <lat> <lon> <alt_ft> [dt]
// Altitude at the previous `track`, so the vertical speed can be derived.
// WITHOUT THIS THE AIRCRAFT IS ALWAYS LEVEL as far as the engine is concerned:
// cmd_track moved it and changed its altitude but left vertical_speed_fpm at
// zero, so poll_altitude_compliance -- which fires only when the aircraft is
// NOT moving toward its cleared level -- challenged a perfectly obedient
// descent three times over one arrival. The engine was right; the harness was
// flying a teleporting aircraft. [C. P. Potter]
static float s_prev_track_alt_ft = -1e9f;

void cmd_track(const std::string &rest) {
  std::istringstream iss(rest);
  double lat, lon;
  float alt;
  float dt = 30.0f;
  if (!(iss >> lat >> lon >> alt)) {
    std::fprintf(stderr, "Usage: track <lat> <lon> <alt_ft> [dt]\n");
    return;
  }
  iss >> dt;
  auto &ctx = xplane_context::g_cli_ctx;
  ctx.latitude = lat;
  ctx.longitude = lon;
  if (s_prev_track_alt_ft > -1e8f && dt > 0.0f)
    ctx.vertical_speed_fpm = (alt - s_prev_track_alt_ft) / dt * 60.0f;
  s_prev_track_alt_ft = alt;
  ctx.altitude_ft_msl = alt;
  ctx.pressure_alt_ft = alt;
  run_polls(dt);
  std::printf("[t=%.0fs] @ %.4f,%.4f %.0fft  freq COM1=%.3f\n",
              g_now_secs, lat, lon, alt, ctx.com1_freq_mhz);
}

void cmd_state(const std::string &callsign) {
  const auto &ctx = xplane_context::g_cli_ctx;
  auto ofp = simbrief_ofp::get();
  std::printf("Callsign:  %s\n", callsign.c_str());
  std::printf("Airport:   %s  towered=%s  runway=%s\n",
              ctx.nearest_airport_id.c_str(),
              ctx.is_towered_airport ? "yes" : "no",
              ctx.active_runway.c_str());
  std::printf("Position:  lat=%.4f lon=%.4f\n", ctx.latitude, ctx.longitude);
  std::printf("Alt:       %.0f ft MSL  PA=%.0f ft  AGL=%.0f ft\n",
              ctx.altitude_ft_msl, ctx.pressure_alt_ft, ctx.height_agl_ft);
  std::printf("Motion:    hdg=%.0f  gs=%.0f kt  vs=%+.0f fpm\n",
              ctx.heading_true, ctx.groundspeed_kts, ctx.vertical_speed_fpm);
  std::printf("Radio:     COM1=%.3f MHz  freq_type=%s\n",
              ctx.com1_freq_mhz,
              xplane_context::frequency_type_name(ctx.frequency_type));
  std::printf("Weather:   QNH=%d hPa  wind=%.0f/%.0f kt  vis=%.0f m\n",
              ctx.qnh_hpa, ctx.wind_direction_deg, ctx.wind_speed_kt,
              ctx.visibility_m);
  std::printf("IFR dest:  %s  CIFP=%s\n",
              ctx.ifr_destination.empty() ? "(none)" : ctx.ifr_destination.c_str(),
              ctx.cifp_dir.empty() ? "(none)" : "set");
  std::printf("OFP:       dest=%s  valid=%s  navlog=%zu fixes\n",
              ofp.destination_icao.empty() ? "(none)" : ofp.destination_icao.c_str(),
              ofp.valid ? "yes" : "no",
              ofp.navlog.size());
  std::printf("ATC state: %s\n",
              atc_state_machine::state_name(atc_state_machine::get_state()));
  std::printf("Time:      t=%.0f s\n", g_now_secs);
  std::printf("Region:    %s\n", settings::atc_profile().c_str());
}

// A reset or a jump breaks the altitude continuity: the next track must not
// derive a vertical speed from a position the aircraft never flew through.
void reset_track_vs() { s_prev_track_alt_ft = -1e9f; }

void cmd_reset() {
  atc_state_machine::reset();
  engine::reset();
  g_now_secs = 0.0;
  xplane_context::g_cli_ctx.now_secs = 0.0;
  std::fprintf(stderr, "Engine state reset (context unchanged, t=0).\n");
  reset_track_vs();
}

// load_ofp <file>: replay a saved SimBrief last_ofp.json (raw API response) --
// runs the SAME parser as the live plugin fetch, so the harness sees the exact
// route/STAR/steps/navlog the sim would. After loading, drive the flight with
// jump/fly/say to observe every sector handoff + clearance. [C. P. Potter]
void cmd_load_ofp(const std::string &path) {
  if (path.empty()) {
    std::fprintf(stderr, "Usage: load_ofp <path-to-last_ofp.json>\n");
    return;
  }
  FILE *fp = std::fopen(path.c_str(), "rb");
  if (!fp) {
    std::fprintf(stderr, "load_ofp: cannot open '%s'\n", path.c_str());
    return;
  }
  std::string body;
  char buf[8192];
  size_t n;
  while ((n = std::fread(buf, 1, sizeof(buf), fp)) > 0)
    body.append(buf, n);
  std::fclose(fp);
  if (body.empty()) {
    std::fprintf(stderr, "load_ofp: '%s' is empty\n", path.c_str());
    return;
  }

  simbrief_client::load_ofp_body(body);
  const auto ofp = simbrief_ofp::get();
  if (!ofp.valid) {
    std::fprintf(stderr,
                 "load_ofp: parsed but OFP not valid for IFR (%s). "
                 "A SimBrief OFP must carry origin/dest + cruise FL.\n",
                 simbrief_client::last_error().c_str());
    return;
  }
  // Mirror the destination into the context so training jumps + arrival logic
  // pick it up (dest ICAO drives best_approach / the STAR/approach resolution).
  auto &ctx = xplane_context::g_cli_ctx;
  ctx.ifr_destination = ofp.destination_icao;
  std::fprintf(stderr,
               "load_ofp: %s -> %s  cruise=%dft  SID=%s  first_fix=%s  "
               "navlog=%zu fixes  route=[%s]\n",
               ofp.origin_icao.c_str(), ofp.destination_icao.c_str(),
               ofp.cruise_alt_ft, ofp.sid_name.empty() ? "none" : ofp.sid_name.c_str(),
               ofp.fpl_first_fix.empty() ? "none" : ofp.fpl_first_fix.c_str(),
               ofp.navlog.size(), ofp.raw_route.c_str());
}

void cmd_help() {
  std::printf(
      "Commands:\n"
      "  say <text>              Process a pilot transcript\n"
      "  load_ofp <file>         Replay a saved SimBrief last_ofp.json (real flight\n"
      "                          plan). Plugin still CHOOSES the SID/STAR/approach\n"
      "                          from CIFP -- the OFP only supplies the route fixes.\n"
      "  poll [dt=5]             Advance dt seconds and run all IFR polls\n"
      "  fly <nm>                Fly NM at current hdg/gs/vs, polling every 5 s\n"
      "  goto <FIXNAME>          Teleport aircraft to fix (earth_fix.dat lookup)\n"
      "  track <lat> <lon> <alt> [dt]  Teleport to a point + run one poll (sector handoffs)\n"
      "  jump approach           Jump to IFR_APPROACH_CONTACT state\n"
      "  jump enroute <alt_ft>   Jump to IFR_ENROUTE_CRUISE state\n"
      "  jump predep             Jump to IFR_PREDEP_CLEARANCE state\n"
      "  set <field> <value>     Modify context (see fields below)\n"
      "  state                   Show full context + ATC state\n"
      "  reset                   Reset engine state (context unchanged, t=0)\n"
      "  help                    This message\n"
      "  quit                    Exit\n"
      "\n"
      "Set fields:\n"
      "  lat <deg>          Aircraft latitude\n"
      "  lon <deg>          Aircraft longitude\n"
      "  alt <ft>           True altitude (also syncs pressure_alt)\n"
      "  pa <ft>            Pressure altitude only\n"
      "  heading <deg>      True heading\n"
      "  gs <kt>            Ground speed\n"
      "  vs <fpm>           Vertical speed (negative = descending)\n"
      "  cifp_dir <path>    Path to X-Plane CIFP directory\n"
      "  dest <ICAO>        IFR destination (also sets OFP.destination_icao)\n"
      "  qnh <hpa>          QNH pressure in hPa\n"
      "  wind_dir <deg>     Wind direction (true) — used for runway selection\n"
      "  wind_kt <kt>       Wind speed\n"
      "  visibility <m>     Visibility in metres\n"
      "  airport <ICAO>     Nearest airport\n"
      "  airport_lat <deg>  Field position (needed by the departure-hold distance)\n"
      "  airport_lon <deg>\n"
      "  cruise <ft>        Filed cruise altitude (drives the SID climb ladder)\n"
      "  com <MHz>          COM1 frequency\n"
      "  freq_type <TYPE>   APPROACH|TOWER|GROUND|DEPARTURE|UNICOM|DELIVERY\n"
      "  runway <id>        Active runway (e.g. 04L)\n"
      "  callsign <text>    Pilot callsign\n"
      "  region EU|US|DE    ATC phraseology region\n"
      "  navlog_fix <IDENT> Append a fix ident to the OFP navlog\n"
      "  approach_desig <D> Force a specific approach (e.g. I04LZ) over best_approach()\n"
      "\n"
      "Typical IFR approach test flow (LFMN):\n"
      "  jump approach\n"
      "  say november romeo charlie, inbound, flight level one one five\n"
      "  fly 20          <- flies toward dest, watch step-down clearances fire\n"
      "  goto FN04A      <- teleport to FAF, then poll for Tower handoff\n"
      "  poll 5\n");
}

} // namespace

int run(xplane_context::XPlaneContext ctx, std::string callsign) {
  ctx.avionics_on = true;
  ctx.com_radio_powered = true;
  if (ctx.aircraft_icao.empty()) ctx.aircraft_icao = "B738";
  if (!callsign.empty()) settings::set_pilot_callsign_raw(callsign);

  // Mirror OFP destination from ctx so training_jump_approach works on start.
  if (!ctx.ifr_destination.empty()) {
    auto ofp = simbrief_ofp::get();
    if (ofp.destination_icao.empty()) {
      ofp.destination_icao = ctx.ifr_destination;
      ofp.valid = true;
      simbrief_ofp::set(ofp);
    }
  }

  xplane_context::g_cli_ctx = std::move(ctx);
  atc_state_machine::reset();
  engine::reset();
  g_now_secs = 0.0;

  std::fprintf(
      stderr,
      "atc_ifr_repl — type 'help' for commands, 'quit' or Ctrl+D to exit.\n");
  cmd_state(callsign);

  std::string line;
  while (true) {
    std::fprintf(stderr, "\n> ");
    if (!std::getline(std::cin, line)) {
      std::fprintf(stderr, "\n");
      return 0;
    }
    line = trim(std::move(line));
    if (line.empty()) continue;

    auto [cmd, rest] = split_first(line);
    if (cmd == "say")
      cmd_say(callsign, rest);
    else if (cmd == "load_ofp")
      cmd_load_ofp(rest);
    else if (cmd == "set")
      cmd_set(callsign, rest);
    else if (cmd == "poll")
      cmd_poll(rest);
    else if (cmd == "fly")
      cmd_fly(rest);
    else if (cmd == "goto")
      cmd_goto(rest);
    else if (cmd == "jump")
      cmd_jump(rest);
    else if (cmd == "enc")
      cmd_enc();
    else if (cmd == "track")
      cmd_track(rest);
    else if (cmd == "arrival") {
      // arrival <dest> <STAR> <approach>  -- force an arrival + build the route.
      auto [dest, r1] = split_first(rest);
      auto [star, appr] = split_first(r1);
      if (dest.empty() || star.empty() || appr.empty())
        std::fprintf(stderr, "Usage: arrival <dest> <STAR> <approach>\n");
      else {
        engine::training_set_arrival(dest, star, appr);
        std::printf("arrival set: dest=%s STAR=%s approach=%s\n",
                    dest.c_str(), star.c_str(), appr.c_str());
      }
    }
    else if (cmd == "tower_freq") {
      std::istringstream ai(rest);
      std::string icao;
      float mhz = 0.0f;
      if (!(ai >> icao >> mhz))
        std::fprintf(stderr, "Usage: tower_freq <ICAO> <MHz>\n");
      else {
        xplane_context::set_tower_mhz(icao, mhz);
        std::printf("tower_freq %s = %.3f\n", icao.c_str(),
                    static_cast<double>(mhz));
      }
    }
    else if (cmd == "airport_pos") {
      std::istringstream ai(rest);
      std::string icao;
      double la = 0.0, lo = 0.0;
      if (!(ai >> icao >> la >> lo))
        std::fprintf(stderr, "Usage: airport_pos <ICAO> <lat> <lon>\n");
      else {
        xplane_context::set_airport_pos(icao, la, lo);
        std::printf("airport_pos %s = %.4f,%.4f\n", icao.c_str(), la, lo);
      }
    }
    else if (cmd == "airport_elev") {
      // airport_elev <ICAO> <ft> -- so the vectoring terrain test (lowest
      // approach-sector MSA MINUS FIELD ELEVATION) is evaluated on the bench
      // against the same numbers the plugin sees. Without it every field sat at
      // 0 ft and the test judged airports by their raw MSA: LSGG came out 7000 ft
      // above its own field instead of 5589, and was refused vectors.
      std::istringstream ai(rest);
      std::string icao;
      float ft = 0.0f;
      if (!(ai >> icao >> ft))
        std::fprintf(stderr, "Usage: airport_elev <ICAO> <ft>\n");
      else {
        xplane_context::set_airport_elevation_ft(icao, ft);
        std::printf("airport_elev %s = %.0f ft\n", icao.c_str(),
                    static_cast<double>(ft));
      }
    }
    else if (cmd == "fmsroute") {
      // The cleared route WITH geometry, for a driver that flies like an FMS.
      // One fix per line: idx ident lat lon alt fl ceil floor spd app
      int idx = 0;
      const auto all = engine::route_fixes_info(&idx);
      std::printf("fmsroute idx=%d n=%zu\n", idx, all.size());
      for (size_t i = 0; i < all.size(); ++i) {
        const auto &f = all[i];
        std::printf("  %zu %s %.6f %.6f %d %d %d %d %d %d\n", i,
                    f.ident.empty() ? "?" : f.ident.c_str(), f.lat, f.lon,
                    f.alt_ft, f.is_fl ? 1 : 0, f.is_ceiling ? 1 : 0,
                    f.is_floor ? 1 : 0, f.speed_kt, f.is_approach ? 1 : 0);
      }
    }
    else if (cmd == "route") {
      int idx = 0;
      const auto all = engine::route_fixes_all_debug(&idx);
      std::printf("route (idx=%d, %zu fixes): ", idx, all.size());
      for (const auto &id : all)
        std::printf("%s ", id.c_str());
      std::printf("\n");
    }
    else if (cmd == "state")
      cmd_state(callsign);
    else if (cmd == "reset")
      cmd_reset();
    else if (cmd == "help")
      cmd_help();
    else if (cmd == "quit" || cmd == "exit")
      return 0;
    else
      std::fprintf(stderr, "Unknown command: %s (try 'help')\n", cmd.c_str());
  }
}

} // namespace ifr_repl
