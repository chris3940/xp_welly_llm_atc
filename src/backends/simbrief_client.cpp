/*
 * xp_wellys_atc - AI-powered ATC voice communication for X-Plane 12
 * Copyright (C) 2026 thWelly & Claude (Anthropic)
 * Copyright (C) 2026 Christopher P. Potter (Linux port + IFR extensions)
 *
 * Licensed under the GNU GPL-3.0-or-later. See LICENSE.
 */

#include "backends/simbrief_client.hpp"
#include "core/logging.hpp"
#include "data/simbrief_ofp.hpp"
#include "persistence/model_paths.hpp"

#include <curl/curl.h>
#include <json.hpp>

#include <atomic>
#include <cctype>
#include <cstdio>
#include <sstream>
#include <string>
#include <thread>

namespace simbrief_client {

namespace {

std::atomic<FetchStatus> g_status{FetchStatus::IDLE};
// g_last_error is only written from the fetch thread before setting
// status=ERROR, and only read from the main thread after that. Access is safe
// without a mutex because the atomic status acts as a release/acquire barrier.
static std::string g_last_error;

size_t write_to_string(char *ptr, size_t size, size_t nmemb, void *userdata) {
  auto *buf = static_cast<std::string *>(userdata);
  buf->append(ptr, size * nmemb);
  return size * nmemb;
}

void do_fetch(int pilot_id) {
  g_status.store(FetchStatus::FETCHING);
  g_last_error.clear();

  char url[256];
  std::snprintf(url, sizeof(url),
                "https://www.simbrief.com/api/xml.fetcher.php?userid=%d&json=1",
                pilot_id);

  CURL *curl = curl_easy_init();
  if (!curl) {
    g_last_error = "curl_easy_init failed";
    g_status.store(FetchStatus::FAILED);
    return;
  }

  std::string body;
  curl_easy_setopt(curl, CURLOPT_URL, url);
  curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, write_to_string);
  curl_easy_setopt(curl, CURLOPT_WRITEDATA, &body);
  curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT, 5L);
  curl_easy_setopt(curl, CURLOPT_TIMEOUT, 15L);
  curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L);

  CURLcode res = curl_easy_perform(curl);
  curl_easy_cleanup(curl);

  if (res != CURLE_OK) {
    g_last_error = curl_easy_strerror(res);
    g_status.store(FetchStatus::FAILED);
    return;
  }

  // Persist the raw JSON response for diagnostics before parsing.  The
  // parser reads only a subset of the SimBrief keys (~14 out of ~50+),
  // so any question of the form "did SimBrief actually send field X?"
  // is settled by inspecting this file rather than guessing.  Overwrites
  // on every fetch; failure is silent (best-effort).
  {
    const std::string dump_path =
        model_paths::plugin_root() + "/Resources/last_ofp.json";
    if (FILE *fp = std::fopen(dump_path.c_str(), "w")) {
      std::fwrite(body.data(), 1, body.size(), fp);
      std::fclose(fp);
      logging::info("[simbrief] raw JSON dumped to %s (%zu bytes)",
                    dump_path.c_str(), body.size());
    } else {
      logging::info("[simbrief] failed to open %s for raw-JSON dump",
                    dump_path.c_str());
    }
  }

  try {
    using json = nlohmann::json;
    auto j = json::parse(body);

    // SimBrief returns {"fetch":{"status":"Success",...},...}
    std::string api_status =
        j.value("fetch", json::object()).value("status", std::string{});
    if (api_status != "Success") {
      g_last_error = api_status.empty() ? "unexpected response" : api_status;
      g_status.store(FetchStatus::FAILED);
      return;
    }

    simbrief_ofp::OfpData ofp;
    ofp.origin_icao =
        j.value("origin", json::object()).value("icao_code", std::string{});
    {
      auto dest = j.value("destination", json::object());
      ofp.destination_icao = dest.value("icao_code", std::string{});
      // Short airport name: SimBrief returns "airport_name" e.g. "Nice Cote
      // D'Azur". Keep only the first word/city name for clean TTS delivery.
      // Fallback to "name" in case the field key differs across plan types.
      std::string full = dest.value("airport_name", std::string{});
      if (full.empty())
        full = dest.value("name", std::string{});
      if (full.empty()) {
        logging::info("[simbrief] destination name fields absent; dest keys:");
        for (auto &[k, v] : dest.items())
          if (v.is_string())
            logging::info("  %s = %s", k.c_str(), v.get<std::string>().c_str());
      }
      if (!full.empty()) {
        auto sp = full.find_first_of(" /");
        ofp.destination_name =
            (sp != std::string::npos) ? full.substr(0, sp) : full;
        // Title-case first char
        if (!ofp.destination_name.empty())
          ofp.destination_name[0] = static_cast<char>(std::toupper(
              static_cast<unsigned char>(ofp.destination_name[0])));
      }
    }

    auto gen = j.value("general", json::object());

    // SID name: sid_trans is a string when a SID is filed, or an empty
    // JSON object {} when none. gen.value() returns "" for non-string types.
    // Fallback: extract the first token of the route string — SimBrief puts
    // the SID designator there even when sid_trans is empty (e.g. "LSE2A ...").
    {
      std::string sid;
      if (gen.contains("sid_trans") && gen["sid_trans"].is_string())
        sid = gen["sid_trans"].get<std::string>();
      if (sid.empty() || sid == "NONE") {
        // Try first route token as SID candidate.
        std::string route = gen.value("route", std::string{});
        auto sp = route.find(' ');
        std::string first =
            (sp != std::string::npos) ? route.substr(0, sp) : route;
        // Heuristic: a SID designator ends with a digit+letter (e.g. "LSE2A",
        // "MOBE2D") and is at most 7 chars. Plain waypoints rarely match this.
        if (first.size() >= 4 && first.size() <= 7) {
          char last = first.back();
          char prev = first[first.size() - 2];
          if (std::isalpha(static_cast<unsigned char>(last)) &&
              std::isdigit(static_cast<unsigned char>(prev)))
            sid = first;
        }
      }
      if (sid != "NONE")
        ofp.sid_name = sid;
    }

    // First FPL fix: the waypoint immediately after the SID in the route
    // string. This is the last fix of the ATC-assigned SID — used by
    // cifp_reader to select the correct SID procedure from the CIFP file for
    // the active runway. Route examples: "ODIK2A ODIKI DCT LFMN" → "ODIKI"
    //                 "LTP2A LTPNO ..." → "LTPNO"
    //                 "ODIKI DCT LFMN" (no SID prefix) → "ODIKI"
    {
      std::string route = gen.value("route", std::string{});
      std::istringstream rss(route);
      std::string tok1, tok2;
      if (rss >> tok1) {
        bool tok1_is_sid =
            (tok1 == ofp.sid_name && !ofp.sid_name.empty()) ||
            (tok1.size() >= 4 && tok1.size() <= 7 &&
             std::isalpha(static_cast<unsigned char>(tok1.back())) &&
             std::isdigit(static_cast<unsigned char>(tok1[tok1.size() - 2])));
        if (tok1_is_sid) {
          if (rss >> tok2)
            ofp.fpl_first_fix = tok2;
        } else {
          ofp.fpl_first_fix = tok1;
        }
      }
    }

    // Cruise altitude: SimBrief uses general.cruise_altitude when set, or
    // general.initial_altitude for simple plans with a single cruise level.
    // Neither field is the ATC departure clearance altitude — we keep
    // initial_alt_ft = 0 so CIFP remains the source for {ifr_initial_altitude}.
    {
      std::string cruise_str = gen.value("cruise_altitude", std::string{});
      if (cruise_str.empty() || cruise_str == "null")
        cruise_str = gen.value("initial_altitude", std::string{});
      if (!cruise_str.empty()) {
        // Non-numeric cruise level: keep the default of 0.
        try {
          ofp.cruise_alt_ft = std::stoi(cruise_str);
        } catch (...) { // NOLINT(bugprone-empty-catch)
        }
      }
    }

    // Aircraft registration and type.
    auto ac = j.value("aircraft", json::object());
    if (ac.contains("reg") && ac["reg"].is_string())
      ofp.aircraft_reg = ac["reg"].get<std::string>();
    if (ac.contains("icao_code") && ac["icao_code"].is_string())
      ofp.aircraft_type = ac["icao_code"].get<std::string>();

    // Scheduled takeoff time (Unix timestamp) — for future slot-time check.
    auto times = j.value("times", json::object());
    if (times.contains("sched_off") && times["sched_off"].is_string()) {
      // Malformed timestamp: leave sched_off unset.
      try {
        ofp.sched_off = std::stoll(times["sched_off"].get<std::string>());
      } catch (...) { // NOLINT(bugprone-empty-catch)
      }
    }

    // Navlog: SimBrief returns navlog.fix as an array of waypoints from
    // origin airport to destination airport. Each entry has ident, via_airway,
    // pos_lat, pos_long, altitude_feet, is_sid_star ("0"/"1").
    // Guard: fix may be a JSON array or a single object (single-leg plans).
    {
      auto nl = j.value("navlog", json::object());
      auto fix_node = nl.value("fix", json{});
      // Normalise to an array so the loop below is uniform.
      if (fix_node.is_object())
        fix_node = json::array({fix_node});
      if (fix_node.is_array()) {
        ofp.navlog.reserve(fix_node.size());
        for (const auto &f : fix_node) {
          if (!f.is_object())
            continue;
          simbrief_ofp::NavlogFix fix;
          fix.ident = f.value("ident", std::string{});
          fix.via_airway = f.value("via_airway", std::string{});
          fix.is_sid_star = (f.value("is_sid_star", std::string{"0"}) == "1");
          fix.stage = f.value("stage", std::string{});
          try {
            auto lat_s = f.value("pos_lat", std::string{});
            auto lon_s = f.value("pos_long", std::string{});
            auto alt_s = f.value("altitude_feet", std::string{});
            if (!lat_s.empty())
              fix.lat = std::stod(lat_s);
            if (!lon_s.empty())
              fix.lon = std::stod(lon_s);
            if (!alt_s.empty())
              fix.alt_ft = std::stoi(alt_s);
            // Non-numeric coord/alt: keep partial fix defaults.
          } catch (...) { // NOLINT(bugprone-empty-catch)
          }
          if (!fix.ident.empty() && fix.ident != "TOC" && fix.ident != "TOD")
            ofp.navlog.push_back(std::move(fix));
        }
      }
    }

    // If the route starts with "DCT" (direct clearance with no named SID exit
    // fix), fpl_first_fix ends up as "DCT" which is useless for CIFP lookup.
    // Fall back to the first non-SID/STAR navlog entry that is not the
    // destination airport itself.
    // SimBrief pseudo-waypoints that must never be used as fpl_first_fix:
    // TOC = Top of Climb, TOD = Top of Descent — flight-planning artifacts,
    // not real navigation fixes, and absent from CIFP.
    auto is_pseudo_fix = [](const std::string &id) {
      return id == "TOC" || id == "TOD";
    };

    if (ofp.fpl_first_fix.empty() || ofp.fpl_first_fix == "DCT" ||
        ofp.fpl_first_fix == ofp.destination_icao ||
        is_pseudo_fix(ofp.fpl_first_fix)) {
      for (const auto &fix : ofp.navlog) {
        if (!fix.is_sid_star && !fix.ident.empty() &&
            fix.ident != ofp.destination_icao && !is_pseudo_fix(fix.ident)) {
          ofp.fpl_first_fix = fix.ident;
          break;
        }
      }
    }

    // Filed ICAO route + explicit FL step markers.  SimBrief stores TWO
    // route representations:
    //   general.route  — pilot-friendly, step markers STRIPPED
    //   atc.route      — ATC/ICAO format, keeps "<FIX>/N<spd>F<FL>" markers
    // Prefer atc.route so filed step-downs (e.g. BANKO/N0307F210) reach the
    // enroute walker.  Fall back to general.route if atc.route absent, and
    // in either case try general.stepclimb_string as a secondary source
    // (format: "<orig>/<FL>/<fix>/<FL>...").  Examples of atc.route seen:
    //   "N0311F220 DCT KUKEV L50 BANKO/N0307F210 Y52 SALEV DCT"
    //   "ODIKI DCT LFMN"                              (no step markers)
    //   "N0385F220 KUKEV L50 BANKO/N0307F210 Y52 SALEV/N0378F210 SALEV3P LFLP"
    {
      auto atc = j.value("atc", json::object());
      std::string atc_route = atc.value("route", std::string{});
      std::string gen_route = gen.value("route", std::string{});
      ofp.raw_route = !atc_route.empty() ? atc_route : gen_route;
      ofp.route_steps = simbrief_ofp::parse_route_steps(ofp.raw_route);

      // Secondary source: general.stepclimb_string
      //   "LIMF/0220/BANKO/0210"  = cruise FL220 from LIMF, step to FL210 at BANKO
      // Format: alternating <token>/<4-digit FL> pairs where token is either
      // the origin ICAO (first entry) or a step-fix ident.  Parse the fix/FL
      // pairs into route_steps only when the primary source produced nothing.
      if (ofp.route_steps.empty()) {
        std::string sc = gen.value("stepclimb_string", std::string{});
        if (!sc.empty()) {
          std::vector<std::string> tokens;
          std::string cur;
          for (char c : sc) {
            if (c == '/') { if (!cur.empty()) tokens.push_back(cur); cur.clear(); }
            else cur += c;
          }
          if (!cur.empty()) tokens.push_back(cur);
          // Walk pairs skipping the leading origin entry.  A valid pair
          // is (fix_ident, 4-digit FL).  Reject the first token as it's
          // the origin ICAO, not a step fix.
          for (size_t i = 2; i + 1 < tokens.size(); i += 2) {
            const std::string &ident = tokens[i];
            const std::string &fl4   = tokens[i + 1];
            if (ident.empty() || ident.size() > 5) continue;
            if (fl4.size() != 4) continue;
            bool ok = true;
            for (char c : fl4)
              if (!std::isdigit(static_cast<unsigned char>(c))) { ok = false; break; }
            if (!ok) continue;
            simbrief_ofp::RouteStep step;
            step.ident = ident;
            // "0210" -> 210, "0080" -> 80.  Filed FLs are 3-digit.
            step.cruise_fl = std::stoi(fl4);
            ofp.route_steps.push_back(std::move(step));
          }
        }
      }
    }

    // An IFR OFP MUST carry an enroute cruise altitude.  SimBrief exposes
    // it via general.cruise_altitude (or initial_altitude for simple
    // plans); if both are missing/zero the OFP is malformed for IFR use
    // and cannot drive ATC — the plugin would fall back to guessing from
    // current MSL, which is not a real clearance.  Reject upfront and
    // surface a loud warning so the pilot refiles.
    ofp.valid = !ofp.destination_icao.empty() && ofp.cruise_alt_ft > 0;
    if (!ofp.destination_icao.empty() && ofp.cruise_alt_ft <= 0) {
      g_last_error = "OFP missing cruise altitude (invalid for IFR)";
      logging::info(
          "[simbrief] REJECT OFP %s -> %s: no cruise altitude filed "
          "(general.cruise_altitude=0). An IFR flight plan must specify "
          "the cruise FL; refile in SimBrief.",
          ofp.origin_icao.c_str(), ofp.destination_icao.c_str());
    }
    simbrief_ofp::set(ofp);

    logging::info(
        "[simbrief] OFP loaded: %s -> %s (%s)  SID=%s  first_fix=%s  "
        "cruise=%dft  reg=%s  type=%s  navlog=%zu fixes",
        ofp.origin_icao.c_str(), ofp.destination_icao.c_str(),
        ofp.destination_name.empty() ? "no name" : ofp.destination_name.c_str(),
        ofp.sid_name.empty() ? "none" : ofp.sid_name.c_str(),
        ofp.fpl_first_fix.empty() ? "none" : ofp.fpl_first_fix.c_str(),
        ofp.cruise_alt_ft,
        ofp.aircraft_reg.empty() ? "?" : ofp.aircraft_reg.c_str(),
        ofp.aircraft_type.empty() ? "?" : ofp.aircraft_type.c_str(),
        ofp.navlog.size());

    // Diagnostic dump: raw filed route + extracted step markers + per-fix
    // navlog with SimBrief's computed altitudes. Enroute FL-clearance bugs
    // are almost always caused by SimBrief serving unexpected altitude_feet
    // values or the plugin misidentifying step points, so both sides need
    // to be visible in every log.
    if (!ofp.raw_route.empty())
      logging::info("[simbrief] route (atc): %s", ofp.raw_route.c_str());
    {
      auto atc_dbg = j.value("atc", json::object());
      std::string sc_dbg = gen.value("stepclimb_string", std::string{});
      if (!sc_dbg.empty())
        logging::info("[simbrief] stepclimb_string: %s", sc_dbg.c_str());
      std::string gen_route_dbg = gen.value("route", std::string{});
      if (!gen_route_dbg.empty() && gen_route_dbg != ofp.raw_route)
        logging::info("[simbrief] route (general, sanitized): %s",
                      gen_route_dbg.c_str());
    }
    if (!ofp.route_steps.empty()) {
      std::string steps_str;
      for (const auto &s : ofp.route_steps) {
        if (!steps_str.empty()) steps_str += ", ";
        char buf[32];
        std::snprintf(buf, sizeof(buf), "%s=FL%d", s.ident.c_str(),
                      s.cruise_fl);
        steps_str += buf;
      }
      logging::info("[simbrief] filed FL steps: %s", steps_str.c_str());
    } else {
      logging::info("[simbrief] filed FL steps: none (single cruise FL)");
    }
    for (size_t i = 0; i < ofp.navlog.size(); ++i) {
      const auto &f = ofp.navlog[i];
      logging::info("[simbrief] navlog[%zu] %-8s via=%-6s stage=%-3s alt=%5dft%s",
                    i,
                    f.ident.empty() ? "?" : f.ident.c_str(),
                    f.via_airway.empty() ? "-" : f.via_airway.c_str(),
                    f.stage.empty() ? "-" : f.stage.c_str(),
                    f.alt_ft, f.is_sid_star ? " SID/STAR" : "");
    }
    // Surface the OFP-rejection to the fetch-status UI so the pilot sees
    // the error instead of a silent "loaded" state that then refuses to
    // drive IFR.
    g_status.store(ofp.valid ? FetchStatus::SUCCESS : FetchStatus::FAILED);

  } catch (const std::exception &e) {
    g_last_error = std::string("parse error: ") + e.what();
    g_status.store(FetchStatus::FAILED);
  }
}

} // namespace

void fetch_async(int pilot_id) {
  if (pilot_id <= 0)
    return;
  if (g_status.load() == FetchStatus::FETCHING)
    return;
  std::thread(do_fetch, pilot_id).detach();
}

FetchStatus status() { return g_status.load(); }

std::string last_error() { return g_last_error; }

} // namespace simbrief_client
