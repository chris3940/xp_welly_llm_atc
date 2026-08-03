/*
 * xp_wellys_atc - AI-powered ATC voice communication for X-Plane 12
 * Copyright (C) 2026 thWelly & Claude (Anthropic)
 * Copyright (C) 2026 Christopher P. Potter (Linux port + IFR extensions)
 *
 * Licensed under the GNU GPL-3.0-or-later. See LICENSE.
 */

#ifndef BACKENDS_SIMBRIEF_CLIENT_HPP
#define BACKENDS_SIMBRIEF_CLIENT_HPP

#include <string>

// Plugin-only. Fetches the latest OFP for a SimBrief pilot ID via HTTPS
// and stores the parsed result in simbrief_ofp::set(). Uses libcurl on a
// detached std::thread — never blocks the X-Plane main thread.
namespace simbrief_client {

// NB: FAILED, not ERROR — ERROR is a <windows.h> macro (#define ERROR 0)
// that mangles a scoped enumerator on the MSVC build.
enum class FetchStatus { IDLE, FETCHING, SUCCESS, FAILED };

// Start an async OFP fetch for the given pilot ID (non-zero).
// No-op if a fetch is already in progress.
void fetch_async(int pilot_id);

FetchStatus status();
std::string last_error(); // non-empty only when status() == FAILED

// Parse a raw SimBrief API JSON response body (the exact bytes the plugin dumps
// to <plugin>/Resources/last_ofp.json) directly into simbrief_ofp, bypassing the
// network fetch. Same parser as the live path (no drift). Used by the headless
// atc_ifr_repl "load_ofp <file>" harness to replay a real flight plan and get
// every sector handoff + clearance. Sets status()/last_error() like a fetch.
void load_ofp_body(const std::string &body);

} // namespace simbrief_client

#endif // BACKENDS_SIMBRIEF_CLIENT_HPP
