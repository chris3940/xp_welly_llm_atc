/*
 * xp_wellys_atc - AI-powered ATC voice communication for X-Plane 12
 * Copyright (C) 2026 thWelly & Claude (Anthropic)
 * Copyright (C) 2026 Christopher P. Potter (Linux port + IFR extensions)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

#ifndef OPENAIR_DB_HPP
#define OPENAIR_DB_HPP

#include <cstdint>
#include <string>
#include <vector>

namespace openair_db {

// Airspace class parsed from the OpenAir "AC" record.
enum class AirspaceClass {
  CTR,   // Control Zone          — AC CTR
  TMA,   // Terminal Maneuvering  — AC TMA
  CTA,   // Control Area          — AC CTA
  FIR,   // Flight Info Region    — AC FIR
  UIR,   // Upper Info Region     — AC UIR
  OTHER, // everything else (R, P, Q, D, E, F, G …)
};

// Result of find_enclosing(). When the position is outside all indexed
// airspaces, ac_class == OTHER and name is empty.
struct AirspaceEntry {
  std::string name;
  AirspaceClass ac_class = AirspaceClass::OTHER;
  int floor_ft = 0;
  int ceiling_ft = 0;
  std::uint32_t freq_khz = 0; // openair "AF" field (overlay-carried controller freq)
};

// Parse CTR / TMA / CTA / FIR / UIR entries from an OpenAir-format airspace
// file (e.g. X-Plane "Custom Data/airspaces/airspace.txt").
// Pass an empty base path to disable (headless tools, no Custom Data).
//
// overlay_path (optional): a second OpenAir file (e.g.
// "<plugin>/Resources/airspace+.txt") whose entries are APPENDED after the
// base, and WIN on a same-name collision -- for hand-maintained polygons that
// are missing or too coarse in the vendor file (sub-CTAs, cross-border
// delegation such as LFFF->LSAS). Empty/absent overlay = base only.
void init(std::string path, std::string overlay_path = {});
void stop();

// Returns true once init() has finished (success or file-absent).
bool ready();

// Returns the innermost (smallest bounding-box area) airspace that contains
// (lat, lon, alt_ft) in 3-D: 2-D point-in-polygon AND floor_ft <= alt_ft
// <= ceiling_ft.  Returns an OTHER entry when the position is outside all
// indexed airspaces.
AirspaceEntry find_enclosing(double lat, double lon, int alt_ft);

// Returns ALL airspaces that contain (lat, lon, alt_ft) in 3-D.
// Use this to check whether the aircraft is inside ANY CTR/TMA regardless
// of nesting — a large background CTR (e.g. MARSEILLE) must not mask a
// CTA that is the real innermost zone.
std::vector<AirspaceEntry> find_all_enclosing(double lat, double lon,
                                               int alt_ft);

// Backward-compat wrapper: returns ceiling of the CTR at (lat, lon)
// ignoring altitude. Returns 0 if not inside any CTR.
int ctr_ceiling_ft(double lat, double lon);

// Returns the ceiling of the BASE (lowest-floor) TMA-class airspace whose
// polygon contains (lat, lon), IGNORING altitude (2-D lateral test only) --
// i.e. the top of the terminal control area sitting directly on the field, not
// any higher TMA stacked above it. Over LFMN this is NICE TMA (11500); over
// LFLP/Annecy it is CHAMBERY TMA (9500), correctly ignoring the GENEVA TMA
// (9500-19500) stacked on top. Ties on floor break to the higher ceiling.
// Returns 0 when the point is over no TMA (AFIS-only field, or a field whose
// terminal area is a plain CTR). Geometry-based, so a field controlled by a
// differently-named unit (LFLP under CHAMBERY) resolves correctly. Used by the
// "descend to enter the terminal area" clearance -- an aircraft cleared above a
// low-ceilinged terminal TMA can never enter it otherwise.
int terminal_tma_ceiling(double lat, double lon);

// Highest TMA ceiling over the point: the MAX ceiling across every TMA block
// (stacked sub-volumes) whose polygon contains (lat, lon). Unlike
// terminal_tma_ceiling (which returns the lowest-floor block's ceiling, for
// descend-to-ENTER), this returns the true TOP of the TMA stack -- used so an
// enroute descent stays ABOVE a tall overflown TMA (LOWI/DOLSKO tops FL245 ->
// stay at/above FL250) instead of diving into a mid-level sub-block. 0 = none.
int highest_tma_ceiling(double lat, double lon);

// "Descend to enter" test. Returns the destination's base terminal-TMA ceiling
// (terminal_tma_ceiling at the destination) IF the aircraft at (acft_lat,
// acft_lon) is laterally inside a TMA whose ceiling matches that reference --
// i.e. the aircraft is over the destination's terminal area -- else 0. Ignores
// altitude (2-D). This is the correct trigger for the descend-to-enter
// clearance: an aircraft over a large TMA can be well beyond a fixed
// field-distance radius (NICE TMA extends 40+ NM), so "over the TMA" must be a
// polygon test, not a distance gate. Matching by the base ceiling keeps a
// stacked overlying TMA (GENEVA over CHAMBERY) from hijacking it.
int descend_to_enter_ceiling(double acft_lat, double acft_lon, double dest_lat,
                             double dest_lon);

} // namespace openair_db

#endif // OPENAIR_DB_HPP
