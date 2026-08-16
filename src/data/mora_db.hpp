// Grid MORA (Minimum Off-Route Altitude) lookup.
//
// Why: under RADAR VECTORS the aircraft has left the published procedure and so
// has lost its altitude protection. msa_db answers that question around a
// procedure's centre point, but only INSIDE the sector radius (typically 25 NM)
// and only where the airport publishes MSA sectors at all -- outside, it
// correctly returns 0, meaning "I have nothing to say". Grid MORA is the second
// net: worldwide, coarse, always available.
//
// Source file: "<Custom Data>/earth_mora.dat" (note: NOT under "Earth nav data").
// One line per 1-degree latitude band and 30-degree longitude block, carrying 30
// cell values, each covering one 1x1 degree cell, in HUNDREDS of feet:
//
//     +47 +000 010 010 024 045 ...
//      ^   ^    ^
//      |   |    altitude of cell [47..48 N, 0..1 E) = 1000 ft
//      |   longitude of the first cell in the line
//      latitude of the band
//
// A value of 000 means the cell carries no figure (open water, or unsurveyed);
// it is reported as 0, never as "no minimum" -- exactly like msa_db, the caller
// must decide rather than treat 0 as permission to descend.
//
// Coarse by construction: a 1-degree cell is ~60 NM tall, so MORA is a fallback
// for "is this level survivable at all", not a substitute for the MSA around a
// procedure. Prefer msa_db when it answers.
//
// SDK-free: parsed from disk, no X-Plane API. [C. P. Potter]

#pragma once

#include <string>

namespace mora_db {

// Parse earth_mora.dat. Pass an empty path to disable (headless tools without
// Custom Data). Safe to call twice; the second call replaces the first.
void init(const std::string &path);
void stop();

// True once init() has finished parsing.
bool ready();

// Number of cells carrying a figure (diagnostics / tests).
int cell_count();

// Minimum off-route altitude in feet for the 1x1 degree cell containing
// (lat, lon). Returns 0 when the database is not loaded, the position is out of
// range, or the cell carries no figure. 0 means "no value available" and must
// never be read as "no minimum".
int minimum_ft(double lat, double lon);

// Highest MORA among the cell containing (lat, lon) and its eight neighbours.
// A vectoring pattern spans tens of miles, so the cell the aircraft happens to
// sit in is not enough -- the leg can cross into a higher one. Returns 0 only
// when no surrounding cell carries a figure.
int minimum_ft_around(double lat, double lon);

} // namespace mora_db
