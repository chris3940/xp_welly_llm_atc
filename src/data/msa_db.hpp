// Minimum Sector Altitude (MSA) lookup.
//
// Why: under RADAR VECTORS the aircraft has left the published procedure and so
// has lost its altitude protection -- the controller owes it a level at or above
// the minimum safe altitude for where it actually is. The published fix altitude
// is NOT enough, and LOWI shows why with its own numbers: the MSA around the IAF
// ELMEM is 11 400 ft in the 220 sector but 14 300 ft in the 270 sector, while
// ELMEM's published altitude is 13 000 ft. A vector swinging the aircraft west
// while descending it to the published figure puts it 1 300 ft BELOW the minimum
// for that sector, in an Alpine valley.
//
// Source file: "<Custom Data>/earth_msa.dat" (note: NOT under "Earth nav data").
// One record per centre point, sectors given clockwise by their START bearing:
//
//     11 ELMEM LO LOWI M 220 114 25  270 143 25  000 131 25  095 107 25  000 000  0
//      ^ ^     ^  ^    ^ \_________/
//      | |     |  |    |  bearing / altitude in HUNDREDS of feet / radius NM
//      | |     |  |    M = bearings are MAGNETIC
//      | |     |  airport ICAO
//      | |     region
//      | centre point ident (a fix, a navaid, or the airport itself)
//      sequence
//
// A sector runs from its start bearing CLOCKWISE to the next sector's start, and
// the list is terminated by a "000 000" pair. Bearings are FROM the centre point
// TO the aircraft.
//
// Coverage on the reference data: LOWI 3 records, LFMN 4, EDLW 2, LFLP 2 --
// centred both on the field and on individual IAFs.
//
// SDK-free: parsed from disk, no X-Plane API. [C. P. Potter]

#pragma once

#include <string>
#include <vector>

namespace msa_db {

// One sector of one MSA record.
struct Sector {
  int start_bearing_deg = 0; // magnetic, sector runs clockwise from here
  int altitude_ft = 0;       // already converted from hundreds of feet
  int radius_nm = 0;
};

// All sectors around one centre point.
struct Record {
  std::string centre_ident; // fix / navaid / airport the sectors are drawn around
  std::string icao;         // airport the record belongs to
  bool magnetic = true;     // "M" in the file; "T" would be true bearings
  std::vector<Sector> sectors;
};

// Parse earth_msa.dat. Pass an empty path to disable (headless tools without
// Custom Data). Safe to call twice; the second call replaces the first.
void init(std::string path);
void stop();

// True once init() has finished parsing.
bool ready();

// Every record belonging to an airport, in file order. Empty when the airport is
// absent or the database is disabled.
std::vector<Record> records_for(const std::string &icao);

// Minimum safe altitude in feet for an aircraft at (lat, lon) relative to a
// centre point at (centre_lat, centre_lon), using that centre's record at the
// given airport. Returns 0 when:
//   - the database is not loaded, or the airport/centre has no record;
//   - the aircraft is OUTSIDE the sector radius (MSA says nothing there).
// Never guesses: 0 means "no protection value available", and the caller must
// decide what to do rather than treat it as "no minimum".
//
// magvar_deg is (true - magnetic) at the aircraft, needed because the file's
// bearings are magnetic; pass 0 to treat them as true.
int minimum_ft(const std::string &icao, const std::string &centre_ident,
               double centre_lat, double centre_lon, double lat, double lon,
               double magvar_deg = 0.0);

} // namespace msa_db
