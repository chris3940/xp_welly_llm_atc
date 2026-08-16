// See mora_db.hpp for the file layout and why this exists. [C. P. Potter]

#include "data/mora_db.hpp"

#include "core/logging.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <sstream>
#include <vector>

namespace mora_db {
namespace {

// 180 latitude bands (-90..+89) x 360 longitude cells (-180..+179), in feet.
// ~65 k ints = 260 kB, small enough to hold flat and index in O(1).
constexpr int kLatBands = 180;
constexpr int kLonCells = 360;

std::vector<int> s_grid; // empty until loaded
bool s_ready = false;
int s_cells = 0;
std::mutex s_mutex;

int index_of(int lat_band, int lon_cell) {
  return lat_band * kLonCells + lon_cell;
}

// Cell indices for a position. Returns false when out of range.
bool cell_for(double lat, double lon, int *out_lat, int *out_lon) {
  if (!(lat >= -90.0 && lat < 90.0))
    return false;
  // Normalise longitude into [-180, 180).
  while (lon >= 180.0)
    lon -= 360.0;
  while (lon < -180.0)
    lon += 360.0;
  *out_lat = static_cast<int>(std::floor(lat)) + 90;
  *out_lon = static_cast<int>(std::floor(lon)) + 180;
  if (*out_lat < 0 || *out_lat >= kLatBands)
    return false;
  if (*out_lon < 0 || *out_lon >= kLonCells)
    return false;
  return true;
}

} // namespace

void init(const std::string &path) {
  {
    std::lock_guard<std::mutex> lk(s_mutex);
    s_grid.clear();
    s_cells = 0;
    s_ready = false;
  }
  if (path.empty()) {
    std::lock_guard<std::mutex> lk(s_mutex);
    s_ready = true; // "loaded, and empty" -- callers get 0 and decide
    return;
  }

  FILE *f = std::fopen(path.c_str(), "r");
  if (f == nullptr) {
    logging::info("mora_db: file not found (%s)", path.c_str());
    std::lock_guard<std::mutex> lk(s_mutex);
    s_ready = true;
    return;
  }

  std::vector<int> grid(static_cast<std::size_t>(kLatBands) * kLonCells, 0);
  int cells = 0;
  char line[1024];
  while (std::fgets(line, sizeof(line), f) != nullptr) {
    std::size_t len = std::strlen(line);
    while (len > 0 && (line[len - 1] == '\n' || line[len - 1] == '\r' ||
                       line[len - 1] == ' '))
      line[--len] = '\0';
    if (len == 0)
      continue;

    std::istringstream iss(line);
    std::string lat_tok, lon_tok;
    if (!(iss >> lat_tok >> lon_tok))
      continue; // header ("I", the version line) and the "99" terminator
    // Both fields are explicitly signed in this file (+47 / -030), which is what
    // distinguishes a data row from the header lines.
    if ((lat_tok[0] != '+' && lat_tok[0] != '-') ||
        (lon_tok[0] != '+' && lon_tok[0] != '-'))
      continue;

    const int lat_deg = std::atoi(lat_tok.c_str());
    const int lon_deg = std::atoi(lon_tok.c_str());
    int lat_band = lat_deg + 90;
    if (lat_band < 0 || lat_band >= kLatBands)
      continue;

    std::string v;
    int col = 0;
    while (iss >> v) {
      const int lon_cell = lon_deg + col + 180;
      ++col;
      if (lon_cell < 0 || lon_cell >= kLonCells)
        continue;
      const int hundreds = std::atoi(v.c_str());
      if (hundreds <= 0)
        continue; // 000 = no figure for this cell; leave it at 0
      grid[static_cast<std::size_t>(index_of(lat_band, lon_cell))] =
          hundreds * 100;
      ++cells;
    }
  }
  std::fclose(f);

  logging::info("mora_db: parsed %d grid cells from %s", cells, path.c_str());
  std::lock_guard<std::mutex> lk(s_mutex);
  s_grid = std::move(grid);
  s_cells = cells;
  s_ready = true;
}

void stop() {
  std::lock_guard<std::mutex> lk(s_mutex);
  s_grid.clear();
  s_cells = 0;
  s_ready = false;
}

bool ready() {
  std::lock_guard<std::mutex> lk(s_mutex);
  return s_ready;
}

int cell_count() {
  std::lock_guard<std::mutex> lk(s_mutex);
  return s_cells;
}

int minimum_ft(double lat, double lon) {
  int la = 0, lo = 0;
  if (!cell_for(lat, lon, &la, &lo))
    return 0;
  std::lock_guard<std::mutex> lk(s_mutex);
  if (s_grid.empty())
    return 0;
  return s_grid[static_cast<std::size_t>(index_of(la, lo))];
}

int minimum_ft_around(double lat, double lon) {
  int la = 0, lo = 0;
  if (!cell_for(lat, lon, &la, &lo))
    return 0;
  std::lock_guard<std::mutex> lk(s_mutex);
  if (s_grid.empty())
    return 0;
  int worst = 0;
  for (int dla = -1; dla <= 1; ++dla) {
    const int a = la + dla;
    if (a < 0 || a >= kLatBands)
      continue;
    for (int dlo = -1; dlo <= 1; ++dlo) {
      // Wrap longitude so a pattern flown across the antimeridian still sees
      // its neighbours rather than silently losing half of them.
      int o = lo + dlo;
      if (o < 0)
        o += kLonCells;
      if (o >= kLonCells)
        o -= kLonCells;
      worst = std::max(worst, s_grid[static_cast<std::size_t>(index_of(a, o))]);
    }
  }
  return worst;
}

} // namespace mora_db
