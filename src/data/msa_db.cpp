// See msa_db.hpp for the record layout and why this exists. [C. P. Potter]

#include "data/msa_db.hpp"

#include "core/logging.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <mutex>
#include <sstream>
#include <unordered_map>

namespace msa_db {
namespace {

std::unordered_map<std::string, std::vector<Record>> s_by_icao;
bool s_ready = false;
std::mutex s_mutex;

// Great-circle bearing, degrees true, from (la1,lo1) to (la2,lo2).
double bearing_deg(double la1, double lo1, double la2, double lo2) {
  const double d2r = M_PI / 180.0;
  const double dlon = (lo2 - lo1) * d2r;
  const double y = std::sin(dlon) * std::cos(la2 * d2r);
  const double x = std::cos(la1 * d2r) * std::sin(la2 * d2r) -
                   std::sin(la1 * d2r) * std::cos(la2 * d2r) * std::cos(dlon);
  double b = std::atan2(y, x) / d2r;
  return std::fmod(b + 360.0, 360.0);
}

double distance_nm(double la1, double lo1, double la2, double lo2) {
  const double d2r = M_PI / 180.0;
  const double dlat = (la2 - la1) * 60.0;
  const double dlon = (lo2 - lo1) * 60.0 * std::cos((la1 + la2) * 0.5 * d2r);
  return std::sqrt(dlat * dlat + dlon * dlon);
}

} // namespace

void init(std::string path) {
  {
    std::lock_guard<std::mutex> lk(s_mutex);
    s_by_icao.clear();
    s_ready = false;
  }
  if (path.empty()) {
    std::lock_guard<std::mutex> lk(s_mutex);
    s_ready = true; // "loaded, and empty" -- callers get 0 and decide
    return;
  }

  FILE *f = std::fopen(path.c_str(), "r");
  if (!f) {
    logging::info("msa_db: file not found (%s)", path.c_str());
    std::lock_guard<std::mutex> lk(s_mutex);
    s_ready = true;
    return;
  }

  std::unordered_map<std::string, std::vector<Record>> out;
  char line[512];
  int records = 0;
  while (std::fgets(line, sizeof(line), f)) {
    // The file is CRLF; strip both, and any trailing blanks.
    std::size_t len = std::strlen(line);
    while (len > 0 && (line[len - 1] == '\n' || line[len - 1] == '\r' ||
                       line[len - 1] == ' '))
      line[--len] = '\0';
    if (len == 0)
      continue;

    std::istringstream iss(line);
    std::string seq, centre, region, icao, ref;
    if (!(iss >> seq >> centre >> region >> icao >> ref))
      continue; // header lines ("I", the version line) fall out here
    if (ref != "M" && ref != "T")
      continue; // not an MSA record
    if (icao.size() < 3)
      continue;

    Record r;
    r.centre_ident = centre;
    r.icao = icao;
    r.magnetic = (ref == "M");

    // Triplets: bearing, altitude (hundreds of feet), radius NM. A "000 000"
    // pair terminates the list -- note it is a PAIR, so a zero bearing with a
    // real altitude (a sector starting due north, e.g. LOWI's "000 131 25") is
    // NOT a terminator and must survive.
    std::string a, b, c;
    while (iss >> a >> b) {
      const int brg = std::atoi(a.c_str());
      const int alt = std::atoi(b.c_str());
      if (brg == 0 && alt == 0)
        break; // terminator
      if (!(iss >> c))
        break; // malformed tail -- keep what we have
      Sector s;
      s.start_bearing_deg = brg;
      s.altitude_ft = alt * 100;
      s.radius_nm = std::atoi(c.c_str());
      r.sectors.push_back(s);
    }
    if (r.sectors.empty())
      continue;
    out[icao].push_back(std::move(r));
    ++records;
  }
  std::fclose(f);

  logging::info("msa_db: parsed %d MSA records for %d airports from %s", records,
                static_cast<int>(out.size()), path.c_str());
  std::lock_guard<std::mutex> lk(s_mutex);
  s_by_icao = std::move(out);
  s_ready = true;
}

void stop() {
  std::lock_guard<std::mutex> lk(s_mutex);
  s_by_icao.clear();
  s_ready = false;
}

bool ready() {
  std::lock_guard<std::mutex> lk(s_mutex);
  return s_ready;
}

std::vector<Record> records_for(const std::string &icao) {
  std::lock_guard<std::mutex> lk(s_mutex);
  auto it = s_by_icao.find(icao);
  return (it == s_by_icao.end()) ? std::vector<Record>{} : it->second;
}

int minimum_ft(const std::string &icao, const std::string &centre_ident,
               double centre_lat, double centre_lon, double lat, double lon,
               double magvar_deg) {
  const auto recs = records_for(icao);
  if (recs.empty())
    return 0;

  const Record *rec = nullptr;
  for (const auto &r : recs)
    if (r.centre_ident == centre_ident) {
      rec = &r;
      break;
    }
  if (rec == nullptr || rec->sectors.empty())
    return 0;

  const double d_nm = distance_nm(centre_lat, centre_lon, lat, lon);
  // Outside every sector's radius the MSA says nothing at all. Returning 0 (and
  // NOT the largest sector value) keeps the caller honest: it must fall back to
  // grid MORA or refuse to descend, rather than silently reuse a figure that
  // does not apply here.
  int max_radius = 0;
  for (const auto &s : rec->sectors)
    max_radius = std::max(max_radius, s.radius_nm);
  if (d_nm > static_cast<double>(max_radius))
    return 0;

  double brg = bearing_deg(centre_lat, centre_lon, lat, lon);
  if (rec->magnetic)
    brg = std::fmod(brg - magvar_deg + 360.0, 360.0);

  // A sector runs CLOCKWISE from its start bearing to the next start. With one
  // sector it covers the whole circle. Sectors are not guaranteed sorted in the
  // file, so sort a local copy by start bearing first.
  std::vector<Sector> ss = rec->sectors;
  std::sort(ss.begin(), ss.end(), [](const Sector &a, const Sector &b) {
    return a.start_bearing_deg < b.start_bearing_deg;
  });
  if (ss.size() == 1)
    return d_nm <= static_cast<double>(ss[0].radius_nm) ? ss[0].altitude_ft : 0;

  for (std::size_t i = 0; i < ss.size(); ++i) {
    const double from = ss[i].start_bearing_deg;
    const double to = ss[(i + 1) % ss.size()].start_bearing_deg;
    const bool inside = (from < to) ? (brg >= from && brg < to)
                                    : (brg >= from || brg < to); // wraps 360
    if (inside)
      return d_nm <= static_cast<double>(ss[i].radius_nm) ? ss[i].altitude_ft : 0;
  }
  return 0;
}

} // namespace msa_db
