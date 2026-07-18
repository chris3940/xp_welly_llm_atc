/*
 * xp_wellys_atc - AI-powered ATC voice communication for X-Plane 12
 * Copyright (C) 2026 thWelly & Claude (Anthropic)
 * Copyright (C) 2026 Christopher P. Potter (Linux port + IFR extensions)
 *
 * Licensed under the GNU GPL-3.0-or-later. See LICENSE.
 */

#include "data/simbrief_ofp.hpp"

#include <cctype>
#include <mutex>
#include <sstream>

namespace simbrief_ofp {

namespace {
static OfpData g_ofp;
static std::mutex g_mutex;
} // namespace

void set(const OfpData &ofp) {
  std::lock_guard<std::mutex> lk(g_mutex);
  g_ofp = ofp;
}

OfpData get() {
  std::lock_guard<std::mutex> lk(g_mutex);
  return g_ofp;
}

void clear() {
  std::lock_guard<std::mutex> lk(g_mutex);
  g_ofp = {};
}

std::vector<RouteStep> parse_route_steps(const std::string &raw_route) {
  std::vector<RouteStep> out;
  if (raw_route.empty())
    return out;
  std::istringstream rs(raw_route);
  std::string tok;
  while (rs >> tok) {
    auto slash = tok.find('/');
    if (slash == std::string::npos || slash == 0 || slash + 1 >= tok.size())
      continue;
    std::string ident = tok.substr(0, slash);
    // Reject non-alphabetic idents (SID/STAR designators end in digit+letter,
    // pure fix names are 2-5 uppercase letters). Guards against parsing
    // things like "SALEV3P/..." if SimBrief ever emits it.
    if (ident.empty() || ident.size() > 5)
      continue;
    bool ident_ok = true;
    for (char c : ident) {
      if (!std::isalpha(static_cast<unsigned char>(c))) {
        ident_ok = false;
        break;
      }
    }
    if (!ident_ok)
      continue;
    std::string suffix = tok.substr(slash + 1);
    auto f_pos = suffix.find('F');
    if (f_pos == std::string::npos || f_pos + 3 >= suffix.size())
      continue;
    std::string fl_str = suffix.substr(f_pos + 1, 3);
    if (fl_str.size() != 3)
      continue;
    bool all_digits = true;
    for (char c : fl_str) {
      if (!std::isdigit(static_cast<unsigned char>(c))) {
        all_digits = false;
        break;
      }
    }
    if (!all_digits)
      continue;
    RouteStep step;
    step.ident = ident;
    step.cruise_fl = std::stoi(fl_str);
    out.push_back(std::move(step));
  }
  return out;
}

} // namespace simbrief_ofp
