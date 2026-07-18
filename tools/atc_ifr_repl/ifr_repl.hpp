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
 * Interactive REPL for simulating IFR approach flows end-to-end
 * without X-Plane. Supports poll_approach, route_tracker, fly, goto,
 * training_jump_*, and standard process_transcript (say).
 */

#ifndef IFR_REPL_HPP
#define IFR_REPL_HPP

#include "core/xplane_context.hpp"

#include <string>

namespace ifr_repl {

int run(xplane_context::XPlaneContext ctx, std::string pilot_callsign);

} // namespace ifr_repl

#endif
