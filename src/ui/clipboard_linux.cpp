/*
 * xp_wellys_atc - AI-powered ATC voice communication for X-Plane 12
 * Copyright (C) 2026 thWelly & Claude (Anthropic)
 *
 * Licensed under the GNU GPL-3.0-or-later. See LICENSE.
 */

#include "ui/clipboard.hpp"

namespace ui::clipboard {

// On Linux the X11 clipboard is not wired up yet.
// Users enter the API key directly via the Settings text field.
std::string read_system_text() { return {}; }

} // namespace ui::clipboard
