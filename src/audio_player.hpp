/*
 * xp_wellys_atc - AI-powered ATC voice communication for X-Plane 12
 * Copyright (C) 2026 thWelly & Claude (Anthropic)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef AUDIO_PLAYER_HPP
#define AUDIO_PLAYER_HPP

#include <cstdint>
#include <string>
#include <vector>

namespace audio_player {

struct AudioDevice {
  std::string uid;  // Core Audio UID (empty = system default)
  std::string name; // Human-readable name
};

void init();
void stop();

// Enumerate available output devices (first entry is always "System Default")
const std::vector<AudioDevice> &get_output_devices();

// Refresh device list (call if user plugs in new device)
void refresh_devices();

// Play a short PTT click sound on the selected device
void play_ptt_click();

// Play MP3 data through speakers at given volume (0.0–1.0)
void play(const std::vector<uint8_t> &mp3_data, float volume);

// Returns true while audio is being played back
bool is_playing();

} // namespace audio_player

#endif // AUDIO_PLAYER_HPP
