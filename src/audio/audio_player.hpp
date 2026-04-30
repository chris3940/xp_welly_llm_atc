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
#include <vector>

namespace audio_player {

void init();
void stop();

// Play a short PTT click on the X-Plane radio bus (routes to Radio Device)
void play_ptt_click();

// Play raw int16 PCM samples on the X-Plane radio bus at given volume
// (0.0–1.0). This is the path used for ATC TTS responses (Piper output)
// and for ATIS playback.
void play_pcm(std::vector<int16_t> pcm16, uint32_t sample_rate_hz, int channels,
              float volume);

// Play WAV data on the X-Plane radio bus at given volume (0.0–1.0).
// Kept for the audio-self-test feature in the UI which records mic →
// playback to validate the device chain.
void play_wav(const std::vector<uint8_t> &wav_data, float volume);

// Returns true while audio is being played back
bool is_playing();

} // namespace audio_player

#endif // AUDIO_PLAYER_HPP
