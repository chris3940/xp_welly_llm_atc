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

#ifndef PTT_INPUT_HPP
#define PTT_INPUT_HPP

namespace ptt_input {

void init();
void stop();

// Called every flight loop frame — joystick button polling + capture
void update();

// Capture mode for UI binding
enum class CaptureMode { NONE, KEYBOARD, JOYSTICK };

void start_capture(CaptureMode mode);
void cancel_capture();
CaptureMode capture_mode();
float capture_elapsed(); // seconds since capture started (for timeout)

// Returns captured value (VK or button index), -1 if still waiting
// Resets capture mode to NONE after returning a valid result
int poll_capture_result();

} // namespace ptt_input

#endif // PTT_INPUT_HPP
