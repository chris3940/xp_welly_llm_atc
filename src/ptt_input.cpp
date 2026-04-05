#include "ptt_input.hpp"
#include "atc_session.hpp"
#include "settings.hpp"

#include <XPLMDataAccess.h>
#include <XPLMDisplay.h>
#include <XPLMProcessing.h>
#include <XPLMUtilities.h>

#include <cstdio>
#include <cstring>
#include <vector>

namespace ptt_input {

// ── X-Plane Command (existing approach) ────���────────────────────

static XPLMCommandRef ptt_cmd_ = nullptr;

static int ptt_command_handler(XPLMCommandRef, XPLMCommandPhase phase, void *) {
  if (phase == xplm_CommandBegin) {
    atc_session::on_ptt_pressed();
  } else if (phase == xplm_CommandEnd) {
    atc_session::on_ptt_released();
  }
  return 0;
}

// ─��� Keyboard VK PTT ─────────────────────────────────────────────

static bool key_is_down_ = false;

// ── Capture mode ────────────────────────────────────────────────

static CaptureMode capture_mode_ = CaptureMode::NONE;
static float capture_start_time_ = 0.0f;
static int capture_result_ = -1;
static constexpr float kCaptureTimeout = 10.0f;

// ── Joystick state ──────────────────────────────────────────────

static XPLMDataRef joy_buttons_dr_ = nullptr;
static std::vector<uint8_t> joy_prev_;
static std::vector<uint8_t> joy_curr_;
static constexpr int kMaxJoyButtons = 3200;
static bool joy_ptt_down_ = false;

// ── Key sniffer callback ────────────────────────────────────────

static int key_sniffer_cb(char, XPLMKeyFlags flags, char vkey, void *) {
  bool is_down = (flags & xplm_DownFlag) != 0;
  bool is_up = (flags & xplm_UpFlag) != 0;

  // Capture mode: keyboard
  if (capture_mode_ == CaptureMode::KEYBOARD && is_down) {
    capture_result_ = static_cast<int>(static_cast<unsigned char>(vkey));
    return 0; // consume
  }

  // Normal PTT via VK
  int bound_vk = settings::ptt_key_vk();
  if (bound_vk < 0)
    return 1; // pass through

  int pressed_vk = static_cast<int>(static_cast<unsigned char>(vkey));
  if (pressed_vk != bound_vk)
    return 1; // not our key

  if (is_down && !key_is_down_) {
    key_is_down_ = true;
    atc_session::on_ptt_pressed();
    return 0;
  }
  if (is_up && key_is_down_) {
    key_is_down_ = false;
    atc_session::on_ptt_released();
    return 0;
  }

  return 0; // consume our bound key
}

// ── Lifecycle ───────────────────────────────────────────────────

void init() {
  // X-Plane command PTT (users can bind via X-Plane settings)
  ptt_cmd_ =
      XPLMCreateCommand("xp_wellys_atc/ptt", "Welly's ATC: Push-to-Talk");
  XPLMRegisterCommandHandler(ptt_cmd_, ptt_command_handler, 1, nullptr);

  // Key sniffer for direct VK binding
  XPLMRegisterKeySniffer(key_sniffer_cb, 1 /* before */, nullptr);

  // Joystick DataRef
  joy_buttons_dr_ = XPLMFindDataRef("sim/joystick/joy_buttons");
  joy_prev_.resize(kMaxJoyButtons, 0);
  joy_curr_.resize(kMaxJoyButtons, 0);

  key_is_down_ = false;
  joy_ptt_down_ = false;
  capture_mode_ = CaptureMode::NONE;
  capture_result_ = -1;

  XPLMDebugString(
      "[xp_wellys_atc] PTT input initialized (command + key + joystick)\n");
}

void stop() {
  if (ptt_cmd_) {
    XPLMUnregisterCommandHandler(ptt_cmd_, ptt_command_handler, 1, nullptr);
    ptt_cmd_ = nullptr;
  }
  XPLMUnregisterKeySniffer(key_sniffer_cb, 1, nullptr);
  joy_buttons_dr_ = nullptr;
}

// ── Flight loop update ──────────────────────────────────────────

void update() {
  // Read current joystick button state
  if (joy_buttons_dr_) {
    int count =
        XPLMGetDatab(joy_buttons_dr_, joy_curr_.data(), 0, kMaxJoyButtons);
    if (count < kMaxJoyButtons)
      std::memset(joy_curr_.data() + count, 0, kMaxJoyButtons - count);
  }

  // Capture mode: joystick — detect first button 0→1
  if (capture_mode_ == CaptureMode::JOYSTICK) {
    for (int i = 0; i < kMaxJoyButtons; ++i) {
      if (joy_curr_[i] != 0 && joy_prev_[i] == 0) {
        capture_result_ = i;
        break;
      }
    }
  }

  // Capture timeout
  if (capture_mode_ != CaptureMode::NONE) {
    float now = XPLMGetElapsedTime();
    if (now - capture_start_time_ > kCaptureTimeout) {
      capture_mode_ = CaptureMode::NONE;
      capture_result_ = -1;
    }
  }

  // Normal joystick PTT
  if (capture_mode_ == CaptureMode::NONE) {
    int bound_btn = settings::ptt_joystick_button();
    if (bound_btn >= 0 && bound_btn < kMaxJoyButtons) {
      bool now_down = (joy_curr_[bound_btn] != 0);
      if (now_down && !joy_ptt_down_) {
        joy_ptt_down_ = true;
        atc_session::on_ptt_pressed();
      } else if (!now_down && joy_ptt_down_) {
        joy_ptt_down_ = false;
        atc_session::on_ptt_released();
      }
    }
  }

  // Save current state as previous for next frame
  std::swap(joy_prev_, joy_curr_);
}

// ── Capture API ──────────────────────────────────��──────────────

void start_capture(CaptureMode mode) {
  capture_mode_ = mode;
  capture_result_ = -1;
  capture_start_time_ = XPLMGetElapsedTime();
}

void cancel_capture() {
  capture_mode_ = CaptureMode::NONE;
  capture_result_ = -1;
}

CaptureMode capture_mode() { return capture_mode_; }

float capture_elapsed() {
  if (capture_mode_ == CaptureMode::NONE)
    return 0.0f;
  return XPLMGetElapsedTime() - capture_start_time_;
}

int poll_capture_result() {
  if (capture_result_ >= 0) {
    int result = capture_result_;
    capture_mode_ = CaptureMode::NONE;
    capture_result_ = -1;
    return result;
  }
  return -1;
}

} // namespace ptt_input
