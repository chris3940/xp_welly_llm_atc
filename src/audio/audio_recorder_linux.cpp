/*
 * xp_wellys_atc - AI-powered ATC voice communication for X-Plane 12
 * Copyright (C) 2026 thWelly & Claude (Anthropic)
 *
 * Licensed under the GNU GPL-3.0-or-later. See LICENSE.
 *
 * Linux PulseAudio mic capture — #include'd inside audio_recorder.cpp
 * inside the #elif defined(__linux__) block. Runs inside
 * namespace audio_recorder and has access to the static vars declared
 * above (buffer_, buffer_mutex_, recording_, initialized_,
 * actual_sample_rate_).
 */

static pa_simple *pa_stream_ = nullptr;
static std::thread capture_thread_;
static std::atomic<bool> capture_running_{false};

// 512 frames = 32 ms at 16 kHz. Short enough that stop() returns
// promptly after setting capture_running_ = false.
static constexpr size_t kCaptureFrames = 512;

static void capture_loop() {
  int16_t chunk[kCaptureFrames];
  constexpr int kMaxConsecutiveErrors = 10;
  int consecutive_errors = 0;
  while (capture_running_.load()) {
    int error = 0;
    if (pa_simple_read(pa_stream_, chunk, sizeof(chunk), &error) < 0) {
      if (!capture_running_.load())
        break;
      ++consecutive_errors;
      char log[160];
      std::snprintf(log, sizeof(log),
                    "[xp_wellys_atc] PulseAudio read error (%d/%d): %s\n",
                    consecutive_errors, kMaxConsecutiveErrors,
                    pa_strerror(error));
      XPLMDebugString(log);
      if (consecutive_errors >= kMaxConsecutiveErrors) {
        XPLMDebugString("[xp_wellys_atc] PulseAudio: stream lost, "
                        "stopping capture. Restart X-Plane to recover.\n");
        pa_simple_free(pa_stream_);
        pa_stream_  = nullptr;
        initialized_ = false;
        capture_running_ = false;
        break;
      }
      continue;
    }
    consecutive_errors = 0;
    if (recording_.load()) {
      std::lock_guard<std::mutex> lock(buffer_mutex_);
      buffer_.insert(buffer_.end(), chunk, chunk + kCaptureFrames);
    }
  }
}

void init() {
  static const pa_sample_spec ss = {PA_SAMPLE_S16LE, kDesiredSampleRate,
                                    kNumChannels};
  int error = 0;
  pa_stream_ =
      pa_simple_new(nullptr, "xp_wellys_atc", PA_STREAM_RECORD, nullptr, "mic",
                    &ss, nullptr, nullptr, &error);
  if (!pa_stream_) {
    char log[128];
    std::snprintf(log, sizeof(log),
                  "[xp_wellys_atc] PulseAudio init failed: %s\n",
                  pa_strerror(error));
    XPLMDebugString(log);
    return;
  }

  actual_sample_rate_ = kDesiredSampleRate;
  initialized_ = true;
  capture_running_ = true;
  capture_thread_ = std::thread(capture_loop);

  XPLMDebugString(
      "[xp_wellys_atc] Audio recorder initialized (PulseAudio 16kHz mono "
      "16-bit)\n");
}

void stop() {
  capture_running_ = false;
  if (capture_thread_.joinable())
    capture_thread_.join();
  if (pa_stream_) {
    pa_simple_free(pa_stream_);
    pa_stream_ = nullptr;
  }
  recording_ = false;
  initialized_ = false;
  std::lock_guard<std::mutex> lock(buffer_mutex_);
  buffer_.clear();
}

void start_recording() {
  if (!initialized_ || !pa_stream_) {
    XPLMDebugString(
        "[xp_wellys_atc] Warning: audio recorder not initialized\n");
    return;
  }
  {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    buffer_.clear();
  }
  recording_ = true;
}

void stop_recording() {
  recording_ = false;
  std::lock_guard<std::mutex> lock(buffer_mutex_);
  if (settings::debug_logging()) {
    int16_t peak = 0;
    for (auto s : buffer_) {
      int16_t abs_s = s < 0 ? static_cast<int16_t>(-s) : s;
      if (abs_s > peak)
        peak = abs_s;
    }
    float peak_pct = (static_cast<float>(peak) / 32767.0f) * 100.0f;
    char log[256];
    std::snprintf(
        log, sizeof(log),
        "[xp_wellys_atc][DEBUG] Recording stopped: %zu samples, peak: %d "
        "(%.1f%%)\n",
        buffer_.size(), static_cast<int>(peak), peak_pct);
    XPLMDebugString(log);
  }
}
