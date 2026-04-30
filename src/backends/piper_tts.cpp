/*
 * xp_wellys_atc - AI-powered ATC voice communication for X-Plane 12
 * Copyright (C) 2026 thWelly & Claude (Anthropic)
 *
 * Licensed under the GNU GPL-3.0-or-later. See LICENSE.
 *
 * Lifted from spikes/spike_e2e/src/piper_tts.cpp; the spike validated
 * the chunked synthesis loop. Adds `length_scale` plumbing so ATIS can
 * speak slower than tower/ground.
 */

#include "backends/piper_tts.hpp"

#include "piper.h"

#include <cstdint>

namespace backends {

namespace {

int16_t f32_to_i16(float x) {
  if (x > 1.0f)
    x = 1.0f;
  if (x < -1.0f)
    x = -1.0f;
  return static_cast<int16_t>(x * 32767.0f);
}

} // namespace

PiperTts::PiperTts() = default;

PiperTts::~PiperTts() {
  if (synth_)
    piper_free(synth_);
}

bool PiperTts::open(const std::string &voice_onnx_path,
                    const std::string &voice_json_path,
                    const std::string &espeakng_data_dir) {
  synth_ = piper_create(voice_onnx_path.c_str(), voice_json_path.c_str(),
                        espeakng_data_dir.c_str());
  return synth_ != nullptr;
}

std::vector<int16_t> PiperTts::synthesize(const std::string &text,
                                          float length_scale,
                                          uint32_t &sample_rate_hz) {
  sample_rate_hz = 0;
  if (!synth_ || text.empty())
    return {};

  piper_synthesize_options opts = piper_default_synthesize_options(synth_);
  // Piper's `length_scale` directly maps speech rate. Our public
  // contract uses the same convention (>1.0 = slower). Clamp to a
  // sensible band so a stray caller cannot wedge the synthesizer.
  if (length_scale < 0.5f)
    length_scale = 0.5f;
  if (length_scale > 2.0f)
    length_scale = 2.0f;
  opts.length_scale = length_scale;

  if (piper_synthesize_start(synth_, text.c_str(), &opts) != PIPER_OK) {
    return {};
  }

  std::vector<int16_t> pcm;
  pcm.reserve(text.size() * 1024);

  piper_audio_chunk chunk{};
  int rc = 0;
  while ((rc = piper_synthesize_next(synth_, &chunk)) != PIPER_DONE) {
    if (rc != PIPER_OK)
      return {};
    if (sample_rate_hz == 0)
      sample_rate_hz = static_cast<uint32_t>(chunk.sample_rate);
    const float *s = chunk.samples;
    for (size_t i = 0; i < chunk.num_samples; ++i) {
      pcm.push_back(f32_to_i16(s[i]));
    }
  }
  // The terminating PIPER_DONE chunk may still carry samples.
  if (chunk.num_samples > 0) {
    if (sample_rate_hz == 0)
      sample_rate_hz = static_cast<uint32_t>(chunk.sample_rate);
    const float *s = chunk.samples;
    for (size_t i = 0; i < chunk.num_samples; ++i) {
      pcm.push_back(f32_to_i16(s[i]));
    }
  }
  return pcm;
}

} // namespace backends
