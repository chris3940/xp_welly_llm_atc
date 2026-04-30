/*
 * xp_wellys_atc - AI-powered ATC voice communication for X-Plane 12
 * Copyright (C) 2026 thWelly & Claude (Anthropic)
 *
 * Licensed under the GNU GPL-3.0-or-later. See LICENSE.
 */

#ifndef BACKENDS_I_TEXT_TO_SPEECH_HPP
#define BACKENDS_I_TEXT_TO_SPEECH_HPP

#include <cstdint>
#include <string>
#include <vector>

namespace backends {

class ITextToSpeech {
public:
  virtual ~ITextToSpeech() = default;

  // Synthesize `text` to mono 16-bit PCM samples. `sample_rate_hz` is
  // filled with the synthesizer's native sample rate (no resampling).
  // `length_scale` controls speech rate: 1.0 is normal, >1.0 is slower
  // (used for ATIS), <1.0 is faster. An empty result indicates failure.
  virtual std::vector<int16_t> synthesize(const std::string &text,
                                          float length_scale,
                                          uint32_t &sample_rate_hz) = 0;
};

} // namespace backends

#endif // BACKENDS_I_TEXT_TO_SPEECH_HPP
