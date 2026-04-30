/*
 * xp_wellys_atc - AI-powered ATC voice communication for X-Plane 12
 * Copyright (C) 2026 thWelly & Claude (Anthropic)
 *
 * Licensed under the GNU GPL-3.0-or-later. See LICENSE.
 */

#ifndef BACKENDS_PIPER_TTS_HPP
#define BACKENDS_PIPER_TTS_HPP

#include "backends/i_text_to_speech.hpp"

#include <string>

struct piper_synthesizer;

namespace backends {

// Concrete ITextToSpeech backed by Piper. open() loads the ONNX voice
// + JSON config + espeak-ng-data directory once; synthesize() reuses
// the synthesizer.
class PiperTts final : public ITextToSpeech {
public:
  PiperTts();
  ~PiperTts() override;

  PiperTts(const PiperTts &) = delete;
  PiperTts &operator=(const PiperTts &) = delete;

  bool open(const std::string &voice_onnx_path,
            const std::string &voice_json_path,
            const std::string &espeakng_data_dir);

  std::vector<int16_t> synthesize(const std::string &text, float length_scale,
                                  uint32_t &sample_rate_hz) override;

private:
  piper_synthesizer *synth_ = nullptr;
};

} // namespace backends

#endif // BACKENDS_PIPER_TTS_HPP
