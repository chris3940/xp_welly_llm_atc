/*
 * xp_wellys_atc - AI-powered ATC voice communication for X-Plane 12
 * Copyright (C) 2026 thWelly & Claude (Anthropic)
 *
 * Licensed under the GNU GPL-3.0-or-later. See LICENSE.
 */

#ifndef BACKENDS_WHISPER_STT_HPP
#define BACKENDS_WHISPER_STT_HPP

#include "backends/i_speech_to_text.hpp"

#include <string>

struct whisper_context;

namespace backends {

// Concrete ISpeechToText backed by whisper.cpp + Metal. open() loads
// the GGUF model once; transcribe() reuses the same context.
class WhisperStt final : public ISpeechToText {
public:
  WhisperStt();
  ~WhisperStt() override;

  WhisperStt(const WhisperStt &) = delete;
  WhisperStt &operator=(const WhisperStt &) = delete;

  // Returns false on load failure.
  bool open(const std::string &model_path);

  std::string transcribe(const std::vector<float> &pcm_16k_mono,
                         const std::string &airport_context) override;

private:
  whisper_context *ctx_ = nullptr;
  int n_threads_ = 0;
};

} // namespace backends

#endif // BACKENDS_WHISPER_STT_HPP
