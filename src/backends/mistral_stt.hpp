/*
 * xp_wellys_atc - AI-powered ATC voice communication for X-Plane 12
 * Copyright (C) 2026 thWelly & Claude (Anthropic)
 *
 * Licensed under the GNU GPL-3.0-or-later. See LICENSE.
 */

#ifndef BACKENDS_MISTRAL_STT_HPP
#define BACKENDS_MISTRAL_STT_HPP

#include "backends/i_speech_to_text.hpp"

#include <string>

namespace backends {

// ISpeechToText backed by Mistral's /v1/audio/transcriptions endpoint
// (Voxtral). Synchronous — backends::manager runs it on a worker
// thread. Every call emits a [STT-MISTRAL] audit log line.
//
// Language is resolved per request from settings::backend_language()
// inside transcribe(), so an ATC-profile switch (EU/US vs. DE) flips
// the Voxtral `language` parameter immediately, without reloading the
// backend.
//
// `airport_context` is the freeform prompt (sent as Voxtral's `prompt`).
// `context_bias` is a COMMA-separated curated list (whole word OR multi-word
// phrases) forwarded as Voxtral's `context_bias[]` array -- split on COMMAS
// only (never whitespace, so phrases stay intact) and capped at 100 entries
// per the API contract. This is the idiomatic biasing path.
class MistralStt final : public ISpeechToText {
public:
  static constexpr const char *kDefaultBaseUrl = "https://api.mistral.ai";

  MistralStt(std::string api_key, std::string model,
             std::string base_url = kDefaultBaseUrl);

  std::string transcribe(const std::vector<float> &pcm_16k_mono,
                         const std::string &airport_context,
                         const std::string &context_bias) override;

  std::string last_error_message() const override { return last_error_; }

private:
  std::string api_key_;
  std::string model_;
  std::string base_url_;
  mutable std::string last_error_;
};

} // namespace backends

#endif // BACKENDS_MISTRAL_STT_HPP
