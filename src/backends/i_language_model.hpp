/*
 * xp_wellys_atc - AI-powered ATC voice communication for X-Plane 12
 * Copyright (C) 2026 thWelly & Claude (Anthropic)
 *
 * Licensed under the GNU GPL-3.0-or-later. See LICENSE.
 */

#ifndef BACKENDS_I_LANGUAGE_MODEL_HPP
#define BACKENDS_I_LANGUAGE_MODEL_HPP

#include <string>

namespace backends {

class ILanguageModel {
public:
  virtual ~ILanguageModel() = default;

  // Run a single completion with the given system prompt + user message.
  // Implementations must clear any KV / sampler state between calls so a
  // long-running orchestrator can hand them an unbounded sequence of
  // independent turns. Returns the assistant reply (empty on failure).
  virtual std::string respond(const std::string &system_prompt,
                              const std::string &user_text) = 0;
};

} // namespace backends

#endif // BACKENDS_I_LANGUAGE_MODEL_HPP
