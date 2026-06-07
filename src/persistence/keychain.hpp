/*
 * xp_wellys_atc - AI-powered ATC voice communication for X-Plane 12
 * Copyright (C) 2026 thWelly & Claude (Anthropic)
 *
 * Licensed under the GNU GPL-3.0-or-later. See LICENSE.
 */

#ifndef PERSISTENCE_KEYCHAIN_HPP
#define PERSISTENCE_KEYCHAIN_HPP

#include <string>

namespace persistence::keychain {

// Secure credential store for API keys.
// macOS: wraps SecKeychain. Linux: stores in ~/.config/xp_wellys_atc/<service>.key (mode 0600).
// The single-argument overloads target the OpenAI service; use the
// explicit (service, account, key) overloads for other providers.
// load() returns "" when no entry exists — callers must treat "" as "no key".

// OpenAI convenience overloads ("com.xp_wellys_atc.openai" / "default").
bool save(const std::string &api_key);
std::string load();
bool remove();
bool has();

// Mistral convenience overloads ("com.xp_wellys_atc.mistral" / "default").
bool save_mistral(const std::string &api_key);
std::string load_mistral();
bool remove_mistral();
bool has_mistral();

// Generic overloads — service + account are the key into the store.
bool save(const std::string &service, const std::string &account,
          const std::string &api_key);
std::string load(const std::string &service, const std::string &account);
bool remove(const std::string &service, const std::string &account);
bool has(const std::string &service, const std::string &account);

} // namespace persistence::keychain

#endif // PERSISTENCE_KEYCHAIN_HPP
