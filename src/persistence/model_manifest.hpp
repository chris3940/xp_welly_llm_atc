/*
 * xp_wellys_atc - AI-powered ATC voice communication for X-Plane 12
 * Copyright (C) 2026 thWelly & Claude (Anthropic)
 *
 * Licensed under the GNU GPL-3.0-or-later. See LICENSE.
 */

#ifndef PERSISTENCE_MODEL_MANIFEST_HPP
#define PERSISTENCE_MODEL_MANIFEST_HPP

#include <cstdint>
#include <string>
#include <vector>

namespace model_manifest {

enum class Kind {
  WhisperModel,
  LlamaModel,
  PiperVoice,       // .onnx
  PiperVoiceConfig, // .onnx.json
};

// Pinned identity of a single model file the plugin needs at runtime.
// The bundled values were captured during the milestone-05 spike on
// the same source URLs the user will hit; treat them as authoritative
// and do not regenerate without updating the README's manual-fallback
// table.
struct Entry {
  Kind kind;
  std::string filename;     // basename only — lives in models_dir()
  uint64_t size_bytes;      // exact expected size, used as a fast pre-check
  std::string sha256_hex;   // lowercase 64-char hex, post-download verify
  std::string url;          // direct HTTPS GET; HuggingFace is the only source
  std::string display_name; // for the UI status panel
};

// All four entries the plugin requires at startup. The order is the
// loading order — UI rows can simply iterate.
const std::vector<Entry> &all();

// Lookup by kind. Aborts (via std::abort) on unknown kind so callers
// don't silently get a wrong file.
const Entry &get(Kind kind);

// Compute SHA256 of `path`. Returns lowercase 64-char hex on success
// or an empty string if the file cannot be opened. Streams the file in
// chunks so files of any size work; safe to call from a worker thread.
std::string sha256_file(const std::string &path);

// Cheap check: does the file exist with the expected size?
// SHA256 verification is the slow path and should only run after this
// returns true.
bool size_matches(const Entry &e, const std::string &full_path);

} // namespace model_manifest

#endif // PERSISTENCE_MODEL_MANIFEST_HPP
