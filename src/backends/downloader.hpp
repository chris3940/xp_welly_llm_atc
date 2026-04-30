/*
 * xp_wellys_atc - AI-powered ATC voice communication for X-Plane 12
 * Copyright (C) 2026 thWelly & Claude (Anthropic)
 *
 * Licensed under the GNU GPL-3.0-or-later. See LICENSE.
 */

#ifndef BACKENDS_DOWNLOADER_HPP
#define BACKENDS_DOWNLOADER_HPP

#include "persistence/model_manifest.hpp"

#include <cstdint>
#include <string>
#include <vector>

namespace backends::downloader {

enum class State {
  Idle,             // no download active for this kind
  Queued,           // waiting for the worker to pick it up
  Downloading,      // libcurl is streaming to <file>.part
  Verifying,        // SHA256 in progress on the freshly written file
  Done,             // .part renamed to final filename, loader notified
  Failed,           // see error_message; .part may still exist for resume
  Cancelled,        // user cancelled; .part kept for later resume
  InsufficientDisk, // pre-flight disk-space check rejected the download
};

struct Progress {
  model_manifest::Kind kind;
  State state = State::Idle;
  // Total expected size and bytes already on disk (.part + already-
  // resumed). The UI feeds these directly into a progress bar.
  uint64_t bytes_total = 0;
  uint64_t bytes_downloaded = 0;
  std::string error_message;
};

// Snapshot of every manifest entry's download state — same length and
// order as model_manifest::all(). Safe to call from the main thread
// every frame.
std::vector<Progress> snapshot();

// Free space in bytes at <plugin>/Resources/models/. Used by the UI
// to warn the user before they kick off a 2.3 GB download. Returns 0
// on stat failure.
uint64_t free_space_bytes();

// Sum of (entry.size_bytes - bytes_already_present) across every
// manifest entry that is not yet Verified. Tells the user how much
// the "Download all missing" button will pull.
uint64_t bytes_still_required();

// Queue a single download. If the file is already present + size-
// matched, this is a no-op (state goes Done immediately). If a
// download for that kind is already queued or in flight, also no-op.
void enqueue(model_manifest::Kind kind);

// Queue every manifest entry that is currently Missing /
// SizeMismatch / HashMismatch in `backends::loader`. Triggered by
// the "Download all missing" button.
size_t enqueue_all_missing();

// Cancel a single in-flight or queued download. The .part file is
// preserved on disk so a future enqueue() can resume from where the
// transfer stopped.
void cancel(model_manifest::Kind kind);

// Cancel everything + join the worker thread. Called from
// XPluginDisable. Bounded wait (~5 s) so a wedged libcurl cannot
// block plugin unload.
void stop();

} // namespace backends::downloader

#endif // BACKENDS_DOWNLOADER_HPP
