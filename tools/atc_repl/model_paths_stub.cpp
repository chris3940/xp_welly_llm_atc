// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Christopher P. Potter
//
// model_paths stub for the headless REPL tools. The real model_paths resolves
// <plugin>/Resources/... via XPLMGetPluginInfo (plugin-only). The REPL links
// simbrief_client.cpp for the "load_ofp" harness; simbrief_client references
// model_paths::plugin_root() only inside the LIVE fetch path (do_fetch), which
// the REPL never calls -- but the symbol must still link. These stubs provide it.

#include "persistence/model_paths.hpp"

namespace model_paths {

const std::string &plugin_root() {
  static const std::string p = ".";
  return p;
}
const std::string &models_dir() {
  static const std::string p = "./Resources/models";
  return p;
}
const std::string &espeakng_data_dir() {
  static const std::string p = ".";
  return p;
}

} // namespace model_paths
