SHELL := /bin/bash

# ── Platform detection ────────────────────────────────────────────────────────
OS := $(shell uname -s)

# XPLANE_ROOT can be overridden: `make install XPLANE_ROOT=/path/to/xplane`
ifeq ($(OS),Darwin)
    XPLANE_ROOT ?= /Users/robertw/X-Plane 12
    PLUGIN_ARCH_DIR := mac_x64
    PLUGIN_DIR := $(XPLANE_ROOT)/Resources/available plugins/xp_wellys_atc
else
    XPLANE_ROOT ?= $(HOME)/X-Plane 12
    PLUGIN_ARCH_DIR := lin_x64
    PLUGIN_DIR := $(XPLANE_ROOT)/Resources/plugins/xp_wellys_atc
endif

SDK_SENTINEL    := sdk/XPLM/XPLMPlugin.h
IMGUI_SENTINEL  := vendor/imgui/imgui.h
JSON_SENTINEL   := vendor/json.hpp
CATCH2_SENTINEL := vendor/catch2/catch_amalgamated.hpp

# SkunkCrafts Updater staging dir (under build/, already gitignored).
SKUNK_DIR := build/skunkcrafts

# One sentinel for the three submodule trees (whisper.cpp, llama.cpp,
# Piper). They are all pulled in by a single
# `git submodule update --init --recursive` invocation, so tracking
# only the first one is sufficient — if it's missing, the whole
# submodule init runs and lands all three.
SUBMODULES_SENTINEL := spikes/spike_whisper/third_party/whisper.cpp/CMakeLists.txt

CATCH2_VERSION := 3.7.1

# Sources fed to clang-tidy. Mirrors the per-platform source selection in
# CMakeLists.txt (if(APPLE) / elseif(UNIX AND NOT APPLE) blocks around
# audio_input_*, mic_permission_*, clipboard_*) so we never lint a TU that
# wouldn't even compile on the current host. Without this the macOS lint
# run trips over pulse/error.h (PulseAudio headers absent), and the Linux
# lint run would trip over CoreAudio headers.
# Windows-only TUs (WASAPI audio, Win32 clipboard) include <windows.h> and
# never compile off Windows — exclude on both macOS and Linux lint runs.
LINT_EXCLUDE_WIN := src/audio/audio_input_wasapi.cpp src/ui/clipboard_win.cpp
ifeq ($(OS),Darwin)
LINT_EXCLUDE := $(LINT_EXCLUDE_WIN) src/audio/audio_input_pulseaudio.cpp src/audio/mic_permission_linux.cpp src/ui/clipboard_linux.cpp
else
LINT_EXCLUDE := $(LINT_EXCLUDE_WIN) src/audio/audio_input_coreaudio.cpp
endif
LINT_SOURCES := $(filter-out $(LINT_EXCLUDE),$(wildcard src/main.cpp src/*/*.cpp))

.PHONY: all help setup setup-cloud build install install-mac install-linux install-data package clean distclean format lint sanitize release release-build cleanup-tags cleanup-branches cleanup-runs cleanup-cache repl run-repl ifr-repl run-ifr-repl replay replay-star replay-lsgg test test-unit test-scenarios test-afis test-stars ci-remote win-artifact skunkcrafts

.DEFAULT_GOAL := help

all: clean format build lint test

# ── Help ──────────────────────────────────────────────────────────────────────
help:
	@echo "xp_wellys_atc - Makefile targets"
	@echo ""
	@echo "  make                   Show this help (default)"
	@echo "  make all               clean + format + build + lint"
	@echo "  make setup             Init submodules + download X-Plane SDK, Dear ImGui, nlohmann/json, Catch2"
	@echo "  make setup-cloud       Setup WITHOUT local-inference submodules (cloud-only; used by CI)"
	@echo "  make build             Build universal plugin (arm64 local+cloud, x86_64 cloud-only) -> build/xp_wellys_atc.xpl"
	@echo "  make repl              Build headless CLI -> build/atc_repl"
	@echo "  make run-repl          Build + run the CLI (stdin transcripts)"
	@echo "  make ifr-repl          Build IFR test CLI -> build/atc_ifr_repl"
	@echo "  make run-ifr-repl      Build + run the IFR test CLI"
	@echo "  make test              Run unit tests + scenario tests"
	@echo "  make test-unit         Build + run Catch2 unit tests"
	@echo "  make test-scenarios    Build + run all scenario tests in testscripts/"
	@echo "  make install           Code-sign and install plugin to X-Plane"
	@echo "  make format            Run clang-format on src/*.cpp src/*.hpp"
	@echo "  make lint              Run clang-tidy on src/*.cpp"
	@echo "  make sanitize          Build atc_repl + tests with ASan+UBSan and run them"
	@echo "  make release VERSION=X Tag and push release (writes VERSION.txt)"
	@echo "  make release-build     Build plugin with RELEASE=ON (embeds VERSION.txt)"
	@echo "  make skunkcrafts       Stage a SkunkCrafts Updater release tree from the installed plugin"
	@echo "  make cleanup-tags      Prune local tags no longer on origin"
	@echo "  make cleanup-branches  Prune local branches whose remote is gone"
	@echo "  make cleanup-runs      Delete all GitHub Actions runs except the newest per workflow"
	@echo "  make cleanup-cache     Delete all GitHub Actions caches (freed on next CI run)"
	@echo "  make ci-remote         Trigger the GitHub CI (mac + Windows slice) on the current branch via gh (builds the PUSHED state)"
	@echo "  make win-artifact      Download the newest Windows CI artifact (xp_wellys_atc-win) via gh -> dist-win/"
	@echo "  make clean             Remove build/, build-lint/ and build-sanitize/"
	@echo "  make distclean         clean + remove sdk/ and vendor/ (everything 'make setup' installed)"
	@echo "  make help              Show this help"

# ── Setup ─────────────────────────────────────────────────────────────────────
setup: $(SUBMODULES_SENTINEL) $(SDK_SENTINEL) $(IMGUI_SENTINEL) $(JSON_SENTINEL) $(CATCH2_SENTINEL)
	@echo "Setup complete. Run 'make build' to compile."

# Cloud-only setup: SDK + ImGui + json + Catch2, WITHOUT the local-inference
# submodules (whisper.cpp / llama.cpp / Piper). Used by CI, which builds
# cloud-only and never compiles those trees — skipping the multi-GB submodule
# fetch is the bulk of the CI speedup.
setup-cloud: $(SDK_SENTINEL) $(IMGUI_SENTINEL) $(JSON_SENTINEL) $(CATCH2_SENTINEL)
	@echo "Cloud-only setup complete (no local-inference submodules)."

$(SUBMODULES_SENTINEL):
	@if [ ! -d .git ]; then \
	    echo "ERROR: not a git checkout - submodules cannot be initialised."; \
	    echo ""; \
	    echo "If you downloaded a release ZIP, the third-party sources"; \
	    echo "(whisper.cpp, llama.cpp, Piper) are not bundled. Re-clone with:"; \
	    echo ""; \
	    echo "    git clone --recurse-submodules <repo-url>"; \
	    echo ""; \
	    exit 1; \
	fi
	@echo "Initialising git submodules (whisper.cpp, llama.cpp, Piper)..."
	@git submodule update --init --recursive
	@echo "Submodules ready."

$(SDK_SENTINEL):
	@echo "Downloading X-Plane SDK..."
	@set -euo pipefail; \
	TMP=$$(mktemp -d); \
	trap "rm -rf $$TMP" EXIT; \
	curl -fsSL "https://developer.x-plane.com/wp-content/plugins/code-sample-generation/sdk_zip_files/XPSDK430.zip" \
	     -o "$$TMP/sdk.zip"; \
	unzip -q "$$TMP/sdk.zip" -d "$$TMP/sdk_extracted"; \
	mkdir -p sdk/XPLM sdk/XPWidgets sdk/Libraries/Win sdk/Libraries/Mac sdk/Libraries/Lin; \
	find "$$TMP/sdk_extracted" -path "*/CHeaders/XPLM/*.h"    -exec cp {} sdk/XPLM/ \;; \
	find "$$TMP/sdk_extracted" -path "*/CHeaders/Widgets/*.h"  -exec cp {} sdk/XPWidgets/ \;; \
	find "$$TMP/sdk_extracted" -path "*/Libraries/Win/*.lib"   -exec cp {} sdk/Libraries/Win/ \;; \
	find "$$TMP/sdk_extracted" -path "*/Libraries/Lin/*.so"    -exec cp {} sdk/Libraries/Lin/ \;; \
	cp -R "$$TMP/sdk_extracted"/*/Libraries/Mac/*.framework sdk/Libraries/Mac/ 2>/dev/null || \
	find "$$TMP/sdk_extracted" -name "*.framework" -exec cp -R {} sdk/Libraries/Mac/ \;
	@echo "SDK headers installed."

$(IMGUI_SENTINEL):
	@echo "Downloading Dear ImGui v1.91.9..."
	@set -euo pipefail; \
	TMP=$$(mktemp -d); \
	trap "rm -rf $$TMP" EXIT; \
	mkdir -p vendor/imgui/backends; \
	curl -fsSL "https://github.com/ocornut/imgui/archive/refs/tags/v1.91.9.zip" -o "$$TMP/imgui.zip"; \
	unzip -q "$$TMP/imgui.zip" -d "$$TMP/"; \
	SRC="$$TMP/imgui-1.91.9"; \
	cp "$$SRC"/imgui.{h,cpp} vendor/imgui/; \
	cp "$$SRC"/imgui_{draw,tables,widgets}.cpp vendor/imgui/; \
	cp "$$SRC"/imgui_internal.h "$$SRC"/imconfig.h vendor/imgui/; \
	cp "$$SRC"/imstb_textedit.h "$$SRC"/imstb_rectpack.h "$$SRC"/imstb_truetype.h vendor/imgui/ 2>/dev/null || true; \
	cp "$$SRC"/backends/imgui_impl_opengl2.{h,cpp} vendor/imgui/backends/
	@echo "Dear ImGui installed."

$(JSON_SENTINEL):
	@echo "Downloading nlohmann/json v3.11.3..."
	@mkdir -p vendor
	@curl -fsSL "https://github.com/nlohmann/json/releases/download/v3.11.3/json.hpp" \
	     -o vendor/json.hpp
	@echo "nlohmann/json installed."

$(CATCH2_SENTINEL):
	@echo "Downloading Catch2 v$(CATCH2_VERSION) (amalgamated)..."
	@set -euo pipefail; \
	TMP=$$(mktemp -d); \
	trap "rm -rf $$TMP" EXIT; \
	mkdir -p vendor/catch2; \
	curl -fsSL "https://github.com/catchorg/Catch2/archive/refs/tags/v$(CATCH2_VERSION).tar.gz" \
	     -o "$$TMP/catch2.tar.gz"; \
	tar -xzf "$$TMP/catch2.tar.gz" -C "$$TMP/"; \
	cp "$$TMP/Catch2-$(CATCH2_VERSION)/extras/catch_amalgamated.hpp" vendor/catch2/; \
	cp "$$TMP/Catch2-$(CATCH2_VERSION)/extras/catch_amalgamated.cpp" vendor/catch2/
	@echo "Catch2 installed."

# ── Build ─────────────────────────────────────────────────────────────────────
# Always produces a universal xp_wellys_atc.xpl that contains both an
# arm64 slice (whisper.cpp + llama.cpp + Piper + OpenAI) and an x86_64
# slice (OpenAI only — Metal + the onnxruntime prebuilt are Apple
# Silicon only). The two slices share src/ and CMakeLists.txt but
# differ via -DXPWELLYS_USE_LOCAL_INFERENCE.
#
# Strategy: two separate CMake configures (build-arm64/, build-x86_64/),
# each producing its own .xpl, then lipo-merged into build/xp_wellys_atc.xpl.
# The arm64 slice ships libpiper.dylib + libonnxruntime.dylib next to the
# .xpl so they're picked up by @loader_path; the x86_64 slice has no
# such dylibs to ship.
#
# `make install` copies build/xp_wellys_atc.xpl into the plugin dir
# together with the staged dylibs.
# `RELEASE_FLAG` is passed through to both CMake configure calls so the
# GitHub Actions workflow can flip `-DRELEASE=ON` for tag-driven
# release builds without duplicating the logic. Empty by default
# (regular dev build); `release-build` sets it to `-DRELEASE=ON`.
RELEASE_FLAG ?=

build: $(SUBMODULES_SENTINEL) $(SDK_SENTINEL) $(IMGUI_SENTINEL) $(JSON_SENTINEL) $(CATCH2_SENTINEL)
	@touch src/main.cpp   # refresh the __DATE__/__TIME__ build stamp logged at startup
ifeq ($(OS),Darwin)
	@echo "=== Building universal xp_wellys_atc (arm64 local+cloud, x86_64 cloud-only) ==="
	@echo ""
	@echo "--- arm64 slice (local + cloud) ---"
	cmake -B build-arm64 -DCMAKE_BUILD_TYPE=Release $(RELEASE_FLAG) \
	    -DCMAKE_OSX_ARCHITECTURES=arm64 \
	    -DXPWELLYS_USE_LOCAL_INFERENCE=ON \
	    -DBUILD_TESTS=OFF \
	    -Wno-dev
	cmake --build build-arm64 --parallel
	@echo ""
	@echo "--- x86_64 slice (cloud-only) ---"
	cmake -B build-x86_64 -DCMAKE_BUILD_TYPE=Release $(RELEASE_FLAG) \
	    -DCMAKE_OSX_ARCHITECTURES=x86_64 \
	    -DXPWELLYS_USE_LOCAL_INFERENCE=OFF \
	    -DBUILD_TESTS=OFF \
	    -Wno-dev
	cmake --build build-x86_64 --parallel
	@echo ""
	@echo "--- lipo merge ---"
	@mkdir -p build
	lipo -create \
	    build-arm64/xp_wellys_atc.xpl \
	    build-x86_64/xp_wellys_atc.xpl \
	    -output build/xp_wellys_atc.xpl
	@cp build-arm64/libpiper.dylib              build/
	@cp build-arm64/libonnxruntime.1.22.0.dylib build/
	@cp build-arm64/libonnxruntime.dylib        build/
	@cp -R build-arm64/espeak_ng-install        build/ 2>/dev/null || true
	@echo ""
	@file build/xp_wellys_atc.xpl
	@lipo -info build/xp_wellys_atc.xpl
	@echo "Done. Run 'make install' to deploy the universal .xpl."
else
	@echo "=== Building xp_wellys_atc for Linux ==="
	cmake -B build -DCMAKE_BUILD_TYPE=Release $(RELEASE_FLAG) \
	    -DXPWELLYS_USE_LOCAL_INFERENCE=ON \
	    -DBUILD_TESTS=OFF \
	    -Wno-dev
	cmake --build build --parallel
	@echo ""
	@file build/xp_wellys_atc.xpl
	@echo "Done. Run 'make install' to deploy."
endif

# ── REPL (headless CLI) ───────────────────────────────────────────────────────
repl: $(SDK_SENTINEL) $(IMGUI_SENTINEL) $(JSON_SENTINEL) $(CATCH2_SENTINEL)
	@echo "=== Building atc_repl ==="
	# SDK-free dev tool — LOCAL_INFERENCE=OFF keeps it submodule-independent.
	cmake -B build -DCMAKE_BUILD_TYPE=Release \
	    -DXPWELLYS_USE_LOCAL_INFERENCE=OFF -Wno-dev
	cmake --build build --target atc_repl --parallel
	@echo ""
	@file build/atc_repl
	@echo "Done. Run 'make run-repl' or './build/atc_repl'."

run-repl: repl
	./build/atc_repl

# ── IFR REPL (headless IFR approach test CLI) ─────────────────────────────────
ifr-repl: $(SDK_SENTINEL) $(IMGUI_SENTINEL) $(JSON_SENTINEL) $(CATCH2_SENTINEL)
	@echo "=== Building atc_ifr_repl ==="
	# SDK-free dev tool — LOCAL_INFERENCE=OFF keeps it submodule-independent.
	cmake -B build -DCMAKE_BUILD_TYPE=Release \
	    -DXPWELLYS_USE_LOCAL_INFERENCE=OFF -Wno-dev
	cmake --build build --target atc_ifr_repl --parallel
	@echo ""
	@file build/atc_ifr_repl
	@echo "Done. Run 'make run-ifr-repl' or './build/atc_ifr_repl'."

run-ifr-repl: ifr-repl
	./build/atc_ifr_repl

# ── Tests ─────────────────────────────────────────────────────────────────────
test: test-unit test-scenarios

test-unit: $(SDK_SENTINEL) $(IMGUI_SENTINEL) $(JSON_SENTINEL) $(CATCH2_SENTINEL)
	@echo "=== Building xp_wellys_atc unit tests ==="
	# Tests exercise the SDK-free engine only; the local backends are never
	# linked, so LOCAL_INFERENCE=OFF is functionally identical here and keeps
	# the configure independent of the whisper/llama/Piper submodules.
	cmake -B build -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=ON \
	    -DXPWELLYS_USE_LOCAL_INFERENCE=OFF -Wno-dev
	cmake --build build --target xp_wellys_atc_tests --parallel
	@echo ""
	@echo "=== Running unit tests ==="
	@./build/xp_wellys_atc_tests

test-scenarios: repl
	@echo "=== Running scenario tests ==="
	# Non-recursive glob: testscripts/experimental_ifr/ is intentionally
	# excluded — those IFR scenarios are quarantined (see that dir's README).
	./build/atc_repl run testscripts/*.json

# Quarantined IFR departure scenarios — currently failing against the merged
# IFR feature code (PR #11). Not part of `make test`. See
# testscripts/experimental_ifr/README.md for the known root causes.
test-scenarios-ifr: repl
	@echo "=== Running quarantined IFR scenario tests ==="
	./build/atc_repl run testscripts/experimental_ifr/*.json

# Real-data AFIS scenario harness (LFLU->LFLP): drives atc_ifr_repl against the
# REAL Custom Data (airport+.json / airspace.txt+overlay / atc.dat / CIFP) and asserts
# the AFIS departure + STAR-shortcut outputs. NOT part of `make test` -- it needs the
# user's local X-Plane data, so it is a local-validation target, not CI.
test-stars: ifr-repl
	@echo "=== Running STAR tracker regression (real CIFP) ==="
	@./testscripts/ifr_real/star_tracker_regression.sh

test-afis: ifr-repl
	@echo "=== Running AFIS real-data scenario (LFLU->LFLP) ==="
	@./testscripts/ifr_real/afis_lflu_lflp.sh

# ── Install ───────────────────────────────────────────────────────────────────
# `install` is a thin dispatcher that routes to the per-platform target.
# `install-mac` and `install-linux` handle binary placement + platform-
# specific post-processing (codesign / rpath on macOS, .so copies on
# Linux), then chain into `install-data` for the platform-independent
# data/* and Resources/* bundle.
install:
ifeq ($(OS),Darwin)
	@$(MAKE) --no-print-directory install-mac
else
	@$(MAKE) --no-print-directory install-linux
endif

install-mac:
	@if [ ! -f "build/xp_wellys_atc.xpl" ]; then \
	    echo "Plugin not built yet. Run 'make build' first."; exit 1; \
	fi
	@echo "=== Installing xp_wellys_atc (macOS) ==="
	@mkdir -p "$(PLUGIN_DIR)/$(PLUGIN_ARCH_DIR)"
	@cp build/xp_wellys_atc.xpl "$(PLUGIN_DIR)/$(PLUGIN_ARCH_DIR)/"
	@cp build/libpiper.dylib              "$(PLUGIN_DIR)/$(PLUGIN_ARCH_DIR)/"
	@cp build/libonnxruntime.1.22.0.dylib "$(PLUGIN_DIR)/$(PLUGIN_ARCH_DIR)/"
	@cp build/libonnxruntime.dylib        "$(PLUGIN_DIR)/$(PLUGIN_ARCH_DIR)/"
	@xattr -dr com.apple.quarantine "$(PLUGIN_DIR)/$(PLUGIN_ARCH_DIR)/xp_wellys_atc.xpl"           2>/dev/null || true
	@xattr -dr com.apple.quarantine "$(PLUGIN_DIR)/$(PLUGIN_ARCH_DIR)/libpiper.dylib"              2>/dev/null || true
	@xattr -dr com.apple.quarantine "$(PLUGIN_DIR)/$(PLUGIN_ARCH_DIR)/libonnxruntime.1.22.0.dylib" 2>/dev/null || true
	@for rp in $$(otool -l "$(PLUGIN_DIR)/$(PLUGIN_ARCH_DIR)/xp_wellys_atc.xpl" \
	    | awk '/LC_RPATH/{flag=1; next} flag && /path/ {print $$2; flag=0}'); do \
	    install_name_tool -delete_rpath "$$rp" "$(PLUGIN_DIR)/$(PLUGIN_ARCH_DIR)/xp_wellys_atc.xpl" 2>/dev/null || true; \
	done
	@install_name_tool -add_rpath "@loader_path" "$(PLUGIN_DIR)/$(PLUGIN_ARCH_DIR)/xp_wellys_atc.xpl"
	@codesign --force --deep --sign - "$(PLUGIN_DIR)/$(PLUGIN_ARCH_DIR)/libonnxruntime.1.22.0.dylib"
	@codesign --force --deep --sign - "$(PLUGIN_DIR)/$(PLUGIN_ARCH_DIR)/libpiper.dylib"
	@codesign --force --deep --sign - "$(PLUGIN_DIR)/$(PLUGIN_ARCH_DIR)/xp_wellys_atc.xpl"
	@$(MAKE) --no-print-directory install-data

install-linux:
	@if [ ! -f "build/xp_wellys_atc.xpl" ]; then \
	    echo "Plugin not built yet. Run 'make build' first."; exit 1; \
	fi
	@echo "=== Installing xp_wellys_atc (Linux) ==="
	@mkdir -p "$(PLUGIN_DIR)/$(PLUGIN_ARCH_DIR)"
	@cp build/xp_wellys_atc.xpl "$(PLUGIN_DIR)/$(PLUGIN_ARCH_DIR)/"
	@if [ -f "build/libpiper.so" ]; then \
	    cp build/libpiper.so "$(PLUGIN_DIR)/$(PLUGIN_ARCH_DIR)/"; \
	fi
	@if [ -f "build/libonnxruntime.so.1.22.0" ]; then \
	    cp build/libonnxruntime.so.1.22.0 "$(PLUGIN_DIR)/$(PLUGIN_ARCH_DIR)/"; \
	    cp build/libonnxruntime.so        "$(PLUGIN_DIR)/$(PLUGIN_ARCH_DIR)/"; \
	    ln -sf libonnxruntime.so.1.22.0   "$(PLUGIN_DIR)/$(PLUGIN_ARCH_DIR)/libonnxruntime.so.1"; \
	fi
	@if [ -f "build/libonnxruntime_providers_shared.so" ]; then \
	    cp build/libonnxruntime_providers_shared.so "$(PLUGIN_DIR)/$(PLUGIN_ARCH_DIR)/"; \
	fi
	@$(MAKE) --no-print-directory install-data

install-data:
	@# Bundle espeak-ng-data (~19 MB) inside the plugin so Piper's
	@# phonemizer finds its dictionary at runtime via the plugin-relative
	@# path resolved by model_paths::espeakng_data_dir(). Models live in
	@# Resources/models/ and are downloaded by the user on first launch
	@# (P5); espeak-ng-data is part of the .xpl bundle, NOT downloaded.
	@if [ -d "build/espeak_ng-install/share/espeak-ng-data" ]; then \
	    mkdir -p "$(PLUGIN_DIR)/Resources/espeak-ng-data"; \
	    rsync -a --delete \
	        "build/espeak_ng-install/share/espeak-ng-data/" \
	        "$(PLUGIN_DIR)/Resources/espeak-ng-data/"; \
	    echo "Installed: $(PLUGIN_DIR)/Resources/espeak-ng-data/"; \
	else \
	    echo "WARNING: build/espeak_ng-install/share/espeak-ng-data missing — run make build first"; \
	fi
	@# Models live under Resources/models/. Created empty here so the
	@# in-plugin downloader has a target dir on first launch even
	@# before the user has downloaded anything.
	@mkdir -p "$(PLUGIN_DIR)/Resources/models"
	@# Hand-maintained overlays (additional airspace + per-airport overrides).
	@# Copied when present; repo is the source of truth so they overwrite.
	@if [ -f Resources/airspace+.txt ]; then \
	    cp "Resources/airspace+.txt" "$(PLUGIN_DIR)/Resources/"; \
	    echo "Installed: $(PLUGIN_DIR)/Resources/airspace+.txt"; \
	fi
	@if [ -f Resources/airport+.json ]; then \
	    cp "Resources/airport+.json" "$(PLUGIN_DIR)/Resources/"; \
	    echo "Installed: $(PLUGIN_DIR)/Resources/airport+.json"; \
	fi
	@mkdir -p "$(PLUGIN_DIR)/data"
	@if [ ! -f "$(PLUGIN_DIR)/data/settings.json" ]; then \
	    cp data/settings.json "$(PLUGIN_DIR)/data/"; \
	    echo "Installed: $(PLUGIN_DIR)/data/settings.json"; \
	else \
	    echo "Kept existing settings.json"; \
	fi
	@cp data/atc_prompt_templates.json "$(PLUGIN_DIR)/data/"
	@echo "Installed: $(PLUGIN_DIR)/data/atc_prompt_templates.json"
	@# Always overwrite the models catalog — slugs and Piper voice
	@# hashes are baked into the bundled JSON, and a stale user copy
	@# (e.g. carried over from a pre-catalog install) would silently
	@# point the loader at outdated entries. Per-user customization
	@# happens via Settings, not by hand-editing this file.
	@cp data/models_catalog.json "$(PLUGIN_DIR)/data/"
	@echo "Installed: $(PLUGIN_DIR)/data/models_catalog.json"
	@mkdir -p "$(PLUGIN_DIR)/data/atc_profiles/eu/vfr" \
	          "$(PLUGIN_DIR)/data/atc_profiles/eu/ifr" \
	          "$(PLUGIN_DIR)/data/atc_profiles/us"
	@cp data/atc_profiles/eu/vfr/atc_templates.json  "$(PLUGIN_DIR)/data/atc_profiles/eu/vfr/"
	@cp data/atc_profiles/eu/vfr/flight_rules.json   "$(PLUGIN_DIR)/data/atc_profiles/eu/vfr/"
	@cp data/atc_profiles/eu/ifr/atc_templates.json  "$(PLUGIN_DIR)/data/atc_profiles/eu/ifr/"
	@cp data/atc_profiles/eu/ifr/flight_rules.json   "$(PLUGIN_DIR)/data/atc_profiles/eu/ifr/"
	@cp data/atc_profiles/eu/intent_rules.json       "$(PLUGIN_DIR)/data/atc_profiles/eu/"
	@cp data/atc_profiles/eu/phraseology_hints.json  "$(PLUGIN_DIR)/data/atc_profiles/eu/"
	@cp data/atc_profiles/eu/ui_strings.json         "$(PLUGIN_DIR)/data/atc_profiles/eu/"
	@echo "Installed: $(PLUGIN_DIR)/data/atc_profiles/eu/ (vfr/ + ifr/ + shared)"
	@cp data/atc_profiles/us/atc_templates.json     "$(PLUGIN_DIR)/data/atc_profiles/us/"
	@cp data/atc_profiles/us/flight_rules.json      "$(PLUGIN_DIR)/data/atc_profiles/us/"
	@cp data/atc_profiles/us/intent_rules.json      "$(PLUGIN_DIR)/data/atc_profiles/us/"
	@cp data/atc_profiles/us/phraseology_hints.json "$(PLUGIN_DIR)/data/atc_profiles/us/"
	@cp data/atc_profiles/us/ui_strings.json        "$(PLUGIN_DIR)/data/atc_profiles/us/"
	@echo "Installed: $(PLUGIN_DIR)/data/atc_profiles/us/*.json"
	@mkdir -p "$(PLUGIN_DIR)/data/vrps"
	@cp data/vrps/airport_vrps.json "$(PLUGIN_DIR)/data/vrps/"
	@echo "Installed: $(PLUGIN_DIR)/data/vrps/airport_vrps.json"
	@# Cleanup of legacy paths from pre-Phase-B installs (region-scoped
	@# data + top-level JSONs from the era before the regions folder).
	@rm -f "$(PLUGIN_DIR)/data/atc_templates.json" \
	       "$(PLUGIN_DIR)/data/flight_rules.json" \
	       "$(PLUGIN_DIR)/data/airport_vrps.json"
	@rm -rf "$(PLUGIN_DIR)/data/regions"
	@echo "Installed and signed."

# ── Package ───────────────────────────────────────────────────────────────────
# Produces a self-contained ZIP ready to drop into X-Plane's
# Resources/plugins/ directory.  On Linux the archive is
# xp_wellys_atc-linux-<version>.zip; on macOS xp_wellys_atc-mac-<version>.zip.
# Usage: make package   (VERSION read from VERSION.txt if not overridden)
DIST_VERSION := $(shell cat VERSION.txt 2>/dev/null | tr -d '[:space:]')
ifeq ($(DIST_VERSION),)
    DIST_VERSION := dev
endif
DIST_GIT_HASH := $(shell git rev-parse --short HEAD 2>/dev/null)
ifeq ($(OS),Darwin)
    DIST_PLATFORM := mac
    DIST_EXT      := zip
else
    DIST_PLATFORM := linux
    DIST_EXT      := tar.gz
endif
# Build counter: monotonic across ALL packages for this platform+version
# (any git hash), computed as max(existing trailing -N) + 1 — NOT a file
# count. This survives manual renames (e.g. -23) and new commits: the next
# build after -23 is always -24, never a reset to -2.
_DIST_BASE    := xp_wellys_atc-$(DIST_PLATFORM)-$(DIST_VERSION)-$(DIST_GIT_HASH)
DIST_BUILD_N  := $(shell m=$$(for f in dist/xp_wellys_atc-$(DIST_PLATFORM)-$(DIST_VERSION)-*.$(DIST_EXT); do [ -e "$$f" ] || continue; b=$${f%.$(DIST_EXT)}; echo $${b##*-}; done | sort -n | tail -1); echo $$(( $${m:-0} + 1 )))
DIST_NAME     := $(_DIST_BASE)-$(DIST_BUILD_N)
DIST_STAGE    := dist/$(DIST_NAME)/xp_wellys_atc

# Closed-loop replay of the DIK -> EDLW vectored arrival: the REPL flies it end
# to end with a driver that obeys ATC, so an arrival change is judged on what the
# ENGINE says, not on a reimplementation of its formulas in a scratch script.
#
# THE GATE: run this before packaging any change to the arrival, the descent or
# the vectoring. On 2026-08-17 six vectoring changes were packaged and flown
# without it; the replay found the defects in one run afterwards -- the last
# vector assigning the axis course instead of an intercept, and "established"
# declared on heading alone. Both were visible in this output. [C. P. Potter]
# The SAME route flown as a PUBLISHED PROCEDURE: no vectors, the pilot flies the
# cleared route read back from the engine (`fmsroute`) exactly as an FMS would --
# flight-plan fixes, then the cleared STAR, then the approach transition. A direct
# to an IAF rewrites that route and the pilot follows it, so this one target covers
# the full STAR and the shortcut alike. Until it existed, every defect of the
# non-vectored arrival could only be found by flying it. [C. P. Potter]
replay-star: ifr-repl
	@echo "=== Replay: DIK -> EDLW, PUBLISHED procedure (no vectors), ILS ==="
	@ATC_FMS=1 XP_ATC_FORCE_ILS=1 XP_ATC_HOLD_PCT=0 \
	    ATC_RAW=build/replay-star-raw.log \
	    python3 testscripts/ifr_real/fly.py testscripts/ifr_real/route_dik_edlw.json
	@echo
	@echo "--- route the pilot flew (build/replay-star-raw.log) ---"
	@grep -h "\[fms\]" build/replay-star-raw.log || true

# LSGG BELU3R (spoken "BELUS THREE ROMEO"), runway 22, arriving from the
# south-west so the axis is reached from the LEFT. Kept as a test case because
# the STAR ENDS IN A VECTORING TERMINATION -- its last leg at GG512 is a CIFP
# 'FM', course from fix to manual termination -- and because LSGG sits just
# inside the terrain gate: MSA 7000 over a field at 1411 gives 5589 ft, 411 ft
# under the 6000 ft threshold. [C. P. Potter]
replay-lsgg: ifr-repl
	@echo "=== Replay: BELU3R -> LSGG 22, forced vectoring, ILS ==="
	@ATC_FMS=1 XP_ATC_FORCE_ILS=1 XP_ATC_FORCE_VECTORING=1 XP_ATC_HOLD_PCT=0 \
	    ATC_RAW=build/replay-lsgg-raw.log \
	    python3 testscripts/ifr_real/fly.py testscripts/ifr_real/route_belus_lsgg.json
	@echo
	@echo "--- vectoring decision (build/replay-lsgg-raw.log) ---"
	@grep -h "\[vector\]" build/replay-lsgg-raw.log | grep -v "turn word" || true

replay: ifr-repl
	@echo "=== Replay: DIK -> EDLW, forced vectoring, ILS ==="
	@XP_ATC_FORCE_ILS=1 XP_ATC_FORCE_VECTORING=1 \
	    ATC_RAW=build/replay-raw.log \
	    python3 testscripts/ifr_real/fly.py testscripts/ifr_real/route_dik_edlw.json
	@echo
	@echo "--- vectoring trace (build/replay-raw.log) ---"
	@grep -h "\[vector\]" build/replay-raw.log | grep -v "turn word" || true

# DEPENDS ON build ON PURPOSE. This target used to only CHECK that
# build/xp_wellys_atc.xpl existed and then copy it -- and `make replay` builds
# the engine and the REPL but never the .xpl. On 2026-08-18 that shipped package
# 89 with the previous day's plugin: the user flew an hour-long test of code
# that was not in the binary, and every defect he reported had already been
# fixed. A stale copy passes the existence check silently, which is the worst
# possible failure mode for a packaging step. [C. P. Potter]
package: build
	@if [ ! -f "build/xp_wellys_atc.xpl" ]; then \
	    echo "Plugin not built. Run 'make build' first."; exit 1; \
	fi
	@echo "=== Packaging xp_wellys_atc $(DIST_VERSION) for $(DIST_PLATFORM) ==="
	@rm -rf dist/$(DIST_NAME)
	@mkdir -p "$(DIST_STAGE)/$(PLUGIN_ARCH_DIR)"
	@# ── Binaries ──
	@cp build/xp_wellys_atc.xpl "$(DIST_STAGE)/$(PLUGIN_ARCH_DIR)/"
ifeq ($(OS),Darwin)
	@[ -f build-arm64/libpiper.dylib ]             && cp build-arm64/libpiper.dylib "$(DIST_STAGE)/$(PLUGIN_ARCH_DIR)/" || true
	@[ -f build-arm64/libonnxruntime.1.22.0.dylib ]&& cp build-arm64/libonnxruntime.1.22.0.dylib "$(DIST_STAGE)/$(PLUGIN_ARCH_DIR)/" || true
	@[ -f build-arm64/libonnxruntime.dylib ]       && cp build-arm64/libonnxruntime.dylib "$(DIST_STAGE)/$(PLUGIN_ARCH_DIR)/" || true
else
	@[ -f build/libpiper.so ]                           && cp build/libpiper.so "$(DIST_STAGE)/$(PLUGIN_ARCH_DIR)/" || true
	@[ -f build/libonnxruntime.so.1.22.0 ]             && cp build/libonnxruntime.so.1.22.0 "$(DIST_STAGE)/$(PLUGIN_ARCH_DIR)/" || true
	@[ -f build/libonnxruntime.so ]                    && cp build/libonnxruntime.so "$(DIST_STAGE)/$(PLUGIN_ARCH_DIR)/" || true
	@[ -f build/libonnxruntime_providers_shared.so ]   && cp build/libonnxruntime_providers_shared.so "$(DIST_STAGE)/$(PLUGIN_ARCH_DIR)/" || true
	@cd "$(DIST_STAGE)/$(PLUGIN_ARCH_DIR)" && \
	    [ -f libonnxruntime.so.1.22.0 ] && ln -sf libonnxruntime.so.1.22.0 libonnxruntime.so.1 || true
endif
	@# ── espeak-ng-data ──
	@if [ -d "build/espeak_ng-install/share/espeak-ng-data" ]; then \
	    mkdir -p "$(DIST_STAGE)/Resources/espeak-ng-data"; \
	    rsync -a "build/espeak_ng-install/share/espeak-ng-data/" \
	             "$(DIST_STAGE)/Resources/espeak-ng-data/"; \
	else \
	    echo "WARNING: espeak-ng-data missing — run 'make build' first"; \
	fi
	@mkdir -p "$(DIST_STAGE)/Resources/models"
	@# ── Hand-maintained overlays (additional airspace + per-airport overrides) ──
	@[ -f Resources/airspace+.txt ] && cp "Resources/airspace+.txt" "$(DIST_STAGE)/Resources/" || true
	@[ -f Resources/airport+.json ] && cp "Resources/airport+.json" "$(DIST_STAGE)/Resources/" || true
	@# ── Data files ──
	@mkdir -p "$(DIST_STAGE)/data/atc_profiles/eu/vfr" \
	          "$(DIST_STAGE)/data/atc_profiles/eu/ifr" \
	          "$(DIST_STAGE)/data/atc_profiles/us" \
	          "$(DIST_STAGE)/data/vrps"
	@cp data/atc_prompt_templates.json   "$(DIST_STAGE)/data/"
	@cp data/models_catalog.json         "$(DIST_STAGE)/data/"
	@cp data/vrps/airport_vrps.json      "$(DIST_STAGE)/data/vrps/"
	@cp data/atc_profiles/eu/vfr/atc_templates.json  "$(DIST_STAGE)/data/atc_profiles/eu/vfr/"
	@cp data/atc_profiles/eu/vfr/flight_rules.json   "$(DIST_STAGE)/data/atc_profiles/eu/vfr/"
	@cp data/atc_profiles/eu/ifr/atc_templates.json  "$(DIST_STAGE)/data/atc_profiles/eu/ifr/"
	@cp data/atc_profiles/eu/ifr/flight_rules.json   "$(DIST_STAGE)/data/atc_profiles/eu/ifr/"
	@cp data/atc_profiles/eu/intent_rules.json       "$(DIST_STAGE)/data/atc_profiles/eu/"
	@cp data/atc_profiles/eu/phraseology_hints.json  "$(DIST_STAGE)/data/atc_profiles/eu/"
	@cp data/atc_profiles/eu/ui_strings.json         "$(DIST_STAGE)/data/atc_profiles/eu/"
	@cp data/atc_profiles/us/atc_templates.json      "$(DIST_STAGE)/data/atc_profiles/us/"
	@cp data/atc_profiles/us/flight_rules.json       "$(DIST_STAGE)/data/atc_profiles/us/"
	@cp data/atc_profiles/us/intent_rules.json       "$(DIST_STAGE)/data/atc_profiles/us/"
	@cp data/atc_profiles/us/phraseology_hints.json  "$(DIST_STAGE)/data/atc_profiles/us/"
	@cp data/atc_profiles/us/ui_strings.json         "$(DIST_STAGE)/data/atc_profiles/us/"
ifeq ($(OS),Darwin)
	@# ── macOS: ZIP (-y preserves symlinks) ──
	@cd dist && zip -qry $(DIST_NAME).zip $(DIST_NAME)/
	@echo "Package: dist/$(DIST_NAME).zip"
	@echo "Drop the xp_wellys_atc/ folder inside the ZIP into:"
	@echo "  <X-Plane 12>/Resources/plugins/"
else
	@# ── Linux: tar.gz (symlinks preserved natively) ──
	@cd dist && tar -czf $(DIST_NAME).tar.gz $(DIST_NAME)/
	@echo "Package: dist/$(DIST_NAME).tar.gz"
	@echo "Extract with: tar -xzf $(DIST_NAME).tar.gz -C '<X-Plane 12>/Resources/plugins/'"
endif

# ── Lint ──────────────────────────────────────────────────────────────────────
format:
	@command -v clang-format >/dev/null 2>&1 || { \
	    echo "clang-format not found. Install with: brew install llvm"; \
	    echo "Then add to PATH: export PATH=\"$$(brew --prefix llvm)/bin:$$PATH\""; \
	    exit 1; }
	clang-format -i src/main.cpp src/*/*.cpp src/*/*.hpp

ifeq ($(OS),Darwin)
LINT_CMAKE_FLAGS   := -DCMAKE_OSX_ARCHITECTURES=arm64
LINT_TIDY_FLAGS    := --extra-arg="-isysroot" --extra-arg="$(shell xcrun --show-sdk-path)"
LINT_INSTALL_HINT  := brew install llvm
else
LINT_CMAKE_FLAGS   :=
LINT_TIDY_FLAGS    :=
LINT_INSTALL_HINT  := sudo apt install clang-tidy
endif

lint: $(SUBMODULES_SENTINEL) $(SDK_SENTINEL) $(IMGUI_SENTINEL) $(JSON_SENTINEL) $(CATCH2_SENTINEL)
	@command -v clang-tidy >/dev/null 2>&1 || { \
	    echo "clang-tidy not found. Install with: $(LINT_INSTALL_HINT)"; \
	    exit 1; }
	cmake -B build-lint -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON $(LINT_CMAKE_FLAGS) -Wno-dev
	clang-tidy -p build-lint $(LINT_TIDY_FLAGS) $(LINT_SOURCES)

# ── Sanitize ──────────────────────────────────────────────────────────────────
# AddressSanitizer + UBSan on the SDK-free engine OBJECT lib + atc_repl +
# Catch2 tests. The plugin module (`xp_wellys_atc.xpl`) is NOT instrumented —
# ASan inside the X-Plane process is fragile on macOS ARM64. For runtime
# leaks in the live plugin use Instruments.app (Leaks / Allocations
# templates) attached to the X-Plane process.
#
# Findings abort with a non-zero exit (`-fno-sanitize-recover=all`), so this
# target is CI-friendly. Build dir is `build-sanitize/` — independent of
# `build/` so Release artifacts stay untouched.
sanitize: $(SUBMODULES_SENTINEL) $(SDK_SENTINEL) $(IMGUI_SENTINEL) $(JSON_SENTINEL) $(CATCH2_SENTINEL)
	@echo "=== Configuring sanitizer build (ASan + UBSan) ==="
	cmake -B build-sanitize -DCMAKE_BUILD_TYPE=Debug -DXP_WELLYS_ATC_SANITIZE=ON -Wno-dev
	@echo "=== Building atc_repl + xp_wellys_atc_tests with ASan + UBSan ==="
	cmake --build build-sanitize --target atc_repl xp_wellys_atc_tests --parallel
	@echo ""
	@echo "=== Running unit tests under ASan + UBSan ==="
	@ASAN_OPTIONS=detect_leaks=0:abort_on_error=1:print_stacktrace=1 \
	 UBSAN_OPTIONS=print_stacktrace=1:halt_on_error=1 \
	     ./build-sanitize/xp_wellys_atc_tests
	@echo ""
	@echo "=== Running scenario tests under ASan + UBSan ==="
	@ASAN_OPTIONS=detect_leaks=0:abort_on_error=1:print_stacktrace=1 \
	 UBSAN_OPTIONS=print_stacktrace=1:halt_on_error=1 \
	     ./build-sanitize/atc_repl run testscripts/*.json
	@echo ""
	@echo "Sanitizer run clean."

# ── Release ───────────────────────────────────────────────────────────────────
release:
	@if [ -z "$(VERSION)" ]; then \
	    echo "Usage: make release VERSION=1.2.1"; exit 1; \
	fi
	@if ! git diff --quiet || ! git diff --cached --quiet; then \
	    echo "Uncommitted changes present. Commit or stash first."; exit 1; \
	fi
	@if [ -n "$$(git ls-files --others --exclude-standard)" ]; then \
	    echo "Untracked files present. Commit or clean up first."; exit 1; \
	fi
	@echo "$(VERSION)" > VERSION.txt
	@git add VERSION.txt
	@git commit -m "release $(VERSION)"
	@git push origin main
	@git tag -a "v$(VERSION)" -m "Release $(VERSION)"
	@git push origin "v$(VERSION)"
	@echo "Released v$(VERSION) and pushed tag to origin."

release-build:
	@$(MAKE) build RELEASE_FLAG=-DRELEASE=ON
	@echo "Done. Universal release build with version from VERSION.txt."

# ── SkunkCrafts Updater staging (local test) ──────────────────────────────────
# Stages a publishable release tree from the INSTALLED plugin and writes the
# SkunkCrafts control files into it. CI does not call this (there's no installed
# plugin in CI); the package job runs generate.py against its own staged bundle.
# The rsync --exclude list must mirror generate.py's IGNORE_GLOBS.
skunkcrafts:
	@if [ ! -d "$(PLUGIN_DIR)" ]; then \
	    echo "Plugin not installed at '$(PLUGIN_DIR)'. Run 'make install' first."; exit 1; \
	fi
	@VER="$(VERSION)"; \
	if [ -z "$$VER" ] && [ -f VERSION.txt ]; then VER="$$(cat VERSION.txt | tr -d '[:space:]')"; fi; \
	if [ -z "$$VER" ]; then \
	    echo "No version. Set VERSION=x.y.z or populate VERSION.txt."; exit 1; \
	fi; \
	echo "=== Staging SkunkCrafts release tree ($$VER) ==="; \
	rm -rf "$(SKUNK_DIR)"; mkdir -p "$(SKUNK_DIR)"; \
	rsync -a \
	    --exclude 'Resources/models/' \
	    --exclude 'Resources/espeak-ng-data/' \
	    --exclude '.DS_Store' \
	    --exclude 'skunkcrafts_updater*' \
	    "$(PLUGIN_DIR)/" "$(SKUNK_DIR)/"; \
	python3 tools/skunkcrafts/generate.py --tree "$(SKUNK_DIR)" --version "$$VER"; \
	echo "Staged release tree at $(SKUNK_DIR)/ (version $$VER)."; \
	echo "Publish its contents to the 'release' branch / your update host."

# ── Cleanup Tags ──────────────────────────────────────────────────────────────
cleanup-tags:
	git fetch --prune --prune-tags origin
	@echo "Local tags synced with remote."

# ── Cleanup Branches ──────────────────────────────────────────────────────────
cleanup-branches:
	@echo "Pruning remote-tracking references..."
	@git fetch --prune origin
	@echo ""
	@echo "Local branches whose upstream is gone:"
	@STALE=$$(git for-each-ref --format '%(refname:short) %(upstream:track)' refs/heads | awk '$$2 == "[gone]" {print $$1}'); \
	if [ -z "$$STALE" ]; then \
	    echo "  (none)"; \
	else \
	    echo "$$STALE" | sed 's/^/  /'; \
	    echo ""; \
	    echo "$$STALE" | xargs -n1 git branch -d; \
	fi
	@echo "Local branches synced with remote."

# ── Cleanup GitHub Actions Runs ───────────────────────────────────────────────
cleanup-runs:
	@command -v gh >/dev/null 2>&1 || { \
	    echo "gh not found. Install with: brew install gh"; exit 1; }
	@echo "Deleting GitHub Actions runs (keeping newest per workflow)..."
	@for wf in $$(gh workflow list --json id -q '.[].id'); do \
	    gh run list --workflow=$$wf --limit 1000 --json databaseId -q '.[1:] | .[].databaseId' \
	        | xargs -I {} gh run delete {}; \
	done
	@echo "Cleanup complete."

# Delete GitHub Actions caches. ccache entries accumulate per branch/key
# (and old keys go stale after a CI change like the cloud-only switch);
# GitHub's 10 GB LRU evicts them eventually, but this frees the quota now.
# All caches are rebuilt automatically on the next run.
cleanup-cache:
	@command -v gh >/dev/null 2>&1 || { \
	    echo "gh not found. Install with: brew install gh"; exit 1; }
	@echo "Current GitHub Actions caches:"
	@gh cache list --limit 100 2>/dev/null || true
	@echo ""
	@echo "Deleting all caches (rebuilt on the next CI run)..."
	@gh cache delete --all 2>/dev/null || echo "No caches to delete."
	@echo "Cache cleanup complete."

# ── Remote CI (Windows build via GitHub Actions) ──────────────────────────────
# The Windows slice can only be compiled by CI (no local MSVC toolchain on a
# Mac). `ci-remote` pushes the current branch and dispatches the build
# workflow against the pushed state; `win-artifact` downloads the resulting
# drop-in Windows plugin folder into dist-win/.
ci-remote:
	@command -v gh >/dev/null 2>&1 || { \
	    echo "gh not found. Install with: brew install gh"; exit 1; }
	@BRANCH=$$(git rev-parse --abbrev-ref HEAD); \
	echo "Pushing $$BRANCH and dispatching CI (builds the pushed state)..."; \
	git push -u origin "$$BRANCH"; \
	gh workflow run build.yml --ref "$$BRANCH" || { \
	    echo ""; \
	    echo "NOTE: workflow_dispatch is only accepted once build.yml (with the"; \
	    echo "'workflow_dispatch' trigger) exists on the default branch (main)."; \
	    echo "Until then, open a PR for this branch — the PR build produces the"; \
	    echo "same xp_wellys_atc-win artifact that 'make win-artifact' downloads."; \
	    exit 1; }
	@echo "Dispatched. Watch: gh run watch  (or: make win-artifact once green)"

win-artifact:
	@command -v gh >/dev/null 2>&1 || { \
	    echo "gh not found. Install with: brew install gh"; exit 1; }
	@BRANCH=$$(git rev-parse --abbrev-ref HEAD); \
	RUN_ID=$$(gh run list --workflow build.yml --branch "$$BRANCH" --limit 1 \
	    --json databaseId -q '.[0].databaseId'); \
	if [ -z "$$RUN_ID" ]; then \
	    echo "No CI run found for branch $$BRANCH. Run 'make ci-remote' first."; \
	    exit 1; \
	fi; \
	echo "Downloading xp_wellys_atc-win from run $$RUN_ID -> dist-win/ ..."; \
	rm -rf dist-win; mkdir -p dist-win; \
	gh run download "$$RUN_ID" -n xp_wellys_atc-win -D dist-win || { \
	    echo "Artifact not available yet (run still in progress or failed)."; \
	    echo "Check status: gh run view $$RUN_ID"; exit 1; }
	@echo "Done. Copy dist-win/xp_wellys_atc/ into X-Plane 12/Resources/plugins/"

# ── Clean ─────────────────────────────────────────────────────────────────────
clean:
	rm -rf build/ build-lint/ build-sanitize/ build-arm64/ build-x86_64/

# ── Distclean ─────────────────────────────────────────────────────────────────
# Remove everything 'make setup' downloaded so a full re-bootstrap is forced.
distclean: clean
	rm -rf sdk/ vendor/
	@echo "Removed sdk/ and vendor/. Run 'make setup' to re-download dependencies."
