SHELL := /bin/bash

OS := $(shell uname -s)

ifeq ($(OS),Darwin)
    XPLANE_ROOT ?= /Users/$(USER)/X-Plane 12
    PLUGIN_DIR  := $(XPLANE_ROOT)/Resources/available plugins/xp_wellys_atc
    PLUGIN_BIN_DIR := $(PLUGIN_DIR)/mac_x64
else
    XPLANE_ROOT ?= $(HOME)/X-Plane 12
    PLUGIN_DIR  := $(XPLANE_ROOT)/Resources/plugins/xp_wellys_atc
    PLUGIN_BIN_DIR := $(PLUGIN_DIR)/lin_x64
endif

SDK_SENTINEL    := sdk/XPLM/XPLMPlugin.h
IMGUI_SENTINEL  := vendor/imgui/imgui.h
JSON_SENTINEL   := vendor/json.hpp
CATCH2_SENTINEL := vendor/catch2/catch_amalgamated.hpp

# One sentinel for the three submodule trees (whisper.cpp, llama.cpp,
# Piper). They are all pulled in by a single
# `git submodule update --init --recursive` invocation, so tracking
# only the first one is sufficient — if it's missing, the whole
# submodule init runs and lands all three.
SUBMODULES_SENTINEL := spikes/spike_whisper/third_party/whisper.cpp/CMakeLists.txt

CATCH2_VERSION := 3.7.1

.PHONY: all help setup build install clean distclean format lint sanitize release release-build cleanup-tags cleanup-branches cleanup-runs repl run-repl test test-unit test-scenarios

.DEFAULT_GOAL := help

all: clean format build lint test

# ── Help ──────────────────────────────────────────────────────────────────────
help:
	@echo "xp_wellys_atc - Makefile targets"
	@echo ""
	@echo "  make                   Show this help (default)"
	@echo "  make all               clean + format + build + lint"
	@echo "  make setup             Init submodules + download X-Plane SDK, Dear ImGui, nlohmann/json, Catch2"
	@echo "  make build             Build plugin -> build/xp_wellys_atc.xpl"
	@echo "  make build CUDA=1      Linux only: build with NVIDIA CUDA GPU acceleration (requires cuda toolkit)"
	@echo "  make repl              Build headless CLI -> build/atc_repl"
	@echo "  make run-repl          Build + run the CLI (stdin transcripts)"
	@echo "  make test              Run unit tests + scenario tests"
	@echo "  make test-unit         Build + run Catch2 unit tests"
	@echo "  make test-scenarios    Build + run all scenario tests in testscripts/"
	@echo "  make install           Code-sign and install plugin to X-Plane"
	@echo "  make format            Run clang-format on src/*.cpp src/*.hpp"
	@echo "  make lint              Run clang-tidy on src/*.cpp"
	@echo "  make sanitize          Build atc_repl + tests with ASan+UBSan and run them"
	@echo "  make release VERSION=X Tag and push release (writes VERSION.txt)"
	@echo "  make release-build     Build plugin with RELEASE=ON (embeds VERSION.txt)"
	@echo "  make cleanup-tags      Prune local tags no longer on origin"
	@echo "  make cleanup-branches  Prune local branches whose remote is gone"
	@echo "  make cleanup-runs      Delete all GitHub Actions runs except the newest per workflow"
	@echo "  make clean             Remove build/, build-lint/ and build-sanitize/"
	@echo "  make distclean         clean + remove sdk/ and vendor/ (everything 'make setup' installed)"
	@echo "  make help              Show this help"

# ── Setup ─────────────────────────────────────────────────────────────────────
setup: $(SUBMODULES_SENTINEL) $(SDK_SENTINEL) $(IMGUI_SENTINEL) $(JSON_SENTINEL) $(CATCH2_SENTINEL)
	@echo "Setup complete. Run 'make build' to compile."

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
	find "$$TMP/sdk_extracted" -path "*/CHeaders/XPLM/*.h"   -exec cp {} sdk/XPLM/ \;; \
	find "$$TMP/sdk_extracted" -path "*/CHeaders/Widgets/*.h" -exec cp {} sdk/XPWidgets/ \;; \
	find "$$TMP/sdk_extracted" -path "*/Libraries/Win/*.lib"  -exec cp {} sdk/Libraries/Win/ \;; \
	find "$$TMP/sdk_extracted" -path "*/Libraries/Lin/*.so"   -exec cp {} sdk/Libraries/Lin/ \;; \
	cp -R "$$TMP/sdk_extracted"/*/Libraries/Mac/*.framework sdk/Libraries/Mac/ 2>/dev/null || \
	find "$$TMP/sdk_extracted" -name "*.framework" -exec cp -R {} sdk/Libraries/Mac/ \;
	@echo "SDK headers and libraries installed."

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
# macOS: produces a universal binary (arm64 local+cloud + x86_64 cloud-only)
#        via two CMake configures + lipo merge.
# Linux: single CMake configure (local+cloud), outputs lin_x64 .xpl.
#
# RELEASE_FLAG is passed through to CMake so the GitHub Actions workflow
# can flip -DRELEASE=ON for tag builds. Empty by default (dev build).
RELEASE_FLAG ?=

# GPU acceleration for Linux (NVIDIA CUDA). Set CUDA=1 on machines with an
# NVIDIA GPU + CUDA toolkit installed (e.g. `make build CUDA=1`).
# Required on the runtime machine — do not set when cross-compiling for a
# different GPU. Leave unset for CPU-only builds (safe on any Linux machine).
ifeq ($(CUDA),1)
    CUDA_FLAG := -DGGML_CUDA=ON
else
    CUDA_FLAG :=
endif

ifeq ($(OS),Darwin)
build: $(SUBMODULES_SENTINEL) $(SDK_SENTINEL) $(IMGUI_SENTINEL) $(JSON_SENTINEL) $(CATCH2_SENTINEL)
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
build: $(SUBMODULES_SENTINEL) $(SDK_SENTINEL) $(IMGUI_SENTINEL) $(JSON_SENTINEL) $(CATCH2_SENTINEL)
	@echo "=== Building xp_wellys_atc for Linux (local+cloud) ==="
	cmake -B build -DCMAKE_BUILD_TYPE=Release $(RELEASE_FLAG) $(CUDA_FLAG) \
	    -DXPWELLYS_USE_LOCAL_INFERENCE=ON \
	    -DBUILD_TESTS=OFF \
	    -Wno-dev
	cmake --build build --parallel
	@echo ""
	@file build/xp_wellys_atc.xpl
	@echo "Done. Run 'make install' to deploy the .xpl."
endif

# ── REPL (headless CLI) ───────────────────────────────────────────────────────
repl: $(SUBMODULES_SENTINEL) $(SDK_SENTINEL) $(IMGUI_SENTINEL) $(JSON_SENTINEL) $(CATCH2_SENTINEL)
	@echo "=== Building atc_repl ==="
	cmake -B build -DCMAKE_BUILD_TYPE=Release -Wno-dev
	cmake --build build --target atc_repl --parallel
	@echo ""
	@file build/atc_repl
	@echo "Done. Run 'make run-repl' or './build/atc_repl'."

run-repl: repl
	./build/atc_repl

# ── Tests ─────────────────────────────────────────────────────────────────────
test: test-unit test-scenarios

test-unit: $(SUBMODULES_SENTINEL) $(SDK_SENTINEL) $(IMGUI_SENTINEL) $(JSON_SENTINEL) $(CATCH2_SENTINEL)
	@echo "=== Building xp_wellys_atc unit tests ==="
	cmake -B build -DCMAKE_BUILD_TYPE=Release -Wno-dev
	cmake --build build --target xp_wellys_atc_tests --parallel
	@echo ""
	@echo "=== Running unit tests ==="
	@./build/xp_wellys_atc_tests

test-scenarios: repl
	@echo "=== Running scenario tests ==="
	./build/atc_repl run testscripts/*.json

# ── Install ───────────────────────────────────────────────────────────────────
# Shared data install helper — copies JSON data files into the plugin dir.
define INSTALL_DATA
	@mkdir -p "$(PLUGIN_DIR)/Resources/models"
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
	@mkdir -p "$(PLUGIN_DIR)/data/atc_profiles/eu" \
	          "$(PLUGIN_DIR)/data/atc_profiles/us" \
	          "$(PLUGIN_DIR)/data/atc_profiles/de"
	@cp data/atc_profiles/eu/atc_templates.json     "$(PLUGIN_DIR)/data/atc_profiles/eu/"
	@cp data/atc_profiles/eu/flight_rules.json      "$(PLUGIN_DIR)/data/atc_profiles/eu/"
	@cp data/atc_profiles/eu/intent_rules.json      "$(PLUGIN_DIR)/data/atc_profiles/eu/"
	@cp data/atc_profiles/eu/phraseology_hints.json "$(PLUGIN_DIR)/data/atc_profiles/eu/"
	@cp data/atc_profiles/eu/ui_strings.json        "$(PLUGIN_DIR)/data/atc_profiles/eu/"
	@cp data/atc_profiles/us/atc_templates.json     "$(PLUGIN_DIR)/data/atc_profiles/us/"
	@cp data/atc_profiles/us/flight_rules.json      "$(PLUGIN_DIR)/data/atc_profiles/us/"
	@cp data/atc_profiles/us/intent_rules.json      "$(PLUGIN_DIR)/data/atc_profiles/us/"
	@cp data/atc_profiles/us/phraseology_hints.json "$(PLUGIN_DIR)/data/atc_profiles/us/"
	@cp data/atc_profiles/us/ui_strings.json        "$(PLUGIN_DIR)/data/atc_profiles/us/"
	@cp data/atc_profiles/de/atc_templates.json     "$(PLUGIN_DIR)/data/atc_profiles/de/"
	@cp data/atc_profiles/de/flight_rules.json      "$(PLUGIN_DIR)/data/atc_profiles/de/"
	@cp data/atc_profiles/de/intent_rules.json      "$(PLUGIN_DIR)/data/atc_profiles/de/"
	@cp data/atc_profiles/de/phraseology_hints.json "$(PLUGIN_DIR)/data/atc_profiles/de/"
	@cp data/atc_profiles/de/ui_strings.json        "$(PLUGIN_DIR)/data/atc_profiles/de/"
	@mkdir -p "$(PLUGIN_DIR)/data/vrps"
	@cp data/vrps/airport_vrps.json "$(PLUGIN_DIR)/data/vrps/"
	@rm -f "$(PLUGIN_DIR)/data/atc_templates.json" \
	       "$(PLUGIN_DIR)/data/flight_rules.json" \
	       "$(PLUGIN_DIR)/data/airport_vrps.json"
	@rm -rf "$(PLUGIN_DIR)/data/regions"
endef

ifeq ($(OS),Darwin)
install:
	@if [ ! -f "build/xp_wellys_atc.xpl" ]; then \
	    echo "Plugin not built yet. Run 'make build' first."; exit 1; \
	fi
	@if [ ! -f "build/libpiper.dylib" ] || [ ! -f "build/libonnxruntime.1.22.0.dylib" ]; then \
	    echo "Runtime dylibs missing in build/. Did 'make build' succeed?"; exit 1; \
	fi
	@echo "=== Installing xp_wellys_atc (macOS) ==="
	@mkdir -p "$(PLUGIN_BIN_DIR)"
	@cp build/xp_wellys_atc.xpl "$(PLUGIN_BIN_DIR)/"
	@cp build/libpiper.dylib "$(PLUGIN_BIN_DIR)/"
	@cp build/libonnxruntime.1.22.0.dylib "$(PLUGIN_BIN_DIR)/"
	@cp build/libonnxruntime.dylib "$(PLUGIN_BIN_DIR)/"
	@xattr -dr com.apple.quarantine "$(PLUGIN_BIN_DIR)/xp_wellys_atc.xpl" 2>/dev/null || true
	@xattr -dr com.apple.quarantine "$(PLUGIN_BIN_DIR)/libpiper.dylib" 2>/dev/null || true
	@xattr -dr com.apple.quarantine "$(PLUGIN_BIN_DIR)/libonnxruntime.1.22.0.dylib" 2>/dev/null || true
	@for rp in $$(otool -l "$(PLUGIN_BIN_DIR)/xp_wellys_atc.xpl" \
	    | awk '/LC_RPATH/{flag=1; next} flag && /path/ {print $$2; flag=0}'); do \
	    install_name_tool -delete_rpath "$$rp" "$(PLUGIN_BIN_DIR)/xp_wellys_atc.xpl" 2>/dev/null || true; \
	done
	@install_name_tool -add_rpath "@loader_path" "$(PLUGIN_BIN_DIR)/xp_wellys_atc.xpl"
	@codesign --force --deep --sign - "$(PLUGIN_BIN_DIR)/libonnxruntime.1.22.0.dylib"
	@codesign --force --deep --sign - "$(PLUGIN_BIN_DIR)/libpiper.dylib"
	@codesign --force --deep --sign - "$(PLUGIN_BIN_DIR)/xp_wellys_atc.xpl"
	@if [ -d "build/espeak_ng-install/share/espeak-ng-data" ]; then \
	    mkdir -p "$(PLUGIN_DIR)/Resources/espeak-ng-data"; \
	    rsync -a --delete \
	        "build/espeak_ng-install/share/espeak-ng-data/" \
	        "$(PLUGIN_DIR)/Resources/espeak-ng-data/"; \
	    echo "Installed: $(PLUGIN_DIR)/Resources/espeak-ng-data/"; \
	else \
	    echo "WARNING: build/espeak_ng-install/share/espeak-ng-data missing — run make build first"; \
	fi
	$(INSTALL_DATA)
	@echo "Installed and signed."
else
install:
	@if [ ! -f "build/xp_wellys_atc.xpl" ]; then \
	    echo "Plugin not built yet. Run 'make build' first."; exit 1; \
	fi
	@echo "=== Installing xp_wellys_atc (Linux) ==="
	@mkdir -p "$(PLUGIN_BIN_DIR)"
	@cp build/xp_wellys_atc.xpl "$(PLUGIN_BIN_DIR)/"
	@[ -f "build/libpiper.so" ] && cp build/libpiper.so "$(PLUGIN_BIN_DIR)/" || true
	@[ -f "build/libonnxruntime.so.1.22.0" ] && cp build/libonnxruntime.so.1.22.0 "$(PLUGIN_BIN_DIR)/" || true
	@[ -f "build/libonnxruntime.so" ] && cp build/libonnxruntime.so "$(PLUGIN_BIN_DIR)/" || true
	@if [ -d "build/espeak_ng-install/share/espeak-ng-data" ]; then \
	    mkdir -p "$(PLUGIN_DIR)/Resources/espeak-ng-data"; \
	    rsync -a --delete \
	        "build/espeak_ng-install/share/espeak-ng-data/" \
	        "$(PLUGIN_DIR)/Resources/espeak-ng-data/"; \
	    echo "Installed: $(PLUGIN_DIR)/Resources/espeak-ng-data/"; \
	else \
	    echo "WARNING: espeak-ng-data missing in build/ — run make build first"; \
	fi
	$(INSTALL_DATA)
	@echo "Installed."
endif

# ── Lint ──────────────────────────────────────────────────────────────────────
format:
	@command -v clang-format >/dev/null 2>&1 || { \
	    echo "clang-format not found. Install with: brew install llvm"; \
	    echo "Then add to PATH: export PATH=\"$$(brew --prefix llvm)/bin:$$PATH\""; \
	    exit 1; }
	clang-format -i src/main.cpp src/*/*.cpp src/*/*.hpp

lint: $(SUBMODULES_SENTINEL) $(SDK_SENTINEL) $(IMGUI_SENTINEL) $(JSON_SENTINEL) $(CATCH2_SENTINEL)
	@command -v clang-tidy >/dev/null 2>&1 || { \
	    echo "clang-tidy not found."; \
	    echo "  macOS: brew install llvm && export PATH=\"\$$(brew --prefix llvm)/bin:\$$PATH\""; \
	    echo "  Linux: sudo apt install clang-tidy"; \
	    exit 1; }
ifeq ($(OS),Darwin)
	cmake -B build-lint -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_OSX_ARCHITECTURES=arm64 -Wno-dev
	clang-tidy -p build-lint --extra-arg="-isysroot" --extra-arg="$(shell xcrun --show-sdk-path)" src/main.cpp src/*/*.cpp
else
	cmake -B build-lint -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -Wno-dev
	clang-tidy -p build-lint src/main.cpp src/*/*.cpp
endif

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

# ── Clean ─────────────────────────────────────────────────────────────────────
clean:
	rm -rf build/ build-lint/ build-sanitize/ build-arm64/ build-x86_64/

# ── Distclean ─────────────────────────────────────────────────────────────────
# Remove everything 'make setup' downloaded so a full re-bootstrap is forced.
distclean: clean
	rm -rf sdk/ vendor/
	@echo "Removed sdk/ and vendor/. Run 'make setup' to re-download dependencies."
