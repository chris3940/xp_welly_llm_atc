# Hybrid KI — Dual-Backend: Local Inference + OpenAI Cloud

> One milestone, four feature branches (`feat/hybrid-ki-phase-{1..4}`) = four PRs.
> Self-contained: a fresh Claude session must be able to execute any phase without context from other phases.
> Plan-of-record: `/Users/robertw/.claude/plans/wie-schwierig-ist-es-quizzical-hamster.md` (approved).

## Context & Goal

Reactivate the v1.3.x OpenAI cloud path (commit `28e33b3`, deleted in `0bc511f`) as a **runtime-selectable alternative** to the current local inference pipeline (whisper.cpp + llama.cpp + Piper). User picks `local` or `openai` in Settings UI; the loader instantiates one full set of three backends — **never mixed**. Single plugin bundle as Universal Binary (`arm64` slice carries both backend sets, `x86_64` slice carries only the cloud set).

**Two user groups unblocked:**
1. Intel-Mac users — current build is hard-coded to `arm64` (Metal kernels in whisper/llama; onnxruntime 1.22 for Piper).
2. Performance-sensitive users — local Llama-3B-Q4 path eats CPU/GPU; cloud offloads that.

**Non-goals:**
- Mixing backends per-stage (e.g. local STT + cloud LM). Either all-local or all-cloud.
- New AI providers beyond OpenAI in this milestone (no Anthropic / Google / Azure).
- Cloud STT prompt biasing for non-English (whisper-1 is multilingual; we ship US/EU phraseology only).

## Codebase Pointers (read first)

Read `CLAUDE.md` end-to-end before anything else.

| Path | What's there |
|---|---|
| `src/backends/i_speech_to_text.hpp:16` | `ISpeechToText::transcribe(pcm_16k_mono, airport_context)`. Cloud impl ignores `airport_context` (OpenAI Whisper accepts `prompt=` field — wire it through). |
| `src/backends/i_language_model.hpp:32` | `respond_constrained(sys_prompt, user_text, grammar_gbnf)` has a default that ignores `grammar_gbnf`. Cloud LM uses `response_format: json_schema` instead — **no interface break needed**. |
| `src/backends/i_text_to_speech.hpp:27` | `synthesize(voice_id, text, length_scale, &sample_rate_hz)`. Cloud impl maps `voice_id` to OpenAI voices (`alloy/echo/fable/onyx/nova/shimmer`); `length_scale` → OpenAI `speed` param (inverse: `speed = 1.0 / length_scale`). |
| `src/backends/loader.cpp:182-329` | Currently hardcoded to `WhisperStt` + `LlamaLm` + `PiperTts`. **This is the mode-switch site.** Wrap in `if (settings::backend_mode() == "openai") { ... } else { /* existing */ }`. |
| `src/backends/manager.{hpp,cpp}` | Async dispatch is already backend-agnostic. **Untouched.** |
| `src/persistence/settings.hpp:26-82` | Function-API (no struct). Add `backend_mode()`, `openai_*_model()`, `openai_tts_voice_*()` getters/setters, `default_config()` JSON. |
| `src/ui/atc_ui.cpp:~2122` | Settings tab. Add backend dropdown + collapsible OpenAI section. |
| `vendor/json.hpp` | nlohmann/json 3.11.3 — used for all JSON parsing in this milestone. |
| `CMakeLists.txt:53-55` | `CMAKE_OSX_ARCHITECTURES = "arm64"` hard-set. Needs the universal-build conditional. |
| `CMakeLists.txt:77` | `find_package(CURL REQUIRED)` — libcurl already linked (model downloader). Cloud clients reuse it. |
| `Makefile:33,169` | Install path is `mac_x64/` (X-Plane uses this folder name for arm64 too). |
| **git: `28e33b3`** | Last commit with full OpenAI integration. Files: `src/openai/{whisper,gpt,tts}_client.{hpp,cpp}` + keychain wrapper in `src/persistence/settings.cpp`. **Primary template source.** Retrieve via `git show 28e33b3:src/openai/whisper_client.cpp` etc. |

## Architecture Constraints (non-negotiable)

- **No mixed mode at runtime.** Loader registers either all three local or all three cloud clients. Never both.
- **Strict client separation on source level.** Cloud clients (`openai_*.cpp`) and local clients (`whisper_stt.cpp` / `llama_lm.cpp` / `piper_tts.cpp`) share **no `#include`** and call **no common helpers** for inference. libcurl is referenced only from `openai_*.cpp` TUs and the existing `downloader.cpp` — nowhere in the local backend path.
- **Audit-grade logging.** Every inference call emits a backend-tagged log line via `logging::info` using a `kBackendTag` constant per client. Backend banner is logged on every mode-change at INFO level regardless of `debug_logging`.
- **API key never on disk in plaintext.** macOS Keychain via `Security.framework` (service `com.xp_wellys_atc.openai`, account `default`). `settings.json` stores only an `api_key_saved: true` flag.
- **API key never fully logged.** Truncate to last 4 chars in any log statement (`sk-...ABCD`).
- **Engine OBJECT lib stays SDK-free.** All cloud clients are plugin-only (libcurl + Keychain are SDK-coupled).
- **Universal Binary, single `.xpl`.** Two CMake configures (one per arch) + `lipo -create` merge. CMake option `XPWELLYS_USE_LOCAL_INFERENCE` (default ON for arm64, OFF for x86_64) gates compilation of whisper/llama/piper TUs.
- **ASCII-only logs and UI labels** (CLAUDE.md rule — `XPLMDebugString` + ImGui font drop UTF-8 specials to `?`).
- **No new external deps.** libcurl + nlohmann/json + ImGui + Catch2 (all already vendored).
- **Realism still rules.** Cloud TTS voices are not aviation-trained; surface this in the UI ("Onyx is closest to ATC"), don't silently swap.

## Phases (each = its own PR)

### Phase 1 — Infrastructure (Keychain + Settings Schema)
Branch: `feat/hybrid-ki-phase-1-infra`
- Port `keychain.{hpp,mm}` from `28e33b3` (Objective-C++ wrapper over `Security.framework`). Functions: `save_api_key`, `load_api_key`, `delete_api_key`, `has_api_key`.
- Extend `src/persistence/settings.{hpp,cpp}` with new keys + getters/setters:
  - `backend_mode` (default `"local"`)
  - `api_key_saved` (default `false`)
  - `openai_stt_model` (default `"whisper-1"`)
  - `openai_lm_model` (default `"gpt-4o-mini"`)
  - `openai_tts_model` (default `"tts-1"`)
  - `openai_tts_voice_atis` / `_tower` / `_ground` (defaults `"onyx"` / `"echo"` / `"alloy"`)
- Update `data/settings.json` with the new defaults.
- Link `Security` framework in `CMakeLists.txt`.
- Catch2 tests: Keychain roundtrip (save → load → delete → load returns empty), settings JSON roundtrip with all new keys.

### Phase 2 — OpenAI Clients + Audit Logging
Branch: `feat/hybrid-ki-phase-2-clients`
- New files (all plugin-only):
  - `src/backends/openai_stt.{hpp,cpp}` — POST `api.openai.com/v1/audio/transcriptions`, multipart WAV, model from settings. Tag `STT-OPENAI`. Template: `28e33b3:src/openai/whisper_client.cpp`.
  - `src/backends/openai_lm.{hpp,cpp}` — POST `/v1/chat/completions`, JSON-mode for constrained intent classification. Tag `LM-OPENAI`. Template: `28e33b3:src/openai/gpt_client.cpp`.
  - `src/backends/openai_tts.{hpp,cpp}` — POST `/v1/audio/speech`, `response_format: wav` for direct PCM. Voice from `voice_id`, `speed = 1.0 / length_scale`. Tag `TTS-OPENAI`. Template: `28e33b3:src/openai/tts_client.cpp`.
- Audit logging:
  - Define `constexpr const char* kBackendTag = "STT-OPENAI"` (etc.) in each client TU.
  - Wrap every outbound call in `logging::info("[%s] POST /v1/... key sk-...%s", kBackendTag, last4(api_key))`.
  - Add `[STT-LOCAL]` / `[LM-LOCAL]` / `[TTS-LOCAL]` log lines at inference entrypoint of `whisper_stt.cpp`, `llama_lm.cpp`, `piper_tts.cpp` (~3 lines each).
- `tests/test_audit_logging.cpp` (~120 LOC): capture log output from a mock `logging::sink`, assert that in local mode no `*-OPENAI` tag appears and vice versa. Use a fake `ILanguageModel` mock that emits its tag and runs a single classify call.
- HTTP unit tests: spin up a local libmicrohttpd (or vendored TCP stub) on `127.0.0.1` and point the client at it via an `OPENAI_BASE_URL` constructor argument (default `https://api.openai.com`).

### Phase 3 — Loader Mode-Switch + UI Settings Tab
Branch: `feat/hybrid-ki-phase-3-switch`
- Refactor `src/backends/loader.cpp:182-329`:
  ```cpp
  const std::string mode = settings::backend_mode();
  logging::info("[xp_wellys_atc] BACKEND MODE: %s",
                mode == "openai" ? "OPENAI (api.openai.com)" : "LOCAL (whisper/llama/piper)");
  if (mode == "openai") {
      std::string key = settings::load_api_key();
      if (key.empty()) { /* error banner + return */ }
      backends::register_stt(std::make_unique<OpenAiStt>(key, settings::openai_stt_model()));
      backends::register_lm (std::make_unique<OpenAiLm> (key, settings::openai_lm_model()));
      backends::register_tts(std::make_unique<OpenAiTts>(key, settings::openai_tts_model()));
      return;
  }
  #ifdef XPWELLYS_USE_LOCAL_INFERENCE
  // existing verify_files + Whisper/Llama/Piper init
  #else
  logging::error("[xp_wellys_atc] Local inference not available in this build (Intel slice)");
  #endif
  ```
- Runtime mode switch = `loader::stop() + loader::start()` (no new lifecycle code). Log `LOCAL -> OPENAI` (and vice versa) at INFO.
- UI `src/ui/atc_ui.cpp:~2122` Settings tab additions:
  - Dropdown **Backend** [Local / OpenAI Cloud].
  - When OpenAI selected, collapsible group with:
    - API-Key password-masked TextInput + `[Paste]` + `[Save Key]` + `[Delete Key]` buttons.
    - Status label `Saved (Keychain) (OK)` / `Not configured`.
    - STT/LM/TTS model combo (small fixed list, no free text).
    - 3x voice combo (ATIS/Tower/Ground) → `alloy/echo/fable/onyx/nova/shimmer`. Annotate `onyx` as `(closest to ATC)`.
  - Hide the **Models** tab (or replace its body with `Not needed - using OpenAI Cloud`) when `backend_mode == "openai"`.
- Error banners (in the existing status panel):
  - `OpenAI API key missing` (no key in Keychain).
  - `OpenAI auth failed (401)` (got 401 from API).
  - `OpenAI request failed (no network)` (libcurl `CURLE_COULDNT_RESOLVE_HOST` etc.).
  - `Local inference requires Apple Silicon` (Intel slice + local mode).

### Phase 4 — Universal Binary (Build System)
Branch: `feat/hybrid-ki-phase-4-universal`
- `CMakeLists.txt`:
  - Add `option(XPWELLYS_USE_LOCAL_INFERENCE "Compile local whisper/llama/piper backends" ON)`.
  - Guard `add_subdirectory(spikes/spike_whisper/...)`, llama, piper, onnxruntime staging with `if (XPWELLYS_USE_LOCAL_INFERENCE)`.
  - Guard `whisper_stt.cpp`, `llama_lm.cpp`, `piper_tts.cpp` source inclusion in the plugin module with the same flag. Cloud backends are always-on.
  - Relax `CMAKE_OSX_ARCHITECTURES` hardcode — allow override from the parent build script.
- `Makefile`:
  - New target `build-universal`:
    ```make
    build-universal:
        cmake -B build-arm64   -DCMAKE_OSX_ARCHITECTURES=arm64   -DXPWELLYS_USE_LOCAL_INFERENCE=ON
        cmake --build build-arm64
        cmake -B build-x86_64  -DCMAKE_OSX_ARCHITECTURES=x86_64  -DXPWELLYS_USE_LOCAL_INFERENCE=OFF
        cmake --build build-x86_64
        lipo -create \
            build-arm64/xp_wellys_atc.xpl \
            build-x86_64/xp_wellys_atc.xpl \
            -output build/xp_wellys_atc.xpl
        file build/xp_wellys_atc.xpl   # must show: Mach-O universal binary with 2 architectures
    ```
  - `make install` stays as-is (single `.xpl` install).
- Verify with `file build/xp_wellys_atc.xpl` → `Mach-O universal binary with 2 architectures: [arm64:...] [x86_64:...]`.
- Verify x86_64 slice loads in X-Plane on Intel host (or Rosetta) → shows the "Local inference requires Apple Silicon" banner if `backend_mode = "local"`; works fine in OpenAI mode.

## Acceptance Criteria (across all phases)

- [ ] **Source-level audit invariant:** `grep -rn "openai" src/backends/whisper_stt.cpp src/backends/llama_lm.cpp src/backends/piper_tts.cpp` returns nothing. `grep -rn "whisper.h\|llama.h\|piper.h" src/backends/openai_*.cpp` returns nothing.
- [ ] **Runtime audit invariant:** running a full ATC session in local mode and `grep -E "\\[(STT|LM|TTS)-OPENAI\\]" Log.txt` returns nothing. Symmetric check for openai mode and `*-LOCAL`.
- [ ] **API key safety:** `grep -i "sk-[A-Za-z0-9]\{20,\}" Log.txt` returns nothing (keys truncated to last 4 chars only).
- [ ] **Bundle:** `file build/xp_wellys_atc.xpl` shows two architectures. `make install` continues to work unchanged.
- [ ] **Mode-switch UX:** changing `Backend` dropdown + clicking Apply unloads previous backends (memory drop visible in Activity Monitor for local) and loads the other set without an X-Plane restart.
- [ ] **Keychain:** `security find-generic-password -s com.xp_wellys_atc.openai` returns the saved entry after `[Save Key]`; gone after `[Delete Key]`.
- [ ] **Latency:** local pipeline still <= 1.5 s warm; cloud pipeline <= 3.0 s warm (3G+).
- [ ] **Regressions:** `make all` (format + lint + build + tests) stays green throughout. Existing scenario tests in `tests/` keep passing in local mode.
- [ ] **Error banners:** missing key, 401, network loss, intel-local each surface the documented user-facing string. PTT remains disabled in the missing-key case.

## Out of Scope (explicit)

- Per-stage backend selection (e.g. local STT + cloud LM).
- Other cloud providers (Anthropic, Google, Azure).
- Self-hosted OpenAI-compatible endpoints (Ollama, vLLM). Possible later via `OPENAI_BASE_URL` setting, but not in this milestone.
- Streaming TTS / partial responses. Cloud TTS returns full WAV; play after receive completes.
- Cost telemetry / token counting UI.

## Verification Recipe

1. `make all` clean on `main`, branch off `feat/hybrid-ki-phase-1-infra`.
2. After each phase: `make all` green, manual smoke test in X-Plane (local mode still works end-to-end).
3. After Phase 3: full E2E in both modes, plus runtime mode switch.
4. After Phase 4: `make build-universal`, install on Intel-Mac (or x86_64 slice via Rosetta-launched X-Plane), confirm cloud path works.
5. Audit-log greps from Acceptance Criteria pass.
