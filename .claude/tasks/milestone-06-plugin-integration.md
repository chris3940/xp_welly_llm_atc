# Milestone 06 — Plugin Integration (Strategy Pattern)

## Goal

Replace the OpenAI-based STT/LLM/TTS backends in the X-Plane plugin with the locally
running implementations validated in milestones 02–05. The ATC state machine, flight
phase detection, ATIS generation, and ImGui UI structure stay functionally unchanged.
**Only enter this milestone after the spike is officially "go"** (per milestone 05).

## Deliverables

1. **Refactored backend layer** based on the Strategy interfaces designed in milestone 01:
   - `ISpeechToText`, `ILanguageModel`, `ITextToSpeech` (or whatever names were agreed).
   - Concrete `WhisperCppStt`, `LlamaCppLlm`, `PiperTts` implementations sourced from
     the spike code.
   - The old OpenAI implementations are **removed**, not kept behind a flag — the fork
     is dedicated to the local stack.
2. **Static linking** of whisper.cpp, llama.cpp, Piper, and onnxruntime into the plugin
   `.xpl` binary. Single deliverable, no dylib hunt at runtime.
3. **Model loader** in the plugin:
   - Looks for models under `Resources/models/` relative to the plugin location.
   - Loads on plugin startup (not on first inference) — predictable cold-start.
   - Surfaces load errors via the existing logging path.
4. **ImGui status panel** addition:
   - Per-backend state: `Loading… / Ready / Error`.
   - RAM consumption (sum of resident sizes, polled every few seconds).
   - Last inference latency per stage (for live tuning).
5. **README updates**:
   - Hardware requirements (M1 / 32 GB).
   - Model download instructions and exact URLs/hashes.
   - Build instructions (CMake, Xcode CLT, submodule init).
6. **Smoke test**: load X-Plane, fly a short scripted scenario, confirm a clearance
   request → controller response cycle works end-to-end inside the sim.

## Acceptance criteria

| Item | Target |
|------|--------|
| Plugin builds cleanly with `cmake --build build --target xp_welly_llm_atc` | yes |
| Plugin loads in X-Plane 12 on M1 without crash | yes |
| One full taxi/takeoff/cruise/landing scenario runs without backend errors | yes |
| End-to-end latency inside the sim matches milestone 05 numbers ±20% | yes |
| ATC state machine logic is functionally equivalent to the original | yes (manual diff review) |
| No OpenAI / network calls remain | verified by running offline |

## Architecture constraints (re-stated)

- ATC state machine: **untouched** beyond the call sites that now hit local backends
  through the Strategy interfaces.
- No background daemons, no helper processes, no Ollama, no external Python.
- Metal backend enabled at build time and runtime-verified.
- All third-party code: MIT or compatible (Apache-2.0, BSD). Document in `THIRD_PARTY.md`.

## Open decisions

- **[DECISION]** Where do model files live in distribution? Options:
  (a) committed under Git LFS — easy install, but bloats repo.
  (b) downloaded on first run by the plugin — adds network code we said we wouldn't have.
  (c) manual download per README — current spike approach, simplest.
  → recommend **(c)** for the spike; revisit only if the user wants a smoother install.
- **[DECISION]** Threading model: dedicated worker threads per backend, or a single
  inference thread with a queue? → propose during this milestone after profiling.
- **[DECISION]** Behavior when a model fails to load: hard-fail the plugin, or run with
  the broken backend disabled and a UI warning? → likely hard-fail for the spike — user
  to confirm.

## Dependencies

- All previous milestones complete.
- Milestone 05 latency goal met → "go" recommendation in the README.

## Out of scope (for the spike phase)

- Automatic model downloads.
- Multi-language support.
- Voice cloning, custom voices.
- Wake-word detection.
- Production-grade error recovery (graceful degradation when a model crashes mid-flight).
- Universal binary (x86_64 build) — Apple Silicon only.
