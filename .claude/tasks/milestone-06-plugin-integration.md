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
2. **Single-deliverable bundling** of whisper.cpp, llama.cpp, Piper, and
   onnxruntime. Static where the upstream allows it (whisper.cpp,
   llama.cpp, espeak-ng), shared with `@loader_path` rpath where it
   does not (Piper builds `libpiper.dylib` from source, onnxruntime is
   a prebuilt vendor dylib — neither realistically goes static without
   forking upstream). The `.dylib`s live **inside** the plugin bundle
   alongside the `.xpl`, so the user still drops a single folder into
   `X-Plane/Resources/plugins/` and never sees a "where is libfoo"
   error. espeak-ng's data directory ships in the same bundle.
3. **Model loader** in the plugin:
   - Looks for models under `Resources/models/` **relative to the plugin's own
     location**, derived via `XPLMGetPluginInfo` + `XPLMGetSystemPath`. Never
     `$HOME`, `~/Library`, or CWD. This makes the plugin portable across
     X-Plane installations on internal SSDs, external USB/Thunderbolt drives,
     and volume paths containing spaces or non-ASCII characters.
   - Loads on plugin startup (not on first inference) — predictable cold-start.
   - Surfaces load errors via the existing logging path.
4. **In-plugin model downloader**:
   - On plugin startup, scan `Resources/models/` for the expected model files
     (whisper, llama, piper) and SHA256-verify each against the bundled
     manifest.
   - If any are missing or mismatch, present an ImGui dialog with a per-model
     "Download now" button. No silent background download — the user opts in.
   - libcurl on a dedicated `std::thread`, streaming-write directly to the
     final destination on whichever disk hosts the plugin (no temp roundtrip
     via the system disk — important for users on small internal SSDs with
     X-Plane on an external drive).
   - HTTP `Range` resume support (`CURLOPT_RESUME_FROM_LARGE`) so that a
     dropped USB/Thunderbolt link or Wi-Fi blip does not restart a 2 GB
     download from scratch.
   - Available-disk-space precheck (~2.3 GB total budget for all three models)
     using `std::filesystem::space()` on the target dir before starting.
   - Progress reported via `std::atomic<size_t>` read once per ImGui frame —
     no locking on the render path.
   - Cancel flag (`std::atomic<bool>`) checked in libcurl's progress
     callback; the worker exits within ~1 s of cancel.
   - `.part` suffix during download; atomic rename to the final filename
     **only** after SHA256 verification succeeds. A corrupted partial file
     can never be loaded as a model.
   - On `XPluginDisable`: set cancel flag, then `join()` the worker thread
     before returning. No threads survive the .xpl unload.
5. **ImGui status panel** addition:
   - Per-backend state: `Missing / Downloading <pct>% / Verifying / Ready / Error`.
   - Per-model: filename, expected size, SHA256, current state, action button
     (Download / Cancel / Re-verify / Retry).
   - RAM consumption (sum of resident sizes, polled every few seconds).
   - Last inference latency per stage (for live tuning).
6. **README updates**:
   - Hardware requirements (M1 / 32 GB).
   - Model download flow: "first launch shows a download dialog; ~2.3 GB total
     over HTTPS from HuggingFace; resumable; expect 5-30 min on typical home
     internet". Include the URLs/hashes as a manual-fallback table for users
     behind restrictive networks.
   - Build instructions (CMake, Xcode CLT, submodule init).
7. **Smoke test**: load X-Plane on a fresh install (no models present),
   trigger the download from the plugin UI, then fly a short scripted
   scenario and confirm a clearance request → controller response cycle works
   end-to-end inside the sim. Repeat with X-Plane installed on an external
   USB-SSD to validate the path-resolution requirement.

   ### Smoke-test checklist (P8 — user-driven)

   Phases 1–7 are done. The remaining work is hands-on testing in
   X-Plane that I cannot automate. Run the following and tick each
   item; anything failing goes back to the relevant phase as a bug.

   **Pre-flight (5 min)**
   - [ ] `make all` is green locally — `Status: PASS, 192 assertions`
         on the scenario suite, clang-tidy clean.
   - [ ] `make install` populates `<plugin>/mac_x64/` with
         `xp_wellys_atc.xpl`, `libpiper.dylib`,
         `libonnxruntime.1.22.0.dylib`, `libonnxruntime.dylib`, plus
         `<plugin>/Resources/espeak-ng-data/` (~19 MB) and an empty
         `<plugin>/Resources/models/`.
   - [ ] Existing `<plugin>/data/settings.json` (if upgrading from
         the cloud build) still parses on first load — legacy keys
         should be silently stripped, no error in
         `X-Plane 12/Log.txt`.

   **Cold-launch download flow**
   - [ ] Launch X-Plane 12. Open *Plugins → Welly's ATC*.
   - [ ] **Status tab** shows the red "models not ready — open Models
         tab" banner.
   - [ ] **Models tab** shows three rows in red (`Missing`) and one
         row for the Piper voice config. "Still need: 1.94 GB"
         approximately.
   - [ ] Click *Download all missing*. Watch the progress bars —
         Whisper finishes first (~30 s on fast internet), Piper voice
         next, Llama last (the 1.88 GB pull). Each row flips
         `Downloading → Verifying → Loading → Ready` automatically;
         no manual click needed between files.
   - [ ] After all rows are green: Status tab banner disappears, PTT
         no longer logs "models not loaded".

   **Cold-launch resume flow**
   - [ ] During the Llama download (after a few hundred MB), click
         *Cancel* on that row. Confirm the row goes
         `Cancelled` and `<file>.gguf.part` is on disk.
   - [ ] Click *Download* on the Llama row again. Confirm libcurl
         resumes from the previous offset (progress bar starts above
         0 %) and finishes successfully.
   - [ ] Mid-download, kill X-Plane (Cmd-Q). Restart. The .part file
         should still be there; clicking Download resumes from where
         the previous run left off.

   **Manual-fallback flow**
   - [ ] Quit X-Plane. Delete `<plugin>/Resources/models/` contents.
         Manually drop the four files from the README's URL/SHA256
         table into the directory.
   - [ ] Launch X-Plane → Models tab. The verify worker should hash
         each file (Llama takes ~4 s on M1) and flip everything to
         `Ready` without any download.

   **Hash-mismatch path**
   - [ ] With models in place, append a single byte to
         `Llama-3.2-3B-Instruct-Q4_K_M.gguf` (`echo X >> <file>`).
   - [ ] Restart X-Plane. The Llama row should go `Wrong size` (size
         changed) → user-readable message in the Models tab pointing
         at re-download.
   - [ ] Click *Download* and confirm the .part flow handles the
         corrupt file (it should treat the corrupt file as the
         starting point but the size pre-check rejects it; fix is to
         delete the file manually first, OR the implementation could
         auto-delete on size mismatch — note which behaviour you see
         and report).

   **Single ATC interaction (golden path)**
   - [ ] Models are Ready. Spawn at LSZG (Grenchen) or your usual
         test airport on the ground. Tune COM1 to Ground frequency.
   - [ ] Hold PTT. Speak: *"Grenchen Ground, Hotel Bravo Romeo
         Charlie Delta, holding short runway 25, ready for taxi"*.
   - [ ] Release PTT. Watch the Models tab footer: `STT 300–400 ms`,
         `LM 600–800 ms`, `TTS 200–300 ms` should appear.
   - [ ] ATC reply plays back through the radio bus (must be audible
         over X-Plane's COM device — set in *Settings → Sound → Radio
         Device*).
   - [ ] Transcript tab shows the pilot row with the freq tag and
         the ATC reply row.

   **Full pattern (acceptance criterion)**
   - [ ] Fly one full taxi → takeoff → pattern → landing scenario.
         Verify each ATC turn (taxi clearance, takeoff clearance,
         downwind report, base/final, landing clearance, runway
         vacated) round-trips through whisper.cpp + state machine +
         Piper without falling back to "say again" repeatedly.
   - [ ] **Latency check** — the Models tab shows per-stage timing
         after every interaction. Total should stay under ~1.5 s on
         M1 for typical short calls. If > 3 s persistently: re-check
         milestone 05 numbers and consider the optimisation levers
         in `spikes/spike_e2e/RESULTS.md`.
   - [ ] **No OpenAI traffic** — confirm offline. With Wi-Fi off,
         the full pattern still works after models are downloaded.
         Verify in *Activity Monitor → Network* that
         `xp_wellys_atc.xpl` produces no outbound HTTPS traffic
         during a flight.

   **External-disk path resolution (deliverable 3)**
   - [ ] Move X-Plane 12 to an external USB-SSD or Thunderbolt drive
         (or do this once with a separate X-Plane install).
   - [ ] Run `make install` (or extract the release ZIP) into the
         external X-Plane's `Resources/plugins/`.
   - [ ] Confirm the plugin loads, Models tab points at the *external*
         volume's `<plugin>/Resources/models/`, downloads land on
         the external disk (not the internal SSD), and a full
         interaction works.

   **Log inspection (final pass)**
   - [ ] Search `X-Plane 12/Log.txt` for `xp_wellys_atc` lines. No
         OpenAI / api_key / Keychain references. Backend init logs
         show "STT backend ready (whisper.cpp)", "LM backend ready
         (llama.cpp)", "TTS backend ready (Piper)".
   - [ ] No crash in `Log.txt`, no `crashreporter.txt` next to it.

   When every checkbox above passes, milestone 06 is complete and
   the spike is officially shipped.

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

- **[DECISION] — resolved 2026-04-30** Where do model files live in distribution?
  Options considered:
  (a) committed under Git LFS — easy install, but bloats the repo and pushes
      the LFS bandwidth cost onto whoever forks.
  (b) downloaded on first run by the plugin — needs HTTP code, but libcurl
      and ImGui are already in the build; adds ~200 LOC.
  (c) manual download per README — simplest, but requires Terminal use,
      which is poor UX for an X-Plane plugin.
  → **Decision: (b).** The plugin already does HTTP for OpenAI today; the
  same `std::thread` pattern (CLAUDE.md: *"All network/audio calls on
  std::thread"*) applies to model fetch. The user-experience win is large
  (single click vs. Terminal session), and SHA256 verification can live next
  to the download code instead of in shell snippets the user has to copy.
  Manual URLs/hashes still live in the README as a fallback for users behind
  restrictive networks. See deliverable 4 for concrete requirements.
- **[DECISION]** Threading model: dedicated worker threads per backend, or a single
  inference thread with a queue? → propose during this milestone after profiling.
- **[DECISION]** Behavior when a model fails to load: hard-fail the plugin, or run with
  the broken backend disabled and a UI warning? → likely hard-fail for the spike — user
  to confirm.

## Dependencies

- All previous milestones complete.
- Milestone 05 latency goal met → "go" recommendation in the README.

## Out of scope (for the spike phase)

- Multi-mirror / CDN fallback. HuggingFace is the single source; if it is
  down, the manual URL/hash table in the README is the user's fallback.
- Background / silent downloads. The download is always opt-in via the
  ImGui dialog.
- Differential / patch updates of model files. A model version bump is a
  full re-download.
- Multi-language support.
- Voice cloning, custom voices.
- Wake-word detection.
- Production-grade error recovery (graceful degradation when a model crashes mid-flight).
- Universal binary (x86_64 build) — Apple Silicon only.
