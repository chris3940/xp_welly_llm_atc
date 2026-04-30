# Milestone 04 — Spike: Piper TTS standalone CLI

## Goal

Prove that Piper can synthesize a 50-word ATC phrase to a WAV file in under 1 s on the
M1, with intelligible, ATC-appropriate voice quality.

## Deliverables

1. **Directory** `spikes/spike_piper/` with its own `CMakeLists.txt`.
2. **Piper** integrated as a static library:
   - Option A (preferred): build `piper-phonemize` + `piper` from source as submodules.
   - Option B (fallback): use the prebuilt `libpiper.a` from the official release if
     source build proves painful on Apple Silicon.
   - Note: Piper depends on `onnxruntime`. We'll need the macOS arm64 prebuilt of
     onnxruntime — verify license compatibility (MIT → fine).
3. **CLI tool** `spike_piper`:
   - Usage: `spike_piper <voice.onnx> <voice.json> <text-or-textfile> <out.wav>`
   - Synthesizes the input to a 22.05 kHz mono WAV.
   - Prints: synthesis time, real-time factor (RTF), peak RAM.
4. **README** at `spikes/spike_piper/README.md`:
   - Build instructions, voice model URL + SHA256, example invocation, measured numbers.

## Acceptance criteria

| Metric | Target |
|--------|--------|
| Synthesis latency for 50-word phrase | **< 1 s** |
| Output is intelligible and free of artifacts | qualitatively yes |
| Real-time factor (audio_seconds / synth_seconds) | **> 5×** (i.e. way faster than realtime) |
| Peak RAM | **< 200 MB** (Piper is small) |

## Voice recommendation

**Primary:** `en_US-lessac-medium` (~63 MB)
- Clear, neutral US accent.
- "Medium" quality is the standard trade-off for real-time use.

**Alternative for evaluation:** `en_US-ryan-high`
- Higher quality, slightly slower, slightly larger.
- Worth A/B-comparing against lessac-medium for ATC clarity.

Voices live at `https://huggingface.co/rhasspy/piper-voices`. README will record the
final pick + SHA256.

## Open decisions

- **[DECISION]** Source vs. prebuilt static lib: source build is cleaner long-term but
  may cost a day of CMake yak-shaving on Apple Silicon. Recommend trying source first
  with a hard time-box; fall back to prebuilt if it stalls.
- **[DECISION]** Sample rate handling: Piper outputs 22.05 kHz, plugin audio path
  probably runs at 44.1 / 48 kHz. Recommend deferring resampling to milestone 06
  (Core Audio side); spike emits raw 22.05 kHz WAV.
- **[DECISION]** Final voice pick happens after listening to both candidates with the
  same ATC sample text — user makes the call.

## Dependencies

- Milestone 01 complete.
- Independent of milestones 02 and 03 — can run in parallel.

## Out of scope

- Streaming synthesis (chunked output) — file-based is enough for the proof.
- Voice cloning / training.
- SSML or prosody markup.
