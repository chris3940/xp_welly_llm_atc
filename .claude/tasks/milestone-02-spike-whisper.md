# Milestone 02 — Spike: whisper.cpp standalone CLI

## Goal

Prove that whisper.cpp with the Metal backend can transcribe a 5-second ATC audio clip in
under 0.5 s with a RAM footprint below 500 MB on the M1.

## Deliverables

1. **Directory** `spikes/spike_whisper/` with its own `CMakeLists.txt` (independent of the
   plugin build).
2. **whisper.cpp** integrated as a git submodule, pinned to a stable tag.
   Build with `WHISPER_METAL=ON`, `WHISPER_NO_AVX*=ON` (irrelevant on arm64 anyway).
3. **CLI tool** `spike_whisper`:
   - Usage: `spike_whisper <model.bin> <audio.wav>`
   - Loads the GGUF model, transcribes the WAV, prints the transcription.
   - Prints timing breakdown: model-load time, inference time, wall-clock total.
   - Prints peak resident memory (via `getrusage` / `mach_task_basic_info`).
4. **Test fixture**: at least one ATC-style WAV in `spikes/spike_whisper/test_audio/`
   (~5 s, 16 kHz mono PCM). Source: see open decisions below.
5. **README** at `spikes/spike_whisper/README.md`:
   - Build instructions (`cmake -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build`).
   - Model download URL + SHA256.
   - Example invocation and expected output.
   - Measured numbers from the M1.

## Acceptance criteria

| Metric | Target |
|--------|--------|
| Transcription latency (5 s clip, after warm-up) | **< 500 ms** |
| Peak RAM during inference | **< 500 MB** |
| Word error rate on ATC test phrase | qualitatively acceptable (no formal WER required) |
| Build is reproducible from a clean checkout | yes |

If targets are missed: document the actual numbers, identify the bottleneck (model size,
quantization, threading), and propose alternatives before moving on.

## Model recommendation

**Primary:** `ggml-distil-small.en-q5_1.bin` (~250 MB)
- English-only → smaller and faster than multilingual variants.
- Q5_1 quantization → good quality/size trade-off.
- Distilled → ~6× faster than the full small.en at minimal quality loss.

**Fallback if quality is insufficient:** `ggml-small.en-q5_1.bin` (~470 MB).

**Do not use** the multilingual variants — wasted capacity for an English-only ATC use case.

Both available at `https://huggingface.co/ggerganov/whisper.cpp` (TBD: confirm exact paths
and hashes during implementation).

## Open decisions

- **[DECISION]** Test audio source: (a) record real ATC-style phrases on the user's mic,
  (b) generate synthetic clips via macOS `say` or Piper, (c) reuse audio fixtures from the
  existing `xp_welly_atc` repo if any exist. → propose after seeing the source repo.
- **[DECISION]** Should the CLI also accept stdin (raw PCM stream) for future plugin
  integration prototyping, or is file-based input enough for the spike? → file-based is
  simpler; recommend deferring streaming to milestone 06.

## Dependencies

- Milestone 01 complete (fork exists, repo is clonable).

## Out of scope

- VAD (voice activity detection) — that lives in the plugin, not in the spike.
- Streaming inference — file-based is sufficient for the latency proof.
- Anything beyond English.
