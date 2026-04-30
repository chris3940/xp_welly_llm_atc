# Milestone 05 — End-to-End CLI (STT → LLM → TTS)

## Goal

Stitch the three spikes together into a single CLI that takes a WAV recording of a pilot
call, transcribes it, runs it through the LLM with an ATC system prompt, synthesizes the
controller response, and writes it back to a WAV. Measure the full pipeline latency and
prove it stays under **3 s** for a typical ATC interaction on the M1.

## Deliverables

1. **Directory** `spikes/spike_e2e/` with its own `CMakeLists.txt` linking the three
   library wrappers built in milestones 02–04.
2. **CLI tool** `spike_e2e`:
   - Usage: `spike_e2e <pilot_call.wav> <out_response.wav>`
   - Stage 1: transcribe pilot call (whisper.cpp).
   - Stage 2: build a fixed ATC system prompt + transcribed user message, run llama.cpp.
   - Stage 3: synthesize the LLM output with Piper.
   - Stage 4: write WAV.
   - Prints a per-stage timing breakdown and a total wall-clock.
3. **Latency report** at `spikes/spike_e2e/RESULTS.md`:
   - Table with stage latencies and totals across at least 5 runs (cold + warm).
   - Identifies the bottleneck stage.
   - Lists which optimizations are still on the table for milestone 06.
4. **Go/no-go recommendation** at the top of the repo `README.md`:
   - Plain English: "Plugin integration recommended" or "Not recommended, here's why".

## Acceptance criteria

| Metric | Target |
|--------|--------|
| End-to-end wall-clock (warm, typical 5 s pilot call) | **< 3 s** |
| Output WAV is intelligible and contextually appropriate | qualitatively yes |
| All three stages reuse model handles across calls (no reload per request) | yes |

## Architecture notes

- Models load **once** at startup; the CLI accepts repeated stdin lines for `<wav-in> <wav-out>`
  pairs so we can measure warm latencies without process restart cost.
- No threading optimization yet — sequential stage execution. If the e2e total is close
  to 3 s, parallelizing TTS streaming with LLM token generation is a milestone-06 lever,
  not a spike-stage lever.
- The ATC system prompt lives in `spikes/spike_e2e/prompts/atc_system.txt` so it can be
  iterated without rebuilds.

## Open decisions

- **[DECISION]** Should the e2e CLI already implement a thin C++ abstraction
  (`ISpeechToText`/`ILanguageModel`/`ITextToSpeech`) so the plugin integration in
  milestone 06 can lift the same code? → recommend **yes**, this is the natural place
  to validate the Strategy interfaces from milestone 01.

## Dependencies

- Milestones 02, 03, 04 all complete and meeting their individual acceptance criteria.

## Out of scope

- VAD or microphone capture — that lives in the plugin.
- Multi-turn dialogue state — the spike is single-shot.
- ImGui or any GUI — CLI only.
