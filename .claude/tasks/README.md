# Milestones — xp_welly_llm_atc

Project goal: replace the cloud-based STT/LLM/TTS stack of `xp_welly_atc` with fully local
inference (whisper.cpp + llama.cpp + Piper) on Apple Silicon, statically linked into the plugin.

This directory tracks **milestones**, not day-to-day tasks. Each milestone has a clear goal,
deliverables, and exit criteria. Implementation only starts after explicit go from the user.

## Sequence

| # | Milestone | Status | File |
|---|-----------|--------|------|
| 01 | Fork, repo setup, architecture analysis | Not started | [milestone-01-fork-and-analysis.md](milestone-01-fork-and-analysis.md) |
| 02 | Spike: whisper.cpp standalone CLI | Not started | [milestone-02-spike-whisper.md](milestone-02-spike-whisper.md) |
| 03 | Spike: llama.cpp standalone CLI | Not started | [milestone-03-spike-llama.md](milestone-03-spike-llama.md) |
| 04 | Spike: Piper TTS standalone CLI | Not started | [milestone-04-spike-piper.md](milestone-04-spike-piper.md) |
| 05 | End-to-end CLI (STT → LLM → TTS) | Not started | [milestone-05-end-to-end-cli.md](milestone-05-end-to-end-cli.md) |
| 06 | Plugin integration (Strategy pattern) | Not started | [milestone-06-plugin-integration.md](milestone-06-plugin-integration.md) |

## Global constraints

- **Hardware target:** Apple Silicon M1, 32 GB RAM (shared with X-Plane 12, expect 8–16 GB
  consumed by the sim → budget ~16 GB headroom for our stack).
- **No cloud dependencies** in the final solution.
- **Static linking** of all inference libraries; no daemons, no helper apps, no Ollama.
- **Metal backend** for whisper.cpp and llama.cpp.
- **English only** (US ATC phraseology) — allows distil-whisper.en variants.
- **License:** MIT (avoid GPL deps to keep the plugin license clean).
- **Models are not committed** to the repo. Download URLs + SHA256 hashes documented in README.

## Definition of Done (whole spike)

The spike is successful (→ "go" for milestone 06) when:

1. All three standalone CLIs pass their per-milestone acceptance criteria on the M1.
2. The end-to-end CLI (milestone 05) completes a typical ATC interaction in **< 3 s** total.
3. The repo README documents build steps, model URLs/hashes, measured numbers, and a
   go/no-go recommendation for plugin integration.

## Decisions still open

Items that need user confirmation before the corresponding milestone starts are tagged
**[DECISION]** inside the individual milestone files.
