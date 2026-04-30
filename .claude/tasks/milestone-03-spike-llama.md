# Milestone 03 — Spike: llama.cpp standalone CLI

## Goal

Prove that a 3 B-class instruction-tuned LLM running on llama.cpp with the Metal backend
can produce ATC phraseology with first-token latency < 500 ms, throughput > 30 tok/s, and
RAM consumption under 3 GB on the M1 — leaving headroom for X-Plane and the other
backends.

## Deliverables

1. **Directory** `spikes/spike_llama/` with its own `CMakeLists.txt`.
2. **llama.cpp** as a git submodule, pinned to a stable tag.
   Build with `LLAMA_METAL=ON`. Use the `common` and `llama` static libs only — no server,
   no examples.
3. **CLI tool** `spike_llama`:
   - Usage: `spike_llama <model.gguf> <prompt.txt>`
   - Loads the model, runs a single completion with a fixed sampling config (temp ~0.3,
     top_p ~0.9, max_tokens ~256).
   - Prints generated text + metrics: first-token latency, tokens/s, total wall-clock,
     peak RAM.
4. **Sample prompts** in `spikes/spike_llama/prompts/`:
   - At least three realistic ATC scenarios (taxi clearance, takeoff clearance, traffic
     advisory) with system prompt + user transcript that mimic what the state machine
     will pass into the LLM.
5. **README** at `spikes/spike_llama/README.md`:
   - Build instructions, model URL + SHA256, example invocation, measured numbers.
   - Snippets of generated output for each sample prompt so quality can be eyeballed.

## Acceptance criteria

| Metric | Target |
|--------|--------|
| First-token latency (after warm-up) | **< 500 ms** |
| Sustained throughput | **> 30 tok/s** |
| Peak RAM during inference | **< 3 GB** |
| Output quality on ATC prompts | recognizable phraseology, no obvious hallucinations |

## Model recommendation

**Primary:** `Llama-3.2-3B-Instruct-Q4_K_M.gguf` (~2.0 GB)
- Strong instruction following at this size class.
- Q4_K_M is the sweet-spot quantization for Metal (well-supported, low quality loss).
- License: Llama 3.2 Community License → permissive for our use, document in README.

**Alternatives to keep in mind if Llama 3.2 3B underdelivers on phraseology quality:**
- `Mistral-7B-Instruct-v0.3-Q4_K_M.gguf` (~4.4 GB) — better quality but eats into the
  X-Plane RAM budget. Apache-2.0 license (cleaner than Llama).
- `Qwen2.5-3B-Instruct-Q4_K_M.gguf` (~2.0 GB) — competitive with Llama 3.2 3B. Apache-2.0.

**Do not start with** 7 B+ models — RAM headroom is tight when X-Plane is running.

Source: `https://huggingface.co/bartowski/` (TBD: confirm exact paths and hashes).

## Open decisions

- **[DECISION]** Context window setting: 2048 / 4096 / 8192 tokens? Each step roughly
  doubles KV-cache RAM. Recommend **2048** for the spike (ATC turns are short); revisit
  in milestone 06 if dialogue history needs more.
- **[DECISION]** Should the spike already exercise grammar/JSON-constrained output via
  GBNF, or is free-form text enough? Recommend **free-form for the spike**; structured
  output is a milestone-06 concern.
- **[DECISION]** Prompt source: do we have canonical ATC prompts from the existing
  `xp_welly_atc` repo, or do I draft them from scratch? → check during milestone 01.

## Dependencies

- Milestone 01 complete.
- Independent of milestone 02 — can be built in parallel.

## Out of scope

- Function calling / tool use.
- Multi-turn conversation history management.
- Quality benchmarking beyond eyeballing the sample outputs.
