# Milestone 01 — Fork, Repo Setup, Architecture Analysis

## Goal

Create the fork `rwellinger/xp_welly_llm_atc` from `rwellinger/xp_welly_atc`, clone it into
this working directory, and produce a written analysis of the existing cloud-call sites
together with a refactoring proposal for the Strategy pattern.

## Deliverables

1. **Fork created** on GitHub via `gh repo fork rwellinger/xp_welly_atc --fork-name xp_welly_llm_atc --clone=false`.
2. **Local clone** in `/Users/robertw/Workspace/x-plane/xp_welly_llm_atc/` (flat layout — clone
   contents directly into the working directory, no nested folder).
3. **Updated `README.md`** in the fork: explain the origin, the new local-inference goal,
   and link to the milestones directory.
4. **Architecture analysis document** at `docs/architecture-analysis.md`:
   - Inventory of files/classes that today talk to OpenAI (STT, LLM, TTS wrappers).
   - Current call flow diagram (text-based, ASCII or Mermaid).
   - Proposed Strategy/Interface boundaries (`ISpeechToText`, `ILanguageModel`, `ITextToSpeech`)
     with method signatures and lifecycle (init/load/infer/shutdown).
   - List of files that need to change vs. files that stay untouched (especially the ATC
     state machine — must be on the "stay" list).
5. **Updated `.gitignore`**: ensure `Resources/models/` is ignored.

## Acceptance criteria

- Fork is visible at `https://github.com/rwellinger/xp_welly_llm_atc`.
- `git remote -v` in the local clone shows the fork URL as `origin`.
- `docs/architecture-analysis.md` is reviewed and approved by the user before any code
  refactoring starts.
- README mentions the project goal and links to `.claude/tasks/README.md`.

## Out of scope

- No actual refactoring of code in this milestone — analysis only.
- No model downloads, no spike code.

## Open decisions

- **[DECISION]** Should the Strategy interfaces live in a new `src/inference/` directory or
  stay in the existing wrapper locations? → propose during analysis, decide with user.
- **[DECISION]** Naming of the abstract interfaces — `IXxx` (Hungarian-ish), `XxxBackend`,
  or `XxxStrategy`? → propose during analysis.

## Dependencies

None. This is the entry milestone.

## Notes

- Branch strategy: work directly on `main` of the fork (per user instruction — fork is
  self-contained, original `xp_welly_atc` will be retired if this spike succeeds).
- Commit style: conventional commits (`feat:`, `chore:`, `docs:`, `refactor:`).
