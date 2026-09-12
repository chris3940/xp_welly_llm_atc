# Welly's ATC — AI Voice ATC for X-Plane 12

> **Historical document.** This is the README of the plugin this project is
> built on: **Welly's ATC for macOS** by thWelly, an AI-powered VFR ATC plugin,
> archived 2026-09-06 at v4.3.1. It is kept because it still documents the
> shell this project inherits — installation, the model downloader, settings,
> and the VFR flows — but it describes the state at v4.3.1 and is not updated.
>
> **For the IFR plugin this repository actually is, read
> [README-IFR.md](README-IFR.md).**

![Welly's ATC panel with ATIS broadcast at LSZB Bern-Belp](images/atc-atis-example.jpg)

> **Cross-platform X-Plane 12 ATC plugin — runs on macOS, Windows and
> Linux. OpenAI Cloud, Mistral Cloud, or (Apple Silicon, from source)
> fully offline local inference. Your choice.**
>
> - **OpenAI Cloud (all platforms)** — Whisper API + Chat Completions +
>   TTS API. Bring your own API key (stored in the OS credential store:
>   macOS Keychain, Windows Credential Manager, or a 0600 file on Linux).
> - **Mistral Cloud (all platforms)** — Voxtral STT + Mistral Chat
>   Completions + Voxtral TTS. Bring your own API key (separate
>   credential entry so OpenAI and Mistral keys coexist). Cheaper per
>   token than OpenAI.
> - **Local, fully offline (Apple Silicon only, from source)** —
>   `whisper.cpp` (Metal) + `llama.cpp` (Metal) + Piper TTS, no daemons,
>   no helper apps, no API keys. **Not in the pre-built releases** (those
>   are cloud-only on every platform so CI stays fast) — enable it by
>   building from source on an Apple Silicon Mac (`make build`, see
>   [Build From Source](#build-from-source)).
>
> The **pre-built releases are cloud-only** on all three platforms: a
> macOS universal `.xpl` (arm64 + x86_64), a Windows `win_x64` `.xpl`,
> and a Linux `lin_x64` `.xpl`. X-Plane loads whichever matches the host;
> you pick OpenAI or Mistral at runtime in Settings. Platform install
> notes: [README-WINDOWS.md](README-WINDOWS.md) ·
> [README-LINUX.md](README-LINUX.md).
>
> The spike-phase architecture and per-backend measurements are archived in
> [`docs/architecture-analysis.md`](docs/architecture-analysis.md) and
> [`spikes/spike_e2e/RESULTS.md`](spikes/spike_e2e/RESULTS.md).
>
> **Measured pipeline latency** (warm, M4, local inference, end-to-end spike):
> STT 321 ms · LM 634 ms · TTS 200 ms · **total ≈ 1.16 s per request** —
> well under the 3 s acceptance target with > 1.8 s of headroom for the
> M4-vs-M1 generational gap and the plugin's main-thread / Core Audio
> overhead. Cloud modes (OpenAI / Mistral) are typically slower: 2–3 s
> warm round-trip dominated by API latency. M1 local re-validation:
> pending real-flight smoke test.

> **Note — German VFR is moving to its own plugin.** This plugin is
> English-only (EU / US ICAO-FAA phraseology). A dedicated **German
> VFR** plugin (NfL / BZF DACH phraseology) is in the works and will
> ship separately.

---

AI-powered ATC voice communication plugin for X-Plane 12 VFR flights.

Talk to ATC using your microphone via push-to-talk. The plugin
transcribes your speech (locally with whisper.cpp, via the OpenAI
Whisper API, or via Mistral's Voxtral STT — your pick), interprets
your intent through a rule-based ATC state machine — with a
low-confidence fallback to a local Llama 3.2 3B classifier, OpenAI's
`gpt-4o-mini`, or Mistral Small, again your pick — and plays back ATC
responses synthesised locally with Piper or via OpenAI's / Mistral's
TTS API.

## Table of Contents

- [What's New 4.2.1](#whats-new-421)
- [Features](#features)
- [Hardware Requirements](#hardware-requirements)
- [Software Requirements](#software-requirements)
- [IFR ATC — What's Included](#ifr-atc--whats-included)
- [IFR ATC — Data Requirements](#ifr-atc--data-requirements)
- [Quick Start](#quick-start-pre-built-release)
- [Backend Modes](#backend-modes)
- [Radio glitch recovery (TTS-failure guard)](#radio-glitch-recovery-tts-failure-guard)
- [Build From Source](#build-from-source)
- [Local Inference Models](#local-inference-models)
- [Configuration](#configuration)
- [Usage](#usage)
- [Make Targets](#make-targets)
- [Known Limitations](#known-limitations)
- [FAQ](#faq)
- [Project Structure](#project-structure)
- [Third-Party Dependencies](#third-party-dependencies)
- [Development Workflow](#development-workflow)
- [License](#license)
- [Flight schools and commercial training](#flight-schools-and-commercial-training)

## What's New 4.2.1

Release 4.2.1 is an IFR-focused maintenance release. It hardens the
en-route / approach handoff chain, tightens frequency gating so a
check-in is only accepted on the correct sector frequency, fixes several
approach + landing phraseology gaps (CIFP-assigned runway, spoken airport
name, unit-consistent altitude handling), adds an IFR flight-plan-closure
flow, and improves taxi-exit detection and STT biasing. IFR remains
EU-profile only and under active development (see
[IFR ATC — What's Included](#ifr-atc--whats-included)).

Each block below opens with a short pilot-facing summary; the bullets
underneath carry the implementation detail (internal symbol / state names)
for contributors.

### Handoff / frequency-gate fixes

Sector and frequency handoffs are now consistent: ATC accepts your
check-in only on the new sector's frequency, does not re-announce a
frequency you are already tuned to, and always acknowledges the first
call on a new frequency instead of clearing you silently.

- En-route sector-change now updates `s_enroute_approach_freq_mhz` when
  the new sector is a TRACON, so the check-in handler no longer accepts a
  call on the old sector's frequency (the unknown/training fallback was
  previously accepting any frequency).
- Sector-change suppresses the "contact X on Y" announcement when the
  pilot's active COM already matches the new sector frequency (aircraft
  re-entering an earlier sector). Applies to both the en-route and the
  approach sector-change paths.
- `check_handoff_reissue` extended to `IFR/DESCENT`,
  `IFR/APPROACH_CONTACT`, `IFR/APPROACH_DESCENT`, `IFR/APPROACH_TOWER`,
  `IFR/LANDING_CLEARED`.
- Frequency-based intent promotion accepts `INITIAL_CALL_CENTER` as an
  approach check-in variant.
- FAF Tower/AFIS handoff updates `s_current_controller_label` and
  `s_pending_handoff_freq_mhz` so the next TTS speaker prefix matches the
  new facility.
- The sector-checkin path defers to the richer check-in handlers when the
  pilot's frequency matches `s_enroute_approach_freq_mhz` — a full
  approach clearance instead of a bare "radar contact" ack.
- Sector-checkin ack is mandatory: the silent clear was removed. ATC now
  replies "radar contact" (or a full clearance) when the pilot first
  calls on the new frequency.

### Readback verifier

A misclassified readback no longer poisons the next transmission with a
spurious "negative" correction.

- Post-hooks silently clear stale readback state when the LM
  misclassifies a valid readback as `INITIAL_CALL_CENTER` /
  `INITIAL_CALL_APPROACH` — this stops the next unrelated readback from
  tripping "negative, flight level seven zero".

### Approach + landing phraseology

Approach and landing clearances are more forgiving of garbled facility
names and now issue the correct, CIFP-assigned runway with the spoken
airport name.

- The approach check-in state gate accepts `IFR/APPROACH_CONTACT` and
  `IFR/DESCENT`.
- Broadened check-in intent set: any `INITIAL_CALL_*` variant or
  `UNKNOWN` fires the full clearance when the pilot is on the approach
  frequency — frequency is authoritative over the spoken facility name.
- Broadened Tower/AFIS check-in similarly — "Reims Prunay Information" and
  Voxtral-garbled forms trigger the landing clearance.
- Landing clearance uses the CIFP-assigned runway
  (`set_assigned_runway`), so Tower issues RWY 07 rather than the
  wind-favoured RWY 29 on calm days.
- The airport name (e.g. "Reims Prunay Information") replaces the ICAO
  code ("LFQA Information") in every spoken controller label and handoff
  phrase.
- `initial_ft` comparisons switched to unit-consistent references: FL vs
  pressure altitude, feet (QNH) vs altitude MSL. The same branching is
  applied to `no_descent_needed`, the floor-only skip, and the
  initial-target loops.
- "Continue descent to X" fires only when Approach's target equals
  Centre's last cleared altitude; a new (lower) target uses "descend X".

### Route + IAF handling

Direct-to-IAF clearances no longer leave the route tracker stuck at the
dual-use fix on RNAV approaches.

- Restored step 4 in `init_route_fixes`: a direct-to-IAF jumps the
  tracker to that fix so the guard clears at the dual-use IAF/MAP-hold
  pattern.
- The `at_faf` primary check is suppressed when
  `s_iaf_route_idx > s_faf_route_idx` (LFQA RNAV R07/R25 dual-use fix
  pattern).

### IFR flight plan closure

After landing, ATC now tells you how to close the IFR flight plan and
keeps you in the landing state until you have read back the taxi
instruction.

- New handler in `IFR/LANDING_CLEARED` distinguishes AFIS vs towered:
  - AFIS → "leaving frequency approved, contact by telephone to close IFR
    flight plan, good day."
  - Towered → "IFR flight plan closed at HHMM, good day."
- `RUNWAY_VACATED_TOWER_ONLY` now stays in `IFR/LANDING_CLEARED` (no
  premature `IDLE`) and requires readback of the taxi + report-on-stand
  instruction.

### Taxi exit detection

The runway-exit / taxi phrase now names the taxiway you are physically
on, and falls back to "to the apron" when you are clear of the graph.

- `nearest_taxiway_phrase` now uses point-to-segment distance against
  every taxiway edge (from the apt.dat 1202 records) instead of the
  closest midpoint, and returns "to the apron" when the aircraft is more
  than 40 m from any named edge. It picks the taxiway the aircraft is
  actually on.

### STT bias

Speech-to-text is biased toward the runway and callsign in play, so RNAV
runway idents and your tail number transcribe more reliably.

- Dynamic "R-NAV NN" + "RNAV NN" bias emitted per assigned landing
  runway.
- Callsign last-2-letters bigram (e.g. "Romeo Charlie") appended to the
  prompt.
- "arm of", "r nav", "r-nav" → "rnav" added to `kPhraseAliases`.

### Verify-descending prompt

ATC now gives a gentle "confirm descending" nudge before the harder
altitude-deviation warning if you are not yet coming down.

- Sub-phase 2.4 fires "confirm descending X" 45-100 s after a clearance
  when `|VS| < 200 fpm` and the aircraft is 500-800 ft off target —
  ahead of the harder "check altitude" deviation warning.

### Altitude deviation warning

The altitude-deviation warning now compares against the right reference
(pressure altitude above the transition altitude, QNH altitude below it)
and uses matching phraseology.

- The comparison branches on the transition altitude: FL clearances
  compare against `ctx.pressure_alt_ft` (ISA), feet clearances against
  `ctx.altitude_ft_msl` (QNH).
- Phraseology "assigned altitude N feet" for feet clearances; "assigned
  flight level N" for FL clearances.

## Features

- **Push-to-Talk** — bound via X-Plane command binding (keyboard or joystick)
- **Triple-backend inference** — pick **OpenAI Cloud** or **Mistral
  Cloud** (all platforms, BYO API key), or **Local** (Apple Silicon
  only, from-source build) in the Settings tab. Switch at runtime, no
  plugin restart. Every
  inference call is tagged with `[STT-LOCAL]` / `[STT-OPENAI]` /
  `[STT-MISTRAL]` (and equivalent for LM/TTS) in X-Plane's `Log.txt`
  so you can audit which side served each request.
- **Local Speech-to-Text** — `whisper.cpp` `small.en-q5_1`, Metal-accelerated
- **Local LLM** — `llama.cpp` running Llama 3.2 3B Instruct (Q4_K_M),
  Metal-accelerated; used for intent disambiguation when the rule-based
  parser is uncertain. Repair output is digit-validated to suppress
  hallucinated runways or frequencies.
- **Local Text-to-Speech** — Piper, neutral US accent (`en_US-lessac-medium`),
  CPU + onnxruntime
- **OpenAI Cloud option** — `whisper-1` for STT, `gpt-4o-mini` for the
  intent classifier (JSON-mode for constrained output), `tts-1` with
  six selectable voices (`alloy/echo/fable/onyx/nova/shimmer`; `onyx`
  is closest to real ATC). Key stored in the macOS Keychain via
  `Security.framework`, never in `settings.json`, never logged in full
  (only the last 4 characters appear in audit lines).
- **Mistral Cloud option** — `voxtral-mini-2507` for STT (with
  `context_bias[]` airport biasing), `mistral-small-latest` for the
  intent classifier (JSON-mode), `voxtral-mini-tts-2603` with 30
  preset voices across British, American and French speakers in 7-9
  emotional registers (default per role: `gb_oliver_neutral` for ATIS,
  `en_paul_confident` for Tower, `en_paul_neutral` for Ground —
  British neutral reads closest to ICAO broadcast cadence). Separate
  Keychain entry so the OpenAI and Mistral keys coexist; switching
  modes never requires re-pasting.
- **ATC State Machine** — VFR phraseology for towered and non-towered airports
- **Flight Phase Detection** — context-aware guards prevent unrealistic ATC
  interactions based on aircraft state (parked, taxi, airborne, etc.)
- **Live Traffic Awareness (v2.1) + Landing Sequencing (v2.2)** —
  provider-agnostic `sim/cockpit2/tcas/targets/...` reader feeding a
  2 Hz `TrafficContext` snapshot. EU-phraseology en-route advisories
  with voice acknowledgement (`"Traffic in sight"` / `"Negative
  contact"` / `"Looking"`) on a side-channel that does not interfere
  with the main ATC flow. **v2.2 adds VFR landing sequencing** —
  *"number N, follow the [type] on left base, cleared to land runway X"*
  when other traffic is on Final or Pattern, *"continue approach,
  traffic on the runway"* when the active runway is blocked, and an
  unsolicited Tower-issued go-around within 1 NM of the threshold when
  the runway stays occupied. Master switch `traffic_features_enabled`
  in Settings turns the whole subsystem off in one click.
- **ATIS Generation** — automatic ATIS broadcasts from live sim weather
  data, on COM1 *or* COM2 (active or standby). Aborts mid-broadcast if
  the pilot retunes the COM that's playing.
- **Radio discipline coaching** — ATC issues a polite reminder when the pilot
  uses inappropriate language, escalating on repeats
- **Phraseology Hints** — context-aware cheat sheet with full ICAO phraseology
  on hover
- **Cross-Country Support** — full VFR departure, en-route frequency change,
  and inbound flow between airports. Approach controller proactively
  hands off to Tower with the destination frequency.
- **Aircraft registration display** — pilot callsign linked to the
  cockpit's actual tail number read from X-Plane
- **"Disregard" recovery** — flow-aware reset (PATTERN_ENTRY when
  airborne near the home airport, EN_ROUTE in transit, IDLE on the
  ground)
- **TTS-failure recovery (radio glitch guard)** — when speech synthesis
  or playback fails (OpenAI timeout, dropped network, Piper IO error),
  the plugin does not strand the pilot in a state the tower never
  actually announced. A snapshot of the ATC state machine is taken
  before each pilot transmission; on failure the plugin plays a short
  in-process squelch burst on the active COM and either rolls the
  state back ("re-issue your call") or — if an auto-correction has
  moved things on in the meantime — keeps the unsent clearance
  accessible via `REQUEST_REPEAT` ("Say again").
  See [Radio glitch recovery](#radio-glitch-recovery-tts-failure-guard).
- **Radio Power Awareness** — ATC panel disables when COM radio has no
  electrical power, with optional bypass for exotic aircraft
- **In-plugin model downloader** — first launch surfaces an ImGui dialog,
  HTTPS-resumable downloads from HuggingFace, SHA256-verified before use
- **ImGui UI** — in-sim ATC panel with frequency management, phraseology
  hints, transcript history, a Models tab for download / re-verify, and
  an optional Traffic tab (debug) listing the 10 nearest aircraft
- **Debug text input (type instead of speak)** — optional InputText
  field below the transcript in the Status tab. Settings toggle
  `debug_text_input` (default off). Skips STT and feeds the typed
  string straight into the engine; LM, state machine and TTS remain
  identical to the voice path, so the Tower reply is still spoken
  through the active backend's TTS. Useful on laptops without a
  headset and for isolating ATC-logic bugs from STT misrecognitions.
  PTT stays functional in parallel; shortcut `REG` expands to the
  pilot's phonetic callsign.
- **Editable models catalog** — `data/models_catalog.json` is the
  single source of truth for every selectable model slug and voice
  across all three backends (OpenAI STT/LM/TTS + voices, Mistral
  STT/LM/TTS + voices, local Piper voice catalog with URL / size /
  SHA256). The Settings dropdowns and the Models-tab download list
  are driven by it. Add a new OpenAI / Mistral slug or a new Piper
  voice by editing the JSON and restarting X-Plane — no recompile.

## Hardware Requirements

Pre-built releases are **cloud-only** on every platform (OpenAI or
Mistral). X-Plane automatically loads the slice that matches the host.

| Platform | Slice | Release backends | Local (offline) inference |
|---|---|---|---|
| macOS Apple Silicon (M1–M4) | `mac_x64` universal `.xpl` (arm64) | OpenAI / Mistral Cloud | **Yes — from source only** (`make build`; Metal whisper.cpp + llama.cpp + Piper) |
| macOS Intel | `mac_x64` universal `.xpl` (x86_64) | OpenAI / Mistral Cloud | No (needs Metal + Apple Silicon) |
| Windows 10 / 11 (x64) | `win_x64` | OpenAI / Mistral Cloud | No |
| Linux (x86_64) | `lin_x64` | OpenAI / Mistral Cloud | No in the release¹ |

> **Local inference is Apple Silicon only** and is **not** shipped in the
> pre-built releases — they are cloud-only on all platforms so CI stays
> fast (the whisper/llama/Piper compile is the long pole). To run fully
> offline (Metal-accelerated), build from source on an Apple Silicon Mac —
> see [Build From Source](#build-from-source).
>
> ¹ The Linux code path *can* build CPU-only local inference from source
> (see [README-LINUX.md](README-LINUX.md)); it is simply not part of the
> cloud-only release bundle.

| Resource | Cloud mode (all platforms) | Local mode (Apple Silicon, from source) |
|---|---|---|
| RAM | 16 GB (no model in RAM — stateless HTTP requests) | 32 GB recommended (X-Plane 12 + ~3 GB for the inference stack) |
| Disk | ~50 MB for the plugin bundle (no models downloaded) | ~2.5 GB free for the models |
| GPU | not used | Metal GPU on the Apple Silicon chip |
| Network | required — every PTT release triggers HTTPS to `api.openai.com` or `api.mistral.ai` | not used at runtime (one-time model download from HuggingFace) |

Both cloud modes cost money per request (STT + LM + TTS APIs).
Mistral is typically cheaper per token than OpenAI (`mistral-small`
≈ 33 % cheaper input / 50 % cheaper output than `gpt-4o-mini`). STT
and TTS are roughly at price parity. Latency for both clouds is
typically 2–3 s warm vs. 1–1.5 s warm for local inference. Plan
accordingly.

## Software Requirements

| Item | Requirement |
|---|---|
| macOS | **13.3 or later** (a from-source local build needs onnxruntime 1.22.0; the x86_64 slice inherits the same deployment target so the lipo'd binary stays consistent) |
| Windows | **Windows 10 / 11 (x64)** — cloud-only. See [README-WINDOWS.md](README-WINDOWS.md). |
| Linux | **x86_64**, X-Plane 12, PulseAudio/PipeWire — cloud-only in the release. See [README-LINUX.md](README-LINUX.md). |
| X-Plane | X-Plane 12 (12.0 or later) |
| OpenAI / Mistral account | Required for the cloud modes (i.e. for every pre-built release) — an API key with billing enabled for the respective provider. Only a from-source Apple Silicon local build has no cloud dependency. |
| For building from source (macOS) | CMake 3.26+, Homebrew LLVM (`brew install llvm`), Xcode Command Line Tools. Windows builds via MSVC in CI; Linux via GCC/Clang — see the per-platform READMEs. |

## IFR ATC — What's Included

> **EU profile only.** IFR is gated to the EU profile — in the US
> profile the flow is never entered (IFR-only intents are stripped at the
> state-machine entry). The feature is under active development; some
> departure-clearance flows are still being refined.

### Departure / Ground

- Pre-departure clearance: ATIS challenge, squawk assignment, SID, initial altitude
- Holding point name resolved from the apt.dat taxiway graph
- Line-up-and-wait → takeoff flow
- Squawk verify at the holding point (transponder code + Mode C)
- Tower → Departure frequency handoff

### CIFP integration

- SID selected by matching the SimBrief FPL first fix to the CIFP SID last waypoint
- Real SID initial altitude from CIFP (fallback: apt.dat 1302 `transition_alt`)
- Runway binding, calm-wind selection, reciprocal fallback

### SimBrief OFP

- Async fetch + full navlog parsing (ident, airway, lat/lon, altitude, SID/STAR flag)
- IFR tab in the ImGui panel: scrollable waypoint list, SID/STAR fixes dimmed

### OpenAir airspace DB

- Reads `airspace.txt`, Class A–G polygons with floor/ceiling
- 3D `find_enclosing(lat, lon, alt_ft)` drives TMA/CTR/FIR boundary detection

### En-route

- Centre check-in after TMA exit (real name + frequency from atc.dat CTR)
- Direct-to shortcut to the first en-route fix > 20 NM (~90–120 s after check-in)
- Sector/FIR frequency-change detection (Marseille sectors, Bordeaux FIR)
- FL altitude-deviation warning (±200 ft RVSM ≥ FL290 / ±300 ft below)
- TMA entry descent + Approach handoff

## IFR ATC — Data Requirements

IFR clearances (SID assignment, initial climb altitude, CTR departure handoff)
depend on several external data sources.  Some are bundled with X-Plane;
others must be installed separately.

### Bundled with X-Plane 12 (no action needed)

| File | Location | Used for |
|---|---|---|
| **Global Airports apt.dat** | `X-Plane 12/Global Scenery/Global Airports/Earth nav data/apt.dat` | Airport frequencies, runway geometry, `transition_alt` (1302 row) |
| **CIFP procedures** | `X-Plane 12/Custom Data/CIFP/{ICAO}.dat` | SID names for the departure runway, initial climb altitude, binding minimum altitude (e.g. `LFLP.dat`, `LSZH.dat`) |

If you have a **custom scenery package** for the departure airport
(e.g. `Custom Scenery/LFLP/Earth nav data/apt.dat`), the plugin also reads its
`transition_alt` row as a fallback when the Global Airports file does not
carry one.

### Must be installed separately

#### OpenAir airspace file (required for CTR departure handoff)

The plugin reads `X-Plane 12/Custom Data/airspaces/airspace.txt` in OpenAir
format to determine the CTR ceiling of the departure airport.  When the
aircraft climbs above that ceiling, the plugin fires the frequency handoff
to Departure/Approach.  Without this file the handoff fires at a generic
2 500 ft AGL fallback.

Recommended source: export your region's CTR boundaries from
[Little Navmap](https://albar965.github.io/littlenavmap.html)
(*File → Export → Export as OpenAir*) or download a pre-built file for
your country (e.g. `europe_p.txt` from the SkyVector OpenAir repository).
Rename / copy the result to:

```
X-Plane 12/Custom Data/airspaces/airspace.txt
```

#### SimBrief OFP (required for correct SID selection)

The plugin fetches your SimBrief Operational Flight Plan via the built-in
SimBrief panel in the ATC window.  The OFP provides:

| Field | How it is used |
|---|---|
| **Destination ICAO** | Spoken in the clearance — "cleared to {destination}" |
| **Route (first fix)** | Identifies the correct SID: the first waypoint in the route is the last fix of the SID.  The plugin looks up the matching SID in the CIFP file for the departure runway (e.g. route starts with `ODIKI` → SID = `ODIK2A`). |
| **Cruise altitude** | Used for the SID climb advisory ("continue climb FL270") |

Without a SimBrief OFP the plugin falls back to:
- Destination: FMS destination entry (if a flight plan is loaded in X-Plane)
- SID: alphabetically first SID for the active runway in the CIFP file
- Cruise altitude: not announced

### Summary checklist before an IFR departure

- [ ] X-Plane 12 is up to date (CIFP and Global Airports present)
- [ ] `X-Plane 12/Custom Data/airspaces/airspace.txt` installed (OpenAir CTR data)
- [ ] SimBrief OFP fetched for the current flight (correct SID + destination)
- [ ] Active COM radio tuned to the correct IFR frequency (Delivery or Tower)
- [ ] ATIS information letter noted (required in the clearance request at towered airports)

## Quick Start (pre-built release)

1. Download `xp_wellys_atc.zip` from the GitHub Releases page. One
   cloud-only bundle carries the macOS universal slice (arm64 + x86_64),
   the Windows `win_x64` slice, and the Linux `lin_x64` slice — X-Plane
   loads whichever matches the host.
2. Extract into `X-Plane 12/Resources/plugins/`. Result (cloud-only —
   each `.xpl` is self-contained, no model files or dylibs):
   ```
   X-Plane 12/Resources/plugins/xp_wellys_atc/
     ├── mac_x64/xp_wellys_atc.xpl       (universal: arm64 + x86_64)
     ├── win_x64/xp_wellys_atc.xpl       (Windows x64)
     ├── lin_x64/xp_wellys_atc.xpl       (Linux x86_64)
     └── data/
           └── (ATC profile bundles, prompt templates, VRP database, etc.)
   ```
   > A **from-source Apple Silicon local build** additionally carries
   > `mac_x64/libpiper.dylib` + `libonnxruntime*.dylib`, a
   > `Resources/espeak-ng-data/` folder, and (downloaded on first launch)
   > `Resources/models/`. The pre-built release has none of these.
3. Launch X-Plane. Open the plugin window via *Plugins → Welly's ATC*.
4. **Pick your backend** in the **Settings** tab (a pre-built release
   offers OpenAI and Mistral; **Local** appears only in a from-source
   Apple Silicon build):
   - **OpenAI Cloud** (all platforms): paste your OpenAI API key into the
     **OpenAI API Key** field in Settings (use the `[Paste]` button —
     Cmd+V inside X-Plane's ImGui context is unreliable). Click
     **Save Key**. The key is stored in the OS credential store under
     service `com.xp_wellys_atc.openai` (macOS Keychain / Windows
     Credential Manager / 0600 file on Linux). PTT is enabled
     immediately; no model download.
   - **Mistral Cloud** (all platforms): paste your Mistral API key into the
     **Mistral API key** field in Settings (same `[Paste]` button
     pattern). Click **Save Key##mistral**. The key is stored under a
     separate credential entry `com.xp_wellys_atc.mistral`, so the
     OpenAI key (if any) stays untouched and you can flip-flop
     between providers without re-pasting. PTT is enabled
     immediately; the three voice slots default to ICAO-friendly
     British / American Voxtral preset voices and can be changed in
     the same panel.
5. Fly. The Status tab's banner will tell you which mode is active and
   `Log.txt` carries a one-line `BACKEND MODE: ...` banner on every
   load so you can prove after the fact which side served the session.

## Backend Modes

You can switch at any time in the Settings tab — the plugin tears
down the active inference stack and brings up another one, no X-Plane
restart. Source-level invariant: each backend family lives in its
own set of `.cpp` files and the three families share no headers and
no code path. The local backends (`whisper_stt.cpp`, `llama_lm.cpp`,
`piper_tts.cpp`) contain no `#include` of any cloud client and zero
`curl_easy_perform` calls; the OpenAI clients (`openai_stt.cpp`,
`openai_lm.cpp`, `openai_tts.cpp`) contain no `#include` of
`whisper.h` / `llama.h` / `piper.h` and none of the Mistral
endpoints; the Mistral clients (`mistral_stt.cpp`, `mistral_lm.cpp`,
`mistral_tts.cpp`) carry neither local headers nor `api.openai.com`.
So in any one mode no code path can call into the other two —
verified at compile time and grep-time by
`tests/test_audit_logging.cpp`.

Auditing which mode served a request: grep `Log.txt`.

| Tag in `Log.txt` | What it means |
|---|---|
| `[xp_wellys_atc] BACKEND MODE: LOCAL ...` | Loader brought up the local pipeline. |
| `[xp_wellys_atc] BACKEND MODE: OPENAI (api.openai.com) ...` | Loader brought up the OpenAI cloud pipeline. |
| `[xp_wellys_atc] BACKEND MODE: MISTRAL (api.mistral.ai) ...` | Loader brought up the Mistral cloud pipeline. |
| `[STT-LOCAL] / [LM-LOCAL] / [TTS-LOCAL]` | Per-call audit for each local inference. |
| `[STT-OPENAI] / [LM-OPENAI] / [TTS-OPENAI]` | Per-call audit for each OpenAI cloud inference. API key is truncated to its last 4 chars (`sk-...ABCD`). |
| `[STT-MISTRAL] / [LM-MISTRAL] / [TTS-MISTRAL]` | Per-call audit for each Mistral cloud inference. API key is truncated to its last 4 chars (`...ABCD`; no `sk-` prefix — Mistral keys are not OpenAI-formatted). |

## Radio glitch recovery (TTS-failure guard)

The pilot-driven TTS path is wrapped in a snapshot/revert guard so a
synthesis or playback failure (OpenAI `curl error: Timeout`, transient
5xx, dropped Wi-Fi, local Piper IO error, audio bus glitch) cannot
leave the ATC state machine ahead of what the pilot actually heard.
The mechanic is uniform across both backend modes — the same code path
handles Local and Cloud.

How it works:

- Before each pilot transmission goes into `atc_state_machine::process()`,
  the plugin captures an opaque snapshot of the full machine state
  (current state, transition history, runway lock, readback flag,
  departure type, last clearance text, last tower utterance). A
  monotonic generation counter is bumped on every semantic mutation —
  per-frame heartbeats (timestamps, auto-correction timers) are not
  counted so they cannot invalidate the snapshot.
- On TTS success: nothing else happens. The state advances as before,
  the pilot hears the reply, the snapshot is dropped.
- On TTS failure: a short squelch burst (~350 ms pink noise plus a
  click) is played on the active COM. The burst is generated
  in-process from a deterministic seeded PRNG — it cannot fail in the
  same way the TTS call just did, and it works in VR or under the
  IFR-hood when the panel is not in view. Then one of two branches
  runs:
  - **Restore branch** — no third party mutated the state machine in
    the meantime. The pre-transmission snapshot is restored, the
    transcript panel shows a dim-amber System entry `Radio failure -
    please repeat your transmission`, and the pilot can re-issue
    the same call cleanly.
  - **Stale branch** — a later auto-correction (or another callback)
    already advanced the generation counter past the snapshot's
    expected value. Rolling back would silently undo that legitimate
    transition, so the rollback is rejected. The clearance text the
    pilot never heard is still parked in `last_tower_response_text_`,
    a System entry `Radio failure - say 'say again' for the missed
    instruction` steers the pilot toward the
    `REQUEST_REPEAT` path, which replays the missed clearance
    verbatim. After the replay the pilot reads back normally and the
    state machine re-synchronises.

ATIS broadcasts, traffic advisories, and the unsolicited go-around
prompt use the unguarded TTS path — they are stateless render-only
events. If a tick drops, the next tick simply tries again.

Implementation:

- `src/atc/atc_state_machine.{hpp,cpp}` — `AtcStateSnapshot`,
  `capture_snapshot()`, `current_gen()`, `restore_snapshot_if_gen()`,
  generation-counter discipline (banner comment in the cpp file
  spells out which fields bump gen and which are heartbeat-only).
- `src/atc/atc_session.cpp` — `speak_response_guarded()` wraps the
  `engine::process_transcript` callback for state-mutating tower
  replies.
- `src/audio/audio_player.{hpp,cpp}` — `play_squelch_burst(com)`, no
  WAV asset, no network.
- `tests/test_state_revert_guard.cpp` — four behavioural cases:
  snapshot+restore round-trip, generation monotonicity, stale-branch
  rejection, `REQUEST_REPEAT`-after-stale recovery.

## Build From Source

This is the **macOS local build** (Apple Silicon): it compiles the
whisper.cpp / llama.cpp / Piper backends so the plugin can run fully
offline. For **Windows** and **Linux** — both cloud-only — follow
[README-WINDOWS.md](README-WINDOWS.md) and [README-LINUX.md](README-LINUX.md)
(Windows builds via MSVC in CI; Linux via `make build`).

```sh
git clone --recurse-submodules <repo-url>
cd xplane-ifr-atc
make setup     # X-Plane SDK, Dear ImGui, nlohmann/json, Catch2, spike submodules
make build     # Universal Release build → build/xp_wellys_atc.xpl (arm64
               # with all three backends + x86_64 cloud-only, lipo'd into
               # one .xpl). This is the local-inference build; the
               # pre-built releases are cloud-only (make setup-cloud).
make install   # Code-sign + install to X-Plane plugins directory
```

`make build` runs CMake twice (arm64 with `XPWELLYS_USE_LOCAL_INFERENCE=ON`
in `build-arm64/`, x86_64 with the same flag `OFF` in `build-x86_64/`)
and `lipo`-merges the two `.xpl`s into one universal binary. Build
time is roughly double a single-arch build; that is the deliberate
trade-off so dev and release artifacts are byte-for-byte identical in
shape. For tag-driven release builds pass `RELEASE_FLAG=-DRELEASE=ON`
(`make release-build` does this for you — embeds the version from
`VERSION.txt`).

The build downloads onnxruntime's prebuilt arm64 dylib (≈ 33 MB) into
`spikes/spike_piper/third_party/piper1-gpl/libpiper/lib/` on first
configure. After that everything is local. The x86_64 slice has no
onnxruntime / Piper / whisper / llama dependency at all — it links
only against libcurl + the system frameworks (Security, AudioToolbox,
etc.) and the OpenAI clients.

## Local Inference Models

The plugin ships **without** the model files (~2.0 GB combined). They live
under `<plugin>/Resources/models/` and are downloaded on first launch via
the **Models** tab. Each download is HTTPS, resumable (`Range` header),
streamed directly to the install volume (no temp roundtrip via the system
disk — important for users running X-Plane on an external SSD), and
SHA256-verified before being renamed from `<file>.part` to its final
filename.

### Manual fallback (restrictive networks)

If the in-plugin downloader cannot reach HuggingFace (corporate proxy,
captive portal, etc.), download these files manually and drop them into
`<plugin>/Resources/models/`. The plugin re-verifies on the next launch
and loads them automatically if the hashes match.

| Model | Lang | Size | SHA256 | URL |
|---|---|---:|---|---|
| `ggml-small.en-q5_1.bin` | en | 181 MB | `bfdff4894dcb76bbf647d56263ea2a96645423f1669176f4844a1bf8e478ad30` | [`huggingface.co/ggerganov/whisper.cpp/resolve/main/ggml-small.en-q5_1.bin`](https://huggingface.co/ggerganov/whisper.cpp/resolve/main/ggml-small.en-q5_1.bin) |
| `Llama-3.2-3B-Instruct-Q4_K_M.gguf` | — | 1.88 GB | `6c1a2b41161032677be168d354123594c0e6e67d2b9227c84f296ad037c728ff` | [`huggingface.co/bartowski/Llama-3.2-3B-Instruct-GGUF/resolve/main/Llama-3.2-3B-Instruct-Q4_K_M.gguf`](https://huggingface.co/bartowski/Llama-3.2-3B-Instruct-GGUF/resolve/main/Llama-3.2-3B-Instruct-Q4_K_M.gguf) |
| `en_US-lessac-medium.onnx` | en | 60 MB | `5efe09e69902187827af646e1a6e9d269dee769f9877d17b16b1b46eeaaf019f` | [`huggingface.co/rhasspy/piper-voices/resolve/main/en/en_US/lessac/medium/en_US-lessac-medium.onnx`](https://huggingface.co/rhasspy/piper-voices/resolve/main/en/en_US/lessac/medium/en_US-lessac-medium.onnx) |
| `en_US-lessac-medium.onnx.json` | en | 4.9 KB | `efe19c417bed055f2d69908248c6ba650fa135bc868b0e6abb3da181dab690a0` | [`huggingface.co/rhasspy/piper-voices/resolve/main/en/en_US/lessac/medium/en_US-lessac-medium.onnx.json`](https://huggingface.co/rhasspy/piper-voices/resolve/main/en/en_US/lessac/medium/en_US-lessac-medium.onnx.json) |

Lang column: `en` files are required for the EU/US ATC profiles. Llama
is multilingual and shared. The Models tab
filters rows by `settings::backend_language()` by default and exposes
a **Show all languages** toggle for power users who want to keep extra
voices on disk.

After dropping the files in, reopen the plugin window — the Models tab
runs SHA256 verification in the background and flips the rows to **Ready**
once each hash matches.

### Expected first-run download time

5–30 minutes on typical home internet; the bottleneck is HuggingFace's
download throughput, not the plugin. The downloader resumes via HTTP
`Range` if your link drops, so a Wi-Fi blip mid-Llama does not restart
the 1.88 GB pull from scratch.

## Configuration

Settings live in `<plugin>/data/settings.json`. The OpenAI and Mistral
API keys are the only secrets — both live in the macOS Keychain under
separate service entries (`com.xp_wellys_atc.openai`,
`com.xp_wellys_atc.mistral`), never in this file.

| Setting | Default | Description |
|---|---|---|
| `pilot_callsign` | *(empty)* | Phonetic callsign (set in plugin settings, written from the registration via ICAO conversion) |
| `active_com` | `1` | Active COM radio (1 or 2) |
| `volume` | `1.0` | Playback volume (0.0–1.0) |
| `pattern_direction` | `left` | Default traffic pattern direction (left/right) — overridden per airport/runway by `airport_vrps.json` |
| `disable_default_atc` | `false` | Suppress X-Plane's built-in default ATC |
| `skip_radio_power_check` | `false` | Bypass radio power detection (workaround for exotic aircraft) |
| `show_phraseology_hints` | `true` | Show phraseology cheat sheet in ATC panel |
| `auto_correction_factor` | `1.0` | ATC recovery time multiplier (0.5 = faster, 2.0 = slower) |
| `start_mode` | `engines_running` | Starting condition assumed by the ATC state machine. `engines_running` (default) puts the pilot on the apron with engines hot → first call is Ground for taxi; `cold_and_dark` allows a clearance-delivery / engine-start sequence before taxi. Aviation-realistic; affects which initial intents the Tower expects. |
| `atc_profile` | `EU` | ATC training profile: `EU` (ICAO/QNH/hPa) or `US` (FAA-TC/altimeter/inHg). The profile is a phraseology style, not a geographic restriction — you can fly either profile anywhere in the world; VRPs only render where AIP data exists in `data/vrps/airport_vrps.json`. The legacy key `flow_region` is mirrored in parallel for rollback safety and will be dropped in a later release. |
| `debug_logging` | `false` | Enable verbose debug output |
| `debug_traffic` | `false` | Show the Traffic tab in the ATC panel (lists the 10 nearest aircraft from the TCAS DataRefs) |
| `debug_text_input` | `false` | Show an InputText field below the transcript in the Status tab. Typed text is fed straight into `engine::process_transcript` via `atc_session::submit_text()` — STT is bypassed, LM + state machine + TTS run as in the voice path. Helpful without a headset and for isolating ATC-logic bugs from STT mistakes. PTT remains active in parallel; the shortcut `REG` expands to the phonetic callsign. |
| `traffic_features_enabled` | `true` | Master switch for the traffic subsystem (advisories, landing sequencing, go-around trigger). Off → `traffic_context::update()` returns an empty snapshot and every downstream consumer becomes a no-op. Requires a traffic provider (LiveTraffic, xPilot, swift, X-IvAp, native AI) to do anything anyway. |
| `backend_mode` | `local` | `local` (whisper + llama + Piper, arm64 only), `openai` (Whisper API + Chat Completions + TTS API), or `mistral` (Voxtral STT + Mistral Chat Completions + Voxtral TTS). The x86_64 slice silently rewrites `local` to `openai` at startup since Local is not available there; `mistral` is honored on both slices. |
| `api_key_saved` | `false` | Flag only — set automatically when the user clicks **Save Key** in Settings. The actual OpenAI key sits in the macOS Keychain under service `com.xp_wellys_atc.openai` / account `default`. Cleared by **Delete Key**. |
| `openai_stt_model` | `whisper-1` | OpenAI Whisper model ID for the STT call. |
| `openai_lm_model` | `gpt-4o-mini` | OpenAI Chat Completions model ID for the intent classifier. JSON mode is enabled automatically. |
| `openai_tts_model` | `tts-1` | OpenAI TTS model ID. Set to `tts-1-hd` for higher-quality (slower) output. |
| `openai_tts_voice_atis` / `openai_tts_voice_tower` / `openai_tts_voice_ground` | `onyx` / `echo` / `alloy` | Per-role OpenAI voice. One of `alloy / echo / fable / onyx / nova / shimmer`. `onyx` is closest to real ATC. |
| `mistral_api_key_saved` | `false` | Flag only — set when **Save Key##mistral** is clicked. The actual Mistral key sits in the macOS Keychain under service `com.xp_wellys_atc.mistral` / account `default`, separate from the OpenAI entry. |
| `mistral_stt_model` | `voxtral-mini-2507` | Voxtral STT model ID. The newer `voxtral-mini-2602` (Voxtral Mini Transcribe 2) is also valid and slightly more expensive. |
| `mistral_lm_model` | `mistral-small-latest` | Mistral Chat Completions model ID for the intent classifier. JSON mode is enabled automatically. `ministral-3b-latest` / `ministral-8b-latest` work too and are cheaper. |
| `mistral_tts_model` | `voxtral-mini-tts-2603` | Voxtral TTS model ID. |
| `mistral_tts_voice_atis` / `mistral_tts_voice_tower` / `mistral_tts_voice_ground` | `gb_oliver_neutral` / `en_paul_confident` / `en_paul_neutral` | Per-role Voxtral preset voice. The UI dropdown lists 30 voices across British (`gb_oliver_*`, `gb_jane_*`), American (`en_paul_*`) and French (`fr_marie_*`) speakers in 7-9 emotional registers (`neutral`, `confident`, `cheerful`, `excited`, `sad`, `angry`, `sarcasm`, …). British neutral reads closest to ICAO broadcast cadence. Custom voice clones from the Mistral dashboard can be set by editing this field directly in `settings.json`. |

ATC response templates are defined in `data/atc_profiles/{eu,us}/atc_templates.json`.
Flight phase detection thresholds, ATC precondition guards, frequency guards,
and auto-correction rules are in `data/atc_profiles/{eu,us}/flight_rules.json`.
Switching the ATC profile hot-reloads both files. All data files can be
edited without rebuilding the plugin.

### Models catalog (`data/models_catalog.json`)

Single source of truth for every selectable model and voice across all
three backends. The Settings dropdowns (`STT model`, `LM model`,
`TTS model`, per-role voices for OpenAI and Mistral) and the local
Models-tab download list are driven by this file. Edit it to:

- add a new OpenAI slug (e.g. a newer `gpt-4.1-*` or `tts-1-hd` variant);
- add a new Mistral slug (Voxtral STT/LM/TTS — Mistral roll model IDs
  forward every few months: `voxtral-mini-2507` → `voxtral-mini-2602`
  etc.);
- add or remove a Mistral preset voice (the public 30-voice catalog from
  the Voxtral TTS demo on HuggingFace is bundled by default);
- add a new Piper voice or repin a SHA256 hash after an upstream model
  update on HuggingFace.

Top-level shape:

```jsonc
{
  "openai":  { "stt": [...], "lm": [...], "tts": [...], "voices": [...] },
  "mistral": { "stt": [...], "lm": [...], "tts": [...], "voices": [...] },
  "local":   { "whisper": [...], "llama": [...], "piper_voices": [...] }
}
```

Each combo entry is `{"id": "<slug>", "label": "<UI label>"}` (label
defaults to the id). Local entries additionally carry the HuggingFace
`url`, `size_bytes` and `sha256` used by the downloader and verifier;
Piper voices have a paired `.onnx` + `.onnx.json` and an `optional`
flag (optional voices fold into the *Optional Voices* accordion on the
Models tab and don't gate readiness). The plugin reads the catalog
once at startup — restart X-Plane after editing.

### Airport Database (`data/vrps/airport_vrps.json`)

Per-airport configuration for Visual Reporting Points (VRPs) and traffic
pattern directions. Single global file shared by all ATC profiles — VRPs
are geographic facts read from the AIP, not phraseology, so they apply
regardless of whether you fly the EU or US profile. Pre-populated
for common Swiss and European VFR airports; other airports ship with
`pattern_direction` only (`vrps: []`) until they are checked against an
authoritative source. Each top-level key is an ICAO code with optional
fields:

- `name` — display name
- `pattern_direction` — per-runway `"left"` / `"right"` (overrides the
  global `pattern_direction` setting); accepts a string for an unconditional
  default or an object keyed by runway designator with optional `_default`
- `vrps` — array of `{ name, lat, lon, alt_ft }`; `name` is the phonetic
  spelling (e.g. `"November"`) so Whisper and Piper handle it cleanly
- `arrival_routes` — per-runway ordered list of VRP names used for
  inbound routing
- `_source` / `_comment` — optional audit annotations; ignored by the loader

#### Optional user override (Navigraph Charts workflow)

If you have a **Navigraph Charts** subscription you can supply your own
VRP coordinates without forking the plugin:

1. Drop a JSON file at
   `<X-Plane>/Output/preferences/xp_wellys_atc/airport_vrps.json` (single
   global file — VRPs are profile-independent). The directory is created
   on first plugin start. This path survives plugin re-installs.
2. Use the same schema as the bundled file. Per-ICAO entries fully replace
   the plugin defaults — there is no field-level merge, so include the
   complete entry for every airport you want to override.
3. Restart X-Plane (or `Reload Settings` from the menu) — a log banner in
   `Log.txt` confirms the load:
   `Airport VRPs loaded: N airports (X plugin, Y user overrides: Z replaced, W added) from <path>`

Navigraph Charts workflow per airport:
- Open the **VFR Approach Chart** (e.g. AD 2.22, section
  *Visual Approach*).
- Read the VRP code (W/N/E/S/Z…), translate to the phonetic name
  (`W` → `Whiskey`, `N` → `November`, …) — this is what Whisper
  transcribes and what Piper pronounces.
- Hover the chart for cursor lat/lon (Navigraph Charts displays the
  pointer coordinates in the toolbar).
- Read the published transit altitude from the chart legend.
- Note the pattern direction per runway from the AIP AD 2.22 (Flight
  Procedures) section.

The Navigraph **FMS Data** add-on for X-Plane Custom Data does *not*
contain VRPs (ARINC-424 is IFR-only). You need the Navigraph **Charts**
product to read the VFR data.

### ATC Response Templates (`data/atc_profiles/{eu,us}/atc_templates.json`)

Defines the ATC response text for every combination of airport type, ATC
state, and pilot intent. `towered` (full ATC flow) and `uncontrolled`
(CTAF/UNICOM self-announce) sections; each entry has `response`,
`next_state`, `requires_readback`. The special key `_INVALID` is the
fallback ("say again your request"). Variables are substituted from
`XPlaneContext` at runtime.

### Flight Rules (`data/atc_profiles/{eu,us}/flight_rules.json`)

Per-profile sections covering phase detection thresholds + hysteresis,
intent preconditions, auto-correction rules (state and frequency),
intent-to-frequency mapping, pilot phraseology, state-machine guards
(`state_frequency_validity`, `idle_redirects`, `state_reverts`,
`tower_only_auto_advance`), and frequency hint text.

### LLM Prompt Templates (`data/atc_prompt_templates.json`)

Prompts the engine sends to the local Llama 3.2 3B model:

| Key | Purpose |
|---|---|
| `whisper_prompt` | Initial-prompt hint for whisper.cpp to bias transcription toward aviation vocabulary and the NATO phonetic alphabet |
| `gpt_classify_prompt` | System prompt for low-confidence intent classification (variables: `{state}`, `{valid_intents}`, `{transcript}`, `{frequency_type}`, `{on_ground}`, `{altitude_ft}`, `{groundspeed_kts}`, `{airport}`) |

The key name keeps the `gpt_*` prefix for backwards compatibility with
existing `atc_prompt_templates.json` files; the local pipeline feeds
this prompt to Llama 3.2 unchanged.

**Push-to-Talk** is configured via X-Plane's keyboard or joystick settings.
The plugin registers the command `xp_wellys_atc/ptt` which can be bound to
any key or joystick button.

## Usage

1. Tune COM1/COM2 to the appropriate frequency in X-Plane (or click a
   frequency in the ATC panel to set it as standby, then flip-flop).
2. Hold the PTT key and speak your radio call — the **Phraseology Hints**
   panel shows you what to say (hover for full ICAO phraseology).
3. Release PTT — the plugin transcribes locally, processes through the
   state machine, and plays back the ATC response.
4. Check the ImGui overlay for transcript history and current ATC state.
5. If you get stuck in a loop, click the **Disregard** button to reset.

**No headset?** Flip `debug_text_input` on in Settings — an InputText
field appears under the transcript on the Status tab. Typed text is
fed directly into the engine (STT is skipped), but the LM, state
machine and TTS still run, so the Tower reply is spoken normally
through the active backend. The shortcut `REG` expands to your
phonetic callsign (e.g. *"Tower REG, ready for departure runway 14"*).
PTT remains active in parallel.

## Make Targets

```sh
make all           # clean + format + build + lint + test (full local CI)
make build         # universal: arm64 (local + both clouds) + x86_64 (clouds only), lipo'd
make setup         # deps incl. local-inference submodules
make setup-cloud   # deps WITHOUT the whisper/llama/Piper submodules (cloud-only; used by CI)
make release-build # same as `make build` but passes -DRELEASE=ON (embeds VERSION.txt)
make test          # unit tests + scenario tests
make install       # code-sign + install to X-Plane
make repl          # build the headless atc_repl tool
make format        # clang-format
make lint          # clang-tidy (some rules promoted to errors)
make ci-remote     # push current branch + trigger the cloud-only CI (mac/win/linux) via gh
make win-artifact  # download the newest Windows CI plugin folder -> dist-win/
make skunkcrafts   # stage a SkunkCrafts Updater release tree from the installed plugin
make cleanup-cache # delete GitHub Actions caches (rebuilt on next run)
make clean         # remove build/, build-arm64/, build-x86_64/, build-lint/, build-sanitize/
make distclean     # also remove sdk/, vendor/
```

## Known Limitations

| Limitation | Impact | Effort |
|---|---|---|
| **Local inference is Apple Silicon only, and only from source** | Pre-built releases are cloud-only on every platform (macOS/Windows/Linux) — OpenAI or Mistral, requires API key + billing. Fully-offline local inference (Metal whisper/llama/Piper) is available only by building from source on an Apple Silicon Mac. | Intel Macs / Windows would need Metal alternatives + a non-arm64 onnxruntime; shipping local in the release would re-add the ~50-min CI build. Linux can build CPU-local from source today (see README-LINUX.md). |
| **Only English supported** | Profiles cover EN (EU / US ICAO-FAA phraseology). Other languages (FR, IT, ES, NL, …) are not modelled — Whisper would transcribe them, but there is no phraseology profile, no LM prompt, and no language-matched Piper voice. | Medium per language — clone a profile under `data/atc_profiles/<code>/`, source AIP phraseology, add a Piper voice (Local) and validate Mistral/OpenAI prompts. Cloud STT/LM are already multilingual. |
| **"via Alpha" hardcoded** — taxiway name is always Alpha | Unrealistic at airports with different taxiway layouts | High — would need taxiway data from apt.dat or WED |
| **No wake-turbulence spacing** — sequencing in v2.2 picks number-by-distance only, no Light/Medium/Heavy separation | Acceptable for GA pattern work; missing for mixed-weight ops | Phase 5 on roadmap |
| **No callsign cross-check against the cockpit registration** — the pilot may say any callsign and Tower repeats it back verbatim; mismatch with the aircraft's actual tail number is not flagged | Realism gap; not a safety issue for single-player practice | Low — compare extracted callsign against `aircraft_icao` and surface a phraseology hint on mismatch |
| **Big-hub airports (LSZH, LSGG, …) not officially supported** — pilot can fly inbound/outbound, but Delivery (slot/VFR-clearance) workflow, RWY-specific Tower routing, and AIP VFR reporting points are not modelled | Generic hints at large hubs do not match real-world procedures (slot enforcement, multiple Tower frequencies, mandatory VFR points) | High — needs per-airport AIP research + new Delivery intent + slot setting + multi-Tower disambiguation |
| **IFR readback content is verified for altitude, FL, frequency, squawk, and runway** — `readback_verifier.cpp` checks the pilot's readback against the last cleared value on those fields and issues *"negative, [expected], readback"* on mismatch. **Fix / waypoint / SID / STAR identifier readback is NOT verified** — Mistral/Voxtral currently garbles those idents (e.g. QA503 → "Quebec Alpha 503" is fine, but longer waypoints like ROMAM often come back as "Rome Am") and any strict comparison would false-reject correct pilot readbacks. Extending verification to idents is deferred until a per-field fuzzy-match approach is added (tokenised NATO decoding + Levenshtein tolerance). Note: fine-tuned aviation STTs like WhisperATC were evaluated but comprehend general phraseology worse than Mistral's Voxtral, so switching STT is not the path forward. |

## FAQ

**Does this support IFR or flight planning?**
Yes — IFR departure flow is implemented (EU profile).  The plugin handles
pre-departure clearance (Delivery or Tower), startup approval, taxi to holding
point, line-up-and-wait, takeoff clearance, CTR departure handoff, and radar
contact with Departure/Approach.  SID assignment uses CIFP data matched to the
filed flight plan's first fix.  See **IFR ATC — What's Included** for the full
feature breakdown and **IFR ATC — Data Requirements** for what must be
installed.  En-route (Centre check-in, sector/FIR handoffs, TMA-entry descent)
is also implemented in the EU profile and under active refinement; full
approach and holding phases remain on the roadmap.

**Will there be a virtual co-pilot or checklist reader?**
Not in scope today. The plugin is a single-pilot Pilot ↔ ATC voice interface;
intercom and checklists are not implemented.

**Is it compatible with all XP12 aircraft and add-ons?**
Yes, in principle. The plugin is aircraft-agnostic and uses only standard
X-Plane DataRefs — no aircraft-specific code paths and no compatibility list.
It works with the default fleet (C172, etc.) and any add-on that exposes the
standard `sim/cockpit/radios/*` DataRefs. For exotic aircraft that don't
expose `com_power`, set `skip_radio_power_check: true` in `settings.json`.
Laminar's default ATC can be suppressed via `disable_default_atc`.

**Can I fly hands-on-yoke without focusing the plugin window?**
Yes — that's the design. Bind Push-To-Talk once to a yoke button or keyboard
key (X-Plane command `xp_wellys_atc/ptt`). After that, every interaction is
voice: press PTT, speak, release, hear ATC reply. The plugin window does not
need keyboard focus during flight, and all inference runs on background
threads so X-Plane never stutters.

**Does the plugin read my COM1/COM2 frequencies automatically?**
Yes. Active and standby frequencies for both COM radios are read live from
X-Plane DataRefs. The plugin also detects which radio is active and
auto-classifies the frequency type (ATIS / Ground / Tower / Approach /
UNICOM) against the apt.dat frequency database. No manual frequency entry.

**Does the plugin set the transponder / squawk code?**
No — spoken only. ATC may say "squawk 1200" (US flow), but the plugin does
not read or write the cockpit transponder DataRefs. You dial the squawk
manually on your transponder.

**How does it compare to BeyondATC or SayIntentions?**
Strengths: 100 % offline option on Apple Silicon (no subscription, no
cloud, no constant internet required — at the user's discretion), ~1.16 s
warm pipeline latency in local mode, ICAO-correct EU phraseology with
realistic Tower reactions to pilot errors. Two cloud options — **OpenAI**
and **Mistral** — are available as paid opt-ins (BYO key) for users who
prefer cloud LLMs or run an Intel Mac. Mistral typically costs less per
token.
Limitations today: IFR is EU-profile only and still maturing, no
wake-turbulence spacing (sequencing in v2.2 is distance-only — Phase 5 on
roadmap), no transponder data link, no co-pilot. It is not yet an all-in-one
replacement for those products.

**Is there an introduction video?**
Not yet.

**How does it compare to OpenSquawk?**
Not yet evaluated.

## Project Structure

```
src/
├── main.cpp                # XPlugin* entry points, menu, flight loop
├── atc/                    # Session coordinator, state machine, intent
│                           #   parser + rules, templates, ATIS, flight
│                           #   phase, engine, traffic_advisor /
│                           #   traffic_dialog, landing_sequence,
│                           #   phraseology_hints, plus
│                           #   flows/ (ground_operations, pattern_flow,
│                           #   crosscountry_flow, flow_coordinator)
├── audio/                  # Push-to-talk, mic capture, PCM playback
│                           #   on the X-Plane radio bus (COM1 or COM2),
│                           #   mic permission
├── backends/               # Strategy interfaces + manager (async
│                           #   dispatch) + loader (verify + load) +
│                           #   downloader (libcurl + resume + SHA256).
│                           #   Concrete backends split by mode:
│                           #   Local: WhisperStt / LlamaLm / PiperTts
│                           #     (arm64 slice only, gated on
│                           #     XPWELLYS_USE_LOCAL_INFERENCE).
│                           #   OpenAI: OpenAiStt / OpenAiLm / OpenAiTts
│                           #     (both slices, libcurl + JSON).
│                           #   Mistral: MistralStt / MistralLm /
│                           #     MistralTts (both slices, libcurl +
│                           #     JSON; Voxtral TTS returns a JSON
│                           #     envelope with base64-encoded WAV).
│                           #   The three client sets share no headers
│                           #   and no code path — audit invariant
│                           #   enforced by tests.
├── core/                   # Logging, XPlaneContext (SDK-free struct +
│                           #   SDK-coupled DataRef reader)
├── data/                   # Airport VRPs, apt.dat-derived airspace
│                           #   index, traffic_context (struct + 2 Hz
│                           #   TCAS reader), traffic_geometry +
│                           #   traffic_phase_classifier
├── persistence/            # settings.json, keychain (OpenAI + Mistral
│                           #   API keys), model_paths, model_manifest
└── ui/                     # Dear ImGui ATC panel + Models + Traffic
                            #   tabs, ui_strings (per-profile i18n),
                            #   clipboard helper
```

The `xp_atc_engine` CMake OBJECT library compiles the SDK-free translation
units (`atc/`, `core/logging`, `core/xplane_context` struct, `data/`,
`backends/manager.cpp`, `persistence/model_manifest`). Both the plugin
module and the headless `atc_repl` tool reuse it. The plugin module adds
the SDK-coupled units (`main.cpp`, `audio/`, `core/xplane_context_runtime.cpp`,
`backends/{loader,downloader,openai_*,mistral_*}.cpp`,
`persistence/{settings,model_paths,keychain}.cpp`, `ui/atc_ui.cpp`).
The arm64 slice additionally compiles
`backends/{whisper_stt,llama_lm,piper_tts}.cpp` and links statically
against `whisper`, `llama`, `common`, plus a shared `libpiper.dylib`
that resolves `libonnxruntime.1.22.0.dylib` through `@loader_path` —
both dylibs co-located inside the plugin bundle alongside the `.xpl`.
The x86_64 slice has none of those dependencies; it links only
libcurl + Security + the audio frameworks and ships both cloud
provider clients.

## Third-Party Dependencies

See [`THIRD_PARTY.md`](THIRD_PARTY.md) for the full list of bundled or
linked libraries, their licenses, and how they are vendored.

## Development Workflow

### CI Pipeline

All CI builds are **cloud-only** (`XPWELLYS_USE_LOCAL_INFERENCE=OFF`) —
the whisper/llama/Piper submodules are neither fetched nor compiled, so a
full run finishes in minutes instead of ~50. Local inference is a
build-from-source feature (`make build` on Apple Silicon), not part of any
CI artifact.

The GitHub Actions pipeline runs in three situations:

- **Pull Request against `main`** — runs the test gate once (unit + scenario
  suites, on Linux since the engine is SDK-free), then builds the cloud-only
  **macOS universal**, **Windows** and **Linux** plugins in parallel before
  the change can be merged.
- **Manual dispatch** (`workflow_dispatch`, via `make ci-remote`) — same
  build set on demand for the current branch; `make win-artifact` pulls the
  resulting drop-in Windows plugin folder.
- **Push of a version tag `v*`** — same gate + builds, then a `package` job
  merges the `mac_x64/`, `win_x64/` and `lin_x64/` slices with the shared
  `data/` + `docs/` into one X-Plane plugin folder, publishes a single
  GitHub Release ZIP (`xp_wellys_atc.zip`), and force-pushes the tree to the
  `release` branch that the **SkunkCrafts Updater** serves for in-sim
  updates (`skunkcrafts_updater.cfg` is bundled; user `settings.json` is
  never overwritten).

Direct pushes to `main` no longer trigger a build. All code changes must go
through a Pull Request.

### Merging to `main`

Branch protection requires:

1. PR (no direct pushes)
2. Status checks `test`, `build-macos`, `build-linux` and `build-windows` passing
3. PR branch up to date with main

## License

This project is licensed under the [GNU General Public License v3.0](https://www.gnu.org/licenses/gpl-3.0.html).
GPLv3 is required because espeak-ng (GPL-3.0-or-later) is statically
linked into the bundled `libpiper.dylib`. Compatible with all other
bundled third-party libraries; see [`THIRD_PARTY.md`](THIRD_PARTY.md)
for the per-dependency breakdown.

## Flight schools and commercial training

GPL-3.0 legally permits commercial use, including paid flight
training — so this section is **not** a paywall. But this is a
single-maintainer project, and a commercial training environment
usually wants more than a free community download:

- **Priority support** — bug triage, in-class issues handled first,
  direct line to the maintainer.
- **Tailored phraseology** — school-specific callsign patterns,
  AIP-aligned local procedures, instructor checklists.
- **Update assurance** — a maintained release cadence and an early
  heads-up before breaking changes land.
- **Optional co-branding** — your school's name in the in-sim welcome
  banner and the setup guide.

If you run a flight school or commercial training operation — EASA ATO,
US ground school, etc. — and want official backing for putting the
plugin into your syllabus, please get in touch:

📧 **rob.wellinger@gmail.com** — subject line `xp_wellys_atc training partnership`

Individual users, hobbyists and student pilots practising on their
own: just download and fly. This section is an offer to commercial
operators — nothing more, nothing less.
