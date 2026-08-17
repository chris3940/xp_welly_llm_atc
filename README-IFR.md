# xp_wellys_atc — IFR by C. Potter

IFR ATC for X-Plane 12, added by this fork on top of upstream's VFR plugin.
Instrument flights are handled end to end: clearance delivery, taxi, departure
with a SID, sector handoffs along the route, descent on a STAR, an instrument
approach, and the transfer to Tower.

Platform notes are in [README-LINUX.md](README-LINUX.md) /
[README-WINDOWS.md](README-WINDOWS.md); the plugin itself is documented in the
upstream [README.md](README.md).

> **Read the [Limitations](#limitations) section before your first IFR flight.**
> The IFR flows have been flown in one configuration only, and several behaviours
> depend on hand-maintained data.

---

## What it does

| Phase | Behaviour |
|---|---|
| **Clearance** | IFR clearance with the SID, the initial climb level and a squawk. At an AFIS field the clearance comes from the overlying area controller. |
| **Ground / Tower** | Taxi to the holding point, runway crossings where applicable, line-up and take-off. |
| **Departure** | Progressive climb ladder shaped by the airspace above the field, with a departure hold where the local procedure calls for one, then handoff to the next sector. |
| **En route** | Sector handoffs driven by the airspace boundaries actually crossed, each new controller acknowledging the pilot's check-in and preserving the previous clearance. |
| **Descent** | Top-of-descent negotiation, descent on the filed STAR with the published step-downs, occasional published hold. |
| **Approach** | Approach selected against the destination weather, cleared at the initial approach fix or positioned onto the final approach course by radar vectors, then Tower. |

**Uncontrolled (AFIS) aerodromes are handled as such.** At a field whose radio
service is "Information" rather than a Tower, there is no line-up or take-off
clearance and no landing clearance to be had: the IFR clearance comes from the
overlying area controller, and Information supplies traffic and field data only.
Both ends of a flight are covered — departing from one, and arriving at one with
no published STAR.

Procedures come from the navigation data, not from a script: SIDs, STARs and
approaches are read from the CIFP, and sector ownership from the OpenAir airspace
export cross-referenced with `atc.dat` for frequencies.

---

## Limitations

Everything below is a known, accepted boundary of the current release — not a bug
report. Read it before filing an issue.

### Tested configuration

The IFR flows have only ever been flown in one configuration. Anything else may
work, but nobody has checked.

| Item | Tested | Not tested |
|---|---|---|
| Inference backend | **Mistral Cloud** — `voxtral-mini-transcribe-2507` (STT), `mistral-large-latest` (intent), `voxtral-mini-tts-2603` (TTS) | Local (whisper/llama/Piper) and OpenAI Cloud — see below. Smaller Mistral models are selectable but untested for IFR. |
| Navigation data | **Navigraph**, current cycle | X-Plane stock navdata; expect missing or stale procedures |
| Region | France, Alps, northern Italy, and one leg through Switzerland / Luxembourg / Belgium / Germany | everywhere else |
| Approach type | **RNAV / RNP** | ILS, VOR, LOC and NDB finals were unusable until 2026-08-14 (the navdata reader dropped their final segment) and are fixed but **unflown** |
| Platform | Linux | macOS / Windows builds are maintained but unflown for IFR |

**Speech recognition is tuned for Voxtral specifically, and that tuning does not
carry over.** IFR radio work is dense with callsigns, fix names, flight levels,
QNH and squawk codes, and getting it transcribed reliably relies on a
`context_bias` list built per situation — the aerodrome and controller in use,
the aircraft callsign, the procedure fixes ahead, numbers spelled digit by digit.
Only the Voxtral backend consumes that list; `whisper_stt` and `openai_stt`
discard the parameter by design and fall back to a freeform prompt, which is a
materially weaker mechanism. So Local and OpenAI mode are not merely unflown for
IFR — they are missing the biasing the IFR phraseology was tuned around, and
should be expected to mis-transcribe more.

Representative test routes:

| Route | What it exercises |
|---|---|
| LFLP ↔ LFMN | the reference flight, and the one leaning hardest on `airport+.json` at both ends: mountain departure held at an intermediate level under the Geneva TMA shelves on the SIDs that need it, Alpine sector handoffs, STAR arrival whose approach is chosen against the destination weather — RNP 04L Alpha above 10 km visibility and a 2500 ft ceiling, ILS 04L Zulu below |
| LFMN → LOWI | cross-border, and a curved RNP final into a valley |
| LIMF → LFLP | cross-border from Italy, high-altitude stepped descent |
| LFLU → LFLP | **departure from an AFIS field** ("Information", no Tower), and a sector missing from the airspace export, supplied by the `airspace+.txt` overlay |
| LFLP → LFQA | **arrival at an AFIS field**, no published STAR |
| LFLP → EDLW | the long leg: four ACC sectors across four countries, a filed step-climb profile, and an **ILS** arrival at a field whose terminal airspace belongs to a neighbouring unit (Duesseldorf) and is not named as such in the airspace export |

The phraseology, the airspace assumptions and the tuning all reflect those
flights.

### Data dependencies

- **A Navigraph subscription is effectively required for PROCEDURES.** SIDs, STARs
  and approaches come from the CIFP, and without current data the plugin will pick
  wrong procedures or fall silent. **Airspace is no longer in that category**:
  sector boundaries come from the OpenAir export when it is present, and where it
  is silent the same query is retried against X-Plane's own `atc.dat`, so
  controllers and handoffs still resolve without a subscription.
- **Sector handoffs are only as good as the OpenAir coverage.** Where a country's
  export omits a volume — upper airspace is the usual gap — no controller
  resolves and the handoff does not happen. Gaps are patched by hand in
  `Resources/airspace+.txt`; several already are. Note the export carries no
  class E, so a country that builds its approach layer in class E leaves a
  vertical hole above its low terminal volumes; the plugin then falls back to the
  destination field's own published approach frequency so the arrival still gets
  a controller.
- **Some exports omit the airspace TYPE altogether.** Germany names almost no
  volume `TMA` or `CTA` — 17 % of its controlled volumes carry a type word,
  against 92-100 % in the eight other countries measured — so a German arrival
  had no terminal area at all. The terminal shelf is now derived from the shape of
  the vertical stack instead of from the name. It is **enabled per destination**
  (`terminal_stack_walk_icao_prefixes` in the IFR `flight_rules.json`, currently
  `ED` and `LI`), applies to **arrivals only**, and the same gap exists at Turin
  inside an otherwise correctly-named Italy.
- **Some data is hand-maintained, per airport, in `Resources/airport+.json`** —
  runway pairings, weather-gated approach selection, controllers missing from
  `atc.dat`, departure holds, published initial-climb altitudes. Airports without
  an entry fall back to generic behaviour.

### Procedures

- **SIDs and STARs are chosen by the plugin from the CIFP**, not read from your
  flight plan. The filed route's first and last fixes select them.
- **Runway in use is derived from wind and `apt.dat`**, not from the published
  AIP preferential-runway rules or a real ATIS. `airport+.json` can override the
  pairing per airport; most airports have no entry.
- **Where an airport lists approaches in `airport+.json`, that list is a filter,
  not a preference.** Any approach missing from it is never selected for that
  runway, even if the navdata has it, and the first rule whose weather gates hold
  wins outright. So each runway offers exactly one approach per weather band,
  never a choice between equivalent ones — at LFMN 04L, RNAV Alpha above 10 km /
  2500 ft and the ILS below, with the RNP Zulu unreachable. Airports with no
  entry fall back to picking from the navdata and are unaffected. The
  **`FORCE ILS IF AVAILABLE`** setting is the escape hatch: it takes the ILS
  whenever the arrival runway has one published, ignoring both the filter and the
  RNAV-first ranking.
- **Curved (RF) RNP finals work — LOWI is flight-tested — but only where the
  handoff point has been entered by hand.** The last runway-aligned fix is set
  per approach in `airport+.json` (`tower_handoff_fixes`); it is **not** computed.
  Without an entry the code falls back to the FAF, which on a curved final can be
  far out and badly placed.
- **A mid-procedure altitude minimum higher than the current climb step is not
  honoured.** The climb floor is read from the first constrained fix of the SID;
  a later, higher minimum can be crossed below its published level. Fixing this
  needs a continuous per-fix floor tracked along the route.
- **Radar vectors to final are OFF by default and incomplete.** Two settings
  control them: `ALLOW VECTORING` (ATC may choose) and `FORCE APP VECTORING`
  (always, for practice). With both unticked nothing changes. Known boundaries:
  - **A sector MSA or grid MORA must resolve**, otherwise ATC refuses to vector
    and the panel says `DISABLED: NO MSA`. No leg level is ever invented.
  - **Vectors to the IAF are decided and logged but not flown.** Where terrain or
    a curved final rule out vectors to final, the arrival keeps the published
    procedure.
  - Vectoring does not hand off to Tower by itself, and the localiser
    re-intercept path has never been exercised in flight.
- **No vertical-profile check.** The plugin verifies lateral alignment on the
  approach and compliance with the last cleared level, but never that the
  aircraft is on the published glide/descent path.
- **Top-of-descent distances follow the routed path** — leg by leg along the
  route, the STAR in published order (loops included), then the approach — but
  **every other distance is great-circle**, so triggers outside the descent
  planner stay approximate on a route with significant dog-legs.
- **The routed path is the wrong measure once the aircraft is vectored**, and it
  errs in both directions: before the vectors start it includes procedure legs the
  vectors will delete (at Dortmund the ILS 06 transition runs overhead the field
  and back out to the FAF), and once on a downwind leg it collapses to the
  straight line. Both distances and the resulting gradients are printed at every
  descent clearance in `Log.txt` so the effect is visible; neither is corrected
  yet.

### Phraseology and speech

- **ICAO / European phraseology only.** US IFR procedures and phraseology are not
  modelled.
- **English only.**
- **Readback verification covers the numeric items only** — runway, altitude,
  flight level, frequency, speed and squawk. Get one wrong and ATC answers
  "negative, <item>, readback". Anything outside that set (route, procedure
  names, conditional instructions) is acknowledged without being checked.
- **A wrong readback is accepted after two attempts.** Speech recognition
  garbles numbers in ways no biasing recovers, so looping "negative, readback"
  forever would be worse than letting it through. The clearance ATC believes it
  issued therefore may not be the one you read back.

### Out of scope

- VFR flows are inherited from upstream and are not maintained here.
- Traffic awareness beyond phase 1 (TCAS snapshot) is not implemented: no en-route
  advisories, taxi holds, landing or take-off sequencing.

---

## Credits

The IFR feature set — state machine, procedure handling, airspace-driven sector
handoffs, phraseology — was designed and built by **Christopher P. Potter**
(GitHub [@chris3940](https://github.com/chris3940)), together with the Linux
port, on top of **thWelly**'s xp_wellys_atc.

It is developed and **validated over tens of hours of real instrument flights**
in X-Plane 12, not against synthetic scenarios — the routes are listed under
[Tested configuration](#tested-configuration). That flight testing is not a
footnote to the work: essentially every behaviour documented above was specified,
and most defects found, from captured `Log.txt` / `transcript.log` pairs of
actual flights. Sector-handoff geometry, climb floors and approach triggers are
things no headless test suite surfaces.

Licensed GPL-3.0-or-later, like the rest of the plugin. Per-file copyright lines
in `src/` record who wrote what.

---

Platform-specific caveats (cosmetic issues, shared-library notes) live in
[README-LINUX.md](README-LINUX.md); everything about the plugin itself is in the
upstream [README.md](README.md).
