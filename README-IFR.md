# xp_wellys_atc — IFR

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
| **Approach** | Approach selected against the destination weather, cleared at the initial approach fix, radar vectors to final where a reversal is needed, then Tower. |

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
| Inference backend | **Mistral Cloud** (Voxtral STT/TTS + `mistral-small`) | Local (whisper/llama/Piper) and OpenAI Cloud — both compile and are wired, but no IFR flight has been flown on them |
| Navigation data | **Navigraph**, current cycle | X-Plane stock navdata; expect missing or stale procedures |
| Region | France, Alps, northern Italy | everywhere else |
| Platform | Linux | macOS / Windows builds are maintained but unflown for IFR |

Representative test routes: LFLP↔LFMN, LFLU→LFLP, LIMF→LFLP, LFLP→LFQA, LOWI
arrivals. The phraseology, the airspace assumptions and the tuning all reflect
those flights.

### Data dependencies

- **A Navigraph subscription is effectively required.** SIDs, STARs and approaches
  come from the CIFP; sector boundaries come from the OpenAir airspace export.
  Without current data the plugin will pick wrong procedures or fall silent.
- **Sector handoffs are only as good as the OpenAir coverage.** Where a country's
  export omits a volume (upper airspace is the usual gap), no controller resolves
  and the handoff does not happen. Gaps are patched by hand in
  `Resources/airspace+.txt`; several already are.
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
- **Curved (RF) RNP finals work — LOWI is flight-tested — but only where the
  handoff point has been entered by hand.** The last runway-aligned fix is set
  per approach in `airport+.json` (`tower_handoff_fixes`); it is **not** computed.
  Without an entry the code falls back to the FAF, which on a curved final can be
  far out and badly placed.
- **A mid-procedure altitude minimum higher than the current climb step is not
  honoured.** The climb floor is read from the first constrained fix of the SID;
  a later, higher minimum can be crossed below its published level. Fixing this
  needs a continuous per-fix floor tracked along the route.
- **No vertical-profile check.** The plugin verifies lateral alignment on the
  approach and compliance with the last cleared level, but never that the
  aircraft is on the published glide/descent path.
- **En-route and top-of-descent distances are great-circle**, not routed. Trigger
  points are therefore approximate on a route with significant dog-legs.

### Phraseology and speech

- **ICAO / European phraseology only.** US IFR procedures and phraseology are not
  modelled.
- **English only.**
- **Readbacks are acknowledged, not verified.** An incorrect readback is not
  challenged or corrected. This is deliberate and waits on better speech
  recognition — a strict check on top of today's error rate would reject correct
  readbacks more often than wrong ones.

### Out of scope

- VFR flows are inherited from upstream and are not maintained here.
- Traffic awareness beyond phase 1 (TCAS snapshot) is not implemented: no en-route
  advisories, taxi holds, landing or take-off sequencing.

---

## Credits

The IFR feature set — state machine, procedure handling, airspace-driven sector
handoffs, phraseology — was designed and built by **Christopher P. Potter**
(GitHub [@chris3940](https://github.com/chris3940)), together with the Linux
port, on top of **thWelly**'s xp_wellys_atc. It is developed against real
instrument flights in X-Plane 12 rather than synthetic scenarios; the routes it
has been flown on are listed under [Tested configuration](#tested-configuration).

Licensed GPL-3.0-or-later, like the rest of the plugin. Per-file copyright lines
in `src/` record who wrote what.

---

Platform-specific caveats (cosmetic issues, shared-library notes) live in
[README-LINUX.md](README-LINUX.md); everything about the plugin itself is in the
upstream [README.md](README.md).
