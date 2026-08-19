# What's new in 4.4 — IFR

*xp_wellys_atc — IFR extensions by C. Potter, on top of upstream's VFR plugin
for X-Plane 12.*

**Compared against 4.3.1, the last public release.** 183 commits, ~20 000 lines.

> This document lists what changed. What the IFR flows *do*, how to set them up,
> and their known limits are in `README-IFR.md`; per-platform notes are in
> `README-LINUX.md` / `README-WINDOWS.md`. Commit-level detail, with the flight
> that exposed each bug, is in `CHANGELOG-IFR.md`.

---

The short version: **4.3.1 flew an IFR arrival on a published STAR and approach.
4.4 adds radar vectoring, a realistic stepped descent, holds, AFIS fields, and an
airspace layer that no longer needs a navdata subscription to answer.**

Everything below is off by default where it changes existing behaviour. The
**Limitations** section of `README-IFR.md` still applies: these flows have been
flown in a limited set of configurations, and several behaviours depend on
hand-maintained data.

## At a glance

Every heading and every item below, in one page. Each section is expanded further
down, in the same order.

**Radar vectors to final** — ATC positions the aircraft onto the final approach
course with headings instead of sending it round the published procedure. Off by
default.
- Geometry from the runway axis and the side the aircraft arrives from — no circuit is flown
- Every leg level floored by sector MSA, then grid MORA; ATC refuses to vector where neither answers
- One working setting, `FORCE APP VECTORING` (every arrival is vectored, for practice). `ALLOW VECTORING` -- ATC choosing to vector when it makes operational sense -- is **declared but not active in this build**; the checkbox is greyed out and says so
- Two modes decided per arrival and logged — vectors to final, or vectors to the IAF
- Non-compliance draws a re-issued vector or a confirmation, never a silent abandon

**The descent is flown down, not dropped** — a ladder of levels instead of one
large clearance, planned on the path actually flown.
- Stepped descent, one rung at a time, never the next until the previous is flown
- Top of descent computed leg by leg, STAR loops included
- No STAR constraint → the target is the first point of the cleared approach
- `DESCEND VIA` clearance, always carrying a level
- "Advise when ready to descend" negotiation before the top of descent
- `EXPEDITE DESCENT` wording when the gradient is steep, and a monitor for the opposite case
- One log line per clearance saying which fix constrained it and why it fired

**Airspace that answers without a navdata subscription** — the airspace layer no
longer falls silent when the export is missing or incomplete.
- `atc.dat` fallback where the OpenAir export is silent
- Volumes classified by ICAO class letter, not only by a keyword in the name (8 100 → 10 239 indexed)
- Terminal stack walk for exports that omit the type word — Germany, and Turin
- Terminal query anchored on the FAF rather than the field where the walk applies
- Blanket blocks resolved to their controller in logs — `AIRSPACE CLASS C (Langen)`
- Regional centre named above the UIR floor ("Reims", not "France") — France only
- Cross-border delegated airspace via an `airspace+.txt` overlay

**Arrival and approach** — more of the published procedure is modelled, and the
approach is chosen against the weather that matters.
- Published holds at a STAR fix, with an expect-further-clearance time
- Direct-to shortcuts on the STAR, and connector chaining when the filed STAR ends off-approach
- Approach selection gated on the **destination** METAR, not the weather at the aircraft
- `FORCE ILS IF AVAILABLE` setting
- Non-RNAV approaches got their final segment back — ILS, VOR, LOC and NDB had no FAF at all
- Per-airport overrides in `airport+.json`

**Departure** — the climb is shaped by the airspace above the field.
- SID initial climb read from the earliest constrained SID fix
- Lateral sector handoff during the SID climb
- Omnidirectional departure phraseology when no SID applies
- Departures are no longer cleared into a neighbour's TMA

**AFIS fields** — fields with an information service and no control are modelled
end to end: clearance from the overlying ACC, no line-up or takeoff clearance,
self-announcements acknowledged, and never an "unable".

**Phraseology and radio** — closer to how it is actually spoken.
- Full registration on first contact, abbreviated afterwards — once per flight
- Multi-item clearances and cumulative readback
- Speed control: 250 kt below FL100 combined with the next fix's published cap
- Designators spoken properly — `ABDI8R` → "ABDIL EIGHT ROMEO"
- Airport names, never ICAO codes, in spoken labels
- Readback correction follows ICAO — NEGATIVE, I SAY AGAIN, then the correction
- Radio check answers "reading you five"
- QNH stated only with a level in feet, never with a flight level

**Ground** — the flight now ends where it started.
- Runway crossing clearance at the Tower check-in, with a report when vacated
- GA parking stands sized and typed from `apt.dat`, not an airline gate
- Post-landing taxi to parking, and the IFR flight plan closed at the stand

**Simulator integration** — **time acceleration is supported.** Every ATC timer
used to run on a wall clock; they now follow sim time.

**Speech recognition** — the vocabulary sent to the recogniser is trimmed to the
current flight phase, with anchors for the phrases that were being mangled.
Voxtral only.

**Performance** — the pre-top-of-descent chain was rebuilding every frame and
consumed 106 ms of a 106 ms flight loop. If an earlier beta stuttered in the
descent, this is it.

**Developer tooling** — a closed-loop REPL pilot that obeys ATC, OFP replay,
real-data harnesses, and versioned algorithm specifications in `docs/`.

**Not done, and known** — read this one before flying.
- Pilot level requests (maintain / higher / lower, with ATC approving or refusing)
- Vectors to the IAF are decided and logged but not flown
- Sector handoffs fire on crossing the boundary, not before it
- Departures are not covered by the terminal stack walk — arrivals only
- Readback *verification* is deliberately not enforced
- Distances outside the descent planner are great-circle
- Upper-airspace regional naming is corrected for France only

---

### New — radar vectors to final

The headline feature. Tick **`FORCE APP VECTORING`** in the IFR tab and ATC
positions the aircraft onto the final approach course with headings instead of
sending it round the published procedure:

```
28 NM  turn left heading 026, reduce speed to 210 knots,
       descend flight level 60, vectoring for ILS approach runway 06
18 NM  descend 5000 feet, QNH 1024
12 NM  12 miles from KOLOT, descend 2500 feet, QNH 1024
       until established, cleared ILS approach runway 06
 9 NM  reduce speed to 160 knots
 6 NM  (silent — the aircraft is on the localiser and has the track)
```

The rule it is built around: **one heading closes with the final approach course,
the pilot makes the interception himself, and he is on the track at least 5 NM
before the FAF.** ICAO Doc 4444 8.9.4.1 puts it plainly — vectoring *terminates
at the time the aircraft leaves the last assigned heading to intercept the final
approach track* — so there is no routine second vector onto the course. A further
heading exists only as a **correction**, when the controller can see the
interception is not working.

The clearance is a package, not a line (6.7.3.2.7): **position relative to a fix
on the final approach track, the altitude to be maintained until established, and
the approach clearance** travel together — and that altitude is the **published
platform**, the FAF crossing altitude, so the aircraft meets the glide path from
below as 8.9.3.6 requires. On the simulated Dortmund arrival it crosses 5.9 NM
from the FAF at 2685 ft with the path at 4360: 1675 ft below it.

- Geometry follows the runway axis and the side the aircraft arrives from —
  intercept straight away when there is room, displace first when sitting on the
  centreline, downwind only to gain axis distance. No circuit is flown: IFR radar
  vectoring has no pattern, only headings. **The downwind shape is not reachable
  in this release**: the manoeuvre arms on the distance measured back from the
  FAF along the final approach track, which is negative for an aircraft arriving
  from the far side of the field, so those arrivals keep the published procedure
  instead of being vectored. Measured on LSGG runway 22 (`make replay-lsgg`).
- **Every leg level is floored by the sector MSA, then by grid MORA.** Where
  neither source answers, ATC refuses to vector and the panel says
  `DISABLED: NO MSA` rather than inventing a safe altitude.
- `FORCE APP VECTORING` (every arrival vectored, for practice) is the setting
  that works. `ALLOW VECTORING` -- ATC choosing to vector when it makes
  operational sense -- is **declared but NOT active in this build**: the
  per-arrival decision is still a placeholder, so the checkbox is greyed out and
  labelled "not yet active". Both default off; with them off nothing changes.
- Two modes are decided per arrival and logged — vectors to final, or vectors to
  the IAF where terrain or a curved final forbid the first. **The IAF mode is not
  implemented**; those arrivals keep the published procedure.
- Non-compliance is met with a re-issued vector or a confirmation, never a silent
  abandon. A re-cut that lands on the heading already assigned is not
  transmitted — below 3° there is nothing to fly.
- **The speed is released** once the FAF is behind -- "resume normal speed". An
  assigned speed stays in force until cancelled (ICAO 4.6.1.2) and the aircraft
  must be told when it no longer applies (4.6.1.7); ours never was. And an
  assigned speed is no longer restated as a maximum when a readback is
  challenged -- "160 knots" and "160 knots or less" are different instructions.
- **Speed is controlled**: 210 kt with the intercept heading for sequencing,
  160 kt separately about 9 NM from the FAF (EUROCONTROL's 160 kt from 8 NM),
  so the pilot is not handed a turn, a descent and a speed in one breath, and
  the last turn does not overshoot the axis.

New readers: `msa_db` (sector minimum altitudes) and `mora_db` (worldwide grid
MORA) were added as prerequisites and are used wherever a floor is needed.

### New — the descent is flown down, not dropped

- **Stepped descent.** A single "descend flight level 100" from FL280 is replaced
  by a ladder — FL100, the destination terminal ceiling, 6 000, then the
  platform — issued one rung at a time and never the next until the previous is
  flown. From a very high cruise the first step is capped so no clearance asks
  for 25 000 ft at once.
- **The top of descent is computed on the path actually flown**, leg by leg,
  including a STAR that loops back on itself, instead of a straight line.
- **When the STAR publishes no constraint**, the target becomes the first point
  of the *cleared approach*.
- **`DESCEND VIA` clearance** (ICAO Doc 4444 / SERA.14001), always carrying a
  level.
- **"Advise when ready to descend"** negotiation before the top of descent.
- **`EXPEDITE DESCENT` wording** when ATC is asking for a steep gradient, and a
  monitor for the other half — an aircraft not descending enough, now judged
  against the constraining *fix* rather than the airport.
- Every descent clearance writes one line to `Log.txt` saying which fix
  constrained it, what was transmitted, whether that level is a rung, and the
  arithmetic that fired it.

### New — airspace that answers without a navdata subscription

- **`atc.dat` fallback.** Where the OpenAir airspace export is silent, the same
  query is retried against X-Plane's own `atc.dat`. Users without a navdata
  subscription can now fly the IFR flows at all.
- **Volumes are classified by their ICAO class letter**, not only by a keyword in
  the name. European exports put the class in one field and the type in the name,
  so anything without a type word used to be invisible. The index grows from
  8 100 to 10 239 volumes, and 74 restricted/danger areas that were being indexed
  by accident are now excluded.
- **Terminal stack walk** — for exports that omit the type word entirely.
  Germany names almost no volume `TMA` or `CTA` (17 % of its controlled volumes
  carry a type word, against 92–100 % in the eight other countries measured), so
  every German arrival had no terminal area at all: no descent rung, no
  descend-to-enter, no terminal handoff. The terminal shelf is now derived from
  the *shape* of the vertical stack instead. Off by default and enabled per
  destination (`ED`, `LI`). It also fixes **Turin**, whose terminal airspace is
  named `MILAN CTA ZONE 24 DON BOSCO` inside a country that is otherwise 99 %
  correctly named.
- The destination terminal query anchors on the **FAF**, not the field, where the
  walk is enabled — at Dortmund the terminal area covers the FAF but not the
  runway.
- **Blanket blocks are resolved to their controller.** The export strips names
  off some volumes, so what a controller calls *Langen* is recorded as
  `AIRSPACE CLASS C`; logs now print both.
- **Above the UIR floor, the regional centre is named** ("Reims"), not the
  country ("France"). France only; frequencies untouched.
- Cross-border **delegated airspace** is supported through an `airspace+.txt`
  overlay (Slovenia upper, LFFF→LSAS).

### New — arrival and approach

- **Published holds.** ATC may issue a hold at a STAR fix — "hold at FIX as
  published, maintain <level>, expect further clearance in N minutes" — read from
  `earth_hold.dat`, blocking other clearances until it expires.
- **Direct-to shortcuts on the STAR**, offered inside a window around the STAR
  entry, and **connector STAR chaining** when the filed STAR ends away from the
  approach.
- **Approach selection gates on the destination METAR** (visibility, ceiling),
  not on the weather at the aircraft.
- **`FORCE ILS IF AVAILABLE` setting** — assigns the ILS whenever the arrival
  runway publishes one, bypassing the per-airport filter and the RNAV-first
  ranking.
- **Non-RNAV approaches got their final segment back.** The CIFP reader had been
  treating the approach-type letter as the segment marker, so every ILS, VOR, LOC
  and NDB approach had no FAF and no missed-approach point.
- Per-airport overrides in `airport+.json`: runway configuration, approach list,
  controllers and frequencies, departure holds, SID initial climbs.

### New — departure

- **SID initial climb** read from the earliest constrained SID fix, with a
  climb ladder driven by the airspace above rather than hardcoded per field.
- **Lateral sector handoff during the SID climb**, probing the aircraft's own
  volume rather than the stack above it.
- **Omnidirectional departure phraseology** when no SID applies.
- **Departures are no longer cleared into a neighbour's TMA**: the climb probe
  now checks who owns the airspace ahead.

### New — AFIS fields

Fields with an information service and no control are modelled end to end: the
IFR clearance comes from the overlying ACC on the ground, there is no line-up or
takeoff clearance, self-announcements are acknowledged, and ATC never answers
"unable". Classification was corrected to use "no Ground **and** no Approach
frequency", which had been mislabelling Innsbruck.

### New — phraseology and radio

- **Full registration on first contact, abbreviated afterwards** — once per
  flight, not once per frequency. Operator callsigns are never abbreviated.
- **Multi-item clearances and cumulative readback**: several instructions in one
  transmission, tracked per field slot with latest-wins.
- **Speed control** — 250 kt below FL100 combined with the next fix's published
  cap, including during the SID climb.
- **SID/STAR designators are spoken properly**: `ABDI8R` → "ABDIL EIGHT ROMEO".
- **Airport names, never ICAO codes**, in spoken labels ("Reims Prunay").
- **Readback correction follows ICAO**: NEGATIVE, I SAY AGAIN, then the correct
  version.
- **Radio check** answers "reading you five".
- QNH is stated only when a level is assigned in feet, never with a flight level.

### New — ground

- **Runway crossing clearance** offered at the Tower check-in, with a report when
  vacated.
- **GA parking stands**: taxi to a size- and type-appropriate stand read from
  `apt.dat`, rather than an airline gate.
- Post-landing "holding short" on Ground leads to taxi-to-parking, and the IFR
  flight plan closes at the stand.

### New — simulator integration

- **Time acceleration is supported.** Every ATC timer used to run on a wall
  clock, so speeding the sim up made the aircraft cover far more track miles per
  timer second than intended. Timers now follow **sim time**, and `Log.txt`
  reports the factor: `Sim time: running at 2.0x wall clock`.

### Speech recognition

- **Phase-aware context bias** — the vocabulary sent to the recogniser is trimmed
  to the current flight phase, and fixes already behind the aircraft are dropped.
- Anchors for the phrases that were being mangled ("report airborne", "say
  again", landing calls), and SimBrief's pseudo-fixes excluded from the
  departure vocabulary.
- Note: the context-bias mechanism is **Voxtral-only**.

### Performance

- **The pre-top-of-descent chain was rebuilding every frame**, re-reading a 15 MB
  navdata file, and consumed 106 ms of a 106 ms flight loop. It is now cached on
  the route and approach identity. If you saw the simulator stutter in the
  descent on an earlier beta, this is it.

### Developer tooling

- A **closed-loop REPL pilot** that obeys ATC — retunes, checks in, flies the
  cleared levels — so an arrival can be replayed headless.
- **OFP replay**: load a real saved flight plan and replay it.
- Real-data test harnesses for AFIS, STAR tracking and the EDLW arrival.
- Algorithm specifications in `docs/`, each carrying its version, date and the
  build it was written against: `algorithm-descent.md`, `algorithm-airspace.md`,
  `force-app-vectoring.md`, and `open-questions.md` for what is still undecided.

### Not done, and known

- **Pilot level requests** (request to maintain / higher / lower, with ATC
  approving or refusing) — still owed before the public build.
- **Vectors to the IAF** are decided and logged but not flown.
- **Sector handoffs fire on crossing the boundary**, not a minute or two before
  it as real ATC does.
- **Departures are not covered by the terminal stack walk** — the German fix
  applies to arrivals only.
- Readback *verification* is deliberately not enforced, pending better speech
  recognition.
- Distances outside the descent planner are great-circle, not routed.
- Upper-airspace regional naming is corrected for France only.
