# Changelog — IFR

Release notes for the IFR feature set of this fork. Platform and build notes are
in [README-LINUX.md](README-LINUX.md) / [README-WINDOWS.md](README-WINDOWS.md);
what the IFR flows do and their known limits are in
[README-IFR.md](README-IFR.md).

Every entry below was diagnosed from a captured `Log.txt` / `transcript.log` pair
of a real flight, and says which one.

---

## 4.4.0 — what changed since 4.3.1

The short version for someone upgrading: **4.3.1 flew an IFR arrival on a
published STAR and approach. 4.4 adds radar vectoring, a realistic stepped
descent, holds, AFIS fields, and an airspace layer that no longer needs a navdata
subscription to answer.** 183 commits, ~20 000 lines.

Everything below is off-by-default where it changes existing behaviour, and the
[Limitations](README-IFR.md#limitations) section still applies: these flows have
been flown in a limited set of configurations.

### New — radar vectors to final

The headline feature. Tick **`FORCE APP VECTORING`** in the IFR tab and ATC
positions the aircraft onto the final approach course with headings instead of
sending it round the published procedure:

```
28 NM  turn left heading 026, reduce speed to 210 knots,
       descend flight level 60, vectoring for ILS approach runway 06
18 NM  descend 5000 feet, QNH 1024
12 NM  12 miles from KOLOT, descend 2500 feet, QNH 1024
       until established on the localiser, cleared ILS approach
       runway 06, report established on the localiser
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
  vectoring has no pattern, only headings.
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

---

## Unreleased — 4.4.0-beta

Three flights drove this batch: **LFLP → LFMN** (2026-08-14 morning) and two
attempts at **LFLP → EDLW** (2026-08-14 afternoon and evening) — the first long
cross-border legs into Germany, the second broken off when the descent went
wrong 200 NM out.

### Added

- **Time acceleration is now supported.** Every ATC countdown ran on a wall
  clock, so speeding the sim up made the aircraft cover far more track miles per
  timer second than intended — approach step-downs, sector handoffs and readback
  budgets all fired late in flight terms. Timers now follow **sim time**, and
  `Log.txt` reports the measured factor once per change:
  `Sim time: running at 2.0x wall clock`.
- **`FORCE ILS IF AVAILABLE` setting** (Settings tab, off by default). Assigns
  the ILS whenever the arrival runway has one published, bypassing both the
  weather-gated choice in `airport+.json` and the RNAV-first ranking. A runway
  with no ILS is unaffected. Added because the `approaches` list in
  `airport+.json` is a filter, not a preference, which can make a published ILS
  structurally unreachable.

### Fixed — approach and arrival

- **Non-RNAV approaches lost their entire final segment.** The CIFP reader treated
  route type `R` as "final approach body", but that letter is the *approach type*
  (`I` = ILS, `R` = RNAV, `D` = VOR/DME, `L` = LOC, `N` = NDB). Every ILS, VOR,
  LOC and NDB approach therefore had no FAF and no missed-approach point, and the
  Tower handoff — which triggers at the FAF — could never fire. On the EDLW ILS 06
  the procedure yielded 2 waypoints instead of 4. RNAV arrivals hid this because
  they were the only type the reader accepted.
- **No approach controller at a field served from elsewhere.** The handoff
  resolved a controller by polygon only, and at Dortmund there is none to find:
  `atc.dat` lists Dortmund as a Tower, not an approach unit, and no approach
  polygon covers the field. That is correct — the service is Langen Radar. The field's own published approach frequency is now the last
  resort, so ATC says "contact Langen Radar on 125.225" instead of going silent.
- **"Request descent" drew no answer once the descent had begun.** The handler
  only covered the cruise phase. Now answered in descent, arrival and approach:
  a restated clearance when the aircraft is above its level, otherwise
  "maintain …, expect lower shortly".
- **ATC asked the pilot to confirm a climb he had never been cleared for.** When
  the aircraft was *below* its assigned level the monitor said "confirm climbing
  flight level 180". Being below a cleared level is not a climb clearance; it now
  queries the level instead.
- **A "direct" was challenged seconds after being issued.** The 90-second settle
  that lets the pilot turn onto the new leg was armed on one direct-to path but
  not on the STAR direct-to-IAF shortcut, so "direct DOR" drew "confirm routing,
  you appear tracking heading 37, expected 319" seven seconds later.

### Fixed — airspace data

- **Controlled airspace whose name carries no type word was invisible.** The
  airspace index decided what a volume *was* from a keyword in its name (CTR /
  TMA / CTA / FIR), because European exports put the ICAO class letter in the
  class field and the type in the name. Anything without such a word — Free
  Route blocks, and plenty of terminal volumes — was never indexed, so the
  plugin saw nothing over them. That is the root cause of both the 200 NM
  descent above and the missing approach controller at Dortmund. The class
  letter is now read as well: A–D are controlled, E/F/G are not, and restricted,
  prohibited and danger areas stay out. On the reference data the index grows
  from 8 100 to 10 239 volumes. Measured over the flight corridor, the effect is
  additive at altitude and almost nil in terminal airspace — 87 % of sampled
  points at FL280 go from *nothing* to a resolved volume, against 6 % at
  3 000 ft — and no existing volume is displaced by a new one.
- **74 restricted and danger areas were being indexed by accident**, because a
  type word appears inside their name — `R-33 (MATMATA)`, `P-33 (FIRGROVE)`,
  `R-302 (REGIONAL PSYCHIATRIC CTR)`, `W-METTMANN`. They could be resolved as
  controlled airspace. They no longer are.

### Fixed — en route

- **Above FL195 in France, ATC announced itself as "France" instead of the
  regional centre.** In the UIR it is the area control centre that gives its
  name — over eastern France that is **Reims**. The navdata models French upper
  airspace as one country-wide controller with 64 frequencies attached, so every
  handoff above the UIR floor used the country name. The regional centre is now
  taken from the lower band at the same position. Germany was checked and needs
  nothing: its upper centres are already named regionally (Rhein for Karlsruhe
  UAC, Hannover). **Only the name changes — the frequency is untouched**, since
  the lower-band entry carries lower-band frequencies. This does not reproduce
  sector-to-sector handoffs *inside* a regional centre, which work several
  frequencies in reality.

- **The first ACC handoff after the terminal phase was swallowed.** The sector
  memory was initialised to the sector just resolved rather than the frequency
  the pilot was actually on, which records the transition as already done at the
  moment it is discovered. Swiss Radar was never announced after Geneva; the
  frequency appeared in standby and the controller label never moved.
- **ATC undid its own descent clearance.** A "climb to cruise" rescue, meant for
  a departure that never received its cruise clearance, fired whenever the
  cleared level sat below cruise — including right after a planned step down.
  Cleared FL230, then climbed back to FL280 six seconds later.

### Fixed — departure

- **Departures were cleared into a neighbour's airspace.** The climb probe read
  the ceiling of the airspace ahead without asking who owned it. Leaving Annecy
  eastbound it found the Geneva TMA and had Chambéry issue "climb flight level
  190" at 6467 ft — 11 000 ft inside Geneva's airspace. The probe now checks
  ownership: a foreign volume caps the climb at its floor, or at the SID's own
  published minimum where the procedure obliges entry, and the transfer follows.
  Six of the eight fields in the Alpine region have a foreign TMA within the
  probe range, so this was not local to Annecy.
- **Sector awareness no longer depends on the cleared level.** The airspace scan
  was skipped until the aircraft neared its first climb step, so a step set too
  high blinded the plugin completely — on that departure Geneva was invisible
  below 17 000 ft. Crossing a boundary is a fact about position; it is now
  observed always and only the handoff is gated.
- **The handoff was attributed to the wrong controller.** The departure handoff
  stored the new controller as the current one, so the reminder read "you are
  still with Chambery Approach, contact the next controller" while the pilot was
  still on Annecy Tower — both halves wrong. It also broke the detector that
  accepts a correct handoff readback silently.

### Fixed — readbacks and phraseology

- **A correct readback could cancel the next clearance.** Reading a handoff back
  on the old frequency was accepted silently for the pilot but left the clearance
  armed. The reminder then counted against a readback already given, and the
  third tick cancelled whatever clearance was current by then — on one arrival,
  "no response received, say again" ten seconds after "cleared to land", followed
  by a state reset at 1 NM on final.
- **A readback timeout no longer resets the state in flight.** `LANDING_CLEARED`,
  `RADAR_CONTACT`, `FREQ_HANDOFF` and `EN_ROUTE` were missing from the list of
  airborne states exempt from the reset.
- **Correcting a readback now follows ICAO.** The procedure is NEGATIVE, then
  I SAY AGAIN, then the correct version. ATC said "negative, 250 knots or less,
  readback"; it now says "negative, I say again, 250 knots or less".

### Still owed before the public build

- **`FORCE VECTORING`**, and radar vectors to final generalised beyond the
  large-turn reversal, so ATC can position the aircraft onto the FAF or the ILS
  intercept at any airfield rather than only where the approach reverses.
- **Pilot level requests** — request to maintain the present level, or request
  higher or lower, with ATC **approving or refusing**. A refusal has to carry a
  reason the pilot can act on, and an approval has to update the clearance ATC
  then holds him to.

### Known, not fixed in this batch

- The ground-side handoff readback still draws "you are still on Ground".
- The departure handoff can be spoken twice, once without a frequency.
- The wrong-frequency reminder says "you are still with me" when the outgoing
  controller has no label yet.
- The garble replies ("your transmission was garbled, say again") are plain
  language rather than standard phraseology; `SAY AGAIN` is standard and present.
- **Sector handoffs fire on crossing the boundary, not before it.** On the
  Annecy departure the transfer to Geneva came at 11 542 ft against a sector
  floor of 11 500 ft — 42 ft *after* entry. Detection is exact; the timing is
  reactive by design. That 42 ft is not representative: the aircraft had drifted
  out of the lower sector's footprint, and on the published SID track the
  boundary is crossed at **FL085** — with the procedure's own FL130 requirement
  sitting 4 500 ft inside the neighbouring TMA. Real ATC hands off while the aircraft is still a minute or
  two out, so the pilot is already on the new frequency when they cross.
  Related: on that same departure the FL130 clearance is issued by Chambéry for a
  level that lies inside Geneva's airspace. That is correct and deliberate — the
  SID publishes a minimum of FL130 at a fix 7 NM out, and Chambéry's own airspace
  tops at 9 500 ft, so no level it "owns" could satisfy the procedure. A published
  SID crossing into a neighbouring TMA implies coordination, which is what real
  ATC does.
- Upper-airspace naming is corrected for France only. Other countries whose
  navdata collapses the UIR into a single country-level entry will still be
  announced by that name, and no country reproduces handoffs between sectors of
  the same regional centre.
- `"readback please"` in the VFR templates is non-standard, but those templates
  belong to upstream and are left alone.
