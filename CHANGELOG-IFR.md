# Changelog — IFR

Release notes for the IFR feature set of this fork. Platform and build notes are
in [README-LINUX.md](README-LINUX.md) / [README-WINDOWS.md](README-WINDOWS.md);
what the IFR flows do and their known limits are in
[README-IFR.md](README-IFR.md).

Every entry below was diagnosed from a captured `Log.txt` / `transcript.log` pair
of a real flight, and says which one.

---

## Unreleased — 4.4.0-beta

Two flights drove this batch: **LFLP → LFMN** (2026-08-14 morning) and
**LFLP → EDLW** (2026-08-14 afternoon, the first long cross-border leg into
Germany).

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
  resolved a controller by polygon only. At Dortmund there is none to find — the
  OpenAir export has a vertical hole from 4500 ft to 10000 ft (it carries no
  class E at all, and the German approach layer there is class E), and `atc.dat`
  lists Dortmund as a Tower, not an approach unit. Both are right: the service is
  Langen Radar. The field's own published approach frequency is now the last
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
  reactive by design. That margin happened to be small because the climb left
  the lower sector's footprint before reaching its 9 500 ft floor; a track that
  stays over it would enter 2 000 ft earlier and be handed off just as late. Real ATC hands off while the aircraft is still a minute or
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
