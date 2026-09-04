# The descent ladder — how a level is chosen, and when

Specification of the IFR arrival descent: which altitude ATC gives, and the moment
it decides to give it.

**Spec version:** 1.4 · **Dated:** 2026-08-29 · **Build:** v4.4.0-beta (pkg 133+)
**Reference slope:** `kDescentSlopeFtPerNm = 265 ft/NM` = 2.5°

All figures below are measured on the DIK → EDLW replay of the flown route, taken
from the engine's own log lines rather than read off the source.

---

## 1. Profile view

```
        │                                                              alt
 FL280  ●━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━●━━●
        ╎                                                   ╎   ╲
        ╎  · · · 2.5° reference from the computed TOD · · ·  ╎    ╲
 FL200  ╎         · · · · ·                                 ╎     ╲
        ╎                   · · · · ·                       ╎      ╲
 FL120  ╎                             · · · · ·             ╎       ╲
        ╎                                       · · · · ·   ╎        ●━━━━━━━━━━━━╲
 FL060  ╎                                                 · ╎· · · ·              ╲━━●━━╲
        ╎                                                   ╎        · · · · · ·        ╲
  3000  ╎                                                   ╎                             ●
        └────────┴──────────┴──────────┴──────────┴─────────┴────────────┴─────────────────┘
       120     102.3        88         82         70       68            10                0
                 ▲                      ▲          ▲        ▲             ▲                ▲
                 │                      │          │        │             │                │
              computed              "advise      FL180    FL100          FL60          3000 at DOR
                TOD                when ready"
```

The aircraft leaves cruise **32 NM after** its own computed top of descent and never
regains the reference slope. The two levels at 70 and 68 NM come from **two different
mechanisms** that do not know about each other.

---

## 2. Three mechanisms, three owners

| | question it answers | owner | how the level is chosen |
|---|---|---|---|
| **Descent negotiation** | when do we leave cruise? | en route (ACC) | `cruise × 0.66`, rounded |
| **The ladder** | when must we be down for the approach? | terminal | walk the published constraints |
| **Descend to enter** | how low must we be to be ALLOWED into the terminal area? | the sector handing over | highest whole thousand BELOW the TMA ceiling |

Three, not two. The third was documented only as a footnote until pkg 107, and
it is the one that failed on the flight of 2026-08-20 — see §2.3 and §8.

### 2.1 The descent negotiation

Asks *"advise when ready to descend"*, then clears — the moment the aircraft stops
being an en-route aircraft.

```
level = cruise × 0.66, rounded to whole thousands
      = 28 000 × 0.66 = 18 480 → FL180
```

The level is a **proportion of the cruise altitude**, tied to no published constraint.
See defect D1.

### 2.2 The ladder

Looks ahead to the first fix that constrains the aircraft, computes the descent that
fix demands, and walks down to it one rung at a time.

```
TOD    = (altitude − target) ÷ 265 ft/NM + reaction
target = first point of the CLEARED approach
fires  = when the distance still to run drops inside the TOD
```

Measured: `(28 000 − 3 000) ÷ 265 = 94.3 NM` + reaction = **102.3 NM**.

### 2.3 Descend to enter the terminal area

Neither of the two above asks the question that actually gates a terminal arrival:
*is the aircraft low enough to be let into the TMA at all?* An openair terminal
volume has a ceiling, and an aircraft above it is not in that controller's
airspace. So the sector working the aircraft descends it to the highest whole
thousand **below** that ceiling — `((ceiling − 100) ÷ 1000) × 1000`, so 9500
gives 9000 and 7500 gives 7000 — **before** handing it over.

Note the rounding runs the OTHER WAY from the ladder's TMA rung in §4, and
deliberately: the rung rounds **up** to sit the aircraft just above the terminal
area it is not yet cleared into, this one rounds **down** to put it inside. Same
ceiling, opposite questions.

```
[dbg dte] tma_ceil=9500 target=9000 alt=15011 cleared=15000 floor=7000 last=0 -> FIRE
IFR descent: descend-to-enter terminal area, TMA ceil 9500 -> flight level 90
```

The trigger is geometric, not a route point: either the enclosing volume already
IS the destination's terminal controller, or the destination's terminal TMA has
appeared **directly below** while the aircraft is still under an overlying ACC.
Bounded at 100 NM from the field, because a terminal area is never a hundred
miles from its own aerodrome.

Three rules govern who may speak it (pkg 105–107):

1. **One target per flight.** `s_descent_tma_target_ft` latches what was issued;
   the same target is never repeated. This latch is the subject of §8.
2. **It yields to a transfer owed in the same frame.** A level belongs to the
   controller about to work the aircraft, not to the one leaving: entering the
   TMA is simultaneously what justifies the descent and what makes the next
   controller responsible. When the terminal handoff is due, the step-down is
   held and re-run *after* the handoff block — in the same frame, so nothing is
   lost if no handoff materialises (AFIS destinations with no approach
   controller).
3. **It never speaks across an unanswered handoff.** Between "contact X on
   \<freq\>" and the pilot's check-in there is no controller in a position to
   clear: the outgoing one has released the aircraft, the incoming one has not
   heard it.

---

## 3. Choosing the target

The ladder never invents an altitude. It walks the path the aircraft will actually fly
and offers every fix carrying a published level as a candidate, each measured at **its
own distance**. The candidate forcing the earliest descent wins.

| order | source | distance used |
|---|---|---|
| 1 | remaining route fixes | routed, leg by leg |
| 2 | the STAR **in published order**, loops included | routed, leg by leg |
| 3 | the FAF of the approach being flown to | routed to the last known fix, then direct |

Under `FORCE APP VECTORING` the **direct** distance is also offered and the shorter
wins, because a vectored aircraft is cut across its route. Under `ALLOW VECTORING` it
is not — see the coordination assumption in `force-app-vectoring.md`.

**When the STAR publishes no constraint at all** — EDLW's ADEM3A carries none — the
target becomes the **first point of the cleared approach**, whatever the sign of its
published constraint. Not the FAF, and not the first fix that happens to force a
descent.

---

## 4. Choosing the rung

A level is never given straight to the target: 15 500 ft in one transmission is not
something a controller says. Rungs are structural, taken highest-first, and only when
one lies genuinely between the aircraft and the target.

| rung | why it exists | on this arrival |
|---|---|---|
| `FL100` | the 250 kt boundary, and the top of most terminal areas | given at 68 NM |
| `TMA ceiling` | destination terminal area, rounded up to the next 1000 ft | **absent — see below** |
| `6000` | the usual last level before the platform | given at 10 NM |
| `target` | the constraint itself | 3000 at DOR |

**At EDLW the TMA rung does not exist at all.** Measured 2026-08-17,
`terminal_tma(EDLW)` returns empty and `terminal_tma_ceiling` returns 0 — nothing
over Dortmund is classified as a TMA:

```
  1000 ft -> DORTMUND CTR         0 - 2500      class CTR
  3000 ft -> DORTMUND SECTOR B    2500 - 4500   class CTA
  6000 ft -> nothing
  9000 ft -> nothing
 12000 ft -> AIRSPACE CLASS C    10000 - 66000
```

The terminal structure is a control zone to 2500 and a CTA-classified sector to
4500, with a hole from 4500 to 10 000 ft. What would fill that hole is the class E
blanket published to FL100, and the OpenAir export carries no class E, F or G at
all. So the rung is skipped because the value is **zero**, not because it is low —
an earlier revision of this document wrongly said "4500 → skipped".

Consequences beyond the rung: `descend-to-enter-TMA` can never fire here either,
which is why an EDLW arrival depends entirely on the CIFP constraint chain.

Three rules bound it:

1. The next rung is withheld until the aircraft has flown the previous one, within
   1000 ft.
2. A rung may only ever clear the aircraft **higher** than the raw target, so it can
   never introduce a terrain or airspace bust the target did not already carry.
3. A level is never re-issued **above** one already given; a stale target is consumed
   silently instead.
4. A level already **transmitted** is never transmitted again, across controller
   changes included — an instruction stays in force through a transfer and the
   receiving controller does not repeat it. The record is
   `s_last_transmitted_alt_ft`, written where the words are emitted and cleared
   only by a new flight. It deliberately holds what was **spoken**, not what the
   profile has merely planned: two earlier attempts compared against the planned
   level instead, which matched on every arrival and suppressed the descent
   entirely — the replay then intercepted 3325 ft above the glide path.
5. QNH is stated once per frequency (`s_qnh_stated_freq`), and only when the
   level is given in feet — never with a flight level.

---

## 5. When the gradient is steep

ATC knows the gradient it is asking for at the moment it asks. Past **1.35×** the
reference slope, the clearance is worded as an instruction rather than a level:

```
[dbg prof] steep: 434 ft/NM needed to 3000 feet in 9.2 NM
ATC: expedite descent to 3000 feet, QNH 1024.
```

434 ft/NM is 4.1°. A separate monitor watches the other half — an aircraft that is
*not* descending enough — judged against **the fix the level is tied to**, which on an
approach is due before the FAF, not overhead the field.

---

## 6. Reading it in a flight log

Every step-down states its own reasoning, without debug mode:

```
IFR descent: DOR -> rung flight level 100 (target 3000 ft at DOR)
             @ PA 28000, 85.8 NM to go, TOD 102.3 NM, 25000 ft to lose
```

The constraining fix · the level transmitted · the real target when that level is only
a rung · the arithmetic that fired it. Here it also shows the aircraft was already
16 NM past its own top of descent when the rung was given.

When the level given **is** the constraint rather than a rung, the word `rung` is
absent and the target is not repeated.

---

## 7. Known defects

**D1 — the first level is a proportion, not a constraint.**
`cruise × 0.66` produced FL180, superseded by the ladder's FL100 two miles later. The
two mechanisms answer different questions without knowing about each other. Replacing
the heuristic with the published-constraint walk the ladder already performs would
remove the collision.

**D2 — the ladder plans on the routed track even when the aircraft will be vectored.**
*Measured 2026-08-17, and the error has two signs, not one.* Every descent
clearance now logs all three distances:

```
IFR descent: distances to DOR -- routed 64.4, direct 47.3 NM (routed is +17.1)
             | gradient routed 123, direct 168 ft/NM
```

The +17.1 NM is the published reversal a vectored aircraft never flies — at EDLW
the ILS 06 transition runs overhead the field (`DOR` is the field VOR, 0.9 NM
from the threshold) and back out to `KOLOT` 6.6 NM southwest. Routed therefore
**overstates** while the descent is planned, so it fires late and is then
genuinely steep. Once on a downwind leg the opposite happens: every remaining fix
falls more than 100° off the nose, is skipped, and routed collapses to the
straight line, **understating** the track and wording "expedite" with room to
spare. Nothing consumes the figures yet — they are logged so the next flight
settles which one dominates.

**D4 — the vectored descent held one level to the FAF.** *Fixed 2026-08-17.* A
vectoring leg was cleared to `FAF + 2000`, issued once and never revisited, so
the aircraft crossed 1.6 NM from the FAF at 4461 ft with the 3° path at 3000 —
1460 ft high. Leg levels are now derived from the path itself (`FAF alt + s×318
− 300`), floored at the FAF crossing altitude and by the MSA, bounded by what the
reference gradient reaches over the **vector track**, and stepped down as the
geometry changes. See `force-app-vectoring.md` §Altitudes.
The pre-TOD calculation takes the shorter of routed and direct when vectoring is
certain; the ladder always uses the routed distance. The two differ by ~20 NM on this
arrival, so every rung comes later than it should. → `open-questions.md`, Q1.

**D3 — two tops of descent are merged into one.**
Leaving cruise belongs to the en-route controller; descending for the platform belongs
to the terminal. The code keeps a single "most binding candidate" across route, STAR
and approach fixes. Whether to model the two explicitly is undecided.
→ `open-questions.md`, Q2.

---

## 8. How long the descent state lives

Every mechanism above carries a latch — the target already issued, the level
already transmitted, the rung already flown. All of them are **per flight**, and
until pkg 107 "per flight" meant *per plugin load*: `engine::reset()` ran when
X-Plane enabled the plugin, and never again.

Restarting a flight, reloading a situation or jumping from the map does none of
that. On 2026-08-20 the same LFML → LSGG arrival was flown three times inside one
X-Plane session, and the state of the first attempt survived all three:

```
attempt 1   [dbg dte] tma_ceil=9500 target=9000 alt=15011 cleared=15000 last=0 -> FIRE
            IFR descent: descend-to-enter terminal area, TMA ceil 9500 -> flight level 90
attempt 3   [dbg dte] tma_ceil=19500 target=19000 alt=13116 cleared=10000 last=9000 -> hold
```

`last=9000` is the latch from attempt 1. `descend flight level 90` fired **once
in the whole session**. The aircraft therefore held FL150 until the expedite net
caught it, reached the vectoring point 6000 ft above its FAF, and the sequencing
leg then ran 24.7 NM against a lateral requirement of 16.7 — the leg was long
because the aircraft was high, not because the geometry asked for it. The same
stale state had Geneva Approach speaking while the aircraft was back near
Marseille, two hundred miles away and on another frequency.

**Since pkg 107** a position discontinuity larger than
`max(15 NM, groundspeed × dt × 3)` is read as a restart: `engine::reset()` clears
all 145 engine variables and the state machine returns to `IDLE`.

```
IFR: position discontinuity 132 NM in 60.0 s at 280 kt (46.3125,6.4234 -> 44.4502,4.7794)
     -- flight restart, resetting the ATC state
```

What survives on purpose is the **SimBrief OFP** — `reset()` does not touch it —
which is what makes the wipe safe. Destination and STAR are rebuilt either by the
descent clearance at the TOD, or by the defensive seed at the Approach check-in
(`[approach] seeded dest ICAO from OFP`, STAR re-derived from the CIFP by entry
fix then by runway).

**Blind spot:** a restart that leaves the aircraft in place — same parking, or a
jump under 15 NM — is invisible, and the old state persists exactly as before.

Regression: `make test-restart`.

---

## 9. The descent sizes the vectored pattern

Not a descent rule, but the coupling is where a defect hid twice. On a vectored
arrival the outbound sequencing leg runs until there is room for **both** the
intercept and the descent:

```
downwind length = max( lateral requirement , (altitude − FAF alt) ÷ 265 + 2 )
```

So an aircraft delivered high does not get a shorter pattern, it gets a longer
one — the leg is flown to lose the altitude. Judging the vectoring geometry
without reading the altitude at the vector point therefore measures the wrong
thing. Details in `force-app-vectoring.md`.

---

*Specification in progress. This document covers the descent only; the vectoring
manoeuvre is specified in `force-app-vectoring.md`.*

---

## 10. The climb is the same ladder, upside down

Everything above sizes a descent. A climb was, until 2026-08-29, unrestricted:
`poll_sid_climb` issued step1 and step2 out of the TMA and then cleared **cruise
in one clearance**, however far away it was. On the LFMN -> LOWI flight of
2026-08-28 that produced, at the pilot's first call to Milan at FL140:

```
Milan: November Romeo Charlie, radar contact, climb flight level 450.
```

31 000 ft in one transmission. The rule is now symmetric with the descent:

| | descent | climb |
|---|---|---|
| cap on one clearance | `kMaxSingleDescentFt` = 10 000 ft | `kMaxSingleClimbFt` = 10 000 ft |
| remainder held in | `s_descent_final_target_ft` | `s_climb_final_target_ft` |
| who owes the rest | `poll_descent_second_step` (one extra step) | `poll_climb_next_step` (loops to cruise) |

`climb_step_ft(now, final)` returns the level to clear now — a whole flight
level, never an odd altitude — and the caller stashes the remainder. Three
emitters share it: the FIR handoff that queues a climb for the pilot's check-in,
the combined cruise+TMA-exit clearance, and the in-block cruise fallback. So a
FL450 departure now reads FL110, FL140, FL260, FL360, FL450.

**Who speaks the next rung.** Whoever has the aircraft when it levels off. While
a handoff is in flight (`s_sector_checkin_pending`) the ladder stays silent: the
rung is the first thing the new sector clears once the pilot checks in, which is
how it sounds on frequency. The rung is deliberately *not* queued into
`s_sid_pending_climb_ft` — `poll_sid_climb` wipes that field on every frame the
state is not `IFR_RADAR_CONTACT`, so a queued rung outside the SID phase would be
erased and the aircraft would sit at an intermediate level for ever. A 180 s
grace guards the same failure from the other side: if no check-in ever comes, the
sector that still has the aircraft clears the rung itself.

The ladder is abandoned the moment the state leaves `IFR_RADAR_CONTACT` /
`IFR_ENROUTE_CRUISE` — from there the descent owns the profile.

`make test-climb`.

---

## Revision history

| version | date | build | change |
|---|---|---|---|
| 1.0 | 2026-08-17 | v4.4.0-beta-85 (`a6bb13b`) | first issue: profile view, the two mechanisms, target and rung selection, steep wording, log format |
| 1.1 | 2026-08-17 | v4.4.0-beta-85 (`a6bb13b`) | corrected the TMA rung at EDLW — it is absent because `terminal_tma` returns 0, not because the ceiling is low |
| 1.3 | 2026-08-21 | v4.4.0-beta (pkg 107) | **descend-to-enter-TMA promoted to a third mechanism** with its own section: the ceiling-below trigger, the one-target latch, and the two rules added in pkg 105–107 (it yields to a transfer owed in the same frame, and never speaks across an unanswered handoff). New §8 **how long the descent state lives** -- the latch used to survive a flight restart inside one X-Plane session, which is what held an LSGG arrival at FL150 and made its sequencing leg run 24.7 NM; restart detection since pkg 107, with its blind spot recorded. New §9 on the descent sizing the vectored pattern. Level rules 4 and 5 written down: never re-transmit a level in force (transmitted, not planned), QNH once per frequency |
| 1.4 | 2026-08-29 | v4.4.0-beta (pkg 133+) | new §10 **the climb is the same ladder, upside down**: `kMaxSingleClimbFt`, `climb_step_ft`, `poll_climb_next_step`, and the two rules that keep a rung from being lost across a handoff (silence during the transfer, 180 s grace if the check-in never comes) |
| 1.2 | 2026-08-17 | v4.4.0-beta (`d7c3f64`, pkg 87) | D2 measured on a real flight — routed overstates by 17.1 NM before the vectors and collapses to the straight line once on downwind, so the error has two signs; all three distances now logged at every clearance. New D4: the vectored descent held one level to the FAF (1460 ft high at 1.6 NM), now derived from the glide path and stepped |
