# The descent ladder — how a level is chosen, and when

Specification of the IFR arrival descent: which altitude ATC gives, and the moment
it decides to give it.

**Build:** v4.4.0-beta-85 (`a6bb13b`) · **Dated:** 2026-08-17
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

## 2. Two mechanisms, two owners

| | question it answers | owner | how the level is chosen |
|---|---|---|---|
| **Descent negotiation** | when do we leave cruise? | en route (ACC) | `cruise × 0.66`, rounded |
| **The ladder** | when must we be down for the approach? | terminal | walk the published constraints |

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
The pre-TOD calculation takes the shorter of routed and direct when vectoring is
certain; the ladder always uses the routed distance. The two differ by ~20 NM on this
arrival, so every rung comes later than it should. → `open-questions.md`, Q1.

**D3 — two tops of descent are merged into one.**
Leaving cruise belongs to the en-route controller; descending for the platform belongs
to the terminal. The code keeps a single "most binding candidate" across route, STAR
and approach fixes. Whether to model the two explicitly is undecided.
→ `open-questions.md`, Q2.

---

*Specification in progress. This document covers the descent only; the vectoring
manoeuvre is specified in `force-app-vectoring.md`.*
