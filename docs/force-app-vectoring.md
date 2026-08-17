# FORCE APP VECTORING — specification

**Spec version:** 2.1 · **Dated:** 2026-08-17 · **Build:** v4.4.0-beta (`11365b5`)

Status: **implemented and flown twice, neither flight completing the arrival.**
The manoeuvre itself works — build 85 flew it to the localiser — but the
approach was never handed to Tower on either flight. See *Known defects* at the
end before reading this as a description of working behaviour.

---

## The governing rule

> **The last vector assigns the FINAL APPROACH COURSE, and the aircraft must be
> established on that axis AT LEAST 2–3 NM BEFORE THE FAF.**

This is not a refinement of the sequence — it is the constraint that
*dimensions* the whole manoeuvre. Everything below is derived from it.

### The rule as ICAO states it, and what we had wrong (2026-08-17)

Our numbers were **stricter than the standard**, and that is what refused a
legal manoeuvre on the EDLW arrival of 2026-08-17. ICAO Doc 4444 (PANS-ATM)
requires the final vector to:

- provide an intercept angle with the final approach track of **45° or less**
  (30° or less only for independent parallel approaches);
- establish the aircraft on that track, in level flight, **at least 2.0 NM
  before it intercepts the glide path**;
- provide at least **1.0 NM straight and level** before the track intercept;
- and have the aircraft intercept the glide path **from below**.

| | ours, before | ICAO | now |
|---|---|---|---|
| intercept angle | 30° | 45° max | 30° **target**, up to 45° when tight |
| aligned before the FAF | 3.0 NM | 2.0 NM | 3.0 NM **target**, 2.0 NM floor |

The distinction between a *target* and a *floor* is the whole point. Between the
two the manoeuvre is steeper than we would choose, entirely legal, and far
better than handing the aircraft back its published procedure a few miles from
the FAF. Measured at the abandon point of that flight — 1.5 NM off axis, 5.1 NM
before the FAF:

```
our rule    3.0 + 1.5/tan 30  = 5.6 NM  ->  refused, "resume own navigation"
ICAO floor  2.0 + 1.5/tan 45  = 3.5 NM  ->  1.6 NM of margin
```

The clearance issued under vectors must also carry the level to hold **until
established**, which is what makes it licit to clear an aircraft that is not yet
on the axis — and why `report established` exists at all. The reference adapts
to the approach type: an ILS or LOC approach has a **localiser**, an RNP/RNAV or
VOR approach does not, and gets the generic form.

Two consequences that must be implemented as such:

1. The final vector's heading **is the axis**, not a 30° intercept heading.
   The intercept leg ends before it; the last leg is flown aligned.
2. It is a **feasibility test evaluated continuously**, not just a sequence
   step. At every moment the algorithm must be able to answer: *can I still
   align the aircraft 3 NM before the FAF?* If the answer becomes no, the
   manoeuvre does not degrade into a late steep intercept — it either extends
   the downwind by one leg, or abandons vectoring and hands back the published
   procedure.

It also gives the acceptance test, measurable without flying: at the moment
`cleared <approach>, report established` is issued, the heading error to the
final course must be < 5° and the distance to the FAF ≥ 3 NM.

## Two modes, not one (user, 2026-08-16)

Radar vectoring is not all-or-nothing. There are two distinct manoeuvres, and
LOWI is the case that makes the difference obvious:

| mode | what ATC does | when |
|---|---|---|
| **vectors to final** | the four legs below, established on the axis 3 NM before the FAF | terrain permits AND there is a straight axis after the FAF |
| **vectors to the IAF** | radar guidance onto the IAF, then the published procedure | terrain, or no axis after the FAF — **LOWI** |

The second is **not a degraded fallback**: it is what a real controller does
there. *"Il y a trop de relief a LOWI pour avoir un guidage radar vers le FAF"*
— the guidance is to the IAF, and the aircraft flies the published procedure
from it. This is what makes the decision to keep `poll_vector_to_intercept` the
right one: the IAF teardrop is the natural CONTINUATION of mode 2, not a spare
wheel.

### Choosing the mode

Two tests, both data-driven; failing either selects vectors-to-the-IAF.

**1. Terrain.** The pattern must be flyable at its own leg altitudes. Compare
the sector MSA over the pattern area against the altitudes the legs require. At
LOWI:

```
LOWI sector 270 : MSA    14300 ft
IAF ELMEM publishes      13000 ft
```

The MSA sits **above** the platform altitude, so the aircraft cannot descend
into the pattern at all — vectors to final are impossible, whatever the
procedure geometry says. This is finally a real job for `msa_db` rather than
only a guard.

**MSA is authoritative here; MORA must NOT be used to choose the mode.**
(user, 2026-08-16: *"LFLP 18200 c'est juste dans un secteur donne"*.) A MORA
cell is 1 degree, roughly 60 NM, so near mountains it reports terrain that is
nowhere near the approach. Measured on the reference data:

| | MORA cell / neighbourhood | MSA sectors around the approach point |
|---|---|---|
| LFLP | 18200 / 18200 | CBY: **6500 / 7200 / 9700** |
| LFMN | 12100 / 15900 | LFMN: **3100 / 3200 / 5600**, 12500 to the east |
| LOWI | 14000 / 15700 | ELMEM: 10700 / 11400 / **13100 / 14300** |
| EDLW | 3700 / 4400 | DOR: **2800 / 3700** |

At Annecy the MORA sits 8000-11000 ft above the actual approach sectors -- it is
reporting Mont Blanc, 40 NM away. Selecting the mode on MORA would forbid
vectoring at Annecy and Nice for terrain that is not on the flight path. LOWI is
refused on its MSA alone (14300 against ELMEM's 13000); MORA is not needed to
reach that verdict.

So the precedence is: **MSA decides. MORA is only the last-resort floor when the
MSA is silent** (outside the sector radius, or an airport that publishes none),
and it is expected to be pessimistic near terrain -- acceptable for a floor,
wrong for a decision.


**2. Geometry: no axis after the FAF**

The rule assumes a **straight** final approach axis. The precise test (user,
2026-08-16) is **whether there is a straight axis AFTER the FAF** -- not merely
whether the procedure contains an arc somewhere.

LOWI R08-Z shows why the distinction matters. Read from the reference CIFP:

```
seq  fix     desc    path_term
020  WI749   E  F    IF     <- the FAF (4th descriptor char = F)
021  WI751           TF
022  WI752           RF     <- arc, AFTER the FAF
024  WI753           TF
027  WI754           RF     <- arc, AFTER the FAF
030  RW08    GY M    TF     <- threshold
```

The legs leading *into* the FAF (ELMEM -> WI749) are straight, so alignment 3 NM
before the FAF would be geometrically achievable. But the path **after** the FAF
curves, so there is no final axis to be established on, and neither the last
vector's heading nor the `< 5 deg` test means anything.

So **FORCE APP VECTORING must select vectors-to-the-IAF when the segment after
the FAF is not a straight track**, rather than vectors to final. Data-driven
test: walk
the chosen approach's final segment from the FAF towards the threshold and look
for a non-straight `path_term` (`RF`, `AF`). An earlier draft of this document
tested for `RF` anywhere in the approach, which is too broad -- an arc in the
*intermediate* segment does not prevent a straight final.

```
[vector] R08-Z has no straight axis after FAF WI749 (RF legs) -- vectoring to IAF ELMEM instead
[vector] LOWI MSA 14300 > platform 13000 -- terrain forbids vectors to final, vectoring to IAF
```

Related known defect: the existing alignment check already assumes a straight
axis and produces false "confirm established" calls on these finals
([[project_curved_final_alignment]]). Vectoring must not add a second consumer
of that wrong assumption.

---

## Relationship with the IAF teardrop (`poll_vector_to_intercept`)

**Decision (user, 2026-08-16): keep the teardrop, coexist.** Not a stopgap --
it is the fallback path. Whenever vectoring refuses (no MSA, under the 15 NM
floor, curved final), something still has to put the aircraft on the IAF course,
and that is what the teardrop does.

Coexistence is close to automatic: vectoring engages ~28 NM from the FAF while
the teardrop arms at 2.5 NM from the IAF, and an aircraft on vectors has left
the published transition, so the teardrop's precondition never becomes true. It
is listed in the suppressions below only to make that explicit rather than
incidental.

---

## The two settings

| setting | meaning |
|---|---|
| `allow_vectoring` | ATC *may* vector when it makes sense: off-route recovery, an impractical reversal, sequencing. ATC decides. |
| `force_app_vectoring` | *Every* arrival is vectored; the published transition is ignored. Instruction mode. |

### UI requirement (user, 2026-08-16)

When no MSA data is available for the destination, the checkbox must SAY SO
rather than silently doing nothing. Label beside `FORCE APP VECTORING`:

```
[x] FORCE APP VECTORING   DISABLED: NO MSA
```

The setting stays where the user left it -- the label reports why it is inert,
it does not un-tick the box. Plain ASCII only (ImGui renders UTF-8 as `?`).

Evaluated per destination, since coverage is per airport: the reference data has
MSA records for LOWI, LFMN, EDLW and LFLP, so absence is the exception, not the
rule -- but it must be visible when it happens.

---

## Where the manoeuvre starts

Measured backwards from the FAF, never forwards from the aircraft:

```
D_start = offset + 2.6  +  1.73 x offset  +  0.5  +  3
          \_base+turns_/   \__intercept__/  \roll/  \_ALIGNED_/
```

- `1.73 x offset` — closing a lateral offset at 30° costs `offset / tan 30°`
- `0.5` — roll-out from the intercept heading onto the axis
- `3` — **the aligned segment before the FAF: the rule above**
- `2.6` — two ~90° turns; at 200 kt / 25° bank the radius is ~1.25 NM, so ~2 NM
  of arc each

**"Offset" = the LATERAL distance between the downwind leg and the final approach
axis** — how far to the side the aircraft is flown before turning base:

```
                  downwind (leg A)
   <-------------------------------------------------o
                                                     |
      offset  |                                      | base (B)
              v                                      v
   ====FAF====================RWY 06           o<----o
        ^                                      |
        |  3 NM aligned                   intercept 30 deg (C)
        +----- final course (leg D) -----+
```

A larger offset means a wider pattern: more room to descend and slow down, but a
more distant trigger. **8 NM chosen** (user, 2026-08-16) — for a TBM that gives a
base leg of roughly 3 minutes, comfortable without being tedious.

| offset | trigger | aligned segment |
|---|---|---|
| 5 NM | ~20 NM | 3 NM |
| **8 NM (proposed default, TBM)** | **~28 NM** | 3 NM |
| 10 NM | ~33 NM | 3 NM |

Distance to the FAF is **great-circle, not routed**: under vectors the aircraft
has left the route and the legs are straight.

**Hard floor ~15 NM**, and it is a *consequence*, not a chosen number: below it
the geometry cannot deliver alignment 3 NM before the FAF even at a minimal
offset. Under the floor, ATC refuses to vector and keeps the published
procedure — an explicit refusal, never a degraded vector.

---

## The four legs

```
A  downwind   "turn left heading 210, descend 5000 feet QNH 1013,
               vectoring for ILS approach runway 06"
B  base       "turn left heading 120, reduce speed 180 knots"
C  intercept  "turn left heading 090"                      <- 30 deg max
D  axis       "turn left heading 060, descend 2500 feet,
               cleared ILS approach runway 06, report established"
                                                           <- THE AXIS COURSE
```

Then `contact Tower on <freq>` on "established". If abandoned:
`resume own navigation direct <fix>`.

Turn side: whichever avoids crossing the final course — the aircraft always
joins from the outside.

---

## Altitudes

**Superseded 2026-08-17 — a fixed offset above the FAF is above the glide path
exactly where that matters.** The nominal 3° path rises 318 ft per NM before the
FAF, so:

```
FAF + 2000  is below the path only beyond  6.3 NM
FAF + 1000  is below the path only beyond  3.1 NM
```

Measured on the EDLW arrival: the aircraft crossed **1.6 NM from the FAF at
4461 ft with the path at 3000** — 1460 ft high — and was then told *"continue
descent to 3000"* for a FAF published at **2500**. Too late, and not low enough.

A leg level is now bounded by three constraints, in this order, then floored by
the sector MSA / grid MORA as before:

| | constraint | why |
|---|---|---|
| 1 | at most `FAF altitude + s × 318 − 300 ft` | intercept the glide path **from below** (ICAO) |
| 2 | never below the FAF crossing altitude | the aircraft joins the vertical profile there |
| 3 | never below what the reference gradient reaches over the **vector track** still to fly | ATC cannot order a level that needs a dive |

And the level is **re-evaluated as the geometry changes**, not issued once. A
level correct 30 NM out is above the path 5 NM out, so it is stepped down when it
has drifted 400 ft — heading untouched, because this is a level, not a vector.
Replayed against the flight: `4500 → 4100 (~6 NM) → 3700 (~4.3 NM)` instead of
holding 4500 all the way to the FAF.

The alignment leg targets **the FAF crossing altitude itself**, not a round
number above it.

This closes the rule settled on 2026-08-16
(`coding_last_assigned_altitude`): the FAF altitude is **wrong** on a published
transition — there the assignment is the *first point of the cleared approach*
— and **right** under vectors. Vectoring is precisely the exception that rule
carves out.

**Non-negotiable guard.** `msa_db::minimum_ft()` returns 0 when it has nothing
to say (outside the sector radius, or no record). Zero does not mean "no
minimum". In that case do not descend below the last cleared level. There is no
secondary net today — no MORA reader exists — so either one is written
(`earth_mora.dat` is present in the user's data) or ATC refuses to vector where
the MSA is silent. **Recommended: write the MORA reader**, otherwise instruction
mode is simply unavailable over uncovered areas.

---

## What must be suppressed while vectoring

The part that is easy to forget and breaks everything:

- the direct-to-IAF shortcut and `direct XX, when able` — meaningless under vectors
- the off-route monitor, or ATC will fault the aircraft for flying ATC's own vectors
- the "first point of the cleared approach" assignment rule (see above)
- the route tracker, which must be frozen rather than re-synced
- the IAF teardrop `poll_vector_to_intercept` — it cannot fire anyway (see above),
  but suppress it explicitly so the interaction is stated, not incidental

---

## Logging (user, 2026-08-16)

Everything above must be traceable in `Log.txt`, not only visible in the UI --
the flight is diagnosed from the logs afterwards, and a refusal that leaves no
trace is indistinguishable from a bug. `XPLMDebugString`, plain ASCII (0x20-0x7E).

One line when the mode is evaluated for the destination, whichever way it goes:

```
[vector] FORCE APP VECTORING armed for EDLW: MSA ok (2 records), offset 8 NM, trigger 28 NM
[vector] FORCE APP VECTORING DISABLED: NO MSA for EDLW -- vectoring refused
```

One line per decision, so the sequence can be reconstructed without the transcript:

```
[vector] leg A downwind hdg 210, alt 5000 (MSA 4300, platform+2000)
[vector] leg B base     hdg 120, speed 180
[vector] leg C intercept hdg 090 (30 deg to axis 060)
[vector] leg D AXIS     hdg 060, alt 2500, 4.2 NM to FAF KOLOT   <- the rule, measured
[vector] established: hdg err 2 deg, 3.8 NM to FAF -- OK
```

And every refusal or abandonment states its reason and its numbers:

```
[vector] refused: 12.4 NM to FAF, below the 15 NM floor -- keeping the published procedure
[vector] cannot align 3 NM before FAF -- extending downwind by one leg
[vector] abandoned: hdg err 34 deg for 30 s -- resume own navigation
[vector] MSA silent at 51.42,7.81 -- holding 5000, no further descent
```

The `leg D` and `established` lines carry the two numbers the governing rule is
about, so the acceptance test can be run against a real flight log and not only
against the replay.

---

## Compliance

`heading_error_deg` already exists. If the error exceeds 20° for 30 s, re-issue
once; if it persists, abandon cleanly with `resume own navigation`. Without
this, a pilot who does not follow ends up vectored into nowhere.

---

## Work breakdown

| item | note |
|---|---|
| MORA reader | new, small — prerequisite for the altitude guard |
| wire `msa_db` | written but currently called from nowhere — prerequisite |
| 2 settings + UI | small |
| `poll_vector_to_final` (4-state machine) | the bulk |
| the suppressions above | highest regression risk |

Existing bricks: `heading_error_deg`, `approach_needs_reversal_vector`,
`poll_vector_to_intercept` — note the last is a *teardrop reversal at the IAF*
(armed at 2.5 NM, ≥100° turns only), not radar vectoring; decide whether
`force_app_vectoring` replaces it or coexists with it.

## Decisions taken (user, 2026-08-16)

1. **Downwind offset: 8 NM.**
2. **MSA missing: refuse to vector**, and say so in the UI —
   `DISABLED: NO MSA` beside the FORCE APP VECTORING checkbox. Refusing is the
   only choice that rests on no assumption; announcing it is what stops the
   refusal from looking like a bug.

## Still open

- Does `force_app_vectoring` replace `poll_vector_to_intercept` (the IAF teardrop,
  armed at 2.5 NM for >=100 deg reversals) or coexist with it?

---

## Planning the descent on the vectored track (and what it assumes)

When vectoring is CERTAIN, the descent must be planned on the track the aircraft
will actually fly, which is the direct one -- not the published route. Measured
on the DIK -> EDLW arrival: the routed chain to the FAF was ~92 NM while the
direct track was ~72. Planning on 92 NM handed the profile 20 NM it was never
going to get, and the aircraft ended up needing more than 2000 fpm to make the
platform (user, 2026-08-16). Before the fix the descent clearance came at 70 NM
when the top of descent was 96; after it, at 97 NM, which is 2.8 degrees instead
of 4.1.

**The assumption this rests on, stated plainly.** An en-route controller may only
plan a descent on a shortened track if he already knows the terminal unit is
going to vector -- that is a COORDINATION, normally standing between the two
units rather than negotiated per flight. Announcing "expect vectors" is how the
pilot is told the published transition will not be flown.

So the shortcut is applied ONLY under `force_app_vectoring`, where vectoring is
certain by construction. Under `allow_vectoring` -- where ATC merely *may*
vector -- the descent stays planned on the ROUTED distance, because an en-route
controller cannot assume a shortcut he has not agreed. That asymmetry is
deliberate, and it is why the two settings are separate rather than one.

**To verify before the public release** (not yet checked against source
material): how the transfer conditions between an ACC and a terminal unit are
normally fixed -- level, transfer point, and whether vectoring is pre-agreed --
and whether the phraseology used to tell the pilot differs from the "expect
vectors" heads-up already implemented. If the real convention turns out to be
narrower than assumed here, the shortcut belongs behind whatever condition that
convention actually specifies.

### Vectors cannot start before the filed cruise ends

"Je ne pense pas que le vectoring cela peut etre avant notre dernier segment au
FL230 qui est accepte par eurocontrol" (user, 2026-08-16). Vectors belong to the
TERMINAL phase: the last filed level is agreed with the network, and an approach
controller does not reach up into a cruise segment the flight plan was accepted
on.

Arming therefore requires the arrival descent to have started -- the ATC state
must be DESCENT, ARRIVAL or an approach state, never ENROUTE_CRUISE. On the
measured arrival this changes nothing (the aircraft was in DESCENT ~60 NM before
the vectors armed), so it is a structural rail rather than a behaviour change:
it makes the case impossible instead of merely unlikely.

---

## Future: en-route vectoring (not implemented)

Recorded at the user's request, 2026-08-16. Everything in this document is
APPROACH vectoring -- positioning an aircraft onto a final approach course. A
separate manoeuvre exists: vectors given by the EN-ROUTE controller, for
spacing, traffic or weather, with no approach involved.

They differ in every respect that matters here:

| | approach vectoring | en-route vectoring |
|---|---|---|
| target | the final approach course | rejoining the cleared route |
| ends with | "cleared approach, report established" | "resume own navigation direct FIX" |
| altitude floor | sector MSA around the procedure | grid MORA / the cleared level |
| phase | terminal only -- never in the filed cruise segment | cruise, by definition |

The phase rail added for approach vectoring (arming requires the arrival
descent to have started) is exactly what keeps the two from colliding: an
en-route vector must NOT be produced by poll_vector_to_final.

Note that the off-route recovery already sketched elsewhere is the same
manoeuvre seen from the other side -- an aircraft that has drifted is vectored
back onto its route. If both are built, they should share one implementation
rather than grow two.

---

## Speed control (2026-08-17)

A vectored approach without speed assignment does not exist: the controller is
building a sequence, and speed is how he builds it. EUROCONTROL practice is
**160 kt maximum from 8 NM to touchdown**, with *"160 knots to 4 DME"* the
standard restriction; the intermediate vectors carry a higher sequencing speed.

| leg | assigned |
|---|---|
| the sequencing vectors | **210 kt** |
| the alignment vector, with the approach clearance | **160 kt** |

**No aircraft-category table**, because we do not have the data and it is not
needed: the instruction is issued only when the aircraft is genuinely faster than
the target, so a light aircraft already at 140 kt is never told to "reduce" to
160. Anchored on the word *speed*, so the readback verifier picks it up unwired.

It feeds back into the geometry, which is the point. The alignment lead is 60 s
of lateral closure, so it is a DISTANCE that shrinks with speed:

```
280 kt -> lead 2.3 NM        210 kt -> lead 1.7 NM        160 kt -> lead 1.3 NM
```

Vectoring an aircraft that was never slowed sizes the whole manoeuvre on a speed
it will not have. Measured on the replay: with the 210 kt instruction in place
the lead drops from 2.3 to 1.7 NM on the same arrival.

**Open:** the alignment rule is expressed in DISTANCE (3 NM before the FAF) while
the lead is expressed in TIME. Three miles is 39 s at 280 kt and 68 s at 160 kt,
so the rule is not equally demanding across the speed range and the margin is
thin at the bottom of it — 2.9 NM measured against a 3 NM target, passing only on
a 0.5 NM tolerance.

---

## Known defects (2026-08-17, build `11365b5`)

Flown twice on LFLP → EDLW. **Neither flight completed the arrival.**

| | defect | status |
|---|---|---|
| **The Tower handoff never fires.** On build 85 the manoeuvre worked end to end, the pilot reported *"established as 06"* — and ATC never answered. `[approach] profile enforcement yielded to Tower handoff` was logged 2 400 times: the profile stands aside for a transfer that never comes. | **open**, deferred by the user |
| **The feasibility margin does not budget the turn.** The sequence armed with 6.7 NM of margin; the turn onto the intercept heading then ate 11 NM of axis distance for 4 NM of lateral closure, at an effective 26° against the 30° assumed. The ICAO floor now absorbs this, but the arithmetic is still wrong. | open |
| ~~No speed control anywhere in the manoeuvre.~~ | **done** — see *Speed control* above |
| **The route tracker is re-initialised behind the aircraft** when the approach waypoints are appended, which cancels any direct-to and made the routed distance to the IAF read 24 NM with the aircraft 2 NM from it. | open |

## Revision history

| version | date | build | change |
|---|---|---|---|
| 1.x | 2026-08-16 | — | specification, written before implementation |
| 2.1 | 2026-08-17 | v4.4.0-beta (`11365b5`) | speed control on the vectors (210 kt sequencing, 160 kt with the approach clearance, EUROCONTROL); no category table -- the target is only issued when the aircraft is faster than it. Records that the lead is a TIME and the alignment rule a DISTANCE, so the margin narrows as speed drops |
| 2.0 | 2026-08-17 | v4.4.0-beta (`d7c3f64`, pkg 87) | brought to the ICAO Doc 4444 limits after the EDLW flight: 45° / 2.0 NM as the floor with 30° / 3 NM kept as the target; `until established on the localiser` added to the clearance, adapting to the approach type; leg levels derived from the glide path instead of a fixed offset above the FAF, and stepped down as the geometry changes; status corrected from "nothing implemented" |
