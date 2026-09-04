# FORCE APP VECTORING — specification

**Spec version:** 2.8 · **Dated:** 2026-08-21 · **Build:** v4.4.0-beta (pkg 108)

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

### Sources, quoted (ICAO Doc 4444 / PANS-ATM, 16th ed.)

Everything below is quoted from the document itself, not from a summary. An
earlier revision of this spec cited figures taken from a web search and got one
of them wrong — see the correction two paragraphs down.

**8.9.3.6 — vectoring for final approach (the general rule, a single runway):**

> *Aircraft vectored for final approach should be given a heading or a series of
> headings calculated to close with the final approach track. The final vector
> shall enable the aircraft to be established on the final approach track prior
> to intercepting the specified or nominal glide path of the approach procedure
> from below, and should provide an intercept angle with the final approach track
> of **45 degrees or less**.*

**6.7.3.2.4 — INDEPENDENT PARALLEL approaches only:**

> *a) enable the aircraft to intercept at an angle **not greater than 30
> degrees**; b) provide at least 1.9 km (**1.0 NM**) straight and level flight
> prior to the final approach course or track intercept; and c) enable the
> aircraft to be established on the final approach course or track, in level
> flight for at least 3.7 km (**2.0 NM**) …*

**8.6.5.5 — terminating vectoring:**

> *In terminating vectoring of an aircraft, the controller shall instruct the
> pilot to resume own navigation, giving the pilot the aircraft's position and
> appropriate instructions … if the current instructions had diverted the
> aircraft from a previously assigned route.*

**12.4.1.4 — termination of vectoring, phraseology:**

> *a) RESUME OWN NAVIGATION (position of aircraft) (specific instructions);
> b) RESUME OWN NAVIGATION [DIRECT] (significant point) [MAGNETIC TRACK (three
> digits) DISTANCE (number) KILOMETRES (or MILES)].*

**12.3.3.2 — approach instructions, phraseology:**

> *d) CLEARED DIRECT (waypoint), DESCEND TO (level), EXPECT TO REJOIN STAR
> [(STAR designator)] AT (waypoint), **then** REJOIN STAR [(designator)] [AT
> (waypoint)]; e) CLEARED DIRECT (waypoint), DESCEND TO (level), **then** REJOIN
> STAR (designator) AT (waypoint); f) CLEARED (type of approach) APPROACH
> [RUNWAY (number)].*

The SID side is symmetric — 12.3.3.1 g) and h), *REJOIN SID … AT (waypoint)*.

**8.9.3.7:**

> *Whenever an aircraft is assigned a vector which will take it through the final
> approach track, it should be advised accordingly, stating the reason.*

**Correction to spec 2.0/2.1 (2026-08-18).** Those revisions said ICAO requires
the aircraft established **2.0 NM** before the glide path intercept, and treated
30° as the ICAO nominal. Both readings came from the wrong section: **6.7.3.2.4
governs independent parallel approaches only.** For a single runway the rule is
8.9.3.6 — 45° or less, established *before* the glide path intercept, from below,
with **no distance figure at all**.

So of our numbers:

| ours | status |
|---|---|
| 45° maximum | **is** the ICAO general limit (8.9.3.6) |
| 30° nominal | a conservatism of ours; it is the *parallel* limit, not a general nominal |
| established 3 NM before the FAF | **our own rule**, from the user's requirement — ICAO gives no figure here |
| 2.0 NM floor | a conservatism of ours; the figure itself comes from the parallel-approach section |

The rules are not weaker for being ours, but they must not be presented as the
standard's.

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

> **DEFECT, measured 2026-08-19 — this section describes the only case that
> works.** `D_start` is a distance measured back from the FAF ALONG THE FINAL
> APPROACH TRACK, and it is positive. An aircraft arriving from the far side of
> the field has a NEGATIVE along-track distance and never enters the arming
> window at all, so the choice between the three shapes below never happens and
> the `Downwind` leg -- written, documented, and commented "otherwise -> DOWNWIND
> to gain axis distance" -- is **unreachable**.
>
> Measured on LSGG BELU3R runway 22 (`make replay-lsgg`), final track 223 deg,
> arrival from the south-west:
>
> ```
> [vector] arm check: -81.0 NM to FAF GG808 (window 15-30.5), y=-23.8
> [vector] arm check: -52.7 NM to FAF GG808 (window 15-30.5), y=-9.1
> [vector] refused: 2.4 NM to FAF GG808, below the 15 NM floor
> ```
>
> The vectoring only armed once the aircraft had passed the field and the
> projection turned positive -- 2.4 NM out, far too late, so the arrival kept the
> published procedure. EDLW and LFMN never showed it because both are approached
> from the inbound side.
>
> **What it needs:** arming on the TRACK DISTANCE STILL TO FLY -- turn, downwind,
> base, final -- instead of on the along-track projection alone. That is what
> makes the downwind shape reachable. It is a real piece of work, not a
> threshold change. [C. P. Potter]

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
               vectoring for sequencing"
B  base       "turn left heading 120, reduce speed 180 knots"   <- 90 deg to the axis
C  intercept  "turn left heading 090"                           <- 30 deg max
D  axis       (SILENT -- the aircraft has the track and joins it itself)
```

Leg C carries the whole ICAO 6.7.3.2.7 package, because it IS the last vector:
position relative to a fix on the final approach track, the level to be
maintained until established, and the approach clearance —

```
"turn left heading 253, 18 miles from GG808, descend 4000 feet,
 QNH 1024 until established, cleared ILS approach runway 22"
```

**Leg D transmits nothing.** Vectoring terminates when the aircraft leaves the
last assigned heading to intercept (8.9.4.1), so there is no routine vector onto
the course — assigning it was ATC doing the pilot's job. A further heading exists
only as a *correction*, when the aircraft is off the axis and diverging ("you
have passed through the localiser"). `report established` is gone for the same
kind of reason: 12.4.2.2 brackets it onto another instruction, and the Tower
handoff a few miles later asks for it anyway.

Then `contact Tower on <freq>` on "established". If abandoned, the published
procedure is handed back at its ENTRY — `cleared direct <IAF>, cleared <approach>`
— never `resume own navigation direct <FAF>`: the approach-instruction set
(12.3.3.2) contains no such phrase, and sending an unaligned aircraft straight at
the FAF is the one thing this manoeuvre exists to avoid.


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

### The last descent is the platform, not another rung (build 89)

Chasing the glide path down in rungs does not reach it. With a 900 ft step the
last eligible rung fell at **5.97 NM from the FAF** -- the very moment of
capture -- so it never went out, and the simulated EDLW arrival intercepted at
**5000 ft with the path at 4376: 624 ft high, from above**, which 8.9.3.6
forbids.

A controller does not chase the path. He puts the aircraft on the **published
platform** -- the FAF crossing altitude -- before the intercept, and the aircraft
then meets the path from below. Inside `kVecPlatformNm` (12 NM, the intermediate
segment) the level assigned is the platform:

- the sector MSA does not apply there (the aircraft is cleared for a published
  approach whose crossing altitude is itself obstacle-protected);
- the 300 ft glide-path margin is meaningless -- the platform *is* the crossing
  altitude;
- the ceiling handed to `vec_leg_level_ft()` must be the platform itself. The
  function starts from the ceiling it is given and only ever pushes it back up,
  so passing the usual `FAF + 2000` produced 4500 and no transmission at all.

Simulated result: `FL60 -> 5000 (17.9 NM) -> 2500 (11.9 NM)`, captured at 5.9 NM
at 2685 ft with the path at 4360 -- **1675 ft below it**.

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

## Never re-issue an instruction already in force (2026-08-19)

**General rule, not a vectoring rule.** ATC does not repeat an instruction the
pilot is already flying, and it holds **across controllers**: an instruction
survives a handoff, and the receiving unit does not restate it (user,
2026-08-19).

What it cost when it was missing: the flown arrival of 2026-08-19 heard
`"descend flight level 100"` from the vector while already cleared to and
descending through FL100, then `"descend flight level 60"` fifteen seconds later
-- and was then challenged, `"negative, I say again, flight level six zero"`, for
correctly reading back the first of the two.

| what | how it is held | where it is compared |
|---|---|---|
| level | `s_last_transmitted_alt_ft` | the vector's leg level |
| speed | `s_atc_assigned_speed_kt` | `vec_speed_phrase()` |
| heading | `s_vtf_hdg` | a recut within 3 deg is not transmitted |

Two mechanics decide whether this works at all:

- **The record must be written where the words are emitted**, not where the value
  is computed. `current_cleared_alt_ft()` and `s_enroute_cleared_alt_ft` also
  hold levels the approach profile has merely PLANNED, and both already contain
  the vectoring's freshly computed level by the time its text is built. Two
  earlier attempts used them, compared equal on every arrival, and suppressed the
  descent itself -- the replay then intercepted 3325 ft ABOVE the glide path.
- **A handoff must not erase it.** The first cut reset the record inside
  `remember_controller_label()`, i.e. on every transfer, which is precisely the
  case the rule exists for. Only a new flight clears it.

The safe failure mode follows from the first point: if the record is not set, the
comparison simply does not match and the instruction is transmitted as before.
Nothing can be suppressed that was never said.

## Releasing the speed (2026-08-18, build 96)

An assigned speed does **not** lapse on its own. ICAO Doc 4444 **4.6.1.2**:
*"Speed control instructions shall remain in effect unless explicitly cancelled
or amended by the controller."* And **4.6.1.7** makes the release mandatory:
*"Aircraft shall be advised when a speed control restriction is no longer
required."* Ours never was -- an aircraft told "reduce speed to 160 knots" for
sequencing was still holding it on short final with nobody releasing it.

**Where.** **4.6.3.7** is the outer bound, not the release point: *"speed control
should not be applied to aircraft after passing a point 7 km (4 NM) from the
threshold on final approach"*. We release at the **FAF**, which is farther out
(6.6 NM at EDLW), so the rule is met with margin, the sequencing the restriction
existed for is over, and the trigger is a fix whose coordinates the approach data
actually gives us -- the threshold's it does not.

Phraseology 12.4.1 (h): `RESUME NORMAL SPEED`.

Two mechanics worth recording, because both cost a debugging pass:

- The FAF must be resolved the way the vectoring resolves it. `s_approach_faf` is
  only populated inside `poll_approach`, which a vectored arrival never goes
  through, so reading it there left the release permanently disarmed.
- "Past the FAF" is an **along-track** test. Plain range grows again once the fix
  is behind.

### "160 knots" and "160 knots or less" are different instructions

12.4.1 (f) is *INCREASE (or REDUCE) SPEED TO (number) KNOTS [OR GREATER (or OR
LESS)]* -- the bracket changes the instruction. Without it the speed is
**assigned**, and sequencing or spacing requires the aircraft to fly it; with it,
it is a maximum. The readback challenge restated every assigned speed as a
maximum (`negative, I say again, 160 knots or less` after
`reduce speed to 160 knots`), quietly relaxing it. It now mirrors the form the
clearance used.

## Terminating the manoeuvre (2026-08-18)

**"Resume own navigation direct <FAF>" was wrong twice over**, and it is what the
flown arrivals kept hearing.

- 12.4.1.4 does allow *RESUME OWN NAVIGATION [DIRECT] (significant point)*, but
  8.6.5.5 requires the aircraft's **position** with it, or the phraseology's
  magnetic track and distance. We gave neither.
- More fundamentally, it does not belong in the approach phase at all: the
  approach-instruction set (12.3.3.2) contains **no such phrase**. After a direct,
  the procedure is **REJOINED**, or the approach is simply cleared.
- And the **FAF is the worst possible target**: it is not a rejoin point, and
  sending an *unaligned* aircraft straight at it is precisely what the manoeuvre
  exists to prevent.

Terminating now hands the published procedure back at its **entry** — the IAF —
and clears the approach with it:

```
cleared direct <IAF>, cleared <approach> runway <NN>.
```

## The clearance package (2026-08-18, build 89)

**ICAO Doc 4444 6.7.3.2.7** -- *when assigning the final heading to intercept the
final approach course or track, the runway shall be confirmed, and the aircraft
shall be advised of:*

> *a) its position relative to a fix on the final approach course or track;*
> *b) the altitude to be maintained until established on the final approach
> course or track, to the glide path or vertical path intercept point; and*
> *c) if required, clearance for the appropriate approach.*

Those three travel together, and **(b) is the intercept altitude** -- not any
level that happens to be assigned at the time. Our intercept heading goes out
around 28 NM from the FAF, where the level is still FL60 and the platform is
below the sector MSA, so pairing the clearance with the level assigned *there*
promised an interception at a level the aircraft leaves long before the
localiser.

So the intercept vector is now a **plain vector** -- `vectoring for <approach>`,
a heads-up, not a clearance -- and the whole package goes out with the platform:

```
<callsign>, 12 miles from KOLOT, descend 2500 feet, QNH 1024
until established, cleared ILS approach runway 06.
```

Two related rules fall out of the same paragraph set:

- **`until established on <ref>` qualifies the intercept altitude and nothing
  else.** On any higher level it promises an interception that will not happen
  there. Above the platform the level is an ordinary descent and is spoken as
  one.
- **`report established` carries its reference.** 12.4.2.2 (e) is *REPORT
  ESTABLISHED ON LOCALIZER (or ON [GLS/RNP/MLS] [FINAL] APPROACH [COURSE])*. The
  bare form exists only bracketed onto another instruction -- (f) `CLOSING FROM
  LEFT`, (g) `TURN LEFT HEADING (three digits)`, (m) `INTERCEPT (LOCALIZER)`.
  Following an approach clearance, which is item (d), it needs the reference.

**Open, for a controller to settle.** Our single vector doubles as two things the
document distinguishes: the *closing* vector, which carries neither clearance nor
platform, and the *final heading to intercept*, which carries all three items.
The question is at what distance a real controller assigns the final intercept
heading, and whether he clears the approach before or with it.

## Alignment is judged at capture (build 89)

The alignment rule -- aligned at least `kVecAlignNm` before the FAF -- is about
where the aircraft **joined** the track, and that instant is the capture. Judging
it on a later frame cannot work: at capture the aircraft is still on the intercept
heading, 31° off the axis, so a test that *also* demands a settled heading can
only pass after the roll-out. A good capture at 5.9 NM was therefore reported as
`established LATE ... 2.9 NM (rule wants 5)`. The distance is now recorded at
capture and judged against that.

Two smaller rules, same build:

- **A correction that changes nothing is not a correction.** Re-cutting the
  intercept can land on the heading already assigned; below 3° there is nothing
  to fly, and the transmission is noise the pilot must read back. Suppressed, and
  logged as suppressed.
- **The capture line states the altitude against the glide path.** Whether the
  aircraft meets the path from below is the one thing that line has to answer,
  and it did not -- the 624 ft high intercept had to be reconstructed by hand
  from two other lines.

## Known defects (2026-08-17, build `11365b5`)

Flown twice on LFLP → EDLW. **Neither flight completed the arrival.**

| | defect | status |
|---|---|---|
| ~~**The Tower handoff never fires.**~~ On build 85 the manoeuvre worked end to end, the pilot reported *"established as 06"* — and ATC never answered. `[approach] profile enforcement yielded to Tower handoff` was logged 2 400 times: the profile stands aside for a transfer that never comes. | **root cause found, build 89.** The state never reached `IFR_APPROACH_DESCENT`: it was set on the first vector, overwritten to `IFR_ARRIVAL` on the same frame by the `st == IFR_DESCENT` line below it, and the approach-state reset then cleared `s_approach_cleared_issued` every frame. The state now switches **with the clearance**, at the platform. The handoff fires in the simulated arrival — **but with no frequency**, which is the next defect |
| **The Tower handoff carries no frequency.** Fires at ~6 NM, state reaches `IFR_APPROACH_TOWER`, no frequency spoken and no landing clearance follows. | open, deferred by the user until the rest is settled |
| **The feasibility margin does not budget the turn.** The sequence armed with 6.7 NM of margin; the turn onto the intercept heading then ate 11 NM of axis distance for 4 NM of lateral closure, at an effective 26° against the 30° assumed. The ICAO floor now absorbs this, but the arithmetic is still wrong. | open |
| ~~No speed control anywhere in the manoeuvre.~~ | **done** — see *Speed control* above |
| **The route tracker is re-initialised behind the aircraft** when the approach waypoints are appended, which cancels any direct-to and made the routed distance to the IAF read 24 NM with the aircraft 2 NM from it. | open |

## Revision history

| version | date | build | change |
|---|---|---|---|
| 2.8 | 2026-08-21 | v4.4.0-beta (pkg 108) | **the release gate had a dead zone and it cost a flight.** 2.7 tightened the release to "within 4 NM, astern, or tracker STRICTLY past" but left the wait on "tracker index below the fix"; the route tracker declares a fix reached about ten miles early, so with the tracker exactly ON the index and the aircraft 9.6 NM out the code neither waited nor armed -- it fell through to the geometric arming, which refused for lack of room, and that refusal LATCHES. No vectoring at all on the LSGG arrival of 2026-08-21. Waiting and releasing are now the same test negated, and **with a position for the fix the tracker is not consulted at all** (the two geometric tests -- within the lead, or astern -- are complete). Release now measured at 2.0 NM in replay against 10.0 before. The harness printed that 10.0 NM for days: the acceptance now surfaces the release distance and any refusal, **and exits non-zero**, so a replay fails instead of reporting |
| 2.7 | 2026-08-21 | v4.4.0-beta (pkg 107) | **the base leg exists again.** VecLeg::Base was specified from version 1.x and written in the engine, and nothing ever assigned it -- every vectored arrival flew ONE ~200 degree reversal from the outbound leg straight onto the intercept. The pattern is now downwind, base at 90 degrees to the axis, intercept at 30. Leg C carries the 6.7.3.2.7 package (without it the aircraft was turned and descended but never cleared). **Specification confronted with the code**: leg D was still described as a spoken vector with "report established" weeks after the capture went silent, and the abandon line still said "resume own navigation direct <fix>" after 2.2 had replaced it with a rejoin at the IAF -- both corrected here. Also: the sequencing vector releases within 4 NM of the prescribed fix instead of on the route tracker's index (it fired 9.9 NM early), and "expect vectors" is no longer promised to an aircraft already being vectored |
| 2.6 | 2026-08-20 | v4.4.0-beta (pkg 105) | **CONTINUE HEADING** (Doc 4444 12.4.2.2) when the vector is within 5 deg of the heading flown -- "turn right heading 043" to an aircraft already on 043 is not a turn. **Position information on the FIRST vector** ("position 32 miles from GG808"), the moment the aircraft leaves the published procedure; the figure is the TRACK distance the plan will fly, not the straight line. **The reason, not the approach**: a procedure vector that overrides a STAR turn is `vectoring for sequencing`. **Sequencing leg shortened** -- sized at 40 deg (kVecDownwindInterceptDeg) instead of the nominal 30, inside the 45 deg maximum of 8.9.3.6: the LSGG outbound turned 3 NM earlier and the clearance came 2 NM closer to the FAF. **A level is no longer squeezed in ahead of a transfer**: the step-down into the destination TMA yields to the terminal handoff owed in the same frame, and never speaks across an unanswered one. **ATC_SEED** makes a replay reproducible -- the wall-clock reseed in `poll_hold` was flipping the STAR-shortcut draw and made two runs of one binary land differently |
| 2.5 | 2026-08-19 | v4.4.0-beta | **never re-issue an instruction already in force** (level, speed, heading), across controllers too -- `s_last_transmitted_alt_ft` written where the words are emitted, cleared only by a new flight. Intercept clearance shortened: `report established` dropped (12.4.2.2 brackets it, the Tower handoff asks for it a few miles later, and it repeated "on the localiser" twice) |
| 2.4 | 2026-08-18 | v4.4.0-beta (pkg 96) | **speed release** past the FAF (4.6.1.2 / 4.6.1.7 / 4.6.3.7, phraseology RESUME NORMAL SPEED) and the readback challenge no longer restates an assigned speed as a maximum. **R1 tolerance**: the closure projection is judged against `kVecAlignNm - 1.5`, not the target itself -- R1 fired seven times on the flown arrival while the aircraft closed steadily, and its last correction steepened an aircraft already committed to the capture. Records the REVERTED attempt to suppress a redundant leg level |
| 2.3 | 2026-08-18 | v4.4.0-beta (pkg 89) | **the clearance package (6.7.3.2.7)**: the intercept vector becomes a plain `vectoring for <approach>`, and position + intercept altitude + approach clearance go out together with the **platform**; `until established on <ref>` restricted to the intercept altitude; `report established` carries its reference (12.4.2.2 e). **The last descent is the platform**, not another glide-path rung — the simulated arrival intercepted 624 ft high, from above, because the last rung fell at the moment of capture. **Alignment judged at capture** instead of after the roll-out. No-op corrections suppressed. Capture line now states altitude vs glide path. Tower-handoff root cause found (state overwritten to `IFR_ARRIVAL`) |
| 1.x | 2026-08-16 | — | specification, written before implementation |
| 2.2 | 2026-08-18 | v4.4.0-beta (`7d7dfe6`) | ICAO Doc 4444 quoted verbatim from the document instead of from a search summary, **and a correction**: the 2.0 NM and the 30° that 2.0/2.1 attributed to ICAO come from 6.7.3.2.4, which governs INDEPENDENT PARALLEL approaches only. The general rule is 8.9.3.6 -- 45° or less, established before the glide path intercept from below, no distance figure. Our 3 NM and 2 NM are ours, not the standard's. Terminating the manoeuvre now REJOINS the procedure at the IAF (12.3.3.2 d/e/f) instead of "resume own navigation direct <FAF>", which the approach-instruction set does not contain |
| 2.1 | 2026-08-17 | v4.4.0-beta (`11365b5`) | speed control on the vectors (210 kt sequencing, 160 kt with the approach clearance, EUROCONTROL); no category table -- the target is only issued when the aircraft is faster than it. Records that the lead is a TIME and the alignment rule a DISTANCE, so the margin narrows as speed drops |
| 2.0 | 2026-08-17 | v4.4.0-beta (`d7c3f64`, pkg 87) | brought to the ICAO Doc 4444 limits after the EDLW flight: 45° / 2.0 NM as the floor with 30° / 3 NM kept as the target; `until established on the localiser` added to the clearance, adapting to the approach type; leg levels derived from the glide path instead of a fixed offset above the FAF, and stepped down as the geometry changes; status corrected from "nothing implemented" |
