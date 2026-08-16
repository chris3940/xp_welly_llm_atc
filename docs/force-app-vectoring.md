# FORCE APP VECTORING — specification

Status: **specification only, nothing implemented.** Agreed with the user
2026-08-16, before the public release.

---

## The governing rule

> **The last vector assigns the FINAL APPROACH COURSE, and the aircraft must be
> established on that axis AT LEAST 2–3 NM BEFORE THE FAF.**

This is not a refinement of the sequence — it is the constraint that
*dimensions* the whole manoeuvre. Everything below is derived from it.

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

### When the rule cannot be evaluated: curved finals

The rule assumes a **straight** final approach axis. An approach whose final
segment is flown as an arc has no axis to align with: neither the last vector's
heading nor the `< 5 deg` test means anything on it.

This is not hypothetical -- it is LOWI, the very airport the existing IAF
teardrop was built for. Measured in the reference CIFP:

```
R08-Z  WI752  RF        <- radius-to-fix: a curved final
R08-Z  WI754  RF
```

So **FORCE APP VECTORING must refuse on any approach whose final segment
contains an RF (or AF) leg**, and hand back the published procedure. The test is
data-driven: scan the chosen approach's final segment `path_term` column for
`RF`/`AF` before arming.

```
[vector] refused: R08-Z has a curved final (RF legs) -- no straight axis, keeping the published procedure
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

Each leg is cleared to `max(sector MSA, leg altitude)`:

| leg | altitude |
|---|---|
| downwind | platform + 2000 |
| base | platform + 1000 |
| axis | **the glide-intercept altitude** (2500 ft at EDLW) |

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
