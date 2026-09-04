# Open questions

Running log of questions and reasoning raised in discussion that are **not
settled** and not yet implemented.

Convention for every entry:

- the **date** it was raised, and the **build** (tag + short SHA) it was raised
  against, so a later reader knows what the code looked like at the time;
- the question in the words it was asked;
- what the code does **today**, with measured figures where they exist;
- an **opinion** where there is one, marked as such and separated from fact;
- what is explicitly left **undecided**.

Thinking is recorded here even when nothing is built: a decision taken and
forgotten costs more than one written down. Answered questions move out into the
relevant document or a commit message; nothing is deleted silently.

---

## Q1 — Is the step-down distance also computed for the VECTORED track?

**Raised:** 2026-08-17, against **v4.4.0-beta-85 (`a6bb13b`)**.

> "Ton calcul pour la descente au FL100, ce sont les points routés vers l'IAF
> (si la STAR n'a pas de contrainte) — mais est-ce que tu fais aussi un calcul
> en parallèle de la distance des segments si vectoring jusqu'au FAF ?"

**Answer as the code stands: no. The two calculations are not symmetric.**

| | distance used |
|---|---|
| pre-TOD chain (`poll_enroute`) — the ASK and the first clearance | routed **and** direct; the shorter wins when `force_app_vectoring` is on (commit b5e9cb2) |
| step-down ladder (`poll_profile_crossing` via `check_next_fix`) — the one that issues FL100 | `routed_distance_to_fix_idx()` **only**, always |

So the descent is *announced* on the understanding that the aircraft will be cut
across its route, and then *stepped down* as though it were going to fly every
published leg. On the DIK → EDLW arrival the two differ by roughly 20 NM.

**Why it matters.** The ladder's trigger is a top-of-descent computed from that
distance:

    IFR descent: DOR -> rung flight level 100 (target 3000 ft at DOR)
                 @ PA 28000, 85.8 NM to go, TOD 102.3 NM, 25000 ft to lose

If the aircraft is going to be vectored, "85.8 NM to go" is not what it will
fly — the real figure is shorter, the TOD is therefore nearer, and every rung
should come earlier. The effect is the same one already measured on the pre-TOD
side, where planning on the routed path left the profile 4.1 degrees steep.

### Mechanism found (2026-08-17, against `741562b`)

The user asked whether the routed distance distinguishes a standard STAR->APP
from a direct to the FAF. It does, both ways -- and that turned out to be the
wrong suspect. `s_route_fixes` carries the STAR fixes AND the CIFP approach
waypoints (`is_approach_proc`, `is_map`), so the published path is summed leg by
leg; a direct-to jumps the tracker so the distance collapses correctly.

**The failure is specific to VECTORING**, and it inverts the sign of the error.
`routed_distance_to_fix_idx()` skips any fix more than 100 degrees off the nose:

```cpp
if (off <= 100.0) break; // ahead -> start summing here
++start;                 // behind -> skip it
```

On a DOWNWIND leg the aircraft flies away from the FAF, so every remaining
STAR/approach fix is behind and gets skipped one by one until `start` reaches
`target_idx`. The "routed" distance degenerates to the STRAIGHT LINE from the
aircraft to the FAF -- much shorter than the vector track still to be flown
(downwind + base + intercept).

That short distance feeds the gradient directly:

```cpp
need_ftnm = (alt_now - issue_ft) / fc.dist_nm;
steep     = need_ftnm > kDescentSlopeFtPerNm * 1.35;
```

So ATC computes a steeper gradient than reality and words the clearance
"expedite descent" when there is in fact room.

**Corrected the same day, by the user: that is only half of it, and the other
half is the half that bites.** The claim above was written as though the error
had one sign. It does not -- there are TWO errors, in OPPOSITE directions, acting
at different moments:

| | when | direction | effect |
|---|---|---|---|
| the published path is not the flown one | while the descent is PLANNED, before vectoring starts | routed **overstates** | descent fires late, then is **genuinely** steep |
| the skip loop collapses the sum | once ON a downwind leg | routed **understates** | "expedite" worded with room to spare |

The first is the user's reading and it is the one that produces a descent that is
actually too fast. Measured at EDLW: the ILS 06 transition runs **overhead the
field** -- `DOR` is the field VOR, 0.9 NM from the threshold -- and then out to
`KOLOT` 6.6 NM to the southwest before turning back inbound on 057. A west
arrival routed through that reversal carries ~13 NM the vectors will delete. The
descent spec already measured the same thing from the other end: routed and
direct differ by roughly 20 NM on this arrival (defect D2).

Note the routed choice is **deliberate and correct on a published path** --
`routed_distance_to_fix_idx()` exists because the straight line under-reads on a
dog-legged STAR and fired a descent 53 NM early at LFLP. The defect is not
"routed is wrong", it is "routed is the wrong question once the aircraft is going
to be taken off the route".

The skip loop is not itself a defect -- it exists because the tracker lags and
summing a backward leg inflated the distance and fired a descent late (LFLP
2026-07-17). It is correct ON a route and meaningless OFF one.

**So the fix is not "routed vs direct" but "use the VECTOR TRACK length while
vectoring"**, which is computable: `poll_vector_to_final()` already builds the
four legs with their geometry.

**Not decided, and deliberately not implemented yet:**

1. Should the ladder mirror the pre-TOD rule exactly — take `min(routed,
   direct)` under `force_app_vectoring` only — or should it compute the vectored
   track properly (the four legs' own geometry) rather than a straight line?
2. Under `allow_vectoring`, where vectoring is possible but not certain, the
   pre-TOD side deliberately stays on the routed distance (see the coordination
   assumption in `force-app-vectoring.md`). The same reasoning presumably
   applies here, but it has not been argued through.
3. Whether a shorter distance should pull the RUNG lower as well as earlier, or
   only earlier.

**Related:** `docs/force-app-vectoring.md` (the vectored-track planning section
and the coordination assumption it rests on); the `cruise * 0.66` heuristic that
still produces the first level (FL180 on this arrival) and is superseded two
miles later by the ladder's FL100 — a separate defect, noted here only because
the two surface together in the same log.

---

## Q2 — Two calculations when both modes coexist? And are there two TODs?

**Raised:** 2026-08-17, against **v4.4.0-beta-85 (`a6bb13b`)**.

> "Pour l'instant on ne fait que le FORCE VECTORING c'est plutot simple, mais
> quand les 2 pourront coexister (par exemple un declenchement RANDOM), il faut
> faire les 2 calculs, avec vectoring et sans, non ? Et finalement on a 2 TOD,
> un TOD enroute et un TOD pour l'arrivee, ou je me trompe ?"

### Part 1 — two parallel profiles?

**Opinion: no, and it would be the wrong model.** Two profiles held at once
means ATC entertaining two intentions simultaneously, which no controller does:
he DECIDES, then announces ("expect vectors" / "cleared via the STAR"). If the
trigger becomes random, the roll belongs BEFORE the top of descent, and from
then on there is one distance.

What is genuinely needed is what already exists: the ladder re-evaluates every
frame, so if the decision changes later the profile re-plans itself. Under
`allow_vectoring`, where vectoring is possible but not agreed, the planning
deliberately stays on the ROUTED distance -- see the coordination assumption in
`force-app-vectoring.md`.

Open sub-question: if the decision is taken late (traffic), the aircraft may
already be committed to a routed profile and be too high for a vectored one. A
controller would then not vector at all, or would extend. Nothing implements
that yet.

### Part 2 — two TODs. Yes, and the code merges them.

The user is right, and it is more than terminology. There are two decisions,
owned by two different units:

| | question | owner |
|---|---|---|
| en-route TOD | when to leave cruise | ACC |
| arrival TOD | when to start down for the platform / FAF | terminal |

The pre-TOD chain currently computes ONE answer: it walks every candidate
(remaining route fixes, STAR fixes, the FAF) and keeps whichever forces the
earliest descent. That merges the two into a single "most binding constraint".

**This merge is what produced the FL180 / FL100 collision** on the DIK → EDLW
arrival:

    70 NM  descend flight level 180, cleared via ADEMI Three Alpha   <- leave cruise
    68 NM  descend flight level 100  (target 3000 ft at DOR)         <- arrival profile

Two clearances two miles apart, the first already superseded before it is
reached. They are not a sequence -- they are two mechanisms answering two
different questions without knowing about each other, one heuristic
(`cruise * 0.66`) and one constraint-driven.

**Undecided:** whether to model the two TODs explicitly (each with its own
constraint set and owner) or to keep one merged calculation and simply fix the
first level to come from the published constraints rather than
`cruise * 0.66`. The second is much smaller; the first is closer to how the
airspace actually works and would make the ACC / terminal split explicit -- which
the arrival phase already needs elsewhere.

---

## Q3 — The terminal-area queries find nothing at EDLW. Three separate causes.

**Raised:** 2026-08-17, against **v4.4.0-beta-85 (`a6bb13b`)**, from the user's
question "quel espace fait office de TMA ?" and the checks that followed.

Measured against the user's own data — AIRAC 2606 r1, 11/JUN/2026 to 09/JUL/2026,
`Custom Data/airspaces/airspace.txt` (OpenAir) and
`Custom Data/Earth nav data/atc.dat`. **Both sources agree at every point tested.**

```
KOLOT (the FAF)                      EDLW (the field)
  0-2500  DORTMUND CTR                 0-2500  DORTMUND CTR
  2500-4500 DORTMUND SECTOR B          2500-4500 DORTMUND SECTOR B
  4500-6500 DUESSELDORF/COLOGNE-BONN   4500-10000  ---- nothing ----
  6500-10000 DUESSELDORF ... Q
  10000+  AIRSPACE CLASS C             10000+  AIRSPACE CLASS C
```

`terminal_tma(EDLW)` returns EMPTY, `terminal_tma_ceiling` returns 0. So the TMA
rung of the descent ladder never exists and `descend-to-enter-TMA` can never fire —
an EDLW arrival rests entirely on the CIFP constraint chain.

There are **three independent causes**, and fixing any one alone would not be
enough:

1. **The query is anchored on the airport.** The terminal airspace relevant to this
   arrival sits over the FAF, not over the field: at KOLOT the stack is continuous
   from the ground up, while over EDLW there is a 4500–10 000 ft hole.
2. **Classification, not position.** Even anchored on the FAF it would fail at the
   altitude that matters: the aircraft crosses KOLOT at 3000 ft, where it is in
   DORTMUND SECTOR B — classified **CTA**, not TMA. Nothing classed TMA exists at
   the crossing altitude anywhere on this arrival.
3. **Class E is absent from the export.** Third confirmation: the AC letters present
   across 25 759 volumes are `A B C CTR D P Q R` — no E, F or G. Class E is IFR
   separation airspace (user, 2026-08-17) and the published blanket to FL100 is
   exactly what would fill the hole over the field.

Also noted: only **two** volumes in the whole file are named `AIRSPACE CLASS C` —
one 10000–66000, one 3000–19500. These are unnamed blanket blocks; the user
identifies the first as Langen, which is consistent with atc.dat placing
`LANGEN ctr 0-24500` over the same points. The export has stripped the name, which
is why an earlier build announced "contact Airspace class c".

And `ANCHORAGE` appears in atc.dat over Dortmund with a polygon that evidently
spans the world. The innermost-volume rule discards it, but it is a data quirk
worth knowing.

**Undecided:** whether to anchor the terminal query on the approach geometry rather
than the airport; whether to treat a CTA-classified terminal sector as a terminal
area for these purposes; and whether to carry an `airspace+.txt` overlay for the
missing class E rather than work around its absence.

---

## Q4 — Infer "this is a TMA" where the name does not say so? — **ANSWERED, shipped `eee4c8b`**

**Raised:** 2026-08-17, against **v4.4.0-beta-85 (`a6bb13b`)**, spec
`algorithm-airspace.md` 1.7.

> "Pourquoi on n'imaginerait pas un algorithme qui peut, pour les pays ou il
> n'y a pas CTR ou TMA dans le libelle, deduire que c'est une TMA avec une
> probabilite assez grande ?"

Germany is the only country whose export omits the type word (17 % typed
against 92–100 % elsewhere, section 8.1), so 138 controlled volumes are
classified CTA by default and no German arrival has a terminal shelf.

### What was measured before answering

The eight correctly-named countries are a ready-made validation set: the name
gives the label, so any inferred rule can be scored rather than asserted.
Ground truth used below: **3313** volumes named TMA, **1678** named CTA,
control classes only.

**Geometry alone does not separate them.**

| | floor p50 | ceiling p50 | bbox diagonal p50 |
|---|---|---|---|
| TMA | 4500 ft | 14 500 ft | 54 NM |
| CTA | 6500 ft | 15 500 ft | 61 NM |

The distributions overlap almost entirely. A classifier on floor / ceiling /
extent would be near-worthless, so the obvious "a TMA is low and small" rule is
dead on arrival.

**A structural signal does better.** S1 = *a CTR sharing the volume's first name
word exists within 60 NM*:

```
S1 holds for  2694/3313 TMA  = 81 %
S1 holds for   899/1678 CTA  = 54 %
```

And on the German volumes that carry no type word, S1 fires exactly where it
should — `BERLIN X` 1500–10000, `BERLIN A` 2500–10000, `BERLIN B/C`
3500–10000, `BERLIN I1..J2` 5500–7500: the Berlin TMA, in layers.

### The finding that changes the question

**That 54 % is not a false-positive rate.** The CTA-labelled volumes passing S1
are largely *terminal* CTAs — `MILAN CTA ZONE 1 BRERA`, `MILAN CTA ZONE 1
LOMBARDIA` — which any descent rule should treat as terminal airspace anyway.
The label is administrative, the question is operational, and the two do not
line up. Scoring against the name therefore penalises the rule for being right.

So a probability threshold is the wrong instrument. **"Is this a TMA?" is not
the question the descent ladder needs answered** — it needs *"which volume is
the terminal airspace of MY destination?"*, which is anchored on an airport and
barely needs the name at all.

### Why the reframing also fixes EDLW

Cause 1 in Q3 was that the terminal query is anchored on the airport, where
Dortmund has a 4500–10 000 ft hole. Anchored on the **FAF** the column is
continuous:

```
KOLOT   0-2500     DORTMUND CTR
      2500-4500    DORTMUND SECTOR B      <- the terminal shelf
      4500-6500    DUESSELDORF/COLOGNE-BONN
      6500-10000   DUESSELDORF ... Q
```

`DORTMUND SECTOR B` is a terminal shelf on every operational reading, and S1
confirms it structurally (`DORTMUND CTR` sits directly beneath it, same stem,
floor meeting the CTR ceiling exactly at 2500). No probability needed — the
stack itself says so.

### Shape as BUILT (see `algorithm-airspace.md` 11bis for the shipped spec)

Replace the classification question with a **terminal-stack walk**, anchored on
the arrival geometry rather than the airport:

1. anchor at the FAF (fall back to the airport when no approach is known);
2. take every controlled volume containing that point, ordered by floor;
3. the stack's base is the CTR; walk upward while each volume's floor meets the
   previous ceiling (a contiguous column) and the extent stays terminal-sized;
4. the shelf is the first volume above the CTR; the terminal ceiling is the top
   of the contiguous run that still belongs to the same controlling unit
   (section 9 — the unit, not the volume).

An explicit type word always wins; inference applies only where there is none,
so the eight working countries cannot regress.

**Validation this makes possible, and which should gate any merge:** run the
walk at airports in the typed countries and check that what it returns is the
volume whose name carries that city and `TMA`. That is an unambiguous test with
no label noise, unlike scoring S1 against the name.

**Undecided:** whether the extent bound is needed at all once the contiguity and
unit rules do the work; whether the walk anchors on the FAF, on the IAF, or on
several points of the approach path; and whether S1 survives as a
tie-breaker or drops out entirely.

**Related:** Q3 (the three causes at EDLW), `algorithm-airspace.md` sections 8.1
(the country survey), 9 (the unit, not the volume) and defects A1 / A2.

**Resolution (2026-08-17, `eee4c8b`).** Built as the terminal stack walk, with
the guard thresholds measured rather than assumed: only a ceiling cap survives,
and the thickness / lateral-extent caps proposed above were measured to make the
result WORSE. Gated per destination on an ICAO-prefix allowlist rather than
applied globally, so the eight correctly-named countries cannot regress. The
open sub-questions below are unchanged: the walk anchors on the point it is
given, so anchoring it on the FAF rather than the airport is still not done, and
departures are not covered. Not yet flown.

---

## Q5 — The QNH-once rule is bolted on, not factored — **OPEN, noted 2026-08-18**

**The rule.** A controller states the QNH with the first altitude he assigns
below the transition level, and does **not** repeat it in his later altitude
instructions. The next controller states it again. It is a property of the
**controller/frequency**, and has nothing to do with vectoring, the descent
profile, or any other phase.

**How it is implemented today.** `alt_clearance_qnh_once()` keys the suppression
on the active frequency (`s_qnh_stated` + `s_qnh_stated_freq`), which is the
right criterion — change frequency and the match fails on its own, so no hook is
needed in the dozen places a handoff is issued. But only **5** call sites go
through it, all of them in the vectoring plus two descent sites patched by hand.
**8 call sites still call `format_alt_clearance(..., ctx.qnh_hpa, ...)`
directly** and will therefore repeat a QNH the same controller has already
given:

```
engine.cpp:2671   readback / cleared-level restatement
engine.cpp:7992   approach-profile clearance
engine.cpp:9902   level-compliance challenge
engine.cpp:11259  vectoring leg text (plain descent branch)
engine.cpp:11718  "maintain <alt>" path
engine.cpp:11987  step-down target
engine.cpp:12944  descent clearance builder
```

**The fix is a factorisation, not a patch.** Every altitude clearance spoken to
the pilot should go through one function that owns both the FL-vs-feet decision
*and* the QNH-once rule; `format_alt_clearance()` should become private to it.
This belongs with the IFR-enforcement centralisation already listed for 4.4
(`current_flight_airport` / `DirectMonitor` / `format_alt_clearance`) — the same
symptom: one rule, many call sites, each free to forget it.

**Why it was not done now.** Touching eight clearance-building sites at once, on
the day of a test flight, is exactly the change that needs its own replay and its
own flight. Noted for a future build (user, 2026-08-18).

---

## Q6 - No record of the last level actually SPOKEN - **OPEN, noted 2026-08-18**

The vector restates the level the previous controller has just assigned whenever
the descent ladder's rung happens to equal it: `"descend flight level 100"` to an
aircraft already cleared to FL100 and descending through it, then
`"descend flight level 60"` fourteen seconds later (real flight, log 25). Two
altitudes back to back, the first of them empty.

**Two attempts to suppress it were reverted.** Neither `current_cleared_alt_ft()`
nor `s_enroute_cleared_alt_ft` distinguishes a level that has been TRANSMITTED
from one the approach profile has merely planned -- both already hold the
vectoring's own newly-computed level by the time the text is built, so the test
compared equal on every arrival and the descent was never issued at all. The
replay then intercepted **3325 ft ABOVE** the glide path.

A redundant transmission is a nuisance; an approach that cannot be flown is not.
Closing this needs a "last level actually spoken to the pilot" record, written
where the text is emitted rather than where the level is computed. That is the
same shape as the QNH-once debt in Q5, and belongs with it.

---

## Q7 — The replay harness hangs about one run in eight — **OPEN, noted 2026-08-21**

> **2026-08-29 — a second, more common flake, and this one is understood.**
> Independently of the hang, `replay-lsgg` produces **two different arrivals**:
> `043 -> 313 -> 253` with a 33 deg intercept, or `043 -> 313 -> 253 -> 186 -> 178`
> with 42 deg. Roughly one run in three takes the long one. Diffing the raw logs
> of the two outcomes shows the divergence is **pilot-side, not plugin-side**:
>
> ```
> [profile] level challenge: cleared 10000 ft, actual 15000 ft (diff +5000), vs +0 fpm after 51 s
> IFR arrival handoff: ... at 15000ft MSL   (vs 13457 ft on the short run)
> ```
>
> `fly.py` occasionally misses a descent clearance, enters the Geneva TMA 1500 ft
> high, and the vector plan legitimately grows two legs to lose it. So the plugin
> is behaving; the simulated pilot is not. Worth fixing in the harness before the
> vectored arrival is shown to a controller, because the acceptance line moves
> under you between runs. Note also that the aircraft never complied with the
> challenge and the approach continued regardless -- that is
> [[project_offroute_recovery]], not this.


`make replay-lsgg` occasionally produces a truncated run: the timeline stops
mid-descent and the acceptance reports `established NO / cleared to land NO` for
a binary that lands cleanly on the next run. It was first read as a code
regression, then wrongly attributed to the wall-clock reseed of `std::rand()` in
`poll_hold` — `ATC_SEED` was added for that and the flapping continued.

Measured: eight consecutive direct runs of `fly.py` pass; a ninth timed out at
60 s with an **empty** log, so the stall happens early and before any output is
flushed. A watcher script with `gdb` ready to attach to `atc_ifr_repl` failed to
reproduce it in eight further runs.

What is not yet known: whether the REPL blocks or the Python driver does, and
whether it is a deadlock on the pipe (a `sync()` waiting for a sentinel the REPL
never prints) or a loop in the engine. The second possibility is the one that
matters beyond the harness — the same polls run inside X-Plane's flight loop.

**Opinion:** a pipe-protocol stall is much the more likely of the two. The engine
polls are straight-line code with no unbounded loop that a hang could sit in, and
the failure always lands at a point where the driver has just spoken. Worth an
hour with a `faulthandler` timeout dump on the Python side before anything else.

`ATC_SEED` is kept regardless: it removes a real source of run-to-run divergence,
just not this one.

---

## Q8 — An aircraft delivered high gets a LONGER pattern, not a steeper descent — **OPEN, noted 2026-08-21**

The outbound sequencing leg runs until there is room for both the intercept and
the descent, so altitude buys distance:

```
downwind length = max( lateral requirement , (altitude − FAF alt) ÷ 265 + 2 )
```

On the flight of 2026-08-20 the aircraft reached the vector point 6000 ft above
its FAF and the leg extended to 24.7 NM against a lateral requirement of 16.7 —
it flew away from the field for four minutes purely to lose altitude. The
immediate cause was a stale latch (`algorithm-descent.md` §8) and is fixed, but
the coupling remains: any arrival that is high for any reason gets the same
treatment.

A real controller has three answers and the engine only has one. He can extend
the downwind (what we do), he can ask for a rate — *"expedite descent"*, which
the profile net already words elsewhere — or he can accept a steeper intercept
and let the aircraft descend on the base. Which of the three he picks depends on
traffic, and traffic is exactly what the engine does not model.

**Opinion:** the reference gradient of 265 ft/NM is a cruise-descent figure and
is too gentle for a terminal pattern; a vectored aircraft under radar control
routinely descends at 1000 ft/min at 180 kt, which is ~330 ft/NM. Using the
terminal figure inside the pattern would cut several miles off the leg without
touching the geometry. Undecided, and it needs a flown arrival to judge.

**Undecided:** whether to cap the leg at all. A cap that fires leaves the
aircraft high on the platform, which is worse than a long leg.

---

## Q9 — The LOWI arrival does not complete, and neither mechanism owns it — **OPEN, noted 2026-08-21**

A LOWI scenario was added (`make replay-lowi`, RTT1B → RNAV Z 08 from the east)
specifically to see whether the two vectoring mechanisms collide. They do not
collide — the problem is the opposite, **neither takes the aircraft**:

```
[vector] LOWI R08-Z: MSA min 10300 ft, field 1907 ft (+8393), terrain TOO HIGH;
         axis after FAF CURVED at WI752 -> WOULD USE vectors to IAF
[vector] arrival decision latched: published procedure (approach R08-Z)
IFR vector: not arming -- precondition empty: faf
IFR arrival: entering APPROACH (IAF eta=-1s, enc='CTA C')
IFR arrival: no Approach controller -- current sector handles approach to established
[approach] profile enforcement yielded to Tower handoff (1.3 NM from FAF WI749)   (repeating)
```

The pattern mechanism (`poll_vector_to_final`) correctly stands aside: the MSA is
8393 ft above the field and the final is a curved RF leg, so it latches the
published procedure. The reversal mechanism (`poll_vector_to_intercept`) then
declines with `precondition empty: faf` — the FAF is resolved only later, on
entering APPROACH (`IFR approach: FAF resolved = WI749`), and by then the arrival
decision has already latched. The aircraft flies to 3 NM at FL190 with no descent
clearance, no approach clearance and no landing clearance, while the profile
enforcement yields for ever to a Tower handoff that never comes.

Three separate things to settle, and they are not the same defect:

1. **Ordering** — the arrival decision latches before the FAF exists. A comment at
   `engine.cpp` (poll_approach, "Ensure the FAF is resolved BEFORE the vector
   arm") states the intent; the resolution still happens after.
2. **Who owns a no-vectoring arrival** — when both mechanisms decline, nothing
   drives the descent onto the published procedure. `replay-star` (EDLW, published
   procedure, no vectors) fails the same way and **already failed before any of
   this work** — verified against `1cba128`. So this is not a LOWI speciality: the
   non-vectored arrival has no owner at all.
3. **The Tower-handoff deadlock** — `profile enforcement yielded to Tower handoff`
   repeating is the known "stands aside for a transfer that never comes" symptom
   from `force-app-vectoring.md`.

**Opinion:** (2) is the real one and it is a bigger hole than the vectoring work
of the last three days, because every arrival the vectoring declines falls into
it. (1) is a small ordering fix that would at least let LOWI use the reversal.

The scenario is kept as `make replay-lowi`, deliberately NOT in `make test` — it
is a known-failing case, and a failing target inside the default suite trains the
eye to ignore red.

---

## Q10 - The departure field drifted, and ATC flew someone else's SID - **FIXED 2026-08-29**

Flight LFMN -> LOWI of 2026-08-28 (packages 129/130). Climbing east out of Nice,
ATC transmitted `"direct ITCAP, when able"`. ITCAP is on no part of the flight
plan: it is the exit fix of Albenga's ITCA1B departure.

`Log(39).txt` names the cause without ambiguity:

```
4146  [route] SID table: 7 fixes (SID (none) from CIFP + navlog, dep=LNMC rwy=)
4147  IFR SID climb: ... dep=LNMC -> step1 FL110 step2 FL140 (sid_min=0)
10127 [cifp] LIMG rwy 09 SID name -> ITCA1B
10129 [cifp] LIMG SID ITCA1B last fix -> ITCAP
10131 IFR SID climb: deferred direct ITCAP (15 NM crossing, forced)
14018 [cifp] LIMJ rwy 10 SID name -> ITCA1H
```

Every departure-side CIFP lookup in `xplane_context_runtime.cpp` keyed on
`ctx.nearest_airport_id`, which is a VFR-native concept: it follows the aircraft.
On an easterly Nice departure it reads LFMN, then LNMC (Monaco), then LIMG
(Albenga), then LIMJ (Genoa) — and the plugin dutifully resolved a departure
procedure for each field it flew over. Two consequences, one visible and one not:

- **visible** — a direct-to clearance naming a foreign SID's exit fix;
- **silent** — the climb ladder was computed for LNMC, which publishes nothing,
  so it fell back to FL110/FL140 instead of the FL100 that BASI8A publishes and
  that the delivery clearance had already read out. The pilot was cleared to an
  initial climb the departure controller then contradicted.

`s_departure_apt_id` in the engine was meant to prevent exactly this, but it was
captured at RADAR CONTACT — minutes after takeoff, by which time the drift had
already happened.

**Two independent defences, both in place:**

1. `ctx.ifr_departure_icao` / `_runway` / `_lat` / `_lon` are latched while the
   aircraft is ON THE GROUND and frozen at lift-off (OFP origin as the fallback
   for a session started airborne). Every SID lookup keys on the latch;
   `poll_sid_climb` prefers it over `nearest_airport_id`.
2. `route_has_fix()` — a direct-to may only ever name a fix on the filed route.
   A refusal is logged once per departure
   (`IFR SID: direct-to REFUSED -- <fix> is not on the filed route`), so the next
   occurrence of this class of bug announces itself instead of reaching the pilot.

`make test-depfield` covers both.

---

## Q11 - Two survivors of the bulk edit of 2026-08-18 - **FIXED 2026-08-29**

A scripted edit on 2026-08-18 grafted a full vectoring reset block into several
unrelated sites and was committed. Two copies were found and removed at the time
(the downwind->base and displace->intercept leg builders carry the note). Two
were missed, and were still in `1cba128`:

1. **The displaced-intercept vector builder.** The graft's last line,
   `s_vtf_recut_secs = 0.0f`, cancelled the `kVecLeadSecs` guard set three lines
   above — the one guarantee that an aircraft is given time to start its turn
   before the next correction is judged. It also wiped `s_atc_assigned_speed_kt`
   and `s_vtf_clearance_pending`, the two fields that record what the PILOT has
   been given.
2. **`poll_descent_second_step`**, on the branch that DROPS a stale stepped-
   descent target: consuming the target also reset the whole vectoring state,
   including `s_arrival_vectored` (the arrival's vectored/not-vectored latch) and
   any leg in progress.

Both removed. `make replay` and `make replay-lsgg` unchanged: the LSGG racetrack
still reads 043 -> 313 -> 253, 90 deg base, 33 deg intercept, released 2.0 NM
from GG512.

**Worth a habit, not just a fix:** these were found by reading around an
unrelated defect, not by any test. A grep for a reset sequence appearing at an
indentation that does not match its block would have found all four in seconds.

---

## Q12 - The runway correction dropped the side - **FIXED 2026-08-29**

Same Nice flight. An aircraft lined up on 04R heard:

```
November Romeo Charlie, negative, I say again, runway zero four.
```

`extract_runway()` returned an `int`, so `[lLrRcC]?` was matched and thrown away.
At a parallel-runway field the side is the word that carries the whole meaning,
and "zero four" is not a runway designator at all. Worse, 04L and 04R were
indistinguishable to the verifier: a readback of the wrong parallel passed.

Now compared as a full designator (`extract_runway_desig` -> "04R"), spoken side
("zero four right") or written ("04R"), and spoken back in full by
`runway_desig_to_speech`. A readback that omits the side is an incomplete
readback per ICAO Doc 4444 and draws the correction. `tests/test_readback_runway.cpp`.

**Known gap, deliberately left:** a readback with no "runway"/"approach" keyword
at all ("cleared for takeoff zero four right") is still invisible to the
extractor — that predates this change and every readback in the flown logs
carries the word.

---

## Q13 - The radio died at the Innsbruck handoff - **FIXED 2026-08-30**

The worst defect in `Log(39).txt`, and the reason there was no Approach clearance
and no Tower at LOWI: **the session was stuck in `PLAYING` from the Innsbruck
Radar handoff to touchdown.** Roughly ten minutes and an entire approach with
every push-to-talk refused:

```
[handoff] reminder suppressed: the readback carries 128.975 -- acknowledged
Set COM1 standby to 128975 (128.975 MHz)
PTT blocked, state=3
PTT blocked, state=3
...                       (to the parking stand)
```

State 3 is `PLAYING`. After that point the log contains **no** TTS request, **no**
synthesis result, **no** playback completion. The pilot flew the STAR, the
approach, and landed, without ATC.

**What could not be established.** Nothing logs the entry INTO a PTT state, so
the log cannot say who set `PLAYING`, or when. The candidates all survive:
`speak_response*` sets `PLAYING` + `tts_pending_` synchronously and relies on an
async callback; `synthesize_async` serialises on `g_tts_call_mtx`, so a worker
stuck under that lock silences every later request without printing anything;
`audio_player`'s `is_playing_` is cleared only by an FMOD completion callback,
and the PTT click that preceded the freeze has no matching "Playback finished".
The HTTP backends all carry timeouts (45 s TTS, 30 s STT/LM), which is what makes
a plain network hang the *least* likely explanation.

**What is in place.** Not a cure — a net, plus the instrument that was missing:

1. every PTT state change is logged with the duration of the previous state, so
   the next occurrence names itself;
2. `PLAYING` beyond 90 s or `PROCESSING` beyond 150 s is forced back to `IDLE`:
   `tts_pending_` cleared, `audio_player::stop()` called (it also clears a stuck
   `is_playing_`), and a System row tells the pilot the radio is back.

Both bounds sit far above the worst honest case. **A lost ATC message is a
nuisance; a lost radio is the flight.**

**The two unbounded waits, found by asking "where is the timeout?" (user,
2026-08-29).** The HTTP layer had timeouts all along — 45 s TTS, 30 s STT/LM —
but a timeout on a call bounds only the call that HOLDS the resource. Two waits
behind it were bounded by nothing at all, and both match the observed silence:

1. **`g_stt_call_mtx` / `g_lm_call_mtx` / `g_tts_call_mtx`.** One in-flight call
   per backend, taken with a plain `std::lock_guard`. A worker parked there waits
   for ever and prints nothing — exactly what the log shows: no request, no
   result, no failure. Now `std::timed_mutex` with `try_lock_for` (45 / 45 / 60 s,
   each a little above the HTTP timeout it queues behind); giving up is reported
   as an ordinary backend failure, so the caller's existing error path runs and
   the session leaves `PLAYING`.

2. **`audio_player::is_playing_`.** Cleared by one thing only: the FMOD
   completion callback. A callback that never arrives pins the flag, and
   `atc_session::update()` hangs its exit from `PLAYING` off it. The last PTT
   click before the freeze has no matching "Playback finished". The buffer's
   duration is known exactly when it is submitted, so `is_playing()` now clears
   itself past `length + 2 s` and says so in the log. No guesswork.

**Order of defence, from precise to blunt:** the audio deadline (exact, seconds),
the HTTP timeouts (30–45 s), the backend-lock timeouts (45–60 s), the PTT
watchdog (90 / 150 s). Each one below is only reached if everything above it
failed.

**ROOT CAUSE FOUND 2026-08-30, on the very next flight, by the state trace.**

```
[STT-MISTRAL] POST /v1/audio/transcriptions, 95744 samples
PTT state TAIL_RECORDING -> PROCESSING (after 0.9s)
STT response (quality=1.00): "one-two-eight decimal nine seven five in ..."
[handoff] reminder suppressed: the readback carries 128.975 -- acknowledged
Set COM1 standby to 128975 (128.975 MHz)
PTT blocked, state=3
...
PTT watchdog: stuck in PROCESSING for 150s (tts_pending=0 audio_playing=0)
```

Two things the trace settled at once, and the first corrects this entry:

1. **The stuck state was PROCESSING, not PLAYING.** `PTTState` is
   `{IDLE, RECORDING, TAIL_RECORDING, PROCESSING, PLAYING}`, so `state=3` is
   PROCESSING. Reading the flight-39 log as PLAYING was wrong, and it aimed the
   first round of fixes at TTS and audio -- neither of which was involved.
   `tts_pending=0 audio_playing=0` says so plainly.

2. **The engine never completed the transcript.** The STT came back fine; the
   handoff-readback suppression then hit a bare `return` -- the ONLY exit in the
   whole of `process_transcript` that did not call `done()`. Verified by script
   over the function: two bare returns, this one and a recursive re-dispatch that
   forwards `done` correctly. Without the completion callback `atc_session` never
   learns the transmission was handled and sits in PROCESSING for ever.

It fires on the frequency readback a pilot makes at EVERY handoff while still on
the old frequency -- the most ordinary transmission there is -- which is why it
cost two Innsbruck arrivals rather than showing up in a test.

Fixed: `done(Output{}); return;`. Silence is the right answer there; dropping the
callback is not how you say nothing. `tests/test_done_leak.cpp`, **verified to
fail against the unfixed engine** before being kept.

**The nets stay.** They are what turned the second occurrence from a lost arrival
into a 150 s gap: the watchdog fired, named the state, and gave the radio back.
The TTS-lock and audio-deadline bounds were aimed at the wrong layer but they
cost nothing and close real holes of their own.

---

## Q14 - The active airport walked down the arrival - **FIXED 2026-08-29**

Second reason LOWI would have been silent even with a working radio. Descending
through 13 000 ft the active airport went LOIK -> LOJK -> LOIZ -> LOIS -> LOII ->
LOIU -> LOWI, one airfield per minute under the flight path, and the whole way
down the frequency read:

```
COM1: 128.975 MHz -> Unknown | Airport: LOIK (1 freqs, ATIS=0.000, tower_only=0)
```

`Unknown`, because Innsbruck Radar 128.975 is an `airport+.json` controller and
matches no apt.dat frequency, and because the field being matched against was not
the destination. Runway, ATIS and towered/tower_only followed the same drift, and
`[[feedback_approach_freq_defines_intent]]` — a pilot on a known Approach
frequency must get a full clearance — cannot hold when the frequency classifies
as Unknown.

**Two causes, stacked. Both fixed.**

1. *The wrong field.* Symmetrically to the departure latch (Q10): airborne,
   within 60 NM of the filed destination, and with no frequency match already
   claiming a field, the **destination is the active airport**. Logged once per
   change.

2. *The overlay could not name its own frequencies.* Even with LOWI as the active
   field, 128.975 would still have read `Unknown`. The classifier's chain was
   apt.dat (`airport_freqs.lookup`) -> atc.dat (`airspace_db::lookup_by_freq`,
   and only when the resolved controller is a TRACON) -> `airport+.json`, **but
   only for the `info` role**. Innsbruck Radar is in the overlay precisely
   *because* atc.dat has no TRACON for LOWI and apt.dat does not list the
   frequency — so the plugin told the pilot to tune a frequency that, by
   construction, it could not then recognise.

   `airport_overrides::role_for_freq(icao, mhz)` is the inverse lookup that was
   missing: it walks the overlay's controllers and returns the role. APPROACH /
   DEPARTURE -> `APPROACH`, TOWER, GROUND, ATIS, INFO. `delivery` stays unmapped
   on purpose (shared ground/airborne — LFLU's Lyon 125.155). Consulted for the
   active airport **and** the destination, because the handoff comes well before
   the destination becomes the active field.

   It also puts `alt_freqs_mhz` to work: that key had been in the file since the
   overlay was written and **was parsed by nothing**, so LOWI's second arrival
   frequency 119.275 classified as UNKNOWN exactly like the primary. Tolerance is
   5 kHz, for 8.33 kHz channel designators.

`tests/test_airport_overrides.cpp`.
