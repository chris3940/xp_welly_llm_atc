# Open questions

Running log of questions raised in discussion that are **not settled** and not
yet implemented. Each entry carries the build it was raised against, so a later
reader knows what the code looked like at the time.

Answered questions move out of here into the relevant document or a commit
message; nothing is deleted silently.

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
