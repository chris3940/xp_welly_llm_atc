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
