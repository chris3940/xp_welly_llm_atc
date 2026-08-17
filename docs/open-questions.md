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
