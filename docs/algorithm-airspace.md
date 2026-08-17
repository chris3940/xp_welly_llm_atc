# Airspace — how a volume is classified, and how "the TMA" is chosen

Specification of the airspace layer: how the plugin decides what a volume **is**,
which one it considers the aircraft to be **in**, and how it distinguishes a
terminal area it is merely **transiting** from the destination's **own**.

**Spec version:** 1.1 · **Dated:** 2026-08-17 · **Build:** v4.4.0-beta-85 (`a6bb13b`)
**Sources:** `Custom Data/airspaces/airspace.txt` (OpenAir) — authoritative;
`Custom Data/Earth nav data/atc.dat` — fallback and controller names.
Measured against AIRAC 2606 r1.

---

## 1. What the data gives us

An OpenAir record carries a class token and a name, and nothing that directly
says "this is a terminal area":

```
AC C                              ← class token: an ICAO letter, or a type word
AN DUESSELDORF/COLOGNE-BONN Q     ← free-text name
AL 6500 MSL                       ← floor
AH 10000 MSL                      ← ceiling
DP ...                            ← polygon
```

The `AC` token is **either** a type (`CTR`, `TMA`, `CTA`, `FIR`, `UIR`) **or** a
bare ICAO class letter (`A`–`G`). Which one you get depends on the exporter, and
on this export it is overwhelmingly the letter.

---

## 2. Classification — two passes

### Pass 1 — the class token

| `AC` token | classified as | indexed? |
|---|---|---|
| `CTR` `TMA` `CTA` `FIR` `UIR` | itself | yes |
| `A` `B` `C` `D` | **CTA** (the en-route default) | yes |
| `E` `F` `G` | — | **no** |
| `P` `Q` `R` and the rest | OTHER | no |

Classes A–D are controlled and carry a clearance obligation. E, F and G are not
indexed. **On this export the point is moot: it contains no E, F or G at all** —
the classes present across 25 759 volumes are `A B C CTR D P Q R`. See defect A2.

### Pass 2 — the name refines it

When the class came from a **letter**, a type word in the name overrides the
CTA default:

```
AC D                    → CTA (letter)
AN BRAVO TMA            → TMA (name wins)
```

When the class came from a **type token**, the name cannot override it.

This ordering matters and was got wrong once: an earlier revision let the name
refine the class *only* when the first pass produced OTHER, so every
letter-classed TMA silently stayed a CTA, `terminal_tma()` returned nothing and
the whole terminal-descent path went dead. Caught by `tests/test_openair_db.cpp`
before it ever flew.

**So, in one sentence: a volume is a TMA because its NAME says so.** The class
letter alone never produces one.

---

## 3. Which volume the aircraft is "in"

`find_enclosing(lat, lon, alt)`:

1. bounding-box reject, then altitude range, then full point-in-polygon;
2. among the survivors, keep the **smallest bounding box** — the innermost.

Innermost, not lowest and not highest: a terminal sector nested inside an ACC
block must win over the block. When OpenAir answers nothing, the same query is
retried against `atc.dat`, mapping its roles (`TWR`→CTR, `TRACON`→TMA,
`CTR`→CTA) and taking the shelf that actually contains the point. OpenAir stays
authoritative wherever it answers.

---

## 4. Transiting versus the destination's own

This is decided by **where the query is anchored**, not by any property of the
volume:

| question | query | anchored at |
|---|---|---|
| what am I flying through? | `find_enclosing` | the **aircraft** |
| what is the destination's terminal area? | `terminal_tma` | the **destination airport** |
| is what I am in the destination's? | `on_destination_terminal` | both, compared |

`terminal_tma(lat, lon)` keeps only entries classified **TMA**, and among those
the one with the **lowest floor** over the point (ties broken by the higher
ceiling) — the base shelf of the stack, not the slice the aircraft happens to be
in.

`on_destination_terminal()` probes the destination just under its terminal
ceiling, probes the aircraft, and compares **controller name fragments** with the
type words stripped — so `CHAMBERY TMA SECTOR 2` and `CHAMBERY CTR` both reduce
to `CHAMBERY` and count as the same unit. It returns **true when it cannot tell**,
deliberately, so a field with no nested terminal areas is never blocked. That
permissive default is safe for its original caller (may the approach clearance be
issued?) and dangerous for any caller that reads it as "suppress" — a distinction
that has bitten once already.

---

## 5. Worked example — EDLW, and why nothing is found

Measured 2026-08-17. **Both sources agree at every point.**

```
KOLOT (the FAF)                        EDLW (the field)
  0-2500     DORTMUND CTR      CTR       0-2500     DORTMUND CTR      CTR
  2500-4500  DORTMUND SECTOR B CTA       2500-4500  DORTMUND SECTOR B CTA
  4500-6500  DUESSELDORF/C-B   CTA       4500-10000 ---- nothing ----
  6500-10000 DUESSELDORF ... Q CTA
  10000+     AIRSPACE CLASS C  CTA       10000+     AIRSPACE CLASS C  CTA
```

`terminal_tma(EDLW)` returns **empty**; `terminal_tma_ceiling` returns **0**.
Three independent reasons, and removing any one alone would not help:

1. **Anchoring.** The terminal airspace relevant to this arrival lies over the
   FAF, where the stack is continuous from the ground up. Over the field there is
   a 4500–10 000 ft hole.
2. **Classification.** Even anchored on the FAF it would fail at the altitude
   that decides: the aircraft crosses KOLOT at **3000 ft**, inside
   `DORTMUND SECTOR B` — classified **CTA**, because its name carries no type
   word. Nothing classified TMA exists at the crossing altitude anywhere on this
   arrival.
3. **Missing class E.** The published blanket to FL100 is what would fill the
   hole over the field, and the export carries no class E.

Consequences: the TMA rung of the descent ladder never exists, and
`descend-to-enter-TMA` can never fire. An EDLW arrival rests entirely on the CIFP
constraint chain.

---

## 6. Reading it in a log

A volume's OpenAir name is **not** what the pilot hears. The export strips names
off the blanket blocks, so the airspace the controller correctly calls *Langen* is
recorded as `AIRSPACE CLASS C`. Log lines therefore print both — the data's name
first, then the controller it resolves to through `atc.dat`:

```
AIRSPACE CLASS C (Langen)
```

The blanket blocks are the whole point of the format: a named volume gains
nothing from the parenthesis, while `AIRSPACE CLASS C` alone tells the reader
nothing at all.

Resolving them needs the **position**. A blanket block carries no usable name, so
the name-based resolver cannot place it — the label therefore falls back to the
same lookup the sector-change path uses: `atc.dat` at that point, innermost
centre, `OCEANIC` discarded, then its spoken label.

---

## 7. Consequence: a whole handoff is lost

The classification is not only a descent problem. On the flown arrival the aircraft
crosses `DUESSELDORF/COLOGNE-BONN Q` between 6500 and 10 000 ft and **is never
handed to Düsseldorf**. The log says why:

```
[approach] sector handoff suppressed -- DUESSELDORF (EDDL) not a
           non-dest terminal (force_forward=0 dest=EDLW)
```

The candidate was found, with its facility. The approach-phase sector block then
allows **only** a forward handoff to another facility's terminal area, which
requires `force_forward` — and that flag is raised solely when the enclosing volume
resolves to a terminal controller **by name**:

```
resolve_terminal_ctrl("DUESSELDORF/COLOGNE-BONN Q")
    strip type words (TMA, CTA, FIR, UIR, SECTOR, SEC) → none present
    fragment = "DUESSELDORF/COLOGNE-BONN Q"

find_by_role_name_contains(TRACON, fragment)
    tests   controller_name.find(fragment)
    i.e.    "duesseldorf".find("duesseldorf/cologne-bonn q") → npos
```

**The comparison runs the wrong way.** It asks whether the CONTROLLER's name
contains the VOLUME's name, when it is the volume that carries the long compound
name. `DUESSELDORF` does not contain `DUESSELDORF/COLOGNE-BONN Q`, so no TRACON is
found, `force_forward` stays false, and the handoff is suppressed.

Measured 2026-08-17: the cause is the **name matching**, not the CTA
classification. Both would have to be right for the handoff to fire, so A1 still
stands, but this one is nearer the surface and much smaller to fix — matching on
the leading token, or testing containment in the other direction, would resolve
`DUESSELDORF/COLOGNE-BONN Q` to `DUESSELDORF`.

The suppression rule itself is sound and was written from real flights: it stops a
handoff to the destination's own facility (Innsbruck announcing its Tower as
"Approach" while the aircraft was already on Innsbruck Radar) and to an en-route
sector caught in passing (Vienna or Munich during the LOWI reversal).

---

## 8. Known defects

**A1 — a volume is a TMA only if its name says so.**
`DORTMUND SECTOR B` is a terminal sector by any operational reading and is
classified CTA, because the classifier has nothing else to go on. Any rule that
keys on "is this a TMA" inherits that.

**A2 — no class E, F or G in the export.**
Class E is IFR separation airspace. Where it is the only controlled airspace —
Germany between the CTA tops and the upper block — the plugin sees a hole. Either
an `airspace+.txt` overlay carries the missing volumes, or every rule that depends
on terminal coverage has to tolerate its absence.

**A3 — `on_destination_terminal()` returns true when it cannot tell.**
Correct for the caller it was written for, wrong for any caller that reads the
result as "suppress". It silenced every sector handoff of an entire replay once,
which was misdiagnosed as a plugin defect before being traced to a missing
airport position in the harness.

---

*Specification in progress. The descent that consumes these queries is specified
in `algorithm-descent.md`; open questions are tracked in `open-questions.md`.*

---

## Revision history

Every specification document carries a **version**, a **date** and the **build** it
was written against, so a reader can tell when the specification changed and what
the code looked like at the time.

| version | date | build | change |
|---|---|---|---|
| 1.0 | 2026-08-17 | v4.4.0-beta-85 (`a6bb13b`) | first issue: classification, enclosing volume, transiting vs destination, the EDLW worked example |
| 1.1 | 2026-08-17 | v4.4.0-beta-85 (`a6bb13b`) | section 7 added — the suppressed Düsseldorf handoff, traced to the name comparison running the wrong way rather than to the CTA classification |
