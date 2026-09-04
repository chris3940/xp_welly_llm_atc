# Airspace — how a volume is classified, and how "the TMA" is chosen

Specification of the airspace layer: how the plugin decides what a volume **is**,
which one it considers the aircraft to be **in**, and how it distinguishes a
terminal area it is merely **transiting** from the destination's **own**.

**Spec version:** 1.12 · **Dated:** 2026-08-21 · **Build:** v4.4.0-beta (pkg 107)
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
| `P` `Q` `R` `W` and the rest | OTHER | no |

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

**There is a fourth branch, and it is not a type word.** Cross-border delegation
overlays are named by their delegation marker, not by a type, and are usually
exported as a bare ICAO class letter:

```
AC C
AN DELEGATED BY LIMM TO LJLA FIR/UIR    → CTA
```

`DELEGATED`, `SKYGUIDE` and `MUAC` all classify as **CTA**, so `find_enclosing`
returns the overlay and the sector resolver routes it to the delegated ACC. This
is the mechanism behind the hand-written `airspace+.txt` overlays — it is how the
Slovenia upper-airspace gap and the LFFF → LSAS delegation were closed.

The branch order matters and is pinned in the source with a `NOLINT`: the **FIR**
test runs before the delegation test, because the example above carries *both* a
delegation marker and the word `FIR`, and must be classified by the type word.
Merging the two branches — which a lint rule would happily suggest, since both
assign CTA — would silently reclassify every such overlay.

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
block must win over the block. When OpenAir answers nothing — **or has not
finished loading** — the same query is retried against `atc.dat`, mapping its
roles (`TWR`→CTR, `TRACON`→TMA, `CTR`→CTA) and taking the shelf that actually
contains the point. OpenAir stays authoritative wherever it answers.

Two details that the callers depend on:

* **a ceiling of 0 means unbounded.** The altitude test is
  `ceiling > 0 && alt > ceiling`, so a volume exported without a top contains
  everything above its floor. A caller that reads a returned ceiling as a real
  altitude must check for zero first.
* **`find_all_enclosing()` is the sibling that returns them all**, in no
  particular order, for the callers that need the whole vertical stack over a
  point rather than the innermost slice of it.

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

### 4.1 Entering the TMA is ONE event with TWO consequences

Crossing into the destination's terminal volume is simultaneously:

* what justifies the **descent** that gets the aircraft in (`algorithm-descent.md`
  §2.3), and
* what makes the **terminal controller** responsible for it.

Both are read from the same geometry, by two polls that did not know about each
other, and the descent poll ran first. On the flight of 2026-08-19 that produced,
seven seconds apart:

```
Marseille: November Romeo Charlie, descend flight level 90.
Marseille: November Romeo Charlie, contact Geneva Approach on 119.530.
```

The aircraft had just descended into `GENEVA TMA SECTOR 10` (11500–15500). The
level plainly belonged to Geneva.

**Rule (pkg 105): the transfer wins.** When the destination's terminal handoff is
owed in the same frame — the aircraft is inside the terminal volume and the
handoff has not been spoken — the step-down is held and re-run after the handoff
block, in the same frame. Nothing can be lost: a destination with no approach
controller never produces a handoff, and the deferred step-down still fires.

The predicate deliberately requires being **inside** the volume, so the opposite
ordering survives untouched: above the ceiling (FL120 over a FL115 TMA) the
step-down still goes first, and it is what puts the aircraft inside.

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

## 8. Why the French and Swiss arrivals always worked

The user reports that these have never given trouble, naming the procedures:

| arrival | STAR | entry fix | spoken | terminal chain |
|---|---|---|---|---|
| LFLP | `ROMA3P` | ROMAM | "ROMAM Three Papa arrival" | Lyon → Chambéry → Annecy |
| LFLP | `SALE3P` | SALEV | "SALEV Three Papa arrival" | Genève → Chambéry → Annecy (from the east) |
| LFMN | `ABDI8R` | ABDIL | — | Nice → Cannes |
| EDLW | `ADEM3A` | ADEMI | "ADEMI Three Alpha arrival" | the case that fails |

Taken from the flight logs rather than from memory: `STAR entry_fix=... -> STAR=...`
in `Log.txt`, cross-checked against the spoken form in `transcript.log`.

Measured, that is not luck — it is the naming convention:

```
CHAMBERY TMA SECTOR 1        class D    name says TMA   -> TMA   OK
LYON TMA SECTOR 4            class C    name says TMA   -> TMA   OK
GENEVA TMA SECTOR 3          class C    name says TMA   -> TMA   OK
ANNECY CTR / CANNES CTR      class CTR                  -> CTR   OK

DUESSELDORF/COLOGNE-BONN Q   class C    no type word    -> CTA   FAILS
```

The resolver follows the same fate. It strips `" SECTOR"` and is left with
`CHAMBERY`, `LYON`, `GENEVA` — a clean single city that matches the atc.dat
TRACON. On the German name there is nothing to strip, the fragment stays the whole
compound string, and nothing matches.

**Both defects therefore have one root: the naming convention.** French and Swiss
data write `CITY TMA SECTOR n`; German data does not write the type word at all.

### It is the whole country, not one airport

Measured across the export, 170 volumes sampled over the sixteen main German
terminal areas:

| | count |
|---|---|
| name contains `TMA` | **0** |
| name contains `CTA` | **0** |
| `CTR` | 21 |
| **no type word at all** | **149** |

```
FRANKFURT A                  class C
MUNICH A                     class C
HAMBURG A                    class C
BERLIN I1 (WEST)             class C
DRESDEN SECTOR A             class D
MUENSTER-OSNABRUECK          class D
DUESSELDORF/COLOGNE-BONN A   class C
```

Only the CTRs carry their type word, which is why low-level operations work
everywhere. **Every German arrival is therefore without a TMA** — no descent rung,
no descend-to-enter, no terminal handoff through the OpenAir path. Dortmund is not
a special case; it is the first German arrival that was flown.

Note a second-order distinction inside the failures: `DRESDEN SECTOR A` strips to
`DRESDEN` and would resolve by name, while `DUESSELDORF/COLOGNE-BONN A` strips to
nothing usable. So the two defects do not fail together everywhere — the
classification fails on all 149, the name resolution only on the compound ones.

### 8.1 The other countries, measured

Nine countries, same file, counting only **controlled** classes (`A B C D CTR`)
whose name carries a known city of that country. Restricting to controlled classes
matters: an unfiltered count drags in danger and restricted areas (`R-`, `P-`,
`M-`) and reports a naming failure that does not exist.

The count applies the classifier's **own** rule — `find("CTR")`, then `find("TMA")`,
then `find("CTA")`, substring and in that order (`openair_db.cpp:197-202`) — so the
table reports what the plugin actually sees, not what an independent reading of the
names would suggest.

| pays | TMA | CTA | CTR | sans type | typées |
|---|---|---|---|---|---|
| Espagne | 230 | 1 | 28 | 1 | **100 %** |
| Autriche | 66 | 0 | 11 | 0 | **100 %** |
| Pologne | 37 | 0 | 8 | 0 | **100 %** |
| France | 115 | 39 | 20 | 2 | **99 %** |
| Italie | 36 | 67 | 32 | 2 | **99 %** |
| Suisse | 82 | 12 | 11 | 2 | **98 %** |
| Benelux | 47 | 26 | 12 | 2 | **98 %** |
| Royaume-Uni | 27 | 59 | 14 | 9 | **92 %** |
| **Allemagne** | **1** | **0** | **28** | **138** | **17 %** |

**Germany is the sole outlier**, and the gap is not marginal — 138 controlled
volumes with no type word, against 0 to 9 everywhere else. Its single typed TMA is
`FRIEDRICHSHAFEN TMA`, which is why the earlier "0 of 170" figure in version 1.5
was slightly wrong: the sample list behind it omitted Friedrichshafen. The
conclusion is unchanged, and the corrected count makes it sharper — one German
field in the sample names its terminal area, and it is not one of the majors.

Three methods were tried and discarded before this one, all recorded so the figures
are not re-derived the same wrong way:

1. **Attributing by polygon centroid to a country bounding box.** Contaminated by
   the borders — the German box captured `LIEGE TMA FOUR`, `MAASTRICHT TMA 2`,
   `NIEUW MILLIGEN TMA C1` and `SALZBURG TMA GERMAN 1`, and reported 53 German
   TMAs where there is one.
2. **Matching city names without filtering the class.** `ROMA` matched
   `ROMANESTI`, `SAINT-ROMAIN` and `REG PARK VENA DEL GESSO ROMAGN` — all
   restricted areas — and reported Italy at 8 %.
3. **Testing the type word on a WORD boundary instead of as a substring.** This
   reported Poland at 73 % and produced a defect entry (A4) claiming its upper
   terminal areas — `GDANSK UTMA`, `KRAKOW UTMA SECTOR A` — went unrecognised.
   The classifier uses `std::string::find`, so `UTMA` already contains `TMA` and
   all six resolve to `AirspaceClass::TMA`; verified against the export itself,
   and Poland is at 100 %. **There was no A4** — the defect was in the measuring
   script. `tests/test_openair_db.cpp` now pins the behaviour, so a later
   "tighten the match to whole words" cleanup cannot silently un-index them.

A third trap, worth stating because it inverts a conclusion: **the export uses
English city spellings.** There is no volume named `MILANO`; the Milan terminal
areas are `MILAN TMA`, `MILAN TMA ZONE 5 VARESE`, `MILAN CTA ZONE 1 BRERA`. An
Italian-spelling sample returns zero matches and reads as a total naming failure
when Italy is in fact at 99 %.

## 9. The unit, not the volume

A related modelling error, visible on the published chart. The arrival crosses
**three sectors of the same TMA**, differing only by floor:

```
DUESSELDORF/COLOGNE-BONN N   4500 MSL -> FL100
DUESSELDORF/COLOGNE-BONN Q   FL065    -> FL100
DUESSELDORF/COLOGNE-BONN R   FL065    -> FL100
```

`terminal_tma()` returns **one volume**. The reality is a stack of adjacent
sectors belonging to **one controlling unit**: crossing N then R then Q is staying
with Düsseldorf throughout, and warrants one handoff at the start and none
between. The right abstraction is the unit, not the volume.

`on_destination_terminal()` already works this way — it compares controller name
FRAGMENTS so that `CHAMBERY TMA SECTOR 2` and `CHAMBERY CTR` count as the same
unit. The handoff path lacks the same idea.

## 10. Vectoring onto final is PUBLISHED at EDLW

Not an option, and not a training aid. The ADEMI 3A arrival chart carries:

> RADAR vectoring will be provided onto final approach track.

So on this arrival `FORCE APP VECTORING` models what the procedure states, which
argues for it being the default behaviour at fields whose charts say so rather
than a checkbox the user must find. How that would be detected from data — the
charts are not machine-readable here — is undecided.

---

## 11bis. The terminal stack walk (shipped)

Answers section 8.1's defect for the countries the naming fails. Selects the
terminal shelf from the SHAPE of the vertical stack instead of from names:

1. every indexed volume containing the point;
2. the base is the CTR — **no CTR means no terminal structure**, so an enroute
   position can never yield a shelf (inferring one from a low-floored ACC sector
   is the 2026-08-14 misfire, 186 NM out);
3. the shelf is the lowest-floor non-CTR volume sitting on the CTR
   (`floor <= ctr_ceiling + 500`) and reaching above it.

Named in the source: the base tolerance is `kStackGapToleranceFt = 500`, the
guard is `kMaxTerminalCeilingFt = 30000` (both in `openair_db.hpp`), and ties on
the floor break to the **higher ceiling**. One guard, `ceiling <= 30 000 ft` —
above FL300 it is an ACC sector, not a terminal area. It was **measured, not assumed**, over the 1355 points where a
named TMA exists so the right answer is known (Europe):

| guards | exact | other | none |
|---|---|---|---|
| none | 90 % | 6 % | 5 % |
| **ceiling ≤ 30 000** | **91 %** | **4 %** | **5 %** |
| + thickness ≤ 20 000 | 86 % | 8 % | 6 % |
| + extent ≤ 200 NM | 82 % | 11 % | 7 % |

The intuitive thickness and extent caps make it **worse** — they reject
`LONDON TMA` (4500–19500). Two other candidate signals were dropped the same
way: floor/ceiling/extent do not separate TMA from CTA at all (medians 4500 vs
6500 ft, 54 vs 61 NM), and the name-stem CTR match scores 81 % against 54 % but
its "errors" are mostly terminal CTAs.

**Off by default, twice.** A named TMA always wins, so the walk only runs where
the export is silent; and callers opt in per destination through
`ifr_defaults.terminal_stack_walk_icao_prefixes` (`["ED","LI"]`), keyed on the
assigned destination so it is off for the whole cruise and off everywhere else.

Measured on the export, `enc` in the IFR REPL printing both answers:

```
EDLW/KOLOT  name (none)               ->  walk DUESSELDORF/COLOGNE-BONN 1500-10000
LIMF        name (none)               ->  walk MILAN CTA ZONE 24 3500-9500
LIMC  name MILAN TMA ZONE 1 LOMBARDIA ->  walk identical
LFMN  name NICE TMA SECTOR 1          ->  walk identical
LFLP  name CHAMBERY TMA SECTOR 1      ->  walk identical
```

### The anchor decides the answer

The walk alone was not enough, and the first EDLW result was wrong. Asked over
the **field** it returns `DORTMUND` 2000-4500 -- a sector overlapping the control
zone (floor 2000 beneath the CTR's 2500 ceiling), not a terminal area. Its
ceiling would have set the terminal top at 4500 and cleared an arrival to
**4000 ft** to enter the terminal area, 5500 ft low.

Asked over the **FAF** it returns `DUESSELDORF/COLOGNE-BONN` 1500-10000 -- class
C, topping FL100 as published, one volume covering the whole band.

```
EDLW field          KOLOT (FAF)
0-2500   CTR        0-2500   CTR
2000-4500 DORTMUND  1500-10000 DUESSELDORF/COLOGNE-BONN   <- the TMA
---- hole ----      6500-10000 DUESSELDORF ... Q
10000+ CLASS C      10000+ CLASS C
```

The Düsseldorf TMA covers the FAF but **not the field**. What encloses the field
between 4500 and 10 000 ft is the class E blanket, which this export does not
carry (defect A2) -- so the hole and the wrong answer have the same cause, and
the field is simply the wrong place to ask. This is cause 1 of Q3, now fixed.

The anchor moved **only** where the stack walk is enabled, so every field whose
export names its TMAs keeps the field anchor it was validated on.

**LIMF Turin is the find.** Italy is 99 % typed overall, yet Turin has no
TMA-named volume overhead — its terminal airspace is `MILAN CTA ZONE 24 DON
BOSCO`. An arrival there would have failed exactly like Dortmund; it was never
seen because Turin has only ever been flown as a departure. A national
percentage does not certify a field.

**Not yet flown.** Departures are not covered: the gate keys on the destination,
which is empty during climb, so an ED** departure keeps the name-based
behaviour.

### A name collision worth knowing about

`engine.cpp` declares its **own** `kMaxTerminalCeilingFt`, locally, at **25 000
ft**, alongside a `kMaxTerminalThicknessFt = 15 000`. It belongs to a different
mechanism — the transfer-floor plausibility clamp, which decides whether a volume
below the aircraft is a terminal area owing a transfer or enroute structure owing
none (it exists because EDLW's generic 10 000–66 000 ft slab produced "below
inner-TMA transfer floor 66000 ft" and suppressed every descent, seen in a real
flight, Log 19).

Two constants, one name, two values, two files. Nothing is wrong today — they are
in different translation units and answer different questions — but a reader who
greps the name gets one of the two at random, and a "harmonisation" that made
them agree would break one of the two mechanisms.

Note also the tension with the table above: the measurement rejected a
**thickness** cap for the stack walk (it throws out `LONDON TMA`, 4500–19500),
while the transfer-floor clamp uses one at 15 000 ft. Both can be right — they
are asked over different volumes — but if the clamp ever misfires, that number is
the first place to look. See `docs/candidates/transfer-floor-plausibility.md`.

---

## 11. Known defects

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

## How long the sector state lives

The boundary logic is stateful: `s_acc_sector_freq_khz` is the sector currently
working the aircraft, `s_acc_visited_sector_freqs` the ones already left (which is
what stops a handoff flickering back at a boundary), and
`s_enroute_approach_handoff_issued` latches the commitment to the approach
controller.

All are per flight — and until pkg 107 "per flight" meant *per plugin load*. A
flight restarted inside the same X-Plane session kept the whole set, so on
2026-08-20 the pilot heard:

```
[31:43 119.755] Geneva Approach: November Romeo Charlie, confirm descending flight level 90.
                @(44.4502,4.7794 alt=17947ft)
```

— Geneva Approach, on Marseille's frequency, two hundred miles from Geneva, for a
level cleared in a previous attempt. Since pkg 107 a position discontinuity
resets the lot; see `algorithm-descent.md` §8 for the mechanism and its blind
spot.

---

## Revision history

Every specification document carries a **version**, a **date** and the **build** it
was written against, so a reader can tell when the specification changed and what
the code looked like at the time.

| version | date | build | change |
|---|---|---|---|
| 1.0 | 2026-08-17 | v4.4.0-beta-85 (`a6bb13b`) | first issue: classification, enclosing volume, transiting vs destination, the EDLW worked example |
| 1.1 | 2026-08-17 | v4.4.0-beta-85 (`a6bb13b`) | section 7 added — the suppressed Düsseldorf handoff, traced to the name comparison running the wrong way rather than to the CTA classification |
| 1.2 | 2026-08-17 | v4.4.0-beta-85 (`a6bb13b`) | sections 8–10: why the French and Swiss arrivals always worked (naming convention is the single root of both defects); the unit-not-volume modelling error; radar vectoring is published on the EDLW chart |
| 1.3 | 2026-08-17 | v4.4.0-beta-85 (`a6bb13b`) | named the validated arrivals so the working set is a checkable list rather than "the French ones" |
| 1.4 | 2026-08-17 | v4.4.0-beta-85 (`a6bb13b`) | exact STAR designators recovered from the flight logs: `ROMA3P`, `SALE3P`, `ABDI8R`, and `ADEM3A` for the failing case, each with its entry fix and spoken form |
| 1.5 | 2026-08-17 | v4.4.0-beta-85 (`a6bb13b`) | measured the naming across Germany — 0 of 170 volumes carry `TMA` or `CTA`, 149 carry no type word at all. The defect is national, not per-airport; other countries remain unmeasured |
| 1.6 | 2026-08-17 | v4.4.0-beta-85 (`a6bb13b`) | section 8.1 — the other eight countries measured. Germany is the sole outlier at 17 % typed against 92–100 % elsewhere; corrects 1.5's "0 of 170" (`FRIEDRICHSHAFEN TMA` exists); new defect A4 (`UTMA` unrecognised, Poland); records the three measurement traps — border contamination, unfiltered classes, and English city spellings in the export |
| 1.12 | 2026-08-21 | v4.4.0-beta (pkg 107) | **specification confronted with the code, §1-3 and §11bis.** Pass 2 was missing an entire branch: the `DELEGATED` / `SKYGUIDE` / `MUAC` names classify as CTA, which is the mechanism behind the hand-written `airspace+.txt` overlays (Slovenia, LFFF -> LSAS), and the FIR test must run before it -- documented with the reason the branches cannot be merged. §3 gains the two details callers depend on: a ceiling of 0 means unbounded, and `find_all_enclosing()` returns the whole stack. §11bis now names its constants (`kStackGapToleranceFt` 500, `kMaxTerminalCeilingFt` 30000) and records the collision with `engine.cpp`'s own local `kMaxTerminalCeilingFt` at 25000, which belongs to the transfer-floor clamp. Pass 1 table completed with class `W`. Everything else in these sections verified conforming |
| 1.11 | 2026-08-21 | v4.4.0-beta (pkg 107) | new **§4.1 entering the TMA is one event with two consequences**: the same boundary crossing justifies the descent AND transfers control, and the transfer wins -- the step-down is held and re-run after the handoff block in the same frame (pkg 105), keyed on being INSIDE the volume so the above-the-ceiling ordering is unchanged. New **how long the sector state lives**: the sector baseline, the visited-sector list and the approach-handoff latch survived a flight restart inside one X-Plane session until pkg 107, which is how Geneva Approach came to speak on Marseille's frequency two hundred miles away |
| 1.10 | 2026-08-17 | v4.4.0-beta (`d7c3f64`, pkg 87) | **the FAF anchor is withdrawn from the two COMPARISON sites.** `on_destination_terminal()` and `dest_terminal_tma_below()` compare a destination probe against one under the aircraft; moving only one breaks the comparison by construction. It read `DUESSELDORF/COLOGNE-BONN` at the FAF against `DORTMUND` under an aircraft on final, returned false, and no approach clearance was issued on the whole EDLW arrival of 2026-08-17. The anchor stays where a terminal CEILING is read — the descend-via level and the descent ladder's rung |
| 1.9 | 2026-08-17 | v4.4.0-beta-85 (`a6bb13b`) | the destination terminal query anchors on the FAF where the walk is enabled (`9d75c6a`). Corrects 1.8, which reported `DORTMUND 2000-4500` as the EDLW result: that volume overlaps the control zone and tops 5500 ft below the real terminal area. Over the FAF the answer is `DUESSELDORF/COLOGNE-BONN` 1500-10000, class C to FL100. Resolves cause 1 of Q3 |
| 1.8 | 2026-08-17 | v4.4.0-beta-85 (`a6bb13b`) | section 11bis — the terminal stack walk, shipped (`eee4c8b`). Guard thresholds measured rather than assumed; the intuitive thickness and extent caps degrade the result and were dropped. Off by default and gated per destination on an ICAO-prefix allowlist. Records LIMF Turin as a second failing field inside a 99 %-typed country — found only because arrivals were probed, never flown |
| 1.7 | 2026-08-17 | v4.4.0-beta-85 (`a6bb13b`) | **A4 withdrawn — it was never a defect.** The classifier matches the type word as a SUBSTRING, so `UTMA` already contains `TMA`; all six Polish upper terminal areas resolve to `TMA` in the export and Poland is at 100 %, not 73 %. The word-boundary test was in the measuring script alone. Table recounted using the classifier's own rule and order; a regression test now pins `UTMA`. Germany remains the only outlier |
