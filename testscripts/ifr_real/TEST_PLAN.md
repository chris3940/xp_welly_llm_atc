# In-sim test plan — beta-61

Supersedes `SID_TEST_PLAN.md` (beta-56), which is kept for its SID-climb detail.

**Principle: fly only what cannot be tested on the ground.** Everything the
headless harnesses already cover is listed below so it is *not* re-flown. Run
them first — they take seconds:

```
make test          # 239 cases / 2974 assertions
make test-afis     # real Custom Data: departure, SID handoff, EDLW arrival
make test-stars    # STAR tracker
```

For each flight drop **both** `Log.txt` and `transcript.log` in
`~/Téléchargements/`. `transcript.log` carries the per-event
coordinates / altitude / heading that `Log.txt` does not.

---

## Already covered headless — do NOT spend a flight on these

| covered | by |
|---|---|
| ILS/VOR/LOC/NDB final segment read (EDLW ILS 06 → 6 waypoints, not 2) | `test-afis` |
| `FORCE ILS` selects the ILS, rejects the RNAV, and says so in the clearance | `test-afis` |
| STAR resolution from the filed last fix | `test-afis` |
| Departure not cleared into a neighbour's TMA (Annecy → Geneva) | `test-afis` |
| Sector handoff resolves laterally, survives the departure hold | `test-afis` |
| SID climb floor = earliest constrained fix | `test-afis` |
| Airspace classification, incl. the two ex-gaps | `make test` (14 openair cases) |
| Non-RNAV FAF/MAP indexing in the procedure list | `make test` (3 ILS cases) |

---

**Three flights, under an hour in total, cover every risky fix.** The airspace
classifier is deliberately last: it is already fenced by 14 unit tests and a
10 800-point measurement, so it is the least urgent thing to fly.

---

## Flight 1 — EDLW, starting at DIK *(~20 min, highest value)*

The one thing blocking everything else: the arrival chain has failed twice, and
the two fixes for it have never flown.

**Setup.** Load the SimBrief OFP (without it the STAR and the filed steps do not
exist and the test is meaningless), then start in cruise at **DIK**, FL270-280.
114 NM to run. **Stay on the route** — leaving it is a separate test.

From DIK you get the whole descent sequence, not just its end:

| at | what it exercises |
|---|---|
| ERUKI, 17 NM after DIK | the filed step down to FL230 |
| en route | Brussels → Hannover → Langen handoffs |
| ~BAMSU | top-of-descent negotiation and the STAR clearance |
| last 23 NM | arrival, ILS 06, Tower |

Note DIK is 114 NM out, just above the 100 NM bound added after the Nancy
misfire — so this also proves the bound does not block a legitimate descent.

**Look for, in order:**

```
IFR arrival handoff: [P2-apt.dat dest EDLW] Langen Radar 125.225
IFR approach: FAF resolved = KOLOT
[route] FAF ap_idx=N MAP ap_idx=M            <- N must be >= 0
```

| symptom | meaning |
|---|---|
| no `[P2-apt.dat dest EDLW]` line | the destination lookup still fails — send the log |
| `FAF ap_idx=-1` | a code path still resolves the FAF after the scan |
| descent stops at the cleared level near the field | step-downs still not issued |

**Also expect:** a handoff to Tower near KOLOT, which never happened on either
previous attempt.

## Flight 2 — Nice departure *(~15 min, the sharpest single check)*

Covers a completely separate set from flight 1, and contains the only place in
the plugin where a clearance depends on the AIRCRAFT TYPE.

**Setup.** Depart LFMN 04L or 04R, any destination that gives you one of the SIDs
below.

**Expected initial climb** — these come from the published chart and are NOT in
the navdata, so they exercise `airport+.json` end to end:

| SID | expected | note |
|---|---|---|
| PERU8A · BADO8A · BODR8A · IRMA8A | **FL130** | single value, no type split |
| BASI8A · EPOL8A · LANK8A · TURI8A · VARE8A · SODR8A · RUBA8A | FL100 | jet SIDs |
| **BASI8X** | **FL070 with the TBM** | jets FL100 / props FL070 — the split |
| EPOL8B · LANK8B · TURI8B · VARE8B · SODR8B | **FL070** | prop variants |

**The sharp check:** on a split SID the TBM must get the PROP figure. Hearing
FL100 on BASI8X would be the bug, and nothing else in the plugin would reveal it.

Also validated here, both unflown: the departure handoff now names the outgoing
controller correctly (it used to say "you are still with Chambery Approach,
contact the next controller"), and "contacting tower on …" is recognised as a
readback instead of drawing "you are still on Ground".

Look for:

```
airport_overrides: LFMN SID <name> -> initial climb <alt> ft
```

Runway crossings on the way out are worth watching too (04L/04R).

## Flight 3 — LOWI, from your saved TOD point *(~20 min)*

Enough for what matters here: the **curved RNP final**, which has not flown since
the non-RNAV reader fix, at the airport where terrain margins are tightest.

It will NOT test the airspace classifier — that needs the whole leg. Measured
along LFMN → LOWI (278 NM), the three upper blocks are: `FREE RT ASPC SE` from
0 NM, `IT ZONE A` from 35 NM (208 NM of it), `DFS EDMM SOUTH` for the last 35 NM.
Starting near the TOD gives you one transition out of three, and misses the
France → Italy change entirely.

## Flight 4 — LFMN → LOWI in full *(the airspace classifier, when time allows)*

Most discriminating route for the classification change: **61 of 61** sampled
points newly resolve, across **three** upper blocks (Italy, France, Munich), so
it exercises the *transitions* and not just the presence of volumes. Ends with
the curved RNP final, unflown since the non-RNAV fix.

**Look for:**

```
openair_db: N airspace entries indexed        <- ~10239, not 8100
IFR en-route: sector change -> <name> <freq>  <- handoffs should chain, not bunch
[acc] atc.dat fallback resolved ...           <- should become RARE at altitude
```

**Alarm signal:** a handoff to an unexpected controller in cruise, or several
"contact X" in quick succession — that would mean a newly visible small volume is
winning the innermost-volume test when it should not.

## Flight 5 — LFLP → LFMN *(non-regression witness)*

Deliberately the *least* affected route: only 15 of 61 points change, and those
are Alpine LTAs rather than upper blocks. At FL190 you stay below the layer the
classifier touched. **It must behave exactly as before.** Any difference here is
a regression, not a feature.

## Flight 6 — LIMF → LFLP *(Milan UIR, optional)*

Short and targeted. 24 of 61 points newly resolve, all in `IT ZONE A` — the Milan
UIR, where the symptom was first noticed. Confirms that one case without
committing to a long leg.

---

## Time acceleration — check on any of the above

`Log.txt` must report the factor once per change:

```
Sim time: running at 2.0x wall clock -- all ATC timers follow sim time
```

If you accelerate and this line never appears, say so: it would mean the sim
clock is not behaving as assumed, and every timer in the plugin depends on it.

---

## Known and deliberately NOT fixed — do not report as new

- Leaving the route makes ATC go inert: it repeats the same challenge every
  180 s, never escalating to vectors or a direct. Recovery is the next feature.
- The route tracker credits fixes passed up to 18 NM abeam, with no lateral cap.
- Nothing watches "close to the destination and far too high" while off-route.
- `request a descent` (with the article) is not recognised.
- The two public-build items: ALLOW VECTORING, and pilot level requests with ATC
  able to refuse.
