# Candidate — the transfer-floor clamp must check it is clamping a *terminal* area

Status: **not applied.** Measured, understood, and deliberately held back
(2026-08-15) because it touches a guard shared by several arrival phases.

## What happens today

`sector_transfer_floor_ft()` (src/atc/engine.cpp) returns
`lower.ceiling_ft` — the ceiling of whatever volume contains the target
level — and `poll_profile_crossing()` refuses any descent below it, on the
correct principle that a controller must not descend an aircraft into the
next unit's airspace before transferring it.

Nothing checks that `lower` is actually a terminal area. On the DIK → EDLW
replay the volume below was the generic upper block over Germany:

```
[dbg prof] crossing KOLOT -> 2500 ft SUPPRESSED:
           below inner-TMA transfer floor 66000 ft (belongs to inner controller)
```

66000 ft is the *ceiling* of `AIRSPACE CLASS C` (10000–66000). Every
conceivable descent is below it, so the clamp suppresses **all** of them,
permanently. In the replay the aircraft held its level to 2 NM from the
field.

## The change

In `sector_transfer_floor_ft()`, before returning:

```cpp
constexpr int kMaxTerminalCeilingFt   = 25000; // above this it is not a TMA
constexpr int kMaxTerminalThicknessFt = 15000;
if (lower.ceiling_ft > kMaxTerminalCeilingFt ||
    (lower.ceiling_ft - lower.floor_ft) > kMaxTerminalThicknessFt)
  return 0;   // enroute structure owes no transfer
```

Rationale: a terminal area does not top out in the flight levels and is not
tens of thousands of feet thick. Anything that is, is enroute structure.

## Measured effect (DIK → EDLW, testscripts/ifr_real/fly.py)

| | without | with |
|---|---|---|
| last descent issued | FL100, then silence | FL100 → **FL60** |
| level at 10 NM | FL100 | FL60 |

Still late — the aircraft holds FL100 until ~10 NM — so this is a necessary
step, not the whole answer.

## Why it is held back

The clamp does real work where the data is good: over LFLP it stops Geneva
descending an aircraft into the Chambéry TMA, an in-sim bug fixed on
2026-07-15. Loosening it risks reopening exactly that. Before applying:

1. Re-run the LFLP arrival (Geneva → Chambéry) and confirm no clearance
   crosses into the inner TMA before the handoff.
2. Check the thresholds against real terminal areas — 25000 ft is comfortably
   above any TMA ceiling seen so far (Düsseldorf 10000, Chambéry FL095,
   Innsbruck), but it is a guess, not a measurement.
3. Prefer keying on the volume's *class* if the data ever supports it; the
   dimensional test is a proxy for "this is not a TMA".

Related: the generic-name rejection (commit bfdfcc0) treats the same
`AIRSPACE CLASS C` block as unspeakable; both are symptoms of enroute slabs
being indexed as if they were controlled terminal volumes.
