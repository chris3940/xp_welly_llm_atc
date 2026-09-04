# Experimental IFR departure scenarios (quarantined)

These three IFR-departure scenarios are **excluded from the default scenario
suite** (`make test` / `make test-scenarios`) because they currently fail
against the merged IFR feature code (PR #11, `chris3940/feat/ifr-dev`). They
were green when first added (commits `ecbe81f`, `a8a495a`) but regressed later
in the same branch.

They are kept here, not deleted, so the work is preserved and the fixes can be
verified later. Run them explicitly with:

```bash
make test-scenarios-ifr
```

## Known root causes (open — to be fixed by the IFR feature author)

- ~~`ifr_lflp_departure_sim.json` — the Tower should issue the IFR clearance
  directly when there is no active ATIS; the redirect guard sends it to Ground
  instead.~~ **WRONG PREMISE, and fixed 2026-08-10. This scenario PASSES (20/20).**
  Annecy HAS a Ground frequency -- `1053 121730 ANNECY GND` in the local
  apt.dat, a recent real-world addition (user). With no Delivery and no ATIS the
  IFR clearance belongs to GROUND, so the redirect guard in
  `ground_operations.cpp::check_freq_precondition()` was right all along and the
  scenario, which opened on Tower, was wrong. It now opens on Ground.
- `ifr_lszh_departure_eu.json`, `ifr_lszh_departure_sim.json` — ATIS-active
  path; separate root cause(s) in the Delivery-based clearance flow, not yet
  isolated. **Still failing (10/22 and 10/24), unchanged since `1cba128`.**
- `ifr_lfmn_arrival_nostar.json` — one assertion of six. Not in the original
  quarantine list; verify whether it ever passed.

## Status, measured 2026-08-22

`make test-scenarios-ifr` -> 7 scenarios, **4 pass, 3 fail**. The three failures
are identical in count to the same run on commit `1cba128`, so nothing in the
vectoring / restart / CIFP-floor work of 2026-08-20..22 touched them.

Read this list against the code before trusting it: two of its four original
claims were already stale when checked.

## Scope note

IFR is an **EU-only** feature. A hard gate in
`atc_state_machine::process()` strips IFR-only intents
(`REQUEST_IFR_CLEARANCE`, `REPORT_HOLDING_SHORT`) in non-EU profiles, so these
scenarios and the IFR flow have no effect in US/DE.
