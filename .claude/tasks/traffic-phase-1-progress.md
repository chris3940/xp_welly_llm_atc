# Phase 1 — Resume Notes

> Branch: `feature/traffic_injection`
> Spec: `.claude/tasks/traffic-phase-1-foundation.md`
> Build state: clean (new files exist but are NOT wired into CMake yet —
> `make build` still works because nothing references them).

## Done

- [x] Task 1 — `src/data/traffic_context.hpp`
  Struct + enums (`TrafficTarget`, `TrafficContext`, `WakeCategory`,
  `TrafficPhase`). Includes a `set_for_test(TrafficContext)` hook so
  the headless `atc_repl --traffic-fixture` and Catch2 tests can
  inject a fixture without touching XPLM.
- [x] Task 2 — `src/data/traffic_geometry.{hpp,cpp}`
  Pure haversine helpers: `bearing_deg`, `distance_nm`,
  `clock_position`. Clock collapses 0/12 onto `12.0` so the value
  stays in `(0, 12]`.

## In progress

- [ ] Task 3 — `src/data/traffic_context_runtime.cpp` (SDK-coupled)
  Not started. Plan:
  - Cache dataRef handles in `init()`:
    `sim/cockpit2/tcas/targets/position/{x,y,z}` (float[64], indexable),
    `.../flight_id` (byte[8*64] — 8 bytes per slot),
    `.../modeS_id` (int[64]),
    `.../V_msc` (float[64], m/s groundspeed),
    `.../vertical_speed` (float[64], fpm),
    `.../psi` (float[64], track deg),
    optional `.../wake/wake_cat` (int[64]) — graceful skip if null.
    Fallback for AGL: keep simple — `alt_agl_ft = alt_msl_ft - airport_elevation_ft`
    via the new `xplane_context::airport_elevation_ft(icao)` accessor
    (see Task 4); if no nearby airport, leave `alt_agl_ft = 0`.
  - `update()`:
    - Read all arrays once via `XPLMGetDatavf` / `XPLMGetDatavi`.
    - Loop slots `i = 1 .. 63` (skip 0 = user). Stop at `_x > 9999999.0`
      sentinel **or** when modeS_id == 0 for that slot — providers vary.
    - For each valid slot: `XPLMLocalToWorld(x,y,z,...)` → lat/lon/alt_m.
    - Compute distance via `traffic_geometry::distance_nm` against
      `xplane_context::get().latitude/longitude`. Drop > 40 NM.
    - Compute bearing, clock_position (vs `ctx.heading_true`),
      altitude_diff_ft (target_msl - user_msl).
    - Trim callsign: copy 8 bytes from `flight_id[i*8 .. i*8+7]`,
      strip trailing nulls + leading/trailing spaces.
    - Push into a private `TrafficContext` then atomic-swap pointer
      with `current()` (single-writer/single-reader on X-Plane main
      thread — just hold the snapshot in a static and return by ref;
      no thread sync needed because update + current both run on the
      flight loop thread).
    - `last_update_secs = XPLMGetElapsedTime()`.
  - `set_for_test()` overwrites the snapshot directly.
  - Plugin-only TU. The headless `atc_repl` and unit tests use a stub
    that holds the static snapshot + no-op init/stop/update.

## Pending

- [ ] Task 4 — `airport_elevation_ft(icao)` cache in
  `src/core/xplane_context_runtime.cpp` (build_towered_cache).
  apt.dat code-1 line: token #1 (0-indexed) is elevation in feet.
  Add `static std::unordered_map<std::string, float> elevation_cache_`,
  populate alongside `name_cache_`, expose
  `float airport_elevation_ft(const std::string&)` from
  `xplane_context.hpp`. Matching stub in
  `tools/atc_repl/xplane_context_stub.cpp` returning 0.
  **Separate `[refactor]` commit inside the same PR.**
- [ ] Task 5 — `debug_traffic` setting (`settings.hpp/.cpp` getter +
  setter, default in `default_config()`, add `"debug_traffic": false`
  line to `data/settings.json`).
- [ ] Task 6 — Traffic ImGui tab gated on `settings::debug_traffic()`.
  Add `draw_traffic_tab(...)` next to `draw_enroute_tab` and a third
  `BeginTabItem("Traffic")` inside the existing `BeginTabBar
  ("ATC_Tabs")` at `src/ui/atc_ui.cpp:1606`. Columns: Callsign,
  Bearing, Clock, Distance (NM), Alt diff (ft), Groundspeed (kts),
  Phase. Sort by `distance_to_user_nm`, top 10.
- [ ] Task 7 — `tools/atc_repl/main.cpp`: `--traffic-fixture <file>`
  flag. Loads the JSON, builds a `TrafficContext`, calls
  `traffic_context::set_for_test(...)`, prints a deterministic dump
  (sorted by distance, fixed precision). Add stub for `traffic_context`
  in `tools/atc_repl/` (or extend `xplane_context_stub.cpp`).
  Fixture file: `tests/fixtures/traffic_lszh_basic.json` — 3-5 targets
  around LSZH (47.4583, 8.5483) at varied bearings/altitudes.
- [ ] Task 8 — Catch2 tests:
  - `tests/traffic_geometry_test.cpp`:
    - LSZH (47.4583, 8.5483) → LSGG (46.2381, 6.1090): distance ≈ 122
      NM, bearing ≈ 234°. Allow ±1 NM / ±1°.
    - clock_position table: 360/090→3, 180/090→9, 000/000→12, plus
      350/010→1 (wrap), 010/350→11 (wrap), 000/355→12 (12 o'clock
      band high side), 000/005→12 (low side).
  - `tests/traffic_context_test.cpp`: hard to drive the runtime
    reader without XPLM, so test the **filtering rules** by exposing
    a small pure helper from the runtime TU OR by writing a
    free-function `filter_target(...)` in `traffic_context.{hpp,cpp}`
    (SDK-free) that the runtime calls per slot. Easier: extract the
    callsign-trim into a free function in `traffic_context.cpp`
    (engine OBJECT lib), then test it directly. Sentinel/40-NM/index-0
    can be tested end-to-end via the fixture loader (Task 7) returning
    a deterministic snapshot.
- [ ] Task 9 — wire CMake + main.cpp lifecycle:
  - `CMakeLists.txt`: add SDK-free sources to `xp_atc_engine` OBJECT
    lib (`src/data/traffic_geometry.cpp`, plus any SDK-free
    `traffic_context.cpp` if introduced for filter helpers + the
    fixture/test stub). Add `src/data/traffic_context_runtime.cpp` to
    the plugin module only.
  - `tools/atc_repl/`: provide a `traffic_context_stub.cpp` with
    init/stop/update no-ops + a settable `static TrafficContext`.
  - `tests/CMakeLists.txt`: add `traffic_geometry_test.cpp`,
    `traffic_context_test.cpp`, plus the same stub if needed.
  - `src/main.cpp`: include `data/traffic_context.hpp`, call
    `traffic_context::init()` after `xplane_context::init()`,
    `traffic_context::stop()` before `xplane_context::stop()`. In
    `flight_loop_cb`, throttle `traffic_context::update()` with
    `% 30` (≈ 2 Hz at 60 FPS).
  - Run `make all` clean.

## Build invariants to keep

- Engine OBJECT lib stays SDK-free. Anything pulling `<XPLM*.h>` MUST
  live in the plugin module.
- ASCII only in any string fed to `XPLMDebugString` or ImGui.
- No new external deps — only nlohmann/json + ImGui + Catch2 +
  CommonCrypto, all already vendored.
- `update()` runs on X-Plane main thread, never blocks.

## Open questions to revisit before finishing PR

- Wake-cat dataRef availability across providers (LiveTraffic / X-IvAp
  / native AI). Confirm during smoke test, default to
  `WakeCategory::Unknown` if not populated.
- Elevation accessor shape: standalone `airport_elevation_ft(icao)`
  vs. `NearbyAirport.elevation_ft`. Spec leaves it open; default to
  the standalone getter.
