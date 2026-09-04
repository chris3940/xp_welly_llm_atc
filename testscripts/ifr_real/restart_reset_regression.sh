#!/usr/bin/env bash
# Flight-restart / teleport detection.
#
# X-Plane can move the aircraft anywhere without reloading the plugin -- "jump
# to" from the map, reloading a situation, restarting the flight -- and until
# 2026-08-21 nothing in the engine noticed. On the session of 2026-08-20 the
# pilot restarted the same LFML -> LSGG arrival three times inside one X-Plane
# run: the step-down into the Geneva TMA had already fired in attempt one, so its
# latch read "done" and "descend flight level 90" was never issued again. The
# aircraft reached the vectoring point 8000 ft above its FAF and the sequencing
# leg then ran 25 NM purely to lose it. Geneva Approach also spoke while the
# aircraft was back near Marseille, two hundred miles away.
#
# Two things have to hold, and the second is what makes the first safe to ship:
#   * a real discontinuity resets the IFR state;
#   * a legitimate `track` step does NOT -- the headless harness advances the
#     aircraft a whole leg at a time with a dt to match, and a naive distance
#     threshold would have declared every replay a teleport.
# [C. P. Potter]
set -u
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
REPL="$REPO/build/atc_ifr_repl"
if [[ ! -x "$REPL" ]]; then
  echo "FAIL: $REPL not built (run 'make ifr-repl')"; exit 1
fi

run() { OUT="$(printf "%s" "$1" | ATC_DETECT_RESTART=1 "$REPL" 2>&1)"; }
FAILED=0
want()   { if grep -qiE -e "$2" <<<"$OUT"; then echo "  PASS  $1";
           else echo "  FAIL  $1"; echo "        expected /$2/"; FAILED=1; fi; }
reject() { if grep -qiE -e "$2" <<<"$OUT"; then echo "  FAIL  $1";
           echo "        did NOT expect /$2/"; FAILED=1; else echo "  PASS  $1"; fi; }

echo "=== Flight restart / teleport ==="

# A cruise leg flown honestly: ~52 NM at 450 kt is 415 s, and that is the dt.
echo "--- a legitimate track step is not a teleport ---"
run "$(cat <<'SCRIPT'
set gs 450
set alt 24000
track 45.0000 5.0000 24000 60
track 45.8660 5.0000 24000 415
poll 3
SCRIPT
)"
reject "52 NM in 415 s at 450 kt reads as flying" "position discontinuity"

# The same 52 NM in five seconds cannot be flown by anything.
echo "--- the same distance in five seconds is ---"
run "$(cat <<'SCRIPT'
set gs 450
set alt 24000
track 45.0000 5.0000 24000 60
track 45.8660 5.0000 24000 5
poll 3
SCRIPT
)"
want "52 NM in 5 s is a restart"      "position discontinuity"
want "and it resets the ATC state"    "resetting the ATC state"

# A GAP IN THE DETECTOR'S OWN SAMPLING is not a teleport.
#
# note_frame used to sit behind the "PTT is idle" gate, so it stopped sampling
# whenever the session was busy. On 2026-08-29 a stuck transmission held the
# session 150 s; the first call after the watchdog released it compared two
# positions 23 NM apart with dt = one frame, called a normal cruise leg a
# teleport, and wiped the approach, the STAR, the destination and the runway
# lock. The pilot got his radio back to an ATC that no longer knew he existed.
# "warp" advances the sim clock with the polls asleep, which is exactly that gap.
echo "--- a gap in the sampling is not a teleport ---"
run "$(cat <<'SCRIPT'
set gs 277
set alt 15000
track 47.4783 12.1560 15000 60
warp 150
set lat 47.4080
set lon 11.5867
poll 1
SCRIPT
)"
reject "23 NM over an unsampled 150 s reads as flying" "position discontinuity"

# ... and the same 23 NM with no gap at all still is one.
echo "--- but a real jump inside one frame still is ---"
run "$(cat <<'SCRIPT'
set gs 277
set alt 15000
track 47.4783 12.1560 15000 60
set lat 45.0000
set lon 9.0000
poll 3
SCRIPT
)"
want "a 170 NM jump in 3 s is a restart" "position discontinuity"

# The case that actually happened: Geneva area back to the Marseille area.
echo "--- the flown case: LSGG arrival restarted near Marseille ---"
run "$(cat <<'SCRIPT'
set gs 280
set alt 15000
track 46.3125 6.4234 15000 60
track 44.4502 4.7794 18000 60
poll 3
SCRIPT
)"
want "a 130 NM jump backwards is a restart" "position discontinuity"

# And the consequence that matters: the aircraft must not still be "on the
# approach" after being put back at cruise. reset() wipes the engine but not the
# state machine, so the restart puts the flow back where a fresh plugin load
# starts.
echo "--- an approach state does not survive the restart ---"
run "$(cat <<'SCRIPT'
set state IFR_APPROACH_DESCENT
set dest LSGG
set gs 280
track 46.3125 6.4234 15000 60
track 44.4502 4.7794 18000 60
poll 3
state
SCRIPT
)"
want "the state falls back to IDLE" "ATC state: IDLE"
reject "and is no longer on the approach" "^ATC state: IFR/APPROACH"

# The plugin's OWN jump buttons are the other way a flight teleports, and the
# en-route one used to leave the route tracker wherever the previous flight had
# pushed it -- often at the end of the route. The tracker only moves forward, so
# it could never recover, and every routed distance built on it collapsed: that
# is what made the distance and the time to top of descent meaningless on the IFR
# tab after a JUMP (user, 2026-08-21). training_jump_approach always cleared it;
# the en-route jump did not.
#
# Clearing turned out not to be enough: the rebuild happened on a path that does
# NOT re-align the tracker, so the chain restarted at the DEPARTURE fix and the
# routed distance read 184 NM for a real 130 (Log 31). The jump now calls
# init_route_fixes(), whose step 3 skips every fix already behind the aircraft.
echo "--- the en-route jump clears the route tracker ---"
run "$(cat <<'SCRIPT'
set gs 450
set alt 28000
jump enroute 28000
tod
SCRIPT
)"
want "the tracker is rebuilt and re-aligned" "route rebuilt and re-aligned on the aircraft"
want "and the TOD estimate is not a stale number" "TOD: n/a"

echo
if [[ $FAILED -eq 0 ]]; then echo "restart reset: PASS"; else echo "restart reset: FAIL"; fi
exit $FAILED
