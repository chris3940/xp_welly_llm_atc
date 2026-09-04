#!/usr/bin/env bash
# Lined up and waiting, and nobody says anything.
#
# "Runway 04R, line up and wait" leaves the ball with the pilot: the takeoff
# clearance only comes once he reports ready. A pilot who lines up and stays
# silent sat on the runway for ever while the frequency stayed quiet (user
# 2026-08-29). A controller asks.
#
# PHRASEOLOGY: "report when ready for DEPARTURE", never "ready for take-off".
# ICAO Doc 4444 reserves TAKE-OFF for issuing or cancelling the clearance itself
# -- the rule written after Tenerife -- and requires DEPARTURE or AIRBORNE
# everywhere else. [C. P. Potter]
set -u
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
REPL="$REPO/build/atc_ifr_repl"
CIFP="$HOME/X-Plane 12/Custom Data/CIFP"
if [[ ! -x "$REPL" ]]; then echo "FAIL: $REPL not built"; exit 1; fi

FAILED=0
run() { OUT="$(printf '%s' "$1" | "$REPL" 2>&1)"; }
want()   { if grep -qiE -e "$2" <<<"$OUT"; then echo "  PASS  $1";
           else echo "  FAIL  $1"; echo "        expected /$2/"; FAILED=1; fi; }
reject() { if grep -qiE -e "$2" <<<"$OUT"; then echo "  FAIL  $1";
           echo "        did NOT expect /$2/"; FAILED=1; else echo "  PASS  $1"; fi; }
count()  { local n; n=$(grep -cE "$3" <<<"$OUT"); if [[ "$n" == "$2" ]]; then
             echo "  PASS  $1 ($n)"; else echo "  FAIL  $1 (got $n, want $2)"; FAILED=1; fi; }

# $1 = groundspeed, $2.. = poll steps
scenario() {
  local gs="$1"; shift
  cat <<SCRIPT
set cifp_dir $CIFP
set airport LFMN
set runway 04R
set on_ground 1
set gs $gs
set com 118.700
set freq_type TOWER
set state IFR_LINE_UP_AND_WAIT
$(printf '%s\n' "$@")
SCRIPT
}

echo "=== lined up, pilot silent ==="

echo "--- nothing before 45 s ---"
run "$(scenario 0 "poll 20" "poll 20")"
reject "the tower does not nag a pilot who has just lined up" "ready for departure"

echo "--- the tower asks, once past 45 s ---"
run "$(scenario 0 "poll 20" "poll 20" "poll 20")"
want   "report when ready for departure" "report when ready for departure"
reject "and never says the word take-off in the question" "ready for (take.?off|takeoff)"

echo "--- two prompts maximum, then silence ---"
run "$(scenario 0 "poll 30" "poll 30" "poll 30" "poll 30" "poll 30" "poll 30" "poll 30" "poll 30" "poll 30" "poll 30")"
count  "prompts capped" 2 "report when ready for departure"

echo "--- an aircraft already rolling is not asked ---"
run "$(scenario 40 "poll 30" "poll 30" "poll 30")"
reject "no prompt during the take-off run" "ready for departure"

echo
if [[ $FAILED -eq 0 ]]; then echo "lineup prompt: PASS"; else echo "lineup prompt: FAIL"; fi
exit $FAILED
