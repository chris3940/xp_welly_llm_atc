#!/usr/bin/env bash
# A climb is stepped, exactly like a descent.
#
# Flight of 2026-08-28, LFMN -> LOWI at FL450: the aircraft checked in with Milan
# at FL140 and was answered "radar contact, climb flight level 450" -- 31 000 ft
# in a single clearance. The descent had been stepped for months
# (kMaxSingleDescentFt); the climb had no equivalent. It does now:
# kMaxSingleClimbFt, and poll_climb_next_step owes the rest rung by rung.
# [C. P. Potter]
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

# $1 = cruise altitude in feet, $2 = distance to fly
scenario() {
  cat <<SCRIPT
set cifp_dir $CIFP
airport_pos LFMN 43.665 7.215
set airport LFMN
set ifr_departure LFMN
set runway 04R
set dest EGLL
set airport_lat 43.665
set airport_lon 7.215
set lat 43.66
set lon 7.21
set alt 5000
set pa 5000
set gs 300
set heading 350
set vs 2500
set cruise $1
set on_ground 0
set agl 4900
set com 120.160
set freq_type APPROACH
set navlog_clear 1
set navlog_fix BASIP 45.50 6.20 8000 CLB 1
set navlog_fix MERLU 49.50 3.00 $1 CRZ 0
set ifr_sid BASI8A
set ifr_sid_last_fix MERLU
set state IFR_RADAR_CONTACT
fly $2
SCRIPT
}

echo "=== IFR climb ladder ==="

echo "--- FL450 is not handed out in one clearance ---"
run "$(scenario 45000 250)"
reject "cruise is never the first climb clearance" "ATC \[sid\]:.*flight level 450"
want   "the first cruise clearance is a rung"        "climb flight level 260"
want   "then the next"                               "climb flight level 360"
want   "and cruise is reached in the end"            "climb ladder: FL450 .*cruise"

echo "--- the rungs are whole flight levels, in order ---"
want   "step1"  "climb flight level 110"
want   "step2"  "climb flight level 140"

echo "--- a modest cruise is still one clearance ---"
run "$(scenario 20000 120)"
want   "FL200 straight from FL140"       "climb flight level 200"
reject "nothing was stepped"             "climb ladder:"

echo
if [[ $FAILED -eq 0 ]]; then echo "climb ladder: PASS"; else echo "climb ladder: FAIL"; fi
exit $FAILED
