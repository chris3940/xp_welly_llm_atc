#!/usr/bin/env bash
# Runway-crossing regression, on the REAL LFMN geometry.
#
# The crossing decision used to measure the range to a runway THRESHOLD, so it
# could only fire at a hold-short within 250 m of a runway end. Most crossings
# are nothing like that: at LFML the taxi to 31R stops at Foxtrot 7, partway
# ALONG 31L and kilometres from either end, and the aircraft crossed an active
# runway with no clearance at all (real flight 2026-08-19). The test is now the
# perpendicular distance to the CENTRELINE SEGMENT.
#
# LFML has no apt.dat in this installation, so the cases below are built on
# LFMN's real runways (Custom Scenery pack) -- the same geometry class, and the
# original motivating case: depart 04R, cross 04L.
#
#   04R  43.64673731  007.20249753  <->  22L  43.66561481  007.22846925
#   04L  43.65180616  007.20403478  <->  22R  43.66855734  007.22708757
#
# The mid-runway hold-short is 1326 m from the 04L threshold -- far outside the
# old 250 m test, which is exactly why LFML went unnoticed. [C. P. Potter]
set -u
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
REPL="$REPO/build/atc_ifr_repl"
if [[ ! -x "$REPL" ]]; then
  echo "FAIL: $REPL not built (run 'make ifr-repl')"; exit 1
fi

run() { OUT="$(printf '%s' "$1" | "$REPL" 2>&1)"; }
FAILED=0
want()   { if grep -qiE -e "$2" <<<"$OUT"; then echo "  PASS  $1";
           else echo "  FAIL  $1"; echo "        expected /$2/"; FAILED=1; fi; }
reject() { if grep -qiE -e "$2" <<<"$OUT"; then echo "  FAIL  $1";
           echo "        did NOT expect /$2/"; FAILED=1; else echo "  PASS  $1"; fi; }

# lat lon heading -> the REPL script that puts the aircraft there, pre-departure
# on Tower at LFMN with both runways known, and makes it call.
setup() {
  cat <<SCRIPT
clear_runways
add_runway 04R 43.64673731 7.20249753 22L 43.66561481 7.22846925
add_runway 04L 43.65180616 7.20403478 22R 43.66855734 7.22708757
set airport LFMN
set runway 04R
set on_ground 1
set gs 0
set heading $3
set lat $1
set lon $2
set com 118.700
set freq_type TOWER
set state TOWER_CONTACT
say November Romeo Charlie holding point alpha one runway zero four right
poll 3
SCRIPT
}

echo "=== Runway crossing (real LFMN geometry) ==="

echo "--- mid-runway hold-short, 180 m from the 04L axis, transverse (the LFML case) ---"
run "$(setup 43.659039 7.217148 135)"
want "a crossing is offered away from the threshold" "cross runway 04L"

echo "--- same point, heading ALONG the runway: alongside, not crossing ---"
run "$(setup 43.659039 7.217148 45)"
reject "no crossing when parallel to the runway" "cross runway"

echo "--- near the 04L threshold: the case that already worked ---"
run "$(setup 43.65280 7.20520 135)"
want "the threshold case still fires" "cross runway 04L"

echo
if (( FAILED )); then echo "Runway crossing: FAILURES"; exit 1; fi
echo "Runway crossing: ALL PASS"
