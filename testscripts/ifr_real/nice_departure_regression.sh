#!/usr/bin/env bash
# Nice IFR departure: the two defects of the flight of 2026-08-26.
#
#   1. every ground transmission said "Nice/Cote" -- apt.dat writes
#      "Nice/Cote D'Azur" and the ground flow cut the name on space-or-hyphen,
#      never on the slash. Reported ten times by the user before it was found.
#   2. the clearance announced "initial climb to 5000 feet" for BASI8A, which
#      publishes FL100 for jets and FL070 for props in airport+.json. The rule
#      was loaded and the reader worked; the clearance interpolates
#      {ifr_initial_altitude}, and the override had been wired into a DIFFERENT
#      variable used in flight. Nothing tested either one. [C. P. Potter]
set -u
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
REPL="$REPO/build/atc_ifr_repl"
CIFP="$HOME/X-Plane 12/Custom Data/CIFP"
if [[ ! -x "$REPL" ]]; then echo "FAIL: $REPL not built"; exit 1; fi

run() { OUT="$(printf '%s' "$1" | "$REPL" 2>&1)"; }
FAILED=0
want()   { if grep -qiE -e "$2" <<<"$OUT"; then echo "  PASS  $1";
           else echo "  FAIL  $1"; echo "        expected /$2/"; FAILED=1; fi; }
reject() { if grep -qiE -e "$2" <<<"$OUT"; then echo "  FAIL  $1";
           echo "        did NOT expect /$2/"; FAILED=1; else echo "  PASS  $1"; fi; }

setup() {
  cat <<SCRIPT
set cifp_dir $CIFP
set airport LFMN
set runway 22L
set on_ground 1
set gs 0
set com 121.700
set freq_type DELIVERY
set dest LOWI
set ifr_sid $1
set state IFR_PREDEP_CLEARANCE
say Nice Delivery November Romeo Charlie request IFR clearance information Alpha on board
SCRIPT
}

echo "=== Nice IFR departure ==="

echo "--- the published SID initial climb reaches the clearance ---"
run "$(setup BASI8A)"
want   "BASI8A resolves its published initial climb" "initial climb -> 7000 ft"
want   "and it is spoken as a FLIGHT LEVEL, three digits" '-> "FL070"' 
reject "and never falls back to the transition altitude" "initial climb -> 5000 ft"

echo "--- a SID with no rule keeps the CIFP value ---"
run "$(setup ZZZZ9Z)"
reject "no override altitude for an unknown SID" "initial climb ->"

echo "--- the aerodrome is 'Nice', never 'Nice/Cote' ---"
run "$(setup BASI8A)"
reject "no slash suffix in any transmission" "Nice/C"

echo
if [[ $FAILED -eq 0 ]]; then echo "Nice departure: PASS"; else echo "Nice departure: FAIL"; fi
exit $FAILED
