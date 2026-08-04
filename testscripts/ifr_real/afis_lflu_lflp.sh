#!/usr/bin/env bash
# AFIS departure scenario harness (LFLU Valence -> LFLP Annecy) against the REAL
# X-Plane data (airport+.json overlay, airspace.txt + overlay, atc.dat, CIFP).
#
# Unlike the Catch2 fixtures in tests/ (synthetic context, portable/CI), this drives
# the atc_ifr_repl -- which loads the user's real Custom Data -- and asserts the key
# outputs of the AFIS departure + STAR-shortcut flow. It guards the class of bugs the
# unit tests structurally cannot: real-data resolution + plugin-ish state flow.
#
# Covers (real vol LFLU->LFLP 2026-08):
#   #1  AFIS taxi "taxi to runway X" -> "<field> Information ... no reported traffic"
#   #5  AFIS taxi STAYS in IFR/CLEARED (not TOWER_CONTACT) so the departure flow works
#       ready-for-departure -> IFR/DEPARTURE_CLEARED
#   #4  STAR shortcut to an OFF-route IAF rebuilds the route (direct-to-IAF rebuild)
#
# Not covered here (needs the plugin flight loop / real DataRefs): the airborne safety
# net advance on lift-off, freq_type derivation. Those are validated in-sim.
#
# Env (defaults point at the REPO overlays + real Custom Data):
#   XP_AIRPORT_OVERRIDES  default: <repo>/Resources/airport+.json
#   XP_AIRSPACE_OVERLAY   default: <repo>/Resources/airspace+.txt
#   XP_CIFP_DIR / XP_AIRSPACE / XP_ATCDAT  default: ~/X-Plane 12/Custom Data/...
#   OFP  optional saved SimBrief OFP for the shortcut leg (default: skip if absent)
set -u

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
REPL="$REPO/build/atc_ifr_repl"
export XP_AIRPORT_OVERRIDES="${XP_AIRPORT_OVERRIDES:-$REPO/Resources/airport+.json}"
export XP_AIRSPACE_OVERLAY="${XP_AIRSPACE_OVERLAY:-$REPO/Resources/airspace+.txt}"
OFP="${OFP:-$HOME/Téléchargements/LFLU_LFLP_FL140.json}"

if [[ ! -x "$REPL" ]]; then
  echo "FAIL: $REPL not built (run 'make ifr-repl')"; exit 1
fi

fails=0
# run <script-on-stdin> ; captures combined output into $OUT
run() { OUT="$(printf '%s' "$1" | "$REPL" 2>&1)"; }
# want <label> <regex>  -> assert $OUT matches
want()   { if grep -qiE -e "$2" <<<"$OUT"; then echo "  PASS  $1"; else echo "  FAIL  $1"; echo "        expected /$2/"; fails=$((fails+1)); fi; }
# reject <label> <regex> -> assert $OUT does NOT match
reject() { if grep -qiE -e "$2" <<<"$OUT"; then echo "  FAIL  $1"; echo "        did NOT expect /$2/"; fails=$((fails+1)); else echo "  PASS  $1"; fi; }

echo "=== AFIS ground flow (LFLU) ==="
run 'set airport LFLU
set dest LFLP
set runway 01
set alt 520
set on_ground 1
set gs 0
set freq_type UNKNOWN
set state IFR/CLEARED
say November Romeo Charlie taxi to runway zero one
say November Romeo Charlie holding point runway zero one
say November Romeo Charlie ready for departure runway zero one
quit'
want   "#1 taxi answered by Information"            "Valence Information, runway 01 in use.*no reported traffic"
reject "#5 taxi does NOT go to TOWER_CONTACT"       "-> TOWER_CONTACT"
want   "#5 taxi stays IFR/CLEARED (no bad revert)"  "Valence Information, runway 01 in use"
want   "ready-for-departure answered by Information" "Valence Information, runway 01,.*no reported traffic"
want   "ready-for-departure -> IFR/DEPARTURE_CLEARED" "-> IFR/DEPARTURE_CLEARED"

echo "=== STAR shortcut route rebuild (needs a saved OFP) ==="
if [[ -f "$OFP" ]]; then
  # XP_ATC_SHORTCUT_ALWAYS forces the 20% roll to 100% for the test.
  OUT="$(XP_ATC_SHORTCUT_ALWAYS=1 bash -c "printf '%s' 'load_ofp $OFP
set state IFR/RADAR_CONTACT
set alt 12000
set on_ground 0
set gs 250
set vs 1000
goto ROMAM
poll 5
set state IFR/ARRIVAL
goto LSE
poll 5
route
quit' | '$REPL'" 2>&1)"
  want   "#4 shortcut rebuilds the route to the IAF" "direct-to-IAF rebuild"
  want   "#4 shortcut offers a direct to an IAF"     "direct (TOLNA|COLLO|PIRUV), when able|direct (TOLNA|COLLO|PIRUV)"
  # The direct-to IAF itself MUST be the current route target [*] (it was dropped
  # because approach_procedure_waypoints skips the IF path-term). Guards the TOLNA-
  # missing bug (user 2026-08-04).
  want   "#4 the direct-to IAF is the route target"  "route \(idx=0.*(TOLNA|COLLO|PIRUV)\[\*\]"
else
  echo "  SKIP  #4 shortcut (no OFP at $OFP)"
fi

echo
if [[ $fails -eq 0 ]]; then echo "AFIS scenario: ALL PASS"; exit 0
else echo "AFIS scenario: $fails FAILURE(S)"; exit 1; fi
