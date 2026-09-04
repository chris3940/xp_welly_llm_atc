#!/usr/bin/env bash
# The defect of the flight of 2026-08-28 (LFMN -> LOWI, package 129/130).
#
# Airborne, ctx.nearest_airport_id drifts to whatever field is underneath:
# LFMN -> LNMC (Monaco) -> LIMG (Albenga) -> LIMJ (Genoa). Every departure-side
# CIFP lookup keyed on it, so ATC resolved a SID for an airport the aircraft was
# merely overflying and cleared the pilot "direct ITCAP" -- the exit fix of
# Albenga's ITCA1B, a point on no part of the flight plan. The same drift also
# made the SID climb fall back to FL110 instead of the published FL100, because
# it computed the ladder for LNMC.
#
# Two independent defences, one test each:
#   1. the departure field is LATCHED and the SID climb keys on the latch;
#   2. a direct-to may only name a fix that is on the filed route.
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

# $1 = the field "airport" has drifted to, $2 = latched departure (may be empty),
# $3 = the SID exit fix ATC would offer a direct to.
scenario() {
  cat <<SCRIPT
set cifp_dir $CIFP
airport_pos LFMN 43.665 7.215
set airport $1
${2:+set ifr_departure $2}
set runway 04R
set dest EGLL
set airport_lat 43.665
set airport_lon 7.215
set lat 43.66
set lon 7.21
set alt 5000
set pa 5000
set gs 250
set heading 330
set vs 2000
set cruise 45000
set on_ground 0
set agl 4900
set com 120.160
set freq_type APPROACH
set navlog_clear 1
set navlog_fix BASIP 43.90 7.05 8000 CLB 1
set navlog_fix MERLU 44.50 6.60 45000 CLB 0
set navlog_fix ROTOS 45.20 6.10 45000 CRZ 0
set ifr_sid BASI8A
set ifr_sid_last_fix $3
set state IFR_RADAR_CONTACT
fly 20
SCRIPT
}

echo "=== SID departure field ==="

echo "--- the climb keys on the LATCHED departure, not the field underneath ---"
run "$(scenario LIMG LFMN MERLU)"
want   "SID ladder computed for the departure field"  "dep=LFMN"
reject "never for the field being overflown"          "dep=LIMG"

echo "--- with no latch it can only fall back to the drifted field ---"
run "$(scenario LIMG '' MERLU)"
want   "fallback is the active airport"               "dep=LIMG"

echo "--- a direct-to must name a fix that is on the filed route ---"
run "$(scenario LFMN LFMN ITCAP)"
reject "no direct to a foreign SID's exit fix"        "direct ITCAP"
want   "and the refusal is stated in the log"         "direct-to REFUSED -- ITCAP"

echo "--- an on-route exit fix is not refused ---"
run "$(scenario LFMN LFMN MERLU)"
reject "MERLU is on the route, nothing refused"       "direct-to REFUSED"

echo
if [[ $FAILED -eq 0 ]]; then echo "SID departure field: PASS"; else echo "SID departure field: FAIL"; fi
exit $FAILED
