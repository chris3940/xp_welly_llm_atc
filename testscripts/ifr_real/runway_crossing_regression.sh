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

# 04L lies between the west-side taxiways and 04R. A hold-short on the FAR side
# has to cross it; one on the SAME side as 04R does not. That is the whole test,
# and it is geometry -- the aircraft's heading is not consulted: at LFML the
# hold-short F7 faced 086 against a ~133 axis, 46 degrees off the perpendicular,
# and a heading window would have rejected a crossing squarely in the way.

echo "--- FAR side, mid-runway, 180 m from the 04L axis (the LFML case) ---"
run "$(setup 43.661325 7.213975 135)"
want "a crossing is offered 1.3 km from any threshold" "cross runway 04L"

echo "--- SAME side as 04R: the runway is not in the way ---"
run "$(setup 43.659039 7.217148 135)"
reject "no crossing when the runway is not between us and the departure" "cross runway"

echo "--- FAR side, 400 m downfield of the 04L threshold ---"
run "$(setup 43.655373 7.206135 135)"
want "a hold-short just downfield of the threshold still fires" "cross runway 04L"

echo "--- heading ALONG the runway, far side: geometry still says cross ---"
run "$(setup 43.661325 7.213975 45)"
want "a curved hold-short is not rejected on its heading" "cross runway 04L"

# The Ground side: the taxi clearance must STOP at the runway, and the reply to a
# hold-short report must name it. A bare "roger, contact Tower" reads as
# permission to carry on -- which is how 13R/31L was crossed at Marseille.
ground() {
  cat <<SCRIPT
clear_runways
add_runway 04R 43.64673731 7.20249753 22L 43.66561481 7.22846925
add_runway 04L 43.65180616 7.20403478 22R 43.66855734 7.22708757
set airport LFMN
set runway 04R
set on_ground 1
set gs 0
set heading 156
set lat $1
set lon $2
set com 121.700
set freq_type GROUND
set state $3
say $4
poll 3
SCRIPT
}

# The TAXI CLEARANCE needs the whole pre-departure sequence: a taxi request
# before the IFR clearance is refused outright, which is what made this look like
# a code gap the first time round. The hold-short REPORT cases below do NOT use
# it -- the clearance assigns a squawk, and the squawk check then answers before
# the holding-short template ever runs.
ground_full() {
  cat <<SCRIPT
clear_runways
add_runway 04R 43.64673731 7.20249753 22L 43.66561481 7.22846925
add_runway 04L 43.65180616 7.20403478 22R 43.66855734 7.22708757
set airport LFMN
set runway 04R
set on_ground 1
set gs 0
set heading 156
set lat $1
set lon $2
set com 121.700
set freq_type GROUND
set dest LSGG
say November Romeo Charlie request IFR clearance to Geneva
say November Romeo Charlie cleared to Geneva squawk two zero one two
say November Romeo Charlie ready to taxi
poll 3
SCRIPT
}

# The taxi clearance must STOP at the runway. This needs the full pre-departure
# sequence: a taxi request before the IFR clearance is refused outright, which is
# what made this look like a code gap the first time round.
# The clearance now stops at the runway by NAMING ITS HOLDING POINT rather than
# by naming the departure runway's and adding a reminder. Cleared to the holding
# point of 04L, the aircraft holds there by definition -- and, crucially, no
# point on the far side is named. Naming one read as permission to cross (user,
# real flight LFMN 2026-08-26).
echo "--- Ground: the taxi clearance stops at the runway ---"
run "$(ground_full 43.661325 7.213975)"
# Scoped to the taxi line: "runway 04R" appears legitimately elsewhere in the
# session (the IFR clearance names the departure runway).
TAXI="$(grep -i 'taxi to' <<<"$OUT" | head -1)"
if grep -qi "runway 04L" <<<"$TAXI"; then
  echo "  PASS  the taxi clearance is limited to the runway in the way"
else
  echo "  FAIL  the taxi clearance is limited to the runway in the way"
  echo "        got: $TAXI"; FAILED=1
fi
# And no holding-point NAME either: the table keeps one point per runway, chosen
# by proximity to the threshold, which is not the one the aircraft taxis to --
# "holding point Alpha Two" for an aircraft arriving at Alpha One (2026-08-27).
if grep -qiE "holding point (alpha|bravo|charlie|delta|echo|whiskey|golf|kilo|november|papa|sierra|tango|victor|zulu)" <<<"$TAXI"; then
  echo "  FAIL  and names no holding point it cannot determine"
  echo "        got: $TAXI"; FAILED=1
else
  echo "  PASS  and names no holding point it cannot determine"
fi
if grep -qi "04R" <<<"$TAXI"; then
  echo "  FAIL  and never names a holding point beyond it"
  echo "        got: $TAXI"; FAILED=1
else
  echo "  PASS  and never names a holding point beyond it"
fi

# The Tower must clear the crossing before it can order a line-up on the far
# runway. At Nice the aircraft reported "holding point Alpha One, runway 04L" and
# was answered "runway 04R, line up and wait" -- impossible to comply with
# without crossing 04L (real flight 2026-08-26).
echo "--- Tower: the hold-short report earns a crossing clearance ---"
run "$(cat <<SCRIPT
clear_runways
add_runway 04R 43.64673731 7.20249753 22L 43.66561481 7.22846925
add_runway 04L 43.65180616 7.20403478 22R 43.66855734 7.22708757
set airport LFMN
set on_ground 1
set gs 0
set runway 04R
set lat 43.6676
set lon 7.2214
set heading 315
set com 121.700
set freq_type GROUND
set state IFR_CLEARED
say November Romeo Charlie ready to taxi
set lat 43.6527
set lon 7.2032
set heading 156
set com 118.700
set freq_type TOWER
set state TOWER_CONTACT
say November Romeo Charlie holding point alpha one
SCRIPT
)"
want   "the crossing is cleared"                 "cross runway 04L"
reject "and no line-up is given across a runway" "line up and wait"

# The clearance asks for a report, so the flow must be able to receive it -- even
# once the aircraft has rolled and the geometry no longer finds the runway. It
# could not: readback and vacated report both drew "say again your request",
# three times, with the aircraft stopped on a runway it had been cleared across
# (real flight LFMN 2026-08-27).
echo "--- Tower: the crossing readback and the vacated report are received ---"
run "$(cat <<SCRIPT
clear_runways
add_runway 04R 43.64673731 7.20249753 22L 43.66561481 7.22846925
add_runway 04L 43.65180616 7.20403478 22R 43.66855734 7.22708757
set airport LFMN
set on_ground 1
set gs 0
set runway 04R
set lat 43.6676
set lon 7.2214
set heading 315
set com 121.700
set freq_type GROUND
set state IFR_CLEARED
say November Romeo Charlie ready to taxi
set lat 43.6527
set lon 7.2032
set heading 156
set com 118.700
set freq_type TOWER
set state TOWER_CONTACT
say November Romeo Charlie holding point alpha one
set lat 43.6516
set lon 7.2047
set heading 136
set gs 8
say November Romeo Charlie cross runway zero four left report vacated
say November Romeo Charlie runway zero four left vacated
SCRIPT
)"
reject "no say-again on the crossing exchange" "say again your request"
want   "the vacated report sends him to the departure holding point" \
       "continue to holding point.*runway 04R"

echo "--- Ground: the hold-short report is answered with the runway named ---"
run "$(ground 43.661325 7.213975 TAXI_CLEARED "November Romeo Charlie holding point alpha one")"
want "the reply repeats the hold-short" "hold short of runway 04L"

echo "--- same, on the near side: nothing to hold short of ---"
run "$(ground 43.659039 7.217148 TAXI_CLEARED "November Romeo Charlie holding point alpha one")"
reject "no spurious hold-short when no runway is in the way" "hold short of runway"

echo
if (( FAILED )); then echo "Runway crossing: FAILURES"; exit 1; fi
echo "Runway crossing: ALL PASS"
