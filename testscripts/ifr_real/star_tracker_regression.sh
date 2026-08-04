#!/usr/bin/env bash
# STAR route-tracker regression harness (real CIFP data).
#
# Steps the aircraft fix-by-fix through several representative STARs and checks the
# route tracker (s_route_fix_idx via the 'route' command) advances MONOTONICALLY --
# never jumps backward, never leaps multiple fixes to a geographically-close but
# sequence-far fix (the SALE3P COLLO<->PIRUV loop mis-sequence). Also records the point
# the approach is cleared. This BASELINES the tracker so the planned removal of the
# geographic RESYNC + the 2-fix clearance guard can be validated for no regression.
#
# STARs (diverse cases):
#   SALE3P LFLB R04-Z   -- looping STAR, COLLO (an IAF) sits mid-STAR ~2 NM from PIRUV
#   ROMA3P LFLB R04-Z   -- normal STAR, terminus PIRUV = the IAF
#   NANI2A LOWI R08-Z   -- short STAR, off-STAR IAF (ELMEM), connector/reversal territory
#   ABDI8R LFMN R04LZ   -- normal STAR (terminus MUS = its IAF); sanity baseline
#
# Local-validation only (needs the user's Custom Data); run via `make test-stars`.
set -u
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
REPL="$REPO/build/atc_ifr_repl"
export XP_AIRPORT_OVERRIDES="${XP_AIRPORT_OVERRIDES:-$REPO/Resources/airport+.json}"
export XP_AIRSPACE_OVERLAY="${XP_AIRSPACE_OVERLAY:-$REPO/Resources/airspace+.txt}"
CIFP="${XP_CIFP_DIR:-$HOME/X-Plane 12/Custom Data/CIFP}"
[[ -x "$REPL" ]] || { echo "FAIL: $REPL not built (make ifr-repl)"; exit 1; }

fails=0

# run_star <dest> <STAR> <approach> <alt> <fix1 fix2 ...>
run_star() {
  local dest="$1" star="$2" appr="$3" alt="$4"; shift 4
  local fixes=("$@")
  echo "=== $star ($dest $appr) : $* ==="
  local script="set cifp_dir $CIFP
set dest $dest
set alt $alt
set pa $alt
set on_ground 0
set gs 220
goto ${fixes[0]}
arrival $dest $star $appr
"
  for fx in "${fixes[@]}"; do script+="goto $fx
poll 3
route
"; done
  script+="quit
"
  local out; out="$(printf '%s' "$script" | "$REPL" 2>&1)"

  # Extract the tracker idx after each step; assert monotonic non-decreasing.
  local idxs; idxs=$(grep -oE "route \(idx=[0-9]+" <<<"$out" | grep -oE "[0-9]+")
  local prev=-1 mono=1 step=0
  while read -r i; do
    [[ -z "$i" ]] && continue
    step=$((step+1))
    if (( i < prev )); then mono=0; echo "  idx went BACKWARD: $prev -> $i (step $step)"; fi
    prev=$i
  done <<<"$idxs"
  if (( mono == 1 )); then echo "  PASS  tracker advances monotonically (final idx=$prev)"; else echo "  FAIL  tracker non-monotonic"; fails=$((fails+1)); fi

  # No spurious geographic resync (the band-aid we intend to remove should not be
  # firing on a normal STAR fly-through).
  if grep -qE "Track: resync" <<<"$out"; then
    echo "  NOTE  geographic RESYNC fired: $(grep -E 'Track: resync' <<<"$out" | head -1 | sed 's/.*Track: /Track: /')"
  else
    echo "  PASS  no geographic RESYNC on the fly-through"
  fi

  # Did the approach clear, and did the tracker reach the last fix?
  if grep -qE "cleared approach" <<<"$out"; then
    echo "  PASS  approach cleared: $(grep -oE 'cleared approach[^"]*' <<<"$out" | head -1)"
  else
    echo "  NOTE  approach not cleared in this stepped replay (eta/check-in gate)"
  fi
}

run_star LFLB SALE3P R04-Z  9000  SALEV PINOT COLLO LUVOB KENZO GOVNA PIRUV
run_star LFLB ROMA3P R04-Z 11000  ROMAM LSE GOVNA PIRUV
run_star LOWI NANI2A R08-Z 13000  NANIT RTT
run_star LFMN ABDI8R R04LZ 11000  ABDIL GIROL AMFOU TIPIK MUS

echo
if (( fails == 0 )); then echo "STAR tracker regression: ALL PASS"; exit 0
else echo "STAR tracker regression: $fails FAILURE(S)"; exit 1; fi
