#!/usr/bin/env python3
"""
Closed-loop REPL pilot for atc_ifr_repl.

Every other harness in this directory sends a FIXED command list, which means the
scripted pilot cannot obey ATC: it descends before it is cleared, or stays level
after it is, and it never checks in on a new frequency. Both distort the result --
the check-in gate keeps ATC deliberately silent until the pilot calls, so a script
that never switches frequency produces a long false "ATC said nothing" that looks
exactly like a bug (measured on the DIK -> EDLW replay, 2026-08-15).

This driver closes the loop. It reads what ATC actually says and reacts the way a
pilot would:

  "contact <who> on <freq>"      -> tune the frequency, then check in on it
  "descend/climb <level>"        -> fly to it at a realistic rate, not instantly
  "maintain <level>"             -> hold it

Everything else is logged, not acted on. The aircraft is flown along the route
leg by leg, so the route tracker advances for real rather than being teleported
past its fixes.

Usage:
    fly.py <route.json>

The route file:
{
  "dest": "EDLW", "runway": "06", "cruise_ft": 27000, "start_ft": 27000,
  "com": "118.750", "gs_kt": 280,
  "navlog": [["DIK", 49.8613, 6.1298, 27000, "CRZ", 0], ...],
  "path":   [[49.8613, 6.1298], ...]        # flown points, interpolated between
}

Environment passes straight through, so XP_ATC_FORCE_ILS=1 etc. still work.
[C. P. Potter]
"""

import json
import math
import os
import re
import select
import subprocess
import sys

REPL = os.environ.get("ATC_REPL", "./build/atc_ifr_repl")
# Set ATC_RAW=<file> to keep the REPL's full output. The driver only PRINTS the
# ATC timeline, so without this the engine's own diagnostics -- the ones that say
# WHY a clearance was withheld -- are read and discarded.
RAW = os.environ.get("ATC_RAW", "")
_raw_fh = open(RAW, "w") if RAW else None

# Descent/climb rate. 1800 fpm at 280 kt groundspeed is ~385 ft per NM of track;
# derived per leg from the actual groundspeed so a slower aircraft descends less
# steeply rather than teleporting onto its cleared level.
FPM = 1800.0


def nm(a, b):
    dlat = (b[0] - a[0]) * 60.0
    dlon = (b[1] - a[1]) * 60.0 * math.cos(math.radians((a[0] + b[0]) / 2.0))
    return math.hypot(dlat, dlon)


def bearing(a, b):
    """True course a -> b, degrees. Without it every leg keeps the previous
    heading and the course monitor fires a spurious "confirm route" on each
    turn -- harness noise that looks exactly like a real off-route detection."""
    la1, lo1, la2, lo2 = map(math.radians, (a[0], a[1], b[0], b[1]))
    dlon = lo2 - lo1
    y = math.sin(dlon) * math.cos(la2)
    x = math.cos(la1) * math.sin(la2) - math.sin(la1) * math.cos(la2) * math.cos(dlon)
    return (math.degrees(math.atan2(y, x)) + 360.0) % 360.0


def interpolate(points, step_nm):
    """Dense flown path: every leg cut into <= step_nm pieces."""
    out = [points[0]]
    for i in range(len(points) - 1):
        a, b = points[i], points[i + 1]
        d = nm(a, b)
        n = max(1, int(math.ceil(d / step_nm)))
        for k in range(1, n + 1):
            out.append((a[0] + (b[0] - a[0]) * k / n, a[1] + (b[1] - a[1]) * k / n))
    return out


class Repl:
    def __init__(self):
        self.p = subprocess.Popen(
            [REPL],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
        )

    def send(self, cmd):
        self.p.stdin.write(cmd + "\n")
        self.p.stdin.flush()

    def drain(self, quiet_s=0.35, cap_s=6.0):
        """Read until the REPL has been silent for quiet_s. Only safe for the
        initial banner -- use sync() to bound a command, since the REPL echoes
        slowly enough that a plain quiet-period read silently loses output."""
        chunks, waited = [], 0.0
        while waited < cap_s:
            r, _, _ = select.select([self.p.stdout], [], [], quiet_s)
            if not r:
                break
            line = self.p.stdout.readline()
            if not line:
                break
            chunks.append(line)
            if _raw_fh:
                _raw_fh.write(line)
            waited = 0.0
        return chunks

    def sync(self, timeout_s=20.0):
        """Send a sentinel and read until its reply appears, so every line the
        previous command produced has certainly been collected. Timing-based
        reads race the REPL and drop ATC calls -- which reads as ATC silence,
        the very thing these replays exist to measure."""
        self.send("state")  # its last line is "Region:", used as the sentinel
        out = []
        waited = 0.0
        while waited < timeout_s:
            r, _, _ = select.select([self.p.stdout], [], [], 0.05)
            if not r:
                waited += 0.05
                continue
            line = self.p.stdout.readline()
            if not line:
                break
            out.append(line)
            if _raw_fh:
                _raw_fh.write(line)
            if line.startswith("Region:") or "Region:    " in line:
                break
        return out

    def close(self):
        try:
            self.send("quit")
            self.p.wait(timeout=5)
        except Exception:
            self.p.kill()


# ── what ATC said, and what a pilot does about it ─────────────────────────

RE_CONTACT = re.compile(r"contact ([A-Za-z .'-]+?) on (\d{3}\.\d{2,3})", re.I)
RE_FL = re.compile(r"(descend|climb)(?: to)? flight level (\d{2,3})", re.I)
RE_ALT = re.compile(r"(descend|climb)(?: to)? ([\d ,]+) feet", re.I)
RE_MAINTAIN_FL = re.compile(r"maintain flight level (\d{2,3})", re.I)
RE_HEADING = re.compile(r"turn (left|right) heading (\d{2,3})", re.I)
RE_RESUME = re.compile(r"resume own navigation", re.I)


class Pilot:
    def __init__(self, repl, callsign="November Romeo Charlie"):
        self.repl = repl
        self.callsign = callsign
        self.cleared_ft = None
        self.vector_hdg = None   # steered heading while under radar vectors
        self.finished = False    # established on final: stop flying
        self.events = []

    def react(self, lines, where, alt):
        for raw in lines:
            line = raw.rstrip("\n")
            # Phase transitions belong in the timeline. They were printed by the
            # REPL all along and I read past them: a vectored arrival stayed in
            # IFR/DESCENT for its whole length because the ARRIVAL and APPROACH
            # triggers both live on the STAR, which vectors leave (2026-08-16).
            if ">> STATE" in line and "->" in line:
                st = line.split(">> STATE", 1)[1].split("@")[0].strip()
                self.events.append((where, alt, "STATE  " + " ".join(st.split())))
                continue
            if "ATC [" not in line:
                continue
            msg = line.split("ATC [", 1)[1]
            self.events.append((where, alt, "ATC [" + msg))

            m = RE_CONTACT.search(msg)
            if m:
                who, freq = m.group(1).strip(), m.group(2)
                # Tune FIRST, then check in -- the gate wants the call on the NEW
                # frequency, and a check-in sent on the old one is simply lost.
                self.repl.send("set com " + freq)
                self.react(self.repl.sync(), where, alt)
                self.repl.send(
                    "say %s with you, %s" % (self.callsign, self._level_words(alt))
                )
                self.repl.sync()
                self.events.append((where, alt, ">> pilot: checks in on %s (%s)" % (freq, who)))
                continue

            m = RE_HEADING.search(msg)
            if m:
                # Under vectors the aircraft leaves the route and flies the
                # assigned heading. Without this the compliance monitor sees a
                # pilot who never turns, re-issues once and then abandons -- which
                # is exactly what a fixed path produced on the first run.
                self.vector_hdg = float(m.group(2))
            if RE_RESUME.search(msg):
                self.vector_hdg = None
            if re.search(r"report established", msg, re.I):
                # The vectoring sequence is over: the aircraft is on the final
                # approach course and flies the procedure. Without this the
                # driver kept dead-reckoning on the axis heading and flew across
                # Europe -- Kaliningrad, Riga, Tallinn -- which looked like an
                # engine runaway and was purely the harness.
                self.vector_hdg = None
                self.finished = True

            m = RE_FL.search(msg)
            if m:
                self.cleared_ft = int(m.group(2)) * 100
                continue
            m = RE_ALT.search(msg)
            if m:
                self.cleared_ft = int(m.group(2).replace(" ", "").replace(",", ""))
                continue
            m = RE_MAINTAIN_FL.search(msg)
            if m:
                self.cleared_ft = int(m.group(1)) * 100

    @staticmethod
    def _level_words(alt):
        if alt >= 10000:
            return "flight level %d" % (alt // 100)
        return "%d feet" % alt


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 2
    route = json.load(open(sys.argv[1]))

    path = interpolate([tuple(p) for p in route["path"]], step_nm=4.0)
    gs = float(route.get("gs_kt", 280))
    ft_per_nm = FPM / (gs / 60.0)

    repl = Repl()
    repl.drain(0.8)
    setup = ["set navlog_clear 1"]
    for f in route["navlog"]:
        setup.append("set navlog_fix %s %.4f %.4f %d %s %d" % tuple(f))
    setup += [
        "set dest " + route["dest"],
        "set cruise %d" % route["cruise_ft"],
        "set runway " + route["runway"],
        "set lat %.4f" % path[0][0],
        "set lon %.4f" % path[0][1],
        "set alt %d" % (route["start_ft"] - 50),
        "set pa %d" % route["start_ft"],
        "set agl %d" % (route["start_ft"] - 1000),
        "set on_ground 0",
        "set gs %d" % int(gs),
        "set vs 0",
        "set com " + route.get("com", "118.750"),
        "set freq_type UNKNOWN",
        "jump enroute %d" % route["start_ft"],
    ]
    for c in setup:
        repl.send(c)
        repl.sync()

    pilot = Pilot(repl)
    alt = float(route["start_ft"])
    prev = path[0]
    flown = 0.0
    dest = None
    for f in route["navlog"]:
        dest = (f[1], f[2])

    repl.send("poll 5")
    pilot.react(repl.sync(), prev, int(alt))

    idx = 1
    while idx < len(path):
        if pilot.vector_hdg is not None:
            # Dead-reckon along the assigned heading instead of following the
            # route. 4 NM steps, matching the route interpolation.
            step = 4.0
            hdg = math.radians(pilot.vector_hdg)
            pt = (prev[0] + step * math.cos(hdg) / 60.0,
                  prev[1] + step * math.sin(hdg) /
                  (60.0 * math.cos(math.radians(prev[0]))))
        else:
            pt = path[idx]
            idx += 1
        leg = nm(prev, pt)
        if pilot.vector_hdg is not None:
            actual = bearing(prev, pt)
            err = (actual - pilot.vector_hdg + 540.0) % 360.0 - 180.0
            if abs(err) > 1.0:
                print("  [dr] assigned %.0f actual %.0f err %+.0f"
                      % (pilot.vector_hdg, actual, err))
        flown += leg
        # Fly toward the cleared level -- never below it, never ahead of it.
        if pilot.cleared_ft is not None:
            if alt > pilot.cleared_ft:
                alt = max(pilot.cleared_ft, alt - leg * ft_per_nm)
            elif alt < pilot.cleared_ft:
                alt = min(pilot.cleared_ft, alt + leg * ft_per_nm)
        dt = max(5, int(leg / gs * 3600.0))
        # EVERY sync's output must go through the pilot. ATC lines do not always
        # land in the sync that follows the command which produced them -- on a long
        # route they slip into the next one, and any sync whose result is discarded
        # silently eats them (13-fix DIK -> EDLW: 8 ATC calls emitted, 0 collected).
        repl.send("set heading %.0f" % bearing(prev, pt))
        pilot.react(repl.sync(), prev, int(alt))
        repl.send("track %.4f %.4f %d %d" % (pt[0], pt[1], int(alt), dt))
        pilot.react(repl.sync(), pt, int(alt))
        prev = pt
        if pilot.finished:
            break
        if pilot.vector_hdg is not None and flown > 900.0:
            break  # runaway guard: a vector that is never cancelled

    repl.close()

    print("\n%-9s %-8s %s" % ("dist", "level", "event"))
    print("-" * 78)
    for where, a, msg in pilot.events:
        d = nm(where, dest) if (dest and isinstance(where, tuple)) else 0.0
        lvl = ("FL%03d" % (a // 100)) if a >= 10000 else ("%d ft" % a)
        print("%6.0f NM %-8s %s" % (d, lvl, msg))
    return 0


if __name__ == "__main__":
    sys.exit(main())
