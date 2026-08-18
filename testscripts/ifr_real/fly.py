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

# Pilot reaction to a heading instruction, and the rate he turns at. 35 s is the
# middle of what the flown arrivals show between an instruction and its readback;
# 3 deg/s is standard rate. Both matter: together they consume axis distance
# while closing almost nothing laterally, which is what makes a vectored
# intercept run out of room.
# Measured on an EMBRAER PHENOM 300 (Data.txt, EDLW arrival 2026-08-18): 189 kt
# to 161 kt in 24 s, essentially level -- about 1.2 kt/s. A light jet in descent,
# with no speedbrake out, does noticeably worse.
#
# THE AIRFRAME MATTERS and this figure is the conservative end: the user normally
# flies a TBM or a Piper Meridian, and a turboprop with the propellers acting as
# drag slows down considerably faster. Those aircraft also arrive slower, so the
# 210 kt and 160 kt targets are frequently not issued at all -- vec_speed_phrase()
# only speaks when the aircraft is more than 10 kt above the target, which is the
# right behaviour and needs no per-type table. Holding the harness to the jet's
# rate therefore tests the WORST case, which is what a harness is for.
#
# The driver used to jump straight to the assigned speed, which made every
# distance-to-go optimistic and hid the question the user actually asked: can 210
# be turned into 160 before the intercept point? [C. P. Potter]
DECEL_KT_PER_S = 1.2
ACCEL_KT_PER_S = 0.8

# Fly the CLEARED route (read back from the engine) instead of the recorded
# ground track. Opt-in while it is being proved against the vectored replay.
FMS_MODE = os.environ.get("ATC_FMS", "") not in ("", "0")
FMS_CAPTURE_NM = 1.5   # a fix is flown over within this
FMS_BEHIND_NM = 12.0   # inside this, a fix more than 120 deg off the nose is behind

REACT_SECS = 35.0
TURN_RATE_DEG_S = 3.0


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


RE_FMS_HDR = re.compile(r"fmsroute idx=(\d+) n=(\d+)")
RE_FMS_FIX = re.compile(
    r"^\s+(\d+) (\S+) (-?\d+\.\d+) (-?\d+\.\d+) (-?\d+) (\d) (\d) (\d) (\d+) (\d)")


def parse_fmsroute(lines):
    """Return (idx, fixes) from a fmsroute block, or (None, None) if absent."""
    idx, fixes, seen = None, [], False
    for ln in lines:
        m = RE_FMS_HDR.search(ln)
        if m:
            idx, fixes, seen = int(m.group(1)), [], True
            continue
        if not seen:
            continue
        m = RE_FMS_FIX.match(ln.rstrip("\n"))
        if m:
            fixes.append({
                "ident": m.group(2),
                "lat": float(m.group(3)),
                "lon": float(m.group(4)),
                "alt_ft": int(m.group(5)),
                "is_fl": m.group(6) == "1",
                "spd": int(m.group(9)),
                "app": m.group(10) == "1",
            })
    return (idx, fixes) if seen else (None, None)


def ramp_speed(current, target, secs):
    """Move the speed toward the target at the aircraft's real rate."""
    if current is None:
        return target
    d = target - current
    lim = (ACCEL_KT_PER_S if d > 0 else DECEL_KT_PER_S) * secs
    if abs(d) <= lim:
        return target
    return current + (lim if d > 0 else -lim)


def cross_track(p, faf, course_deg):
    """Signed lateral offset of p from the track through `faf` on `course_deg`,
    in NM. Positive = right of the track. Lets the driver judge its OWN
    establishment instead of taking ATC's request for the event."""
    d = nm(p, faf)
    if d < 1e-6:
        return 0.0
    b = bearing(p, faf)
    return -d * math.sin(math.radians(b - course_deg))


def advance(a, course_deg, dist_nm):
    """Point `dist_nm` ahead of `a` on the true course `course_deg`. Flat-earth,
    which is exact enough for a final approach segment."""
    c = math.radians(course_deg)
    dlat = dist_nm * math.cos(c) / 60.0
    dlon = dist_nm * math.sin(c) / (60.0 * math.cos(math.radians(a[0])))
    return (a[0] + dlat, a[1] + dlon)


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
RE_SPEED = re.compile(r"reduce speed(?: to)?,? (\d{3}) knots", re.I)
# "contact Tower" arrives WITHOUT a frequency when the engine fails to resolve
# one -- which is the state of things today. RE_CONTACT needs "on NNN.NNN", so
# the driver used to ignore it entirely and the arrival simply stopped. Matching
# it separately makes the gap visible instead of silent.
RE_TOWER = re.compile(r"contact tower", re.I)
RE_LAND  = re.compile(r"cleared to land", re.I)
# The engine prints the FAF it resolved. Parsing it gives the driver the REAL
# axis -- position AND published final track -- instead of guessing. Without it
# the driver flew the last INTERCEPT heading (037) as though it were the
# localiser course (057) and diverged, producing a bogus "established LATE".
RE_FAF = re.compile(
    r"FAF: ident=(\S+) lat=([-\d.]+) lon=([-\d.]+) alt=(\d+)ft track=(\d+)", re.I)


class Pilot:
    def __init__(self, repl, callsign="November Romeo Charlie"):
        self.repl = repl
        self.callsign = callsign
        self.cleared_ft = None
        self.vector_hdg = None   # steered heading while under radar vectors
        self.finished = False    # arrival over: cleared to land, or on the ground
        self.established = False # on the final approach course
        self.final_course = None # course to fly inbound once established
        self.runway = ""         # for the "established" report
        self.faf = None          # (lat, lon) of the FAF, from the engine's own log
        self.faf_track = None    # published final approach track
        self.assigned_kt = None  # speed ATC has assigned, and the pilot flies
        # THE CLEARED ROUTE, as the engine itself is tracking it. Read back from
        # the REPL every step (`fmsroute`), so the driver flies what it has been
        # CLEARED rather than a recorded ground track. This is what lets one
        # route file exercise all three arrivals: the full STAR, the STAR cut
        # short by a direct to an IAF (which rewrites this list), and the
        # vectored arrival (which abandons it for a heading). A recorded track
        # can only ever reproduce the arrival it was recorded from -- ours was
        # recorded vectored, which is why the published-procedure mode could
        # never be replayed and every one of its defects had to be found in a
        # real flight. [C. P. Potter]
        self.fms = []       # [{ident, lat, lon, alt_ft, is_fl, spd, app}]
        self.fms_idx = 0    # the engine's own tracker index
        self.fms_i = None   # the PILOT's own index -- see fms_advance()
        self.tower_called = False  # checked in on Tower
        self.tower_no_freq = False # Tower handoff arrived without a frequency
        self.cleared_approach = False # approach clearance received
        self.hdg = None          # heading actually FLOWN (lags the assignment)
        self.react_s = 0.0       # seconds still to elapse before the turn starts
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
            # The FAF comes from a PLUGIN LOG line, not a transmission, so it has
            # to be read before the ATC filter below. It was placed after it and
            # therefore never ran: the driver had no axis and fell back to a
            # bearing to the aerodrome, which is not the localiser.
            m = RE_FAF.search(line)
            if m:
                self.faf = (float(m.group(2)), float(m.group(3)))
                self.faf_track = float(m.group(5))
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

            if RE_TOWER.search(msg) and not self.tower_called:
                # A frequency-less handoff cannot be complied with: there is
                # nothing to tune. Record it rather than pretend.
                if not RE_CONTACT.search(msg):
                    self.tower_no_freq = True
                    self.events.append(
                        (where, alt,
                         "!! pilot: cannot comply -- 'contact Tower' with NO frequency"))
                else:
                    self.tower_called = True

            m = RE_SPEED.search(msg)
            if m:
                # A speed instruction is flown, not logged and ignored. Without
                # this the driver kept cruise-descent speed through the whole
                # sequence and the geometry was sized on a speed no aircraft has.
                self.assigned_kt = float(m.group(1))

            m = RE_HEADING.search(msg)
            if m:
                # Under vectors the aircraft leaves the route and flies the
                # assigned heading. Without this the compliance monitor sees a
                # pilot who never turns, re-issues once and then abandons -- which
                # is exactly what a fixed path produced on the first run.
                if self.vector_hdg != float(m.group(2)):
                    # A pilot does not roll onto a new heading the instant ATC
                    # says it: he reads it back, then turns at a normal rate.
                    # Measured on the real flight of 2026-08-17: 39 s elapsed
                    # between the instruction and the readback, during which the
                    # aircraft ate 4.3 NM of axis distance and closed 0.2 NM
                    # laterally. Modelling neither is why this harness reported a
                    # clean intercept where the real one ran out of room.
                    self.react_s = REACT_SECS
                self.vector_hdg = float(m.group(2))
            if RE_RESUME.search(msg):
                self.vector_hdg = None
            if re.search(r"cleared .*approach", msg, re.I):
                # The clearance now travels WITH the intercept vector (ICAO
                # 8.9.4.1), so "report established" is a REQUEST, not the event.
                # Treating it as the event made the pilot announce established
                # 33 NM out and beeline for the FAF instead of flying his vector.
                self.cleared_approach = True
            if re.search(r"cleared to land", msg, re.I):
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
    def strip_fmsroute(lines):
        """Remove the fmsroute block before the lines reach react().

        Those lines are numbers -- "2 DOR 51.525342 7.631056 3000 0 0 1 210 1" --
        and react() matches instructions by pattern, so feeding them in let a
        route dump be read as a clearance. It moved the measured capture from
        5.9 NM to 5.4 NM the moment the query was added, which is a harness
        artefact of exactly the kind these replays exist to rule out."""
        out, inside = [], False
        for ln in lines:
            if RE_FMS_HDR.search(ln):
                inside = True
                continue
            if inside and RE_FMS_FIX.match(ln.rstrip("\n")):
                continue
            inside = False
            out.append(ln)
        return out

    def read_route(self, lines):
        """Absorb a fmsroute block and keep the pilot's own place in it.

        The engine's tracker index is its opinion of where the aircraft is; the
        pilot keeps his own, because the two legitimately differ (the tracker
        advances on proximity, the pilot on having actually flown the leg). On
        the FIRST read the engine's index seeds ours -- otherwise the driver
        would set off toward fixes already behind it. When ATC rewrites the
        route -- a direct to an IAF replaces the remaining STAR -- we keep the
        fix we were flying to if it survived, else fall back to the engine's
        index."""
        idx, fixes = parse_fmsroute(lines)
        if idx is None:
            return
        now = [f["ident"] for f in fixes]
        if self.fms_i is None:
            self.fms, self.fms_idx, self.fms_i = fixes, idx, idx
            return
        prev = [f["ident"] for f in self.fms]
        if prev != now:
            cur = (self.fms[self.fms_i]["ident"]
                   if 0 <= self.fms_i < len(self.fms) else None)
            self.fms_i = now.index(cur) if cur in now else idx
            if _raw_fh:
                _raw_fh.write("  [fms] route changed: %s -> %s (now at %s)\n"
                              % (" ".join(prev), " ".join(now),
                                 now[self.fms_i] if self.fms_i < len(now)
                                 else "END"))
        self.fms, self.fms_idx = fixes, idx

    def fms_advance(self, pos):
        """Sequence the cleared route like an FMS, and return the fix to fly to.

        Two ways a fix stops being the target: the aircraft REACHES it (within
        the capture radius), or it ends up BEHIND -- which happens after a route
        change, or when a leg is cut short. Chasing a fix that is behind is what
        sent the first version of this driver back out to 40 NM after passing the
        field. Fixes the CIFP gives no coordinates for (runway ends, some
        transitions) are skipped. Returns None when the route is flown out, which
        is the signal to hand over to the final-approach segment. [C. P. Potter]"""
        while self.fms_i is not None and self.fms_i < len(self.fms):
            f = self.fms[self.fms_i]
            if abs(f["lat"]) < 1e-4 and abs(f["lon"]) < 1e-4:
                self.fms_i += 1
                continue
            d = nm(pos, (f["lat"], f["lon"]))
            if d <= FMS_CAPTURE_NM:
                if _raw_fh:
                    _raw_fh.write("  [fms] captured %s\n" % f["ident"])
                self.fms_i += 1
                continue
            if self.hdg is not None and d < FMS_BEHIND_NM:
                rel = abs((bearing(pos, (f["lat"], f["lon"])) - self.hdg
                           + 540.0) % 360.0 - 180.0)
                if rel > 120.0:
                    if _raw_fh:
                        _raw_fh.write("  [fms] skipped %s (behind, rel %.0f, "
                                      "d %.1f)\n" % (f["ident"], rel, d))
                    self.fms_i += 1
                    continue
            return f
        if _raw_fh:
            _raw_fh.write("  [fms] route exhausted at i=%s of %d\n"
                          % (self.fms_i, len(self.fms)))
        return None

    def check_established(self, where):
        """THE PILOT DECIDES WHEN HE IS ESTABLISHED, by looking at his
        instruments -- he does not become established because ATC asked him to
        report it. Cleared for the approach and within half a mile of the
        published track, he calls it, stops flying the vector and tracks the
        axis inbound.

        Called once per FLOWN STEP, not per ATC line. Buried inside react()'s
        per-line loop it ran only when ATC happened to speak -- eleven times in
        a whole arrival -- so the aircraft sailed past the localiser between
        transmissions and the capture was never seen."""
        if (self.established or not self.cleared_approach
                or self.faf is None or self.faf_track is None
                or not isinstance(where, tuple)):
            return
        # ESTABLISHED MEANS ESTABLISHED INBOUND. Lateral offset alone is not
        # enough: on a published transition the route passes over the field
        # (DOR is the aerodrome VOR) and back out to the approach fixes, so the
        # aircraft crosses the EXTENDED axis while flying away from the runway.
        # Testing only |y| declared it established there, one mile from the
        # field, and the final segment then flew it 40 NM outbound. Two more
        # conditions: the FAF must be AHEAD along the axis, and the aircraft
        # must be pointing down the final approach track. [C. P. Potter]
        d_faf = nm(where, self.faf)
        rel = math.radians(bearing(where, self.faf) - self.faf_track)
        if d_faf * math.cos(rel) <= 0.5:
            return  # the FAF is behind -- this is not an inbound capture
        if self.hdg is not None:
            err = abs((self.hdg - self.faf_track + 540.0) % 360.0 - 180.0)
            # 50 deg, not 30: the intercept heading is up to 45 deg off the
            # axis (ICAO 8.9.3.6), and the aircraft is ON that heading at the
            # moment it captures -- it turns onto the course as it captures, it
            # is not already on it. A 30 deg gate refused our own 31 deg
            # intercept and silently killed the vectored replay. What this test
            # is really for is rejecting a crossing at a large angle, such as
            # overflying the field VOR on the extended axis.
            if err > 50.0:
                return  # crossing the axis, not tracking it
        if abs(cross_track(where, self.faf, self.faf_track)) < 0.5:
            self.established = True
            self.final_course = self.faf_track
            self.vector_hdg = None

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
    # Distances and the final approach segment are measured to the AERODROME.
    # Falling back to the last navlog fix is only a default: on this route that
    # fix is ADEMI, 10.5 NM east of EDLW, which made every printed distance wrong
    # and sent the final approach segment away from the runway.
    dest = None
    if isinstance(route.get("field"), list) and len(route["field"]) == 2:
        dest = (float(route["field"][0]), float(route["field"][1]))
    else:
        for f in route["navlog"]:
            dest = (f[1], f[2])

    pilot.runway = str(route.get("runway", ""))
    repl.send("poll 5")
    repl.send("fmsroute")          # seed the pilot's route before the first leg
    _l0 = repl.sync()
    pilot.read_route(_l0)
    pilot.react(Pilot.strip_fmsroute(_l0), prev, int(alt))

    idx = 1
    # In FMS mode the route, not the recorded track, decides when the enroute
    # phase ends -- so the loop is bounded by a step budget and exits when the
    # cleared route has been flown out.
    steps_cap = 4000 if FMS_MODE else len(path)
    while idx < steps_cap:
        if pilot.vector_hdg is not None:
            # Dead-reckon, but as an aircraft flies: 1 NM steps, a reaction delay
            # before the turn begins, then a standard-rate turn onto the assigned
            # heading. 4 NM steps with an instantaneous turn modelled a pilot who
            # does not exist and hid a real geometric failure.
            step = 1.0
            dt_s = step / max(60.0, gs) * 3600.0
            if pilot.hdg is None:
                pilot.hdg = pilot.vector_hdg
            if pilot.react_s > 0.0:
                pilot.react_s = max(0.0, pilot.react_s - dt_s)
            else:
                d = (pilot.vector_hdg - pilot.hdg + 540.0) % 360.0 - 180.0
                mx = TURN_RATE_DEG_S * dt_s
                pilot.hdg = (pilot.hdg + max(-mx, min(mx, d))) % 360.0
            hdg = math.radians(pilot.hdg)
            pt = (prev[0] + step * math.cos(hdg) / 60.0,
                  prev[1] + step * math.sin(hdg) /
                  (60.0 * math.cos(math.radians(prev[0]))))
        elif FMS_MODE and pilot.fms:
            tgt = pilot.fms_advance(prev)
            if tgt is None:
                break  # route flown out -- the final approach segment takes over
            brg = bearing(prev, (tgt["lat"], tgt["lon"]))
            d_t = nm(prev, (tgt["lat"], tgt["lon"]))
            step = min(4.0, max(0.3, d_t))
            pt = advance(prev, brg, step)
            pilot.hdg = brg
            idx += 1
        else:
            pt = path[idx]
            idx += 1
        leg = nm(prev, pt)
        # AFTER leg is known: `step` exists only on the vectored branch, and the
        # ramp needs the duration of the segment actually flown.
        if pilot.assigned_kt is not None:
            gs = ramp_speed(gs, pilot.assigned_kt,
                            max(5.0, leg / max(gs, 1.0) * 3600.0))
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
        repl.send("set gs %.0f" % gs)
        repl.send("set heading %.0f" % bearing(prev, pt))
        pilot.react(repl.sync(), prev, int(alt))
        repl.send("track %.4f %.4f %d %d" % (pt[0], pt[1], int(alt), dt))
        repl.send("fmsroute")
        lines = repl.sync()
        pilot.read_route(lines)
        pilot.react(Pilot.strip_fmsroute(lines), pt, int(alt))
        pilot.check_established(pt)
        prev = pt
        if pilot.finished or pilot.established:
            break
        if pilot.vector_hdg is not None and flown > 900.0:
            break  # runaway guard: a vector that is never cancelled

    # ── final approach ────────────────────────────────────────────────────────
    # Established on the axis, the aircraft flies the course inbound and descends
    # on the nominal 3 degree path. This is the segment where the Tower handoff
    # and the landing clearance are due; without it the harness stopped one
    # transmission short of the only part that has never worked.
    if pilot.established and dest:
        pilot.events.append((prev, int(alt), ">> pilot: reports established"))
        repl.send("say %s established runway %s" % (pilot.callsign, pilot.runway))
        pilot.react(repl.sync(), prev, int(alt))
        # The axis is the published final approach track through the FAF, which
        # the engine logged. The last vector was an INTERCEPT heading, so flying
        # it onward would take the aircraft across the localiser and off it.
        crs = pilot.faf_track if pilot.faf_track is not None else bearing(prev, dest)
        for _ in range(40):
            if pilot.finished:
                break
            d = nm(prev, dest)
            if d < 0.8:
                pilot.events.append((prev, int(alt), ">> pilot: over the field"))
                break
            step = min(1.0, d)
            # EUROCONTROL sequencing: 160 kt maximum from 8 NM to touchdown, and
            # "160 knots to 4 DME" is the standard restriction. Flying the whole
            # arrival at cruise-descent speed, as the route file does, is not a
            # profile any aircraft flies and it skews every distance measured
            # here. [C. P. Potter]
            # ATC's assigned speed wins; otherwise EUROCONTROL's 160 kt from
            # 8 NM to touchdown.
            want_kt = (pilot.assigned_kt if pilot.assigned_kt is not None
                       else (160.0 if d < 8.0 else float(route.get("gs_kt", 280))))
            gs = ramp_speed(gs, want_kt,
                            max(5.0, step / max(gs, 1.0) * 3600.0))
            # Steer at the destination once inside 6 NM: the published course and
            # a straight line differ a little, and drifting off it here would look
            # like a lateral deviation the plugin would rightly challenge.
            # Fly the LOCALISER COURSE, not a bearing to the field. Steering at
            # the field dragged the aircraft off the axis and produced a bogus
            # "established LATE, 2.1 NM off" that was the harness, not the
            # plugin. Only inside 3 NM does the runway itself become the target.
            # Capture the localiser, then track it: steer at the FAF while the
            # FAF is still AHEAD, then follow the published track to the runway.
            # "Ahead" is the ALONG-TRACK distance, not the plain range: past the
            # FAF the range grows again, so a range test turned the aircraft
            # round and the aircraft shuttled back and forth across the FAF
            # forever -- 40 steps spent between 5.5 and 6.5 NM, and the runway
            # never reached. [C. P. Potter]
            ahead = 0.0
            if pilot.faf is not None:
                dfaf = nm(prev, pilot.faf)
                ahead = dfaf * math.cos(math.radians(bearing(prev, pilot.faf) - crs))
            if ahead > 0.6:
                crs_now = bearing(prev, pilot.faf)
            else:
                crs_now = crs
            pt = advance(prev, crs_now, step)
            alt = max(500.0, alt - step * 318.0)
            # The assigned level holds until the FAF; from the FAF inbound the
            # aircraft is on the glide path and ATC's floor no longer applies
            # (ICAO 8.9.4.2 -- maintain the last level until intercepting the
            # glide path). Keeping the floor past the FAF pinned the aircraft at
            # 4900 ft all the way to the field.
            if pilot.cleared_ft is not None and ahead > 0.6:
                alt = max(alt, float(pilot.cleared_ft) - 100.0)
            repl.send("set gs %.0f" % gs)
            repl.send("set heading %.0f" % crs_now)
            pilot.react(repl.sync(), prev, int(alt))
            repl.send("track %.4f %.4f %d %d" % (pt[0], pt[1], int(alt),
                                                 max(5, int(step / gs * 3600.0))))
            pilot.react(repl.sync(), pt, int(alt))
            prev = pt
            if _raw_fh:
                _raw_fh.write("  [final] d=%.1f NM alt=%d crs=%.0f\n"
                              % (nm(prev, dest), int(alt), crs_now))
        else:
            pilot.events.append((prev, int(alt),
                                 ">> harness: final loop ended %.1f NM out" % nm(prev, dest)))

    repl.close()

    # Acceptance summary: the handful of numbers an arrival is judged on, so a
    # replay is read at a glance instead of by scrolling the timeline.
    print("\n=== acceptance ===")
    est = [e for e in pilot.events if "established" in str(e[2]).lower()]
    print("  %-34s %s" % ("established reported by the pilot",
                          "yes" if pilot.established else "NO"))
    print("  %-34s %s" % ("Tower handoff received",
                          "NO" if not (pilot.tower_called or pilot.tower_no_freq)
                          else ("yes, but WITHOUT a frequency" if pilot.tower_no_freq
                                else "yes, with a frequency")))
    print("  %-34s %s" % ("cleared to land", "yes" if pilot.finished else "NO"))
    print("  %-34s %s" % ("final speed assigned",
                          ("%.0f kt" % pilot.assigned_kt) if pilot.assigned_kt else "none"))

    print("\n%-9s %-8s %s" % ("dist", "level", "event"))
    print("-" * 78)
    for where, a, msg in pilot.events:
        d = nm(where, dest) if (dest and isinstance(where, tuple)) else 0.0
        lvl = ("FL%03d" % (a // 100)) if a >= 10000 else ("%d ft" % a)
        print("%6.0f NM %-8s %s" % (d, lvl, msg))
    return 0


if __name__ == "__main__":
    sys.exit(main())
