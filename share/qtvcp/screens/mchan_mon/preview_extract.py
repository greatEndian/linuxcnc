#!/usr/bin/env python3
"""mchan-preview - multichannel combined-preview core (s4: extraction).

Runs each channel's G-code through THAT channel's interpreter (its own
INI/var/units), collects the canon motion segments via GLCanon (correct
arc flattening), transforms them into the ONE machine world frame by the
channel's [CHANNEL]ORIGIN/ORIENT, and emits JSON. This is the data core
the static viewer (s5) and live layer (s6) draw from.

modes:
  dump-one <chN.ini> <prog.ngc>          one channel -> JSON on stdout
  dump     <master.ini> <ngc0> [ngc1..]  all channels -> combined JSON
                                         (programs given in channel order)
  selftest                               numeric gate on the lathe set

Channel placement = [CHANNEL]ORIGIN (mm, world) + ORIENT (deg rx ry rz,
applied Rz*Ry*Rx to the channel-local point before translation).
"""
import sys, os, json, math, shutil, subprocess, tempfile

RIP = "/home/user/linuxcnc-grandixximo-6.1"
sys.path.insert(0, os.path.join(RIP, "lib/python"))


# ---------------------------------------------------------------- ini ----
def ini_find(path, sec, var, default=None):
    """tiny ini reader (no linuxcnc import needed in the parent process)"""
    cur = None
    try:
        with open(path) as f:
            for line in f:
                line = line.split(';')[0].split('#')[0].strip()
                if not line:
                    continue
                if line.startswith('['):
                    cur = line.strip('[]').strip()
                elif '=' in line and cur == sec:
                    k, v = line.split('=', 1)
                    if k.strip() == var:
                        return v.strip()
    except OSError:
        pass
    return default


def triplet(s, default=(0.0, 0.0, 0.0)):
    if not s:
        return list(default)
    p = s.split()
    return [float(p[0]), float(p[1]), float(p[2])]


def color_rgb(s, default=(255, 255, 255)):
    if not s:
        return list(default)
    p = s.split(',')
    return [int(p[0]), int(p[1]), int(p[2])]


def rot_matrix(rx, ry, rz):
    """Rz*Ry*Rx, degrees -> 3x3"""
    rx, ry, rz = (math.radians(v) for v in (rx, ry, rz))
    cx, sx, cy, sy, cz, sz = (math.cos(rx), math.sin(rx), math.cos(ry),
                              math.sin(ry), math.cos(rz), math.sin(rz))
    return [
        [cz * cy, cz * sy * sx - sz * cx, cz * sy * cx + sz * sx],
        [sz * cy, sz * sy * sx + cz * cx, sz * sy * cx - cz * sx],
        [-sy, cy * sx, cy * cx],
    ]


def world_xyz(p9, R, O):
    x, y, z = p9[0], p9[1], p9[2]
    return [
        R[0][0] * x + R[0][1] * y + R[0][2] * z + O[0],
        R[1][0] * x + R[1][1] * y + R[1][2] * z + O[1],
        R[2][0] * x + R[2][1] * y + R[2][2] * z + O[2],
    ]


# ------------------------------------------------------------ dump-one ----
def dump_one(ini, ngc):
    """run in a SUBPROCESS per channel: gcode module keeps global state"""
    import gcode
    from rs274 import glcanon

    class Progress:
        def nextphase(self, *a): pass
        def progress(self, *a): pass
        def __getattr__(self, _n): return lambda *a, **k: None

    class SegCanon(glcanon.GLCanon):
        def __init__(self):
            glcanon.GLCanon.__init__(self, {}, "XYZABCUVW")
            self.progress = Progress()
        def get_external_length_units(self): return 1.0
        def get_external_angular_units(self): return 1.0
        def get_axis_mask(self): return 0x3f      # XYZABC
        def get_tool(self, t):
            return t, 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0
        def get_block_delete(self): return False
        def check_abort(self): pass
        def next_line(self, st):
            glcanon.GLCanon.next_line(self, st)

    os.environ["INI_FILE_NAME"] = os.path.abspath(ini)
    inidir = os.path.dirname(os.path.abspath(ini))
    os.chdir(inidir)

    canon = SegCanon()
    # never mutate the channel's live var file
    par = ini_find(ini, "RS274NGC", "PARAMETER_FILE", "preview.var")
    tmpdir = tempfile.mkdtemp(prefix="mchan-preview-")
    tmppar = os.path.join(tmpdir, os.path.basename(par))
    if os.path.exists(os.path.join(inidir, par)):
        shutil.copy(os.path.join(inidir, par), tmppar)
    canon.parameter_file = tmppar

    units = (ini_find(ini, "TRAJ", "LINEAR_UNITS", "mm") or "mm").lower()
    unitcode = "G21" if units.startswith("mm") else "G20"
    initcode = ini_find(ini, "RS274NGC", "RS274NGC_STARTUP_CODE", "") or ""

    result, seq = gcode.parse(os.path.abspath(ngc), canon, unitcode, initcode)
    if result > gcode.MIN_ERROR:
        sys.stderr.write("interp error %d: %s\n" % (result, gcode.strerror(result)))
        return 1

    O = triplet(ini_find(ini, "CHANNEL", "ORIGIN"))
    A = triplet(ini_find(ini, "CHANNEL", "ORIENT"))
    R = rot_matrix(*A)
    mc = int(ini_find(ini, "EMCMOT", "MOTION_CHANNEL", "0") or 0)
    # canon positions are interp-INTERNAL units (inch); machine units first
    lin = 25.4 if unitcode == "G21" else 1.0

    def seg(entry, kind):
        lineno, start, end = entry[0], entry[1], entry[2]
        s9 = [start[0]*lin, start[1]*lin, start[2]*lin] + list(start[3:])
        e9 = [end[0]*lin, end[1]*lin, end[2]*lin] + list(end[3:])
        return {
            "line": lineno, "kind": kind,
            "start": world_xyz(s9, R, O), "end": world_xyz(e9, R, O),
            "start_abc": list(start[3:6]), "end_abc": list(end[3:6]),
        }

    segs = ([seg(e, "traverse") for e in canon.traverse]
            + [seg(e, "feed") for e in canon.feed]
            + [seg(e, "arc") for e in canon.arcfeed])
    out = {
        "channel": mc, "ini": os.path.abspath(ini), "ngc": os.path.abspath(ngc),
        "origin": O, "orient": A,
        "color": color_rgb(ini_find(ini, "CHANNEL", "COLOR")),
        "counts": {"traverse": len(canon.traverse), "feed": len(canon.feed),
                   "arc": len(canon.arcfeed)},
        "segments": segs,
    }
    json.dump(out, sys.stdout)
    shutil.rmtree(tmpdir, ignore_errors=True)
    return 0


# ---------------------------------------------------------------- dump ----
def dump(master, progs):
    """master ini + per-channel programs in channel order -> combined JSON"""
    mdir = os.path.dirname(os.path.abspath(master))
    inis = [os.path.abspath(master)]
    n = 1
    while True:
        ci = ini_find(master, "MCHAN", "CHANNEL_INI")  # first only via tiny reader
        break
    # tiny reader can't do -num; walk the file for repeated CHANNEL_INI keys
    inis_extra = []
    cur = None
    with open(master) as f:
        for line in f:
            ls = line.split(';')[0].split('#')[0].strip()
            if ls.startswith('['):
                cur = ls.strip('[]').strip()
            elif cur == "MCHAN" and ls.startswith("CHANNEL_INI"):
                v = ls.split('=', 1)[1].strip()
                inis_extra.append(v if os.path.isabs(v) else os.path.join(mdir, v))
    inis += inis_extra
    if len(progs) != len(inis):
        sys.stderr.write("need %d programs (one per channel: %s)\n"
                         % (len(inis), ", ".join(os.path.basename(i) for i in inis)))
        return 1
    chans = []
    for ini, ngc in zip(inis, progs):
        r = subprocess.run([sys.executable, os.path.abspath(__file__),
                            "dump-one", ini, os.path.abspath(ngc)],
                           capture_output=True, text=True)
        if r.returncode != 0:
            sys.stderr.write("channel %s failed:\n%s\n" % (ini, r.stderr))
            return 1
        chans.append(json.loads(r.stdout))
    json.dump({"channels": chans, "interfere":
               (ini_find(master, "MCHAN", "INTERFERE", "") or "").split()},
              sys.stdout, indent=1)
    print()
    return 0


# ---------------------------------------------------------------- check ----
def envelope_box(ini):
    """world-frame bounding box of the channel envelope (ORIGIN/ORIENT +
    [AXIS_*] limits, 8-corner transform - same math as lint/viewer)"""
    O = triplet(ini_find(ini, "CHANNEL", "ORIGIN"))
    R = rot_matrix(*triplet(ini_find(ini, "CHANNEL", "ORIENT")))
    lim = []
    for L in "XYZ":
        lo = ini_find(ini, "AXIS_" + L, "MIN_LIMIT", "0")
        hi = ini_find(ini, "AXIS_" + L, "MAX_LIMIT", "0")
        lim.append((float(lo), float(hi)))
    pts = [world_xyz([x, y, z] + [0] * 6, R, O)
           for x in lim[0] for y in lim[1] for z in lim[2]]
    return [(min(p[k] for p in pts), max(p[k] for p in pts))
            for k in range(3)]


def check(master, progs):
    """verify every program stays inside ITS channel's working envelope
    (the 'examples must fit the workspace' rule - run before shipping any
    demo or production program pair)"""
    r = subprocess.run([sys.executable, os.path.abspath(__file__),
                        "dump", master] + progs, capture_output=True, text=True)
    if r.returncode != 0:
        sys.stderr.write(r.stderr)
        return 1
    d = json.loads(r.stdout)
    bad = 0
    for ch in d["channels"]:
        box = envelope_box(ch["ini"])
        ext = [[9e99, -9e99], [9e99, -9e99], [9e99, -9e99]]
        for s_ in ch["segments"]:
            for p in (s_["start"], s_["end"]):
                for k in range(3):
                    ext[k][0] = min(ext[k][0], p[k])
                    ext[k][1] = max(ext[k][1], p[k])
        ok = True
        for k, name in enumerate("XYZ"):
            if ext[k][0] < box[k][0] - 1e-6 or ext[k][1] > box[k][1] + 1e-6:
                ok = False
                print("FAIL ch%d %s: program world %s %.3f..%.3f OUTSIDE "
                      "envelope %.3f..%.3f"
                      % (ch["channel"], os.path.basename(ch["ngc"]), name,
                         ext[k][0], ext[k][1], box[k][0], box[k][1]))
        if ok:
            print("PASS ch%d %s inside its envelope "
                  "(X %.0f..%.0f Y %.0f..%.0f Z %.0f..%.0f)"
                  % (ch["channel"], os.path.basename(ch["ngc"]),
                     box[0][0], box[0][1], box[1][0], box[1][1],
                     box[2][0], box[2][1]))
        else:
            bad += 1
    print("CHECK: %s" % ("ALL INSIDE" if not bad else "%d program(s) OUT" % bad))
    return 1 if bad else 0


# ------------------------------------------------------------- selftest ----
def selftest():
    # reference machine lives with the dev assets, not in the repo;
    # point MCHAN_SELFTEST_DIR at it (defaults to the script dir)
    here = (os.environ.get("MCHAN_SELFTEST_DIR")
            or os.path.dirname(os.path.abspath(__file__)))
    master = os.path.join(here, "lathe-ch0.ini")
    p0 = os.path.join(here, "lathe-prog0.ngc")
    p1 = os.path.join(here, "lathe-prog1.ngc")
    r = subprocess.run([sys.executable, os.path.abspath(__file__),
                        "dump", master, p0, p1], capture_output=True, text=True)
    if r.returncode != 0:
        print(r.stderr)
        print("SELFTEST: dump failed")
        return 1
    d = json.loads(r.stdout)
    fails = []

    def chk(name, ok):
        print("%s: %s" % ("PASS" if ok else "FAIL", name))
        if not ok:
            fails.append(name)

    ch0, ch1 = d["channels"][0], d["channels"][1]
    chk("two channels extracted", len(d["channels"]) == 2)
    chk("ch0 is channel 0 / ch1 is channel 1",
        ch0["channel"] == 0 and ch1["channel"] == 1)
    chk("ch0 color cyan", ch0["color"] == [0, 255, 255])
    chk("ch1 color orange", ch1["color"] == [255, 165, 0])
    chk("ch1 origin 0 0 500 (Z-coaxial spindles, ISO 841)",
        ch1["origin"] == [0.0, 0.0, 500.0])
    chk("ch1 orient 0 180 0 (faces the main spindle)",
        ch1["orient"] == [0.0, 180.0, 0.0])
    chk("both have feed + traverse segments",
        ch0["counts"]["feed"] > 0 and ch0["counts"]["traverse"] > 0
        and ch1["counts"]["feed"] > 0 and ch1["counts"]["traverse"] > 0)

    def last_end(ch):
        return ch["segments"][-1]["end"] if ch["segments"] else None
    # programs end with G0 X0 Y0 (Z up) in CHANNEL frame; world = +ORIGIN
    e0 = sorted(ch0["segments"], key=lambda s: s["line"])[-1]["end"]
    e1 = sorted(ch1["segments"], key=lambda s: s["line"])[-1]["end"]
    chk("ch0 final point (0,0,5) world", all(abs(a - b) < 1e-6
        for a, b in zip(e0, [0, 0, 5])))
    # ch1 program ends locally at (0,0,3); ORIENT Ry(180) + ORIGIN 0 0 500
    # puts that at world (0,0,497) - the subspindle face side
    chk("ch1 final point (0,0,497) world (ORIGIN+ORIENT applied)",
        all(abs(a - b) < 1e-6 for a, b in zip(e1, [0, 0, 497])))

    zs0 = [p for s in ch0["segments"] for p in (s["start"][2], s["end"][2])]
    zs1 = [p for s in ch1["segments"] for p in (s["start"][2], s["end"][2])]
    xs1 = [p for s in ch1["segments"] for p in (s["start"][0], s["end"][0])]
    chk("ch0 world Z inside its envelope [-100,300]",
        min(zs0) >= -100 - 1e-6 and max(zs0) <= 300 + 1e-6)
    chk("ch1 world Z inside its envelope [200,600]",
        min(zs1) >= 200 - 1e-6 and max(zs1) <= 600 + 1e-6)
    chk("ch1 world X stays radial [-100,100] (mirrored by ORIENT)",
        min(xs1) >= -100 - 1e-6 and max(xs1) <= 100 + 1e-6)
    chk("zero cross-frame leakage (ch1 never below world Z 200)",
        min(zs1) >= 200 - 1e-6)
    c1 = [s["end_abc"][2] for s in sorted(ch1["segments"], key=lambda s: s["line"])]
    chk("ch1 C axis exercised and returns to 0",
        any(abs(v - 180) < 1e-6 for v in c1) and abs(c1[-1]) < 1e-6)

    # ORIENT math spot check: Ry(180) maps local (10,0,20) ->
    # world (-10, 0, 500-20) with ORIGIN (0,0,500)
    w = world_xyz([10, 0, 20, 0, 0, 0, 0, 0, 0], rot_matrix(0, 180, 0),
                  [0, 0, 500])
    chk("ORIENT math: Ry(180) mirrors X and Z about the ORIGIN",
        abs(w[0] + 10) < 1e-9 and abs(w[2] - 480) < 1e-9)

    print()
    print("MCHAN-PREVIEW SELFTEST: %s"
          % ("ALL PASS" if not fails else "%d FAILURE(S)" % len(fails)))
    return 1 if fails else 0


if __name__ == "__main__":
    if len(sys.argv) >= 4 and sys.argv[1] == "dump-one":
        sys.exit(dump_one(sys.argv[2], sys.argv[3]))
    if len(sys.argv) >= 3 and sys.argv[1] == "dump":
        sys.exit(dump(sys.argv[2], sys.argv[3:]))
    if len(sys.argv) >= 3 and sys.argv[1] == "check":
        sys.exit(check(sys.argv[2], sys.argv[3:]))
    if len(sys.argv) >= 2 and sys.argv[1] == "selftest":
        sys.exit(selftest())
    sys.stderr.write(__doc__)
    sys.exit(2)
