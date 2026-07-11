"""mchan_verify.session - one place for every live-session trap this
project has already paid for twice:

  - per-channel NML connections (linuxcnc.nmlfile trick)
  - persistent `halcmd -f` oracle with a sync handshake (a re-sourced
    rip-environment prints a warning line to stdout that desyncs the
    one-line-per-getp protocol - never re-source, always handshake)
  - D5 power rule (estop must be released on ALL channels before
    machine-on sticks)
  - SEQUENTIAL homing (homing sessions are exclusive across channels -
    intended behavior, home ch0, wait, then ch1)
  - auto(AUTO_RUN, 0) needs the start-line argument

The verificator CONNECTS to a running session; it never boots one.
"""
import os
import subprocess
import time

import linuxcnc

AXIS_LETTERS = "XYZABCUVW"


def ini_find(path, sec, var, default=None):
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


def ini_find_all(path, sec, var):
    out = []
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
                        out.append(v.strip())
    except OSError:
        pass
    return out


def parse_map(s):
    """'X:0 Y:1 Z:2 C:3' -> {'X': 0, ...} (letter -> global joint)"""
    out = {}
    for tok in (s or "").split():
        letter, _, joint = tok.partition(':')
        letter = letter.strip().upper()
        if letter in AXIS_LETTERS and joint.strip().lstrip('-').isdigit():
            out[letter] = int(joint)
    return out


class Hal:
    """persistent halcmd -f child; one getp/setp round-trip per call."""

    def __init__(self):
        self.p = subprocess.Popen(
            ["halcmd", "-f"], stdin=subprocess.PIPE,
            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)
        self.p.stdin.write("getp motion.servo.last-period\n")
        self.p.stdin.flush()
        for _ in range(10):
            try:
                float(self.p.stdout.readline().strip())
                return
            except ValueError:
                continue
        raise RuntimeError("halcmd pipe would not sync - session up?")

    def getp(self, pin):
        self.p.stdin.write("getp %s\n" % pin)
        self.p.stdin.flush()
        return self.p.stdout.readline().strip()

    def num(self, pin):
        try:
            return float(self.getp(pin))
        except ValueError:
            return float("nan")

    def flag(self, pin):
        return self.getp(pin) == "TRUE"

    def signal_of(self, pin):
        """net name a pin is connected to, or None (fresh halcmd -s query;
        kept outside the -f pipe: show output is multi-line)."""
        r = subprocess.run(["halcmd", "-s", "show", "pin", pin],
                           capture_output=True, text=True)
        for line in r.stdout.splitlines():
            parts = line.split()
            # "<owner> <type> <dir> <value> <name> [<==/=>/<=> <signal>]"
            if len(parts) >= 7 and parts[4] == pin:
                return parts[6]
            if len(parts) >= 7 and pin in parts:
                i = parts.index(pin)
                if len(parts) > i + 2:
                    return parts[i + 2]
        return None


class Chan:
    def __init__(self, idx, nml=None):
        if nml:
            linuxcnc.nmlfile = nml
        self.s = linuxcnc.stat()
        self.c = linuxcnc.command()
        self.e = linuxcnc.error_channel()
        self.idx = idx

    def poll(self):
        self.s.poll()
        return self.s

    def drain_errors(self):
        msgs = []
        while True:
            r = self.e.poll()
            if r is None:
                return msgs
            msgs.append(r[1])


class Session:
    """Parsed multichannel config + live connections."""

    def __init__(self, master_ini):
        self.master_ini = os.path.abspath(master_ini)
        self.ini_dir = os.path.dirname(self.master_ini)
        self.channel_inis = [self.master_ini]
        for ci in ini_find_all(self.master_ini, "MCHAN", "CHANNEL_INI"):
            p = ci if os.path.isabs(ci) else os.path.join(self.ini_dir, ci)
            self.channel_inis.append(os.path.abspath(p))
        self.maps = [parse_map(ini_find(ini, "CHANNEL", "MAP"))
                     for ini in self.channel_inis]
        self.num_channels = len(self.channel_inis)
        self.hal = None
        self.chans = []

    # ---- config views -----------------------------------------------
    def joints_of(self, ch):
        return sorted(self.maps[ch].values())

    def all_joints(self):
        out = []
        for m in self.maps:
            out.extend(m.values())
        return sorted(out)

    def axis_index(self, letter):
        return AXIS_LETTERS.index(letter.upper())

    def jvar(self, joint, var, default=None):
        """joint config comes from the MASTER ini only (MC28)."""
        return ini_find(self.master_ini, "JOINT_%d" % joint, var, default)

    # ---- live --------------------------------------------------------
    def connect(self):
        self.hal = Hal()
        self.chans = [Chan(0)]
        for ch in range(1, self.num_channels):
            nml = ini_find(self.channel_inis[ch], "EMC", "NML_FILE")
            if nml and not os.path.isabs(nml):
                nml = os.path.join(self.ini_dir, nml)
            self.chans.append(Chan(ch, nml=nml))
        for ch in self.chans:
            ch.poll()          # a real protocol round-trip per channel

    def power_on_all(self):
        """D5: release estop on every channel FIRST, then machine-on."""
        for ch in self.chans:
            ch.c.state(linuxcnc.STATE_ESTOP_RESET)
            ch.c.wait_complete()
        for ch in self.chans:
            ch.c.state(linuxcnc.STATE_ON)
            ch.c.wait_complete()
        return all(c.poll().task_state == linuxcnc.STATE_ON
                   for c in self.chans)

    def unhome_channel(self, ch):
        """unhome this channel's own joints (through its OWN connection -
        cross-channel unhome is correctly refused)."""
        c = self.chans[ch]
        c.c.mode(linuxcnc.MODE_MANUAL)
        c.c.wait_complete()
        c.c.teleop_enable(0)
        c.c.wait_complete()
        c.c.unhome(-1)
        c.c.wait_complete()

    def home_channel(self, ch, timeout=45.0):
        """home one channel and WAIT (exclusivity: never overlap these)."""
        c = self.chans[ch]
        c.c.mode(linuxcnc.MODE_MANUAL)
        c.c.wait_complete()
        c.c.home(-1)
        t0 = time.time()
        joints = self.joints_of(ch)
        while time.time() - t0 < timeout:
            if all(self.hal.flag("joint.%d.homed" % j) for j in joints):
                return time.time() - t0
            time.sleep(0.25)
        return None

    def joint_pos(self, joint):
        return self.hal.num("joint.%d.pos-cmd" % joint)

    def wait_motion_idle(self, ch, timeout=30.0):
        c = self.chans[ch]
        t0 = time.time()
        while time.time() - t0 < timeout:
            s = c.poll()
            if s.interp_state == linuxcnc.INTERP_IDLE \
                    and self.hal.num("motion.%d.current-vel" % ch) == 0.0:
                return True
            time.sleep(0.1)
        return False

    def abort_all(self):
        for ch in self.chans:
            try:
                ch.c.abort()
                ch.c.wait_complete()
            except linuxcnc.error:
                pass
