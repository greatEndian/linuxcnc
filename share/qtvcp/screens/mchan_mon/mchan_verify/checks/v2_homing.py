"""V2 - homing.

Static: every owned joint has a HOME_SEQUENCE (master ini owns ALL joint
config, MC28); every HOME_USE_INDEX joint must have joint.N.index-enable
netted in the LIVE HAL (the drive-homing bridge - an unwired claim means
homing waits forever on iron). Live: home channel by channel
(exclusivity is intended - demonstrated, not fought), per-channel time
recorded, homed positions must match each joint's HOME. Ask: integrator
confirms every head physically parked at its reference.
"""
from . import Ask, Check, Info


class V2Homing(Check):
    id = "V2"
    title = "homing (sequence, index wiring, live per channel)"
    needs_power = True

    def steps(self):
        # ---- static: sequence coverage + index-enable wiring ----------
        for j in self.s.all_joints():
            seq = self.s.jvar(j, "HOME_SEQUENCE")
            if seq is None:
                self.r.detail("joint %d: no HOME_SEQUENCE (Home-All will "
                              "skip it)" % j)
            use_index = (self.s.jvar(j, "HOME_USE_INDEX", "NO") or "NO").upper()
            if use_index in ("YES", "1", "TRUE"):
                sig = self.s.hal.signal_of("joint.%d.index-enable" % j)
                self.r.detail("joint %d: HOME_USE_INDEX with index-enable "
                              "net: %s" % (j, sig or "NOT NETTED"))
                self.require(sig,
                             "joint %d claims HOME_USE_INDEX but "
                             "joint.%d.index-enable is not netted - homing "
                             "would wait forever on the drive" % (j, j))

        # ---- live: sequential per channel ------------------------------
        yield Info("homing every channel (sequential - homing sessions "
                   "are exclusive)")
        for ch in range(self.s.num_channels):
            # unhome first so the homing run is deterministic even when the
            # session arrives pre-homed (home(-1) on homed joints is a no-op)
            self.s.unhome_channel(ch)
            took = self.s.home_channel(ch)
            self.require(took is not None,
                         "channel %d did not finish homing in time" % ch)
            self.r.detail("ch%d homed in %.1fs" % (ch, took))
            for j in self.s.joints_of(ch):
                home = float(self.s.jvar(j, "HOME", "0") or 0)
                pos = self.s.joint_pos(j)
                self.r.detail("  joint %d at %.4f (HOME=%.4f)" % (j, pos, home))
                self.require(abs(pos - home) < 0.01,
                             "joint %d homed to %.4f, expected HOME=%.4f"
                             % (j, pos, home))

        ans = yield Ask("v2.physical-reference",
                        "Is every head physically parked at its reference "
                        "position (switch/mark) right now?", "yesno")
        self.require(ans, "homed position does not match the physical "
                          "reference - check HOME/HOME_OFFSET/switch wiring")
