"""V8 - spindle / C-axis convention.

Per channel: M3 at low RPM, operator confirms the rotation sense (ISO 841:
CCW seen from +Z looking at the chuck face for M3), M5. Then, if the
channel has a C axis, jog C +10deg and confirm the SAME visual sense.
Catches a spindle-vs-C sign mismatch before rigid tapping relies on it.

Oracle part: after M3 the channel's commanded spindle speed is > 0 and the
sign matches M3 (positive); after M5 it is 0. The visual sense is the
operator's call.
"""
from . import Ask, Check, Info

RPM = 100
C_DELTA = 10.0


class V8Spindle(Check):
    id = "V8"
    title = "spindle / C-axis convention"
    needs_power = True
    needs_homed = True

    def steps(self):
        for ch in range(self.s.num_channels):
            go = yield Ask("v8.ch%d.spin-go" % ch,
                           "Head %d: about to start the spindle at %d RPM "
                           "(M3). Clear to spin?" % (ch, RPM), "yesno")
            self.require(go, "operator halted before spindle start on ch%d"
                             % ch)
            good, err = self.s.mdi(ch, "M3 S%d" % RPM)
            self.require(good, "ch%d M3 refused: %s" % (ch, err))
            spd = self.s.spindle_speed(ch)
            self.r.detail("ch%d M3 S%d -> spindle speed-out %.1f"
                          % (ch, RPM, spd))
            self.require(spd > 0,
                         "ch%d: M3 commanded but spindle speed-out=%.1f "
                         "(spindle not turning / wrong index)" % (ch, spd))
            sense = yield Ask(
                "v8.ch%d.m3-sense" % ch,
                "Head %d spindle: is it turning the M3 sense (CCW seen from "
                "+Z / looking at the chuck face)?" % ch, "yesno")
            self.s.mdi(ch, "M5")
            self.require(self.s.spindle_speed(ch) == 0,
                         "ch%d: spindle still turning after M5" % ch)
            self.require(sense, "ch%d: spindle rotates the wrong way for M3 "
                                "- fix the spindle direction wiring/scale"
                         % ch)

            if "C" in self.s.maps[ch]:
                yield Info("ch%d: jogging C +%.0fdeg" % (ch, C_DELTA))
                moved = self.s.jog_and_settle(ch, "C", C_DELTA)
                self.require(abs(moved - C_DELTA) < 0.05,
                             "ch%d C: jogged %+.3f, expected +%.0f"
                             % (ch, moved, C_DELTA))
                same = yield Ask(
                    "v8.ch%d.c-sense" % ch,
                    "Head %d: did C+ rotate the SAME visual sense the spindle "
                    "(M3) did?" % ch, "yesno")
                self.s.jog_and_settle(ch, "C", -C_DELTA)
                self.require(same,
                             "ch%d: C+ and M3 rotate opposite ways - the "
                             "C-axis sign disagrees with the spindle; rigid "
                             "tap/threading will be wrong" % ch)
