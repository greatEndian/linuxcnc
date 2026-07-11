"""V4 - MAP letter->joint truth + channel isolation, per channel per letter.

Auto oracle: a small low-speed incremental world jog of the LETTER on
channel N must move exactly the mapped joint by the increment and leave
EVERY other joint (including the other channels') untouched - the
mchan_jog routing/isolation, re-proven on this machine. Ask: integrator
confirms the correct PHYSICAL axis of the correct head moved (the one
thing HAL cannot know).
"""
import time

import linuxcnc

from . import Ask, Check, Info

LINEAR_INCR = 2.0      # mm
ANGULAR_INCR = 5.0     # deg
SPEED_FRACTION = 0.10  # of the axis MAX_VELOCITY


class V4Map(Check):
    id = "V4"
    title = "MAP letter->joint truth + isolation"
    needs_power = True
    needs_homed = True

    def _axis_vmax(self, ini, letter, default=10.0):
        from ..session import ini_find
        v = ini_find(ini, "AXIS_%s" % letter.upper(), "MAX_VELOCITY")
        try:
            return float(v)
        except (TypeError, ValueError):
            return default

    def _jog_incr(self, ch, letter, incr, speed):
        c = self.s.chans[ch].c
        c.mode(linuxcnc.MODE_MANUAL)
        c.wait_complete()
        c.teleop_enable(1)
        c.wait_complete()
        c.jog(linuxcnc.JOG_INCREMENT, False,
              self.s.axis_index(letter), speed, incr)

    def _settle(self, joint, timeout=20.0):
        """position-stability settle: a ch0 teleop jog drives the axis
        planners (not free_tp), so motion.N.current-vel reads 0 mid-move -
        watch the target joint's pos-cmd instead."""
        t0 = time.time()
        last = self.s.joint_pos(joint)
        quiet = 0
        time.sleep(0.3)         # command latency: let the jog start
        while time.time() - t0 < timeout:
            now = self.s.joint_pos(joint)
            if abs(now - last) < 1e-9:
                quiet += 1
                if quiet >= 4:
                    return True
            else:
                quiet = 0
            last = now
            time.sleep(0.1)
        return False

    def steps(self):
        all_joints = self.s.all_joints()
        for ch in range(self.s.num_channels):
            ini = self.s.channel_inis[ch]
            for letter, joint in sorted(self.s.maps[ch].items()):
                angular = letter in "ABC"
                incr = ANGULAR_INCR if angular else LINEAR_INCR
                speed = self._axis_vmax(ini, letter) * SPEED_FRACTION
                unit = "deg" if angular else "mm"

                before = {j: self.s.joint_pos(j) for j in all_joints}
                yield Info("ch%d: jogging axis %s by +%.1f%s at %.1f%s/s "
                           "(joint %d expected)"
                           % (ch, letter, incr, unit, speed, unit, joint))
                self._jog_incr(ch, letter, incr, speed)
                self.require(self._settle(joint),
                             "ch%d axis %s jog did not settle" % (ch, letter))

                moved = self.s.joint_pos(joint) - before[joint]
                self.require(abs(moved - incr) < 0.01,
                             "ch%d axis %s: mapped joint %d moved %.4f, "
                             "expected %.4f"
                             % (ch, letter, joint, moved, incr))
                for j in all_joints:
                    if j == joint:
                        continue
                    drift = abs(self.s.joint_pos(j) - before[j])
                    self.require(drift < 1e-6,
                                 "ISOLATION: ch%d axis %s jog moved joint %d "
                                 "by %.6f" % (ch, letter, j, drift))
                self.r.detail("ch%d %s -> joint %d moved %+.3f%s, all other "
                              "joints still" % (ch, letter, joint, moved, unit))

                ans = yield Ask("v4.ch%d.%s" % (ch, letter),
                                "Head %d: did the physical %s axis (and "
                                "nothing else) just move %+.1f%s?"
                                % (ch, letter, incr, unit), "yesno")
                # jog back regardless, keep the machine where it started
                self._jog_incr(ch, letter, -incr, speed)
                self._settle(joint)
                self.require(ans,
                             "ch%d axis %s: operator says the wrong physical "
                             "axis moved - check MAP / drive wiring" % (ch, letter))
