"""V1 - session + power chain.

Auto: a real NML round-trip on every channel; then demonstrate the D5
rule live: with the OTHER channel still in estop, machine-on on ch0 must
NOT stick; after releasing all, it must. Ask: the integrator confirms
the PHYSICAL estop chain drops every head (config cannot know that).
"""
import time

import linuxcnc

from . import Ask, Check, Info


class V1Power(Check):
    id = "V1"
    title = "session + D5 power chain"

    def steps(self):
        # every channel answers
        for ch in self.s.chans:
            st = ch.poll()
            self.r.detail("ch%d NML ok, task_state=%d" % (ch.idx, st.task_state))

        if self.s.num_channels < 2:
            self.skip("single-channel config - D5 rule has nothing to show")

        yield Info("demonstrating the D5 power rule (machine will estop briefly)")
        # all into estop
        for ch in self.s.chans:
            ch.c.state(linuxcnc.STATE_ESTOP)
            ch.c.wait_complete()
        # release ONLY ch0, try to power on -> must NOT stick
        c0 = self.s.chans[0]
        c0.c.state(linuxcnc.STATE_ESTOP_RESET)
        c0.c.wait_complete()
        c0.c.state(linuxcnc.STATE_ON)
        c0.c.wait_complete()
        time.sleep(0.5)
        stuck = c0.poll().task_state == linuxcnc.STATE_ON
        self.r.detail("machine-on with ch1 still in estop: %s"
                      % ("STUCK ON (wrong)" if stuck else "refused (correct)"))
        self.require(not stuck,
                     "machine-on stuck while another channel was in estop "
                     "(D5 rule violated)")
        # now release all -> must stick
        ok = self.s.power_on_all()
        self.r.detail("machine-on after releasing ALL channels: %s"
                      % ("ON (correct)" if ok else "did not stick"))
        self.require(ok, "machine-on did not stick with all estops released")

        ans = yield Ask("v1.physical-estop",
                        "Press the PHYSICAL estop and confirm EVERY head "
                        "drops (then release it). Did all heads drop?",
                        "yesno")
        self.require(ans, "physical estop chain does not drop every head")
        # operator released the physical estop - power back up for later checks
        self.require(self.s.power_on_all(),
                     "machine would not power back on after the estop test")
