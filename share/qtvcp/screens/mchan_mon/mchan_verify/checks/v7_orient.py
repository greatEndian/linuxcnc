"""V7 - ORIENT truth (direction pattern).

For each shared world axis, jog the corresponding local axis + on both
channels and ask the integrator whether the two heads physically moved the
SAME or OPPOSITE world direction. Compare against the sign pattern implied
by each channel's [CHANNEL]ORIENT rotation. A mismatch means ORIENT does
not describe the real placement; the message names the pattern actually
observed so the integrator can pick the matching ORIENT.

Example (lathe, ch1 ORIENT 0 180 0): X opposite, Y same, Z opposite.
"""
from . import Ask, Check, Info

DELTA = 3.0     # mm


class V7Orient(Check):
    id = "V7"
    title = "ORIENT truth (world-direction pattern)"
    needs_power = True
    needs_homed = True

    def _predicted_same(self, letter):
        """does +letter on ch1 point the SAME world way as +letter on ch0?
        ch0 is the reference frame; compare the world direction of a unit
        +letter jog on each channel."""
        li = "XYZ".index(letter.upper())
        unit = [0.0, 0.0, 0.0]
        unit[li] = 1.0
        d0 = self.s.world_dir(0, unit)
        d1 = self.s.world_dir(1, unit)
        dot = sum(a * b for a, b in zip(d0, d1))
        return dot >= 0     # same-ish vs opposed on the dominant component

    def steps(self):
        if self.s.num_channels < 2:
            self.skip("single channel - ORIENT has no cross-channel pattern")

        shared = [l for l in "XYZ"
                  if l in self.s.maps[0] and l in self.s.maps[1]]
        for letter in shared:
            predicted = self._predicted_same(letter)
            pstr = "SAME" if predicted else "OPPOSITE"
            yield Info("jogging %s+ on both heads (config predicts %s world "
                       "direction)" % (letter, pstr))
            self.s.jog_and_settle(0, letter, DELTA)
            self.s.jog_and_settle(1, letter, DELTA)
            ans = yield Ask(
                "v7.%s" % letter,
                "%s+ on head 0 and %s+ on head 1: did the two tools move the "
                "SAME physical world direction? (no = opposite)"
                % (letter, letter), "yesno")
            # restore
            self.s.jog_and_settle(0, letter, -DELTA)
            self.s.jog_and_settle(1, letter, -DELTA)
            self.r.detail("%s: predicted %s, operator says %s"
                          % (letter, pstr, "SAME" if ans else "OPPOSITE"))
            self.require(
                ans == predicted,
                "ORIENT mismatch on %s: config predicts %s but heads move %s "
                "- ch1 [CHANNEL]ORIENT does not match the real placement"
                % (letter, pstr, "SAME" if ans else "OPPOSITE"))
        yield Info("ORIENT direction pattern matches the config")
