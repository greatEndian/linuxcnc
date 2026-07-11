"""V3 - joint direction truth.

For each channel's principal axis: a small low-speed incremental world jog
of +DELTA. Oracle: the mapped joint moved in the POSITIVE direction by
DELTA (a wired-backwards drive shows up as a negative or zero delta). Ask:
the integrator confirms the physical motion matches the axis-direction
convention (ISO 841 - generated per letter). The jog is then reversed to
leave the machine where it started.

This is distinct from V4 (which proves WHICH joint moves and isolation);
V3 is about the SIGN/sense an operator has to eyeball on real iron.
"""
from . import Ask, Check, Info

LINEAR_DELTA = 2.0     # mm
ANGULAR_DELTA = 5.0    # deg

# per-letter physical-direction convention prompts (ISO 841 lathe framing;
# generic fallback names the world axis)
CONV = {
    "X": "the tool moves in +X (on a lathe: radially AWAY from the "
         "spindle centerline)",
    "Y": "the tool moves in +Y",
    "Z": "the tool moves in +Z (on a lathe: AWAY from the headstock, "
         "toward the tailstock)",
    "A": "the A axis rotates in the +A (right-hand about X) sense",
    "B": "the B axis rotates in the +B (right-hand about Y) sense",
    "C": "the C axis rotates in the +C (right-hand about Z, CCW seen "
         "from +Z) sense",
}


class V3Direction(Check):
    id = "V3"
    title = "joint direction truth (sign + convention)"
    needs_power = True
    needs_homed = True

    def steps(self):
        for ch in range(self.s.num_channels):
            for letter, joint in sorted(self.s.maps[ch].items()):
                delta = ANGULAR_DELTA if self.s.is_angular(letter) \
                    else LINEAR_DELTA
                unit = "deg" if self.s.is_angular(letter) else "mm"
                yield Info("ch%d: jogging %s by +%.1f%s (joint %d)"
                           % (ch, letter, delta, unit, joint))
                moved = self.s.jog_and_settle(ch, letter, delta)
                self.require(
                    abs(moved - delta) < 0.01,
                    "ch%d %s: commanded +%.1f but joint %d moved %+.4f - "
                    "wrong magnitude/sign (drive SCALE sign or wiring)"
                    % (ch, letter, delta, joint, moved))
                self.r.detail("ch%d %s: +cmd -> joint %d %+.3f%s (positive)"
                              % (ch, letter, joint, moved, unit))
                ans = yield Ask(
                    "v3.ch%d.%s" % (ch, letter),
                    "Head %d, %s+ jog: did %s?"
                    % (ch, letter, CONV.get(letter.upper(),
                                            "the %s axis move in +%s"
                                            % (letter, letter))),
                    "yesno")
                # reverse to restore position regardless of the answer
                self.s.jog_and_settle(ch, letter, -delta)
                self.require(
                    ans,
                    "ch%d %s: physical direction is inverted vs convention "
                    "- flip the drive SCALE sign and swap MIN/MAX limits"
                    % (ch, letter))
