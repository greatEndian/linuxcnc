"""V6 - ORIGIN truth (measured).

Drive both channels to a designed world meet point with a generous gap,
then have the integrator MEASURE the real tool-to-tool gap and type it in.
|measured - designed| beyond tolerance -> FAIL, naming the ORIGIN
correction on the separation-dominant axis.

Designed geometry comes from the config: the two channels' [CHANNEL]ORIGIN
(+ ORIENT) place their local frames in the world; we command each to a
local point and compute the resulting world gap ourselves, so the number
the operator is checking against is derived from the very config under
test (a wrong ORIGIN makes the designed gap wrong too, which is the
point - the physical measurement is the ground truth).

Only meaningful for 2+ channels that physically share space.
"""
from . import Ask, Check, Info

APPROACH = 20.0      # mm gap to leave between the two tools at the meet
DEFAULT_TOL = 0.5    # mm


class V6Origin(Check):
    id = "V6"
    title = "ORIGIN truth (measured tool-to-tool gap)"
    needs_power = True
    needs_homed = True

    def _meet_axis(self, ch0, ch1):
        """world axis (0/1/2) along which the two origins are most
        separated - the handover/meet direction."""
        o0, o1 = self.s.origin(ch0), self.s.origin(ch1)
        sep = [abs(o1[i] - o0[i]) for i in range(3)]
        return sep.index(max(sep))

    def steps(self):
        if self.s.num_channels < 2:
            self.skip("single channel - no ORIGIN separation to measure")

        ch0, ch1 = 0, 1
        ax = self._meet_axis(ch0, ch1)
        axname = "XYZ"[ax]
        o0, o1 = self.s.origin(ch0), self.s.origin(ch1)
        # meet plane: midpoint of the two origins on the separation axis;
        # pull each tool APPROACH/2 back from it toward its own origin
        mid = (o0[ax] + o1[ax]) / 2.0
        sign0 = 1.0 if o0[ax] <= mid else -1.0
        sign1 = 1.0 if o1[ax] <= mid else -1.0
        w0 = mid + sign0 * (APPROACH / 2.0)
        w1 = mid + sign1 * (APPROACH / 2.0)

        # world target -> each channel's LOCAL coordinate on the meet axis.
        # for 90-deg-multiple ORIENTs the axis maps straight through with a
        # sign; solve local so that world_point == target on that axis.
        def local_for_world(ch, world_val):
            R = self.s.orient(ch)
            # map world separation axis back to the local letter that drives
            # it: use the ORIENT rotation's column signs
            from ..session import rot_matrix
            M = rot_matrix(*R)
            O = self.s.origin(ch)
            # find local axis l whose world contribution is on `ax`
            best, bestv = 0, 0.0
            for l in range(3):
                if abs(M[ax][l]) > bestv:
                    bestv, best = abs(M[ax][l]), l
            coeff = M[ax][best]
            local_val = (world_val - O[ax]) / coeff if coeff else 0.0
            return "XYZ"[best], local_val

        l0, lv0 = local_for_world(ch0, w0)
        l1, lv1 = local_for_world(ch1, w1)

        # the meet point sits inside the handover keep-out zone by design, so
        # the measurement is a SANCTIONED handover - assert the permit or the
        # interference guard will (correctly) stop the second tool short.
        allowed = self.s.set_interfere_allow(True)
        self.r.detail("interfere-allow asserted for the measurement: %s"
                      % ("yes" if allowed else "PIN NOT SETTABLE"))
        try:
            yield Info("moving ch%d %s->%.3f and ch%d %s->%.3f (designed "
                       "world gap %.1fmm on %s)"
                       % (ch0, l0, lv0, ch1, l1, lv1, APPROACH, axname))
            good0, err0 = self.s.mdi(ch0, "G53 G0 %s%.4f" % (l0, lv0))
            self.require(good0, "ch0 meet move refused: %s" % err0)
            good1, err1 = self.s.mdi(ch1, "G53 G0 %s%.4f" % (l1, lv1))
            self.require(good1, "ch1 meet move refused: %s" % err1)

            # designed gap = |world position difference| on the meet axis
            p0 = self.s.channel_world_xyz(ch0)[ax]
            p1 = self.s.channel_world_xyz(ch1)[ax]
            designed = abs(p1 - p0)
            self.r.detail("designed world gap on %s = %.3fmm (ch0@%.2f "
                          "ch1@%.2f)" % (axname, designed, p0, p1))
        finally:
            pass

        tol = yield Ask("v6.tolerance",
                        "Acceptance tolerance for the gap, in mm "
                        "(default %.1f)" % DEFAULT_TOL, "measure")
        if not tol or tol <= 0:
            tol = DEFAULT_TOL
        measured = yield Ask(
            "v6.measured-gap",
            "MEASURE the physical tool-to-tool gap along %s now (designed "
            "%.1fmm) and enter it in mm" % (axname, designed), "measure")
        err = abs(measured - designed)
        self.r.detail("measured %.3fmm, designed %.3fmm, error %.3fmm "
                      "(tol %.3f)" % (measured, designed, err, tol))
        # retreat both tools clear of the zone (local decreasing moves each
        # toward its own origin, out of the shared band) and drop the permit
        # before judging, so the machine is left safe whatever the verdict
        RETREAT = APPROACH + 60.0
        self.s.mdi(ch0, "G53 G0 %s%.4f" % (l0, lv0 - RETREAT))
        self.s.mdi(ch1, "G53 G0 %s%.4f" % (l1, lv1 - RETREAT))
        self.s.set_interfere_allow(False)
        self.require(
            err <= tol,
            "ORIGIN mismatch: measured gap off by %.3fmm on %s - adjust ch1 "
            "[CHANNEL]ORIGIN %s component by about %+.3fmm"
            % (err, axname, axname, (measured - designed)
               * (1.0 if p1 >= p0 else -1.0)))
        yield Info("ORIGIN gap within tolerance")
