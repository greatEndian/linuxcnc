"""V9 - live interference (MC31) protective stop, guarded.

(a) Drive ONE channel to just OUTSIDE the keep-out zone: no protective
    stop may fire (false-positive guard).
(b) Park that channel inside the zone (alone - permitted), then drive the
    OTHER channel toward a target past it INSIDE the zone. The guard must
    latch (interfere-active) and HOLD the second channel short of its
    commanded target (it stops a few mm inside the boundary - the decel
    margin - instead of colliding).
(c) Retreat both and confirm the flag clears.

Every leg is announced and operator-gated. Requires a declared
[MCHAN]INTERFERE_ZONE. This is the check that caught the zone never being
armed at boot (fixed: linuxcnc.in --machine chmap call).
"""
import time

from . import Ask, Check, Info

EDGE_GAP = 8.0     # mm short of the near edge for the single-channel test
INSIDE = 20.0      # mm inside the zone to park the first channel


class V9Interference(Check):
    id = "V9"
    title = "live interference (MC31) protective stop"
    needs_power = True
    needs_homed = True

    def _zone(self):
        from ..session import ini_find
        z = ini_find(self.s.master_ini, "MCHAN", "INTERFERE_ZONE")
        if not z:
            return None
        p = z.split()
        if len(p) < 6:
            return None
        try:
            return [float(x) for x in p[:6]]
        except ValueError:
            return None

    def _world_axis_of_zone(self, zone):
        ext = [zone[1] - zone[0], zone[3] - zone[2], zone[5] - zone[4]]
        return ext.index(min(ext))

    def _local_target(self, ch, ax, world_val):
        from ..session import rot_matrix
        M = rot_matrix(*self.s.orient(ch))
        O = self.s.origin(ch)
        best, bestv = 0, 0.0
        for l in range(3):
            if abs(M[ax][l]) > bestv:
                bestv, best = abs(M[ax][l]), l
        coeff = M[ax][best]
        return "XYZ"[best], ((world_val - O[ax]) / coeff if coeff else 0.0)

    def _side(self, ch, ax, center):
        """which world side of the zone this channel approaches from."""
        return -1 if self.s.origin(ch)[ax] < center else +1

    def steps(self):
        if self.s.num_channels < 2:
            self.skip("single channel - no interference to test")
        zone = self._zone()
        if zone is None:
            self.skip("no [MCHAN]INTERFERE_ZONE declared")

        ax = self._world_axis_of_zone(zone)
        axname = "XYZ"[ax]
        zmin, zmax = zone[2 * ax], zone[2 * ax + 1]
        center = (zmin + zmax) / 2.0
        self.r.detail("keep-out band on %s: %.1f..%.1f (center %.1f)"
                      % (axname, zmin, zmax, center))

        s0, s1 = self._side(0, ax, center), self._side(1, ax, center)
        # near edge each channel faces (s<0 approaches from below -> zmin)
        edge0 = zmin if s0 < 0 else zmax
        edge1 = zmin if s1 < 0 else zmax
        # a clear parking spot OUTSIDE each channel's near edge (further out
        # in the approach direction = edge + s*margin)
        o0 = self._local_target(0, ax, edge0 + s0 * 40.0)
        o1 = self._local_target(1, ax, edge1 + s1 * 40.0)

        # clean start: a prior interference hold forces feed to 0 for tools
        # in the zone, so retreat them under the sanctioned permit, then drop
        # it - otherwise the guard would deadlock its own recovery move
        if self.s.interfere_active():
            self.s.set_interfere_allow(True)
            self.s.mdi(0, "G53 G0 %s%.4f" % o0)
            self.s.mdi(1, "G53 G0 %s%.4f" % o1)
        self.s.set_interfere_allow(False)
        self.s.mdi(0, "G53 G0 %s%.4f" % o0)
        self.s.mdi(1, "G53 G0 %s%.4f" % o1)
        self.require(not self.s.interfere_active(),
                     "could not clear a pre-existing interference hold at "
                     "start of V9")

        # (a) ch0 to just OUTSIDE its near edge -> no stop
        near = edge0 + s0 * EDGE_GAP
        l0, lv0 = self._local_target(0, ax, near)
        go = yield Ask("v9.a.go",
                       "About to move ONLY head 0 to the zone edge (world %s "
                       "%.1f, just OUTSIDE). No stop expected. Clear?"
                       % (axname, near), "yesno")
        self.require(go, "operator halted before the single-channel leg")
        good, err = self.s.mdi(0, "G53 G0 %s%.4f" % (l0, lv0))
        self.require(good, "ch0 edge move refused: %s" % err)
        self.require(not self.s.interfere_active(),
                     "FALSE POSITIVE: interference fired with only ch0 near "
                     "the edge")
        self.r.detail("ch0 alone at the edge (world %.1f): no interference"
                      % near)

        # park ch0 INSIDE the zone (alone -> permitted, no second occupant)
        inside0 = edge0 - s0 * INSIDE
        li0, liv0 = self._local_target(0, ax, inside0)
        good, err = self.s.mdi(0, "G53 G0 %s%.4f" % (li0, liv0))
        self.require(good, "ch0 could not park inside the zone alone: %s" % err)
        self.require(not self.s.interfere_active(),
                     "interference active with ch0 alone inside the zone")

        # (b) drive ch1 toward a target PAST ch0 inside the zone -> held short
        target1 = center - s1 * 5.0     # a hair past centre, well inside
        lt1, ltv1 = self._local_target(1, ax, target1)
        go = yield Ask("v9.b.go",
                       "About to drive head 1 INTO the zone toward head 0. A "
                       "PROTECTIVE STOP must hold it short. Ready?", "yesno")
        self.require(go, "operator halted before the collision leg")
        self.s.mdi_nowait(1, "G53 G0 %s%.4f" % (lt1, ltv1))
        # watch for the latch + hold-short
        fired = False
        t0 = time.time()
        while time.time() - t0 < 8.0:
            if self.s.interfere_active() and self.s.interfere_hold(1):
                fired = True
                break
            time.sleep(0.1)
        # let it settle at the held position
        time.sleep(1.0)
        held_world = self.s.channel_world_xyz(1)[ax]
        reached_target = abs(held_world - target1) < 2.0
        self.r.detail("ch1 driven toward world %.1f: interfere latched=%s, "
                      "held at world %.1f (target %.1f)"
                      % (target1, fired, held_world, target1))
        self.require(fired,
                     "NO protective stop: interfere-active/hold never "
                     "latched as ch1 entered the occupied zone")
        self.require(not reached_target,
                     "ch1 reached its in-zone target (%.1f) - the guard did "
                     "NOT hold it short of collision" % target1)
        held = yield Ask("v9.b.held",
                         "Did head 1 STOP short (protective stop) instead of "
                         "reaching head 0?", "yesno")
        self.require(held, "operator reports head 1 did not hold")

        # (c) recover: both tools are in the zone and held at feed 0, so the
        # retreat itself needs the sanctioned permit (that is the documented
        # recovery: jog one clear under motion.interfere-allow). Assert it,
        # back both out, then drop it - and the flag must clear.
        self.s.abort_all()
        self.s.set_interfere_allow(True)
        self.s.mdi(0, "G53 G0 %s%.4f" % o0)
        self.s.mdi(1, "G53 G0 %s%.4f" % o1)
        self.s.set_interfere_allow(False)
        self.require(not self.s.interfere_active(),
                     "interference still active after permitted retreat - "
                     "recovery failed")
        yield Info("interference held the collision and cleared on retreat")
