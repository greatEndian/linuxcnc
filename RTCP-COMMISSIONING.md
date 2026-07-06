# RTCP / G43.4-G43.5 — real-machine commissioning checklist (D6)

Everything that could be prepared without hardware is done (see
RTCP-STATUS.md: all topologies offline-proved, rs274-verified, live-sim
tip-hold validated at every servo cycle, and vismach-visualized). This file
is the procedure for the day a real 5-axis machine is available. Steps 1-5
are configuration and static checks; 6-9 are first-motion tests, each with
the sim asset that shows the expected reference behavior.

--------------------------------------------------------------------------
## 1. Topology + kins layout (get this right FIRST)
--------------------------------------------------------------------------
- [ ] `[RS274NGC]TCP_ORIENT_AXES` matches the machine: AB / AC / BC /
      BCHEAD / ACHEAD / BCHT, or GENERIC + the four `TCP_GENERIC_*` keys.
- [ ] THE IDENTITYFIRST TRAP (bit us in sim, will bite on hw): G43.4/G43.5
      request switchkins type `[RS274NGC]TCP_KINSTYPE` (default 1), which
      assumes the identityfirst layout (type 0 = identity, type 1 = TCP).
      A switchkins module loaded WITHOUT `sparm=identityfirst` puts the TCP
      kins at type 0 — G43.5 then silently selects IDENTITY and the tip is
      NOT held while the program looks otherwise normal (solved angles are
      right, XYZ never compensate). Verify with the in-air tip-hold (step 6)
      before trusting anything else. Fix: load the kins with
      `sparm=identityfirst` (5axiskins: `sparm=identityfirst,tiltA` for the
      A+C head) or point TCP_KINSTYPE at the type where TCP actually lives.
- [ ] maxkins-style permanently-full-kinematics module: `TCP_NO_SWITCH=1`
      (no identity mode exists; G43.4/.5/G49 must not request a switch).
- [ ] trt/maxkins machines: `[RS274NGC]TCP_CONVENTIONAL_DIRECTIONS` MUST
      equal the kins module's `conventional-directions` HAL pin. Symptom of
      mismatch: mirrored rotary angle (tool tilts the right amount, wrong
      side). GENERIC ignores this knob — direction senses are the explicit
      `-` prefixes on TCP_GENERIC_OUTER/INNER.

--------------------------------------------------------------------------
## 2. Pivot length / tool length plumbing
--------------------------------------------------------------------------
- [ ] The kins pivot pin (e.g. `5axiskins.pivot-length`, `maxkins.pivot-
      length`) must be mechanical pivot + current tool Z offset. Use the
      5axisgui.hal pattern: `sum2` of the constant and `motion.tooloffset.z`
      feeding the pin — a static `setp` is only correct for one tool.
- [ ] Measure the mechanical pivot (spindle gauge line to tilt-axis
      centerline) and record it here: ______ mm.
- [ ] Tool table lengths measured from the gauge line (same datum the pivot
      was measured to), not from an arbitrary tool tip.

--------------------------------------------------------------------------
## 3. Rotary direction sense (per rotary, before any TCP motion)
--------------------------------------------------------------------------
- [ ] Jog each rotary + a few degrees and confirm the PHYSICAL direction
      follows the right-hand rule about the letter's axis (A about +X,
      B about +Y, C about +Z) as the DRO counts up — or note the machine's
      inverted convention and capture it in the config (kins pin /
      GENERIC `-` sign), NOT by flipping servo polarity after the fact.
- [ ] Note 5axiskins item 10: the bridgemill tilt reads opposite to the
      conventional axis direction by design; the shipped BCHEAD/ACHEAD
      solver conventions already match the module.

--------------------------------------------------------------------------
## 4. Rotary centerline calibration (table machines: AC/BC/AB, BCHT C)
--------------------------------------------------------------------------
- [ ] Indicate a reference point on the table, rotate the rotary through
      its range, record runout: the offset between the rotary centerline
      and the machine origin goes into the kins offset pins (trt kins
      x/y/z-offset pins) — TCP quality is bounded by this measurement.
- [ ] Repeat for the second rotary (trunnion tilt axis height etc.).

--------------------------------------------------------------------------
## 5. Soft limits + safe envelope
--------------------------------------------------------------------------
- [ ] Rotary MIN/MAX_LIMIT set to the physical range (the dual-solution
      nearest-travel picker assumes both branches are reachable; a machine
      with e.g. B in [-5, 120] will still get correct single-branch
      solutions, but set the limits so the planner refuses the other one).
- [ ] Linear soft limits verified with the head fully tilted (worst-case
      reach differs from the vertical pose).

--------------------------------------------------------------------------
## 6. First TCP motion: in-air tip-hold (the make-or-break test)
--------------------------------------------------------------------------
Reference behavior: bchead-sim/run_tiphold_bchead.sh (head machine),
xyzac-trt-dev/run_vis_ac.sh (table machine); RT-coherent expectation is
tip deviation at numeric noise (< 1e-5 mm in sim; on hw expect the servo
following error instead).
- [ ] Tool over a reference point in air, comfortable Z clearance.
- [ ] Feed override 25%, single block ON.
- [ ] `G43.4 Hn` (or G43.5), then a small tilt: G43.4: `G1 F500 X.. B5`;
      G43.5: `G1 F500 X.. I0.087 J0 K0.996` (5 deg).
- [ ] WATCH XYZ: they must move to compensate the moment the rotary tilts.
      XYZ frozen while the rotary moves = the identityfirst trap (step 1).
- [ ] Dial indicator on a pin in the spindle vs the reference point:
      creep should be at following-error level, not millimetres.
- [ ] Increase to 30-45 deg tilts, both directions, then combined tilt +
      rotary moves.

--------------------------------------------------------------------------
## 7. Solver behavior spot checks (mirror the sim spot checks)
--------------------------------------------------------------------------
- [ ] Singularity: command the vector straight along +Z (`I0 J0 K1`): the
      tilt goes to 0, the azimuth rotary must HOLD its current angle (no
      word emitted), no error. (Sim reference: tachead_sing.ngc.)
- [ ] Nearest-branch: pre-position the rotaries at the alternate-branch
      solution of a target vector; the solver must choose it (near-zero
      travel), not swing 180+ deg through the canonical branch.
      (Sim reference: tachead_branch.ngc.)
- [ ] Work offsets: `G10 L2 Pn` with a rotary offset, re-run the tip-hold;
      table-mounted offsets also rotate the part frame — verify the tool
      leans the correct WAY relative to the fixtured part, not just the
      correct amount. (Sim reference: achead_rs274_test.py /
      bcht_offset_rs274_test.py conventions.)

--------------------------------------------------------------------------
## 8. Dynamics / servo (genuinely hw-only)
--------------------------------------------------------------------------
- [ ] Following error per joint during combined TCP moves at production
      feeds (the XYZ compensation velocity peaks near large tilts).
- [ ] FERROR/MIN_FERROR tuned on the rotaries (deg-domain, different
      magnitudes than the linears).
- [ ] Latency/servo-thread headroom with the full kins active (switchkins
      forward+inverse both run every cycle).

--------------------------------------------------------------------------
## 9. Sign-off matrix
--------------------------------------------------------------------------
Per topology actually commissioned, record: config hash, pivot value,
centerline offsets, tip-hold indicator reading at 45 deg, date/operator.

--------------------------------------------------------------------------
## Sim reference assets (all under ~/cnc-dev/rtcp-dev/)
--------------------------------------------------------------------------
  bchead-sim/    corrected identityfirst BCHEAD headless sim + tip-hold
                 driver + vismach visual driver (run_vis_bchead.sh)
  achead-sim/    ACHEAD (5axiskins sparm=identityfirst,tiltA) equivalents
  bcht-sim/      maxkins BCHT headless sim + vismach visual (bcht-vis.ini,
                 run_vis_bcht.sh; B=joint4, C=joint5 in max5gui hookup)
  xyzac-trt-dev/ AC/BC trunnion headless sims + vismach visual (run_vis_ac.sh)
  vismach-proof/ before/after model screenshots for BCHEAD, AC, BCHT
  g435-acbc/     rs274-level ini matrix incl. GENERIC oracle configs
