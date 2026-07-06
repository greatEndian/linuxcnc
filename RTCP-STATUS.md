# RTCP / 5-axis TCP — status (fork: G43.4 / G43.5)

Native Fanuc-style RTCP in this fork. Branch: fork-upstream (production RTCP),
on LINUXCNC upstream + PR #4154 base. Updated 2026-07-06.

================================================================================
## 5-AXIS RTCP COVERAGE MAP
================================================================================

  FAMILY                      AXES         PREVALENCE        RTCP G43.5   EFFORT
  ----------------------------------------------------------------------------
  TABLE-TABLE (trunnion)      A+C  [AC]    #####  #1         [DONE]       0
   both rotaries in the table B+C  [BC]    most common       [DONE]       0
   head fixed                 A+B  [AB]    med/aero/auto     [DONE]       0
  ----------------------------------------------------------------------------
  HEAD-HEAD (swivel head)     B+C          ###    #2         [DONE]       0
   both rotaries in spindle   A+C          Europe / large    [DONE]       0
  ----------------------------------------------------------------------------
  HEAD-TABLE (mixed)          head+table   ##     #3         [DONE]       0
  NUTATING (45 deg spindle)   special      .      niche      [skip]       --
  ----------------------------------------------------------------------------
  GENERIC  config-driven: any 2 rotaries, any head/table mix [DONE]       0
  con=+1   conventional-directions variant (applies to all) [DONE]       0

  COVERAGE:  [####################]  all three industrial families DONE+sim
             (AB/AC/BC/BCHEAD/ACHEAD/BCHT) + the config-driven GENERIC
             topology (D3) covering every remaining solvable 2-rotary
             layout; G43.4 TCP on/off, con=+1, D7 refinements also DONE.
             ALL software-side RTCP work is complete. Remaining: only D6
             (real-machine commissioning, hw-gated).

================================================================================
## DONE  (built / gated 9/9 / D7 / verified)
================================================================================
  - G43.4 / G49  TCP on/off (switchkins; defer-until-idle; guards). Sim-validated.
  - G43.5 vector TCP (tool-axis IJK -> rotary angles), all three industrial
    5-axis families, each offline-proved (2M+ random vectors, 1.5e-15 or
    better) + rs274 matrix + live headless sim:
      AB   (xyzab_tdr, table-table, head fixed)
      AC   (xyzac-trt, table-table trunnion)
      BC   (xyzbc-trt, table-table trunnion)
      BCHEAD (5axiskins, head-head swivel head, B+C variant)
      ACHEAD (5axiskins sparm=tiltA, head-head swivel head, A+C variant)
      BCHT (maxkins, head-table mixed, B-head + C-table)
  - [RS274NGC]TCP_ORIENT_AXES = AB | AC | BC | BCHEAD | ACHEAD | BCHT. Per-topology
    part->table offset transform, singularity guard, unwrap-near, G53
    one-shot, G91 delta -- all five now support rotary work offsets.
  - [RS274NGC]TCP_CONVENTIONAL_DIRECTIONS (con=+1 variant, D5) for AC/BC/BCHT.
  - [RS274NGC]TCP_NO_SWITCH (BCHT/maxkins: permanently full-kinematics,
    no switchkins request needed) and [RS274NGC]TCP_KINSTYPE (switchkins
    type G43.4/G43.5 request, configurable away from the default 1).
  - D7 refinements: dual-solution nearest-travel (all 5 topologies pick
    whichever of the two valid solutions needs less combined joint travel),
    G43.5-with-arcs reviewed (safe, documented), BCHT rotary-offset
    transform (last item, closed D7).
  - GENERIC topology (D3): TCP_ORIENT_AXES=GENERIC + TCP_GENERIC_OUTER/
    INNER (+_MOUNT) describe any solvable 2-rotary layout; every fixed
    topology is an exact GENERIC instance (used as cross-oracles).
  - Verify assets: rtcp-dev/g435_trt_test.c, g435_con_test.c,
    maxkins_inv_test.c, maxkins_offset_test.c, bcht_offset_rs274_test.py,
    g435_achead_test.c, achead_rs274_test.py, g435_generic_test.c,
    generic_oracle_rs274_test.py, g435-acbc/, g435fix-tests/,
    bchead-sim/, achead-sim/, bcht-sim/, xyzac-trt-dev/.

================================================================================
## OPEN  (choose direction)
================================================================================
  D1  Live-sim validate AC/BC on vismach xyzac-trt/xyzbc-trt    [DONE]    0
  D2  HEAD-HEAD swivel: B+C [DONE, via BCHEAD/5axiskins];       [DONE]    0
      A+C variant [DONE 2026-07-05, via ACHEAD/5axiskins sparm=tiltA]
  D3  GENERIC config-driven topology (any two rotaries, any     [DONE]    0
      head/table mounts) [DONE 2026-07-05]
  D4  HEAD-TABLE mixed                                           [DONE]    0
      (via BCHT/maxkins, TCP_NO_SWITCH decouple, not a switchkins add)
  D5  con=+1 conventional-directions variant (all topologies)   [DONE]    0
  D6  Real-machine commissioning: sim-side prep DONE 2026-07-06 hw-gated  [future]
      (vismach visual validation + RTCP-COMMISSIONING.md checklist;
      only the physical-machine steps remain)
  D7  Refinements: dual-solution nearest-travel, kinstype knob, [DONE]    0
      G43.5-with-arcs review, BCHT rotary-offset transform

  RECOMMENDATION: every software-side item (D1-D5, D7, and now D3's
  GENERIC topology) is DONE+validated. The only remaining item is D6,
  real-machine commissioning, gated on hardware access.

================================================================================
## FULL NATIVE KINEMATICS INVENTORY — RTCP relevance
================================================================================
RTCP (tool-center-point with tool-axis orientation, G43.4/G43.5) is meaningful
only where the machine has 2 orientation DOF that tilt the TOOL vs the PART
(mill paradigm). Classed below.

  -- RTCP-RELEVANT (2-DOF tool orientation, mill) --------------------------
  xyzab_tdr-kins   AB  table dual-rotary      switchkins   [DONE]
  xyzac-trt-kins   AC  trunnion tilt+rotary   switchkins   [DONE]
  xyzbc-trt-kins   BC  trunnion tilt+rotary   switchkins   [DONE]
  5axiskins        BCHEAD swivel head (B+C)   switchkins   [DONE]; A+C variant
                   ACHEAD swivel head (A+C,                 [DONE] (sparm=tiltA);
                   sparm=tiltA)                             GENERIC config-driven
                                                            topology [DONE] (D3)
  maxkins          XYZBC head-table 5ax mill  NO-switchkins[DONE, as BCHT]
                   ^ Chris Radek's 'max' - a real 5-axis mill (B head + C
                     table = the concrete HEAD-TABLE case). Turned out NOT
                     to need switchkins support added: maxkins is
                     permanently full-kinematics, so BCHT decouples via
                     [RS274NGC]TCP_NO_SWITCH=1 instead (skips the
                     switchkins-type request entirely) rather than adding
                     switching capability to the kins module.

  -- ORIENTATION-CAPABLE, but TCP is inherent to the kins (RTCP optional) ---
  genhexkins       hexapod 6-DOF              switchkins   parallel; kins does
  pentakins        pentapod 5-DOF            NO-switchkins  full Cartesian+orient
  genserkins/u-    serial robot xyzabcuvw     switchkins   robots: tool-frame
  pumakins         PUMA 6-DOF robot           switchkins   programming, not the
  scorbot-kins     5-DOF educational arm      (arm)        mill G43.4/.5 paradigm

  -- NOT RTCP-RELEVANT (no 2-DOF tool orientation) -------------------------
  trivkins (3-axis/identity), corexykins, rotatekins, matrixkins, rosekins,
  scarakins (4-axis, tool vertical), tripodkins, lineardeltakins,
  rotarydeltakins (3-DOF translation), userkins (template).

--------------------------------------------------------------------------------
## maxkins — DONE (was "the additional one that needs RTCP")
--------------------------------------------------------------------------------
maxkins was the only remaining NATIVE 5-axis MILL not RTCP-covered. It is a
HEAD-TABLE machine (B-axis tilt on the head + C-axis rotary table), the
concrete D4 (head-table) target -- now DONE as topology BCHT. It turned out
NOT to need switchkins support added (it's permanently full-kinematics);
BCHT instead decouples via [RS274NGC]TCP_NO_SWITCH=1, skipping the
switchkins-type request rather than adding switching capability.

The parallel (genhex/penta) and robot (genser/puma/scorbot) kins are
orientation-capable but already produce Cartesian+orientation from their own
kins - they use tool-frame programming, not the mill RTCP switchkins paradigm,
so RTCP there is optional/non-standard.

================================================================================
## VISUAL OVERVIEW — RTCP effort by kinematics (2026-07-02)
================================================================================

 GROUP 1 — RTCP-RELEVANT  (2-DOF tool orientation = mill paradigm; G43.4/.5 fit)
 ┌────────────┬────────────────────────┬──────────┬─────────┬─────────────────┐
 │ kins       │ topology               │switchkins│ status  │ effort          │
 ├────────────┼────────────────────────┼──────────┼─────────┼─────────────────┤
 │ xyzab_tdr  │ AB  table dual-rotary  │   yes    │ DONE ✅ │ ·····  0        │
 │ xyzac-trt  │ AC  trunnion tilt+rot  │   yes    │ DONE ✅ │ ·····  0        │
 │ xyzbc-trt  │ BC  trunnion tilt+rot  │   yes    │ DONE ✅ │ ·····  0        │
 │ 5axiskins  │ BCHEAD swivel head B+C │   yes    │ DONE ✅ │ ·····  0        │
 │ 5axiskins  │ ACHEAD swivel head A+C │   yes    │ DONE ✅ │ ·····  0        │
 │ (config)   │ GENERIC any-2-rotary   │   yes    │ DONE ✅ │ ·····  0        │
 │ maxkins    │ BCHT head-table mixed  │ NO (kept)│ DONE ✅ │ ·····  0        │
 └────────────┴────────────────────────┴──────────┴─────────┴─────────────────┘
   side items:  AC/BC live-sim validate  DONE ✅  |  con=+1 variant  DONE ✅  |
                D7 refinements (nearest-travel, kinstype, arcs, BCHT offset) DONE ✅
   >>> remaining mill RTCP work: NONE (software side complete) <<<

 GROUP 2 — ORIENTATION-CAPABLE, RTCP OPTIONAL (kins already emits Cart+orient)
 ┌────────────┬────────────────────────┬──────────┬──────────┬────────────────┐
 │ kins       │ type                   │switchkins│ RTCP fit │ effort if forced│
 ├────────────┼────────────────────────┼──────────┼──────────┼────────────────┤
 │ genhexkins │ hexapod 6-DOF          │   yes    │ weak     │ ████·  ~2-3 day │
 │ pentakins  │ pentapod 5-DOF         │ NO (add) │ weak     │ ████·  ~3 day   │
 │ genserkins │ serial robot 6-DOF     │   yes    │ mismatch │ █████  ~3-4 day │
 │ ugenserkins│ serial robot           │   yes    │ mismatch │ █████  ~3-4 day │
 │ pumakins   │ PUMA robot 6-DOF       │   yes    │ mismatch │ █████  ~3-4 day │
 │ scorbot    │ 5-DOF educational arm  │   arm    │ mismatch │ █████  ~3-4 day │
 └────────────┴────────────────────────┴──────────┴──────────┴────────────────┘
   These use TOOL-FRAME programming (robot/parallel). Mill G43.4/.5 is NOT the
   standard paradigm here -> RECOMMEND NOT implementing unless a need arises.

 LEGEND  effort bar: ····· none · █···· ~0.5d · ███·· ~2-3d · █████ ~3-4d

================================================================================
## 2026-06-15 UPDATE — D1 DONE: AC + BC live-sim validated (tip-hold)
================================================================================
Followed DEV-WORKFLOW. Headless xyzac-trt / xyzbc-trt sims (TCP_ORIENT_AXES=AC/BC),
G43.5 tip-hold program (tilt tool to vector 0.5,0,0.866 at a FIXED tip):
 - AC: tip held at (0,0,0); joints -> A30.00 C90, Y7.68 Z6.34 compensate. ERROR OK.
 - BC: tip held at (0,0,0); joints -> B30.00 C180, X-0.18 Z17.99 compensate. ERROR OK.
=> RTCP holds the tool tip through the FULL pipeline (interp G43.5 IJK->angles ->
   motion -> TCP kins) on a live 5-axis sim, for both topologies. AC/BC now at
   AB's validation bar. Assets: ~/cnc-dev/rtcp-dev/xyzac-trt-dev/{ac,bc}-hl.ini,
   tiphold_{ac,bc}.ngc. AB/AC/BC RTCP = DONE+VALIDATED.

================================================================================
## 2026-06-15 UPDATE — HEAD-HEAD family DONE (BCHEAD via 5axiskins)
================================================================================
Implemented G43.5 vector TCP for the swivel-HEAD family (5axiskins XYZBCW, B+C
in the spindle). TCP_ORIENT_AXES=BCHEAD. Derived from 5axiskins' s2r forward:
v=(-sinB cosC, -sinB sinC, cosB) -> B=atan2(hypot(i,j),k), C=atan2(-j,-i).
HEAD machine: no part->table transform (part fixed). VERIFIED offline (2M+ vecs,
1.5e-15) + rs274 matrix + LIVE-SIM tip-hold on the 5axiskins bridgemill (B30/
C-180, Z compensates, tip held). fork-upstream a935774b2c + multichannel-upstream
893db8dbf1. Gate 9/9. Sim assets: ~/cnc-dev/rtcp-dev/bchead-sim/.

  UPDATED COVERAGE:  table-table (AB/AC/BC) DONE+sim  +  head-head (5axiskins)
  DONE+sim  =>  the TWO most common 5-axis families fully covered.
  Remaining: head-table mixed (maxkins ~2d, needs switchkins) ; con=+1 ~0.5d ;
  refinements. The big families are done.

================================================================================
## 2026-06-15 UPDATE — con=+1 (conventional-directions) variant DONE for AC/BC
================================================================================
Closed the conventional-directions gap. trtfuncs.c builds its forward rotation
with con = conventional-directions ? +1 : -1; the G43.5 AC/BC solver assumed the
kins default (con-1) only, so a machine with conventional-directions=1 got a
mirrored C angle. Generalised the inverse + offset transform with con:
  AC: R=Rz(con*c)*Rx(con*a) -> c=atan2(i,-con*j)
  BC: R=Rz(con*c)*Ry(con*b) -> c=atan2(j, con*i)
New INI key [RS274NGC]TCP_CONVENTIONAL_DIRECTIONS (default 0=con-1, must match
the kins HAL pin; AB/BCHEAD have no such pin and ignore it). con=-1 is
algebraically identical to the old code => default behaviour unchanged.
VERIFIED: offline g435_con_test.c (grid + 2,000,000 random incl. offsets, BOTH
con, worst 1.5e-15); rs274 discriminating matrix (AC C0<->C180, BC C180<->C0 as
con flips, a/b held 30); gate basic/tlo/abort/mdi-queue 8/8. Community docs
updated (g-code.adoc + ini-config.adoc). fork-upstream 6ace82c912 +
multichannel-upstream 3bab3954dc.

  REMAINING: head-table mixed (maxkins ~2d, needs switchkins added first) ;
  refinements (dual-solution nearest-travel, kinstype INI knob, G43.5 on arcs).

================================================================================
## 2026-06-15 UPDATE — BCHT head-table (maxkins) DONE + switchkins decoupled
================================================================================
Head-table "mixed" family (maxkins: B tilting head + C rotary table). Two parts:

 1. DECOUPLE: maxkins is permanently full-kinematics + KINS_NOT_SWITCHABLE (no
    identity mode). New [RS274NGC]TCP_NO_SWITCH=1 makes G43.4/G43.5/G49 skip the
    switchkins-type request (which otherwise trips the R10 non-switchable guard).
    Default 0 => AB/AC/BC/BCHEAD bit-identical.
 2. BCHT topology (=5): machine-frame tool axis u=(con sinB,0,cosB); the C table
    carries the part so relative to the part the axis is Rz(C)*u. Part-frame
    vector v=(i,j,k) -> C=atan2(j,i), B=atan2(con*hypot(i,j),k). con follows
    TCP_CONVENTIONAL_DIRECTIONS. Non-zero B/C work offsets refused for now.

A numeric probe (maxkins_axis_probe.c) confirmed maxkins' program-frame
orientation is C-INDEPENDENT (1-DOF in program coords) but full 2-DOF relative to
the PART - hence the part-frame interpretation.

VERIFIED: offline maxkins_inv_test.c (grid + 2,000,000 random, both con, 1.5e-15)
+ rs274 (con0 B-45/C45, con1 B+45/C45; BCHT emits 0 switchkins vs AC 1; AC/BC
unchanged) + gate 8/8 + LIVE headless max5 sim (vector .5/.5/.707 -> B-45/C45,
tip held at programmed point, linear joints compensate, ERROR OK, NO R10).
fork-upstream 5fd05e549a + multichannel-upstream 74481bb829. Sim assets:
~/cnc-dev/rtcp-dev/bcht-sim/. Community docs updated.

  COVERAGE NOW: table-table (AB/AC/BC, +con variants) + head-head (BCHEAD) +
  head-table (BCHT/maxkins) = all three industrial 5-axis families DONE+sim.
  REMAINING: refinements only (dual-solution nearest-travel, kinstype knob,
  G43.5 on arcs, BCHT rotary-offset transform).

================================================================================
## 2026-06-15 UPDATE — TCP_KINSTYPE knob DONE (D7 partial)
================================================================================
G43.4/G43.5 hard-coded a request for switchkins type 1 (the standard
identityfirst layout: type 0 = identity, type 1 = TCP). Added
[RS274NGC]TCP_KINSTYPE (default 1) so TCP can live at a different switchkins
type (e.g. a user kinematics module at type 2). G49 still always selects
identity (type 0). Ignored when TCP_NO_SWITCH=1 (nothing to switch there).
Default 1 keeps every existing config (AB/AC/BC/BCHEAD/BCHT) bit-identical.

VERIFIED: rs274 emits SET_SWITCHKINS_TYPE(1) by default and (2) with
TCP_KINSTYPE=2, solved angles unchanged; gate basic/tlo/abort/mdi-queue 8/8.
Community docs updated (ini-config.adoc). fork-upstream 5bf87199ec.

  D7 REMAINING: dual-solution nearest-travel, G43.5-with-arcs review, BCHT
  rotary-offset transform (non-zero B/C work offsets still refused).

================================================================================
## 2026-07-02 UPDATE — dual-solution nearest-travel DONE (D7 partial)
================================================================================
Every 2-DOF tilt+rotary orientation mechanism has two mathematically valid
solutions for any tool-axis vector. For the atan2(hypot(i,j),k)-based
topologies (AC/BC/BCHEAD/BCHT), (tilt, rotary) and (-tilt, rotary+180) reach
the identical physical orientation; for AB's dual-rotary table, the identity
is (a,b) and (a+180, 180-b). All five solvers only ever computed the
canonical branch (non-negative tilt, or asin's principal value for AB), so a
target near the branch boundary could force a needless large swing on one
axis when the other branch was already close to the current position.

Added tcp_pick_nearest_branch(): given both candidates unwrapped near the
current position, pick whichever has the smaller combined travel across
both rotary joints. Applied to all five topologies (interp_convert.cc).

VERIFIED: rs274 for each of AB/AC/BC/BCHEAD/BCHT with a vector whose current
position sits exactly at the alternate-branch solution -- confirmed the
solver now picks it (zero net travel) instead of the canonical branch
(which would need 120-180+ degrees of needless swing). Full tests/interp
suite: 77/80 pass, unchanged from before this change (3 pre-existing
failures, see below). fork-upstream 44c3e55c3e.

  NOTE: found 3 pre-existing tests/interp failures unrelated to this work
  (g10/g10-l1-l10, g71-endless-loop, inside-corners) -- caused by an
  earlier commit (6c1972a17d, not TCP_KINSTYPE as first suspected) making
  G49 emit a new SET_SWITCHKINS_TYPE(0) canon call; those 3 tests' expected
  files predate it (2026-05-04 vs 2026-06-16) and were never refreshed.
  Confirmed pre-existing (fails identically on a clean checkout without
  today's change). FIXED below.

  D7 REMAINING: G43.5-with-arcs review, BCHT rotary-offset transform
  (non-zero B/C work offsets still refused).

================================================================================
## 2026-07-02 UPDATE — stale tests fixed + G43.5-with-arcs review DONE (D7 partial)
================================================================================
Fixed the 3 pre-existing test failures noted above: regenerated
g10-l1-l10, g71-endless-loop, and inside-corners's expected files to
include G49's SET_SWITCHKINS_TYPE(0) (no code change -- the tests were
just stale since 6c1972a17d). tests/interp/ now 80/80 (was 77/80).
fork-upstream ee6a379d67.

Reviewed the G43.5-with-arcs interaction (D7). convert_arc() never checks
tcp_vector_mode, so I/J/K on a G2/G3 line are always the arc center, never
a tool vector -- verified via rs274: an arc's I/J/K solves as normal arc
geometry with no ambiguity while G43.5 is active, and the tool orientation
set by the last G1-with-vector holds fixed through the arc (A/B/C stay
unchanged in the ARC_FEED canon call). Explicit A/B/C words on the G2/G3
line still work normally to reorient during the arc, independent of
vector mode. Conclusion: no bug, no guard needed, just non-obvious --
documented in g-code.adoc so users don't assume G43.5 continuously
interpolates the tool vector along an arc (it doesn't; that would be a
separate, larger feature). fork-upstream 334976fdbf.

  D7 REMAINING: BCHT rotary-offset transform only (non-zero B/C work
  offsets still refused -- the last open D7 item).

================================================================================
## 2026-07-02 UPDATE — BCHT rotary-offset transform DONE — D7 COMPLETE
================================================================================
Derived and implemented the last open D7 item. BCHT (maxkins head-table)
refused any nonzero B/C rotary work offset with a G43.5 tool vector, since
the offset transform hadn't been derived.

B is head-mounted and never touches the part, so a B offset is a plain
angle shift with no frame-rotation implication -- the existing additive
off_b handling already covered it correctly, no change needed. C carries
the part, so a nonzero C offset rotates the part frame relative to the
machine; derived from first principles (a body-fixed part frame
calibrated at machine-C=0 vs one calibrated at machine-C=off_c differ by
exactly that rotation) that part frame -> table frame is
v_table = Rz(off_c) * v_prog -- unlike AC/BC's own C-offset transform,
this rotation is NOT con-scaled, matching how the BCHT forward formula
uses a plain Rz(C) with con only inside u(B).

VERIFIED:
- New offline C test (maxkins_offset_test.c, same methodology as the
  existing maxkins_inv_test.c): grid + 2,000,000 random vectors x random
  offsets, both con, worst error 2.3e-15.
- New automated rs274 -g test (bcht_offset_rs274_test.py): 200 random
  cases, each verified by feeding the solved program angles back through
  forward kinematics + the inverse offset rotation to confirm the
  original target vector is reproduced; worst error 1.9e-6 (limited by
  rs274 -g's 4-decimal text output, not the math).
- Confirmed the offset correctly interacts with the pre-existing
  dual-solution nearest-travel logic (an offset can shift which branch
  has less combined travel; both branches independently verified to
  reproduce the identical physical vector).
- Full tests/interp suite: 80/80 pass, unchanged.

Docs updated (g-code.adoc: removed the "not yet supported" caveat).
fork-upstream 4bbb69cf61.

  D7 IS NOW FULLY COMPLETE: dual-solution nearest-travel, kinstype knob,
  G43.5-with-arcs review, and BCHT rotary-offset transform are all done.

  NOTE: the "5-AXIS RTCP COVERAGE MAP" table and "OPEN" list near the top
  of this file still show D2 (head-head) and D4 (head-table) as [OPEN] --
  that's stale; both were completed by the BCHEAD (2026-06-15) and BCHT
  (2026-06-15) work documented further down. The genuinely remaining open
  items are: D3 (5axiskins generic head/table -- a broader, config-driven
  case beyond BCHEAD's specific swivel-head derivation) and D6
  (real-machine commissioning, hw-gated). Worth a top-of-file cleanup
  pass at some point, not done here to stay in scope.

================================================================================
## 2026-07-05 UPDATE — ACHEAD head-head A+C DONE — D2 FULLY COMPLETE
================================================================================
Implemented the second HEAD-HEAD variant (A tilt + C rotary, both in the
spindle; common on European/large gantry machines). Two parts:

 1. KINS VEHICLE: no native A+C swivel-head kins existed. Added a minimal
    sparm=tiltA variant to 5axiskins: geometrically the same spherical head
    with the azimuth phase-shifted 90 deg, r = s2r(R, C+90, 180-A); required
    coordinates become XYZACW (B optional instead of A). Default (no sparm
    flag) is bit-identical to the original XYZBCW module.
 2. ACHEAD topology (=6): tool axis v = Rz(C)*Rx(A)*z =
    ( sinA sinC, -sinA cosC, cosA ). Inverse: A = atan2(hypot(i,j), k),
    C = atan2(i, -j). Dual branch (-A, C+180) via the shared
    tcp_pick_nearest_branch(). HEAD machine: part fixed, so no part-frame
    transform; A/C work offsets are plain additive angle shifts (like
    BCHEAD). con knob n/a (no conventional-directions pin), like BCHEAD.

VERIFIED (same bar as every other topology):
- offline g435_achead_test.c: grid + 2,000,000 random unit vectors,
  three checks per point -- inverse recovers orientation (1.5e-15),
  interp forward == KINS forward s2r(R,C+90,180-A) (1.2e-15), and
  dual-branch identity (9e-16).
- achead_rs274_test.py: 200 random cases through real rs274 -g, half with
  random A/C work offsets, round-tripped through forward kins; worst
  1.9e-6 (print-precision limited).
- rs274 spot checks: nearest-branch (current at the alternate solution ->
  picked with zero travel instead of a 240 deg swing) and singularity
  (K1 -> A0, C word withheld, C holds current).
- LIVE headless sim tip-hold (5axiskins sparm=identityfirst,tiltA,
  coordinates=xyzacwy, achead-hl.ini): G43.5 vector (.5,0,.866) -> A30 C90,
  RT-COHERENT sampling via sampler/halsampler at every servo cycle:
  4,453 cycles during/after the reorientation, worst tip deviation
  7.5e-6 mm (double-precision noise). Joints land exactly on the
  kins-predicted compensation (J0 +50.000, J2 -3.3975). ERROR OK.
- Full tests/interp suite: 80/80, unchanged.

GOTCHA found while validating (matters for BCHEAD/ACHEAD sim configs):
G43.5 requests switchkins type 1 (TCP_KINSTYPE default), which assumes the
identityfirst module layout. Loading 5axiskins WITHOUT sparm=identityfirst
makes type0=fiveaxis/type1=identity, so G43.5 actually selects IDENTITY and
the tip is NOT held (joints track axis words 1:1 -- verified live, the
sampled joint stream shows zero compensation). The sim ini must use
sparm=identityfirst,tiltA (or set TCP_KINSTYPE to the fiveaxis type).

Docs updated (g-code.adoc topology table + ini-config.adoc TCP_ORIENT_AXES/
TCP_CONVENTIONAL_DIRECTIONS/TCP_NO_SWITCH). Sim assets:
~/cnc-dev/rtcp-dev/achead-sim/ (achead-hl.ini, achead_pivot.hal with the
RT sampler, tiphold_achead.ngc, run_tiphold_achead.sh driver).

  D2 IS NOW FULLY COMPLETE (BCHEAD 2026-06-15 + ACHEAD 2026-07-05).
  REMAINING: D3 (5axiskins generic config-driven case), D6 (hw-gated).

  NOTE (validation infra, NOT committed to rtcp): this sandbox has no IPv6,
  so linuxcncrsh could not listen (the exact bug fixed on fix/emcrsh-ipv4).
  The fix is applied to the local working tree only (src/emc/usr_intf/
  emcrsh.cc) to make headless-sim validation possible; it belongs to its
  own branch/PR and must NOT be committed with RTCP work.

================================================================================
## 2026-07-05 UPDATE — GENERIC config-driven topology DONE — D3 COMPLETE
================================================================================
Implemented TCP_ORIENT_AXES=GENERIC: instead of a hard-coded topology case,
the machine's two orientation rotaries are described in the INI --

  TCP_GENERIC_OUTER / TCP_GENERIC_INNER = [-]A | [-]B | [-]C
    (letter = rotation axis: A about X, B about Y, C about Z; optional '-'
     flips that rotary's direction sense; "outer" = nearer the machine
     frame in its chain, "inner" = nearer the tool/part)
  TCP_GENERIC_OUTER_MOUNT / TCP_GENERIC_INNER_MOUNT = HEAD | TABLE

THE MATH (one solver for everything): every solvable 2-rotary orientation
mechanism reduces to v = R_a(alpha)*R_b(beta)*z in the part frame, where
R_b (applied to the tool axis first) must tilt it (axis X or Y -- enforced
at INI parse, C rejected in that position) and R_a is about any other
principal axis. Mounts map the configured rotaries onto (alpha, beta):
head-head alpha=s_o*t_o, beta=s_i*t_i; table-table (transpose) alpha=
-s_i*t_i (INNER), beta=-s_o*t_o; mixed alpha=-s_t*t_t, beta=s_h*t_h.
Only 4 closed forms needed: (Z,X), (Z,Y), (X,Y), (Y,X). Dual branch:
(alpha+180, -beta) for the Z forms, (alpha+180, 180-beta) for the XY
forms, fed through the shared tcp_pick_nearest_branch().

WORK OFFSETS, one uniform rule that reproduces every per-topology special
case: rotate the programmed vector through the SAME R_a*R_b expression
evaluated at the offset angles, keeping only the table-mounted factors
(head-mounted offsets stay purely additive). Equivalently: the part frame
is the table pose at the offset angles, v_part = P(off)*P(th)^T*u(th).
This reproduces AC/BC's con-scaled Rz*Rx(/Ry) offset transform AND BCHT's
non-con-scaled plain Rz(off_c) automatically.

Every fixed topology is an exact GENERIC instance (the cross-oracle set):
  AB         = OUTER=-B/TABLE,  INNER=-A/TABLE
  AC  con-1  = OUTER= A/TABLE,  INNER= C/TABLE   (con+1: both '-')
  BC  con-1  = OUTER= B/TABLE,  INNER= C/TABLE   (con+1: both '-')
  BCHEAD     = OUTER= C/HEAD,   INNER=-B/HEAD
  ACHEAD     = OUTER= C/HEAD,   INNER= A/HEAD
  BCHT con-1 = OUTER=-C/TABLE,  INNER=-B/HEAD    (con+1: INNER=B/HEAD)
GENERIC ignores TCP_CONVENTIONAL_DIRECTIONS (signs are explicit per rotary).

VERIFIED:
- Offline g435_generic_test.c: enumerates ALL 64 valid configs (96 combos
  minus unsolvable/duplicate), 50,000 random vectors each (3.2M cases,
  half with random offsets on all three letters), solved with the
  implementation math and round-tripped through an INDEPENDENT physical
  forward (pure rotation-matrix products, v_part = P(off)*P(th)^T*u(th)):
  worst 3.9e-14, canonical AND dual branch.
- Cross-oracle generic_oracle_rs274_test.py: 270 cases through real
  rs274 -g -- 9 fixed-topology mappings x 30 random (start pose, offsets,
  vector) cases, fixed ini vs GENERIC-equivalent ini: worst STRAIGHT_FEED
  word difference 0.000e+00 (bit-identical output).
- LIVE headless sim (generic-hl.ini: GENERIC OUTER=C/HEAD INNER=A/HEAD on
  the 5axiskins sparm=identityfirst,tiltA machine): tip-hold, RT-coherent
  sampling, worst tip deviation 7.5e-6 mm across 4,805 servo cycles,
  joints land on the kins-predicted compensation exactly.
- Full tests/interp suite: 80/80, unchanged.

Docs updated (g-code.adoc GENERIC row + paragraph; ini-config.adoc: the
four TCP_GENERIC_* keys, solvability rule, offset semantics).

  D3 IS NOW COMPLETE. ALL SOFTWARE-SIDE RTCP WORK IS DONE (D1-D5, D7).
  REMAINING: D6 only (real-machine commissioning, hw-gated).

================================================================================
## 2026-07-06 UPDATE — D6 sim-side prep DONE (vismach validation + checklist)
================================================================================
Did everything in D6 (real-machine commissioning) that does not require
hardware:

1. VISMACH VISUAL VALIDATION (isolated Xvfb :99, frames captured from the
   real vismach models driven by the live headless sims via linuxcncrsh):
   - BCHEAD bridgemill (5axisgui): G43.5 vector -> B swings 0 to -45 with
     the tool visibly tilted 45 deg and the tip still on the same table
     point, gantry compensating.
   - AC trunnion (xyzac-trt-gui): table tilts A30/rotates C while the tool
     stays on the part point.
   - BCHT max5 (max5gui): head tilts B-30, spindle nose stays on the part.
     Gotcha: maxkins hard-codes B=joint4, C=joint5 (joints[3] is the unused
     A slot) -- max5gui hookup must net j4->tilt, j5->rotate; and the max5
     ini has no HOME_SEQUENCE, so "home all" fails -- home joints 0..5
     individually. max5's toolchange is auto-looped (no manual ack needed).
   Before/after frames: ~/cnc-dev/rtcp-dev/vismach-proof/. Drivers saved
   next to each sim's assets. AB table-dual-rotary visual skipped (same
   recipe, nothing new to learn); ACHEAD has no vismach model (the tiltA
   kins is validated numerically; building a model variant is optional
   polish).

2. FOUND+FIXED A LATENT SIM MISCONFIGURATION: the 2026-06-15 bchead-sim
   snapshot loads 5axiskins WITHOUT sparm=identityfirst, so G43.5's
   switchkins type-1 request selects IDENTITY -- re-ran its tip-hold with
   RT-coherent sampling: 63.4 mm tip deviation, joints tracking the axis
   words 1:1 (the tip-hold was NOT actually working in that layout; the
   solver output itself was correct). Fixed bchead-hl.ini with
   sparm=identityfirst and re-validated: 8.6e-6 mm worst deviation over
   2,686 servo cycles, joints exactly on the kins-predicted compensation.
   Snapshot updated. This is the same trap documented in the 2026-07-05
   ACHEAD update, now recorded as a first-class commissioning check.

3. RTCP-COMMISSIONING.md (new, committed): the hardware-day procedure --
   topology/identityfirst/TCP_NO_SWITCH/conventional-directions config
   checks, pivot+tool-length plumbing (sum2 pattern), rotary direction
   sense, centerline calibration, soft limits, the in-air tip-hold
   first-motion test (with the identityfirst symptom spelled out), solver
   spot checks (singularity / nearest-branch / offsets) mapped to their
   sim reference assets, and the genuinely hw-only dynamics items.

  D6 REMAINING: only the steps that need a physical machine (sections
  2-9 of RTCP-COMMISSIONING.md executed on real iron).
