# RTCP / 5-axis TCP — status (fork: G43.4 / G43.5)

Native Fanuc-style RTCP in this fork. Branch: fork-upstream (production RTCP),
on LINUXCNC upstream + PR #4154 base. Updated 2026-06-15.

================================================================================
## 5-AXIS RTCP COVERAGE MAP
================================================================================

  FAMILY                      AXES         PREVALENCE        RTCP G43.5   EFFORT
  ----------------------------------------------------------------------------
  TABLE-TABLE (trunnion)      A+C  [AC]    #####  #1         [DONE]       0
   both rotaries in the table B+C  [BC]    most common       [DONE]       0
   head fixed                 A+B  [AB]    med/aero/auto     [DONE]       0
  ----------------------------------------------------------------------------
  HEAD-HEAD (swivel head)     B+C          ###    #2         [OPEN]       ~1 day
   both rotaries in spindle   A+C          Europe / large    [OPEN]       ~1 day
  ----------------------------------------------------------------------------
  HEAD-TABLE (mixed)          head+table   ##     #3         [OPEN]       ~2-3 days
  NUTATING (45 deg spindle)   special      .      niche      [skip]       --
  ----------------------------------------------------------------------------
  GENERIC  5axiskins (head/table configurable, one case)    [OPEN]       ~2-3 days
  con=+1   conventional-directions variant (applies to all) [OPEN]       ~0.5 day

  COVERAGE:  [##########--------]  most-common family DONE; head/mixed open
             3 topologies live (AB/AC/BC) ; G43.4 TCP on/off also DONE

================================================================================
## DONE  (built / gated 9/9 / D7 / verified)
================================================================================
  - G43.4 / G49  TCP on/off (switchkins; defer-until-idle; guards). Sim-validated.
  - G43.5 vector TCP (tool-axis IJK -> rotary angles):
      AB (xyzab_tdr) sim-validated ; AC (xyzac-trt) , BC (xyzbc-trt) offline
      proof (2M+ vectors, |dv| 1.5e-15) + rs274 matrix (normal/singularity/G53).
  - [RS274NGC]TCP_ORIENT_AXES = AB | AC | BC. Per-topology: part->table offset
    transform, C-singularity guard, unwrap-near, G53 one-shot, G91 delta.
  - Verify assets: rtcp-dev/g435_trt_test.c , g435-acbc/ , g435fix-tests/.

================================================================================
## OPEN  (choose direction)
================================================================================
  D1  Live-sim validate AC/BC on vismach xyzac-trt/xyzbc-trt    ~0.5 day  [val]
  D2  HEAD-HEAD swivel (B+C, then A+C) - 2nd most common         ~1 day ea [feat]
  D3  5axiskins generic head/table (broad coverage in one)      ~2-3 days [feat]
  D4  HEAD-TABLE mixed                                           ~2-3 days [feat]
  D5  con=+1 conventional-directions variant (all topologies)   ~0.5 day  [fix]
  D6  Real-machine commissioning (AB/AC/BC)                      hw-gated  [future]
  D7  Refinements: dual-solution nearest-travel [DONE], kinstype knob
      [DONE], G43.5-with-arcs review [DONE]                        ~1 day    [polish]
      -- only item left: BCHT rotary-offset transform

  RECOMMENDATION: most common family (trunnion) is DONE. Highest value next =
  D1 (finish AC/BC to AB's bar, cheap) then D2 (head-head swivel) or D3 (generic).

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
  5axiskins        generic head/table         switchkins   [OPEN D3]
  maxkins          XYZBC head-table 5ax mill  NO-switchkins[OPEN *NEEDS RTCP*]
                   ^ Chris Radek's 'max' - a real 5-axis mill (B head + C table,
                     = the concrete HEAD-TABLE case). Needs switchkins support
                     ADDED first, then a G43.5 topology case. Effort ~2 days
                     (switchkins wrap + derive B/C from its fwd + verify).

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
## THE ADDITIONAL ONE THAT NEEDS RTCP:  maxkins
--------------------------------------------------------------------------------
maxkins is the only remaining NATIVE 5-axis MILL not RTCP-covered. It is a
HEAD-TABLE machine (B-axis tilt on the head + C-axis rotary table), so it
doubles as the concrete D4 (head-table) target. Caveat: it is NOT switchkins-
capable today, so RTCP needs switchkins support wrapped around it first
(unlike AC/BC/AB which were already switchkins). Effort ~2 days.

The parallel (genhex/penta) and robot (genser/puma/scorbot) kins are
orientation-capable but already produce Cartesian+orientation from their own
kins - they use tool-frame programming, not the mill RTCP switchkins paradigm,
so RTCP there is optional/non-standard.

================================================================================
## VISUAL OVERVIEW — RTCP effort by kinematics (2026-06-15)
================================================================================

 GROUP 1 — RTCP-RELEVANT  (2-DOF tool orientation = mill paradigm; G43.4/.5 fit)
 ┌────────────┬────────────────────────┬──────────┬─────────┬─────────────────┐
 │ kins       │ topology               │switchkins│ status  │ effort          │
 ├────────────┼────────────────────────┼──────────┼─────────┼─────────────────┤
 │ xyzab_tdr  │ AB  table dual-rotary  │   yes    │ DONE ✅ │ ·····  0        │
 │ xyzac-trt  │ AC  trunnion tilt+rot  │   yes    │ DONE ✅ │ ·····  0        │
 │ xyzbc-trt  │ BC  trunnion tilt+rot  │   yes    │ DONE ✅ │ ·····  0        │
 │ 5axiskins  │ generic head/table     │   yes    │ OPEN ▢  │ ███··  ~2-3 day │
 │ maxkins    │ BC  head-table mill    │ NO (add) │ OPEN ▢  │ ███··  ~2 day   │
 └────────────┴────────────────────────┴──────────┴─────────┴─────────────────┘
   side items:  AC/BC live-sim validate  █····  ~0.5d   |  con=+1 variant █····  ~0.5d
   >>> finish ALL mill RTCP (5axiskins + maxkins): ~4-5 days total <<<

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
