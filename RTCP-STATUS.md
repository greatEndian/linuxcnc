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
  D7  Refinements: dual-solution nearest-travel, kinstype knob,
      G43.5-with-arcs review                                     ~1 day    [polish]

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
