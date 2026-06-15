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
