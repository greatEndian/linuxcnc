# MCHAN F2 — Commissioning Verificator (design)

Status: DESIGN approved-pending-review (2026-07-11). Scope decisions by the
user (2026-07-11): check-module core + CLI wizard + qtvcp page ("both from
the start"), all three check areas (geometry/axes core, homing, live
interference), lives IN-REPO. This document is the F2 problem map +
validation plan, per the project's pre-build discipline (see
WAITM-RESEARCH.md for the pattern). Origin of F2: user note 2026-06-12
(testing.txt SECTION 13), deferred to finalization in
multichannel-DESIGN.txt.

## 1. WHAT IT IS (and what it is not)

A guided, executable commissioning procedure for a multichannel machine:
it moves axes (or asks the integrator to), watches ground truth (HAL pins +
per-channel NML), asks the integrator targeted yes/no/measured questions,
and writes a dated, signed commissioning REPORT with a sign-off matrix
(house style: RTCP-COMMISSIONING.md).

The two existing tools answer different questions and stay as-is:
  - mchan-lint      : "is the CONFIG self-consistent?"   (static, no session)
  - mchan-preview   : "what WOULD programs do, geometrically?" (interp, no iron)
  - mchan-verify    : "does the config's claim match the REAL IRON?"  <- F2

Not in scope: drive tuning (belongs to the drive vendor tooling), servo/RT
budget measurement (RTCP checklist section 8 owns that pattern), tool-table
management.

## 2. ARCHITECTURE

    share/qtvcp/screens/mchan_mon/mchan_verify/
        __init__.py
        session.py      one class: per-channel NML connections (the proven
                        linuxcnc.nmlfile trick), HAL oracle (persistent
                        halcmd -f with sync handshake), bring-up helpers
                        (D5 power rule, SEQUENTIAL homing - exclusivity),
                        teardown. All the traps already solved in
                        validation/val_waitm_live.py move here.
        checks/         one module per check (see catalog): class with
                          id, title, needs (preconditions), steps()
                        steps() yields typed actions:
                          Auto(fn)               machine does something
                          Ask(id, text, kind)    operator answers
                                                 kind = yesno | measure | choice
                          Assert(fn, text)       oracle decides
        report.py       journal -> markdown report + sign-off matrix +
                        config fingerprint (sha256 of every ini/hal used)
        cli.py          the wizard: run/skip/repeat/abort per check,
                        --batch <answers.json> for headless gating
        (qtvcp page)    verify_page.py + ui: SAME check modules, buttons
                        instead of terminal prompts (phase P3)

    scripts/mchan-verify        thin launcher (sources rip env, runs cli)

Design rule that makes the whole thing testable: every operator interaction
is an `Ask` with a stable string id. Interactive mode prints it; batch mode
looks it up in the answers file. The tool's own regression gate is a batch
run against the lathe sim where the "operator" is an answers file — plus
NEGATIVE configs where a deliberately wrong ini must make the check FAIL.

## 3. CHECK CATALOG

Order matters: each check declares `needs` (e.g. V3+ need V2 homed).
Every check ends PASS / FAIL / SKIPPED(reason) in the report.

V0  static pre-flight        AUTO. Run mchan-lint (rc) + mchan-preview
                             selftest. Any lint FAIL blocks the wizard
                             (config bugs are cheaper fixed before iron).
V1  session + power chain    AUTO+ASK. Channels all up (real NML round-trip
                             per channel), D5 rule demonstrated: estop-reset
                             on ch0 only -> machine-on must NOT stick; all
                             channels -> must stick. Operator confirms the
                             physical estop chain drops BOTH heads.
V2  homing                   AUTO+ASK. Static: HOME_SEQUENCE covers all
                             owned joints; HOME_USE_INDEX joints have
                             joint.N.index-enable netted (drive-homing
                             bridge, Section-20 story). Live: home ch0,
                             WAIT, home ch1 (exclusivity is intended
                             behavior and gets *demonstrated*, not fought);
                             per-joint homed + time; operator confirms each
                             head parked at its physical reference.
V3  joint direction truth    ASK per joint. Low-vel incremental jog +DELTA
                             on ONE joint; operator answers the convention
                             question generated from config (lathe: "ch1 X+
                             must move the subspindle tool AWAY from
                             spindle centerline (radial, ISO 841) - did
                             it?"). -DELTA returns it. Wrong answer names
                             the fix (drive/SCALE sign + limits swap).
V4  MAP letter->joint truth  AUTO+ASK per channel per letter. World jog of
                             the LETTER on channel N; oracle: exactly the
                             mapped joint moved (HAL pos-cmd), zero motion
                             on every other joint INCLUDING the other
                             channel's (the mchan_jog isolation, re-proven
                             on iron); operator confirms the right physical
                             axis moved.
V5  envelope reality         AUTO+ASK per axis. Creep to MIN_LIMIT and
                             MAX_LIMIT at reduced feed with operator
                             watching (single-step confirm before each
                             leg); then command 1mm beyond -> must be
                             REFUSED (soft-limit oracle). Operator confirms
                             physical clearance at both ends.
V6  ORIGIN truth             ASK (measured). Both channels to a designed
                             world meet point with a generous gap (default:
                             zone center, gap 20mm, from the config like
                             val-handover). Operator MEASURES the physical
                             tool-to-tool gap and types it in. |measured -
                             designed| > tolerance (default 0.5mm, asked)
                             -> FAIL with the computed ORIGIN correction on
                             the separation-dominant axis.
V7  ORIENT truth             ASK per axis. Same world direction commanded
                             on both channels in sequence; operator reports
                             same/opposite physical direction; compared
                             against the sign pattern of the ORIENT
                             rotation matrix (e.g. lathe 0 180 0: X
                             opposite, Y same, Z opposite). Wrong pattern
                             -> FAIL naming which ORIENT would match the
                             observed pattern (the 3-option table from the
                             mirroring discussion).
V8  spindle / C convention   ASK per channel. M3 low RPM: operator confirms
                             rotation sense viewed per ISO 841 (from +Z
                             looking at the chuck face); M5. Then C-axis
                             jog +10deg: same visual sense as M3. Catches
                             the classic spindle-vs-C sign mismatch before
                             rigid tapping does.
V9  interference (MC31) live AUTO+ASK, guarded. (a) ch0 alone to zone edge
                             +margin: NO stop may fire (false-positive
                             check), retreat. (b) both channels creep into
                             the zone: protective stop MUST fire; verify
                             both channels held, error reported on both,
                             recovery by retreat works. Speeds capped, each
                             leg single-step confirmed by the operator.
VR  report                   AUTO. Markdown: per-check verdict + operator
                             answers + measured values + config fingerprint
                             + free-text notes + sign-off matrix (integrator
                             name/date). One file per run:
                             commissioning-<machine>-<date>.md

## 4. RISKS / DESIGN DECISIONS

 - Moving iron from a wizard is the danger. Rules: every motion leg is
   printed BEFORE it runs with distance/feed, requires Enter (interactive)
   or an explicit answers entry (batch); global feed cap (default 10% of
   MAX_VELOCITY, ini-overridable); abort key at every prompt wired to
   per-channel abort; V5/V9 legs are single-stepped.
 - Sim-first: every check must run and gate against the lathe sim before it
   ever sees iron. The sim IS the batch-mode oracle; iron adds only the
   operator's eyes.
 - The exclusivity/D5/AUTO_RUN traps live in session.py ONCE (they were
   re-learned twice in this project already; see memory).
 - Batch answers double as DOCUMENTATION of the expected machine behavior:
   an integrator can read lathe-sim.answers.json as a worked example.
 - qtvcp page (P3) must not fork the logic: it renders steps() of the same
   check objects. If a check needs GUI-only affordances, that is a design
   smell - fix the check.

## 5. BUILD PHASES + GATES

 P1  session.py + report.py + cli.py + V0,V1,V2,V4  (all AUTO-heavy)
     GATE: batch run vs lathe sim = all PASS; negative test: break MAP in a
     scratch ini -> V4 FAIL; unwired index-enable claim -> V2 static FAIL.
     -> DONE 2026-07-11: positive batch run V0/V1/V2/V4 all PASS on the
     lathe sim; NEG1 swapped-MAP scratch ini -> V4 oracle FAIL ("mapped
     joint 1 moved 0.0000"); NEG2 HOME_USE_INDEX claim w/o net -> V2
     static FAIL; NEG3 operator answers no -> V4 verdict FAIL. Two
     build-time findings folded in: settle must watch TARGET-JOINT
     pos-cmd (ch0 teleop jogs drive the axis planners, so
     motion.N.current-vel reads 0 mid-move) and V2 must UNHOME before
     homing (home(-1) on homed joints is a no-op, the position check
     would compare leftovers).
 P2  V3,V5,V6,V7,V8,V9
     GATE: batch run all PASS on sim (operator answers scripted); negative:
     ORIENT flipped in scratch ini -> V7 FAIL; zone removed -> V9(b) FAIL;
     limits narrowed -> V5 refusal-check FAIL.
 P3  qtvcp commissioning page in mchan_mon
     GATE: xdotool-driven page walk on :99 mirroring the P1 batch run
     (same pattern as the G5 matrix).

Each phase = its own commit(s) + gate record here.
