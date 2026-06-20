# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this repository is

A personal fork of LinuxCNC 2.10 (pre-release) with three active feature streams on top of upstream:

1. **RTCP / G43.4 / G43.5** — Native Fanuc-style tool-center-point for 5-axis mills. All three major mill families are DONE+sim-validated (table-table AB/AC/BC, head-head BCHEAD via 5axiskins, head-table BCHT via maxkins). See `RTCP-STATUS.md` for the coverage map and remaining items.
2. **GCODE_HOMING** — `[RS274NGC]GCODE_HOMING=1` makes a plain G28 trigger the real homing cycle when the machine is unhomed, then continue with the legacy waypoint+return. G28.2/G28.3 are upstream extensions (home specific joints by joint number).
3. **Multichannel** — Design-frozen (see `multichannel-DESIGN.txt`) but no code written yet. One motmod, N independent G-code channels on one EtherCAT bus (Fanuc-style). `home_permit_mask` in `src/emc/motion/homing.c` is the first multichannel hook landed.

The working sim config is `configs/sim/axis/axis_mm_scurve.ini` (S-curve planner demo, XYZ).

---

## Build (Run-In-Place)

```bash
# One-time: from src/
cd src
./autogen.sh
./configure --with-realtime=uspace
make -j$(nproc)
```

Activate the RIP environment in any shell before running commands:
```bash
. scripts/rip-environment
```

After sourcing rip-environment, `linuxcnc`, `halrun`, `halcmd`, `rs274`, and the Python `linuxcnc` module are all available.

---

## Running tests

```bash
# All integration tests:
scripts/runtests tests/

# Single test directory:
scripts/runtests tests/interp/gcode-homing/

# With verbose output (shows stdout/stderr):
scripts/runtests -v tests/interp/gcode-homing/homing-on/

# Stop on first failure:
scripts/runtests -s tests/

# Unit tests (built with meson under build/):
cd build && meson test
```

Each integration test lives in a directory containing `test.sh` (or `test.hal`, or `test`) plus an `expected` file (or a `checkresult` script). `scripts/runtests` compares stdout to `expected`.

Relevant integration tests for fork features:
- `tests/interp/gcode-homing/` — GCODE_HOMING on/off
- `tests/interp/rotation/g28/` — G28 waypoint behavior

---

## Architecture

### Execution pipeline (simplified)

```
G-code file
  → rs274ngc interpreter (src/emc/rs274ngc/)
      interp_execute.cc   dispatch loop
      interp_convert.cc   one convert_* function per G/M code
      interp_check.cc     pre-execution guards
  → canon calls (STRAIGHT_FEED, ARC_FEED, etc.)  →  emccanon.cc (src/emc/task/)
  → NML message queue
  → motmod (src/emc/motion/)   realtime servo thread
      motion.c            main RT module
      homing.c            homing state machine
      axis.c / tp/        trajectory planner (S-curve or trapezoidal)
  → kinematics module (src/emc/kinematics/)
      switchkins           identity ↔ full-kins switching (G43.4 on/off)
      trtfuncs.c           shared AC/BC math
      xyzac-trt-kins.c, xyzbc-trt-kins.c, xyzab_tdr-kins.c, 5axiskins.c, maxkins
  → HAL pins → hardware
```

### RTCP (G43.4 / G43.5) — how it works

`[RS274NGC]TCP_ORIENT_AXES` selects the topology (AB=1, AC=2, BC=3, BCHEAD=4, BCHT=5). The interpreter stores it in `settings->tcp_orient_axes` (`interp_internal.hh:798`).

- **G43.4** — switches the kinematics module from identity to full-kins via `motion.switchkins-type` HAL pin. Implemented in `convert_tool_length_offset()` (`interp_convert.cc:6620`).
- **G43.5 IJK** — calls `G43_5_VECTOR` (`interp_convert.cc:5474`). Converts the tool-axis vector (I,J,K) to rotary joint angles using the topology-specific inverse: a `switch(settings->tcp_orient_axes)` at line 5513. Then issues `SET_JOINT_POSITION` commands and activates full-kins.
- `TCP_NO_SWITCH=1` (for maxkins, which is not switchkins-capable) makes G43.4/G43.5/G49 skip the switchkins-type request.
- `TCP_CONVENTIONAL_DIRECTIONS` matches the `conventional-directions` HAL pin of trtfuncs-based kins; flips the C-angle sign in the AC/BC inverse.

### GCODE_HOMING — how it works

`[RS274NGC]GCODE_HOMING=1` sets `FEATURE_GCODE_HOMING` in `setup.feature_set` (`rs274ngc_pre.cc:965`). In `convert_home()` (`interp_convert.cc:3159`), a plain G28 calls `HOME_CYCLE_IF_UNHOMED()` before the waypoint move, only when the machine is not already homed.

### Multichannel homing hook

`home_permit_mask` (`homing.c:80`) is a bitmask of joints the current homing session is allowed to touch. Set via `set_home_permit_mask()` before issuing a Home All. All-ones (default) = legacy single-channel behavior. The `/* MCHAN */` comments throughout `homing.c` mark the multichannel-aware guard points.

### INI keys added by this fork

All under `[RS274NGC]`:

| Key | Values | Effect |
|-----|--------|--------|
| `TCP_ORIENT_AXES` | `AB`, `AC`, `BC`, `BCHEAD`, `BCHT` | Enable G43.5 for the named topology |
| `TCP_CONVENTIONAL_DIRECTIONS` | `0` / `1` | Match the kins `conventional-directions` pin (AC/BC only) |
| `TCP_NO_SWITCH` | `0` / `1` | Skip switchkins request (for non-switchable kins like maxkins) |
| `GCODE_HOMING` | `0` / `1` | Make plain G28 trigger homing when unhomed |

---

## Key files for fork features

| File | What it contains |
|------|-----------------|
| `src/emc/rs274ngc/interp_convert.cc` | `convert_home()` (G28/GCODE_HOMING), `G43_5_VECTOR` block, `convert_tool_length_offset()` (G43.4/G43.5/G49) |
| `src/emc/rs274ngc/rs274ngc_pre.cc` | INI key parsing for all fork keys (TCP_ORIENT_AXES etc., ~line 894) |
| `src/emc/rs274ngc/interp_internal.hh` | `setup` struct — `tcp_orient_axes`, `tcp_no_switch`, `tcp_conventional_directions`, `feature_set` |
| `src/emc/motion/homing.c` | Homing state machine + `home_permit_mask` multichannel hook |
| `src/emc/motion/homing.h` | `set_home_permit_mask()` declaration and HOME_* flag constants |
| `src/emc/kinematics/trtfuncs.c` | Shared AC/BC kinematic math, `con` direction parameter |
| `src/emc/kinematics/xyzac-trt-kins.c` | AC trunnion kinematics (switchkins) |
| `src/emc/kinematics/xyzbc-trt-kins.c` | BC trunnion kinematics (switchkins) |
| `RTCP-STATUS.md` | Full RTCP coverage map, done/open items, sim validation notes |
| `multichannel-DESIGN.txt` | Frozen multichannel design (no code yet) |
