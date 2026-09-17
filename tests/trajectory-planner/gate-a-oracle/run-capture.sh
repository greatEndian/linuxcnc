#!/bin/bash
# Capture one commanded-motion trace from a real LinuxCNC run.
#
#   run-capture.sh <case> <planner_type> <output-file>
#     case         straight | circle | tinyblocks
#     planner_type 0 (trapezoidal) | 1 (S-curve)
#
# The config is copied to a temporary work directory and run there, so the
# repository stays clean and every run is hermetic.
set -eu
here=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
repo=$(CDPATH= cd -- "$here/../../.." && pwd)

case_name=${1:?usage: run-capture.sh <case> <planner_type> <output>}
planner=${2:?usage: run-capture.sh <case> <planner_type> <output>}
output=${3:?usage: run-capture.sh <case> <planner_type> <output>}
program="$here/gcode/$case_name.ngc"
[ -f "$program" ] || { echo "no such case: $program" >&2; exit 2; }

if pgrep -x rtapi_app >/dev/null 2>&1 || pgrep -x milltask >/dev/null 2>&1; then
    echo "refusing to start: a LinuxCNC session is already running" >&2
    exit 3
fi

work=$(mktemp -d "${TMPDIR:-/tmp}/gate-a.XXXXXX")
trap 'rm -rf "$work"' EXIT HUP INT TERM
cp "$here/configs/oracle.ini" "$here/configs/oracle.hal" "$here/configs/driver.py" "$work/"
cp "$program" "$work/"
chmod +x "$work/driver.py"
sed -i "s/^PLANNER_TYPE = .*/PLANNER_TYPE = $planner/" "$work/oracle.ini"

# Optional sweep overrides, used by the constraint-3 experiments.
if [ -n "${ORACLE_JERK:-}" ]; then
    sed -i "s/^MAX_LINEAR_JERK = .*/MAX_LINEAR_JERK = $ORACLE_JERK/;s/^MAX_JERK = .*/MAX_JERK = $ORACLE_JERK/" "$work/oracle.ini"
fi
if [ -n "${ORACLE_BASE_NS:-}" ]; then
    sed -i "s/^BASE_PERIOD = .*/BASE_PERIOD = $ORACLE_BASE_NS/" "$work/oracle.ini"
fi
if [ -n "${ORACLE_SERVO_NS:-}" ]; then
    sed -i "s/^SERVO_PERIOD = .*/SERVO_PERIOD = $ORACLE_SERVO_NS/" "$work/oracle.ini"
fi

output=$(readlink -f "$output")
mkdir -p "$(dirname "$output")"

export ORACLE_PROGRAM="$work/$case_name.ngc"
export ORACLE_OUT="$output"
set +u   # rip-environment references unset variables
# shellcheck disable=SC1091
. "$repo/scripts/rip-environment" >/dev/null
set -u

echo "case=$case_name planner_type=$planner servo=1ms -> $output"
linuxcnc -r "$work/oracle.ini" >"$work/linuxcnc.log" 2>&1 || {
    echo "linuxcnc exited nonzero; log follows" >&2
    tail -40 "$work/linuxcnc.log" >&2
    exit 4
}
cp "$work/linuxcnc.log" "$output.log" 2>/dev/null || true
grep -c . "$output" | sed 's/^/samples: /'
