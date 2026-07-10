#!/bin/bash
# run-workspace.sh - one-command launcher for the merged multi-channel AXIS
# workspace: starts a 2-channel session (two stock AXIS windows), waits for
# both, then pulls them into the single MULTI-CHANNEL WORKSPACE window with
# the Per-channel/Common 3D preview toggle (mchan_workspace.py, this dir).
#
# usage:  run-workspace.sh <master.ini>
#   <master.ini> = the channel-0 ini of a multichannel config whose
#   channels use [DISPLAY]DISPLAY = axis.
#
# Close the workspace window (or Ctrl+Q) when done - the AXIS windows pop
# back out and you'll be asked whether to shut the whole session down.
# (Never SIGKILL the workspace: that skips the window hand-back and takes
# AXIS - and with it the session - down.)
DIR="$(cd "$(dirname "$0")" && pwd)"

MASTER="${1:-}"
[ -n "$MASTER" ] && [ -f "$MASTER" ] || {
    echo "usage: $(basename "$0") <master.ini>   (file must exist)"; exit 1; }
MASTER="$(cd "$(dirname "$MASTER")" && pwd)/$(basename "$MASTER")"

# in a run-in-place tree this script sits in share/qtvcp/screens/mchan_mon;
# source rip-environment from that tree unless one is already active
if [ -z "${EMC2_HOME:-}" ]; then
    RIP="$(cd "$DIR/../../../.." && pwd)"
    # rip-environment is not set-u-clean - source it BEFORE enabling set -u
    # shellcheck disable=SC1091
    source "$RIP/scripts/rip-environment"
fi
set -u

log() { echo "run-workspace: $*"; }

# --- clean slate ----------------------------------------------------------
for nm in linuxcncrsh milltask linuxcncsvr rtapi_app tpmod; do
    pgrep -x "$nm" 2>/dev/null | xargs -r kill 2>/dev/null
done
sleep 1
halrun -U >/dev/null 2>&1
ipcs -m | awk '$6==0 {print $2}' | while read -r id; do ipcrm -m "$id" 2>/dev/null; done

# --- session: two channel stacks, each with its own stock AXIS -------------
SESSION_LOG="${TMPDIR:-/tmp}/run-workspace-session.log"
log "starting session: $(basename "$MASTER") (log: $SESSION_LOG)"
linuxcnc "$MASTER" > "$SESSION_LOG" 2>&1 &
SIM=$!
trap 'log "interrupted - shutting session down"; kill $SIM 2>/dev/null; exit 130' INT

# --- wait for BOTH AXIS windows --------------------------------------------
log "waiting for both AXIS windows..."
for _i in $(seq 1 60); do
    kill -0 $SIM 2>/dev/null || { log "FATAL: session died during startup:";
                                  tail -20 "$SESSION_LOG"; exit 1; }
    n=$(xwininfo -root -tree 2>/dev/null | grep -c '("axis[^"]*" "Axis")')
    [ "$n" -ge 2 ] && break
    sleep 2
done
n=$(xwininfo -root -tree 2>/dev/null | grep -c '("axis[^"]*" "Axis")')
[ "$n" -ge 2 ] || { log "FATAL: only $n AXIS window(s) after 2 min"; exit 1; }
log "both AXIS up - merging into the workspace"
sleep 1   # let AXIS finish drawing before the reparent

# --- the merged window (blocks until you close it) --------------------------
MCHAN_INI="$MASTER" python3 "$DIR/mchan_workspace.py"
log "workspace closed - AXIS windows are back on the desktop"

# --- offer a full shutdown ---------------------------------------------------
if [ -t 0 ]; then
    # default = YES: "I closed it" should mean closed (a lingering session
    # once kept a wedged homing latch alive across a supposed restart)
    read -r -p "run-workspace: shut the LinuxCNC session down too? [Y/n] " ans
    if [ "${ans,,}" = "n" ]; then
        log "session left running (pid $SIM)"
    else
        kill $SIM 2>/dev/null
        log "session shutdown requested"
    fi
else
    log "session left running (pid $SIM); stop it with: kill $SIM"
fi
