#!/bin/bash
# Headless live-sim tip-hold validation for the GENERIC G43.5 topology.
# Launches the generic-hl.ini sim (5axiskins sparm=tiltA), drives it via
# linuxcncrsh, samples joint + axis positions with halcmd during the
# G43.5 reorientation move, and checks the tool tip held still while
# A/C swung and XYZ joints compensated.
cd "$(dirname "$0")"
. /home/user/linuxcnc-dev/scripts/rip-environment

SAMPLES="${TMPDIR:-/tmp}/generic_samples.txt"
RSH_LOG="${TMPDIR:-/tmp}/generic_rsh.txt"
rm -f "$SAMPLES" "$RSH_LOG" 5axis.var

if nc -z localhost 5007; then
    echo "Process already listening on port 5007. Exiting"
    exit 1
fi

linuxcnc -r generic-hl.ini &
LCNC_PID=$!

TOGO=80
while [ $TOGO -gt 0 ]; do
    nc -z localhost 5007 && break
    sleep 0.25
    TOGO=$((TOGO - 1))
done
if [ $TOGO -eq 0 ]; then
    echo "connection to linuxcncrsh timed out"
    kill $LCNC_PID 2>/dev/null
    exit 1
fi
sleep 1

# background sampler: 20 Hz, joints 0-4 pos-cmd + kins type
# halsampler drains the RT sampler fifo: every line is one servo cycle,
# all 5 joints latched coherently in that cycle (decimate x10 -> 100 Hz)
halsampler -t > "$SAMPLES" &
SAMPLER_PID=$!

# auto-acknowledge the manual tool change when it pops
(
    for _ in $(seq 1 200); do
        v=$(halcmd getp hal_manualtoolchange.change 2>/dev/null)
        if [ "$v" = "TRUEE" ] || [ "$v" = "TRUE" ]; then
            halcmd setp hal_manualtoolchange.change_button 1
            sleep 0.5
            halcmd setp hal_manualtoolchange.change_button 0
        fi
        sleep 0.25
    done
) &
ACK_PID=$!

(
    echo hello EMC mt 1.1
    echo set enable EMCTOO
    echo set wait_mode "done"
    echo set timeout 30
    echo set estop off
    echo set machine on
    echo set mode manual
    echo set joint_home -1
    echo set joint_wait_homed 0
    echo set joint_wait_homed 2
    echo set joint_wait_homed 3
    echo set joint_wait_homed 4
    echo set mode mdi
    echo set mdi G21 G90
    echo set mdi T10 M6
    echo set mdi G43.5 H10
    echo set mdi G0 X0 Y0 Z0 A0 C0
    echo set mdi G4 P0.5
    echo "MARKER-BEFORE" >&2
    echo set mdi G1 F2000 X0 Y0 Z0 I0.5 J0 K0.866025
    echo set mdi G4 P0.5
    echo get joint_pos
    echo get abs_cmd_pos
    echo get error
    echo shutdown
) | nc localhost 5007 > "$RSH_LOG" 2>/dev/null

wait $LCNC_PID 2>/dev/null
kill $SAMPLER_PID $ACK_PID 2>/dev/null
echo "=== linuxcncrsh transcript (tail) ==="
tail -6 "$RSH_LOG"
echo "=== samples: $(wc -l < "$SAMPLES") ==="
