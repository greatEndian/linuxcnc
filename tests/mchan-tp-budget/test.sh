#!/bin/bash
# MCHAN MC1: trajectory-planner CPU-budget measurement (multichannel gate).
# Runs a dense short-segment program through the S-curve planner at a 1 ms
# servo period and reports motion.tp-time-last/max (ns per servo cycle).
# The multichannel budget is roughly N x this cost + fixed overhead.

set -e

rm -f dense.ngc gcode-output

# deterministic dense program: 2000 zig-zag segments, 0.01 inch each
{
    echo "G20 G90 G64 P0.001"
    echo "F120"
    echo "G1 X0 Y0"
    python3 -c "
x = 0.0
for i in range(2000):
    x += 0.01
    y = 0.05 if (i % 2) else 0.0
    print(f'G1 X{x:.3f} Y{y:.3f}')
"
    echo "M2"
} > dense.ngc

if ! command -v nc; then echo "E: nc missing"; exit 1; fi
if nc -z localhost 5007; then echo "E: port 5007 busy"; exit 1; fi

linuxcnc -r test.ini &

# wait for linuxcncrsh
TOGO=80
while [ $TOGO -gt 0 ]; do
    if nc -z localhost 5007; then break; fi
    sleep 0.25
    TOGO=$((TOGO - 1))
done
if [ $TOGO -eq 0 ]; then echo "connection to linuxcncrsh timed out"; exit 1; fi

(
    echo "hello EMC mt 1.0"
    echo "set enable EMCTOO"
    echo "set mode manual"
    echo "set estop off"
    echo "set machine on"
    echo "set mode auto"
    echo "set open dense.ngc"
    echo "set run"
    # keep the connection open while the program runs
    sleep 45
    echo "shutdown"
) | nc localhost 5007 > /dev/null &
NC_PID=$!

# wait for motion to start, then to finish (poll HAL, not the rsh socket)
TOGO=120
while [ $TOGO -gt 0 ]; do
    INPOS=$(halcmd getp motion.in-position 2>/dev/null || echo TRUE)
    if [ "$INPOS" = "FALSE" ]; then break; fi
    sleep 0.25
    TOGO=$((TOGO - 1))
done
echo "I: motion started (TOGO=$TOGO)"

TOGO=160
while [ $TOGO -gt 0 ]; do
    INPOS=$(halcmd getp motion.in-position 2>/dev/null || echo TRUE)
    if [ "$INPOS" = "TRUE" ]; then break; fi
    sleep 0.25
    TOGO=$((TOGO - 1))
done
echo "I: motion finished (TOGO=$TOGO)"

# the measurement
TPMAX=$(halcmd getp motion.tp-time-max 2>/dev/null || echo -1)
TPLAST=$(halcmd getp motion.tp-time-last 2>/dev/null || echo -1)
echo "tp-time-max-ns=$TPMAX"
echo "tp-time-last-ns=$TPLAST"

kill $NC_PID 2>/dev/null || true
# ask the session to shut down cleanly
(echo "hello EMC mt 1.0"; echo "set enable EMCTOO"; echo "shutdown") | nc -w 2 localhost 5007 > /dev/null 2>&1 || true
wait %1 2>/dev/null || true
exit 0
