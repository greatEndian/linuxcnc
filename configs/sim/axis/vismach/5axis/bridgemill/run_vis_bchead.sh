#!/bin/bash
# Visual (vismach) tip-hold demo for BCHEAD with frame capture.
cd "$(dirname "$0")"
. /home/user/linuxcnc-dev/scripts/rip-environment
export DISPLAY=:99
OUT="${TMPDIR:-/tmp}/rtcp-vis-bchead"
mkdir -p "$OUT"
rm -f 5axis.var "$OUT"/visframe_*.png
xwininfo -root -tree > "$OUT/win_before.txt" 2>/dev/null

if nc -z localhost 5007; then echo "port 5007 busy"; exit 1; fi
linuxcnc -r bchead-vis.ini &
LCNC=$!
for i in $(seq 1 80); do nc -z localhost 5007 && break; sleep 0.25; done
sleep 4
xwininfo -root -tree > "$OUT/win_after.txt" 2>/dev/null
# the vismach window is the one that appeared since launch
WIN=$(diff "$OUT/win_before.txt" "$OUT/win_after.txt" | grep "^>" | grep -oE "0x[0-9a-f]+" | head -1)
echo "vismach window: $WIN"

( for _ in $(seq 1 240); do
    v=$(halcmd getp hal_manualtoolchange.change 2>/dev/null)
    [ "$v" = "TRUE" ] && { halcmd setp hal_manualtoolchange.change_button 1; sleep 0.5; halcmd setp hal_manualtoolchange.change_button 0; }
    sleep 0.25
  done ) & ACK=$!

# continuous frames every 2 s, tagged with the current B joint angle
( n=0
  while :; do
    b=$(halcmd getp joint.3.pos-cmd 2>/dev/null || echo x)
    import -window "$WIN" "$OUT/visframe_$(printf %03d $n)_B${b}.png" 2>/dev/null
    n=$((n+1)); sleep 2
  done ) & SNAP=$!

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
    echo set mode mdi
    echo set mdi G21 G90
    echo set mdi T10 M6
    echo set mdi G43.5 H10
    echo set mdi G0 X0 Y0 Z0 B0 C0
    echo set mdi G4 P4
    echo set mdi G1 F300 X0 Y0 Z0 I0.7071 J0 K0.7071
    echo set mdi G4 P4
    echo get abs_cmd_pos
    echo get error
    echo shutdown
) | nc localhost 5007 > "$OUT/vis_bchead_rsh.txt" 2>/dev/null

kill $SNAP $ACK 2>/dev/null
wait $LCNC 2>/dev/null
tail -4 "$OUT/vis_bchead_rsh.txt"
ls "$OUT"/visframe_*.png | wc -l
