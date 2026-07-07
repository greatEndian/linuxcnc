#!/usr/bin/env python3
"""mchan_workspace - TWO stock AXIS UIs merged into ONE window, with a
toggle that switches the 3D view between the per-channel AXIS backplots
and a COMMON workspace scene (both channels' toolpaths in the machine
world frame + envelopes + handover zone + live tool markers/distance).

AXIS stays 100% stock: each channel's own AXIS process is X11-embedded
(QWindow.fromWinId -> createWindowContainer; proven by embed_spike.py:
menus, keyboard, resize and live status all work while embedded).

Run AFTER a session whose channels use DISPLAY=axis is up, on the same
X display:

    DISPLAY=... MCHAN_INI=~/cnc-dev/mchan-dev/lathe-ch0.ini \
        python3 mchan_workspace.py
"""
import os
import signal
import subprocess
import sys

import linuxcnc
from qtpy import QtCore, QtGui, QtWidgets

# the preview components live alongside this file
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from mchan_mon_handler import discover_channels, ORIG_NML
from preview_widget import (PreviewWidget, extract_channel, ini_find,
                            triplet, rot_matrix, channel_box, box_overlap,
                            PALETTE)


def find_axis_window(machine_name):
    """X window id of the stock AXIS window titled '... on <machine>'."""
    out = subprocess.run(
        "xwininfo -root -tree | grep -i 'AXIS .* on %s' | head -1"
        % machine_name, shell=True, capture_output=True, text=True
    ).stdout.strip()
    return int(out.split()[0], 16) if out else None


class ChannelLive:
    """Slim per-channel poller: stat only (commands stay in AXIS)."""

    def __init__(self, nml, ini_path, idx):
        self.ini_path = ini_path
        linuxcnc.nmlfile = nml if nml else ORIG_NML
        self.stat = linuxcnc.stat()
        self.origin = triplet(ini_find(ini_path, "CHANNEL", "ORIGIN"))
        self.R = rot_matrix(*triplet(ini_find(ini_path, "CHANNEL", "ORIENT")))
        color = ini_find(ini_path, "CHANNEL", "COLOR")
        self.rgb = ([int(v) for v in color.split(",")] if color
                    else list(PALETTE[idx % len(PALETTE)]))
        self.last_file = None
        self.chan_data = None

    def poll(self):
        try:
            self.stat.poll()
        except Exception:
            return None, False
        changed = False
        if self.stat.file != self.last_file:
            self.last_file = self.stat.file
            self.chan_data = (extract_channel(self.ini_path, self.stat.file)
                              if self.stat.file and os.path.exists(self.stat.file)
                              else None)
            changed = True
        p = self.stat.position
        world = [self.R[k][0] * p[0] + self.R[k][1] * p[1]
                 + self.R[k][2] * p[2] + self.origin[k] for k in range(3)]
        return world, changed


_EMBEDDED_WIDS = []


def main():
    master = os.environ.get("MCHAN_INI")
    if not master or not os.path.exists(master):
        print("set MCHAN_INI to the master ini of the running session")
        return 1
    app = QtWidgets.QApplication(sys.argv)
    chans = discover_channels(master)

    win = QtWidgets.QMainWindow()
    win.setWindowTitle("MULTI-CHANNEL WORKSPACE")
    central = QtWidgets.QWidget()
    outer = QtWidgets.QVBoxLayout(central)

    # top bar: the toggle the whole exercise is about
    bar = QtWidgets.QHBoxLayout()
    bar.addWidget(QtWidgets.QLabel("3D preview:"))
    rb_axis = QtWidgets.QRadioButton("Per-channel (AXIS)")
    rb_axis.setChecked(True)
    rb_common = QtWidgets.QRadioButton("Common workspace")
    bar.addWidget(rb_axis)
    bar.addWidget(rb_common)
    bar.addStretch(1)
    outer.addLayout(bar)

    split = QtWidgets.QSplitter(QtCore.Qt.Vertical)
    axis_row = QtWidgets.QSplitter(QtCore.Qt.Horizontal)

    # embed each channel's stock AXIS
    embedded = 0
    for _name, _nml, ini_path in chans:
        machine = ini_find(ini_path, "EMC", "MACHINE") or ""
        wid = find_axis_window(machine)
        if wid is None:
            lab = QtWidgets.QLabel("(no AXIS window for '%s')" % machine)
            lab.setAlignment(QtCore.Qt.AlignCenter)
            axis_row.addWidget(lab)
            continue
        foreign = QtGui.QWindow.fromWinId(wid)
        cont = QtWidgets.QWidget.createWindowContainer(foreign)
        cont.setMinimumSize(500, 500)
        cont.setFocusPolicy(QtCore.Qt.StrongFocus)
        axis_row.addWidget(cont)
        _EMBEDDED_WIDS.append(wid)
        embedded += 1
        print("embedded AXIS of '%s' (0x%x)" % (machine, wid))
    split.addWidget(axis_row)

    # the common workspace pane (hidden in per-channel mode)
    common = PreviewWidget()
    lives = [ChannelLive(nml, ini_path, i)
             for i, (_n, nml, ini_path) in enumerate(chans)]
    boxes, triads = [], []
    for lv in lives:
        boxes.append((channel_box(lv.ini_path, lv.origin,
                                  triplet(ini_find(lv.ini_path, "CHANNEL",
                                                   "ORIENT"))), lv.rgb))
        triads.append((lv.origin, lv.R))
    zones = []
    for i in range(len(boxes)):
        for j in range(i + 1, len(boxes)):
            ov = box_overlap(boxes[i][0], boxes[j][0])
            if ov:
                zones.append(ov)
    common.set_context(boxes, zones, triads)
    common.warn_radius = float(
        ini_find(master, "MCHAN", "PROXIMITY_WARN", "25") or 25)
    common.hide()
    split.addWidget(common)
    split.setSizes([700, 0])
    outer.addWidget(split, 1)

    def preview_mode():
        on = rb_common.isChecked()
        if on:
            # re-collect: extractions may have happened while hidden
            common.set_channels([lv.chan_data for lv in lives
                                 if lv.chan_data])
        common.setVisible(on)
        split.setSizes([560, 440] if on else [1000, 0])
    rb_common.toggled.connect(preview_mode)

    def tick():
        markers, dirty = [], False
        for lv in lives:
            world, changed = lv.poll()
            dirty = dirty or changed
            if world:
                markers.append((world, lv.rgb))
        if common.isVisible():
            if dirty:
                common.set_channels([lv.chan_data for lv in lives
                                     if lv.chan_data])
            common.set_live(markers)
    timer = QtCore.QTimer()
    timer.timeout.connect(tick)
    timer.start(200)

    # LIFECYCLE: if the container is destroyed with AXIS still reparented
    # inside, X destroys the AXIS windows too -> AXIS exits -> the whole
    # LinuxCNC session shuts down. Give the windows back to the root window
    # on close so closing the workspace leaves the session running.
    axis_wids = [w for w in _EMBEDDED_WIDS]

    class _Win(QtWidgets.QMainWindow):
        def closeEvent(self, ev):
            for w in axis_wids:
                subprocess.run("xdotool windowreparent 0x%x %d" % (w, root_id),
                               shell=True)
            ev.accept()

    root_id = int(subprocess.run(
        "xwininfo -root | awk '/Window id/ {print $4}'", shell=True,
        capture_output=True, text=True).stdout.strip(), 16)
    new_win = _Win()
    new_win.setWindowTitle(win.windowTitle())
    win = new_win
    win.setCentralWidget(central)

    # graceful quit paths that all run closeEvent (and thus the reparent):
    # Ctrl+Q shortcut, and SIGTERM/SIGINT (Qt would otherwise die without
    # cleanup and take the embedded AXIS windows - and the session - along)
    QtWidgets.QShortcut(QtGui.QKeySequence("Ctrl+Q"), win, win.close)
    signal.signal(signal.SIGTERM, lambda *_a: QtCore.QTimer.singleShot(0, win.close))
    signal.signal(signal.SIGINT, lambda *_a: QtCore.QTimer.singleShot(0, win.close))

    win.resize(1800, 1100)
    win.show()
    print("WORKSPACE UP (%d AXIS embedded)" % embedded)
    sys.stdout.flush()
    return app.exec_()


if __name__ == "__main__":
    sys.exit(main())
