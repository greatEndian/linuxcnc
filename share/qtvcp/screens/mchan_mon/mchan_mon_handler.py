#!/usr/bin/env python3
"""mchan_mon - multi-channel GUI increment G2: the read-only qtvcp skeleton.

One qtvcp process, one window, N side-by-side channel panes, each fed from its
OWN channel's NML status buffer (design multichannelGUI.md section 1: one
process, N per-channel NML connections; the CP1 enabler proven by pverify1.py
and G1's mchanmon.py, whose polling logic this ports verbatim).

Channel discovery (CP8): the master INI (env MCHAN_INI, or the session's
INI_FILE_NAME) is parsed for [MCHAN]CHANNEL_INI entries; channel 0 uses the
master's own [EMC]NML_FILE or the compiled-in default, each secondary channel
uses its own INI's [EMC]NML_FILE. Pane construction loops over whatever list
discovery returns (CP7: generic N, nothing hard-codes 2).

Launch (sim already running):
    DISPLAY=:99 MCHAN_INI=~/cnc-dev/mchan-dev/lathe-ch0-rsh.ini \
        qtvcp -u mchan_mon_handler.py mchan_mon.ui
"""
import os
import sys

import linuxcnc
from qtpy import QtWidgets, QtCore

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from preview_widget import (PreviewWidget, extract_channel, ini_find,
                            triplet, rot_matrix, channel_box, box_overlap,
                            PALETTE)

# the compiled-in default NML path, saved before any pane overrides it
# (setting linuxcnc.nmlfile = "" clobbers the default -> "no connection")
ORIG_NML = linuxcnc.nmlfile

TASK_STATE = {1: "ESTOP", 2: "ESTOP-RESET", 3: "OFF", 4: "ON"}
TASK_MODE = {1: "MANUAL", 2: "AUTO", 3: "MDI"}
INTERP = {1: "IDLE", 2: "RUNNING", 3: "PAUSED", 4: "WAITING"}
AXES = ("X", "Y", "Z", "A", "B", "C", "U", "V", "W")


def discover_channels(master_ini):
    """[MCHAN]CHANNEL_INI-based discovery -> [(name, nmlfile-or-None), ...].

    None = use the compiled-in default NML (the normal channel-0 case)."""
    base = os.path.dirname(os.path.abspath(master_ini))
    ini = linuxcnc.ini(master_ini)
    nml0 = ini.find("EMC", "NML_FILE")
    chans = [("Channel 0", os.path.join(base, nml0) if nml0 else None,
              os.path.abspath(master_ini))]
    for i, rel in enumerate(ini.findall("MCHAN", "CHANNEL_INI"), start=1):
        cpath = os.path.join(base, rel)
        try:
            cini = linuxcnc.ini(cpath)
            nml = cini.find("EMC", "NML_FILE")
        except Exception:
            nml = None
        chans.append(("Channel %d" % i,
                      os.path.join(os.path.dirname(cpath), nml) if nml else None,
                      cpath))
    return chans


class ChannelPane(QtWidgets.QGroupBox):
    """One channel's live view; owns that channel's stat object (CP1: the
    widgets are fed from the pane's own poll, never a global singleton)."""

    def __init__(self, name, nmlfile, ini_path, idx=0):
        super().__init__(name)
        self.nmlfile = nmlfile
        self.ini_path = ini_path
        self.stat = None
        self.last_file = None
        self.preview_dirty = False
        # s6: this channel's world frame + color for the live marker
        self.origin = triplet(ini_find(ini_path, "CHANNEL", "ORIGIN"))
        self.R = rot_matrix(*triplet(ini_find(ini_path, "CHANNEL", "ORIENT")))
        # this channel's GLOBAL joint numbers: [CHANNEL]MAP = X:4 Y:5 ...
        # (motion's ownership checks and stat.homed are global-indexed);
        # the master channel has no MAP -> identity over its COORDINATES
        coords = (ini_find(ini_path, "TRAJ", "COORDINATES", "") or "").split()
        map_s = ini_find(ini_path, "CHANNEL", "MAP")
        if map_s:
            self.joints = [int(t.split(":")[1]) for t in map_s.split()]
        else:
            self.joints = list(range(len(coords) if coords else 4))
        color = ini_find(ini_path, "CHANNEL", "COLOR")
        self.rgb = ([int(v) for v in color.split(",")] if color
                    else list(PALETTE[idx % len(PALETTE)]))
        self.world_pos = None
        self._make_stat()

        outer = QtWidgets.QVBoxLayout(self)
        form = QtWidgets.QWidget()
        outer.addWidget(form)

        # G5 command strip: MDI + Run/Abort/Home, all routed to THIS channel
        mdirow = QtWidgets.QHBoxLayout()
        self.mdi_edit = QtWidgets.QLineEdit()
        self.mdi_edit.setPlaceholderText("MDI for this channel")
        self.btn_mdi = QtWidgets.QPushButton("Send")
        mdirow.addWidget(self.mdi_edit, 1)
        mdirow.addWidget(self.btn_mdi)
        outer.addLayout(mdirow)
        btnrow = QtWidgets.QHBoxLayout()
        self.btn_run = QtWidgets.QPushButton("Run")
        self.btn_abort = QtWidgets.QPushButton("Abort")
        self.btn_home = QtWidgets.QPushButton("Home")
        for b in (self.btn_run, self.btn_abort, self.btn_home):
            btnrow.addWidget(b)
        outer.addLayout(btnrow)
        self.err_label = QtWidgets.QLabel("")
        self.err_label.setWordWrap(True)
        self.err_label.setStyleSheet("color:#c00;")
        outer.addWidget(self.err_label)
        self.last_error = None      # (time, text) for the handler's banner
        self.btn_mdi.clicked.connect(self.send_mdi)
        self.mdi_edit.returnPressed.connect(self.send_mdi)
        self.btn_run.clicked.connect(self.run_program)
        self.btn_abort.clicked.connect(self.abort)
        self.btn_home.clicked.connect(self.home_all)

        self.preview = PreviewWidget()
        outer.addWidget(self.preview, 1)
        lay = QtWidgets.QFormLayout(form)
        self.state = QtWidgets.QLabel("-")
        self.mode = QtWidgets.QLabel("-")
        self.interp = QtWidgets.QLabel("-")
        self.line = QtWidgets.QLabel("-")
        f = self.state.font()
        f.setBold(True)
        self.state.setFont(f)
        self.fname = QtWidgets.QLabel("-")
        self.feed = QtWidgets.QLabel("-")
        self.vel = QtWidgets.QLabel("-")
        self.homed = QtWidgets.QLabel("-")
        lay.addRow("State:", self.state)
        lay.addRow("Mode:", self.mode)
        lay.addRow("Interp:", self.interp)
        lay.addRow("Program:", self.fname)
        lay.addRow("Line:", self.line)
        lay.addRow("Feed ovr:", self.feed)
        lay.addRow("Velocity:", self.vel)
        lay.addRow("Homed:", self.homed)
        self.pos = {}
        for ax in AXES:
            lab = QtWidgets.QLabel("0.000")
            mono = lab.font()
            mono.setStyleHint(mono.Monospace)
            mono.setFamily("monospace")
            lab.setFont(mono)
            self.pos[ax] = lab
            lay.addRow("%s:" % ax, lab)

    def _make_stat(self):
        try:
            linuxcnc.nmlfile = self.nmlfile if self.nmlfile else ORIG_NML
            self.stat = linuxcnc.stat()
            # G5 (CP4): this pane's OWN command channel, same NML binding
            self.cmd = linuxcnc.command()
            # G6 (MC30): this pane's OWN error ring - channel-scoped errors
            # (refusals, waiting-M reports) surface HERE and nowhere else
            self.err = linuxcnc.error_channel()
        except Exception:
            self.stat = None
            self.cmd = None
            self.err = None

    # ---- G5 command handlers: every action goes to THIS pane's cmd ----
    def _cmd_mode(self, mode):
        if not self.cmd:
            return False
        self.cmd.mode(mode)
        self.cmd.wait_complete(1.0)
        return True

    def send_mdi(self):
        text = self.mdi_edit.text().strip()
        if not text or not self.cmd:
            return
        self._cmd_mode(linuxcnc.MODE_MDI)
        self.cmd.mdi(text)

    def run_program(self):
        if not self.cmd:
            return
        self._cmd_mode(linuxcnc.MODE_AUTO)
        self.cmd.auto(linuxcnc.AUTO_RUN, 0)

    def abort(self):
        """Per-channel soft stop (D5: abort is NOT machine-global)."""
        if self.cmd:
            self.cmd.abort()

    def home_all(self):
        """Home THIS channel's joints: ONE home(-1) (home-all). Motion scopes
        it to this channel via the mchan home permit mask, so nothing crosses
        channels. (Two wrong attempts, both caught by matrix row M2/M3: a
        loop over stat.joints homed the OTHER channel's joints; a rapid-fire
        per-joint loop tripped the ownership check AND wedged homing.c's
        sequence-active flag, locking the other channel out entirely.)"""
        if not self.cmd:
            return
        self._cmd_mode(linuxcnc.MODE_MANUAL)
        self.cmd.home(-1)

    def refresh(self):
        if self.stat is None:
            self._make_stat()
            if self.stat is None:
                self.state.setText("(no connection)")
                self.state.setStyleSheet("color:#888;")
                return False
        try:
            self.stat.poll()
        except Exception:
            self.state.setText("(no connection)")
            self.state.setStyleSheet("color:#888;")
            self.stat = None
            return False
        st = self.stat.task_state
        self.state.setText(TASK_STATE.get(st, str(st)))
        self.state.setStyleSheet("color:#0a0;" if st == 4 else "color:#c00;")
        self.mode.setText(TASK_MODE.get(self.stat.task_mode, "-"))
        self.interp.setText(INTERP.get(self.stat.interp_state, "-"))
        self.line.setText(str(self.stat.motion_line))
        self.fname.setText(os.path.basename(self.stat.file) if self.stat.file else "-")
        if self.stat.file != self.last_file:
            self.last_file = self.stat.file
            if self.stat.file and os.path.exists(self.stat.file):
                self.preview.set_channels(
                    [extract_channel(self.ini_path, self.stat.file)])
            else:
                self.preview.set_channels([])
            self.preview_dirty = True
        self.feed.setText("%.0f %%" % (self.stat.feedrate * 100.0))
        self.vel.setText("%.1f" % self.stat.current_vel)
        self.homed.setText("".join(
            "Y" if self.stat.homed[j] else "n" for j in self.joints))
        p = self.stat.position
        for i, ax in enumerate(AXES):
            self.pos[ax].setText("%8.3f" % p[i])
        # G6: drain this channel's error ring into the pane's error line
        if self.err:
            # drain the whole ring each tick (one-at-a-time polling delivered
            # queued messages ticks late, showing stale reports); keep newest
            while True:
                try:
                    e = self.err.poll()
                except Exception:
                    e = None
                if not e:
                    break
                _kind, text = e
                self.err_label.setText(text)
                self.last_error = (QtCore.QTime.currentTime(), text)
        # s6: local XYZ -> world frame, marker on this pane's own preview
        self.world_pos = [
            self.R[k][0] * p[0] + self.R[k][1] * p[1] + self.R[k][2] * p[2]
            + self.origin[k] for k in range(3)]
        self.preview.set_live([(self.world_pos, self.rgb)])
        return True


class HandlerClass:
    def __init__(self, halcomp, widgets, paths):
        self.hal = halcomp
        self.w = widgets
        self.paths = paths
        self.panes = []

    def initialized__(self):
        master = os.environ.get("MCHAN_INI") or os.environ.get("INI_FILE_NAME")
        if not master or not os.path.exists(master):
            self.w.statusbar.setText(
                "no master INI (set MCHAN_INI or launch inside a session)")
            return
        chans = discover_channels(master)
        for idx, (name, nml, ini_path) in enumerate(chans):
            pane = ChannelPane(name, nml, ini_path, idx)
            self.panes.append(pane)
            self.w.channel_row.addWidget(pane)
        self.w.statusbar.setText(
            "%d channel(s) from %s" % (len(chans), os.path.basename(master)))

        # G5 (CP3 / design D5): Estop is machine-GLOBAL - one button, all
        # channels; machine-on likewise. Soft abort stays per-channel (pane).
        grow = QtWidgets.QHBoxLayout()
        self.btn_estop = QtWidgets.QPushButton("ESTOP (all)")
        self.btn_estop.setStyleSheet(
            "background:#b00; color:white; font-weight:bold;")
        self.btn_on = QtWidgets.QPushButton("Machine On (all)")
        self.btn_abort_all = QtWidgets.QPushButton("Abort (all)")
        self.btn_commission = QtWidgets.QPushButton("Commissioning…")
        grow.addWidget(self.btn_estop)
        grow.addWidget(self.btn_on)
        grow.addWidget(self.btn_abort_all)
        grow.addStretch(1)
        grow.addWidget(self.btn_commission)
        self.w.outer.insertLayout(1, grow)
        self.btn_estop.clicked.connect(self._estop_all)
        self.btn_on.clicked.connect(self._machine_on_all)
        self.btn_abort_all.clicked.connect(self._abort_all)
        self.btn_commission.clicked.connect(self._open_commissioning)
        self._master_ini = master
        self._commission_dlg = None

        # G4b: preview mode toggle (mockup: "Preview: ( )Per-channel (*)Combined")
        row = QtWidgets.QHBoxLayout()
        row.addWidget(QtWidgets.QLabel("Preview:"))
        self.rb_per = QtWidgets.QRadioButton("Per-channel")
        self.rb_per.setChecked(True)
        self.rb_comb = QtWidgets.QRadioButton("Combined")
        row.addWidget(self.rb_per)
        row.addWidget(self.rb_comb)
        row.addStretch(1)
        self.w.outer.insertLayout(2, row)
        self.combined = PreviewWidget()
        # G4c: envelope boxes + overlap zones + origin triads (world frame)
        boxes, triads = [], []
        for idx, (_n, _nml, ini_path) in enumerate(chans):
            origin = triplet(ini_find(ini_path, "CHANNEL", "ORIGIN"))
            orient = triplet(ini_find(ini_path, "CHANNEL", "ORIENT"))
            color = ini_find(ini_path, "CHANNEL", "COLOR")
            rgb = ([int(v) for v in color.split(",")] if color
                   else list(PALETTE[idx % len(PALETTE)]))
            boxes.append((channel_box(ini_path, origin, orient), rgb))
            triads.append((origin, rot_matrix(*orient)))
        zones = []
        for i in range(len(boxes)):
            for j in range(i + 1, len(boxes)):
                ov = box_overlap(boxes[i][0], boxes[j][0])
                if ov:
                    zones.append(ov)
        self.combined.set_context(boxes, zones, triads)
        self.combined.warn_radius = float(
            ini_find(master, "MCHAN", "PROXIMITY_WARN", "25") or 25)
        self.combined.hide()
        # title(0), global row(1), toggle row(2), channel_row(3) -> combined at 4
        self.w.outer.insertWidget(4, self.combined, 1)
        self.rb_comb.toggled.connect(self._preview_mode)

        # G6 (CP9 / design D4): waiting-M rendezvous banner - "who waits
        # for whom", fed by the motion-side report on the waiter's own ring
        self.wm_banner = QtWidgets.QLabel("")
        self.wm_banner.setStyleSheet(
            "color:#e80; font-weight:bold; background:#222; padding:2px;")
        self.wm_banner.hide()
        self.w.outer.insertWidget(self.w.outer.count() - 1, self.wm_banner)

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self._tick)
        self.timer.start(150)

    def _estop_all(self):
        """D5: the estop chain is machine-global - stop EVERY channel."""
        for pane in self.panes:
            if pane.cmd:
                pane.cmd.state(linuxcnc.STATE_ESTOP)

    def _machine_on_all(self):
        """D5 power rule: release estop on ALL channels first, then on."""
        for pane in self.panes:
            if pane.cmd:
                pane.cmd.state(linuxcnc.STATE_ESTOP_RESET)
                pane.cmd.wait_complete(1.0)
        for pane in self.panes:
            if pane.cmd:
                pane.cmd.state(linuxcnc.STATE_ON)
                pane.cmd.wait_complete(2.0)

    def _abort_all(self):
        for pane in self.panes:
            pane.abort()

    def _open_commissioning(self):
        """F2 P3: the commissioning verificator page - drives the same
        mchan_verify check objects the CLI wizard runs."""
        from mchan_verify.verify_page import CommissioningDialog
        if self._commission_dlg is None:
            self._commission_dlg = CommissioningDialog(
                self._master_ini, parent=self.w, default_nml=ORIG_NML)
        self._commission_dlg.show()
        self._commission_dlg.raise_()

    def _preview_mode(self, combined_on=None):
        combined_on = self.rb_comb.isChecked()
        for pane in self.panes:
            pane.preview.setVisible(not combined_on)
        if combined_on:
            self.combined.set_channels(
                [c for pane in self.panes for c in pane.preview.channels])
        self.combined.setVisible(combined_on)

    def _tick(self):
        ok = sum(1 for p in self.panes if p.refresh())
        if self.rb_comb.isChecked():
            if any(p.preview_dirty for p in self.panes):
                self._preview_mode()
            self.combined.set_live(
                [(p.world_pos, p.rgb) for p in self.panes if p.world_pos])
        for p in self.panes:
            p.preview_dirty = False
        # G6: show the newest waiting-M report while its channel still waits
        banner = ""
        for pane in self.panes:
            if pane.last_error and "waiting for ch" in pane.last_error[1]:
                st = pane.stat.interp_state if pane.stat else 0
                if st and st != linuxcnc.INTERP_IDLE:
                    banner = "RENDEZVOUS: " + pane.last_error[1]
                else:
                    pane.last_error = None      # released - drop the report
        self.wm_banner.setText(banner)
        self.wm_banner.setVisible(bool(banner))
        self.w.statusbar.setText(
            "polling %d/%d channel(s) live | preview: %s" % (
                ok, len(self.panes),
                "combined" if self.rb_comb.isChecked() else "per-channel"))


def get_handlers(halcomp, widgets, paths):
    return [HandlerClass(halcomp, widgets, paths)]
