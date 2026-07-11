"""mchan_verify.verify_page - the qtvcp commissioning page (F2 phase P3).

Renders the SAME check objects the CLI wizard runs (checks.load_all()):
each Info becomes a log line, each Ask becomes a Yes/No or number dialog.
The driver loop below mirrors cli.run_check exactly - the checks are the
logic, this is only the front end (design rule: the page must not fork the
logic). Motion legs run inline in the GUI thread; the Ask dialogs' own
event loops keep the window painted while an operator answers.

Embed (from the mchan_mon handler):
    from mchan_verify.verify_page import CommissioningDialog
    dlg = CommissioningDialog(master_ini, parent=self.w)
    dlg.show()
"""
import json
import os
import time

from qtpy import QtCore, QtWidgets

from .checks import Ask, CheckFailed, CheckSkipped, Info, load_all
from .report import FAIL, PASS, SKIP, Report
from .session import Session

STATUS_COLOR = {
    "pending": "#888", "running": "#08e",
    PASS: "#0a0", FAIL: "#c00", SKIP: "#a80",
}


class GuiOperator:
    """answers Ask steps with dialogs (modal - their own event loop keeps
    the main window responsive while the operator decides)."""

    def __init__(self, parent):
        self.parent = parent

    def answer(self, ask):
        if ask.kind == "measure":
            val, ok = QtWidgets.QInputDialog.getDouble(
                self.parent, "Commissioning — measurement", ask.text,
                0.0, -1e6, 1e6, 4)
            if not ok:
                raise CheckFailed("operator cancelled the measurement")
            return val
        # yesno / confirm
        box = QtWidgets.QMessageBox(self.parent)
        box.setWindowTitle("Commissioning — confirm")
        box.setText(ask.text)
        if ask.kind == "confirm":
            box.setStandardButtons(QtWidgets.QMessageBox.Ok
                                   | QtWidgets.QMessageBox.Abort)
            r = box.exec_()
            if r == QtWidgets.QMessageBox.Abort:
                raise CheckFailed("operator aborted")
            return True
        box.setStandardButtons(QtWidgets.QMessageBox.Yes
                               | QtWidgets.QMessageBox.No)
        return box.exec_() == QtWidgets.QMessageBox.Yes


class BatchOperator:
    """answers Ask steps from a JSON answers file instead of dialogs. Parity
    with the CLI wizard's --batch mode: lets an integrator pre-script the run
    and is how the qtvcp page is gated headlessly (env MCHAN_VERIFY_ANSWERS).
    Falls back to raising if an id is missing so a stale answers file can't
    silently pass."""

    def __init__(self, path, log):
        with open(path) as f:
            self.book = json.load(f)
        self._log = log

    def answer(self, ask):
        if ask.id not in self.book:
            raise CheckFailed("no scripted answer for '%s' in the answers file"
                              % ask.id)
        v = self.book[ask.id]
        self._log("   (batch answer %s = %r)" % (ask.id, v))
        return v


class CommissioningDialog(QtWidgets.QDialog):
    def __init__(self, master_ini, parent=None, default_nml=None):
        super().__init__(parent)
        self.master_ini = master_ini
        # the host (mchan_mon) captured the pristine linuxcnc.nmlfile before its
        # panes retargeted the global; pass it so the check's channel-0
        # connection binds to the real channel 0, not the last pane's buffer.
        self.default_nml = default_nml
        self.setWindowTitle("mchan commissioning verificator")
        self.resize(560, 620)
        self.session = None
        self.report = None
        self._rows = {}          # check id -> status QLabel

        outer = QtWidgets.QVBoxLayout(self)
        outer.addWidget(QtWidgets.QLabel(
            "<b>Commissioning verificator</b> — %s"
            % os.path.basename(master_ini)))

        oprow = QtWidgets.QHBoxLayout()
        oprow.addWidget(QtWidgets.QLabel("Integrator:"))
        self.operator = QtWidgets.QLineEdit()
        oprow.addWidget(self.operator, 1)
        outer.addLayout(oprow)

        # the check list with live status
        self.checks = load_all()
        grid = QtWidgets.QGridLayout()
        for r, c in enumerate(self.checks):
            grid.addWidget(QtWidgets.QLabel(c.id), r, 0)
            grid.addWidget(QtWidgets.QLabel(c.title), r, 1)
            st = QtWidgets.QLabel("pending")
            st.setStyleSheet("color:%s;" % STATUS_COLOR["pending"])
            grid.addWidget(st, r, 2)
            self._rows[c.id] = st
        grid.setColumnStretch(1, 1)
        outer.addLayout(grid)

        self.log = QtWidgets.QPlainTextEdit()
        self.log.setReadOnly(True)
        outer.addWidget(self.log, 1)

        self.overall = QtWidgets.QLabel("")
        self.overall.setStyleSheet("font-weight:bold;")
        outer.addWidget(self.overall)

        btnrow = QtWidgets.QHBoxLayout()
        self.btn_start = QtWidgets.QPushButton("Start commissioning")
        self.btn_close = QtWidgets.QPushButton("Close")
        btnrow.addWidget(self.btn_start)
        btnrow.addStretch(1)
        btnrow.addWidget(self.btn_close)
        outer.addLayout(btnrow)
        self.btn_start.clicked.connect(self._start)
        self.btn_close.clicked.connect(self.close)

    # ---- logging helpers ----------------------------------------------
    def _say(self, text):
        self.log.appendPlainText(text)
        self.log.ensureCursorVisible()
        QtWidgets.QApplication.processEvents()

    def _set_status(self, cid, text):
        lbl = self._rows.get(cid)
        if lbl:
            lbl.setText(text)
            lbl.setStyleSheet("color:%s; font-weight:bold;"
                              % STATUS_COLOR.get(text, "#000"))
        QtWidgets.QApplication.processEvents()

    # ---- the run ------------------------------------------------------
    def _start(self):
        self.btn_start.setEnabled(False)
        self.overall.setText("")
        # a re-run must not show the previous run's verdicts while it works
        for cid in self._rows:
            self._set_status(cid, "pending")
        try:
            self.session = Session(self.master_ini,
                                   default_nml=self.default_nml)
            self._say("connecting to %d channel(s)..."
                      % self.session.num_channels)
            self.session.connect()
        except Exception as e:
            self._say("cannot attach to a running session: %s" % e)
            self._say("Start the machine first, then reopen this page.")
            self.btn_start.setEnabled(True)
            return

        self.report = Report(self.session, operator=self.operator.text())
        batch = os.environ.get("MCHAN_VERIFY_ANSWERS")
        if batch and os.path.exists(batch):
            self._say("batch mode: answering from %s" % batch)
            operator = BatchOperator(batch, self._say)
        else:
            operator = GuiOperator(self)
        try:
            for cls in self.checks:
                v = self._run_check(cls, operator)
                if v == FAIL and cls.id == "V0":
                    self._say("static pre-flight failed - stopping before "
                              "moving iron")
                    break
        finally:
            self.session.abort_all()

        path = os.path.join(
            os.path.dirname(self.master_ini),
            "commissioning-%s-%s.md"
            % (os.path.basename(self.master_ini).rsplit(".", 1)[0],
               time.strftime("%Y%m%d-%H%M%S")))
        try:
            self.report.write(path)
            self._say("report written: %s" % path)
        except Exception as e:
            self._say("report write failed: %s" % e)
        verdict = self.report.verdict()
        self.overall.setText("Overall: %s" % verdict)
        self.overall.setStyleSheet("font-weight:bold; color:%s;"
                                   % STATUS_COLOR.get(verdict, "#000"))
        self.btn_start.setEnabled(True)

    def _run_check(self, cls, operator):
        chk = cls(self.session, self.report)
        self.report.begin(chk.id, chk.title)
        self._set_status(chk.id, "running")
        self._say("\n== %s — %s" % (chk.id, chk.title))
        try:
            if getattr(chk, "needs_power", False):
                if not self.session.power_on_all():
                    self._set_status(chk.id, FAIL)
                    return self.report.end(FAIL, "could not power on")
            if getattr(chk, "needs_homed", False):
                for ch in range(self.session.num_channels):
                    joints = self.session.joints_of(ch)
                    if not all(self.session.hal.flag("joint.%d.homed" % j)
                               for j in joints):
                        if self.session.home_channel(ch) is None:
                            self._set_status(chk.id, FAIL)
                            return self.report.end(FAIL,
                                                   "pre-homing ch%d failed" % ch)
            gen = chk.steps()
            to_send = None
            while True:
                try:
                    step = gen.send(to_send)
                except StopIteration:
                    break
                to_send = None
                if isinstance(step, Info):
                    self._say("   .. %s" % step.text)
                elif isinstance(step, Ask):
                    val = operator.answer(step)
                    self.report.answer(step.id, step.text, val)
                    to_send = val
            self._set_status(chk.id, PASS)
            self._say("   PASS")
            return self.report.end(PASS)
        except CheckSkipped as e:
            self._set_status(chk.id, SKIP)
            self._say("   SKIPPED (%s)" % e)
            return self.report.end(SKIP, str(e))
        except CheckFailed as e:
            self._set_status(chk.id, FAIL)
            self._say("   FAIL: %s" % e)
            return self.report.end(FAIL, str(e))
