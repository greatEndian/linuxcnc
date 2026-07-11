"""V0 - static pre-flight: run the config-time tools if they are
available next to the master ini (mchan-lint.sh, mchan-preview.py
selftest). Config bugs are cheaper fixed before the iron moves; a lint
FAIL fails this check and the runner stops the wizard.
"""
import os
import subprocess

from . import Check, Info


class V0Static(Check):
    id = "V0"
    title = "static pre-flight (lint + preview selftest)"

    def _tool(self, name):
        p = os.path.join(self.s.ini_dir, name)
        return p if os.access(p, os.X_OK) or os.path.isfile(p) else None

    def steps(self):
        ran_any = False
        lint = self._tool("mchan-lint.sh")
        if lint:
            r = subprocess.run(["bash", lint, self.s.master_ini],
                               capture_output=True, text=True)
            fails = [ln for ln in r.stdout.splitlines()
                     if ln.startswith("FAIL")]
            self.r.detail("mchan-lint rc=%d, %d FAIL line(s)"
                          % (r.returncode, len(fails)))
            for ln in fails[:10]:
                self.r.detail(ln)
            self.require(r.returncode == 0,
                         "mchan-lint reports FAILs - fix the config first")
            ran_any = True
        else:
            self.r.detail("mchan-lint.sh not found next to master ini - skipped")

        prev = self._tool("mchan-preview.py")
        if prev:
            r = subprocess.run(["python3", prev, "selftest"],
                               capture_output=True, text=True)
            self.r.detail("mchan-preview selftest rc=%d" % r.returncode)
            tail = (r.stdout or r.stderr).strip().splitlines()
            if tail:
                self.r.detail(tail[-1])
            self.require(r.returncode == 0, "preview selftest failed")
            ran_any = True
        else:
            self.r.detail("mchan-preview.py not found next to master ini - skipped")

        yield Info("static pre-flight done")
        if not ran_any:
            self.skip("no static tools found next to %s"
                      % os.path.basename(self.s.master_ini))
