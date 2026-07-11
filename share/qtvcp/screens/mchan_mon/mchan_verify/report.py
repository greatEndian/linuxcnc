"""mchan_verify.report - the commissioning record.

One markdown file per run: per-check verdicts + operator answers +
measured values + config fingerprint + sign-off matrix (house style:
RTCP-COMMISSIONING.md).
"""
import hashlib
import os
import time

PASS, FAIL, SKIP = "PASS", "FAIL", "SKIPPED"


class Report:
    def __init__(self, session, operator=""):
        self.session = session
        self.operator = operator
        self.started = time.strftime("%Y-%m-%d %H:%M:%S")
        self.results = []          # (check_id, title, verdict, [detail lines])
        self.current = None

    def begin(self, check_id, title):
        self.current = [check_id, title, None, []]

    def detail(self, text):
        if self.current:
            self.current[3].append(str(text))

    def answer(self, ask_id, text, value):
        self.detail("ASK %s: %s -> %r" % (ask_id, text, value))

    def end(self, verdict, reason=""):
        self.current[2] = verdict
        if reason:
            self.current[3].append(reason)
        self.results.append(tuple(self.current))
        self.current = None
        return verdict

    def verdict(self):
        if any(r[2] == FAIL for r in self.results):
            return FAIL
        return PASS

    def _fingerprint(self):
        lines = []
        for ini in self.session.channel_inis:
            try:
                h = hashlib.sha256(open(ini, "rb").read()).hexdigest()[:16]
            except OSError:
                h = "unreadable"
            lines.append("  - `%s`  sha256:%s" % (os.path.basename(ini), h))
        return lines

    def write(self, path):
        machine = os.path.basename(self.session.master_ini)
        out = []
        out.append("# Commissioning record — %s" % machine)
        out.append("")
        out.append("Run started: %s   Operator: %s" %
                   (self.started, self.operator or "(not given)"))
        out.append("Overall: **%s**" % self.verdict())
        out.append("")
        out.append("## Sign-off matrix")
        out.append("")
        out.append("| check | title | verdict |")
        out.append("|---|---|---|")
        for cid, title, verdict, _ in self.results:
            out.append("| %s | %s | %s |" % (cid, title, verdict))
        out.append("")
        out.append("## Details")
        for cid, title, verdict, details in self.results:
            out.append("")
            out.append("### %s — %s: %s" % (cid, title, verdict))
            for d in details:
                out.append("  - %s" % d)
        out.append("")
        out.append("## Config fingerprint")
        out.extend(self._fingerprint())
        out.append("")
        out.append("Integrator sign-off: ______________________  date: ________")
        out.append("")
        with open(path, "w") as f:
            f.write("\n".join(out))
        return path
