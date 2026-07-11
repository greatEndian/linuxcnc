"""mchan_verify.cli - the commissioning wizard.

Interactive: every motion is announced before it runs; every question is
asked on the terminal. Batch (--batch answers.json): the answers file
stands in for the operator - that run against the sim is the tool's own
regression gate, and the file doubles as a worked example of the
expected machine behavior.

usage:
  mchan-verify <master.ini> [--batch answers.json] [--report out.md]
               [--only V2,V4] [--operator NAME] [--list]
"""
import argparse
import json
import sys
import time

from .checks import Ask, CheckFailed, CheckSkipped, Info, load_all
from .report import FAIL, PASS, SKIP, Report
from .session import Session


class Operator:
    """terminal or answers-file front end for Ask steps."""

    def __init__(self, batch=None):
        self.batch = batch

    def answer(self, ask):
        if self.batch is not None:
            if ask.id not in self.batch:
                raise CheckFailed("batch mode: no answer for '%s'" % ask.id)
            raw = self.batch[ask.id]
        else:
            prompt = {"yesno": " [y/n] ", "measure": " (number) ",
                      "confirm": " [Enter] "}[ask.kind]
            raw = input("\n  ?? %s%s" % (ask.text, prompt)).strip()
        if ask.kind == "yesno":
            if isinstance(raw, bool):
                return raw
            return str(raw).lower() in ("y", "yes", "true", "1")
        if ask.kind == "measure":
            return float(raw)
        return True


def run_check(check_cls, session, report, operator):
    chk = check_cls(session, report)
    report.begin(chk.id, chk.title)
    print("\n== %s — %s" % (chk.id, chk.title))
    try:
        if getattr(chk, "needs_power", False):
            if not session.power_on_all():
                return report.end(FAIL, "could not power the machine on")
        if getattr(chk, "needs_homed", False):
            for ch in range(session.num_channels):
                joints = session.joints_of(ch)
                if not all(session.hal.flag("joint.%d.homed" % j)
                           for j in joints):
                    if session.home_channel(ch) is None:
                        return report.end(FAIL, "pre-homing ch%d failed" % ch)
        gen = chk.steps()
        to_send = None
        while True:
            try:
                step = gen.send(to_send)
            except StopIteration:
                break
            to_send = None
            if isinstance(step, Info):
                print("   .. %s" % step.text)
            elif isinstance(step, Ask):
                val = operator.answer(step)
                report.answer(step.id, step.text, val)
                to_send = val
        print("   %s" % PASS)
        return report.end(PASS)
    except CheckSkipped as e:
        print("   %s (%s)" % (SKIP, e))
        return report.end(SKIP, str(e))
    except CheckFailed as e:
        print("   %s: %s" % (FAIL, e))
        return report.end(FAIL, str(e))


def main(argv=None):
    ap = argparse.ArgumentParser(prog="mchan-verify", description=__doc__)
    ap.add_argument("master_ini")
    ap.add_argument("--batch", help="answers json (headless run)")
    ap.add_argument("--report", help="report path (default: "
                    "commissioning-<machine>-<date>.md)")
    ap.add_argument("--only", help="comma list of check ids (e.g. V2,V4)")
    ap.add_argument("--operator", default="", help="name for the record")
    ap.add_argument("--list", action="store_true", help="list checks and exit")
    args = ap.parse_args(argv)

    checks = load_all()
    if args.list:
        for c in checks:
            print("%-4s %s" % (c.id, c.title))
        return 0
    if args.only:
        wanted = {w.strip().upper() for w in args.only.split(",")}
        checks = [c for c in checks if c.id in wanted]

    batch = None
    if args.batch:
        with open(args.batch) as f:
            batch = json.load(f)

    session = Session(args.master_ini)
    print("mchan-verify: %d channel(s), joints %s"
          % (session.num_channels, session.all_joints()))
    session.connect()
    report = Report(session, operator=args.operator)
    operator = Operator(batch)

    stop = False
    try:
        for check_cls in checks:
            verdict = run_check(check_cls, session, report, operator)
            if verdict == FAIL and check_cls.id == "V0":
                print("\nstatic pre-flight failed - fix the config before "
                      "moving iron; stopping")
                stop = True
                break
    finally:
        session.abort_all()

    path = args.report or ("commissioning-%s-%s.md"
                           % (session.master_ini.split("/")[-1].rsplit(".", 1)[0],
                              time.strftime("%Y%m%d-%H%M%S")))
    report.write(path)
    print("\nreport: %s   overall: %s%s"
          % (path, report.verdict(), "  (stopped early)" if stop else ""))
    return 0 if report.verdict() == PASS else 1


if __name__ == "__main__":
    sys.exit(main())
