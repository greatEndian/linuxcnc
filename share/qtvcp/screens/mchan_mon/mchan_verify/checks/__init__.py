"""mchan_verify.checks - check base + registry.

A check is a generator of typed steps; the runner (CLI today, qtvcp page
in P3) executes them. Every operator interaction is an Ask with a STABLE
id so a --batch answers file can stand in for the operator (that batch
run against the sim IS the tool's own regression gate).
"""


class Ask:
    """operator question. kind: yesno | measure | confirm.
    yesno -> bool, measure -> float, confirm -> Enter/ack (True)."""

    def __init__(self, ask_id, text, kind="yesno"):
        self.id = ask_id
        self.text = text
        self.kind = kind


class Info:
    def __init__(self, text):
        self.text = text


class CheckFailed(Exception):
    pass


class CheckSkipped(Exception):
    pass


class Check:
    id = "V?"
    title = "?"
    needs_power = False      # runner powers the machine on first
    needs_homed = False      # runner ensures both channels homed first

    def __init__(self, session, report):
        self.s = session
        self.r = report

    def steps(self):
        """generator: yield Ask(...) -> receives the answer via .send();
        yield Info(...) -> no answer. Raise CheckFailed/CheckSkipped or
        return normally (PASS)."""
        raise NotImplementedError

    # helpers ------------------------------------------------------------
    def require(self, cond, why):
        if not cond:
            raise CheckFailed(why)

    def skip(self, why):
        raise CheckSkipped(why)


def load_all():
    from . import v0_static, v1_power, v2_homing, v4_map
    checks = [v0_static.V0Static, v1_power.V1Power,
              v2_homing.V2Homing, v4_map.V4Map]
    return sorted(checks, key=lambda c: c.id)
