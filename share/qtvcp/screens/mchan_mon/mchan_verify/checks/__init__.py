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
    from . import (v0_static, v1_power, v2_homing, v3_direction, v4_map,
                   v5_envelope, v6_origin, v7_orient, v8_spindle,
                   v9_interference)
    checks = [v0_static.V0Static, v1_power.V1Power, v2_homing.V2Homing,
              v3_direction.V3Direction, v4_map.V4Map, v5_envelope.V5Envelope,
              v6_origin.V6Origin, v7_orient.V7Orient, v8_spindle.V8Spindle,
              v9_interference.V9Interference]
    return sorted(checks, key=lambda c: c.id)
