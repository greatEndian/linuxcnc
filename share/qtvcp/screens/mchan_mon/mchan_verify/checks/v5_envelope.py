"""V5 - envelope reality.

For each linear axis of each channel: creep to just inside MIN_LIMIT and
MAX_LIMIT (operator watches, single-step confirm), then command 1 unit
BEYOND the limit and require it to be REFUSED (soft-limit oracle). Ask:
the integrator confirms physical clearance at both ends. Angular axes are
skipped (typically unlimited on a rotary; nothing to creep to).

Uses MDI so the move is a single, bounded, announced command per leg.
"""
from . import Ask, Check, Info

MARGIN = 1.0     # mm inside / beyond the limit


class V5Envelope(Check):
    id = "V5"
    title = "envelope reality (limits + soft-limit refusal)"
    needs_power = True
    needs_homed = True

    def steps(self):
        for ch in range(self.s.num_channels):
            for letter in sorted(self.s.maps[ch]):
                if self.s.is_angular(letter):
                    continue
                lim = self.s.axis_limits(ch, letter)
                if lim is None:
                    self.r.detail("ch%d %s: no MIN/MAX_LIMIT - skipped"
                                  % (ch, letter))
                    continue
                lo, hi = lim
                if hi - lo < 3 * MARGIN:
                    self.r.detail("ch%d %s: envelope %.1f..%.1f too narrow "
                                  "to test - skipped" % (ch, letter, lo, hi))
                    continue

                for name, inside, beyond in (
                        ("MIN", lo + MARGIN, lo - MARGIN),
                        ("MAX", hi - MARGIN, hi + MARGIN)):
                    ok = yield Ask(
                        "v5.ch%d.%s.%s.go" % (ch, letter, name),
                        "Head %d: about to move %s to %.3f (just inside %s "
                        "limit %.3f). Path clear?"
                        % (ch, letter, inside, name, lo if name == "MIN" else hi),
                        "yesno")
                    self.require(ok, "operator halted before the %s leg"
                                     % name)
                    good, err = self.s.mdi(ch, "G53 G0 %s%.4f" % (letter, inside))
                    self.require(good, "ch%d %s: move to just-inside %s (%.3f) "
                                       "was refused: %s"
                                 % (ch, letter, name, inside, err))
                    pos = self.s.joint_pos(self.s.maps[ch][letter.upper()])
                    self.require(abs(pos - inside) < 0.01,
                                 "ch%d %s: reached %.4f, expected %.4f"
                                 % (ch, letter, pos, inside))
                    self.r.detail("ch%d %s: reached %.3f inside %s limit"
                                  % (ch, letter, pos, name))
                    conf = yield Ask(
                        "v5.ch%d.%s.%s.clear" % (ch, letter, name),
                        "Head %d %s at %.3f: physical clearance ok at the %s "
                        "end?" % (ch, letter, inside, name), "yesno")
                    self.require(conf, "ch%d %s: no physical clearance at %s "
                                       "- narrow the limit" % (ch, letter, name))
                    # the refusal test: 1 unit beyond must be rejected
                    good, err = self.s.mdi(ch, "G53 G0 %s%.4f" % (letter, beyond))
                    self.require(not good,
                                 "ch%d %s: move BEYOND %s limit to %.3f was "
                                 "ACCEPTED - soft limit not enforced!"
                                 % (ch, letter, name, beyond))
                    self.r.detail("ch%d %s: beyond-%s command correctly "
                                  "refused (%s)" % (ch, letter, name,
                                                    err or "no motion"))
                # park mid-travel
                self.s.mdi(ch, "G53 G0 %s%.4f" % (letter, (lo + hi) / 2.0))
            yield Info("ch%d envelope legs done" % ch)
