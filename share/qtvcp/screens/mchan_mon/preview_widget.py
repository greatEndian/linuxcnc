#!/usr/bin/env python3
"""G4a: per-channel toolpath preview widget.

Rendering = a Qt port of mchan-preview-view.py's projection (the s5 combined
viewer): lathe view basis, screen-right = world Z, screen-up = world Y,
depth = world X, with a fixed slight-iso azim/elev. Data = the s4 extractor
(mchan-preview.py dump-one): the channel's program run through THAT channel's
own interpreter, transformed to the machine world frame via [CHANNEL]
ORIGIN/ORIENT. One widget shows one or more channels' segment lists, so the
same class serves the per-channel panes (G4a) and the combined view (G4b).
"""
import json
import math
import os
import subprocess
import sys

from qtpy import QtCore, QtGui, QtWidgets

# the s4 extraction core ships alongside this widget (repo-self-contained)
MCHAN_PREVIEW = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                             "preview_extract.py")


# ---- world-frame envelope helpers, ported verbatim from
# ---- mchan-preview-view.py (s5) to keep tkinter out of the Qt process ----
def ini_find(path, sec, var, default=None):
    cur = None
    with open(path) as f:
        for line in f:
            ls = line.split(';')[0].split('#')[0].strip()
            if not ls:
                continue
            if ls.startswith('['):
                cur = ls.strip('[]').strip()
            elif '=' in ls and cur == sec:
                k, v = ls.split('=', 1)
                if k.strip() == var:
                    return v.strip()
    return default


def rot_matrix(rx, ry, rz):
    rx, ry, rz = (math.radians(v) for v in (rx, ry, rz))
    cx, sx = math.cos(rx), math.sin(rx)
    cy, sy = math.cos(ry), math.sin(ry)
    cz, sz = math.cos(rz), math.sin(rz)
    return [[cz*cy, cz*sy*sx - sz*cx, cz*sy*cx + sz*sx],
            [sz*cy, sz*sy*sx + cz*cx, sz*sy*cx - cz*sx],
            [-sy, cy*sx, cy*cx]]


def triplet(v, default=(0.0, 0.0, 0.0)):
    if not v:
        return list(default)
    p = v.split()
    return [float(p[0]), float(p[1]), float(p[2])]


def channel_box(ini, origin, orient=(0, 0, 0)):
    """world-frame bounding box of the channel envelope (s5, exact for
    90-degree-multiple ORIENTs)."""
    lim = []
    for L in "XYZ":
        lo = ini_find(ini, "AXIS_" + L, "MIN_LIMIT")
        hi = ini_find(ini, "AXIS_" + L, "MAX_LIMIT")
        lim.append((float(lo) if lo is not None else 0.0,
                    float(hi) if hi is not None else 0.0))
    R = rot_matrix(*orient)
    pts = []
    for x in lim[0]:
        for y in lim[1]:
            for z in lim[2]:
                pts.append([R[0][0]*x + R[0][1]*y + R[0][2]*z + origin[0],
                            R[1][0]*x + R[1][1]*y + R[1][2]*z + origin[1],
                            R[2][0]*x + R[2][1]*y + R[2][2]*z + origin[2]])
    return [(min(p[k] for p in pts), max(p[k] for p in pts))
            for k in range(3)]


def box_overlap(a, b):
    ov = []
    for (a1, a2), (b1, b2) in zip(a, b):
        lo, hi = max(a1, b1), min(a2, b2)
        if lo > hi:
            return None
        ov.append((lo, hi))
    return ov


def box_edges(box):
    (x1, x2), (y1, y2), (z1, z2) = box
    c = [(x, y, z) for z in (z1, z2) for y in (y1, y2) for x in (x1, x2)]
    idx = [(0, 1), (2, 3), (4, 5), (6, 7), (0, 2), (1, 3), (4, 6), (5, 7),
           (0, 4), (1, 5), (2, 6), (3, 7)]
    return [(c[i], c[j]) for i, j in idx]

# per-channel fallback palette (used when the ini has no [CHANNEL]COLOR)
PALETTE = [(80, 200, 255), (255, 170, 60), (140, 255, 140), (255, 120, 220)]


def extract_channel(ini_path, ngc_path):
    """Run the s4 extractor for one channel; returns the channel dict or None.

    Uses the SAME python + code path the numerically-proven selftest uses."""
    try:
        r = subprocess.run(
            [sys.executable, MCHAN_PREVIEW, "dump-one", ini_path, ngc_path],
            capture_output=True, text=True, timeout=30)
        if r.returncode != 0:
            return None
        return json.loads(r.stdout)
    except Exception:
        return None


class PreviewWidget(QtWidgets.QWidget):
    """Paints channel toolpaths in the world frame. Rapids dashed, feeds
    solid; per-channel color; auto-fit to extents."""

    AZIM = 0.45   # rad, around the lathe base view (defaults)
    ELEV = 0.30

    def __init__(self, parent=None):
        super().__init__(parent)
        self.channels = []          # list of channel dicts (s4 schema)
        self.boxes = []             # [(box, (r,g,b))] channel envelopes
        self.zones = []             # [box] computed overlaps, drawn red
        self.triads = []            # [(origin, R)] per channel
        self.zoom = 1.0
        self.pan = [0.0, 0.0]
        self.azim = self.AZIM       # interactive view rotation (s5 parity)
        self.elev = self.ELEV
        self._drag = None
        self.live = []              # s6: [(world_pos, rgb)] live tool markers
        self.trails = {}            # s6: marker idx -> recent world positions
        self.warn_radius = 0.0      # s6: >0 draws tool-to-tool distance
        self.setMinimumSize(220, 160)
        self.setSizePolicy(QtWidgets.QSizePolicy.Expanding,
                           QtWidgets.QSizePolicy.Expanding)

    def set_channels(self, channels):
        self.channels = [c for c in channels if c]
        self.update()

    def set_live(self, markers):
        """s6: live tool positions (world frame), one per channel."""
        self.live = markers
        for i, (pos, _rgb) in enumerate(markers):
            t = self.trails.setdefault(i, [])
            if not t or any(abs(a - b) > 1e-6 for a, b in zip(t[-1], pos)):
                t.append(list(pos))
                del t[:-200]
        self.update()

    def clear_trails(self):
        self.trails = {}

    def set_context(self, boxes, zones, triads):
        """G4c: envelope boxes, overlap zones, origin triads (combined view)."""
        self.boxes = boxes
        self.zones = zones
        self.triads = triads
        self.update()

    # ---- G4c interaction: wheel zoom, drag pan, double-click refit ----
    def wheelEvent(self, ev):
        d = ev.angleDelta().y()
        if d:
            self.zoom *= 1.15 if d > 0 else 1 / 1.15
            self.update()

    def mousePressEvent(self, ev):
        # left-drag = ROTATE (s5 viewer parity); shift+left or middle = pan
        mode = ("pan" if (ev.button() == QtCore.Qt.MiddleButton
                          or ev.modifiers() & QtCore.Qt.ShiftModifier)
                else "rot")
        self._drag = (mode, ev.x(), ev.y(),
                      self.pan[0], self.pan[1], self.azim, self.elev)

    def mouseMoveEvent(self, ev):
        if not self._drag:
            return
        mode, x0, y0, px, py, az, el = self._drag
        dx, dy = ev.x() - x0, ev.y() - y0
        if mode == "pan":
            self.pan = [px + dx, py + dy]
        else:
            self.azim = az + dx * 0.01
            self.elev = max(-1.55, min(1.55, el + dy * 0.01))
        self.update()

    def mouseReleaseEvent(self, _ev):
        self._drag = None

    def mouseDoubleClickEvent(self, _ev):
        self.zoom = 1.0
        self.pan = [0.0, 0.0]
        self.azim = self.AZIM
        self.elev = self.ELEV
        self.update()

    # ---- projection (port of mchan-preview-view.py Viewer.proj) ----
    def _fit(self):
        pts = [p for c in self.channels for s in c["segments"]
               for p in (s["start"], s["end"])]
        for box, _c in self.boxes:
            pts += [[box[0][i0], box[1][i1], box[2][i2]]
                    for i0 in (0, 1) for i1 in (0, 1) for i2 in (0, 1)]
        if not pts:
            return None
        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        zs = [p[2] for p in pts]
        cx = (min(xs) + max(xs)) / 2.0
        cy = (min(ys) + max(ys)) / 2.0
        cz = (min(zs) + max(zs)) / 2.0
        span = max(max(xs) - min(xs), max(ys) - min(ys),
                   max(zs) - min(zs), 1.0)
        scale = 0.85 * min(self.width(), self.height()) / span * self.zoom
        return cx, cy, cz, scale

    def _proj(self, p, fit):
        cx, cy, cz, scale = fit
        x, y, z = p[0] - cx, p[1] - cy, p[2] - cz
        u0, v0, w0 = z, x, y              # lathe basis: right, depth, up
        ca, sa = math.cos(self.azim), math.sin(self.azim)
        ce, se = math.cos(self.elev), math.sin(self.elev)
        u = ca * u0 + sa * v0
        v = -sa * u0 + ca * v0
        w = ce * w0 - se * v
        return (self.width() / 2.0 + u * scale + self.pan[0],
                self.height() / 2.0 - w * scale + self.pan[1])

    def paintEvent(self, _ev):
        qp = QtGui.QPainter(self)
        qp.fillRect(self.rect(), QtGui.QColor(24, 24, 28))
        fit = self._fit()
        if fit is None:
            qp.setPen(QtGui.QColor(120, 120, 120))
            qp.drawText(self.rect(), QtCore.Qt.AlignCenter, "(no program)")
            qp.end()
            return
        # G4c context: envelopes (dim channel color), overlap zones (red),
        # origin triads (X red / Y green / Z blue)
        for box, rgb in self.boxes:
            qp.setPen(QtGui.QPen(QtGui.QColor(rgb[0], rgb[1], rgb[2], 90), 1))
            for a, b in box_edges(box):
                qp.drawLine(QtCore.QPointF(*self._proj(a, fit)),
                            QtCore.QPointF(*self._proj(b, fit)))
        for box in self.zones:
            qp.setPen(QtGui.QPen(QtGui.QColor(255, 60, 60, 200), 2))
            for a, b in box_edges(box):
                qp.drawLine(QtCore.QPointF(*self._proj(a, fit)),
                            QtCore.QPointF(*self._proj(b, fit)))
        tlen = 0.06 * max(self.width(), self.height()) / fit[3]
        tri_cols = [QtGui.QColor(255, 80, 80), QtGui.QColor(80, 255, 80),
                    QtGui.QColor(90, 120, 255)]
        for origin, R in self.triads:
            for ax in range(3):
                tip = [origin[k] + R[k][ax] * tlen for k in range(3)]
                qp.setPen(QtGui.QPen(tri_cols[ax], 2))
                qp.drawLine(QtCore.QPointF(*self._proj(origin, fit)),
                            QtCore.QPointF(*self._proj(tip, fit)))
        # s6 live layer: trails, tool markers, tool-to-tool distance
        for i, trail in self.trails.items():
            if len(trail) > 1 and i < len(self.live):
                rgb = self.live[i][1]
                qp.setPen(QtGui.QPen(QtGui.QColor(rgb[0], rgb[1], rgb[2], 90), 1))
                pts = [QtCore.QPointF(*self._proj(p, fit)) for p in trail]
                for a, b in zip(pts, pts[1:]):
                    qp.drawLine(a, b)
        for pos, rgb in self.live:
            x, y = self._proj(pos, fit)
            col = QtGui.QColor(*rgb)
            qp.setPen(QtGui.QPen(col, 2))
            qp.drawEllipse(QtCore.QPointF(x, y), 6, 6)
            qp.drawLine(QtCore.QPointF(x - 10, y), QtCore.QPointF(x + 10, y))
            qp.drawLine(QtCore.QPointF(x, y - 10), QtCore.QPointF(x, y + 10))
        if self.warn_radius > 0 and len(self.live) >= 2:
            a, b = self.live[0][0], self.live[1][0]
            dist = math.sqrt(sum((a[k] - b[k]) ** 2 for k in range(3)))
            warn = dist < self.warn_radius
            col = QtGui.QColor(255, 60, 60) if warn else QtGui.QColor(200, 200, 200)
            qp.setPen(QtGui.QPen(col, 1, QtCore.Qt.DotLine))
            pa, pb = self._proj(a, fit), self._proj(b, fit)
            qp.drawLine(QtCore.QPointF(*pa), QtCore.QPointF(*pb))
            mid = ((pa[0] + pb[0]) / 2, (pa[1] + pb[1]) / 2)
            qp.setPen(col)
            f = qp.font()
            f.setBold(warn)
            qp.setFont(f)
            qp.drawText(QtCore.QPointF(mid[0] + 6, mid[1] - 6),
                        "%.1f mm%s" % (dist, "  << WARN" if warn else ""))
        for idx, ch in enumerate(self.channels):
            rgb = ch.get("color") or PALETTE[idx % len(PALETTE)]
            col = QtGui.QColor(*rgb)
            solid = QtGui.QPen(col, 2)
            dash = QtGui.QPen(QtGui.QColor(rgb[0], rgb[1], rgb[2], 130), 1,
                              QtCore.Qt.DashLine)
            for s in ch["segments"]:
                qp.setPen(dash if s["kind"] == "traverse" else solid)
                a = self._proj(s["start"], fit)
                b = self._proj(s["end"], fit)
                qp.drawLine(QtCore.QPointF(*a), QtCore.QPointF(*b))
        qp.end()
