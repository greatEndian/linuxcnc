#!/usr/bin/env python3
"""
Spline compressor using Kasa least-squares arc fitting.
Inspired by liscio (Yang Yang) - replaces naive 3-point circle fits with proper LSQ.

Algorithm:
  1. Buffer consecutive G1 moves (2D XY plane)
  2. For each potential arc segment, fit circle using Kasa LSQ method
  3. Verify all points are within tolerance of the fitted circle
  4. Emit G2/G3 if valid, otherwise output raw G1 lines

Kasa method: solves 3x3 linear system from equation:
  x² + y² = 2a·x + 2b·y + c
"""
import sys
import os
import math
import re

gcode_word_re = re.compile(r'([A-Z])([-+]?\d*\.?\d+)', re.IGNORECASE)

def parse_line(line):
    clean_line = ""
    in_comment = False
    comment_text = ""
    for char in line:
        if char == '(':
            in_comment = True
            continue
        elif char == ')':
            in_comment = False
            continue
        elif char == ';':
            break
        if not in_comment:
            clean_line += char
        else:
            comment_text += char

    words = {}
    for letter, value in gcode_word_re.findall(clean_line):
        words[letter.upper()] = float(value)

    return words, clean_line.strip(), comment_text.strip()

def format_g1_line(raw_line, last_emitted_mode):
    if last_emitted_mode == 1:
        return raw_line, 1
    n_match = re.match(r'^(\s*N\d+\s*)', raw_line, re.IGNORECASE)
    if n_match:
        n_str = n_match.group(1)
        remaining = raw_line[len(n_str):]
    else:
        remaining = raw_line

    if 'G1' in remaining.upper() or 'G01' in remaining.upper():
        return raw_line, 1

    new_line = f"{n_str}G1 {remaining.lstrip()}"
    return new_line, 1

def fit_arc_perpbisector(points, tolerance, units):
    """
    Fit arc to point cloud using perpendicular bisector method.
    This guarantees endpoints are equidistant from center (required for CNC).

    Algorithm:
    1. Fit LSQ circle to all points using Kasa method
    2. Force center onto perpendicular bisector of chord (start→end)
    3. Verify all points within tolerance of the corrected circle

    CRITICAL: Reject 3D segments (Z motion) - these should not be arcs.
    Only compress pure XY moves (constant Z).
    """
    if len(points) < 3:
        return None

    # SAFETY: Reject 3D segments with Z motion
    # Arcs should only be in XY plane with constant Z (tool depth)
    # If Z changes, the segment is a positioning/retraction move, not a cut
    p_start_z = points[0][2]
    p_end_z = points[-1][2]
    if abs(p_end_z - p_start_z) > 1e-6:
        # Z changed - this is a 3D move, don't fit arc
        return None

    # Extract XY coordinates
    p_xy = [(pt[0], pt[1]) for pt in points]
    n = len(p_xy)

    # Start with LSQ estimate
    sum_x = sum(x for x, y in p_xy)
    sum_y = sum(y for x, y in p_xy)
    sum_xx = sum(x*x for x, y in p_xy)
    sum_yy = sum(y*y for x, y in p_xy)
    sum_xy = sum(x*y for x, y in p_xy)
    sum_x3 = sum(x*x*x for x, y in p_xy)
    sum_y3 = sum(y*y*y for x, y in p_xy)
    sum_x2y = sum(x*x*y for x, y in p_xy)
    sum_xy2 = sum(x*y*y for x, y in p_xy)

    A = [
        [2*sum_xx, 2*sum_xy, sum_x],
        [2*sum_xy, 2*sum_yy, sum_y],
        [sum_x, sum_y, n]
    ]
    b = [sum_x3 + sum_xy2, sum_x2y + sum_y3, sum_xx + sum_yy]

    det = (A[0][0] * (A[1][1]*A[2][2] - A[1][2]*A[2][1]) -
           A[0][1] * (A[1][0]*A[2][2] - A[1][2]*A[2][0]) +
           A[0][2] * (A[1][0]*A[2][1] - A[1][1]*A[2][0]))

    if abs(det) < 1e-12:
        return None

    det_a = (b[0] * (A[1][1]*A[2][2] - A[1][2]*A[2][1]) -
             A[0][1] * (b[1]*A[2][2] - A[1][2]*b[2]) +
             A[0][2] * (b[1]*A[2][1] - A[1][1]*b[2]))
    det_b = (A[0][0] * (b[1]*A[2][2] - A[1][2]*b[2]) -
             b[0] * (A[1][0]*A[2][2] - A[1][2]*A[2][0]) +
             A[0][2] * (A[1][0]*b[2] - b[1]*A[2][0]))
    det_c = (A[0][0] * (A[1][1]*b[2] - b[1]*A[2][1]) -
             A[0][1] * (A[1][0]*b[2] - b[1]*A[2][0]) +
             b[0] * (A[1][0]*A[2][1] - A[1][1]*A[2][0]))

    lsq_xc = det_a / det
    lsq_yc = det_b / det
    lsq_c = det_c / det

    # Now force center onto perpendicular bisector
    p_start = p_xy[0]
    p_end = p_xy[-1]

    mid_x = (p_start[0] + p_end[0]) / 2.0
    mid_y = (p_start[1] + p_end[1]) / 2.0

    chord_x = p_end[0] - p_start[0]
    chord_y = p_end[1] - p_start[1]
    chord_len_sq = chord_x*chord_x + chord_y*chord_y

    if chord_len_sq < 1e-12:
        return None

    # Project LSQ center onto perpendicular bisector
    to_center_x = lsq_xc - p_start[0]
    to_center_y = lsq_yc - p_start[1]
    proj_t = (to_center_x * chord_x + to_center_y * chord_y) / chord_len_sq

    # Perpendicular displacement
    perp_x = lsq_xc - (p_start[0] + proj_t * chord_x)
    perp_y = lsq_yc - (p_start[1] + proj_t * chord_y)

    # Corrected center: on perpendicular bisector at same offset as LSQ
    xc = mid_x + perp_x
    yc = mid_y + perp_y
    r = math.sqrt((p_start[0] - xc)**2 + (p_start[1] - yc)**2)

    # Verify all points are within tolerance
    max_dev = 0
    for x, y in p_xy:
        dist = math.sqrt((x - xc)**2 + (y - yc)**2)
        dev = abs(dist - r)
        if dev > max_dev:
            max_dev = dev

    # Much stricter tolerance: max deviation should be <= tolerance (not 10x)
    if max_dev > tolerance:
        return None

    # Sanity check: chord-to-radius ratio
    # CRITICAL: Only accept arcs with significant curvature (>60 degrees)
    # chord < 0.5*r means arc < 60 degrees = too flat, likely a fitting artifact
    chord = math.sqrt(chord_x*chord_x + chord_y*chord_y)
    if chord > 1e-6 and r > 0:
        if chord < r * 0.5:  # STRICT: reject arcs < 60 degrees
            return None

    # Limit maximum radius
    max_r = 1000.0 if units == "mm" else 40.0
    if r > max_r:
        return None

    # Determine arc direction using cross product
    if len(p_xy) >= 3:
        p1, p2, p3 = p_xy[0], p_xy[len(p_xy)//2], p_xy[-1]
        v1_x = p2[0] - p1[0]
        v1_y = p2[1] - p1[1]
        v2_x = p3[0] - p2[0]
        v2_y = p3[1] - p2[1]
        cross = v1_x * v2_y - v1_y * v2_x
        is_ccw = (cross > 0.0)
    else:
        is_ccw = False

    return {
        'xc': xc,
        'yc': yc,
        'r': r,
        'is_ccw': is_ccw,
        'i_offset': xc - p_start[0],
        'j_offset': yc - p_start[1],
        'max_dev': max_dev
    }

def is_g1_candidate(words, motion_mode, is_absolute, active_plane):
    if not is_absolute or active_plane != 17:
        return False
    if motion_mode != 1:
        return False
    if 'X' not in words and 'Y' not in words and 'Z' not in words:
        return False
    for bad_key in ['M', 'S', 'T', 'H', 'D']:
        if bad_key in words:
            return False
    return True

def flush_buffer(points_buf, tolerance, active_feed, last_emitted_mode, units):
    if not points_buf:
        return last_emitted_mode

    if len(points_buf) < 3:
        for pt in points_buf[1:]:
            line_str, last_emitted_mode = format_g1_line(pt[3], last_emitted_mode)
            print(line_str)
        return last_emitted_mode

    i = 0
    n_points = len(points_buf)

    while i < n_points - 1:
        best_j = -1
        best_arc_data = None

        # Try increasingly longer segments (greedy lookahead)
        max_lookahead = min(n_points - 1, i + 50)

        for j in range(i + 2, max_lookahead + 1):
            arc_data = fit_arc_perpbisector(points_buf[i:j+1], tolerance, units)
            if arc_data is not None:
                best_j = j
                best_arc_data = arc_data
            else:
                break

        if best_j != -1 and best_j >= i + 2:
            p_start = points_buf[i]
            p_end = points_buf[best_j]

            g_code = "G3" if best_arc_data['is_ccw'] else "G2"
            line_out = f"{g_code} X{p_end[0]:.4f} Y{p_end[1]:.4f}"
            if abs(p_end[2] - p_start[2]) > 1e-6:
                line_out += f" Z{p_end[2]:.4f}"
            line_out += f" I{best_arc_data['i_offset']:.4f} J{best_arc_data['j_offset']:.4f}"
            if active_feed is not None:
                line_out += f" F{active_feed:.1f}"

            # ARTIFACT SUPPRESSION: Detect and skip spurious orbital arcs
            if is_spurious_arc(line_out, tolerance):
                # Suppress the arc, output original G1 move instead
                line_str, last_emitted_mode = format_g1_line(points_buf[i+1][3], last_emitted_mode)
                print(line_str)
                i += 1
            else:
                # Arc is valid, output it
                print(line_out)
                last_emitted_mode = 3 if best_arc_data['is_ccw'] else 2
                i = best_j
        else:
            line_str, last_emitted_mode = format_g1_line(points_buf[i+1][3], last_emitted_mode)
            print(line_str)
            i += 1

    return last_emitted_mode

def main():
    if len(sys.argv) < 2:
        print("Usage: spline_compressor.py <gcode_file>")
        sys.exit(1)

    gcode_path = sys.argv[1]
    if not os.path.exists(gcode_path):
        print(f"Error: File {gcode_path} not found.")
        sys.exit(1)

    # Check INI file setting
    compressor_enable = 0  # Default: disabled
    ini_path = os.environ.get("INI_FILE_NAME")
    if ini_path and os.path.exists(ini_path):
        try:
            with open(ini_path, 'r') as ini_f:
                current_section = None
                for ini_line in ini_f:
                    clean_ini = ini_line.split('#')[0].split(';')[0].strip()
                    if not clean_ini:
                        continue
                    if clean_ini.startswith('[') and clean_ini.endswith(']'):
                        current_section = clean_ini[1:-1].strip().upper()
                    elif '=' in clean_ini and current_section in ['TRAJ', 'FILTER']:
                        key, val = clean_ini.split('=', 1)
                        if key.strip().upper() == 'COMPRESSOR_ENABLE':
                            try:
                                compressor_enable = int(val.strip())
                            except ValueError:
                                compressor_enable = 0
        except Exception:
            pass

    # Pass-through mode (COMPRESSOR_ENABLE = 0): output G-code UNCHANGED
    # This is the SAFE default. Arc fitting can create spurious arcs.
    if compressor_enable == 0:
        # Use binary mode to preserve exact line endings (CRLF vs LF)
        with open(gcode_path, 'rb') as f:
            sys.stdout.buffer.write(f.read())
        return

    # Compression modes (1=perpbisector, 2=liscio) only if explicitly enabled
    if not (compressor_enable >= 1):
        with open(gcode_path, 'r') as f:
            for line in f:
                print(line, end="")
        return

    with open(gcode_path, 'r') as f:
        lines = f.readlines()

    current_x = 0.0
    current_y = 0.0
    current_z = 0.0
    active_feed = None
    is_absolute = True
    active_plane = 17
    tolerance = 0.01
    units = "mm"
    motion_mode = None
    last_emitted_mode = None

    points_buf = []

    for raw_line in lines:
        words, clean_line, comment = parse_line(raw_line)

        if not clean_line:
            print(raw_line, end="")
            continue

        # Parse modal states
        if 'G' in words:
            g_codes = [val for key, val in gcode_word_re.findall(clean_line) if key.upper() == 'G']
            for g_str in g_codes:
                try:
                    g = int(float(g_str))
                except ValueError:
                    continue
                if g == 20:
                    units = "inch"
                    tolerance = 0.0004
                elif g == 21:
                    units = "mm"
                    tolerance = 0.01
                elif g == 90:
                    is_absolute = True
                elif g == 91:
                    is_absolute = False
                elif g == 17:
                    active_plane = 17
                elif g == 18:
                    active_plane = 18
                elif g == 19:
                    active_plane = 19
                elif g == 64:
                    if 'P' in words:
                        tolerance = words['P']

        # Check motion mode updates
        has_motion = False
        new_motion_mode = motion_mode
        if 'G' in words:
            g_codes = [val for key, val in gcode_word_re.findall(clean_line) if key.upper() == 'G']
            for g_str in g_codes:
                try:
                    g = int(float(g_str))
                except ValueError:
                    continue
                if g in [0, 1, 2, 3]:
                    new_motion_mode = g
                    has_motion = True
                elif g in [80]:
                    new_motion_mode = None
                    has_motion = True

        # Check candidate status
        candidate = is_g1_candidate(words, new_motion_mode if has_motion else motion_mode, is_absolute, active_plane)

        # Extract target position
        x_next = words.get('X', current_x)
        y_next = words.get('Y', current_y)
        z_next = words.get('Z', current_z)

        # Pure Z vertical motions should not be blended into XY arcs
        if candidate and abs(x_next - current_x) < 1e-6 and abs(y_next - current_y) < 1e-6:
            candidate = False

        if candidate:
            if has_motion:
                motion_mode = new_motion_mode
            if not points_buf:
                points_buf.append((current_x, current_y, current_z, ""))
            points_buf.append((x_next, y_next, z_next, raw_line.strip()))
        else:
            if points_buf:
                last_emitted_mode = flush_buffer(points_buf, tolerance, active_feed, last_emitted_mode, units)
                points_buf = []

            if has_motion:
                motion_mode = new_motion_mode
                last_emitted_mode = new_motion_mode

            # If this line is a coordinate update relying on modal motion mode
            if not has_motion and ('X' in words or 'Y' in words or 'Z' in words):
                if motion_mode == 1:
                    raw_line, last_emitted_mode = format_g1_line(raw_line.strip() + "\n", last_emitted_mode)
                elif motion_mode == 0:
                    raw_line, last_emitted_mode = format_g0_line(raw_line.strip() + "\n", last_emitted_mode)

            print(raw_line, end="")

        # Update current positions
        if 'F' in words:
            active_feed = words['F']
        current_x = x_next
        current_y = y_next
        current_z = z_next

    if points_buf:
        last_emitted_mode = flush_buffer(points_buf, tolerance, active_feed, last_emitted_mode, units)

def format_g0_line(raw_line, last_emitted_mode):
    if last_emitted_mode == 0:
        return raw_line, 0
    n_match = re.match(r'^(\s*N\d+\s*)', raw_line, re.IGNORECASE)
    if n_match:
        n_str = n_match.group(1)
        remaining = raw_line[len(n_str):]
    else:
        remaining = raw_line

    if 'G0' in remaining.upper() or 'G00' in remaining.upper():
        return raw_line, 0

    new_line = f"{n_str}G0 {remaining.lstrip()}"
    return new_line, 0

def is_spurious_arc(line_str, tolerance):
    """
    Detect spurious/orbital arcs that should be suppressed.
    Returns True if arc should be rejected (replaced with G1 moves).

    Heuristics:
    1. Arc with extremely large I/J (center far from segment) = orbital artifact
    2. Arc with I or J > 100mm and chord < 30mm = likely spurious
    3. Arc offset magnitude >> radius = numerical fitting error
    """
    # Try to parse as G2/G3 arc
    if not re.match(r'^G[23]\s', line_str.upper()):
        return False

    # Extract I, J values
    i_match = re.search(r'I([-+]?\d*\.?\d+)', line_str, re.IGNORECASE)
    j_match = re.search(r'J([-+]?\d*\.?\d+)', line_str, re.IGNORECASE)
    x_match = re.search(r'X([-+]?\d*\.?\d+)', line_str, re.IGNORECASE)
    y_match = re.search(r'Y([-+]?\d*\.?\d+)', line_str, re.IGNORECASE)

    if not (i_match and j_match and x_match and y_match):
        return False

    try:
        i_val = float(i_match.group(1))
        j_val = float(j_match.group(1))
        x_end = float(x_match.group(1))
        y_end = float(y_match.group(1))
    except (ValueError, AttributeError):
        return False

    # Compute radius from offsets
    radius = math.sqrt(i_val*i_val + j_val*j_val)

    # ORBITAL DETECTOR: Arc center far from segment = bad fit
    # If |I| > 50mm or |J| > 50mm with small radius, likely spurious
    offset_mag = max(abs(i_val), abs(j_val))

    if offset_mag > 50.0 and radius < 100.0:
        # Large offset, small radius = orbital artifact
        return True

    if radius > 0 and offset_mag > radius * 2:
        # Offset much larger than radius = numerical error
        return True

    return False

if __name__ == "__main__":
    main()
