#!/usr/bin/env python3
import sys
import os
import math
import re

# Regex to parse G-code words
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

def circle_from_3_points(p1, p2, p3):
    x1, y1 = p1
    x2, y2 = p2
    x3, y3 = p3
    
    det = (x1 - x2) * (y2 - y3) - (x2 - x3) * (y1 - y2)
    if abs(det) < 1e-9:
        return None
        
    mx1, my1 = (x1 + x2) / 2.0, (y1 + y2) / 2.0
    mx2, my2 = (x2 + x3) / 2.0, (y2 + y3) / 2.0
    
    if abs(y1 - y2) < 1e-9:
        if abs(y2 - y3) < 1e-9:
            return None
        slope2 = (x2 - x3) / (y3 - y2)
        xc = mx1
        yc = slope2 * (xc - mx2) + my2
    elif abs(y2 - y3) < 1e-9:
        slope1 = (x1 - x2) / (y2 - y1)
        xc = mx2
        yc = slope1 * (xc - mx1) + my1
    else:
        slope1 = (x1 - x2) / (y2 - y1)
        slope2 = (x2 - x3) / (y3 - y2)
        if abs(slope1 - slope2) < 1e-9:
            return None
        xc = (slope1 * mx1 - slope2 * mx2 + my2 - my1) / (slope1 - slope2)
        yc = slope1 * (xc - mx1) + my1
        
    r = ((xc - x1)**2 + (yc - y1)**2)**0.5
    return (xc, yc, r)

def unwrap_angles(angles):
    unwrapped = []
    if not angles:
        return unwrapped
    unwrapped.append(angles[0])
    for i in range(1, len(angles)):
        diff = angles[i] - angles[i-1]
        diff = (diff + math.pi) % (2 * math.pi) - math.pi
        unwrapped.append(unwrapped[-1] + diff)
    return unwrapped

def check_angle_monotonicity(angles):
    unwrapped = unwrap_angles(angles)
    if len(unwrapped) < 2:
        return True
    diffs = [unwrapped[i] - unwrapped[i-1] for i in range(1, len(unwrapped))]
    all_positive = all(d >= -1e-9 for d in diffs)
    all_negative = all(d <= 1e-9 for d in diffs)
    return all_positive or all_negative

def fit_arc(points, start_idx, end_idx, tolerance, units):
    if end_idx - start_idx < 2:
        return None

    p_start = points[start_idx]
    p_end = points[end_idx]
    mid_idx = (start_idx + end_idx) // 2
    p_mid = points[mid_idx]

    p1 = (p_start[0], p_start[1])
    p2 = (p_mid[0], p_mid[1])
    p3 = (p_end[0], p_end[1])

    circle = circle_from_3_points(p1, p2, p3)
    if not circle:
        return None

    xc, yc, r = circle

    # Calculate the chord length (straight-line distance from start to end)
    chord_len = ((p3[0] - p1[0])**2 + (p3[1] - p1[1])**2)**0.5

    # Sanity check: if chord is too short relative to radius, it's a flat arc (spurious fit)
    # For a valid arc, chord_len should be at least ~0.3 * radius
    # (chord = 0.3*r means the arc spans ~35 degrees, which is meaningful curvature)
    # Flatter arcs tend to be numerical artifacts from fitting nearly-collinear segments
    if chord_len > 1e-6 and r > 0:
        if chord_len < r * 0.3:
            return None

    # Limit maximum radius to prevent flat lines from being fitted as circles (unit-aware)
    max_r = 1000.0 if units == "mm" else 40.0
    if r > max_r:
        return None
        
    angles = []
    z_start = p_start[2]
    z_end = p_end[2]
    
    for idx in range(start_idx, end_idx + 1):
        x, y, z = points[idx][0], points[idx][1], points[idx][2]
        dist = abs(((x - xc)**2 + (y - yc)**2)**0.5 - r)
        if dist > tolerance:
            return None
            
        angle = math.atan2(y - yc, x - xc)
        angles.append(angle)
        
    if not check_angle_monotonicity(angles):
        return None
        
    unwrapped = unwrap_angles(angles)
    theta_start = unwrapped[0]
    theta_end = unwrapped[-1]
    theta_range = theta_end - theta_start
    
    if abs(theta_range) < 1e-9:
        return None
        
    # Prevent giant/full circles and rounding errors:
    # Ensure the arc has a minimum angular travel (approx 3 degrees)
    if abs(theta_range) < 0.05:
        return None
        
    # Restrict arcs to less than 180 degrees to prevent endpoint ambiguity
    if abs(theta_range) > math.pi:
        return None
        
    # Verify Z linear interpolation along the helix
    for k, idx in enumerate(range(start_idx, end_idx + 1)):
        z = points[idx][2]
        theta_k = unwrapped[k]
        ratio = (theta_k - theta_start) / theta_range
        z_expected = z_start + (z_end - z_start) * ratio
        if abs(z - z_expected) > tolerance:
            return None
            
    # Calculate arc direction CCW vs CW using robust cross-product
    dx1 = p_mid[0] - p_start[0]
    dy1 = p_mid[1] - p_start[1]
    dx2 = p_end[0] - p_mid[0]
    dy2 = p_end[1] - p_mid[1]
    cross = dx1 * dy2 - dy1 * dx2
    
    is_ccw = (cross > 0.0)
    return {
        'xc': xc,
        'yc': yc,
        'r': r,
        'is_ccw': is_ccw,
        'i_offset': xc - p_start[0],
        'j_offset': yc - p_start[1]
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
        
        max_lookahead = min(n_points - 1, i + 40)
        
        for j in range(i + 2, max_lookahead + 1):
            arc_data = fit_arc(points_buf, i, j, tolerance, units)
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
        
    # Check INI file setting (bulletproof manual parser)
    compressor_enable = True
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
                            val_clean = val.strip().upper()
                            if val_clean in ['0', 'NO', 'FALSE', 'OFF']:
                                compressor_enable = False
        except Exception:
            pass

    if not compressor_enable:
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

if __name__ == "__main__":
    main()
