#!/usr/bin/env python3
"""
Spline compressor wrapper supporting multiple backends:
- COMPRESSOR_ENABLE = 1: perpbisector (Python, faster startup)
- COMPRESSOR_ENABLE = 2: liscio (C99, industrial-grade)

Routes G-code through the selected compressor based on INI setting.
"""
import sys
import os
import subprocess

def get_compressor_version():
    """Read COMPRESSOR_ENABLE from INI_FILE_NAME environment variable."""
    version = 1  # default
    ini_path = os.environ.get("INI_FILE_NAME")

    if ini_path and os.path.exists(ini_path):
        try:
            with open(ini_path, 'r') as f:
                current_section = None
                for line in f:
                    clean = line.split('#')[0].split(';')[0].strip()
                    if not clean:
                        continue
                    if clean.startswith('[') and clean.endswith(']'):
                        current_section = clean[1:-1].strip().upper()
                    elif '=' in clean and current_section in ['TRAJ', 'FILTER']:
                        key, val = clean.split('=', 1)
                        if key.strip().upper() == 'COMPRESSOR_ENABLE':
                            try:
                                version = int(val.strip())
                            except ValueError:
                                pass
        except Exception:
            pass

    return version

def main():
    if len(sys.argv) < 2:
        print("Usage: spline_compressor_wrapper.py <gcode_file>", file=sys.stderr)
        sys.exit(1)

    gcode_file = sys.argv[1]
    version = get_compressor_version()

    script_dir = os.path.dirname(os.path.abspath(__file__))

    if version == 2:
        # Use liscio
        try:
            result = subprocess.run(
                ['liscio_compress', '--tol_xyz=0.01', gcode_file],
                capture_output=False,
                check=True
            )
            sys.exit(result.returncode)
        except (FileNotFoundError, subprocess.CalledProcessError) as e:
            print(f"[warn] liscio_compress failed, falling back to perpbisector",
                  file=sys.stderr)
            version = 1

    if version == 1 or version != 2:
        # Use perpbisector (Python)
        perpbisector_script = os.path.join(script_dir, 'spline_compressor.py')
        try:
            result = subprocess.run(
                ['python3', perpbisector_script, gcode_file],
                check=True
            )
            sys.exit(result.returncode)
        except (FileNotFoundError, subprocess.CalledProcessError) as e:
            print(f"Error: Failed to run compressor: {e}", file=sys.stderr)
            sys.exit(1)

if __name__ == "__main__":
    main()
