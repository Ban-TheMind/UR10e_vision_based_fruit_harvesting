"""Compare shared deployment code in committed refs; never fetch or contact devices."""
import argparse
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
SHARED = (
    'src/camera/camera/geometry.py',
    'src/camera/camera/depth_utils.py',
    'src/camera/camera/tf_utils.py',
    'src/camera/camera/calibration/camera_to_base.json',
    'src/camera/test/test_site_geometry.py',
    'src/gripper/gripper/robotiq_sdk.py',
    'src/robotiq_sdk_bridge',
    'scripts/check_branch_alignment.py',
)

def git(*args):
    return subprocess.check_output(['git', '-C', str(ROOT), *args], text=True).strip()

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--remote', default='', help='Compare fetched refs, e.g. origin')
    args = parser.parse_args()
    prefix = args.remote + '/' if args.remote else ''
    refs = [prefix + name for name in ('main', 'codex/foxy-ubuntu2004', 'codex/pre-foxy-baseline-20260928')]
    errors = []
    for path in SHARED:
        objects = [git('rev-parse', ref + ':' + path) for ref in refs]
        if len(set(objects)) != 1:
            errors.append(path)
    if git('rev-parse', refs[0] + '^{tree}') != git('rev-parse', refs[2] + '^{tree}'):
        errors.append('historical branch tip differs from main')
    for error in errors:
        print('FAIL ' + error)
    if not errors:
        print('PASS all three refs share calibration, geometry and official gripper SDK')
        print('PASS historical branch tip matches main; Foxy environment adapters remain separate')
    return bool(errors)

if __name__ == '__main__':
    sys.exit(main())
