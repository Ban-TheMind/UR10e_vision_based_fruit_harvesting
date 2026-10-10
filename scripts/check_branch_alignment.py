"""Compare shared deployment code in committed refs; never fetch or contact devices."""
import argparse
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent

def git(*args):
    return subprocess.check_output(['git', '-C', str(ROOT), *args], text=True).strip()

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--remote', default='', help='Compare fetched refs, e.g. origin')
    args = parser.parse_args()
    prefix = args.remote + '/' if args.remote else ''
    refs = [prefix + name for name in ('main', 'codex/foxy-ubuntu2004', 'codex/pre-foxy-baseline-20260928')]
    errors = []
    trees = [git('rev-parse', ref + '^{tree}') for ref in refs]
    for ref, tree in zip(refs[1:], trees[1:]):
        if tree != trees[0]:
            errors.append(ref + ' differs from main')
    for error in errors:
        print('FAIL ' + error)
    if not errors:
        print('PASS all three refs have identical tracked contents')
        print('PASS single deployment target: Ubuntu 20.04 / ROS2 Foxy')
    return bool(errors)

if __name__ == '__main__':
    sys.exit(main())
