#!/usr/bin/env python3
"""Validate launch inputs before loading ROS or contacting devices."""
import importlib.util
from pathlib import Path
import sys
root = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('deployment', root / 'src/harvesting_bringup/launch/deployment.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
if __name__ == '__main__':
    try:
        profile = module.load_profile(sys.argv[1])
        overrides = dict(arg.split(':=', 1) for arg in sys.argv[3:])
        if set(overrides) - {'motion_enabled', 'rviz'}:
            raise ValueError('Supported overrides: motion_enabled:=true/false rviz:=true/false')
        module.boolean(overrides.get('rviz', 'true'))
        plan = module.build_plan(profile, sys.argv[2], overrides.get('motion_enabled', 'false'))
        print('Profile valid; mode=' + sys.argv[2] + '; execution=' + str(plan['allow_execution']))
    except (ValueError, KeyError, OSError, TypeError, IndexError) as error:
        sys.exit('Profile error: ' + str(error))
