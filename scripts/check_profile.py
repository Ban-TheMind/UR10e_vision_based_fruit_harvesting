#!/usr/bin/env python3
"""Validate deployment inputs with plain Python before loading ROS."""
from pathlib import Path
import sys

# This pre-build command deliberately imports the source package, without ROS.
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src/harvesting_config'))
from harvesting_config import deployment

if __name__ == '__main__':
    try:
        resolved = deployment.resolve_deployment(
            sys.argv[1], sys.argv[2], deployment.parse_overrides(sys.argv[3:]))
        print('Profile valid; mode=' + sys.argv[2] + '; execution=' + str(resolved.plan['allow_execution']))
    except (ValueError, KeyError, OSError, TypeError, IndexError) as error:
        sys.exit('Profile error: ' + str(error))
