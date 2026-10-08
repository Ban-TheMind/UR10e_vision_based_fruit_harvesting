"""Record the configured sensor topics rather than fixed example names."""
import os
from pathlib import Path
import sys
import yaml

config = yaml.safe_load(Path(sys.argv[1]).read_text())
camera = config['camera_server']['ros__parameters']
topics = list(dict.fromkeys(['/tf', '/tf_static', '/joint_states',
    camera['color_topic'], camera['depth_topic'], camera['camera_info_topic']]))
os.execvp('ros2', ['ros2', 'bag', 'record', *topics, *sys.argv[2:]])
