#!/usr/bin/env python3
"""
Entry point.

    PEARL_STUB=1 python3 run.py     # laptop: synthetic robot, no ROS2 (default)
    PEARL_STUB=0 python3 run.py     # on the Pi, after `source /opt/ros/humble/setup.bash`

See README.md.
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from pearl.app import main  # noqa: E402

if __name__ == "__main__":
    main()
