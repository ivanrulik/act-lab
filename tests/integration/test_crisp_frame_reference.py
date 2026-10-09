"""Regenerate stationary geometry in the ROS-free simulation environment."""

import subprocess
import sys


def test_stationary_mjcf_reference_matches_local_geometry() -> None:
    subprocess.run(
        [sys.executable, "scripts/crisp-frame-reference.py", "--check"], check=True
    )
