"""The optional execution path must not load ROS or MuJoCo into normal clients."""

import subprocess
import sys


def test_simulation_runtime_import_is_lazy():
    subprocess.run(
        [
            sys.executable,
            "-c",
            "import sys; import act_lab.domain; import act_lab.application; "
            "import act_lab.cli; import act_lab.adapters.ros2.simulation; "
            "import act_lab.adapters.ros2.simulation_smoke; "
            "assert not any(name in sys.modules for name in "
            "('rclpy', 'mujoco', 'pinocchio', 'act_lab_interfaces.msg'))",
        ],
        check=True,
    )


def test_missing_ros_simulation_has_actionable_profile_hint(tmp_path):
    import pytest

    from act_lab.adapters.ros2.simulation_smoke import MotionSession

    with pytest.raises(RuntimeError, match="profile ros2-simulation"):
        MotionSession(tmp_path)


def test_nonrecording_expert_runs_without_storage_dependencies(tmp_path):
    from pathlib import Path

    config = tmp_path / "short.toml"
    config.write_text(
        Path("configs/sim/ur5e_pick_place.toml")
        .read_text()
        .replace("episode_steps = 500", "episode_steps = 3")
    )
    script = """
import importlib.abc
import sys

class NoStorage(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path=None, target=None):
        if fullname == "mcap" or fullname.startswith("mcap."):
            raise ImportError("MCAP unavailable")
        if fullname == "google.protobuf" or fullname.startswith("google.protobuf."):
            raise ImportError("Protobuf unavailable")

sys.meta_path.insert(0, NoStorage())
from act_lab.cli import main
assert main(["sim", "expert", "--config", sys.argv[1], "--episodes", "1",
             "--min-success-rate", "0", "--json"]) == 0
assert "act_lab.adapters.mcap.recording" not in sys.modules
"""
    subprocess.run([sys.executable, "-c", script, str(config)], check=True)


def test_recording_transport_imports_are_lazy():
    subprocess.run(
        [
            sys.executable,
            "-c",
            "import sys; import act_lab.adapters.ros2.recording; "
            "import act_lab.adapters.ros2.recording_demo; "
            "import act_lab.adapters.ros2.recording_contracts; "
            "assert not any(n in sys.modules for n in "
            "('rclpy', 'rosbag2_py', 'mcap', 'google.protobuf', "
            "'act_lab_interfaces.msg', 'mujoco'))",
        ],
        check=True,
    )


def test_missing_ros_recording_has_compose_hint(tmp_path):
    import pytest

    from act_lab.adapters.ros2.recording_demo import run_recording

    with pytest.raises(RuntimeError, match="profile ros2-recording"):
        run_recording(tmp_path / "recording")
