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
