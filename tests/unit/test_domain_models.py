from act_lab.domain import Action, CameraFrame, Pose


def test_action_is_an_immutable_cartesian_intent() -> None:
    pose = Pose("world", (0.1, 0.2, 0.3), (1.0, 0.0, 0.0, 0.0))
    action = Action(42, pose, 0.5, True)
    assert action.target_pose == pose
    assert action.enabled


def test_camera_frame_is_framework_neutral_rgb_data() -> None:
    frame = CameraFrame(10, 2, 1, b"\x00\x01\x02\x03\x04\x05")
    assert frame.width == 2
    assert frame.rgb_bytes[-1] == 5
