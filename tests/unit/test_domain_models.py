from act_lab.domain import Action, Pose


def test_action_is_an_immutable_cartesian_intent() -> None:
    pose = Pose((0.1, 0.2, 0.3), (1.0, 0.0, 0.0, 0.0))
    action = Action(42, pose, 0.5, True)
    assert action.target_pose == pose
    assert action.enabled

