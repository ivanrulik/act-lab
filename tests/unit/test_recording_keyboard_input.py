from pathlib import Path

from act_lab.adapters.mujoco.config import SimulationConfig
from act_lab.adapters.mujoco.keyboard import KeyboardTeleoperator, X11KeyboardAdapter


def test_function_key_listener_requires_focus_and_press():
    config = SimulationConfig.load(Path("configs/sim/ur5e_pick_place.toml"))
    teleoperator = KeyboardTeleoperator(config.keyboard, config.control)
    received = []
    adapter = X11KeyboardAdapter(teleoperator, received.append)
    adapter.handle_key("f6", True)
    assert received == []
    teleoperator.update_focus(True)
    for name in ("f6", "f7", "f8"):
        adapter.handle_key(name, True)
        adapter.handle_key(name, False)
    assert received == [295, 296, 297]
    teleoperator.update_focus(False)
    adapter.handle_key("f8", True)
    assert received == [295, 296, 297]
