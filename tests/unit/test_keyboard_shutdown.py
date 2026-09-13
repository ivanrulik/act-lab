from unittest.mock import Mock

import pytest

from act_lab.adapters.mujoco.keyboard import (
    X11KeyboardAdapter,
    _close_viewer_and_wait,
)


def test_shutdown_waits_for_render_destruction_not_running_flag() -> None:
    handle = Mock()
    handle.is_running.return_value = False
    handle._sim.side_effect = [object(), object(), None]
    _close_viewer_and_wait(handle)
    handle.close.assert_called_once()
    assert handle._sim.call_count == 3


def test_render_shutdown_timeout_is_explicit() -> None:
    handle = Mock()
    with pytest.raises(RuntimeError, match="shutdown deadline"):
        _close_viewer_and_wait(handle, timeout_s=0.0)
    handle.close.assert_called_once()


def test_keyboard_shutdown_joins_workers_before_closing_display() -> None:
    operator = Mock()
    adapter = X11KeyboardAdapter(operator)
    calls = Mock()
    adapter._listener = calls.listener
    adapter._thread = calls.monitor
    adapter._display = calls.display
    calls.monitor.is_alive.return_value = False
    adapter.stop()
    operator.update_focus.assert_called_once_with(False)
    assert [call[0] for call in calls.mock_calls] == [
        "listener.stop", "listener.is_alive",
        "listener._display_stop.record_disable_context",
        "listener._display_stop.flush", "listener.join",
        "monitor.join", "monitor.is_alive",
        "display.close",
    ]


def test_stop_before_start_is_safe() -> None:
    adapter = X11KeyboardAdapter(Mock())
    adapter.stop()
    adapter.stop()
