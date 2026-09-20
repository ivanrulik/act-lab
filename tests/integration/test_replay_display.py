"""Opt-in smoke check for the OpenCV replay window in the Compose UI image."""

import os
import subprocess
import sys

import pytest


@pytest.mark.skipif(
    os.environ.get("ACT_LAB_TEST_DISPLAY") != "1",
    reason="requires explicit X11 display validation in the Compose UI image",
)
def test_opencv_replay_window_opens_and_closes() -> None:
    # Qt aborts the interpreter on a missing platform dependency; isolate it so
    # that pytest reports an actionable failure instead of terminating itself.
    result = subprocess.run(
        [
            sys.executable,
            "-c",
            "import cv2, numpy as np; "
            "image = np.zeros((64, 64, 3), dtype=np.uint8); "
            "cv2.imshow('ACT Lab replay smoke', image); "
            "cv2.waitKey(100); cv2.destroyAllWindows()",
        ],
        capture_output=True,
        text=True,
        timeout=15,
        check=False,
    )
    assert result.returncode == 0, result.stderr
