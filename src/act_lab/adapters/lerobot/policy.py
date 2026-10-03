"""Load a trained LeRobot ACT checkpoint for closed-loop inference."""

from __future__ import annotations

from importlib import import_module
from pathlib import Path
from typing import Any

import numpy as np
from numpy.typing import NDArray

from act_lab.domain import Action, Observation, Pose


def _bounded_gripper_position(value: float) -> float:
    """Saturate small ACT extrapolations to the physical gripper range."""
    return min(1.0, max(0.0, value))


class LeRobotACTPolicy:
    """Translate ACT's normalized LeRobot interface into domain actions."""

    def __init__(
        self,
        checkpoint: Path,
        device: str,
        cameras: tuple[str, ...],
        n_action_steps: int = 1,
    ) -> None:
        try:
            torch = import_module("torch")
            act_module = import_module("lerobot.policies.act.modeling_act")
            policy_factory = import_module("lerobot.policies.factory")
        except ImportError as error:
            raise RuntimeError(
                "ACT evaluation requires the pinned training image"
            ) from error
        if device not in {"cpu", "cuda"}:
            raise ValueError("device must be cpu or cuda")
        if device == "cuda" and not torch.cuda.is_available():
            raise ValueError("CUDA was requested but is unavailable")
        model_path = (
            checkpoint / "pretrained_model"
            if (checkpoint / "pretrained_model").is_dir()
            else checkpoint
        )
        self._policy = act_module.ACTPolicy.from_pretrained(
            str(model_path), device=device, n_action_steps=n_action_steps
        )
        if self._policy.config.device != device:
            raise RuntimeError(
                f"ACT checkpoint resolved device {self._policy.config.device!r}; "
                f"requested {device!r}"
            )
        self._policy.to(device)
        self._policy.eval()
        self._preprocessor, self._postprocessor = (
            policy_factory.make_pre_post_processors(
                self._policy.config,
                pretrained_path=str(model_path),
                preprocessor_overrides={
                    "device_processor": {"device": device}
                },
                postprocessor_overrides={"device_processor": {"device": device}},
            )
        )
        self._torch = torch
        self._device = device
        self._cameras = cameras

    def reset(self) -> None:
        """Clear ACT's queued chunk actions between independent episodes."""
        self._policy.reset()

    def act(
        self, observation: Observation, images: dict[str, NDArray[np.uint8]]
    ) -> Action:
        torch = self._torch
        state = observation.robot
        inputs: dict[str, Any] = {
            "observation.state": torch.tensor(
                (*state.joint_positions_rad, state.gripper_position),
                dtype=torch.float32,
                device=self._device,
            ).unsqueeze(0)
        }
        for camera in self._cameras:
            image = images[camera]
            inputs[f"observation.images.{camera}"] = (
                torch.from_numpy(np.ascontiguousarray(image))
                .to(device=self._device, dtype=torch.float32)
                .permute(2, 0, 1)
                .div_(255)
                .unsqueeze(0)
            )
        processed_inputs = self._preprocessor(inputs)
        with torch.inference_mode():
            output = self._policy.select_action(processed_inputs)
            output = self._postprocessor(output)
        values = output.detach().to("cpu").reshape(-1).tolist()
        if len(values) != 8 or not np.isfinite(values).all():
            raise RuntimeError("ACT policy returned an invalid 8-value action")
        position = (float(values[0]), float(values[1]), float(values[2]))
        quaternion = (
            float(values[3]),
            float(values[4]),
            float(values[5]),
            float(values[6]),
        )
        return Action(
            timestamp_ns=observation.timestamp_ns,
            target_pose=Pose(
                "world",
                position,
                quaternion,
            ),
            gripper_position=_bounded_gripper_position(float(values[7])),
            enabled=True,
        )
