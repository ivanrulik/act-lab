"""Record the existing safe-control path without exposing storage frameworks."""

from collections.abc import Callable
from threading import Lock

from act_lab.application.cartesian_control import (
    CartesianDriver,
    SafeCartesianRobot,
    SafetyLimits,
)
from act_lab.domain.models import Action, CameraFrame, Observation, RobotState
from act_lab.domain.ports import EpisodeSink
from act_lab.domain.recording import EpisodeOutcome, EpisodeProvenance, EpisodeSample


class RecordingRobot(SafeCartesianRobot):
    """One reset starts one attempt; every safety report records its measured result."""

    def __init__(
        self,
        driver: CartesianDriver,
        limits: SafetyLimits,
        sink: EpisodeSink,
        provenance: EpisodeProvenance,
        images: Callable[[Observation], tuple[tuple[str, CameraFrame], ...]],
        after_sample: Callable[[int], None],
    ) -> None:
        super().__init__(driver, limits)
        self._sink = sink
        self._provenance = provenance
        self._images = images
        self._after_sample = after_sample
        self._recording_started = False
        self._finished = False
        self._finish_request: tuple[EpisodeOutcome, str] | None = None
        self._request_lock = Lock()
        self.recording_status = "recording"

    @property
    def recording_finished(self) -> bool:
        return self._finished

    def request_finish(self, outcome: EpisodeOutcome, reason: str) -> None:
        """Queue viewer-thread input; all storage stays on the control thread."""
        if (
            outcome
            not in {
                EpisodeOutcome.SUCCESS,
                EpisodeOutcome.FAILURE,
                EpisodeOutcome.DISCARDED,
            }
            or not reason.strip()
        ):
            raise ValueError("recording finish requires a terminal label and reason")
        with self._request_lock:
            if self._finish_request is None and not self._finished:
                self._finish_request = (outcome, reason)

    def finish(self, outcome: EpisodeOutcome, reason: str) -> None:
        """Finalize once on the control thread, honoring pending operator input."""
        if self._finished:
            return
        with self._request_lock:
            selected = self._finish_request or (outcome, reason)
        self._sink.stop(*selected)
        self._finished = True
        self.recording_status = selected[0].value

    def reset(self, seed: int) -> Observation:
        if self._recording_started:
            raise RuntimeError("start a new recording for each reset")
        if seed != self._provenance.seed:
            raise ValueError("reset seed disagrees with recording provenance")
        observation = super().reset(seed)
        self._sink.start(self._provenance, observation.timestamp_ns)
        self._recording_started = True
        return observation

    def command(self, action: Action) -> RobotState:
        if not self._recording_started:
            raise RuntimeError("reset before recording commands")
        state = super().command(action)
        if self._finished:
            return state
        report = self.last_report
        assert report is not None
        observation = self.observe()
        try:
            self._sink.append(
                EpisodeSample(observation, report, self._images(observation))
            )
            self._after_sample(observation.timestamp_ns)
            with self._request_lock:
                requested = self._finish_request
            if requested is not None:
                self.finish(*requested)
        except BaseException:
            # Recording failure ends acquisition and applies the same disabled
            # grasp-preserving hold before propagating to session cleanup.
            try:
                super().command(
                    Action(
                        state.timestamp_ns,
                        state.end_effector_pose,
                        state.gripper_position,
                        False,
                    )
                )
            finally:
                self._sink.interrupt()
            raise
        return state
