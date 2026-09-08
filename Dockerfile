# syntax=docker/dockerfile:1.7
FROM python:3.11-slim-bookworm AS base

ENV PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1 \
    PIP_DISABLE_PIP_VERSION_CHECK=1 \
    MUJOCO_GL=egl \
    MESA_SHADER_CACHE_DIR=/tmp/act-lab-mesa-cache \
    MPLCONFIGDIR=/tmp/act-lab-matplotlib \
    ACT_LAB_HAND_MODEL=/opt/act-lab/models/hand_landmarker.task

# Official MediaPipe float16 Hand Landmarker model, pinned independently of pip.
ADD --checksum=sha256:fbc2a30080c3c557093b5ddfc334698132eb341044ccee322ccf8bcf3607cde1 \
    https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/1/hand_landmarker.task \
    /opt/act-lab/models/hand_landmarker.task
RUN chmod 0444 /opt/act-lab/models/hand_landmarker.task

RUN groupadd --gid 1000 actlab \
    && useradd --uid 1000 --gid actlab --create-home actlab

# Mesa provides a software EGL implementation for reproducible headless rendering.
RUN apt-get update \
    && apt-get install --yes --no-install-recommends \
        libegl1 libgl1 libgl1-mesa-dri libgles2 libglib2.0-0 \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace
RUN chown actlab:actlab /workspace

FROM base AS runtime
COPY pyproject.toml README.md ./
COPY requirements ./requirements
COPY src ./src
COPY configs ./configs
RUN python -m pip install --no-cache-dir \
    --constraint requirements/constraints-py311.txt .
USER actlab
ENTRYPOINT ["act-lab"]
CMD ["doctor"]

FROM base AS dev
COPY pyproject.toml README.md compose.yaml ./
COPY requirements ./requirements
COPY src ./src
COPY tests ./tests
COPY configs ./configs
RUN python -m pip install --no-cache-dir \
    --constraint requirements/constraints-py311.txt -e '.[dev]'
USER actlab
CMD ["act-lab", "doctor"]

FROM dev AS ci
CMD ["pytest"]

FROM dev AS ui
USER root
# pynput's Linux backend depends on evdev, whose extension is built against
# the kernel userspace headers when no wheel is available for this platform.
RUN apt-get update \
    && apt-get install --yes --no-install-recommends build-essential linux-libc-dev \
    && rm -rf /var/lib/apt/lists/*
RUN python -m pip install --no-cache-dir \
    --constraint requirements/constraints-py311.txt -e '.[dev,ui]'
USER actlab
