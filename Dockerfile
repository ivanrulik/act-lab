# syntax=docker/dockerfile:1.7
FROM python:3.11-slim-bookworm AS base

ENV PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1 \
    PIP_DISABLE_PIP_VERSION_CHECK=1 \
    MUJOCO_GL=egl \
    MESA_SHADER_CACHE_DIR=/tmp/act-lab-mesa-cache

RUN groupadd --gid 1000 actlab \
    && useradd --uid 1000 --gid actlab --create-home actlab

# Mesa provides a software EGL implementation for reproducible headless rendering.
RUN apt-get update \
    && apt-get install --yes --no-install-recommends libegl1 libgl1 libgl1-mesa-dri \
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
COPY pyproject.toml README.md ./
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
