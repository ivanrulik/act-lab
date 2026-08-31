# syntax=docker/dockerfile:1.7
FROM python:3.11-slim-bookworm AS base

ENV PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1 \
    PIP_DISABLE_PIP_VERSION_CHECK=1

RUN groupadd --gid 1000 actlab \
    && useradd --uid 1000 --gid actlab --create-home actlab

WORKDIR /workspace
RUN chown actlab:actlab /workspace

FROM base AS runtime
COPY pyproject.toml README.md ./
COPY src ./src
RUN python -m pip install --no-cache-dir .
USER actlab
ENTRYPOINT ["act-lab"]
CMD ["doctor"]

FROM base AS dev
COPY pyproject.toml README.md ./
COPY src ./src
COPY tests ./tests
RUN python -m pip install --no-cache-dir -e '.[dev]'
USER actlab
CMD ["act-lab", "doctor"]

FROM dev AS ci
CMD ["pytest"]
