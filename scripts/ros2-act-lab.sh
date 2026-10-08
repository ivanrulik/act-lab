#!/bin/sh
# Source-only CLI; no simulation/training package installation in the ROS image.
exec python -m act_lab.cli "$@"
