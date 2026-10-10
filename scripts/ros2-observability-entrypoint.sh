#!/bin/sh
set -e
. /opt/ros/jazzy/setup.sh
. /opt/crisp/install/setup.sh
. /opt/simulation/install/setup.sh
export PYTHONPATH=/workspace/src:${PYTHONPATH:-}
exec "$@"
