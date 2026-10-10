#!/bin/bash
set -e
source /opt/ros/jazzy/setup.bash
source /opt/crisp/install/setup.bash
source /opt/simulation/install/setup.bash
exec "$@"
