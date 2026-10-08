#!/bin/bash
set -e
source /opt/ros/jazzy/setup.bash
source /opt/crisp/install/setup.bash
source /opt/bench/install/setup.bash
export PATH="/opt/bench/install/act_lab_crisp_bench/lib/act_lab_crisp_bench:$PATH"
exec "$@"
