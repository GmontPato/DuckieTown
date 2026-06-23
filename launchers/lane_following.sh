#!/bin/bash
source /environment.sh
dt-launchfile-init

# your code here
dt-exec-FG python3 /code/solution/scripts/lane_follower.py

dt-launchfile-join
