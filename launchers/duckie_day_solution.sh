#!/bin/bash
source /environment.sh
source /opt/ros/noetic/setup.bash
source /code/catkin_ws/devel/setup.bash --extend

dt-launchfile-init

export DELIVERIES_PATH="/data/deliveries.json"
dt-exec-BG roscore

dt-exec-BG roslaunch --wait duckie_day_solution duckie_day_competition.launch \
    veh:="${VEHICLE_NAME}" \
    deliveries:="${DELIVERIES_PATH}"

# Tune lane controller params for smooth driving
dt-exec-BG bash -c "
    sleep 20
    # Wider theta thresholds (phi often exceeds [-0.5, 0.75])
    rosparam set /${VEHICLE_NAME}/lane_controller_node/theta_thres_min -2.0
    rosparam set /${VEHICLE_NAME}/lane_controller_node/theta_thres_max 2.0
    # Lower PID gains to reduce oscillation
    rosparam set /${VEHICLE_NAME}/lane_controller_node/k_d -1.5
    rosparam set /${VEHICLE_NAME}/lane_controller_node/k_theta -1.2
    # Wider integral bounds
    rosparam set /${VEHICLE_NAME}/lane_controller_node/integral_bounds/phi/bot -2.0
    rosparam set /${VEHICLE_NAME}/lane_controller_node/integral_bounds/phi/top 2.0
    # Apply the new params
    rosservice call /${VEHICLE_NAME}/lane_controller_node/request_parameters_update '{}'
    echo 'Tuned lane controller params'
"

# Trigger LANE_FOLLOWING via joystick_override
dt-exec-BG bash -c "
    sleep 15
    rostopic pub --once /${VEHICLE_NAME}/joy_mapper_node/joystick_override duckietown_msgs/BoolStamped '{header: {stamp: now}, data: false}'
    echo 'Set joystick_override=false -> triggering LANE_FOLLOWING'
"

dt-launchfile-join
