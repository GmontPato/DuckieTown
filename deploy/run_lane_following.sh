#!/bin/bash
# Inicia el contenedor de lane_following en el duckie
# con las modificaciones de controller.py y boundary enforcement
#
# Uso: bash run_lane_following.sh DUCKIEBOT_NAME
# Ej:  bash run_lane_following.sh duckie4

DUCKIE=${1:-duckie4}
DOCKER_HOST="tcp://${DUCKIE}.local:2375"

echo "Starting lane_following on $DUCKIE..."

# Stop existing container if running
DOCKER_HOST=$DOCKER_HOST docker stop lane_following 2>/dev/null || true
DOCKER_HOST=$DOCKER_HOST docker rm lane_following 2>/dev/null || true

# Start lane_following with modified controller files
DOCKER_HOST=$DOCKER_HOST docker run -d --name lane_following \
  -v /data:/data \
  -v /data/controller.py:/code/catkin_ws/src/dt-core/packages/lane_control/include/lane_controller/controller.py \
  -v /data/lane_controller_node.py:/code/catkin_ws/src/dt-core/packages/lane_control/src/lane_controller_node.py \
  --net=host \
  duckietown/dt-core:daffy-arm64v8 \
  roslaunch duckietown_demos lane_following.launch veh:=${DUCKIE}

echo "Container started. Waiting for initialization..."
sleep 8

# Trigger FSM transition to LANE_FOLLOWING
DOCKER_HOST=$DOCKER_HOST docker exec ros bash -c \
  '. /code/catkin_ws/devel/setup.bash && \
   rostopic pub /'"${DUCKIE}"'/joy_mapper_node/joystick_override \
   duckietown_msgs/BoolStamped \
   "{header: {seq: 0, stamp: {secs: 0, nsecs: 0}, frame_id: \"\"}, data: False}" --once'

sleep 3
echo "FSM transitioned to LANE_FOLLOWING. Applying tuned parameters..."

# Apply tuned PID parameters
DOCKER_HOST=$DOCKER_HOST docker exec ros bash -c \
  '. /code/catkin_ws/devel/setup.bash && \
   rosparam set /'"${DUCKIE}"'/lane_controller_node/v_bar 0.30 && \
   rosparam set /'"${DUCKIE}"'/lane_controller_node/k_d -10.0 && \
   rosparam set /'"${DUCKIE}"'/lane_controller_node/k_theta -8.0 && \
   rosparam set /'"${DUCKIE}"'/lane_controller_node/k_Id -0.5 && \
   rosparam set /'"${DUCKIE}"'/lane_controller_node/d_thres 0.12 && \
   rosparam set /'"${DUCKIE}"'/lane_controller_node/boundary_yellow -0.10 && \
   rosparam set /'"${DUCKIE}"'/lane_controller_node/boundary_white 0.10 && \
   rosparam set /'"${DUCKIE}"'/lane_controller_node/boundary_margin 0.04 && \
   rosparam set /'"${DUCKIE}"'/lane_controller_node/k_boundary 12.0'

echo "Done! $DUCKIE is now in auto-drive mode at 0.30 m/s"
echo "To stop: DOCKER_HOST=$DOCKER_HOST docker stop lane_following"
