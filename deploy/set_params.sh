#!/bin/bash
# Aplica los parametros PID tuneados al lane_controller_node
# Debe ejecutarse DESPUES de que el FSM este en LANE_FOLLOWING
#
# Uso: bash set_params.sh DUCKIEBOT_NAME
# Ej:  bash set_params.sh duckie4

DUCKIE=${1:-duckie4}
DOCKER_HOST="tcp://${DUCKIE}.local:2375"

echo "Setting tuned parameters for $DUCKIE..."

DOCKER_HOST=$DOCKER_HOST docker exec ros bash -c \
  ". /code/catkin_ws/devel/setup.bash && \
   rosparam set /${DUCKIE}/lane_controller_node/v_bar 0.30 && \
   rosparam set /${DUCKIE}/lane_controller_node/k_d -10.0 && \
   rosparam set /${DUCKIE}/lane_controller_node/k_theta -8.0 && \
   rosparam set /${DUCKIE}/lane_controller_node/k_Id -0.5 && \
   rosparam set /${DUCKIE}/lane_controller_node/d_thres 0.12 && \
   rosparam set /${DUCKIE}/lane_controller_node/boundary_yellow -0.10 && \
   rosparam set /${DUCKIE}/lane_controller_node/boundary_white 0.10 && \
   rosparam set /${DUCKIE}/lane_controller_node/boundary_margin 0.04 && \
   rosparam set /${DUCKIE}/lane_controller_node/k_boundary 12.0 && \
   echo 'v_bar: \$(rosparam get /${DUCKIE}/lane_controller_node/v_bar)' && \
   echo 'k_d: \$(rosparam get /${DUCKIE}/lane_controller_node/k_d)' && \
   echo 'k_theta: \$(rosparam get /${DUCKIE}/lane_controller_node/k_theta)'"

echo "Parameters applied successfully!"
