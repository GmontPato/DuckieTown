cmake_minimum_required(VERSION 2.8.3)
project(duckie_day_solution)
find_package(catkin REQUIRED COMPONENTS rospy std_msgs sensor_msgs geometry_msgs duckietown_msgs)
catkin_package()
install(PROGRAMS
  src/duckie_day_node.py
  src/delivery_manager.py
  src/safety_monitor.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
