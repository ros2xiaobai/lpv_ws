cmake_minimum_required(VERSION 3.0.2)
project(lpv_attack_sim)

find_package(catkin REQUIRED COMPONENTS
  gazebo_msgs
  geometry_msgs
  mavros_msgs
  nav_msgs
  rospy
  std_msgs
  tf
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/setpoint_fdi_attack_demo.py
  scripts/plot_attack_results.py
  scripts/make_attack_video.py
  scripts/gps_spoof_attack_demo.py
  scripts/make_gps_spoof_video.py
  scripts/true_gps_input_attack_demo.py
  scripts/make_true_gps_input_video.py
  scripts/gazebo_attack_preview.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
