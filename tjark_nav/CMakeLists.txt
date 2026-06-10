cmake_minimum_required(VERSION 3.0.2)
project(tjark_nav)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  move_base_msgs
  gazebo_msgs
  actionlib
  tf
  tf2_ros
  tf2_geometry_msgs
  sensor_msgs
  visualization_msgs
  cv_bridge
  message_filters
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/pedestrian_controller.py
  scripts/odom_tf_broadcaster.py
  scripts/navigation_demo.py
  scripts/marker_detector.py
  scripts/waypoint_saver.py
  maps/generate_map.py
  maps/generate_cafe_map.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

foreach(dir launch config worlds maps rviz models)
  install(DIRECTORY ${dir}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach()
