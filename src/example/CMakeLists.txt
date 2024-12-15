cmake_minimum_required(VERSION 3.0.2)
project(example)
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  utils
)

catkin_package(
#  INCLUDE_DIRS include
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/task_2/localization_kalman_filter.py
  scripts/task_3/control_follow_path.py
  scripts/task_3/path_planning.py
  scripts/task_2/localization.py
  scripts/task_4/experimentalDetector.py
  scripts/task_4/experimentalControl.py

  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
