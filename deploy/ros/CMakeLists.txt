cmake_minimum_required(VERSION 3.0.2)
project(x3plus_g2_lidar)
find_package(catkin REQUIRED)
catkin_package()
catkin_install_python(PROGRAMS g2_self_return_filter.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
