##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.3)
project(segbot_navigation_concert)

##############################################################################
# Catkin
##############################################################################

find_package(catkin REQUIRED)
catkin_package()

##############################################################################
# Installs
##############################################################################

install(
  PROGRAMS 
    scripts/multi_robot_navigation
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

foreach(dir concert services solutions)
  install(DIRECTORY ${dir}
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
endforeach()
