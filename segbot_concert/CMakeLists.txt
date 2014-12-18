##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.3)
project(segbot_gazebo_concert)

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
    scripts/segbot_pond
    scripts/segbot_stroll
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

foreach(dir concert rapps services solutions)
  install(DIRECTORY ${dir}
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
endforeach()
