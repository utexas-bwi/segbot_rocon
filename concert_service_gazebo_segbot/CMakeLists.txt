##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.3)
project(concert_service_gazebo)

##############################################################################
# Catkin
##############################################################################

find_package(catkin REQUIRED)
catkin_package()
catkin_python_setup()

##############################################################################
# Installs
##############################################################################


install(
    PROGRAMS 
    #        scripts/gazebo_turtlebot_manager
             scripts/gazebo_segbot_manager
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY services
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
