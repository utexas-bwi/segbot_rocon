cmake_minimum_required(VERSION 2.8.3)
project(bwi_guidance_concert)

find_package(catkin REQUIRED)
catkin_package()

# Install scripts and configuration/launch files
install(
  PROGRAMS 
    scripts/acquire_robots
    scripts/gazebo
    scripts/server
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
foreach(dir concert launch rapps services solutions)
  install(DIRECTORY ${dir}
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
endforeach()

# Unit tests are enabled selectively
if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch)
  roslaunch_add_file_check(rapps)
  roslaunch_add_file_check(services)
  roslaunch_add_file_check(solutions)
endif()
