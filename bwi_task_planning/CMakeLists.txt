cmake_minimum_required(VERSION 2.8.3)
project(bwi_task_planning)

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
foreach(dir concert rapps services solutions)
  install(DIRECTORY ${dir}
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
endforeach()
