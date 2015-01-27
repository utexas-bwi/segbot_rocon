##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.3)
project(segbot_concert_services)

##############################################################################
# Catkin
##############################################################################

find_package(catkin REQUIRED COMPONENTS
  actionlib
  bwi_mapper
  message_generation
  geometry_msgs
  move_base_msgs
  multi_level_map_msgs
  nav_msgs
  std_msgs
  std_srvs
)
find_package(Boost REQUIRED)

catkin_python_setup()

add_message_files(
  FILES
  AvailableRobotArray.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS 
    actionlib 
    bwi_mapper
    geometry_msgs
    message_runtime
    move_base_msgs
    multi_level_map_msgs
    nav_msgs
    std_msgs
    std_srvs
)

include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
add_executable(multi_robot_navigator src/nodes/multi_robot_navigator.cpp)
target_link_libraries(multi_robot_navigator ${Boost_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(multi_robot_navigator ${PROJECT_NAME}_generate_messages_cpp)

##############################################################################
# Installs
##############################################################################

install(TARGETS multi_robot_navigator
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  PROGRAMS 
    scripts/segbot_requestor
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

foreach(dir concert services solutions)
  install(DIRECTORY ${dir}
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
endforeach()