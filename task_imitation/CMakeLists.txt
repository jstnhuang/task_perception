cmake_minimum_required(VERSION 2.8.3)
project(task_imitation)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  eigen_conversions
  geometry_msgs
  mongodb_store
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  pcl_conversions
  rapid_collision
  rapid_manipulation
  rapid_pr2
  rapid_robot
  rapid_ros
  rapid_utils
  rapid_viz
  robot_markers
  roscpp
  surface_perception
  task_db
  task_perception
  task_perception_msgs
  task_utils
  trajectory_msgs
  transform_graph
  urdf
  visualization_msgs
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
find_package(PCL REQUIRED)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_BUILD_TYPE RelWithDebInfo)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   task_perception_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
    task_imitation_bimanual_manipulation
    task_imitation_collision_checker
    task_imitation_demo_state
    task_imitation_grasp_planner
    task_imitation_grasp_planning_context
    task_imitation_hand_state_machine
    task_imitation_ik
    task_imitation_object_initialization
    task_imitation_program_executor
    task_imitation_program_generator
    task_imitation_program_server
    task_imitation_program_slice
    task_imitation_program_step
  CATKIN_DEPENDS
    actionlib
    eigen_conversions
    geometry_msgs
    mongodb_store
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    pcl_conversions
    rapid_collision
    rapid_manipulation
    rapid_pr2
    rapid_robot
    rapid_ros
    rapid_utils
    rapid_viz
    robot_markers
    roscpp
    surface_perception
    task_db
    task_perception
    task_perception_msgs
    task_utils
    trajectory_msgs
    transform_graph
    urdf
    visualization_msgs
  DEPENDS Boost PCL
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(task_imitation_bimanual_manipulation src/bimanual_manipulation.cpp)
add_dependencies(task_imitation_bimanual_manipulation ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(task_imitation_bimanual_manipulation
  ${catkin_LIBRARIES}
)

add_library(task_imitation_collision_checker src/collision_checker.cpp)
add_dependencies(task_imitation_collision_checker ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(task_imitation_collision_checker
  ${catkin_LIBRARIES}
)

add_library(task_imitation_demo_state src/demo_state.cpp)
add_dependencies(task_imitation_demo_state ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(task_imitation_demo_state
  ${catkin_LIBRARIES}
)

add_library(task_imitation_grasp_planner src/grasp_planner.cpp)
add_dependencies(task_imitation_grasp_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(task_imitation_grasp_planner
  task_imitation_grasp_planning_context
  task_imitation_ik
  task_imitation_motion_planning
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_library(task_imitation_grasp_planning_context src/grasp_planning_context.cpp)
add_dependencies(task_imitation_grasp_planning_context ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(task_imitation_grasp_planning_context
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_library(task_imitation_hand_state_machine src/hand_state_machine.cpp)
add_dependencies(task_imitation_hand_state_machine ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(task_imitation_hand_state_machine
  task_imitation_collision_checker
  task_imitation_demo_state
  ${catkin_LIBRARIES}
)

add_library(task_imitation_ik src/ik.cpp)
add_dependencies(task_imitation_ik ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(task_imitation_ik
  ${catkin_LIBRARIES}
)

add_library(task_imitation_motion_planning src/motion_planning.cpp)
add_dependencies(task_imitation_motion_planning ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(task_imitation_motion_planning
  task_imitation_ik
  ${catkin_LIBRARIES}
)

add_library(task_imitation_object_initialization src/object_initialization.cpp)
add_dependencies(task_imitation_object_initialization ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(task_imitation_object_initialization
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_library(task_imitation_program_executor src/program_executor.cpp)
add_dependencies(task_imitation_program_executor ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(task_imitation_program_executor
  task_imitation_bimanual_manipulation
  task_imitation_motion_planning
  task_imitation_program_slice
  task_imitation_program_step
  ${catkin_LIBRARIES}
)

add_library(task_imitation_program_generator src/program_generator.cpp)
add_dependencies(task_imitation_program_generator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(task_imitation_program_generator
  task_imitation_collision_checker
  task_imitation_demo_state
  task_imitation_grasp_planner
  task_imitation_grasp_planning_context
  task_imitation_hand_state_machine
  task_imitation_ik
  ${catkin_LIBRARIES}
)

add_library(task_imitation_program_server src/program_server.cpp)
add_dependencies(task_imitation_program_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(task_imitation_program_server
  task_imitation_object_initialization
  task_imitation_program_executor
  task_imitation_program_generator
  ${catkin_LIBRARIES}
)

add_library(task_imitation_program_slice src/program_slice.cpp)
add_dependencies(task_imitation_program_slice ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(task_imitation_program_slice
  ${catkin_LIBRARIES}
)

add_library(task_imitation_program_step src/program_step.cpp)
add_dependencies(task_imitation_program_step ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(task_imitation_program_step
  ${catkin_LIBRARIES}
)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(task_imitation_program_server_node src/program_server_node.cpp)
set_target_properties(task_imitation_program_server_node PROPERTIES OUTPUT_NAME program_server PREFIX "")
add_dependencies(task_imitation_program_server_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(task_imitation_program_server_node
  task_imitation_program_server
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_add_gtest(task_imitation_bimanual_manipulation_test test/bimanual_manipulation_test.cpp)
  target_link_libraries(task_imitation_bimanual_manipulation_test
    task_imitation_bimanual_manipulation
    ${catkin_LIBRARIES})

  catkin_add_gtest(task_imitation_program_executor_test test/program_executor_test.cpp)
  target_link_libraries(task_imitation_program_executor_test
    task_imitation_program_executor
    ${catkin_LIBRARIES})

  catkin_add_gtest(task_imitation_program_slice_test test/program_slice_test.cpp)
  target_link_libraries(task_imitation_program_slice_test
    task_imitation_program_slice
    ${catkin_LIBRARIES})
endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
