cmake_minimum_required(VERSION 2.8.3)
project(task_perception)

## Compile as C++11, supported in ROS Kinetic and newer
#add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  dbot_ros_msgs
  eigen_conversions
  geometry_msgs
  image_geometry
  message_filters
  mongodb_store
  pcl_conversions
  rapid_collision
  robot_markers
  rosbag
  roscpp
  roslib
  rospy
  sensor_msgs
  skin_segmentation_msgs
  task_db
  task_utils
  tf
  transform_graph
  urdf
  visualization_msgs
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
find_package(PCL REQUIRED)
#find_package(CUDA REQUIRED)
#add_definitions(-DCUDA_BUILD)

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
#   std_msgs  # Or other packages containing msgs
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
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
    pbi_annotator_server
    pbi_contact_detection
    pbi_database
    pbi_default_skeleton
    pbi_demo_model
    pbi_demo_runtime
    pbi_demo_visualizer
    pbi_lazy_object_model
    pbi_multi_object_tracker
    pbi_pcl_utils
    pbi_pose_utils
    pbi_pr2_gripper_model
    pbi_record_video_action_server
    pbi_task_perception_context
    pbi_video_scrubber
  CATKIN_DEPENDS
    actionlib
    dbot_ros_msgs
    eigen_conversions
    geometry_msgs
    image_geometry
    message_filters
    mongodb_store
    pcl_conversions
    rapid_collision
    robot_markers
    rosbag
    roscpp
    roslib
    rospy
    sensor_msgs
    skin_segmentation_msgs
    task_db
    task_perception_msgs
    task_utils
    tf
    transform_graph
    urdf
    visualization_msgs
  DEPENDS
    Boost
    PCL
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(pbi_annotator_server src/annotator_server.cpp)
add_dependencies(pbi_annotator_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pbi_annotator_server
  pbi_database
  pbi_demo_model
  pbi_demo_runtime
  pbi_demo_visualizer
  pbi_multi_object_tracker
  pbi_video_scrubber
  ${catkin_LIBRARIES}
)

add_library(pbi_contact_detection src/contact_detection.cpp)
add_dependencies(pbi_contact_detection ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pbi_contact_detection
  pbi_pcl_utils
  pbi_pose_utils
  pbi_task_perception_context
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_library(pbi_database src/database.cpp)
add_dependencies(pbi_database ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pbi_database
  ${catkin_LIBRARIES}
)

add_library(pbi_demo_model src/demo_model.cpp)
add_dependencies(pbi_demo_model ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pbi_demo_model
  ${catkin_LIBRARIES}
)

add_library(pbi_demo_runtime src/demo_runtime.cpp)
add_dependencies(pbi_demo_runtime ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pbi_demo_runtime
  pbi_contact_detection
  pbi_default_skeleton
  pbi_demo_model
  pbi_demo_visualizer
  pbi_multi_object_tracker
  pbi_video_scrubber
  ${catkin_LIBRARIES}
)

add_library(pbi_demo_visualizer src/demo_visualizer.cpp)
add_dependencies(pbi_demo_visualizer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pbi_demo_visualizer
  pbi_pr2_gripper_model
  pbi_task_perception_context
  ${catkin_LIBRARIES}
)

add_library(pbi_default_skeleton src/default_skeleton.cpp)
add_dependencies(pbi_default_skeleton ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pbi_default_skeleton
  ${catkin_LIBRARIES}
)

add_library(pbi_lazy_object_model src/lazy_object_model.cpp)
add_dependencies(pbi_lazy_object_model ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pbi_lazy_object_model
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_library(pbi_multi_object_tracker src/multi_object_tracker.cpp)
add_dependencies(pbi_multi_object_tracker ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pbi_multi_object_tracker
  ${catkin_LIBRARIES}
)

add_library(pbi_pcl_utils src/pcl_utils.cpp)
add_dependencies(pbi_pcl_utils ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pbi_pcl_utils
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_library(pbi_pose_utils src/pose_utils.cpp)
add_dependencies(pbi_pose_utils ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pbi_pose_utils
  ${catkin_LIBRARIES}
)

add_library(pbi_pr2_gripper_model src/pr2_gripper_model.cpp)
add_dependencies(pbi_pr2_gripper_model ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pbi_pr2_gripper_model
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_library(pbi_record_video_action_server src/record_video_action_server.cpp)
add_dependencies(pbi_record_video_action_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pbi_record_video_action_server
  ${catkin_LIBRARIES}
)

add_library(pbi_task_perception_context src/task_perception_context.cpp)
add_dependencies(pbi_task_perception_context ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pbi_task_perception_context
  pbi_lazy_object_model
  ${catkin_LIBRARIES}
)

add_library(pbi_video_scrubber src/video_scrubber.cpp)
add_dependencies(pbi_video_scrubber ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pbi_video_scrubber
  ${catkin_LIBRARIES}
)

## Declare a C++ executable
add_executable(pbi_annotator_node src/annotator_node.cpp)
set_target_properties(pbi_annotator_node PROPERTIES OUTPUT_NAME annotator PREFIX "")
add_dependencies(pbi_annotator_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pbi_annotator_node
  pbi_annotator_server
  pbi_database
  pbi_demo_visualizer
  pbi_multi_object_tracker
  pbi_video_scrubber
  ${catkin_LIBRARIES}
  ${CUDA_LIBRARIES}
)

add_executable(pbi_process_bag_main src/process_bag_main.cpp)
set_target_properties(pbi_process_bag_main PROPERTIES OUTPUT_NAME process_bag PREFIX "")
add_dependencies(pbi_process_bag_main ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pbi_process_bag_main
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

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_task_perception.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
