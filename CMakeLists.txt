cmake_minimum_required(VERSION 2.8.3)
project(tmc_gazebo_task_evaluators)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

find_package(gazebo REQUIRED)
if(gazebo_VERSION_MAJOR GREATER 8)
  #ign_import_target(UUID)
endif()
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

if(gazebo_VERSION_MAJOR GREATER 8)
  find_package(Qt5Widgets REQUIRED)
  find_package(Qt5Core REQUIRED)
  include_directories(${Qt5Core_INCLUDE_DIRS})
else()
  find_package(Qt4)
  include(${QT_USE_FILE})
  add_definitions(${QT_DEFINITIONS})
endif()

FIND_PATH(POCO_INCLUDE_DIR Poco/Poco.h PATHS /usr/include)
FIND_LIBRARY(POCO_FOUNDATION PocoFoundation PATHS /usr/lib)
include_directories(${POCO_INCLUDE_DIR})

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

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
#  INCLUDE_DIRS include
  LIBRARIES score_widget
  CATKIN_DEPENDS roscpp
  #  DEPENDS system_lib
  DEPENDS Eigen3
)

###########
## Build ##
###########

add_subdirectory(env-hooks)

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)

include_directories(include ${catkin_INCLUDE_DIRS})

## Declare a C++ library
# add_library(tmc_gazebo_task_evaluators
#   src/${PROJECT_NAME}/tmc_gazebo_task_evaluators.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(tmc_gazebo_task_evaluators ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
add_executable(undesired_contact_detector src/undesired_contact_detector.cpp)
add_executable(reached_region_calculator src/reached_region_calculator.cpp)
add_executable(object_in_box_detector src/object_in_box_detector.cpp)
add_executable(hhcc_score_counter src/hhcc_score_counter.cpp)
add_executable(hhcc_move_task_score_counter src/hhcc_move_task_score_counter.cpp)
add_executable(hhcc_tidy_camera_controller src/hhcc_tidy_camera_controller.cpp)
add_executable(hhcc_move_camera_controller src/hhcc_move_camera_controller.cpp)
add_executable(wrs_camera_controller src/wrs_camera_controller.cpp)
add_executable(wrs_score_counter src/wrs_score_counter.cpp)

if(gazebo_VERSION_MAJOR GREATER 8)
  QT5_WRAP_CPP(headers_MOC src/score_widget.hh)
else()
  QT4_WRAP_CPP(headers_MOC src/score_widget.hh)
endif()

add_library(score_widget SHARED ${headers_MOC}
  src/score_widget.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(tmc_gazebo_task_evaluators_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(undesired_contact_detector
    ${GAZEBO_LIBRARIES}
    ${POCO_FOUNDATION}
    ${catkin_LIBRARIES}
    pthread
)
target_link_libraries(reached_region_calculator
    ${GAZEBO_LIBRARIES}
    ${catkin_LIBRARIES}
    pthread
)
target_link_libraries(object_in_box_detector
    ${GAZEBO_LIBRARIES}
    ${POCO_FOUNDATION}
    ${catkin_LIBRARIES}
    pthread
)
target_link_libraries(hhcc_score_counter
    ${catkin_LIBRARIES}
    pthread
)
target_link_libraries(hhcc_move_task_score_counter
    ${catkin_LIBRARIES}
    pthread
)
target_link_libraries(hhcc_tidy_camera_controller
    ${GAZEBO_LIBRARIES}
    ${catkin_LIBRARIES}
    pthread
)
target_link_libraries(hhcc_move_camera_controller
    ${GAZEBO_LIBRARIES}
    ${catkin_LIBRARIES}
    pthread
)
target_link_libraries(wrs_camera_controller
    ${GAZEBO_LIBRARIES}
    ${catkin_LIBRARIES}
    pthread
)
target_link_libraries(wrs_score_counter
    ${POCO_FOUNDATION}
    ${catkin_LIBRARIES}
    pthread
)
target_link_libraries(score_widget
    ${GAZEBO_LIBRARIES}
    ${catkin_LIBRARIES}
    ${OGRE_LIBRARIES}
    ${OGRE-Terrain_LIBRARIES}
    ${QT_LIBRARIES}
    ${PROTOBUF_LIBRARIES}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
install(PROGRAMS
  scripts/setup_score_widget
  scripts/wait_for
  scripts/record_gazebo_stat
  scripts/count_objects_test
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark executables and/or libraries for installation
install(TARGETS
  undesired_contact_detector
  reached_region_calculator
  object_in_box_detector
  hhcc_score_counter
  hhcc_move_task_score_counter
  wrs_score_counter
  hhcc_tidy_camera_controller
  hhcc_move_camera_controller
  wrs_camera_controller
  score_widget
  # ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY
  launch
  tests
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_tmc_gazebo_task_evaluators.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(tests/no-block-in-the-box.test)
  add_rostest(tests/one-block-in-the-box.test)
  add_rostest(tests/two-block-in-the-box.test)
endif()
