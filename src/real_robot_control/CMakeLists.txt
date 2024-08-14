cmake_minimum_required(VERSION 3.0.2)
project(real_robot_control)
add_definitions("-Wall -g")
## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  control_msgs
  roscpp
  rospy
  std_msgs
  std_srvs
  tf
  moveit_ros_planning_interface
  moveit_msgs
  message_generation
  sensor_msgs
  trajectory_msgs
  moveit_commander
)

find_package(Eigen3 REQUIRED)




## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


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
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
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
add_service_files(
  FILES
  screwsrv.srv
)

## Generate actions in the 'action' folder
add_action_files(
  FILES
  screw.action
)

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   control_msgs#   std_msgs
# )

add_message_files(
  FILES
  force_pub.msg
  gripper.msg
  current_pub.msg
  pose.msg
  robot_pos_pub.msg
)


generate_messages(
  DEPENDENCIES
  std_msgs
  actionlib_msgs
)
################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
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
#  LIBRARIES real_robot_control
   CATKIN_DEPENDS actionlib actionlib_msgs control_msgs roscpp rospy std_msgs std_srvs tf message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
link_directories(
  ${PROJECT_SOURCE_DIR}/lib
  ${catkin_LIB_DIRS}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  include/real_robot_control
  include/dynamixel
)

include_directories(${EIGEN3_INCLUDE_DIR})


add_library(SERVO_MOTOR
  SHARED
  include/real_robot_control/servo_motor_function.h
  src/servo_motor_function.cpp
)

add_library(SCREWING_TOOL
  SHARED
  include/real_robot_control/screwing_tool.h
  src/screwing_tool.cpp
)

add_library(LEFT_ADM_CONTROL
  SHARED
  include/real_robot_control/left_robot_control.h
  src/left_robot_control.cpp
)

add_library(RIGHT_ADM_CONTROL
  SHARED
  include/real_robot_control/right_robot_control.h
  src/right_robot_control.cpp
)

target_link_libraries(SERVO_MOTOR
  ${catkin_LIBRARIES}
  ${PROJECT_SOURCE_DIR}/lib/libdxl_x64_cpp.so
)
target_link_libraries(SCREWING_TOOL
  ${catkin_LIBRARIES}
  ${PROJECT_SOURCE_DIR}/lib/libdxl_x64_cpp.so
  SERVO_MOTOR
)
target_link_libraries(LEFT_ADM_CONTROL
  ${catkin_LIBRARIES}
  ${PROJECT_SOURCE_DIR}/lib/libdxl_x64_cpp.so
  jakaAPI
)
target_link_libraries(RIGHT_ADM_CONTROL
  ${catkin_LIBRARIES}
  ${PROJECT_SOURCE_DIR}/lib/libdxl_x64_cpp.so
  jakaAPI
)
# link_directories(
#   lib
#   ${catkin_LIB_DIRS}
# )

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/real_robot_control.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
  add_executable(robot_connet_test src/robot_connet_test.cpp)
  add_executable(right_robot_control src/right_robot_control.cpp)
  # add_executable(left_robot_control src/left_robot_control.cpp)
  add_executable(code_test src/code_test.cpp)
  add_executable(motion_planning src/motion_planning.cpp)
  add_executable(task_control_screw src/task_control_screw.cpp)
  add_executable(move_out src/move_out.cpp)
  add_executable(left_or_right_assembly src/left_or_right_assembly.cpp)
  add_executable(left_or_right_disassembly src/left_or_right_disassembly.cpp)
  # add_executable(screwing_tool src/screwing_tool.cpp)
  # add_executable(servo_motor_function src/servo_motor_function.cpp)
## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
  
  # add_dependencies(servo_motor_function ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
## Specify libraries to link a library or executable target against
  target_link_libraries(robot_connet_test
  ${catkin_LIBRARIES}
  jakaAPI
  ${PROJECT_SOURCE_DIR}/lib/libdxl_x64_cpp.so
  )
  target_link_libraries(right_robot_control
  ${catkin_LIBRARIES}
  jakaAPI
  ${PROJECT_SOURCE_DIR}/lib/libdxl_x64_cpp.so
  SCREWING_TOOL
  )

  # target_link_libraries(left_robot_control
  # ${catkin_LIBRARIES}
  # jakaAPI
  # ${PROJECT_SOURCE_DIR}/lib/libdxl_x64_cpp.so
  # )

  target_link_libraries(code_test
  ${catkin_LIBRARIES}
  jakaAPI
  ${PROJECT_SOURCE_DIR}/lib/libdxl_x64_cpp.so
  SCREWING_TOOL
  # SERVO_MOTOR
  )

  target_link_libraries(task_control_screw
  ${catkin_LIBRARIES}
  jakaAPI
  ${PROJECT_SOURCE_DIR}/lib/libdxl_x64_cpp.so
  SCREWING_TOOL
  # SERVO_MOTOR
  )

  target_link_libraries(motion_planning
  ${catkin_LIBRARIES}
  jakaAPI
  # SERVO_MOTOR
  )

  target_link_libraries(move_out
  ${catkin_LIBRARIES}
  jakaAPI
  LEFT_ADM_CONTROL
  )

  target_link_libraries(left_or_right_assembly
  ${catkin_LIBRARIES}
  jakaAPI
  LEFT_ADM_CONTROL
  )

  target_link_libraries(left_or_right_disassembly
  ${catkin_LIBRARIES}
  jakaAPI
  LEFT_ADM_CONTROL
  )



  # target_link_libraries(screwing_tool
  # ${catkin_LIBRARIES}
  # ${PROJECT_SOURCE_DIR}/lib/libdxl_x64_cpp.so
  # SERVO_MOTOR
  # )

  add_dependencies(robot_connet_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  add_dependencies(right_robot_control ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  # add_dependencies(left_robot_control ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  add_dependencies(code_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  add_dependencies(motion_planning ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  add_dependencies(left_or_right_disassembly ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  add_dependencies(move_out ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  add_dependencies(left_or_right_assembly ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  # add_dependencies(screwing_tool ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

  # target_link_libraries(servo_motor_function
  # ${catkin_LIBRARIES}
  # ${PROJECT_SOURCE_DIR}/lib/libdxl_x64_cpp.so
  # )

  # target_link_libraries(code_test
  # ${catkin_LIBRARIES}
  # ${PROJECT_SOURCE_DIR}/lib/libjakaAPI.so
  # )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# set(PYTHON_EXECUTABLE "/home/yanji/anaconda3/envs/mujo/bin/python3") 
 # 这个只能放在这里，放在前面因为ROS的生成消息文件时使用的Python环境和包版本不兼容。特别是em模块的问题，所以等消息生成完成后再指定编译器
# catkin_install_python(PROGRAMS
#   scripts/grasp_detection/grasp_planning.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# catkin_install_python(PROGRAMS
#   scripts/pub_test.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# catkin_install_python(PROGRAMS
#   scripts/sub_test.py
#   # scripts/test_1.py
#   # scripts/test_2.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )



## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_real_robot_control.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
