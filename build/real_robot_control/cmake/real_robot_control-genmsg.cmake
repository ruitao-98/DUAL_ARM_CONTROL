# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "real_robot_control: 11 messages, 0 services")

set(MSG_I_FLAGS "-Ireal_robot_control:/home/yanji/dual_arm_control/devel/share/real_robot_control/msg;-Ireal_robot_control:/home/yanji/dual_arm_control/src/real_robot_control/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg;-Iactionlib_msgs:/opt/ros/noetic/share/actionlib_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(real_robot_control_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/yanji/dual_arm_control/devel/share/real_robot_control/msg/screwAction.msg" NAME_WE)
add_custom_target(_real_robot_control_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "real_robot_control" "/home/yanji/dual_arm_control/devel/share/real_robot_control/msg/screwAction.msg" "real_robot_control/screwActionFeedback:real_robot_control/screwFeedback:real_robot_control/screwResult:actionlib_msgs/GoalID:std_msgs/Header:real_robot_control/screwGoal:real_robot_control/screwActionResult:real_robot_control/screwActionGoal:actionlib_msgs/GoalStatus"
)

get_filename_component(_filename "/home/yanji/dual_arm_control/devel/share/real_robot_control/msg/screwActionGoal.msg" NAME_WE)
add_custom_target(_real_robot_control_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "real_robot_control" "/home/yanji/dual_arm_control/devel/share/real_robot_control/msg/screwActionGoal.msg" "real_robot_control/screwGoal:actionlib_msgs/GoalID:std_msgs/Header"
)

get_filename_component(_filename "/home/yanji/dual_arm_control/devel/share/real_robot_control/msg/screwActionResult.msg" NAME_WE)
add_custom_target(_real_robot_control_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "real_robot_control" "/home/yanji/dual_arm_control/devel/share/real_robot_control/msg/screwActionResult.msg" "actionlib_msgs/GoalStatus:real_robot_control/screwResult:std_msgs/Header:actionlib_msgs/GoalID"
)

get_filename_component(_filename "/home/yanji/dual_arm_control/devel/share/real_robot_control/msg/screwActionFeedback.msg" NAME_WE)
add_custom_target(_real_robot_control_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "real_robot_control" "/home/yanji/dual_arm_control/devel/share/real_robot_control/msg/screwActionFeedback.msg" "actionlib_msgs/GoalStatus:real_robot_control/screwFeedback:std_msgs/Header:actionlib_msgs/GoalID"
)

get_filename_component(_filename "/home/yanji/dual_arm_control/devel/share/real_robot_control/msg/screwGoal.msg" NAME_WE)
add_custom_target(_real_robot_control_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "real_robot_control" "/home/yanji/dual_arm_control/devel/share/real_robot_control/msg/screwGoal.msg" ""
)

get_filename_component(_filename "/home/yanji/dual_arm_control/devel/share/real_robot_control/msg/screwResult.msg" NAME_WE)
add_custom_target(_real_robot_control_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "real_robot_control" "/home/yanji/dual_arm_control/devel/share/real_robot_control/msg/screwResult.msg" ""
)

get_filename_component(_filename "/home/yanji/dual_arm_control/devel/share/real_robot_control/msg/screwFeedback.msg" NAME_WE)
add_custom_target(_real_robot_control_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "real_robot_control" "/home/yanji/dual_arm_control/devel/share/real_robot_control/msg/screwFeedback.msg" ""
)

get_filename_component(_filename "/home/yanji/dual_arm_control/src/real_robot_control/msg/force_pub.msg" NAME_WE)
add_custom_target(_real_robot_control_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "real_robot_control" "/home/yanji/dual_arm_control/src/real_robot_control/msg/force_pub.msg" ""
)

get_filename_component(_filename "/home/yanji/dual_arm_control/src/real_robot_control/msg/gripper.msg" NAME_WE)
add_custom_target(_real_robot_control_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "real_robot_control" "/home/yanji/dual_arm_control/src/real_robot_control/msg/gripper.msg" ""
)

get_filename_component(_filename "/home/yanji/dual_arm_control/src/real_robot_control/msg/current_pub.msg" NAME_WE)
add_custom_target(_real_robot_control_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "real_robot_control" "/home/yanji/dual_arm_control/src/real_robot_control/msg/current_pub.msg" ""
)

get_filename_component(_filename "/home/yanji/dual_arm_control/src/real_robot_control/msg/pose.msg" NAME_WE)
add_custom_target(_real_robot_control_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "real_robot_control" "/home/yanji/dual_arm_control/src/real_robot_control/msg/pose.msg" "std_msgs/MultiArrayLayout:std_msgs/MultiArrayDimension:std_msgs/Float32MultiArray"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(real_robot_control
  "/home/yanji/dual_arm_control/devel/share/real_robot_control/msg/screwAction.msg"
  "${MSG_I_FLAGS}"
  "/home/yanji/dual_arm_control/devel/share/real_robot_control/msg/screwActionFeedback.msg;/home/yanji/dual_arm_control/devel/share/real_robot_control/msg/screwFeedback.msg;/home/yanji/dual_arm_control/devel/share/real_robot_control/msg/screwResult.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/yanji/dual_arm_control/devel/share/real_robot_control/msg/screwGoal.msg;/home/yanji/dual_arm_control/devel/share/real_robot_control/msg/screwActionResult.msg;/home/yanji/dual_arm_control/devel/share/real_robot_control/msg/screwActionGoal.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/real_robot_control
)
_generate_msg_cpp(real_robot_control
  "/home/yanji/dual_arm_control/devel/share/real_robot_control/msg/screwActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/home/yanji/dual_arm_control/devel/share/real_robot_control/msg/screwGoal.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/real_robot_control
)
_generate_msg_cpp(real_robot_control
  "/home/yanji/dual_arm_control/devel/share/real_robot_control/msg/screwActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/yanji/dual_arm_control/devel/share/real_robot_control/msg/screwResult.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/real_robot_control
)
_generate_msg_cpp(real_robot_control
  "/home/yanji/dual_arm_control/devel/share/real_robot_control/msg/screwActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/yanji/dual_arm_control/devel/share/real_robot_control/msg/screwFeedback.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/real_robot_control
)
_generate_msg_cpp(real_robot_control
  "/home/yanji/dual_arm_control/devel/share/real_robot_control/msg/screwGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/real_robot_control
)
_generate_msg_cpp(real_robot_control
  "/home/yanji/dual_arm_control/devel/share/real_robot_control/msg/screwResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/real_robot_control
)
_generate_msg_cpp(real_robot_control
  "/home/yanji/dual_arm_control/devel/share/real_robot_control/msg/screwFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/real_robot_control
)
_generate_msg_cpp(real_robot_control
  "/home/yanji/dual_arm_control/src/real_robot_control/msg/force_pub.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/real_robot_control
)
_generate_msg_cpp(real_robot_control
  "/home/yanji/dual_arm_control/src/real_robot_control/msg/gripper.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/real_robot_control
)
_generate_msg_cpp(real_robot_control
  "/home/yanji/dual_arm_control/src/real_robot_control/msg/current_pub.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/real_robot_control
)
_generate_msg_cpp(real_robot_control
  "/home/yanji/dual_arm_control/src/real_robot_control/msg/pose.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float32MultiArray.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/real_robot_control
)

### Generating Services

### Generating Module File
_generate_module_cpp(real_robot_control
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/real_robot_control
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(real_robot_control_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(real_robot_control_generate_messages real_robot_control_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/yanji/dual_arm_control/devel/share/real_robot_control/msg/screwAction.msg" NAME_WE)
add_dependencies(real_robot_control_generate_messages_cpp _real_robot_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yanji/dual_arm_control/devel/share/real_robot_control/msg/screwActionGoal.msg" NAME_WE)
add_dependencies(real_robot_control_generate_messages_cpp _real_robot_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yanji/dual_arm_control/devel/share/real_robot_control/msg/screwActionResult.msg" NAME_WE)
add_dependencies(real_robot_control_generate_messages_cpp _real_robot_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yanji/dual_arm_control/devel/share/real_robot_control/msg/screwActionFeedback.msg" NAME_WE)
add_dependencies(real_robot_control_generate_messages_cpp _real_robot_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yanji/dual_arm_control/devel/share/real_robot_control/msg/screwGoal.msg" NAME_WE)
add_dependencies(real_robot_control_generate_messages_cpp _real_robot_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yanji/dual_arm_control/devel/share/real_robot_control/msg/screwResult.msg" NAME_WE)
add_dependencies(real_robot_control_generate_messages_cpp _real_robot_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yanji/dual_arm_control/devel/share/real_robot_control/msg/screwFeedback.msg" NAME_WE)
add_dependencies(real_robot_control_generate_messages_cpp _real_robot_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yanji/dual_arm_control/src/real_robot_control/msg/force_pub.msg" NAME_WE)
add_dependencies(real_robot_control_generate_messages_cpp _real_robot_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yanji/dual_arm_control/src/real_robot_control/msg/gripper.msg" NAME_WE)
add_dependencies(real_robot_control_generate_messages_cpp _real_robot_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yanji/dual_arm_control/src/real_robot_control/msg/current_pub.msg" NAME_WE)
add_dependencies(real_robot_control_generate_messages_cpp _real_robot_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yanji/dual_arm_control/src/real_robot_control/msg/pose.msg" NAME_WE)
add_dependencies(real_robot_control_generate_messages_cpp _real_robot_control_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(real_robot_control_gencpp)
add_dependencies(real_robot_control_gencpp real_robot_control_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS real_robot_control_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(real_robot_control
  "/home/yanji/dual_arm_control/devel/share/real_robot_control/msg/screwAction.msg"
  "${MSG_I_FLAGS}"
  "/home/yanji/dual_arm_control/devel/share/real_robot_control/msg/screwActionFeedback.msg;/home/yanji/dual_arm_control/devel/share/real_robot_control/msg/screwFeedback.msg;/home/yanji/dual_arm_control/devel/share/real_robot_control/msg/screwResult.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/yanji/dual_arm_control/devel/share/real_robot_control/msg/screwGoal.msg;/home/yanji/dual_arm_control/devel/share/real_robot_control/msg/screwActionResult.msg;/home/yanji/dual_arm_control/devel/share/real_robot_control/msg/screwActionGoal.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/real_robot_control
)
_generate_msg_eus(real_robot_control
  "/home/yanji/dual_arm_control/devel/share/real_robot_control/msg/screwActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/home/yanji/dual_arm_control/devel/share/real_robot_control/msg/screwGoal.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/real_robot_control
)
_generate_msg_eus(real_robot_control
  "/home/yanji/dual_arm_control/devel/share/real_robot_control/msg/screwActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/yanji/dual_arm_control/devel/share/real_robot_control/msg/screwResult.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/real_robot_control
)
_generate_msg_eus(real_robot_control
  "/home/yanji/dual_arm_control/devel/share/real_robot_control/msg/screwActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/yanji/dual_arm_control/devel/share/real_robot_control/msg/screwFeedback.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/real_robot_control
)
_generate_msg_eus(real_robot_control
  "/home/yanji/dual_arm_control/devel/share/real_robot_control/msg/screwGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/real_robot_control
)
_generate_msg_eus(real_robot_control
  "/home/yanji/dual_arm_control/devel/share/real_robot_control/msg/screwResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/real_robot_control
)
_generate_msg_eus(real_robot_control
  "/home/yanji/dual_arm_control/devel/share/real_robot_control/msg/screwFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/real_robot_control
)
_generate_msg_eus(real_robot_control
  "/home/yanji/dual_arm_control/src/real_robot_control/msg/force_pub.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/real_robot_control
)
_generate_msg_eus(real_robot_control
  "/home/yanji/dual_arm_control/src/real_robot_control/msg/gripper.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/real_robot_control
)
_generate_msg_eus(real_robot_control
  "/home/yanji/dual_arm_control/src/real_robot_control/msg/current_pub.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/real_robot_control
)
_generate_msg_eus(real_robot_control
  "/home/yanji/dual_arm_control/src/real_robot_control/msg/pose.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float32MultiArray.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/real_robot_control
)

### Generating Services

### Generating Module File
_generate_module_eus(real_robot_control
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/real_robot_control
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(real_robot_control_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(real_robot_control_generate_messages real_robot_control_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/yanji/dual_arm_control/devel/share/real_robot_control/msg/screwAction.msg" NAME_WE)
add_dependencies(real_robot_control_generate_messages_eus _real_robot_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yanji/dual_arm_control/devel/share/real_robot_control/msg/screwActionGoal.msg" NAME_WE)
add_dependencies(real_robot_control_generate_messages_eus _real_robot_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yanji/dual_arm_control/devel/share/real_robot_control/msg/screwActionResult.msg" NAME_WE)
add_dependencies(real_robot_control_generate_messages_eus _real_robot_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yanji/dual_arm_control/devel/share/real_robot_control/msg/screwActionFeedback.msg" NAME_WE)
add_dependencies(real_robot_control_generate_messages_eus _real_robot_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yanji/dual_arm_control/devel/share/real_robot_control/msg/screwGoal.msg" NAME_WE)
add_dependencies(real_robot_control_generate_messages_eus _real_robot_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yanji/dual_arm_control/devel/share/real_robot_control/msg/screwResult.msg" NAME_WE)
add_dependencies(real_robot_control_generate_messages_eus _real_robot_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yanji/dual_arm_control/devel/share/real_robot_control/msg/screwFeedback.msg" NAME_WE)
add_dependencies(real_robot_control_generate_messages_eus _real_robot_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yanji/dual_arm_control/src/real_robot_control/msg/force_pub.msg" NAME_WE)
add_dependencies(real_robot_control_generate_messages_eus _real_robot_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yanji/dual_arm_control/src/real_robot_control/msg/gripper.msg" NAME_WE)
add_dependencies(real_robot_control_generate_messages_eus _real_robot_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yanji/dual_arm_control/src/real_robot_control/msg/current_pub.msg" NAME_WE)
add_dependencies(real_robot_control_generate_messages_eus _real_robot_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yanji/dual_arm_control/src/real_robot_control/msg/pose.msg" NAME_WE)
add_dependencies(real_robot_control_generate_messages_eus _real_robot_control_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(real_robot_control_geneus)
add_dependencies(real_robot_control_geneus real_robot_control_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS real_robot_control_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(real_robot_control
  "/home/yanji/dual_arm_control/devel/share/real_robot_control/msg/screwAction.msg"
  "${MSG_I_FLAGS}"
  "/home/yanji/dual_arm_control/devel/share/real_robot_control/msg/screwActionFeedback.msg;/home/yanji/dual_arm_control/devel/share/real_robot_control/msg/screwFeedback.msg;/home/yanji/dual_arm_control/devel/share/real_robot_control/msg/screwResult.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/yanji/dual_arm_control/devel/share/real_robot_control/msg/screwGoal.msg;/home/yanji/dual_arm_control/devel/share/real_robot_control/msg/screwActionResult.msg;/home/yanji/dual_arm_control/devel/share/real_robot_control/msg/screwActionGoal.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/real_robot_control
)
_generate_msg_lisp(real_robot_control
  "/home/yanji/dual_arm_control/devel/share/real_robot_control/msg/screwActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/home/yanji/dual_arm_control/devel/share/real_robot_control/msg/screwGoal.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/real_robot_control
)
_generate_msg_lisp(real_robot_control
  "/home/yanji/dual_arm_control/devel/share/real_robot_control/msg/screwActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/yanji/dual_arm_control/devel/share/real_robot_control/msg/screwResult.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/real_robot_control
)
_generate_msg_lisp(real_robot_control
  "/home/yanji/dual_arm_control/devel/share/real_robot_control/msg/screwActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/yanji/dual_arm_control/devel/share/real_robot_control/msg/screwFeedback.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/real_robot_control
)
_generate_msg_lisp(real_robot_control
  "/home/yanji/dual_arm_control/devel/share/real_robot_control/msg/screwGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/real_robot_control
)
_generate_msg_lisp(real_robot_control
  "/home/yanji/dual_arm_control/devel/share/real_robot_control/msg/screwResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/real_robot_control
)
_generate_msg_lisp(real_robot_control
  "/home/yanji/dual_arm_control/devel/share/real_robot_control/msg/screwFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/real_robot_control
)
_generate_msg_lisp(real_robot_control
  "/home/yanji/dual_arm_control/src/real_robot_control/msg/force_pub.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/real_robot_control
)
_generate_msg_lisp(real_robot_control
  "/home/yanji/dual_arm_control/src/real_robot_control/msg/gripper.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/real_robot_control
)
_generate_msg_lisp(real_robot_control
  "/home/yanji/dual_arm_control/src/real_robot_control/msg/current_pub.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/real_robot_control
)
_generate_msg_lisp(real_robot_control
  "/home/yanji/dual_arm_control/src/real_robot_control/msg/pose.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float32MultiArray.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/real_robot_control
)

### Generating Services

### Generating Module File
_generate_module_lisp(real_robot_control
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/real_robot_control
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(real_robot_control_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(real_robot_control_generate_messages real_robot_control_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/yanji/dual_arm_control/devel/share/real_robot_control/msg/screwAction.msg" NAME_WE)
add_dependencies(real_robot_control_generate_messages_lisp _real_robot_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yanji/dual_arm_control/devel/share/real_robot_control/msg/screwActionGoal.msg" NAME_WE)
add_dependencies(real_robot_control_generate_messages_lisp _real_robot_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yanji/dual_arm_control/devel/share/real_robot_control/msg/screwActionResult.msg" NAME_WE)
add_dependencies(real_robot_control_generate_messages_lisp _real_robot_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yanji/dual_arm_control/devel/share/real_robot_control/msg/screwActionFeedback.msg" NAME_WE)
add_dependencies(real_robot_control_generate_messages_lisp _real_robot_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yanji/dual_arm_control/devel/share/real_robot_control/msg/screwGoal.msg" NAME_WE)
add_dependencies(real_robot_control_generate_messages_lisp _real_robot_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yanji/dual_arm_control/devel/share/real_robot_control/msg/screwResult.msg" NAME_WE)
add_dependencies(real_robot_control_generate_messages_lisp _real_robot_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yanji/dual_arm_control/devel/share/real_robot_control/msg/screwFeedback.msg" NAME_WE)
add_dependencies(real_robot_control_generate_messages_lisp _real_robot_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yanji/dual_arm_control/src/real_robot_control/msg/force_pub.msg" NAME_WE)
add_dependencies(real_robot_control_generate_messages_lisp _real_robot_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yanji/dual_arm_control/src/real_robot_control/msg/gripper.msg" NAME_WE)
add_dependencies(real_robot_control_generate_messages_lisp _real_robot_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yanji/dual_arm_control/src/real_robot_control/msg/current_pub.msg" NAME_WE)
add_dependencies(real_robot_control_generate_messages_lisp _real_robot_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yanji/dual_arm_control/src/real_robot_control/msg/pose.msg" NAME_WE)
add_dependencies(real_robot_control_generate_messages_lisp _real_robot_control_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(real_robot_control_genlisp)
add_dependencies(real_robot_control_genlisp real_robot_control_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS real_robot_control_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(real_robot_control
  "/home/yanji/dual_arm_control/devel/share/real_robot_control/msg/screwAction.msg"
  "${MSG_I_FLAGS}"
  "/home/yanji/dual_arm_control/devel/share/real_robot_control/msg/screwActionFeedback.msg;/home/yanji/dual_arm_control/devel/share/real_robot_control/msg/screwFeedback.msg;/home/yanji/dual_arm_control/devel/share/real_robot_control/msg/screwResult.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/yanji/dual_arm_control/devel/share/real_robot_control/msg/screwGoal.msg;/home/yanji/dual_arm_control/devel/share/real_robot_control/msg/screwActionResult.msg;/home/yanji/dual_arm_control/devel/share/real_robot_control/msg/screwActionGoal.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/real_robot_control
)
_generate_msg_nodejs(real_robot_control
  "/home/yanji/dual_arm_control/devel/share/real_robot_control/msg/screwActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/home/yanji/dual_arm_control/devel/share/real_robot_control/msg/screwGoal.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/real_robot_control
)
_generate_msg_nodejs(real_robot_control
  "/home/yanji/dual_arm_control/devel/share/real_robot_control/msg/screwActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/yanji/dual_arm_control/devel/share/real_robot_control/msg/screwResult.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/real_robot_control
)
_generate_msg_nodejs(real_robot_control
  "/home/yanji/dual_arm_control/devel/share/real_robot_control/msg/screwActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/yanji/dual_arm_control/devel/share/real_robot_control/msg/screwFeedback.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/real_robot_control
)
_generate_msg_nodejs(real_robot_control
  "/home/yanji/dual_arm_control/devel/share/real_robot_control/msg/screwGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/real_robot_control
)
_generate_msg_nodejs(real_robot_control
  "/home/yanji/dual_arm_control/devel/share/real_robot_control/msg/screwResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/real_robot_control
)
_generate_msg_nodejs(real_robot_control
  "/home/yanji/dual_arm_control/devel/share/real_robot_control/msg/screwFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/real_robot_control
)
_generate_msg_nodejs(real_robot_control
  "/home/yanji/dual_arm_control/src/real_robot_control/msg/force_pub.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/real_robot_control
)
_generate_msg_nodejs(real_robot_control
  "/home/yanji/dual_arm_control/src/real_robot_control/msg/gripper.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/real_robot_control
)
_generate_msg_nodejs(real_robot_control
  "/home/yanji/dual_arm_control/src/real_robot_control/msg/current_pub.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/real_robot_control
)
_generate_msg_nodejs(real_robot_control
  "/home/yanji/dual_arm_control/src/real_robot_control/msg/pose.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float32MultiArray.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/real_robot_control
)

### Generating Services

### Generating Module File
_generate_module_nodejs(real_robot_control
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/real_robot_control
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(real_robot_control_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(real_robot_control_generate_messages real_robot_control_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/yanji/dual_arm_control/devel/share/real_robot_control/msg/screwAction.msg" NAME_WE)
add_dependencies(real_robot_control_generate_messages_nodejs _real_robot_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yanji/dual_arm_control/devel/share/real_robot_control/msg/screwActionGoal.msg" NAME_WE)
add_dependencies(real_robot_control_generate_messages_nodejs _real_robot_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yanji/dual_arm_control/devel/share/real_robot_control/msg/screwActionResult.msg" NAME_WE)
add_dependencies(real_robot_control_generate_messages_nodejs _real_robot_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yanji/dual_arm_control/devel/share/real_robot_control/msg/screwActionFeedback.msg" NAME_WE)
add_dependencies(real_robot_control_generate_messages_nodejs _real_robot_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yanji/dual_arm_control/devel/share/real_robot_control/msg/screwGoal.msg" NAME_WE)
add_dependencies(real_robot_control_generate_messages_nodejs _real_robot_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yanji/dual_arm_control/devel/share/real_robot_control/msg/screwResult.msg" NAME_WE)
add_dependencies(real_robot_control_generate_messages_nodejs _real_robot_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yanji/dual_arm_control/devel/share/real_robot_control/msg/screwFeedback.msg" NAME_WE)
add_dependencies(real_robot_control_generate_messages_nodejs _real_robot_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yanji/dual_arm_control/src/real_robot_control/msg/force_pub.msg" NAME_WE)
add_dependencies(real_robot_control_generate_messages_nodejs _real_robot_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yanji/dual_arm_control/src/real_robot_control/msg/gripper.msg" NAME_WE)
add_dependencies(real_robot_control_generate_messages_nodejs _real_robot_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yanji/dual_arm_control/src/real_robot_control/msg/current_pub.msg" NAME_WE)
add_dependencies(real_robot_control_generate_messages_nodejs _real_robot_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yanji/dual_arm_control/src/real_robot_control/msg/pose.msg" NAME_WE)
add_dependencies(real_robot_control_generate_messages_nodejs _real_robot_control_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(real_robot_control_gennodejs)
add_dependencies(real_robot_control_gennodejs real_robot_control_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS real_robot_control_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(real_robot_control
  "/home/yanji/dual_arm_control/devel/share/real_robot_control/msg/screwAction.msg"
  "${MSG_I_FLAGS}"
  "/home/yanji/dual_arm_control/devel/share/real_robot_control/msg/screwActionFeedback.msg;/home/yanji/dual_arm_control/devel/share/real_robot_control/msg/screwFeedback.msg;/home/yanji/dual_arm_control/devel/share/real_robot_control/msg/screwResult.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/yanji/dual_arm_control/devel/share/real_robot_control/msg/screwGoal.msg;/home/yanji/dual_arm_control/devel/share/real_robot_control/msg/screwActionResult.msg;/home/yanji/dual_arm_control/devel/share/real_robot_control/msg/screwActionGoal.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/real_robot_control
)
_generate_msg_py(real_robot_control
  "/home/yanji/dual_arm_control/devel/share/real_robot_control/msg/screwActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/home/yanji/dual_arm_control/devel/share/real_robot_control/msg/screwGoal.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/real_robot_control
)
_generate_msg_py(real_robot_control
  "/home/yanji/dual_arm_control/devel/share/real_robot_control/msg/screwActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/yanji/dual_arm_control/devel/share/real_robot_control/msg/screwResult.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/real_robot_control
)
_generate_msg_py(real_robot_control
  "/home/yanji/dual_arm_control/devel/share/real_robot_control/msg/screwActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/yanji/dual_arm_control/devel/share/real_robot_control/msg/screwFeedback.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/real_robot_control
)
_generate_msg_py(real_robot_control
  "/home/yanji/dual_arm_control/devel/share/real_robot_control/msg/screwGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/real_robot_control
)
_generate_msg_py(real_robot_control
  "/home/yanji/dual_arm_control/devel/share/real_robot_control/msg/screwResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/real_robot_control
)
_generate_msg_py(real_robot_control
  "/home/yanji/dual_arm_control/devel/share/real_robot_control/msg/screwFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/real_robot_control
)
_generate_msg_py(real_robot_control
  "/home/yanji/dual_arm_control/src/real_robot_control/msg/force_pub.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/real_robot_control
)
_generate_msg_py(real_robot_control
  "/home/yanji/dual_arm_control/src/real_robot_control/msg/gripper.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/real_robot_control
)
_generate_msg_py(real_robot_control
  "/home/yanji/dual_arm_control/src/real_robot_control/msg/current_pub.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/real_robot_control
)
_generate_msg_py(real_robot_control
  "/home/yanji/dual_arm_control/src/real_robot_control/msg/pose.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float32MultiArray.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/real_robot_control
)

### Generating Services

### Generating Module File
_generate_module_py(real_robot_control
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/real_robot_control
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(real_robot_control_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(real_robot_control_generate_messages real_robot_control_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/yanji/dual_arm_control/devel/share/real_robot_control/msg/screwAction.msg" NAME_WE)
add_dependencies(real_robot_control_generate_messages_py _real_robot_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yanji/dual_arm_control/devel/share/real_robot_control/msg/screwActionGoal.msg" NAME_WE)
add_dependencies(real_robot_control_generate_messages_py _real_robot_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yanji/dual_arm_control/devel/share/real_robot_control/msg/screwActionResult.msg" NAME_WE)
add_dependencies(real_robot_control_generate_messages_py _real_robot_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yanji/dual_arm_control/devel/share/real_robot_control/msg/screwActionFeedback.msg" NAME_WE)
add_dependencies(real_robot_control_generate_messages_py _real_robot_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yanji/dual_arm_control/devel/share/real_robot_control/msg/screwGoal.msg" NAME_WE)
add_dependencies(real_robot_control_generate_messages_py _real_robot_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yanji/dual_arm_control/devel/share/real_robot_control/msg/screwResult.msg" NAME_WE)
add_dependencies(real_robot_control_generate_messages_py _real_robot_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yanji/dual_arm_control/devel/share/real_robot_control/msg/screwFeedback.msg" NAME_WE)
add_dependencies(real_robot_control_generate_messages_py _real_robot_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yanji/dual_arm_control/src/real_robot_control/msg/force_pub.msg" NAME_WE)
add_dependencies(real_robot_control_generate_messages_py _real_robot_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yanji/dual_arm_control/src/real_robot_control/msg/gripper.msg" NAME_WE)
add_dependencies(real_robot_control_generate_messages_py _real_robot_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yanji/dual_arm_control/src/real_robot_control/msg/current_pub.msg" NAME_WE)
add_dependencies(real_robot_control_generate_messages_py _real_robot_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yanji/dual_arm_control/src/real_robot_control/msg/pose.msg" NAME_WE)
add_dependencies(real_robot_control_generate_messages_py _real_robot_control_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(real_robot_control_genpy)
add_dependencies(real_robot_control_genpy real_robot_control_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS real_robot_control_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/real_robot_control)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/real_robot_control
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(real_robot_control_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET actionlib_msgs_generate_messages_cpp)
  add_dependencies(real_robot_control_generate_messages_cpp actionlib_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/real_robot_control)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/real_robot_control
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(real_robot_control_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET actionlib_msgs_generate_messages_eus)
  add_dependencies(real_robot_control_generate_messages_eus actionlib_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/real_robot_control)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/real_robot_control
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(real_robot_control_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET actionlib_msgs_generate_messages_lisp)
  add_dependencies(real_robot_control_generate_messages_lisp actionlib_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/real_robot_control)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/real_robot_control
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(real_robot_control_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET actionlib_msgs_generate_messages_nodejs)
  add_dependencies(real_robot_control_generate_messages_nodejs actionlib_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/real_robot_control)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/real_robot_control\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/real_robot_control
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(real_robot_control_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET actionlib_msgs_generate_messages_py)
  add_dependencies(real_robot_control_generate_messages_py actionlib_msgs_generate_messages_py)
endif()
