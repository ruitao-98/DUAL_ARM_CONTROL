# Install script for directory: /home/yanji/dual_arm_control/src/real_robot_control

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/yanji/dual_arm_control/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/real_robot_control/srv" TYPE FILE FILES
    "/home/yanji/dual_arm_control/src/real_robot_control/srv/screwsrv.srv"
    "/home/yanji/dual_arm_control/src/real_robot_control/srv/leftrobotsrv.srv"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/real_robot_control/action" TYPE FILE FILES "/home/yanji/dual_arm_control/src/real_robot_control/action/screw.action")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/real_robot_control/msg" TYPE FILE FILES
    "/home/yanji/dual_arm_control/devel/share/real_robot_control/msg/screwAction.msg"
    "/home/yanji/dual_arm_control/devel/share/real_robot_control/msg/screwActionGoal.msg"
    "/home/yanji/dual_arm_control/devel/share/real_robot_control/msg/screwActionResult.msg"
    "/home/yanji/dual_arm_control/devel/share/real_robot_control/msg/screwActionFeedback.msg"
    "/home/yanji/dual_arm_control/devel/share/real_robot_control/msg/screwGoal.msg"
    "/home/yanji/dual_arm_control/devel/share/real_robot_control/msg/screwResult.msg"
    "/home/yanji/dual_arm_control/devel/share/real_robot_control/msg/screwFeedback.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/real_robot_control/msg" TYPE FILE FILES
    "/home/yanji/dual_arm_control/src/real_robot_control/msg/force_pub.msg"
    "/home/yanji/dual_arm_control/src/real_robot_control/msg/gripper.msg"
    "/home/yanji/dual_arm_control/src/real_robot_control/msg/current_pub.msg"
    "/home/yanji/dual_arm_control/src/real_robot_control/msg/pose.msg"
    "/home/yanji/dual_arm_control/src/real_robot_control/msg/robot_pos_pub.msg"
    "/home/yanji/dual_arm_control/src/real_robot_control/msg/orientation_pub.msg"
    "/home/yanji/dual_arm_control/src/real_robot_control/msg/force_pos_pub.msg"
    "/home/yanji/dual_arm_control/src/real_robot_control/msg/width_pub.msg"
    "/home/yanji/dual_arm_control/src/real_robot_control/msg/pose_pub.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/real_robot_control/cmake" TYPE FILE FILES "/home/yanji/dual_arm_control/build/real_robot_control/catkin_generated/installspace/real_robot_control-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/yanji/dual_arm_control/devel/include/real_robot_control")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/yanji/dual_arm_control/devel/share/roseus/ros/real_robot_control")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/yanji/dual_arm_control/devel/share/common-lisp/ros/real_robot_control")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/yanji/dual_arm_control/devel/share/gennodejs/ros/real_robot_control")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python3" -m compileall "/home/yanji/dual_arm_control/devel/lib/python3/dist-packages/real_robot_control")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages" TYPE DIRECTORY FILES "/home/yanji/dual_arm_control/devel/lib/python3/dist-packages/real_robot_control")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/yanji/dual_arm_control/build/real_robot_control/catkin_generated/installspace/real_robot_control.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/real_robot_control/cmake" TYPE FILE FILES "/home/yanji/dual_arm_control/build/real_robot_control/catkin_generated/installspace/real_robot_control-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/real_robot_control/cmake" TYPE FILE FILES
    "/home/yanji/dual_arm_control/build/real_robot_control/catkin_generated/installspace/real_robot_controlConfig.cmake"
    "/home/yanji/dual_arm_control/build/real_robot_control/catkin_generated/installspace/real_robot_controlConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/real_robot_control" TYPE FILE FILES "/home/yanji/dual_arm_control/src/real_robot_control/package.xml")
endif()

