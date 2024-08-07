from modbus_test import RobotiqGripper


gripper = RobotiqGripper("/dev/ttyUSB0")
# gripper.activate_gripper()
gripper.set_gripper_position(160)

# gripper.activate_gripper()
# gripper.close_gripper()
# gripper.open_gripper()