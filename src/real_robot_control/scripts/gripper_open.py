# from modbus_test import RobotiqGripper
import rospy
from real_robot_control.msg import gripper
from pyRobotiqGripper import RobotiqGripper




def dogripper(p):
    print(p.open)
    if p.open == 0:
        position_in_bit = gripper_obj.getPosition()
        if position_in_bit != 0:
            gripper_obj.close()
        else:
            print("closed")

    if p.open == 1:
        position_in_bit = gripper_obj.getPosition()
        if position_in_bit != 160:
            gripper_obj.goTo(160)
        else:
            print("setted to 160")

    if p.open == 2:
        position_in_bit = gripper_obj.getPosition()
        if position_in_bit != 20:
            gripper_obj.goTo(20)
        else:
            print("setted to 20")



if __name__ == "__main__":
    gripper_obj = RobotiqGripper()
    rospy.init_node("gripper")
    gripper_obj.activate()
    gripper_obj.goTo(160)
    
    # gripper_control = RobotiqGripper("/dev/ttyUSB0")
    # gripper.activate_gripper()
    # gripper_control.activate_gripper()
    # gripper_control.set_gripper_position(160)


    #2.创建订阅者对象
    sub = rospy.Subscriber("gripper_siginal", gripper, dogripper, queue_size=5)
    rospy.spin() #4.循环

