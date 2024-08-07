#include <iostream>
#include <string>  // 引入string头文件
#include "real_robot_control/JAKAZuRobot.h"
#include "real_robot_control/jkerr.h"
#include "real_robot_control/jktypes.h"
#include <unistd.h>  // 包含unistd.h头文件以使用sleep函数

// # include <real_robot_control/left_robot_control.h>
// ros::Publisher gripper_pub;
// real_robot_control::gripper gri;

int main(int argc, char** argv){

    // RobotAdmittanceControl robot_control;
    // robot_control.ros_init(argc, argv);
    // int choice;
    
    // std::cout << "select a put place:" << std::endl;
    // std::cin >> choice;
    // robot_control.move_to_target(choice);

    // gripper_pub = robot_control.nh->advertise<real_robot_control::gripper>("gripper_siginal", 10);
    // gri.open = 0.0;
    // int item = 0;
    // while (item < 2)
    // {
    //     item = item + 1;
    //     gripper_pub.publish(gri);
    //     std::this_thread::sleep_for(std::chrono::milliseconds(50));
    // }
    // std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
    // robot_control.pick_up();

    // robot_control.reset();
    // robot_control.go_to_pose();
    // robot_control.start();
    JAKAZuRobot left_robot;
    JAKAZuRobot right_robot;
    left_robot.login_in("192.168.3.200");
    right_robot.login_in("192.168.3.201");
    // robot.set_status_data_update_time_interval(100);
    using namespace std;
    cout << "turn off the robots" << endl;
    
    left_robot.disable_robot();
    right_robot.disable_robot();
    sleep(2);
    right_robot.power_off();
    left_robot.power_off();




    return 0;
}
