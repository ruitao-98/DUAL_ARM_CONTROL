#include <real_robot_control/left_robot_control.h>
#include "real_robot_control/leftrobotsrv.h"
#include <iostream>


// 更换旋拧口末端的tips的程序

ros::Publisher gripper_pub;
real_robot_control::gripper gri;

// JAKAZuRobot robot;
// RobotAdmittanceControl robot_control;
RobotAdmittanceControl* pRobotControl = nullptr;
bool doReq(real_robot_control::leftrobotsrv::Request& req,
          real_robot_control::leftrobotsrv::Response& resp){

    // 初始化机器人
    pRobotControl->robot.login_in("192.168.3.200"); 
    pRobotControl->robot.set_tool_id(0);
    // robot.servo_speed_foresight(15, 0.03);
    pRobotControl->robot.servo_move_use_carte_NLF(50, 200, 800, 30, 60, 100);
    pRobotControl->robot.set_torque_sensor_mode(1);
    pRobotControl->robot.set_compliant_type(1, 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    pRobotControl->robot.set_compliant_type(0, 0);

    int num = req.num;
    ROS_INFO("received data is: num = %d, ",num);
    int item;
    //逻辑处理
    int result;
    if (num == 1)
    {
        // 执行插入左tip的程序
        std::cout << "insertion for the left tip" << std::endl;
        pRobotControl->move_to_left_insert(); //move to the assembly point
        pRobotControl->spiral_search(); //搜索，插入成功

        // 发布张开夹爪的信息
        gri.open = 1.0;
        item = 0;
        while (item < 3)
        {
            item = item + 1;
            gripper_pub.publish(gri);
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            std::cout<<"publishing"<< std::endl;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    else if (num == 2)
    {
        // 执行插入右tip的程序
        std::cout << "insertion for the right tip" << std::endl;
        pRobotControl->move_to_right_insert(); //move to the assembly point
        pRobotControl->spiral_search();

        // 发布张开夹爪的信息
        gri.open = 1.0;
        item = 0;
        while (item < 3)
        {
            item = item + 1;
            gripper_pub.publish(gri);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            std::cout<<"publishing"<< std::endl;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    else if (num == 3)
    {
        // 执行拔出左tip的程序
        std::cout << "plug the left tip" << std::endl;
        pRobotControl->move_to_left_pick(); //move to the assembly point
        
        // 发布闭合夹爪的信息
        gri.open = 0.0;
        item = 0;
        while (item < 2)
        {
            item = item + 1;
            gripper_pub.publish(gri);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            std::cout<<"publishing"<< std::endl;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        pRobotControl->plug_out();
        pRobotControl->move_to_left_middle();
    }

    else if (num == 4)
    {
        // 执行拔出右tip的程序
        std::cout << "plug the right tip" << std::endl;
        pRobotControl->move_to_right_pick(); //move to the assembly point

        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        // 发布闭合夹爪的信息
        gri.open = 0.0;
        item = 0;
        while (item < 3)
        {
            item = item + 1;
            gripper_pub.publish(gri);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            std::cout<<"publishing"<< std::endl;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        pRobotControl->plug_out();
        pRobotControl->move_to_right_middle();
    }

    else if (num == 0)
    {
        // 执行抓取新tip
        std::cout << "move out from the base" << std::endl;
        int choice;
        ros::param::get("choice_int", choice);
        std::cout << "choice = " << choice << std::endl;

        pRobotControl->move_to_target(choice);

        gri.open = 0.0;
        item = 0;
        while (item < 2)
        {
            item = item + 1;
            gripper_pub.publish(gri);
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        
        pRobotControl->pick_up();
    }

    else if (num == 5)
    {
        // 执行回收
        std::cout << "move to recycle" << std::endl;

        // robot_control.move_to_recycle();
        gri.open = 1.0;
        item = 0;
        while (item < 2)
        {
            item = item + 1;
            gripper_pub.publish(gri);
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    else if (num == 6)
    {
        // 执行handover
        std::cout << "move to recycle" << std::endl;

        // robot_control.move_to_recycle();
        gri.open = 1.0;
        item = 0;
        while (item < 2)
        {
            item = item + 1;
            gripper_pub.publish(gri);
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    //设置最终结果

    ROS_INFO("final results:%d",result);
    pRobotControl->robot.login_out(); //调用完就登出
    //如果没有异常，那么相加并将结果赋值给 resp
    resp.result = result;
    return true;

}

int main(int argc, char** argv) {
    char input;
    bool running = true;
    ros::init(argc, argv, "left_robot_control");
    RobotAdmittanceControl robot_control;
    pRobotControl = &robot_control;

    
    // 初始化机器人
    robot_control.robot.login_in("192.168.3.200"); 
    robot_control.robot.power_on();
    robot_control.robot.enable_robot();
    robot_control.robot.set_tool_id(0);
    // robot.servo_speed_foresight(15, 0.03);
    robot_control.robot.servo_move_use_carte_NLF(50, 200, 800, 30, 60, 100);
    robot_control.robot.servo_move_enable(TRUE);
    robot_control.robot.set_torque_sensor_mode(1);
    robot_control.robot.set_compliant_type(1, 0);
    sleep(1);
    robot_control.robot.set_compliant_type(0,0);

    gripper_pub = robot_control.nh->advertise<real_robot_control::gripper>("gripper_siginal", 10);
    int item;
    while (running) {
        std::cout << "Enter 1 for insertion for the left tip, 2 for insertion for the right tip, press 8 to exit:" << std::endl;
        std::cin >> input;
        
        switch(input) {
            case '1':
                // 执行插入左tip的程序
                std::cout << "insertion for the left tip" << std::endl;
                robot_control.move_to_left_insert(); //move to the assembly point
                robot_control.spiral_search(); //搜索，插入成功

                // 发布张开夹爪的信息
                gri.open = 1.0;
                item = 0;
                while (item < 2)
                {
                    item = item + 1;
                    gripper_pub.publish(gri);
                    std::this_thread::sleep_for(std::chrono::milliseconds(50));
                    std::cout<<"publishing"<< std::endl;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                break;

            case '2':
                // 执行插入右tip的程序
                std::cout << "insertion for the right tip" << std::endl;
                robot_control.move_to_right_insert(); //move to the assembly point
                robot_control.spiral_search();

                // 发布张开夹爪的信息
                gri.open = 1.0;
                item = 0;
                while (item < 10)
                {
                    item = item + 1;
                    gripper_pub.publish(gri);
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                    std::cout<<"publishing"<< std::endl;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                break;
            case '3':
                // 执行拔出左tip的程序
                std::cout << "plug the left tip" << std::endl;
                robot_control.move_to_left_pick(); //move to the assembly point
                
                // 发布闭合夹爪的信息
                gri.open = 0.0;
                item = 0;
                while (item < 10)
                {
                    item = item + 1;
                    gripper_pub.publish(gri);
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                    std::cout<<"publishing"<< std::endl;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(500));

                robot_control.plug_out();
                robot_control.move_to_left_middle();

                break;
            
            case '4':
                // 执行拔出右tip的程序
                std::cout << "plug the right tip" << std::endl;
                robot_control.move_to_right_pick(); //move to the assembly point

                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                // 发布闭合夹爪的信息
                gri.open = 0.0;
                item = 0;
                while (item < 5)
                {
                    item = item + 1;
                    gripper_pub.publish(gri);
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                    std::cout<<"publishing"<< std::endl;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(500));

                robot_control.plug_out();
                robot_control.move_to_right_middle();

                break;
            
            case '0':
                // 执行抓取新tip
                std::cout << "move out from the base" << std::endl;
                int input_case;
                std::cin >> input_case;
                // int choice;
                // ros::param::get("choice_int", choice);
                // std::cout << "choice" << choice << std::endl;

                robot_control.move_to_target(input_case);

                gri.open = 0.0;
                item = 0;
                while (item < 2)
                {
                    item = item + 1;
                    gripper_pub.publish(gri);
                    std::this_thread::sleep_for(std::chrono::milliseconds(50));
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                
                robot_control.pick_up();
                break;
            
            case '5':
                // 执行回收
                std::cout << "move to recycle" << std::endl;

                // robot_control.move_to_recycle();

                gri.open = 1.0;
                item = 0;
                while (item < 2)
                {
                    item = item + 1;
                    gripper_pub.publish(gri);
                    std::this_thread::sleep_for(std::chrono::milliseconds(50));
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                break;

            default:
                // 程序结束
                std::cout << "task stoped" << std::endl;
                running = false; // 设置循环为不运行状态，以退出
                break;

        }
    }
    robot_control.robot.login_out(); //键盘控制

    ros::ServiceServer server = robot_control.nh->advertiseService("leftrobotservice",doReq);
    ROS_INFO("server started....");
    ros::spin();
}

