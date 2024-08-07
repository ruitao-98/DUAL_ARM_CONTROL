#include <real_robot_control/left_robot_control.h>
#include <iostream>


// 更换旋拧口末端的tips的程序

ros::Publisher gripper_pub;
real_robot_control::gripper gri;

int main(int argc, char** argv) {
    char input;
    bool running = true;
    RobotAdmittanceControl robot_control;
    robot_control.ros_init(argc, argv);
    gripper_pub = robot_control.nh->advertise<real_robot_control::gripper>("gripper_siginal", 10);
    int item;
    while (running) {
        std::cout << "Enter 1 for insertion for the left tip, 2 for insertion for the right tip, or any other key to exit:" << std::endl;
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
                // 执行拔出左和右tip的程序
                std::cout << "plug for the right tip" << std::endl;
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
                // 执行拔出左和右tip的程序
                std::cout << "plug for the right tip" << std::endl;
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
            
            case '5':
                // 执行抓取新tip
                std::cout << "move out from the base" << std::endl;
                int choice;
                
                std::cout << "select a put place:" << std::endl;
                std::cin >> choice;

                robot_control.move_to_target(choice);

                gripper_pub = robot_control.nh->advertise<real_robot_control::gripper>("gripper_siginal", 10);
                gri.open = 0.0;
                item = 0;
                while (item < 5)
                {
                    item = item + 1;
                    gripper_pub.publish(gri);
                    std::this_thread::sleep_for(std::chrono::milliseconds(50));
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                
                robot_control.pick_up();
                break;
            
            case '6':
                // 执行回收
                std::cout << "move to recycle" << std::endl;

                // robot_control.move_to_recycle();

                gripper_pub = robot_control.nh->advertise<real_robot_control::gripper>("gripper_siginal", 10);
                gri.open = 1.0;
                item = 0;
                while (item < 5)
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
}