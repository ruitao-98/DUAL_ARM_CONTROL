#include "real_robot_control/current_pub.h"
#include "real_robot_control/width_pub.h"
#include "real_robot_control/screwing_tool.h"
#include <actionlib/server/simple_action_server.h>
#include "real_robot_control/screwAction.h"
#include "ros/ros.h"
#include "real_robot_control/screwsrv.h"


using namespace std;
endeffector ef;
ros::Publisher current_pub;
real_robot_control::current_pub msg;

ros::Publisher width_pub;
real_robot_control::width_pub msg1;


// action 的回调函数

typedef actionlib::SimpleActionServer<real_robot_control::screwAction> Server;
void cb(const real_robot_control::screwGoalConstPtr &goal, Server* server){
    //获取目标值
    int num = goal->num;
    std::cout << "**************************" << std::endl;
    ROS_INFO("goal is :%d", num);
    int result;
    real_robot_control::screwFeedback feedback;

    if (num == 0)
    {
        // 对准成功，执行下一步旋拧装配
        std::cout << "执行下一步旋拧装配" << std::endl;
        feedback.screw_status = 1; //执行器开始运行
        server->publishFeedback(feedback);

        result = ef.screwing_s2(210, current_pub, msg); //2表示全部执行成功，0表示没有卡住，也没有旋拧完成，接着旋拧
        // sleep(10);
        // result = 2;

        feedback.screw_status = 0;  //执行器运行结束
        server->publishFeedback(feedback);
    }

    else if (num == 1)
    {
        // "对准失败，但没有卡住，换一个点搜索"
        std::cout << "执行第一阶段的螺纹搜索" << std::endl;
        // 
        feedback.screw_status = 1; //执行器开始运行
        server->publishFeedback(feedback);

        sleep(1); //等机器人稳定了
        result = ef.screwing_s1(200, 1, current_pub, msg); //0表示没有卡住， 1表示卡住了
        // sleep(10);
        // result = 0;

        feedback.screw_status = 0;  //执行器运行结束
        server->publishFeedback(feedback);
        
    }

    else if (num == 2)
    {
        // "对准失败，卡住了，退出"
        std::cout << "对准失败，卡住了，退出" << std::endl;
        feedback.screw_status = 1; //执行器开始运行
        server->publishFeedback(feedback);

        result = ef.unscrew_to_zero(200);  //return 0;
        // sleep(2);
        // result = 0;

        feedback.screw_status = 0;  //执行器运行结束
        server->publishFeedback(feedback);

    }

    else if (num == 3)
    {
        // 一切任务成功了，执行旋拧口张开
        // std::cout << "执行旋拧口张开" << std::endl;
        // cout << "******* 有 10s 的时间，请打开夹持装置的开关 *****" << endl;
        // for (int timesec = 0;  timesec<10; timesec++){
        //     std::cout << "\r" << "倒计时: " << 10 - timesec << " 秒" << std::flush; 
        //     sleep(1);
        //     if (timesec == 9){
        //         std::cout << "结束" << std::flush; 
        //     }
        // }
        
        feedback.screw_status = 1; //执行器开始运行

        server->publishFeedback(feedback);

        result = ef.width_recovery(); // return 3， 完全张开
        usleep(100); //0.1s

        feedback.screw_status = 0;  //执行器运行结束
        server->publishFeedback(feedback);
    }

    else if (num == 4)
    {
        // 旋拧口复位

        feedback.screw_status = 1; //执行器开始运行

        server->publishFeedback(feedback);

        ef.screw_to_zero();  // 旋拧口复位，才能退出

        result = 4;

        feedback.screw_status = 0;  //执行器运行结束
        server->publishFeedback(feedback);
    }

    else if (num == 5)
    {

        feedback.screw_status = 1; //执行器开始运行

        server->publishFeedback(feedback);

        int goal_width;
        ros::param::get("goal_width", goal_width);
        std::cout << "goal_width = " << goal_width << std::endl;
        
        result = ef.width_reduce_full_for_handover(goal_width, current_pub, msg, width_pub, msg1);  //这个函数会设置present_width的参数，反馈当前夹爪的宽度
        //result 1表示没有达到预期位置，0表示达到了

        feedback.screw_status = 0;  //执行器运行结束
        server->publishFeedback(feedback);
    }

    else if (num == 6)
    {
        // 旋拧口打开

        feedback.screw_status = 1; //执行器开始运行
        server->publishFeedback(feedback);

        ef.width_increase(6, current_pub,msg, width_pub, msg1);  // 旋拧口复位，才能退出

        result = 2; //2，表示又回到了原本的状态了，打开

        feedback.screw_status = 0;  //执行器运行结束
        server->publishFeedback(feedback);
    }
    //设置最终结果
    real_robot_control::screwResult r;
    r.result = result;
    server->setSucceeded(r);
    ROS_INFO("final results:%d",r.result);
}

int main(int argc, char *argv[]){
    ros::init(argc, argv, "screw_control");
    ros::NodeHandle nh;

    current_pub = nh.advertise<real_robot_control::current_pub>("current_p", 10);
    width_pub = nh.advertise<real_robot_control::width_pub>("width_p", 10);

    // ros::ServiceServer server = nh.advertiseService("screwservice",doReq);
    // ROS_INFO("服务已经启动....");
    // ros::spin();
    bool running = true;
    char input;
    int goal_width = -56461;
    while (running) {
        std::cout << "Enter 1 for screwing, 2 for unscrewing, or any other key to exit:" << std::endl;
        std::cin >> input;

        // std::string action_str;
        // nh.getParam("robot_action", action_str) && !action_str.empty()
        // char action = action_str[0];  // 获取字符串的第一个字符作为 action
        // switch(action){

        switch(input) {
            case '1':
                // 执行插入左tip的程序
                std::cout << "screwing" << std::endl;
                
                ef.screwing_s1(200, 1.4, current_pub, msg);
                // ef.unscrew_to_zero(200);
                
                break;

            case '2':
                // 执行插入右tip的程序
                std::cout << "width_recovery" << std::endl;
                // ef.unscrewing_s1(150,80);
                // ef.unscrew_to_zero(100);
                // ef.screw_to_zero();
                ef.width_recovery();
                break;
            case '3':
                // 执行插入右tip的程序
                std::cout << "width_reduce_full_for_handover" << std::endl;
                // ef.width_reduce_or_increase_full(1); 
                
                ef.width_reduce_full_for_handover(goal_width, current_pub, msg, width_pub, msg1);  
                break;
            case '4':
                // 执行插入右tip的程序
                std::cout << "width_increase 3" << std::endl;
                ef.screw_to_zero();
                // ef.width_recovery();
                // ef.width_increase(3,current_pub, msg, width_pub, msg1);  // 旋拧口复位，才能退出
                break;

            default:
                // 程序结束
                std::cout << "task stoped" << std::endl;
                running = false; // 设置循环为不运行状态，以退出
                break;
        }
    }

    // 启动action server，阻塞后续代码
    Server server(nh,"screwactions",boost::bind(&cb,_1,&server),false);
    server.start();
    std::cout << "server started" << std::endl;
    ros::spin();


    return 0;
}



