#include "real_robot_control/current_pub.h"
#include "real_robot_control/screwing_tool.h"
#include <actionlib/server/simple_action_server.h>
#include "real_robot_control/screwAction.h"
#include "ros/ros.h"
#include "real_robot_control/screwsrv.h"

endeffector ef;
ros::Publisher current_pub;
real_robot_control::current_pub msg;

bool doReq(real_robot_control::screwsrv::Request& req,
          real_robot_control::screwsrv::Response& resp){

    int num = req.num;
    ROS_INFO("服务器接收到的请求数据为:num1 = %d, ",num);

    //逻辑处理
    int result;
    if (num == 0)
    {
        std::cout << "screwing" << std::endl;
        result = ef.screwing_s1(200, 1, current_pub, msg); // 0表示发生了错误，其他数字表示正常
        // sleep(2);
        // result = 0;
    }

    else if (num == 1)
    {
        result = ef.unscrewing_s1(150,80);
    }

    else if (num == 2)
    {
        result = ef.width_reduce_or_increase_full(1);
    }

    else if (num == 3)
    {
        result = ef.width_recovery();
    }
    //设置最终结果

    ROS_INFO("final results:%d",result);

    //如果没有异常，那么相加并将结果赋值给 resp
    resp.result = result;
    return true;

}

int main(int argc, char *argv[]){
    ros::init(argc, argv, "screw_control");
    ros::NodeHandle nh;

    current_pub = nh.advertise<real_robot_control::current_pub>("current_p", 10);

    ros::ServiceServer server = nh.advertiseService("screwservice",doReq);
    ROS_INFO("服务已经启动....");
    ros::spin();

    // Server server(nh,"screwactions",boost::bind(&cb,_1,&server),false);
    // server.start();
    bool running = true;
    char input;
    while (running) {
        std::cout << "Enter 1 for screwing, 2 for unscrewing, or any other key to exit:" << std::endl;
        std::cin >> input;
        switch(input) {
            case '1':
                // 执行插入左tip的程序
                std::cout << "screwing" << std::endl;
                ef.screwing_s1(200, 1, current_pub, msg);
                break;

            case '2':
                // 执行插入右tip的程序
                std::cout << "insertion for the right tip" << std::endl;
                ef.unscrewing_s1(150,80);
                break;
            case '3':
                // 执行插入右tip的程序
                std::cout << "insertion for the right tip" << std::endl;
                ef.width_reduce_or_increase_full(1);
                break;
            case '4':
                // 执行插入右tip的程序
                std::cout << "insertion for the right tip" << std::endl;
                ef.width_recovery();
                break;

            default:
                // 程序结束
                std::cout << "task stoped" << std::endl;
                running = false; // 设置循环为不运行状态，以退出
                break;
        }
    }
    return 0;
}


// action 的回调函数

// typedef actionlib::SimpleActionServer<real_robot_control::screwAction> Server;
// void cb(const real_robot_control::screwGoalConstPtr &goal, Server* server){
//     //获取目标值
//     int num = goal->num;
//     ROS_INFO("goal is :%d", num);
//     int result;
//     if (num == 0)
//     {
//         std::cout << "screwing" << std::endl;
//         // int result = ef.screwing_s1(200, 1, current_pub, msg);
//         sleep(2);
//         result = 0;
//         // real_robot_control::screwFeedback feedback;
//         // for (int i = 0; i <= 100; i++)
//         // {
//         //     feedback.progress_bar = i;
//         //     server->publishFeedback(feedback);
//         //     ros::Duration(0.5).sleep();
//         // }

//     }

//     else if (num == 1)
//     {
//         result = ef.unscrewing_s1(150,80);
//     }

//     else if (num == 2)
//     {
//         result = ef.width_reduce_or_increase_full(1);
//     }

//     else if (num == 3)
//     {
//         result = ef.width_recovery();
//     }
//     //设置最终结果
//     real_robot_control::screwResult r;
//     r.result = result;
//     server->setSucceeded(r);
//     ROS_INFO("final results:%d",r.result);
// }
