// // #include <vector>
// // #include "real_robot_control/JAKAZuRobot.h"
// // #include "ros/ros.h"
// // #include <stdio.h>
// // #include <thread>
// // #include "real_robot_control/force_pub.h"
// // #include "Eigen/Dense"
// // #include "Eigen/Core"
// // #include "Eigen/Geometry"
// // #include "Eigen/StdVector"
// // #include <mutex>
// // #include <chrono>

// // int main(int argc, char** argv){
// //     JAKAZuRobot robot;
// //     Eigen::VectorXd world_force = Eigen::VectorXd::Zero(6);
// //     Eigen::VectorXd local_force = Eigen::VectorXd::Zero(6);

// //     Eigen::Matrix3d link6_rotm;
// //     Eigen::Vector3d link6_pos;

// //     Eigen::Matrix3d eef_rotm;
// //     Eigen::Vector3d eef_pos;

// //     std::thread excution_thread;
// //     std::thread sensor_thread;
    
// //     //jaka 库相关声明
// //     Rpy current_rpy;
// //     RotMatrix current_rotm;

// //     CartesianPose tcp_pos;
// //     RobotStatus status;
// //     CartesianPose cart;
// //     ros::init(argc, argv, "code_control");
// //     ros::NodeHandle nh;
// //     ros::Publisher for_pub = nh.advertise<real_robot_control::force_pub>("test_force", 10);
// //     real_robot_control::force_pub f;
    
// //     // nh = std::make_shared<ros::NodeHandle>();
// //     // for_pub = nh->advertise<real_robot_control::force_pub>("robot_force", 10);

// //         // 初始化机器人
// //     robot.login_in("192.168.3.201");
// //     robot.power_on();
// //     robot.enable_robot();
// //     robot.servo_speed_foresight(15, 0.03);
// //     robot.servo_move_enable(TRUE);
// //     robot.set_torque_sensor_mode(1);
// //     robot.set_compliant_type(1, 0);
// //     std::this_thread::sleep_for(std::chrono::milliseconds(100));
// //     robot.set_compliant_type(0,0);


// //     while (ros::ok())
// //     {
// //         robot.get_robot_status(&status);
// //         for (int i=0; i<6; i++){
// //             local_force[i] = status.torq_sensor_monitor_data.actTorque[i];
// //         }
// //         link6_pos[0] = status.cartesiantran_position[0];
// //         link6_pos[1] = status.cartesiantran_position[1];
// //         link6_pos[2] = status.cartesiantran_position[2];
// //         // 姿态欧拉角赋值
// //         current_rpy.rx = status.cartesiantran_position[3];
// //         current_rpy.ry = status.cartesiantran_position[4];
// //         current_rpy.rz = status.cartesiantran_position[5];
// //         // 欧拉角转旋转矩阵，赋值
// //         robot.rpy_to_rot_matrix(&current_rpy, &current_rotm);
// //         link6_rotm << current_rotm.x.x, current_rotm.y.x, current_rotm.z.x,
// //                     current_rotm.x.y, current_rotm.y.y, current_rotm.z.y,
// //                     current_rotm.x.z, current_rotm.y.z, current_rotm.z.z;
        
// //         // left_world_force[:3] = left_link6_rotm @ left_eef_force[:3]
// //         // left_world_force[3:6] = left_link6_rotm @ left_eef_force[3:6]
// //         world_force.head<3>() = link6_rotm * local_force.head<3>();
// //         world_force.tail<3>() = link6_rotm * local_force.tail<3>();

// //         f.X = world_force[0];
// //         f.Y = world_force[1];
// //         f.Z = world_force[2];
// //         f.MX = world_force[3];
// //         f.MY = world_force[4];
// //         f.MZ = world_force[5];
// //         for_pub.publish(f);
// //         std::cout<<"local z force"<<world_force[2] << std::endl;
// //     }

// //     return 0;
// // }

// #include "real_robot_control/JAKAZuRobot.h"
// #include "ros/ros.h"
// // #include "jaka_admittance.h"
// // #include "jaka_control/force_pub.h"
// #include "real_robot_control/force_pub.h"
// #include "Eigen/Dense"
// #include "Eigen/Core"
// #include "Eigen/Geometry"
// #include "Eigen/StdVector"
// #include <chrono>


// JAKAZuRobot robot;
// RobotStatus ret_status;
// // Parameter param;
// CartesianPose cart;


// int initflag = 1;
// const int dimension = 3;
// double e[dimension];
// double ed[dimension];
// double edd[dimension];
// double e_old[dimension];
// double Force[dimension];
// const double PI = 3.1415926;

// double sample_time = 0.024;

// double x, y, z;
// int gain = 1;


// int main(int argc, char *argv[])
// {
    
//     ros::init(argc, argv, "robot_control");
//     ros::NodeHandle nh;
//     robot.login_in("192.168.3.201");
//     robot.power_on();
//     robot.enable_robot();
//     robot.servo_move_use_carte_NLF(1000,4000,8000,120,180,720);
//     robot.set_tool_id(0);

//     // robot.servo_speed_foresight(100,3);
//     robot.servo_move_enable(TRUE);
//     robot.set_torque_sensor_mode(1);
//     robot.set_compliant_type(1, 0);
//     ros::Duration(0.5).sleep();
//     robot.set_compliant_type(0,0);

//     CartesianPose tcp_pos;
//     ros::Publisher for_pub = nh.advertise<real_robot_control::force_pub>("robot_force",10);
//     // ros::Publisher po_pub = nh.advertise<jaka_control::pose_pub>("robot_pose",10);
//     real_robot_control::force_pub f;
//     // jaka_control::pose_pub p;
    
//     ros::Rate rate(100);
//     robot.get_tcp_position(&tcp_pos);
//     robot.get_robot_status(&ret_status);
//     // ROS_INFO("the tcp pose is %.5f, %.5f, %.5f,%.5f, %.5f, %.5f", tcp_pos.tran.x,tcp_pos.tran.y,tcp_pos.tran.z,tcp_pos.rpy.rx,tcp_pos.rpy.ry,tcp_pos.rpy.rz);
//     // ROS_INFO("the tcp pose is %.5f, %.5f, %.5f,%.5f, %.5f, %.5f", ret_status.cartesiantran_position[0], ret_status.cartesiantran_position[1], ret_status.cartesiantran_position[2], ret_status.cartesiantran_position[3] * 180 / 3.14159, ret_status.cartesiantran_position[4]* 180 / 3.14159, ret_status.cartesiantran_position[5]* 180 / 3.14159);

//     auto start_time = std::chrono::high_resolution_clock::now();
//     auto end_time = std::chrono::high_resolution_clock::now();
//     auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time-start_time);

//     // Eigen::Matrix4d matrix;
//     // matrix << 1,0,0,-185.72,
//     //                         0,-1, 5.35898e-08,495.76,
//     //                         0, -5.35898e-08,-1,397.732,
//     //                         0,0,0,1;
//     // Eigen::Matrix4d matrix_sub;
//     // matrix_sub <<           1     ,      0   ,        0 -0.00246684,
//     //         0       ,    1     ,      0 , -0.0142371,
//     //         0      ,     0       ,    1 ,-0.00875405,
//     //         0         ,  0       ,    0      ,     1;
//     // Eigen::Matrix4d transform_matrix = matrix * matrix_sub;
//     // ROS_INFO_STREAM("Matrix:\n" << transform_matrix);

//     while(ros::ok()){
//         start_time = std::chrono::high_resolution_clock::now();
//         robot.get_robot_status(&ret_status);
//         // robot.get_tcp_position(&tcp_pos);

//         for (int i = 0; i < 3; i++)
//         {
//         Force[i] = ret_status.torq_sensor_monitor_data.actTorque[i];
//         }
//         cart.tran.x = 0; cart.tran.y = 0; cart.tran.z = 0;
//         cart.rpy.rx = 0; cart.rpy.ry = 0; cart.rpy.rz = 0;

//         // robot.servo_p(&cart, INCR); //step_num*8ms 运动周期
//                 //发布消息
//         f.X=Force[0];
//         f.Y=Force[1];
//         f.Z=Force[2];
//         f.MX = Force[3];
//         f.MY = Force[4];
//         f.MZ = Force[5];
//         // p.X=tcp_pos.tran.x; 
//         // p.Y=tcp_pos.tran.y;
//         // p.Z=tcp_pos.tran.z;
//         std::cout << "the current force is" << f.X << "N " << f.Y << "N " << f.Z << "N " <<std::endl;
//         // ROS_INFO("the present position in robot base is %.5f, %.5f, %.5f", p.X, p.Y,p.Z);
//         // ROS_INFO("the current force  is %.5f, %.5f, %.5f", f.X, f.Y, f.Z);
//         for_pub.publish(f);
//         // po_pub.publish(p);
        
//         rate.sleep();
//         end_time = std::chrono::high_resolution_clock::now();
//         //计算循环时间
//         duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
//         std::cout << "calculation time is" << duration.count() << "ms" << std::endl;
//         // ROS_INFO("calculation time is %d ms", duration.count());
//         }
//         ROS_INFO("--------------------------------------------------------------------------------");
 



        
//         // ros::Duration loop_duration = end_time - start_time;
        
//         // std::cout << "Duration: " << duration.count() << " ms" << std::endl;
        
        


//     ros::Duration(1).sleep();
//     ROS_INFO("over");
//     // robot.servo_move_enable(false);
//     // robot.disable_robot();
//     return 0;
// }


#include "real_robot_control/right_robot_control.h"
#include "actionlib/client/simple_action_client.h"
#include "real_robot_control/screwAction.h"
#include "real_robot_control/screwsrv.h"

Eigen::VectorXd selection_vector;
typedef actionlib::SimpleActionClient<real_robot_control::screwAction> Client;

void done_cb(const actionlib::SimpleClientGoalState &state, const real_robot_control::screwResultConstPtr &result){
    if (state.state_ == state.SUCCEEDED)
    {
        ROS_INFO("final:%d",result->result);
    } else {
        ROS_INFO("failed！");
    }

}
//服务已经激活
void active_cb(){
    ROS_INFO("activated....");
}
//处理连续反馈
void  feedback_cb(const real_robot_control::screwFeedbackConstPtr &feedback){
    // ROS_INFO("洗涤进度为:%d%s", feedback->progress_bar, "%");
}
// #include "ros/ros.h"

// endeffector ef;


int main(int argc, char *argv[]){
    ros::init(argc, argv, "robot_control");
    ros::NodeHandle nh;
    Eigen::VectorXd selection_vector;
    Eigen::Vector3d _vector;
    selection_vector.resize(6);
    selection_vector<<2, 0, 1, 0, 0, 0;
    _vector << 2 , 3, 8;

        // 4.创建action客户端对象;
    // SimpleActionClient(ros::NodeHandle & n, const std::string & name, bool spin_thread = true)
    // actionlib::SimpleActionClient<demo01_action::AddIntsAction> client(nh,"addInts");
    // Client client(nh,"screwactions",true);
    // //等待服务启动
    // client.waitForServer();

    // real_robot_control::screwGoal goal;
    // goal.num = 0;

    // client.sendGoal(goal,&done_cb,&active_cb,&feedback_cb);
    // // 6.spin().
    // ros::spin();
    // ROS_INFO("finished");

    ros::ServiceClient client = nh.serviceClient<real_robot_control::screwsrv>("screwservice");
    //等待服务启动成功
    //方式1
    ros::service::waitForService("screwservice");
    //方式2
    // client.waitForExistence();
    // 5.组织请求数据
    real_robot_control::screwsrv scr;
    scr.request.num = 0;
    // 6.发送请求,返回 bool 值，标记是否成功
    bool flag = client.call(scr);
    // 7.处理响应
    if (flag)
    {
        ROS_INFO("请求正常处理,响应结果:%d",scr.response.result);
    }
    else
    {
        ROS_ERROR("请求处理失败....");
        return 1;
    }



    // std::cout << selection_vector.head<3>().cwiseProduct(_vector) << std::endl;
    // ros::init(argc, argv, "robot_control");
    // ros::NodeHandle nh;

    // // ef.screwing_s1(100, 1);
    // // ef.width_reduce_or_increase_full(1);
    // // ef.width_recovery();
    // ef.width_reduce(3);
    // sleep(3);
    // ef.width_increase(3);
    return 0;
}

