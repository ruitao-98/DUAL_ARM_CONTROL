#include "real_robot_control/JAKAZuRobot.h"
#include "ros/ros.h"
// #include "jaka_admittance.h"
#include "real_robot_control/force_pub.h"
#include "Eigen/Dense"
#include "Eigen/Core"
#include "Eigen/Geometry"
#include "Eigen/StdVector"
#include <chrono>
#include "real_robot_control/right_robot_control.h"


JAKAZuRobot robot;
RobotStatus ret_status;
// Parameter param;
CartesianPose cart;


int initflag = 1;
const int dimension = 3;
double e[dimension];
double ed[dimension];
double edd[dimension];
double e_old[dimension];
double Force[dimension];
const double PI = 3.1415926;

double sample_time = 0.024;

double x, y, z;
int gain = 1;


int main(int argc, char *argv[])
{
    
    ros::init(argc, argv, "real_robot_control");
    ros::NodeHandle nh;
    robot.login_in("192.168.3.201");
    robot.power_on();
    robot.enable_robot();
    // robot.servo_move_use_carte_NLF(1000,4000,8000,120,180,720);
    // robot.set_tool_id(0);

    // robot.servo_speed_foresight(100,3);
    // robot.servo_move_enable(TRUE);
    // robot.set_torque_sensor_mode(1);
    // robot.set_compliant_type(1, 0);
    // ros::Duration(0.5).sleep();
    // robot.set_compliant_type(0,0);

    CartesianPose tcp_pos;
    ros::Publisher for_pub = nh.advertise<real_robot_control::force_pub>("robot_force",10);

    real_robot_control::force_pub f;

    
    ros::Rate rate(100);
    robot.get_tcp_position(&tcp_pos);
    robot.get_robot_status(&ret_status);
    ROS_INFO("the tcp pose is %.5f, %.5f, %.5f,%.5f, %.5f, %.5f", tcp_pos.tran.x,tcp_pos.tran.y,tcp_pos.tran.z,tcp_pos.rpy.rx,tcp_pos.rpy.ry,tcp_pos.rpy.rz);
    ROS_INFO("the tcp pose is %.5f, %.5f, %.5f,%.5f, %.5f, %.5f", ret_status.cartesiantran_position[0], ret_status.cartesiantran_position[1], ret_status.cartesiantran_position[2], ret_status.cartesiantran_position[3] * 180 / 3.14159, ret_status.cartesiantran_position[4]* 180 / 3.14159, ret_status.cartesiantran_position[5]* 180 / 3.14159);

    // auto start_time = std::chrono::high_resolution_clock::now();
    // auto end_time = std::chrono::high_resolution_clock::now();
    // auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time-start_time);

    // // Eigen::Matrix4d matrix;
    // // matrix << 1,0,0,-185.72,
    // //                         0,-1, 5.35898e-08,495.76,
    // //                         0, -5.35898e-08,-1,397.732,
    // //                         0,0,0,1;
    // // Eigen::Matrix4d matrix_sub;
    // // matrix_sub <<           1     ,      0   ,        0 -0.00246684,
    // //         0       ,    1     ,      0 , -0.0142371,
    // //         0      ,     0       ,    1 ,-0.00875405,
    // //         0         ,  0       ,    0      ,     1;
    // // Eigen::Matrix4d transform_matrix = matrix * matrix_sub;
    // // ROS_INFO_STREAM("Matrix:\n" << transform_matrix);

    // while(ros::ok()){
    //     start_time = std::chrono::high_resolution_clock::now();
    //     robot.get_robot_status(&ret_status);
    //     robot.get_tcp_position(&tcp_pos);

    //     for (int i = 0; i < 3; i++)
    //     {
    //     Force[i] = ret_status.torq_sensor_monitor_data.actTorque[i];
    //     }
    //     cart.tran.x = 0; cart.tran.y = 0; cart.tran.z = 0;
    //     cart.rpy.rx = 0; cart.rpy.ry = 0; cart.rpy.rz = 0;

    //     robot.servo_p(&cart, INCR); //step_num*8ms 运动周期
    //             //发布消息
    //     f.X=Force[0];
    //     f.Y=Force[1];
    //     f.Z=Force[2];
    
    //     ROS_INFO("the current force  is %.5f, %.5f, %.5f", f.X, f.Y, f.Z);
    //     for_pub.publish(f);

        
    //     rate.sleep();
    //     end_time = std::chrono::high_resolution_clock::now();
    //     //计算循环时间
    //     duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    //     ROS_INFO("calculation time is %d ms", duration.count());
    //     }
    //     ROS_INFO("--------------------------------------------------------------------------------");
 



        
    //     // ros::Duration loop_duration = end_time - start_time;
        
    //     // std::cout << "Duration: " << duration.count() << " ms" << std::endl;
        
        


    // ros::Duration(1).sleep();
    // ROS_INFO("over");
    // robot.servo_move_enable(false);
    // robot.disable_robot();
    return 0;
}