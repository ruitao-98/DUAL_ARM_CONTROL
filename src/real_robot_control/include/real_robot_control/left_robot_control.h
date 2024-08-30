#include <vector>
#include "real_robot_control/JAKAZuRobot.h"
#include "ros/ros.h"
#include <stdio.h>
#include <thread>
#include "real_robot_control/force_pub.h"
#include "real_robot_control/gripper.h"
#include "Eigen/Dense"
#include "Eigen/Core"
#include "Eigen/Geometry"
#include "Eigen/StdVector"
#include <mutex>
#include "real_robot_control/jktypes.h"
#include "actionlib/client/simple_action_client.h"
#include "real_robot_control/screwAction.h"

namespace jaka {
    using Quaternion = ::Quaternion; // 创建别名
}

class RobotAdmittanceControl
{
public:
    RobotAdmittanceControl();
    ~RobotAdmittanceControl();

    
    void step();
    void reset();
    void update_robot_state();
    void go_to_pose();
    void admittance_control();
    void tcp_admittance_control();
    void get_eef_pose();
    
    void updata_rotation(const Eigen::Matrix3d& current_rotm, const Eigen::Vector3d& angluar_disp, Eigen::Matrix3d& new_orientation);
    void get_new_link6_pose(const Eigen::Vector3d& new_linear_eef, const Eigen::Matrix3d& new_angular_eef);

    void get_robot_pose();
    void get_world_force();
    void get_tcp_force();

    void calculation_loop();
    void excution_loop();
    void excution_calculation_loop();
    void start();  //测试
    void pick_up();  //使用导纳控制抓取tip并抬上来
    void move_to_target(int choice);  //移动到抓取位姿
    void move_to_recycle(int choice, int int_value);  //移动到回收位姿，取下旋拧头后放置到盒子里面
    void move_to_left_pick(); //移动到工具左边的更换位置，取走 /
    void move_to_right_pick(); //移动到工具右边的更换位置，取走
    void move_to_left_insert(); //移动到工具左边的更换位置，插入
    void move_to_right_insert(); //移动到工具右边的更换位置，插入
    void move_to_left_middle(); //执行更换(左）操作的中间点
    void move_to_right_middle(); //执行更换（右）操作的中间点
    void spiral_search();  //螺旋搜索
    void plug_out();  //螺旋搜索
    // void hybird_control();  //交接工件
    // void move_to_handeover();  //交接工件
    void pure_passive_model(); //交接工件，机器人开启柔顺，并发布action让旋拧执行器运动，根据反馈结果进一步调整
    void done_cb(const actionlib::SimpleClientGoalState &state, const real_robot_control::screwResultConstPtr &result);
    void active_cb();
    void feedback_cb(const real_robot_control::screwFeedbackConstPtr &feedback);
    void back_to_middle();

    Eigen::VectorXd adm_m;
    Eigen::VectorXd adm_k;
    Eigen::VectorXd adm_d;

    // ROS相关成员变量
    std::shared_ptr<ros::NodeHandle> nh;
    ros::Publisher for_pub;
    real_robot_control::force_pub f;
    JAKAZuRobot robot;
    actionlib::SimpleActionClient<real_robot_control::screwAction> client;
    real_robot_control::screwGoal goal;

private:
    // 参数
    // Eigen::VectorXd adm_m;
    // Eigen::VectorXd adm_k;
    // Eigen::VectorXd adm_d;

    Eigen::VectorXd world_force = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd local_force = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd tcp_force = Eigen::VectorXd::Zero(6);

    Eigen::Matrix3d link6_rotm;
    Eigen::Vector3d link6_pos;

    Eigen::Matrix3d eef_rotm;
    Eigen::Vector3d eef_pos;



    // std::thread excution_thread;
    // std::thread sensor_thread;
    
    //jaka 库相关声明
    Rpy current_rpy;
    RotMatrix current_rotm;

    RobotStatus status;
    CartesianPose cart;
    CartesianPose new_pos;
    RotMatrix new_rotm;
    Rpy new_rpy;


    
    const double PI = 3.1415926;

    // std::mutex robot_mutex;
    int loop_rate = 1; //loop_rate * 8ms = real_rate
    double T = loop_rate * 0.008;

    Eigen::Vector3d eef_offset;
    Eigen::Matrix3d eef_offset_rotm;
    Eigen::Vector3d eef_offset_to_sensor;
    Eigen::Matrix3d eef_offset_rotm_to_sensor;
    jaka::Quaternion current_eef_quat;
    RotMatrix current_eef_rotm;

     // 定义力上下限
    double lower = -30.0;
    double upper = 30.0;


    //导纳控制相关变量
    Eigen::VectorXd selection_vector;
    Eigen::VectorXd clipped_world_force;
    Eigen::VectorXd clipped_tcp_force;
    Eigen::VectorXd wish_force;


    Eigen::Vector3d new_linear;
    Eigen::Matrix3d new_angular;

    Eigen::Vector3d eef_pos_d;
    Eigen::Matrix3d eef_rotm_d;
    Eigen::VectorXd eef_vel = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd e = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd e_dot = Eigen::VectorXd::Zero(6);
    Eigen::Vector3d linear_disp;
    Eigen::Vector3d angular_disp;

    Eigen::Vector3d linear_disp_clipped;
    Eigen::Vector3d angluer_disp_clipped;
    Eigen::Vector3d new_linear_eef;
    Eigen::Matrix3d new_rotm_eef;

    int screw_execute_result; //末端执行器运行结果 
    // 0：未卡
    // 1：卡住了
    // 2: 还没开始

    
    int screw_execute_status; //末端执行器运行结果 
    // 0：运行后
    // 1：正在运行
    // 2: 运行前


};