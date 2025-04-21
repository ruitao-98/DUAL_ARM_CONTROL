#include <vector>
#include "real_robot_control/JAKAZuRobot.h"
#include "ros/ros.h"
#include <stdio.h>
#include <thread>
#include "real_robot_control/force_pub.h"
#include "real_robot_control/pose_pub.h"
#include "real_robot_control/robot_pos_pub.h"
#include "Eigen/Dense"
#include "Eigen/Core"
#include "Eigen/Geometry"
#include "Eigen/StdVector"
#include <mutex>
#include "real_robot_control/jktypes.h"
#include "real_robot_control/screwsrv.h"
#include "actionlib/client/simple_action_client.h"
#include "real_robot_control/screwAction.h"
#include "real_robot_control/force_pos_pub.h"
#include "real_robot_control/orientation_pub.h"
#include "real_robot_control/ori_adj_rec.h"

namespace jaka {
    using Quaternion = ::Quaternion; // 创建别名
}

class RobotAdmittanceControl
{
public:
    RobotAdmittanceControl();
    ~RobotAdmittanceControl();

    
    void test();
    void reset();
    void update_robot_state();
    void go_to_pose(char choice_tcp);
    void admittance_control();
    void tcp_admittance_control();
    void get_eef_pose();
    
    void updata_rotation(const Eigen::Matrix3d& current_rotm, const Eigen::Vector3d& angluar_disp, Eigen::Matrix3d& new_orientation);
    void get_new_link6_pose(const Eigen::Vector3d& new_linear_eef, const Eigen::Matrix3d& new_angular_eef);
    // void ros_init(int argc, char** argv);
    
    void get_robot_pose();
    void get_world_force();
    void get_tcp_force();

    void calculation_loop();
    void excution_loop();
    void excution_calculation_loop();
    void start();
    void screw_assembly_search();
    void linear_search(char choice_tcp);
    void pos_ori_search();
    void pos_search();
    void ori_fine();
    bool getParam(const std::string& param_name, std::string& value);
    void robot_finish();
    void grasp_obj();
    void passive_fine();
    void print_eef(char choice);
    void screw_assembly_directly();
    void tcp_admittance_run();
    void jump_to_adjust();
    void done_cb(const actionlib::SimpleClientGoalState &state, const real_robot_control::screwResultConstPtr &result);
    void active_cb();
    void feedback_cb(const real_robot_control::screwFeedbackConstPtr &feedback);
    void do_orien(const real_robot_control::orientation_pub::ConstPtr& orien_p);

    std::vector<Eigen::Matrix3d> calculateRotationMatrices(int N, double theta);
    Eigen::Matrix3d angleaxistoMatrix(double phi, double theta);



private:
    // 参数
    Eigen::VectorXd adm_m;
    Eigen::VectorXd adm_k;
    Eigen::VectorXd adm_d;

    Eigen::VectorXd world_force = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd local_force = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd tcp_force = Eigen::VectorXd::Zero(6);

    Eigen::Matrix3d link6_rotm;
    Eigen::Vector3d link6_pos;

    Eigen::Matrix3d eef_rotm;
    Eigen::Vector3d eef_pos;
    Eigen::Vector3d eigen_rpy;


    // std::thread excution_thread;
    // std::thread sensor_thread;
    
    //jaka 库相关声明
    Rpy current_rpy;
    RotMatrix current_rotm;

    
    JAKAZuRobot robot;
    RobotStatus status;
    CartesianPose cart;
    CartesianPose new_pos;
    RotMatrix new_rotm;
    Rpy new_rpy;

    // ROS相关成员变量
    std::shared_ptr<ros::NodeHandle> nh;
    ros::Publisher for_pub;
    ros::Publisher pos_pub_6;
    ros::Publisher pos_pub; //xyz theta
    ros::Publisher for_pos_pub;
    real_robot_control::force_pub f;
    real_robot_control::pose_pub pose_p;
    real_robot_control::robot_pos_pub p;
    real_robot_control::force_pos_pub fp;

    ros::Publisher ori_adj_pub;
    real_robot_control::ori_adj_rec oar;

    ros::Subscriber listener_sub;
    // ros::ServiceClient client;
    real_robot_control::screwsrv scr;
    actionlib::SimpleActionClient<real_robot_control::screwAction> client;
    
    const double PI = 3.1415926;

    // std::mutex robot_mutex;
    int loop_rate = 1; //loop_rate * 8ms = real_rate
    double T = loop_rate * 0.008;

    Eigen::Vector3d eef_offset;
    Eigen::Vector3d eef_offset_basic;
    Eigen::Vector3d object_length;
    Eigen::Matrix3d eef_offset_rotm;
    Eigen::Vector3d eef_offset_to_sensor;
    Eigen::Vector3d eef_offset_to_sensor_basic;
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
    Eigen::Matrix3d eef_rotm_d_modified;
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
    // 2：全部运行结束
    
    int screw_execute_status; //末端执行器运行结果 
    // 0：运行后
    // 1：正在运行
    // 2: 运行前

    int adjust_phi;  
    //phi 传递过来的数值
    
    int phi_changed; 
    // 1：改变了
    // 0：改变后，用完了，重置




};