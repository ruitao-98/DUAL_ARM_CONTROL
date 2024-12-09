#include "ros/ros.h"
#include "real_robot_control/JAKAZuRobot.h"
#include "real_robot_control/jkerr.h"
#include "real_robot_control/jktypes.h"
#include <sensor_msgs/JointState.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <string>
#include <map>
#include "std_srvs/Empty.h"
#include "std_srvs/SetBool.h"
#include "std_msgs/Empty.h"
#include <thread>


using namespace std;
JAKAZuRobot left_robot;
JAKAZuRobot right_robot;
const  double PI = 3.1415926;
BOOL in_pos;
int ret_preempt;
int ret_inPos;

typedef actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> Server;

bool jointStates(JointValue joint_pose, int choice)
{
    RobotStatus robotstatus;
    if (choice == 0){
        left_robot.get_robot_status(&robotstatus);
    }
    else{
        right_robot.get_robot_status(&robotstatus);
    }
    
    bool joint_state = true;
   
    for (int i = 0; i < 6; i++)
    {
        bool ret = joint_pose.jVal[i] * 180 / PI - 0.1 < robotstatus.joint_position[i] * 180 / PI
        && robotstatus.joint_position[i] * 180 / PI < joint_pose.jVal[i] * 180 / PI + 0.1;
        joint_state = joint_state && ret; 
    }
    cout << "Whether the robot has reached the target position: " << joint_state << endl;       //1到达；0未到达
    return joint_state;
}

//Moveit server
void goalCb(const control_msgs::FollowJointTrajectoryGoalConstPtr& torso_goal, Server* as)
{
    string left_robot_ip = "192.168.3.200";
    left_robot.login_in(left_robot_ip.c_str()); // 每次执行回调函数都重新登陆，运行完再登出，防止和其他控制机器人程序冲突

    BOOL in_pos;
    // cout << torso_goal->trajectory << endl;
    left_robot.servo_move_enable(false);
    int point_num=torso_goal->trajectory.points.size();
    ROS_INFO("number of points: %d",point_num);
    JointValue joint_pose;
    float lastDuration=0.0;
    OptionalCond* p = nullptr;
    for (int i=1; i<point_num; i++) {        
        joint_pose.jVal[0] = torso_goal->trajectory.points[i].positions[0];
        joint_pose.jVal[1] = torso_goal->trajectory.points[i].positions[1];
        joint_pose.jVal[2] = torso_goal->trajectory.points[i].positions[2];
        joint_pose.jVal[3] = torso_goal->trajectory.points[i].positions[3];
        joint_pose.jVal[4] = torso_goal->trajectory.points[i].positions[4];
        joint_pose.jVal[5] = torso_goal->trajectory.points[i].positions[5];      
        float Duration=torso_goal->trajectory.points[i].time_from_start.toSec();

        float dt=Duration-lastDuration;
        lastDuration=Duration;
        
        int step_num=int (dt * 1/0.008);

        int sdk_res=left_robot.joint_move(&joint_pose, MoveMode::ABS, true, 0.2);  //不拍视频用这个，稳定

        // int sdk_res=left_robot.servo_j(&joint_pose, MoveMode::ABS, step_num);

        if (sdk_res !=0)
        {
            ROS_INFO("Servo_j Motion Failed");
        } 
        ROS_INFO("step_num= %d", step_num);
        ROS_INFO("The return status of servo_j:%d", sdk_res, "the current point is: %d", point_num);
        ROS_INFO("Accepted joint angle: %f %f %f %f %f %f %f %d", joint_pose.jVal[0],joint_pose.jVal[1],joint_pose.jVal[2],joint_pose.jVal[3],joint_pose.jVal[4],joint_pose.jVal[5],dt,step_num);
    }

    while(true)
    {
        if(jointStates(joint_pose, 0))
        {
            left_robot.servo_move_enable(false);
            errno_t resj = left_robot.joint_move(&joint_pose, MoveMode::ABS, true, 0.2); 
            // 进一步执行，此处可以增加关节空间规划的精度，但是，对于urdf 和真实机器人的tcp空间规划精度仍然无法解决
            cout << resj << endl;
            left_robot.login_out();
            ROS_INFO("Servo Mode Disable");
            cout<<"==============Motion stops or reaches the target position=============="<<endl;
            break;
        }

        if ( ret_preempt = as->isPreemptRequested())      
        {
            left_robot.motion_abort();
            left_robot.servo_move_enable(false);

            ROS_INFO("motion_abort Servo Mode Disable");
            cout<<"==============Motion stops or reaches the target position=============="<<endl;
            break;
        }
        ros::Duration(0.5).sleep();
    }
as->setSucceeded();    
ros::Duration(0.5).sleep();
}

void goalCb1(const control_msgs::FollowJointTrajectoryGoalConstPtr& torso_goal, Server* as){

}
void goalCb2(const control_msgs::FollowJointTrajectoryGoalConstPtr& torso_goal, Server* as){

}

//Send the joint value of the physical robot to move_group
void joint_states_callback(ros::Publisher joint_states_pub)
{
    sensor_msgs::JointState joint_position;
    RobotStatus left_robotstatus;
    RobotStatus right_robotstatus;
    left_robot.get_robot_status(&left_robotstatus);
    right_robot.get_robot_status(&right_robotstatus);

  
    for (int i = 0; i < 6; i++)
    {
        joint_position.position.push_back(left_robotstatus.joint_position[i]);
        int j = i + 1;
        joint_position.name.push_back("l_j" + to_string(j));
    }
    for (int i = 0; i < 6; i++)
    {
        joint_position.position.push_back(right_robotstatus.joint_position[i]);
        int j = i + 1;
        joint_position.name.push_back("r_j" + to_string(j));
    }
    joint_position.position.push_back(0);
    joint_position.name.push_back("l_p1");
    joint_position.position.push_back(0);
    joint_position.name.push_back("l_p2");
    joint_position.position.push_back(0);
    joint_position.name.push_back("r_c1");
    joint_position.position.push_back(0);
    joint_position.name.push_back("r_p1");
    joint_position.position.push_back(0);
    joint_position.name.push_back("r_p2");
    joint_position.header.stamp = ros::Time::now();
    
    joint_states_pub.publish(joint_position);
}

void reset(){
    // -np.pi / 3, np.pi / 3, np.pi * 2 / 3, np.pi / 2, -np.pi / 2, np.pi / 2
    JointValue joint_pos = { -PI / 3, PI / 3, PI * 2 / 3, PI / 2, -PI / 2,  PI / 2 };
    right_robot.joint_move(&joint_pos, ABS, TRUE, 0.15);
}

int main(int argc, char *argv[])
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "moveit_server");
    ros::NodeHandle nh;
    string left_robot_ip = "192.168.3.200";
    string right_robot_ip = "192.168.3.201";
    left_robot.login_in(left_robot_ip.c_str());
    right_robot.login_in(right_robot_ip.c_str());
    // robot.set_status_data_update_time_interval(100);
    ros::Rate rate(125);
    left_robot.servo_move_enable(false);
    ros::Duration(0.5).sleep();
    //Set filter parameter
    left_robot.servo_move_use_joint_LPF(0.5);
    left_robot.power_on();
    left_robot.enable_robot();
    right_robot.power_on();
    right_robot.enable_robot();
    reset();
    left_robot.login_out();
    right_robot.login_out();
    //Create topic "/joint_states"
    ros::Publisher joint_states_pub = nh.advertise<sensor_msgs::JointState>("/joint_states", 10);
    //Create action server object
    Server moveit_server(nh, "arm/follow_joint_trajectory", boost::bind(&goalCb, _1, &moveit_server), false);
    moveit_server.start();
    Server gripper(nh, "gripper/follow_joint_trajectory", boost::bind(&goalCb1, _1, &gripper), false);
    gripper.start();
    Server screw(nh, "screw_tool/follow_joint_trajectory", boost::bind(&goalCb2, _1, &screw), false);
    screw.start();
    cout << "==================Moveit Start==================" << endl;

    while(ros::ok())
    {
        //Report robot joint information to RVIZ
        joint_states_callback(joint_states_pub);
        rate.sleep();
        ros::spinOnce();
    }
    //ros::spin();
}