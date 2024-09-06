#include "real_robot_control/servo_motor_function.h"
#include "real_robot_control/current_pub.h"
#include "real_robot_control/width_pub.h"
#include <iostream>
#include <fstream>
#include <string>
#include "ros/ros.h"



class endeffector: public motors
{
private:
	std::string delta_width = "/home/yanji/dual_arm_control/src/real_robot_control/launch/delta_width.txt";
	std::string delta_circle = "/home/yanji/dual_arm_control/src/real_robot_control/launch/delta_circle.txt";
	std::string standard_path = "/home/yanji/dual_arm_control/src/real_robot_control/launch/standard.txt";

public:
	endeffector();

	void width_reduce(int distance, ros::Publisher &pub, real_robot_control::current_pub &msg);
	void width_increase(int distance, ros::Publisher &pub, real_robot_control::current_pub &msg, 
	ros::Publisher &pub1, real_robot_control::width_pub &msg1);

	int width_reduce_full_for_handover(int goal_width, ros::Publisher &pub1, real_robot_control::width_pub &msg1); 
    int width_reduce_or_increase_full(int judge); 
    int width_recovery();
    /**
	* @brief +1 reduce -1 increase
    * @return void
	*/

	int screwing_s1(int speed, float circle, ros::Publisher &pub, real_robot_control::current_pub &msg);
	int screwing_s2(int speed, ros::Publisher &pub, real_robot_control::current_pub &msg); 
	int screwing_s3(int speed,int yuzhi);


	//
	int unscrewing_s1(int speed_1, int speed_2);
	void unscrewing_sx(int speed);
	void unscrewing_final(int speed, float circle);

	float average_function(int *data, int length);

	void rotate_to(float angle);

	int unscrew_to_zero(int speed);

	void screw_to_zero();

	int measure_angle(int standard);



};