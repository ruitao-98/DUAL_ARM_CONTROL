#include "real_robot_control/screwing_tool.h"
#include "real_robot_control/current_pub.h"
#include<cmath>
#include <unistd.h>
#include <thread>
#include <chrono>
#include <mutex>

using namespace std;

float endeffector::average_function(int *data, int length) {

	for (int i = 0; i < length; i++)
	{
		data[i] = data[i];
	}
	int max = data[0];
	int min = data[0];
	int sum = 0;
	for (int i = 0; i < (length); i++) {
		if (data[i] > max)
		{
			max = data[i];
		}
		if (data[i] < min) {
			min = data[i];
		}
		sum = sum + data[i];
	}
	float ave;
	ave = (float(sum - max - min) / (length - 2));
	return ave;
}



void endeffector::width_reduce(int distance, ros::Publisher &pub, real_robot_control::current_pub &msg) {

	std::vector<int> current_vec; //电流容器

	ifstream in("delta_width.txt");
	string content;
	while (getline(in, content))
	{
		cout << content << endl;
	}
	in.close();
	int goal_position = atoi(content.c_str());

    this->open_port();
	this->setbaundrate();
		// 检测 夹持装置的电机 是否打开
	while (!(this->torque_on(1))) {
		cout << "\r" <<"******请打开夹持装置的电源，以执行功能*********" << flush; 
	}
	this->setdelaytime(1, 0);
	if (distance < 0)
	{
		distance = -distance;
	}
	else {
		distance = distance;
	}

    this->setbaundrate();
	this->torque_off(1);
	this->set_expositionmode(1);
	this->set_goalprofile(1, 200);
	this->torque_on(1);
	int start_position_1 = this->get_presentposition(1);
	int goal_position_1 = int(start_position_1 - 4095 * distance);
	std::cout << goal_position_1 <<std::endl;
	int item = 0;
	while ( item < 1000)
	{	
		item = item + 1;
		msg.current = 0.0; // Assuming 'current_position' is a field in your custom message
		pub.publish(msg);
		cout << "wait" << item << endl;
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}
	
	this->set_goalposition(1, goal_position_1);
	sleep(1);
	while (true) {
		int current = this->get_presentcurrent(1);
		msg.current = current; // Assuming 'current_position' is a field in your custom message
		current_vec.push_back(current);
		pub.publish(msg);
		if (this->get_presentvelocity(1) >= -1) {
			this->torque_off(1);
			goal_position = goal_position - 4095 * distance;
			fstream ofs;
			ofs.open("delta_width.txt", ios::out);
			ofs << goal_position;
			ofs.close();
			break;
		}
	}

	time_t now;
    struct tm time_info;
    // Get the current time
    time(&now);
    // Convert the `time_t` value to local time in a thread-safe way
    localtime_r(&now, &time_info);
	// 创建一个时间格式化的字符串流
	std::ostringstream oss;
	oss << std::put_time(&time_info, "%Y-%m-%d_%H-%M-%S"); // 格式化时间 YYYY-MM-DD_HH-MM-SS
	std::string time_str = oss.str();
	// 创建文件名，包含当前时间
	std::string file_name1 = "current_s1_" + time_str + ".txt";
	// 打开文件流
	std::ofstream file1(file_name1);
	// 将列表的元素写入文件
	for (double elem1 : current_vec) {
		file1 << elem1 << std::endl;
	}
	// 关闭文件流
	file1.close();

    this->close_port();
}

void endeffector::width_increase(int distance, ros::Publisher &pub, real_robot_control::current_pub &msg) {
	std::vector<int> current_vec; //电流容器
	ifstream in("delta_width.txt");
	string content;
	while (getline(in, content))
	{
		cout << content << endl;
	}
	in.close();
	int goal_position = atoi(content.c_str());

    this->open_port();
	this->setbaundrate();
			// 检测 夹持装置的电机 是否打开
	while (!(this->torque_on(1))) {
		cout << "\r" <<"******请打开夹持装置的电源，以执行功能*********" << flush; 
	}
	this->setdelaytime(1, 0);
	if (distance < 0)
	{
		distance = -distance;
	}
	else {
		distance = distance;
	}
	int item = 0;
	while ( item < 1000)
	{	
		item = item + 1;
		msg.current = 0.0; // Assuming 'current_position' is a field in your custom message
		pub.publish(msg);
		cout << "wait" << item << endl;
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}
	
    this->setbaundrate();
	this->torque_off(1);
	this->set_expositionmode(1);
	this->set_goalprofile(1, 200);
	this->torque_on(1);
	int start_position_1 = this->get_presentposition(1);
	int goal_position_1 = int(start_position_1 + 4095 * distance);
	this->set_goalposition(1, goal_position_1);
	sleep(1);
	while (true) {
		int current = this->get_presentcurrent(1);
		msg.current = current; // Assuming 'current_position' is a field in your custom message
		pub.publish(msg);
		current_vec.push_back(current);
		if (this->get_presentvelocity(1) <= 1) {
			this->torque_off(1);
			sleep(0.5);

			goal_position = goal_position + 4095 * distance;
			fstream ofs;
			ofs.open("delta_width.txt", ios::out);
			ofs << goal_position;
			ofs.close();
			break;
		}
	}
	time_t now;
    struct tm time_info;
    // Get the current time
    time(&now);
    // Convert the `time_t` value to local time in a thread-safe way
    localtime_r(&now, &time_info);
	// 创建一个时间格式化的字符串流
	std::ostringstream oss;
	oss << std::put_time(&time_info, "%Y-%m-%d_%H-%M-%S"); // 格式化时间 YYYY-MM-DD_HH-MM-SS
	std::string time_str = oss.str();
	// 创建文件名，包含当前时间
	std::string file_name1 = "current_s1_" + time_str + ".txt";
	// 打开文件流
	std::ofstream file1(file_name1);
	// 将列表的元素写入文件
	for (double elem1 : current_vec) {
		file1 << elem1 << std::endl;
	}
	// 关闭文件流
	file1.close();
    this->close_port();
}

int endeffector::width_reduce_or_increase_full(int judge){
    this->open_port();
	this->setbaundrate();
			// 检测 夹持装置的电机 是否打开
	while (!(this->torque_on(1))) {
		cout << "\r" <<"******请打开夹持装置的电源，以执行功能*********" << flush; 
	}
	this->setdelaytime(1, 0);
	this->torque_off(1);
	this->set_velocitymode(1);
	this->torque_on(1);
	int base_current[15] = { 0 };
	fstream ofs;
	int start_position = this->get_presentposition(1);
	if (judge == 1){
		this->set_goalvelocity(1, -200); //reduce
	}
	else if (judge == -1)
	{
		this->set_goalvelocity(1, 200); //increase
	}

	sleep(1);
	for (int i = 0; i < 15; i = i + 1) {
		base_current[i] = this->get_presentcurrent(1);
	}
	float start_current = average_function(base_current, 15);
	printf("the started current%.3f \n", start_current);
	int yuzhi = 35;
	int present_cu[5] = { 0 };
	int i = 0;
	int fin_position;
    double ave_current;
	while (true)
	{
		int j;
		j = i % 5;
		i = i + 1;
		present_cu[j] = this->get_presentcurrent(1);
    if (i<=5){
      ave_current = start_current;
    }
    if (i > 5){
      ave_current = average_function(present_cu, 5);
    }
		if ((fabs(ave_current - start_current) > yuzhi) && (i > 5))
		{
			printf("ֹͣthe present current %.3f\n", ave_current);
			this->set_goalvelocity(1, 0);
			sleep(0.5);
			int end_position = this->get_presentposition(1);
			int delta_position = end_position - start_position;

			ofs.open("delta_width.txt", ios::out);
			ofs << delta_position;
			ofs.close();

			this->torque_off(0);
			return 0;
			break;
		}
	}
  this->close_port();
  return 1;
}

int endeffector::width_recovery(){
    this->open_port();
	this->setbaundrate();
			// 检测 夹持装置的电机 是否打开
	while (!(this->torque_on(1))) {
		cout << "\r" <<"******请打开夹持装置的电源，以执行功能*********" << flush; 
	}
	this->setdelaytime(1, 0);
    this->setbaundrate();
	this->torque_off(1);
	ifstream in("delta_width.txt");
	string content;
	while (getline(in, content))
	{
		cout << content << endl;
	}
	in.close();
	int delta_position = atoi(content.c_str());

	this->set_expositionmode(1);
	this->set_goalprofile(1, 200);
	this->torque_on(1);
	int start_position_1 = this->get_presentposition(1);
	int goal_position_1 = int(start_position_1 - delta_position);
	this->set_goalposition(1, goal_position_1);
	sleep(1);
	while (true) {
		if (this->get_presentvelocity(1) <= 1) {
			this->torque_off(1);
			sleep(0.5);
			int final_position = this->get_presentposition(1);
			ofstream ofs;
			ofs.open("delta_width.txt", ios::out);
			ofs << (final_position - start_position_1) + delta_position;
			ofs.close();
			break;
		}
	}
  this->close_port();
  return 3;
}



int endeffector::screwing_s1(int speed, float circle, ros::Publisher &pub, real_robot_control::current_pub &msg)
{
	if (speed < 0) {
		speed = -speed;
	}
	else {
		speed = speed;
	}
	if (circle < 0) {
		circle = circle;
	}
	else
	{
		circle = -circle;
	}

	 // Assuming 'current_position' is a field in your custom message
	fstream ofs;
    this->open_port();
	this->setbaundrate();
	// 检测 夹持装置的电机 是否关闭
	while (this->torque_on(1)) {
		cout << "\r" <<"******请关闭夹持装置的电源，以保护电机*********" << flush; 
	}
	this->setdelaytime(0, 0);
	this->torque_off(0);
	this->set_expositionmode(0);
	this->set_goalprofile(0, speed);
	this->torque_on(0);
	int goal_position;
	int present_position;
	present_position = this->get_presentposition(0);

	ofs.open("standard.txt", ios::out);
	ofs << present_position;
	ofs.close();

	goal_position = int(present_position + 6.238 * 4095 * circle); 
	this->set_goalposition(0, goal_position);
	int base_current[15] = { 0 };

	std::this_thread::sleep_for(std::chrono::milliseconds(100)); 
	for (int i = 0; i < 15; i = i + 1) {
		base_current[i] = this->get_presentcurrent(0);
	}
	float start_current = average_function(base_current, 15);
	printf("the started current%.3f \n", start_current);
	int yuzhi = 35;
	int present_cu[10] = { 0 };
	int i = 0;
	// std::this_thread::sleep_for(std::chrono::milliseconds(200)); 
	int fin_position;
  	float ave_current;
	while (true)
	{
		int j;
		j = i % 10;
		i = i + 1;
		present_cu[j] = this->get_presentcurrent(0);
		int realtime_position = this->get_presentposition(0);
		if (i<=10){
			ave_current = start_current;
		}
		if (i > 10){
			ave_current = average_function(present_cu, 10);
			msg.current = ave_current;
			// cout << msg.current << endl;
		}
		printf("ֹͣthe present current %.3f\n", ave_current);
		int pre_velo = this->get_presentvelocity(0);
		printf("ֹͣthe present velocity %d\n", pre_velo);
		pub.publish(msg);
		if ((fabs(ave_current - start_current) > yuzhi) && (i > 10))
		{
			printf("ֹͣthe ended current %.3f\n", ave_current);
			printf("ͣwith jamming! \n");
			this->set_goalvelocity(0, 0);
			sleep(1);

			fin_position = this->get_presentposition(0);
			this->torque_off(0);
      		this->close_port();

			// 存储位置
			int delta_position = fin_position - present_position;
			ofs.open("delta_circle.txt", ios::out);
			ofs << delta_position;
			ofs.close();

			return 1;
			break;
		}
		else if (realtime_position<=goal_position+2)
		{
			fin_position = this->get_presentposition(0);
			// 存储位置
			int delta_position = fin_position - present_position;
			printf("ֹthe end position %.3f\n", (float(fin_position - present_position) / 4095));
			ofs.open("delta_circle.txt", ios::out);
			ofs << delta_position;
			ofs.close();
			this->torque_off(0);
			this->close_port();

			return 0;
			break;
		}
	}

}

int endeffector::screwing_s2(int speed, ros::Publisher &pub, real_robot_control::current_pub &msg)
{
	if (speed < 0) {
		speed = speed;
	}
	else {
		speed = -speed;
	}
	this->open_port();
	this->setbaundrate();
	// 检测 夹持装置的电机 是否关闭
	while (this->torque_on(1)) {
		cout << "\r" <<"******请关闭夹持装置的电源，以保护电机*********" << flush; 
	}
	this->setdelaytime(0, 0);
	this->torque_off(0);
	this->set_velocitymode(0);
	this->torque_on(0);
	this->set_goalvelocity(0, speed);
	int start_position = this->get_presentposition(0);

	int base_current[15] = { 0 };
	std::this_thread::sleep_for(std::chrono::milliseconds(100)); 
	for (int i = 0; i < 15; i = i + 1) {
		base_current[i] = this->get_presentcurrent(0);
	}
	double start_current = average_function(base_current, 15);
	printf("the started current %.3f \n", start_current);
	for (int m = 0; m < 15; m++) {
		std::cout << base_current[m] << std::endl;
	}
	int yuzhi = 35;
	int goal_position = int(start_position - 4095 * 6.238 * 2);
	int present_cu[5] = { 0 };
	int i = 0;
  	double ave_current;
	while (true)	
	{
		int j;
		j = i % 5;
		i = i + 1;
		present_cu[j] = this->get_presentcurrent(0);
		int present_position = this->get_presentposition(0);

		if (i <= 5) {
			ave_current = start_current;
			msg.current = ave_current;
		}
		if (i > 5) {
     		ave_current = average_function(present_cu, 5);
			printf("the present current %.3f \n", ave_current);
			msg.current = ave_current;
		}
		pub.publish(msg);
		if ((fabs(ave_current - start_current) > yuzhi) && (i > 5))
		{
			printf("\n");
			printf("ֹͣthe final current%.3f\n", ave_current);
			this->set_goalvelocity(0, 0);
			this->torque_off(0);
			return 2; // 表示所有任务都结束了
			break;
		}
		else if (present_position <= goal_position)
		{
			printf("final contact\n");
			this->set_goalvelocity(0, 0);
			this->torque_off(0);
			return 0;
			break;
		}
	}
}

int endeffector::screwing_s3(int speed, int yuzhi) {
	printf("screwing s3");
	if (speed < 0) {
		speed = speed;
	}
	else {
		speed = -speed;
	}
	this->open_port();
	this->setbaundrate();
		// 检测 夹持装置的电机 是否关闭
	while (this->torque_on(1)) {
		cout << "\r" <<"******请关闭夹持装置的电源，以保护电机*********" << flush; 
	}
	this->setdelaytime(0, 0);
	this->torque_off(0);
	this->set_velocitymode(0);
	this->torque_on(0);
	this->set_goalvelocity(0, speed);
	int start_position = this->get_presentposition(0);
	int base_current[15] = { 0 };
	sleep(1);
	for (int i = 0; i < 15; i = i + 1) {
		base_current[i] = this->get_presentcurrent(0);
	}
	double start_current = average_function(base_current, 15);
	printf("the start current %.3f \n", start_current);
	for (int m = 0; m < 15; m++) {
		std::cout << base_current[m] << std::endl;
	}
	int present_cu[5] = { 0 };
	int i = 0;
	int end_position = 0;
	int num = 0;
  double ave_current;
	while (true) {
		int j;
		j = i % 5;
		i = i + 1;
		present_cu[j] = this->get_presentcurrent(0);

    if (i<=5){
      ave_current = start_current;
    }
		if (i > 5) {
			printf("the present current%.3f \n", ave_current);
      ave_current = average_function(present_cu, 5);
		}
		if ((fabsf(ave_current - start_current) > yuzhi) && (i > 5))
		{
			printf("ֹͣthe final current%.3f\n", ave_current);
			this->set_goalvelocity(0, 0);
			end_position = this->get_presentposition(0);
			this->torque_off(0);
			break;
		}
		end_position = this->get_presentposition(0);
	}
	std::cout << end_position << std::endl;
	return end_position;
}

int endeffector::unscrew_to_zero(int speed) {
	if (speed < 0) {
		speed = -speed;
	}
	else {
		speed = speed;
	}

	ifstream in("delta_circle.txt");
	string content;
	while (getline(in, content))
	{
		cout << content << endl;
	}
	in.close();
	int delta_position = atoi(content.c_str());

	this->open_port();
	this->setbaundrate();
		// 检测 夹持装置的电机 是否关闭
	while (this->torque_on(1)) {
		cout << "\r" <<"******请关闭夹持装置的电源，以保护电机*********" << flush; 
	}
	this->setdelaytime(0, 0);
	this->torque_off(0);
	this->set_expositionmode(0);
	this->set_goalprofile(0, speed);
	this->torque_on(0);
	int start_position_1 = this->get_presentposition(0);
	int goal_position_1 = int(start_position_1 + abs(delta_position)); // 加上delta_position，正转，拧出
	this->set_goalposition(0, goal_position_1);
	sleep(1);
	while (true) {
		if (this->get_presentvelocity(0) == 0) {
			this->torque_off(0);
			break;
		}
	}
	return 0;
}

int endeffector::unscrewing_s1(int speed_1, int speed_2) {
	if (speed_1 < 0) {
		speed_1 = -speed_1;
	}
	else {
		speed_1 = speed_1;
	}
	if (speed_2 < 0) {
		speed_2 = -speed_2;
	}
	else {
		speed_2 = speed_2;
	}
	this->open_port();
	this->setbaundrate();
		// 检测 夹持装置的电机 是否关闭
	while (this->torque_on(1)) {
		cout << "\r" <<"******请关闭夹持装置的电源，以保护电机*********" << flush; 
	}
	this->setdelaytime(0, 0);
	this->torque_off(0);
	this->set_expositionmode(0);
	this->set_goalprofile(0, speed_1);
	this->torque_on(0);
	int start_position_1 = this->get_presentposition(0);
	int goal_position_1 = int(start_position_1 + 4095 * 1 * 6.238);
	this->set_goalposition(0, goal_position_1);
	sleep(1);
	while (true) {
		if (this->get_presentvelocity(0) == 0) {
			this->torque_off(0);
			break;
		}
	}
}

void endeffector::unscrewing_sx(int speed) {
	if (speed < 0) {
		speed = -speed;
	}
	else {
		speed = speed;
	}
	using namespace std;
	this->open_port();
		// 检测 夹持装置的电机 是否关闭
	while (this->torque_on(1)) {
		cout << "\r" <<"******请关闭夹持装置的电源，以保护电机*********" << flush; 
	}
	this->setbaundrate();
	this->setdelaytime(0, 0);
	this->torque_off(0);
	this->set_expositionmode(0);
	this->set_goalprofile(0, speed);
	this->torque_on(0);
	int start_position_1 = this->get_presentposition(0);
	int goal_position_1 = start_position_1 + int(4095 * 6.238 * 2.28);
	this->set_goalposition(0, goal_position_1);
	sleep(1);
	while (true) {
		if (this->get_presentvelocity(0) == 0) {
			this->torque_off(0);
			break;
		}
	}
}

void endeffector::unscrewing_final(int speed, float circle) {
	if (speed < 0) {
		speed = -speed;
	}
	else {
		speed = speed;
	}
	if (circle < 0) {
		circle = -circle;
	}
	else {
		circle = circle;
	}
	this->setbaundrate();
		// 检测 夹持装置的电机 是否关闭
	while (this->torque_on(1)) {
		cout << "\r" <<"******请关闭夹持装置的电源，以保护电机*********" << flush; 
	}
	this->setdelaytime(0, 0);
	this->torque_off(0);
	this->set_expositionmode(0);
	this->set_goalprofile(0, speed);
	this->torque_on(0);
	int start_position_1 = this->get_presentposition(0);
	int goal_position_1 = start_position_1 + int(4095 * 6.238 * circle);
	this->set_goalposition(0, goal_position_1);
	sleep(0.5);
	while (true) {
		if ((this->get_presentvelocity(0) == 0)) {
			this->torque_off(0);
			break;
		}
	}
}

void endeffector::rotate_to(float angle) {

	this->setbaundrate();
		// 检测 夹持装置的电机 是否关闭
	while (this->torque_on(1)) {
		cout << "\r" <<"******请关闭夹持装置的电源，以保护电机*********" << flush; 
	}
	this->setdelaytime(0, 0);
	this->torque_off(0);
	this->set_expositionmode(0);
	this->set_goalprofile(0, 150);
	this->torque_on(0);
	int start_position_1 = this->get_presentposition(0);
	printf("angle; %f\n", angle);
	int delta = int(4095 * (6.238 * (angle / 360)));
	printf("delta; %d\n", delta);
	int goal_position_1 = start_position_1 + delta;
	this->set_goalposition(0, goal_position_1);
	sleep(1000);
	while (true) {
		if (this->get_presentvelocity(0) == 0) {
			this->torque_off(0);
			break;
		}
	}
	printf("the delta position %0.3f\n", (((double(this->get_presentposition(0)) - start_position_1)) * 360.0) / (4095 * 6.238));
}

int endeffector::measure_angle(int standard) {
	this->setbaundrate();
		// 检测 夹持装置的电机 是否关闭
	while (this->torque_on(1)) {
		cout << "\r" <<"******请关闭夹持装置的电源，以保护电机*********" << flush; 
	}
	this->setdelaytime(0, 0);
	this->torque_off(0);
	this->set_expositionmode(0);
	this->set_goalprofile(0, 150);
	this->torque_on(0);
	int present = this->get_presentposition(0);
	int delt = abs(present - standard);

	double judge = (double)delt / (4095.0 * 6.238) - int((double)delt / (4095.0 * 6.238));
	if (present > standard) {
		judge = judge;
	}
	else {
		judge = 1 - judge;
	}
	double angle_real = judge * 360.0;
	if (angle_real <= 180) {
		angle_real = angle_real + 180;
	}
	else {
		angle_real = angle_real - 180;
	}
	int angle = angle_real;
	return angle;
}

void endeffector::screw_to_zero() {
	ifstream in("standard.txt");
	string content;
	while (getline(in, content))
	{
		cout << content << endl;
	}
	in.close();
	int standard = atoi(content.c_str());
	this->open_port();
	this->setbaundrate();
		// 检测 夹持装置的电机 是否关闭
	while (this->torque_on(1)) {
		cout << "\r" <<"******请关闭夹持装置的电源，以保护电机*********" << flush; 
	}
	this->setdelaytime(0, 0);
	this->torque_off(0);
	this->set_expositionmode(0);
	this->set_goalprofile(0, 150);
	this->torque_on(0);
	int present = this->get_presentposition(0);
	int delt = abs(present - standard);
	int goal_position;
	double judge = (double)delt / (4095.0 * 6.238) - int((double)delt / (4095.0 * 6.238));
	if (judge > 0.5) {
		double goal = 4095.0 * 6.238 - judge * 4095.0 * 6.238;
		if (present > standard) {
			goal_position = (int)goal + present;
		}
		else {
			goal_position = -(int)goal + present;
		}
	}
	else {
		double goal = judge * 4095.0 * 6.238;
		if (present > standard) {
			goal_position = -(int)goal + present;
		}
		else {
			goal_position = (int)goal + present;
		}
	}
	this->set_goalposition(0, goal_position);
	std::this_thread::sleep_for(std::chrono::milliseconds(100)); 
	while (true) {
		if (this->get_presentvelocity(0) == 0) {
			this->torque_off(0);
			break;
		}
	}
}

endeffector::endeffector() {};
