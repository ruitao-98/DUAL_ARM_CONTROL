#ifndef SERVO_MOTOR_FUNCTION_H_
#define SERVO_MOTOR_FUNCTION_H_

#include "dynamixel_sdk.h"
#include <stdlib.h>
#include <stdio.h>


//Control table-mx/x
#define Return_delay_time          9
#define Operating_Mode          11
#define Max_Voltage_Limit       32
#define Min_Voltage_Limit       34
#define Acceleration_Limit      40
#define Velocity_Limit          44
#define Max_Position_Limit      48
#define Min_Position_Limit      52
#define Torque_Enable           64
#define LED                     65

#define GOAL_PWM                100
#define Goal_Current            102
#define GOAL_Velocity           104
#define Velocity_profile        112
#define Present_Current         126
#define Present_velocity        128
#define Present_Position        132
#define Present_Temperature     146
#define GOAL_POSITION           116

//Protocol version
#define Protocol_version_2      2.0

//PC
#define Baudratemx              57600
#define device_port             "/dev/ttyUSB1"

//PC
#define Baudratexm              1000000

class motors
{
private:
	//Motors
	uint8_t m_ID;
	//Control
	int8_t moving;
	int16_t dxl_present_load16;
	int16_t dxl_present_position16;
	int16_t dxl_present_velocity16;
	//int8_t m_operating_mode;
	int32_t dxl_present_position;
	int32_t dxl_present_velocity;
	int16_t dxl_present_current;
	//Communication instance and variables
	dynamixel::PacketHandler* packetHandler;  //抽象类
	dynamixel::PortHandler* portHandler;
	int dxl_comm_result;
	uint8_t dxl_error;

public:
	//构造函数
	motors();
	//析构函数
	~motors();

	void open_port();
	void close_port();

	

	/* xm mx */
	void setbaundrate();
	
	bool torque_on(int m_ID);
	void torque_off(int m_ID);

	void setdelaytime(int m_ID,int time);

	void set_velocitymode(int m_ID);

	void set_expositionmode(int m_ID);

	void set_currentmode(int m_ID);

	int get_presentvelocity(int m_ID);

	int get_presentposition(int m_ID);

	int get_presentcurrent(int m_ID);

	void set_goalposition(int m_ID, int position);

	void set_goalvelocity(int m_ID, int velocity);

	void set_goalcurrent(int m_ID, int current);

	void set_goalprofile(int m_ID, int velocity);

};

#endif // !SERVO_MOTOR_FUNCTION_H_