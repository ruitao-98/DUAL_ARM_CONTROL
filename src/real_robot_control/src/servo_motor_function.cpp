#include "real_robot_control/servo_motor_function.h"

void motors::open_port()
{
	if (portHandler->openPort())
	{

		printf("Motor %d: Successed to open the port!\n", m_ID);
	}
	else
	{
		char* a = portHandler->getPortName();
		printf(a);
		printf("\n");
		printf("Motor %d: Failed to open the port!\n", m_ID);
	}
}

void motors::close_port()
{
	portHandler->closePort();
}

void motors::setbaundrate()
{
	portHandler->setBaudRate(Baudratemx);
	printf("set baundrate to %d!\n", Baudratemx);
}


void motors::torque_on(int m_ID)
{
	dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, m_ID, Torque_Enable, 1, &dxl_error);
	if ((dxl_error == 0) && (dxl_comm_result == 0))
		printf("Motor %d: Torque Enabled!\n", m_ID);
	else
		printf("Motor %d: Failed to Enable Torque!\n", m_ID);
}
void motors::torque_off(int m_ID)
{
	packetHandler->write1ByteTxRx(portHandler, m_ID, Torque_Enable, 0, &dxl_error);
}
void motors::setdelaytime(int m_ID, int time)
{
	int re_time;
	dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, m_ID, Return_delay_time, time, &dxl_error);
	re_time = time * 2;
	if ((dxl_error == 0) && (dxl_comm_result == 0))
		printf("Motor %d: Set return delay time to %d us!\n", m_ID, re_time);
	else
		printf("Motor %d: Failed to set return delay time!\n", m_ID);
}

void motors::set_velocitymode(int m_ID)
{
	dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, m_ID, Operating_Mode, 1, &dxl_error);
	if ((dxl_error == 0) && (dxl_comm_result == 0))
		printf("Motor %d: Set to Velocity Mode!\n", m_ID);
	else
		printf("Motor %d: Failed to Set to Velocity Mode!\n", m_ID);
}
void motors::set_expositionmode(int m_ID)
{
	dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, m_ID, Operating_Mode, 4, &dxl_error);
	if ((dxl_error == 0) && (dxl_comm_result == 0))
		printf("Motor %d: Set to Extended Position Mode!\n", m_ID);
	else
		printf("Motor %d: Failed to Set to Extended Position Mode!\n", m_ID);
}

void motors::set_currentmode(int m_ID)
{
	dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, m_ID, Operating_Mode, 0, &dxl_error);
	if ((dxl_error == 0) && (dxl_comm_result == 0))
		printf("Motor %d: Set to Current Mode!\n", m_ID);
	else
		printf("Motor %d: Failed to Set to Current Mode!\n", m_ID);
}

int motors::get_presentvelocity(int m_ID)
{
	dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, m_ID, Present_velocity, (uint32_t*)&dxl_present_velocity, &dxl_error);
	if ((dxl_error == 0) && (dxl_comm_result == 0))
		printf("Motor %d: Present Velocity: %d\n", m_ID, dxl_present_velocity);
	else
		printf("Failed to Read the Velocity!\n");
	return dxl_present_velocity;
}

int motors::get_presentcurrent(int m_ID)
{
	dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, m_ID, Present_Current, (uint16_t*)&dxl_present_current, &dxl_error);
	if ((dxl_error == 0) && (dxl_comm_result == 0)){
		
	}
		// printf("Motor %d: Present Current: %d\n", m_ID, dxl_present_current);
	else
		printf("Failed to Read the Current!\n");
	return dxl_present_current;
}

int motors::get_presentposition(int m_ID)
{
	dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, m_ID, Present_Position, (uint32_t*)&dxl_present_position, &dxl_error);
	if ((dxl_error == 0) && (dxl_comm_result == 0))
		printf("Motor %d: Present Position: %d\n", m_ID, dxl_present_position);
	else{
        printf("Failed to Read the Position!\n");
    }
	return dxl_present_position;
}

void motors::set_goalposition(int m_ID, int position)
{
	dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, m_ID, GOAL_POSITION, position, &dxl_error);
	if ((dxl_error == 0) && (dxl_comm_result == 0))
		printf("Motor %d: Set Goal Position: %d!\n", m_ID, position);
	else
		printf("Failed to Set Goal Position!\n");
}

void motors::set_goalvelocity(int m_ID, int velocity)
{
	dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, m_ID, GOAL_Velocity, velocity, &dxl_error);
	if ((dxl_error == 0) && (dxl_comm_result == 0))
		printf("Motor %d: Set Goal Velocity: %d!\n", m_ID, velocity);
	else
		printf("Failed to Set Goal Velocity!\n");
}

void motors::set_goalcurrent(int m_ID, int current)
{
	dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, m_ID, Goal_Current, current, &dxl_error);
	if ((dxl_error == 0) && (dxl_comm_result == 0))
		printf("Motor %d: Set Goal Current: %d!\n", m_ID, current);
	else
		printf("Failed to Set Goal Current!\n");
}

void motors::set_goalprofile(int m_ID, int velocity)
{
	dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, m_ID, Velocity_profile, velocity, &dxl_error);
	if ((dxl_error == 0) && (dxl_comm_result == 0))
		printf("Motor %d: Set Velocity Profile in Position Mode: %d!\n", m_ID, velocity);
	else
		printf("Failed to Set Velocity Profile in Position Mode!\n");
}


//
motors::motors(): portHandler(dynamixel::PortHandler::getPortHandler(device_port)), 
				packetHandler(dynamixel::PacketHandler::getPacketHandler(Protocol_version_2))
{
	dxl_comm_result = COMM_TX_FAIL;
	dxl_error = 0;
	m_ID = 0;
	dxl_present_position = 0;
	dxl_present_velocity = 0;
	dxl_present_current = 0;
	moving = 0;
	dxl_present_load16 = 0;
	dxl_present_position16 = 0;
	dxl_present_velocity16 = 0;

}

motors::~motors(){

} 