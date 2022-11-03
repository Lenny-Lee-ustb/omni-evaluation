#ifdef __linux__
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#endif

#include <unistd.h>

#include <chrono>
#include <thread>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <signal.h>
#include <fstream>
#include <iostream>
#include <stdio.h>
#include <math.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#define GM6020_KV  13.33 // constant rpm/V. direct-drive motor, output 0~24V -> 0~320rpm
#define GM6020_K  0.741 // constant Nm/A. 
#define GM6020_T_MAX 1.2 // max torque 1.2 Nm
#define GM6020_N_MAX 320 // max speed 320 rpm
#define GM6020_V_MAX 30000.0 // max voltage output 24V (30000)

// 上电位置为
// n = KV * V
// v_out = V / 24.0 * 30000 

class Platform_GM6020
{
	double P_angle, D_angle; // parameter for motor tune
	double a_1,b_0,b_1; // parameter for filter
public:
	Platform_GM6020();
	int16_t MotorPosTune();
	double AngleRound(double theta);
	void sendZero(int s);
	double angleCalculate(double vx,double vy);
	void calWheelSpeed(double vx,double vy, double avz);
	
	int16_t curRx, velRx, angleRx, volTx;
    int8_t thermalRx; // Monitor thermal

	double volOut; 
	double angle, angleLast, angleVel; // state
	double angleDes, angleErr, angleRate, angleInit; // control

	// servo position in robot corrdinate
	double x_wheel, y_wheel;
	double vx_wheel, vy_wheel;

};

Platform_GM6020::Platform_GM6020()
{
	P_angle = 15.0;
	D_angle = 0.0;
}

// return voltage command to send in CAN bus
int16_t Platform_GM6020::MotorPosTune(){
	// This method is borrowed from MIT Cheetah, which is used in Tmotor AK series. 
	int16_t vRef;
	double v;

	angleErr = angleDes - angle;
	if (angleErr > M_PI)
	{
		angleErr = angleErr - 2 * M_PI;
	}
	else if (angleErr < - M_PI)
	{
		angleErr = angleErr + 2 * M_PI;
	}
	
	angleRate = P_angle * angleErr;
	v =  (angleRate / (2* M_PI / 60.0 * GM6020_KV) ) / 24.0 * 30000.0;
	v = std::min(std::max(v, -GM6020_V_MAX), GM6020_V_MAX);
	vRef = (int16_t) round(v);
	volOut = vRef /30000.0 *24.0;

	return vRef;
}

double Platform_GM6020::AngleRound(double theta){
	while (theta > M_PI || theta < -M_PI)
	{
		if (theta> M_PI)
		{
			theta = theta - 2 * M_PI;
		}
		if (theta < -M_PI)
		{
			theta = theta + 2 * M_PI;
		}
		
	}
	return theta;
}


void Platform_GM6020::sendZero(int s)
{
	int nbytes;
    can_frame frame;
	frame.can_id = 0x1FF;
	frame.can_dlc = 8;
    for(int i=0; i<4; i++){
        frame.data[2*i] = 0;
        frame.data[2*i+1] = 0;
    }
	nbytes = write(s, &frame, sizeof(struct can_frame));

	if (nbytes == -1)
	{
		printf("send error\n");
	}
}

void Platform_GM6020::calWheelSpeed(double vx,double vy, double avz){
	vx_wheel = vx - y_wheel * avz;
	vy_wheel = vy + x_wheel * avz;
}

double Platform_GM6020::angleCalculate(double vx,double vy){
	return atan2(vy,vx);
}

