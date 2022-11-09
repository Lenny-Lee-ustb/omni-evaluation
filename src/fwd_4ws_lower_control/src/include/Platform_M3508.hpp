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

#define M3508_T_MAX 7.4
#define M3508_T_MIN -7.4

class Platform_M3508
{
private:
	double D_speed, I_speed, K;
	double a_1, b_0, b_1;
	double getSignNum(double x);
public:
	Platform_M3508();
	~Platform_M3508();

	int16_t curRx, velRx, angleRx, curTx;
    int8_t thermalRx; // Monitor thermal

	double tor, torLast, torF, torF_Last, torDes, torConst; // tor = K*cur
    double speed, speedLast, speedF, speedF_Last, speedDes, speedErr; // m/s

	int16_t MotorTune();
	double LowPassFilter(double yLast, double x, double xLast);
	void SendZero(int s);
};

Platform_M3508::Platform_M3508()
{
	D_speed = 1.0;
	I_speed = 0.0;
	torDes = 0.15;
	a_1 = 0.828;
	b_0 = 0.086;
	b_1 = 0.086;
	K = 0.37;  // M3508 19:1 constant N'm/A
}

Platform_M3508::~Platform_M3508()
{
}

int16_t Platform_M3508::MotorTune(){
	// This method is borrowed from MIT Cheetah, which is used in Tmotor AK series. 
	double T_ref;
	int16_t iqref;

	// T_ref = Kd * (speedDes-speed) + torDes;
	T_ref = D_speed * (speedDes - speed);
	T_ref = T_ref + getSignNum(T_ref) * torDes;
	T_ref = std::min(std::max(T_ref, M3508_T_MIN), M3508_T_MAX);
	iqref = (int16_t) round(T_ref/K*(16384.0/20.0));
	iqref = std::min(std::max(iqref, (int16_t) -16384), (int16_t) 16384);
	return iqref;
}

double Platform_M3508::LowPassFilter(double yLast, double x, double xLast){
	// low-pass filter for motor's sensor, ylast is the last estimate value, x is observed value
	double f;
	// y_{n} = a_1 * y_{n-1} + b_0 * x_{n} + b_1 * x_{n-1}
	f = a_1 * yLast + b_0 * x + b_1 * xLast;
	return f;
}


double Platform_M3508::getSignNum(double x){
	if (x > 0) return 1;
	if (x < 0) return -1;
	return 0;
}

void Platform_M3508::SendZero(int s)
{
	int nbytes;
    can_frame frame;
	frame.can_id = 0x200;
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