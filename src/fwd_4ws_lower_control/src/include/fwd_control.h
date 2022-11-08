#include <ros/ros.h>

#include "std_msgs/String.h"
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PolygonStamped.h>
#include <sensor_msgs/Temperature.h>

# include"Platform_M3508.hpp"
# include"Platform_GM6020.hpp"
# include "sbus_serial_driver.h"
# include "Sbus.h"

#define PI 3.14159265

class fwd_control
{
private:
    ros::NodeHandle n_;
    ros::Publisher motorInfoPub, servoInfoPub;
    ros::Subscriber simulinkSub, sbusSub;
    ros::Timer motorTimerTx,motorTimerRx;
    ros::Timer servoTimerTx,servoTimerRx;

    geometry_msgs::PolygonStamped MotorInfo, ServoInfo;
    geometry_msgs::Twist cmdUp, cmdSbus;
    sensor_msgs::Temperature MotorTemp, ServoTemp;

    double vX_cmd, vY_cmd, avZ_cmd;
    long int motorRxCount, servoRxCount, cmdMuxCount;
    int s_servo, s_motor; //socketcan handle for two line
    int  moveable_in, direct_in, control_in;
    bool failsafe, frame_lost;

    void sbusCB(const sbus_serial::Sbus::ConstPtr &sbus);
    void cmdCB(const geometry_msgs::Twist::ConstPtr &cmd_vel);
    void CmdMux();
    void fwdKinematicCal(const double vX,const double vY,const double avZ);

public:
    fwd_control(int s_motor_can, int s_servo_can);
    ~fwd_control();

    void txMotorThread(const ros::TimerEvent &);
    void rxMotorThread(const ros::TimerEvent &);
    void txServoThread(const ros::TimerEvent &);
    void rxServoThread(const ros::TimerEvent &);

    double speedMax, angularSpeedMax, minSpeedThreadhold;

    Platform_M3508 motor[4];
    Platform_GM6020 servo[4];
};

