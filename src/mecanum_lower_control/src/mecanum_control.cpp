# include"include/Platform_M3508.hpp"
#include "include/sbus_serial_driver.h"
#include "include/Sbus.h"

#define CAR_LENGTH 0.370 //m
#define CAR_WIDTH 0.315 //m
#define MOTOR_RATIO 19.2 // input/output M3508 P19
#define WHEEL_RADIUS 0.076 // wheel radius of 1/10 car
#define MAX_SPPED 3.4 // 3.4m/s at 420rpm(out axle)
#define MAX_ANGULAR_VLOCITY 2.0 // rad/s

ros::Publisher motorInfoPub;
ros::Subscriber simulinkSub, sbusSub;
geometry_msgs::PolygonStamped MotorInfo;
geometry_msgs::Twist cmdUp, cmdSbus;
sensor_msgs::Temperature MotorTemp;

double temp, tempLast, tempAlpha = 0.05;
double speedMax = 0.0, speedMin = -0.0;
double frictionTor = 0.10;
double vX_cmd, vY_cmd, avZ_cmd;

int  moveable_in, direct_in, control_in = 0;
bool failsafe, frame_lost = 0;

Motor3508 motor[4];


void sbusCB(const sbus_serial::Sbus::ConstPtr& sbus){

    int longSbusIn, lateralSbusIn, steerSbusIn = 0; // from upper controller   

    longSbusIn = sbus->mappedChannels[1]-500;
    lateralSbusIn = sbus->mappedChannels[3]-500;
    steerSbusIn = sbus->mappedChannels[0] - 500;
    // remap steer angle to symatric[-500, 500]
    moveable_in = sbus->mappedChannels[6];
    direct_in = sbus->mappedChannels[7];
    control_in = sbus->mappedChannels[9];
    failsafe = sbus->failsafe;
    frame_lost = sbus->frame_lost;
    // signal inout and store

    cmdSbus.linear.x = double(longSbusIn)/500.0 * speedMax;
    cmdSbus.linear.y = double(lateralSbusIn)/500.0 * speedMax;
    cmdSbus.angular.z = double(steerSbusIn) / 500 * MAX_ANGULAR_VLOCITY;
};

void cmdCB(const geometry_msgs::Twist::ConstPtr &cmd_vel){
    cmdUp = *cmd_vel;
};

void CmdMux(){

    if ( (!moveable_in == !control_in) || failsafe == 1 || frame_lost == 1){
        vX_cmd = 0.0;
        vY_cmd = 0.0;
        avZ_cmd = 0.0;
        if (failsafe == 1 || frame_lost == 1){
            ROS_ERROR("RC SIGNAL LOST!!! Check RC status!");
        }
        else{
            // ROS_ERROR("RC NOT enable, please toggle SWA to down!!");
            ROS_WARN("moveable_in=%d, control_in=%d", !!moveable_in, !!control_in);
        }
        // ROS_WARN("input speed is: %lf, %d", vt_cmd, speedSbusIn);
        // ROS_WARN("input steer is: %lf, %d", delta_cmd, steerSbusIn);
    }
    else{
        if (!moveable_in == false)
        {
            vX_cmd = cmdSbus.linear.x;
            vY_cmd = cmdSbus.linear.y;
            avZ_cmd = cmdSbus.angular.z;
        }
        if (!control_in == false)
        {
            vX_cmd = cmdUp.linear.x;
            vY_cmd = cmdUp.linear.y;
            avZ_cmd = cmdUp.angular.z;
        }
        
        // ROS_INFO("input speed is: %lf, %d", vt_cmd, speedSbusIn);
        // ROS_INFO("input steer is: %lf, %d", delta_cmd, steerSbusIn);
    }
}

// INPUT: vx(m/s), vy(m/s), omega(rad/s) 
// OUTPUT: mecanum wheel robot(X-configuration)
void mecanumKinectCal(double vX, double vY, double avZ){
    double R, vFL, vFR, vRL, vRR; // variable for ackermann

    // ROS_INFO("R: %.2f, FL: %.2f, FR: %.2f, ", R, deltaFL, deltaFR);
    // ROS_INFO("FL-int: %d, FR-int: %d, ", servo[0].posTx, servo[1].posTx);
    motor[0].speedDes = vX - vY + avZ;
    motor[1].speedDes = -vX - vY +avZ;
    motor[2].speedDes = -vX + vY +avZ;
    motor[3].speedDes = vX + vY +avZ;
    motor[0].torDes = motor[0].torConst;
    motor[1].torDes = -motor[1].torConst;
    motor[2].torDes = -motor[2].torConst;
    motor[3].torDes = motor[3].torConst;
    // ROS_INFO("FL: %.2f, FR: %.2f, RL: %.2f, RR: %.2f", vFL,vFR,vRL,vRR);

    for (int i = 0; i < 4; i++)
    {
        motor[i].curTx = MotorTune(motor[i]);
        // ROS_INFO("[%d]: %d",i,motor[i].curTx);
    }
};


void rxMotorThread(int s){
    int ID, nbytes;
    struct can_frame frame;
	frame.can_dlc = 8;

    for (int i = 0;; i++)
    {
		ros::spinOnce();
        nbytes = read(s, &frame, sizeof(struct can_frame));
        if (nbytes < 0){
            perror("Read Error at low can");
            break;
        }
        else if(int(frame.can_id) > 0x200)
        {
            ID = int(frame.can_id - 0x200)-1;
            motor[ID].angleRx = ((frame.data[0]<<8) + frame.data[1]); 
            motor[ID].velRx = (frame.data[2]<<8) + frame.data[3]; 
            motor[ID].curRx = (frame.data[4]<<8) + frame.data[5];  
            motor[ID].thermalRx = frame.data[6]; // 'C
            motor[ID].speed = WHEEL_RADIUS * (motor[ID].velRx / MOTOR_RATIO) * (2.0 * PI/60.0);
            // motor[ID].tor = K * motor[ID].curRx / 16384.0 * 20.0;
            motor[ID].tor = motor[ID].curRx / 16384.0 * 20.0;


            motor[ID].speedF = LowPassFilter(motor[ID], motor[ID].speedF_Last, motor[ID].speed, motor[ID].speedLast);
            motor[ID].torF = LowPassFilter(motor[ID], motor[ID].torF_Last, motor[ID].tor, motor[ID].torLast);

            MotorTemp.temperature = double(motor[ID].thermalRx) *(1-tempAlpha) + tempLast* tempAlpha ;
            tempLast = MotorTemp.temperature;
            motor[ID].speedLast = motor[ID].speed;
            motor[ID].speedF_Last = motor[ID].speedF;
            motor[ID].torLast = motor[ID].tor;
            motor[ID].torF_Last = motor[ID].torF;

            // due to the configuration, it needs change the direction
            if ((ID==1) || (ID==2))
            {
                MotorInfo.polygon.points[ID].x = -motor[ID].speedF;
                MotorInfo.polygon.points[ID].y = -motor[ID].torF; 
                MotorInfo.polygon.points[ID].z = MotorTemp.temperature;
            }
            else if ((ID==0) || (ID==3))
            {
                MotorInfo.polygon.points[ID].x = motor[ID].speedF; // m/s
                MotorInfo.polygon.points[ID].y = motor[ID].torF;  // -16384~0~16384 -> -20-0-20A -> -1.626 - 1.626Nm
                MotorInfo.polygon.points[ID].z = MotorTemp.temperature;  // 'c
            }
        }
        if(i%4==0){
        motorInfoPub.publish(MotorInfo);
        }
		std::this_thread::sleep_for(std::chrono::nanoseconds(10000));
    }
};


void txMotorThread(int s)
{
    struct can_frame frame;
	frame.can_id = 0x200;
	frame.can_dlc = 8;

    for (int i = 0;; i++) 
	{
        int nbytes;
        CmdMux(); // choose input source
        vX_cmd = fmin(fmax(vX_cmd,-speedMax),speedMax);
        vY_cmd = fmin(fmax(vY_cmd,-speedMax),speedMax);
        avZ_cmd = fmin(fmax(avZ_cmd,-MAX_ANGULAR_VLOCITY),MAX_ANGULAR_VLOCITY);

        mecanumKinectCal(vX_cmd,vY_cmd,avZ_cmd);

        if(stopFlag == 0){
            for (int j = 0; j < 4; j++) 
            {
                frame.data[2*j] = motor[j].curTx>>8; //控制电流值高 8 位
                frame.data[2*j+1] = motor[j].curTx>>0; //控制电流值低 8 位
            }
            nbytes = write(s, &frame, sizeof(struct can_frame));
            if (nbytes == -1) {
                printf("send error\n");
            }
        }
        else if (stopFlag == 1){
            for (int i = 0; i < 8; i++)
            {
                frame.data[i] = 0;
            }
            nbytes = write(s, &frame, sizeof(struct can_frame));
        }
        // if (i%100==0)
        // {
        //     ROS_INFO("[0]:%d,[1]:%d,[2]:%d,[3]:%d",motor[0].curTx, motor[1].curTx, motor[2].curTx, motor[3].curTx);
        // }
        
		std::this_thread::sleep_for(std::chrono::nanoseconds(1000000)); 
    }
    
};

int main(int argc, char** argv) {
	int s;
	struct sockaddr_can addr;
	struct ifreq ifr;
    std::string canSeries="can0";

	ros::init(argc,argv,"Mecanum_control_node");
	ros::NodeHandle n("~");
	ros::Rate loop_rate(100);
    sbusSub = n.subscribe<sbus_serial::Sbus>("/sbus", 10, sbusCB);
    simulinkSub = n.subscribe<geometry_msgs::Twist>("/cmd_vel", 10, cmdCB);
    motorInfoPub = n.advertise<geometry_msgs::PolygonStamped>("/M3508_Rx_State", 10);
    MotorInfo.polygon.points.resize(4);
    
    signal(SIGINT, signalCallback);

    // get param from ros 
    n.getParam("canSeries", canSeries);
    n.getParam("speedMax",speedMax);

    // CAN bus init
	if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
		perror("Error while opening socket");
		return -1;
	}
	strcpy(ifr.ifr_name, canSeries.c_str());
	ioctl(s, SIOCGIFINDEX, &ifr);
	addr.can_family = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;
	printf("%s at index %d\n", ifr.ifr_name, ifr.ifr_ifindex);
	if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
		perror("Error in socket bind");
		return -2;
	}

    sleep(1);
	std::thread canTx(txMotorThread, s);
	std::thread canRx(rxMotorThread, s);

	while (ros::ok())
    {
		ros::spinOnce();
		loop_rate.sleep();
	}

    return 0;
}