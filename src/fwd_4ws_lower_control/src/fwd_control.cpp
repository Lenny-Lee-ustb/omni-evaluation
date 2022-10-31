#include"include/fwd_control.h"

#define CAR_LENGTH 0.380 //m
#define CAR_WIDTH 0.320 //m
#define MOTOR_RATIO 19.2 // input/output M3508 P19
#define WHEEL_RADIUS 0.065 // wheel radius of 1/10 car
#define MAX_SPPED 3.4 // 3.4m/s at 420rpm(out axle)
#define MAX_ANGULAR_VLOCITY 2.0 // rad/s

int s_servo, s_motor;


fwd_control::fwd_control(int s_motor_can, int s_servo_can)
{
	ros::NodeHandle n("~");
    n.getParam("speedMax", speedMax);
    
    s_motor = s_motor_can;
    s_servo = s_servo_can;
    moveable_in, direct_in, control_in = 0;
    motorRxCount = 0;
    servoRxCount = 0;
    MotorInfo.polygon.points.resize(4);
    ServoInfo.polygon.points.resize(4);

    servo[0].angleInit = 2.2296;
    servo[1].angleInit = 3.2881;
    servo[2].angleInit = 4.1203;
    servo[3].angleInit = 2.9805;


    sbusSub = n_.subscribe<sbus_serial::Sbus>("/sbus", 10, &fwd_control::sbusCB, this);
    simulinkSub = n_.subscribe<geometry_msgs::Twist>("/cmd_vel", 10, &fwd_control::cmdCB, this);
    motorInfoPub = n_.advertise<geometry_msgs::PolygonStamped>("/M3508_Rx_State", 10);
    servoInfoPub = n_.advertise<geometry_msgs::PolygonStamped>("/GM6020_Rx_State", 10);
    motorTimerTx = n_.createTimer(ros::Duration(1.0/1000.0),
                          &fwd_control::txMotorThread,
                          this); // 1kHz send
    motorTimerRx = n_.createTimer(ros::Duration(1.0/10000.0),
                          &fwd_control::rxMotorThread,
                          this); // 10kHz receive 

    servoTimerTx = n_.createTimer(ros::Duration(1.0/1000.0),
                          &fwd_control::txServoThread,
                          this); // 1kHz send
    servoTimerRx = n_.createTimer(ros::Duration(1.0/10000.0),
                          &fwd_control::rxServoThread,
                          this); // 10kHz receive 
}

fwd_control::~fwd_control()
{
}

void fwd_control::sbusCB(const sbus_serial::Sbus::ConstPtr& sbus)
{

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

void fwd_control::cmdCB(const geometry_msgs::Twist::ConstPtr &cmd_vel){
    cmdUp = *cmd_vel;
};


void fwd_control::CmdMux(){
    if ( (!moveable_in == !control_in) || failsafe == 1 || frame_lost == 1){
        vX_cmd = 0.0;
        vY_cmd = 0.0;
        avZ_cmd = 0.0;
        if (failsafe == 1 || frame_lost == 1){
            ROS_ERROR("RC SIGNAL LOST!!! Check RC status!");
        }
        else{
            ROS_WARN("moveable_in=%d, control_in=%d", !!moveable_in, !!control_in);
        }

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
    }
    vX_cmd = fmin(fmax(vX_cmd,-speedMax),speedMax);
    vY_cmd = fmin(fmax(vY_cmd,-speedMax),speedMax);
    avZ_cmd = fmin(fmax(avZ_cmd,-MAX_ANGULAR_VLOCITY),MAX_ANGULAR_VLOCITY);
}



// INPUT: vx(m/s), vy(m/s), omega(rad/s) 
// OUTPUT: 4wd-4ws robot 
void fwd_control::fwdKinematicCal(double vX, double vY, double avZ){
    motor[0].speedDes = -vX_cmd;
    motor[1].speedDes = -vX_cmd;
    motor[2].speedDes = vX_cmd;
    motor[3].speedDes = vX_cmd;

    // ROS_INFO("FL: %.2f, FR: %.2f, RL: %.2f, RR: %.2f", vFL,vFR,vRL,vRR);

    for (int i = 0; i < 4; i++)
    {
        servo[i].angleDes = avZ_cmd;
    }
    
    for (int i = 0; i < 4; i++)
    {
        motor[i].curTx = motor[i].MotorTune();
        servo[i].volTx = servo[i].MotorPosTune();
        // ROS_INFO("[%d]: %d",i,motor[i].curTx);
    }
};



void fwd_control::rxMotorThread(const ros::TimerEvent &){
    int ID, nbytes;
    struct can_frame frame;
	frame.can_dlc = 8;

    // ros::spinOnce();
    nbytes = read(s_motor, &frame, sizeof(struct can_frame));
    if (nbytes < 0){
        perror("Read Error at motor can\n");
        motorRxCount =-1;
    }
    else if(int(frame.can_id) > 0x200 && int(frame.can_id) < 0x205)
    {
        ID = int(frame.can_id - 0x200)-1;
        motor[ID].angleRx = ((frame.data[0]<<8) + frame.data[1]); 
        motor[ID].velRx = (frame.data[2]<<8) + frame.data[3]; 
        motor[ID].curRx = (frame.data[4]<<8) + frame.data[5];  
        motor[ID].thermalRx = frame.data[6]; // 'C
        motor[ID].speed = WHEEL_RADIUS * (motor[ID].velRx / MOTOR_RATIO) * (2.0 * PI/60.0);
        // motor[ID].tor = K * motor[ID].curRx / 16384.0 * 20.0;
        motor[ID].tor = motor[ID].curRx / 16384.0 * 20.0;


        motor[ID].speedF = motor[ID].LowPassFilter(motor[ID].speedF_Last, motor[ID].speed, motor[ID].speedLast);
        motor[ID].torF = motor[ID].LowPassFilter(motor[ID].torF_Last, motor[ID].tor, motor[ID].torLast);

        MotorTemp.temperature = double(motor[ID].thermalRx);
        motor[ID].speedLast = motor[ID].speed;
        motor[ID].speedF_Last = motor[ID].speedF;
        motor[ID].torLast = motor[ID].tor;
        motor[ID].torF_Last = motor[ID].torF;

        // due to the configuration, it needs change the direction
        if ((ID==1) || (ID==0))
        {
            MotorInfo.polygon.points[ID].x = -motor[ID].speedF;
            MotorInfo.polygon.points[ID].y = -motor[ID].torF; 
            MotorInfo.polygon.points[ID].z = MotorTemp.temperature;
        }
        else if ((ID==2) || (ID==3))
        {
            MotorInfo.polygon.points[ID].x = motor[ID].speedF; // m/s
            MotorInfo.polygon.points[ID].y = motor[ID].torF;  // -16384~0~16384 -> -20-0-20A -> -1.626 - 1.626Nm
            MotorInfo.polygon.points[ID].z = MotorTemp.temperature;  // 'c
        }
    }
    if(motorRxCount%4==0){
        motorInfoPub.publish(MotorInfo);
    }
    motorRxCount++;
};


void fwd_control::txMotorThread(const ros::TimerEvent &)
{
    struct can_frame frame;
	frame.can_id = 0x200;
	frame.can_dlc = 8;

    int nbytes;
    CmdMux(); // choose input source
    fwdKinematicCal(vX_cmd,vY_cmd,avZ_cmd);

    if(ros::ok()){
        for (int j = 0; j < 4; j++) 
        {
            frame.data[2*j] = motor[j].curTx>>8; //控制电流值高 8 位
            frame.data[2*j+1] = motor[j].curTx>>0; //控制电流值低 8 位
        }
        nbytes = write(s_motor, &frame, sizeof(struct can_frame));
        if (nbytes == -1) {
            printf("send error in txMotorThread\n");
        }
    }
    else{
        for (int i = 0; i < 8; i++)
        {
            frame.data[i] = 0;
        }
        nbytes = write(s_motor, &frame, sizeof(struct can_frame));
        printf("volTx[0]: %d", motor[0].curTx);
    }
};


void fwd_control::rxServoThread(const ros::TimerEvent &){
    int ID, nbytes;
    struct can_frame frame;
	frame.can_dlc = 8;

    // ros::spinOnce();
    nbytes = read(s_servo, &frame, sizeof(struct can_frame));
    if (nbytes < 0){
        perror("Read Error at servo can\n");
        motorRxCount =-1;
    }
    else if (int(frame.can_id) > 0x204 && int(frame.can_id) < 0x209)
    {
        ID = int(frame.can_id - 0x204)-1;
        servo[ID].angleRx = ((frame.data[0]<<8) + frame.data[1]); // 0~8191
        servo[ID].velRx = (frame.data[2]<<8) + frame.data[3];  // rpm
        servo[ID].curRx = (frame.data[4]<<8) + frame.data[5];  //
        servo[ID].thermalRx = frame.data[6]; // 'C

        servo[ID].angle = servo[ID].angleRx / 8192.0 * 2 * M_PI; //rad
        servo[ID].angle = servo[ID].AngleRound(servo[ID].angle - servo[ID].angleInit);
        servo[ID].angleVel = servo[ID].velRx * (2.0 * M_PI/60.0); 

        ServoInfo.polygon.points[ID].x = servo[ID].angle;
        ServoInfo.polygon.points[ID].y = servo[ID].angleVel; 
        ServoInfo.polygon.points[ID].z = (float) servo[ID].curRx;
        // ServoInfo.polygon.points[ID].z = ServoTemp.temperature;

    }
    if(servoRxCount%4==0){
        servoInfoPub.publish(ServoInfo);
    }


    servoRxCount++;
}

void fwd_control::txServoThread(const ros::TimerEvent &){
    struct can_frame frame;
	frame.can_id = 0x1FF;
	frame.can_dlc = 8;

    int nbytes;

    if(ros::ok()){
        for (int j = 0; j < 4; j++) 
        {
            frame.data[2*j] = servo[j].volTx>>8; //控制电流值高 8 位
            frame.data[2*j+1] = servo[j].volTx>>0; //控制电流值低 8 位
        }
        nbytes = write(s_servo, &frame, sizeof(struct can_frame));

        if (nbytes == -1) {
            printf("send error in txMotorThread\n");
        }
    }
}

// Begin when Ctrl+C triggered, send motor zero command to stop!
void signalCallback(int signum)
{	
	ROS_WARN("Get Ctrl+c signal to shutdwon the node.");
    
    int nbytes;
    can_frame frame;

	frame.can_id = 0x1FF; // servo
	frame.can_dlc = 8;
    for (int i = 0; i < 8; i++){
        frame.data[i] = 0;
    }
	nbytes = write(s_servo, &frame, sizeof(struct can_frame));
	ros::Duration(0.005).sleep();

    frame.can_id = 0x200; // motor
    for (int i = 0; i < 8; i++){
        frame.data[i] = 0;
    }
	nbytes = write(s_motor, &frame, sizeof(struct can_frame));
    ros::Duration(0.005).sleep();
	ROS_WARN("shutdown!!");
	exit(1);
}

int main(int argc, char** argv) {
    std::string canServoSeries="can0"; // GM6020 ID: 205~208  control_ID: 1FF
    std::string canMotorSeries="can1"; // M3508 ID: 201~204   control_ID: 200

	struct sockaddr_can addr_servo;
	struct ifreq ifr_servo;
	/*建立套接字，设置为原始套接字，原始CAN协议 */
	if ((s_servo = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
		perror("Error while opening socket");
		return -1;
	}
    /*以下是对CAN接口进行初始化，如设置CAN接口名，即当我们用ifconfig命令时显示的名字 */
	strcpy(ifr_servo.ifr_name, canServoSeries.c_str());
	ioctl(s_servo, SIOCGIFINDEX, &ifr_servo);

    /*设置CAN协议 */
	addr_servo.can_family = AF_CAN;
	addr_servo.can_ifindex = ifr_servo.ifr_ifindex;
	printf("%s at index %d\n", ifr_servo.ifr_name, ifr_servo.ifr_ifindex);

	if (bind(s_servo, (struct sockaddr *)&addr_servo, sizeof(addr_servo)) < 0) {
		perror("Error in socket bind");
		return -2;
	}


	struct sockaddr_can addr_motor;
	struct ifreq ifr_motor;
    // CAN bus init
	if ((s_motor = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
		perror("Error while opening socket");
		return -1;
	}
	strcpy(ifr_motor.ifr_name, canMotorSeries.c_str());
	ioctl(s_motor, SIOCGIFINDEX, &ifr_motor);
	addr_motor.can_family = AF_CAN;
	addr_motor.can_ifindex = ifr_motor.ifr_ifindex;
	printf("%s at index %d\n", ifr_motor.ifr_name, ifr_motor.ifr_ifindex);
	if (bind(s_motor, (struct sockaddr *)&addr_motor, sizeof(addr_motor)) < 0) {
		perror("Error in socket bind");
		return -2;
	}

    ros::init(argc,argv,"fwd_control");
    fwd_control control(s_motor, s_servo);

    signal(SIGINT, signalCallback);
    
    ros::spin();
    return 0;
}