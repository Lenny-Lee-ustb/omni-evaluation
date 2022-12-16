#!/bin/bash
source /opt/ros/noetic/setup.bash
source /home/car/omni-evaluation/devel/setup.bash
rosbag record -o square-25-1 /odometry/filtered /cmd_state /cmd_vel /M3508_Rx_State /GM6020_Rx_State /power_comsume