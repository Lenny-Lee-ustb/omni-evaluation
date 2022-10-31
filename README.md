# Omni-directional robot evaluation program
This program list


## General setup
install ros packages:

    sudo apt install ros-noetic-joy ros-noetic-realsense2-camera ros-noetic-plotjuggler-ros

instal other modules:

    sudo apt install expect setserial can-utils

Init submodule and download it:

    git submodule init 
    git submodule update 


## Some tips in debugging process
Get CAN bus load in terminal
    canbusload can0@1000000 can1@1000000 -r -t -b -c

