// #include "stdafx.h"

#include <time.h>
#include <unistd.h>

#include <iostream>
#include <sstream>

#include "FlyCapture2.h" //official api

#include "ros/ros.h"
#include "flycapture_camera_server/FlycaptureCam.h" //ros msg

#include "FlycaptureCamServer.h" //main class


int main(int argc, char** argv)
{
    ros::init(argc, argv, "flycapture_cam_server");

    FlycaptureCamServer camera_server;

    return 0;
}

