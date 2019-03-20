// #include "stdafx.h"

#include <time.h>
#include <unistd.h>

#include <iostream>
#include <sstream>

// Official Flycapture API
#include "FlyCapture2.h"

#include "ros/ros.h"
#include "flycapture_camera_server/FlycaptureCam.h"
#include <sensor_msgs/image_encodings.h> // ROS header for the different supported image encoding types
#include <sensor_msgs/fill_image.h>

using namespace FlyCapture2;
using namespace std;

class FlycaptureCamServer
{
private:
    ros::NodeHandle nh_;
    ros::ServiceServer flycapture_cam_server;

    Error error_;   //error which stop the camera
    Camera cam_;    //cam object

    TriggerMode triggerMode_; // trigger mode

    FC2Config config_;   //camera configuration

    Image image_;

    ImageMetadata metadata_; ///< Metadata from the last image, stores useful information such as timestamp, gain, shutter, brightness, exposure.
    

    //! main call back function for capture the image
    bool getFCImageCallback(flycapture_camera_server::FlycaptureCam::Request &req,
                            flycapture_camera_server::FlycaptureCam::Response &res);

    //! print the build info and flycature lib info
    void PrintBuildInfo();

    //! print out the error type
    void PrintError(Error error) { error.PrintErrorTrace(); }

    //! get the universal guid
    PGRGuid GetCameraGuid();

    //! power on the camera
    void PowerOnCamera();

    //! get the camera info
    CameraInfo GetCameraInfo();

    //! print the camera info
    void PrintCameraInfo(CameraInfo *pCamInfo);

    void SetTriggerMode();

    // Poll to ensure camera is ready
    bool PollForTriggerReady(Camera *pCam);

    bool CheckSoftwareTriggerPresence(Camera *pCam);

    // Fire software trigger
    bool FireSoftwareTrigger(Camera *pCam);

    void convertFcImage2RosMsg(const Image& fcImage, sensor_msgs::Image& rRosImage);


public:
    FlycaptureCamServer(/* args */);
    ~FlycaptureCamServer();
};