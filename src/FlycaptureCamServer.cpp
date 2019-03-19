#include "FlycaptureCamServer.h"

FlycaptureCamServer::FlycaptureCamServer(/* args */)
{
    // get the info of the build and flycapture
    PrintBuildInfo();

    // get the camera guid
    PGRGuid guid = GetCameraGuid();

    // Connect to a camera
    error_ = cam_.Connect(&guid);
    if (error_ != PGRERROR_OK)
    {
        PrintError(error_);
        exit(-1);
    }

    // power on the camera
    PowerOnCamera();

    // get and print camera info
    CameraInfo camInfo = GetCameraInfo();
    PrintCameraInfo(&camInfo);

    //set the trigger mode
    SetTriggerMode();

    // Poll to ensure camera is ready
    bool retVal = PollForTriggerReady(&cam_);
    if (!retVal)
    {
        ROS_ERROR("Error polling for trigger ready!");
        exit(-1);
    }

    // Get the camera configuration
    error_ = cam_.GetConfiguration(&config_);
    if (error_ != PGRERROR_OK)
    {
        PrintError(error_);
        exit(-1);
    }

    // Set the grab timeout to 5 seconds
    config_.grabTimeout = 5000;

    // Set the camera configuration
    error_ = cam_.SetConfiguration(&config_);
    if (error_ != PGRERROR_OK)
    {
        PrintError(error_);
        exit(-1);
    }

    // Camera is ready, start capturing images
    error_ = cam_.StartCapture();
    if (error_ != PGRERROR_OK)
    {
        PrintError(error_);
        exit(-1);
    }

    if (!CheckSoftwareTriggerPresence(&cam_))
    {
        ROS_ERROR("SOFT_ASYNC_TRIGGER not implemented on this camera! Stopping "
                "application");
        exit(-1);
    }

    flycapture_cam_server = nh_.advertiseService<flycapture_camera_server::FlycaptureCam::Request, flycapture_camera_server::FlycaptureCam::Response>
                                                ("flycapture_cam_service", boost::bind(&FlycaptureCamServer::getFCImageCallback, this, _1, _2));

    ROS_INFO("Ready to capture the flycapture image!");
    ros::spin();
}

bool FlycaptureCamServer::getFCImageCallback(flycapture_camera_server::FlycaptureCam::Request &req,
                                        flycapture_camera_server::FlycaptureCam::Response &res)
{
    ROS_INFO("FlycaptureCamServer gets a client call!");

    // Check that the trigger is ready
    PollForTriggerReady(&cam_);

    // fire the trigger
    bool retVal = FireSoftwareTrigger(&cam_);
    if (!retVal)
    {
        ROS_ERROR("Error firing software trigger");
        return false;
    }

    // Grab image
    error_ = cam_.RetrieveBuffer(&image_);
    if (error_ != PGRERROR_OK)
    {
        PrintError(error_);
        return false;
    }

    // Get the raw image dimensions
    PixelFormat pixFormat;
    unsigned int rows, cols, stride;
    image_.GetDimensions(&rows, &cols, &stride, &pixFormat);

    // Create a converted image
    Image convertedImage;

    // Convert the raw image
    error_ = image_.Convert(PIXEL_FORMAT_RGB8, &convertedImage);
    if (error_ != PGRERROR_OK)
    {
        PrintError(error_);
        return false;
    }

    convertFcImage2RosMsg(convertedImage, res.image);

    metadata_ = convertedImage.GetMetadata();

    res.success = true;

    return true;
}

void FlycaptureCamServer::PrintBuildInfo()
{
    FC2Version fc2Version;
    Utilities::GetLibraryVersion(&fc2Version);

    ostringstream version;
    version << "FlyCapture2 library version: " << fc2Version.major << "."
            << fc2Version.minor << "." << fc2Version.type << "."
            << fc2Version.build;
    cout << version.str() << endl;

    ostringstream timeStamp;
    timeStamp << "Application build date: " << __DATE__ << " " << __TIME__;
    cout << timeStamp.str() << endl << endl;
}

PGRGuid FlycaptureCamServer::GetCameraGuid()
{
    // check the connection of the camera
    BusManager busMgr;
    unsigned int numCameras;
    error_ = busMgr.GetNumOfCameras(&numCameras);

    if (error_ != PGRERROR_OK)
    {
        PrintError(error_);
        ROS_ERROR("ERROR occur when counting the camera number");
        exit(-1);
    }
    else if (numCameras < 1)
    {
        ROS_ERROR("Insufficient number of cameras... exiting");
        exit(-1);
    }
    else
        ROS_INFO("Number of cameras detected: %d", numCameras);


    PGRGuid guid;
    error_ = busMgr.GetCameraFromIndex(0, &guid);
    if (error_ != PGRERROR_OK)
    {
        PrintError(error_);
        exit(-1);
    }

    return guid;
}

void FlycaptureCamServer::PowerOnCamera()
{
    // Power on the camera
    const unsigned int k_cameraPower = 0x610;
    const unsigned int k_powerVal = 0x80000000;
    error_ = cam_.WriteRegister(k_cameraPower, k_powerVal);
    if (error_ != PGRERROR_OK)
    {
        PrintError(error_);
        exit(-1);
    }

    const unsigned int millisecondsToSleep = 100;
    unsigned int regVal = 0;
    unsigned int retries = 10;

    // Wait for camera to complete power-up
    do
    {
        struct timespec nsDelay;
        nsDelay.tv_sec = 0;
        nsDelay.tv_nsec = (long)millisecondsToSleep * 1000000L;
        nanosleep(&nsDelay, NULL);

        error_ = cam_.ReadRegister(k_cameraPower, &regVal);
        if (error_ == PGRERROR_TIMEOUT)
        {
            // ignore timeout errors, camera may not be responding to
            // register reads during power-up
        }
        else if (error_ != PGRERROR_OK)
        {
            PrintError(error_);
            exit(-1);
        }

        retries--;
    } while ((regVal & k_powerVal) == 0 && retries > 0);

    // Check for timeout errors after retrying
    if (error_ == PGRERROR_TIMEOUT)
    {
        PrintError(error_);
        exit(-1);
    }
}

CameraInfo FlycaptureCamServer::GetCameraInfo()
{
    // Get the camera information
    CameraInfo camInfo;
    error_ = cam_.GetCameraInfo(&camInfo);
    if (error_ != PGRERROR_OK)
    {
        PrintError(error_);
        exit(-1);
    }

    return camInfo;
}

void FlycaptureCamServer::PrintCameraInfo(CameraInfo *pCamInfo)
{
    cout << endl;
    cout << "*** CAMERA INFORMATION ***" << endl;
    cout << "Serial number - " << pCamInfo->serialNumber << endl;
    cout << "Camera model - " << pCamInfo->modelName << endl;
    cout << "Camera vendor - " << pCamInfo->vendorName << endl;
    cout << "Sensor - " << pCamInfo->sensorInfo << endl;
    cout << "Resolution - " << pCamInfo->sensorResolution << endl;
    cout << "Firmware version - " << pCamInfo->firmwareVersion << endl;
    cout << "Firmware build time - " << pCamInfo->firmwareBuildTime << endl
         << endl;
}

void FlycaptureCamServer::SetTriggerMode()
{
    // Get current trigger settings
    error_ = cam_.GetTriggerMode(&triggerMode_);
    if (error_ != PGRERROR_OK)
    {
        PrintError(error_);
        exit(-1);
    }

    // Set camera to trigger mode 0
    triggerMode_.onOff = true;
    triggerMode_.mode = 0;
    triggerMode_.parameter = 0;

    // A source of 7 means software trigger
    triggerMode_.source = 7;

    error_ = cam_.SetTriggerMode(&triggerMode_);
    if (error_ != PGRERROR_OK)
    {
        PrintError(error_);
        exit(-1);
    }
}

bool FlycaptureCamServer::PollForTriggerReady(Camera *pCam)
{
    const unsigned int k_softwareTrigger = 0x62C;
    unsigned int regVal = 0;

    do
    {
        error_ = pCam->ReadRegister(k_softwareTrigger, &regVal);
        if (error_ != PGRERROR_OK)
        {
            PrintError(error_);
            return false;
        }

    } while ((regVal >> 31) != 0);

    return true;
}

bool FlycaptureCamServer::CheckSoftwareTriggerPresence(Camera *pCam)
{
    const unsigned int k_triggerInq = 0x530;

    Error error;
    unsigned int regVal = 0;

    error = pCam->ReadRegister(k_triggerInq, &regVal);

    if (error != PGRERROR_OK)
    {
        PrintError(error);
        return false;
    }

    if ((regVal & 0x10000) != 0x10000)
    {
        return false;
    }

    return true;
}

bool FlycaptureCamServer::FireSoftwareTrigger(Camera *pCam)
{
    const unsigned int k_softwareTrigger = 0x62C;
    const unsigned int k_fireVal = 0x80000000;
    Error error;

    error = pCam->WriteRegister(k_softwareTrigger, k_fireVal);
    if (error != PGRERROR_OK)
    {
        PrintError(error);
        return false;
    }

    return true;   
}

void FlycaptureCamServer::convertFcImage2RosMsg(const Image& fcImage, sensor_msgs::Image& rRosImage)
{

    // Set header timestamp as embedded for now
    TimeStamp embeddedTime = fcImage.GetTimeStamp();
    rRosImage.header.stamp.sec = embeddedTime.seconds;
    rRosImage.header.stamp.nsec = 1000 * embeddedTime.microSeconds;

    // define the image encoding
    std::string imageEncoding = sensor_msgs::image_encodings::RGB8;

    // complete the image conversion
    sensor_msgs::fillImage(rRosImage, imageEncoding, fcImage.GetRows(), fcImage.GetCols(), fcImage.GetStride(), fcImage.GetData());
}