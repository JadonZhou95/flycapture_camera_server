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

    // set video mode including mode(resolution, pixel format)
    setVideoMode();

    setExposure(settings.exposure, settings.auto_exposure);
    setSharpness(settings.sharpness, settings.auto_sharpness);
    setSaturation(settings.saturation, settings.auto_saturation);
    setShutter(settings.shutter * 1000, settings.auto_shutter);
    setGain(settings.gain, settings.auto_gain);
    setWhiteBalance(settings.auto_white_balance, settings.white_balance_blue, settings.white_balance_red);
    setBrightness(settings.brightness);
    setGamma(settings.gamma);

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

FlycaptureCamServer::~FlycaptureCamServer()
{
    // Turn trigger mode off.
    triggerMode_.onOff = false;
    error_ = cam_.SetTriggerMode(&triggerMode_);
    if (error_ != PGRERROR_OK)
    {
        PrintError(error_);
        exit(-1);
    }

    ROS_INFO("Finished grabbing images!");

    // Stop capturing images
    error_ = cam_.StopCapture();
    if (error_ != PGRERROR_OK)
    {
        PrintError(error_);
        exit(-1);
    }

    // Disconnect the camera
    error_ = cam_.Disconnect();
    if (error_ != PGRERROR_OK)
    {
        PrintError(error_);
        exit(-1);
    }
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

    // metadata_ = convertedImage.GetMetadata();

    // ROS_INFO("Gain: %d, Shutter: %d, Brightness: %d, Exposure: %d, WhiteBalance: %d",
                // getGain(), getShutter(), getBrightness(), getExposure(), getWhiteBalance());

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

    unsigned int regVal = 0;

    error_ = pCam->ReadRegister(k_triggerInq, &regVal);

    if (error_ != PGRERROR_OK)
    {
        PrintError(error_);
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

    error_ = pCam->WriteRegister(k_softwareTrigger, k_fireVal);
    if (error_ != PGRERROR_OK)
    {
        PrintError(error_);
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

void FlycaptureCamServer::setVideoMode()
{   
    // necessary parameter
    Mode fmt7Mode = MODE_1;
    PixelFormat fmt7PixFmt = PIXEL_FORMAT_RGB;
    uint16_t uwidth(0), uheight(0), uoffsetx(0), uoffsety(0);

    bool retVal = setFormat7(fmt7Mode, fmt7PixFmt, uwidth, uheight, uoffsetx, uoffsety);

    //set video mode and frame rate
    VideoMode vMode = VIDEOMODE_FORMAT7;   
    FrameRate frameRate = FRAMERATE_FORMAT7;

    
}

bool FlycaptureCamServer::setProperty(const FlyCapture2::PropertyType &type, const bool &autoSet, double &value)
{
    // return true if we can set values as desired.
    bool retVal = true;

    PropertyInfo pInfo;
    pInfo.type = type;
    error_ = cam_.GetPropertyInfo(&pInfo);
    if (error_ != PGRERROR_OK)
    {
        PrintError(error_);
        exit(-1);
    }

    std::cout << "pInfo state: " << pInfo.present << " " << __LINE__ << std::endl;
    if(pInfo.present)
    {
        Property prop;
        prop.type = type;
        prop.autoManualMode = (autoSet && pInfo.autoSupported);
        prop.absControl = pInfo.absValSupported;
        prop.onOff = pInfo.onOffSupported;

        cout << "Param range: " << pInfo.absMax << " " << pInfo.absMin << " " << value << std::endl;
        if(value < pInfo.absMin)
        {
            value = pInfo.absMin;
            retVal &= false;
        }
        else if(value > pInfo.absMax)
        {
            value = pInfo.absMax;
            retVal &= false;
        }
        prop.absValue = value;
        error_ = cam_.SetProperty(&prop);
        if (error_ != PGRERROR_OK)
        {
            PrintError(error_);
            exit(-1);
        }

        // PointGreyCamera::handleError("PointGreyCamera::setProperty  Failed to set property ", error); /** @todo say which property? */

        // Read back setting to confirm
        error_ = cam_.GetProperty(&prop);
        // PointGreyCamera::handleError("PointGreyCamera::setProperty  Failed to confirm property ", error); /** @todo say which property? */
        if(!prop.autoManualMode)
        {
            value = prop.absValue;
        }
    }
    else     // Not supported
    {
        value = 0.0;
    }
    return retVal;
}

bool FlycaptureCamServer::setFormat7(FlyCapture2::Mode &fmt7Mode, FlyCapture2::PixelFormat &fmt7PixFmt, 
        uint16_t &roi_width, uint16_t &roi_height, uint16_t &roi_offset_x, uint16_t &roi_offset_y)
{
    // return true if we can set values as desired.
    bool retVal = true;

    // Get Format7 information
    Format7Info fmt7Info;
    bool supported;
    fmt7Info.mode = fmt7Mode;
    error_ = cam_.GetFormat7Info(&fmt7Info, &supported);
    if (error_ != PGRERROR_OK)
    {
        PrintError(error_);
        exit(-1);
    }
    if(!supported)
    {
        throw std::runtime_error("FlycaptureCamServer::setFormat7 Format 7 mode not supported on this camera.");
    }

    // Make Format7 Configuration
    Format7ImageSettings fmt7ImageSettings;
    fmt7ImageSettings.mode = fmt7Mode;
    fmt7ImageSettings.pixelFormat = fmt7PixFmt;

    // Check Width
    roi_width = roi_width / fmt7Info.imageHStepSize * fmt7Info.imageHStepSize; // Locks the width into an appropriate multiple using an integer divide
    if(roi_width == 0)
    {
        fmt7ImageSettings.width = fmt7Info.maxWidth;
    }
    else if(roi_width > fmt7Info.maxWidth)
    {
        roi_width = fmt7Info.maxWidth;
        fmt7ImageSettings.width = fmt7Info.maxWidth;
        retVal &= false;
    }
    else
    {
        fmt7ImageSettings.width = roi_width;
    }

    // Check Height
    roi_height = roi_height / fmt7Info.imageVStepSize * fmt7Info.imageVStepSize; // Locks the height into an appropriate multiple using an integer divide
    if(roi_height == 0)
    {
        fmt7ImageSettings.height = fmt7Info.maxHeight;
    }
    else if(roi_height > fmt7Info.maxHeight)
    {
        roi_height = fmt7Info.maxHeight;
        fmt7ImageSettings.height = fmt7Info.maxHeight;
        retVal &= false;
    }
    else
    {
        fmt7ImageSettings.height = roi_height;
    }

    // Check OffsetX
    roi_offset_x = roi_offset_x / fmt7Info.offsetHStepSize * fmt7Info.offsetHStepSize;  // Locks the X offset into an appropriate multiple using an integer divide
    if(roi_offset_x > (fmt7Info.maxWidth - fmt7ImageSettings.width))
    {
        roi_offset_x = fmt7Info.maxWidth - fmt7ImageSettings.width;
        retVal &= false;
    }
    fmt7ImageSettings.offsetX  = roi_offset_x;

    // Check OffsetY
    roi_offset_y = roi_offset_y / fmt7Info.offsetVStepSize * fmt7Info.offsetVStepSize;  // Locks the X offset into an appropriate multiple using an integer divide
    if(roi_offset_y > fmt7Info.maxHeight - fmt7ImageSettings.height)
    {
        roi_offset_y = fmt7Info.maxHeight - fmt7ImageSettings.height;
        retVal &= false;
    }
    fmt7ImageSettings.offsetY  = roi_offset_y;

    // Validate the settings to make sure that they are valid
    Format7PacketInfo fmt7PacketInfo;
    bool valid;
    error_ = cam_.ValidateFormat7Settings(&fmt7ImageSettings, &valid, &fmt7PacketInfo);
    if (error_ != PGRERROR_OK)
    {
        PrintError(error_);
        exit(-1);
    }
    if(!valid)
    {
        throw std::runtime_error("FlycaptureCamServer::setFormat7 Format 7 Settings Not Valid.");
    }

    // Stop the camera to allow settings to change.
    error_ = cam_.SetFormat7Configuration(&fmt7ImageSettings, fmt7PacketInfo.recommendedBytesPerPacket);
    if (error_ != PGRERROR_OK)
    {
        PrintError(error_);
        exit(-1);
    }
    //   PointGreyCamera::handleError("PointGreyCamera::setFormat7 Could not send Format7 configuration to the camera", error);

    // Get camera info to check if camera is running in color or mono mode
    //   CameraInfo cInfo;
    //   error = cam_.GetCameraInfo(&cInfo);
    //   PointGreyCamera::handleError("PointGreyCamera::setFormat7  Failed to get camera info.", error);
    //   isColor_ = cInfo.isColorCamera;

    return retVal;
}

bool FlycaptureCamServer::setWhiteBalance(bool auto_white_balance, uint16_t blue, uint16_t red)
{
    // Get camera info to check if color or black and white chameleon
    CameraInfo cInfo;
    error_ = cam_.GetCameraInfo(&cInfo);
    if (error_ != PGRERROR_OK)
    {
        PrintError(error_);
        exit(-1);
    }

    if(!cInfo.isColorCamera)
    {
        // Not a color camera, does not support auto white balance
        auto_white_balance = false;
        red = 0;
        blue = 0;
        return false;
    }

    unsigned white_balance_addr = 0x80c;
    unsigned enable = 1 << 31;
    unsigned value = 1 << 25;

    if (auto_white_balance) {
        PropertyInfo prop_info;
        prop_info.type = WHITE_BALANCE;
        error_ = cam_.GetPropertyInfo(&prop_info);
        if (error_ != PGRERROR_OK)
        {
            PrintError(error_);
            exit(-1);
        }

        if (!prop_info.autoSupported) {
            // This is typically because a color camera is in mono mode, so we set
            // the red and blue to some reasonable value for later use
            auto_white_balance = false;
            blue = 800;
            red = 550;
            return false;
        }
        // Auto white balance is supported
        error_ = cam_.WriteRegister(white_balance_addr, enable);
        if (error_ != PGRERROR_OK)
        {
            PrintError(error_);
            exit(-1);
        }

        // Auto mode
        value |= 1 << 24;
    } else {
        // Manual mode
        value |= 0 << 24;
    }
    // Blue is bits 8-19 (0 is MSB), red is 20-31.
    value |= blue << 12 | red;
    error_ = cam_.WriteRegister(white_balance_addr, value);
    if (error_ != PGRERROR_OK)
    {
        PrintError(error_);
        exit(-1);
    }
    return true;
}

bool FlycaptureCamServer::setFrameRate(double frame_rate)
{
    bool retVal = setProperty(FRAME_RATE, false, frame_rate);
    return frame_rate;
}

bool FlycaptureCamServer::setExposure(double exposure_value, bool auto_exposure)
{
    bool retVal = setProperty(AUTO_EXPOSURE, auto_exposure, exposure_value);
    return retVal;
}

bool FlycaptureCamServer::setSharpness(double sharpness_value, bool auto_sharpness)
{
    bool retVal = setProperty(SHARPNESS, auto_sharpness, sharpness_value);
    return retVal;
}

bool FlycaptureCamServer::setSaturation(double saturation_value, bool auto_saturation)
{
    bool retVal = setProperty(SATURATION, auto_saturation, saturation_value);
    return retVal;
}

bool FlycaptureCamServer::setShutter(double shutter_value, bool auto_shutter)
{
    bool retVal = setProperty(SHUTTER, auto_shutter, shutter_value);
    return retVal;
}

bool FlycaptureCamServer::setGain(double gain_value, bool auto_gain)
{
    bool retVal = setProperty(GAIN, auto_gain, gain_value);
    return retVal;
}

bool FlycaptureCamServer::setBrightness(double brightness_value)
{
    bool retVal = setProperty(BRIGHTNESS, false, brightness_value);
    return retVal;
}

bool FlycaptureCamServer::setGamma(double gamma_value)
{
    bool retVal = setProperty(GAMMA, false, gamma_value);
    return retVal;
}


uint FlycaptureCamServer::getGain()
{
  return metadata_.embeddedGain >> 20;
}

uint FlycaptureCamServer::getShutter()
{
  return metadata_.embeddedShutter >> 20;
}

uint FlycaptureCamServer::getBrightness()
{
  return metadata_.embeddedTimeStamp >> 20;
}

uint FlycaptureCamServer::getExposure()
{
  return metadata_.embeddedBrightness >> 20;
}

uint FlycaptureCamServer::getWhiteBalance()
{
  return metadata_.embeddedExposure >> 8;
}

uint FlycaptureCamServer::getROIPosition()
{
  return metadata_.embeddedROIPosition >> 24;
}