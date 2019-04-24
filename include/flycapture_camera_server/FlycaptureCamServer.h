#include <time.h>
#include <unistd.h>
#include <iostream>
#include <sstream>

// Official Flycapture API
#include "FlyCapture2.h"

// ros header
#include "ros/ros.h"
#include <sensor_msgs/image_encodings.h> // ROS header for the different supported image encoding types
#include <sensor_msgs/fill_image.h>

#include "flycapture_camera_server/FlycaptureCam.h"
#include "FlirSetting.h"

using namespace FlyCapture2;
using namespace std;

class FlycaptureCamServer
{
private:
    ros::NodeHandle nh_;
    ros::ServiceServer flycapture_cam_server;

    // parameter settings of the camera
    FlirSetting settings;

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

    // set the camera property
    bool setProperty(const FlyCapture2::PropertyType &type, const bool &autoSet, double &value);

    /*!
    * \brief Set the camera parameter including Video Mode and frames per second
    * 
    * This function is to set the camera paramter including Video Modes and Frames per Second 
    * Refering to the user manual, video modes seems to control the resolution
    */
    void setVideoMode();

    bool setFormat7(FlyCapture2::Mode &fmt7Mode, FlyCapture2::PixelFormat &fmt7PixFmt, 
        uint16_t &roi_width, uint16_t &roi_height, uint16_t &roi_offset_x, uint16_t &roi_offset_y);

    bool setWhiteBalance(bool auto_white_balance, uint16_t blue, uint16_t red);

    /*!
    * \brief Set the frame rate of camera
    * 
    * This function is to set the camera frame rate (fps).
    * \param frame_rate - the camera frame rate to be set
    * 
    * \return true if we can set the value as desired
    */
    bool setFrameRate(double frame_rate);

    /*!
    * \brief Set the exposure parameter of camera
    * 
    *  This function is to set the camera exposure
    * \param auto_exposure - ture to enable the auto exposure; flase to disable the auto exposure
    * \param exposure_value - exposure value to be used if the auto-exposure is disabled
    * 
    * \return true if we can set the value as desired
    * */
    bool setExposure(double exposure_value, bool auto_exposure = true);

    bool setSharpness(double sharpness_value, bool auto_sharpness = true);

    bool setSaturation(double saturation_value, bool auto_saturation = true);

    // millisecond
    bool setShutter(double shutter_value, bool auto_shutter = true);

    bool setGain(double gain_value, bool auto_gain = true);

    bool setBrightness(double brightness_value);

    bool setGamma(double gamma_value);



public:
    FlycaptureCamServer(/* args */);
    ~FlycaptureCamServer();

    uint getGain();

    uint getShutter();

    uint getBrightness();

    uint getExposure();

    uint getWhiteBalance();

    uint getROIPosition();
};