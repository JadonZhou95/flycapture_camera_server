#include "FlirSetting.h"

FlirSetting::FlirSetting()
{
    reset();
}

void FlirSetting::reset()
{   
    nh.param<bool>("/flir_camera/auto_exposure", auto_exposure, false);
    nh.param<double>("/flir_camera/exposure", exposure, 1.35);
    
    nh.param<bool>("/flir_camera/auto_sharpness", auto_sharpness, false);
    nh.param<double>("/flir_camera/sharpness", sharpness, 1024.0);

    nh.param<bool>("/flir_camera/auto_saturation", auto_saturation, false);
    nh.param<double>("/flir_camera/saturation", saturation, 100.0);

    nh.param<bool>("/flir_camera/auto_shutter", auto_shutter, false);
    nh.param<double>("/flir_camera/shutter", shutter, 0.01);

    nh.param<bool>("/flir_camera/auto_gain", auto_gain, false);
    nh.param<double>("/flir_camera/gain", gain, 0.0);

    nh.param<bool>("/flir_camera/auto_white_balance", auto_white_balance, false);
    nh.param<int>("/flir_camera/white_balance_blue", white_balance_blue, 750);
    nh.param<int>("/flir_camera/white_balance_red", white_balance_red, 550);

    nh.param<double>("/flir_camera/gamma", gamma, 1.0);

    nh.param<double>("/flir_camera/brightness", brightness, 0.0);
}