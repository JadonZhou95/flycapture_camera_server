#include <iostream>
#include <ros/ros.h>

class FlirSetting
{
    private:
        ros::NodeHandle nh;

    public:
        
        bool auto_exposure;
        double exposure;
        
        bool auto_sharpness;
        double sharpness;
        
        bool auto_saturation;
        double saturation;
        
        bool auto_shutter;
        double shutter;
        
        bool auto_gain;
        double gain;
        
        bool auto_white_balance;
        int white_balance_blue;
        int white_balance_red;
        
        double gamma;

        double brightness;

        FlirSetting();
        ~FlirSetting(){}

        /**
         * @brief Set the camera param
         * 
         * The funtion is to set all the camera parameter by reading the ros parameter server.
         * All the parameters involoved are launched from the file in the 'launch' folder 
        */
        void reset();
};