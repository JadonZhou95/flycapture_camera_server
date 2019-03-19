#include <string>

#include "ros/ros.h"
#include "flycapture_camera_server/FlycaptureCam.h"

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

const std::string OPENCV_WINDOW = "flycapture image";

int main(int argc, char** argv)
{
    ros::init(argc, argv, "flycapture_camera_client");
    
    ros::NodeHandle nh;

    ros::ServiceClient client = nh.serviceClient<flycapture_camera_server::FlycaptureCam>("flycapture_cam_service");

    //declear the srv element
    flycapture_camera_server::FlycaptureCam srv;
    srv.request.capture_image = true;

    //call the service
    if (client.call(srv))
    {
        ROS_INFO("FlycaptureCameraClient get feedbacks from the server! Image Resolution: %d x %d", srv.response.image.height, srv.response.image.width);
        
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(srv.response.image, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return -1;
        }
        cv::namedWindow(OPENCV_WINDOW);
        cv::imshow(OPENCV_WINDOW, cv_ptr->image);
        cv::waitKey(0);
        cv::destroyWindow(OPENCV_WINDOW);
    }
    else
    {
        ROS_ERROR("Fail to get the server feedbacks!");
        return -1;
    }

    return 0;
}