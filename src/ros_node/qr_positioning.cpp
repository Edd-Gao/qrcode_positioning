#include <iostream>
#include "../library/QRCodeStateEstimator.hpp"
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <camera_calibration_parsers/parse.h>

void imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
    Mat frame = cv_bridge::toCvShare(msg,"bgr8")->image;

}

int main(int argc, char **argv)
{

    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("~/camera/image_raw", 1, imageCallback);

    ros::spin();

}