#include <iostream>
#include "../library/QRCodeStateEstimator.hpp"
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <string>
#include <memory>
#include <zbar.h>

std::unique_ptr<QRCodeStateEstimator> stateEstimator;

void imageCallback(const sensor_msgs::ImageConstPtr &msg)
{


    ros::Time timeStamp = ros::Time::now();

    
    //Wrap the image data so that it can be used by zbar
    int frameWidth = msg->width;
    int frameHeight = msg->height;
    const uint8_t *rawData = msg->data.data();

// Wrap image data
    zbar::Image zbarFrame(frameWidth, frameHeight, "Y800", rawData, frameWidth * frameHeight);


    if(zbarScanner.scan(zbarFrame) == -1)
    {
        printf("TestScanner error\n");
    }

    ROS_ERROR("scan consumes: %d", (ros::Time::now() - timeStamp).toNSec()/1000000);
    
    for (zbar::Image::SymbolIterator symbol = zbarFrame.symbol_begin();  symbol != zbarFrame.symbol_end();  ++symbol) 
    {
        ROS_ERROR("symbol:%s", symbol->get_data());
    }




}

int main(int argc, char **argv)
{
    ros::init(argc,argv,"qrcode_positioning");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("camera/image_raw", 1, imageCallback);

    if(sub != NULL){
        ROS_INFO("subscribe to camera/image_raw");
    }else{
        ROS_ERROR("failed to subscribe to camera/image_raw");
        exit(-1);
    }

    ros::spin();

}
