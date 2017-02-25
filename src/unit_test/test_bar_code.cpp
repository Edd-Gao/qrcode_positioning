#include <iostream>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <string>
#include <memory>
#include <zbar.h>
#include <opencv2/opencv.hpp>


zbar::ImageScanner zbarScanner;

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
    ROS_ERROR_STREAM("scan consumes:"<< ((ros::Time::now() - timeStamp).toNSec())/1000000);
    //ROS_ERROR("scan consumes: %ld", ((ros::Time::now() - timeStamp).toNSec())/1000000);

    for (zbar::Image::SymbolIterator symbol = zbarFrame.symbol_begin();  symbol != zbarFrame.symbol_end();  ++symbol) 
    {
        ROS_ERROR("symbol:%s", symbol->get_data().c_str());
    }
    //ROS_ERROR_STREAM("decode consumes:"<< ((ros::Time::now() - timeStamp).toNSec())/1000000);
    //ROS_ERROR("decode consumes: %ld", (ros::Time::now() - timeStamp).toNSec()/1000000);


}

int main(int argc, char **argv)
{
    zbarScanner.set_config(zbar::ZBAR_NONE , zbar::ZBAR_CFG_ENABLE, 0);
    zbarScanner.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_X_DENSITY, 1);
    zbarScanner.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_Y_DENSITY, 1);
    zbarScanner.set_config(zbar::ZBAR_CODE39 , zbar::ZBAR_CFG_ENABLE, 1);
    zbarScanner.set_config(zbar::ZBAR_QRCODE , zbar::ZBAR_CFG_ENABLE, 1);
    zbarScanner.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_NUM, 1);

    zbarScanner.enable_cache(false); //Set it so that it will show QR code result even if it was in the last frame
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
