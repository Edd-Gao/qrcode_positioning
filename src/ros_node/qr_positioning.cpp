#include <iostream>
#include "../library/QRCodeStateEstimator.hpp"
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <camera_calibration_parsers/parse.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <ecl/command_line.hpp>
#include <string>
#include <opencv2/opencv.hpp>
#include <memory>

std::unique_ptr<QRCodeStateEstimator> stateEstimator;

void imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
    //convert the new frame to cv format
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

    //Initialize some variables we are going to use while processing frames
    cv::Mat cameraPoseBuffer;
    std::string QRCodeIdentifierBuffer;
    double QRCodeDimensionBuffer;
    bool thereIsANewFrame = false;

    thereIsANewFrame = stateEstimator->estimateStateFromBGRFrame(cv_ptr->image, cameraPoseBuffer, QRCodeIdentifierBuffer, QRCodeDimensionBuffer);

    //Print out values of camera's pose matrix
    if(thereIsANewFrame){
        std::cout<<cameraPoseBuffer<<std::endl;
    }

}

int main(int argc, char **argv)
{

    //parse command line arguments
    ecl::CmdLine cmd("qrcode positioning node.");
    ecl::ValueArg<std::string> pathArg("f","file","calibration file to read",true,"~/.calibration/default.ini","string");
    cmd.add(pathArg);
    cmd.parse(argc,argv);
    std::string filePath = pathArg.getValue();

    //read camera calibration info
    std::string camera_name;
    sensor_msgs::CameraInfo cam_info;
    camera_calibration_parsers::readCalibration(filePath, camera_name, cam_info);

    //Define camera matrix according to webcam calibration (from OpenCV camera calibration file)
    cv::Mat_<double> cameraMatrix(3,3);
    cameraMatrix.at<double>(0,0) = cam_info.K[0];
    cameraMatrix.at<double>(0,1) = cam_info.K[1];
    cameraMatrix.at<double>(0,2) = cam_info.K[2];
    cameraMatrix.at<double>(1,0) = cam_info.K[3];
    cameraMatrix.at<double>(1,1) = cam_info.K[4];
    cameraMatrix.at<double>(1,2) = cam_info.K[5];
    cameraMatrix.at<double>(2,0) = cam_info.K[6];
    cameraMatrix.at<double>(2,1) = cam_info.K[7];
    cameraMatrix.at<double>(2,2) = cam_info.K[8];

    //Define distortion parameters according to webcam calibration (from OpenCV camera calibration file)
    cv::Mat_<double> distortionParameters(1,5);
    distortionParameters.at<double>(0, 0) = cam_info.D[0];
    distortionParameters.at<double>(0, 1) = cam_info.D[1];
    distortionParameters.at<double>(0, 2) = cam_info.D[2];
    distortionParameters.at<double>(0, 3) = cam_info.D[3];
    distortionParameters.at<double>(0, 4) = cam_info.D[4];

    //Initialize the state estimator, while wrapping any exceptions so we know where it came from
    stateEstimator.reset(new QRCodeStateEstimator(cam_info.width, cam_info.height, cameraMatrix,distortionParameters));

    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("~/camera/image_raw", 1, imageCallback);

    ros::spin();

}