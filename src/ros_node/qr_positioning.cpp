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
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <boost/algorithm/string.hpp>

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
        //initialize tf2 broadcastors
        tf2_ros::TransformBroadcaster marker_br;    //broadcast the marker to marker to marker basis transform
        tf2_ros::TransformBroadcaster drone_cam_br; //broadcast the drone camera to marker transform

        //ROS_INFO("QRCodeIdentifierBuffer:%s",QRCodeIdentifierBuffer);
        std::vector<std::string> coordinate_str_vec;
        boost::split(coordinate_str_vec,QRCodeIdentifierBuffer,boost::is_any_of(","));

        if(coordinate_str_vec.size() != 2){
            ROS_INFO("coordinate in the qrcode is not 2 ,but %d, omitting...", coordinate_str_vec.size());
        }else{
            geometry_msgs::TransformStamped marker_transform_stamped;
            marker_transform_stamped.header.stamp = ros::Time::now();
            marker_transform_stamped.header.frame_id = "marker_basis";
            marker_transform_stamped.child_frame_id = "marker";

            marker_transform_stamped.transform.translation.x = std::stod(coordinate_str_vec[0]);
            marker_transform_stamped.transform.translation.y = std::stod(coordinate_str_vec[1]);
            marker_transform_stamped.transform.translation.z = 0;
            marker_transform_stamped.transform.rotation.w = 1;
            marker_transform_stamped.transform.rotation.x = 0;
            marker_transform_stamped.transform.rotation.y = 0;
            marker_transform_stamped.transform.rotation.z = 0;

            marker_br.sendTransform(marker_transform_stamped);

            geometry_msgs::TransformStamped drone_base_transform_stamped;
            drone_base_transform_stamped.header.stamp = ros::Time::now();
            drone_base_transform_stamped.header.frame_id = "marker";
            drone_base_transform_stamped.child_frame_id = "drone_base";

            drone_base_transform_stamped.transform.translation.x =cameraPoseBuffer.at<double>(0,3);
            drone_base_transform_stamped.transform.translation.y =cameraPoseBuffer.at<double>(1,3);
            drone_base_transform_stamped.transform.translation.z =cameraPoseBuffer.at<double>(2,3);
            tf2::Matrix3x3 m;
            tf2::Quaternion q;
            m.setValue(cameraPoseBuffer.at<double>(0,0),cameraPoseBuffer.at<double>(0,1),cameraPoseBuffer.at<double>(0,2),cameraPoseBuffer.at<double>(1,0),cameraPoseBuffer.at<double>(1,1),cameraPoseBuffer.at<double>(1,2),cameraPoseBuffer.at<double>(2,0),cameraPoseBuffer.at<double>(2,1),cameraPoseBuffer.at<double>(2,2));
            m.getRotation(q);
            drone_base_transform_stamped.transform.rotation.w = q.w();
            drone_base_transform_stamped.transform.rotation.x = q.x();
            drone_base_transform_stamped.transform.rotation.y = q.y();
            drone_base_transform_stamped.transform.rotation.z = q.z();
            drone_cam_br.sendTransform(drone_base_transform_stamped);
        }
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



    //parse command line arguments
    ecl::CmdLine cmd("qrcode positioning node.");
    ecl::ValueArg<std::string> pathArg("f","file","calibration file to read",false,"/home/calibrations/default.ini","string");
    cmd.add(pathArg);
    cmd.parse(argc,argv);
    std::string filePath = pathArg.getValue();

    if(filePath.size() > 0){
        ROS_DEBUG("calibration file path:%s",filePath.c_str());
    }else{
        ROS_ERROR("empty calibration file path");
        exit(-2);
    }

    bool success;
    //read camera calibration info
    std::string camera_name;
    sensor_msgs::CameraInfo cam_info;
    success = camera_calibration_parsers::readCalibration(filePath, camera_name, cam_info);

    if(success){
        ROS_DEBUG("calibration readed");
    }else{
        ROS_ERROR("read calibration file failed");
        exit(-3);
    }

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
    stateEstimator.reset(new QRCodeStateEstimator(cam_info.width, cam_info.height, cameraMatrix,distortionParameters,true));

    //initialize tf2 broadcasters
    tf2_ros::StaticTransformBroadcaster marker_basis_br;    //broadcast the marker basis to world transform

    //marker basis initialization
    geometry_msgs::TransformStamped marker_basis_transform_stamped;
    marker_basis_transform_stamped.header.stamp = ros::Time::now();
    marker_basis_transform_stamped.header.frame_id = "world";
    marker_basis_transform_stamped.child_frame_id = "marker_basis";

    marker_basis_transform_stamped.transform.translation.x = 0;
    marker_basis_transform_stamped.transform.translation.y = 0;
    marker_basis_transform_stamped.transform.translation.z = 3.0;
    tf2::Quaternion q;
    q.setRPY(0,0,0);
    marker_basis_transform_stamped.transform.rotation.x = q.x();
    marker_basis_transform_stamped.transform.rotation.y = q.y();
    marker_basis_transform_stamped.transform.rotation.z = q.z();
    marker_basis_transform_stamped.transform.rotation.w = q.w();

    //broadcast marker basis
    marker_basis_br.sendTransform(marker_basis_transform_stamped);

    ros::spin();

}
