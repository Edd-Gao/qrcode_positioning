#include <camera_calibration_parsers/parse.h>
#include <sensor_msgs/CameraInfo.h>
#include <string>
#include <iostream>

using namespace std;
using namespace camera_calibration_parsers;

int main(){
    string camera_name;
    sensor_msgs::CameraInfo cam_info;
    readCalibration("test.ini", camera_name, cam_info);

    cout << "camera name:"<< camera_name << endl;
    cout << "camera info:"<<cam_info<<endl;
}