#ifndef HCE_DUMPDETECTOR_HCE_DETECTOR_HPP
#define HCE_DUMPDETECTOR_HCE_DETECTOR_HPP

#include <ros/ros.h>
#include <ros/service_traits.h>

#include "hce_msgs/CallDumpDetector.h"

#include "apriltag_ros/AprilTagDetectionArray.h"

#include <cstdlib>
#include <string>

#include <eigen3/Eigen/Dense>

typedef Eigen::Matrix<double, 3, 1> Vec3d;
typedef Eigen::Matrix<double, 4, 1> Vec4d;
typedef Eigen::Matrix<double, 6, 1> Vec6d;
typedef Eigen::Matrix<double, 3, 3> Mat33d;
typedef Eigen::Matrix<double, 4, 4> Mat44d;
typedef Eigen::Matrix<double, 6, 6> Mat66d;
typedef Eigen::Matrix<double, 4, 6> Mat46d;

using namespace std;

struct S_RELATIVE_POSE
{
    double x;
    double y;
    double z;
    double qw;
    double qx;
    double qy;
    double qz;

};

class HceSingleImageDetector
{
private:
    ros::ServiceServer server_hce_dumpdetector;

    ros::ServiceClient client_apriltag_ros;
    // nh.serviceClient<apriltag_ros::HceSingleImage>(
    //     "single_image_tag_detection");
    ros::NodeHandle nh_;

    ros::Publisher point_pub;

    vector<geometry_msgs::Pose> tag_pose;
    vector<geometry_msgs::Point> tag_points_final;
    
    S_RELATIVE_POSE tag_pose_00;
    S_RELATIVE_POSE tag_pose_01;
    S_RELATIVE_POSE tag_pose_02;
    S_RELATIVE_POSE tag_pose_03;

    vector<double> tag_quat_00;
    vector<double> tag_quat_01;
    vector<double> tag_quat_02;
    vector<double> tag_quat_03;

    Mat44d tag_T_00;
    Mat44d tag_T_01;
    Mat44d tag_T_02;
    Mat44d tag_T_03;

    Mat44d T_cam2tag0;
    Mat44d T_cam2tag1;
    Mat44d T_cam2tag2;
    Mat44d T_cam2tag3;

private:
    int n_tag_detection;

public:
    string dir_yaml_;

public:
    HceSingleImageDetector(ros::NodeHandle &nh, const string& dir_yaml);

    bool callbackService(hce_msgs::CallDumpDetector::Request &request,
                         hce_msgs::CallDumpDetector::Response &response);

    void calculateTagPoints(apriltag_ros::AprilTagDetectionArray &tag_detection_array);

    void readYamlFileFunction(string& yaml_tag_relative_pose);

    Mat33d quat2rot(vector<double>& quat);
};

#endif // HCE_DUMPDETECTOR_HCE_DETECTOR_HPP