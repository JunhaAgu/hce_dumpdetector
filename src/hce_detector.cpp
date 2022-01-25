#include <std_msgs/Header.h>

#include <cv_bridge/cv_bridge.h>

#include <opencv2/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <hce_msgs/CallDumpDetector.h>
#include <apriltag_ros/HceSingleImage.h>

#include "hce_detector.hpp"
#include "ROS_print_in_color.h"

using namespace std;
using namespace cv;

HceSingleImageDetector::HceSingleImageDetector(ros::NodeHandle& nh, const string& dir_yaml) : nh_(nh), dir_yaml_(dir_yaml)
{
  string yaml_tag_relative_pose = dir_yaml_ + "tag_relative_pose.yaml";

  // Read yaml file function
  this->readYamlFileFunction(yaml_tag_relative_pose);

  // Advertise the single image analysis service
  server_hce_dumpdetector =
      nh.advertiseService("tag_centers",
                          &HceSingleImageDetector::callbackService, this);
  
  // point_pub = nh.advertise<geometry_msgs::Point>("point_pub", 1);

  ROS_INFO_STREAM("\n <hce_dumpdetector server> Ready to communicate");
  
}

bool HceSingleImageDetector::callbackService(
    hce_msgs::CallDumpDetector::Request& request,
    hce_msgs::CallDumpDetector::Response& response)
{
  // for(int i=0; i<4; ++i)
  // {
  //   response.tag_centers.push_back(new vector<geometry_msgs::Point>);
  // }

  ROS_INFO("\n <hce_dumpdetector server> callback");
  client_apriltag_ros = nh_.serviceClient<apriltag_ros::HceSingleImage>(
                "single_image_tag_detection");
  apriltag_ros::HceSingleImage srv;
  srv.request.img0 = request.img0;

  // srv.request.camera_info.distortion_model = "plumb_bob";
  // double fx, fy, cx, cy, k1, k2, p1, p2;
  // nh_.getParam("fx", fx);
  // nh_.getParam("fy", fy);
  // nh_.getParam("cx", cx);
  // nh_.getParam("cy", cy);
  // nh_.getParam("k1", k1);
  // nh_.getParam("k2", k2);
  // nh_.getParam("p1", p1);
  // nh_.getParam("p2", p2);

  // // Intrinsic camera matrix for the raw (distorted) images
  // srv.request.camera_info.K[0] = fx;
  // srv.request.camera_info.K[2] = cx;
  // srv.request.camera_info.K[4] = fy;
  // srv.request.camera_info.K[5] = cy;
  // srv.request.camera_info.K[8] = 1.0;
  // srv.request.camera_info.P[0] = fx;
  // srv.request.camera_info.P[2] = cx;
  // srv.request.camera_info.P[5] = fy;
  // srv.request.camera_info.P[6] = cy;
  // srv.request.camera_info.P[10] = 1.0;

  // srv.request.camera_info.D.push_back(k1);
  // srv.request.camera_info.D.push_back(k2);
  // srv.request.camera_info.D.push_back(p1);
  // srv.request.camera_info.D.push_back(p2);

  // std::cout<< srv.request.camera_info.D.at(0)<<std::endl;

  if (client_apriltag_ros.call(srv))
  {
    ROS_INFO("\n <hce_dumpdetector server> call service_apriltag_ros");
    n_tag_detection = srv.response.tag_detections.detections.size();

    if (n_tag_detection == 0)
    {
      ROS_WARN_STREAM("No detected tags!");
    }
    else
    {
      response.success = true;
      for (int i = 0; i < n_tag_detection; ++i)
      {
        std::cout << " <hce_dumpdetector server> ========== "
                  << "id = " << srv.response.tag_detections.detections[i].id.at(0) << " =========="<< std::endl;
        printf("{x, y, z} = {%.4f, %.4f, %.4f}\n", srv.response.tag_detections.detections[i].pose.pose.pose.position.x,
               srv.response.tag_detections.detections[i].pose.pose.pose.position.y,
               srv.response.tag_detections.detections[i].pose.pose.pose.position.z);
        // printf("{qx, qy, qz, qw} = {%.4f, %.4f, %.4f, %.8f}\n", srv.response.tag_detections.detections[i].pose.pose.pose.orientation.x,
        //        srv.response.tag_detections.detections[i].pose.pose.pose.orientation.y,
        //        srv.response.tag_detections.detections[i].pose.pose.pose.orientation.z,
        //        srv.response.tag_detections.detections[i].pose.pose.pose.orientation.w);

        // response.tag_centers.push_back(srv.response.tag_detections.detections[i].pose.pose.pose.position);
        tag_pose.push_back(srv.response.tag_detections.detections[i].pose.pose.pose);
        
        // std::cout << "============ "
        //           << "id = " << srv.response.tag_detections.detections[i].id.at(0) << " ============" << std::endl;
        // printf("{x, y, z} = {%.4f, %.4f, %.4f}\n", response.tag_centers[i].x,
        //        response.tag_centers[i].y,
        //        response.tag_centers[i].z);
      }

      this->calculateTagPoints(srv.response.tag_detections);
      response.tag_centers = tag_points_final;

      tag_pose.clear();
      tag_points_final.clear();
                  
      response.header.stamp = ros::Time::now();
      response.header.stamp.sec = ros::Time::now().toSec();
      response.header.stamp.nsec = ros::Time::now().toNSec();
    }
  }
  else
  {
    ROS_ERROR("Failed to call service single_image_tag_detection");
    return 1;
  }

  // printf("%d",srv.response.tag_detections.detections.size());
  // point_pub.publish(response.tag_centers[0]);

  ROS_INFO("\n <hce_dumpdetector server> Done!");
  std::cout << pc::GREEN << "Is the tag detection successful?: " << (response.success ? "YES" : "NO") << pc::ENDCOLOR << std::endl;

  return true;
}

void HceSingleImageDetector::calculateTagPoints(apriltag_ros::AprilTagDetectionArray &tag_detection_array)
{
  vector<unsigned char> tag_ids;
  for (int i = 0; i < n_tag_detection; ++i)
  {
    tag_ids.push_back((unsigned char)tag_detection_array.detections.at(i).id.at(0));
  }

  // ROS_INFO("%d", n_tag_detection);
  // for (int i = 0; i < n_tag_detection; ++i)
  // {
  //   printf("{x, y, z} = {%.4f, %.4f, %.4f}\n", tag_detection_array.detections[i].pose.pose.pose.position.x,
  //              tag_detection_array.detections[i].pose.pose.pose.position.y,
  //              tag_detection_array.detections[i].pose.pose.pose.position.z);
  // }
  
  // ROS_INFO("%d", tag_ids.at(0));
  // ROS_INFO("%d", tag_ids.at(1));
  // ROS_INFO("%d", tag_ids.at(2));
  // ROS_INFO("%d", tag_ids.at(3));

  double dist_min = 100;
  double dist_tmp = 0;
  unsigned char id_min = 10;
  unsigned char i_min = 10;

  for(int i=0; i<n_tag_detection; ++i)
  {
    dist_tmp = pow( pow(tag_pose.at(i).position.x, 2) + pow(tag_pose.at(i).position.y, 2) + pow(tag_pose.at(i).position.z, 2) , 0.5);
    if (dist_tmp < dist_min)
    {
      dist_min = dist_tmp;
      id_min = tag_ids.at(i);
      i_min = i;
    }
  }
  vector<double> dist_min_quat{tag_pose.at(i_min).orientation.w, tag_pose.at(i_min).orientation.x,
                              tag_pose.at(i_min).orientation.y, tag_pose.at(i_min).orientation.z};
  Mat33d rotm_ = Mat33d::Zero();
  Mat44d tag_T_ = Mat44d::Zero();
  rotm_ = HceSingleImageDetector::quat2rot(dist_min_quat);
  tag_T_ = Mat44d::Zero();
  tag_T_.block<3, 3>(0, 0) = rotm_;
  tag_T_.block<3, 1>(0, 3) << tag_pose.at(i_min).position.x, tag_pose.at(i_min).position.y, tag_pose.at(i_min).position.z;
  tag_T_(3, 3) = 1;

  ROS_INFO("Min Dist id: %d",id_min);

  geometry_msgs::Point tag_point_result;
  switch (id_min)
  {
  case 0:
    T_cam2tag0 = tag_T_;
    T_cam2tag1 = tag_T_ * tag_T_01;
    T_cam2tag2 = tag_T_ * tag_T_02;
    T_cam2tag3 = tag_T_ * tag_T_03;
    break;
  case 1:
    T_cam2tag0 = tag_T_ * tag_T_01.inverse();
    T_cam2tag1 = tag_T_;
    T_cam2tag2 = T_cam2tag0 * tag_T_02;
    T_cam2tag3 = T_cam2tag0 * tag_T_03;
    break;
  case 2:
    T_cam2tag0 = tag_T_ * tag_T_02.inverse();
    T_cam2tag1 = T_cam2tag0 * tag_T_01;
    T_cam2tag2 = tag_T_;
    T_cam2tag3 = T_cam2tag0 * tag_T_03;
    break;
  case 3:
    T_cam2tag0 = tag_T_ * tag_T_03.inverse();
    T_cam2tag1 = T_cam2tag0 * tag_T_01;
    T_cam2tag2 = T_cam2tag0 * tag_T_02;
    T_cam2tag3 = tag_T_;
    break;
  }

  tag_point_result.x = T_cam2tag0(0,3);
  tag_point_result.y = T_cam2tag0(1,3);
  tag_point_result.z = T_cam2tag0(2,3);
  tag_points_final.push_back(tag_point_result);
  tag_point_result.x = T_cam2tag1(0,3);
  tag_point_result.y = T_cam2tag1(1,3);
  tag_point_result.z = T_cam2tag1(2,3);
  tag_points_final.push_back(tag_point_result);
  tag_point_result.x = T_cam2tag2(0,3);
  tag_point_result.y = T_cam2tag2(1,3);
  tag_point_result.z = T_cam2tag2(2,3);
  tag_points_final.push_back(tag_point_result);
  tag_point_result.x = T_cam2tag3(0,3);
  tag_point_result.y = T_cam2tag3(1,3);
  tag_point_result.z = T_cam2tag3(2,3);
  tag_points_final.push_back(tag_point_result);

  dist_min_quat.clear();
}

Mat33d HceSingleImageDetector::quat2rot(vector<double>& q)
{
  Mat33d q2m = Mat33d::Zero();
  q2m.block<3,3>(0,0)<< 1 - 2 * (pow(q[2],2) + pow(q[3],2)),         2 * (q[1]*q[2] - q[0]*q[3]),         2 * (q[1]*q[3] + q[0]*q[2]),
                                2 * (q[1]*q[2] + q[0]*q[3]), 1 - 2 * (pow(q[1],2) + pow(q[3],2)),         2 * (q[2]*q[3] - q[0]*q[1]),
                                2 * (q[1]*q[3] - q[0]*q[2]),         2 * (q[2]*q[3] + q[0]*q[1]), 1 - 2 * (pow(q[1],2) + pow(q[2],2));
  return q2m;
}


void HceSingleImageDetector::readYamlFileFunction(string& yaml_tag_relative_pose)
{
  cv::FileStorage tag_relative_pose(yaml_tag_relative_pose, cv::FileStorage::READ);
  if (!tag_relative_pose.isOpened())
  {
    std::cout << "Failed to open <tag_relative_pose>." << std::endl;
    abort();
  }
  tag_pose_00.x = tag_relative_pose["x_00"];
  tag_pose_00.y = tag_relative_pose["y_00"];
  tag_pose_00.z = tag_relative_pose["z_00"];
  tag_pose_00.qw = tag_relative_pose["qw_00"];
  tag_pose_00.qx = tag_relative_pose["qx_00"];
  tag_pose_00.qy = tag_relative_pose["qy_00"];
  tag_pose_00.qz = tag_relative_pose["qz_00"];

  //qw qx qy qz
  tag_quat_00.push_back(tag_pose_00.qw);
  tag_quat_00.push_back(tag_pose_00.qx);
  tag_quat_00.push_back(tag_pose_00.qy);
  tag_quat_00.push_back(tag_pose_00.qz);
  
  Mat33d rotm_00 = Mat33d::Zero();
  rotm_00 = HceSingleImageDetector::quat2rot(tag_quat_00);
  tag_T_00 = Mat44d::Zero();
  tag_T_00.block<3,3>(0,0) = rotm_00;
  tag_T_00.block<3,1>(0,3) << tag_pose_00.x, tag_pose_00.y, tag_pose_00.z;
  tag_T_00(3,3) = 1;
  // printf("[%.8f, %.8f, %.8f, %.8f\n %.8f, %.8f, %.8f, %.8f\n %.8f, %.8f, %.8f, %.8f\n %.8f, %.8f, %.8f, %.8f]\n"
  // ,tag_T_00(0,0), tag_T_00(0,1), tag_T_00(0,2), tag_T_00(0,3)
  // , tag_T_00(1,0), tag_T_00(1,1), tag_T_00(1,2), tag_T_00(1,3)
  // , tag_T_00(2,0), tag_T_00(2,1), tag_T_00(2,2), tag_T_00(2,3)
  // , tag_T_00(3,0), tag_T_00(3,1), tag_T_00(3,2), tag_T_00(3,3) );

  tag_pose_01.x = tag_relative_pose["x_01"];
  tag_pose_01.y = tag_relative_pose["y_01"];
  tag_pose_01.z = tag_relative_pose["z_01"];
  tag_pose_01.qw = tag_relative_pose["qw_01"];
  tag_pose_01.qx = tag_relative_pose["qx_01"];
  tag_pose_01.qy = tag_relative_pose["qy_01"];
  tag_pose_01.qz = tag_relative_pose["qz_01"];
  tag_quat_01.push_back(tag_pose_01.qw);
  tag_quat_01.push_back(tag_pose_01.qx);
  tag_quat_01.push_back(tag_pose_01.qy);
  tag_quat_01.push_back(tag_pose_01.qz);

  Mat33d rotm_01 = Mat33d::Zero();
  rotm_01 = HceSingleImageDetector::quat2rot(tag_quat_01);
  tag_T_01 = Mat44d::Zero();
  tag_T_01.block<3,3>(0,0) = rotm_01;
  tag_T_01.block<3,1>(0,3) << tag_pose_01.x, tag_pose_01.y, tag_pose_01.z;
  tag_T_01(3,3) = 1;
  // printf("[%.8f, %.8f, %.8f, %.8f\n %.8f, %.8f, %.8f, %.8f\n %.8f, %.8f, %.8f, %.8f\n %.8f, %.8f, %.8f, %.8f]\n"
  // ,tag_T_01(0,0), tag_T_01(0,1), tag_T_01(0,2), tag_T_01(0,3)
  // , tag_T_01(1,0), tag_T_01(1,1), tag_T_01(1,2), tag_T_01(1,3)
  // , tag_T_01(2,0), tag_T_01(2,1), tag_T_01(2,2), tag_T_01(2,3)
  // , tag_T_01(3,0), tag_T_01(3,1), tag_T_01(3,2), tag_T_01(3,3) );

  tag_pose_02.x = tag_relative_pose["x_02"];
  tag_pose_02.y = tag_relative_pose["y_02"];
  tag_pose_02.z = tag_relative_pose["z_02"];
  tag_pose_02.qw = tag_relative_pose["qw_02"];
  tag_pose_02.qx = tag_relative_pose["qx_02"];
  tag_pose_02.qy = tag_relative_pose["qy_02"];
  tag_pose_02.qz = tag_relative_pose["qz_02"];
  tag_quat_02.push_back(tag_pose_02.qw);
  tag_quat_02.push_back(tag_pose_02.qx);
  tag_quat_02.push_back(tag_pose_02.qy);
  tag_quat_02.push_back(tag_pose_02.qz);

  Mat33d rotm_02 = Mat33d::Zero();
  rotm_02 = HceSingleImageDetector::quat2rot(tag_quat_02);
  tag_T_02 = Mat44d::Zero();
  tag_T_02.block<3,3>(0,0) = rotm_02;
  tag_T_02.block<3,1>(0,3) << tag_pose_02.x, tag_pose_02.y, tag_pose_02.z;
  tag_T_02(3,3) = 1;
  // printf("[%.8f, %.8f, %.8f, %.8f\n %.8f, %.8f, %.8f, %.8f\n %.8f, %.8f, %.8f, %.8f\n %.8f, %.8f, %.8f, %.8f]\n"
  // ,tag_T_02(0,0), tag_T_02(0,1), tag_T_02(0,2), tag_T_02(0,3)
  // , tag_T_02(1,0), tag_T_02(1,1), tag_T_02(1,2), tag_T_02(1,3)
  // , tag_T_02(2,0), tag_T_02(2,1), tag_T_02(2,2), tag_T_02(2,3)
  // , tag_T_02(3,0), tag_T_02(3,1), tag_T_02(3,2), tag_T_02(3,3) );

  tag_pose_03.x = tag_relative_pose["x_03"];
  tag_pose_03.y = tag_relative_pose["y_03"];
  tag_pose_03.z = tag_relative_pose["z_03"];
  tag_pose_03.qw = tag_relative_pose["qw_03"];
  tag_pose_03.qx = tag_relative_pose["qx_03"];
  tag_pose_03.qy = tag_relative_pose["qy_03"];
  tag_pose_03.qz = tag_relative_pose["qz_03"];
  tag_quat_03.push_back(tag_pose_03.qw);
  tag_quat_03.push_back(tag_pose_03.qx);
  tag_quat_03.push_back(tag_pose_03.qy);
  tag_quat_03.push_back(tag_pose_03.qz);

  Mat33d rotm_03 = Mat33d::Zero();
  rotm_03 = HceSingleImageDetector::quat2rot(tag_quat_03);
  tag_T_03 = Mat44d::Zero();
  tag_T_03.block<3,3>(0,0) = rotm_03;;
  tag_T_03.block<3,1>(0,3) << tag_pose_03.x, tag_pose_03.y, tag_pose_03.z;
  tag_T_03(3,3) = 1;
  // printf("[%.8f, %.8f, %.8f, %.8f\n %.8f, %.8f, %.8f, %.8f\n %.8f, %.8f, %.8f, %.8f\n %.8f, %.8f, %.8f, %.8f]\n"
  // ,tag_T_03(0,0), tag_T_03(0,1), tag_T_03(0,2), tag_T_03(0,3)
  // , tag_T_03(1,0), tag_T_03(1,1), tag_T_03(1,2), tag_T_03(1,3)
  // , tag_T_03(2,0), tag_T_03(2,1), tag_T_03(2,2), tag_T_03(2,3)
  // , tag_T_03(3,0), tag_T_03(3,1), tag_T_03(3,2), tag_T_03(3,3) );
}