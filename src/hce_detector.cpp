
#include <opencv2/highgui/highgui.hpp>
#include <std_msgs/Header.h>

#include <cv_bridge/cv_bridge.h>

#include <hce_msgs/CallDumpDetector.h>
#include <hce_msgs/HceSingleImage.h>

#include "hce_detector.hpp"

#include "vector"

using namespace std;


HceSingleImageDetector::HceSingleImageDetector(ros::NodeHandle& nh) : nh_(nh)
{
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
  client_apriltag_ros = nh_.serviceClient<hce_msgs::HceSingleImage>(
                "single_image_tag_detection");
  hce_msgs::HceSingleImage srv;
  srv.request.img0 = request.img0;

  if (client_apriltag_ros.call(srv))
  {
    ROS_INFO("\n <hce_dumpdetector server> call service_apriltag_ros");

    if (srv.response.tag_detections.detections.size() == 0)
    {
      ROS_WARN_STREAM("No detected tags!");
    }
    else
    {
      response.success = true;
      for (int i = 0; i < srv.response.tag_detections.detections.size(); ++i)
      {
        std::cout << " <hce_dumpdetector server> ========== "
                  << "id = " << srv.response.tag_detections.detections[i].id.at(0) << " =========="<< std::endl;
        printf("{x, y, z} = {%.4f, %.4f, %.4f}\n", srv.response.tag_detections.detections[i].pose.pose.pose.position.x,
               srv.response.tag_detections.detections[i].pose.pose.pose.position.y,
               srv.response.tag_detections.detections[i].pose.pose.pose.position.z);
        printf("{qx, qy, qz, qw} = {%.4f, %.4f, %.4f, %.8f}\n", srv.response.tag_detections.detections[i].pose.pose.pose.orientation.x,
               srv.response.tag_detections.detections[i].pose.pose.pose.orientation.y,
               srv.response.tag_detections.detections[i].pose.pose.pose.orientation.z,
               srv.response.tag_detections.detections[i].pose.pose.pose.orientation.w);

        response.tag_centers.push_back(srv.response.tag_detections.detections[i].pose.pose.pose.position);
        
        // std::cout << "============ "
        //           << "id = " << srv.response.tag_detections.detections[i].id.at(0) << " ============" << std::endl;
        // printf("{x, y, z} = {%.4f, %.4f, %.4f}\n", response.tag_centers[i].x,
        //        response.tag_centers[i].y,
        //        response.tag_centers[i].z);
      }
      
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
  

  ROS_INFO("\n <hce_dumpdetector server> Done!\n");

  return true;
}
