
#include <opencv2/highgui/highgui.hpp>
#include <std_msgs/Header.h>

#include <cv_bridge/cv_bridge.h>

#include <hce_msgs/CallDumpDetector.h>
#include <hce_msgs/HceSingleImage.h>

#include "hce_detector.hpp"


HceSingleImageDetector::HceSingleImageDetector(ros::NodeHandle& nh) : nh_(&nh)
{
  // Advertise the single image analysis service
  service_gcs =
      nh.advertiseService("tag_centers",
                          &HceSingleImageDetector::callbackService, this);

  ROS_INFO_STREAM("Ready to do tag detection on single images");
}

bool HceSingleImageDetector::callbackService(
    hce_msgs::CallDumpDetector::Request& request,
    hce_msgs::CallDumpDetector::Response& response)
{
  ROS_INFO("good good");
  client_apriltag = nh_->serviceClient<hce_msgs::HceSingleImage>(
                "single_image_tag_detection");
  hce_msgs::HceSingleImage srv;
  srv.request.img0 = request.img0;

  if (client_apriltag.call(srv))
  {
    ROS_INFO("call service_apriltag");

    if (srv.response.tag_detections.detections.size() == 0)
    {
      ROS_WARN_STREAM("No detected tags!");
    }
    else
    {
      response.success = true;
      for (int i = 0; i < srv.response.tag_detections.detections.size(); ++i)
    {
      std::cout<<"============ "<<"id = "<<srv.response.tag_detections.detections[i].id.at(0)<<" ============"<<std::endl;
      printf("{x, y, z} = {%.4f, %.4f, %.4f}\n", srv.response.tag_detections.detections[i].pose.pose.pose.position.x,
                                                 srv.response.tag_detections.detections[i].pose.pose.pose.position.y,
                                                 srv.response.tag_detections.detections[i].pose.pose.pose.position.z);
      printf("{qx, qy, qz, qw} = {%.4f, %.4f, %.4f, %.8f}\n", srv.response.tag_detections.detections[i].pose.pose.pose.orientation.x,
                                                              srv.response.tag_detections.detections[i].pose.pose.pose.orientation.y,
                                                              srv.response.tag_detections.detections[i].pose.pose.pose.orientation.z,
                                                              srv.response.tag_detections.detections[i].pose.pose.pose.orientation.w);
    }
    }
  }
  else
  {
    ROS_ERROR("Failed to call service single_image_tag_detection");
    return 1;
  }

  ROS_INFO("Done !");

  return true;
}
