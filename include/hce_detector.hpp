#ifndef HCE_DUMPDETECTOR_HCE_DETECTOR_HPP
#define HCE_DUMPDETECTOR_HCE_DETECTOR_HPP

#include <ros/ros.h>
#include <ros/service_traits.h>

#include "hce_msgs/CallDumpDetector.h"

class HceSingleImageDetector
{
 private:
  ros::ServiceServer service_gcs;

  ros::ServiceClient client_apriltag;
                // nh.serviceClient<apriltag_ros::HceSingleImage>(
                //     "single_image_tag_detection");
  ros::NodeHandle* nh_;
  
 public:

  HceSingleImageDetector(ros::NodeHandle& nh);

  bool callbackService(hce_msgs::CallDumpDetector::Request& request,
                    hce_msgs::CallDumpDetector::Response& response);
};

#endif // HCE_DUMPDETECTOR_HCE_DETECTOR_HPP