#include <ros/ros.h>
#include <ros/console.h>

#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <std_msgs/Header.h>
#include "hce_msgs/CallDumpDetector.h"

#include <functional>

#include "hce_detector.hpp"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "hce_dumpdetector_server_node");
    ros::NodeHandle nh;

    HceSingleImageDetector hce_tag_detector(nh);
    ros::spin();

    return 0;
}