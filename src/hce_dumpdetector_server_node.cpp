#include <ros/ros.h>
#include <ros/console.h>

#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <std_msgs/Header.h>
#include "hce_msgs/CallDumpDetector.h"

#include <functional>

#include "hce_detector.hpp"
// #include "hce_detector.cpp"

// bool advertise_flag = false;
// sensor_msgs::Image image_tmp;

// bool add(hce_msgs::CallDumpDetector::Request  &request,
//          hce_msgs::CallDumpDetector::Response &response)
// {
//     //   advertise_flag = true;
//     //   image_tmp = request.img0;

//     // ros::ServiceClient client_apriltag =
//     //     nh.serviceClient<apriltag_ros::HceSingleImage>(
//     //         "single_image_tag_detection");
//     // apriltag_ros::HceSingleImage service_apriltag;

//     // ROS_INFO("hce_msg::CallDumpDetector is received");
//     // service_apriltag.request.img0 = request.img0;

//     // if (client_apriltag.call(service_apriltag))
//     // {
//     //     ROS_INFO("call service_apriltag");

//     //     if (service_apriltag.response.tag_detections.detections.size() == 0)
//     //     {
//     //         ROS_WARN_STREAM("No detected tags!");
//     //     }
//     // }
//     // else
//     // {
//     //     ROS_ERROR("Failed to call service single_image_tag_detection");
//     //     return 1;
//     // }

//     return true;
// }

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hce_dumpdetector_server_node");
    ros::NodeHandle nh;
    // ros::NodeHandle pnh("~");
    // ros::Rate rate(100);

    // ros::ServiceServer service_gcs = nh.advertiseService("tag_centers", add);

    HceSingleImageDetector hce_tag_detector(nh);
    ros::spin();

    // ros::ServiceClient client_apriltag =
    //             nh.serviceClient<apriltag_ros::HceSingleImage>(
    //                 "single_image_tag_detection");
    // apriltag_ros::HceSingleImage service_apriltag;

    // ros::ServiceServer service_gcs = nh.advertiseService(
    //     "tag_centers",
    //     [&] (hce_msgs::CallDumpDetector::Request& req, hce_msgs::CallDumpDetector::Response& res) {
    //         advertise_flag = true;
    //         return advertise_flag;
    //         // DO SOMETHING
    //     }
    // );
    // std::function cb = [&] (hce_msgs::CallDumpDetector::Request& req, hce_msgs::CallDumpDetector::Response& res) {
    //         advertise_flag = true;
    //         return advertise_flag;
    //         // DO SOMETHING
    //     };
    // std::function<bool(hce_msgs::CallDumpDetector::Request&, hce_msgs::CallDumpDetector::Response&)> cb 
    // = [&](hce_msgs::CallDumpDetector::Request& req, hce_msgs::CallDumpDetector::Response& res)
    // {
    //     advertise_flag = true;
    //     return advertise_flag;
    //     // DO SOMETHING
    // };
    // ros::ServiceServer service_gcs = nh.advertiseService("tag_centers", cb);

    // ROS_INFO("Ready to detect dump region");
    // ros::spin();

    // while (ros::ok())
    // {
    //     while (advertise_flag == true)
    //     {
    //         ROS_INFO("hce_msg::CallDumpDetector is received");
    //         service_apriltag.request.img0 = image_tmp;
            
    //         if (client_apriltag.call(service_apriltag))
    //         {
    //             ROS_INFO("call service_apriltag");

    //             if (service_apriltag.response.tag_detections.detections.size() == 0)
    //             {
    //                 ROS_WARN_STREAM("No detected tags!");
    //             }
    //         }
    //         else
    //         {
    //             ROS_ERROR("Failed to call service single_image_tag_detection");
    //             return 1;
    //         }
    //         advertise_flag = false;
    //     }
    //     ros::spinOnce();
    //     rate.sleep();
    // }

    // if (advertise_flag == true)
    // {
    //     ros::ServiceClient client_apriltag =
    //         nh.serviceClient<apriltag_ros::HceSingleImage>(
    //             "single_image_tag_detection");
    //     apriltag_ros::HceSingleImage service_apriltag;
    //     // service_apriltag.request.img0 =
    //     cv::Mat img = cv::imread("/home/junhakim/6.png", 1);
    //     cv_bridge::CvImage img_bridge;
    //     sensor_msgs::Image img_msg;

    //     std_msgs::Header header; // empty header
    //     // header.seq = counter; // user defined counter
    //     header.stamp = ros::Time::now(); // time
    //     img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, img);
    //     img_bridge.toImageMsg(img_msg);

    //     service_apriltag.request.img0 = img_msg;

    //     // Call the service (detect tags in the image specified by the
    //     // image_load_path)
    //     if (client_apriltag.call(service_apriltag))
    //     {
    //         // use parameter run_quielty=false in order to have the service
    //         // print out the tag position and orientation
    //         // if (service.response.tag_detections.detections.size() == 0)
    //         // {
    //         //     ROS_WARN_STREAM("No detected tags!");
    //         // }
    //         ROS_INFO("call service_apriltag");
    //     }
    //     else
    //     {
    //         ROS_ERROR("Failed to call service single_image_tag_detection");
    //         return 1;
    //     }

    //     return 0; // happy ending
    // }

    return 0;
}