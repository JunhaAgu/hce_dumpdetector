#include "ros/ros.h"
#include "hce_dumpdetector/AddTwoInts.h"
#include <cstdlib>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui.hpp>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include "hce_msgs/CallDumpDetector.h"

using namespace cv;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "example_client");
  if (argc != 2)
  {
    ROS_INFO("usage: start with any number");
    return 1;
  }

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<hce_msgs::CallDumpDetector>("tag_centers");
  hce_msgs::CallDumpDetector srv;
  // srv.request.a = atoll(argv[1]);
  // srv.request.b = atoll(argv[2]);

  Mat img = imread("/home/junhakim/6.png", 1);
  cv_bridge::CvImage img_bridge;
  sensor_msgs::Image img_msg;

  std_msgs::Header header; // empty header
  // header.seq = counter; // user defined counter
  header.stamp = ros::Time::now(); // time
  img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, img);
  img_bridge.toImageMsg(img_msg); // from cv_bridge to sensor_msgs::Image
  // pub_img.publish(img_msg); // ros::Publisher pub_img = node.advertise<sensor_msgs::Image>("topic", queuesize);
  srv.request.header = header;
  srv.request.img0 = img_msg;

  // cv_bridge::CvImagePtr cv_ptr;
  // cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::RGB8);
  // cv::Mat image = cv_ptr->image;

  // cv::imwrite("/home/junhakim/service_test_results/image_request.png", image);

  if (client.call(srv))
  {
    // ROS_INFO("Sum: %ld", (long int)srv.response.sum);
    // ROS_INFO("tag detection success?: Yes");
    ROS_INFO("tag detection success?: %s", srv.response.success ? "Yes" : "No");
    
    // for (int i = 0; i < srv.response.tag_centers.size(); ++i)
    // {
    //   std::cout<<"============ "<<"id = "<<srv.response.tag_centers[i].id.at(0)<<" ============"<<std::endl;
    //   printf("id = %d, {x, y, z} = {%.4f, %.4f, %.4f}\n", srv.response.tag_centers[i].id.at(0), srv.response.tag_centers[i].pose.pose.pose.position.x, srv.response.tag_centers[i].pose.pose.pose.position.y, srv.response.tag_centers[i].pose.pose.pose.position.z);
    //   printf("{qx, qy, qz, qw} = {%.4f, %.4f, %.4f, %.8f}\n", srv.response.tag_centers[i].pose.pose.pose.orientation.x, srv.response.tag_centers[i].pose.pose.pose.orientation.y, srv.response.tag_centers[i].pose.pose.pose.orientation.z, srv.response.tag_centers[i].pose.pose.pose.orientation.w);
    // }
    for (int i = 0; i < srv.response.tag_centers.size(); ++i)
    {
      std::cout<<"============ "<<"id = "<< i <<" ============"<<std::endl;
      printf("id = %d, {x, y, z} = {%.4f, %.4f, %.4f}\n", i, srv.response.tag_centers[i].x, srv.response.tag_centers[i].y, srv.response.tag_centers[i].z );
    }
  }
  else
  {
    ROS_ERROR("Failed to call service tag_centers");
    return 1;
  }

  return 0;
}