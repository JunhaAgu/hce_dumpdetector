#include "ros/ros.h"
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
  ros::ServiceClient client = n.serviceClient<hce_msgs::CallDumpDetector>("/service_dump_detector");
  hce_msgs::CallDumpDetector srv;
  // srv.request.a = atoll(argv[1]);
  // srv.request.b = atoll(argv[2]);
  // srv.response.tag_centers.reserve(4);

  Mat img = imread("/home/junhakim/image.png", 1);
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

  srv.request.fx = 893.4800446716952;
  srv.request.fy = 893.2138912525431;
  srv.request.cx = 529.8848649123668;
  srv.request.cy = 395.1783894490890;

  srv.request.dist_k1 = 0.155682618969427;  //Radial Distortion
  srv.request.dist_k2 = 0.118825328777770;  //Radial Distortion
  srv.request.dist_p1 = 0.0;                //Tangential Distortion
  srv.request.dist_p2 = 0.0;                //Tangential Distortion

  // cv_bridge::CvImagePtr cv_ptr;
  // cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::RGB8);
  // cv::Mat image = cv_ptr->image;

  // cv::imwrite("/home/junhakim/service_test_results/image_request.png", image);

  if (client.call(srv))
  {

    ROS_INFO("tag detection success?: %s", srv.response.success ? "Yes" : "No");
    
    if (srv.response.success == true)
    {
    for (int i = 0; i < srv.response.tag_centers.size(); ++i)
    {
      // ROS_INFO("%.4f", srv.response.tag_centers.at(i).x);
      std::cout<<"============ "<<"id = "<< i <<" ============"<<std::endl;
      printf("id = %d, {x, y, z} = {%.8f, %.8f, %.8f}\n", i, srv.response.tag_centers[i].x, srv.response.tag_centers[i].y, srv.response.tag_centers[i].z );
    }
    }
  }
  else
  {
    ROS_ERROR("Failed to call service tag_centers");
    return 1;
  }

  return 0;
}