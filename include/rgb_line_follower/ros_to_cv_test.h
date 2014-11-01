/*
 * \ros_to_cv_test.h
 *
 * \author Chris Dunkers, CMU - cmdunkers@cmu.edu
 * \date October 31, 2014
 */

#ifndef ROS_TO_CV_TEST_H_
#define ROS_TO_CV_TEST_H_

#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <std_msgs/Empty.h>

static const std::string OPENCV_WINDOW = "Image window";

using namespace std;

class rgb_line_follower
{
  public:
    rgb_line_follower(); 
    ~rgb_line_follower();

  private:
  
	std::string image_topic;
    
    ros::Subscriber image_sub;
    ros::Publisher traj_pub;
    
    void update_image(const sensor_msgs::Image::ConstPtr& img_msg); 
};

/*!
 * Creates and runs the laser_scanner_test_node.
 *
 * \param argc argument count that is passed to ros::init
 * \param argv arguments that are passed to ros::init
 * \return 0 
 */
int main(int argc, char **argv);

#endif
