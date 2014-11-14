/*
 * \grid_follower.cpp
 *
 * \author Chris Dunkers, CMU - cmdunkers@cmu.edu
 * \date November 14, 2014
 */

#ifndef GRID_FOLLOWER_H_
#define GRID_FOLLOWER_H_

#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "geometry_msgs/Twist.h"
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <std_msgs/Empty.h>
#include "opencv2/core/core.hpp"
#include "oddbot_msgs/Lines.h"

static const std::string OPENCV_WINDOW = "Image window";

using namespace std;
using namespace cv;

class line_trajectory_planner
{
  public:
    line_trajectory_planner(); 
    
    void find_commands();

  private:
	std::string image_topic;
	int rows;
	int cols;
	int threshold;
    double linear;
    double max_angular;
    
    cv::Mat image;
    
    ros::Subscriber image_sub;
    ros::Publisher cmd_pub;

    void update_image(const sensor_msgs::Image::ConstPtr& img_msg);
};

/*!
 * Creates and runs the line_trajectory_planner
 *
 * \param argc argument count that is passed to ros::init
 * \param argv arguments that are passed to ros::init
 * \return 0 
 */
int main(int argc, char **argv);

#endif
