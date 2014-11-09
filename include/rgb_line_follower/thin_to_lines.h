/*
 * \thin_to_lines.h
 *
 * \author Chris Dunkers, CMU - cmdunkers@cmu.edu
 * \date October 31, 2014
 */

#ifndef THIN_TO_LINES_H_
#define THIN_TO_LINES_H_

#include <ros/ros.h>
#include "sensor_msgs/Image.h"
#include "oddbot_msgs/Lines.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <std_msgs/Empty.h>
#include "opencv2/core/core.hpp"

static const std::string OPENCV_WINDOW = "Image window";

using namespace cv;
using namespace std;

class thin_to_lines
{
  public:
    thin_to_lines(); 
    ~thin_to_lines();

  private:
	int rho;
	double theta;
	int threshold; 	
	int minLineLength;
	int maxLineGap;	
	
	std::string image_topic;
    
    ros::Subscriber image_sub;
    ros::Publisher lines_pub;
    
    void check_int8(int * val);
    void update_lines(const sensor_msgs::Image::ConstPtr& img_msg); 
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
