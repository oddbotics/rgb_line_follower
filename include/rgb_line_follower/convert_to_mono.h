/*
 * \convert_to_mono.h
 *
 * \author Chris Dunkers, CMU - cmdunkers@cmu.edu
 * \date October 31, 2014
 */

#ifndef CONVERT_TO_MONO_H_
#define CONVERT_TO_MONO_H_

#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <std_msgs/Empty.h>
#include "opencv2/core/core.hpp"

static const std::string OPENCV_WINDOW = "Image window";

using namespace std;

class convert_to_mono
{
  public:
    convert_to_mono(); 
    ~convert_to_mono();

  private:
	int hue_min;
	int hue_max;
	int sat_min;
	int sat_max;
	int val_min;
	int val_max;	
	
	std::string image_topic;
    
    ros::Subscriber image_sub;
    ros::Publisher image_pub;
    
    void check_int8(int * val);
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
