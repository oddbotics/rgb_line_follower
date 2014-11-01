/*
 * \zhang_suen_thinning.h
 *
 * \author Chris Dunkers, CMU - cmdunkers@cmu.edu
 * \date October 31, 2014
 */

#ifndef GUO_HALL_THINNING_H_
#define GUO_HALL_THINNING_H_

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

class convert_to_line
{
  public:
    convert_to_line(); 
    ~convert_to_line();
    
    void make_thin_image();

  private:
	std::string image_topic;
	
	cv::Mat thin_image;
	std::string frame;
    
    ros::Subscriber image_sub;
    ros::Publisher image_pub;

    void update_image(const sensor_msgs::Image::ConstPtr& img_msg); 
};

/**
 * Perform one thinning iteration.
 * Normally you wouldn't call this function directly from your code.
 *
 * @param  im    Binary image with range = 0-1
 * @param  iter  0=even, 1=odd
 */
void thinningGuoHallIteration(cv::Mat& im, int iter);

/**
 * Function for thinning the given binary image
 *
 * @param  im  Binary image with range = 0-255
 */
void thinningGuoHall(cv::Mat& im);

/*!
 * Creates and runs the laser_scanner_test_node.
 *
 * \param argc argument count that is passed to ros::init
 * \param argv arguments that are passed to ros::init
 * \return 0 
 */
int main(int argc, char **argv);

#endif
