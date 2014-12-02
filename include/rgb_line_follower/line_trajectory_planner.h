/*
 * \line_trajectory_planner.h
 *
 * \author Chris Dunkers, CMU - cmdunkers@cmu.edu
 * \date October 31, 2014
 */

#ifndef LINE_TRAJECTORY_PLANNER_H_
#define LINE_TRAJECTORY_PLANNER_H_

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
#include "oddbot_msgs/Lines.h"

static const std::string OPENCV_WINDOW = "Image window";

using namespace std;

class line_trajectory_planner
{
  public:
    line_trajectory_planner(); 
    ~line_trajectory_planner();
    
    void find_trajectory();

  private:
	std::string lines_topic;
	std::vector<Line> lines;
	double threshold;
    
    ros::Subscriber lines_sub;
    ros::Publisher image_pub;

    void update_lines(const sensor_msgs::Image::ConstPtr& img_msg); 
};

/*!
 * Creates and runs the line_trajectory_planner
 *
 * \param argc argument count that is passed to ros::init
 * \param argv arguments that are passed to ros::init
 * \return 0 
 */
int main(int argc, char **argv);

class Line
{
	public:
		Line(int start_x, int start_y, int end_x, int end_y){
			start_x = start_x;
			start_y = start_y;
			end_x = end_x;
			end_y = end_y;
			gradient = (end_y-start_y)/(end_x-start_x);
		}
		~Line();
		float get_gradient(){
			return gradient;
		}
		bool equals(Line l){
			if(start_x == l.start_x){
				if(start_y == l.start_y){
					if(end_x == l.end_x){
						if(end_y == l.end_y){
							return true;
						}
					}
				}
			}
			return false;
		}		
		
		int start_x;
		int start_y;
		int end_x;
		int end_y;
		double gradient; 
};
		
	
	


#endif
