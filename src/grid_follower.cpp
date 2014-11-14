/*
 * \convert_to_mono.cpp
 * \takes a image and converts it to a trajectory for the robot to follow
 *
 * \author Chris Dunkers, CMU - cmdunkers@cmu.edu
 * \date October 31, 2014
 */

#include "rgb_line_follower/line_trrajectory_planner.h"

line_trajectory_planner::line_trajectory_planner(){
	
  //the main node handle
  ros::NodeHandle nh;

  //grab the parameters
  ros::NodeHandle private_node_handle_("~");
  private_node_handle_.param<std::string>("topic_lines", lines_topic, "/denoise");
  private_node_handle_.param<int>("rows", rows, 3);
  private_node_handle_.param<int>("cols", cols, 3);
  private_node_handle_.param<int>("threshold", threshold, 75);
  private_node_handle_.param<double>("linear", linear, 1.0);
  private_node_handle_.param<double>("max_angular", max_angular, 1.0);
    
  //initialize the publishers and subscribers
  image_pub = nh.advertise<oddbot_msgs::Lines>("image_thin", 1000);
  image_sub = nh.subscribe(image_topic, 1000, &convert_to_line::update_image, this);
  // min and max hsv values values
  
  cv::namedWindow(OPENCV_WINDOW);
}

line_trajectory_planner::~line_trajectory_planner(){
	cv::destroyWindow(OPENCV_WINDOW);
}

void line_trajectory_planner::update_lines(const oddbot_msgs::Lines::ConstPtr& lines_msg){
	//store image
}

void line_trajectory_planner::find_trajectory(){
	//split the image up into RxC grid
	vector<int> following_grid = ;
	int r_step = image.rows/rows; //number of row pixels to include 	
	int c_step = image.cols/cols; //number of col pixels to include 
	int count = 0;
	for(int c = 0; c < cols; c++){
		count = 0;
		for(int r = 0; r < rows; r++){
			// Setup a rectangle to define your region of interest
			cv::Rect myROI(r*r_step, c*c_step, (r+1)*r_step, (c+1)*c_step);

			// Crop the full image to that image contained by the rectangle myROI
			// Note that this doesn't copy the data
			cv::Mat croppedImage = image(myROI);
			
			if(cvCountNonZero(croppedImage)/croppedImage.total() > threshold){
				count += 1;
			}
		}
		following_grid.push_back(count);
	}
		
	
	//figure out if one side has more black than white or vice versa
	int left = 0;
	int right = 0;
	int upper_left = floor(cols/2);
	int lower_right = ceil(cols/2);
	
	for(int c = 0; c < cols; c++){
		if(c < upper_left){
			left += following_grid[c];
		} else if(c >= lower_right){
			right += following_grid[c];
		}
	}
	
	//turn to make the sides have equal sum
	geometry_msgs::Twist command;
	command.linear.x = linear;
	command.angular.z = max_angular*((left-right)/(floor(cols/2)*rows));
	
	cmd_pub.publish(command);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "line_trajectory_planner");

  line_trajectory_planner lines_to_traj = line_trajectory_planner();
  
  ROS_INFO("line trajectory planner node started!");	

  ros::Rate loop_rate(10);

  while (ros::ok())
  {    
	lines_to_traj.find_trajectory();  
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
