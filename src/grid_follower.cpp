/*
 * \grid_follower.cpp
 * \takes an image and determines a trajectory based on the lines that it sees
 *
 * \author Chris Dunkers, CMU - cmdunkers@cmu.edu
 * \date November 14, 2014
 */

#include "rgb_line_follower/grid_follower.h"

grid_follower::grid_follower(){
	
  //the main node handle
  ros::NodeHandle nh;

  //grab the parameters
  ros::NodeHandle private_node_handle_("~");
  private_node_handle_.param<std::string>("topic_image", image_topic, "/image_denoised");
  private_node_handle_.param<int>("rows", rows, 3);
  private_node_handle_.param<int>("cols", cols, 3);
  private_node_handle_.param<int>("threshold", threshold, 75);
  private_node_handle_.param<double>("linear", linear, 1.0);
  private_node_handle_.param<double>("max_angular", max_angular, 1.0);
    
  //initialize the publishers and subscribers
  cmd_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
  image_sub = nh.subscribe(image_topic, 1000, &grid_follower::update_image, this); 
}

void grid_follower::update_image(const sensor_msgs::Image::ConstPtr& img_msg){
	cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    
    //cv_ptr->image.copyTo(image);
    //cv::threshold(image, image, 250, 255, CV_THRESH_BINARY);
    
    //---------------------------------------
    //split the image up into RxC grid
    
    cv::Mat manipImage;
    cv_ptr->image.copyTo(manipImage);
    cv::threshold(manipImage, manipImage, 250, 255, CV_THRESH_BINARY);
    
    vector<int> following_grid;
	bool row_white = false;
	vector<int> white_row;
	int r_step = manipImage.rows/rows; //number of row pixels to include 	
	int c_step = manipImage.cols/cols; //number of col pixels to include 
	int count = 0;
	int nonZero = 0;
	
	for(int c = 0; c < cols; c++){
		count = 0;
		for(int r = 0; r < rows; r++){
			// Setup a rectangle to define your region of interest
			
			//ROS_INFO("rows: %d", manipImage.rows);
			//ROS_INFO("rstep max: %d, %d", r,(r+1)*r_step);
			//ROS_INFO("cols: %d", manipImage.cols);
			//ROS_INFO("cstep max: %d, %d", c,(c+1)*c_step);
			
			//cv::Rect myROI(r*r_step, c*c_step, ((r+1)*r_step)-1, ((c+1)*c_step-1));
			cv::Rect myROI(c*c_step, (r*r_step), c_step, r_step);

			// Crop the full image to that image contained by the rectangle myROI
			// Note that this doesn't copy the data
			nonZero = countNonZero(manipImage(myROI));
			if(nonZero > 0){
				//ROS_INFO("ratio: %f", ((float)nonZero)/((float)manipImage(myROI).total()));
				if(((float)nonZero)/((float)manipImage(myROI).total()) > threshold/100.0){
					count += 1;
					//ROS_INFO("COUNTED!");
				}
			}
		}
		following_grid.push_back(count);
	}
	
	//determine when to stop
	
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
	
	ROS_INFO("left: %d", left);
	ROS_INFO("right: %d", right);
	ROS_INFO("difference: %d", left-right);
	ROS_INFO("divisor: %d", (floor(cols/2)*rows));
	ROS_INFO("%f", (float)((left-right)/(floor(cols/2)*rows)));
	
	//turn to make the sides have equal sum
	geometry_msgs::Twist command;
	command.linear.x = linear;
	command.angular.z = max_angular*((left-right)/(floor(cols/2)*rows));
	
	cmd_pub.publish(command);
	//---------------------------------------------
}

void grid_follower::find_commands(){
	//split the image up into RxC grid
	vector<int> following_grid;
	bool row_white = false;
	vector<int> white_row;
	int r_step = image.rows/rows; //number of row pixels to include 	
	int c_step = image.cols/cols; //number of col pixels to include 
	int count = 0;
	int nonZero = 0;
	for(int c = 0; c < cols; c++){
		count = 0;
		for(int r = 0; r < rows; r++){
			// Setup a rectangle to define your region of interest
			cv::Rect myROI(r*r_step, c*c_step, (r+1)*r_step, (c+1)*c_step);

			// Crop the full image to that image contained by the rectangle myROI
			// Note that this doesn't copy the data
			ROS_INFO("Hi %d", image.channels());
			nonZero = countNonZero(image(myROI));
			
			if(((float)nonZero)/((float)image(myROI).total()) > threshold/100.0){
				count += 1;
			}
		}
		following_grid.push_back(count);
	}
	
	//determine when to stop
	
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
	ROS_INFO("%f", command.angular.z);
	
	cmd_pub.publish(command);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "line_trajectory_planner");

  grid_follower follower = grid_follower();
  
  ROS_INFO("line trajectory planner node started!");	

  ros::Rate loop_rate(10);

  while (ros::ok())
  {    
	//follower.find_commands();  
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
