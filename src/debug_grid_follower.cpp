/*
 * \grid_follower.cpp
 * \takes an image and determines a trajectory based on the lines that it sees
 *
 * \author Chris Dunkers, CMU - cmdunkers@cmu.edu
 * \date November 14, 2014
 */

#include "rgb_line_follower/debug_grid_follower.h"

grid_follower::grid_follower(){
	
  //the main node handle
  ros::NodeHandle nh;

  //grab the parameters
  ros::NodeHandle private_node_handle_("~");
  private_node_handle_.param<std::string>("image_topic_bw", image_topic, "/image_denoised");
  private_node_handle_.param<int>("rows", rows, 3);
  private_node_handle_.param<int>("cols", cols, 3);
  private_node_handle_.param<int>("threshold", threshold, 75);
  private_node_handle_.param<double>("linear", linear, 1.0);
  private_node_handle_.param<double>("max_angular", max_angular, 1.0);
    
  //initialize the publishers and subscribers
  cmd_pub = nh.advertise<geometry_msgs::Twist>("cmd_follow_line", 1000);
  image_sub = nh.subscribe(image_topic, 1000, &grid_follower::update_image, this); 
}

void grid_follower::update_image(const sensor_msgs::Image::ConstPtr& img_msg){
	cv_bridge::CvImagePtr cv_ptr, cv_test;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);
      cv_test = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    
    cv::Mat manipImage;
    cv_ptr->image.copyTo(manipImage);
    cv::threshold(manipImage, manipImage, 250, 255, CV_THRESH_BINARY);
    
	vector<int> white_row;
	int r_step = manipImage.rows/rows; //number of row pixels to include 	
	int c_step = manipImage.cols/cols; //number of col pixels to include 
	int count = 0;
	int nonZero = 0;
	
	for(int c = 0; c < cols; c++){
		count = 0;
		for(int r = 0; r < rows; r++){
			// Setup a rectangle to define your region of interest
			
			//cv::Rect myROI(r*r_step, c*c_step, ((r+1)*r_step)-1, ((c+1)*c_step-1));
			cv::Rect myROI(c*c_step, (r*r_step), c_step, r_step);
			
			cv::rectangle(cv_test->image, myROI, Scalar( 0, 255, 255 ), 1, 8, 0 );

		}
	}
	// Update GUI Window
    cv::imshow(OPENCV_WINDOW, cv_test->image);
    cv::waitKey(3);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "debug_follower");

  grid_follower follower = grid_follower();
  
  ROS_INFO("debug node started!");	

  ros::Rate loop_rate(10);

  while (ros::ok())
  {      
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
