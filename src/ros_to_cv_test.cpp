/*
 * \ros_to_cv_test.cpp
 * \takes a image and converts it to a cv image and displays it
 *
 * \author Chris Dunkers, CMU - cmdunkers@cmu.edu
 * \date October 31, 2014
 */

#include "rgb_line_follower/ros_to_cv_test.h"

rgb_line_follower::rgb_line_follower(){
	
  //the main node handle
  ros::NodeHandle nh;

  //grab the parameters
  ros::NodeHandle private_node_handle_("~");
  private_node_handle_.param<string>("image_topic", image_topic, "/camera/rgb/image_raw");
  //private_node_handle_.param<double>("grid_y", grid_y, 12.0);
  //private_node_handle_.param<double>("resolution", resolution, 0.1);
  
  //initialize the publishers and subscribers
  traj_pub = nh.advertise<std_msgs::Empty>("traj", 1000);
  image_sub = nh.subscribe(image_topic, 1000, &rgb_line_follower::update_image, this);
  
  cv::namedWindow(OPENCV_WINDOW);
}

rgb_line_follower::~rgb_line_follower(){
	cv::destroyWindow(OPENCV_WINDOW);
}


void rgb_line_follower::update_image(const sensor_msgs::Image::ConstPtr& img_msg){
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    cv::circle(cv_ptr->image, cv::Point(cv_ptr->image.cols/2, cv_ptr->image.rows/2), 10, CV_RGB(255,0,0));
    
    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);
    
    // Output modified video stream
    //image_pub_.publish(cv_ptr->toImageMsg());
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ros_to_cv_test");

  rgb_line_follower line_folower = rgb_line_follower();
  
  ROS_INFO("ros to cv test started!");	

  ros::Rate loop_rate(10);

  while (ros::ok())
  {    
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
