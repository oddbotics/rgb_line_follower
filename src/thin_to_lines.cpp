/*
 * \thin_to_lines.cpp
 * \takes a image and converts it to a trajectory for the robot to follow
 *
 * \author Chris Dunkers, CMU - cmdunkers@cmu.edu
 * \date October 31, 2014
 */

#include "rgb_line_follower/thin_to_lines.h"

thin_to_lines::thin_to_lines(){
	
  //the main node handle
  ros::NodeHandle nh;

  //grab the parameters
  ros::NodeHandle private_node_handle_("~");
  private_node_handle_.param<std::string>("image_topic_lines", image_topic, "/image_thin");
  private_node_handle_.param<int>("rho", rho, 1);
  private_node_handle_.param<double>("theta", theta, CV_PI/180);
  private_node_handle_.param<int>("threshold", threshold, 50);
  private_node_handle_.param<int>("minLineLength", minLineLength, 50);
  private_node_handle_.param<int>("maxLineGap", maxLineGap, 10);
  private_node_handle_.param<bool>("display_lines", display, false);
    
  //initialize the publishers and subscribers
  lines_pub = nh.advertise<oddbot_msgs::Lines>("lines", 1000);
  image_sub = nh.subscribe(image_topic, 1000, &thin_to_lines::update_lines, this);
  // min and max hsv values values
  
  cv::namedWindow(OPENCV_WINDOW);
}

thin_to_lines::~thin_to_lines(){
	cv::destroyWindow(OPENCV_WINDOW);
}

void thin_to_lines::update_lines(const sensor_msgs::Image::ConstPtr& img_msg){
	ROS_INFO("GETTING LINES");
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
	
    //declare opencv images
	cv::Mat LinesImage;
	cv::Mat displayImage;

	//Transform the thin into lines
	vector<Vec4i> lines;
	//lines is a vector that will store the parameters (x_start, y_start, x_end, y_end) 
	//of the detected lines
	
	Canny(cv_ptr->image, LinesImage, 50, 200, 3);	
	cvtColor(LinesImage, displayImage, CV_GRAY2BGR);
	HoughLinesP(LinesImage, lines, rho, theta, threshold, minLineLength, maxLineGap );
	
	if(display == true){
		//now display the result by drawing the lines
		for( size_t i = 0; i < lines.size(); i++ )
		{
		  Vec4i l = lines[i];
		  line(displayImage, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 3, CV_AA);
		}    	
		imshow("source", cv_ptr->image);
		imshow("detected lines", displayImage);
		cv::waitKey(3);
    }
	
	//convert lines vector to Lines message type
	oddbot_msgs::Lines lines_msg;
	lines_msg.pixel_height_of_img = cv_ptr->image.rows;
	lines_msg.pixel_width_of_img = cv_ptr->image.cols;
	for( size_t i = 0; i < lines.size(); i++ )
	{
	  Vec4i l = lines[i];
	  lines_msg.start_x.push_back(l[0]);
	  lines_msg.start_y.push_back(l[1]);
	  lines_msg.end_x.push_back(l[2]);
	  lines_msg.end_y.push_back(l[3]);
	}    	


    lines_pub.publish(lines_msg);
    ROS_INFO("GOT LINES");
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "thin_to_lines");

  thin_to_lines img_to_lines = thin_to_lines();
  
  ROS_INFO("Filter image for thin to lines has started!");	

  ros::Rate loop_rate(10);

  while (ros::ok())
  {    
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
