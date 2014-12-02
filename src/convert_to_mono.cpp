/*
 * \convert_to_mono.cpp
 * \takes a image and converts it to a trajectory for the robot to follow
 *
 * \author Chris Dunkers, CMU - cmdunkers@cmu.edu
 * \date October 31, 2014
 */

#include "rgb_line_follower/convert_to_mono.h"

convert_to_mono::convert_to_mono(){
	
  //the main node handle
  ros::NodeHandle nh;

  //grab the parameters
  ros::NodeHandle private_node_handle_("~");
  private_node_handle_.param<std::string>("image_topic_hsv", image_topic, "/camera/rgb/image_raw");
  private_node_handle_.param<int>("hue_min", hue_min, 100);
  private_node_handle_.param<int>("hue_max", hue_max, 120);
  private_node_handle_.param<int>("sat_min", sat_min, 100);
  private_node_handle_.param<int>("sat_max", sat_max, 200);
  private_node_handle_.param<int>("val_min", val_min, 90);
  private_node_handle_.param<int>("val_max", val_max, 255);
  
  check_int8(&hue_min);
  check_int8(&hue_max);
  check_int8(&sat_min);
  check_int8(&sat_max);
  check_int8(&val_min);
  check_int8(&val_max);
    
  //initialize the publishers and subscribers
  image_pub = nh.advertise<sensor_msgs::Image>("image_hsv", 1000);
  image_sub = nh.subscribe(image_topic, 1000, &convert_to_mono::update_image, this);
  // min and max hsv values values
  
 // cv::namedWindow(OPENCV_WINDOW);
}

convert_to_mono::~convert_to_mono(){
	cv::destroyWindow(OPENCV_WINDOW);
}

void convert_to_mono::check_int8(int * val){
	if(*val > 255){
		*val = 255;
	} 
	if(*val < 0){
		*val = 0;
	} 
}

void convert_to_mono::update_image(const sensor_msgs::Image::ConstPtr& img_msg){
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
	cv::Mat HSVImage;
	cv::Mat ThreshImage;

	//Transform the colors into HSV
	cvtColor(cv_ptr->image,HSVImage,CV_BGR2HSV);
	
	//threshold based on the tape trying to follow
	//void inRange(InputArray src, InputArray lowerb, InputArray upperb, OutputArray dst)
	inRange(HSVImage,cv::Scalar(hue_min,sat_min,val_min),cv::Scalar(hue_max,sat_max,val_max),ThreshImage);
    
    // publish modified video stream
    cv_bridge::CvImage out_msg;
    out_msg.header.frame_id   = img_msg->header.frame_id; // Same tf frame as input image
    out_msg.header.stamp = ros::Time::now();
    out_msg.encoding = sensor_msgs::image_encodings::MONO8; // black and white
    out_msg.image = ThreshImage; //cv::Mat image

    image_pub.publish(out_msg.toImageMsg());
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "convert_to_mono");

  convert_to_mono img_to_mono = convert_to_mono();
  
  ROS_INFO("Filter image for tape color node started!");	

  ros::Rate loop_rate(10);

  while (ros::ok())
  {    
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
