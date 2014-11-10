/*
 * \denoise.cpp
 * \takes a image and converts it to a trajectory for the robot to follow
 *
 * \author Eric Danziger, CMU - ericdanziger@cmu.edu
 * \date November 9, 2014
 */

#include "rgb_line_follower/denoise.h"



denoise::denoise(){
	
  //the main node handle
  ros::NodeHandle nh;

  //grab the parameters
  ros::NodeHandle private_node_handle_("~");
  private_node_handle_.param<std::string>("image_topic_hsv", image_topic, "/camera/rgb/image_hsv");
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
  image_pub = nh.advertise<sensor_msgs::Image>("image_denoised", 1000);
  image_sub = nh.subscribe(image_topic, 1000, &denoise::update_image, this);
  // min and max hsv values values
  
  cv::namedWindow(OPENCV_WINDOW);
}

denoise::~denoise(){
	cv::destroyWindow(OPENCV_WINDOW);
}

void denoise::check_int8(int * val){
	if(*val > 255){
		*val = 255;
	} 
	if(*val < 0){
		*val = 0;
	} 
}

void denoise::update_image(const sensor_msgs::Image::ConstPtr& img_msg){
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
	cv::Mat DenoiseImage;

	//Transform the colors into HSV
	cvtColor(cv_ptr->image,HSVImage,CV_BGR2HSV);
	
        //Denoise image
        //void fastNlMeansDenoising(InputArray src, OutputArray dst, float h=3, int templateWindowSize=7, 
	//int searchWindowSize=21 )
	fastNlMeansDenoising(HSVImage, DenoiseImage);

	//threshold based on the tape trying to follow
	//void inRange(InputArray src, InputArray lowerb, InputArray upperb, OutputArray dst)
	//inRange(HSVImage,cv::Scalar(100,100,90),cv::Scalar(120,200,255),DenoiseImage);
    
    // publish modified video stream
    cv_bridge::CvImage out_msg;
    out_msg.header.frame_id   = img_msg->header.frame_id; // Same tf frame as input image
    out_msg.header.stamp = ros::Time::now();
    out_msg.encoding = sensor_msgs::image_encodings::MONO8; // black and white
    out_msg.image = DenoiseImage; //cv::Mat image

    image_pub.publish(out_msg.toImageMsg());
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "denoise");

  denoise denoise_img = denoise();
  
  ROS_INFO("Filter image for tape color node started!");	

  ros::Rate loop_rate(10);

  while (ros::ok())
  {    
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
