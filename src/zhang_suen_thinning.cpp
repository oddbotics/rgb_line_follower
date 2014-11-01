/*
 * \convert_to_mono.cpp
 * \takes a image and converts it to a trajectory for the robot to follow
 *
 * \author Chris Dunkers, CMU - cmdunkers@cmu.edu
 * \date October 31, 2014
 */

#include "rgb_line_follower/zhang_suen_thinning.h"

convert_to_line::convert_to_line(){
	
  //the main node handle
  ros::NodeHandle nh;

  //grab the parameters
  ros::NodeHandle private_node_handle_("~");
  private_node_handle_.param<std::string>("image_topic", image_topic, "/image_hsv");
    
  //initialize the publishers and subscribers
  image_pub = nh.advertise<sensor_msgs::Image>("image_thin", 1000);
  image_sub = nh.subscribe(image_topic, 1000, &convert_to_line::update_image, this);
  // min and max hsv values values
  
  cv::namedWindow(OPENCV_WINDOW);
}

convert_to_line::~convert_to_line(){
	cv::destroyWindow(OPENCV_WINDOW);
}

void convert_to_line::update_image(const sensor_msgs::Image::ConstPtr& img_msg){
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
    cv_ptr->image.copyTo(thin_image);
    frame = img_msg->header.frame_id;
    
    //run the thinning algorithm
    //cv::Mat ThinImage;
	//cv::cvtColor(cv_ptr->image, ThinImage, CV_BGR2GRAY);
	//cv::threshold(ThinImage, ThinImage, 10, 255, CV_THRESH_BINARY);
	
	//thinning(ThinImage, ThinImage); 
	
	//make the cv image to a ros message
	//cv_bridge::CvImage out_msg;
    //out_msg.header.frame_id   = img_msg->header.frame_id; // Same tf frame as input image
    //out_msg.header.stamp = ros::Time::now();
    //out_msg.encoding = sensor_msgs::image_encodings::MONO8; // black and white
    //out_msg.image = ThinImage; //cv::Mat image

    //image_pub.publish(out_msg.toImageMsg());
}

void convert_to_line::make_thin_image(){
	if (!thin_image.empty()) {	
		cv::Mat ThinImage;
		cv::cvtColor(thin_image, ThinImage, CV_BGR2GRAY);
		cv::threshold(ThinImage, ThinImage, 10, 255, CV_THRESH_BINARY);
		
		thinning(ThinImage, ThinImage); 
		
		//make the cv image to a ros message
		cv_bridge::CvImage out_msg;
		out_msg.header.frame_id   = frame; // Same tf frame as input image
		out_msg.header.stamp = ros::Time::now();
		out_msg.encoding = sensor_msgs::image_encodings::MONO8; // black and white
		out_msg.image = ThinImage; //cv::Mat image

		image_pub.publish(out_msg.toImageMsg());	
	}
}

/**
 * Perform one thinning iteration.
 * Normally you wouldn't call this function directly from your code.
 *
 * Parameters:
 * 		im    Binary image with range = [0,1]
 * 		iter  0=even, 1=odd
 */
void thinningIteration(cv::Mat& img, int iter)
{
    CV_Assert(img.channels() == 1);
    CV_Assert(img.depth() != sizeof(uchar));
    CV_Assert(img.rows > 3 && img.cols > 3);

    cv::Mat marker = cv::Mat::zeros(img.size(), CV_8UC1);

    int nRows = img.rows;
    int nCols = img.cols;

    if (img.isContinuous()) {
        nCols *= nRows;
        nRows = 1;
    }

    int x, y;
    uchar *pAbove;
    uchar *pCurr;
    uchar *pBelow;
    uchar *nw, *no, *ne;    // north (pAbove)
    uchar *we, *me, *ea;
    uchar *sw, *so, *se;    // south (pBelow)

    uchar *pDst;

    // initialize row pointers
    pAbove = NULL;
    pCurr  = img.ptr<uchar>(0);
    pBelow = img.ptr<uchar>(1);

    for (y = 1; y < img.rows-1; ++y) {
        // shift the rows up by one
        pAbove = pCurr;
        pCurr  = pBelow;
        pBelow = img.ptr<uchar>(y+1);

        pDst = marker.ptr<uchar>(y);

        // initialize col pointers
        no = &(pAbove[0]);
        ne = &(pAbove[1]);
        me = &(pCurr[0]);
        ea = &(pCurr[1]);
        so = &(pBelow[0]);
        se = &(pBelow[1]);

        for (x = 1; x < img.cols-1; ++x) {
            // shift col pointers left by one (scan left to right)
            nw = no;
            no = ne;
            ne = &(pAbove[x+1]);
            we = me;
            me = ea;
            ea = &(pCurr[x+1]);
            sw = so;
            so = se;
            se = &(pBelow[x+1]);

            int A  = (*no == 0 && *ne == 1) + (*ne == 0 && *ea == 1) + 
                     (*ea == 0 && *se == 1) + (*se == 0 && *so == 1) + 
                     (*so == 0 && *sw == 1) + (*sw == 0 && *we == 1) +
                     (*we == 0 && *nw == 1) + (*nw == 0 && *no == 1);
            int B  = *no + *ne + *ea + *se + *so + *sw + *we + *nw;
            int m1 = iter == 0 ? (*no * *ea * *so) : (*no * *ea * *we);
            int m2 = iter == 0 ? (*ea * *so * *we) : (*no * *so * *we);

            if (A == 1 && (B >= 2 && B <= 6) && m1 == 0 && m2 == 0)
                pDst[x] = 1;
        }
    }

    img &= ~marker;
}

/**
 * Function for thinning the given binary image
 *
 * Parameters:
 * 		src  The source image, binary with range = [0,255]
 * 		dst  The destination image
 */
void thinning(const cv::Mat& src, cv::Mat& dst)
{
    dst = src.clone();
    dst /= 255;         // convert to binary image

    cv::Mat prev = cv::Mat::zeros(dst.size(), CV_8UC1);
    cv::Mat diff;

    do {
        thinningIteration(dst, 0);
        thinningIteration(dst, 1);
        cv::absdiff(dst, prev, diff);
        dst.copyTo(prev);
    } 
    while (cv::countNonZero(diff) > 0);

    dst *= 255;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "thining_algorithm");

  convert_to_line img_to_line = convert_to_line();
  
  ROS_INFO("convert to line node started!");	

  ros::Rate loop_rate(10);

  while (ros::ok())
  {    
	img_to_line.make_thin_image();  
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
