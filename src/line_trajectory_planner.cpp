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
  private_node_handle_.param<std::string>("topic_lines", lines_topic, "/lines");
  private_node_handle_.param<double>("threshold", threshold, 1.0);
    
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
	int sx,sy,ex,ey;
    for(int i = 0; i < lines_msg.start_x.size(); i++){
		sx = lines_msg.start_x[i];
		sy = lines_msg.start_y[i];
		ex = lines_msg.end_y[i];
		ey = lines_msg.end_y[i];
		lines.push_back(Line(sx,sy,ex,ey);
	}
}

void line_trajectory_planner::find_trajectory(){
	if(!lines.empty()){
		
		//organize by gradient value;
		
		std::vector<std::vector<Line>> grad_lines;
		std::vector<Line> path;
		int l = 0;
		//find lines with similiar gradients
		
		for(int i = 0; i < lines.size(); i++){
			grad_lines[l].push_back(lines[i]);
			for(int j = 0; j < lines.size(); j++){
				if(!lines[i].equals(lines[j])){
					if(abs(lines[i].gradient - lines[j].gradient) < threshold){
						grad_lines[l].push_back(lines[j]);
					}
				}
			}
			l++;
		}
		
		//find the X point to start (min) and finish (max)
		//find the Y point to start (min) and finish (max)
		int sum_sx, sum_sy, sum_ex, sum_ey, sum_grad;
		for(int i = 0; i < lines.size(); i++){
			if(grad_lines[i].size() > 1){
				sum_sx = 0;
				sum_sy = 0;
				sum_ex = 0;
				sum_ey = 0;
				for(int j = 0; j < grad_lines[i].size(); j++){
					sum_sx += grad_lines[i][j].start_x;
					sum_sy += grad_lines[i][j].start_y;
					sum_ex += grad_lines[i][j].end_x;
					sum_ey += grad_lines[i][j].end_y;
					sum_grad += grad_lines[i][j].gradient
				}
				sum_sx /= grad_lines[i].size();
				sum_sy /= grad_lines[i].size();
				sum_ex /= grad_lines[i].size();
				sum_ey /= grad_lines[i].size();
				sum_grad /= grad_lines[i].size();
				path.push_back(Line(sum_sx, sum_sy, sum_ex, sum_ey);
			}
		}
			
		
		//based on the start and finsh of the lines find a set of poses to satisfy the line
		// at the start and end of each line pointing along the gradient 
		
		// based on math of the camera from the pan tilt info 
		// based on odometry
		// find the global positions of the pose desired based on the start and end of the lines
				
	}
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
