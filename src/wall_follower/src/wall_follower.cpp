// Released under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
// Author: Claude Sammut
// Last Modified: 2024.10.14

// Use this code as the basis for a wall follower

#include "wall_follower/wall_follower.hpp"

#include <memory>

using namespace std::chrono_literals;

//Class Definition and Constructor Method
WallFollower::WallFollower()
: Node("wall_follower_node")
{
	/************************************************************
	** Initialise variables
	************************************************************/
	for (int i = 0; i < 12; i++)
	{
		scan_data_close_[i] = 0.0; //array of 12 points. init all cells to 0.
		scan_data_avg_[i] = 0.0;
	}

	robot_pose_ = 0.0;
	near_start = false; // Variable to tell if back at start

	/************************************************************
	** Initialise ROS publishers and subscribers
	************************************************************/
	auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

	// Initialise publishers
	//Publishes to command vel
	cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", qos);
	//qos = quality of service. Most network traffic is TCP with acknowledgement. UDP is just broadcast.
	//Most publishers will be UDP. QOS keeps a buffer of 10 to ensure that we circumvent perfect alignment.

	// Initialise subscribers
	scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>( //create sub to laser scan.
		"scan", \
		rclcpp::SensorDataQoS(), \
		std::bind(
			&WallFollower::scan_callback, \
			this, \
			std::placeholders::_1)); /// placeholders argument.
	odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
		"odom", qos, std::bind(&WallFollower::odom_callback, this, std::placeholders::_1));

	// Subscriptions work by:
	// When we receive a new laser scan, we want to call our own function to do a laser scan.
	// The subscriber knoews to use the callback when we receive a laser scan. We also have a
	// callback for odometry. (odom_callback). These are set up to trigger when sensor info is rcvd.

	/************************************************************
	** Initialise ROS timers
	************************************************************/
	update_timer_ = this->create_wall_timer(20ms, std::bind(&WallFollower::update_callback, this));
	// Callback created on a timed mechanism. Callback triggers in response to timer, not in response 
	//to sensor input.

	RCLCPP_INFO(this->get_logger(), "Wall follower node has been initialised");
}

WallFollower::~WallFollower()//fns in a class with the class name prefixed by tilde are destructors.
{
	RCLCPP_INFO(this->get_logger(), "Wall follower node has been terminated");
}

/********************************************************************************
** Callback functions for ROS subscribers
********************************************************************************/

#define START_RANGE	0.2

void WallFollower::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
	static bool first = true;
	static bool start_moving = true;

	tf2::Quaternion q(
		msg->pose.pose.orientation.x,
		msg->pose.pose.orientation.y,
		msg->pose.pose.orientation.z,
		msg->pose.pose.orientation.w);
	tf2::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);

	robot_pose_ = yaw;

	double current_x =  msg->pose.pose.position.x;
	double current_y =  msg->pose.pose.position.y;
	if (first)
	{
		start_x = current_x;
		start_y = current_y;
		first = false;
	}
	else if (start_moving)
	{
		if (fabs(current_x - start_x) > START_RANGE || fabs(current_y - start_y) > START_RANGE)
			start_moving = false;
	}
	else if (fabs(current_x - start_x) < START_RANGE && fabs(current_y - start_y) < START_RANGE)
	{
		fprintf(stderr, "Near start!!\n");
		near_start = true;
		first = true;
		start_moving = true;
	}
}

#define BEAM_WIDTH 15


/* 
This takes a range of points, and then just finds the minimum.
This seems naive? What if we get a fkd value? The robot just shits itself and turns?
Maybe we should take the average of this
*/
void WallFollower::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
	uint16_t scan_angle[12] = {0, 30, 60, 90, 120, 150, 180, 210, 240, 270, 300, 330};

	double closest = msg->range_max;
	//this whole loop is just for the special case of forward
	auto fsum = 0;
for (int angle = 360-BEAM_WIDTH; angle < 360; angle++) {
		fsum += msg->ranges.at(angle);
		if (msg->ranges.at(angle) < closest)
			closest = msg->ranges.at(angle);
	}
	for (int angle = 0; angle < BEAM_WIDTH; angle++) {
		fsum += msg->ranges.at(angle);
		if (msg->ranges.at(angle) < closest)
			closest = msg->ranges.at(angle);
	}
	scan_data_close_[0] = closest;
	scan_data_avg_[0] = fsum / 30;

	//Rest of the elements are in this loop

	for (int i = 1; i < 12; i++)
	{	auto osum = 0;
		closest = msg->range_max;
		for (int angle = scan_angle[i]-BEAM_WIDTH; angle < scan_angle[i]+BEAM_WIDTH; angle++) {
			osum += msg->ranges.at(angle);
			if (msg->ranges.at(angle) < closest)
				closest = msg->ranges.at(angle);
		}
		scan_data_close_[i] = closest;
		scan_data_avg_[i] = osum / 30;
	}
}

void WallFollower::update_cmd_vel(double linear, double angular)
{
	geometry_msgs::msg::Twist cmd_vel;
	cmd_vel.linear.x = linear;
	cmd_vel.angular.z = angular;

	cmd_vel_pub_->publish(cmd_vel);
}

/********************************************************************************
** Update functions
********************************************************************************/

bool pl_near;



/*
* This is the main part of the code we will need to modify.
*/
void WallFollower::update_callback()
// Front Left = Front dominant left diagonal
// Left Front = Left Dominant Front Diagonal etc

{
	// RCLCPP_INFO(this->get_logger(), "Reached update callback");
	// --- State (keep across calls) ---
	static double current_ang_vel = 0.0;
	static double current_lin_vel = 0.0;
	double integ_smooth_ang = 0.7;   	// 0 < smoothing \leq 1. Larger = More influence from previous heading
	double diff_smooth_ang = 0.1;		// 0 < smoothing \leq 1. Larger = More influence from new ideal heading		

	double integ_smooth_lin = 0.3;
	double diff_smooth_lin = 0.5;


	double target_lin = 0.20;
	double target_ang = 0.0;

	static bool right_state = false;
	static bool left_state = false;


	// RCLCPP_INFO(this->get_logger(), std::to_string(scan_data_close_[LEFT_FRONT]).c_str());


	if (near_start) {
	    update_cmd_vel(0.0, 0.0);
	    exit(0);
	}

	// +ve is Left
	// -ve is right
	else if (right_state) {
		update_cmd_vel(0,-0.6);
		// std::cout << "Right state: " << scan_data_close_[FRONT] << std::endl; 
		if (scan_data_close_[FRONT] > 0.8 || 
			((scan_data_close_[FRONT] > 0.8 || scan_data_close_[FRONT] == 0) && 
			(scan_data_close_[FRONT_LEFT] > 0.45 || scan_data_close_[FRONT_LEFT] == 0) && 
			(scan_data_close_[FRONT_RIGHT] > 0.45 || scan_data_close_[FRONT_RIGHT] == 0))) {
			right_state = false;
			// std::cout << "EXITED OUT OF RIGHT STATE " << scan_data_close_[FRONT] << std::endl; 
			// std::cout << "EXIT FRONT-left: " << scan_data_close_[FRONT_LEFT] << std::endl; 
			// std::cout << "EXIT FRONT-right: " << scan_data_close_[FRONT_RIGHT] << std::endl; 
		}

		return;
	}
	else if (left_state) {
		//(lin_vel, ang_vel)
		// RCLCPP_INFO(this->get_logger(), "In Left State");
		target_ang = 1.2;
		target_lin = 0.15;
		current_ang_vel = integ_smooth_ang * current_ang_vel + diff_smooth_ang * target_ang;
		current_lin_vel = integ_smooth_lin * current_lin_vel + diff_smooth_lin * target_lin;
		update_cmd_vel(current_lin_vel, current_ang_vel);
		
		// Once left wall detected: 
		if ((scan_data_close_[LEFT_FRONT] < 0.5 && scan_data_close_[LEFT_FRONT] != 0) ||
			(scan_data_close_[FRONT_LEFT] < 0.5 && scan_data_close_[FRONT_LEFT] != 0 ) ||
			(scan_data_close_[FRONT] < 0.5 && scan_data_close_[FRONT] != 0 ) 
		)	
		{
			// RCLCPP_INFO(this->get_logger(), "EXIT LEFT STATE");
			// RCLCPP_INFO(this->get_logger(), std::to_string(scan_data_close_[FRONT_LEFT]).c_str());
			
			left_state = false;
		}
		return;
	}
  	else if ((scan_data_close_[FRONT] < 0.42 && scan_data_close_[FRONT] != 0 ) || 
		(scan_data_close_[FRONT_LEFT] < 0.3 && scan_data_close_[FRONT_LEFT] != 0) || 
		(scan_data_close_[FRONT_RIGHT] < 0.3 && scan_data_close_[FRONT_RIGHT] != 0 )) // Something in front of the robot
	{	
		
		// std::cout << "FRONT: " << scan_data_close_[FRONT] << std::endl; 
		// std::cout << "FRONT-left: " << scan_data_close_[FRONT_LEFT] << std::endl; 
		// std::cout << "FRONT-right: " << scan_data_close_[FRONT_RIGHT] << std::endl; 
		right_state = true;
		// target_ang = -1.2, target_lin = 0.0; // turn right
	}
	else if (scan_data_close_[FRONT_RIGHT] < 0.4 && scan_data_close_[FRONT_RIGHT] > 0) {
	// std::cout << "CORRECTIVE LEFT: " << scan_data_close_[FRONT_RIGHT] << std::endl;
	target_ang = 1.3, target_lin = 0.1;
	}
	else if (scan_data_close_[FRONT_LEFT] < 0.4 && scan_data_close_[FRONT_LEFT] > 0) {
		// std::cout << "CORRECTIVE RIGHT: " << scan_data_close_[FRONT_LEFT] << std::endl;
		target_ang = -1.2, target_lin = 0.1;
	}
	
	else if (scan_data_close_[LEFT_FRONT] > 0.6 || (scan_data_close_[LEFT_FRONT] == 0 && scan_data_close_[LEFT] !=0 )) {
		// std::cout << "LEFT_FRONT: " << scan_data_close_[LEFT_FRONT] << std::endl;
		left_state = true;
		// target_ang = 1.3, target_lin = 0.15;
	}
	// else if (scan_data_close_[FRONT_LEFT] > 1.6 || scan_data_close_[FRONT_LEFT] == 0) {
	// 	target_ang = 1.2, target_lin = 0.15;
	// }  
	
	// // else if (scan_data_close_[LEFT_FRONT] > 0.65) target_ang = 1.8, target_lin = 0.2;
	// else if (scan_data_close_[FRONT_LEFT] < 0.6 && scan_data_close_[FRONT_LEFT] > 0) {
		// 	std::cout << "Corrective Right: " << scan_data_close_[FRONT_LEFT] << std::endl;
		// 	target_ang = -1.2, target_lin = 0.2;
		// }
		// else if (scan_data_close_[FRONT_RIGHT] < 0.5) {
	// 	std::cout << "Corrective Left: " << scan_data_close_[FRONT_LEFT] << std::endl;
	// 	target_ang =  1.2, target_lin = 0.2;
	// }

		// CORRECTIVE LEFT AND RIGHTS
		

	// --- Smooth angular velocity ---
	current_ang_vel = integ_smooth_ang * current_ang_vel + diff_smooth_ang * target_ang;
	current_lin_vel = integ_smooth_lin * current_lin_vel + diff_smooth_lin * target_lin;

	// current_ang_vel = (1.0 - smooth2) * current_ang_vel + smooth2 * target_ang;

	// --- Apply command ---
	update_cmd_vel(current_lin_vel, current_ang_vel);

}


/*******************************************************************************
** Main
*******************************************************************************/
/*
* RCLCPP Is the main class to interface with ROS using cpp.
* Spin says to just start the node and begin loop - listen and publish
* shutdown cleans stuff up
*/
int main(int argc, char ** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<WallFollower>()); //Spin 
	rclcpp::shutdown();

	return 0;
}