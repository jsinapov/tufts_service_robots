// move_k_points.cpp

#include <ros/ros.h>
#include <tf/tf.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_listener.h>
#include <actionlib/server/simple_action_server.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PointStamped.h"

// Function declarations 
void update_endpoints(const geometry_msgs::PointStamped::ConstPtr& msg); 
void move_turtlebot(double x, double y, double yaw); 
void callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg); 

int num_locations;
std::vector<float> locations_x;
std::vector<float> locations_y;
int seq_counter = 0;

int main(int argc, char **argv)
{ 
	ros::init(argc, argv, "patrolling_turtle"); 
	ros::NodeHandle n;

	// Subscribes to two topics: the points given by the user and the position of the robot
	ros::Subscriber sub = n.subscribe("/clicked_point", 1000, update_endpoints); 

	ros::Subscriber sub2 = n.subscribe("/amcl_pose", 1000, callback); 

	std::cout << "Number of locations: ";
	std::cin >> num_locations;
	while (num_locations <= 1) {
		ROS_INFO("Please enter a number greater than 1\n");
		std::cin >> num_locations;
	}							

	ros::spin();
}

void update_endpoints(const geometry_msgs::PointStamped::ConstPtr& msg)
{
	if (seq_counter >= 0 && seq_counter < num_locations - 1) {
		locations_x.push_back(msg -> point.x);
		locations_y.push_back(msg -> point.y);
		seq_counter++;
	} else if (seq_counter == num_locations - 1) {
		locations_x.push_back(msg -> point.x);
		locations_y.push_back(msg -> point.y);
		seq_counter++;

		ROS_INFO("Thank you! Beginning patrol now...\n"); 
		move_turtlebot(locations_x[0], locations_y[0], 0.0); 
	}
}

void callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
	int goal_index = 0;

	if (locations_x.size() < num_locations) {
		ROS_INFO("Please use R-Viz to 'Publish' the specified number of points\n");
		return;
	}

	while (ros::ok()) {
		move_turtlebot(locations_x[goal_index], locations_y[goal_index], 0.0);

		goal_index++;

		if (goal_index >= num_locations) {
			goal_index = 0;
		}
	}
}

void move_turtlebot(double x, double y, double yaw)
{
	actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);

        ac.waitForServer();
        move_base_msgs::MoveBaseGoal goal;

        goal.target_pose.header.stamp = ros::Time::now();
     	goal.target_pose.header.frame_id = "/map";

	goal.target_pose.pose.position.x = x;
	goal.target_pose.pose.position.y = y;
	goal.target_pose.pose.position.z = 0.0;  
	goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
	ac.sendGoal(goal);
	ac.waitForResult(); 

}
