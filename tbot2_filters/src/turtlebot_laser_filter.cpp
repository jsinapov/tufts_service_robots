#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#include <cmath>

ros::Publisher pub;

//how many of the left-most readings are garbage
#define NUM_FALSE_READINGS 2

void scan_callback(const sensor_msgs::LaserScan::ConstPtr& msg){
	sensor_msgs::LaserScan filtered = *msg;
	
	int num_readings = msg->ranges.size();
	
	//set last 10 readings to nan
	//in simulation, the reeadings from the end of the list correspond to the readings on the left side 
	for (int i = num_readings - NUM_FALSE_READINGS; i < num_readings; i++){
		filtered.ranges[i] = NAN;
	}
	
	pub.publish(filtered);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "turtlebot_laser_filter");
	ros::NodeHandle n("/turtlebot_laser_filter/");
	
	//get the input topic
	std::string input_topic;
	n.param<std::string>("scan_topic_in", input_topic, "/cfd");
	
	//get the output topic
	std::string output_topic;
	n.param<std::string>("scan_topic_out", output_topic, "/dfdfsda");

	//subscriber to unfiltered laser scan
	ros::Subscriber sub = n.subscribe(input_topic, 1, scan_callback);
	
	//publisher for the filtered data
	pub = n.advertise<sensor_msgs::LaserScan>(output_topic, 1);

	//spin and block
	ros::spin();
}

