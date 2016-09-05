#include "ros/ros.h"
#include "serial_mavlink.h"
using namespace std;
int main(int argc, char** argv)
{
	ros::init(argc, argv, "serial_mavlink_node");
	ros::NodeHandle nh;
	Serial_mavlink CSerial_mavlink(nh);
	ros::spin();
	return 0;
}