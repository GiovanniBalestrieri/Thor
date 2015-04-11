#include "arduino_ekf_response.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "ekf_response");	
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("/robot_pose_ekf/odom_combined", 1000, received_ekf_callback);
	ros::spin();
}

void received_ekf_callback(const geometry_msgs::PoseWithCovarianceStamped resp)
{
	geometry_msgs::Pose pose = resp.pose.pose;
	float pos[3];
	pos[0] = pose.position.x;
	pos[1] = pose.position.y;
	pos[2] = pose.position.z;
	
	float orient[4];
	orient[0] = pose.orientation.x;
	orient[1] = pose.orientation.y;
	orient[2] = pose.orientation.z;
	orient[3] = pose.orientation.w;
	
	
}
