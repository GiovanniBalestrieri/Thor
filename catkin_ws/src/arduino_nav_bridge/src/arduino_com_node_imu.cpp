#include "arduino_com_node_imu.h"

int send_status[] = {0, 0, 0};

sensor_msgs::Imu imu_msg;
nav_msgs::Odometry odom;

ros::Publisher imu_pub;
ros::Publisher odom_pub;

int main(int argc, char **argv){

	ros::init(argc, argv, "imu_bridge");	
	ros::NodeHandle n;
		
	/**** COVARIANZE ****/
	set_covariance();
	
	/**** GESTIONE TOPIC ****/	
	ros::Subscriber quatsub = n.subscribe("quaternion", 1000, quat_callback);
	ros::Subscriber velsub = n.subscribe("ang_vel", 1000, vel_callback);
	ros::Subscriber accsub = n.subscribe("lin_accel", 1000, acc_callback);
	imu_pub = n.advertise<sensor_msgs::Imu>("imu_data", 1000);
	odom_pub = n.advertise<nav_msgs::Odometry>("pr2_base_odometry/odom", 1000);
	//odom_pub = n.advertise<nav_msgs::Odometry>("odom", 1000);
	
	ros::spin();
	
	return 0;
}

void set_covariance(){
	// Covarianze IMU
	imu_msg.orientation_covariance[0] = 0.0;
	imu_msg.orientation_covariance[1] = 0.0;
	imu_msg.orientation_covariance[2] = 0.0;
	imu_msg.orientation_covariance[3] = 0.0;
	imu_msg.orientation_covariance[4] = 0.0;
	imu_msg.orientation_covariance[5] = 0.0;
	imu_msg.orientation_covariance[6] = 0.0;
	imu_msg.orientation_covariance[7] = 0.0;	
	imu_msg.orientation_covariance[8] = 0.0;
	
	imu_msg.angular_velocity_covariance[0] = 0.0;
	imu_msg.angular_velocity_covariance[1] = 0.0;
	imu_msg.angular_velocity_covariance[2] = 0.0;
	imu_msg.angular_velocity_covariance[3] = 0.0;
	imu_msg.angular_velocity_covariance[4] = 0.0;
	imu_msg.angular_velocity_covariance[5] = 0.0;
	imu_msg.angular_velocity_covariance[6] = 0.0;
	imu_msg.angular_velocity_covariance[7] = 0.0;
	imu_msg.angular_velocity_covariance[8] = 0.0;

	imu_msg.linear_acceleration_covariance[0] = 0.0;
	imu_msg.linear_acceleration_covariance[1] = 0.0;
	imu_msg.linear_acceleration_covariance[2] = 0.0;
	imu_msg.linear_acceleration_covariance[3] = 0.0;
	imu_msg.linear_acceleration_covariance[4] = 0.0;
	imu_msg.linear_acceleration_covariance[5] = 0.0;
	imu_msg.linear_acceleration_covariance[6] = 0.0;
	imu_msg.linear_acceleration_covariance[7] = 0.0;
	imu_msg.linear_acceleration_covariance[8] = 0.0;
	
	//Covarianze odometry
	odom.pose.covariance[0] = 0.0;
	odom.pose.covariance[1] = 0.0;
	odom.pose.covariance[2] = 0.0;
	odom.pose.covariance[3] = 0.0;
	odom.pose.covariance[4] = 0.0;
	odom.pose.covariance[5] = 0.0;
	odom.pose.covariance[6] = 0.0;
	odom.pose.covariance[7] = 0.0;
	odom.pose.covariance[8] = 0.0;
	odom.pose.covariance[9] = 0.0;
	odom.pose.covariance[10] = 0.0;
	odom.pose.covariance[11] = 0.0;
	odom.pose.covariance[12] = 0.0;
	odom.pose.covariance[13] = 0.0;
	odom.pose.covariance[14] = 0.0;
	odom.pose.covariance[15] = 0.0;
	odom.pose.covariance[16] = 0.0;
	odom.pose.covariance[17] = 0.0;
	odom.pose.covariance[18] = 0.0;
	odom.pose.covariance[19] = 0.0;
	odom.pose.covariance[20] = 0.0;
	odom.pose.covariance[21] = 0.0;
	odom.pose.covariance[22] = 0.0;
	odom.pose.covariance[13] = 0.0;
	odom.pose.covariance[24] = 0.0;
	odom.pose.covariance[25] = 0.0;
	odom.pose.covariance[26] = 0.0;
	odom.pose.covariance[27] = 0.0;
	odom.pose.covariance[28] = 0.0;
	odom.pose.covariance[29] = 0.0;
	odom.pose.covariance[30] = 0.0;
	odom.pose.covariance[31] = 0.0;
	odom.pose.covariance[32] = 0.0;
	odom.pose.covariance[33] = 0.0;
	odom.pose.covariance[34] = 0.0;
	odom.pose.covariance[35] = 0.0;
	
	odom.twist.covariance[0] = 0.0;
	odom.twist.covariance[1] = 0.0;
	odom.twist.covariance[2] = 0.0;
	odom.twist.covariance[3] = 0.0;
	odom.twist.covariance[4] = 0.0;
	odom.twist.covariance[5] = 0.0;
	odom.twist.covariance[6] = 0.0;
	odom.twist.covariance[7] = 0.0;
	odom.twist.covariance[8] = 0.0;
	odom.twist.covariance[9] = 0.0;
	odom.twist.covariance[10] = 0.0;
	odom.twist.covariance[11] = 0.0;
	odom.twist.covariance[12] = 0.0;
	odom.twist.covariance[13] = 0.0;
	odom.twist.covariance[14] = 0.0;
	odom.twist.covariance[15] = 0.0;
	odom.twist.covariance[16] = 0.0;
	odom.twist.covariance[17] = 0.0;
	odom.twist.covariance[18] = 0.0;
	odom.twist.covariance[19] = 0.0;
	odom.twist.covariance[20] = 0.0;
	odom.twist.covariance[21] = 0.0;
	odom.twist.covariance[22] = 0.0;
	odom.twist.covariance[13] = 0.0;
	odom.twist.covariance[24] = 0.0;
	odom.twist.covariance[25] = 0.0;
	odom.twist.covariance[26] = 0.0;
	odom.twist.covariance[27] = 0.0;
	odom.twist.covariance[28] = 0.0;
	odom.twist.covariance[29] = 0.0;
	odom.twist.covariance[30] = 0.0;
	odom.twist.covariance[31] = 0.0;
	odom.twist.covariance[32] = 0.0;
	odom.twist.covariance[33] = 0.0;
	odom.twist.covariance[34] = 0.0;
	odom.twist.covariance[35] = 0.0;
}

void send(int rec){
	send_status[rec] = 1;
	if(send_status[0] == 1 && send_status[1] == 1 && send_status[2] == 1){

		/**** INVIO MESSAGGIO IMU ****/
		imu_msg.header.stamp = ros::Time::now();
		imu_msg.header.frame_id = "imu_link";
		imu_pub.publish(imu_msg);
		/**** INVIO MESSAGGIO ODOMETRY ****/
		send_odometry();
		
		/**** RESET STATO DI RICEZIONE DATI ****/
		send_status[0] = 0;
		send_status[1] = 0;
		send_status[2] = 0;
	}
}

void quat_callback(const geometry_msgs::Quaternion q){
	imu_msg.orientation.x = q.x;
	imu_msg.orientation.y = q.y;
	imu_msg.orientation.z = q.z;
	imu_msg.orientation.w = q.w;
	
	send(QUATERNION_MSG);
}

void vel_callback(const geometry_msgs::Vector3 s){
	imu_msg.angular_velocity.x = s.x;
	imu_msg.angular_velocity.y = s.y;
	imu_msg.angular_velocity.z = s.z;

	send(VELOCITY_MSG);
}

void acc_callback(const geometry_msgs::Vector3 a){
	imu_msg.linear_acceleration.x = a.x;
	imu_msg.linear_acceleration.y = a.y;
	imu_msg.linear_acceleration.z = a.z;
	
	send(ACCELERATION_MSG);
}

void send_odometry(){

	odom.header = imu_msg.header;
	odom.pose.pose.position.x = 0;
	odom.pose.pose.position.y = 0;
	odom.pose.pose.position.z = 0;
	
	odom.pose.pose.orientation.x = 0;
	odom.pose.pose.orientation.y = 0;
	odom.pose.pose.orientation.z = 0;
	odom.pose.pose.orientation.w = 0;
	
	
	odom.twist.twist.linear.x = 0;
	odom.twist.twist.linear.y = 0;
	odom.twist.twist.linear.z = 0;
	
	odom.twist.twist.angular.x = 0;
	odom.twist.twist.angular.y = 0;
	odom.twist.twist.angular.z = 0;
	
	odom_pub.publish(odom);
}
