#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>

#include <sstream>

#define QUATERNION_MSG 0
#define VELOCITY_MSG 1
#define ACCELERATION_MSG 2


void set_covariance();
void send(int rec);
void quat_callback(const geometry_msgs::Quaternion q);
void vel_callback(const geometry_msgs::Vector3 s);
void acc_callback(const geometry_msgs::Vector3 a);
void send_odometry();
