#ifndef ARDUINOCOMNODE_H
#define ARDUINOCOMNODE_H

#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include "hlserial_cpp.h"

#include <sstream>

#define QUATERNION_MSG 0
#define VELOCITY_MSG 1
#define ACCELERATION_MSG 2

#define POSE_MSG 0
#define TWIST_MSG 1

#define IMU_SENSOR 0
#define ODOM_SENSOR 1

#define QUAT_SIZE 4
#define VEC3_SIZE 3

#define SEQNO_OFF 0
#define SENSORID_OFF 1
#define MSGSIZE_OFF 2
#define DATA_OFF 3

#define IMUXOFFSET 0.23
#define IMUYOFFSET 0.06
#define IMUZOFFSET 0.0

#define ODOMXOFFSET 0
#define ODOMYOFFSET 0
#define ODOMZOFFSET 0

void set_covariance();
void sendImu(int);
void sendOdometry(int type);
void imu_quat(const float *);
void imu_vel(const float *);
void imu_acc(const float *);
void pose(const float *, const float *);
void twist(const float *, const float *);

/* DEPRECATE */
//void quat_callback(const geometry_msgs::Quaternion q);
//void vel_callback(const geometry_msgs::Vector3 s);
//void acc_callback(const geometry_msgs::Vector3 a);
//void pose_callback(const geometry_msgs::Pose p);
//void twist_callback(const geometry_msgs::Twist t);

#endif
