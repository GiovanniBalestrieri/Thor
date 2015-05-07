#ifndef ARDUINOEKFRESPONSE_H
#define ARDUINOEKFRESPONSE_H

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Pose.h>
#include "hlserial_cpp.h"

void received_ekf_callback(const geometry_msgs::PoseWithCovarianceStamped);

#endif
