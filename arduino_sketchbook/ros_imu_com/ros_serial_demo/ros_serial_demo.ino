#include <ros.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>

ros::NodeHandle nh;
geometry_msgs::Quaternion orientation;
geometry_msgs::Vector3 angular_velocity;
geometry_msgs::Vector3 linear_acceleration;

geometry_msgs::Pose pose;
geometry_msgs::Twist twist;

/* Imu */
ros::Publisher pub_quat("quaternion", &orientation);
ros::Publisher pub_vel("ang_vel", &angular_velocity);
ros::Publisher pub_acc("lin_accel", &linear_acceleration);
/* Odometry */
ros::Publisher pub_pose("odom_pose", &pose);
ros::Publisher pub_twist("odom_twist", &twist);

void setup() {
  init_ros_system();
}

void loop(){
  //
  float quat[] = {0.0, 0.0, 0.0, 0.0};
  float vel[] = {0, 0, 0};
  float acc[] = {0, 0, 0};
  publishImu(quat, vel, acc);
  //
  float pos[] = {0, 0, 0};
  float lin[] = {0, 0, 0};
  float ang[] = {0, 0, 0};
  publishOdom(pos, quat, lin, ang);
  //
  delay(100);
}

void publishImu(float qu[], float av[], float la[])
{
  orientation.x = qu[0];
  orientation.y = qu[1];
  orientation.z = qu[2];
  orientation.w = qu[3];

  angular_velocity.x = av[0];
  angular_velocity.y = av[1];
  angular_velocity.z = av[2];
  
  linear_acceleration.x = la[0];
  linear_acceleration.y = la[1];
  linear_acceleration.z = la[2];
  
  pub_quat.publish( &orientation );
  pub_vel.publish( &angular_velocity );
  pub_acc.publish( &linear_acceleration );
  nh.spinOnce();
}

void publishOdom(float po[], float qu[], float li[], float an[])
{
  pose.position.x = po[0];
  pose.position.y = po[1];
  pose.position.z = po[2];
  
  pose.orientation.x = qu[0];
  pose.orientation.y = qu[1];
  pose.orientation.z = qu[2];
  pose.orientation.w = qu[3];
  
  twist.linear.x = li[0];
  twist.linear.y = li[1];
  twist.linear.z = li[2];
  
  twist.angular.x = an[0];
  twist.angular.y = an[1];
  twist.angular.z = an[2];
  
  pub_pose.publish( &pose );
  pub_twist.publish( &twist );
  nh.spinOnce();
}

void init_ros_system()
{
  nh.initNode();
  
  nh.advertise(pub_quat);
  nh.advertise(pub_vel);
  nh.advertise(pub_acc);
  
  nh.advertise(pub_pose);
  nh.advertise(pub_twist);
}
