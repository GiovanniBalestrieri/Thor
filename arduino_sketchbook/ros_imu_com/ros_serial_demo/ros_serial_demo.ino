#include <ros.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>

ros::NodeHandle nh;
geometry_msgs::Quaternion orientation;
geometry_msgs::Vector3 angular_velocity;
geometry_msgs::Vector3 linear_acceleration;

ros::Publisher pub_quat("quaternion", &orientation);
ros::Publisher pub_vel("ang_vel", &angular_velocity);
ros::Publisher pub_acc("lin_accel", &linear_acceleration);

void setup() {
  nh.initNode();
  nh.advertise(pub_quat);
  nh.advertise(pub_vel);
  nh.advertise(pub_acc);

}

void loop(){
  // Quaternioni
  orientation.x = 0.4;//q.x;
  orientation.y = 0.3;//q.y;
  orientation.z = 0.2;//q.z;
  orientation.w = 0.1;//q.w;
        //
        // Velocità angolare
  angular_velocity.x = 1.0;//gyroTransform(g[0]);
  angular_velocity.y = 1.0;//gyroTransform(g[1]);
  angular_velocity.z = 1.0;//gyroTransform(g[2]);
        //
        // Accelerazione lineare
  linear_acceleration.x = 1.1;//accelTransform(aa.x);
  linear_acceleration.y = 2.2;//accelTransform(aa.y);
  linear_acceleration.z = 3.3;//accelTransform(aa.z);
        //
  pub_quat.publish( &orientation );
  pub_vel.publish( &angular_velocity );
  pub_acc.publish( &linear_acceleration );
  nh.spinOnce();      
        //
  delay(500);
}
