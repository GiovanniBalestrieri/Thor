/* Librerie IMU */
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <MPU6050.h>
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
MPU6050 mpu;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
int16_t g[3];
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector


/* ROS */
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

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}


// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
    /* ROS */
    init_ros_system();

    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize device
    mpu.initialize();

    // verify connection
    devStatus = mpu.dmpInitialize();

    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    //mpu.setZAccelOffset(1788); // 1688 factory default for my test chip
    if (devStatus == 0) {
        mpu.setDMPEnabled(true);
        attachInterrupt(0, dmpDataReady, RISING);
        
        mpuIntStatus = mpu.getIntStatus();
        
        
        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {/* ERROR!*/}    

}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
    if (!dmpReady) return;

    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    fifoCount = mpu.getFIFOCount();

    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
          mpu.resetFIFO();          
      } else if (mpuIntStatus & 0x02) {
        
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        fifoCount -= packetSize;
        
        
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetAccel(&aa, fifoBuffer);
        mpu.dmpGetGyro(g, fifoBuffer);
        //mpu.dmpGetGravity(&gravity, &q);
        //mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
        
        /* ROS */
        float quat[] = {q.x, q.y, q.z, q.w};
        float vel[] = {g[0], g[1], g[2]};
        float acc[] = {aa.x, aa.y, aa.z};
        publishImu(quat, vel, acc);
        //
        float pos[] = {0, 0, 0};
        float lin[] = {0, 0, 0};
        float ang[] = {0, 0, 0};
        publishOdom(pos, quat, lin, ang);
    }
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
