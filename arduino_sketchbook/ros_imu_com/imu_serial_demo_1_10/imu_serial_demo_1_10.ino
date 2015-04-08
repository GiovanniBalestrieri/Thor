/* Librerie ROS */
//#include <ros.h>
//#include <geometry_msgs/Vector3.h>
//#include <geometry_msgs/Quaternion.h>

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
//ros::NodeHandle nh;
//ros::geometry_msgs::Quaternion orientation;
//ros::geometry_msgs::angular_speed;
//ros::geometry_msgs::linear_acceleration;
//ros::Publisher pub_quat("quaternion", &orientation);
//ros::Publisher pub_vel("ang_vel", &angular_speed);
//ros::Publisher pub_acc("lin_accel", &linear_acceleration);
//sensor_msgs.Imu imu_msg;
//uint32 seq_num;

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
    //nh.initNode();
    //nh.advertise(pub_quat);
    //nh.advertise(pub_vel);
    //nh.advertise(pub_acc);

    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    Serial.begin(115200);

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    //mpu.setZAccelOffset(1788); // 1688 factory default for my test chip
    if (devStatus == 0) {
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);
        
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        
        mpuIntStatus = mpu.getIntStatus();
        
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        
        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {// ERROR!
      Serial.println("ERROR!");
    }

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
          Serial.println(F("FIFO overflow!"));
          
      } else if (mpuIntStatus & 0x02) {
        
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        fifoCount -= packetSize;
        
        
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetAccel(&aa, fifoBuffer);
        mpu.dmpGetGyro(g, fifoBuffer);
        //mpu.dmpGetGravity(&gravity, &q);
        //mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
        Serial.print("accel\t");
        Serial.print(aa.x);
        Serial.print("\t");
        Serial.print(aa.y);
        Serial.print("\t");
        Serial.println(aa.z);
        Serial.print("gyro\t");
        Serial.print(g[0]);
        Serial.print("\t");
        Serial.print(g[1]);
        Serial.print("\t");
        Serial.println(g[2]);
        Serial.print("quaternion\t");
        Serial.print(q.x);
        Serial.print("\t");
        Serial.print(q.y);
        Serial.print("\t");
        Serial.println(q.z);
        Serial.print("\t");
        Serial.println(q.w);
        
        /* ROS */
        /*
        sensor_msgs/Imu
	std_msgs/Header
		uint32 seq
		time stamp
		string frame_id
		string child_frame_id

        */
        // Quaternioni
        //orientation.x = q.x;
        //orientation.y = q.y;
        //orientation.z = q.z;
        //orientation.w = q.w;
        //
        // Velocità angolare
        //angular_velocity.x = gyroTransform(g[0]);
        //angular_velocity.y = gyroTransform(g[1]);
        //angular_velocity.z = gyroTransform(g[2]);
        //
        // Accelerazione angolare
        //linear_acceleration.x = accelTransform(aa.x);
        //linear_acceleration.y = accelTransform(aa.y);
        //linear_acceleration.z = accelTransform(aa.z);
        //
        //pub_quat.publish( &orientation );
        //pub_quat.publish( &angular_velocity );
        //pub_quat.publish( &linear_acceleration );
        //
        //nh.spinOnce();
        //seq_num;
    }
}
