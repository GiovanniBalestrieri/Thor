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
int seqno;

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
int16_t g[3];
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector


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
    Serial.begin(115200);
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
  seqno = 0;
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
        seqno = (seqno+1)%256;
        //
        float pos[] = {0, 0, 0};
        float lin[] = {0, 0, 0};
        float ang[] = {0, 0, 0};
        publishOdom(pos, quat, lin, ang);
        seqno = (seqno+1)%256;
    }
}


void publishImu(float qu[], float av[], float la[])
{
  
  byte buffer[43];
  buffer[0] = (byte)seqno;
  buffer[1] = 0;
  buffer[2] = 40;
  
  memcpy(buffer+3, qu, 16);
  memcpy(buffer+19, av, 12);
  memcpy(buffer+31, la, 12);
  
  Serial.write(buffer, 43);
}

void publishOdom(float po[], float qu[], float li[], float an[])
{
  byte buffer[55];
  buffer[0] = (byte)seqno;
  buffer[1] = 1;
  buffer[2] = 53;
  
  memcpy(buffer+3, po, 12);
  memcpy(buffer+15, qu, 16);
  memcpy(buffer+31, li, 12);
  memcpy(buffer+43, an, 12);
  
  Serial.write(buffer, 55);
}
