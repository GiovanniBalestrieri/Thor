/**********************************************************************
 *             Arduino Uno & sabertooth & Motors & PID & Odometry     *
 *                        by Thor Group, 2015                         *
 **********************************************************************/

/**
 *  Features:
 *  
 *  - Motors Interface
 *  - Angular Velocity from encoders
 *  - ISR 80Hz
 *  - encoder b direction
 *  - clear code
 *  - TODO: overflow (1 ora), tarare pid
 *  - Serial Routine, Press:
 *                    - 'a' -> TestMotors()
 *                    - 'i' -> Check ISR freq [Hz]
 *                    - 'p' -> Toggles generalPrint
 *                    - 'o' -> Toogles printOdom
 *                    - 's' -> Toggles printMotorsSx
 *                    - 'd' -> Toogles printMotorsDx
 *                    - 'w' -> Toogles printAngVel
 *                    - 'h' -> Toogles printPidVals
 *                    - 'r' -> Reset Pids
 *
 **/

/**********************************************************************/
/**************************** Wiring: *********************************/
/**********************************************************************/
/*
 * Pin A0 -> AttoPilot V
 * Pin A1 -> AttoPilot I -> doesn't work
 * Pin 9 -> Sabertooth S1
 * Pin 10 -> Sabertooth S2
 * Pin 2 -> encoder M1 ( SX ) motorSx 
 * Pin 3 -> encoder M2 ( DX ) motorDx
 * GND -> Sabertooth 0V
 * GND -> AttoPilot GND 
 * VIN <- Sabertooth 5V (OPTIONAL, Sabertooth powers Arduino)
*/

#include "Pid.h"
#include <Servo.h>
/* Librerie IMU */
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <MPU6050.h>
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

MPU6050 mpu;
Servo motorDx, motorSx;

boolean generalPrint = true;
boolean printTimerInfo = false; // Infos
boolean printOdom = true; // Odometry
boolean printMotorsDx = false;
boolean printMotorsSx = true;
boolean printAngVel= true;
boolean printLinVel= false;
boolean printOrientation= false;
boolean printPidVals= false;

#define OUTPUT_READABLE_EULER
#define DERIV_SAT 500
#define MS_RAPPORTO_INGR  (52.0/14.0)
#define MD_RAPPORTO_INGR  (52.0/18.0)
#define MAX_TIME_ENCODER  30000
#define Ts 2000 //us
#define Kp_1 2.0//5.2f
#define Ki_1 0.0//0.3f
#define Kd_1 0.0//0.0f

#define Kp_2 2.8f
#define Ki_2 0.2f
#define Kd_2 0.0f

#define pinEncoderBS 4
#define pinEncoderBD 5
#define NUMEROIMPULSI 48.0f

#define DISTRUOTE 0.595
#define RAGGIORUOTA 0.13
#define KRAPP 1.0
#define TCAMP 50000

float riferimentoSx = 2.0 * MS_RAPPORTO_INGR; // [-470 ; 470]  (rad/s)
float riferimentoDx = 2.0 * MD_RAPPORTO_INGR;

float MD_STATIC_FRIC  = 0;//17.0;
float MS_STATIC_FRIC  = 0;//-22.0;

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

volatile unsigned long int timerPid;
volatile unsigned long int timerMisure;
volatile unsigned long int timerSabertooth;
volatile unsigned long int timerISR;
volatile unsigned long int deltaPid = 12000;/**Dettato da freq ISR 80Hz*/
volatile unsigned long int dtPid;

// Input Motors
volatile float uSx = 0, uDx = 0, uSxOld = 0, uDxOld = 0;

Pid pidSx = Pid(-Kp_1, -Ki_1*deltaPid/1000000.0, -Kd_1*1000000.0/deltaPid);
Pid pidDx = Pid(Kp_2, Ki_2*deltaPid/1000000.0, Kd_2*1000000.0/deltaPid);

volatile int u1 = 0; //control variable
volatile float erroreSx = 0;
volatile float errore_old1 = 0;
volatile int u2 = 0; //control variable
volatile float erroreDx = 0;
volatile float errore_old2 = 0;

//variabili per encoder motore 1 
volatile long int MStOld = 0;
volatile int MSverso = 0;
volatile long int MSperiodAtt = 1;
volatile long int M1deltaPos = 0;
char M1verso = 1;
volatile float m1VelAng = 0;
volatile float M1velLin = 0;

//variabili per encoder motore 2
volatile long int MDtOld = 0;
volatile int MDverso = 0;
volatile long int MDperiodAtt = 1;
volatile long int M2deltaPos = 0;
char M2verso = 1;
volatile float m2VelAng = 0;
volatile float M2velLin = 0;

// Filter costante
float VSdynamic = 0.6;
float VDdynamic = 0.6;

//variabili rover
volatile float velLinAss = 0;
volatile float omegaZAss = 0;
volatile float velXAss = 0;
volatile float velYAss = 0;
volatile float xAss = 0;
volatile float yAss = 0;
volatile float thetaZAss = 0;

// variabili test static friction
 boolean test_friction = false;
 
//variabili controllore
long tOld = 0;
int count = 0;

long firstTime;
long timeToMove = 5000000L;

volatile int contIsr = 0;

int donTouch = 10;


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

void setup()
{   
  Serial.begin(115200);
  /*imu*/
    // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
      TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif

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
  if (devStatus == 0) 
  {
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);
    
    Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(0, dmpDataReady, RISING);
    
    mpuIntStatus = mpu.getIntStatus();
    
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
  } 
  else 
  {  // ERROR!
    Serial.println("ERROR!");
  }
  
  cli(); // Stops interrupts
  
  TCCR2A = 0;// set entire TCCR1A register to 0
  TCCR2B = 0;// same for TCCR1B
  OCR2A=700; //16*10^6/(20Hz*1024) - 1 = 780 -> 20 Hz 
  TCCR2A |= (1 << WGM21); // turn on CTC mode
  TCCR2B |= (1 << CS20) | (1 << CS21) | (1 << CS22); // Set CS10 and CS12 bits for 1024 prescaler
  TIMSK2 |= (1 << OCIE2A); // enable timer compare interrupt
  
  // Define encoder pin Interrupt
  attachInterrupt(0, MSencVel, RISING);
  attachInterrupt(1, MDencVel, RISING);
  
  sei(); //enable global interrupts
  
 // motorDx.attach(10, 1000, 2000); // DX 
  motorSx.attach(11, 1000, 2000); // SX  
  firstTime = micros();
}

void loop()
{
//  if(test_friction == false)
//    dinamic_static_friction();
//  else
//  {
    if(micros()-tOld >= TCAMP)
    {
      tOld = micros();
      count += 1;
      if(count >= donTouch) // Runs @ 2 Hz
      {
        count = 0;
        serialRoutine();
        odometry();
        info();
        contIsr=0;// resets ISR counter
        
        if (!dmpReady)
          return;
      
        mpuInterrupt = false;
        mpuIntStatus = mpu.getIntStatus();
      
        fifoCount = mpu.getFIFOCount();
      
        if ((mpuIntStatus & 0x10) || fifoCount == 1024) 
        {
          // reset so we can continue cleanly
            mpu.resetFIFO();
            Serial.println(F("FIFO overflow!"));
        }//else if (mpuIntStatus & 0x02) {
      
        while (fifoCount < packetSize) 
          fifoCount = mpu.getFIFOCount();
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        fifoCount -= packetSize;
        

        #ifdef OUTPUT_READABLE_EULER
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetEuler(euler, &q);
            Serial.print("euler\t");
            Serial.print(euler[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(euler[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(euler[2] * 180/M_PI);
        #endif  
      }
    //}  
  } 
}

ISR(TIMER2_COMPA_vect)  // Interrupt service routine @ 200 Hz
{
  cli();
  timerISR = micros();
 
  //misure();
  if(micros()-MStOld >= MAX_TIME_ENCODER){
    m1VelAng = 0;
  }else{
    float act1Vel = float(MSverso)*2.0*PI*1000000.0/(NUMEROIMPULSI*MSperiodAtt);
    m1VelAng -= VSdynamic * (m1VelAng - act1Vel);
  } 
  if(micros()-MDtOld >= MAX_TIME_ENCODER){
    m2VelAng = 0;
  }else{
    float act2Vel = float(MDverso)*2.0*PI*1000000.0/(NUMEROIMPULSI*MDperiodAtt);
    m2VelAng -= VDdynamic * (m2VelAng - act2Vel);
  }
  
  deltaPid = micros() - deltaPid;
  dtPid = deltaPid;
  timerPid = micros();
  
  if(m1VelAng == 0) pidSx.reset();  //FOR DEBUGGING
  if(m2VelAng == 0) pidDx.reset();
  
  misure();
  
  pidSx.change_ki(-Ki_1*deltaPid/1000000.0);
  pidDx.change_ki(Ki_2*deltaPid/1000000.0); 
  pidSx.change_kd(-Kd_1*1000000.0/deltaPid); 
  pidDx.change_kd(Kd_2*1000000.0/deltaPid);
  
  uSx = pidSx.get_u(riferimentoSx, m1VelAng) + MS_STATIC_FRIC + 90; /**+90 per sabertooth*/
  uDx = pidDx.get_u(riferimentoDx, m2VelAng) + MD_STATIC_FRIC + 90; /**+90 per sabertooth*/
//  uDx = 90+MD_STATIC_FRIC;
//  uSx = 90+MS_STATIC_FRIC;
  timerPid = micros() - timerPid;
 
  sabertooth();
  deltaPid = micros();
  
  contIsr++;
    
  timerISR = micros() - timerISR; 
  sei();
}

void serialRoutine()
{
 if (Serial.available()>0)
 {
   char t = Serial.read();
   if (t == 'a')
   {
     Serial.println(" Search static friction: ");
     dinamic_static_friction(); 
   }
   else if (t == 'i') 
   {
     printTimerInfo = !printTimerInfo;
     Serial.print(" Toogles Information verbosity: ");
     Serial.println(printTimerInfo);
   }
   else if (t == 'p') 
   {
     Serial.println(" Toggles generalPrint ");
     generalPrint = !generalPrint;
   }
   else if (t == 'o') 
   {
     Serial.println(" Toggles printOdom ");
     printOdom = !printOdom;
   }
   else if (t == 's') 
   {
     Serial.println(" Toggles printMotorsSx ");
     printMotorsSx = !printMotorsSx;
   }
   else if (t == 'd') 
   {
     Serial.println(" Toggles printMotorsDx ");
     printMotorsDx = !printMotorsDx;
   }
   else if (t == 'w') 
   {
     Serial.println(" Toggles printAngVel ");
     printAngVel = !printAngVel;
   }
   else if (t == 'h') 
   {
     Serial.println(" Toggles printPidVals ");
     printPidVals = !printPidVals;
   }
   else if (t == 'r') 
   {
     Serial.println(" Reset Pids: ");
     pidSx.reset();
     pidDx.reset();
   }
   else if (t == 'k') 
   {
     Serial.println(" Choose gains (insert 'p', 'i' or 'd' or other to ESC) ");
     while(!(Serial.available() > 0));
     t = Serial.read();
     if(t == 'p'){
       pidSx.change_kp(Serial.parseFloat());
     } else if(t == 'i'){
       pidSx.change_ki(Serial.parseFloat());
     } else if(t == 'd'){
       pidSx.change_kd(Serial.parseFloat());
     }
   }
 } 
}

void info()
{
 if (generalPrint)
 {
  if (printTimerInfo)
  {
     Serial.println();
     Serial.print(" ISR Freq: ");
     Serial.print(contIsr*2);
     Serial.print(" ISR exec time: ");
     Serial.print(timerISR);
     Serial.print(" pid DT: ");
     Serial.println(dtPid);
  }  
 } 
}

void misure()
{
  timerMisure = micros();
  
  M1velLin = m1VelAng*RAGGIORUOTA;
  M2velLin = m2VelAng*RAGGIORUOTA;
  velLinAss = (M1velLin+M2velLin)/2;
  omegaZAss = (M1velLin-M2velLin)/DISTRUOTE;
  thetaZAss = thetaZAss + omegaZAss*dtPid/(1000000);
  velXAss = velLinAss*sin(thetaZAss);
  velYAss = velLinAss*cos(thetaZAss);
  xAss = xAss + velXAss*dtPid/(1000000);
  yAss = yAss + velYAss*dtPid/(1000000);
  
  timerMisure = micros() - timerMisure;
}

void odometry()
{    
  if (printOdom && generalPrint)
  {
    if (printOrientation)
    {
      Serial.print("  thetaZAss: ");
      Serial.print(thetaZAss);
      Serial.print(" xAss: ");
      Serial.print(xAss);
      Serial.print(" yAss: ");
      Serial.print(yAss);
    }
    if (printLinVel)
    {
      Serial.println();
      Serial.print(" velYAss: ");
      Serial.print(velYAss);
      Serial.print(" velXAss: ");
      Serial.print(velXAss);
      Serial.print(" M1velLin: ");
      Serial.print(M1velLin);
      Serial.print(" M2velLin: ");
      Serial.print(M2velLin);
      Serial.println();
    }
    if (printAngVel)
    {      
      Serial.println();
      Serial.print(" m1VelAng (SX): ");
      Serial.print(m1VelAng);
      Serial.print(" m2VelAng (DX): ");
      Serial.print(m2VelAng);
      Serial.println();
    }
    if (printMotorsSx)
    {
      Serial.println();  
      Serial.print("  Motor Sx:   ");
      Serial.println(uSx);
    }
    if (printMotorsDx)
    {
      Serial.println();  
      Serial.print("  Motor Dx:   ");
      Serial.println(uDx);
    } 
    if (printPidVals)
    {      
      Serial.println();
      Serial.print("ErrSX= ");
      Serial.print(pidSx.get_error()); 
      Serial.print(" uSx= ");
      Serial.print(uSx);         
      Serial.print(" intSx= ");
      Serial.print(pidSx.get_integ()); 
      Serial.print(" dervSx= ");
      Serial.print(pidSx.get_deriv());
      Serial.print(" propSx= ");
      Serial.print(pidSx.get_prop());
      
      Serial.println();
      Serial.print(" ErrDX= ");
      Serial.print(pidDx.get_error());  
      Serial.print(" uDx= ");
      Serial.print(uDx); 
      Serial.print(" intDx= ");
      Serial.print(pidDx.get_integ()); 
      Serial.print(" dervDx= ");
      Serial.print(pidDx.get_deriv());
      Serial.print(" propDx= ");
      Serial.print(pidDx.get_prop());
      
      Serial.println();
    } 
  }
}

void sabertooth()
{  
  timerSabertooth = micros();
  // salutrazione dulla u
  if (uSx >= 180) 
   uSx = 180;
  else if (uSx < 0)
   uSx = 0;
  if (uDx >= 180)
   uDx = 180;
  else if (uDx < 0)
   uDx = 0;
   //saturazione sulla derivata
  if (uDx - uDxOld > DERIV_SAT)
    uDx = uDxOld + DERIV_SAT;
  else if (uDx - uDxOld < -DERIV_SAT)
    uDx = uDxOld - DERIV_SAT;
  if (uSx - uSxOld > DERIV_SAT)
    uSx = uSxOld + DERIV_SAT;
  else if (uSx - uSxOld < -DERIV_SAT)
    uSx = uSxOld - DERIV_SAT;
  uDxOld = uDx;
  uSxOld = uSx;
  motorDx.write(int(uDx));
  motorSx.write(int(uSx));

  timerSabertooth = micros() - timerSabertooth;
}

void MSencVel()
{
   cli();
   MSperiodAtt = micros()-MStOld;
   MSverso = digitalRead(pinEncoderBS)? -1:1;
   MStOld = micros(); 
   sei();
}

void MDencVel()
{
   cli();
   MDperiodAtt = micros()-MDtOld;
   MDverso = digitalRead(pinEncoderBD)? 1:-1;
   MDtOld = micros(); 
   sei();
}

void dinamic_static_friction()
{
  int count = 0;
 for (int i = 90; i<255;i++)
 {
   uSx = i;
   uDx = i;
   sabertooth();
   delay(100);
//   if (m1VelAng > 0)
//   {  
//     count ++;
//     if(count == 5)
//     {
//       MS_STATIC_FRIC = i-90;
//       Serial.print("MS_STATIC_FRIC = ");
//       Serial.println(MS_STATIC_FRIC);
//       test_friction=true;
//       count = 0;
//       break;
//       }
//   }
 }
// if(test_friction == true)
// {
//   for (int i = 90; i<255;i++)
//   {
//     uSx = 90;
//     uDx = i;
//     sabertooth();
//     delay(500);
//     if (m2VelAng > 0)
//     { 
//       count ++;
//       if(count == 5)
//       {
//         MD_STATIC_FRIC = i-90;
//         Serial.print("MD_STATIC_FRIC = ");
//         Serial.println(MD_STATIC_FRIC);
//         break;
//       }
//     }
//   }
// }
}

