/************************************************************
******************* Wiring: **************
*************************************************************
Arduino Pin A0 -> AttoPilot V
Arduino Pin A1 -> AttoPilot I -> doesn't work
Arduino Pin 9 -> Sabertooth S1
Arduino Pin 10 -> Sabertooth S2
Arduino Pin 2 -> encoder M1 ( SX ) motorSx 
Arduind Pin 3 -> encoder M2 ( DX ) motorDx
Arduino GND -> Sabertooth 0V
Arduino GND -> AttoPilot GND
Arduino VIN -> Sabertooth 5V (OPTIONAL, Sabertooth powers Arduino)
*/

boolean printOdom = false;
boolean printError = true;
boolean printPidVal= false;
boolean printMotorsDx = false;
boolean printMotorsSx = false;
boolean printAngVel= false;
boolean printLinVel= false;
boolean printEncoder= false;
boolean printOrientation= false;

int riferimentoSx = 100; // [-470 ; 470]  (rad/s)

int riferimentoDx = 0;

// Measures interrupt freq
volatile long bla;
volatile long blu;
volatile long isra,isrb;

#define pinEncoderBS 4
#define pinEncoderBD 5

#define NUMEROIMPULSI 2078.4
#define DISTRUOTE 0.59
#define RAGGIORUOTA 0.13
#define KRAPP 1
#define TCAMP 50000

#define Ts 2000 //sampling times of the control loop in microseconds
#define Kp_1 0.8
#define Ki_1 0.0
#define Kd_1 0.0
#define ControlDeadzone_1 5 //deadzone for u, the control variable

#define Kp_2 0.0
#define Ki_2 0.0
#define Kd_2 0.0
#define ControlDeadzone_2 5 //deadzone for u, the control variable

int u1 = 0; //control variable
float integrale1 = 0;
float erroreSx = 0;
float errore_old1 = 0;
int Pwm_Static_Friction1 = 0;
//variabili pid 2
int u2 = 0; //control variable
float integrale2 = 0;
float erroreDx = 0;
float errore_old2 = 0;
int Pwm_Static_Friction2 = 0;

// Input Motors
float uSx = 0, uDx = 0;

//variabili per encoder motore 1 
volatile long int MSencoderPos = 0;
volatile long int MStOld = -30000;
volatile long int MSperiodAtt = 1;
long int M1deltaPos = 0;
long int M1oldPos = 0;
char M1verso = 1;
float m1VelAng = 0;
float M1velLin = 0;
//variabili per encoder motore 2
volatile long int MDencoderPos = 0;
volatile long int MDtOld = 0;
volatile long int MDperiodAtt = 0;
long int M2deltaPos = 0;
long int M2oldPos = 0;
char M2verso = 1;
float m2VelAng = 0;
float M2velLin = 0;
// Filter costante
float alpha = 1.0;//0.08;
//variabili rover
float velLinAss = 0;
float omegaZAss = 0;
float velXAss = 0;
float velYAss = 0;
float xAss = 0;
float yAss = 0;
float thetaZAss = 0;
//variabili controllore
long tOld = 0;
int count = 0;

long firstTime;
long timeToMove = 5000000L;

#include <Servo.h>

Servo motorDx, motorSx;
int VRaw; 
int IRaw;
float VFinal; 
float IFinal;

void setup()
{ 
  // Valore per superare attrito statico
  Pwm_Static_Friction1 = 0;// trovato tramite test motor, a 98 inizia a muoversi
  Pwm_Static_Friction2 = 0;//da trovare;
  //Pwm_Static_Friction = IdentifyStaticFrictionPwm(); // va fatto
  
  Serial.begin(9600);
  
  // Define encoder pin Interrupt
  attachInterrupt(0, MSencVel, RISING);
  attachInterrupt(1, MDencoder, RISING);
  motorDx.attach(10, 1000, 2000); // SX ??
  motorSx.attach(11, 1000, 2000); // DX ?? 
  firstTime = micros();
}
void loop()
{
  if(micros()-tOld >= TCAMP)
  {
    //sistemare overflow
    tOld = micros();
    misure();
    //controllo_encoder(); // da fare
    pid();      
    count += 1;
    if(count >= 10)
    {
      count = 0;
      //serialRoutine();
      //testMotor();
      odometry();
//      Serial.println("Controlli");
//      Serial.print("uSx = ");
//      Serial.print(uSx);
//      Serial.print(" uDx = ");
//      Serial.println(uDx);
      
      //sabertooth(90,90);
      
//      Serial.println();
//      Serial.print(" blu ");
//      Serial.print(isra);
//      Serial.println();
//      Serial.print(" bla ");
//      Serial.println(isrb);
    }
    sabertooth(uDx,uSx);
  }
  handleOverflow();  
}

void serialRoutine()
{
 if (Serial.available()>0)
 {
   char t = Serial.read();
   if (t == 'a')
   {
     Serial.println(" Testing Motors ");
     testMotor(); 
   }
 } 
}

void handleOverflow()
{
  //handles overflow OK
  if(abs(MSencoderPos)>60000)
  {
    MSencoderPos=0;
    M1oldPos = -M1deltaPos;
    //Serial.println("overflow");
  } 
  if(abs(MDencoderPos)>60000)
  {
    MDencoderPos=0;
    M2oldPos = -M2deltaPos;
    //Serial.println("overflow ");
  }
}

void pid()
{ 
  // pid M1
//  if(micros()-MStOld >= 30000){
//    m1VelAng = 0;
//    Serial.println("start");
//  }else{
    m1VelAng = 2.0*PI*1000000.0/(2048.0*MSperiodAtt);
 // }
  erroreSx = riferimentoSx - (m1VelAng); //*(52.0/14.0)
  integrale1 = integrale1 + (Ki_1*TCAMP*erroreSx)/1000.0;
  if (printMotorsSx)
  {
    Serial.print("IntegraleSx:  ");
    Serial.println(integrale1);
  }
  u1 = int(Kp_1*erroreSx) + (int) integrale1 + int(1000000*Kd_1*(erroreSx-errore_old1)/TCAMP);
  //u1 = 0;
  errore_old1 = erroreSx;
  // pid M2
  erroreDx = riferimentoDx;//- int(m2VelAng)); //*(52.0/18.0)
  integrale2 = integrale2 + (Ki_2*TCAMP*erroreDx)/1000;
  if (printMotorsDx)
  {
    Serial.print("IntegraleDx:  ");
    Serial.println(integrale2);
  }
  u2 = int(Kp_2*erroreDx) + int(integrale2) + int(1000000*Kd_2*(erroreDx-errore_old2)/TCAMP);
  //u2 = 0;
  errore_old2 = erroreDx;
  if (printPidVal)
  {
    if (printMotorsDx)
    {
      Serial.print("erroreDx = ");
      Serial.print(erroreDx);
      Serial.print("  velAng DX = ");
      Serial.print(m2VelAng);
    }    
    if (printMotorsSx)
    {
      Serial.print("erroreSx = ");
      Serial.print(erroreSx);
      Serial.print("  velAng SX = ");
      Serial.println(m1VelAng);
    }
  }  
  
  uSx = u1;
  uDx = u2;
  controllo_deadzone(); 
}

void controllo_deadzone(void)
{
  /* if(u1 < -ControlDeadzone_1)
  {
   u1 = abs(u1) + Pwm_Static_Friction1; //There is almost a deadzone for |u|<=DeadZone 
  }
  else 
   if(u1 > ControlDeadzone_1)
   {
    u1 = u1 + Pwm_Static_Friction1;
   }
  else u1 = 0;
  
  if(u2 < -ControlDeadzone_2)
  {  
   u2 = abs(u2) + Pwm_Static_Friction2; //There is almost a deadzone for |u|<=DeadZone 
  }
  else 
   if(u2 > ControlDeadzone_2)
   {
    u2 = u2 + Pwm_Static_Friction2;
   }
  else u2 = 0;*/
  
  /*u1 = constrain(u1,-90,89);
  u2 = constrain(u2,-90,89);
  map(u1,0,180,-90,89);
  map(u2,0,180,-90,89);
  u1 = constrain(u1,-90,89);
  u2 = constrain(u2,-90,89);*/
  //u2 = u2+90; // u2 positivo in antiorario (0 90 dopo mappato)
  uSx = uSx + 90;
  uDx = uDx + 90;
  if(uSx != 90)
    uSx = uSx+Pwm_Static_Friction1;
  if(uDx != 90)
    uDx = uDx+Pwm_Static_Friction2;
//  u2 = 90;
  //u1 = u1; // u1 positivo in orario (-90 0 dopo mappato)
  
}

void misure()
{
  M1deltaPos = (MSencoderPos - M1oldPos);
  M1oldPos = MSencoderPos;
  M2deltaPos = (MDencoderPos - M2oldPos);
  M2oldPos = MDencoderPos;
  m1VelAng = (1-alpha)*m1VelAng + alpha*(2*PI*M1deltaPos)/(NUMEROIMPULSI)*1000000/TCAMP;
  m2VelAng = (1-alpha)*m2VelAng + alpha*(2*PI*M2deltaPos)/(NUMEROIMPULSI)*1000000/TCAMP;;
  M1velLin = m1VelAng*RAGGIORUOTA;
  M2velLin = m2VelAng*RAGGIORUOTA;
  velLinAss = (M1velLin+M2velLin)/2;
  omegaZAss = (M1velLin-M2velLin)/DISTRUOTE;
  thetaZAss = thetaZAss + omegaZAss*TCAMP/(1000000);
  velXAss = velLinAss*sin(thetaZAss);
  velYAss = velLinAss*cos(thetaZAss);
  xAss = xAss + velXAss*TCAMP/(1000000);
  yAss = yAss + velYAss*TCAMP/(1000000);
}

void odometry()
{  
  if (printOdom)
  {
    if(printError){
      Serial.print("ErrSX= ");
      Serial.println(erroreSx);  
    }    
    if (printOrientation)
    {
      Serial.print("thetaZAss: ");
      Serial.print(thetaZAss);
      Serial.print(" xAss: ");
      Serial.print(xAss);
      Serial.print(" yAss: ");
      Serial.print(yAss);
    }
    if (printLinVel)
    {
      Serial.print(" velYAss: ");
      Serial.print(velYAss);
      Serial.print(" velXAss: ");
      Serial.print(velXAss);
      Serial.print(" M1velLin: ");
      Serial.print(M1velLin);
      Serial.print(" M2velLin: ");
      Serial.print(M2velLin);
    }
    if (printAngVel)
    {      
      Serial.print(" m1VelAng: ");
      Serial.print(m1VelAng);
      Serial.print(" m2VelAng: ");
      Serial.print(m2VelAng);
    }
    if (printEncoder)
    {
      Serial.print(" M1pos: ");
      Serial.print(MSencoderPos);
      Serial.print(" M2pos: ");
      Serial.println(MDencoderPos);
    }
  }
}

void attoPilot()
{
  //Measurement
  VRaw = analogRead(A0);
  IRaw = analogRead(A1); // doesn't work ...
  //Conversion
  VFinal = VRaw/49.44; //45 Amp board
  IFinal = IRaw/14.9; //45 Amp board
  Serial.print(VFinal);
  Serial.println(" Volts");
  Serial.print(IFinal);
  Serial.println(" Amps");
  Serial.println("encoder");
  delay(200);
}

/** 
 ** Sets max and min pwm values and sends control inputs 
 ** to motors. 
 ** 
 ** Called: from main loop
 ** Exec Time: TODO!
 **/
void sabertooth(float uDx, float uSx)
{  
  if (uSx >= 190)
   uSx = 190;
  else if (uSx < 0)
   uSx = 0;
   
  if (uDx >= 190)
   uDx = 190;
  else if (uDx < 0)
   uDx = 0;
   
  motorDx.write(int(uDx));
  motorSx.write(int(uSx)); 
   
  Serial.println();
  if (printMotorsSx)
  {
    Serial.print("  Motor Sx:   ");
    Serial.println(uSx);
  }
  if (printMotorsDx)
  {
    Serial.print("  Motor Dx:   ");
    Serial.println(uDx);
  } 
}

void MSencoder() /////////////////////////////////////////////////////////////Verso cambiato
{   // check channel B to see which way encoder is turning
//    if (digitalRead(pinEncoderBS)){
//      MSencoderPos++;
//    }
//    else{
//      MSencoderPos--;
//    }     
  MSencoderPos++;
    //PastB ? encoderPos--:  encoderPos++;     
}
void MDencoder()
{   // check channel B to see which way encoder is turning
    // check channel B to see which way encoder is turning
    if (digitalRead(pinEncoderBD)){
      MDencoderPos--;
    }
    else{
      MDencoderPos++;
    }     
    //PastB ? encoderPos--:  encoderPos++;     
}



void MSencVel()
{
   MSperiodAtt = micros()-MStOld;
   MStOld = micros(); 
}


void testMotor()
{
 float v_m1,v_m2;
 int soglia1=0; // partito
 int soglia2=0; // stop
 boolean cond;
 for (int i = 90; i<255;i++)
 {
   sabertooth(i,i);
   delay(100);
   if (m1VelAng > 0)
   {  
     cond=true;
     soglia1 = i;
   }
   misure();
   odometry();
   Serial.print("  u1 =");      
   Serial.println(i);   
 }
 
 Serial.print(" Soglia");
 Serial.println(soglia1);
}
