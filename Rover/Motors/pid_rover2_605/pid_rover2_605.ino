 /**********************************************************************
 *             Arduino Uno & sabertooth & Motors & PID & Odometry     *
 *                        by Thor Group, 2015                         *
 **********************************************************************/

/**
 *  Features:
 *  
 *  - Motors Interface
 *  - Angular Velocity from encoders
 *  - PID Motors control
 *  - Serial Routine, Press:
 *                    - 'a' -> TestMotors()
 *                    - 'i' -> Check ISR freq [Hz]
 *                    - 'p' -> Toggles generalPrint
 *                    - 'o' -> Toogles printOdom
 *                    - 's' -> Toggles printMotorsSx
 *                    - 'd' -> Toogles printMotorsDx
 *                    - 'w' -> Toogles printAngVel
 *                    - 'e' -> Toogles printEncoder
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

#include <Servo.h>
Servo motorDx, motorSx;

/**
 ** Serial 
 **/
 
boolean generalPrint = true;

// Infos
boolean printTimerInfo = true;

// Odometry
boolean printOdom = false;
boolean printError = true;
boolean printPidVal= false;
boolean printMotorsDx = false;
boolean printMotorsSx = false;
boolean printAngVel= false;
boolean printLinVel= false;
boolean printEncoder= false;
boolean printOrientation= false;

/**
 ** Setpoint References
 **/
 
int riferimentoSx = 0; // [-470 ; 470]  (rad/s)
int riferimentoDx = 0;

/**
 ** Control 
 **/
 
#define Ts 2000 //us
#define Kp_1 0.8
#define Ki_1 0.0
#define Kd_1 0.0
#define ControlDeadzone_1 5 //deadzone for u, the control variable

#define Kp_2 0.0
#define Ki_2 0.0
#define Kd_2 0.0
#define ControlDeadzone_2 5

/**
 ** Other
 **/
 
// Timing
volatile unsigned long int timerPid;
volatile unsigned long int timerMisure;
volatile unsigned long int timerSabertooth;
volatile unsigned long int timerISR;
volatile unsigned long int deltaPid, dtPid;

// Input Motors
volatile float uSx = 0, uDx = 0;

//Measures interrupt freq -> servono?
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

/**
 ** Timer 1 vars
 **/
 volatile int contIsr = 0;
 // TODO:
 // add misure e pid

int donTouch = 10;

void setup()
{   
  Serial.begin(115200);
  // Valore per superare attrito statico
  Pwm_Static_Friction1 = 0; // trovato tramite test motor, a 98 inizia a muoversi
  Pwm_Static_Friction2 = 0; //da trovare;
  //Pwm_Static_Friction = IdentifyStaticFrictionPwm();
  
  /**
   ** Timer 2 @ 200 Hz
   **/
   
  cli(); // Stops interrupts
  
  TCCR2A = 0;// set entire TCCR1A register to 0
  TCCR2B = 0;// same for TCCR1B
  
  // set compare match register for 200 Hz increments
  OCR2A = 77;// = (16*10^6) / (200*1024) - 1 -> 200 Hz
  //OCR2A=193; //16*10^6/(80Hz*1024) - 1 = 193 -> 80 Hz 
  //OCR2A=780; //16*10^6/(20Hz*1024) - 1 = 780 -> 20 Hz 
  //OCR2A=50; //16*10^6/(308Hz*1024) - 1 = 50 -> 308 Hz 
  
  // turn on CTC mode
  TCCR2A |= (1 << WGM21);
  // Set CS10 and CS12 bits for 1024 prescaler
  TCCR2B |= (1 << CS20) | (1 << CS21) | (1 << CS22);  
  // enable timer compare interrupt
  TIMSK2 |= (1 << OCIE2A);
  
  sei(); //enable global interrupts
  
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
    tOld = micros();
    //controllo_encoder(); // da fare
    
    // The followings are exec in ISR @ 200HZ
    //misure();  
    //pid();      
    count += 1;
    if(count >= donTouch) // Runs @ 2 Hz
    {
      count = 0;
      serialRoutine();
      odometry();
      info();
      
      // resets ISR counter
      contIsr=0;
    }
    // Now runs in ISR after pid and misure
    //sabertooth(uDx,uSx);
  }
  handleOverflow();  
}

/**
 ** Interrupt service routine @ 200 Hz
 **/
ISR(TIMER2_COMPA_vect)
{
  timerISR = micros();
  
  misure();
  pid();
  sabertooth(uDx,uSx);
  contIsr++;
  
  timerISR = micros() - timerISR;
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
   else if (t == 'e') 
   {
     Serial.println(" Toggles printEncoder ");
     printEncoder = !printEncoder;
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
     Serial.println(timerISR);
     Serial.print(" pid DT: ");
     Serial.println(dtPid);
//     Serial.print(" Pid exec time: ");
//     Serial.print(timerPid);
//     Serial.print(" Misure exec time: ");
//     Serial.print(timerMisure);
//     Serial.print(" Sabertooth exec time: ");
//     Serial.println(timerSabertooth);
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

void pid()   // Non modificato uK -- siamo in una  ISR -> change vars to volatile
{ 
 deltaPid = micros() - deltaPid;
 dtPid = deltaPid;
 timerPid = micros();
 
  // pid M1
//  if(micros()-MStOld >= 30000){
//    m1VelAng = 0;
//    Serial.println("start");
//  }else{
    m1VelAng = 2.0*PI*1000000.0/(2048.0*MSperiodAtt);
 // }
  erroreSx = riferimentoSx - (m1VelAng); //*(52.0/14.0)
  integrale1 = integrale1 + (Ki_1*TCAMP*erroreSx)/1000.0;
  u1 = int(Kp_1*erroreSx) + (int) integrale1 + int(1000000*Kd_1*(erroreSx-errore_old1)/TCAMP);
  //u1 = 0;
  errore_old1 = erroreSx;
  // pid M2
  erroreDx = riferimentoDx;//- int(m2VelAng)); //*(52.0/18.0)
  integrale2 = integrale2 + (Ki_2*TCAMP*erroreDx)/1000;
  u2 = int(Kp_2*erroreDx) + int(integrale2) + int(1000000*Kd_2*(erroreDx-errore_old2)/TCAMP);
  //u2 = 0;
  errore_old2 = erroreDx;
  
  /* DEBUG NON CONSENTITO SIAMO IN UNA ISR niente serialPrint
  
  if (printMotorsSx)
  {
    Serial.print("IntegraleSx:  ");
    Serial.println(integrale1);
  }
  
  if (printMotorsDx)
  {
    Serial.print("IntegraleDx:  ");
    Serial.println(integrale2);
  }
  
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
  */
  
  uSx = u1;
  uDx = u2;
  controllo_deadzone(); 
  
  timerPid = micros() - timerPid;
  deltaPid = micros();
}

void controllo_deadzone(void) // * ISR -> change to volatile
{
  uSx = uSx + 90;
  uDx = uDx + 90;
  if(uSx != 90)
    uSx = uSx+Pwm_Static_Friction1;
  if(uDx != 90)
    uDx = uDx+Pwm_Static_Friction2;
}

void misure()
{
  timerMisure = micros();
  
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
  
  timerMisure = micros() - timerMisure;
}

void odometry()
{  
  if (printOdom && generalPrint)
  {
    if(printError)
    {
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
  }
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
  timerSabertooth = micros();
  
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

  timerSabertooth = micros() - timerSabertooth;
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
