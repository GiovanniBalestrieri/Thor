#define ACK 1
int seqno;
byte syncseq[] = {170, 170, 170, 170};

void setup() {
  Serial.begin(57600);
  seqno = 0;
}

void loop(){

    /* ROS */
  float quat[] = {1.0, 2.0, 3.0, 4.0};
  float vel[] = {0.0, 0.0, 0.0};
  float acc[] = {0.0, 0.0, 0.0};
  publishImu(quat, vel, acc);
  seqno = (seqno+1)%256;
  //
  float pos[] = {0, 0, 0};
  float lin[] = {0, 0, 0};
  float ang[] = {0, 0, 0};
  publishOdom(pos, quat, lin, ang);
  seqno = (seqno+1)%256;
  delay(500);
}


void publishImu(float qu[], float av[], float la[])
{
  byte buffer[56];
  memset(buffer, 0, 56);
  buffer[0] = (byte)seqno;
  buffer[1] = 0;
  buffer[2] = 40;
  
  memcpy(buffer+3, qu, 16);
  memcpy(buffer+19, av, 12);
  memcpy(buffer+31, la, 12);
 
  s_write(buffer);
}

void publishOdom(float po[], float qu[], float li[], float an[])
{
  byte buffer[56];
  memset(buffer, 0, 56);
  buffer[0] = (byte)seqno;
  buffer[1] = 1;
  buffer[2] = 52;
  
  memcpy(buffer+3, po, 12);
  memcpy(buffer+15, qu, 16);
  memcpy(buffer+31, li, 12);
  memcpy(buffer+43, an, 12);
  
  s_write(buffer);
}
void s_write(byte * buffer)
{
 
 int k;
  byte checksum = 0;
  for(k=0; k<55; k++){
    checksum = checksum + buffer[k];
  }
  buffer[55] = checksum;
  Serial.write(syncseq, 4);
  Serial.write(buffer, 56);
  /*
  do{
    Serial.write(syncseq, 4);
    Serial.write(buffer, 56);
    
    while(!Serial.available());
    
  }while(Serial.read() != ACK);
  */
}
