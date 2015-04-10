int seqno;

void setup() {
  Serial.begin(115200);
  seqno = 0;
}

void loop(){

    /* ROS */
  float quat[] = {0.0, 0.0, 0.0, 0.0};
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
