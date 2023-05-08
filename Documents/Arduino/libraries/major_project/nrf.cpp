#include "nrf.h"

RF24 radio(9, 10); // CE, CSN
//unsigned long Time;

struct Data_Package {
  float m1 = 0.0;
  float m2 = 0.0;
  float m3 = 0.0; 
  float m4 = 0.0;
  float m5 = 0.0;
  float m6 = 0.0;
  float m7 = 0.0;
  float m8 = 0.0;/*
  float m9 = 0.0;
  float m10 = 0.0;
  float m11 = 0.0;
  float m12 = 0.0;
  float m13 = 0.0; 
  float m14 = 0.0;
  float m15 = 0.0;
  float m16 = 0.0;
  float m17 = 0.0;
  float m18 = 0.0;
  float m19 = 0.0;
  float m20 = 0.0;
  float m21 = 0.0;
  float m22 = 0.0;
  float m23 = 0.0; 
  float m24 = 0.0;
  float m25 = 0.0;
  */
};

Data_Package msg;

boolean buttonState = 1;

void sendData(float m1=99.99, float m2=99.99, float m3=99.99, float m4=99.99, float m5=99.99, float m6=99.99, float m7=99.99, float m8=99.99)//, float m9 = 99.99, float m10 = 99.99, float m11 = 99.99, float m12=99.99, float m13=99.99, float m14=99.99, float m15=99.99,  float m16=99.99, float m17=99.99, float m18=99.99, float m19 = 99.99, float m20 = 99.99, float m21 = 99.99, float m22=99.99, float m23=99.99,  float m24=99.99, float m25=99.99)
{
  msg.m1 = m1;
  msg.m2 = m2;
  msg.m3 = m3;
  msg.m4 = m4;
  msg.m5 = m5;
  msg.m6 = m6;
  msg.m7 = m7;
  msg.m8 = m8;/*
  msg.m9 = m9;
  msg.m10 = m10;
  msg.m11 = m11;
  msg.m12 = m12;
  msg.m13 = m13;
  msg.m14 = m14;
  msg.m15 = m15;
  msg.m16 = m16;
  msg.m17 = m17;
  msg.m18 = m18;
  msg.m19 = m19;
  msg.m20 = m20;
  msg.m21 = m21;
  msg.m22 = m22;
  msg.m23 = m23;
  msg.m24 = m24;
  msg.m25 = m25;
*/
  
  radio.stopListening();
  radio.write(&msg, sizeof(Data_Package));
  radio.startListening();
}

void nrf_setup() {
  radio.begin();
  radio.openWritingPipe(addresses[1]);//efghi
  radio.openReadingPipe(1, addresses[0]);//abcde

  radio.setDataRate(RF24_2MBPS);
 
  radio.setPALevel(RF24_PA_MAX);
  radio.setAutoAck(false);
  radio.startListening();
}


void readButtonState(){

  if (radio.available())
  {
    radio.read(&buttonState, sizeof(buttonState));
  }

}
