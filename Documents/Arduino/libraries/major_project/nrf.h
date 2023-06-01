#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

const byte addresses[][6] = {"abcde","efghi"};

struct Data_Package {
  float m1 = 0.0;
  float m2 = 0.0;
  float m3 = 0.0; 
  float m4 = 0.0;
  float m5 = 0.0;
  float m6 = 0.0;
  float m7 = 0.0;
  float m8 = 0.0;
};

extern Data_Package msg;


void sendData(float m1=99.99, float m2=99.99, float m3=99.99, float m4=99.99, float m5=99.99, float m6=99.99, float m7=99.99, float m8=99.99);
bool receiveData();
void nrf_setup();
