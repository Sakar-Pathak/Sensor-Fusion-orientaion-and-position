#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

const byte addresses[][6] = {"abcde","efghi"};

extern boolean buttonState;

void sendData(float m1=99.99, float m2=99.99, float m3=99.99, float m4=99.99, float m5=99.99, float m6=99.99, float m7=99.99, float m8=99.99);//, float m9 = 99.99, float m10 = 99.99, float m11 = 99.99, float m12=99.99, float m13=99.99, float m14=99.99, float m15=99.99,  float m16=99.99, float m17=99.99, float m18=99.99, float m19 = 99.99, float m20 = 99.99, float m21 = 99.99, float m22=99.99, float m23=99.99,  float m24=99.99, float m25=99.99);
void nrf_setup();
void readButtonState();
