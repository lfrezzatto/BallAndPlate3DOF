#include <Servo.h>

#define SERVO1 5 // Porta Digital 6 PWM
#define SERVO2 6
#define SERVO3 7

Servo s1; // Variável Servo
Servo s2;
Servo s3;
int pos1; // Posição Servo
int pos2;
int pos3;

int inic = -30;
int fin = 30;

int amp = 30;

int prbs1[] = {amp,0,0,amp,amp,amp,amp,0,amp,0,0,0,amp,amp,amp,0,0,amp,0,0,amp,0,amp,amp,0,amp,amp,amp,0,amp,amp,0,0,amp,amp,0,amp,0,amp,0,amp,amp,amp,amp,amp,amp,0,0,0,0,0,amp,0,0,0,0,amp,amp,0,0,0,amp,0,amp,0,0,amp,amp,amp,amp,0,amp,0,0,0,amp,amp,amp,0,0,amp,0,0,amp,0,amp,amp,0,amp,amp,amp,0,amp,amp,0,0,amp,amp,0,amp};
int prbs2[] = {0,0,0,0,amp,0,0,0,0,amp,amp,0,0,0,amp,0,amp,0,0,amp,amp,amp,amp,0,amp,0,0,0,amp,amp,amp,0,0,amp,0,0,amp,0,amp,amp,0,amp,amp,amp,0,amp,amp,0,0,amp,amp,0,amp,0,amp,0,amp,amp,amp,amp,amp,amp,0,0,0,0,0,amp,0,0,0,0,amp,amp,0,0,0,amp,0,amp,0,0,amp,amp,amp,amp,0,amp,0,0,0,amp,amp,amp,0,0,amp,0,0,amp};
int prbs3[] = {0,amp,amp,0,amp,amp,amp,0,amp,amp,0,0,amp,amp,0,amp,0,amp,0,amp,amp,amp,amp,amp,amp,0,0,0,0,0,amp,0,0,0,0,amp,amp,0,0,0,amp,0,amp,0,0,amp,amp,amp,amp,0,amp,0,0,0,amp,amp,amp,0,0,amp,0,0,amp,0,amp,amp,0,amp,amp,amp,0,amp,amp,0,0,amp,amp,0,amp,0,amp,0,amp,amp,amp,amp,amp,amp,0,0,0,0,0,amp,0,0,0,0,amp,amp};

void setup ()
{
  s1.attach(SERVO1);
  s2.attach(SERVO2);
  s3.attach(SERVO3);
  Serial.begin(9600);
  s1.write(0); // Inicia motor posição zero
  s2.write(0);
  s3.write(0);
}

void loop() {

  s1.write(inic);
  s2.write(inic);
  s3.write(inic);
  delay(1000);
  s1.write(fin);
  s2.write(fin);
  s3.write(fin);
  delay(1000);
  }
