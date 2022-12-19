#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <Servo.h>

#define SERVO1 5
#define SERVO2 6
#define SERVO3 7

Servo s1;
Servo s2;
Servo s3;

void setup() {

  s1.attach(SERVO1);
  s2.attach(SERVO2);
  s3.attach(SERVO3);
  
  Serial.begin(9600);

  s1.write(0); // Inicia motor posição zero
  s2.write(0);
  s3.write(0);
}


void loop() {
  
  if(Serial.available() > 0)
  {
    int val[3];
    float delta[3];
    String msg = Serial.readStringUntil('\n');
    char msg_array[msg.length()];
    msg.toCharArray(msg_array, msg.length());

    char * token = strtok(msg_array, ", "); 

    int i = 0;
    while(token != NULL){
        val[i] = atoi(token);
        token = strtok(NULL, ", ");
        i++;
    }

    for(i=0; i<3;i++){
      delta[i] = val[i];
      delta[i] = delta[i]/100;
    }

    s1.write(delta[0]);
    s2.write(delta[1]);
    s3.write(delta[2]);
    

  }

}
