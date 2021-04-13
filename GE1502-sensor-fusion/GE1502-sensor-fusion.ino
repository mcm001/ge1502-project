#include <Arduino.h>

#include "Drive.h"
#include "Constants.h"

Drive drive;

using namespace constants;

long lastCommandTime;

void setup() {
  // set the motor control pins as outputs
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);

  Serial.begin(115200);
  Serial.println("Hello!");

  lastCommandTime = millis();
}


void loop() {
  if(Serial.available()) {
    float left = (Serial.readStringUntil(',')).toFloat();
    float right = (Serial.readStringUntil('\n')).toFloat();
    Serial.print("left: "); Serial.print(left); Serial.print(" right: "); Serial.println(right); 
    lastCommandTime = millis();
    
    drive.drive(left, right);
  }
  if(millis() - lastCommandTime > 500) { // if its been half a sec, time out
    drive.drive(0, 0);
  }
}
