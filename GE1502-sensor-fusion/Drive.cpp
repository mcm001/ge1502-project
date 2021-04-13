/**
 * GE1501 Group 5.
 */

#include "Drive.h"

#include "Constants.h"
#include "RunningMedian.h"

using namespace constants;

RunningMedian distanceFilter = RunningMedian(3);
String tempStr;

void rightMotor(int motorSpeed) // function for driving the right motor
{
  if (motorSpeed > 0) // if the motor should drive forward (positive speed)
  {
    digitalWrite(AIN1, HIGH); // set pin 1 to high
    digitalWrite(AIN2, LOW);  // set pin 2 to low
  } else if (motorSpeed <
             0) // if the motor should drive backward (negative speed)
  {
    digitalWrite(AIN1, LOW);  // set pin 1 to low
    digitalWrite(AIN2, HIGH); // set pin 2 to high
  } else                      // if the motor should stop
  {
    digitalWrite(AIN1, HIGH); // set pin 1 to low
    digitalWrite(AIN2, HIGH); // set pin 2 to low
  }
  analogWrite(PWMA,
              abs(motorSpeed)); // now that the motor direction is set,
                                     // drive it at the entered speed
}

/********************************************************************************/
void leftMotor(int motorSpeed) // function for driving the left motor
{
  if (motorSpeed > 0) // if the motor should drive forward (positive speed)
  {
    digitalWrite(BIN1, HIGH); // set pin 1 to high
    digitalWrite(BIN2, LOW);  // set pin 2 to low
  } else if (motorSpeed <
             0) // if the motor should drive backward (negative speed)
  {
    digitalWrite(BIN1, LOW);  // set pin 1 to low
    digitalWrite(BIN2, HIGH); // set pin 2 to high
  } else                      // if the motor should stop
  {
    digitalWrite(BIN1, HIGH); // set pin 1 to low
    digitalWrite(BIN2, HIGH); // set pin 2 to low
  }
  analogWrite(PWMB,
              abs(motorSpeed)); // now that the motor direction is set,
                                     // drive it at the entered speed
}

void Drive::drive(float left, float right) {
  leftMotor(left * 255);
  rightMotor(right * 255);
}

float Drive::getGyro() {
  while (!Serial.available()) {
  }
  tempStr = Serial.readStringUntil('\n');
  float ret = tempStr.toFloat();
  Serial.print("Got: ");
  Serial.println(ret);
  return ret;
}

// Adopted from WPILib's MathExtras
float Drive::normalizeAngle(float theta) {
  // Constrain theta to within (-3pi, pi)
  const int n_pi_pos = (theta + 180.0) / 2.0 / 180.0;
  theta = theta - n_pi_pos * 2.0 * 180.0;

  // Cut off the bottom half of the above range to constrain within
  // (-pi, pi]
  const int n_pi_neg = (theta - 180.0) / 2.0 / 180.0;
  theta = theta - n_pi_neg * 2.0 * 180.0;

  return theta;
}

bool Drive::turnToAngle(const float reference) {

  unsigned int count = 0;
  unsigned long start = millis();
  // Time out after 2 seconds
  while (millis() - start <= 4000) {
    const float measurement = getGyro();
    const float delta = normalizeAngle(reference - measurement);
    // Positive CCW
    const float input = constrain(TURN_KP * delta, -MAX_TURN, MAX_TURN);

    // Positive CCW means that the left motor needs to be the negative
    drive(-input, input);

    if (abs(delta) < 6)
      count++;
    if (count > 15)
      return true;

    /*Serial.print("Delta ");
    Serial.print(delta);
    Serial.print(" Input ");
    Serial.print(input);
    Serial.print(" Count ");
    Serial.println(count);*/

    delay(10);
  }
  return false;
}

