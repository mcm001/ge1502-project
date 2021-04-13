/**
 * GE1501 Group 5.
 */

#pragma once
#include "pins_arduino.h"

namespace constants {

// the right motor will be controlled by the motor A pins on the motor driver
const int AIN1 = 13; // control pin 1 on the motor driver for the right motor
const int AIN2 = 12; // control pin 2 on the motor driver for the right motor
const int PWMA = 11; // speed control pin on the motor driver for the right
                     // motor

// the left motor will be controlled by the motor B pins on the motor driver
const int PWMB = 10; // speed control pin on the motor driver for the left motor
const int BIN2 = 9; // control pin 2 on the motor driver for the left motor
const int BIN1 = 8; // control pin 1 on the motor driver for the left motor

const float TURN_KP = 0.08;
const float MAX_TURN = 0.6;

} // namespace constants
