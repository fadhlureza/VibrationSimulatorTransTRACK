#pragma once
#include <stdint.h>

// Motor
void motor_init();
void motorStop();
void motorForward(int speedPWM);
void motorReverse(int speedPWM);