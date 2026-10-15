#pragma once

#include <stdint.h>

// Initialize Pitch Motor GPIOs (R_EN, L_EN) and LEDC PWM channels (Timer 1, 20 kHz, 10-bit)
void pitch_motor_init();

// Active low-side braking: R_EN=1, L_EN=1, RPWM=0, LPWM=0 (shorts motor windings to GND)
void pitch_motor_brake();

// Signed drive: duty range -1023 to +1023 (+ = positive tilt / FWD, - = negative tilt / REV)
void pitch_motor_drive(int duty);

// Stop motor (engages active low-side brake)
void pitch_motor_stop();
