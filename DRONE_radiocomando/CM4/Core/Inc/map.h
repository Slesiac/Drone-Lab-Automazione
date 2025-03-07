#ifndef MAP_H
#define MAP_H

#define MIN_CALIB_DUTY 5
#define MAX_CALIB_DUTY 10
#define ARM_DUTY 5.3
#define MIN_DUTY 5.4
#define MAX_DUTY 6.6
#define MIN_SPEED 366
#define MAX_SPEED 800

#include "main.h"

// Funzioni
void setPwm(float pwm1, float pwm2, float pwm3, float pwm4); //Imposta i duty cycle dei motori sui registri CCR del timer
void setPwmNoControl(float pwm1, float pwm2, float pwm3, float pwm4);
void ESC_Calibrate();
float calcoloDuty(float speed);
void armMotors();
void stopMotors();

#endif // MAP_H
