#include "map.h"
#include "IMU.h"
#include "PID.h"

//Funzione che dati i duty cycle per ogni canale del timer associa i 4 rispettivi valori ai registri CCR per creare i segnali PWM
void setPwm(float pwm1, float pwm2, float pwm3, float pwm4)
{
	//pwmX è un duty cycle in percentuale
	pwm1 = pwm1 > MAX_DUTY ? MAX_DUTY : pwm1 < MIN_DUTY ? MIN_DUTY : pwm1; //Condizione con operatore ternario
	pwm2 = pwm2 > MAX_DUTY ? MAX_DUTY : pwm2 < MIN_DUTY ? MIN_DUTY : pwm2;
	pwm3 = pwm3 > MAX_DUTY ? MAX_DUTY : pwm3 < MIN_DUTY ? MIN_DUTY : pwm3;
	pwm4 = pwm4 > MAX_DUTY ? MAX_DUTY : pwm4 < MIN_DUTY ? MIN_DUTY : pwm4;

	TIM3->CCR1 = (uint32_t)(TIM3->ARR * pwm1/100);
	TIM3->CCR2 = (uint32_t)(TIM3->ARR * pwm2/100);
	TIM3->CCR3 = (uint32_t)(TIM3->ARR * pwm3/100);
	TIM3->CCR4 = (uint32_t)(TIM3->ARR * pwm4/100);
}

//Stessa funzione di prima, senza il controllo, per calibrare correttamente i motori
void setPwmNoControl(float pwm1, float pwm2, float pwm3, float pwm4)
{
	TIM3->CCR1 = (uint32_t)(TIM3->ARR * pwm1/100);
	TIM3->CCR2 = (uint32_t)(TIM3->ARR * pwm2/100);
	TIM3->CCR3 = (uint32_t)(TIM3->ARR * pwm3/100);
	TIM3->CCR4 = (uint32_t)(TIM3->ARR * pwm4/100);
}

void ESC_Calibrate()
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET); //Led verde
	setPwmNoControl(MAX_CALIB_DUTY, MAX_CALIB_DUTY, MAX_CALIB_DUTY, MAX_CALIB_DUTY); //10
	HAL_Delay(3000); //ATTACCARE BATTERIA PRIMA CHE SI SPENGA IL LED VERDE
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
	setPwmNoControl(MIN_CALIB_DUTY, MIN_CALIB_DUTY, MIN_CALIB_DUTY, MIN_CALIB_DUTY); //5
}

float calcoloDuty(float speed)
{
	float duty = (((MAX_DUTY - MIN_DUTY)*speed) + ((MIN_DUTY * MAX_SPEED)-(MAX_DUTY * MIN_SPEED)))/(MAX_SPEED - MIN_SPEED);

	if (duty < MIN_DUTY) duty = MIN_DUTY;
	else if (duty > MAX_DUTY) duty = MAX_DUTY;
	return duty;
}

void armMotors()
{
	setPwmNoControl(ARM_DUTY, ARM_DUTY, ARM_DUTY, ARM_DUTY);
}


void stopMotors()
{
	setPwmNoControl(MIN_CALIB_DUTY, MIN_CALIB_DUTY, MIN_CALIB_DUTY, MIN_CALIB_DUTY);
}
