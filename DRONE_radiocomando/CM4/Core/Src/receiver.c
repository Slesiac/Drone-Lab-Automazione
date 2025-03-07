#include <receiver.h>
#include "stm32h7xx_hal.h"

FrequencyStruct chFrequency;
DutyCycleStruct chDuty[RECEIVER_NUM_CHANNELS];

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim5;

void Receiver_Init(FrequencyStruct *chFrequency, DutyCycleStruct *chDuty, uint8_t NUMBER_CHANNELS)
{
    chFrequency->frequency = 0;
    chFrequency->firstCaptured = 0;
    for (uint8_t i = 0; i < NUMBER_CHANNELS; i++) {
        chDuty[i].duty = 0;
        chDuty[i].usWidth = 0.0;
        chDuty[i].firstCaptured = 0;
    }
}

//Funzione di attesta stabilizzazione segnale
void waitingForGettingFrequency(void) {
    while (chFrequency.frequency < 48 || chFrequency.frequency > 52) //Fin quando non riceve la giusta frequenza di 50Hz
    {
        HAL_Delay(10);
    }
}

void jumpHalfPeriod(float frequency)
{
    float period = 1.0f / frequency;
    HAL_Delay((uint16_t)(period * 500)); // Aspetta metà periodo
}

void startInputCaptureInterruptDutyCycle(void)
{
    HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
}

void startInputCaptureInterruptPitchRoll(void)
{
    HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_3); //Nope Channel 3(?)
}
