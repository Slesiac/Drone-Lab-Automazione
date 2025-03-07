#ifndef RECEIVER_H
#define RECEIVER_H

#include "main.h"
#include "stm32h7xx_hal.h"
#include <stdint.h>

// Definizione del numero massimo di canali supportati
#define RECEIVER_NUM_CHANNELS 4

// Definizione della struttura per la frequenza del segnale
typedef struct {
    float frequency;      // Frequenza del segnale PWM
    uint8_t firstCaptured; // Flag per la prima cattura
    uint32_t val;         // Valore catturato
} FrequencyStruct;

// Definizione della struttura per il duty cycle
typedef struct {
    float duty;           // Duty cycle in percentuale
    float usWidth; //Tempo in microsecondi di salita del segnale di duty
    uint8_t firstCaptured; // Flag per la prima cattura
    uint32_t val;         // Valore catturato
} DutyCycleStruct;

// Variabili globali
extern FrequencyStruct chFrequency;
extern DutyCycleStruct chDuty[RECEIVER_NUM_CHANNELS];

// Funzioni dichiarate
void Receiver_Init(FrequencyStruct *chFrequency, DutyCycleStruct *chDuty, uint8_t numChannels);
void waitingForGettingFrequency(void);
void jumpHalfPeriod(float frequency);
void startInputCaptureInterruptDutyCycle(void);
void startInputCaptureInterruptPitchRoll(void);

#endif /* RECEIVER_H */
