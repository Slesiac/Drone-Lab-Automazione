#ifndef INC_PID_H_
#define INC_PID_H_

#include "main.h"

// Dichiarazione delle variabili globali (se necessarie)
extern float b; // coefficiente di spinta
extern float l; // distanza tra motore e centro di massa
extern float d; // coefficiente di resistenza aerodinamica

// Struttura per i parametri del PID
typedef struct PID
{
	//TUNING PARAM (parallel form)
    float kp; // Costante proporzionale
    float ki; // Costante derivativa
    float kd; // Costante integrativa
    //OTHER PARAM
    float dt; // Intervallo di tempo tra le iterazioni
    float Iterm; // Componente integrativa accumulata
    float DtermFiltered; //Filtro passa-basso
    float lastError; // Errore dell'iterazione precedente
    float outMax; // Limite massimo dell'uscita
    float outMin; // Limite minimo dell'uscita
} PID;

// Dichiarazione delle funzioni
void PID_Init(PID* pid, float kp, float ki, float kd, float dt, float outMin, float outMax);
float PID_Controller(PID* pid, float input, float setPoint);
float* SpeedCompute(float virtualInputs[]);

#endif // INC_PID_H_
