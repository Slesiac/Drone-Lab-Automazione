#include "PID.h"
#include <stdio.h>

// Definizione delle variabili
float b = 0.00001163; // coefficiente di spinta
float l = 0.3; // distanza tra motore e centro di massa (35 cm)
float d = 0.000001;//0.000008; // resistenza aerodinamica
// d = 0.000000038124

//Assegnamo i valori ai coefficienti della struct del PID
// Init e Tune PID
void PID_Init(PID* pid, float kp, float ki, float kd, float dt, float outMin, float outMax)
{
	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;
	pid->dt = dt;
	pid->Iterm = 0;
	pid->DtermFiltered = 0;
	pid->lastError = 0;
	pid->outMax = outMax;
	pid->outMin = outMin;
}


/*
//Funzione che calcola il valore che corrisponde al PID conoscendo l’errore in tempo reale
//PID calcolato ogni dt di tempo
float PID_Controller(PID* pid, float input, float setPoint)
{
    float output;
    float newIterm;

    float error = setPoint - input; //Differenza di angoli, tra il SetPoint (0° o -180°) e l'input (angolo di roll o pitch)

    float Pterm = pid->kp * error;
    newIterm = pid->Iterm + (pid->ki)*pid->dt * pid->lastError;
    float Dterm = (pid->kd/pid->dt) * (error - pid->lastError);

    pid->lastError = error;

    output = Pterm + newIterm + Dterm;

    if(output > pid->outMax)
    {
    	output = pid->outMax; // upper limit saturation
    }
    else if (output < pid->outMin)
    {
    	output = pid->outMin; // lower limit saturation
    }
    else
    {
        pid->Iterm= newIterm; // clamping anti-windup
    }

    return output;
}*/


// Funzione PID con filtro passa-basso sul termine derivativo
float PID_Controller(PID* pid, float input, float setPoint)
{
    float output;
    float newIterm;

    float error = setPoint - input;

    float Pterm = pid->kp * error;

    newIterm = pid->Iterm + (pid->ki) * pid->dt * pid->lastError;

    float Dterm = (pid->kd/pid->dt) * (error - pid->lastError);

    /*
    // FILTRO
    // Termine derivativo (filtrato passa-basso)
    float DtermRaw = 0;
    if (pid->dt > 0) {  // Evita divisione per zero
        DtermRaw = (pid->kd / pid->dt) * (error - pid->lastError);
    }

    // Applica filtro passa-basso (α = 0.8 per smorzare il rumore)
    pid->DtermFiltered = 0.8 * pid->DtermFiltered + 0.2 * DtermRaw;
    float Dterm = pid->DtermFiltered;
    */

    pid->lastError = error;

    output = Pterm + newIterm + Dterm;

    // Saturazione dell'output & anti-windup
    if (output > pid->outMax) {
        output = pid->outMax;
    } else if (output < pid->outMin) {
        output = pid->outMin;
    } else {
        pid->Iterm = newIterm; // Applica anti-windup
    }

    return output;
}


float* SpeedCompute(float virtualInputs[])
{
    static float Speeds_quad[4];
    static float Speeds[4];

    Speeds_quad[0] = (1/(4*b))*virtualInputs[0] - (1/(2*l*b))*virtualInputs[2] + (1/(4*d))*virtualInputs[3];
	Speeds_quad[1] = (1/(4*b))*virtualInputs[0] - (1/(2*l*b))*virtualInputs[1] - (1/(4*d))*virtualInputs[3];
	Speeds_quad[2] = (1/(4*b))*virtualInputs[0] + (1/(2*l*b))*virtualInputs[2] + (1/(4*d))*virtualInputs[3];
	Speeds_quad[3] = (1/(4*b))*virtualInputs[0] + (1/(2*l*b))*virtualInputs[1] - (1/(4*d))*virtualInputs[3];

    /*
     * Calcoliamo le velocità dei motori al quadrato, poiché non possono essere negative.
     * Partendo dal valore di throttle e seguendo le matrici di controllo dei droni,
     * andiamo a sommare e sottrarre le variabili date tramite il PID per il controllo delle velocità.
     */

    Speeds[0] = sqrt(Speeds_quad[0]);
    Speeds[1] = sqrt(Speeds_quad[1]);
    Speeds[2] = sqrt(Speeds_quad[2]);
    Speeds[3] = sqrt(Speeds_quad[3]);

    // Una volta calcolata la velocità dei motori al quadrato, viene eseguita la radice

    return Speeds;
}
