#include "IMU.h"
#include <stdio.h>

// Costanti di conversione per il sensore BNO055
#define ACC_LSB_TO_MS2    (1.0 / 100.0)  // Conversione accelerazione
#define GYR_LSB_TO_DPS    (1.0 / 16.0)   // Conversione giroscopio
#define MAG_LSB_TO_UT     (1.0 / 16.0)   // Conversione magnetometro

// Definizione delle variabili
I2C_HandleTypeDef *_bno055_i2c_port;  // Porta I2C del sensore BNO055

float magScale = 16;         // Scala per la lettura dei magnetometri
float accelScale = 100;      // Scala per la lettura degli accelerometri
float angularRateScale = 16; // Scala per la lettura delle velocità angolari
float eulerScale = 16;       // Scala per l'angolo di Eulero
float quaScale = (1<<14);    // Scala per la lettura della quaternion


void bno055_assignI2C(I2C_HandleTypeDef *hi2c_device) //Associa un'interfaccia I2C al driver
{
	_bno055_i2c_port = hi2c_device;
}

void bno055_delay(uint16_t ms)
{
    HAL_Delay(ms);
}

//Funzione per leggere i dati da un registro specifico del BNO055
//Argomenti:
// reg = indirizzo del registro che si vuole leggere nel BNO055
// puntatore ad un buffer (array) -> probabilmente una parte del registro dedicata a lettura/scrittura del dato
// lenght = Numero di byte da leggere dal registro specificato.
void bno055_readData(uint8_t reg, uint8_t *buffer, uint8_t length)
{
    // Indirizzo del dispositivo con bit di scrittura impostato
    uint16_t dev_addr = BNO055_I2C_ADDR << 1;
    // Sposta l'indirizzo di 1 bit verso sinistra
    // Il tipo di operazione (di lettura o scrittura) non deve essere specificato esplicitamente perché è gestito automaticamente dalla libreria HAL.
    // poichè in I2C il bit meno significativo (LSB) dell'indirizzo è riservato per indicare se si sta leggendo (1) o scrivendo (0) al dispositivo.

    // Legge i dati utilizzando la funzione HAL per la memoria I2C
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read
    (
        _bno055_i2c_port,  // Gestore I2C
        dev_addr,          // Indirizzo del dispositivo
        reg,               // Registro da leggere
        I2C_MEMADD_SIZE_8BIT, // Dimensione dell'indirizzo del registro
        buffer,            // Buffer in cui salvare i dati
        length,            // Numero di byte da leggere
        HAL_MAX_DELAY      // Timeout massimo
    );

    if (status != HAL_OK) {
        printf("Errore nella lettura del registro 0x%02X\n", reg);
    }
}


// Funzione per scrivere un valore in un registro specifico del BNO055
void bno055_writeData(uint8_t reg, uint8_t value)
{
    uint16_t dev_addr = BNO055_I2C_ADDR << 1;

    // Scrive il dato utilizzando la funzione HAL per la memoria I2C
    HAL_StatusTypeDef status = HAL_I2C_Mem_Write
    (
        _bno055_i2c_port,
        dev_addr,
        reg,               // Registro da scrivere
        I2C_MEMADD_SIZE_8BIT,
        &value,            // Puntatore al dato da scrivere
        1,                 // Numero di byte da scrivere (il sensore BNO055 è progettato per scrivere un byte alla volta nei suoi registri)
        HAL_MAX_DELAY
    );

    if (status != HAL_OK) {
        printf("Errore nella scrittura del registro 0x%02X\n", reg);
    }
}


// Funzione per impostare la pagina di configurazione del BNO055
void bno055_setPage(uint8_t page)
{
    // Scrive il valore della pagina nel registro PAGE_ID
    bno055_writeData(BNO055_PAGE_ID, page);
}


void bno055_reset(void) //Soft Reset command
{
    bno055_writeData(BNO055_SYS_TRIGGER, BNO055_VECTOR_QUATERNION); //L'indirizzo del registro System Trigger per resettare il registro del quaternione
    bno055_delay(650); // Attendi che il reset sia completato
}


void bno055_setOperationMode(bno055_opmode_t mode)//funzione che imposta la modalità operativa del sensore
{
    bno055_writeData(BNO055_OPR_MODE, mode);
    if (mode == BNO055_OPERATION_MODE_CONFIG)
    {
        bno055_delay(19); //Delay maggiore per la modalità iniziale di configurazione
    }
    else
    {
        bno055_delay(7);
    }
}


// Funzione per impostare il sensore in modalità di configurazione
void bno055_setOperationModeConfig() {
    bno055_setOperationMode(BNO055_OPERATION_MODE_CONFIG); //Ritardo maggiore
}


// Funzione per impostare il sensore in modalità NDOF
void bno055_setOperationModeNDOF() {
    bno055_setOperationMode(BNO055_OPERATION_MODE_NDOF);
}


void bno055_init() //funzione utilizzata per inizializzare il sensore BNO055
{
    bno055_reset();

    uint8_t id = 0;
    bno055_readData(BNO055_CHIP_ID, &id, 1);
    if (id != BNO055_CHIP_ID) {
        printf("Can't find BNO055, id: 0x%02x. Please check your wiring.\r\n", id);
    }
    //Imposta la pagina 0, poiché il BNO055 utilizza una struttura di registri a pagine
    bno055_setPage(0);
    bno055_writeData(BNO055_SYS_TRIGGER, 0x0);

    // Select BNO055 config mode
    //Mette il chip in modalità di configurazione, necessaria per modificare alcune impostazioni o inizializzarlo
    bno055_setOperationModeConfig();
    bno055_delay(10);
}


bno055_vector_t bno055_getVector(uint8_t vec)
{
    bno055_setPage(0);
    uint8_t buffer[8];  // I quaternioni richiedono 8 byte

    if (vec == BNO055_VECTOR_QUATERNION)
    {
        bno055_readData(vec, buffer, 8);
    } else
    {
        bno055_readData(vec, buffer, 6);
    }

    double scale = 1;

    //BNO055_VECTOR_ACCELEROMETER... sono definizioni preimpostate (costanti)
    //che rappresentano indirizzi specifici dei registri dell'IMU BNO055
    //In base al tipo di dato gli associa una scala (costante) che identifica una certa unità di misura
    //Unità di misura di: accelerazione, velocità angolare, gradi (per gli angoli di Eulero)
    if (vec == BNO055_MAG_DATA_X_LSB) //Misura il campo magnetico in micro Tesla
    {
        scale = magScale;
    } else if (vec == BNO055_ACC_DATA_X_LSB ||
               vec == BNO055_LIA_DATA_X_LSB ||
               vec == BNO055_VECTOR_GRAVITY) {
        scale = accelScale;
    } else if (vec == BNO055_GYR_DATA_X_LSB) {
        scale = angularRateScale;
    } else if (vec == BNO055_VECTOR_EULER) {
        scale = eulerScale;
    } else if (vec == BNO055_VECTOR_QUATERNION) {
        scale = quaScale; //Generica variazione di posizione nello spazio tridimensionale
    }

    bno055_vector_t xyz = {.w = 0, .x = 0, .y = 0, .z = 0};

    //Conversione da dati grezzi a dati identificati e puliti
    //L'informazione grezza è salvata su 8 bit di cui ogni 2 bit sono riservati rispettivamente
    //ai valori della grandezza relativi alle componenti x, y, z ed eventualmente w
    xyz.x = ((int16_t)((buffer[1] << 8) | buffer[0])) / scale;
    xyz.y = ((int16_t)((buffer[3] << 8) | buffer[2])) / scale;
    xyz.z = ((int16_t)((buffer[5] << 8) | buffer[4])) / scale;

    if (vec == BNO055_VECTOR_QUATERNION) //Se la grandezza è un quaternione
    {
        xyz.w = ((int16_t)((buffer[7] << 8) | buffer[6])) / scale;
    }

    return xyz;
}


bno055_vector_t bno055_getVectorEuler() //Legge i dati di orientamento del sensore in termini di angoli Euleriani (roll, pitch, yaw)
{
    return bno055_getVector(BNO055_VECTOR_EULER);
}
