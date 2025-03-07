#ifndef INC_IMU_BNO055_H_
#define INC_IMU_BNO055_H_

#include "main.h"

// Sensor's I2C address
#define BNO055_I2C_ADDR 0x28 // Default I2C bno055 address (if ADR pin is low)
// Sensor's registers (partial list)
#define BNO055_CHIP_ID          	0x00  // Who am I
#define BNO055_PAGE_ID 				0x07 // Page ID register
#define BNO055_OPR_MODE        	 	0x3D  // Operation mode
#define BNO055_PWR_MODE        	 	0x3E  // Power mode
#define BNO055_UNIT_SEL        	 	0x3B  // Unità di misura
#define BNO055_AXIS_MAP_CONFIG  	0x41  // Configurazione assi
#define BNO055_VECTOR_EULER    		0x1A  // Registro per gli angoli di Eulero
#define BNO055_VECTOR_GRAVITY       0x2E  // Registro per la gravità
#define BNO055_LIA_DATA_X_LSB       0x28  // Registro per l'accelerazione lineare
#define BNO055_ACC_DATA_X_LSB   	0x08  // Accelerometro dati
#define BNO055_GYR_DATA_X_LSB   	0x14  // Giroscopio dati
#define BNO055_MAG_DATA_X_LSB   	0x0E  // Magnetometro dati
#define BNO055_SYS_TRIGGER      	0x3F  // Registro per il trigger di sistema
#define BNO055_VECTOR_QUATERNION    0x20  // Registro per i quaternioni

// Modalità operative
#define BNO055_MODE_CONFIG      0x00  // Modalità configurazione
#define BNO055_MODE_NDOF        0x0C  // Modalità 9DOF (acc+gyro+mag)

// EXTERNAL VARIABLES
extern I2C_HandleTypeDef *_bno055_i2c_port; // Main I2C handler
extern float magScale, accelScale, angularRateScale, eulerScale, quaScale;


//Definizione della struttura di un generico dato acquisito dalla IMU BNO055
typedef struct
{
    double w; //(per quaternioni)
    double x;
    double y;
    double z;
} bno055_vector_t;


// Definizione delle modalità operative del BNO055
typedef enum {
    BNO055_OPERATION_MODE_CONFIG = 0x00,
    BNO055_OPERATION_MODE_NDOF = 0x0C,
    // Aggiungere altre modalità operative se necessarie
} bno055_opmode_t;


/*
// Struttura per i dati IMU
typedef struct BNO055_Data {
    float Ax, Ay, Az;  // Accelerazione (m/s^2)
    float Gx, Gy, Gz;  // Giroscopio (°/s)
    float Mx, My, Mz;  // Magnetometro (µT)
} BNO055_Data;*/


// FUNCTION DECLARATIONS
void bno055_assignI2C(I2C_HandleTypeDef *hi2c_device); // Assegnazione dell'interfaccia I2C
void bno055_delay(uint16_t ms);
void bno055_readData(uint8_t reg, uint8_t *buffer, uint8_t length);
void bno055_writeData(uint8_t reg, uint8_t value);
void bno055_setPage(uint8_t page);
void bno055_reset(void);
void bno055_setOperationMode(bno055_opmode_t mode); // Imposta la modalità operativa del sensore
void bno055_setOperationModeConfig(void);
void bno055_setOperationModeNDOF(void);

void bno055_init(void); // Configurazione iniziale del sensore
bno055_vector_t bno055_getVector(uint8_t vec); // Legge i dati da un registro vettoriale specifico
bno055_vector_t bno055_getVectorEuler(void); // Legge i dati degli angoli di Eulero

//void bno055_read_sensors(BNO055_Data* imu_data);

#endif /* INC_IMU_BNO055_H_ */
