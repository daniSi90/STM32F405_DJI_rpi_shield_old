/*
 * bno080.h
 *
 *  Created on: Jul 28, 2020
 *      Author: Danijel
 */

#ifndef LIBS_BNO080_H_
#define LIBS_BNO080_H_

#ifdef __cplusplus
 extern "C" {
#endif

#include "main.h"
#include "spi.h"

#include "quaternion.h"

/* Link With Hardware */
//#define BNO_SPI_CHANNEL SPI2
/* BNO CS Pin */
//#define BNO_CS_PORT  	GPIOA			// CHANGE 2
//#define BNO_CS_PIN 		GPIO_PIN_8		// CHANGE 3
/* BNO RST Pin */
//#define BNO_RST_PORT 	GPIOC			// CHANGE 4
//#define BNO_RST_PIN  	GPIO_PIN_7		// CHANGE 5
/* BNO Inter. Pin -- Falling Edge, PullUp Resistor */
//#define BNO_INT_PORT_F 	GPIOA			// CHANGE 6
//#define BNO_INT_PIN_F 	GPIO_PIN_10	    // CHANGE 7
//#define BNO_IRQN		EXTI15_10_IRQn  // CHANGE 8


#define BNO_SPI_CHANNEL SPI3
/* BNO CS Pin */
#define BNO_CS_PORT  	GPIOB			// CHANGE 2
#define BNO_CS_PIN 		GPIO_PIN_4		// CHANGE 3
/* BNO RST Pin */
#define BNO_RST_PORT 	GPIOB			// CHANGE 4
#define BNO_RST_PIN  	GPIO_PIN_5		// CHANGE 5
/* BNO Inter. Pin */
#define BNO_INT_PORT_F 	GPIOC			// CHANGE 6
#define BNO_INT_PIN_F 	GPIO_PIN_10	    // CHANGE 7
#define BNO_IRQN		EXTI15_10_IRQn  // CHANGE 8

#define BNO_SELECT   HAL_GPIO_WritePin(BNO_CS_PORT, BNO_CS_PIN, GPIO_PIN_RESET);
#define BNO_DESELECT HAL_GPIO_WritePin(BNO_CS_PORT, BNO_CS_PIN, GPIO_PIN_SET);

#define BNO_RESET    HAL_GPIO_WritePin(BNO_RST_PORT, BNO_RST_PIN, GPIO_PIN_RESET);
#define BNO_ENABLE   HAL_GPIO_WritePin(BNO_RST_PORT, BNO_RST_PIN, GPIO_PIN_SET);


/* Registers */
typedef enum
{
	CHANNEL_COMMAND = 0,
	CHANNEL_EXECUTABLE = 1,
	CHANNEL_CONTROL = 2,
	CHANNEL_REPORTS = 3,
	CHANNEL_WAKE_REPORTS = 4,
	CHANNEL_GYRO = 5
}registers;

typedef enum
{
	BNO_NOREAD = 0,
	BNO_READ = 1
}wait_cmd;


//All the ways we can configure or talk to the BNO080, figure 34, page 36 reference manual
//These are used for low level communication with the sensor, on channel 2
#define SHTP_REPORT_COMMAND_RESPONSE 0xF1
#define SHTP_REPORT_COMMAND_REQUEST 0xF2
#define SHTP_REPORT_FRS_READ_RESPONSE 0xF3
#define SHTP_REPORT_FRS_READ_REQUEST 0xF4
#define SHTP_REPORT_PRODUCT_ID_RESPONSE 0xF8
#define SHTP_REPORT_PRODUCT_ID_REQUEST 0xF9
#define SHTP_REPORT_BASE_TIMESTAMP 0xFB
#define SHTP_REPORT_SET_FEATURE_COMMAND 0xFD

//All the different sensors and features we can get reports from
//These are used when enabling a given sensor
#define SENSOR_REPORTID_ACCELEROMETER 0x01
#define SENSOR_REPORTID_GYROSCOPE 0x02
#define SENSOR_REPORTID_MAGNETIC_FIELD 0x03
#define SENSOR_REPORTID_LINEAR_ACCELERATION 0x04
#define SENSOR_REPORTID_ROTATION_VECTOR 0x05
#define SENSOR_REPORTID_GRAVITY 0x06
#define SENSOR_REPORTID_GAME_ROTATION_VECTOR 0x08
#define SENSOR_REPORTID_GEOMAGNETIC_ROTATION_VECTOR 0x09
#define SENSOR_REPORTID_GYRO_INTEGRATED_ROTATION_VECTOR 0x2A
#define SENSOR_REPORTID_TAP_DETECTOR 0x10
#define SENSOR_REPORTID_STEP_COUNTER 0x11
#define SENSOR_REPORTID_STABILITY_CLASSIFIER 0x13
#define SENSOR_REPORTID_RAW_ACCELEROMETER 0x14
#define SENSOR_REPORTID_RAW_GYROSCOPE 0x15
#define SENSOR_REPORTID_RAW_MAGNETOMETER 0x16
#define SENSOR_REPORTID_PERSONAL_ACTIVITY_CLASSIFIER 0x1E
#define SENSOR_REPORTID_AR_VR_STABILIZED_ROTATION_VECTOR 0x28
#define SENSOR_REPORTID_AR_VR_STABILIZED_GAME_ROTATION_VECTOR 0x29

//Record IDs from figure 29, page 29 reference manual
//These are used to read the metadata for each sensor type
#define FRS_RECORDID_ACCELEROMETER 0xE302
#define FRS_RECORDID_GYROSCOPE_CALIBRATED 0xE306
#define FRS_RECORDID_MAGNETIC_FIELD_CALIBRATED 0xE309
#define FRS_RECORDID_ROTATION_VECTOR 0xE30B

//Command IDs from section 6.4, page 42
//These are used to calibrate, initialize, set orientation, tare etc the sensor
#define COMMAND_ERRORS 1
#define COMMAND_COUNTER 2
#define COMMAND_TARE 3
#define COMMAND_INITIALIZE 4
#define COMMAND_DCD 6
#define COMMAND_ME_CALIBRATE 7
#define COMMAND_DCD_PERIOD_SAVE 9
#define COMMAND_OSCILLATOR 10
#define COMMAND_CLEAR_DCD 11

#define CALIBRATE_ACCEL 0
#define CALIBRATE_GYRO 1
#define CALIBRATE_MAG 2
#define CALIBRATE_PLANAR_ACCEL 3
#define CALIBRATE_ACCEL_GYRO_MAG 4
#define CALIBRATE_STOP 5

#define MAX_PACKET_SIZE 128 //Packets can be up to 32k but we don't have that much RAM.
#define MAX_METADATA_SIZE 9 //This is in words. There can be many but we mostly only care about the first 9 (Qs, range, etc)


/* Global Functions */
uint8_t bno080_Initialization(void);
void bno080_start_IT(void);
void bno080_stop_IT(void);
float bno080_qToFloat(int16_t fixedPointValue, uint8_t qPoint);
void bno080_enableRotationVector(uint16_t timeBetweenReports);
void bno080_enableGameRotationVector(uint16_t timeBetweenReports);
void bno080_enableAccelerometer(uint16_t timeBetweenReports);
void bno080_enableLinearAccelerometer(uint16_t timeBetweenReports);
void bno080_enableGyro(uint16_t timeBetweenReports);
void bno080_enableMagnetometer(uint16_t timeBetweenReports);
void bno080_enableStepCounter(uint16_t timeBetweenReports);
void bno080_enableStabilityClassifier(uint16_t timeBetweenReports);
void bno080_calibrateAccelerometer(void);
void bno080_calibrateGyro(void);
void bno080_calibrateMagnetometer(void);
void bno080_calibratePlanarAccelerometer(void);
void bno080_calibrateAll(void);
void bno080_endCalibration(void);

void parseCommandReport(void);
void parseInputReport(void);
float bno080_getQuatI(void);
float bno80_getQuatJ(void);
float bno080_getQuatK(void);
float bno080_getQuatReal(void);
float bno080_getQuatRadianAccuracy(void);
/*uint8_t BNO080_getQuatAccuracy();
float BNO080_getAccelX();
float BNO080_getAccelY();
float BNO080_getAccelZ();
uint8_t BNO080_getAccelAccuracy();
float BNO080_getLinAccelX();
float BNO080_getLinAccelY();
float BNO080_getLinAccelZ();
uint8_t BNO080_getLinAccelAccuracy();
float BNO080_getGyroX();
float BNO080_getGyroY();
float BNO080_getGyroZ();
uint8_t BNO080_getGyroAccuracy();
float BNO080_getMagX();
float BNO080_getMagY();
float BNO080_getMagZ();
uint8_t BNO080_getMagAccuracy();
uint16_t BNO080_getStepCount();
uint8_t BNO080_getStabilityClassifier();
uint8_t BNO080_getActivityClassifier();
uint32_t BNO080_getTimeStamp();
int16_t BNO080_getQ1(uint16_t recordID);
int16_t BNO080_getQ2(uint16_t recordID);
int16_t BNO080_getQ3(uint16_t recordID);
float BNO080_getResolution(uint16_t recordID);
float BNO080_getRange(uint16_t recordID);
*/

/* Internal Functions */
void setFeatureCommand(uint8_t reportID, uint32_t microsBetweenReports, uint32_t specificConfig);
void sendCommand(uint8_t command);
void sendCalibrateCommand(uint8_t thingToCalibrate);
void sendCalibrateCommand(uint8_t thingToCalibrate);
void requestCalibrationStatus();
void saveCalibration();

uint8_t waitForSPI(void);
uint8_t receivePacket(void);
uint8_t receivePacket_IT(void);
uint8_t sendPacket(uint8_t channelNumber, uint8_t dataLength);

uint8_t LL_SPI_SendByte(uint8_t data);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

#ifdef __cplusplus
}
#endif

#endif /* LIBS_BNO080_H_ */
