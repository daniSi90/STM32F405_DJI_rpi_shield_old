/*
 * bno080.c
 *
 *  Created on: Jul 28, 2020
 *      Author: Danijel
 */

#include "bno080.h"

/* Global Variables */
uint8_t shtpHeader[4]; //Each packet has a header of 4 bytes
uint8_t shtpData[MAX_PACKET_SIZE];
uint8_t sequenceNumber[6] = {0, 0, 0, 0, 0, 0}; //There are 6 com channels. Each channel has its own seqnum
uint8_t commandSequenceNumber = 0;				//Commands have a seqNum as well. These are inside command packet, the header uses its own seqNum per channel
uint32_t metaData[MAX_METADATA_SIZE];			//There is more than 10 words in a metadata record but we'll stop at Q point 3

/* These are the raw sensor values pulled from the user requested Input Report */
uint16_t rawAccelX, rawAccelY, rawAccelZ, accelAccuracy;
uint16_t rawLinAccelX, rawLinAccelY, rawLinAccelZ, accelLinAccuracy;
uint16_t rawGyroX, rawGyroY, rawGyroZ, gyroAccuracy;
uint16_t rawMagX, rawMagY, rawMagZ, magAccuracy;
uint16_t rawQuatI=0, rawQuatJ=0, rawQuatK=0, rawQuatReal=0, rawQuatRadianAccuracy=0, quatAccuracy=0;
uint16_t stepCount;
uint32_t timeStamp;
uint8_t stabilityClassifier;
uint8_t activityClassifier;
uint8_t *_activityConfidences; //Array that store the confidences of the 9 possible activities
uint8_t calibrationStatus;	 //Byte R0 of ME Calibration Response

/* These Q values are defined in the datasheet but can also be obtained by querying the meta data records
 * See the read metadata example for more info
 */
int16_t rotationVector_Q1 = 14;
int16_t accelerometer_Q1 = 8;
int16_t linear_accelerometer_Q1 = 8;
int16_t gyro_Q1 = 9;
int16_t magnetometer_Q1 = 4;

//VARIABLEs
float q[4]={0,0,0,10};
float quatRadianAccuracy;


volatile uint8_t bnoIntFl = 0;
volatile uint8_t constantRead = 0;


uint8_t bno080_Initialization(void)
{
	HAL_NVIC_DisableIRQ(BNO_IRQN);

	BNO_DESELECT;
	BNO_RESET;
	HAL_Delay(10);
	BNO_ENABLE;

	//Wait for first assertion of INT before using WAK pin. Can take ~104ms
	if(waitForSPI(BNO_NOREAD) != 1)
	{
		//return 0;
	}

	//At system startup, the hub must send its full advertisement message (see 5.2 and 5.3) to the
	//host. It must not send any other data until this step is complete.
	//When bno080 first boots it broadcasts big startup packet
	//Read it and dump it
	if(waitForSPI(BNO_READ) != 1) //Wait for assertion of INT before reading advert message.
	{
		//return 0;
	}
	//receivePacket();

	//The bno080 will then transmit an unsolicited Initialize Response (see 6.4.5.2)
	//Read it and dump it
	if(waitForSPI(BNO_READ) != 1) //Wait for assertion of INT before reading Init response
	{
		//return 0;
	}
	//receivePacket();

	//Check communication with device
	shtpData[0] = SHTP_REPORT_PRODUCT_ID_REQUEST; //Request the product ID and reset info
	shtpData[1] = 0;							  //Reserved

	//Transmit packet on channel 2, 2 bytes
	sendPacket(CHANNEL_CONTROL, 2);

	//Now we wait for response
	if(waitForSPI(BNO_READ) != 1)
	{
		//return 0;
	}

	if (shtpData[0] == SHTP_REPORT_PRODUCT_ID_RESPONSE){
		//return 1;
	}

	return 0; //Something went wrong
}

/*
 * Start Reading in Interrupt Mode
 * */
void bno080_start_IT(void)
{
	constantRead = 1;
	HAL_NVIC_EnableIRQ(BNO_IRQN);
}

//Given a register value and a Q point, convert to float
//See https://en.wikipedia.org/wiki/Q_(number_format)
float bno080_qToFloat(int16_t fixedPointValue, uint8_t qPoint)
{
	return fixedPointValue * powf(2, qPoint * -1);
}

//Sends the packet to enable the rotation vector
void bno080_enableRotationVector(uint16_t timeBetweenReports)
{
	setFeatureCommand(SENSOR_REPORTID_ROTATION_VECTOR, timeBetweenReports, 0);
}

//Sends the packet to enable the rotation vector
void bno080_enableGameRotationVector(uint16_t timeBetweenReports)
{
	setFeatureCommand(SENSOR_REPORTID_GAME_ROTATION_VECTOR, timeBetweenReports, 0);
}

//Sends the packet to enable the accelerometer
void bno080_enableAccelerometer(uint16_t timeBetweenReports)
{
	setFeatureCommand(SENSOR_REPORTID_ACCELEROMETER, timeBetweenReports, 0);
}

//Sends the packet to enable the accelerometer
void bno080_enableLinearAccelerometer(uint16_t timeBetweenReports)
{
	setFeatureCommand(SENSOR_REPORTID_LINEAR_ACCELERATION, timeBetweenReports, 0);
}

//Sends the packet to enable the gyro
void bno080_enableGyro(uint16_t timeBetweenReports)
{
	setFeatureCommand(SENSOR_REPORTID_GYROSCOPE, timeBetweenReports, 0);
}

//Sends the packet to enable the magnetometer
void bno080_enableMagnetometer(uint16_t timeBetweenReports)
{
	setFeatureCommand(SENSOR_REPORTID_MAGNETIC_FIELD, timeBetweenReports, 0);
}

//Sends the packet to enable the step counter
void bno080_enableStepCounter(uint16_t timeBetweenReports)
{
	setFeatureCommand(SENSOR_REPORTID_STEP_COUNTER, timeBetweenReports, 0);
}

//Sends the packet to enable the Stability Classifier
void bno080_enableStabilityClassifier(uint16_t timeBetweenReports)
{
	setFeatureCommand(SENSOR_REPORTID_STABILITY_CLASSIFIER, timeBetweenReports, 0);
}

//Sends the commands to begin calibration of the accelerometer
void bno080_calibrateAccelerometer(void)
{
	sendCalibrateCommand(CALIBRATE_ACCEL);
}

//Sends the commands to begin calibration of the gyro
void bno080_calibrateGyro(void)
{
	sendCalibrateCommand(CALIBRATE_GYRO);
}

//Sends the commands to begin calibration of the magnetometer
void bno080_calibrateMagnetometer(void)
{
	sendCalibrateCommand(CALIBRATE_MAG);
}

//Sends the commands to begin calibration of the planar accelerometer
void bno080_calibratePlanarAccelerometer(void)
{
	sendCalibrateCommand(CALIBRATE_PLANAR_ACCEL);
}

//See 2.2 of the Calibration Procedure document 1000-4044
void bno080_calibrateAll(void)
{
	sendCalibrateCommand(CALIBRATE_ACCEL_GYRO_MAG);
}

void bno080_endCalibration(void)
{
	sendCalibrateCommand(CALIBRATE_STOP); //Disables all calibrations
}


//Given a sensor's report ID, this tells the BNO080 to begin reporting the values
//Also sets the specific config word. Useful for personal activity classifier
void setFeatureCommand(uint8_t reportID, uint32_t microsBetweenReports, uint32_t specificConfig)
{
	shtpData[0] = SHTP_REPORT_SET_FEATURE_COMMAND;	 //Set feature command. Reference page 55
	shtpData[1] = reportID;						 //Feature Report ID. 0x01 = Accelerometer, 0x05 = Rotation vector
	shtpData[2] = 0;							 //Feature flags
	shtpData[3] = 0;							 //Change sensitivity (LSB)
	shtpData[4] = 0;							 //Change sensitivity (MSB)
	shtpData[5] = (microsBetweenReports >> 0) & 0xFF;  //Report interval (LSB) in microseconds. 0x7A120 = 500ms
	shtpData[6] = (microsBetweenReports >> 8) & 0xFF;  //Report interval
	shtpData[7] = (microsBetweenReports >> 16) & 0xFF; //Report interval
	shtpData[8] = (microsBetweenReports >> 24) & 0xFF; //Report interval (MSB)
	shtpData[9] = 0;							 //Batch Interval (LSB)
	shtpData[10] = 0;							 //Batch Interval
	shtpData[11] = 0;							 //Batch Interval
	shtpData[12] = 0;							 //Batch Interval (MSB)
	shtpData[13] = (specificConfig >> 0) & 0xFF;	   	 //Sensor-specific config (LSB)
	shtpData[14] = (specificConfig >> 8) & 0xFF;	   	 //Sensor-specific config
	shtpData[15] = (specificConfig >> 16) & 0xFF;	 //Sensor-specific config
	shtpData[16] = (specificConfig >> 24) & 0xFF;	 //Sensor-specific config (MSB)

	//Transmit packet on channel 2, 17 bytes
	sendPacket(CHANNEL_CONTROL, 17);
}






//Tell the sensor to do a command
//See 6.3.8 page 41, Command request
//The caller is expected to set P0 through P8 prior to calling
void sendCommand(uint8_t command)
{
	shtpData[0] = SHTP_REPORT_COMMAND_REQUEST; //Command Request
	shtpData[1] = commandSequenceNumber++;	 //Increments automatically each function call
	shtpData[2] = command;					   //Command

	//Caller must set these
	/*shtpData[3] = 0; //P0
	shtpData[4] = 0; //P1
	shtpData[5] = 0; //P2
	shtpData[6] = 0;
	shtpData[7] = 0;
	shtpData[8] = 0;
	shtpData[9] = 0;
	shtpData[10] = 0;
	shtpData[11] = 0;*/

	//Transmit packet on channel 2, 12 bytes
	sendPacket(CHANNEL_CONTROL, 12);
}

//This tells the bno080 to begin calibrating
//See page 50 of reference manual and the 1000-4044 calibration doc
void sendCalibrateCommand(uint8_t thingToCalibrate)
{
	/*shtpData[3] = 0; //P0 - Accel Cal Enable
	shtpData[4] = 0; //P1 - Gyro Cal Enable
	shtpData[5] = 0; //P2 - Mag Cal Enable
	shtpData[6] = 0; //P3 - Subcommand 0x00
	shtpData[7] = 0; //P4 - Planar Accel Cal Enable
	shtpData[8] = 0; //P5 - Reserved
	shtpData[9] = 0; //P6 - Reserved
	shtpData[10] = 0; //P7 - Reserved
	shtpData[11] = 0; //P8 - Reserved*/

	for (uint8_t x = 3; x < 12; x++) //Clear this section of the shtpData array
		shtpData[x] = 0;

	if (thingToCalibrate == CALIBRATE_ACCEL)
		shtpData[3] = 1;
	else if (thingToCalibrate == CALIBRATE_GYRO)
		shtpData[4] = 1;
	else if (thingToCalibrate == CALIBRATE_MAG)
		shtpData[5] = 1;
	else if (thingToCalibrate == CALIBRATE_PLANAR_ACCEL)
		shtpData[7] = 1;
	else if (thingToCalibrate == CALIBRATE_ACCEL_GYRO_MAG)
	{
		shtpData[3] = 1;
		shtpData[4] = 1;
		shtpData[5] = 1;
	}
	else if (thingToCalibrate == CALIBRATE_STOP)
		; //Do nothing, bytes are set to zero

	//Make the internal calStatus variable non-zero (operation failed) so that user can test while we wait
	calibrationStatus = 1;

	//Using this shtpData packet, send a command
	sendCommand(COMMAND_ME_CALIBRATE);
}

//Request ME Calibration Status from bno080
//See page 51 of reference manual
void requestCalibrationStatus()
{
	/*shtpData[3] = 0; //P0 - Reserved
	shtpData[4] = 0; //P1 - Reserved
	shtpData[5] = 0; //P2 - Reserved
	shtpData[6] = 0; //P3 - 0x01 - Subcommand: Get ME Calibration
	shtpData[7] = 0; //P4 - Reserved
	shtpData[8] = 0; //P5 - Reserved
	shtpData[9] = 0; //P6 - Reserved
	shtpData[10] = 0; //P7 - Reserved
	shtpData[11] = 0; //P8 - Reserved*/

	for (uint8_t x = 3; x < 12; x++) //Clear this section of the shtpData array
		shtpData[x] = 0;

	shtpData[6] = 0x01; //P3 - 0x01 - Subcommand: Get ME Calibration

	//Using this shtpData packet, send a command
	sendCommand(COMMAND_ME_CALIBRATE);
}

//This tells the bno080 to save the Dynamic Calibration Data (DCD) to flash
//See page 49 of reference manual and the 1000-4044 calibration doc
void saveCalibration()
{
	/*shtpData[3] = 0; //P0 - Reserved
	shtpData[4] = 0; //P1 - Reserved
	shtpData[5] = 0; //P2 - Reserved
	shtpData[6] = 0; //P3 - Reserved
	shtpData[7] = 0; //P4 - Reserved
	shtpData[8] = 0; //P5 - Reserved
	shtpData[9] = 0; //P6 - Reserved
	shtpData[10] = 0; //P7 - Reserved
	shtpData[11] = 0; //P8 - Reserved*/

	for (uint8_t x = 3; x < 12; x++) //Clear this section of the shtpData array
		shtpData[x] = 0;

	//Using this shtpData packet, send a command
	sendCommand(COMMAND_DCD); //Save DCD command
}


//Return the rotation vector quaternion I
float bno080_getQuatI(void)
{
	return bno080_qToFloat(rawQuatI, rotationVector_Q1);
}

//Return the rotation vector quaternion J
float bno080_getQuatJ(void)
{
	return bno080_qToFloat(rawQuatJ, rotationVector_Q1);
}

//Return the rotation vector quaternion K
float bno080_getQuatK(void)
{
	return bno080_qToFloat(rawQuatK, rotationVector_Q1);
}

//Return the rotation vector quaternion Real
float bno080_getQuatReal(void)
{
	return bno080_qToFloat(rawQuatReal, rotationVector_Q1);
}

//Return the rotation vector accuracy
float bno080_getQuatRadianAccuracy(void)
{
	return bno080_qToFloat(rawQuatRadianAccuracy, rotationVector_Q1);
}




/*
uint8_t waitForSPI(wait_cmd val)
{
	uint32_t tickstart = HAL_GetTick();
	bnoIntFl = 0;

	while(bnoIntFl == 0){
		if((HAL_GetTick() - tickstart) > 300){
			return 0;
		}
	}

	if(val == BNO_READ){
		receivePacket();
	}

	return 1;
}

*/


uint8_t waitForSPI(wait_cmd val)
{
	uint32_t tickstart = HAL_GetTick();

	while(HAL_GPIO_ReadPin(BNO_INT_PORT_F, BNO_INT_PIN_F)){
		if((HAL_GetTick() - tickstart) > 300){
			return 0;
		}
	}

	if(val == BNO_READ){
		receivePacket();
	}

	return 1;
}



uint8_t receivePacket(void)
{

	BNO_SELECT;
	//Get the first four bytes, aka the packet header
	HAL_SPI_Receive(BNO_SPI, shtpHeader, 4, 100);

	//Calculate the number of data bytes in this packet
	uint16_t dataLength = ((uint16_t)shtpHeader[1] << 8 | shtpHeader[0]);
	dataLength &= ~(1 << 15); //Clear the MSbit.
	//This bit indicates if this package is a continuation of the last. Ignore it for now.
	//TODO catch this as an error and exit
	if (dataLength == 0)
	{
		//Packet is empty
		return 0; //All done
	}
	dataLength -= 4; //Remove the header bytes from the data count

	if(dataLength < MAX_PACKET_SIZE){
		HAL_SPI_Receive(BNO_SPI, shtpData, dataLength, 200);
	}else{
		HAL_SPI_Receive(BNO_SPI, shtpData, MAX_PACKET_SIZE, 200);

		uint16_t data_left = dataLength - MAX_PACKET_SIZE;
		uint8_t tmp_buff;
		for(int i=0; i < data_left; i++){
			HAL_SPI_Receive(BNO_SPI, &tmp_buff, 1, 100); /// ??
		}

	}

	BNO_DESELECT;

	return 0;
}

uint8_t receivePacket_IT(void)
{

	BNO_SELECT;
	//Get the first four bytes, aka the packet header
	HAL_SPI_Receive(BNO_SPI, shtpHeader, 4, 100);

	//Calculate the number of data bytes in this packet
	uint16_t dataLength = ((uint16_t)shtpHeader[1] << 8 | shtpHeader[0]);
	dataLength &= ~(1 << 15); //Clear the MSbit.
	//This bit indicates if this package is a continuation of the last. Ignore it for now.
	//TODO catch this as an error and exit
	if (dataLength == 0)
	{
		//Packet is empty
		return 0; //All done
	}
	dataLength -= 4; //Remove the header bytes from the data count

	if(dataLength < MAX_PACKET_SIZE){
		HAL_SPI_Receive(BNO_SPI, shtpData, dataLength, 200);
	}else{
		HAL_SPI_Receive(BNO_SPI, shtpData, MAX_PACKET_SIZE, 200);

		uint16_t data_left = dataLength - MAX_PACKET_SIZE;
		uint8_t tmp_buff;
		for(int i=0; i < data_left; i++){
			HAL_SPI_Receive(BNO_SPI, &tmp_buff, 1, 100); /// ??
		}

	}

	BNO_DESELECT;

	return 0;
}



//Given the data packet, send the header then the data
//Returns false if sensor does not ACK
//TODO - Arduino has a max 32 byte send. Break sending into multi packets if needed.
uint8_t sendPacket(uint8_t channelNumber, uint8_t dataLength)
{
	uint8_t packetLength = dataLength + 4; //Add four bytes for the header

	//Wait for bno080 to indicate it is available for communication
	if(waitForSPI(BNO_NOREAD) == 0)
	{
		return 0; //Data is not available
	}

	uint8_t bno_buffer[4] = {packetLength & 0xFF, packetLength >> 8, channelNumber, sequenceNumber[channelNumber]++};

	BNO_SELECT;
	//Send the 4 byte packet header
	HAL_SPI_Transmit(BNO_SPI, bno_buffer, 4, 200);
	//Send the user's data packet
	HAL_SPI_Transmit(BNO_SPI, shtpData, dataLength, 200);

	BNO_DESELECT;
	return 0;
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == BNO_INT_PIN_F){
		bnoIntFl = 1;

		// Read data constantly after each interrupt
		if(constantRead)
		{
			receivePacket();
			q[0] = bno080_getQuatI();
			q[1] = bno080_getQuatJ();
			q[2] = bno080_getQuatK();
			q[3] = bno080_getQuatReal();
			quatRadianAccuracy = bno080_getQuatRadianAccuracy();

			Quaternion_Update(&q[0]);
		}

	}
}
