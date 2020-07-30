#include "TR_One_HAL.h"

HAL_StatusTypeDef wrt; // primoz
uint8_t val[3];
uint8_t aa;
uint8_t addr[1];  // primoz
uint16_t datr;


TERAONE_Result TrOne_WhoAmI(TRONE_Str *senStruct) {
	
	/* Check if device is ready for communication */
	//if (HAL_I2C_IsDeviceReady(senStruct->i2cHandle, senStruct->Address, 3, 200) != HAL_OK) {
		/* Return error */
	//	return TERAONE_ERROR;
	//}
	
	/* Check who am I */
	HAL_Delay(1000);
	//val[0]	= TRONE_WHO_AM_I;
	//uint8_t who_i = TRONE_WHO_AM_I;
	HAL_I2C_Mem_Read(senStruct->i2cHandle, (uint16_t)senStruct->Address, TRONE_WHO_AM_I, 1, val, 4, 500);
	
	if (val[0] != TRONE_I_AM) {
		/* Return error */
		return TERAONE_ERROR;
	}
	
  return TERAONE_OK;
}

TERAONE_Result TrOne_ReadDist(TRONE_Str *senStruct){
	
	
	HAL_I2C_Master_Receive(senStruct->i2cHandle, (uint16_t)senStruct->Address, val, 3, 300);
	
	//HAL_I2C_Mem_Read(senStruct->i2cHandle, (uint16_t)senStruct->Address, TRONE_WHO_AM_I, 1, val, 4, 500);
	senStruct->distance = (uint16_t)((val[0] << 8) | val[1]);
	aa = TrOne_crc8(val, 2);
	if(aa != val[2]){
		return TERAONE_ERROR;
	}
	
	return TERAONE_OK;
}

TERAONE_Result TrOne_ChangeBaseAddr(TRONE_Str *senStruct) {  //primoz

	addr[0] = 0x40;
	//wrt = HAL_I2C_Master_Transmit(senStruct->i2cHandle, (uint16_t)TRONE_CHA_ADDR, buf, 1, HAL_MAX_DELAY);
	//HAL_I2C_Mem_Write(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout)
	wrt = HAL_I2C_Mem_Write(senStruct->i2cHandle,(uint16_t)senStruct->Address, (uint16_t)TRONE_CHA_ADDR, (uint16_t)1 , addr, (uint16_t)1, 1000);

	if (wrt != HAL_OK) {
		/* Return error */
		return TERAONE_ERROR;
	}

  return TERAONE_OK;
}


/*
 * Brief : Calculate a Cyclic Redundancy Checks of 8 bits
 * Param1 : (*p) pointer to receive buffer
 * Param2 : (len) number of bytes returned by the TeraRanger
 * Return : (crc & 0xFF) checksum calculated locally
 */
uint8_t TrOne_crc8(uint8_t *p, uint8_t len) {
  uint8_t i;
  uint8_t crc = 0x0;
  while (len--) {
    i = (crc ^ *p++) & 0xFF;
    crc = (crc_table[i] ^ (crc << 8)) & 0xFF;
  }
  return crc & 0xFF;
}

