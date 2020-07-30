/*
 * gnss_connector.h
 *
 *  Created on: Jun 10, 2020
 *      Author: danijel
 */

#ifndef INC_GNSS_WRAPPER_H_
#define INC_GNSS_WRAPPER_H_

#include "zedf9p/basic_types.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// GNSS Structure
typedef struct CGNSS_c{
    Pos pos;
	PosRel relPos;
	int fixType;
	float hAcc;
	float vAcc;
    float iTOW;
    int UTCyear;
    int UTCmonth;
    int UTCday;
    int UTChour;
    int UTCminute;
    int UTCsecond;
    bool parseUBX;
	int msgs;
}CGNSS_c;

typedef struct CGNSS CGNSS;
CGNSS* newCGNSS();

typedef struct CSensors CSensors;
CSensors* newCSensors();

void handleGNSS_c(void);
void initGNSSrx_c(void);
int addUBXpktByte_c(byte ch, tGNSSrx *pr);
int readUBXpkt_c(byte *retbuf);
int checkUBX_c(byte *buf, int cnt);
void crcUBX_c(byte *buf, int cnt, byte *pcha, byte *pchb);
void EventsCommGNSS_c(uint8_t *msgbuf, int32_t cnt);
int32_t bytesToLong_c(uint8_t *b);
int16_t bytesToShort_c(uint8_t *b);

// Custom Functions
//void linkVariables(uint8_t *rx_buff, CSensors* sens);
CSensors* copy_struct();

#ifdef __cplusplus
}
#endif

#endif /* INC_GNSS_WRAPPER_H_ */
