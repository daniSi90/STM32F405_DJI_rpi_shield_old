/*
 * gnss.hpp
 *
 *  Created on: Jun 10, 2020
 *      Author: danijel
 */

#ifndef INC_GNSS_HPP_
#define INC_GNSS_HPP_

#include "zedf9p/basic_types.h"

/*UBX sync chars*/
#define UBX_SYN_CHAR1    0xB5
#define UBX_SYN_CHAR2    0x62

/*UBX SM states*/
#define  SM_UBX_BEFORE      0
#define  SM_UBX_SYN2    	1
#define  SM_UBX_CLASS     	2
#define  SM_UBX_ID    		3
#define  SM_UBX_PAYLEN1  	4
#define  SM_UBX_PAYLEN2   	5
#define  SM_UBX_PAYLOAD     6
#define  SM_UBX_CHK1      	7
#define  SM_UBX_CHK2      	8
#define  SM_UBX_ERR         9
#define  SM_UBX_END        10



class CGNSS
{
  public:
    CGNSS(){}
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
    bool parseUBX(byte *b, int cnt);
	int msgs;
};

class CSensors
{
public:
      CSensors() {}
      CGNSS gnss;
};


void handleGNSS(void);
void initGNSSrx(void);
int addUBXpktByte(byte ch, tGNSSrx *pr);
int readUBXpkt(byte *retbuf);
int checkUBX(byte *buf, int cnt);
void crcUBX(byte *buf, int cnt, byte *pcha, byte *pchb);
void EventsCommGNSS(uint8_t *msgbuf, int32_t cnt);
int32_t bytesToLong(uint8_t *b);
int16_t bytesToShort(uint8_t *b);

extern uint8_t *rxBufferGNSSp;
extern CSensors gnss_sensor;

#endif /* INC_GNSS_HPP_ */
