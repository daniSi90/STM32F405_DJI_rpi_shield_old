/*
 * basic_types.h
 *
 *  Created on: Jun 10, 2020
 *      Author: danijel
 */

#ifndef INC_BASIC_TYPES_H_
#define INC_BASIC_TYPES_H_

#include "stdint.h"

typedef float float32_t;
typedef double float64_t;
typedef unsigned char byte;

#define MAX_GNSS          256
#define UART_BUF_SIZE     256

extern uint8_t rxx;

typedef struct
{
  int state;
  int ctr;
  byte buf[MAX_GNSS];
} tGNSSrx;

typedef union
{
  uint8_t b[4];
  int32_t i;
} mlong;

typedef union
{
  int8_t b[2];
  int16_t i;
} mshort;

enum
{
  // UBX CLASS ID
   UBX_NAV     	 	   = 0x01   //Navigation Results Messages
  ,UBX_MON     	 	   = 0x0A   //Monitoring Messages
  // UBX MESSAGE ID
  ,UBX_NAV_PVT         = 0x07   //NAV-PVT: Navigation Position Velocity Time Solution
  ,UBX_NAV_RELPOSNED   = 0x3C   //NAV-RELPOSNED: Relative Positioning Information in NED frame
  ,UBX_MON_MSGPP       = 0x06   //MON-MSGPP: Message Parse and Process Status
};

typedef struct
{
	float lat;
	float lon;
	float alt;
} Pos;

typedef struct
{
	float N;
	float E;
	float D;
} PosRel;

#endif /* INC_BASIC_TYPES_H_ */
