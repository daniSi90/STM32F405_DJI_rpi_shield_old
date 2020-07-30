#ifndef _QUATERNION_H
#define _QUATERNION_H

#ifdef __cplusplus
extern "C" {
#endif

#include <math.h>

extern float BNO080_Roll;
extern float BNO080_Pitch;
extern float BNO080_Yaw;

void Quaternion_Update(float* q);
float invSqrt(float x);

#ifdef __cplusplus
}
#endif

#endif
