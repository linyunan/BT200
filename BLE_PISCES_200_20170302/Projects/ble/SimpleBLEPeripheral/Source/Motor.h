#ifndef __motor__h_
#define __motor__h__

#include "OnBoard.h"

extern void Motor_Init(void);
extern void Motor_Foreward(void);
extern void Motor_Backward(void);
extern void Motor_Stop(void);

void KeepHigh_Init(void);
void Keep_High(void);
void Keep_Low(void);
#endif