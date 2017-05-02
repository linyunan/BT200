#include "motor.h"
#include "hal_sensor.h"

void Motor_Init(void)
{
  P0SEL &= ~(BV(0)); // Configure Port 1 as GPIO
  P0SEL &= ~(BV(1)); // Configure Port 1 as GPIO
  P0DIR |= BV(0); // All port 1 pins (P1.2) as output
  P0DIR |= BV(1); // All port 1 pins (P1.3) as output
//  P2INP &= ~(BV(5));  // 设置P0口为上拉
//  P0INP &= ~(BV(0));
//  P0INP &= ~(BV(1));
#if 1
  P1 &= ~(BV(1));   // All pins on port 1 to low
  P1 &= ~(BV(2));   // All pins on port 1 to low
#else
  P0 |= (BV(1));   // All pins on port 1.3 to High
  P0 |= (BV(0));   // All pins on port 1.2 to High
#endif
}


void Motor_Foreward(void)
{
  
  
  P0 |= (BV(1));   // All pins on port 1.3 to High
//  DelayMS(30);
  P0 &= ~(BV(0));   // All pins on port 1.2 to low
  
}

void Motor_Backward(void)
{
  
  P0 |= (BV(0));   // All pins on port 1.2 to High
//  DelayMS(30);
  P0 &= ~(BV(1));   // All pins on port 1.3 to low
  
}

void Motor_Stop(void)
{
  P0 &= ~(BV(0));   // All pins on port 1 to low
  P0 &= ~(BV(1));   // All pins on port 1 to low
}


void KeepHigh_Init(void)
{
  P1SEL &= ~(BV(3));
  P1DIR |= BV(3);
  P1 |= (BV(3)); // 初始化输出0
}

void Keep_High(void)
{
  P1 |= (BV(3)); // 初始化输出1
}

void Keep_Low(void)
{
  
  P1 &= ~(BV(3)); // 初始化输出0
}
