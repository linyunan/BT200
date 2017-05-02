#include <ioCC2541.h>
#include "bcomdef.h"
#include "OSAL.h"
#include "pwm.h"
#include "bledoorlock.h"

#include "hal_mcu.h"
//此资料为amomcu参考网络上的资料

//pwm pins:
//P0_3:R  (TX)--ch1
//P0_4:G  (CTS)--ch2
//P0_5:B  (RTS)--ch3
//P0_6:w 
/*
static uint16 gRed =1;
static uint16 gGreen =1;
static uint16 gBlue =1;
static uint16 gWhite =1;
*/

//extern SYS_CONFIG  sys_config;

pwm_rgbw PWM_rgbw;
#if 1
void PWM_Init(void)
{
//  EA = 0 ;
/*  
  T1CTL = 0x00; // STOP TIMER1 

  //set TIMER1 CLOCK to 32M
  CLKCONCMD = (CLKCONCMD & 0x80) | (TICKSPD(0) | CLKSPD(0));							  
  while ( (CLKCONSTA & ~0x80) != (TICKSPD(0) | CLKSPD(0)));
 */
  //设置pwm端口为输出
  P0DIR|= BV(3)|BV(4)|BV(5);
  //设置pwm端口为外设端口，非gpio
  P0SEL|= BV(3)|BV(4)|BV(5);

  
  //由于uart等会占用我们当前使用的pwm端口，因此需要将uart等重映射到别的端口去。
  PERCFG |= 0x03;             // Move USART1&2 to alternate2 location so that T1 is visible
//  PERCFG |= 0x03;             // Move USART1&2 to alternate2 location so that T1 is visible
    
//  PERCFG = (PERCFG & ~0x40) | 0x03; // Select Timer 1 Alternative 0 location, set U1CFG and U0CFG to Alternative 1
//  Initialize Timer 1
  T1CTL = 0x0F;               // Div = 128, CLR, MODE = Suspended          
  T1CCTL1 = 0x00;             // IM = 0; CMP = Clear output on compare; Mode = Compare
  T1CCTL2 = 0x00;             // IM = 0; CMP = Clear output on compare; Mode = Compare
  T1CCTL3 = 0x00;             // IM = 0, CMP = Clear output on compare; Mode = Compare
  T1CCTL4 = 0x00;             // IM = 0, CMP = Clear output on compare; Mode = Compare
  T1CNTL = 0x00;                 // Reset timer to 0;
  T1CNTH = 0x00;
    //必须设置，否则定时器不工作
  T1CCTL0 = 0x4C;            // IM = 1, CMP = Clear output on compare; Mode = Compare

  //设置周期的tick为375, 也就是1.5ms
#if 1
  T1CC0H = 0x00;              // Ticks = 375 (1.5ms initial duty cycle)
  T1CC0L = 0xFA;              //             
  T1CC1H = 0x00;              // Ticks = 375 (1.5ms initial duty cycle)
  T1CC1L = 0x00;
  T1CC2H = 0x00;              // Ticks = 375 (1.5ms initial duty cycle)
  T1CC2L = 0x00;
  T1CC3H = 0x00;              // Ticks = 375 (1.5ms initial duty cycle)
  T1CC3L = 0x00;  
  T1CC4H = 0x00;              // Ticks = 375 (1.5ms initial duty cycle)
  T1CC4L = 0x00;
#else//以下设置会是1khz
#define VALUE_H     0x00
#define VALUE_L     0x10
  T1CC0H = VALUE_H;    
  T1CC0L = VALUE_L;    
  T1CC1H = VALUE_H;    
  T1CC1L = VALUE_L;
  T1CC2H = VALUE_H;    
  T1CC2L = VALUE_L;
  T1CC3H = VALUE_H;    
  T1CC3L = VALUE_L;  
#endif 

//  EA=1;
//  IEN1 |= 0x02;               // Enable T1 cpu interrupt
}
#else
void PWM_Init(void)
{
  P0SEL &= ~(BV(3)); // Configure Port 1 as GPIO
  P0SEL &= ~(BV(4)); // Configure Port 1 as GPIO
  P0SEL &= ~(BV(5)); // Configure Port 1 as GPIO
  
  P0DIR |= BV(3); // All port 1 pins (P1.2) as output
  P0DIR |= BV(4); // All port 1 pins (P1.3) as output
  P0DIR |= BV(5); // All port 1 pins (P1.3) as output

  P0 |= (BV(3));   // All pins on port 1 to low
  P0 |= (BV(4));   // All pins on port 1 to low
  P0 |= (BV(5));   // All pins on port 1 to low
}


#endif

#if 1
//red， green， blue 的值必须是 1~375, 其他值无效
void PWM_Pulse(uint16 red, uint16 green, uint16 blue)
{
  uint16 r,g,b;

  // stop,注意，不能加这句，加了周期偏差十几倍，具体原因未查明
//  T1CTL &= BV(0)|BV(1); 

  r=red;
  g=green;
  b=blue;

  T1CC1L = (uint8)r;
  T1CC1H = (uint8)(r >> 8);
  //避免比较值为0时，输出一直为高
  if(r!=0){
    T1CCTL1 = 0x24;
  }else{
    T1CCTL1 = 0x00;
  }
  T1CC2L = (uint8)g;
  T1CC2H = (uint8)(g >> 8);
  if(g!=0){
    T1CCTL2 = 0x24;
  }else{
    T1CCTL2 = 0x00;
  }
  T1CC3L = (uint8)b;
  T1CC3H = (uint8)(b >> 8);
  if(b!=0){
    T1CCTL3 = 0x24;
  }else{
    T1CCTL3 = 0x00;
  }

  // Reset timer
//  T1CNTL = 0x0;
//  T1CNTH = 0;

  // Start timer in modulo mode.
//  T1CTL |= 0x02;   
}

#else
void PWM_Pulse(uint16 red, uint16 green, uint16 blue)
{
  if(red)
  {
    P0 &= ~(BV(3));   // All pins on port 1 to low
  }
  else
  {
    P0 |= (BV(3));   // All pins on port 1 to low
  }
  
  if(red)
  {
    P0 &= ~(BV(4));   // All pins on port 1 to low
  }
  else
  {
    P0 |= (BV(4));   // All pins on port 1 to low
  }
  
  if(red)
  {
    P0 &= ~(BV(5));   // All pins on port 1 to low
  }
  else
  {
    P0 |= (BV(5));   // All pins on port 1 to low
  }
  
  
  
}
#endif
//red， green， blue 的值必须是 1~375, 其他值无效
void PWM_RGB(uint16 red, uint16 green, uint16 blue )
{    
  PWM_rgbw.gRed=red;
  PWM_rgbw.gGreen=green;
  PWM_rgbw.gBlue=blue;

}

//#pragma register_bank=2
#pragma vector = T1_VECTOR
__interrupt void pwmISR (void) 
{
    uint8 flags = T1STAT;
#if 1
    // T1 ch 0
    if (flags & 0x01){          
      
      // Stop Timer 1

      T1CNTL = 0x00;
//      T1CNTH = 0x00;
      
    }
#else
#endif
   
//    PWM_Pulse(PWM_rgbw.gRed,PWM_rgbw.gGreen,PWM_rgbw.gBlue,PWM_rgbw.gWhite);
    T1STAT = ~ flags;
}
