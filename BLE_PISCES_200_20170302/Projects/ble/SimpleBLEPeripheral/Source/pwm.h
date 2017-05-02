#ifndef _PWM_H_
#define _PWM_H_

typedef struct
{
    uint16 gRed ;
    uint16 gGreen ;
    uint16 gBlue ;
    uint16 gWhite ;
} pwm_rgbw;

void PWM_Init(void);
void PWM_RGB(uint16 red, uint16 green, uint16 blue);
void PWM_Pulse(uint16 red, uint16 green, uint16 blue);
#endif