//******************************************************************************
// �ļ���   ��adc.c
// ����ʱ�� ��2014-04-22
// ��ǰ�汾 ��V1.0
// �������� ������ADC����
//******************************************************************************

//******************************************************************************
// ͷ�ļ�
#include "adc.h"
#include "hal_adc.h"
#include "bledoorlock.h"
//#include "doorGATTprofile.h"
//******************************************************************************
// �궨��

//******************************************************************************
// ������   ��ADCInit
// �������� ��ADC��ʼ��
// ������� ��
// ������� ��
//******************************************************************************
void ADCInit(void)
{
  //P0_7��Ϊ ADC����
  APCFG |= BV(7);    //analog io enabled 
  P0SEL |= BV(7);   //peripheral function  
  P0DIR &= ~(BV(7));  //input  
//  P0INP |= BV(7);    //3 state  
  //P1_0��Ϊ�����Ƿ��ȡ����
  P1DIR |= BV(0); // Port 1.0 as output,

  P1SEL &= ~(BV(0)); // Configure Port 1.0 as GPIO
  
  P1 &= ~(BV(0));   //��ʼ��Ϊ����ȡ����
}

void Adc_Stop(void)
{
	P1 &= ~(BV(0));
}

void Adc_Start(void)
{
	P1 |= BV(0);
}
//******************************************************************************
// ������   ��ADC Measure
// �������� ��ADC����
// ������� ��
// ������� ��adc
//******************************************************************************
uint16 BatADMeasure( void )
{
  // Configure ADC and perform a read
 // HalAdcSetReference( HAL_ADC_REF_AVDD );HAL_ADC_REF_125V
  HalAdcSetReference( HAL_ADC_REF_125V );
  return HalAdcRead( HAL_ADC_CHANNEL_7, HAL_ADC_RESOLUTION_12 );
}


uint16 BatMeasure(void)
{
  /*
  uint16 BatAD = 0 ;6
  uint16 BatValue ;
  BatAD = BatADMeasure();
  BatValue = BatAD *RefVoltage /4096 * 600 ; // ���ڲ��� ��+10
  return (uint16)BatValue ;
  */
//  uint8 res[4] = {0};
  uint16 BatAD = 0 ;
  uint16 BatValue ;
  BatAD = BatADMeasure();

//  res[0] = BatAD/1000;
//  res[1] = BatAD%1000/100;
//  res[2] = BatAD%1000%100/10;
//  res[3] = BatAD%1000%100%10;
  
//  BatValue =(uint16) ((BatAD *1.25 /2048)* (3500 /1500)*100) ; 
  
  
 // BatValue =(uint16) (((BatAD *RefVoltage /2048)* (1499/499))*100) ; 
//  BatValue =(uint16) ((BatAD *1.25 /2048)*(4600/1000)* 1000) ; 
  BatValue =(uint16) ((BatAD *1.25 /2048)* 1000) ;
//  BatValue =(uint16) ((BatAD *3.28 /2048)*(95/20)*100) ;   //  3.3v
  //(1499/499))*100)
  return (uint16)BatValue;
  
}


