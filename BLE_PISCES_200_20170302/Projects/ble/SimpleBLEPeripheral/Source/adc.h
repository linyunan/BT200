//******************************************************************************
// �ļ���   ��adc.h
// ����ʱ�� ��2016-04-22
// ��ǰ�汾 ��V1.0
// �������� ������ADC����
//******************************************************************************
#ifndef ADC_H
#define ADC_H

//******************************************************************************
// ͷ�ļ�
#include "hal_types.h"

//******************************************************************************
// ���ܺ���
// ADC Measure
extern void ADCInit(void);
extern uint16 BatMeasure( void );
extern uint16 BatADMeasure( void );


extern void Adc_Stop(void);
extern void Adc_Start(void);

#define RefVoltage   3.3  //�ο���ѹ
#endif // ADC_H

