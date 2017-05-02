#ifndef SIMPLEBLE_H
#define SIMPLEBLE_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * global variable to snv
 */
typedef struct 
{
  uint8 Admin_Pswd[16];		        // ����Ա����	

//  uint8 MY_KEYCODE[16];
  
  
}SYS_CONFIG;

typedef struct 
{
  uint8 Admin_mgr[14];		        // ����Ա��Ȩ��	
  uint8 Gen_mgr[14];                    // ��ͨ��Ȩ��	
//  uint8 MY_KEYCODE[16];
  
  
}SYS_MGR;

typedef struct
{
  uint8 key_2s;         // ���������ſ����������룬Ĭ��Ϊ0
  uint8 key_index;      // ��������λ������ֵ ��ʼΪ 0
  uint8 key_temp[8];    // ������ʱ�洢
  uint8 key_num;        // ����λ�� 1-9
  uint8 key_out2s;      // ����ֱ��2��ļ��
  
  uint8 key_first;  // ��һ�������ϱ� Ĭ��״̬Ϊ1���ϱ��ɹ���Ϊ0
  
  uint8 power_num;  // ��Ȩ�봫���������APP�ط� ���2��   ֵ0-2  Ĭ��0
  
//  bool Lock_OC;                         // �ж�������״̬
  uint8 pswd_num;       // ����ȶԺ����˸����
  bool pswd_flag;      // ������ȷ Ϊ TRUE  �������Ϊ FALSE
  
  uint8 openDoorTurn;
  uint16 batteryADC;
  uint8 batteryPer;
  uint8 batteryNum;
  uint8 batteryPower[3];
  
  uint8 RGB_Flag;
  uint8 Key_2;
  bool Key_Lock;
  bool key_down;
  
}GLOBAL_VAR;
  

typedef struct
{
  uint8 KEY_AES[8];
}SYS_TEST;

 
extern GLOBAL_VAR global_var;
extern SYS_TEST sys_test;
extern SYS_CONFIG sys_config;
extern SYS_MGR sys_mgr;

void Array_Stack(void);
void *osal_memcpyz( void *dst, uint8 data, unsigned int len );
uint8 osal_memcmpn( const void GENERIC *src1, const void GENERIC *src2, unsigned int len );
void key_varInit(void);
void Sys_Init(void);
uint8 osal_array(uint8 * src);


#endif /* SIMPLEBLE_H */
