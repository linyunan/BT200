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
  uint8 Admin_Pswd[16];		        // 管理员密码	

//  uint8 MY_KEYCODE[16];
  
  
}SYS_CONFIG;

typedef struct 
{
  uint8 Admin_mgr[14];		        // 管理员鉴权码	
  uint8 Gen_mgr[14];                    // 普通鉴权码	
//  uint8 MY_KEYCODE[16];
  
  
}SYS_MGR;

typedef struct
{
  uint8 key_2s;         // 长按两秒后才可以输入密码，默认为0
  uint8 key_index;      // 按键密码位数索引值 初始为 0
  uint8 key_temp[8];    // 密码临时存储
  uint8 key_num;        // 密码位数 1-9
  uint8 key_out2s;      // 密码直接2秒的间隔
  
  uint8 key_first;  // 第一次连接上报 默认状态为1，上报成功后为0
  
  uint8 power_num;  // 鉴权码传输错误请求APP重发 最多2次   值0-2  默认0
  
//  bool Lock_OC;                         // 判断锁开关状态
  uint8 pswd_num;       // 密码比对后灯闪烁次数
  bool pswd_flag;      // 密码正确 为 TRUE  密码错误为 FALSE
  
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
