/**************************************************************************************************
  Filename:       simpleBLEPeripheral.c
  Revised:        $Date: 2010-08-06 08:56:11 -0700 (Fri, 06 Aug 2010) $
  Revision:       $Revision: 23333 $

  Description:    This file contains the Simple BLE Peripheral sample application
                  for use with the CC2540 Bluetooth Low Energy Protocol Stack.

  Copyright 2010 - 2013 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED 揂S IS?WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com.
**************************************************************************************************/

/*********************************************************************
 * INCLUDES
 */

#include "bcomdef.h"
#include "OSAL.h"
#include "OSAL_PwrMgr.h"

#include "OnBoard.h"
#include "hal_adc.h"
#include "hal_led.h"
#include "hal_key.h"
#include "hal_sensor.h"
#include "hal_i2c.h"
#include "hal_lcd.h"
#include "hal_aes.h"
#include "hal_dma.h"
#include "gatt.h"

#include "hci.h"

#include "gapgattserver.h"
#include "gattservapp.h"
#include "devinfoservice.h"

#include "doorGATTprofile.h"

#if defined( CC2540_MINIDK )
//  #include "simplekeys.h"
#endif

#include "peripheral.h"

#include "gapbondmgr.h"

#include "Bledoorlock.h"

#if defined FEATURE_OAD
  #include "oad.h"
  #include "oad_target.h"
#endif


//#ifdef wdog
  #include "wdog.h"
//#endif

#include "stdio.h"
#include "osal_clock.h"
#include "OSAL_Timers.h"
#include <string.h>
#include "osal_snv.h"
#include "motor.h"
#include <math.h>
#include <stdlib.h>
#include "adc.h"
#include "hal_crc.h"
//#include "crc.h"
#include "simpleble.h"
#include "timer.h"
    
#include "osal_snv.h"
//static UTCTimeStruct UTCTimeSys; //160421
#include "pwm.h"
#include "phy.h"
uint8 key[] = { 0x01, 0x03, 0x05, 0x07, 0xFF, 0xFE, 0x77, 0x88, 0x12, 0x99, 0x32, 0x41, 0x14, 0x24, 0x25, 0x36}; 
static char MAC_KEY[] = {0x69, 0xc4, 0xe0, 0xd8, 0x6a, 0x7b};//, 0x04, 0x30};

uint8 MGR_AES[14] = { 0x50,   // 'P'
                      0x49,   // 'I'
                      0x53,   // 'S'
                      0x43,   // 'C'
                      0x45,   // 'E'
                      0x53,   // 'S'};
                      0x85,0x74,0x11,0x01,0x98,0xE1, 0xBC, 0xAA };

uint8 NOL_AES[14] = { 0x23,0x52,0x33,0x43,0xAC,0x90,0x85,0xDC,0x11,0x01,0x98,0xE1,0x00,0xAA };
//                          0x67, 0x00, 0xCE, 0xAA, 0xAB, 0xBB, 0xDF};

uint8 Lock_OC[1] = {0};
#if 0
#pragma constseg = "MY_MAC_CODE"
__no_init uint8 myCode[6] @0xFFF9;
#pragma constseg = default
//#endif

#pragma constseg = "FLASH_LOCK_BITS"
__no_init uint8 myCode[16] @0x7FFF0;

//#if 0
#pragma location = "FLASH_LOCK_BITS" 
//__no_init uint8 arry[16] @0x7FFF0;
__root const char lockbits[]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
#endif
/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

// How often to perform periodic event
#define DOOR_PERIODIC_EVT_PERIOD                   200


// What is the advertising interval when device is discoverable (units of 625us, 160=100ms)
#define DEFAULT_ADVERTISING_INTERVAL          160       // 广播间隔， 数值越大功耗越低但是广播的包的时间间隔就太长

// Limited discoverable mode advertises for 30.72s, and then stops
// General discoverable mode advertises indefinitely

#if defined ( CC2540_MINIDK )
#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_LIMITED
#else
#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_GENERAL
#endif  // defined ( CC2540_MINIDK )

// Minimum connection interval (units of 1.25ms, 80=100ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL     10//80//  连接间隔与数据发送量有关， 连接间隔越短， 单位时间内就能发送越多的数据
// Maximum connection interval (units of 1.25ms, 800=1000ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     20//800//  连接间隔与数据发送量有关， 连接间隔越短， 单位时间内就能发送越多的数据

// Slave latency to use if automatic parameter update request is enabled
#define DEFAULT_DESIRED_SLAVE_LATENCY         0

// Supervision timeout value (units of 10ms, 1000=10s) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_CONN_TIMEOUT          100//1000  -各种原因断开连接后，超时并重新广播的时间:  100 = 1s

// Whether to enable automatic parameter update request when a connection is formed
#define DEFAULT_ENABLE_UPDATE_REQUEST         TRUE//FALSE//TRUE

// Connection Pause Peripheral time value (in seconds)
#define DEFAULT_CONN_PAUSE_PERIPHERAL         6

// Company Identifier: Texas Instruments Inc. (13)
#define TI_COMPANY_ID                         0x000D

#define INVALID_CONNHANDLE                    0xFFFF

// Length of bd addr as a string
#define B_ADDR_STR_LEN                        15


/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
static uint8 bleDoorLock_TaskID;   // Task ID for internal task/event processing

static gaprole_States_t gapProfileState = GAPROLE_INIT;



/*********************************************************************
 * globle variable don't need save
 *
 */


enum POWER_ONOFF {  PowerOff =0,  PowerOn};   //添加系统状态位
bool Power_OnOff = PowerOff ;

enum GAP_CONNECT {  Gapconnecting =0,  Gapconnected};   //权限模式下的连接状态
bool Gap_Connect = Gapconnecting ;

enum OPTION_DOOR {  Door_Close =0,  Door_Open};   //开关门状态位
bool Option_Door = Door_Close;

//enum KEY_ISR {  Key_Onetouch =0,  Key_Sectouch};   //按键触发
//bool Key_Isr = Key_Onetouch;



// GAP - SCAN RSP data (max size = 31 bytes)
static uint8 scanRspData[] =
{
  // complete name
  0x0D,   // length of this data
  GAP_ADTYPE_LOCAL_NAME_COMPLETE,

  0x50,   // 'P'
  0x49,   // 'I'
  0x53,   // 'S'
  0x43,   // 'C'
  0x45,   // 'E'
  0x53,   // 'S'
  0x2D,   // '-'
  0x42,   // 'B'
  0x54,   // 'T'
  0x32,   // '2'
  0x30,   // '0'
  0x30,   // '0'
  // connection interval range
  0x05,   // length of this data
  GAP_ADTYPE_SLAVE_CONN_INTERVAL_RANGE,
  LO_UINT16( DEFAULT_DESIRED_MIN_CONN_INTERVAL ),   // 100ms
  HI_UINT16( DEFAULT_DESIRED_MIN_CONN_INTERVAL ),
  LO_UINT16( DEFAULT_DESIRED_MAX_CONN_INTERVAL ),   // 1s
  HI_UINT16( DEFAULT_DESIRED_MAX_CONN_INTERVAL ),

  // Tx power level
  0x02,   // length of this data
  GAP_ADTYPE_POWER_LEVEL,
  0       // 0dBm
};

// GAP - Advertisement data (max size = 31 bytes, though this is
// best kept short to conserve power while advertisting)
static uint8 advertData[] =
{
  // Flags; this sets the device to use limited discoverable
  // mode (advertises for 30 seconds at a time) instead of general
  // discoverable mode (advertises indefinitely)
//  0x05,   // length of this data
  0x02,
  GAP_ADTYPE_FLAGS,
  DEFAULT_DISCOVERABLE_MODE | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,
//  0x42,   // 'B'
//  0x4a,   // 'J'
//  0x51,   // 'Q'

  // service UUID, to notify central devices what services are included
  // in this peripheral
  0x03,   // length of this data
  GAP_ADTYPE_16BIT_MORE,      // some of the UUID's, but not all
  LO_UINT16( DOORPROFILE_SERV_UUID ),
  HI_UINT16( DOORPROFILE_SERV_UUID ),

};

// GAP GATT Attributes
static uint8 attDeviceName[GAP_DEVICE_NAME_LEN] = "**PISCES-BT200**";

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void bleDoorLock_ProcessOSALMsg( osal_event_hdr_t *pMsg );
static void doorStateNotificationCB( gaprole_States_t newState );
static void performPeriodicTask( void );
static void doorProfileChangeCB( uint8 paramID );
//static void doorLockRssiCB( int8 newRSSI )  ;
static uint8 appData_Crc(uint8  *src,uint8 crc, uint8 len);   

static void bleDoorLock_HandleKeys( uint8 shift, uint8 keys );

//static void Battery_Auto(void);
static uint8 SendOrGetBatvalue(uint8 value);    

//static void Bat_AdcAuto(void);
#if (defined HAL_LCD) && (HAL_LCD == TRUE)
//static char *bdAddr2Str ( uint8 *pAddr );
#endif // (defined HAL_LCD) && (HAL_LCD == TRUE)

uint8 MGR_CRC(uint8 * data)
{
  uint8 crc = 0;
  crc = appData_Crc(data, crc, 14);
  return crc;
}

uint8 NOL_CRC(uint8 * data)
{
  uint8 crc = 0;
  crc = appData_Crc(data, crc, 14);
  return crc;
}



void ReadMac(unsigned char * TempMacAddress) ;
void ReadMac(unsigned char * TempMacAddress)  // Len 一定是6
{
#if 0
  TempMacAddress[5]  =  XREG((BLE_REG_BASE_ADDR  + 22));  // BLE_ASI_OWNADDR
  TempMacAddress[4]  =  XREG((BLE_REG_BASE_ADDR  + 23));  // BLE_ASI_OWNADDR
  TempMacAddress[3]  =  XREG((BLE_REG_BASE_ADDR  + 24));  // BLE_ASI_OWNADDR
  TempMacAddress[2]  =  XREG((BLE_REG_BASE_ADDR  + 25));  // BLE_ASI_OWNADDR
  TempMacAddress[1]  =  XREG((BLE_REG_BASE_ADDR  + 26));  // BLE_ASI_OWNADDR
  TempMacAddress[0]  =  XREG((BLE_REG_BASE_ADDR  + 27));  // BLE_ASI_OWNADDR
#else
  TempMacAddress[5]=*(unsigned char *)(0x780E); // 直接指向指针内容
  TempMacAddress[4]=*(unsigned char *)(0x780F);
  TempMacAddress[3]=*(unsigned char *)(0x7810);
  TempMacAddress[2]=XREG(0x7811);                // define 函数直接读出数据
  TempMacAddress[1]=XREG(0x7812);
  TempMacAddress[0]=XREG(0x7813);
#endif
}




/*******************************************************************************
 * 函数名称 :   RandomNumberGenerator                                                                  
 * 描述     :   生产随机数     （利用单片机自带的硬件随机数生成器）                                                    
 *                                                                               
 * 输   入  :  void                                                                   
 * 输   出  :  生产的16位随机数
 * 返   回  :                                                            
 * 修改时间 : 2014?ê3??9è?                                                                    
 *******************************************************************************/
uint16  RandomNumberGenerator(void)
{
  uint16 RN_H = RNDH;
  uint16 RN_L = RNDL;
  ADCCON1 = 0x37 ;
  
  return (RN_H <<8 |RN_L);
}



/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks
static gapRolesCBs_t bleDoorLock_PeripheralCBs =
{
  doorStateNotificationCB,  // Profile State Change Callbacks
  NULL//doorLockRssiCB//NULL                            // When a valid RSSI is read from controller (not used by application)
};

// GAP Bond Manager Callbacks
static gapBondCBs_t bleDoorLock_BondMgrCBs =
{
  NULL,                     // Passcode callback (not used by application)
  NULL                      // Pairing / Bonding state Callback (not used by application)
};

// Simple GATT Profile Callbacks
static doorProfileCBs_t bleDoorLock_DoorProfileCBs =
{
  doorProfileChangeCB    // Charactersitic value change callback
};
/*********************************************************************
 * PUBLIC FUNCTIONS
 */

char* encrypt(char* source, char* pass)
{
    int source_length = strlen(source);
    int pass_length = strlen(pass);
 
    char* tmp_str = (char*)malloc((source_length + 1) * sizeof(char));
    memset(tmp_str, 0, source_length + 1);
 
    for(int i = 0; i < source_length; ++i)
    {
        tmp_str[i] = source[i]^pass[i%pass_length];
        if(tmp_str[i] == 0)              // 要考虑到XOR等于0的情况，如果等于0，就相当
        {                                // 于字符串就提前结束了， 这是不可以的。
            tmp_str[i] = source[i];      // 因此tmp_str[i]等于0的时候，保持原文不变
        }
    }
    tmp_str[source_length] = 0;
 
    return tmp_str;
}


char * Num_SN(void)
{
  unsigned char MAC_BUF[6] = {0};
  char * tmp_str;
  ReadMac(MAC_BUF);
//  MAC_BUF[6] = 0xCC;
//  MAC_BUF[7] = 0xBB;
  tmp_str = encrypt((char *)MAC_BUF, MAC_KEY);
  
  return tmp_str;
}


void InitLed(void)
{
    //P1DIR |= 0x13; // P1.0、P1.1 、P1.4定义为输出
   // LedOnOrOff(0); //使所有LED灯默认为熄灭状态  
    P1DIR |= 0x13; // P1.0定义为输出
    LED1 = 0; //LED1灭
    LED2 = 1; //LED1灭
    LED3 = 0; //LED1灭
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_Init
 *
 * @brief   Initialization function for the Simple BLE Peripheral App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notificaiton ... ).
 *
 * @param   task_id - the ID assigned by OSAL.  This ID should be
 *                    used to send messages and set timers.
 *
 * @return  none
 */

void BleDoorLock_Init( uint8 task_id )
{
  /*
  uint8 mac_temp1[6] = {0};
  uint8 mac_temp2[6] = {0};
  char * mac_temp;
  char * enmac_temp;
  */
  bleDoorLock_TaskID = task_id;
/*  
  mac_temp = Num_SN();
  
  ReadMac(mac_temp1);
  
  mac_temp = encrypt((char *)mac_temp1,MAC_KEY);
  
  osal_memcpy(mac_temp2, mac_temp, 6);
  
  enmac_temp = encrypt((char *)mac_temp,MAC_KEY);
  
  osal_memcpy(mac_temp2, enmac_temp, 6);
  
  osal_memcpy(mac_temp1, mac_temp, 6);
  
  enmac_temp = EdNum_SN((char *)mac_temp1);
  
  osal_memcpy(mac_temp2, enmac_temp, 6);
  
  */
  
//#ifdef wdog  
//看门狗初始化
  watchdog_init();
//#endif

  ADCInit();
  
  Motor_Init();
  
  PWM_Init();
  
  
  // Setup the GAP
  VOID GAP_SetParamValue( TGAP_CONN_PAUSE_PERIPHERAL, DEFAULT_CONN_PAUSE_PERIPHERAL );

//  GAP_SetParamValue(TGAP_CONN_EST_SUPERV_TIMEOUT, 100);  //central mode
  
  // Setup the GAP Peripheral Role Profile
  {
    #if defined( CC2540_MINIDK )
      // For the CC2540DK-MINI keyfob, device doesn't start advertising until button is pressed
      uint8 initial_advertising_enable = FALSE;
    #else
      // For other hardware platforms, device starts advertising upon initialization
      uint8 initial_advertising_enable = FALSE;//  TRUE   FALSE
    #endif

    // By setting this to zero, the device will go into the waiting state after
    // being discoverable for 30.72 second, and will not being advertising again
    // until the enabler is set back to TRUE
    uint16 gapRole_AdvertOffTime = 5000;

    uint8 enable_update_request = DEFAULT_ENABLE_UPDATE_REQUEST;
    uint16 desired_min_interval = DEFAULT_DESIRED_MIN_CONN_INTERVAL;
    uint16 desired_max_interval = DEFAULT_DESIRED_MAX_CONN_INTERVAL;
    uint16 desired_slave_latency = DEFAULT_DESIRED_SLAVE_LATENCY;
    uint16 desired_conn_timeout = DEFAULT_DESIRED_CONN_TIMEOUT;


	//设置RSSI的读取速率，默认是0  
//	uint16 desired_rssi_rate = 100;  
//	GAPRole_SetParameter(GAPROLE_RSSI_READ_RATE,sizeof(uint16),&desired_rssi_rate); 
//    uint8 advType = GAP_ADTYPE_ADV_IND;
//    GAPRole_SetParameter( GAPROLE_ADV_EVENT_TYPE, sizeof( uint8 ), &advType );
	
    // Set the GAP Role Parameters
    GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &initial_advertising_enable );
    GAPRole_SetParameter( GAPROLE_ADVERT_OFF_TIME, sizeof( uint16 ), &gapRole_AdvertOffTime );

//    GAPRole_SetParameter( GAPROLE_SCAN_RSP_DATA, sizeof ( scanRspData2 ), scanRspData2 );
    GAPRole_SetParameter( GAPROLE_SCAN_RSP_DATA, sizeof ( scanRspData ), scanRspData );
    GAPRole_SetParameter( GAPROLE_ADVERT_DATA, sizeof( advertData ), advertData );

    GAPRole_SetParameter( GAPROLE_PARAM_UPDATE_ENABLE, sizeof( uint8 ), &enable_update_request );
    GAPRole_SetParameter( GAPROLE_MIN_CONN_INTERVAL, sizeof( uint16 ), &desired_min_interval );
    GAPRole_SetParameter( GAPROLE_MAX_CONN_INTERVAL, sizeof( uint16 ), &desired_max_interval );
    GAPRole_SetParameter( GAPROLE_SLAVE_LATENCY, sizeof( uint16 ), &desired_slave_latency );
    GAPRole_SetParameter( GAPROLE_TIMEOUT_MULTIPLIER, sizeof( uint16 ), &desired_conn_timeout );
  }
  
//  LL_Encrypt( key, attDeviceName, Name_Adv );
  // Set the GAP Characteristics
//  GGS_SetParameter( GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, Name_Adv );
  GGS_SetParameter( GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName );

  // Set advertising interval
  {
    uint16 advInt = DEFAULT_ADVERTISING_INTERVAL;

    GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MIN, advInt );
    GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MAX, advInt );
    GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MIN, advInt );
    GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MAX, advInt );
  }

  // Setup the GAP Bond Manager
  {   

    uint32 passkey = 0; // passkey "000000"
    uint8 pairMode = GAPBOND_PAIRING_MODE_WAIT_FOR_REQ;//GAPBOND_PAIRING_MODE_INITIATE ;//GAPBOND_PAIRING_MODE_NO_PAIRING;GAPBOND_PAIRING_MODE_WAIT_FOR_REQ
    uint8 mitm = TRUE;
    uint8 ioCap = GAPBOND_IO_CAP_DISPLAY_ONLY;//;GAPBOND_IO_CAP_DISPLAY_ONLY;GAPBOND_IO_CAP_NO_INPUT_NO_OUTPUT
    uint8 bonding = TRUE;
    GAPBondMgr_SetParameter( GAPBOND_DEFAULT_PASSCODE, sizeof ( uint32 ), &passkey );
    GAPBondMgr_SetParameter( GAPBOND_PAIRING_MODE, sizeof ( uint8 ), &pairMode );
    GAPBondMgr_SetParameter( GAPBOND_MITM_PROTECTION, sizeof ( uint8 ), &mitm );
    GAPBondMgr_SetParameter( GAPBOND_IO_CAPABILITIES, sizeof ( uint8 ), &ioCap );
    GAPBondMgr_SetParameter( GAPBOND_BONDING_ENABLED, sizeof ( uint8 ), &bonding );
  }

  // Initialize GATT attributes
  GGS_AddService( GATT_ALL_SERVICES );            // GAP
  GATTServApp_AddService( GATT_ALL_SERVICES );    // GATT attributes
  DevInfo_AddService();                           // Device Information Service
//  Batt_AddService( );     // Battery Service
  DoorProfile_AddService( GATT_ALL_SERVICES );  // Simple GATT Profile
  
  // Register for all key events - This app will handle all key events
  RegisterForKeys( bleDoorLock_TaskID );   //  一定需要添加这个， 否则按键不起作用

//  InitLed();

  // Register callback with SimpleGATTprofile
  VOID DoorProfile_RegisterAppCBs( &bleDoorLock_DoorProfileCBs );
  
  // Enable clock divide on halt
  // This reduces active current while radio is active and CC254x MCU
  // is halted
//  HCI_EXT_ClkDivOnHaltCmd( HCI_EXT_ENABLE_CLK_DIVIDE_ON_HALT );

  HCI_EXT_SetTxPowerCmd(HCI_EXT_TX_POWER_0_DBM);//HCI_EXT_TX_POWER_MINUS_23_DBM,HCI_EXT_TX_POWER_0_DBM

//turn on overlapped processing
  HCI_EXT_HaltDuringRfCmd(HCI_EXT_HALT_DURING_RF_DISABLE);
  HCI_EXT_OverlappedProcessingCmd(HCI_EXT_ENABLE_OVERLAPPED_PROCESSING);
  // Set
//  HCI_EXT_SetTxPowerCmd
  // Setup a delayed profile startup
  osal_set_event( bleDoorLock_TaskID, DOOR_START_DEVICE_EVT );

  #if defined ( POWER_SAVING )
     osal_pwrmgr_device( PWRMGR_ALWAYS_ON ); // Revert to battery mode after LED off    PWRMGR_ALWAYS_ON  PWRMGR_BATTERY
  #endif

}




/********************************************************************
*   电池检测
*   时间: 2015-07-28
*   作者：林玉阑
*********************************************************************
* For a coin cell battery 3.0v = 100%.  The minimum operating
   * voltage of the CC2541 is 2.0v so 2.0v = 0%.
   *
   * To convert a voltage to an ADC value use: 12bite
   *
   *   (v/3)/1.25 * 2048 = adc
   *
   * 3.0v = 1638 ADC
   * 2.0v = 1092 ADC
   * 2.1v = 1147 ADC
   * We need to map ADC values from 409-273 to 100%-0%.
   *
   * Normalize the ADC values to zero:
   *
   * 1637 - 1092 = 545
   *
   * And convert ADC range to percentage range:
   *
   * percent/adc = 100/545 = 20/109
   *
   * Resulting in the final equation, with round:
   *
   * percent = ((adc - 1092) * 20) / 109
********************************************************************/
// value
// 0: get
// 1: send 
static uint8 SendOrGetBatvalue(uint8 value)    
{
  uint16 BatValue0,BatValue1,BatValue2 ;
  uint16 temp;

//  uint8 * responseData = osal_mem_alloc(16);
  uint8 responseData[16] = {0};

  BatValue0    = BatMeasure();
  BatValue1    = BatMeasure();
  BatValue2    = BatMeasure(); 
  global_var.batteryADC=( BatValue0 < BatValue1 ?
  ((BatValue1 < BatValue2) ? BatValue1 : (BatValue0 < BatValue2 ? BatValue2 : BatValue0)) :
  ((BatValue1 > BatValue2) ? BatValue1 : (BatValue0 > BatValue2 ? BatValue2 : BatValue0)));
  
#if 1      

#else
//      responseData[3] = global.batteryADC;         
      responseData[3] = (global_var.batteryADC)/1000;
      responseData[4] = (global_var.batteryADC)%1000/100;
      responseData[5] = (global_var.batteryADC)%1000%100/10;
      responseData[6] = (global_var.batteryADC)%10;
//#else
//      responseData[3] = res[0];
//      responseData[4] = res[1]; 
//      responseData[5] = res[2];
#endif
  
  if( global_var.batteryADC > 265 )
  {
    temp =(uint16) global_var.batteryADC - 265; 
    global_var.batteryPer = (uint8)( temp*100/131 ) ;
    
  }
  else
  {
    global_var.batteryPer = 0;
  }
  

  if( (gapProfileState == GAPROLE_CONNECTED) && (value == 1)) 
//  if( value == 1 )
  {
    responseData[0] = 0x31;
    
    if((global_var.batteryPer /10) >= 10)
    {
      responseData[1] = 0x01;
      responseData[2] = 0x00;
    }
    else
    {
      responseData[1] = 0x00;
      responseData[2] = ((uint8)global_var.batteryPer /10 *16) + ((uint8)global_var.batteryPer %10);
    }
#if 1
      DoorProfile_SetParameter( DOORPROFILE_CHAR2, 16, responseData );
#else
      DoorProfile_SetParameter( DOORPROFILE_CHAR2, 7, responseData );
#endif
  }
//  osal_mem_free(responseData);
  return global_var.batteryPer;
}

/*********************************************************************
 * @fn      BleDoorLock_ProcessEvent
 *
 * @brief   Simple BLE Peripheral Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  events not processed
 */
uint16 BleDoorLock_ProcessEvent( uint8 task_id, uint16 events )
{

  VOID task_id; // OSAL required parameter that isn't used in this function

  if ( events & SYS_EVENT_MSG )
  {
    uint8 *pMsg;

    if ( (pMsg = osal_msg_receive( bleDoorLock_TaskID )) != NULL )
    {
      bleDoorLock_ProcessOSALMsg( (osal_event_hdr_t *)pMsg );

      // Release the OSAL message
      VOID osal_msg_deallocate( pMsg );
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }

  if ( events & DOOR_START_DEVICE_EVT )
  {
    // Start the Device
    VOID GAPRole_StartDevice( &bleDoorLock_PeripheralCBs );

    // Start Bond Manager
    VOID GAPBondMgr_Register( &bleDoorLock_BondMgrCBs );

    // Set timer for first periodic event
    PWM_Pulse(0, 0, 100);  //  变绿
    osal_start_timerEx(bleDoorLock_TaskID, DOOR_POWERONOFF_EVT,300);
    
//    osal_set_event(bleDoorLock_TaskID, DOOR_PERIODIC_EVT);
    return ( events ^ DOOR_START_DEVICE_EVT );
  }

  /*
  if ( events & DOOR_KEY_LONG_EVT )
  {
    if( (KEY_DOWN == 0 && global_var.key_2s == 0) || (global_var.key_2s == 1 && global_var.key_num > 0))
    {
      global_var.RGB_Flag += 20;
      if(global_var.RGB_Flag <= 250)
      {
        //PWM_Pulse( 250 - global_var.RGB_Flag, global_var.RGB_Flag, 0);//
        
        PWM_Pulse( 250 - global_var.RGB_Flag, 0, global_var.RGB_Flag);//
        
        osal_start_timerEx( bleDoorLock_TaskID, DOOR_KEY_LONG_EVT, 100 );
      }
      else
      {
        global_var.RGB_Flag = 0;
//        PWM_Pulse(0, 100, 0);
        osal_stop_timerEx( bleDoorLock_TaskID, DOOR_KEY_LONG_EVT );
      }
      
//      osal_start_timerEx( bleDoorLock_TaskID, DOOR_PERIODIC_EVT, 1000 );
    }
    
    return (events ^ DOOR_KEY_LONG_EVT);
  }

//作为系统实时性任务检测
  if ( events & DOOR_PERIODIC_EVT )
  {

  	
  	//osal_stop_timerEx(bleDoorLock_TaskID, DOOR_PERIODIC_EVT);


	if(global_var.key_2s == 1 && global_var.key_num > 0)
	{
		global_var.RGB_Flag += 15;
		if(global_var.RGB_Flag <= 250)
		{
			//PWM_Pulse(  global_var.RGB_Flag, 0,250 - global_var.RGB_Flag);
			osal_start_timerEx( bleDoorLock_TaskID, DOOR_PERIODIC_EVT, 100 );
		}
		else
		{
			global_var.RGB_Flag = 0;
			//        global_var.key_num = 0;
			//        PWM_Pulse(0, 100, 100);
			osal_stop_timerEx( bleDoorLock_TaskID, DOOR_PERIODIC_EVT );
		}
	}
	  
    
    return (events ^ DOOR_PERIODIC_EVT);
  }
*/
//  开机 待机任务
  if( events & DOOR_POWERONOFF_EVT)
  {
//    uint8 scanRspData2[23] = {0};

      Power_OnOff = !Power_OnOff; 
      
      
      if( Power_OnOff == PowerOn )					// 唤醒开机
      {
//        uint8 initial_advertising_enable = TRUE;
        
        #if defined ( POWER_SAVING )
        osal_pwrmgr_device( PWRMGR_ALWAYS_ON ); // 唤醒
        #endif

//        watchdog_init();
        
        PWM_Pulse(0, 0, 0);   // 绿灯
        
        osal_snv_read(0x80, sizeof(SYS_CONFIG), &sys_config );

        osal_start_timerEx(bleDoorLock_TaskID, DOOR_POWERONOFF_EVT, 8000);		//  10秒无操作进入睡眠
        
        osal_set_event(bleDoorLock_TaskID, DOOR_BATTERY_EVT);
        
        uint8 Lock_OC[1] = {0x00};
        osal_snv_read(0xF0, 1, Lock_OC); 
        if(Lock_OC[0] == 0x01)  // 关锁情况下按键才有效
        {
          PWM_Pulse(50, 0, 0);
        }
//        osal_set_event(bleDoorLock_TaskID, DOOR_PERIODIC_EVT);
        // 断开所有连接
//        GAPRole_TerminateConnection();

         // 设置蓝牙广播状态
//        GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &initial_advertising_enable );
        
//        GAPRole_SetParameter( GAPROLE_SCAN_RSP_DATA, sizeof ( scanRspData ), scanRspData );
        
        
      }
#if 1
      else											// 时间到关机
      {
        
        uint8 initial_advertising_enable = FALSE;
        
        // 断开所有连接
        GAPRole_TerminateConnection();

         // 设置蓝牙广播状态
        GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &initial_advertising_enable );  

//        GAP_UpdateAdvertisingData( bleDoorLock_TaskID, FALSE, sizeof(attDeviceName), Name_Adv );
//        key_varInit();
        
       
        
        PWM_Pulse(0, 0, 0);   // 关灯
        Motor_Stop();
        
        
        osal_stop_timerEx( bleDoorLock_TaskID, DOOR_POWERONOFF_EVT);
        osal_stop_timerEx( bleDoorLock_TaskID, DOOR_SECKEY_EVT);
        osal_stop_timerEx( bleDoorLock_TaskID, DOOR_2S_EVT);
        osal_stop_timerEx( bleDoorLock_TaskID, DOOR_OPENDOOR_EVT);
        osal_stop_timerEx( bleDoorLock_TaskID, DOOR_PERIODIC_EVT);
        osal_stop_timerEx( bleDoorLock_TaskID, DOOR_OUTTIME_EVT);
        osal_stop_timerEx( bleDoorLock_TaskID, DOOR_PSWD_OPEN_EVT);
        osal_stop_timerEx( bleDoorLock_TaskID, DOOR_BATTERY_EVT);
        

        #if defined ( POWER_SAVING )
        osal_pwrmgr_device( PWRMGR_BATTERY ); // 进入低功耗
        #endif
        
        Keep_Low();
      }
#endif
    return ( events ^ DOOR_POWERONOFF_EVT );
  }

  
  //  2秒处理函数
    if ( events & DOOR_2S_EVT)
    {
      osal_stop_timerEx( bleDoorLock_TaskID, DOOR_2S_EVT );
      osal_stop_timerEx( bleDoorLock_TaskID, DOOR_OUTTIME_EVT );
      osal_stop_timerEx( bleDoorLock_TaskID, DOOR_PERIODIC_EVT );

/*      
      PWM_Pulse(0, 100, 0);  //  变绿
      osal_start_timerEx( bleDoorLock_TaskID, DOOR_LED_ONETIME_EVT, 250 );
//      osal_set_event(bleDoorLock_TaskID, DOOR_PERIODIC_EVT);
     
      if(global_var.key_2s >0)  // 第二次长按2秒重新输入密码
      {
        global_var.key_index = 0;
        global_var.key_out2s = 0;
        global_var.key_num = 0;
        osal_memcmpn(global_var.key_temp, 0, 8);
      }
*/
      global_var.key_out2s ++;
      global_var.key_temp[global_var.key_index] = global_var.key_num + 0x30;
      global_var.key_index ++;
      global_var.key_2s = 1;
      global_var.key_num = 0;
        
      PWM_Pulse(0, 100, 0);   // 蓝色亮
      osal_start_timerEx( bleDoorLock_TaskID, DOOR_LED_ONETIME_EVT, 200 );

      //PWM_Pulse(0, 0, 0);   // 灭灯
      
      if(global_var.key_out2s >= 6)   // 密码比对
      {
        global_var.key_index = 0;
        global_var.key_out2s = 0;
        osal_set_event( bleDoorLock_TaskID, DOOR_PSWD_OPEN_EVT );
      }
      return (events ^ DOOR_2S_EVT);
    }
  
  // 2S后处理函数 若key_index >= 6 就去密码匹配
    if ( events & DOOR_OUTTIME_EVT)
    {
      if(KEY_DOWN != 0)
      {
        //PWM_Pulse(0, 100, 100);   // 变黄
        //PWM_Pulse(0, 0, 100);   // 变?
        //DelayMS(100);
        //osal_start_timerEx( bleDoorLock_TaskID, DOOR_PERIODIC_EVT, 250 );
	 //PWM_Pulse(0, 0, 0);   // 变?
  //      Buzzer_Open();
        osal_stop_timerEx( bleDoorLock_TaskID, DOOR_PERIODIC_EVT);
        osal_stop_timerEx( bleDoorLock_TaskID, DOOR_OUTTIME_EVT );
        
        global_var.key_out2s ++;
        global_var.key_temp[global_var.key_index] = global_var.key_num + 0x30;
        global_var.key_index ++;

        global_var.key_num = 0;
        
        PWM_Pulse(0, 100, 0);   // 蓝色亮
        osal_start_timerEx( bleDoorLock_TaskID, DOOR_LED_ONETIME_EVT, 200 );

        //PWM_Pulse(0, 0, 0);   // 灭灯
        
        if(global_var.key_out2s >= 6)   // 密码比对
        {
          global_var.key_index = 0;
          global_var.key_out2s = 0;
          osal_set_event( bleDoorLock_TaskID, DOOR_PSWD_OPEN_EVT );
        }
      }
      else
      {
          osal_stop_timerEx( bleDoorLock_TaskID, DOOR_OUTTIME_EVT );   
      }
      return (events ^ DOOR_OUTTIME_EVT);
    }

  

// 按键密码开门
  if ( events & DOOR_PSWD_OPEN_EVT)
  {                            
    if(osal_memcmpn(sys_config.Admin_Pswd, global_var.key_temp, 6))
    {
      global_var.pswd_flag = TRUE;
      osal_set_event( bleDoorLock_TaskID, DOOR_OPENDOOR_EVT);  
    }
    else
    {
      global_var.pswd_flag = FALSE;
    }
    global_var.key_index = 0;
    global_var.key_num = 0;
    osal_memcpyz(global_var.key_temp, 0, 8);
    osal_set_event( bleDoorLock_TaskID, DOOR_LED_ONOFF_EVT);
    
    return (events ^ DOOR_PSWD_OPEN_EVT);
  }
  
  
  // 灯闪烁
  if ( events & DOOR_LED_ONOFF_EVT)
  {  
    global_var.power_num ++;
    if(global_var.power_num < 7)
    {
      if( global_var.pswd_flag )
      {
        if(global_var.power_num % 2 )  // 开锁
        {
          PWM_Pulse(0, 50, 0);   // 绿灯
        }
        else
        {
          PWM_Pulse(0, 0, 0);   //  灭
        }
        
      }
      else
      {
        if(global_var.power_num % 2 )  // 密码错误
        {
          PWM_Pulse(100, 0, 100);   // 变黄
        }
        else
        {
          PWM_Pulse(0, 0, 0);   // 关
        }
      }
      osal_start_timerEx( bleDoorLock_TaskID, DOOR_LED_ONOFF_EVT, 300);
    }
    else
    {
      if( global_var.pswd_flag )
      {
        PWM_Pulse(50, 0, 0);   //  开锁后灯常亮
      }
      global_var.power_num = 0;
      osal_stop_timerEx( bleDoorLock_TaskID, DOOR_LED_ONOFF_EVT);
    }
    return (events ^ DOOR_LED_ONOFF_EVT);
  }

  // LED 定时灭任务
  if(events & DOOR_LED_ONETIME_EVT )
  {
  	osal_stop_timerEx( bleDoorLock_TaskID, DOOR_LED_ONETIME_EVT);
	PWM_Pulse(0, 0, 0);   // 关
  	return (events ^ DOOR_LED_ONETIME_EVT);
  }
  
  

// 开关门任务 
  if ( events & DOOR_OPENDOOR_EVT)
  {
    uint8 Lock_OC[1] = {0x01};
    osal_snv_write(0xF0, 1, Lock_OC);
//    global_var.Key_Lock = FALSE;     // 开锁
#if 0
    osal_stop_timerEx(bleDoorLock_TaskID, DOOR_OPENDOOR_EVT );
    osal_stop_timerEx( bleDoorLock_TaskID, DOOR_POWERONOFF_EVT);       //停止进入低功耗，直到锁梁按下
    Motor_Foreward();
    DelayMS(80);
    Motor_Stop();
    
#else
    if(global_var.openDoorTurn == 0)
    {
      global_var.openDoorTurn ++;
      Motor_Foreward();
      osal_start_timerEx(bleDoorLock_TaskID, DOOR_OPENDOOR_EVT, 160);
    }
    
    else if(global_var.openDoorTurn == 1)
    {

      global_var.openDoorTurn = 0;
      
      Motor_Stop();
      osal_stop_timerEx(bleDoorLock_TaskID, DOOR_OPENDOOR_EVT );
      osal_stop_timerEx( bleDoorLock_TaskID, DOOR_POWERONOFF_EVT);       //停止进入低功耗，直到锁梁按下
    }
#endif
    return (events ^ DOOR_OPENDOOR_EVT);
  }
  
  // 开关门任务 
  if ( events & DOOR_CLOSEDOOR_EVT)
  {
#if 0
    osal_stop_timerEx(bleDoorLock_TaskID, DOOR_OPENDOOR_EVT );
    osal_set_event( bleDoorLock_TaskID, DOOR_POWERONOFF_EVT);       //停止进入低功耗，直到锁梁按下
    Motor_Backward();
    DelayMS(80);
    Motor_Stop();
#else
    if(global_var.openDoorTurn == 0)
    {
      global_var.openDoorTurn ++;
      Motor_Backward();
      osal_start_timerEx(bleDoorLock_TaskID, DOOR_CLOSEDOOR_EVT, 140);
    }
    
    else if(global_var.openDoorTurn == 1)
    {

      global_var.openDoorTurn = 0;
      
      Motor_Stop();
      osal_stop_timerEx(bleDoorLock_TaskID, DOOR_CLOSEDOOR_EVT );
      osal_start_timerEx( bleDoorLock_TaskID, DOOR_POWERONOFF_EVT , 50);  // 关锁后关灯切断电源
    }
#endif
    return (events ^ DOOR_CLOSEDOOR_EVT);
  }
  
  
// 秘钥验证任务
  if ( events & DOOR_SECKEY_EVT )
  {
    // Terminate all connection
    VOID GAPRole_TerminateConnection();

    osal_stop_timerEx( bleDoorLock_TaskID, DOOR_SECKEY_EVT ); 
    
    return ( events ^ DOOR_SECKEY_EVT );
  }
  
  
  if( events & DOOR_BATTERY_EVT )
  {
    uint8 temp = 0;

    global_var.batteryNum ++ ;
    if( global_var.batteryNum > 3 && global_var.batteryNum < 13)
    {
      temp = osal_array(global_var.batteryPower);
      Adc_Stop();
      if(temp <= 20)
      {
        if( global_var.batteryNum %2 == 0)  // 低电量
        {
          PWM_Pulse(100, 0, 0);
        }
        else
        {
          PWM_Pulse(0, 0, 0);
        }
        osal_start_timerEx(bleDoorLock_TaskID, DOOR_BATTERY_EVT,50);
      }
      else
      {
#if 1        
        uint8 initial_advertising_enable = TRUE;
         
        // 断开所有连接
        GAPRole_TerminateConnection();
         // 设置蓝牙广播状态
        GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &initial_advertising_enable ); 
          
        GAPRole_SetParameter( GAPROLE_SCAN_RSP_DATA, sizeof ( scanRspData ), scanRspData );
//        GAP_UpdateAdvertisingData( bleAutoLock_TaskID, FALSE, sizeof(attDeviceName), attDeviceName );
#endif        
        global_var.batteryNum = 0;
        
//        PWM_Pulse(0, 100, 0);
        
        osal_memcpyz(global_var.batteryPower , 0, 3);
        osal_stop_timerEx(bleDoorLock_TaskID, DOOR_BATTERY_EVT);
        return ( events ^ DOOR_BATTERY_EVT );
      }
      
    }
    else if( global_var.batteryNum <= 3 && global_var.batteryNum > 0)
    {
      Adc_Start();
      SendOrGetBatvalue(0);
      {
        global_var.batteryPower[global_var.batteryNum-1] = global_var.batteryPer;
      }
      
      osal_start_timerEx(bleDoorLock_TaskID, DOOR_BATTERY_EVT,20);
      
    }
    else if(global_var.batteryNum >= 13)
    {
#if 1
      uint8 initial_advertising_enable = TRUE;
       // 断开所有连接
      GAPRole_TerminateConnection();
       // 设置蓝牙广播状态
      GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &initial_advertising_enable ); 

      GAPRole_SetParameter( GAPROLE_SCAN_RSP_DATA, sizeof ( scanRspData ), scanRspData );
//      GAP_UpdateAdvertisingData( bleAutoLock_TaskID, FALSE, sizeof(attDeviceName), attDeviceName );
#endif     
      global_var.batteryNum = 0;
      
//      PWM_Pulse(0, 100, 0);
      
      osal_memcpyz(global_var.batteryPower , 0, 3);
      osal_stop_timerEx(bleDoorLock_TaskID, DOOR_BATTERY_EVT);

      return ( events ^ DOOR_BATTERY_EVT );
    }
//    return ( events ^ DOOR_BATTERY_EVT );
  }
  
  
  return 0;
}

/*********************************************************************
 * @fn      simpleBLEPeripheral_ProcessOSALMsg
 *
 * @brief   Process an incoming task message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */

static void bleDoorLock_ProcessOSALMsg( osal_event_hdr_t *pMsg )
{
  switch ( pMsg->event )
  {
 // #if defined( CC2540_MINIDK )
    case KEY_CHANGE:
      bleDoorLock_HandleKeys( ((keyChange_t *)pMsg)->state, ((keyChange_t *)pMsg)->keys );
      break;
 // #endif // #if defined( CC2540_MINIDK )

  default:
    // do nothing
    break;
  }
}



/*********************************************************************
 * @fn      simpleBLEPeripheral_HandleKeys
 *
 * @brief   Handles all key events for this device.
 *
 * @param   shift - true if in shift/alt.
 * @param   keys - bit field for key events. Valid entries:
 *                 HAL_KEY_SW_2
 *                 HAL_KEY_SW_1
 *
 * @return  none
 */
static void bleDoorLock_HandleKeys( uint8 shift, uint8 keys )
{
  VOID shift;
 
  uint8 Lock_OC[1] = {0x00};
  osal_snv_read(0xF0, 1, Lock_OC);
  


  if(Power_OnOff == PowerOn )
  {
    uint8 Lock_OC[1] = {0x00};
    osal_snv_read(0xF0, 1, Lock_OC); 
    if(Lock_OC[0] == 0x00)  // 关锁情况下按键才有效
    {
      if( keys == HAL_KEY_SW_1)             // 按键功能
      {
        global_var.key_down = TRUE;
        // 2S 任务
        PWM_Pulse(0, 0, 50);    // 蓝色
        //osal_start_timerEx( bleDoorLock_TaskID, DOOR_KEY_LONG_EVT, 300 );  
        osal_stop_timerEx( bleDoorLock_TaskID, DOOR_OUTTIME_EVT );  // 
        osal_start_timerEx( bleDoorLock_TaskID, DOOR_2S_EVT, 1500 );
        
      }
      else if(global_var.key_down == TRUE)
      {
        PWM_Pulse(0, 0, 0);  // 
        global_var.key_down = FALSE;
        osal_stop_timerEx( bleDoorLock_TaskID, DOOR_2S_EVT );
  //      osal_stop_timerEx( bleDoorLock_TaskID, DOOR_KEY_LONG_EVT );
        if(global_var.key_2s == 0)
        {
          global_var.key_num++;
          if(global_var.key_num > 9)
          {
            global_var.key_num = 9;
          }
          osal_start_timerEx( bleDoorLock_TaskID, DOOR_OUTTIME_EVT, 1500 );  // 间隔 2 秒
        }

        else
        {
          osal_stop_timerEx( bleDoorLock_TaskID, DOOR_OUTTIME_EVT );  // 间隔 2 秒
          global_var.key_2s = 0;
        }
       }        
      
       osal_start_timerEx( bleDoorLock_TaskID, DOOR_POWERONOFF_EVT, 8000 );  

    }
  }
  if( keys == HAL_KEY_SW_2 )
  {
    if(Lock_OC[0] != 0)      // 非零锁开的状态   0 锁关状态
    {
      Lock_OC[0] = 0x00;
      osal_snv_write(0xF0, 1, Lock_OC); 

//      PWM_Pulse(0, 0, 0);   // 关
      osal_set_event( bleDoorLock_TaskID, DOOR_CLOSEDOOR_EVT);     //  锁住
    }// 锁处于关闭状态
    else
    {
      Power_OnOff = PowerOn;
      osal_set_event( bleDoorLock_TaskID, DOOR_POWERONOFF_EVT );  
    }

//    return;
  }
  
  /*
  else if(Power_OnOff == PowerOff && keys )
  {

    PWM_Pulse(0, 0, 50);   // 绿灯
    osal_start_timerEx( bleDoorLock_TaskID, DOOR_POWERONOFF_EVT , 300);
    
    
  }
*/
}


/*********************************************************************
 * @fn      doorLockRssiCB
 *
 * @brief   show Rssi value.
 *
 * @param   newRSSI - new Rssi
 *
 * @return  none
 */
#if 0
static void doorLockRssiCB( int8 newRSSI )  
{ 
/* 
  int8 iRssi;
  char  s[10];
  float power;
  float RSI;
  
  
  
  iRssi = abs(newRSSI); 
  
  power = (iRssi-59)/(10*2.0); 
  
  RSI = pow(10, power); 
  
  sprintf(s, "%f", RSI);
  
  HalLcdWriteString(s, HAL_LCD_LINE_6);
  */
//    HalLcdWriteStringValue( "RSSI -dB:", (uint8) (-newRSSI), 10, HAL_LCD_LINE_7 );  
    
}
#endif
/*********************************************************************
 * @fn      peripheralStateNotificationCB
 *
 * @brief   Notification from the profile of a state change.
 *
 * @param   newState - new state
 *
 * @return  none
 */
static void doorStateNotificationCB( gaprole_States_t newState )
{
#ifdef PLUS_BROADCASTER
//  static uint8 first_conn_flag = 0;
#endif // PLUS_BROADCASTER
//  static uint8 first_conn_flag = 0;
  
  switch ( newState )
  {
    case GAPROLE_STARTED:
      {
        uint8 ownAddress[B_ADDR_LEN];
        uint8 systemId[DEVINFO_SYSTEM_ID_LEN];

        GAPRole_GetParameter(GAPROLE_BD_ADDR, ownAddress);

        // use 6 bytes of device address for 8 bytes of system ID value
        systemId[0] = ownAddress[0];
        systemId[1] = ownAddress[1];
        systemId[2] = ownAddress[2];

        // set middle bytes to zero
        systemId[4] = 0x00;
        systemId[3] = 0x00;

        // shift three bytes up
        systemId[7] = ownAddress[5];
        systemId[6] = ownAddress[4];
        systemId[5] = ownAddress[3];

        DevInfo_SetParameter(DEVINFO_SYSTEM_ID, DEVINFO_SYSTEM_ID_LEN, systemId);

        #if (defined HAL_LCD) && (HAL_LCD == TRUE)
          // Display device address
          HalLcdWriteString( bdAddr2Str( ownAddress ),  HAL_LCD_LINE_2 );
          HalLcdWriteString( "Initialized",  HAL_LCD_LINE_3 );
        #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)
      }
      break;

    case GAPROLE_ADVERTISING:
      {
        uint8 initial_advertising_enable = FALSE;
        #if (defined HAL_LCD) && (HAL_LCD == TRUE)
          HalLcdWriteString( "Advertising",  HAL_LCD_LINE_3 );
        #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)
        if (Power_OnOff == PowerOff )
        {
          // 断开所有连接
          GAPRole_TerminateConnection();

          // 设置蓝牙广播状态
          GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &initial_advertising_enable ); 
        }
      }
      break;

    case GAPROLE_CONNECTED:
      {        
        #if (defined HAL_LCD) && (HAL_LCD == TRUE)
          HalLcdWriteString( "Connected",  HAL_LCD_LINE_3 );
        #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)
      }
      osal_start_timerEx( bleDoorLock_TaskID, DOOR_SECKEY_EVT, 3000);
      break;
/*
    case GAPROLE_CONNECTED_ADV:
      {
        #if (defined HAL_LCD) && (HAL_LCD == TRUE)
          HalLcdWriteString( "Connected Advertising",  HAL_LCD_LINE_3 );
        #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)
      }
      break; 
 */     
    case GAPROLE_WAITING:
      {
        #if (defined HAL_LCD) && (HAL_LCD == TRUE)
          HalLcdWriteString( "Disconnected",  HAL_LCD_LINE_3 );
        #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)
      }
      break;

    case GAPROLE_WAITING_AFTER_TIMEOUT:
      {
        #if (defined HAL_LCD) && (HAL_LCD == TRUE)
          HalLcdWriteString( "Timed Out",  HAL_LCD_LINE_3 );
        #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)

      }
      break;

    case GAPROLE_ERROR:
      {
        #if (defined HAL_LCD) && (HAL_LCD == TRUE)
          HalLcdWriteString( "Error",  HAL_LCD_LINE_3 );
        #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)
      }
      break;

    default:
      {
        #if (defined HAL_LCD) && (HAL_LCD == TRUE)
          HalLcdWriteString( "",  HAL_LCD_LINE_3 );
        #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)
      }
      break;

  }

  gapProfileState = newState;

#if !defined( CC2540_MINIDK )
  VOID gapProfileState;     // added to prevent compiler warning with
                            // "CC2540 Slave" configurations
#endif


}

/*********************************************************************
 * @fn      performPeriodicTask
 *
 * @brief   Perform a periodic application task. This function gets
 *          called every five seconds as a result of the SBP_PERIODIC_EVT
 *          OSAL event. In this example, the value of the third
 *          characteristic in the SimpleGATTProfile service is retrieved
 *          from the profile, and then copied into the value of the
 *          the fourth characteristic.
 *
 * @param   none
 *
 * @return  none
 */
static void performPeriodicTask( void )
{

//  LED1 = ~LED1;
//  if(global_var.key_2s == 0 && KEY_DOWN == 0)
    {
      global_var.RGB_Flag += 25;
      if(global_var.RGB_Flag <= 100)
      {
        PWM_Pulse( 100 - global_var.RGB_Flag, 0, 0);//global_var.RGB_Flag
//        osal_start_timerEx( bleDoorLock_TaskID, DOOR_PERIODIC_EVT, 500 );
      }
      else
      {
        global_var.RGB_Flag = 0;
//        global_var.RGB_Flag = 0;
//        PWM_Pulse(0, 100, 0);
//        osal_stop_timerEx( bleDoorLock_TaskID, DOOR_PERIODIC_EVT );
      }
      
//      osal_start_timerEx( bleDoorLock_TaskID, DOOR_PERIODIC_EVT, 1000 );
    }

}

/*********************************************************************
 * @fn      doorProfileChangeCB
 *
 * @brief   Callback from SimpleBLEProfile indicating a value change
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  none
 */
static void doorProfileChangeCB( uint8 paramID )
{
  static uint8 responseData[DOORPROFILE_CHAR1_LEN]={0};

  static uint8 responseData2[DOORPROFILE_CHAR1_LEN]={0};
//  uint8 initial_advertising_enable = FALSE;

  static uint8 Updoor_open[16] = {0x02, 0x4B, 0x4F};
  if(  Power_OnOff == PowerOn  )
  {
    switch( paramID )
    {
      case DOORPROFILE_CHAR1:
      {
        DoorProfile_GetParameter( DOORPROFILE_CHAR1, responseData2 );
        LL_EXT_Decrypt(key,responseData2,responseData);
        osal_snv_read(0x80, sizeof(SYS_CONFIG), &sys_config);
        switch(responseData[0])
        {
          case 0x10:  // 第一次连接管理员上报密码后，APP更新成功下发
          {
//            uint8 key_upcrc = 0;
            uint8 key_downture[2] = {0x4F, 0x4B};
            uint8 key_downfalse[2] = {0x4B, 0x4A};
//            key_upcrc = app_CrcData(sys_config.Admin_Pswd, key_upcrc, 6);
            // 上报数据正确
            if( osal_memcmp(key_downture, &responseData[2], 2))  
            {
              global_var.key_first = 0;
            }
            // 上报数据出错
            else if( osal_memcmp(key_downfalse, &responseData[2], 2))
            {
              global_var.key_first = 1;
            }
          }
          break;
          
          case 0x20:  // 离线更新密码下发
          {
            uint8 key_crc = 0;
            uint8 key_ture[16] = {0x04, 0x4B, 0x4B,0x4F, 0x4F };
            uint8 key_false[16] = {0x04, 0x4A, 0x4A,0x4C, 0x4C };
            key_crc = appData_Crc(&responseData[1], key_crc, 6);
            if(key_crc == responseData[15])
            {
              osal_memcpy(sys_config.Admin_Pswd, &responseData[1], 6);
              osal_snv_write(0x80,sizeof(SYS_CONFIG), &sys_config);
              // 更新密码成功上报消息
              DoorProfile_SetParameter(DOORPROFILE_CHAR2, 16, key_ture);
            }
            else
            {
              // 更新密码失败上报消息
              DoorProfile_SetParameter(DOORPROFILE_CHAR2, 16, key_false);
            }
          }
          break;
          
          case 0x30:  // 管理员鉴权
          {
            
            uint8 Lock_OC[1] = {0x00};
            osal_snv_read(0xF0, 1, Lock_OC);
            if( Lock_OC[0] == 0 )      //只有当锁是关时候才有必要开启
            {
              uint8 Mgr_Crc = 0;
              uint8 Aes_Updata[16] = {0x04};
              uint8 temp[14] = {0};
              osal_snv_read(0x95, sizeof(SYS_MGR), &sys_mgr);
              if( osal_memcmp(sys_mgr.Admin_mgr, temp, 14) )
              {
                uint8 Key_Crc = 0;
                uint8 Upkey_data[16] = {0x00};
                
                
                osal_stop_timerEx( bleDoorLock_TaskID, DOOR_SECKEY_EVT);
                osal_memcpy(sys_mgr.Admin_mgr, &responseData[1], 14); //第一次拷贝鉴权码
                osal_snv_write(0x95, sizeof(SYS_MGR), &sys_mgr);
                
                Upkey_data[0] = 0x01;
                osal_memcpy(&Upkey_data[1], sys_config.Admin_Pswd, 6);
                Key_Crc = appData_Crc(sys_config.Admin_Pswd, Key_Crc, 6);
                Upkey_data[15] = Key_Crc;
                
                DoorProfile_SetParameter(DOORPROFILE_CHAR2, 16, Upkey_data);      // 第一次上传密码
                // 鉴权通过开锁
                global_var.pswd_flag = TRUE;
                
                DoorProfile_SetParameter(DOORPROFILE_CHAR2, 16, Updoor_open);      // 锁开了
                // 断开所有连接
//                GAPRole_TerminateConnection();

                 // 设置蓝牙广播状态
//                GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &initial_advertising_enable ); 
                
                osal_start_timerEx( bleDoorLock_TaskID, DOOR_LED_ONOFF_EVT, 300 );
                osal_start_timerEx( bleDoorLock_TaskID, DOOR_OPENDOOR_EVT, 300 ); 
                
                
              }
              else
              {
                Mgr_Crc = MGR_CRC(sys_mgr.Admin_mgr);
                if( responseData[15] == Mgr_Crc )
                {
                  if(osal_memcmpn(sys_mgr.Admin_mgr, &responseData[1], 14))  
                  {
                    global_var.power_num = 0;
                    osal_stop_timerEx( bleDoorLock_TaskID, DOOR_SECKEY_EVT);
                    // 鉴权通过开锁
                    global_var.pswd_flag = TRUE;
                    DoorProfile_SetParameter(DOORPROFILE_CHAR2, 16, Updoor_open);      // 锁开了
                    
                    // 断开所有连接
//                    GAPRole_TerminateConnection();

                     // 设置蓝牙广播状态
//                    GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &initial_advertising_enable ); 
                    
                    osal_start_timerEx( bleDoorLock_TaskID, DOOR_LED_ONOFF_EVT, 300 );
                    osal_start_timerEx( bleDoorLock_TaskID, DOOR_OPENDOOR_EVT, 300 ); 
                    
                  }
                  else
                  {
                    global_var.power_num ++;
                    
                    if(global_var.power_num>=1 && global_var.power_num<=2)
                    {
                      Aes_Updata[1] = global_var.power_num;
                      DoorProfile_SetParameter(DOORPROFILE_CHAR2, 16, Aes_Updata);
                    }
                    else
                    {
                      global_var.power_num = 0;
                    }
                  }
                }
   
                // crc出错 可能因为传输错位，可请求重新发一次
                else  
                {
                  global_var.power_num ++;
                    
                  if(global_var.power_num>=1 && global_var.power_num<=2)
                  {
                    Aes_Updata[1] = global_var.power_num;
                    DoorProfile_SetParameter(DOORPROFILE_CHAR2, 16, Aes_Updata);
                  }
                  else
                  {
                    global_var.power_num = 0;
                  }
                }
              }
            }
            else
            {
              DoorProfile_SetParameter(DOORPROFILE_CHAR2, 16, Updoor_open);      // 锁开了
            }
          }
          break;
          
          case 0x40:  //普通权限
          {
            uint8 Lock_OC[1] = {0x00};
            osal_snv_read(0xF0, 1, Lock_OC);
            if( Lock_OC[0] == 0 )      //只有当锁是关时候才有必要开启
            {
              uint8 Nol_Crc = 0;
              uint8 Aes_Updata[16] = {0x04};
              uint8 temp[14] = {0};
              osal_snv_read(0x95, sizeof(SYS_MGR), &sys_mgr);
              if( osal_memcmp(sys_mgr.Gen_mgr, temp, 14) )
              {
                osal_stop_timerEx( bleDoorLock_TaskID, DOOR_SECKEY_EVT);
                osal_memcpy(sys_mgr.Gen_mgr, &responseData[1], 14); //第一次拷贝鉴权码
                osal_snv_write(0x95, sizeof(SYS_MGR), &sys_mgr);
                
                
                // 鉴权通过开锁
                global_var.pswd_flag = TRUE;

                DoorProfile_SetParameter(DOORPROFILE_CHAR2, 16, Updoor_open);      // 锁开了
                // 断开所有连接
//                GAPRole_TerminateConnection();

                 // 设置蓝牙广播状态
//                GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &initial_advertising_enable ); 
                
                osal_start_timerEx( bleDoorLock_TaskID, DOOR_LED_ONOFF_EVT, 300 );
                osal_start_timerEx( bleDoorLock_TaskID, DOOR_OPENDOOR_EVT, 300 ); 
              }
              else
              {

                Nol_Crc = NOL_CRC(sys_mgr.Gen_mgr);

                if( responseData[15] == Nol_Crc )
                {
                  if(osal_memcmpn(sys_mgr.Gen_mgr, &responseData[1], 14))
                  {
                    global_var.power_num = 0;
                    osal_stop_timerEx( bleDoorLock_TaskID, DOOR_SECKEY_EVT);
                    // 鉴权通过开锁
                    global_var.pswd_flag = TRUE;

                    DoorProfile_SetParameter(DOORPROFILE_CHAR2, 16, Updoor_open);      // 锁开了
                    // 断开所有连接
//                    GAPRole_TerminateConnection();

                     // 设置蓝牙广播状态
//                    GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &initial_advertising_enable ); 
                
                    osal_start_timerEx( bleDoorLock_TaskID, DOOR_LED_ONOFF_EVT, 300 );
                    osal_start_timerEx( bleDoorLock_TaskID, DOOR_OPENDOOR_EVT, 300 ); 
                    
                  }
                  // crc位正确  数据出错 可能因为传输错位，可请求重新发一次
                  else
                  {
                    global_var.power_num ++;
                    
                    if(global_var.power_num>=1 && global_var.power_num<=2)
                    {
                      Aes_Updata[1] = global_var.power_num;
                      DoorProfile_SetParameter(DOORPROFILE_CHAR2, 16, Aes_Updata);
                    }
                    else
                    {
                      global_var.power_num = 0;
                    }
                  }
                }
                // crc出错 可能因为传输错位，可请求重新发一次
                else  
                {
                  global_var.power_num ++;
                    
                  if(global_var.power_num>=1 && global_var.power_num<=2)
                  {
                    Aes_Updata[1] = global_var.power_num;
                    DoorProfile_SetParameter(DOORPROFILE_CHAR2, 16, Aes_Updata);
                  }
                  else
                  {
                    global_var.power_num = 0;
                  }
                }
              }
            }
            else
            {
              DoorProfile_SetParameter(DOORPROFILE_CHAR2, 16, Updoor_open);      // 锁开了
            }
          }
          break;
          
          default:
          break;
        }
        break;
      }
      
      case DOORPROFILE_CHAR3:
      {
        
        break;
      }
      
      default:
        // should not reach here!
        break;
    }
  }
}


static uint8 appData_Crc(uint8  *src,uint8 crc, uint8 len)   
{   
  uint8 i; 
  uint8 bb; 
  for(uint8 j=0;j<len;j++)
  {
    bb = src[j];
    for(i=8;i>0;--i)   
    {   
      if(((bb^crc)&0x01))      //判断与x7异或的结果(x8)   
      {
        crc^=0x18;               //反馈到x5   x4   
        crc>>=1;                     //移位   
        crc|=0x80;               //x7异或的结果送x0   
      }
      else
      {
        crc>>=1;
      }  
      bb>>=1;   
    }
  }   
  return(crc);   
} 

#if (defined HAL_LCD) && (HAL_LCD == TRUE)
/*********************************************************************
 * @fn      bdAddr2Str
 *
 * @brief   Convert Bluetooth address to string. Only needed when
 *          LCD display is used.
 *
 * @return  none
 */
char *bdAddr2Str( uint8 *pAddr )
{
  uint8       i;
  char        hex[] = "0123456789ABCDEF";
  static char str[B_ADDR_STR_LEN];
  char        *pStr = str;

  *pStr++ = '0';
  *pStr++ = 'x';

  // Start from end of addr
  pAddr += B_ADDR_LEN;

  for ( i = B_ADDR_LEN; i > 0; i-- )
  {
    *pStr++ = hex[*--pAddr >> 4];
    *pStr++ = hex[*pAddr & 0x0F];
  }

  *pStr = 0;

  return str;
}
#endif // (defined HAL_LCD) && (HAL_LCD == TRUE)



/*********************************************************************
*********************************************************************/
