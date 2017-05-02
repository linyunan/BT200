/**************************************************************************************************
  Filename:       simpleBLEperipheral.h
  Revised:        $Date: 2010-08-01 14:03:16 -0700 (Sun, 01 Aug 2010) $
  Revision:       $Revision: 23256 $

  Description:    This file contains the Simple BLE Peripheral sample application
                  definitions and prototypes.

  Copyright 2010 - 2011 Texas Instruments Incorporated. All rights reserved.

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

#ifndef SIMPLEBLEPERIPHERAL_H
#define SIMPLEBLEPERIPHERAL_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
  
#define LED1 P1_0
#define LED2 P1_7
#define LED3 P1_4
  
  
  
#if 0
#define ADMIN_PSWD_LEN  10
#define GENERAL_PSWD_LEN  10

#define ADMIN_PHONE_LEN   6
#define GENERAL_PHONE_LEN	6

#endif
extern uint8 key[16];
  
#define BUTTON_UINT8(hiByte, loByte) \
          ((uint8)(((loByte-0x30) & 0x0F) + (((hiByte-0x30) & 0x0F))*10))

#define SYS_INIT    P1_7
#define KEY_DOWN    P1_4
//extern  uint8 bleDoorLock_TaskID;

/*********************************************************************
 * CONSTANTS
 */
extern uint8 key[16]; 


// Simple BLE Peripheral Task Events
#define DOOR_START_DEVICE_EVT                   0x0001
#define DOOR_PERIODIC_EVT                       0x0002	 //	系统实时性任务
#define DOOR_SECKEY_EVT			        0x0004	 //	秘钥检测任务
#define DOOR_2S_EVT                             0x0008   //     2S任务处理
          
#define DOOR_OPENDOOR_EVT			0x0010	 //     开关门任务
#define DOOR_CLOSEDOOR_EVT                      0x0020   //     关锁
#define DOOR_POWERONOFF_EVT			0x0040	 //	待机 和唤醒任务
#define DOOR_LED_ONOFF_EVT                      0x0080   //     灯闪烁任务
#define DOOR_KEY_LONG_EVT                       0x0100   //     按键长按
#define DOOR_LED_ONETIME_EVT                       0x0200   //     按键长按
#define DOOR_PSWD_OPEN_EVT                      0x0800   //     按键密码开门
#define DOOR_OUTTIME_EVT                        0x1000   //     2S后处理函数 若key_index >= 6 就去密码匹配

//#define DOOR_5S_EVT                             0x2000   //     5秒就去密码匹配
#define DOOR_BATTERY_EVT                        0x4000   //     电池检测任务
/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * FUNCTIONS
 */

/*
 * Task Initialization for the BLE Application
 */
extern void BleDoorLock_Init( uint8 task_id );

/*
 * Task Event Processor for the BLE Application
 */
extern uint16 BleDoorLock_ProcessEvent( uint8 task_id, uint16 events );
#if (defined HAL_LCD) && (HAL_LCD == TRUE)
extern char *bdAddr2Str( uint8 *pAddr );
#endif

void Encrty_Addr(unsigned char * Dest);
/*********************************************************************
*********************************************************************/

extern uint8 Lock_OC[1] ;


#ifdef __cplusplus
}
#endif

#endif /* SIMPLEBLEPERIPHERAL_H */
