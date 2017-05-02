#include "bcomdef.h"
#include "OSAL.h"
#include "OSAL_PwrMgr.h"
#include "OnBoard.h"
#include "hal_led.h"
#include "hal_key.h"
#include "hal_lcd.h"
#include "gatt.h"
#include "ll.h"
#include "hci.h"
#include "gapgattserver.h"
#include "gattservapp.h"
#include "central.h"
#include "peripheral.h"
#include "gapbondmgr.h"
#include "simpleGATTprofile.h"
#include "npi.h"
#include "osal_snv.h"
#include "simpleBLE.h"

#include "bledoorlock.h"

#include "osal_clock.h"

#include <string.h>

SYS_CONFIG sys_config;
SYS_TEST sys_test;
GLOBAL_VAR global_var;
SYS_MGR sys_mgr;

static uint8 Init_Pswd[6] = "123456";



/*********************************************************************
 * @fn      osal_memcmpn
 *
 * @brief
 *
 *   Generic memory compare . 若src2 在src1  里面有len 长度字节
 *
 * @param   src1 - source 1 addrexx
 * @param   src2 - source 2 address
 * @param   len - number of bytes to compare
 *
 * @return  TRUE - same, FALSE - different
 */
uint8 osal_memcmpn( const void GENERIC *src1, const void GENERIC *src2, unsigned int len )
{
  const uint8 GENERIC *pSrc1;
  const uint8 GENERIC *pSrc2;
  
  
  
  uint8 temp1=0;

  pSrc1 = src1;
  pSrc2 = src2;
  
//  scanf("%s", pSrc2);
  uint8 lenght = strlen(src2)+3;
//  uint8 lenght = sizeof(pSrc2)/sizeof(pSrc2[0]);
  
  while( lenght -- )
  {
    if( *pSrc1++ == *pSrc2++ )
    {
      temp1 ++;
      if(temp1 == len)
        return TRUE;
      
    }
    else
    {
      while(temp1--)
      {
        *pSrc2 --;
        lenght ++;
      }

      temp1 = 0;
      pSrc1 = src1;  // 重新赋值比较
    }
  } 
  return FALSE;
}

/*********************************************************************
 * @fn      osal_memcpyz
 *
 * @brief
 *
 *   Generic memory copy.
 *
 *   Note: This function differs from the standard memcpy(), since
 *         it returns the pointer to the next destination uint8. The
 *         standard memcpy() returns the original destination address.
 *
 * @param   dst - destination address
 * @param   src - source address
 * @param   len - number of bytes to copy
 *
 * @return  pointer to end of destination buffer
 */
void *osal_memcpyz( void *dst, uint8 data, unsigned int len )
{
  uint8 *pDst;
//  const uint8 GENERIC *pSrc;

//  pSrc = src;
  pDst = dst;

  while ( len-- )
    *pDst++ = data;

  return ( pDst );
}
/********************************************************************
 * 求取数组中间值
 *
 *
********************************************************************/
uint8 osal_array(uint8 * src)
{
  uint8 lenght = strlen((char *)src);
  uint8 temp = 0;
  uint8 *psrc ;
  psrc = src;
  for(uint8 i=0;i<lenght-1;i++)
  {
    for(uint8 j=i+1;j<lenght;j++)  
    {
      if(psrc[i]>psrc[j])
      {     
        temp = psrc[i];     
        psrc[i] = psrc[j];         
        psrc[j] = temp;
      }
    } 
  }
  return psrc[lenght/2 + 1];
}
uint8 Crc_Check(uint8 * src, uint16 len)
{
  uint8 i;
  uint8 temp = 0;
  const uint8 * pSrc;
  pSrc = src;
#if 1
  for( i=0; i<len; i++)
  {
    temp = (pSrc[i] + temp)&0x0F;
  }

  return temp;
#else
  for( i=0; i<len; i++)
  {
    if(!(pSrc[i]%2))
      temp ++;

  }
  if((temp)&&(!(temp%2)))
  {
    return TRUE;
  }
  else
  {
    return FALSE;
  }
#endif  
  
}

void Sys_Init(void)
{
  osal_memcpy(sys_config.Admin_Pswd, Init_Pswd, 6);
  
}

void key_varInit(void)
{
  global_var.key_first = 1;
  global_var.power_num = 0;
  global_var.key_2s = 0;
  global_var.key_index = 0;
  global_var.key_num = 0;
  global_var.key_out2s = 0;
  osal_memcpyz(global_var.key_temp, 0, 8);
  global_var.RGB_Flag = 0;
  global_var.pswd_flag = FALSE;
}


