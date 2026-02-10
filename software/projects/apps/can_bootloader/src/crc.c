/***************************************************************************************
 * @file     crc.c
 * @version  V1.1
 * @date     23 June 2021
 *
 * @note
 * VORAGO Technologies
 *
 * @note
 * Copyright (c) 2013-2021 VORAGO Technologies.
 *
 * @par
 * BY DOWNLOADING, INSTALLING OR USING THIS SOFTWARE, YOU AGREE TO BE BOUND BY
 * ALL THE TERMS AND CONDITIONS OF THE VORAGO TECHNOLOGIES END USER LICENSE AGREEMENT.
 * THIS SOFTWARE IS PROVIDED "AS IS". NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 * OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY
 * AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. VORAGO TECHNOLOGIES
 * SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
 * DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 ****************************************************************************************/
 
/*****************************************************************************/ 
/* Include files                                                             */ 
/*****************************************************************************/

#include "crc.h"

/*****************************************************************************/ 
/* Function implementation - global ('extern') and local ('static')          */ 
/*****************************************************************************/

/*******************************************************************************
 **
 ** @brief  Calculates CRC16 of a block from 'begin' to 'end'
 **         CRC starts at zero in this implementation
 **
 ******************************************************************************/
uint16_t CalculateCRC16(const uint8_t* pu8Begin, const uint8_t* pu8End)
{
  uint16_t u16Crc = 0x0;
  for (; pu8Begin < pu8End; pu8Begin++)
  {
    u16Crc  = (u16Crc >> 8) | (u16Crc << 8);
    u16Crc ^= *pu8Begin;
    u16Crc ^= (u16Crc & 0xff) >> 4;
    u16Crc ^= (u16Crc << 12);
    u16Crc ^= (u16Crc & 0xff) << 5;
  }
  return u16Crc;
}

/*****************************************************************************/ 
/* End of file                                                               */ 
/*****************************************************************************/
