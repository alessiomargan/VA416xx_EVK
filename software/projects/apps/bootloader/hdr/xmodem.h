/***************************************************************************************
 * @file     xmodem.h
 * @version  V1.1
 * @date     29 June 2021
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
 
#ifndef __XMODEM_H
#define __XMODEM_H

/*****************************************************************************/ 
/* Include files                                                             */ 
/*****************************************************************************/

#include <stdint.h>
#include <stdbool.h>

/*****************************************************************************/ 
/* Global pre-processor symbols/macros ('#define')                           */ 
/*****************************************************************************/

/** XMODEM control characters */
#define XMODEM_SOH                1
#define XMODEM_EOT                4
#define XMODEM_ACK                6
#define XMODEM_NAK                21
#define XMODEM_CAN                24
#define XMODEM_NCG                67

/** XMODEM packet data size */
#define XMODEM_DATA_SIZE          128

/*****************************************************************************/
/* Global type definitions ('typedef')                                       */ 
/*****************************************************************************/

/** XMODEM packet */
typedef struct stc_xmodem_packet
{
  uint8_t u8Padding; ///< Padding to make sure data is 32 bit aligned
  uint8_t u8Header;
  uint8_t u8PacketNumber;
  uint8_t u8PacketNumberC;
  uint8_t au8Data[XMODEM_DATA_SIZE];
  uint8_t u8CrcHigh;
  uint8_t u8CrcLow;
} stc_xmodem_packet_t;

/*****************************************************************************/ 
/* Global function prototypes ('extern', definition in C source)             */ 
/*****************************************************************************/

extern size_t XMODEM_ReceiveFile(uint8_t* pu8BaseAddress, uint8_t* pu8EndAddress, uint8_t u8Uart);
extern size_t XMODEM_SendFile(uint8_t* pu8BaseAddress, size_t szLength, uint8_t u8Uart);

/*****************************************************************************/ 
/* End of file                                                               */ 
/*****************************************************************************/
#endif // __XMODEM_H
