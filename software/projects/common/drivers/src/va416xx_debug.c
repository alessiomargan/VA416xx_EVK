/***************************************************************************************
 * @file     va416xx_hal_debug.c
 * @version  V2.05
 * @date     23 January 2024
 *
 * @note
 * VORAGO Technologies
 *
 * @note
 * Copyright (c) 2013-2024 VORAGO Technologies.
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

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdarg.h>
#include "va416xx.h"
#include "va416xx_debug.h"

#ifndef DEBUG_NO_HAL
#include "va416xx_hal_uart.h"
#else
#include "uart.h"
#endif

#ifdef ENABLE_RTT
#include "segger_rtt.h"
#endif

/*****************************************************************************/ 
/* Local pre-processor symbols/macros ('#define')                            */ 
/*****************************************************************************/

/*****************************************************************************/ 
/* Global variable definitions (declared in header file with 'extern')       */ 
/*****************************************************************************/

/*****************************************************************************/ 
/* Local type definitions ('typedef')                                        */ 
/*****************************************************************************/

/*****************************************************************************/ 
/* Local variable definitions ('static')                                     */ 
/*****************************************************************************/

static en_stdio_t ioOut = en_stdio_none;
#ifdef ENABLE_RTT
static uint8_t log_buff[100];
#endif

/*****************************************************************************/ 
/* Local function prototypes ('static')                                      */ 
/*****************************************************************************/

/*****************************************************************************/ 
/* Function implementation - global ('extern') and local ('static')          */ 
/*****************************************************************************/

void VOR_printf(const char *fmt, ...)
{
  va_list args;
  va_start(args, fmt);
  
  if(ioOut == en_stdio_rtt){
#ifdef ENABLE_RTT
    vsnprintf((char *)(log_buff), 100, fmt, args);
    SEGGER_RTT_WriteString(0, (const char *)log_buff);
#endif
  } else {
    vfprintf(stdout, fmt, args);
    fflush(stdout);
  }
  
  va_end(args);
}

void DBG_printf(const char *fmt, ...)
{
  va_list args;
  va_start(args, fmt);
  
  if(ioOut == en_stdio_rtt){
#ifdef ENABLE_RTT
    vsnprintf((char *)(log_buff), 100, fmt, args);
    SEGGER_RTT_WriteString(0, "\nDEBUG: ");
    SEGGER_RTT_WriteString(0, (const char *)log_buff);
#endif
  } else {
    fprintf(stderr, "\nDEBUG: ");
    vfprintf(stderr, fmt, args);
    fflush(stderr);
  }
  
  va_end(args);
}

void DBG_println(const char *fmt, ...)
{
  va_list args;
  va_start(args, fmt);
  
  if(ioOut == en_stdio_rtt){
#ifdef ENABLE_RTT
    vsnprintf((char *)(log_buff), 100, fmt, args);
    SEGGER_RTT_WriteString(0, "\nDEBUG: ");
    SEGGER_RTT_WriteString(0, (const char *)log_buff);
    SEGGER_RTT_WriteString(0, "\r\n");
#endif
  } else {
    fprintf(stderr, "\nDEBUG: ");
    vfprintf(stderr, fmt, args);
    fprintf(stderr, "\r\n");
    fflush(stderr);
  }
  
  va_end(args);
}

void DBG_SetStdioOutput(en_stdio_t io)
{
  ioOut = io;
  
  switch(io)
  {
    case en_stdio_porta:
      // Configure BANK0 as outputs
      VOR_GPIO->BANK[0].DIR |= 0x000000FFU;
      // Configure BANK0 bit 7 as pulse mode
      VOR_GPIO->BANK[0].PULSE |= 0x00000080U;
      break;
    case en_stdio_portb:
      // Configure BANK1 as outputs
      VOR_GPIO->BANK[1].DIR |= 0x000000FFU;
      // Configure BANK1 bit 7 as pulse mode
      VOR_GPIO->BANK[1].PULSE |= 0x00000080U;
      break;
    case en_stdio_portc:
      // Configure BANK2 as outputs
      VOR_GPIO->BANK[2].DIR |= 0x000000FFU;
      // Configure BANK2 bit 7 as pulse mode
      VOR_GPIO->BANK[2].PULSE |= 0x00000080U;
      break;
    case en_stdio_portd:
      // Configure BANK3 as outputs
      VOR_GPIO->BANK[3].DIR |= 0x000000FFU;
      // Configure BANK3 bit 7 as pulse mode
      VOR_GPIO->BANK[3].PULSE |= 0x00000080U;
      break;
    case en_stdio_rtt:
#ifdef ENABLE_RTT
      SEGGER_RTT_Init();
	    SEGGER_RTT_ConfigUpBuffer(0, NULL, NULL, 0, SEGGER_RTT_MODE_NO_BLOCK_SKIP);
#endif
      break;
    default:
      break;
  }
}

/* this is the write function used by IAR, Keil */
#ifndef LOCAL_FPUTC
int fputc(int c, FILE *fPointer)
{
  uint32_t dm;
  
  switch(ioOut)
  {
    case en_stdio_none:
      break;
    case en_stdio_uart0:
#ifdef DEBUG_NO_HAL
      UART0TxByte((uint8_t)c);
#else
      HAL_Uart_TxByte(VOR_UART0, (uint8_t)c);
#endif
      break;
    case en_stdio_uart1:
#ifdef DEBUG_NO_HAL
      UART1TxByte((uint8_t)c);
#else
      HAL_Uart_TxByte(VOR_UART1, (uint8_t)c);
#endif
      break;
    case en_stdio_uart2:
#ifdef DEBUG_NO_HAL
      UART2TxByte((uint8_t)c);
#else
      HAL_Uart_TxByte(VOR_UART2, (uint8_t)c);
#endif
      break;
    case en_stdio_porta:
      dm = VOR_GPIO->BANK[0].DATAMASK;
      VOR_GPIO->BANK[0].DATAMASK  = 0xff;       // Write DataMask
      VOR_GPIO->BANK[0].DATAOUT   = 0x80 | c;  // Pulse bit 7
      VOR_GPIO->BANK[0].DATAMASK = dm;
      break;
    case en_stdio_portb:
      dm = VOR_GPIO->BANK[1].DATAMASK;
      VOR_GPIO->BANK[1].DATAMASK  = 0xff;       // Write DataMask
      VOR_GPIO->BANK[1].DATAOUT   = 0x80 | c;  // Pulse bit 7
      VOR_GPIO->BANK[1].DATAMASK = dm;
      break;
    case en_stdio_portc:
      dm = VOR_GPIO->BANK[2].DATAMASK;
      VOR_GPIO->BANK[2].DATAMASK  = 0xff;       // Write DataMask
      VOR_GPIO->BANK[2].DATAOUT   = 0x80 | c;  // Pulse bit 7
      VOR_GPIO->BANK[2].DATAMASK = dm;
      break;
    case en_stdio_portd:
      dm = VOR_GPIO->BANK[3].DATAMASK;
      VOR_GPIO->BANK[3].DATAMASK  = 0xff;       // Write DataMask
      VOR_GPIO->BANK[3].DATAOUT   = 0x80 | c;  // Pulse bit 7
      VOR_GPIO->BANK[3].DATAMASK = dm;
      break;
    case en_stdio_rtt:
#ifdef ENABLE_RTT
      SEGGER_RTT_PutChar(0, (uint8_t)c);
#endif
      break;
  }
  return c;
}
#endif

/* returns the IO interface chosen */
en_stdio_t DBG_GetStdioOutput(void)
{
  return ioOut;
}

/* this is the write function used when using GCC */
#ifndef LOCAL_WRITE_FUNC
int _write(int file, char *ptr, int len)
{
  switch(ioOut)
  {
    case en_stdio_none:
      break;
    case en_stdio_uart0:
#ifdef DEBUG_NO_HAL
      UART0TxStr(ptr);
#else
      HAL_Uart_TxStr(VOR_UART0, ptr);
#endif
      break;
    case en_stdio_uart1:
#ifdef DEBUG_NO_HAL
      UART1TxStr(ptr);
#else
      HAL_Uart_TxStr(VOR_UART1, ptr);
#endif
      break;
    case en_stdio_uart2:
#ifdef DEBUG_NO_HAL
      UART2TxStr(ptr);
#else
      HAL_Uart_TxStr(VOR_UART2, ptr);
#endif
      break;
    default:
      break;
  }
  return (len);
}
#endif

/*****************************************************************************/ 
/* End of file                                                               */ 
/*****************************************************************************/
