/***************************************************************************************
 * @file     stdio_remap.c
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

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdarg.h>
#include "va416xx.h"
#include "stdio_remap.h"
#include "uart.h"

/*****************************************************************************/ 
/* Local pre-processor symbols/macros ('#define')                            */ 
/*****************************************************************************/

//#define DEBUG_BUFFER_LEN (128)

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

static void UART0TxB(uint8_t b);
static void UART0Tx(char *pcStr, int len);
static void UART1TxB(uint8_t b);
static void UART1Tx(char *pcStr, int len);

/*****************************************************************************/ 
/* Function implementation - global ('extern') and local ('static')          */ 
/*****************************************************************************/

#ifdef PRINTF_REDEFINE
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
#endif

void SetStdioOutput(en_stdio_t io)
{
  ioOut = io;
}

/*******************************************************************************
 **
 ** @brief  TX a byte on UART0
 **
 ******************************************************************************/
static void UART0TxB(uint8_t b)
{
  // Block if UART TX FIFO is full
  while((VOR_UART0->TXSTATUS & UART_TXSTATUS_WRRDY_Msk) == 0){}
  
  // Send the byte
  VOR_UART0->DATA = b;
    
  WDFEED();
}

/*******************************************************************************
 **
 ** @brief  TX a string on UART0, with len
 **
 ******************************************************************************/
static void UART0Tx(char *pcStr, int len)
{
	while(len){
    /* Block until UART FIFO has available room */
    while((VOR_UART0->TXSTATUS & UART_TXSTATUS_WRRDY_Msk) == 0){}

    /* Transmit another character. */
    VOR_UART0->DATA = *pcStr++;
    len--;
	}
}

/*******************************************************************************
 **
 ** @brief  TX a byte on UART1
 **
 ******************************************************************************/
static void UART1TxB(uint8_t b)
{
  // Block if UART TX FIFO is full
  while((VOR_UART1->TXSTATUS & UART_TXSTATUS_WRRDY_Msk) == 0){}
  
  // Send the byte
  VOR_UART1->DATA = b;
    
  WDFEED();
}

/*******************************************************************************
 **
 ** @brief  TX a string on UART1, with len
 **
 ******************************************************************************/
static void UART1Tx(char *pcStr, int len)
{
	while(len){
    /* Block until UART FIFO has available room */
    while((VOR_UART1->TXSTATUS & UART_TXSTATUS_WRRDY_Msk) == 0){}

    /* Transmit another character. */
    VOR_UART1->DATA = *pcStr++;
    len--;
	}
}

/*******************************************************************************
 **
 ** @brief  Block until all characters are sent from TX FIFO
 **
 ******************************************************************************/
void waitForTxComplete(void)
{
  switch(ioOut)
  {
    case en_stdio_uart0:
      while(VOR_UART0->TXSTATUS & UART_TXSTATUS_WRBUSY_Msk);
      break;
    case en_stdio_uart1:
      while(VOR_UART1->TXSTATUS & UART_TXSTATUS_WRBUSY_Msk);
      break;
    case en_stdio_uart01:
      while(VOR_UART0->TXSTATUS & UART_TXSTATUS_WRBUSY_Msk);
      while(VOR_UART1->TXSTATUS & UART_TXSTATUS_WRBUSY_Msk);
      break;
    default:
      break;
  }
}

/*******************************************************************************
 **
 ** @brief  write (called by GCC printf)
 **
 ******************************************************************************/
int _write(int file, char* ptr, int len)
{
  switch(ioOut)
  {
    case en_stdio_uart0:
		  UART0Tx(ptr, len);
      break;
    case en_stdio_uart1:
		  UART1Tx(ptr, len);
      break;
    case en_stdio_uart2:
      break;
    case en_stdio_uart01:
      UART0Tx(ptr, len);
      UART1Tx(ptr, len);
      break;
    default: 
      break;
  }
  return len;
}

/*******************************************************************************
 **
 ** @brief called to print a character, called by Keil / IAR printf
 **
 ******************************************************************************/
int fputc(int c, FILE *fPointer)
{
  uint32_t dm;
  
  switch(ioOut)
  {
    case en_stdio_none:
      break;
    case en_stdio_uart0:
		  UART0TxB((uint8_t)c);
      break;
    case en_stdio_uart1:
		  UART1TxB((uint8_t)c);
      break;
    case en_stdio_uart2:
      break;
    case en_stdio_uart01:
      UART0TxB((uint8_t)c);
      UART1TxB((uint8_t)c);
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

en_stdio_t GetStdioOutput(void)
{
  return ioOut;
}

/*****************************************************************************/ 
/* End of file                                                               */ 
/*****************************************************************************/
