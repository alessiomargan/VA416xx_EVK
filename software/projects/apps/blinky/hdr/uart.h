/***************************************************************************************
 * @file     uart.h
 * @version  V2.00
 * @date     09 December 2022
 *
 * @note
 * VORAGO Technologies
 *
 * @note
 * Copyright (c) 2013-2022 VORAGO Technologies. 
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
/** @file */ 
/** \addtogroup UART 
 *  @{
 */
#ifndef __UART_H
#define __UART_H

/*****************************************************************************/ 
/* Include files                                                             */ 
/*****************************************************************************/

#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "board.h"

/*****************************************************************************/ 
/* Global pre-processor symbols/macros ('#define')                           */ 
/*****************************************************************************/

/* 0 = no echo, 1 = echo enabled */
#define UART_ECHO 0

#ifndef UART_RX_BUFFER_SIZE
#define UART_RX_BUFFER_SIZE   32
#endif

#define UART_CLK (SystemCoreClock / 4)
#define UART2_CLK (SystemCoreClock / 2)
#define UART_CALC_CLOCKSCALE(_scc,_baud) ((_scc / (_baud * 16)) << \
                   UART_CLKSCALE_INT_Pos) |  \
                   (((((_scc % (_baud * 16)) * \
                   64 + (_baud * 8)) / \
                   (_baud * 16))) << \
                   UART_CLKSCALE_FRAC_Pos)

#define UART0SendBreak(x) VOR_UART0->TXBREAK=(x)
#define UART1SendBreak(x) VOR_UART1->TXBREAK=(x)
#define UART2SendBreak(x) VOR_UART2->TXBREAK=(x)

/*****************************************************************************/ 
/* Global function prototypes ('extern', definition in C source)             */ 
/*****************************************************************************/

/* Intializes UART peripheral. */
extern void UartInit(VOR_UART_Type* uart, uint32_t baudrate);


/* Reads a byte from the software RX Buffer. */
extern bool UART0Read(uint8_t *data);

/* Writes a string to UART0, and blocks until complete. */
extern void UART0TxStr(char *pcStr);

/* TX one byte over UART0 */
extern void UART0TxByte(uint8_t b);

/* RX one byte over UART0 (with timeout) */
extern uint8_t UART0RxByte(void);

/* check if a byte is available over UART0 */
extern bool UART0IsRxByteAvailable(void);

/* Get string of characters in the buffer up to limit chars */
extern uint32_t UART0GetBuffer(uint8_t *data, uint32_t limit);


/* Reads a byte from the software RX Buffer. */
extern bool UART1Read(uint8_t *data);

/* Writes a string to UART1, and blocks until complete. */
extern void UART1TxStr(char *pcStr);

/* TX one byte over UART1 */
extern void UART1TxByte(uint8_t b);

/* RX one byte over UART1 (with timeout) */
extern uint8_t UART1RxByte(void);

/* check if a byte is available over UART1 */
extern bool UART1IsRxByteAvailable(void);

/* Get string of characters in the buffer up to limit chars */
extern uint32_t UART1GetBuffer(uint8_t *data, uint32_t limit);


/* Reads a byte from the software RX Buffer. */
extern bool UART2Read(uint8_t *data);

/* Writes a string to UART1, and blocks until complete. */
extern void UART2TxStr(char *pcStr);

/* TX one byte over UART1 */
extern void UART2TxByte(uint8_t b);

/* RX one byte over UART1 (with timeout) */
extern uint8_t UART2RxByte(void);

/* check if a byte is available over UART1 */
extern bool UART2IsRxByteAvailable(void);

/* Get string of characters in the buffer up to limit chars */
extern uint32_t UART2GetBuffer(uint8_t *data, uint32_t limit);

/*****************************************************************************/ 
/* End of file                                                               */ 
/*****************************************************************************/
#endif /* __UART_H */
