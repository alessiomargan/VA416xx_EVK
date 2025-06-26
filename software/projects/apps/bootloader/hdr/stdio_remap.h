/***************************************************************************************
 * @file     stdio_remap.h
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
 
#ifndef __STDIO_REMAP_H
#define __STDIO_REMAP_H

/*****************************************************************************/ 
/* Include files                                                             */ 
/*****************************************************************************/

#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include "board.h"
#include "main.h"
#ifdef ENABLE_RTT
#include "segger_rtt.h"
#endif

/*****************************************************************************/ 
/* Global pre-processor symbols/macros ('#define')                           */ 
/*****************************************************************************/

/** Runtime assert */
#ifdef USE_ASSERT
#define c_assert(e) assert(e)
#else
#define c_assert(e) ((void)0)
#endif
  
/** Compile time assert */
#ifdef USE_ASSERT
#define COMPILE_TIME_ASSERT(pred) \
  switch(0){case 0:case pred:;}
#else
#define COMPILE_TIME_ASSERT(pred) ((void)0)
#endif

#ifdef PRINTF_REDEFINE
#define printf(...) VOR_printf(__VA_ARGS__)
#endif

/*****************************************************************************/
/* Global type definitions ('typedef')                                       */ 
/*****************************************************************************/
  
typedef enum{
  en_stdio_none,
  en_stdio_uart0,
  en_stdio_uart1,
  en_stdio_uart2,
  en_stdio_uart01,
  en_stdio_porta,
  en_stdio_portb,
  en_stdio_portc,
  en_stdio_portd,
  en_stdio_rtt
} en_stdio_t;

/*****************************************************************************/
/* Global variable declarations ('extern', definition in C source)           */
/*****************************************************************************/

/*****************************************************************************/ 
/* Global function prototypes ('extern', definition in C source)             */ 
/*****************************************************************************/

extern void SetStdioOutput(en_stdio_t io);
extern en_stdio_t GetStdioOutput(void);

/*****************************************************************************/ 
/* End of file                                                               */ 
/*****************************************************************************/
#endif /* __STDIO_REMAP_H */
