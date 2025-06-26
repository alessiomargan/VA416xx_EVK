/***************************************************************************************
 * @file     va416xx_hal.h
 * @version  V2.04
 * @date     25 April 2023
 *
 * @note
 * VORAGO Technologies
 *
 * @note
 * Copyright (c) 2013-2023 VORAGO Technologies.
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
 
#ifndef __VA416XX_HAL_H
#define __VA416XX_HAL_H

/*****************************************************************************/ 
/* Include files                                                             */ 
/*****************************************************************************/

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "va416xx.h"
#include "hal_config.h"

/*****************************************************************************/ 
/* Global pre-processor symbols/macros ('#define')                           */ 
/*****************************************************************************/

/** NULL pointer */
#ifndef NULL
#define NULL      ((void *)0)
#endif

/* UNUSED macro - avoids gcc warnings */
#define UNUSED(X) (void)X

/** NVIC maximum priority value */    
#define HAL_NVIC_MAX_PRIO  ((1UL << __NVIC_PRIO_BITS) - 1UL)  // highest priority number (lowest priority)

/** APB clocks */
#define APB1_CLK  (SystemCoreClock/2) // APB1 peripherals 0x4001xxxx
#define APB2_CLK  (SystemCoreClock/4) // APB2 peripherals 0x4002xxxx

/** Watchdog feed */
#ifndef WDFEED
#define WDFEED()  VOR_WATCH_DOG->WDOGINTCLR = 1
#endif

/** SysTick defaults (if not defined in hal_config.h) */
#ifndef SYSTICK_INTERVAL_MS
#define SYSTICK_INTERVAL_MS (10U)
#endif
#ifndef SYSTICK_PRIORITY
#define SYSTICK_PRIORITY    (7U)
#endif
#define HAL_MAX_DELAY       (0xFFFFFFFFU)

#define HAL_LOCK(__HANDLEPTR__)                                         \
                                  do{                                   \
                                    if((__HANDLEPTR__)->locked == true) \
                                    {                                   \
                                      return hal_status_busy;           \
                                    }                                   \
                                    else                                \
                                    {                                   \
                                      (__HANDLEPTR__)->locked = true;   \
                                    }                                   \
                                  }while(0U)

#define HAL_UNLOCK(__HANDLEPTR__)                                       \
                                  do{                                   \
                                    (__HANDLEPTR__)->locked = false;    \
                                  } while(0U)

/*****************************************************************************/
/* Global type definitions ('typedef')                                       */ 
/*****************************************************************************/

/** HAL status (driver call return value) */
typedef enum
{
  hal_status_ok             = 0, 
  hal_status_initError      = 1,
  hal_status_badParam       = 2,
  hal_status_notInitialized = 3,
  hal_status_badPeriphID    = 4,
  hal_status_timeout        = 5,
  hal_status_rxError        = 6,
  hal_status_txError        = 7,
  hal_status_bufEmpty       = 8,
  hal_status_bufFull        = 9,
  hal_status_nak            = 10,
  hal_status_arblost        = 11,
  hal_status_busy           = 12,
  hal_status_notImplemented = 13,
  hal_status_alignmentErr   = 14,
  hal_status_periphErr      = 15,
  hal_status_crcErr         = 16,
  hal_status_end            = 17  // end of list indicator
} hal_status_t;

/*****************************************************************************/
/* Global variable declarations ('extern', definition in C source)           */
/*****************************************************************************/

/** System millisecond counter */
extern volatile uint64_t HAL_time_ms;
extern volatile bool newSysTick;

extern const char * HALStatusStrArr[(uint32_t)hal_status_end+1];

/*****************************************************************************/ 
/* Global function prototypes ('extern', definition in C source)             */ 
/*****************************************************************************/

extern hal_status_t HAL_Init(void);

extern hal_status_t HAL_SysTick_Init(uint32_t intervalMs, uint32_t priority);
extern const char * HAL_StatusToString(hal_status_t stat);
extern void         HAL_WaitMs(const uint32_t ms);
extern uint64_t     HAL_GetTimeMs(void);

// define this in your project somewhere
extern void OnSystemClockChanged(void);

/*****************************************************************************/ 
/* End of file                                                               */ 
/*****************************************************************************/
#endif /* __VA416XX_HAL_H */
