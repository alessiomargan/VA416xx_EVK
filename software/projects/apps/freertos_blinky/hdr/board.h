/***************************************************************************************
 * @file     board.h
 * @version  V2.05
 * @date     06 March 2024
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
 
#ifndef __BOARD_H
#define __BOARD_H

/*****************************************************************************/ 
/* Include files                                                             */ 
/*****************************************************************************/

#include <stdint.h>
#include <stdbool.h>
#include "va416xx.h"
#include "va416xx_hal_ioconfig.h"
#include "va416xx_hal_clkgen.h"

/*****************************************************************************/ 
/* Global pre-processor symbols/macros ('#define')                           */ 
/*****************************************************************************/

/* Define one of these */
#define __VA41628__
//#define __VA41629__
//#define __VA416X0__

/** BSP selection */
#if defined(__VA41628__)
#include "peb2_va41628_evk.h"
#define DEVICE_NAME "VA41628"
#elif defined(__VA416X0__)
#include "peb1_va416xx_evk.h"
#define DEVICE_NAME "VA416X0"
elif defined(__VA41629__)
#include "peb1_va416xx_evk.h"
#define DEVICE_NAME "VA41629"
#endif

/** Software Version */
#define SOFTWARE_VERSION_STR  "2023_12_14_2.05"
#define SOFTWARE_VERSION (0x00020005UL)

// assert enable/disable (comment out to disable)
//#define USE_ASSERT

// debug print enable/disable (comment out to disable)
#define DEBUG_PRINTS

// debug prints do not call the Uart HAL
#define DEBUG_NO_HAL

// Enable Segger RTT (comment out to disable)
//#define ENABLE_RTT

// turn calls to printf() into VOR_printf()
#define PRINTF_REDEFINE

// enable watchdog (turn off if using breakpoints/debug)
//#define ENABLE_WATCHDOG

/** Default pin IOCONFIG register. type: un_iocfg_reg_t - see va416xx_hal_ioconfig.h */
/** A pin's IOCONFIG is set to this by HAL_Iocfg_Init() if that pin is not in the cfg array */
#define DEFAULT_PIN_IOCFG   (IOCFG_REG_PULLDN) // internal pulldown enabled for input pin

/** Default pin direction (input/output) type: en_iocfg_dir_t - see va416xx_hal_ioconfig.h */
/** A pin's DIR is set to this by HAL_Iocfg_Init() if that pin is not in the cfg array */
#define DEFAULT_PIN_DIR     (en_iocfg_dir__input) // default pin input

/* Override external clocks (project specific) */
#undef   EXTCLK
#undef   XTAL
#undef   HBO
#define  EXTCLK          (40000000UL)      /* XTAL minus frequency */
#define  XTAL            (10000000UL)      /* Oscillator frequency */
#define  HBO             (18500000UL)      /* Internal HBO frequency (18-22mhz) */

/* Expected VREF voltage */
#define ADC_VREF         (3.3f)

/* UART baudrate */ 
#define UART_BAUDRATE    (115200)

/*****************************************************************************/
/* Global type definitions ('typedef')                                       */ 
/*****************************************************************************/

/*****************************************************************************/
/* Global variable declarations ('extern', definition in C source)           */
/*****************************************************************************/

// default IO pin configuration array (all pins)
extern const stc_iocfg_pin_cfg_t ioPinCfgArr[];

// EBI pin configuration array (EBI pins only)
extern const stc_iocfg_pin_cfg_t ebiPinCfgArr[];

// Ethernet pin configuration array (ETH pins only)
extern const stc_iocfg_pin_cfg_t ethPinCfgArr[];

// Current external clock source type (none, crystal, or external clock)
extern hal_xtalsel_t gCurrentXtalsel; // defined in main.c

// system seconds counter, defined in main.c
extern volatile uint32_t gSecondsCounter;

/*****************************************************************************/ 
/* Global function prototypes ('extern', definition in C source)             */ 
/*****************************************************************************/

/*****************************************************************************/ 
/* End of file                                                               */ 
/*****************************************************************************/
#endif /* __BOARD_H */
