/***************************************************************************************
 * @file     board.h
 * @version  V2.05
 * @date     14 December 2023
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

#ifndef __BOARD_H
#define __BOARD_H

/*****************************************************************************/
/* Include files                                                             */
/*****************************************************************************/

#include <stdint.h>
#include <stdbool.h>
#include "va416xx.h"
#include "va416xx_hal_ioconfig.h"
#include "main.h"

/*****************************************************************************/
/* Global pre-processor symbols/macros ('#define')                           */
/*****************************************************************************/

/** Version */
#define SOFTWARE_VERSION_STR  "2023_12_14_v2.05"
#define SOFTWARE_VERSION (0x00020005UL)

/* assert enable/disable (comment out to disable) */
/*#define USE_ASSERT*/

/* if defined, enables CRC corruption test functions * DISABLE FOR RELEASE* */
/*#define ENABLE_CRC_TEST*/

/* memory test enable / disable */
//#define BL_DO_MEMORY_TEST

/* fastboot mode enable / disable */
/*#define BL_FASTBOOT_MODE */

/* debug print enable/disable (comment out to disable) */
#define DEBUG_PRINTS
/* debug not use HAL */
#define DEBUG_NO_HAL
/* fputc defined within project */
#define LOCAL_FPUTC
// disables compilation of __write() in va416xx_debug.c if defined
#define LOCAL_WRITE_FUNC

/* enable watchdog (turn off if using breakpoints/debug) */
//#define ENABLE_WATCHDOG

/** Default pin IOCONFIG register. type: un_iocfg_reg_t - see va416xx_hal_ioconfig.h */
/** A pin's IOCONFIG is set to this by HAL_Iocfg_Init() if that pin is not in the cfg array */
#define DEFAULT_PIN_IOCFG   (IOCFG_REG_PULLDN) // internal pulldown enabled for input pin

/** Default pin direction (input/output) type: en_iocfg_dir_t - see va416xx_hal_ioconfig.h */
/** A pin's DIR is set to this by HAL_Iocfg_Init() if that pin is not in the cfg array */
#define DEFAULT_PIN_DIR     (en_iocfg_dir__input) // default pin input

/* Override external clocks (board specific) */
#undef  XTAL
#undef EXTCLK
#undef  HBO
#define XTAL            (10000000UL)      /* 10 MHz xtal */
#define EXTCLK          (40000000UL)      /* EVK ext clk 40M */
#define HBO             (18500000UL)      /* Internal clock */

/* UART rate */
#define UART_BAUDRATE   (115200)
#define BL_NUM_UARTS    (2)

/** LED on MCU top board */
#define EVK_LED_PORT    PORTG
#define EVK_LED_BANK    VOR_GPIO->BANK[6]
#define EVK_LED_PIN     (5)

/*****************************************************************************/
/* Global type definitions ('typedef')                                       */
/*****************************************************************************/

/*****************************************************************************/
/* Global variable declarations ('extern', definition in C source)           */
/*****************************************************************************/

extern const stc_iocfg_pin_cfg_t bootDefaultConfig[];

/*****************************************************************************/
/* Global function prototypes ('extern', definition in C source)             */
/*****************************************************************************/

/*****************************************************************************/
/* End of file                                                               */
/*****************************************************************************/
#endif /* __BOARD_H */
