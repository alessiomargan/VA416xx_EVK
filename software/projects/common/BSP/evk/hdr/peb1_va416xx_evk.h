/***************************************************************************************
 * @file     peb1_va416xx_evk.h
 * @version  V2.05
 * @date     13 February 2024
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
 
#ifndef __BSP_PEB1_EVK_H
#define __BSP_PEB1_EVK_H

/*****************************************************************************/ 
/* Include files                                                             */ 
/*****************************************************************************/

/*****************************************************************************/ 
/* Global pre-processor symbols/macros ('#define')                           */ 
/*****************************************************************************/

/** Define external clocks on EVK (board specific) */
/** Can be overridden in project specific board.h */
#define EXTCLK          (40000000UL)      /* XTAL minus frequency */
#define XTAL            (10000000UL)      /* Oscillator frequency */

/** LED on MCU top board */
#define EVK_LED_PORT    PORTG
#define EVK_LED_BANK    VOR_GPIO->BANK[6]
#define EVK_LED_PIN     (5)

/** Board select (EBI or GPIO base board) */
#define BOARD_SEL_PORT  PORTG
#define BOARD_SEL_BANK  VOR_GPIO->BANK[6]
#define BOARD_SEL_PIN   (0)

/** AUX F-ram */
#define FRAM_AUX_SPI_BANK (1)
#define FRAM_AUX_SPI_CSN  (3)

/** LIS2DE12 accelerometer I2C0 */
#define ACCEL_INCLUDE     "i2c_lis2de12.h"
#define ACCEL_I2C         (VOR_I2C0)
#define ACCEL_I2C_BANK    (0)
#define ACCEL_I2C_ADDR    ((i2caddr_shifted_t)(0x32))

/** Accelerometer functions */
#define ACCEL_INIT           LIS2DE12_InitI2C
#define ACCEL_GET_RAW_VALUES LIS2DE12_GetAccelRawValues
#define ACCEL_GET_MG_VALUES  LIS2DE12_GetAccelMgValues
#define ACCEL_GET_TEMP       LIS2DE12_GetBoardTemp
#define ACCEL_UNINIT         LIS2DE12_UnInit

/*****************************************************************************/ 
/* End of file                                                               */ 
/*****************************************************************************/
#endif /* __BSP_PEB1_EVK_H */
