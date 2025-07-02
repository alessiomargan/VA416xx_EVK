/***************************************************************************************
 * @file     hal_config.h
 * @version  V1.21
 * @date     9 Sept 2022
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
 
#ifndef __HAL_CFG_H
#define __HAL_CFG_H

/*****************************************************************************/ 
/* Include files                                                             */ 
/*****************************************************************************/

#include <stdint.h>

/*****************************************************************************/ 
/* Global pre-processor symbols/macros ('#define')                           */ 
/*****************************************************************************/

/** Hardware version (define for VA416xx RevB) */
#define __MCU_HW_VER_REVB

/** Expected VREF voltage */
#define ADC_VREF         (3.3f)     /* units: volts */
#define ADC_VREF_MV      (3300ul)   /* units: millivolts */

/** SysTick setup */
//#define __HAL_DISABLE_SYSTICK       /* allows user SysTick interrupt if defined */
#define SYSTICK_INTERVAL_MS (1u)    /* Internal in milliseconds between SysTick interrupts */
#define SYSTICK_PRIORITY    (7u)

/** PLL setup */
//#define __HAL_PLL_DEBUG_LOG         /* If defined, output detailed PLL config information */

/** SPI setup */
#define __HAL_SPI_MODULE_ENABLED
//#define __HAL_SPI0_ISR_ENABLED
//#define __HAL_SPI1_ISR_ENABLED
//#define __HAL_SPI2_ISR_ENABLED
//#define __HAL_SPI3_ISR_ENABLED

// remove I2C interrupt handlers from build if not using
#define __HAL_DISABLE_I2C0_MASTER
#define __HAL_DISABLE_I2C1_MASTER
#define __HAL_DISABLE_I2C2_MASTER
#define __HAL_DISABLE_I2C0_SLAVE
#define __HAL_DISABLE_I2C1_SLAVE
#define __HAL_DISABLE_I2C2_SLAVE

// remove UART interrupt handlers from build if not using
#define __HAL_DISABLE_UART0
#define __HAL_DISABLE_UART1
#define __HAL_DISABLE_UART2

/** If defined, DMA DONE ISRs will be defined inside the DMA HAL */
#define __HAL_DMA0_DONE_ISR_ENABLED
#define __HAL_DMA1_DONE_ISR_ENABLED
#define __HAL_DMA2_DONE_ISR_ENABLED
#define __HAL_DMA3_DONE_ISR_ENABLED

/*****************************************************************************/
/* Global type definitions ('typedef')                                       */ 
/*****************************************************************************/

/*****************************************************************************/ 
/* Global function prototypes ('extern', definition in C source)             */ 
/*****************************************************************************/

/*****************************************************************************/ 
/* End of file                                                               */ 
/*****************************************************************************/
#endif /* __HAL_CFG_H */
