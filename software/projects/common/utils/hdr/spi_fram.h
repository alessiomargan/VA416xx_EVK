/***************************************************************************************
 * @file     spi_fram.h
 * @version  V1.12
 * @date     04 March 2022
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

#ifndef __SPI_FRAM_H
#define __SPI_FRAM_H

/*****************************************************************************/
/* Include files                                                             */
/*****************************************************************************/

#include "va416xx_hal.h"

/*****************************************************************************/
/* Global pre-processor symbols/macros ('#define')                           */
/*****************************************************************************/

#define FRAM_LEN (0x40000)

//#define USE_HAL_DRIVER

/* Set FRAM into sleep mode on FRAM_UnInit() */
/* Strongly recommended in radiation environment */
#define ENABLE_FRAM_SLEEP

/* ROM SPI info */
#define ROM_SPI_BANK  (3)
#define ROM_SPI_CSN   (0)

/*****************************************************************************/
/* Global type definitions ('typedef')                                       */
/*****************************************************************************/

/*****************************************************************************/
/* Global variable declarations ('extern', definition in C source)           */
/*****************************************************************************/

/*****************************************************************************/
/* Global function prototypes ('extern', definition in C source)             */
/*****************************************************************************/

extern hal_status_t FRAM_Init  (uint8_t spiBank, uint8_t csNum);
extern hal_status_t FRAM_Write (uint8_t spiBank, uint32_t addr, uint8_t *buf, uint32_t len);
extern hal_status_t FRAM_Read  (uint8_t spiBank, uint32_t addr, uint8_t *buf, uint32_t len);
extern hal_status_t FRAM_Read16(uint8_t spiBank, uint32_t addr, uint16_t *buf, uint32_t len);
extern uint32_t     FRAM_Verify(uint8_t spiBank, uint32_t addr, uint8_t *buf, uint32_t len);
extern hal_status_t FRAM_UnInit(uint8_t spiBank);

/*****************************************************************************/
/* End of file                                                               */
/*****************************************************************************/
#endif /* __SPI_FRAM_H */
