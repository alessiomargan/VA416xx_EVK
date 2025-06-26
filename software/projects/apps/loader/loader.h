/***************************************************************************************
 * @file     loader.h
 * @version  V2.05
 * @date     28 July 2023
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
#ifndef LOADER_H
#define LOADER_H

/*****************************************************************************/ 
/* Global pre-processor symbols/macros ('#define')                           */ 
/*****************************************************************************/

#define BOOT_FROM_SPI3                              // Chip is booting from SPI3
//#define BOOT_FROM_EBI                               // Chip is booting from EBI
#define SZDEV                       (0x00040000)    // Device Size in Bytes (256KB)

#ifdef BOOT_FROM_SPI0
    #define BOOT_SPI                VOR_SPI0
    #define BOOT_SPI_RESET          SYSCONFIG_PERIPHERAL_RESET_SPI0_Msk
    #define BOOT_SPI_CLK_ENABLE     CLK_ENABLE_SPI0
#endif

#ifdef BOOT_FROM_SPI1
    #define BOOT_SPI                VOR_SPI1
    #define BOOT_SPI_RESET          SYSCONFIG_PERIPHERAL_RESET_SPI1_Msk
    #define BOOT_SPI_CLK_ENABLE     CLK_ENABLE_SPI1
#endif

#ifdef BOOT_FROM_SPI2
    #define BOOT_SPI                VOR_SPI2
    #define BOOT_SPI_RESET          SYSCONFIG_PERIPHERAL_RESET_SPI2_Msk
    #define BOOT_SPI_CLK_ENABLE     CLK_ENABLE_SPI2
#endif

#ifdef BOOT_FROM_SPI3
    #define BOOT_SPI                VOR_SPI3
    #define BOOT_SPI_RESET          SYSCONFIG_PERIPHERAL_RESET_SPI3_Msk
    #define BOOT_SPI_CLK_ENABLE     CLK_ENABLE_SPI3
#endif

#ifdef BOOT_FROM_EBI
    #define BOOT_EBI_START_ADDR     (0x10000000)
#endif

// FM25V20A Commands
#define WREN                        (0x06)
#define WRDI                        (0x04)
#define RDSR                        (0x05)
#define WRSR                        (0x01)
#define READ                        (0x03)
#define WRITE                       (0x02)
#define RDID                        (0x9F) 
#define TTZ2564_SE                  (0x20) 
#define TTZ2564_BE                  (0x52)
#define TTZ2564_CE                  (0x60) 

/*****************************************************************************/ 
/* End of file                                                               */ 
/*****************************************************************************/
#endif  /* LOADER_H */
