/***************************************************************************************
 * @file     bootloader.h
 * @version  V2.02
 * @date     16 March 2023
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
 
#ifndef __BOOTLOADER_H
#define __BOOTLOADER_H

/*****************************************************************************/ 
/* Include files                                                             */ 
/*****************************************************************************/

#include <stdint.h>
#include <stdbool.h>
#include "board.h"

/*****************************************************************************/ 
/* Global pre-processor symbols/macros ('#define')                           */ 
/*****************************************************************************/

/* PC MODE */
/* Prints more verbose programming information, ideal for use with a terminal
   program such as PuTTY. Undef for serial MCU<->MCU mode to remove excess prints */
#define PC_MODE

/* Bootloader timeout */
/* Defines the amount of time (in milliseconds) to wait for the heralding 
   character to indicate intent to upload a new user application */
#ifdef PC_MODE
#define BOOTLOADER_MSEC_TIMEOUT	5000
#else
#define BOOTLOADER_MSEC_TIMEOUT	100
#endif

/* Bootloader heralding character */
/* This is the character that the bootloader will look for on the UART
   to indicate intent to upload a new user application */
#define BOOTLOADER_HERALDING_CHAR ' ' // ASCII 'space' character
/* and this is the response sent when in MCU<->MCU mode */
#define BOOTLOADER_HERALDING_RESP_CHAR 'C' // ASCII 'C' character

/* Important bootloader addresses and offsets, vector table information */
#define BOOTLOADER_START_ADDR    (0x0UL)
#define BOOTLOADER_END_ADDR      (0x4000UL)
#define BOOTLOADER_CRC_ADDR      (0x3FFEUL)
#define APP_A_START_ADDR         (0x4000UL)
#define APP_A_END_ADDR           (0x22000UL)
#define APP_A_CRC_LEN_ADDR       (0x21FF8UL)
#define APP_A_CRC_ADDR           (0x21FFCUL)
#define APP_B_START_ADDR         (0x22000UL)
#define APP_B_END_ADDR           (0x40000UL)
#define APP_B_CRC_LEN_ADDR       (0x3FFF8UL)
#define APP_B_CRC_ADDR           (0x3FFFCUL)
#define APP_IMG_SZ               (0x1E000UL)

#define VECTOR_TABLE_OFFSET      (0x0UL)
#define VECTOR_TABLE_LEN         (0x350UL)
#define RESET_VECTOR_OFFSET      (0x4UL)

/* ROM SPI info */
#define ROM_SPI_BANK  (3)
#define ROM_SPI_CSN   (0)

/* Programming page size */
#define SZPAGE        (256)
 
/*****************************************************************************/
/* Global type definitions ('typedef')                                       */ 
/*****************************************************************************/

typedef enum{
  en_memcheck_no_app_valid = 0,
  en_memcheck_app_a_valid = 1,
  en_memcheck_app_b_valid = 2,
  en_memcheck_txt_a_valid = 4,
  en_memcheck_txt_b_valid = 8
} en_memcheck_t;

typedef enum{
  en_runapp_a,
  en_runapp_b
} en_runapp_t;

/*****************************************************************************/ 
/* Global function prototypes ('extern', definition in C source)             */ 
/*****************************************************************************/

extern en_memcheck_t Bootloader_CheckAppIsValid(bool fast);
extern bool Bootloader_ReloadCode(void);
extern void Bootloader_RunApp(en_runapp_t);
extern void Bootloader_Task(void);
extern void Bootloader_CheckOwnCRC(void);
extern void vector_reset(void); // in vectrst.s

/*****************************************************************************/ 
/* End of file                                                               */ 
/*****************************************************************************/
#endif /* __BOOTLOADER_H */
