/***************************************************************************************
 * @file     memtest.h
 * @version  V1.1
 * @date     24 June 2021
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
 
#ifndef __MEMTEST_H
#define __MEMTEST_H

/*****************************************************************************/ 
/* Include files                                                             */ 
/*****************************************************************************/

#include "va416xx_hal.h"
#include "main.h"

/*****************************************************************************/ 
/* Global pre-processor symbols/macros ('#define')                           */ 
/*****************************************************************************/

/*****************************************************************************/
/* Global type definitions ('typedef')                                       */ 
/*****************************************************************************/

/*****************************************************************************/
/* Global variable declarations ('extern', definition in C source)           */
/*****************************************************************************/

extern bool chkInv; // memory check inverse (0xaa/55 or 0x55/aa)

/*****************************************************************************/ 
/* Global function prototypes ('extern', definition in C source)             */ 
/*****************************************************************************/

extern void sramTest_init(void);
extern uint32_t sramTest(void);

/*****************************************************************************/ 
/* End of file                                                               */ 
/*****************************************************************************/
#endif /* __MAIN_H */
