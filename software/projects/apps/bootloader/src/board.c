/***************************************************************************************
 * @file     board.c
 * @version  V1.5
 * @date     24 February 2022
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
 
/*****************************************************************************/ 
/* Include files                                                             */ 
/*****************************************************************************/

#include "board.h"

/*****************************************************************************/ 
/* Local pre-processor symbols/macros ('#define')                            */ 
/*****************************************************************************/

/*****************************************************************************/ 
/* Global variable definitions (declared in header file with 'extern')       */ 
/*****************************************************************************/

const stc_iocfg_pin_cfg_t bootDefaultConfig[] = 
{

{VOR_PORTB,14,en_iocfg_dir_dncare, {{.fltclk=0,.invinp=0,.iewo=0,.opendrn=0,.invout=0,.plevel=0,.pen=0,.pwoa=0,.funsel=3,.iodis=0}}}, /* Uart1 TX */
{VOR_PORTB,15,en_iocfg_dir_dncare, {{.fltclk=0,.invinp=0,.iewo=0,.opendrn=0,.invout=0,.plevel=0,.pen=0,.pwoa=0,.funsel=3,.iodis=0}}}, /* Uart1 RX */
  
{VOR_PORTG, 0,en_iocfg_dir_dncare, {{.fltclk=0,.invinp=0,.iewo=0,.opendrn=0,.invout=0,.plevel=0,.pen=0,.pwoa=0,.funsel=1,.iodis=0}}}, /* Uart0 TX */
{VOR_PORTG, 1,en_iocfg_dir_dncare, {{.fltclk=0,.invinp=0,.iewo=0,.opendrn=0,.invout=0,.plevel=0,.pen=0,.pwoa=0,.funsel=1,.iodis=0}}}, /* Uart0 RX */
{VOR_PORTG, 2,en_iocfg_dir_output, {{.fltclk=0,.invinp=0,.iewo=0,.opendrn=0,.invout=0,.plevel=0,.pen=0,.pwoa=0,.funsel=1,.iodis=0}}}, /* out low */

{VOR_PORTG, 5,en_iocfg_dir_output, {{.fltclk=0,.invinp=0,.iewo=0,.opendrn=0,.invout=0,.plevel=0,.pen=0,.pwoa=0,.funsel=0,.iodis=0}}}, /* LED */

IOCFG_PINCFG_END
};

/*****************************************************************************/ 
/* End of file                                                               */ 
/*****************************************************************************/

