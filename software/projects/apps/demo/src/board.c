/***************************************************************************************
 * @file     board.c
 * @version  V1.0
 * @date     15 November 2019
 *
 * @note
 * VORAGO Technologies
 *
 * @note
 * Copyright (c) 2013-2019 VORAGO Technologies.
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


// Generated with excel pin configurator
const stc_iocfg_pin_cfg_t ioPinCfgArr[] = 
{
{VOR_PORTA, 0,en_iocfg_dir__input, {{.fltclk=0,.invinp=0,.iewo=0,.opendrn=0,.invout=0,.plevel=0,.pen=0,.pwoa=0,.funsel=0,.iodis=0}}},
{VOR_PORTA, 1,en_iocfg_dir__input, {{.fltclk=0,.invinp=0,.iewo=0,.opendrn=0,.invout=0,.plevel=0,.pen=0,.pwoa=0,.funsel=0,.iodis=0}}},
{VOR_PORTA, 2,en_iocfg_dir__input, {{.fltclk=0,.invinp=0,.iewo=0,.opendrn=0,.invout=0,.plevel=0,.pen=0,.pwoa=0,.funsel=0,.iodis=0}}},
{VOR_PORTA, 3,en_iocfg_dir__input, {{.fltclk=0,.invinp=0,.iewo=0,.opendrn=0,.invout=0,.plevel=0,.pen=0,.pwoa=0,.funsel=0,.iodis=0}}},
{VOR_PORTA, 4,en_iocfg_dir_dncare, {{.fltclk=0,.invinp=0,.iewo=0,.opendrn=0,.invout=0,.plevel=0,.pen=0,.pwoa=0,.funsel=2,.iodis=0}}},
{VOR_PORTA, 5,en_iocfg_dir_dncare, {{.fltclk=0,.invinp=0,.iewo=0,.opendrn=0,.invout=0,.plevel=0,.pen=0,.pwoa=0,.funsel=2,.iodis=0}}},
{VOR_PORTA, 6,en_iocfg_dir_dncare, {{.fltclk=0,.invinp=0,.iewo=0,.opendrn=0,.invout=0,.plevel=0,.pen=0,.pwoa=0,.funsel=2,.iodis=0}}},
{VOR_PORTA, 7,en_iocfg_dir_dncare, {{.fltclk=0,.invinp=0,.iewo=0,.opendrn=0,.invout=0,.plevel=0,.pen=0,.pwoa=0,.funsel=2,.iodis=0}}},
{VOR_PORTA, 8,en_iocfg_dir__input, {{.fltclk=0,.invinp=0,.iewo=0,.opendrn=0,.invout=0,.plevel=0,.pen=0,.pwoa=0,.funsel=0,.iodis=0}}},
{VOR_PORTA, 9,en_iocfg_dir__input, {{.fltclk=0,.invinp=0,.iewo=0,.opendrn=0,.invout=0,.plevel=0,.pen=0,.pwoa=0,.funsel=0,.iodis=0}}},
{VOR_PORTA,10,en_iocfg_dir__input, {{.fltclk=0,.invinp=0,.iewo=0,.opendrn=0,.invout=0,.plevel=0,.pen=0,.pwoa=0,.funsel=0,.iodis=0}}},
{VOR_PORTA,11,en_iocfg_dir__input, {{.fltclk=0,.invinp=0,.iewo=0,.opendrn=0,.invout=0,.plevel=0,.pen=0,.pwoa=0,.funsel=0,.iodis=0}}},
{VOR_PORTA,12,en_iocfg_dir__input, {{.fltclk=0,.invinp=0,.iewo=0,.opendrn=0,.invout=0,.plevel=0,.pen=0,.pwoa=0,.funsel=0,.iodis=0}}},
{VOR_PORTA,13,en_iocfg_dir__input, {{.fltclk=0,.invinp=0,.iewo=0,.opendrn=0,.invout=0,.plevel=0,.pen=0,.pwoa=0,.funsel=0,.iodis=0}}},
{VOR_PORTA,14,en_iocfg_dir__input, {{.fltclk=0,.invinp=0,.iewo=0,.opendrn=0,.invout=0,.plevel=0,.pen=0,.pwoa=0,.funsel=0,.iodis=0}}},
{VOR_PORTA,15,en_iocfg_dir__input, {{.fltclk=0,.invinp=0,.iewo=0,.opendrn=0,.invout=0,.plevel=0,.pen=0,.pwoa=0,.funsel=0,.iodis=0}}},
{VOR_PORTB, 0,en_iocfg_dir__input, {{.fltclk=0,.invinp=0,.iewo=0,.opendrn=0,.invout=0,.plevel=0,.pen=0,.pwoa=0,.funsel=0,.iodis=0}}},
{VOR_PORTB, 1,en_iocfg_dir__input, {{.fltclk=0,.invinp=0,.iewo=0,.opendrn=0,.invout=0,.plevel=0,.pen=0,.pwoa=0,.funsel=0,.iodis=0}}},
{VOR_PORTB, 2,en_iocfg_dir__input, {{.fltclk=0,.invinp=0,.iewo=0,.opendrn=0,.invout=0,.plevel=0,.pen=0,.pwoa=0,.funsel=0,.iodis=0}}},
{VOR_PORTB, 3,en_iocfg_dir__input, {{.fltclk=0,.invinp=0,.iewo=0,.opendrn=0,.invout=0,.plevel=0,.pen=0,.pwoa=0,.funsel=0,.iodis=0}}},
{VOR_PORTB, 4,en_iocfg_dir__input, {{.fltclk=0,.invinp=0,.iewo=0,.opendrn=0,.invout=0,.plevel=0,.pen=0,.pwoa=0,.funsel=0,.iodis=0}}},
{VOR_PORTB, 5,en_iocfg_dir__input, {{.fltclk=0,.invinp=0,.iewo=0,.opendrn=0,.invout=0,.plevel=0,.pen=0,.pwoa=0,.funsel=0,.iodis=0}}},
{VOR_PORTB, 6,en_iocfg_dir__input, {{.fltclk=0,.invinp=0,.iewo=0,.opendrn=0,.invout=0,.plevel=0,.pen=0,.pwoa=0,.funsel=0,.iodis=0}}},
{VOR_PORTB, 7,en_iocfg_dir__input, {{.fltclk=0,.invinp=0,.iewo=0,.opendrn=0,.invout=0,.plevel=0,.pen=0,.pwoa=0,.funsel=0,.iodis=0}}},
{VOR_PORTB, 8,en_iocfg_dir__input, {{.fltclk=0,.invinp=0,.iewo=0,.opendrn=0,.invout=0,.plevel=0,.pen=0,.pwoa=0,.funsel=0,.iodis=0}}},
{VOR_PORTB, 9,en_iocfg_dir__input, {{.fltclk=0,.invinp=0,.iewo=0,.opendrn=0,.invout=0,.plevel=0,.pen=0,.pwoa=0,.funsel=0,.iodis=0}}},
{VOR_PORTB,10,en_iocfg_dir__input, {{.fltclk=0,.invinp=0,.iewo=0,.opendrn=0,.invout=0,.plevel=0,.pen=0,.pwoa=0,.funsel=0,.iodis=0}}},
{VOR_PORTB,11,en_iocfg_dir__input, {{.fltclk=0,.invinp=0,.iewo=0,.opendrn=0,.invout=0,.plevel=0,.pen=0,.pwoa=0,.funsel=0,.iodis=0}}},
{VOR_PORTB,12,en_iocfg_dir_dncare, {{.fltclk=0,.invinp=0,.iewo=0,.opendrn=0,.invout=0,.plevel=0,.pen=0,.pwoa=0,.funsel=3,.iodis=0}}},
{VOR_PORTB,13,en_iocfg_dir_dncare, {{.fltclk=0,.invinp=0,.iewo=0,.opendrn=0,.invout=0,.plevel=0,.pen=0,.pwoa=0,.funsel=3,.iodis=0}}},
{VOR_PORTB,14,en_iocfg_dir_dncare, {{.fltclk=0,.invinp=0,.iewo=0,.opendrn=0,.invout=0,.plevel=0,.pen=0,.pwoa=0,.funsel=3,.iodis=0}}},
{VOR_PORTB,15,en_iocfg_dir_dncare, {{.fltclk=0,.invinp=0,.iewo=0,.opendrn=0,.invout=0,.plevel=0,.pen=0,.pwoa=0,.funsel=3,.iodis=0}}},
{VOR_PORTC, 0,en_iocfg_dir__input, {{.fltclk=0,.invinp=0,.iewo=0,.opendrn=0,.invout=0,.plevel=0,.pen=0,.pwoa=0,.funsel=0,.iodis=0}}},
{VOR_PORTC, 1,en_iocfg_dir__input, {{.fltclk=0,.invinp=0,.iewo=0,.opendrn=0,.invout=0,.plevel=0,.pen=0,.pwoa=0,.funsel=0,.iodis=0}}},
{VOR_PORTC, 2,en_iocfg_dir__input, {{.fltclk=0,.invinp=0,.iewo=0,.opendrn=0,.invout=0,.plevel=0,.pen=0,.pwoa=0,.funsel=0,.iodis=0}}},
{VOR_PORTC, 3,en_iocfg_dir__input, {{.fltclk=0,.invinp=0,.iewo=0,.opendrn=0,.invout=0,.plevel=0,.pen=0,.pwoa=0,.funsel=0,.iodis=0}}},
{VOR_PORTC, 4,en_iocfg_dir__input, {{.fltclk=0,.invinp=0,.iewo=0,.opendrn=0,.invout=0,.plevel=0,.pen=0,.pwoa=0,.funsel=0,.iodis=0}}},
{VOR_PORTC, 5,en_iocfg_dir__input, {{.fltclk=0,.invinp=0,.iewo=0,.opendrn=0,.invout=0,.plevel=0,.pen=0,.pwoa=0,.funsel=0,.iodis=0}}},
{VOR_PORTC, 6,en_iocfg_dir__input, {{.fltclk=0,.invinp=0,.iewo=0,.opendrn=0,.invout=0,.plevel=0,.pen=0,.pwoa=0,.funsel=0,.iodis=0}}},
{VOR_PORTC, 7,en_iocfg_dir__input, {{.fltclk=0,.invinp=0,.iewo=0,.opendrn=0,.invout=0,.plevel=0,.pen=0,.pwoa=0,.funsel=0,.iodis=0}}},
{VOR_PORTC, 8,en_iocfg_dir__input, {{.fltclk=0,.invinp=0,.iewo=0,.opendrn=0,.invout=0,.plevel=0,.pen=0,.pwoa=0,.funsel=0,.iodis=0}}},
{VOR_PORTC, 9,en_iocfg_dir__input, {{.fltclk=0,.invinp=0,.iewo=0,.opendrn=0,.invout=0,.plevel=0,.pen=0,.pwoa=0,.funsel=0,.iodis=0}}},
{VOR_PORTC,10,en_iocfg_dir__input, {{.fltclk=0,.invinp=0,.iewo=0,.opendrn=0,.invout=0,.plevel=0,.pen=0,.pwoa=0,.funsel=0,.iodis=0}}},
{VOR_PORTC,11,en_iocfg_dir__input, {{.fltclk=0,.invinp=0,.iewo=0,.opendrn=0,.invout=0,.plevel=0,.pen=0,.pwoa=0,.funsel=0,.iodis=0}}},
{VOR_PORTC,12,en_iocfg_dir__input, {{.fltclk=0,.invinp=0,.iewo=0,.opendrn=0,.invout=0,.plevel=0,.pen=0,.pwoa=0,.funsel=0,.iodis=0}}},
{VOR_PORTC,13,en_iocfg_dir__input, {{.fltclk=0,.invinp=0,.iewo=0,.opendrn=0,.invout=0,.plevel=0,.pen=0,.pwoa=0,.funsel=0,.iodis=0}}},
{VOR_PORTC,14,en_iocfg_dir__input, {{.fltclk=0,.invinp=0,.iewo=0,.opendrn=0,.invout=0,.plevel=0,.pen=0,.pwoa=0,.funsel=0,.iodis=0}}},
{VOR_PORTC,15,en_iocfg_dir__input, {{.fltclk=0,.invinp=0,.iewo=0,.opendrn=0,.invout=0,.plevel=0,.pen=0,.pwoa=0,.funsel=0,.iodis=0}}},
{VOR_PORTD, 0,en_iocfg_dir__input, {{.fltclk=0,.invinp=0,.iewo=0,.opendrn=0,.invout=0,.plevel=0,.pen=0,.pwoa=0,.funsel=0,.iodis=0}}},
{VOR_PORTD, 1,en_iocfg_dir__input, {{.fltclk=0,.invinp=0,.iewo=0,.opendrn=0,.invout=0,.plevel=0,.pen=0,.pwoa=0,.funsel=0,.iodis=0}}},
{VOR_PORTD, 2,en_iocfg_dir__input, {{.fltclk=0,.invinp=0,.iewo=0,.opendrn=0,.invout=0,.plevel=0,.pen=0,.pwoa=0,.funsel=0,.iodis=0}}},
{VOR_PORTD, 3,en_iocfg_dir__input, {{.fltclk=0,.invinp=0,.iewo=0,.opendrn=0,.invout=0,.plevel=0,.pen=0,.pwoa=0,.funsel=0,.iodis=0}}},
{VOR_PORTD, 4,en_iocfg_dir__input, {{.fltclk=0,.invinp=0,.iewo=0,.opendrn=0,.invout=0,.plevel=0,.pen=0,.pwoa=0,.funsel=0,.iodis=0}}},
{VOR_PORTD, 5,en_iocfg_dir__input, {{.fltclk=0,.invinp=0,.iewo=0,.opendrn=0,.invout=0,.plevel=0,.pen=0,.pwoa=0,.funsel=0,.iodis=0}}},
{VOR_PORTD, 6,en_iocfg_dir__input, {{.fltclk=0,.invinp=0,.iewo=0,.opendrn=0,.invout=0,.plevel=0,.pen=0,.pwoa=0,.funsel=0,.iodis=0}}},
{VOR_PORTD, 7,en_iocfg_dir__input, {{.fltclk=0,.invinp=0,.iewo=0,.opendrn=0,.invout=0,.plevel=0,.pen=0,.pwoa=0,.funsel=0,.iodis=0}}},
{VOR_PORTD, 8,en_iocfg_dir__input, {{.fltclk=0,.invinp=0,.iewo=0,.opendrn=0,.invout=0,.plevel=0,.pen=0,.pwoa=0,.funsel=0,.iodis=0}}},
{VOR_PORTD, 9,en_iocfg_dir__input, {{.fltclk=0,.invinp=0,.iewo=0,.opendrn=0,.invout=0,.plevel=0,.pen=0,.pwoa=0,.funsel=0,.iodis=0}}},
{VOR_PORTD,10,en_iocfg_dir__input, {{.fltclk=0,.invinp=0,.iewo=0,.opendrn=0,.invout=0,.plevel=0,.pen=0,.pwoa=0,.funsel=0,.iodis=0}}},
{VOR_PORTD,11,en_iocfg_dir__input, {{.fltclk=0,.invinp=0,.iewo=0,.opendrn=0,.invout=0,.plevel=0,.pen=0,.pwoa=0,.funsel=0,.iodis=0}}},
{VOR_PORTD,12,en_iocfg_dir__input, {{.fltclk=0,.invinp=0,.iewo=0,.opendrn=0,.invout=0,.plevel=0,.pen=0,.pwoa=0,.funsel=0,.iodis=0}}},
{VOR_PORTD,13,en_iocfg_dir__input, {{.fltclk=0,.invinp=0,.iewo=0,.opendrn=0,.invout=0,.plevel=0,.pen=0,.pwoa=0,.funsel=0,.iodis=0}}},
{VOR_PORTD,14,en_iocfg_dir__input, {{.fltclk=0,.invinp=0,.iewo=0,.opendrn=0,.invout=0,.plevel=0,.pen=0,.pwoa=0,.funsel=0,.iodis=0}}},
{VOR_PORTD,15,en_iocfg_dir__input, {{.fltclk=0,.invinp=0,.iewo=0,.opendrn=0,.invout=0,.plevel=0,.pen=0,.pwoa=0,.funsel=0,.iodis=0}}},
{VOR_PORTE, 0,en_iocfg_dir__input, {{.fltclk=0,.invinp=0,.iewo=0,.opendrn=0,.invout=0,.plevel=0,.pen=0,.pwoa=0,.funsel=0,.iodis=0}}},
{VOR_PORTE, 1,en_iocfg_dir__input, {{.fltclk=0,.invinp=0,.iewo=0,.opendrn=0,.invout=0,.plevel=0,.pen=0,.pwoa=0,.funsel=0,.iodis=0}}},
{VOR_PORTE, 2,en_iocfg_dir__input, {{.fltclk=0,.invinp=0,.iewo=0,.opendrn=0,.invout=0,.plevel=0,.pen=0,.pwoa=0,.funsel=0,.iodis=0}}},
{VOR_PORTE, 3,en_iocfg_dir__input, {{.fltclk=0,.invinp=0,.iewo=0,.opendrn=0,.invout=0,.plevel=0,.pen=0,.pwoa=0,.funsel=0,.iodis=0}}},
{VOR_PORTE, 4,en_iocfg_dir__input, {{.fltclk=0,.invinp=0,.iewo=0,.opendrn=0,.invout=0,.plevel=0,.pen=0,.pwoa=0,.funsel=0,.iodis=0}}},
{VOR_PORTE, 5,en_iocfg_dir__input, {{.fltclk=0,.invinp=0,.iewo=0,.opendrn=0,.invout=0,.plevel=0,.pen=0,.pwoa=0,.funsel=0,.iodis=0}}},
{VOR_PORTE, 6,en_iocfg_dir__input, {{.fltclk=0,.invinp=0,.iewo=0,.opendrn=0,.invout=0,.plevel=0,.pen=0,.pwoa=0,.funsel=0,.iodis=0}}},
{VOR_PORTE, 7,en_iocfg_dir__input, {{.fltclk=0,.invinp=0,.iewo=0,.opendrn=0,.invout=0,.plevel=0,.pen=0,.pwoa=0,.funsel=0,.iodis=0}}},
{VOR_PORTE, 8,en_iocfg_dir__input, {{.fltclk=0,.invinp=0,.iewo=0,.opendrn=0,.invout=0,.plevel=0,.pen=0,.pwoa=0,.funsel=0,.iodis=0}}},
{VOR_PORTE, 9,en_iocfg_dir__input, {{.fltclk=0,.invinp=0,.iewo=0,.opendrn=0,.invout=0,.plevel=0,.pen=0,.pwoa=0,.funsel=0,.iodis=0}}},
{VOR_PORTE,10,en_iocfg_dir__input, {{.fltclk=0,.invinp=0,.iewo=0,.opendrn=0,.invout=0,.plevel=0,.pen=0,.pwoa=0,.funsel=0,.iodis=0}}},
{VOR_PORTE,11,en_iocfg_dir__input, {{.fltclk=0,.invinp=0,.iewo=0,.opendrn=0,.invout=0,.plevel=0,.pen=0,.pwoa=0,.funsel=0,.iodis=0}}},
{VOR_PORTE,12,en_iocfg_dir__input, {{.fltclk=0,.invinp=0,.iewo=0,.opendrn=0,.invout=0,.plevel=0,.pen=0,.pwoa=0,.funsel=0,.iodis=0}}},
{VOR_PORTE,13,en_iocfg_dir__input, {{.fltclk=0,.invinp=0,.iewo=0,.opendrn=0,.invout=0,.plevel=0,.pen=0,.pwoa=0,.funsel=0,.iodis=0}}},
{VOR_PORTE,14,en_iocfg_dir__input, {{.fltclk=0,.invinp=0,.iewo=0,.opendrn=0,.invout=0,.plevel=0,.pen=0,.pwoa=0,.funsel=0,.iodis=0}}},
{VOR_PORTE,15,en_iocfg_dir__input, {{.fltclk=0,.invinp=0,.iewo=0,.opendrn=0,.invout=0,.plevel=0,.pen=0,.pwoa=0,.funsel=0,.iodis=0}}},
{VOR_PORTF, 0,en_iocfg_dir__input, {{.fltclk=0,.invinp=0,.iewo=0,.opendrn=0,.invout=0,.plevel=0,.pen=0,.pwoa=0,.funsel=0,.iodis=0}}},
{VOR_PORTF, 1,en_iocfg_dir__input, {{.fltclk=0,.invinp=0,.iewo=0,.opendrn=0,.invout=0,.plevel=0,.pen=0,.pwoa=0,.funsel=0,.iodis=0}}},
{VOR_PORTF, 2,en_iocfg_dir_dncare, {{.fltclk=0,.invinp=0,.iewo=0,.opendrn=0,.invout=0,.plevel=0,.pen=0,.pwoa=0,.funsel=1,.iodis=0}}},
{VOR_PORTF, 3,en_iocfg_dir_dncare, {{.fltclk=0,.invinp=0,.iewo=0,.opendrn=0,.invout=0,.plevel=0,.pen=0,.pwoa=0,.funsel=1,.iodis=0}}},
{VOR_PORTF, 4,en_iocfg_dir_dncare, {{.fltclk=0,.invinp=0,.iewo=0,.opendrn=0,.invout=0,.plevel=0,.pen=0,.pwoa=0,.funsel=1,.iodis=0}}},
{VOR_PORTF, 5,en_iocfg_dir_dncare, {{.fltclk=0,.invinp=0,.iewo=0,.opendrn=0,.invout=0,.plevel=0,.pen=0,.pwoa=0,.funsel=1,.iodis=0}}},
{VOR_PORTF, 6,en_iocfg_dir_dncare, {{.fltclk=0,.invinp=0,.iewo=0,.opendrn=0,.invout=0,.plevel=0,.pen=0,.pwoa=0,.funsel=1,.iodis=0}}},
{VOR_PORTF, 7,en_iocfg_dir_dncare, {{.fltclk=0,.invinp=0,.iewo=0,.opendrn=0,.invout=0,.plevel=0,.pen=0,.pwoa=0,.funsel=1,.iodis=0}}},
{VOR_PORTF, 8,en_iocfg_dir_dncare, {{.fltclk=0,.invinp=0,.iewo=0,.opendrn=0,.invout=0,.plevel=0,.pen=0,.pwoa=0,.funsel=1,.iodis=0}}},
{VOR_PORTF, 9,en_iocfg_dir_dncare, {{.fltclk=0,.invinp=0,.iewo=0,.opendrn=0,.invout=0,.plevel=0,.pen=0,.pwoa=0,.funsel=1,.iodis=0}}},
{VOR_PORTF,10,en_iocfg_dir__input, {{.fltclk=0,.invinp=0,.iewo=0,.opendrn=0,.invout=0,.plevel=0,.pen=0,.pwoa=0,.funsel=0,.iodis=0}}},
{VOR_PORTF,11,en_iocfg_dir__input, {{.fltclk=0,.invinp=0,.iewo=0,.opendrn=0,.invout=0,.plevel=0,.pen=0,.pwoa=0,.funsel=0,.iodis=0}}},
{VOR_PORTF,12,en_iocfg_dir__input, {{.fltclk=0,.invinp=0,.iewo=0,.opendrn=0,.invout=0,.plevel=0,.pen=0,.pwoa=0,.funsel=0,.iodis=0}}},
{VOR_PORTF,13,en_iocfg_dir__input, {{.fltclk=0,.invinp=0,.iewo=0,.opendrn=0,.invout=0,.plevel=0,.pen=0,.pwoa=0,.funsel=0,.iodis=0}}},
{VOR_PORTF,14,en_iocfg_dir__input, {{.fltclk=0,.invinp=0,.iewo=0,.opendrn=0,.invout=0,.plevel=0,.pen=0,.pwoa=0,.funsel=0,.iodis=0}}},
{VOR_PORTF,15,en_iocfg_dir__input, {{.fltclk=0,.invinp=0,.iewo=0,.opendrn=0,.invout=0,.plevel=0,.pen=0,.pwoa=0,.funsel=0,.iodis=0}}},
{VOR_PORTG, 0,en_iocfg_dir_dncare, {{.fltclk=0,.invinp=0,.iewo=0,.opendrn=0,.invout=0,.plevel=0,.pen=0,.pwoa=0,.funsel=1,.iodis=0}}},
{VOR_PORTG, 1,en_iocfg_dir_dncare, {{.fltclk=0,.invinp=0,.iewo=0,.opendrn=0,.invout=0,.plevel=0,.pen=0,.pwoa=0,.funsel=1,.iodis=0}}},
{VOR_PORTG, 2,en_iocfg_dir__input, {{.fltclk=0,.invinp=0,.iewo=0,.opendrn=0,.invout=0,.plevel=0,.pen=0,.pwoa=0,.funsel=0,.iodis=0}}},
{VOR_PORTG, 3,en_iocfg_dir__input, {{.fltclk=0,.invinp=0,.iewo=0,.opendrn=0,.invout=0,.plevel=0,.pen=0,.pwoa=0,.funsel=0,.iodis=0}}},
{VOR_PORTG, 4,en_iocfg_dir_dncare, {{.fltclk=0,.invinp=0,.iewo=0,.opendrn=0,.invout=0,.plevel=0,.pen=0,.pwoa=0,.funsel=1,.iodis=0}}},
{VOR_PORTG, 5,en_iocfg_dir_output, {{.fltclk=0,.invinp=0,.iewo=0,.opendrn=0,.invout=0,.plevel=0,.pen=0,.pwoa=0,.funsel=0,.iodis=0}}},
{VOR_PORTG, 6,en_iocfg_dir__input, {{.fltclk=0,.invinp=0,.iewo=0,.opendrn=0,.invout=0,.plevel=0,.pen=0,.pwoa=0,.funsel=0,.iodis=0}}},
{VOR_PORTG, 7,en_iocfg_dir__input, {{.fltclk=0,.invinp=0,.iewo=0,.opendrn=0,.invout=0,.plevel=0,.pen=0,.pwoa=0,.funsel=0,.iodis=0}}},


IOCFG_PINCFG_END
};

/*****************************************************************************/ 
/* End of file                                                               */ 
/*****************************************************************************/

