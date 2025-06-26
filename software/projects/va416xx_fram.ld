/***************************************************************************************
 * @file     va416xx.ld
 * @version  V1.12
 * @date     18 August 2022
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
/* Entry Point */
ENTRY(wr_fram)

/* Highest address of the user mode stack */
_estack = ORIGIN(RAM) + LENGTH(RAM);	/* end of IRAM1 */

_Min_Stack_Size = 0x00000400; /* required amount of stack */ 

/* Memory areas */
MEMORY
{
  FLASH  (rx)    :ORIGIN = 0x00000000, LENGTH = 256K
  RAM    (xrw)   :ORIGIN = 0x1FFF8000, LENGTH = 32K
  CCMRAM (rw)    :ORIGIN = 0x20000000, LENGTH = 32K
}

/* Define output sections */
SECTIONS
{
    /* The program code and all data goes into RAM */
  .text :
  {
    . = ALIGN(4);
    *(.text)           /* .text  sections   (code)                     */
    *(.text*)          /* .text* sections   (code)                     */
    *(.rodata)         /* .rodata sections  (constants, strings, etc.) */
    *(.rodata*)        /* .rodata* sections (constants, strings, etc.) */
    *(.data)           /* .data sections                               */
    *(.data*)          /* .data* sections                              */
    . = ALIGN(4);
    _etext = .;        /* define a global symbols at end of code       */
  } >RAM
}
