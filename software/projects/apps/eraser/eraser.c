/***************************************************************************************
 * @file     eraser.c
 * @version  V1.11
 * @date     22 August 2022
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
#include "va416xx.h"
#include "eraser.h"


/*******************************************************************************
 **
 ** @brief  
 **
 ******************************************************************************/
void wait(void)
{
    for(int i=0;i<10000;i++)
    {
        asm("nop");
    }
}

/*******************************************************************************
 **
 ** @brief  
 **
 ******************************************************************************/
void init(void)
{
    // disable watchdog
    VOR_WATCH_DOG->WDOGLOCK    = 0x1ACCE551;
    VOR_WATCH_DOG->WDOGCONTROL = 0x0;

    // enable clocks
    VOR_SYSCONFIG->PERIPHERAL_CLK_ENABLE = BOOT_SPI_CLK_ENABLE | CLK_ENABLE_SYSTEM | CLK_ENABLE_CLKGEN | CLK_ENABLE_IOCONFIG;

    // reset boot spi
    VOR_SYSCONFIG->PERIPHERAL_RESET &= ~BOOT_SPI_RESET;
    asm("nop");
    asm("nop");
    VOR_SYSCONFIG->PERIPHERAL_RESET |= BOOT_SPI_RESET;

    BOOT_SPI->CLKPRESCALE = 2;                                                                      // Set fast clock rate
    BOOT_SPI->CTRL0       = (uint32_t)(3+1)<<SPI_CTRL0_SCRDV_Pos | 0x07;                            // Set clock pol/ph = 00, clock div=03 and word size=7(8bit)
    BOOT_SPI->CTRL1       = SPI_CTRL1_BLOCKMODE_Msk | SPI_CTRL1_ENABLE_Msk | SPI_CTRL1_BMSTALL_Msk; // Enable SPI Master mode, block xfer    
    BOOT_SPI->FIFO_CLR    = SPI_FIFO_CLR_RXFIFO_Msk | SPI_FIFO_CLR_TXFIFO_Msk;                      // Clear TX & RX FIFO

    wait();

    BOOT_SPI->DATA = WREN | SPIC_DATA_BMSTOP;                                                       // Send single "Write enable" byte      
    wait();

    BOOT_SPI->DATA = WREN | SPIC_DATA_BMSTOP;                                                       // Set Write Enable Latch(WEL) bit
    wait();

    BOOT_SPI->DATA = WRSR;                                                                          // Write single-byte Status Register message
    BOOT_SPI->DATA = SPIC_DATA_BMSTOP;                          
    wait();
}

/*******************************************************************************
 **
 ** @brief  
 **
 ******************************************************************************/
void EraseChip(void)
{
    uint32_t volatile voidRead __attribute__((unused));

    BOOT_SPI->DATA = WREN | SPIC_DATA_BMSTOP;   // Send single "Write enable" byte      
    wait();
    
    BOOT_SPI->DATA = WRITE;                     // Write command 
    BOOT_SPI->DATA = 0;                         // Address high byte
    BOOT_SPI->DATA = 0;                         // Address mid byte 
    BOOT_SPI->DATA = 0;                         // Address low byte
                
    for(int i=0; i<SZDEV; i++ )
    {
        while (!( BOOT_SPI->STATUS & SPI_STATUS_TNF_Msk));
        BOOT_SPI->DATA = 0;
    }
        
    while( !( BOOT_SPI->STATUS & SPI_STATUS_TNF_Msk) );
    BOOT_SPI->DATA = SPIC_DATA_BMSTOP;          // Last Write also has End Block Transfer
    wait();
}

/*******************************************************************************
 **
 ** @brief  
 **
 ******************************************************************************/
void __attribute__((naked)) wr_fram(void) 
{
    asm("cpsid if");                            // Disable interrupts.
	asm("ldr sp,=_estack");                     // Set the stack at the end of IRAM1.

    init();
    EraseChip();

    asm("cpsie if");                            // Enable interrupts

done:
    goto done;
}