/***************************************************************************************
 * @file     loader.c
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

/*****************************************************************************/ 
/* Include files                                                             */ 
/*****************************************************************************/
#include "va416xx.h"
#include "loader.h"


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
    NVIC_ClearPendingIRQ(WATCHDOG_IRQn);

    // disable SysTick
    SysTick->VAL = 0x0;
    NVIC_ClearPendingIRQ(SysTick_IRQn);

    // enable clocks
    VOR_SYSCONFIG->PERIPHERAL_CLK_ENABLE = CLK_ENABLE_SYSTEM | CLK_ENABLE_CLKGEN | CLK_ENABLE_IOCONFIG;

#ifdef BOOT_FROM_EBI
    VOR_SYSCONFIG->PERIPHERAL_CLK_ENABLE |= (CLK_ENABLE_EBI | CLK_ENABLE_PORTC | CLK_ENABLE_PORTD | CLK_ENABLE_PORTE | CLK_ENABLE_PORTF);
    VOR_SYSCONFIG->PERIPHERAL_RESET &= ~SYSCONFIG_PERIPHERAL_RESET_EBI_Msk;
    __NOP();
    __NOP();
    VOR_SYSCONFIG->PERIPHERAL_RESET |= SYSCONFIG_PERIPHERAL_RESET_EBI_Msk;

    //C2-15
    //D0-15
    //E0-15
    //F0-1
    // the above pins set to funsel 1
    for(int i = 0; i<16; i++)
    {   
        if(i > 1) VOR_IOCONFIG->PORTC[i] = 1 << IOCONFIG_PORTC_FUNSEL_Pos;
        VOR_IOCONFIG->PORTD[i] = 1 << IOCONFIG_PORTD_FUNSEL_Pos;
        VOR_IOCONFIG->PORTE[i] = 1 << IOCONFIG_PORTE_FUNSEL_Pos;
    }
    VOR_IOCONFIG->PORTF[0] = 1 << IOCONFIG_PORTF_FUNSEL_Pos;
    VOR_IOCONFIG->PORTF[1] = 1 << IOCONFIG_PORTF_FUNSEL_Pos;

    //512k Byte FRAM on CE0
    //addr[23:16] 0x0 to 0x7
    //0x6000_0000 -> 0x6007_ffff as Data Space using System Bus
    //0x1000_0000 -> 0x1007_ffff as Instruction Space using IBus for inst fetch, Data Bus for data
    // F-ram needs to use slowest settings only (100ns access time)
    VOR_SYSCONFIG->EBI_CFG0=(
        (0x00<<SYSCONFIG_EBI_CFG0_ADDRLOW0_Pos)
        |(0x07<<SYSCONFIG_EBI_CFG0_ADDRHIGH0_Pos)
        |(6<<SYSCONFIG_EBI_CFG0_CFGREADCYCLE_Pos)
        |(6<<SYSCONFIG_EBI_CFG0_CFGWRITECYCLE_Pos)
        |(6<<SYSCONFIG_EBI_CFG0_CFGTURNAROUNDCYCLE_Pos)
        |(SYSCONFIG_EBI_CFG0_CFGSIZE_Msk) //16 bit mode
    );

    //512k Byte SRAM on CE1
    //addr[23:16] 0x8 to 0xf
    //0x6008_0000 -> 0x600f_ffff as Data Space using System Bus
    //0x1008_0000 -> 0x100f_ffff as Instruction Space using IBus for inst fetch, Data Bus for data
    //fastest allowable read timing: 0x3
    VOR_SYSCONFIG->EBI_CFG1=(
        (0x08<<SYSCONFIG_EBI_CFG0_ADDRLOW0_Pos)
        |(0x0f<<SYSCONFIG_EBI_CFG0_ADDRHIGH0_Pos)
        |(3<<SYSCONFIG_EBI_CFG0_CFGREADCYCLE_Pos)
        |(0<<SYSCONFIG_EBI_CFG0_CFGWRITECYCLE_Pos)
        |(0<<SYSCONFIG_EBI_CFG0_CFGTURNAROUNDCYCLE_Pos)
        |(SYSCONFIG_EBI_CFG0_CFGSIZE_Msk) //16 bit mode
    );

    //not used, cannot disable, so map end of range
    VOR_SYSCONFIG->EBI_CFG2=(
        (0xfd<<SYSCONFIG_EBI_CFG0_ADDRLOW0_Pos)
        |(0xfd<<SYSCONFIG_EBI_CFG0_ADDRHIGH0_Pos)
        |(6<<SYSCONFIG_EBI_CFG0_CFGREADCYCLE_Pos)
        |(6<<SYSCONFIG_EBI_CFG0_CFGWRITECYCLE_Pos)
        |(6<<SYSCONFIG_EBI_CFG0_CFGTURNAROUNDCYCLE_Pos)
        |(SYSCONFIG_EBI_CFG0_CFGSIZE_Msk)  //16 bit mode
    );

    //not used, cannot disable, so map end of range
    VOR_SYSCONFIG->EBI_CFG3=(
        (0xff<<SYSCONFIG_EBI_CFG0_ADDRLOW0_Pos)
        |(0xff<<SYSCONFIG_EBI_CFG0_ADDRHIGH0_Pos)
        |(6<<SYSCONFIG_EBI_CFG0_CFGREADCYCLE_Pos)
        |(6<<SYSCONFIG_EBI_CFG0_CFGWRITECYCLE_Pos)
        |(6<<SYSCONFIG_EBI_CFG0_CFGTURNAROUNDCYCLE_Pos)
        |(SYSCONFIG_EBI_CFG0_CFGSIZE_Msk)  //16 bit mode
    );
#else
    // reset boot spi
    VOR_SYSCONFIG->PERIPHERAL_CLK_ENABLE |= BOOT_SPI_CLK_ENABLE;
    VOR_SYSCONFIG->PERIPHERAL_RESET &= ~BOOT_SPI_RESET;
    asm("nop");
    asm("nop");
    VOR_SYSCONFIG->PERIPHERAL_RESET |= BOOT_SPI_RESET;

    BOOT_SPI->CLKPRESCALE = 2;                                                                      // Set fast clock rate
    BOOT_SPI->CTRL0       = (uint32_t)(3+1)<<SPI_CTRL0_SCRDV_Pos | 0x07;                            // Set clock pol/ph = 00, clock div=03 and word size=7(8bit)
    BOOT_SPI->CTRL1       = SPI_CTRL1_BLOCKMODE_Msk | SPI_CTRL1_ENABLE_Msk | SPI_CTRL1_BMSTALL_Msk; // Enable SPI Master mode, block xfer    
    BOOT_SPI->FIFO_CLR    = SPI_FIFO_CLR_RXFIFO_Msk | SPI_FIFO_CLR_TXFIFO_Msk;                      // Clear TX & RX FIFO
    wait();
    BOOT_SPI->DATA = WREN | SPI_DATA_BMSTART_BMSTOP_Msk;                                            // Send single "Write enable" byte      
    wait();
    BOOT_SPI->DATA = WREN | SPI_DATA_BMSTART_BMSTOP_Msk;                                            // Send single "Write enable" byte
    wait();
    BOOT_SPI->DATA = WRSR;                                                                          // Write single-byte Status Register message
    BOOT_SPI->DATA = SPI_DATA_BMSTART_BMSTOP_Msk;                          
    wait();
#endif
}

/*******************************************************************************
 **
 ** @brief  
 **
 ******************************************************************************/
void uninit(void)
{
#ifdef BOOT_FROM_EBI
    for(int i = 0; i<16; i++)
    {   
        if(i > 1) VOR_IOCONFIG->PORTC[i] = 0 << IOCONFIG_PORTC_FUNSEL_Pos;
        VOR_IOCONFIG->PORTD[i] = 0 << IOCONFIG_PORTD_FUNSEL_Pos;
        VOR_IOCONFIG->PORTE[i] = 0 << IOCONFIG_PORTE_FUNSEL_Pos;
    }
    VOR_IOCONFIG->PORTF[0] = 0 << IOCONFIG_PORTF_FUNSEL_Pos;
    VOR_IOCONFIG->PORTF[1] = 0 << IOCONFIG_PORTF_FUNSEL_Pos;

    VOR_SYSCONFIG->PERIPHERAL_RESET &= ~SYSCONFIG_PERIPHERAL_RESET_EBI_Msk;
    __NOP();
    __NOP();
    VOR_SYSCONFIG->PERIPHERAL_RESET |= SYSCONFIG_PERIPHERAL_RESET_EBI_Msk;
    VOR_SYSCONFIG->PERIPHERAL_CLK_ENABLE &= ~(CLK_ENABLE_EBI | CLK_ENABLE_PORTC | CLK_ENABLE_PORTD | CLK_ENABLE_PORTE | CLK_ENABLE_PORTF);
#else
    VOR_SYSCONFIG->PERIPHERAL_RESET &= ~BOOT_SPI_RESET;
    asm("nop");
    asm("nop");
    VOR_SYSCONFIG->PERIPHERAL_RESET |= BOOT_SPI_RESET;
    VOR_SYSCONFIG->PERIPHERAL_CLK_ENABLE &= ~BOOT_SPI_CLK_ENABLE;
#endif
}

/*******************************************************************************
 **
 ** @brief  
 **
 ******************************************************************************/
void WriteChip(void)
{
#ifdef BOOT_FROM_EBI
    uint32_t* ebi_addr = (uint32_t*)BOOT_EBI_START_ADDR;
    uint32_t* rom_addr = (uint32_t*)0;
    for(int i=0; i<(SZDEV/4); i++ )
    {
        *ebi_addr++ = *rom_addr++;
    }
#else
    uint32_t volatile voidRead __attribute__((unused));
    uint32_t addr = 0;

    BOOT_SPI->DATA = WREN | SPI_DATA_BMSTART_BMSTOP_Msk;   // Send single "Write enable" byte      
    wait();
    
    BOOT_SPI->DATA = WRITE;                     // Write command 
    BOOT_SPI->DATA = 0;                         // Address high byte
    BOOT_SPI->DATA = 0;                         // Address mid byte 
    BOOT_SPI->DATA = 0;                         // Address low byte
                
    for(int i=0; i<SZDEV; i++ )
    {
        while (!( BOOT_SPI->STATUS & SPI_STATUS_TNF_Msk));
        BOOT_SPI->DATA = *((uint8_t *)(addr++));
    }
        
    while( !( BOOT_SPI->STATUS & SPI_STATUS_TNF_Msk) );
    BOOT_SPI->DATA = SPI_DATA_BMSTART_BMSTOP_Msk | SPI0_DATA_BMSKIPDATA_Msk; // Last Write Is End Block Transfer
    wait();
#endif
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
    WriteChip();
    uninit();

    asm("cpsie if");                            // Enable interrupts
    asm("mov r1, #0");
    asm("ldr r0,[r1,#4]");                      // Load reset vector
    asm("bx r0");                               // Branch to reset handler
}

/*****************************************************************************/ 
/* End of file                                                               */ 
/*****************************************************************************/
