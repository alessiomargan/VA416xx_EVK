/* Copyright (c) 2010 - 2015 ARM LIMITED

   All rights reserved.
   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:
   - Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.
   - Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in the
     documentation and/or other materials provided with the distribution.
   - Neither the name of ARM nor the names of its contributors may be used
     to endorse or promote products derived from this software without
     specific prior written permission.
   *
   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
   AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
   IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
   ARE DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS AND CONTRIBUTORS BE
   LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
   CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
   SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
   INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
   CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
   ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
   POSSIBILITY OF SUCH DAMAGE.
   ---------------------------------------------------------------------------*/
#include "va416xx.h"
#include "flash_os.h"    

#define FLASH_BASE (0x60000000)
#define FLASH_ADDR_LOW_23_16 (0x00)
#define FLASH_ADDR_HIGH_23_16 (0xff)


/* 
	Mandatory Flash Programming Functions (Called by FlashOS):
		int Init();					// Initialize Flash 
    int UnInit();  			// De-initialize Flash
    int EraseSector();  // Erase Sector Function
    int ProgramPage();  // Program Page Function
  Optional  Flash Programming Functions (Called by FlashOS):
    int BlankCheck(); 	// Blank Check
    int EraseChip();    // Erase complete Device
    unsigned long Verify(); // Verify Function
  - BlankCheck is necessary if Flash space is not mapped into CPU memory space
  - Verify is necessary if Flash space is not mapped into CPU memory space
  - if EraseChip is not provided than EraseSector for all sectors is called
*/

void VOR_GPIOC_PinMux(uint32_t pin, uint32_t funsel) {
		VOR_IOCONFIG->PORTC[pin] = (VOR_IOCONFIG->PORTC[pin] & ~((0x3)<<IOCONFIG_PORTC_FUNSEL_Pos))	| ((funsel)<<IOCONFIG_PORTC_FUNSEL_Pos);
}
void VOR_GPIOD_PinMux(uint32_t pin, uint32_t funsel) {
		VOR_IOCONFIG->PORTD[pin] = (VOR_IOCONFIG->PORTD[pin] & ~((0x3)<<IOCONFIG_PORTD_FUNSEL_Pos))	| ((funsel)<<IOCONFIG_PORTD_FUNSEL_Pos);
} 
void VOR_GPIOE_PinMux(uint32_t pin, uint32_t funsel) {
		VOR_IOCONFIG->PORTE[pin] = (VOR_IOCONFIG->PORTE[pin] & ~((0x3)<<IOCONFIG_PORTE_FUNSEL_Pos))	| ((funsel)<<IOCONFIG_PORTE_FUNSEL_Pos);
}
void VOR_GPIOF_PinMux(uint32_t pin, uint32_t funsel) {
		VOR_IOCONFIG->PORTF[pin] = (VOR_IOCONFIG->PORTF[pin] & ~((0x3)<<IOCONFIG_PORTF_FUNSEL_Pos))	| ((funsel)<<IOCONFIG_PORTF_FUNSEL_Pos);
} 

void initEBI_funsel(){
  uint32_t i;
  for(i=2;i<=15;i++){
     VOR_GPIOC_PinMux(i, 1); //addr
  }
  for(i=0;i<=15;i++){
     VOR_GPIOD_PinMux(i, 1);//addr & data
  } 
  for(i=0;i<=9;i++){
     VOR_GPIOE_PinMux(i, 1);//data
  } 
  VOR_GPIOE_PinMux(12, 1); //ce0
  VOR_GPIOE_PinMux(13, 1); //ce1
  VOR_GPIOE_PinMux(14, 1); //ce2
  VOR_GPIOE_PinMux(15, 1); //ce3
  
  VOR_GPIOF_PinMux(0, 1); //oen
  VOR_GPIOF_PinMux(1, 1); //wen
}

int Init(unsigned long adr, unsigned long clk, unsigned long fnc) 
{ 
  VOR_SYSCONFIG->PERIPHERAL_CLK_ENABLE = CLK_ENABLE_EBI | CLK_ENABLE_SYSTEM | CLK_ENABLE_IRQ | CLK_ENABLE_IOCONFIG |
                                         CLK_ENABLE_PORTC | CLK_ENABLE_PORTD | CLK_ENABLE_PORTE | CLK_ENABLE_PORTF;
  //VOR_SYSCONFIG->PERIPHERAL_CLK_ENABLE = 0x7FFFFFFF;  // Enable clocks to all peripherals 
  VOR_SYSCONFIG->PERIPHERAL_RESET |= SYSCONFIG_PERIPHERAL_RESET_EBI_Msk;
  
  initEBI_funsel();
  
  //Claim it all with CS_N0
  //go slow
  VOR_SYSCONFIG->EBI_CFG0=(
     (FLASH_ADDR_LOW_23_16<<SYSCONFIG_EBI_CFG0_ADDRLOW0_Pos) 
    |(FLASH_ADDR_HIGH_23_16<<SYSCONFIG_EBI_CFG0_ADDRHIGH0_Pos)
    |(6<<SYSCONFIG_EBI_CFG0_CFGREADCYCLE_Pos)
    |(6<<SYSCONFIG_EBI_CFG0_CFGWRITECYCLE_Pos)  
    |(6<<SYSCONFIG_EBI_CFG0_CFGTURNAROUNDCYCLE_Pos)
    |(SYSCONFIG_EBI_CFG0_CFGSIZE_Msk)    //16bit
  );
  
  // clock to HBO, to not run EBI too fast
  //NVIC_DisableIRQ(LoCLK_IRQn); // disable IRQ on loss of clock
  //VOR_CLKGEN->CTRL0 = 0x30; // default (HBO)
  //VOR_CLKGEN->CTRL1 = 0x18; // default
  
	return (0); 
}

int UnInit (unsigned long fnc) 
{
  return (0);                                  
}                               

int EraseChip(void)
{
  uint16_t *writePtr=(uint16_t *)(FLASH_BASE);
  uint32_t i;
  for(i=0;i<(SZDEV/2);i++){
    *writePtr++=0;
  }
  
  return(0);
}

int EraseSector (unsigned long adr) 
{
  return (0);                                  
}

int ProgramPage(unsigned long adr, unsigned long sz, unsigned char *buf) {
  uint32_t i = 0;  
	uint16_t *writePtr=(uint16_t *)(FLASH_BASE+adr);
  uint16_t *buf16Ptr=(uint16_t *)buf;

  //Doing it as 16 bit, this is a 16 bit EBI
  if(sz % 2){ sz++; } // make sz even, round up
  sz /= 2; // number of bytes -> number of 16-bit words
  
  for(i=0;i<sz;i++){
    *writePtr++=*buf16Ptr++;
  }
	
  return(0);
}

int BlankCheck(unsigned long adr, unsigned long sz, unsigned char pat)
{
  return (0);       
}

unsigned long Verify(unsigned long adr, unsigned long sz, unsigned char *buf)
{
	return(adr+sz);
}

	
