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

#define SPIC_DATA_BMSTOP_Pos	31                                    /*!< SPIC DATA: BMSTOP Position */
#define SPIC_DATA_BMSTOP_Msk	(0x01UL << SPIC_DATA_BMSTOP_Pos)      /*!< SPIC DATA: BMSTOP Mask     */

#define RDSR_WIP_Pos	0                                    
#define RDSR_WIP_Msk	(0x01UL << RDSR_WIP_Pos)         

/* Commands */ 
#define WREN		0x06
#define WRDI		0x04
#define RDSR		0x05
#define WRSR		0x01
#define READ		0x03
#define WRITE	  0x02
#define RDID		0x9F 
#define TTZ2564_SE			0x20 
#define TTZ2564_BE			0x52
#define TTZ2564_CE			0x60 
		    
		
/* Address Masks */ 
#define ADDR_MSB_MASK   (uint32_t)0xFF0000
#define ADDR_MID_MASK   (uint32_t)0x00FF00
#define ADDR_LSB_MASK   (uint32_t)0x0000FF
#define MSB_ADDR_BYTE(addr)   ((uint8_t)((addr & ADDR_MSB_MASK)>>16))
#define MID_ADDR_BYTE(addr)   ((uint8_t)((addr & ADDR_MID_MASK)>>8))
#define LSB_ADDR_BYTE(addr)   ((uint8_t)(addr & ADDR_LSB_MASK))

/* SPI bank */
/* VA416xx has boot NVM on SPI[3] */
#define SPI_BANK (3)

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
void wait_idle(void)
{
	while( !(VOR_SPI->BANK[SPI_BANK].STATUS & SPI_STATUS_TFE_Msk) ) { };	// Wait until TxBuf sends all		
  while( VOR_SPI->BANK[SPI_BANK].STATUS & SPI_STATUS_BUSY_Msk ) { };	// Wait here until bytes are fully transmitted.
  VOR_SPI->BANK[SPI_BANK].FIFO_CLR = SPI_FIFO_CLR_RXFIFO_Msk|SPI_FIFO_CLR_TXFIFO_Msk;	// Clear Tx & RX fifo 
}

void wait_WIP(void) 
{
	#ifdef USE_WIP
	int32_t volatile voidRead, readVal, delay;	
	
	wait_idle();

	while(1) { 
		VOR_SPI->BANK[SPI_BANK].DATA = RDSR;	// Read Status Register message
		VOR_SPI->BANK[SPI_BANK].DATA = (uint32_t)0x00 | SPIC_DATA_BMSTOP_Msk;	// Pump the SPI	
 	  while( !(VOR_SPI->BANK[SPI_BANK].STATUS & SPIC_STATUS_TFE_Msk) ) { };	// Wait until TxBuf sends all		
		while( !(VOR_SPI->BANK[SPI_BANK].STATUS & SPIC_STATUS_RNE_Msk) ) { };
	
		voidRead = VOR_SPI->BANK[SPI_BANK].DATA;	// Void read	
		while( !(VOR_SPI->BANK[SPI_BANK].STATUS & SPIC_STATUS_RNE_Msk) ) {};		

		readVal = VOR_SPI->BANK[SPI_BANK].DATA;
		if( !(readVal & RDSR_WIP_Msk ) ) 
			break;

		VOR_SPI->BANK[SPI_BANK].FIFO_CLR = SPIC_FIFO_CLR_RXFIFO_Msk | SPIC_FIFO_CLR_TXFIFO_Msk; // Clear TX & RX FIFO

		for( delay=0; delay<50000; delay++ )
			__NOP();
	}
	VOR_SPI->BANK[SPI_BANK].FIFO_CLR = SPIC_FIFO_CLR_RXFIFO_Msk | SPIC_FIFO_CLR_TXFIFO_Msk; // Clear TX & RX FIFO
  #endif
	return;
}  

static void delayUs(uint32_t us)
{
  uint32_t i;
  while(us){
    for(i=0; i<20; i++){} /* pause at least 1us at 100MHz */
    us--;
  }
}

int Init(unsigned long adr, unsigned long clk, unsigned long fnc) 
{
	//VOR_SYSCONFIG->PERIPHERAL_CLK_ENABLE = 0x7FFFFFFF;  // Enable clocks to all peripherals   
  VOR_SYSCONFIG->PERIPHERAL_CLK_ENABLE = (CLK_ENABLE_SPI0 << SPI_BANK) | CLK_ENABLE_SYSTEM | CLK_ENABLE_CLKGEN | CLK_ENABLE_IOCONFIG | CLK_ENABLE_WDOG;
  VOR_WATCH_DOG->WDOGLOCK = 0x1ACCE551;
  VOR_WATCH_DOG->WDOGCONTROL = 0x0; // disable watchdog
	VOR_SYSCONFIG->PERIPHERAL_RESET &= ~(SYSCONFIG_PERIPHERAL_RESET_SPI0_Msk << SPI_BANK);
  __NOP();
  __NOP();
  VOR_SYSCONFIG->PERIPHERAL_RESET |= (SYSCONFIG_PERIPHERAL_RESET_SPI0_Msk << SPI_BANK);
  __NOP();
  VOR_SPI->BANK[SPI_BANK].CLKPRESCALE = 2; // Set fast clock rate   
	VOR_SPI->BANK[SPI_BANK].CTRL0 = (uint32_t)(3+1)<<SPI_CTRL0_SCRDV_Pos | 0x07;   // Set clock pol/ph=00, clock div=03 and word size=7(8bit) 
	VOR_SPI->BANK[SPI_BANK].CTRL1 = SPI_CTRL1_BLOCKMODE_Msk | SPI_CTRL1_ENABLE_Msk | SPI_CTRL1_BMSTALL_Msk;  // Enable SPI Master mode, block xfer
  VOR_SPI->BANK[SPI_BANK].FIFO_CLR = SPI_FIFO_CLR_RXFIFO_Msk | SPI_FIFO_CLR_TXFIFO_Msk; // Clear TX & RX FIFO  

	wait_idle();
	
	//Clear Block Protection bits to enable programming	
	//Does not set SRWD, so WP_n pin has no effect
	VOR_SPI->BANK[SPI_BANK].DATA = (uint32_t)WREN | SPIC_DATA_BMSTOP_Msk; // Set Write Enable Latch(WEL) bit 
	wait_idle();
  delayUs(600); /* FRAM wake-up time (first WREN write was just to wake up FRAM) */
  VOR_SPI->BANK[SPI_BANK].DATA = (uint32_t)WREN | SPIC_DATA_BMSTOP_Msk; /* Set Write Enable Latch(WEL) bit  */
	wait_idle();	
	
	#ifdef IS_TTZ
	VOR_SPI->BANK[SPI_BANK].DATA = WRSR;	// Write two-byte Status Register message
	VOR_SPI->BANK[SPI_BANK].DATA = (uint32_t)0x00                       ;	// Clear the BP1/BP0 protection
	VOR_SPI->BANK[SPI_BANK].DATA = (uint32_t)0x00 | SPIC_DATA_BMSTOP_Msk;	// Upper status reg word
  #else
	VOR_SPI->BANK[SPI_BANK].DATA = WRSR;	// Write single-byte Status Register message
	VOR_SPI->BANK[SPI_BANK].DATA = (uint32_t)0x00 | SPIC_DATA_BMSTOP_Msk;	// Clear the BP1/BP0 protection
	#endif

  wait_idle();
	
	return (0); 
}

int UnInit (unsigned long fnc) 
{
	wait_WIP();
	wait_idle();
	
	VOR_SPI->BANK[SPI_BANK].DATA = (uint32_t)WREN | SPIC_DATA_BMSTOP_Msk; // Set Write Enable Latch(WEL) bit 
	wait_idle();	
	
	#ifdef IS_TTZ
	VOR_SPI->BANK[SPI_BANK].DATA = WRSR;	// Write two-byte Status Register message
	VOR_SPI->BANK[SPI_BANK].DATA = (uint32_t)0xfd                       ;	// Clear the BP1/BP0 protection
	VOR_SPI->BANK[SPI_BANK].DATA = (uint32_t)0x00 | SPIC_DATA_BMSTOP_Msk;	// Upper status reg word
  #else
	VOR_SPI->BANK[SPI_BANK].DATA = WRSR;	// Write single-byte Status Register message
	VOR_SPI->BANK[SPI_BANK].DATA = (uint32_t)0xfd | SPIC_DATA_BMSTOP_Msk;	// Clear the BP1/BP0 protection
	#endif
	wait_idle();
	
  return (0);                                  
}                               

int EraseChip(void) {
	#ifdef USE_CHIP_ERASE
	wait_WIP();	
  wait_idle();
	VOR_SPI->BANK[SPI_BANK].DATA = (uint32_t)WREN | SPIC_DATA_BMSTOP_Msk; // Send single "Write enable" byte  	
	VOR_SPI->BANK[SPI_BANK].DATA = (uint32_t)TTZ2564_CE | SPIC_DATA_BMSTOP_Msk;
  wait_idle();
	wait_WIP();	
	#else 
	//Byte-by-byte erasing, CANNOT cross page boundaries if device requires page writes
	//will require multiple page write ops
	//This will NOT work for Flash, which require ChipErase to clear memory to 0xff
	//Erase is not necessary for non-Flash types
	uint32_t i, j, adr;
	uint32_t volatile voidRead;
	
	for( i=0; i<(SZDEV/SZPAGE); i++ ) {
		VOR_SPI->BANK[SPI_BANK].DATA = (uint32_t)WREN | SPIC_DATA_BMSTOP_Msk; // Send single "Write enable" byte  	
	  wait_idle();
		
		adr=SZPAGE*i;
		
		VOR_SPI->BANK[SPI_BANK].DATA = WRITE;	// Write command 
	  VOR_SPI->BANK[SPI_BANK].DATA = MSB_ADDR_BYTE(adr); // Address high byte
	  VOR_SPI->BANK[SPI_BANK].DATA = MID_ADDR_BYTE(adr); // Address mid byte 
		VOR_SPI->BANK[SPI_BANK].DATA = LSB_ADDR_BYTE(adr); // Address low byte
				
		for( j=0; j<(SZPAGE-1); j++) {
			while (!(VOR_SPI->BANK[SPI_BANK].STATUS & SPI_STATUS_TNF_Msk)) { };
			VOR_SPI->BANK[SPI_BANK].DATA = (uint32_t)(VALEMPTY);
	  }
    while( !(VOR_SPI->BANK[SPI_BANK].STATUS & SPI_STATUS_TNF_Msk) );
		VOR_SPI->BANK[SPI_BANK].DATA = (uint32_t)(VALEMPTY)  | SPIC_DATA_BMSTOP_Msk; // Last Write also has End Block Transfer
				
	  wait_idle();
 		wait_WIP();		
  }	
	#endif
  return(0);
	
}

int EraseSector (unsigned long adr) 
{
	#ifdef USE_SECTOR_ERASE
 	wait_WIP();		
	wait_idle();
  //Send new Write Enable
	VOR_SPI->BANK[SPI_BANK].DATA = (uint32_t)WREN | SPIC_DATA_BMSTOP_Msk; // Set Write Enable Latch(WEL) bit before sector erase   	
  wait_idle();
	
	// Send Sector Erase	
  VOR_SPI->BANK[SPI_BANK].DATA = (uint32_t)TTZ2564_SE;	// Send sector erase command  
	VOR_SPI->BANK[SPI_BANK].DATA = (uint32_t)MSB_ADDR_BYTE(adr); // Address high byte
	VOR_SPI->BANK[SPI_BANK].DATA = (uint32_t)MID_ADDR_BYTE(adr); // Address mid byte 
  VOR_SPI->BANK[SPI_BANK].DATA = (uint32_t)LSB_ADDR_BYTE(adr) | SPIC_DATA_BMSTOP_Msk; // Address low byte
	
 	wait_WIP();		
	wait_idle();
	#endif
  return (0);                                  
}

int ProgramPage(unsigned long adr, unsigned long sz, unsigned char *buf) {
  int32_t volatile i = 0;  
	int32_t volatile voidRead;
  uint32_t timeout;

 	wait_WIP();		
  wait_idle();
	VOR_SPI->BANK[SPI_BANK].DATA = (uint32_t)WREN | SPIC_DATA_BMSTOP_Msk; // Set Write Enable Latch(WEL) bit 
	
	wait_idle();
	
	VOR_SPI->BANK[SPI_BANK].DATA = WRITE; // Write command 
	VOR_SPI->BANK[SPI_BANK].DATA = MSB_ADDR_BYTE(adr); // Address high byte
	VOR_SPI->BANK[SPI_BANK].DATA = MID_ADDR_BYTE(adr); // Address mid byte 
	VOR_SPI->BANK[SPI_BANK].DATA = LSB_ADDR_BYTE(adr); // Address low byte
		
  while(sz - 1) {
    timeout = 10000;
		while (!(VOR_SPI->BANK[SPI_BANK].STATUS & SPI_STATUS_TNF_Msk)){
      if(--timeout == 0) return 1;
    }
	  VOR_SPI->BANK[SPI_BANK].DATA = (unsigned char)(*buf++);
    voidRead=VOR_SPI->BANK[SPI_BANK].DATA;
		++i;
	  --sz;
  }
  timeout = 10000;
	while( !(VOR_SPI->BANK[SPI_BANK].STATUS & SPI_STATUS_TNF_Msk)){
    if(--timeout == 0) return 1;
  }
	VOR_SPI->BANK[SPI_BANK].DATA = (uint32_t)(*buf)  | SPIC_DATA_BMSTOP_Msk;
		
  wait_idle();
 	wait_WIP();		
  return(0);
}

int BlankCheck(unsigned long adr, unsigned long sz, unsigned char pat)
{
  return (0);       
}

unsigned long Verify(unsigned long adr, unsigned long sz, unsigned char *buf)
{
	int32_t i  ; 
	unsigned char readVal  ;  
	
 	wait_WIP();		
	wait_idle();
	
	VOR_SPI->BANK[SPI_BANK].DATA = READ; // Read command 
	VOR_SPI->BANK[SPI_BANK].DATA = MSB_ADDR_BYTE(adr); // Address high byte
	VOR_SPI->BANK[SPI_BANK].DATA = MID_ADDR_BYTE(adr); // Address mid byte 
	VOR_SPI->BANK[SPI_BANK].DATA = LSB_ADDR_BYTE(adr); // Address low byte

  for( i=0; i<4; i++ ) {
		VOR_SPI->BANK[SPI_BANK].DATA = 0x00; // Pump the SPI
	  while( !(VOR_SPI->BANK[SPI_BANK].STATUS & SPI_STATUS_RNE_Msk) ) { };
	  readVal =	VOR_SPI->BANK[SPI_BANK].DATA; // Void read
  }
	
	for( i=0; i<sz; i++) {
	  VOR_SPI->BANK[SPI_BANK].DATA = 0x00; // Pump the SPI
		while( !(VOR_SPI->BANK[SPI_BANK].STATUS & SPI_STATUS_RNE_Msk) ) { };
	  readVal = 	VOR_SPI->BANK[SPI_BANK].DATA; 
	  if(*buf != readVal) {
		  return(adr+i);
		}
		buf++;
	}
	VOR_SPI->BANK[SPI_BANK].DATA = SPIC_DATA_BMSTOP_Msk; // Terminate Block Transfer
	wait_idle();
		
	return(adr+sz);
	
}

	
