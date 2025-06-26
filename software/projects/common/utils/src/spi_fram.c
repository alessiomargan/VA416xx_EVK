/***************************************************************************************
 * @file     spi_fram.c
 * @version  V2.00
 * @date     07 Sept 2022
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

#include "spi_fram.h"
#ifdef USE_HAL_DRIVER
#include "va416xx_hal_spi.h"
#endif

/*****************************************************************************/
/* Local pre-processor symbols/macros ('#define')                            */
/*****************************************************************************/

/* Commands */
#define FRAM_WREN       0x06
#define FRAM_WRDI       0x04
#define FRAM_RDSR       0x05
#define FRAM_WRSR       0x01
#define FRAM_READ       0x03
#define FRAM_WRITE      0x02
#define FRAM_RDID       0x9F
#define FRAM_SLEEP      0xB9

/* Address Masks */
#define ADDR_MSB_MASK         (uint32_t)0xFF0000
#define ADDR_MID_MASK         (uint32_t)0x00FF00
#define ADDR_LSB_MASK         (uint32_t)0x0000FF
#define MSB_ADDR_BYTE(addr)   ((uint8_t)((addr & ADDR_MSB_MASK)>>16))
#define MID_ADDR_BYTE(addr)   ((uint8_t)((addr & ADDR_MID_MASK)>>8))
#define LSB_ADDR_BYTE(addr)   ((uint8_t)(addr & ADDR_LSB_MASK))

#ifndef USE_HAL_DRIVER
#define HAL_SPI_VERSION     (0x00000300) /* 0.3.0 */

#define SPI_NUM_BANKS       (4)

#define SPI_MASTER_MSK      (0xF) /* SPI 0-3 support master mode */
#define SPI_SLAVE_MSK       (0x7) /* SPI 0-2 support slave mode */

#define SPI0_BANK           (0)
#define SPI1_BANK           (1)
#define SPI2_BANK           (2)
#define SPI3_BANK           (3)

#define SPI_CLK  (SystemCoreClock/2) /* SPI peripherals reside on APB1 */

#define SPI_MIN_WORDLEN     (4)
#define SPI_MAX_WORDLEN     (16)
#endif

/*****************************************************************************/
/* Global variable definitions (declared in header file with 'extern')       */
/*****************************************************************************/

#ifdef USE_HAL_DRIVER
hal_spi_handle_t spiHandle;
#endif

/*****************************************************************************/
/* Local type definitions ('typedef')                                        */
/*****************************************************************************/

/*****************************************************************************/
/* Local variable definitions ('static')                                     */
/*****************************************************************************/

static bool isSpiInit[SPI_NUM_BANKS] = {0};

/*****************************************************************************/
/* Local function prototypes ('static')                                      */
/*****************************************************************************/

#ifndef USE_HAL_DRIVER
static void wait_idle(uint8_t spiBank);
#endif
static void delayUs(uint32_t us);

/*****************************************************************************/
/* Function implementation - global ('extern') and local ('static')          */
/*****************************************************************************/

/*******************************************************************************
 **
 ** @brief  Wait for the SPI to be idle, then clear the FIFOs
 **
 ******************************************************************************/
//#ifndef USE_HAL_DRIVER
static void wait_idle(uint8_t spiBank)
{
  if(spiBank >= SPI_NUM_BANKS){ return; }
  while( !(VOR_SPI->BANK[spiBank].STATUS & SPI_STATUS_TFE_Msk) ) { };	/* Wait until TxBuf sends all */
  while( VOR_SPI->BANK[spiBank].STATUS & SPI_STATUS_BUSY_Msk ) { };	/* Wait here until bytes are fully transmitted */
  VOR_SPI->BANK[spiBank].FIFO_CLR = SPI_FIFO_CLR_RXFIFO_Msk|SPI_FIFO_CLR_TXFIFO_Msk;	/* Clear Tx & RX fifo */
}
//#endif

/*******************************************************************************
 **
 ** @brief  Delay a specified number of us (approximate) - for FRAM wakeup time
 **
 ******************************************************************************/

static void delayUs(uint32_t us)
{
  while(us){
    for(volatile uint32_t i=0; i<20; i++){
      __ASM("NOP");
    } /* pause at least 1us at 100MHz */
    us--;
  }
}


/*******************************************************************************
 **
 ** @brief  Init a SPI periph for F-ram access (SPI in simple polling mode)
 **
 ******************************************************************************/
hal_status_t FRAM_Init(uint8_t spiBank, uint8_t csNum)
{
  if(spiBank >= SPI_NUM_BANKS){ return hal_status_badParam; }
  if(csNum > 7){ return hal_status_badParam; }
  hal_status_t stat = hal_status_ok;

#ifdef USE_HAL_DRIVER
  /* using HAL driver */
  spiHandle.locked = false;
  spiHandle.state = hal_spi_state_reset;
  spiHandle.spi = &VOR_SPI->BANK[spiBank];
  spiHandle.init.blockmode = true;
  spiHandle.init.bmstall = true;
  spiHandle.init.clkDiv = 4;
  spiHandle.init.loopback = false;
  spiHandle.init.mdlycap = false;
  spiHandle.init.mode = hal_spi_clkmode_0;
  spiHandle.init.ms = hal_spi_ms_master;
  spiHandle.init.slaveSelect = csNum;
  spiHandle.init.wordLen = 8;

  stat = HAL_Spi_Init(&spiHandle);
  uint8_t spiData[2];
  if(stat != hal_status_ok){ return stat; } /* abort if driver init issue */
  spiData[0] = FRAM_WREN; /* Set Write Enable Latch(WEL) bit  */
  stat = HAL_Spi_Transmit(&spiHandle, spiData, 1, 0, true);
  delayUs(1000);
  stat = HAL_Spi_Transmit(&spiHandle, spiData, 1, 0, true);
  spiData[0] = FRAM_WRSR;	/* Write single-byte Status Register message */
  spiData[1] = 0x00;	    /* Clear the BP1/BP0 protection */
  stat = HAL_Spi_Transmit(&spiHandle, spiData, 2, 0, true);
#else
  VOR_SYSCONFIG->PERIPHERAL_CLK_ENABLE |= (CLK_ENABLE_SPI0 << spiBank);
  VOR_SPI->BANK[spiBank].CLKPRESCALE = 0x4;
  VOR_SPI->BANK[spiBank].CTRL0 = 0x7;
  VOR_SPI->BANK[spiBank].CTRL1 = 0x382 | (csNum << SPI_CTRL1_SS_Pos);

  /* Clear Block Protection bits to enable programming */
  /* Does not set SRWD, so WP_n pin has no effect */
  wait_idle(spiBank);
  VOR_SPI->BANK[spiBank].DATA = (uint32_t)FRAM_WREN | SPI_DATA_BMSTART_BMSTOP_Msk; /* Set Write Enable Latch(WEL) bit  */
  wait_idle(spiBank);
  delayUs(1000);
  VOR_SPI->BANK[spiBank].DATA = (uint32_t)FRAM_WREN | SPI_DATA_BMSTART_BMSTOP_Msk; /* Set Write Enable Latch(WEL) bit  */
  wait_idle(spiBank);
  VOR_SPI->BANK[spiBank].DATA = FRAM_WRSR;	/* Write single-byte Status Register message */
  VOR_SPI->BANK[spiBank].DATA = (uint32_t)0x00 | SPI_DATA_BMSTART_BMSTOP_Msk;	/* Clear the BP1/BP0 protection */
  wait_idle(spiBank);
#endif /* HAL driver */

  isSpiInit[spiBank] = true;
  return stat;
}

/*******************************************************************************
 **
 ** @brief  Write to F-ram on spi[spiBank]
 **
 ******************************************************************************/
hal_status_t FRAM_Write(uint8_t spiBank, uint32_t addr, uint8_t *buf, uint32_t len)
{
  if(spiBank >= SPI_NUM_BANKS){ return hal_status_badParam; }
  if(isSpiInit[spiBank] == false){ return hal_status_notInitialized; }

  hal_status_t stat = hal_status_ok;

#ifdef USE_HAL_DRIVER

  uint8_t spiData[4];
  spiData[0] = FRAM_WREN;
  stat = HAL_Spi_Transmit(&spiHandle, spiData, 1, 0, true);
  spiData[0] = FRAM_WRITE;          /* Write command */
  spiData[1] = MSB_ADDR_BYTE(addr); /* Address high byte */
  spiData[2] = MID_ADDR_BYTE(addr); /* Address mid byte  */
  spiData[3] = LSB_ADDR_BYTE(addr); /* Address low byte */
  stat = HAL_Spi_Transmit(&spiHandle, spiData, 4, 0, false);
  stat = HAL_Spi_Transmit(&spiHandle, buf, len, 0, true);

#else

  uint32_t volatile voidRead __attribute__((unused));

  wait_idle(spiBank);
  VOR_SPI->BANK[spiBank].DATA = (uint32_t)FRAM_WREN | SPI_DATA_BMSTART_BMSTOP_Msk; /* Set Write Enable Latch(WEL) bit  */
  wait_idle(spiBank);
  VOR_SPI->BANK[spiBank].DATA = FRAM_WRITE;          /* Write command */
  VOR_SPI->BANK[spiBank].DATA = MSB_ADDR_BYTE(addr); /* Address high byte */
  VOR_SPI->BANK[spiBank].DATA = MID_ADDR_BYTE(addr); /* Address mid byte  */
  VOR_SPI->BANK[spiBank].DATA = LSB_ADDR_BYTE(addr); /* Address low byte */

  while(len - 1) {
    while (!(VOR_SPI->BANK[spiBank].STATUS & SPI_STATUS_TNF_Msk)) {};
    VOR_SPI->BANK[spiBank].DATA = *buf++;
    voidRead = VOR_SPI->BANK[spiBank].DATA;
    --len;
  }
  while(!(VOR_SPI->BANK[spiBank].STATUS & SPI_STATUS_TNF_Msk)){}
  VOR_SPI->BANK[spiBank].DATA = (uint32_t)(*buf) | SPI_DATA_BMSTART_BMSTOP_Msk;
  wait_idle(spiBank);

#endif

  return stat;
}

/*******************************************************************************
 **
 ** @brief  Read from F-ram on spi[spiBank]
 **
 ******************************************************************************/
hal_status_t FRAM_Read(uint8_t spiBank, uint32_t addr, uint8_t *buf, uint32_t len)
{
  if(spiBank >= SPI_NUM_BANKS){ return hal_status_badParam; }
  if(isSpiInit[spiBank] == false){ return hal_status_notInitialized; }

#ifdef USE_HAL_DRIVER

   uint8_t spiData[4];
   spiData[0] = FRAM_READ;           /* Read command */
   spiData[1] = MSB_ADDR_BYTE(addr); /* Address high byte */
   spiData[2] = MID_ADDR_BYTE(addr); /* Address mid byte  */
   spiData[3] = LSB_ADDR_BYTE(addr); /* Address low byte */
   return HAL_Spi_TransmitReceive(&spiHandle, spiData, buf, 4, 4, len, 0, true);

#else
  
  uint32_t volatile voidRead __attribute__((unused));
  uint32_t i;
  
  wait_idle(spiBank);
  VOR_SPI->BANK[spiBank].DATA = FRAM_READ;           /* Read command      */
  VOR_SPI->BANK[spiBank].DATA = MSB_ADDR_BYTE(addr); /* Address high byte */
  VOR_SPI->BANK[spiBank].DATA = MID_ADDR_BYTE(addr); /* Address mid byte  */
  VOR_SPI->BANK[spiBank].DATA = LSB_ADDR_BYTE(addr); /* Address low byte  */
  
  for(i=0; i<4; i++) {
    VOR_SPI->BANK[spiBank].DATA = 0x00; // Pump the SPI
    while( !(VOR_SPI->BANK[spiBank].STATUS & SPI_STATUS_RNE_Msk) ) { };
    voidRead = VOR_SPI->BANK[spiBank].DATA; // Void read
  }
  
  for(i=0; i<len; i++) {
    VOR_SPI->BANK[spiBank].DATA = 0x00; // Pump the SPI
    while(!(VOR_SPI->BANK[spiBank].STATUS & SPI_STATUS_RNE_Msk)){}
    *buf = VOR_SPI->BANK[spiBank].DATA;
    buf++;
  }
  VOR_SPI->BANK[spiBank].DATA = SPI_DATA_BMSTART_BMSTOP_Msk; // Terminate Block Transfer
  wait_idle(spiBank);

  return hal_status_ok;

#endif
}

/*******************************************************************************
 **
 ** @brief  Read from F-ram on spi[spiBank]
 **
 ******************************************************************************/
hal_status_t FRAM_Read16(uint8_t spiBank, uint32_t addr, uint16_t *buf, uint32_t len)
{
  if(spiBank >= SPI_NUM_BANKS){ return hal_status_badParam; }
  if(isSpiInit[spiBank] == false){ return hal_status_notInitialized; }

  hal_status_t stat = hal_status_ok;

  uint32_t volatile voidRead __attribute__((unused));
  uint32_t i;
  uint16_t tmp;

#ifdef USE_HAL_DRIVER
  HAL_LOCK(&spiHandle);
  spiHandle.state = hal_spi_state_busy;
#endif

  wait_idle(spiBank);
  VOR_SPI->BANK[spiBank].DATA = FRAM_READ;           /* Read command      */
  VOR_SPI->BANK[spiBank].DATA = MSB_ADDR_BYTE(addr); /* Address high byte */
  VOR_SPI->BANK[spiBank].DATA = MID_ADDR_BYTE(addr); /* Address mid byte  */
  VOR_SPI->BANK[spiBank].DATA = LSB_ADDR_BYTE(addr); /* Address low byte  */

  for(i=0; i<4; i++) {
    VOR_SPI->BANK[spiBank].DATA = 0x00; /* Pump the SPI */
    while( !(VOR_SPI->BANK[spiBank].STATUS & SPI_STATUS_RNE_Msk) ) { };
    voidRead = VOR_SPI->BANK[spiBank].DATA; /* Void read */
  }

  if((len % 2) > 0){ len++; } /* make len even */
  len /= 2;
  for(i=0; i<len; i++) {
    VOR_SPI->BANK[spiBank].DATA = 0x00; /* Pump the SPI */
    while(!(VOR_SPI->BANK[spiBank].STATUS & SPI_STATUS_RNE_Msk)){}
    tmp = VOR_SPI->BANK[spiBank].DATA;
    VOR_SPI->BANK[spiBank].DATA = 0x00; /* Pump the SPI */
    while(!(VOR_SPI->BANK[spiBank].STATUS & SPI_STATUS_RNE_Msk)){}
    tmp |= ((uint16_t)((VOR_SPI->BANK[spiBank].DATA) & 0xff))<<8;
    *buf = tmp;
    buf++;
    WDFEED();
  }
  VOR_SPI->BANK[spiBank].DATA = SPI_DATA_BMSTART_BMSTOP_Msk; /* Terminate Block Transfer */
  wait_idle(spiBank);

#ifdef USE_HAL_DRIVER
  spiHandle.state = hal_spi_state_ready;
  HAL_UNLOCK(&spiHandle);
#endif

  return stat;
}

 /*******************************************************************************
 **
 ** @brief  Reads a section from FRAM and verifys against a buffer for match
 **
 ** @return uint32_t if return value = addr + len, verify ok. Else, returns first
 **                  failing address
 **
 ******************************************************************************/
uint32_t FRAM_Verify(uint8_t spiBank, uint32_t addr, uint8_t *buf, uint32_t len)
{
  if(spiBank >= SPI_NUM_BANKS){ return 0; }
  if(isSpiInit[spiBank] == false){ return 0; }

  uint32_t i;
  unsigned char readVal;

#ifdef USE_HAL_DRIVER
  HAL_LOCK(&spiHandle);
  spiHandle.state = hal_spi_state_busy;
#endif

  wait_idle(spiBank);
  VOR_SPI->BANK[spiBank].DATA = FRAM_READ; /* Read command  */
  VOR_SPI->BANK[spiBank].DATA = MSB_ADDR_BYTE(addr); /* Address high byte */
  VOR_SPI->BANK[spiBank].DATA = MID_ADDR_BYTE(addr); /* Address mid byte  */
  VOR_SPI->BANK[spiBank].DATA = LSB_ADDR_BYTE(addr); /* Address low byte */

  for( i=0; i<4; i++ ) {
    VOR_SPI->BANK[spiBank].DATA = 0x00; /* Pump the SPI */
    while( !(VOR_SPI->BANK[spiBank].STATUS & SPI_STATUS_RNE_Msk) ) { };
    readVal = VOR_SPI->BANK[spiBank].DATA; /* Void read */
  }

  for( i=0; i<len; i++) {
    VOR_SPI->BANK[spiBank].DATA = 0x00; /* Pump the SPI */
    while( !(VOR_SPI->BANK[spiBank].STATUS & SPI_STATUS_RNE_Msk) ) { };
    readVal = VOR_SPI->BANK[spiBank].DATA;
    if(*buf != readVal) {
      //while( !(VOR_SPI->BANK[spiBank].STATUS & SPI_STATUS_RNE_Msk) ) { };
      //return((i<<24)|(*buf<<16)|(readVal<<8)| (VOR_SPI->BANK[spiBank].DATA));
      break;
    }
    buf++;
  }
  VOR_SPI->BANK[spiBank].DATA = SPI_DATA_BMSTART_BMSTOP_Msk; /* Terminate Block Transfer */
  wait_idle(spiBank);

#ifdef USE_HAL_DRIVER
  spiHandle.state = hal_spi_state_ready;
  HAL_UNLOCK(&spiHandle);
#endif

  return(addr+i);
}

/*******************************************************************************
 **
 ** @brief  Un-init the F-ram and SPI
 **
 ******************************************************************************/
hal_status_t FRAM_UnInit(uint8_t spiBank)
{
  if(spiBank >= SPI_NUM_BANKS){ return hal_status_badParam; }

  hal_status_t stat = hal_status_ok;

#ifdef USE_HAL_DRIVER
  uint8_t spiData[2];
  spiData[0] = FRAM_WREN;  /* Set Write Enable Latch(WEL) bit  */
  stat = HAL_Spi_Transmit(&spiHandle, spiData, 1, 0, true);
  spiData[0] = FRAM_WRSR;	 /* Write single-byte Status Register message */
  spiData[1] = 0xfd;    	 /* Set the BP1/BP0 protection */
  stat = HAL_Spi_Transmit(&spiHandle, spiData, 2, 0, true);

#ifdef ENABLE_FRAM_SLEEP
  spiData[0] = FRAM_SLEEP; /* Set sleep mode */
  stat = HAL_Spi_Transmit(&spiHandle, spiData, 1, 0, true);
#endif

  stat = HAL_Spi_DeInit(&spiHandle);
#else
  wait_idle(spiBank);
  VOR_SPI->BANK[spiBank].DATA = (uint32_t)FRAM_WREN | SPI_DATA_BMSTART_BMSTOP_Msk; /* Set Write Enable Latch(WEL) bit  */
  wait_idle(spiBank);
  VOR_SPI->BANK[spiBank].DATA = FRAM_WRSR;	/* Write single-byte Status Register message */
  VOR_SPI->BANK[spiBank].DATA = (uint32_t)0xfd | SPI_DATA_BMSTART_BMSTOP_Msk;	/* Set the BP1/BP0 protection */
  wait_idle(spiBank);

#ifdef ENABLE_FRAM_SLEEP
  VOR_SPI->BANK[spiBank].DATA = (uint32_t)FRAM_SLEEP | SPI_DATA_BMSTART_BMSTOP_Msk; /* Set sleep mode */
  wait_idle(spiBank);
#endif

  VOR_SPI->BANK[spiBank].CTRL1 = 0;
  VOR_SPI->BANK[spiBank].CTRL0 = 0;
  VOR_SYSCONFIG->PERIPHERAL_CLK_ENABLE &= ~(CLK_ENABLE_SPI0 << spiBank);
#endif /* USE_HAL_DRIVER */

  isSpiInit[spiBank] = false;
  return stat;
}

/*****************************************************************************/
/* End of file                                                               */
/*****************************************************************************/
