/*************************************************************************
 *
 *    Used with ICCARM and AARM.
 *
 *    (c) Copyright VORAGO Technologies 2020
 *
 *    File name   : FlashVA416X0_SPI_FRAM.c
 *    Description : Flashloader for Vorago VA416X0 SPI FRAM Memory (FM25V20A)
 *
 *    History :
 *    1. Date        : December, 2020
 *       Author      : Tim Alexander
 *       Description : Inital release.
 *
 *
 *    $Revision: 69 $
 **************************************************************************/

/** include files **/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <intrinsics.h>

/* The flash loader framework API declarations. */
#include "flash_loader.h"
#include "flash_loader_extra.h"

/* MCU header file (perpheral register definitions) */
#include "va416xx.h"

/** local definitions **/
/* Available SPI Memory Commands */
#define SPI_CMD_WREN          0x06
#define SPI_CMD_WRDI          0x04
#define SPI_CMD_RDSR          0x05
#define SPI_CMD_WRSR          0x01
#define SPI_CMD_READ          0x03
#define SPI_CMD_FSTRD         0x0B
#define SPI_CMD_WRITE         0x02
#define SPI_CMD_SLEEP         0xB9
#define SPI_CMD_RDID          0x9F

#define ARRAY_SIZE(t)         (sizeof(t)/sizeof(t[0]))

#define SPI_PAGE_SIZE         256

#define SPI_SR_WIP_Msk        0x01

#define SPI_MAX_FAILED_WRITES 10
#define SPI_SR_MAX_POLL_COUNT 3000
#define SFR_MAX_POLL_COUNT    1000

/** missing from va416xx.h header **/
/* ----------------------------------  SPI_DATA  --------------------------- */
#ifndef SPI_DATA_VALUE_Pos
#define SPI_DATA_VALUE_Pos             0
#endif
#ifndef SPI_DATA_VALUE_Msk
#define SPI_DATA_VALUE_Msk             (0x000000ffffUL << SPI_DATA_VALUE_Pos)
#endif
#ifndef SPI_DATA_BMSKIPDATA_Pos
#define SPI_DATA_BMSKIPDATA_Pos        30
#endif
#ifndef SPI_DATA_BMSKIPDATA_Msk
#define SPI_DATA_BMSKIPDATA_Msk        (0x01UL << SPI_DATA_BMSKIPDATA_Pos)
#endif
#ifndef SPI_DATA_BMSTART_BMSTOP_Pos
#define SPI_DATA_BMSTART_BMSTOP_Pos    31
#endif
#ifndef SPI_DATA_BMSTART_BMSTOP_Msk
#define SPI_DATA_BMSTART_BMSTOP_Msk    (0x01UL << SPI_DATA_BMSTART_BMSTOP_Pos)
#endif

/** external functions **/
extern uint32_t READ_MEM(volatile const uint32_t *sfr);
extern void     WRITE_MEM(volatile uint32_t *sfr, uint32_t value);

/** internal functions **/
#if USE_ARGC_ARGV
static const char* findOption(char *option, int with_value,
                              int argc, char const *argv[]);
#endif /* USE_ARGC_ARGV */

static uint32_t WaitToClear(volatile const uint32_t *reg, uint32_t mask);
static uint32_t WaitToSet(volatile const uint32_t *reg, uint32_t mask);
static void     SPI_Init(void);
static uint32_t SPI_WritePage(uint32_t addr, char const *data);
static uint32_t SPI_WriteData(char const *data, uint32_t count);
static uint32_t SPI_WriteStatusReg(uint8_t value);
static uint32_t SPI_PollStatusReg(uint16_t count,
                                  uint8_t mask,
                                  uint8_t *status);
static uint32_t SPI_WaitIdle(void);
static uint32_t SPI_Read(uint32_t addr, uint8_t *buf, uint32_t count);
static uint16_t CalcCRC16(uint16_t crcBegin, uint8_t* buf, uint32_t count);

/** private data **/
__no_init volatile uint32_t hardfault;
__no_init uint32_t chip_erase;
__no_init uint8_t ee_sr_bp;
__no_init uint8_t ee_sr_srwd;

/** public functions **/
void __HardFault_Handler(void)
{
  hardfault = 1;
  __DSB();
}

/*************************************************************************
 * Function Name: FlashInit
 * Parameters: Flash Base Address, Image Size, Link Address,
 *             Flags, Argument count, Argument values
 *
 * Return:  RESULT_OK (0)    - Init Successful
 *          RESULT_ERROR (1) - Init Fail
 *
 * Description: Inits SPI and clears EEPROM status register.
 *************************************************************************/
#if USE_ARGC_ARGV
uint32_t FlashInit(void *base_of_flash, uint32_t image_size,
                   uint32_t link_address, uint32_t flags,
                   int argc, char const *argv[])
#else
uint32_t FlashInit(void *base_of_flash, uint32_t image_size,
                   uint32_t link_address, uint32_t flags)
#endif
{
  chip_erase = 0;
  ee_sr_bp = 0;
  ee_sr_srwd = 0;

  /* SPI Init */
  SPI_Init();

  /* Try to clear SRWD and Block Protect bits */
  if (RESULT_OK != SPI_WriteStatusReg(0x00))
    return RESULT_ERROR;

#if USE_ARGC_ARGV
const char *bp_str;

  /* Get Block Protect argument value if passed */
  if ((bp_str = findOption("--bp", 1, argc, argv)))
  {
    ee_sr_bp = *bp_str - 0x30;
    if ((ee_sr_bp > 3) || (bp_str[1] != 0)) ee_sr_bp = 0;
  }
  else
    ee_sr_bp = 0;

  /* Get SRWD argument presence */
  ee_sr_srwd = !!findOption("--srwd", 0, argc, argv);

#endif /* USE_ARGC_ARGV */

  if (flags & FLAG_ERASE_ONLY)
  {
    /* The flash loader has been invoked with the sole purpose of erasing
       the whole flash memory */
    chip_erase = 1;
  }

  return RESULT_OK;
}

/*************************************************************************
 * Function Name: FlashWrite
 * Parameters: block base address, offset in block, data size, ram buffer
 *             pointer
 *
 * Return: RESULT_OK (0)    - Write Successful
 *         RESULT_ERROR (1) - Write Fail
 *
 * Description. Writes data to FRAM
 *************************************************************************/
uint32_t FlashWrite(void *block_start,
                    uint32_t offset_into_block,
                    uint32_t count,
                    char const *buffer)
{
uint32_t size = 0;
/* Set destination address */
uint32_t dest = ((uint32_t)block_start + offset_into_block);

  while (size < count)
  {
    /* Write a page to the FRAM */
    if (RESULT_OK != SPI_WritePage(dest, buffer))
      return RESULT_ERROR;

    size += SPI_PAGE_SIZE;
    dest += SPI_PAGE_SIZE;
    buffer += SPI_PAGE_SIZE;
  }

  return RESULT_OK;
}

/*************************************************************************
 * Function Name: FlashErase
 * Parameters:  Block Address, Block Size
 *
 * Return: RESULT_OK (0)    - Erase Successful
 *         RESULT_ERROR (1) - Erase Fail
 *
 * Description: Erases a page only if flashloader is called
 *              with FLAG_ERASE_ONLY set.
 *************************************************************************/
uint32_t FlashErase(void *block_start,
                    uint32_t block_size)
{
  /* If flashloader is called with FLAG_ERASE_ONLY set,
     write a page with zeroes */
  if (chip_erase && (RESULT_OK != SPI_WritePage((uint32_t)block_start, NULL)))
  {
    return RESULT_ERROR;
  }

  return RESULT_OK;
}

/*************************************************************************
 * Function Name: FlashChecksum
 * Parameters:  begin - flash address to begin checksum
                count - length of area to checksum (bytes)
 *
 * Return: checksum
 *
 * Description: Calculate CRC-16 checksum of flash area
 *************************************************************************/
OPTIONAL_CHECKSUM
#if CODE_ADDR_AS_VOID_PTR
uint32_t FlashChecksum(void const *begin, uint32_t count)
#else
uint32_t FlashChecksum(uint32_t begin, uint32_t count)
#endif
{
  uint32_t addr = (uint32_t)begin;
  uint32_t size = SPI_PAGE_SIZE;
  uint8_t  readBuf[SPI_PAGE_SIZE];
  uint16_t crc = 0;
  
  while(count){
    if(count < size){ size = count; }
    SPI_Read(addr, readBuf, size);
    crc = CalcCRC16(crc, readBuf, size);
    addr += size;
    count -= size;
  }
  return crc;
}

/*************************************************************************
 * Function Name: FlashSignoff
 * Parameters:  none
 *
 * Return: RESULT_OK (0)    - Signoff Successful
 *         RESULT_ERROR (1) - Signoff Fail
 *
 * Description: Writes FRAM status register options if required.
 *************************************************************************/
OPTIONAL_SIGNOFF
uint32_t FlashSignoff()
{
uint8_t ee_status = (ee_sr_srwd << 7) | (ee_sr_bp << 2);

  if (ee_status && (RESULT_OK != SPI_WriteStatusReg(ee_status)))
    return RESULT_ERROR;

  /* Ensure FRAM has completed writing */
  return SPI_PollStatusReg(SPI_SR_MAX_POLL_COUNT, SPI_SR_WIP_Msk, NULL);
}

/** private functions **/
#if USE_ARGC_ARGV
static const char* findOption(char *option, int with_value,
                              int argc, char const *argv[])
{
  int i;

  for (i = 0; i < argc; i++)
  {
    if (strcmp(option, argv[i]) == 0)
    {
      if (with_value)
      {
        if (i + 1 < argc)
          return argv[i + 1]; /* The next argument is the value. */
        else
          return 0; /* The option was found but there is no value to return. */
      }
      else
      {
        /* Return the flag argument itself just to get a non-zero pointer. */
        return argv[i];
      }
    }
  }
  return 0;
}
#endif /* USE_ARGC_ARGV */

/*************************************************************************
 * Function Name: WaitToClear
 * Parameters: volatile uint32_t *reg - SFR
 *             uint32_t mask - mask
 *
 * Return: RESULT_OK (0)    - Operation completed successfully
 *         RESULT_ERROR (1) - Timeout error
 *
 * Description: Waits until bit(s) in SFR, defined by the mask, is cleared.
 *************************************************************************/
static uint32_t WaitToClear(volatile const uint32_t *reg, uint32_t mask)
{
  uint32_t timeout = SFR_MAX_POLL_COUNT;

  while (timeout && (READ_MEM(reg) & mask))
    timeout--;

  if (timeout == 0)
    return RESULT_ERROR;
  else
    return RESULT_OK;
}

/*************************************************************************
 * Function Name: WaitToSet
 * Parameters: volatile uint32_t *reg - SFR
 *             uint32_t mask - mask
 *
 * Return: RESULT_OK (0)    - Operation completed successfully
 *         RESULT_ERROR (1) - Timeout error
 *
 * Description: Waits until bit(s) in SFR, defined by the mask, is set.
 *************************************************************************/
static uint32_t WaitToSet(volatile const uint32_t *reg, uint32_t mask)
{
  uint32_t timeout = SFR_MAX_POLL_COUNT;

  while (timeout && !(READ_MEM(reg) & mask))
    timeout--;

  if (timeout == 0)
    return RESULT_ERROR;
  else
    return RESULT_OK;
}

/*************************************************************************
 * Function Name: SPI_Init
 * Parameters: none
 *
 * Return: void
 *
 * Description: SPI Init. SPI Clock = 5 MHz, BLOCKMODE, 8-bit, mode 0.
 *************************************************************************/
static void SPI_Init(void)
{
  /* Turn-on clocks to peripheral modules */
  WRITE_MEM(&VOR_SYSCONFIG->PERIPHERAL_CLK_ENABLE, (CLK_ENABLE_SPI3 | \
    CLK_ENABLE_SYSTEM | CLK_ENABLE_CLKGEN | CLK_ENABLE_IOCONFIG));
  
  /* Set CLKGEN to internal clock, 20MHz */
  WRITE_MEM(&VOR_CLKGEN->CTRL0, 0x00000030);
  WRITE_MEM(&VOR_CLKGEN->CTRL1, 0x00000020);
  __NOP();
  __NOP();

  /* Init SPI3 controller */
  WRITE_MEM(&VOR_SPI3->CLKPRESCALE, 0x02);
  WRITE_MEM(&VOR_SPI3->CTRL0, (4 << SPI_CTRL0_SCRDV_Pos) |
                              (7 << SPI_CTRL0_SIZE_Pos));
  WRITE_MEM(&VOR_SPI3->CTRL1, (SPI_CTRL1_BLOCKMODE_Msk |
                               SPI_CTRL1_BMSTALL_Msk));
  WRITE_MEM(&VOR_SPI3->TXFIFOIRQTRG, 8);
  WRITE_MEM(&VOR_SPI3->RXFIFOIRQTRG, 8);
  WRITE_MEM(&VOR_SPI3->CTRL1, (READ_MEM(&VOR_SPI3->CTRL1) |
                               SPI_CTRL1_ENABLE_Msk));
  WRITE_MEM(&VOR_SPI3->FIFO_CLR, (SPI_FIFO_CLR_RXFIFO_Msk |
                                  SPI_FIFO_CLR_TXFIFO_Msk));
}

/*************************************************************************
 * Function Name: SPI_WritePage
 * Parameters: uint32_t addr - start address of page
 *             char const *data - pointer to data buffer
 *
 * Return: RESULT_OK (0)    - Write successful
 *         RESULT_ERROR (1) - Write failed
 *
 * Description: Writes a data page to FRAM.
 *************************************************************************/
static uint32_t SPI_WritePage(uint32_t addr, char const *data)
{
  uint32_t retry_count = 0;

  do
  {
    /* Wait while FRAM is busy. The hardfault flag will be cleared. */
    if (RESULT_OK != SPI_PollStatusReg(SPI_SR_MAX_POLL_COUNT,
                                       SPI_SR_WIP_Msk,
                                       NULL))
      return RESULT_ERROR;

    /* Wait until TX FIFO level is acceptable */
    if (RESULT_OK != WaitToSet(&VOR_SPI3->STATUS,
                               SPI_STATUS_TXTRIGGER_Msk))
    return RESULT_ERROR;

    /* Enable write */
    WRITE_MEM(&VOR_SPI3->DATA, (SPI_CMD_WREN |
                                SPI_DATA_BMSTART_BMSTOP_Msk));

    /* Send WRITE command */
    WRITE_MEM(&VOR_SPI3->DATA, SPI_CMD_WRITE);

    /* Write address bytes on SPI bus */
    WRITE_MEM(&VOR_SPI3->DATA, (addr>>16) & 0xFF);
    WRITE_MEM(&VOR_SPI3->DATA, (addr>>8)  & 0xFF);
    WRITE_MEM(&VOR_SPI3->DATA, (addr>>0)  & 0xFF);

    /* Check if a hardfault has occurred so far */
    if (hardfault)
    {
      /* Abort the current SPI transfer */
      WRITE_MEM(&VOR_SPI3->DATA, (SPI_DATA_BMSKIPDATA_Msk |
                                  SPI_DATA_BMSTART_BMSTOP_Msk));
      /* Try again */
      continue;
    }

    /* Send data bytes */
    if (RESULT_OK != SPI_WriteData(data, SPI_PAGE_SIZE))
      return RESULT_ERROR;
  }
  while (hardfault && (++retry_count < SPI_MAX_FAILED_WRITES));

  if (retry_count == SPI_MAX_FAILED_WRITES)
    return RESULT_ERROR;

  return RESULT_OK;
}

/*************************************************************************
 * Function Name: SPI_WriteData
 * Parameters: char const *data - pointer to data buffer
 *             uint32_t count - transmit data size
 *
 * Return: RESULT_OK (0)    - Write successful or ended by a hardfault
           RESULT_ERROR (1) - Peripheral error occurred
 *
 * Description: Write data to FRAM over SPI and close the SPI frame.
 *************************************************************************/
static uint32_t SPI_WriteData(char const *data, uint32_t count)
{
  uint32_t txdata;

  while (count--)
  {
    /* Prepare data. If no data pointer is passed - prepare 0 */
    txdata = data ? *data++ : 0;
    /* If the last byte - close SPI frame */
    if (0 == count) txdata |= SPI_DATA_BMSTART_BMSTOP_Msk;
    /* Wait while Tx FIFO is full */
    if (RESULT_OK != WaitToSet(&VOR_SPI3->STATUS,
                               SPI_STATUS_TNF_Msk))
      return RESULT_ERROR;
    /* If a hardfault has occurred - end transfer */
    if (hardfault)
    {
      /* Close SPI frame */
      WRITE_MEM(&VOR_SPI3->DATA, (SPI_DATA_BMSKIPDATA_Msk |
                                  SPI_DATA_BMSTART_BMSTOP_Msk));
      break;
    }
    /* Write a data byte on SPI bus */
    WRITE_MEM(&VOR_SPI3->DATA, txdata);
  }
  return RESULT_OK;
}

/*************************************************************************
 * Function Name: SPI_WriteStatusReg
 * Parameters: uint8_t value - status register value to be written
 *
 * Return: RESULT_OK (0)    - Write successful
 *         RESULT_ERROR (1) - Write failed
 *
 * Description: Writes the FRAM status register.
 *************************************************************************/
static uint32_t SPI_WriteStatusReg(uint8_t value)
{
  uint32_t retry_count = 0;

  do
  {
    /* Wait until FRAM is ready. The hardfault flag will be cleared. */
    if (RESULT_OK != SPI_PollStatusReg(SPI_SR_MAX_POLL_COUNT,
                                       SPI_SR_WIP_Msk,
                                       NULL))
      return RESULT_ERROR;
    /* Wait until TX FIFO level is acceptable */
    if (RESULT_OK != WaitToSet(&VOR_SPI3->STATUS,
                               SPI_STATUS_TXTRIGGER_Msk))
      return RESULT_ERROR;

    /* Enable write */
    WRITE_MEM(&VOR_SPI3->DATA, SPI_CMD_WREN |
                               SPI_DATA_BMSTART_BMSTOP_Msk);
    /* Write FRAM status register */
    WRITE_MEM(&VOR_SPI3->DATA, SPI_CMD_WRSR);
    WRITE_MEM(&VOR_SPI3->DATA, value | SPI_DATA_BMSTART_BMSTOP_Msk);
  }
  while (hardfault && (++retry_count < SPI_MAX_FAILED_WRITES));

  if (retry_count == SPI_MAX_FAILED_WRITES)
    return RESULT_ERROR;

  return RESULT_OK;
}

/*************************************************************************
 * Function Name: SPI_PollStatusReg
 * Parameters:  uint16_t count - times to poll the status
 *              uint8_t  mask - mask
 *              uint8_t *status - pointer used to pass back the read status
 *                                 value
 *
 * Return: RESULT_OK (0)    - Operation completed successfully
 *         RESULT_ERROR (1) - Timeout error
 *
 * Description: Polls the FRAM status register until the bit(s) defined
 *              by the mask are cleared.
 *************************************************************************/
static uint32_t SPI_PollStatusReg(uint16_t count,
                                  uint8_t mask,
                                  uint8_t *status)
{
  volatile uint8_t value;
  volatile uint32_t result = RESULT_ERROR;

  do
  {
    hardfault = 0;
    /* Wait until SPI3 is ready */
    if (RESULT_OK != WaitToClear(&VOR_SPI3->STATUS,
                                 SPI_STATUS_BUSY_Msk))
      return RESULT_ERROR;
    /* Clear RX and TX FIFO */
    WRITE_MEM(&VOR_SPI3->FIFO_CLR, (SPI_FIFO_CLR_RXFIFO_Msk |
                                    SPI_FIFO_CLR_TXFIFO_Msk));
    __DSB();

    /* Send RDSR command */
    WRITE_MEM(&VOR_SPI3->DATA, SPI_CMD_RDSR);
    /* Dummy write. Data in RX FIFO is available after 16 received bits */
    WRITE_MEM(&VOR_SPI3->DATA, 0x00);
    if (RESULT_OK != WaitToSet(&VOR_SPI3->STATUS, SPI_STATUS_RNE_Msk)){
      return RESULT_ERROR;
    }
    /* Skip first byte */
    READ_MEM(&VOR_SPI3->DATA);

    do
    {
      /* Dummy write to force SPI read */
      WRITE_MEM(&VOR_SPI3->DATA, 0x00);
      if (RESULT_OK != WaitToSet(&VOR_SPI3->STATUS, SPI_STATUS_RNE_Msk)){
        return RESULT_ERROR;
      }
      /* Read next byte */
      value = READ_MEM(&VOR_SPI3->DATA);
      /* If a harfault occurred so far - repeat the whole SPI frame */
      if (hardfault) break;
      /* Are the bit(s) defined by the mask cleared? */
      if (!(value & mask))
      {
        /* Yes - exit successfully */
        result = RESULT_OK;
        break;
      }
      /* No - continue polling */
      count--;
    }
    while (count);
    /* Close the SPI frame */
    WRITE_MEM(&VOR_SPI3->DATA, (SPI_DATA_BMSKIPDATA_Msk |
                                SPI_DATA_BMSTART_BMSTOP_Msk));
  }
  while (hardfault && count && (result != RESULT_OK));

  if (status != NULL) *status = value;

  return result;
}

/*************************************************************************
 * Function Name: SPI_WaitIdle
 * Parameters: none
 *
 * Return: RESULT_OK (0)    - Operation completed successfully
 *         RESULT_ERROR (1) - Timeout error
 *
 * Description: Wait for SPI3 to be idle, then clear te FIFOs
 *************************************************************************/
static uint32_t SPI_WaitIdle(void)
{
  uint32_t timeout = SFR_MAX_POLL_COUNT;
	while(!(READ_MEM(&VOR_SPI3->STATUS) & SPI_STATUS_TFE_Msk)) {
    if(--timeout == 0){ break; }
  }
  while(READ_MEM(&VOR_SPI3->STATUS) & SPI_STATUS_BUSY_Msk){
    if(--timeout == 0){ break; }
  }
  WRITE_MEM(&VOR_SPI3->FIFO_CLR, (SPI_FIFO_CLR_RXFIFO_Msk | \
                                  SPI_FIFO_CLR_TXFIFO_Msk));
  if(timeout == 0){ return RESULT_ERROR; }
  return RESULT_OK;
}

/*************************************************************************
 * Function Name: SPI_Read
 * Parameters: uint32_t addr  - beginning address of SPI to read from
 *             uint8_t *buf   - pointer to data buffer [out]
 *             uint32_t count - read data size
 *
 * Return: RESULT_OK (0)    - Write successful or ended by a hardfault
           RESULT_ERROR (1) - Peripheral error occurred
 *
 * Description: Read data from SPI FRAM
 *************************************************************************/
static uint32_t SPI_Read(uint32_t addr, uint8_t *buf, uint32_t count)
{
  uint32_t volatile voidRead;
  uint32_t i;
  
  if(SPI_WaitIdle() != RESULT_OK){ return RESULT_ERROR; }
  WRITE_MEM(&VOR_SPI3->DATA, SPI_CMD_READ); // Read command 
  
  /* Write address bytes on SPI bus */
  WRITE_MEM(&VOR_SPI3->DATA, (addr>>16) & 0xFF);
  WRITE_MEM(&VOR_SPI3->DATA, (addr>>8)  & 0xFF);
  WRITE_MEM(&VOR_SPI3->DATA, (addr>>0)  & 0xFF);
  
  for(i=0; i<4; i++) {
		WRITE_MEM(&VOR_SPI3->DATA, 0x00); // Pump the SPI
	  while( !(READ_MEM(&VOR_SPI3->STATUS) & SPI_STATUS_RNE_Msk) ) { };
	  voidRead = READ_MEM(&VOR_SPI3->DATA); // Void read
  }
  
  for(i=0; i<count; i++) {
	  WRITE_MEM(&VOR_SPI3->DATA, 0x00); // Pump the SPI
		while(!(READ_MEM(&VOR_SPI3->STATUS) & SPI_STATUS_RNE_Msk)){}
	  *buf = READ_MEM(&VOR_SPI3->DATA);
		buf++;
	}
	WRITE_MEM(&VOR_SPI3->DATA, SPI_DATA_BMSTART_BMSTOP_Msk); // End block
	if(SPI_WaitIdle() != RESULT_OK){ return RESULT_ERROR; }
  
  return RESULT_OK;
}

/*************************************************************************
 * Function Name: CalcCRC16
 * Parameters: crcBegin - initial crc (allows doing the crc in chunks)
 *             buf - pointer to buffer to crc
 *             count - length of buf
 *
 * Return: uint16_t the crc
 *
 * Description: Calculates crc. Implements CRC-16-CCITT (poly 0x1021)
 *************************************************************************/
static uint16_t CalcCRC16(uint16_t crcBegin, uint8_t* buf, uint32_t count)
{
  uint16_t crc = crcBegin;
  uint32_t i;
  for (i=0; i<count; i++)
  {
    crc  = (crc >> 8) | (crc << 8);
    crc ^= *buf++;
    crc ^= (crc & 0xff) >> 4;
    crc ^=  crc << 12;
    crc ^= (crc & 0xff) << 5;
  }
  return crc;
}
