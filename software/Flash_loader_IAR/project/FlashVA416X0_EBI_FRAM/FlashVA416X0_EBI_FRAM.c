/*************************************************************************
 *
 *    Used with ICCARM and AARM.
 *
 *    (c) Copyright VORAGO Technologies 2020
 *
 *    File name   : FlashVA416X0_SPI_FRAM.c
 *    Description : Flashloader for Vorago VA416X0 Parallel FRAM Memory (FM22L16)
 *
 *    History :
 *    1. Date        : December, 2020
 *       Author      : Tim Alexander
 *       Description : Inital release.
 *
 *
 *    $Revision: 1 $
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

/* Part of EBI register config, address range bits 23-16 */ 
#define FLASH_ADDR_LOW_23_16 (0x00)
#define FLASH_ADDR_HIGH_23_16 (0x7)

#define ARRAY_SIZE(t)         (sizeof(t)/sizeof(t[0]))

#define EBI_MAX_FAILED_WRITES 10
#define SFR_MAX_POLL_COUNT    1000

#define FRAM_ERASE_VAL (0x00000000)

/** external functions **/
extern uint32_t READ_MEM(volatile const uint32_t *sfr);
extern void     WRITE_MEM(volatile uint32_t *sfr, uint32_t value);

/** internal functions **/
#if USE_ARGC_ARGV
static const char* findOption(char *option, int with_value,
                              int argc, char const *argv[]);
#endif /* USE_ARGC_ARGV */

#if 0
static uint32_t WaitToClear(volatile const uint32_t *reg, uint32_t mask);
static uint32_t WaitToSet(volatile const uint32_t *reg, uint32_t mask);
#endif
static void     VOR_GPIOC_PinMux(uint32_t pin, uint32_t funsel);
static void     VOR_GPIOD_PinMux(uint32_t pin, uint32_t funsel);
static void     VOR_GPIOE_PinMux(uint32_t pin, uint32_t funsel);
static void     VOR_GPIOF_PinMux(uint32_t pin, uint32_t funsel);
static void     EBI_Init(void);
static uint32_t EBI_Write(uint32_t addr, char const *data, uint32_t count);

/** private data **/
__no_init volatile uint32_t hardfault;
__no_init uint32_t chip_erase;

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
  
  /* EBI Init */
  EBI_Init();
  
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
  /* Set destination address */
  uint32_t dest = ((uint32_t)block_start + offset_into_block);

  /* Write a page to the FRAM */
  if (RESULT_OK != EBI_Write(dest, buffer, count))
  {
    return RESULT_ERROR;
  }
  buffer += count;

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
  if (chip_erase && (RESULT_OK != EBI_Write((uint32_t)block_start, NULL, block_size)))
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
 * Description: Calculate checksum of flash area
 *************************************************************************/
OPTIONAL_CHECKSUM
#if CODE_ADDR_AS_VOID_PTR
uint32_t FlashChecksum(void const *begin, uint32_t count)
#else
uint32_t FlashChecksum(uint32_t begin, uint32_t count)
#endif
{
  uint8_t const * addr = (uint8_t const *)begin;
  return Crc16(addr, count);
}


/*************************************************************************
 * Function Name: FlashSignoff
 * Parameters:  none
 *
 * Return: RESULT_OK (0)    - Signoff Successful
 *         RESULT_ERROR (1) - Signoff Fail
 *
 * Description: Writes EEPROM status register options if required.
 *************************************************************************/
OPTIONAL_SIGNOFF
uint32_t FlashSignoff()
{
  uint32_t i;
  
  /* Return GPIO to their default state */
  for(i=2;i<=15;i++){
     VOR_GPIOC_PinMux(i, 0); //addr
  }
  for(i=0;i<=15;i++){
     VOR_GPIOD_PinMux(i, 0);//addr & data
  } 
  for(i=0;i<=9;i++){
     VOR_GPIOE_PinMux(i, 0);//data
  } 
  VOR_GPIOE_PinMux(12, 0); //ce0
  VOR_GPIOE_PinMux(13, 0); //ce1
  VOR_GPIOE_PinMux(14, 0); //ce2
  VOR_GPIOE_PinMux(15, 0); //ce3
  
  VOR_GPIOF_PinMux(0, 0); //oen
  VOR_GPIOF_PinMux(1, 0); //wen
  
  return RESULT_OK;
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
#if 0
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
#endif

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
#if 0
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
#endif

/*************************************************************************
 * Function Name: VOR_GPIOC_PinMux
 * Parameters: uint32_t pin - pin number (0-15)
               uit32_t funsel - pin alternalte function select (0-3)
 *
 * Return: void
 *
 * Description: Sets GPIO port C pin mux
 *************************************************************************/
void VOR_GPIOC_PinMux(uint32_t pin, uint32_t funsel) {
  uint32_t iocfg = READ_MEM(&(VOR_IOCONFIG->PORTC[pin]));
  WRITE_MEM(&(VOR_IOCONFIG->PORTC[pin]), (iocfg & ~((0x3)<<IOCONFIG_PORTC_FUNSEL_Pos)) | \
                                         ((funsel)<<IOCONFIG_PORTC_FUNSEL_Pos));
}
            
/*************************************************************************
 * Function Name: VOR_GPIOD_PinMux
 * Parameters: uint32_t pin - pin number (0-15)
               uit32_t funsel - pin alternalte function select (0-3)
 *
 * Return: void
 *
 * Description: Sets GPIO port D pin mux
 *************************************************************************/
void VOR_GPIOD_PinMux(uint32_t pin, uint32_t funsel) {
  uint32_t iocfg = READ_MEM(&(VOR_IOCONFIG->PORTD[pin]));
  WRITE_MEM(&(VOR_IOCONFIG->PORTD[pin]), (iocfg & ~((0x3)<<IOCONFIG_PORTD_FUNSEL_Pos))	| \
                                         ((funsel)<<IOCONFIG_PORTD_FUNSEL_Pos));
}
            
/*************************************************************************
 * Function Name: VOR_GPIOE_PinMux
 * Parameters: uint32_t pin - pin number (0-15)
               uit32_t funsel - pin alternalte function select (0-3)
 *
 * Return: void
 *
 * Description: Sets GPIO port E pin mux
 *************************************************************************/
void VOR_GPIOE_PinMux(uint32_t pin, uint32_t funsel) {
  uint32_t iocfg = READ_MEM(&(VOR_IOCONFIG->PORTE[pin]));
  WRITE_MEM(&(VOR_IOCONFIG->PORTE[pin]), (iocfg & ~((0x3)<<IOCONFIG_PORTE_FUNSEL_Pos))	| \
                                         ((funsel)<<IOCONFIG_PORTE_FUNSEL_Pos));
}
            
/*************************************************************************
 * Function Name: VOR_GPIOF_PinMux
 * Parameters: uint32_t pin - pin number (0-15)
               uit32_t funsel - pin alternalte function select (0-3)
 *
 * Return: void
 *
 * Description: Sets GPIO port F pin mux
 *************************************************************************/
void VOR_GPIOF_PinMux(uint32_t pin, uint32_t funsel) {
  uint32_t iocfg = READ_MEM(&(VOR_IOCONFIG->PORTF[pin]));
  WRITE_MEM(&(VOR_IOCONFIG->PORTF[pin]), (iocfg & ~((0x3)<<IOCONFIG_PORTF_FUNSEL_Pos))	| \
                                         ((funsel)<<IOCONFIG_PORTF_FUNSEL_Pos));
}

/*************************************************************************
 * Function Name: EBI_Init
 * Parameters: none
 *
 * Return: void
 *
 * Description: EBI Init. Configure clocks, external bus interface. Set slow timings.
 *************************************************************************/
static void EBI_Init(void)
{
  uint32_t i;
  
  /* Turn-on clocks to peripheral modules */
  WRITE_MEM(&VOR_SYSCONFIG->PERIPHERAL_CLK_ENABLE, (CLK_ENABLE_EBI | \
                                                    CLK_ENABLE_SYSTEM | \
                                                    CLK_ENABLE_CLKGEN | \
                                                    CLK_ENABLE_IOCONFIG | \
                                                    CLK_ENABLE_PORTC | \
                                                    CLK_ENABLE_PORTD | \
                                                    CLK_ENABLE_PORTE | \
                                                    CLK_ENABLE_PORTF));
  
  /* Set CLKGEN to internal clock, 20MHz */
  WRITE_MEM(&VOR_CLKGEN->CTRL0, 0x00000030);
  WRITE_MEM(&VOR_CLKGEN->CTRL1, 0x00000020);
  __NOP();
  __NOP();

  /* Init EBI controller */
  WRITE_MEM(&VOR_SYSCONFIG->EBI_CFG0,
    ((FLASH_ADDR_LOW_23_16<<SYSCONFIG_EBI_CFG0_ADDRLOW0_Pos) 
    |(FLASH_ADDR_HIGH_23_16<<SYSCONFIG_EBI_CFG0_ADDRHIGH0_Pos)
    |(6<<SYSCONFIG_EBI_CFG0_CFGREADCYCLE_Pos)
    |(6<<SYSCONFIG_EBI_CFG0_CFGWRITECYCLE_Pos)  
    |(6<<SYSCONFIG_EBI_CFG0_CFGTURNAROUNDCYCLE_Pos)
    |(SYSCONFIG_EBI_CFG0_CFGSIZE_Msk) // 1 = 16 bit, 0 = 8 bit
  ));
  
  /* Init GPIO - set GPIO mux to external bus alternate function */
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

/*************************************************************************
 * Function Name: EBI_Write
 * Parameters: uint32_t *addr - start address of page
 *             char const *data - pointer to data buffer
 *             uint32_t count - data buffer length
 *
 * Return: RESULT_OK (0)    - Write successful
 *         RESULT_ERROR (1) - Write failed
 *
 * Description: Writes a data page to EBI FRAM.
 *************************************************************************/
static uint32_t EBI_Write(uint32_t addr, char const *data, uint32_t count)
{
  uint32_t retry_count = 0;
  uint32_t erase = 0;
  
  /* Check that addr is within VA416x0 external memory range */
  if((addr < 0x60000000) || (addr > (0x61000000 - count))){
    return RESULT_ERROR;
  }
  
  /* Doing it as 32 bit, this is a 16 bit EBI (byte writes not allowed)
     Also, WRITE_MEM() is 32-bit */
  while(count % 4){ count++; } // make count a multiple of 4 bytes, round up
  count /= 4; // count is now the number of 32-bit words
  
  uint32_t const * dataPtrWord = (uint32_t const *)(data);
  
  if(data == NULL){
    erase = 1;
  }

  do{
    /* Clear the hardfault flag. */
    hardfault = 0;

    /* Send data words */
    if(erase){
      /* Write zeroes */
      while(count){
        WRITE_MEM((volatile uint32_t*)addr, FRAM_ERASE_VAL);
        count--;
        addr += 4;
      }
    } else {
      /* Write data */
      while(count){
        WRITE_MEM((volatile uint32_t*)addr, *dataPtrWord);
        count--;
        addr += 4;
        dataPtrWord++;
      }
    }
  }
  while (hardfault && (++retry_count < EBI_MAX_FAILED_WRITES));

  if (retry_count == EBI_MAX_FAILED_WRITES){
    return RESULT_ERROR;
  }

  return RESULT_OK;
}
