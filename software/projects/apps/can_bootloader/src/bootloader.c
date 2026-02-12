/***************************************************************************************
 * @file     bootloader.c
 * @version  V2.04
 * @date     19 July 2023
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

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#include "va416xx.h"

#include "board.h"
#include "can.h"
#include "bootloader.h"
#include "can_protocol.h"
#include "crc.h"
#include "main.h"

#include "spi_fram.h"

/*****************************************************************************/
/* Local pre-processor symbols/macros ('#define')                            */
/*****************************************************************************/

#define CTRL0_STATUS_VAL ((uint32_t)(3+1)<<SPI_PERIPHERAL_CTRL0_SCRDV_Pos)

/*****************************************************************************/
/* Local type definitions ('typedef')                                        */
/*****************************************************************************/

/* Bootloader state machine states */
typedef enum en_bl_states
{
	ST_BEGIN = 0,
	ST_CAN_DL,
	ST_COPY_ROM,
	ST_RUN_APP,
	ST_ERROR
} en_bl_states_t;

/* Bootloader status/error codes */
typedef enum en_bl_status
{
  BL_SUCCESS = 0,
  BL_WAITING_FOR_CMD = 1,
  BL_WAITING_FOR_IMG = 2,
  BL_IMG_A_VALID = 3,
  BL_IMG_B_VALID = 4,
  BL_ERR_CAN = 5,
  BL_ERR_IMG_A_INVALID = 6,
  BL_ERR_IMG_B_INVALID = 7,
  BL_ERR_VERIFY_FAIL = 8,
} en_bl_status_t;

/*****************************************************************************/
/* Local variable definitions ('static')                                     */
/*****************************************************************************/

/*****************************************************************************/
/* Local function prototypes ('static')                                      */
/*****************************************************************************/

static bool CopyApplicationImageToFRAM(en_runapp_t appSlot, size_t len);

#ifdef ENABLE_CRC_TEST
static void Fram_GenerateSBEInAppSlot(en_runapp_t appSlot);
static void Fram_GenerateSBEInBootloader(void);
#endif

/*****************************************************************************/
/* Function implementation - global ('extern') and local ('static')          */
/*****************************************************************************/

/*******************************************************************************
 **
 ** @brief  Bootloader startup selfcheck. Writes CRC to NVMEM if CRC block is all 0 or F.
 **         If CRC passes, bootloader runs normally. If CRC fails, bootloader is
 **         corrupted, NVMEM algorithms are not to be trusted so BL will not do
 **         any writes. In this state the BL will only attempt to run APP A.
 **
 ******************************************************************************/
void Bootloader_CheckOwnCRC(void)
{
  uint16_t crc_calc, crc_exp;

  // read CRC block
  crc_exp = *(uint16_t*)(BOOTLOADER_CRC_ADDR);

  // calculate CRC
  WDFEED();
  crc_calc = CalculateCRC16((uint8_t*)BOOTLOADER_START_ADDR, (uint8_t*)BOOTLOADER_CRC_ADDR);
  WDFEED();

  if((crc_exp == 0x0000) || (crc_exp == 0xFFFF)){
    // CRC blank - write it (only happens on first run)

    // F-RAM
#ifdef DEBUG_PRINTS
    printf("BL CRC blank - prog new CRC: 0x%x\n", crc_calc);
#endif
		FRAM_Init(ROM_SPI_BANK, ROM_SPI_CSN);
		FRAM_Write(ROM_SPI_BANK, BOOTLOADER_CRC_ADDR, (uint8_t*)&crc_calc, 2);
		FRAM_UnInit(ROM_SPI_BANK);
    NVIC_SystemReset(); // reset
  }else{
    if(crc_calc != crc_exp){
      // bootloader is corrupted - just try to run App A
      //UART0TxByte('A');
      //UART1TxByte('A');
      Bootloader_RunApp(en_runapp_a);
    }
  }
}

/*******************************************************************************
 **
 ** @brief  Checks the application A and B images (CRC and reset vector)
 **
 ******************************************************************************/
en_memcheck_t Bootloader_CheckAppIsValid(bool fast)
{
  en_memcheck_t retVal = en_memcheck_no_app_valid;
  uint32_t resetVector;
  uint16_t crc_calc, crc_exp;
  uint32_t crc_len;
  uint64_t time_ms;

  // Check APP A
  crc_len = *(uint32_t*)(APP_A_CRC_LEN_ADDR);
  crc_exp = *(uint16_t*)(APP_A_CRC_ADDR);
  time_ms = HAL_time_ms;
  crc_calc = CalculateCRC16((uint8_t*)APP_A_START_ADDR, (uint8_t*)(APP_A_START_ADDR+crc_len));
  time_ms = HAL_time_ms - time_ms;
#ifdef DEBUG_PRINTS
  if(fast == false){
    printf("\nApp A CRC calc time: %ld ms\n", (uint32_t)time_ms);
    printf("App A CRC: 0x%x, calc: 0x%x\n", crc_exp, crc_calc);
  }
#endif
  if(crc_exp == crc_calc){
    resetVector = *((uint32_t*)(APP_A_START_ADDR+RESET_VECTOR_OFFSET));
    /* application A reset vector must point to within application A space */
    if((resetVector >= APP_A_START_ADDR) && (resetVector < APP_A_END_ADDR)){
      /* reset vector is an acceptable value */
      retVal |= en_memcheck_app_a_valid;
      if(fast){ return retVal; } // A is good, return A
    } else {
#ifdef DEBUG_PRINTS
      printf("App A rstVect 0x%lx invalid\n", resetVector);
#endif
    }
  } else {
#ifdef DEBUG_PRINTS
    printf("App A CRC invalid\n");
#endif
  }

  // Check APP B
  crc_len = *(uint32_t*)(APP_B_CRC_LEN_ADDR);
  crc_exp = *(uint16_t*)(APP_B_CRC_ADDR);
  time_ms = HAL_time_ms;
  crc_calc = CalculateCRC16((uint8_t*)APP_B_START_ADDR, (uint8_t*)(APP_B_START_ADDR+crc_len));
  time_ms = HAL_time_ms - time_ms;
#ifdef DEBUG_PRINTS
  if(fast == false){
    printf("App B CRC calc time: %ld ms\n", (uint32_t)time_ms);
    printf("App B CRC: 0x%x, calc: 0x%x\n", crc_exp, crc_calc);
  }
#endif
  if(crc_exp == crc_calc){
    resetVector = *((uint32_t*)(APP_B_START_ADDR+RESET_VECTOR_OFFSET));
    /* application B reset vector must point to within application B space */
    if((resetVector >= APP_B_START_ADDR) && (resetVector < APP_B_END_ADDR)){
      /* reset vector is an acceptable value */
      retVal |= en_memcheck_app_b_valid;
    } else {
#ifdef DEBUG_PRINTS
      printf("App B rstVect 0x%lx invalid\n", resetVector);
#endif
    }
  } else {
#ifdef DEBUG_PRINTS
    printf("App B CRC invalid\n");
#endif
  }
  return retVal;
}

/*******************************************************************************
 **
 ** @brief  Jump to application image A (en_runapp_a) or B (en_runapp_b)
 ** @note   This function will never return
 **
 ******************************************************************************/
void Bootloader_RunApp(en_runapp_t app)
{
  /* Feed watchdog. */
  WDFEED();

  /* Set clkgen to hbo (default) */
  VOR_CLKGEN->CTRL0 &= ~CLKGEN_CTRL0_CLKSEL_SYS_Msk; // force to HBO clk
  for(uint32_t i=0; i<500; i++){
    __NOP();
  }
  VOR_CLKGEN->CTRL0 &= ~CLKGEN_CTRL0_CLK_DIV_SEL_Msk; // select 1x divide

  SysTick->CTRL = 0x0; // disable SysTick

  /* Clear all interrupts set. */
  for(int i = 0; i < 8; i++)
  {
    NVIC->ICER[i] = 0xFFFFFFFF;
    NVIC->ICPR[i] = 0xFFFFFFFF;
  }

  // reset all timers 
  VOR_SYSCONFIG->TIM_RESET = 0;
  VOR_SYSCONFIG->TIM_RESET = 0xFFFFFFFF;
  // disable all timer clocks (default)
  VOR_SYSCONFIG->TIM_CLK_ENABLE = 0;
  // reset all peripherals (except CLKGEN)
  VOR_SYSCONFIG->PERIPHERAL_RESET = CLK_ENABLE_CLKGEN;
  VOR_SYSCONFIG->PERIPHERAL_RESET = ~CLK_ENABLE_CLKGEN;
  // enable only CLKGEN (default)
  VOR_SYSCONFIG->PERIPHERAL_CLK_ENABLE = CLK_ENABLE_CLKGEN; 

  if(app == en_runapp_a){
    // todo move vector table to app_a
    // Barriers
    __DSB();
    __ISB();
    SCB->VTOR = APP_A_START_ADDR;
    // Barriers
    __DSB();
    __ISB();
  } else if(app == en_runapp_b){
    // Barriers
    __DSB();
    __ISB();
    SCB->VTOR = APP_B_START_ADDR;
    // Barriers
    __DSB();
    __ISB();
  }
  vector_reset();
}


/*******************************************************************************
 **
 ** @brief  Re-load code memory after a memory test
 ** @return True if read is successful, else false
 **
 ******************************************************************************/
bool Bootloader_ReloadCode(void)
{
  bool success = true;

  /* set up FRAM SPI */
  hal_status_t stat = FRAM_Init(ROM_SPI_BANK, ROM_SPI_CSN);
  if (stat != hal_status_ok){ success = false; }

  /* Enable writes to code memory space */
  VOR_SYSCONFIG->ROM_PROT |= SYSCONFIG_ROM_PROT_WREN_Msk;

  /* Read FRAM */
  stat = FRAM_Read16(ROM_SPI_BANK, APP_A_START_ADDR, (uint16_t*)APP_A_START_ADDR, 0x40000-APP_A_START_ADDR);
  if (stat != hal_status_ok){ success = false; }

  /* Disable writes to code memory space */
  VOR_SYSCONFIG->ROM_PROT &= ~SYSCONFIG_ROM_PROT_WREN_Msk;

  /* Finished writing to FRAM */
  stat = FRAM_UnInit(ROM_SPI_BANK);
  if (stat != hal_status_ok){ success = false; }

  return success;
}

/*******************************************************************************
 **
 ** @brief  Bootloader state machine
 **
 ******************************************************************************/
void Bootloader_Task(void)
{
	static en_bl_status_t status = BL_WAITING_FOR_CMD;
	static en_bl_states_t state = ST_BEGIN;
  static en_runapp_t app = en_runapp_a;
	static size_t bytesRcvd = 0;
  static en_memcheck_t app_status;

	// Process CAN protocol messages
	CANProtocol_Task();

	switch(state){
		case ST_BEGIN:
		  /* Wait for CAN bootloader command */
		  status = BL_WAITING_FOR_CMD;
		  // CAN protocol task handles incoming commands
		  break;

		case ST_CAN_DL:
			/* Receive file via CAN */
			app = en_runapp_a; // Default to slot A, can be changed via CAN protocol
			bytesRcvd = CANProtocol_ReceiveFile((uint8_t*)APP_A_START_ADDR, (uint8_t*)APP_A_CRC_LEN_ADDR);
			
			if (bytesRcvd > 0){
				/* Transfer complete */
		    	state = ST_COPY_ROM;
				status = BL_SUCCESS;
			} else {
				state = ST_ERROR;
				status = BL_ERR_CAN;
			}
			break;

		case ST_COPY_ROM:
		  /* copy app image from RAM to FRAM,
		     and verify a successful burn. If successful, write the reset vector
		     and CRC to flag the application image as valid/bootable */
			if (CopyApplicationImageToFRAM(app,bytesRcvd) == true){
				state = ST_RUN_APP;
				status = BL_SUCCESS;
			} else {
				state = ST_ERROR;
				status = BL_ERR_VERIFY_FAIL;
			}
			break;

		case ST_RUN_APP:
			/* run the application, first checking if it is valid/safe to run */
			app_status = Bootloader_CheckAppIsValid(false);
		  	if ((app == en_runapp_a) && ((app_status & en_memcheck_app_a_valid) > 0)){
            	/* this function will never return - copies app IVT then generates a soft reset
				   if it somehow does return, throw an error and restart */
				Bootloader_RunApp(en_runapp_a);
			}
			if ((app == en_runapp_b) && ((app_status & en_memcheck_app_b_valid) > 0)){
				/* this function will never return - copies app IVT then generates a soft reset
				   if it somehow does return, throw an error and restart */
				Bootloader_RunApp(en_runapp_b);
			}
			state = ST_ERROR;
			status = (app == en_runapp_a) ? BL_ERR_IMG_A_INVALID : BL_ERR_IMG_B_INVALID;
			break;

		case ST_ERROR:
			/* Error - restart upload sequence */
			state = ST_BEGIN;
			break;

		default:
			/* How did I get here? throw an error */
			state = ST_ERROR;
			return;
	}
}

/*******************************************************************************
 **
 ** @brief  Writes the application FW image space in RAM over to FRAM
 ** Returns true if the copy was successful
 **
 ******************************************************************************/
static bool CopyApplicationImageToFRAM(en_runapp_t appSlot, size_t len)
{
  uint32_t addr;
  uint32_t bytesToWrite = APP_IMG_SZ;
  uint32_t resetVector;
  uint32_t verify;
  uint32_t crc_calc;

  if (appSlot == en_runapp_a){
    addr = APP_A_START_ADDR;
  } else if (appSlot == en_runapp_b){
    addr = APP_B_START_ADDR;
  } else {
    // error
	return false;
  }

  /* We want to first calculate the image CRC, then clear the
	application reset vector in RAM, write the whole image from RAM,
	then write the reset vector last. This is to ensure the app
	image will be flagged as invalid if the write procedure fails
	or is interrupted by a reset */

  /* calculate the image CRC */
  crc_calc = (uint32_t)CalculateCRC16((uint8_t*)addr, (uint8_t*)(addr+len));

  /* save the reset vector */
  resetVector = *((uint32_t*)(addr+RESET_VECTOR_OFFSET));

  /* now clear reset vector in program RAM */
  VOR_SYSCONFIG->ROM_PROT |= SYSCONFIG_ROM_PROT_WREN_Msk; /* Allow writes to code memory space */
  *((uint32_t*)(addr+RESET_VECTOR_OFFSET)) = 0xFFFFFFFF; /* an invalid address */

  /* write the calculated CRC into the image */
  *((uint32_t*)(addr+APP_IMG_SZ-8)) = (uint32_t)(len);
  *((uint32_t*)(addr+APP_IMG_SZ-4)) = crc_calc;

  FRAM_Init(ROM_SPI_BANK, ROM_SPI_CSN); /* set up FRAM SPI */
  while(bytesToWrite >= SZPAGE)
  {
	FRAM_Write(ROM_SPI_BANK, addr, (uint8_t*)addr, SZPAGE);
	bytesToWrite -= SZPAGE;
	addr += SZPAGE;

	/* feed watchdog */
	WDFEED();
  }
  if (bytesToWrite > 0)
  {
	/* write the last page if the end of code doesn't fall on a page boundary */
	FRAM_Write(ROM_SPI_BANK, addr, (uint8_t*)addr, bytesToWrite);

	/* feed watchdog */
	WDFEED();
  }

  /* Reset addr */
  if(appSlot == en_runapp_a){
   addr = APP_A_START_ADDR;
  }else if(appSlot == en_runapp_b){
    addr = APP_B_START_ADDR;
  }

  verify = FRAM_Verify(ROM_SPI_BANK, addr, (uint8_t*)addr, APP_IMG_SZ);
  if (verify != (addr + APP_IMG_SZ))
  {
	/* verify now contains the address of first mismatch */

	/* Finished writing to FRAM */
	FRAM_UnInit(ROM_SPI_BANK);

	/* indicate that verify failed */
	return false;
  }

  /* If the verify was successful, now write the reset vector to flag the image as valid */
  FRAM_Write(ROM_SPI_BANK, addr+RESET_VECTOR_OFFSET, (uint8_t*)&resetVector, 4);

  /* Finished writing to FRAM */
  FRAM_UnInit(ROM_SPI_BANK);

  /* restore reset vector in program RAM */
  *((uint32_t*)(addr+RESET_VECTOR_OFFSET)) = resetVector;

  /* Disable writes to code memory space */
  VOR_SYSCONFIG->ROM_PROT &= ~SYSCONFIG_ROM_PROT_WREN_Msk;

  return true;
}

 /*******************************************************************************
 **
 ** @brief  Toggle a bit in app A or app B, for testing of CRC check
 **
 ** @param  appSlot (app A or app B)
 **
 ** @return none
 **
 ******************************************************************************/
#ifdef ENABLE_CRC_TEST
static void Fram_GenerateSBEInAppSlot(en_runapp_t appSlot)
{
  uint32_t addr;
  uint8_t  u8Temp;
  uint16_t u16Temp;

  if(appSlot == en_runapp_a){
	  addr = APP_A_START_ADDR+0x100;
#ifdef PC_MODE
	  printf("\r\nCreating a SBE in App A");
#endif
  }else if(appSlot == en_runapp_b){
    addr = APP_B_START_ADDR+0x100;
#ifdef PC_MODE
	  printf("\r\nCreating a SBE in App B");
#endif
  }

  FRAM_Init(ROM_SPI_BANK, ROM_SPI_CSN); /* set up FRAM SPI */
  FRAM_Read16(ROM_SPI_BANK, addr, &u16Temp, 1);
  u8Temp = (uint8_t)u16Temp;
  u8Temp ^= 0x1; // toggle 1 bit
  FRAM_Write(ROM_SPI_BANK, addr, &u8Temp, 1);
  FRAM_UnInit(ROM_SPI_BANK);
}
#endif

 /*******************************************************************************
 **
 ** @brief  Toggle a bit in the bootloader, for testing of CRC check
 **
 ** @return none
 **
 ******************************************************************************/
#ifdef ENABLE_CRC_TEST
static void Fram_GenerateSBEInBootloader(void)
{
  uint32_t addr;
  uint8_t  u8Temp;
  uint16_t u16Temp;

  addr = BOOTLOADER_START_ADDR+0x1FF0;
#ifdef PC_MODE
  printf("\r\nCreating a SBE in the bootloader");
#endif

  FRAM_Init(ROM_SPI_BANK, ROM_SPI_CSN); /* set up FRAM SPI */
  FRAM_Read16(ROM_SPI_BANK, addr, &u16Temp, 1);
  u8Temp = (uint8_t)u16Temp;
  u8Temp ^= 0x1; // toggle 1 bit
  FRAM_Write(ROM_SPI_BANK, addr, &u8Temp, 1);
  FRAM_UnInit(ROM_SPI_BANK);
}
#endif

/*****************************************************************************/
/* End of file                                                               */
/*****************************************************************************/
