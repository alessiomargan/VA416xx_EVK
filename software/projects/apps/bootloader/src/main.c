/***************************************************************************************
 * @file     main.c
 * @version  V2.05
 * @date     14 December 2023
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

/***************************************************************************************
 * Version 2023_07_19_2p04. Changes since last build
 *
 * Edited printf calls in hardfault_handler.c to suppress GCC warnings
 * Changes to main.c (HAL_SysTick_Init() call) and hal_config.c to support HAL_SysTick_Init change
 * Edited printf calls in hardfault handler to suppress GCC warnings
 * Added peripheral resets to Bootloader_RunApp()
 *
****************************************************************************************/

/***************************************************************************************
 * Version 2023_04_18_2p03. Changes since last build
 *
 * Bugfix: Bootloader_RunApp() did not disable SysTick before running app (caused issue with FreeRTOS)
 *
****************************************************************************************/

/***************************************************************************************
 * Version 2023_03_16_2p02. Changes since last build
 *
 * Bugfix: CopyApplicationImageToFRAM() wasn't performing verify
 * Bugfix: Logic operation in UART0GetBuffer(), UART1GetBuffer(), UART2GetBuffer() used bitwise AND instead of boolean
 * Bugfix: Size calculation wrong if FW image larger than 64kb due to using 16-bit integer for length storage
 *
****************************************************************************************/

/***************************************************************************************
 * Version 2023_01_20_2p01. Changes since last build
 *
 * NONE (changes were in other projects within this release)
 *
****************************************************************************************/

/***************************************************************************************
 * Version 2022_09_28_2p00. Changes since last build
 *
 * 1. Simplified clockgen init to reduce code size
 * 2. Now also supporting GCC
 *
****************************************************************************************/

/***************************************************************************************
 * Version 2022_02_24_v1p5. Changes since last build
 *
 * 1. Cleaned up for application note release
 * 2. Changed memory map to use full 256KB for VA416xx
 * 3. Relaxed reset vector address range requirements, in case the app SW uses a shorter
 *    than expected interrupt vector table.
 *
****************************************************************************************/

/***************************************************************************************
 * Version 2022_01_21_v1p4. Changes since last build
 *
 * 1. IAR inregration complete - bugfix with memcpy to instruction RAM
 *
****************************************************************************************/

/***************************************************************************************
 * Version 2022_01_07_v1p3. Changes since last build
 *
 * 1. IAR inregration begin. Compiles OK, but there is a runtime XMODEM bug
 *
****************************************************************************************/

/***************************************************************************************
 * Version 2021_06_24_v1p2. Changes since last build
 *
 * 1. Added memory test option. Tests code and ram areas not
 *    used by the bootloader itself.
 * 2. spi_fram.c/.h copied from common to the local project to make custom edits.
 *    The edits involve using 16b writes to the buffer when doing FRAM reads, to read
 *    FRAM directly to code memory. Used to refresh the code RAM after a memory test.
 *
****************************************************************************************/

/***************************************************************************************
 * Version 2021_06_23_v1p1. Changes since last build
 *
 * 1. Removed text file upload/check
 * 2. Changed MODE pin behavior
 * 3. Updated GPIO init
 *
****************************************************************************************/

/***************************************************************************************
 * Bootloader memory map
 *
 * <0x0>     Bootloader start                         <code up to 0x3FFE bytes>
 * <0x3FFE>  Bootloader CRC                           <halfword>
 * <0x4000>  App image A start                        <code up to 0x1DFF8 (~120K) bytes>
 * <0x21FF8> App image A CRC check length             <word>
 * <0x21FFC> App image A CRC check value              <word>
 * <0x22000> App image B start                        <code up to 0x1DFF8 (~120K) bytes>
 * <0x3FFF8> App image B CRC check length             <word>
 * <0x3FFFC> App image B CRC check value              <word>
 * <0x40000>                                          <end>
 *
****************************************************************************************/

/***************************************************************************************
 * Pin connection info. Both UARTs in dual redundant mode (accept RX from either, TX
 * to both). Modify UART funsels in board.c to move to other pins. LED pin assignment 
 * is in board.h.
 *
 * PB14 Uart1 TX 115200bps 8N1
 * PB15 Uart1 RX 115200bps 8N1
 * PG0  Uart0 TX 115200bps 8N1
 * PG1  Uart0 RX 115200bps 8N1
 * PG5  LED pin (blinks)
 *
****************************************************************************************/

/***************************************************************************************
 * External clock info. This bootloader is set up to expect a 40MHz input clock, and 
 * multiplies that 2X to 80MHz. If the external clock value is different, change the 
 * value of EXTCLK defined in board.h. If using the crystal oscillator instead of
 * externally driven clock, set the value of XTAL defined in board.h, and change
 * gCurrentXtalsel to equal hal_xtalsel_xtal_en (Line 216 of main.c).
 *
****************************************************************************************/

/*****************************************************************************/
/* Include files                                                             */
/*****************************************************************************/

/* System */
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "va416xx.h"
#include "board.h"

/** VA416xx hal includes */
#include "va416xx_hal.h"
#include "va416xx_hal_clkgen.h"
#include "va416xx_hal_irqrouter.h"

/* Bootloader components */
#include "main.h"
#include "uart.h"
#include "bootloader.h"
#include "stdio_remap.h"
#include "memtest.h"

/*****************************************************************************/
/* Local pre-processor symbols/macros ('#define')                            */
/*****************************************************************************/

/*****************************************************************************/
/* Global variable definitions (declared in header file with 'extern')       */
/*****************************************************************************/

volatile uint32_t gSecondsCounter = 0; // will overflow after about 136 years
//hal_xtalsel_t gCurrentXtalsel;

/*****************************************************************************/
/* Local type definitions ('typedef')                                        */
/*****************************************************************************/

/*****************************************************************************/
/* Local variable definitions ('static')                                     */
/*****************************************************************************/

/*****************************************************************************/
/* Local function prototypes ('static')                                      */
/*****************************************************************************/

#ifdef ENABLE_WATCHDOG
static void EnableWatchdog(void);
#endif
static void ConfigEdac(uint32_t ramScrub, uint32_t romScrub);
static hal_status_t Initialize(void);

/*****************************************************************************/
/* Function implementation - global ('extern') and local ('static')          */
/*****************************************************************************/

/*******************************************************************************
 **
 ** @brief  Set up watchdog counter
 **
 ******************************************************************************/
#define WDOG_MS (50)
#ifdef ENABLE_WATCHDOG
static void EnableWatchdog(void)
{
  VOR_SYSCONFIG->PERIPHERAL_CLK_ENABLE |= CLK_ENABLE_WDOG;
  VOR_SYSCONFIG->PERIPHERAL_RESET &= ~SYSCONFIG_PERIPHERAL_RESET_WDOG_Msk;
  __NOP();
  __NOP();
  VOR_SYSCONFIG->PERIPHERAL_RESET |= SYSCONFIG_PERIPHERAL_RESET_WDOG_Msk;
  VOR_WATCH_DOG->WDOGLOAD = (SystemCoreClock/4/1000)*WDOG_MS;
  VOR_WATCH_DOG->WDOGCONTROL = 3; // enable INTEN and RESEN
  VOR_WATCH_DOG->WDOGLOCK = 1; // lock registers
}
#endif

/*******************************************************************************
 **
 ** @brief  Set up memory Error Detection and Correction (EDAC)
 **
 ******************************************************************************/
static void ConfigEdac(uint32_t ramScrub, uint32_t romScrub)
{
  VOR_SYSCONFIG->RAM0_SCRUB = ramScrub;
  VOR_SYSCONFIG->RAM1_SCRUB = ramScrub;
  VOR_SYSCONFIG->ROM_SCRUB = romScrub;

  IRQROUTER_ENABLE_CLOCK();
  NVIC_EnableIRQ(EDAC_MBE_IRQn);
  NVIC_SetPriority(EDAC_MBE_IRQn, 0);
  NVIC_EnableIRQ(EDAC_SBE_IRQn);
  NVIC_SetPriority(EDAC_SBE_IRQn, 0);

  VOR_SYSCONFIG->IRQ_ENB = 0x3f; // enable all IRQ
}

/*******************************************************************************
 **
 ** @brief  Initialization setup. Called at the beginning of main()
 **
 ******************************************************************************/
static hal_status_t Initialize(void)
{
  hal_status_t clkgenStatus = hal_status_ok;

  /* Configure CLKGEN */

  clkgenStatus = HAL_Clkgen_XtalN(); // external clock input - xtal amplifier disabled - freq = EXTCLK
  //clkgenStatus = HAL_Clkgen_XtalOsc(); // external clock input - xtal amplifier enabled - freq = XTAL


  // Init HAL
  hal_status_t halStatus = HAL_Init();

  // Init EDAC
  ConfigEdac(1000,125);

#ifdef ENABLE_WATCHDOG
  EnableWatchdog();
#endif

  // UART
  UartInit(VOR_UART0, UART_BAUDRATE);
  UartInit(VOR_UART1, UART_BAUDRATE);

  // GPIO and pin mux
  hal_status_t ioStatus = HAL_Iocfg_SetupPins(bootDefaultConfig);

  // Redirect stdio/stderr (printf) to UART0 and UART1
  SetStdioOutput(en_stdio_uart01);

  // reset marker
#ifdef PC_MODE
  printf("\r\n*** SW INIT ***\n\n");
#endif
  WDFEED();
  if(clkgenStatus != hal_status_ok){
#ifdef PC_MODE
    printf("clkgenStatus: %s\n", HAL_StatusToString(clkgenStatus));
#endif
    return clkgenStatus;
  }
  if(halStatus != hal_status_ok){
#ifdef PC_MODE
    printf("halStatus: %s\n", HAL_StatusToString(halStatus));
#endif
    return halStatus;
  }
  if(ioStatus != hal_status_ok){
#ifdef PC_MODE
    printf("ioStatus: %s\n", HAL_StatusToString(ioStatus));
#endif
    return ioStatus;
  }
  return hal_status_ok;
}

/*******************************************************************************
 **
 ** @brief  Application entry point. SystemInit() has already been called
 **
 ******************************************************************************/
int main(void)
{
  bool enterBootloader = false;
  en_memcheck_t app_valid = en_memcheck_no_app_valid;
  static uint64_t nextSecTask_ms = 0;
  bool fastboot = false;
  uint64_t blTimeout;
  uint64_t dotPrintTimeMs = 1000;

  (void)Initialize(); // Setup clock, GPIO, and peripherals



  // Check bootloader's CRC (and write it if blank)
  //Bootloader_CheckOwnCRC(); // TODO uncomment this when not debugging from RAM

#ifdef BL_DO_MEMORY_TEST
  sramTest_init();
  sramTest();
  chkInv = true; // flip the bit pattern, check again
  sramTest_init();
  printf("Memtest errors: %d\r\n", sramTest());
  Bootloader_ReloadCode();
#endif
#ifdef BL_FASTBOOT_MODE
  fastboot = true;
#endif

  // Begin bootloader startup
#ifdef PC_MODE
  if(fastboot == false){
    printf("SW Built: %s @ %s\r\n" , __DATE__, __TIME__);
    printf("VA416xx Bootloader version %s\r\n", SOFTWARE_VERSION_STR);
    printf("SYSCLK: %ld\r\n", SystemCoreClock);
    printf("EF_ID = 0x%08lx\r\n" , VOR_SYSCONFIG->EF_ID0);
    printf("Press spacebar to enter bootloader command");
  } else {
    printf("*FB* BL ver %s\r\n", SOFTWARE_VERSION_STR); // fastboot msg
  }
#endif

  // for timeout period, looks for heralding character
  if(fastboot){
    blTimeout = HAL_time_ms + 10; // very small timeout when in fastboot mode
  }else {
    blTimeout = HAL_time_ms + BOOTLOADER_MSEC_TIMEOUT;
  }
  while(HAL_time_ms < blTimeout){
    if (UART0IsRxByteAvailable()){
      if(UART0RxByte() == BOOTLOADER_HERALDING_CHAR){
        // caught enter bootloader cmd
        enterBootloader = true;
#ifndef PC_MODE
        // send response to heralding char
        UART0TxByte(BOOTLOADER_HERALDING_RESP_CHAR);
#endif
        break;
      }
    }
    if (UART1IsRxByteAvailable()){
      if(UART1RxByte() == BOOTLOADER_HERALDING_CHAR){
        // caught enter bootloader cmd
        enterBootloader = true;
#ifndef PC_MODE
        // send response to heralding char
        UART1TxByte(BOOTLOADER_HERALDING_RESP_CHAR);
#endif
        break;
      }
    }
    HAL_WaitMs(5);

#ifdef PC_MODE
    if (HAL_time_ms > dotPrintTimeMs){
      dotPrintTimeMs += 1000;
      printf(".");
    }
#endif

    // Feed watchdog
    WDFEED();
  }

  if(false == enterBootloader){
    // no hearalding char heard - attempt to run application
    app_valid = Bootloader_CheckAppIsValid(fastboot);
    if((app_valid & en_memcheck_app_a_valid) > 0){
#ifdef PC_MODE
      if(fastboot == false){
        // print info
        printf("\r\nApp A valid. ");
        if ((app_valid & en_memcheck_app_b_valid) > 0){
          printf("App B valid. ");
        }else{
          printf("App B invalid. ");
        }
        printf("Running App ");
        HAL_WaitMs(12);
      }
      printf("A\r\n");
      HAL_WaitMs(1);
#endif
      Bootloader_RunApp(en_runapp_a);
    }
    else if ((app_valid & en_memcheck_app_b_valid) > 0){
#ifdef PC_MODE
      if(fastboot == false){
        // print info
        printf("\r\nApp A invalid. App B valid. Running App ");
        HAL_WaitMs(12);
      }
      printf("B\r\n");
      HAL_WaitMs(1);
#endif
      Bootloader_RunApp(en_runapp_b);
    }else{
#ifdef PC_MODE
      printf("\r\nBoth app images invalid. Upload new code to app A or B\r\n");
#endif
    }
  }

  // bootloader loop - wait for code upload
  while(1)
  {
    if(HAL_time_ms >= nextSecTask_ms){
      gSecondsCounter++;
      nextSecTask_ms += 1000;
      EVK_LED_PORT->TOGOUT = 1<<EVK_LED_PIN;
    }

    Bootloader_Task();

    // Feed watchdog
    WDFEED();
  }
}   // main

/*******************************************************************************
 **
 ** @brief  Loss of clock handler - set up new clock freq (switch to internal)
 **
 ******************************************************************************/
void LoCLK_IRQHandler(void)
{
  (void)HAL_Clkgen_Rearm();
  SystemCoreClockUpdate();
  OnSystemClockChanged();
#ifdef PC_MODE
  (void)printf("\nloclk\n");
#endif
}

/*******************************************************************************
 **
 ** @brief  Called when system clock has changed - re-init peripherals
 **
 ******************************************************************************/
void OnSystemClockChanged(void)
{
  UartInit(VOR_UART0, UART_BAUDRATE);
  UartInit(VOR_UART1, UART_BAUDRATE);
  (void)HAL_SysTick_Init(SYSTICK_INTERVAL_MS, SYSTICK_PRIORITY);
#ifdef PC_MODE
  (void)printf("\nnew sysclk: %ld", SystemCoreClock);
#endif
}

/*******************************************************************************
 **
 ** @brief  Catch NMI (avoid hang)
 **
 ******************************************************************************/
void NMI_Handler(void)
{
  return;
}

/*******************************************************************************
 **
 ** @brief  Called on RAM0/RAM1/ROM SBE
 **
 ******************************************************************************/
void EDAC_SBE_IRQHandler(void)
{
#ifdef PC_MODE
  printf("EDAC SBE, IRQrw: 0x%lx", VOR_SYSCONFIG->IRQ_RAW);
#endif
  VOR_SYSCONFIG->IRQ_CLR = SYSCONFIG_IRQ_CLR_RAM0SBE_Msk |
                           SYSCONFIG_IRQ_CLR_RAM1SBE_Msk |
                           SYSCONFIG_IRQ_CLR_ROMSBE_Msk;
}

/*******************************************************************************
 **
 ** @brief  Called on RAM0/RAM1 MBE
 **
 ******************************************************************************/
void EDAC_MBE_IRQHandler(void)
{
#ifdef PC_MODE
  printf("EDAC MBE, IRQrw: 0x%lx", VOR_SYSCONFIG->IRQ_RAW);
  for(uint32_t i=0; i<1000000; i++){} // let print complete b4 rst
#endif
  VOR_SYSCONFIG->IRQ_CLR = SYSCONFIG_IRQ_CLR_RAM0MBE_Msk |
                           SYSCONFIG_IRQ_CLR_RAM1MBE_Msk |
                           SYSCONFIG_IRQ_CLR_ROMMBE_Msk;

  // reset
  NVIC_SystemReset();
}

/*****************************************************************************/
/* End of file                                                               */
/*****************************************************************************/
