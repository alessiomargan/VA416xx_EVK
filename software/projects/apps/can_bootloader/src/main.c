/***************************************************************************************
 * @file     main.c
 * @version  V1.00
 * @date     10 February 2026
 *
 * @note
 * VORAGO Technologies - CAN Bootloader
 *
 * @note
 * Copyright (c) 2013-2026 VORAGO Technologies.
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
#include "can.h"
#include "bootloader.h"
#include "can_protocol.h"

/*****************************************************************************/
/* Local pre-processor symbols/macros ('#define')                            */
/*****************************************************************************/

/*****************************************************************************/
/* Global variable definitions (declared in header file with 'extern')       */
/*****************************************************************************/

volatile uint32_t gSecondsCounter = 0; // will overflow after about 136 years

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

  // Init HAL
  hal_status_t halStatus = HAL_Init();

  // Init EDAC
  ConfigEdac(1000,125);

#ifdef ENABLE_WATCHDOG
  EnableWatchdog();
#endif

  // GPIO and pin mux
  hal_status_t ioStatus = HAL_Iocfg_SetupPins(bootDefaultConfig);

  // Initialize CAN for bootloader
  CAN_BootloaderInit();

  // Initialize CAN protocol handler
  CANProtocol_Init();

  WDFEED();
  if(clkgenStatus != hal_status_ok){
    return clkgenStatus;
  }
  if(halStatus != hal_status_ok){
    return halStatus;
  }
  if(ioStatus != hal_status_ok){
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

  (void)Initialize(); // Setup clock, GPIO, and peripherals

  // Check bootloader's CRC (and write it if blank)
  //Bootloader_CheckOwnCRC(); // TODO uncomment this when not debugging from RAM

#ifdef BL_FASTBOOT_MODE
  fastboot = true;
#endif

  // Begin bootloader startup
  // Wait for CAN message to enter bootloader
  if(fastboot){
    blTimeout = HAL_time_ms + 10; // very small timeout when in fastboot mode
  }else {
    blTimeout = HAL_time_ms + BOOTLOADER_MSEC_TIMEOUT;
  }
  
  while(HAL_time_ms < blTimeout){
    // Check for CAN message indicating bootloader entry
    if (CAN_IsRxMsgAvailable()){
      uint32_t msgId;
      uint8_t msgData[8];
      uint8_t msgLen;
      
      if (CAN_ReceiveMessage(&msgId, msgData, &msgLen)) {
        if (msgId == CAN_BL_CMD_ID && msgLen > 0 && msgData[0] == CAN_BL_CMD_PING) {
          // Received ping command - enter bootloader
          enterBootloader = true;
          CANProtocol_SendResponse(CAN_BL_RESP_ACK, NULL, 0);
          break;
        }
      }
    }
    HAL_WaitMs(5);

    // Feed watchdog
    WDFEED();
  }

  if(false == enterBootloader){
    // no bootloader entry command - attempt to run application
    app_valid = Bootloader_CheckAppIsValid(fastboot);
    if((app_valid & en_memcheck_app_a_valid) > 0){
      Bootloader_RunApp(en_runapp_a);
    }
    else if ((app_valid & en_memcheck_app_b_valid) > 0){
      Bootloader_RunApp(en_runapp_b);
    }
  }

  // bootloader loop - wait for firmware upload via CAN
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
}

/*******************************************************************************
 **
 ** @brief  Called when system clock has changed - re-init peripherals
 **
 ******************************************************************************/
void OnSystemClockChanged(void)
{
  (void)HAL_SysTick_Init(SYSTICK_INTERVAL_MS, SYSTICK_PRIORITY);
  CAN_BootloaderInit(); // Reinitialize CAN with new clock
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
  VOR_SYSCONFIG->IRQ_CLR = SYSCONFIG_IRQ_CLR_RAM0MBE_Msk |
                           SYSCONFIG_IRQ_CLR_RAM1MBE_Msk |
                           SYSCONFIG_IRQ_CLR_ROMMBE_Msk;

  // reset
  NVIC_SystemReset();
}

/*****************************************************************************/
/* End of file                                                               */
/*****************************************************************************/
