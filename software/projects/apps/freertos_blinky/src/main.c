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

/*****************************************************************************/
/* Project Description                                                       */
/*****************************************************************************/

/*

This is a demo FreeRTOS blinky project.

The project sets up two tasks:

1.  The BlinkingTask toggles the PG5 LED pin every BLINK_INTERVAL_MS milliseconds
2.  The SecondsTask increments a seconds counter and prints this to the UART. 
    UART information: UART0, pins TX PG0, RX PG1, baud: 115200 8N1. 
    UART Header: J7 on EVK top board

*/

/*****************************************************************************/
/* Include files                                                             */
/*****************************************************************************/

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "va416xx.h"
#include "board.h"
#include "uart.h"

#include "va416xx_hal.h"
#include "va416xx_hal_clkgen.h"
#include "va416xx_hal_irqrouter.h"
#include "va416xx_hal_timer.h"
#include "va416xx_debug.h"
#include "spi_fram.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"

/*****************************************************************************/
/* Local pre-processor symbols/macros ('#define')                            */
/*****************************************************************************/

//#define FREERTOS_TICK_TIM_NR  (0)
#define BLINK_INTERVAL_MS   (100)

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

/*****************************************************************************/
/* Function implementation - global ('extern') and local ('static')          */
/*****************************************************************************/
extern void xPortSysTickHandler(void);

/*******************************************************************************
 **
 ** @brief  Set up memory Error Detection and Correction (EDAC)
 **
 ** *** IMPORTANT FOR FLIGHT / RADIATION ENVIRONMENT ***
 **
 ******************************************************************************/
static void ConfigureEdac(void)
{
  /* Example. Change these values based on orbit / expected bit error rate */
  VOR_SYSCONFIG->RAM0_SCRUB = 500;
  VOR_SYSCONFIG->RAM1_SCRUB = 500;
  VOR_SYSCONFIG->ROM_SCRUB  = 500;

  IRQROUTER_ENABLE_CLOCK();
  NVIC_EnableIRQ(EDAC_MBE_IRQn);
  NVIC_SetPriority(EDAC_MBE_IRQn, 0);
  NVIC_EnableIRQ(EDAC_SBE_IRQn);
  NVIC_SetPriority(EDAC_SBE_IRQn, 0);

  VOR_SYSCONFIG->IRQ_ENB = 0x3f; // enable all IRQ
}

/*******************************************************************************
 **
 ** @brief  Set up watchdog counter
 **
 ** *** IMPORTANT FOR FLIGHT / RADIATION ENVIRONMENT ***
 **
 ******************************************************************************/
#define WDOG_MS (50)
#define WDFEED() VOR_WATCH_DOG->WDOGINTCLR = 1
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
 ** @brief  Initialization setup. Called at the beginning of main()
 **
 ******************************************************************************/
static uint8_t Initialize(void)
{
  hal_status_t clkgen_status = hal_status_ok;
  uint8_t      initerrs      = 0;

#ifdef ENABLE_RTT
  DBG_SetStdioOutput(en_stdio_rtt); // stdio/stderr -> Segger RTT (if ENABLE_RTT defined in board.h)
#endif

  // Configure CLKGEN
  clkgen_status = HAL_Clkgen_PLL(CLK_CTRL0_XTAL_N_PLL2P5X);

  // Init HAL
  hal_status_t status = HAL_Init();
  if(status != hal_status_ok)
  {
    initerrs++;
  }

  // Watchdog, EDAC
  // *** IMPORTANT FOR FLIGHT / RADIATION ENVIRONMENT ***
#ifdef ENABLE_WATCHDOG
  EnableWatchdog(); // disable if running in debug to avoid reset on bkpt
#endif
  ConfigureEdac();
	// TMR refresh (0, fastest rate)
  VOR_SYSCONFIG->REFRESH_CONFIG_H = 0x0; // no test mode by default (normal mode)
  VOR_SYSCONFIG->REFRESH_CONFIG_L = 0x0; // this has to be set to 0 (fastest) for UART stability

  // Put the boot FRAM into sleep mode
  // *** IMPORTANT FOR FLIGHT / RADIATION ENVIRONMENT ***
  FRAM_Init(ROM_SPI_BANK,ROM_SPI_CSN);
  FRAM_UnInit(ROM_SPI_BANK); // UnInit sets sleep mode

  // Configure IO
  status = HAL_Iocfg_Init(&ioPinCfgArr[0]);
  if(status != hal_status_ok)
  {
    initerrs++;
  }

  // setup UART0 (PG0, PG1)
  UartInit(VOR_UART0, UART_BAUDRATE);
  VOR_PORTG->CLROUT = 1UL<<2; // PG2 output low

  // setup UART1 (PB12-15, PMOD UART J36)
  UartInit(VOR_UART1, UART_BAUDRATE);

  // setup UART2 (PF6-9, PMOD UART J37)
  UartInit(VOR_UART2, UART_BAUDRATE);

  // Debug prints and printf destination
  // Un-comment one of these to send prints to a uart instead of RTT
#ifndef ENABLE_RTT
  DBG_SetStdioOutput(en_stdio_uart0);
  //DBG_SetStdioOutput(en_stdio_uart1);
  //DBG_SetStdioOutput(en_stdio_uart2);
#endif

  // clkgen report (most likely fail is PLL failed to lock, if no external clk)
  if(clkgen_status != hal_status_ok)
  {
    initerrs++;
    dbgprintln("CLKGEN status: %s", HAL_StatusToString(clkgen_status));
  }
  dbgprintln("sysclk: %ld\n", SystemCoreClock);

  return initerrs;
}

/*******************************************************************************
 **
 ** @brief
 **
 ******************************************************************************/
void BlinkingTask(void *pvParameters)
{
  for(;;)
  {
    EVK_LED_PORT->TOGOUT = 1<<EVK_LED_PIN;
    vTaskDelay(BLINK_INTERVAL_MS);
  }
}

/*******************************************************************************
 **
 ** @brief
 **
 ******************************************************************************/
void SecondsTask(void *pvParameters)
{
  for(;;)
  {
    gSecondsCounter++;
    printf("seconds: %d\n", gSecondsCounter);
    vTaskDelay(1000);
  }
}

/*******************************************************************************
 **
 ** @brief  Application entry point. SystemInit() has already been called
 **
 ******************************************************************************/
int main(void)
{
  // static uint64_t nextSecTask_ms = 0;

  // Initialize HAL functions (clock, IO config, debug prints, etc)
  uint8_t initerrs = Initialize();

  if(initerrs)
  {
    printf("\nInit errors = %d\n", initerrs);
  }
  
  /* will print to UART0 on PG0/PG1 at 115k baud 8n1
  ** or to RTT terminal if 'ENABLE_RTT' defined in board.h */
  printf("\nHello World!\n"); 

  xTaskCreate(BlinkingTask, "Task 1", 128, NULL, 1, NULL);
  xTaskCreate(SecondsTask,  "Task 2", 128, NULL, 1, NULL);

  // Start FreeRTOS Scheduler
  vTaskStartScheduler();
}

/*******************************************************************************
 **
 ** @brief  loss of clock handler - now running on internal osc
 **
 ******************************************************************************/
void LoCLK_IRQHandler(void)
{
  // rearm sysclk lost det
  (void)HAL_Clkgen_Rearm();

  SystemCoreClockUpdate(); // calculate new SystemCoreClock
  OnSystemClockChanged(); // update peripherals
  dbgprintln("loclk");
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
  UartInit(VOR_UART2, UART_BAUDRATE);
  dbgprintln("new sysclk: %d", SystemCoreClock);
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
  dbgprintln("EDAC MBE Handler");

  // wait for print to complete
  for(volatile int i=0; i<1000000; i++){}

  // reset
  NVIC_SystemReset();
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
  dbgprintln("EDAC SBE Handler");
}

/*******************************************************************************
 **
 ** @brief  Test NMI
 **
 ******************************************************************************/
void NMI_Handler(void)
{
  dbgprintln("NMI Handler");
}

/*****************************************************************************/
/* End of file                                                               */
/*****************************************************************************/
