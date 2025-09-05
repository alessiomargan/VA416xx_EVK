/***************************************************************************************
 * @file     main.c
 * @version  V2.05
 * @date     31 January 2024
 *
 * @note
 * VORAGO Technologies
 *
 * @note
 * Copyright (c) 2013-2024 VORAGO Technologies.
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
#include "uart.h"
#include "can.h"
#include "ads1278.h"

#include "va416xx_hal.h"
#include "va416xx_hal_clkgen.h"
#include "va416xx_hal_irqrouter.h"
#include "va416xx_hal_timer.h"
#include "va416xx_hal_dma.h"
#include "va416xx_debug.h"
#include "spi_fram.h"

/*****************************************************************************/
/* Local pre-processor symbols/macros ('#define')                            */
/*****************************************************************************/

#define BLINK_TIMER_NUM  (0)
#define BLINK_TIMER_MS   (200)
#define BLINK_TIMER_PRIO (7)

#define RESET_PERIPHERALS() do{                   \
  VOR_SYSCONFIG->PERIPHERAL_RESET = 0x00000000UL; \
  __NOP();                                        \
  __NOP();                                        \
  VOR_SYSCONFIG->PERIPHERAL_RESET = 0xFFFFFFFFUL; \
} while(0)

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
#define WDOG_MS (50U)
#ifdef ENABLE_WATCHDOG
static void EnableWatchdog(void)
{
  VOR_SYSCONFIG->PERIPHERAL_CLK_ENABLE |= CLK_ENABLE_WDOG;
  VOR_SYSCONFIG->PERIPHERAL_RESET &= ~SYSCONFIG_PERIPHERAL_RESET_WDOG_Msk;
  __NOP();
  __NOP();
  VOR_SYSCONFIG->PERIPHERAL_RESET |= SYSCONFIG_PERIPHERAL_RESET_WDOG_Msk;
  VOR_WATCH_DOG->WDOGLOAD = (SystemCoreClock/4U/1000U)*WDOG_MS;
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
  uint8_t initerrs = 0;

    RESET_PERIPHERALS();

#ifdef ENABLE_RTT
  // stdio/stderr -> Segger RTT (if ENABLE_RTT defined in board.h)
  DBG_SetStdioOutput(en_stdio_rtt);
#endif

  // Configure CLKGEN
  clkgen_status = HAL_Clkgen_PLL(CLK_CTRL0_XTAL_N_PLL2P5X);
  //clkgen_status = HAL_Clkgen_Init(CLK_CFG_X_OSC_PLL80);

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

  ConfigureCAN0();
  ConfigureADS1278();

  // Debug prints and printf destination
  // Will route printf to a UART if RTT is not enabled
#ifndef ENABLE_RTT
  DBG_SetStdioOutput(en_stdio_uart0);
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
 ** @brief  Application entry point. SystemInit() has already been called
 **
 ******************************************************************************/
int main(void)
{
  static uint64_t nextSecTask_ms = 0;
  static ads1278_data_t data;

  // Initialize HAL functions (clock, IO config, debug prints, etc)
  uint8_t initerrs = Initialize();
  if(initerrs)
  {
    printf("\nInit errors = %d\n", initerrs);
  }

  /* will print to UART0 on PG0/PG1 at 115k baud 8n1
  ** or to RTT terminal if 'ENABLE_RTT' defined in board.h */
  printf("\r\nHello World!\r\n");

  // setup timer 0 to interrupt every 0.5 second (blinks PG5 LED)
  (void)HAL_Timer_SetupPeriodicIrqMs(BLINK_TIMER_NUM, BLINK_TIMER_MS, BLINK_TIMER_PRIO);
  while(1)
  {
    if(HAL_time_ms >= nextSecTask_ms){
      gSecondsCounter++;
      nextSecTask_ms += 1000;
      //printf("seconds: %d\r\n", gSecondsCounter);
      ADS1278_ReadAllChannels(&data);
    }

    WDFEED();
    __DSB();
    __WFI(); // wait for character received or next SysTick
  }
}

/*******************************************************************************
 **
 ** @brief  Timer0 IRQ Handler - blinks the led
 **
 ******************************************************************************/
void TIM0_IRQHandler(void)
{
  EVK_LED_PORT->TOGOUT = 1<<EVK_LED_PIN;
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
  dbgprintln("loclk\r\n");
}

/*******************************************************************************
 **
 ** @brief  Called when system clock has changed - re-init peripherals
 **
 ******************************************************************************/
void OnSystemClockChanged(void)
{
  UartInit(VOR_UART0, UART_BAUDRATE);
  (void)HAL_SysTick_Init(SYSTICK_INTERVAL_MS, SYSTICK_PRIORITY);
  (void)HAL_Timer_SetupPeriodicIrqMs(BLINK_TIMER_NUM, BLINK_TIMER_MS, BLINK_TIMER_PRIO);
  dbgprintln("new sysclk: %d\r\n", SystemCoreClock);
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
  dbgprintln("EDAC MBE Handler\r\n");

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
  dbgprintln("EDAC SBE Handler\r\n");
}

/*******************************************************************************
 **
 ** @brief  Test NMI
 **
 ******************************************************************************/
void NMI_Handler(void)
{
  dbgprintln("NMI Handler\r\n");
}

/*****************************************************************************/
/* End of file                                                               */
/*****************************************************************************/
