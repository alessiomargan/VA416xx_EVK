/***************************************************************************************
 * @file     main.c
 * @version  V2.05
 * @date     06 March 2024
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
#include "hal_config.h"
#include "va416xx_hal.h"
#include "va416xx_hal_adc.h"
#include "va416xx_hal_clkgen.h"
#include "va416xx_hal_dac.h"
#include "va416xx_hal_dma.h"
#include "va416xx_hal_irqrouter.h"
#include "va416xx_hal_uart.h"
#include "va416xx_hal_timer.h"
#include "va416xx_hal_canbus.h"
#include "va416xx_debug.h"

#include "cmd_interface.h"
#include "spi_fram.h"

/*****************************************************************************/
/* Local pre-processor symbols/macros ('#define')                            */
/*****************************************************************************/

#define BLINK_TIMER_NUM  (0)
#define BLINK_TIMER_MS   (500)
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
hal_xtalsel_t gCurrentXtalsel;

/*****************************************************************************/
/* Local type definitions ('typedef')                                        */
/*****************************************************************************/

/*****************************************************************************/
/* Local variable definitions ('static')                                     */
/*****************************************************************************/

/*****************************************************************************/
/* Local function prototypes ('static')                                      */
/*****************************************************************************/

static uint8_t Initialize(void);

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
  VOR_SYSCONFIG->ROM_SCRUB = 500;

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
  uint8_t initerrs = 0;

  RESET_PERIPHERALS();

#ifdef ENABLE_RTT
  DBG_SetStdioOutput(en_stdio_rtt); // stdio/stderr -> Segger RTT (if RTT enabled in board.h)
#endif

  // Configure CLKGEN
  gCurrentXtalsel = hal_xtalsel_xtal_n;
  clkgen_status = HAL_Clkgen_Init((hal_clkgen_init_t)
  {
    .xtalsel = gCurrentXtalsel, \
    .clksel = hal_clksel_sys_pll, \
    .pllcfg = hal_pllcfg_enabled, \
    .clk_div_sel = hal_clk_div_1x, \
    .lost_det_en = true, \
    .pll_out_hz = 100000000UL
  });
  SystemCoreClockUpdate();
  NVIC_EnableIRQ(LoCLK_IRQn); // enable IRQ on loss of clock

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
  
#ifdef LOG_UART0
  // setup UART0 (PG0, PG1)
  status = HAL_Uart_Init(VOR_UART0, UART_CFG_115K_8N1);
  if(status != hal_status_ok)
  {
    initerrs++;
  }
  DBG_SetStdioOutput(en_stdio_uart0);
  //HAL_Uart_SetRXIntCallback(VOR_UART0, eventHandlerIdx_u0ExampleCall); // rx calls U0_ExampleCallback()
#endif
#ifdef LOG_UART1
  // setup UART1 (PB12-15, PMOD UART J36)
  status = HAL_Uart_Init(VOR_UART1, UART_CFG_115K_8N1);
  if(status != hal_status_ok)
  {
    initerrs++;
  }
  DBG_SetStdioOutput(en_stdio_uart1);
#endif
#ifdef LOG_UART2
  // setup UART2 (PF6-9, PMOD UART J37)
  status = HAL_Uart_Init(VOR_UART2, UART_CFG_115K_8N1);
  if(status != hal_status_ok)
  {
    initerrs++;
  }
  DBG_SetStdioOutput(en_stdio_uart2);
#endif

  // clkgen report (most likely fail is PLL failed to lock, if no/bad external clk)
  if(clkgen_status != hal_status_ok)
  {
    initerrs++;
    dbgprintln("CLKGEN status: %s", HAL_StatusToString(clkgen_status));
  }

  // ADC, DAC
  HAL_ADC_Init();
  HAL_DAC_Reset();
  status = HAL_DAC_Init(VOR_DAC0);
  if(status != hal_status_ok)
  {
    dbgprintln("DAC0 init error, status code: %d", status);
    initerrs++;
  }
  status = HAL_DAC_Init(VOR_DAC1);
  if(status != hal_status_ok)
  {
    dbgprintln("DAC1 init error, status code: %d", status);
    initerrs++;
  }

  // DMA
  status = HAL_DMA_Init(NULL, false, false, false);
  if(status != hal_status_ok)
  {
    dbgprintln("DMA init error, status code: %d", status);
    initerrs++;
  }

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

  // Initialize HAL functions (clock, IO config, debug prints, etc)
  uint8_t initerrs = Initialize();

  dbgprintln( "Built:"" %s @ %s" , __DATE__, __TIME__);

  // Welcome message
  printf("\nWelcome to %s EVK\n", DEVICE_NAME);
  printf("Software version: %s\n", SOFTWARE_VERSION_STR);
  printf("System clock: %d\n", SystemCoreClock);
  printf("PROCID: 0x%08x\n", VOR_SYSCONFIG->PROCID);
  printf("ID0: 0x%08x\n", VOR_SYSCONFIG->EF_ID0);
  printf("ID1: 0x%08x\n", VOR_SYSCONFIG->EF_ID1);
  printf("Type '$help' for help\n");

  if(initerrs)
  {
    dbgprintln("initialize() num errors: %d", initerrs);
  }

  // setup timer to blink LED
  (void)HAL_Timer_SetupPeriodicIrqMs(BLINK_TIMER_NUM, \
    BLINK_TIMER_MS, BLINK_TIMER_PRIO);

   while(1)
  {
    if(HAL_time_ms >= nextSecTask_ms)
    {
      OnceEverySecondTasks();
      gSecondsCounter++;
      nextSecTask_ms += 1000;
    }
    CommandInterfaceMainTask();
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
  dbgprintln("loclk");
}

/*******************************************************************************
 **
 ** @brief  Called when system clock has changed - re-init peripherals
 **
 ******************************************************************************/
void OnSystemClockChanged(void)
{
#ifdef LOG_UART0
  HAL_Uart_Init(VOR_UART0, UART_CFG_115K_8N1);
#endif
#ifdef LOG_UART1
  HAL_Uart_Init(VOR_UART1, UART_CFG_115K_8N1);
#endif
#ifdef LOG_UART2
  HAL_Uart_Init(VOR_UART2, UART_CFG_115K_8N1);
#endif
  HAL_SysTick_Init(SYSTICK_INTERVAL_MS, SYSTICK_PRIORITY);
  HAL_ADC_Init();
  HAL_Timer_SetupPeriodicIrqMs(BLINK_TIMER_NUM, BLINK_TIMER_MS, BLINK_TIMER_PRIO);
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

/*******************************************************************************
 **
 ** @brief  Uart0 example callback (called on character received)
 **
 ******************************************************************************/
void U0_ExampleCallback(void)
{
  uint8_t c;
  HAL_Uart_RxByte(VOR_UART0, &c);
  dbgprintln("U0_ExampleCallback(): '%c'", c);
}

/*****************************************************************************/
/* End of file                                                               */
/*****************************************************************************/
