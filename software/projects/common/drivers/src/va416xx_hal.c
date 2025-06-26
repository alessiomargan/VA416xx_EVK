/***************************************************************************************
 * @file     va416xx_hal.c
 * @version  V2.04
 * @date     25 April 2023
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

#include "va416xx_hal.h"
#include "va416xx_hal_ioconfig.h"
#include "va416xx_hal_irqrouter.h"
#include "va416xx_debug.h"

/*****************************************************************************/ 
/* Local pre-processor symbols/macros ('#define')                            */ 
/*****************************************************************************/

#define SYSCONFIG_PROCID (0x040057E3)
#define SYSCONFIG_PERID  (0x028007E9)

/*****************************************************************************/ 
/* Global variable definitions (declared in header file with 'extern')       */ 
/*****************************************************************************/

volatile uint64_t HAL_time_ms = 0;  // millisecond tick counter (64-bit)
volatile bool newSysTick = false;

const char * HALStatusStrArr[(uint32_t)hal_status_end+1] = 
{
  "ok",             //hal_status_ok             = 0, 
  "initError",      //hal_status_initError      = 1,
  "badParam",       //hal_status_badParam       = 2,
  "notInitialized", //hal_status_notInitialized = 3,
  "badPeriphID",    //hal_status_badPeriphID    = 4,
  "timeout",        //hal_status_timeout        = 5,
  "rxError",        //hal_status_rxError        = 6,
  "txError",        //hal_status_txError        = 7,
  "bufEmpty",       //hal_status_bufEmpty       = 8,
  "bufFull",        //hal_status_bufFull        = 9,
  "nak",            //hal_status_nak            = 10,
  "arblost",        //hal_status_arblost        = 11,
  "busy",           //hal_status_busy           = 12,
  "notImplemented", //hal_status_notImplemented = 13,
  "alignmentErr",   //hal_status_alignmentErr   = 14,
  "periphErr",      //hal_status_periphErr      = 15,
  "crcErr",         //hal_status_crcErr         = 16,
  "",               //all other values
};

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
 ** @brief  Initialize HAL framework
 **
 ******************************************************************************/
hal_status_t HAL_Init(void)
{
  hal_status_t status;
  
  // Check sysconfig peripheral
  c_assert(SYSCONFIG_PROCID == VOR_SYSCONFIG->PROCID);
  c_assert(SYSCONFIG_PERID == VOR_SYSCONFIG->PERID);
  
  // Enable clock gating to GPIO and critical periphs
  VOR_SYSCONFIG->PERIPHERAL_CLK_ENABLE =
        (CLK_ENABLE_PORTA | CLK_ENABLE_PORTB | CLK_ENABLE_PORTC |
        CLK_ENABLE_PORTD | CLK_ENABLE_PORTE | CLK_ENABLE_PORTF |
        CLK_ENABLE_PORTG | CLK_ENABLE_IOCONFIG | CLK_ENABLE_CLKGEN);
  
  // IRQ router init
  status = HAL_Irqrouter_Init();
  c_assert(status == hal_status_ok);
  if(status != hal_status_ok){
    return hal_status_initError;
  }
  
#ifndef __HAL_DISABLE_SYSTICK
  // SysTick setup
  status = HAL_SysTick_Init(SYSTICK_INTERVAL_MS, SYSTICK_PRIORITY);
  c_assert(status == hal_status_ok);
  if(status != hal_status_ok){
    return hal_status_initError;
  }
#endif

  return hal_status_ok;
}

/*******************************************************************************
 **
 ** @brief  Initialize SysTick counter
 **
 ******************************************************************************/
hal_status_t HAL_SysTick_Init(uint32_t intervalMs, uint32_t priority)
{
  uint32_t ticks = ((SystemCoreClock/1000UL)*intervalMs)-1UL;
  if(ticks > SysTick_LOAD_RELOAD_Msk)
  {
    return hal_status_badParam;                          /* Reload value impossible */
  }
  if(priority > HAL_NVIC_MAX_PRIO)
  {
    return hal_status_badParam;                          /* Priority value impossible */
  }
  SysTick->LOAD  = ticks;                                /* set reload register */
  NVIC_SetPriority(SysTick_IRQn, priority);              /* set Priority for Systick Interrupt */
  SysTick->VAL   = 0UL;                                  /* Load the SysTick Counter Value */
  SysTick->CTRL  = SysTick_CTRL_CLKSOURCE_Msk |
                   SysTick_CTRL_TICKINT_Msk   |
                   SysTick_CTRL_ENABLE_Msk;              /* Enable SysTick IRQ and SysTick Timer */
  return hal_status_ok;
}

/*******************************************************************************
 **
 ** @brief  Returns a string version of the hal_status_t status
 **
 ******************************************************************************/
const char * HAL_StatusToString(hal_status_t stat)
{
  if(stat > hal_status_end){ stat = hal_status_end; }
  return HALStatusStrArr[(uint32_t)stat];
}

/*******************************************************************************
 **
 ** @brief  Delay (sleep) a specified number of milliseconds
 **
 ******************************************************************************/
void HAL_WaitMs(const uint32_t ms)
{
  uint64_t waitEnd_ms = (uint64_t)(HAL_time_ms + (uint64_t)ms);
  while(HAL_time_ms < waitEnd_ms){
    WDFEED();
    __DMB();
    __WFI(); /* sleep until next tick or other interrupt */
  }
  WDFEED();
}

/*******************************************************************************
 **
 ** @brief  Returns HAL_time_ms with interrupt blocking
 **
 ******************************************************************************/
uint64_t HAL_GetTimeMs(void)
{  
  __ASM("cpsid if");                            /* Disable interrupts */
  uint64_t retval = HAL_time_ms;              /* 64 bit read not atomic so needs interrupts off */
  __ASM("cpsie if");                            /* Enable interrupts */
  return retval;
}

/*******************************************************************************
 **
 ** @brief  SysTick handler. Increments HAL_time_ms
 **
 ** @note   To disable this ISR (defined elsewhere), define __HAL_DISABLE_SYSTICK
 **         in hal_config.h
 **
 ******************************************************************************/
#ifndef __HAL_DISABLE_SYSTICK
void SysTick_Handler(void)
{
  __ASM("cpsid if");                            /* Disable interrupts */
  HAL_time_ms += SYSTICK_INTERVAL_MS;         /* not atomic so needs interrupts off */
  newSysTick = true;
  __ASM("cpsie if");                            /* Enable interrupts */
}
#endif

/*****************************************************************************/ 
/* End of file                                                               */ 
/*****************************************************************************/
