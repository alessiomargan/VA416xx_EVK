/***************************************************************************************
 * @file     dac_sine.c
 * @version  V2.05
 * @date     10 January 2024
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
#include "va416xx_hal.h"
#include "va416xx_hal_dac.h"
#include "va416xx_hal_irqrouter.h"
#include "va416xx_hal_dma.h"
#include "va416xx_hal_timer.h"
#include "va416xx_debug.h"

#include "dac_sine.h"

#ifdef INC_DAC_SINE
/*****************************************************************************/ 
/* Local pre-processor symbols/macros ('#define')                            */ 
/*****************************************************************************/

// defualt timer and DMA assignments
#ifndef DAC0_TIM_NUM
#define DAC0_TIM_NUM  (10)
#endif
#ifndef DAC1_TIM_NUM
#define DAC1_TIM_NUM  (11)
#endif
#ifndef DAC0_DMA_CH
#define DAC0_DMA_CH   (0)
#endif
#ifndef DAC1_DMA_CH
#define DAC1_DMA_CH   (1)
#endif

/*****************************************************************************/ 
/* Global variable definitions (declared in header file with 'extern')       */ 
/*****************************************************************************/

/*****************************************************************************/ 
/* Local type definitions ('typedef')                                        */ 
/*****************************************************************************/

/*****************************************************************************/ 
/* Local variable definitions ('static')                                     */ 
/*****************************************************************************/

// size 16
// generated using https://www.daycounter.com/Calculators/Sine-Generator-Calculator.phtml
// parameters: num points: 16, max amplitude: 4095 numbers per row: 8 hex/decimal: hex
static uint16_t sineTable16[16] = {
  0x800,0xb0f,0xda7,0xf63,0xfff,0xf63,0xda7,0xb0f,
  0x800,0x4f0,0x258,0x9c,0x0,0x9c,0x258,0x4f0};

// size 64
// generated using https://www.daycounter.com/Calculators/Sine-Generator-Calculator.phtml
// parameters: num points: 64, max amplitude: 4095 numbers per row: 8 hex/decimal: hex
static uint16_t sineTable64[64] = {
  0x800,0x8c8,0x98f,0xa52,0xb0f,0xbc5,0xc71,0xd12,
  0xda7,0xe2e,0xea6,0xf0d,0xf63,0xfa7,0xfd8,0xff5,
  0xfff,0xff5,0xfd8,0xfa7,0xf63,0xf0d,0xea6,0xe2e,
  0xda7,0xd12,0xc71,0xbc5,0xb0f,0xa52,0x98f,0x8c8,
  0x800,0x737,0x670,0x5ad,0x4f0,0x43a,0x38e,0x2ed,
  0x258,0x1d1,0x159,0xf2,0x9c,0x58,0x27,0xa,
  0x0,0xa,0x27,0x58,0x9c,0xf2,0x159,0x1d1,
  0x258,0x2ed,0x38e,0x43a,0x4f0,0x5ad,0x670,0x737};

// size 256
// generated using https://www.daycounter.com/Calculators/Sine-Generator-Calculator.phtml
// parameters: num points: 256, max amplitude: 4095 numbers per row: 8 hex/decimal: hex
static uint16_t sineTable256[256] = {
  0x800,0x832,0x864,0x896,0x8c8,0x8fa,0x92c,0x95e,
  0x98f,0x9c0,0x9f1,0xa22,0xa52,0xa82,0xab1,0xae0,
  0xb0f,0xb3d,0xb6b,0xb98,0xbc5,0xbf1,0xc1c,0xc47,
  0xc71,0xc9a,0xcc3,0xceb,0xd12,0xd39,0xd5f,0xd83,
  0xda7,0xdca,0xded,0xe0e,0xe2e,0xe4e,0xe6c,0xe8a,
  0xea6,0xec1,0xedc,0xef5,0xf0d,0xf24,0xf3a,0xf4f,
  0xf63,0xf76,0xf87,0xf98,0xfa7,0xfb5,0xfc2,0xfcd,
  0xfd8,0xfe1,0xfe9,0xff0,0xff5,0xff9,0xffd,0xffe,
  0xfff,0xffe,0xffd,0xff9,0xff5,0xff0,0xfe9,0xfe1,
  0xfd8,0xfcd,0xfc2,0xfb5,0xfa7,0xf98,0xf87,0xf76,
  0xf63,0xf4f,0xf3a,0xf24,0xf0d,0xef5,0xedc,0xec1,
  0xea6,0xe8a,0xe6c,0xe4e,0xe2e,0xe0e,0xded,0xdca,
  0xda7,0xd83,0xd5f,0xd39,0xd12,0xceb,0xcc3,0xc9a,
  0xc71,0xc47,0xc1c,0xbf1,0xbc5,0xb98,0xb6b,0xb3d,
  0xb0f,0xae0,0xab1,0xa82,0xa52,0xa22,0x9f1,0x9c0,
  0x98f,0x95e,0x92c,0x8fa,0x8c8,0x896,0x864,0x832,
  0x800,0x7cd,0x79b,0x769,0x737,0x705,0x6d3,0x6a1,
  0x670,0x63f,0x60e,0x5dd,0x5ad,0x57d,0x54e,0x51f,
  0x4f0,0x4c2,0x494,0x467,0x43a,0x40e,0x3e3,0x3b8,
  0x38e,0x365,0x33c,0x314,0x2ed,0x2c6,0x2a0,0x27c,
  0x258,0x235,0x212,0x1f1,0x1d1,0x1b1,0x193,0x175,
  0x159,0x13e,0x123,0x10a,0xf2,0xdb,0xc5,0xb0,
  0x9c,0x89,0x78,0x67,0x58,0x4a,0x3d,0x32,
  0x27,0x1e,0x16,0xf,0xa,0x6,0x2,0x1,
  0x0,0x1,0x2,0x6,0xa,0xf,0x16,0x1e,
  0x27,0x32,0x3d,0x4a,0x58,0x67,0x78,0x89,
  0x9c,0xb0,0xc5,0xdb,0xf2,0x10a,0x123,0x13e,
  0x159,0x175,0x193,0x1b1,0x1d1,0x1f1,0x212,0x235,
  0x258,0x27c,0x2a0,0x2c6,0x2ed,0x314,0x33c,0x365,
  0x38e,0x3b8,0x3e3,0x40e,0x43a,0x467,0x494,0x4c2,
  0x4f0,0x51f,0x54e,0x57d,0x5ad,0x5dd,0x60e,0x63f,
  0x670,0x6a1,0x6d3,0x705,0x737,0x769,0x79b,0x7cd};

static uint16_t *dac0buf;
static uint16_t *dac1buf;
static uint16_t dac0sz;
static uint16_t dac1sz;

/*****************************************************************************/ 
/* Local function prototypes ('static')                                      */ 
/*****************************************************************************/

static void DAC0_Dma_Callback(void* handle);
static void DAC1_Dma_Callback(void* handle);

/*****************************************************************************/ 
/* Function implementation - global ('extern') and local ('static')          */ 
/*****************************************************************************/

// Outputs a sine wave using lookup table. Uses DAC interrupts to fill the DAC FIFO, not DMA
hal_status_t DAC_SetupSine(uint32_t dacNum, uint32_t freqHz)
{
  uint32_t timerNum = (dacNum==0?DAC0_TIM_NUM:DAC1_TIM_NUM);
  uint32_t rstVal = TIMER_CLK(timerNum)/(freqHz*256);
  if(rstVal < TIMER_CLK(timerNum)/500000){
    rstVal = TIMER_CLK(timerNum)/(freqHz*64);
    if(rstVal < TIMER_CLK(timerNum)/500000){
      rstVal = TIMER_CLK(timerNum)/(freqHz*16);
      return DAC_LoopBuffer(dacNum, sineTable16, 16, rstVal);
    }
    return DAC_LoopBuffer(dacNum, sineTable64, 64, rstVal);
  }
  return DAC_LoopBuffer(dacNum, sineTable256, 256, rstVal);
}
  
// outputs a buffer to the DAC, looping at the end. tmrRstVal sets output rate. Uses interrupts
hal_status_t DAC_LoopBuffer(uint32_t dacNum, uint16_t* buf, uint32_t len, uint32_t tmrRstVal)
{
  VOR_DAC_Type * dac;
  
  // dac
  if(dacNum == 0){
    dac = VOR_DAC0;
    dac0buf = buf;
    dac0sz = len;
  } else if(dacNum == 1){
    dac = VOR_DAC1;
    dac1buf = buf;
    dac1sz = len;
  } else {
    return hal_status_badParam;
  }
  VOR_SYSCONFIG->PERIPHERAL_CLK_ENABLE |= CLK_ENABLE_DAC;
  dac->CTRL0 = 0;
  dac->CTRL1 = 0;
  dac->FIFO_CLR = 1;
  dac->IRQ_CLR = 0xf;
  dac->TXFIFOIRQTRG = 0x10;
  dac->CTRL0 |= DAC_CTRL0_EXT_TRIG_EN_Msk; // enable ext trigger
  dac->CTRL1 |= (1UL << DAC_CTRL1_DAC_EN_Pos) | (0UL << DAC_CTRL1_DAC_SETTLING_Pos);
  dac->IRQ_ENB |= DAC_IRQ_ENB_FIFO_DEPTH_TRIG_Msk; // enable fifo depth irq
  NVIC_EnableIRQ((IRQn_Type)(DAC0_IRQn+dacNum));
  
  // irq router
  uint32_t timerNum = (dacNum==0?DAC0_TIM_NUM:DAC1_TIM_NUM);
  VOR_SYSCONFIG->PERIPHERAL_CLK_ENABLE |= CLK_ENABLE_IRQ;
  HAL_Irqrouter_SetDacSel(dacNum, timerNum);
  
  // timer init (DAC output sample rate)
  HAL_Timer_InitTimer(timerNum, (stc_tim_cfg_t){
                                            .irq_en = true,
                                            .rst_value = tmrRstVal,
                                            .cnt_value = 0
                                            },
                                            true);
  
  return hal_status_ok;
}

// Outputs a sine wave using lookup table. Uses DMA to fill the DAC FIFO (less CPU intervention)
hal_status_t DAC_SetupSineDma(uint32_t dacNum, uint32_t freqHz)
{
  uint32_t timerNum = (dacNum==0?DAC0_TIM_NUM:DAC1_TIM_NUM);
  uint32_t rstVal = TIMER_CLK(timerNum)/(freqHz*256);
  if(rstVal < TIMER_CLK(timerNum)/500000){
    rstVal = TIMER_CLK(timerNum)/(freqHz*64);
    if(rstVal < TIMER_CLK(timerNum)/500000){
      rstVal = TIMER_CLK(timerNum)/(freqHz*16);
      return DAC_LoopBufferDma(dacNum, sineTable16, 16, rstVal);
    }
    return DAC_LoopBufferDma(dacNum, sineTable64, 64, rstVal);
  }
  return DAC_LoopBufferDma(dacNum, sineTable256, 256, rstVal);
}

// outputs a buffer to the DAC, looping at the end. tmrRstVal sets output rate. Uses DMA
hal_status_t DAC_LoopBufferDma(uint32_t dacNum, uint16_t* buf, uint32_t len, uint32_t tmrRstVal)
{
  hal_status_t status;
  VOR_DAC_Type * dac;
  uint32_t dmaNum = (dacNum==0?DAC0_DMA_CH:DAC1_DMA_CH);
  
  // dac
  if(dacNum == 0){
    dac = VOR_DAC0;
    dac0sz = len;
  } else if(dacNum == 1){
    dac = VOR_DAC1;
    dac1sz = len;
  } else {
    return hal_status_badParam;
  }
  VOR_SYSCONFIG->PERIPHERAL_CLK_ENABLE |= CLK_ENABLE_DAC;
  dac->CTRL0 = 0;
  dac->CTRL1 = 0;
  dac->FIFO_CLR = 1;
  dac->IRQ_CLR = 0xf;
  dac->TXFIFOIRQTRG = 0x10;
  dac->CTRL0 |= DAC_CTRL0_EXT_TRIG_EN_Msk; // enable ext trigger
  dac->CTRL1 |= (1UL << DAC_CTRL1_DAC_EN_Pos) | (0UL << DAC_CTRL1_DAC_SETTLING_Pos);
  
  // irq router
  uint32_t timerNum = (dacNum==0?DAC0_TIM_NUM:DAC1_TIM_NUM);
  VOR_SYSCONFIG->PERIPHERAL_CLK_ENABLE |= CLK_ENABLE_IRQ;
  HAL_Irqrouter_SetDacSel(dacNum, timerNum);
  HAL_Irqrouter_SetDmaSel(dmaNum, (en_irqr_dmasel_t)(en_irqr_dmasel_dac0+dacNum));
  
  // timer init (DAC output sample rate)
  HAL_Timer_InitTimer(timerNum, (stc_tim_cfg_t){
                                            .irq_en = true,
                                            .rst_value = tmrRstVal,
                                            .cnt_value = 0
                                            },
                                            true);
                                            
  // dma
  VOR_SYSCONFIG->PERIPHERAL_CLK_ENABLE |= CLK_ENABLE_DMA;
  status = HAL_DMA_SRAMtoPeriph16(buf, (uint16_t*)&dac->FIFO_DATA, len, dmaNum);
  if(status != hal_status_ok){ return status; }
  if(dacNum == 0)
  {
    HAL_DMA_RegDoneCallback(dmaNum, DAC0_Dma_Callback, NULL);
  }
  else
  {
    HAL_DMA_RegDoneCallback(dmaNum, DAC1_Dma_Callback, NULL);
  }
  VOR_DMA->CFG |= (0x1);
  
  // enable
  HAL_DMA_SetChannelEnable(dmaNum, true);
  dac->IRQ_ENB |= DAC_IRQ_ENB_FIFO_DEPTH_TRIG_Msk; // enable fifo depth irq
  NVIC_EnableIRQ((IRQn_Type)(DMA_DONE0_IRQn+dmaNum));
  
  return hal_status_ok;
}

// stops the DAC sine wave output
hal_status_t DAC_StopDacLoop(uint32_t dacNum)
{
  VOR_DAC_Type * dac;
  uint32_t dmaNum = (dacNum==0?DAC0_DMA_CH:DAC1_DMA_CH);
  if(dacNum == 0){
    dac = VOR_DAC0;
  } else if(dacNum == 1){
    dac = VOR_DAC1;
  } else {
    return hal_status_badParam;
  }
  DISABLE_TIMER(dacNum==0?DAC0_TIM_NUM:DAC1_TIM_NUM);
  HAL_DMA_SetChannelEnable(dmaNum, false);
  NVIC_DisableIRQ((IRQn_Type)(DAC0_IRQn+dacNum));
  NVIC_DisableIRQ((IRQn_Type)(DMA_DONE0_IRQn+dmaNum));
	dac->IRQ_ENB &= ~DAC_IRQ_ENB_FIFO_DEPTH_TRIG_Msk;
  dac->CTRL0 &= ~DAC_CTRL0_EXT_TRIG_EN_Msk;
  dac->FIFO_CLR = 1;
  dac->IRQ_CLR = 0xf;
  return hal_status_ok;
}

// for interrupt (non-DMA mode) - fills the FIFOS upon 1/2 full trigger
void DAC0_IRQHandler(void)
{
  static uint16_t off = 0;
  for(int i=0; i<16; i++){
    VOR_DAC0->FIFO_DATA = dac0buf[off++];
  }
  if(off == dac0sz){ off = 0; }
}

void DAC1_IRQHandler(void)
{
  static uint16_t off = 0;
  for(int i=0; i<16; i++){
    VOR_DAC1->FIFO_DATA = dac1buf[off++];
  }
  if(off == dac1sz){ off = 0; }
}

// for DMA mode - restart the DMA cycle
static void DAC0_Dma_Callback(void* handle)
{
  HAL_DMA_RestartPriBasic(DAC0_DMA_CH, dac0sz);
}

static void DAC1_Dma_Callback(void* handle)
{
  HAL_DMA_RestartPriBasic(DAC1_DMA_CH, dac1sz);
}

#endif
/*****************************************************************************/ 
/* End of file                                                               */ 
/*****************************************************************************/
