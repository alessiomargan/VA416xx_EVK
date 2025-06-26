/***************************************************************************************
 * @file     va416xx_hal_adc.h
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
 
#ifndef __HAL_ADC_H
#define __HAL_ADC_H

/*****************************************************************************/ 
/* Include files                                                             */ 
/*****************************************************************************/

#include "va416xx_hal.h"

/*****************************************************************************/ 
/* Global pre-processor symbols/macros ('#define')                           */ 
/*****************************************************************************/

/** Version */
#define HAL_ADC_VERSION (0x00020005) // 2.05

/** ADC Peripheral clock source (digital side, registers) */
#define ADC_REGISTER_CLK (APB2_CLK)

/** CLKGEN->CTRL1[6:5] */
#define ADC_CLK_DIV_BY_ONE    0x60
#define ADC_CLK_DIV_BY_TWO    0x40
#define ADC_CLK_DIV_BY_FOUR   0x20
#define ADC_CLK_DIV_BY_EIGHT  0x0

#define ADC_NUM_CHANNELS      (16)
#define ADC_MAX_COUNT         (4095)

/** ADC Channel bit definitions
    ADC External inputs */
#define ADC_IN0         0x0001
#define ADC_IN1         0x0002
#define ADC_IN2         0x0004
#define ADC_IN3         0x0008
#define ADC_IN4         0x0010
#define ADC_IN5         0x0020
#define ADC_IN6         0x0040
#define ADC_IN7         0x0080
/** ADC Internal inputs */
#define ADC_DAC0        0x0100
#define ADC_DAC1        0x0200
#define ADC_TEMP_SENSE  0x0400
#define ADC_BG_1P0      0x0800
#define ADC_BG_1P5      0x1000
#define ADC_AVDD15      0x2000
#define ADC_DVDD15      0x4000
#define ADC_VREFP5      0x8000

/*****************************************************************************/
/* Global type definitions ('typedef')                                       */ 
/*****************************************************************************/

/** ADC_CTRL register in struct form (packed bitfield) */
typedef struct
{
  uint32_t chan_en     :16; // see channel defs above, multiple can be OR'ed together
  uint32_t chan_tag_en :1;  // adds channel number in result[15:12], useful for sweep
  uint32_t sweep_en    :1;  // set to true if multiple channels in chan_en
  uint32_t ext_trig_en :1;
  uint32_t manual_trig :1;
  uint32_t conv_cnt_m1 :4;  // num conversions-1 (1-16 conversions per channel)
  uint32_t reserved    :8;
} stc_adc_ctrl_t;

/** ADC_CTRL register union */
typedef union 
{
  stc_adc_ctrl_t ctrl;
  uint32_t ctrl_raw;
} un_adc_ctrl_t;


/*****************************************************************************/
/* Global variable declarations ('extern', definition in C source)           */
/*****************************************************************************/

/*****************************************************************************/ 
/* Global function prototypes ('extern', definition in C source)             */ 
/*****************************************************************************/

extern hal_status_t HAL_ADC_Init(void);
extern hal_status_t HAL_ADC_DeInit(void);
extern hal_status_t HAL_ADC_FIFO_Clear(void);
extern uint32_t     HAL_ADC_ReadCal(void);
extern hal_status_t HAL_ADC_FIFO_SetTrigLevel(uint32_t level);
extern hal_status_t HAL_ADC_SetExtTrigEnable(bool extTrig);
extern hal_status_t HAL_ADC_ReadTempSensorManualTrigger(uint32_t *result);
extern hal_status_t HAL_ADC_ReadSingle(uint8_t channelNum, uint16_t *result);
extern hal_status_t HAL_ADC_ManualTrigger(stc_adc_ctrl_t ctrl, uint16_t *result);
extern hal_status_t HAL_ADC_SetCtrl(const stc_adc_ctrl_t *ctrl);

/*****************************************************************************/ 
/* End of file                                                               */ 
/*****************************************************************************/
#endif /* __HAL_ADC_H */
