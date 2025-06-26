/***************************************************************************************
 * @file     va416xx_hal_clkgen.h
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
#ifndef __HAL_CLKGEN_H
#define __HAL_CLKGEN_H 

/*****************************************************************************/ 
/* Include files                                                             */ 
/*****************************************************************************/

#include "va416xx_hal.h"

/*****************************************************************************/ 
/* Global pre-processor symbols/macros ('#define')                           */ 
/*****************************************************************************/

#define HAL_CLKGEN_VERSION (0x00020005) // 2.05

#define PLLOUT_MIN_HZ     (7000000UL)
#define PLLOUT_MAX_HZ     (100000000UL)

/** Common sys clock configuration macros to use with HAL_Clkgen_Init() */
#define CLK_CFG_XTAL_N      (hal_clkgen_init_t){.xtalsel = hal_xtalsel_xtal_n, \
                                                .clksel = hal_clksel_sys_xtal_n, \
                                                .pllcfg = hal_pllcfg_pwrdn, \
                                                .clk_div_sel = hal_clk_div_1x, \
                                                .lost_det_en = true}

#define CLK_CFG_XTAL_OSC    (hal_clkgen_init_t){.xtalsel = hal_xtalsel_xtal_osc, \
                                                .clksel = hal_clksel_sys_xtal_osc, \
                                                .pllcfg = hal_pllcfg_pwrdn, \
                                                .clk_div_sel = hal_clk_div_1x, \
                                                .lost_det_en = true}

#define CLK_CFG_HBO         (hal_clkgen_init_t){.xtalsel = hal_xtalsel_none, \
                                                .clksel = hal_clksel_sys_hbo, \
                                                .pllcfg = hal_pllcfg_pwrdn, \
                                                .clk_div_sel = hal_clk_div_1x, \
                                                .lost_det_en = true}

#define CLK_CFG_XTAL_N_PLL80 (hal_clkgen_init_t){.xtalsel = hal_xtalsel_xtal_n, \
                                                .clksel = hal_clksel_sys_pll, \
                                                .pllcfg = hal_pllcfg_enabled, \
                                                .clk_div_sel = hal_clk_div_1x, \
                                                .lost_det_en = true, \
                                                .pll_out_hz = 80000000UL}

#define CLK_CFG_X_OSC_PLL80  (hal_clkgen_init_t){.xtalsel = hal_xtalsel_xtal_osc, \
                                                .clksel = hal_clksel_sys_pll, \
                                                .pllcfg = hal_pllcfg_enabled, \
                                                .clk_div_sel = hal_clk_div_1x, \
                                                .lost_det_en = true, \
                                                .pll_out_hz = 80000000UL}

#define CLK_CFG_XTAL_N_PLL100 (hal_clkgen_init_t){.xtalsel = hal_xtalsel_xtal_n, \
                                                .clksel = hal_clksel_sys_pll, \
                                                .pllcfg = hal_pllcfg_enabled, \
                                                .clk_div_sel = hal_clk_div_1x, \
                                                .lost_det_en = true, \
                                                .pll_out_hz = 100000000UL}

#define CLK_CFG_X_OSC_PLL100  (hal_clkgen_init_t){.xtalsel = hal_xtalsel_xtal_osc, \
                                                .clksel = hal_clksel_sys_pll, \
                                                .pllcfg = hal_pllcfg_enabled, \
                                                .clk_div_sel = hal_clk_div_1x, \
                                                .lost_det_en = true, \
                                                .pll_out_hz = 100000000UL}

#define CLK_CTRL0_XTAL_OSC_PLL5X (0x40DAB611UL) /* 10MHz -> 50MHz */
#define CLK_CTRL0_XTAL_N_PLL2P5X (0x4161181AUL) /* 40MHz -> 100MHz */

/*****************************************************************************/
/* Global type definitions ('typedef')                                       */ 
/*****************************************************************************/

/** XTAL mode and select input to PLL (xtalsel) */
typedef enum
{
  hal_xtalsel_none,
  hal_xtalsel_xtal_osc,
  hal_xtalsel_xtal_n
} hal_xtalsel_t;

/** Clock select to system clock (clksel) */
typedef enum
{
  hal_clksel_sys_hbo,
  hal_clksel_sys_xtal_n,
  hal_clksel_sys_pll,
  hal_clksel_sys_xtal_osc
} hal_clksel_sys_t;

/** PLL setting (pllcfg) */
typedef enum
{
  hal_pllcfg_pwrdn,
  hal_pllcfg_enabled,
  hal_pllcfg_bypass
} hal_pllcfg_t;

/** System clock divider (clk_div_sel) */
typedef enum
{
  hal_clk_div_1x = 0,
  hal_clk_div_2x,
  hal_clk_div_4x,
  hal_clk_div_8x
} hal_clk_div_t;

/** The main init struct */
typedef struct
{
  hal_xtalsel_t    xtalsel;
  hal_clksel_sys_t clksel;
  hal_pllcfg_t     pllcfg;
  hal_clk_div_t    clk_div_sel;
  bool             lost_det_en;
  uint32_t         pll_out_hz;
} hal_clkgen_init_t;

/*****************************************************************************/ 
/* Global function prototypes ('extern', definition in C source)             */ 
/*****************************************************************************/

// Note: makes code size very large. But most full featured. Also useful
// to run once to get CTRL0 register value, then use HAL_Clkgen_PLL()
extern hal_status_t HAL_Clkgen_Init(hal_clkgen_init_t clkConfig);

// lean, fast clockgen init routines (these enable sysclk_lost detection and interrupt)
extern hal_status_t HAL_Clkgen_XtalN(void);
extern hal_status_t HAL_Clkgen_XtalOsc(void);
extern hal_status_t HAL_Clkgen_PLL(uint32_t u32Ctrl0Val);

// rearm sysclk lost detection
extern hal_status_t HAL_Clkgen_Rearm(void);

/*****************************************************************************/ 
/* End of file                                                               */ 
/*****************************************************************************/
#endif /* __HAL_CLKGEN_H */

