/***************************************************************************************
 * @file     i2c_lis2de12.h
 * @version  V1.2
 * @date     17 June 2022
 *
 * @note
 * VORAGO Technologies
 *
 * @note
 * Copyright (c) 2013-2022 VORAGO Technologies.
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
 
#ifndef __I2C_LIS2DE12_H
#define __I2C_LIS2DE12_H

/*****************************************************************************/ 
/* Include files                                                             */ 
/*****************************************************************************/

#include "accel_generic.h"

/*****************************************************************************/ 
/* Global pre-processor symbols/macros ('#define')                           */ 
/*****************************************************************************/

/*****************************************************************************/
/* Global type definitions ('typedef')                                       */ 
/*****************************************************************************/

/*****************************************************************************/
/* Global variable declarations ('extern', definition in C source)           */
/*****************************************************************************/

/*****************************************************************************/ 
/* Global function prototypes ('extern', definition in C source)             */ 
/*****************************************************************************/

extern hal_status_t LIS2DE12_InitI2C(stc_accel_handle_t * const handle);
extern hal_status_t LIS2DE12_GetAccelRawValues(stc_accel_handle_t * const handle, \
  stc_accel_raw_values_t* const accelDataRaw);
extern hal_status_t LIS2DE12_GetAccelMgValues(stc_accel_handle_t * const handle, \
  stc_accel_mg_values_t* const accelDataMg);
extern hal_status_t LIS2DE12_GetBoardTemp(stc_accel_handle_t * const handle, float* const temp);
extern hal_status_t LIS2DE12_UnInit(stc_accel_handle_t * const handle);

/*****************************************************************************/ 
/* End of file                                                               */ 
/*****************************************************************************/
#endif /* __I2C_LIS2DE12_H */
