/***************************************************************************************
 * @file     accel_generic.h
 * @version  V1.0
 * @date     26 April 2022
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
 
#ifndef __ACCEL_GENERIC_H
#define __ACCEL_GENERIC_H

/*****************************************************************************/ 
/* Include files                                                             */ 
/*****************************************************************************/

#include "va416xx_hal.h"
#include "va416xx_hal_i2c.h"

/*****************************************************************************/ 
/* Global pre-processor symbols/macros ('#define')                           */ 
/*****************************************************************************/

/*****************************************************************************/
/* Global type definitions ('typedef')                                       */ 
/*****************************************************************************/

typedef struct stc_accel_raw_values {
  int16_t x;
  int16_t y;
  int16_t z;
} stc_accel_raw_values_t;

typedef struct stc_accel_mg_values {
  float x;
  float y;
  float z;
} stc_accel_mg_values_t;

typedef enum en_accel_serial {
  en_accel_serial_i2c = 0
} en_accel_serial_t;

typedef struct stc_accel_handle{
  VOR_I2C_Type *i2c;
  i2caddr_shifted_t i2cAddr;
  en_accel_serial_t serialType;
  bool isInit;
} stc_accel_handle_t;

typedef struct stc_accel_interf{
  hal_status_t (*init)(stc_accel_handle_t* const);
  hal_status_t (*getRawValues)(stc_accel_handle_t* const, stc_accel_raw_values_t* const);
  hal_status_t (*getMgValues)(stc_accel_handle_t* const, stc_accel_mg_values_t* const);
  hal_status_t (*getTemp)(stc_accel_handle_t* const, float* const);
  hal_status_t (*unInit)(stc_accel_handle_t* const);
} stc_accel_interf_t;

/*****************************************************************************/
/* Global variable declarations ('extern', definition in C source)           */
/*****************************************************************************/

/*****************************************************************************/ 
/* Global function prototypes ('extern', definition in C source)             */ 
/*****************************************************************************/

/*****************************************************************************/ 
/* End of file                                                               */ 
/*****************************************************************************/
#endif /* __ACCEL_GENERIC_H */
