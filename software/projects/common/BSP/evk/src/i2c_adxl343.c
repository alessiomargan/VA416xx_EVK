/***************************************************************************************
 * @file     i2c_adxl343.c
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
 
/*****************************************************************************/ 
/* Include files                                                             */ 
/*****************************************************************************/

#include "i2c_adxl343.h"
#include "va416xx_hal_i2c.h"
#include "va416xx_debug.h"

/*****************************************************************************/ 
/* Local pre-processor symbols/macros ('#define')                            */ 
/*****************************************************************************/

#define I2C_INVALID_BANKNUM (0xff) // bad index

#define ADXL343_DEVID (0xE5) // identifier address 0

#define ONE_G_RAW_COUNTS (128)

/*****************************************************************************/ 
/* Global variable definitions (declared in header file with 'extern')       */ 
/*****************************************************************************/

/*****************************************************************************/ 
/* Local type definitions ('typedef')                                        */ 
/*****************************************************************************/

/*****************************************************************************/ 
/* Local variable definitions ('static')                                     */ 
/*****************************************************************************/

/*****************************************************************************/ 
/* Local function prototypes ('static')                                      */ 
/*****************************************************************************/

__STATIC_INLINE uint8_t GetBankNum(VOR_I2C_Type* const i2c);
static bool isi2cInit(VOR_I2C_Type * const i2c);
static hal_status_t ADXL343_WriteSingle(stc_accel_handle_t * const handle, uint8_t addr, uint8_t val);
static hal_status_t ADXL343_ReadSingle(stc_accel_handle_t * const handle, uint8_t addr, uint8_t* val);
static hal_status_t ADXL343_ReadMulti(stc_accel_handle_t * const handle, uint8_t addr, uint8_t* buf, uint32_t len);

/*****************************************************************************/ 
/* Function implementation - global ('extern') and local ('static')          */ 
/*****************************************************************************/

hal_status_t ADXL343_InitI2C(stc_accel_handle_t * const handle)
{
  hal_status_t stat;
  if(handle->i2c == NULL){ return hal_status_badParam; }
  if(handle->serialType != en_accel_serial_i2c){ return hal_status_badParam; }
  
  // setup the i2c peripheral
  if(!isi2cInit(handle->i2c)){
    stc_i2c_masterInit_t initm = {0};
    initm.speed = en_i2c_100k;
    stat = HAL_I2C_Reset(handle->i2c);
    if(stat != hal_status_ok){ return stat; }
    stat = HAL_I2CM_Init(handle->i2c, initm);
    if(stat != hal_status_ok){ return stat; }
  }
  handle->isInit = true; 

  // setup the adxl343
  
  uint8_t devid;
  stat = ADXL343_ReadSingle(handle, 0x00, &devid);
  if((stat != hal_status_ok) || (devid != ADXL343_DEVID)){
    handle->isInit = false;
    stat = hal_status_initError;
  }else{
    stat = ADXL343_ReadSingle(handle, 0x2D, &devid);
    if(devid != 0x28){
      ADXL343_WriteSingle(handle, 0x31, 0x01); //4g setting, RJ
      ADXL343_WriteSingle(handle, 0x38, 0x00); //fifo bypass mode
      ADXL343_WriteSingle(handle, 0x2D, 0x28); //link=1, measure=1
      HAL_WaitMs(25); // wait for first sample
    }
  }
  return stat;
}

hal_status_t ADXL343_GetAccelRawValues(stc_accel_handle_t * const handle, \
  stc_accel_raw_values_t* const accelDataRaw)
{
  if(!handle->isInit){ return hal_status_notInitialized; }
  uint8_t buf[6] = {0};
  hal_status_t stat = ADXL343_ReadMulti(handle, 0x32, buf, 6);
  accelDataRaw->x = (((int16_t)(buf[1]))<<8) + buf[0];
  accelDataRaw->y = (((int16_t)(buf[3]))<<8) + buf[2];
  accelDataRaw->z = (((int16_t)(buf[5]))<<8) + buf[4];
  return stat;
}

hal_status_t ADXL343_GetAccelMgValues(stc_accel_handle_t * const handle, \
  stc_accel_mg_values_t* const accelDataMg)
{
  hal_status_t stat;
  stc_accel_raw_values_t accelRaw;
  if(!handle->isInit){ return hal_status_notInitialized; }
  stat = ADXL343_GetAccelRawValues(handle, &accelRaw);
  if(stat == hal_status_ok){
    int32_t temp;
    temp = (int32_t)(accelRaw.x);
    temp = (temp*1000)/ONE_G_RAW_COUNTS;
    accelDataMg->x = (float)(temp);
    temp = (int32_t)(accelRaw.y);
    temp = (temp*1000)/ONE_G_RAW_COUNTS;
    accelDataMg->y = (float)(temp);
    temp = (int32_t)(accelRaw.z);
    temp = (temp*1000)/ONE_G_RAW_COUNTS;
    accelDataMg->z = (float)(temp);
  }
  return stat;
}

hal_status_t ADXL343_GetTemperature(stc_accel_handle_t * const handle, float* const temp)
{
  return hal_status_notImplemented;
}

hal_status_t ADXL343_UnInit(stc_accel_handle_t * const handle)
{
  hal_status_t stat = ADXL343_WriteSingle(handle, 0x2D, 0x20); //link=1, measure=0
  handle->isInit = false;
  return stat;
}

/*******************************************************************************
 **
 ** @brief  Returns bank 0, 1, or 2 based on I2C peripheral pointer value
 **
 ******************************************************************************/
__STATIC_INLINE uint8_t GetBankNum(VOR_I2C_Type* const i2c)
{
  if(VOR_I2C0 == i2c)
  { return 0; }
  else if(VOR_I2C1 == i2c)
  { return 1; }
  else if(VOR_I2C2 == i2c)
  { return 2; }
  else
  {
    // invalid i2c bank
    return I2C_INVALID_BANKNUM;
  }
}

/*******************************************************************************
 **
 ** @brief  Returns false if the I2C peripheral is not clocked or not enabled, else true
 **
 ******************************************************************************/
static bool isi2cInit(VOR_I2C_Type * const i2c)
{
  uint8_t bank = GetBankNum(i2c);
  if(bank == I2C_INVALID_BANKNUM){ return false; }
  if((VOR_SYSCONFIG->PERIPHERAL_CLK_ENABLE & (CLK_ENABLE_I2C0 << bank)) == 0){
    return false;
  }
  if((i2c->CTRL & I2C_CTRL_ENABLE_Msk) == 0){ return false; }
  return true;
}

/*******************************************************************************
 **
 ** @brief  Write single register
 **
 ******************************************************************************/
static hal_status_t ADXL343_WriteSingle(stc_accel_handle_t * const handle, uint8_t addr, uint8_t val)
{
  if(false == handle->isInit){ return hal_status_notInitialized; }
  char dat[2] = {addr, val};
  return HAL_I2CM_WriteB(handle->i2c, handle->i2cAddr, dat, 2);
}

/*******************************************************************************
 **
 ** @brief  Read single register
 **
 ******************************************************************************/
static hal_status_t ADXL343_ReadSingle(stc_accel_handle_t * const handle, uint8_t addr, uint8_t* val)
{
  if(false == handle->isInit){ return hal_status_notInitialized; }
  if(val == NULL){ return hal_status_badParam; }
  return HAL_I2CM_WrReadB(handle->i2c, handle->i2cAddr, (char*)&addr, 1, (char*)val, 1);
}

/*******************************************************************************
 **
 ** @brief  Read multiple registers
 **
 ******************************************************************************/
static hal_status_t ADXL343_ReadMulti(stc_accel_handle_t * const handle, uint8_t addr, uint8_t* buf, uint32_t len)
{
  if(false == handle->isInit){ return hal_status_notInitialized; }
  if((buf == NULL) || (len == 0)){ return hal_status_badParam; }
  addr |= 0x80; // set address auto increment
  return HAL_I2CM_WrReadB(handle->i2c, handle->i2cAddr, (char*)&addr, 1, (char*)buf, len);
}

/*****************************************************************************/ 
/* End of file                                                               */ 
/*****************************************************************************/
