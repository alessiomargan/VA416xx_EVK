/***************************************************************************************
 * @file     i2c_lis2de12.c
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

#include "i2c_lis2de12.h"
#include "va416xx_hal_i2c.h"
#include "va416xx_debug.h"
#include "board.h"

/*****************************************************************************/ 
/* Local pre-processor symbols/macros ('#define')                            */ 
/*****************************************************************************/

#define I2C_INVALID_BANKNUM (0xff) // bad index


#define LIS_STATUS_REG_AUX  (0x07)
#define LIS_OUT_TEMP_L      (0x0C)
#define LIS_OUT_TEMP_H      (0x0D)
#define LIS_WHO_AM_I        (0x0F)
#define LIS_CTRL_REG0       (0x1E)
#define LIS_TEMP_CFG_REG    (0x1F)
#define LIS_CTRL_REG1       (0x20)
#define LIS_CTRL_REG2       (0x21)
#define LIS_CTRL_REG3       (0x22)
#define LIS_CTRL_REG4       (0x23)
#define LIS_REFERENCE       (0x26)
#define LIS_STATUS_REG      (0x27)
#define LIS_FIFO_READ_START (0x28)
#define LIS_OUT_X_H         (0x29)
#define LIS_OUT_Y_H         (0x2B)
#define LIS_OUT_Z_H         (0x2D)
#define LIS_FIFO_CTRL_REG   (0x2E)
#define LIS_FIFO_SRC_REG    (0x2F)
#define LIS_INT1_CFG        (0x30)
#define LIS_INT1_SRC        (0x31)
#define LIS_INT1_THS        (0x32)
#define LIS_INT1_DURATION   (0x33)
#define LIS_INT2_CFG        (0x34)
#define LIS_INT2_SRC        (0x35)
#define LIS_INT2_THS        (0x36)
#define LIS_INT2_DURATION   (0x37)
#define LIS_CLICK_CFG       (0x38)
#define LIS_CLICK_SRC       (0x39)
#define LIS_CLICK_THS       (0x3A)
#define LIS_TIME_LIMIT      (0x3B)
#define LIS_TIME_LATENCY    (0x3C)
#define LIS_TIME_WINDOW     (0x3D)
#define LIS_ACT_THS         (0x3E)
#define LIS_ACT_DUR         (0x3F)

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
static hal_status_t LIS2DE12_WriteSingle(stc_accel_handle_t * const handle, uint8_t addr, uint8_t val);
#if 0
// disabled to suppress warning
static hal_status_t LIS2DE12_WriteMulti(stc_accel_handle_t * const handle, uint8_t* buf, uint32_t len);
#endif
static hal_status_t LIS2DE12_ReadSingle(stc_accel_handle_t * const handle, uint8_t addr, uint8_t* val);
static hal_status_t LIS2DE12_ReadMulti(stc_accel_handle_t * const handle, uint8_t addr, uint8_t* buf, uint32_t len);

static float lis2de12_from_fs2_to_mg(int16_t lsb);
static float lis2de12_from_fs4_to_mg(int16_t lsb);
static float lis2de12_from_fs8_to_mg(int16_t lsb);
static float lis2de12_from_fs16_to_mg(int16_t lsb);

/*****************************************************************************/ 
/* Function implementation - global ('extern') and local ('static')          */ 
/*****************************************************************************/

/*******************************************************************************
 **
 ** @brief  Init the I2C port lis2de12 is connected to and init accel
 **
 ******************************************************************************/
hal_status_t LIS2DE12_InitI2C(stc_accel_handle_t * const handle)
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
  
  // auto-detect address bit of I2C accelerometer (0x30 or 0x32)
  uint8_t rdval;
  stat = LIS2DE12_ReadSingle(handle, LIS_WHO_AM_I, &rdval);
  if(stat != hal_status_ok){
    handle->i2cAddr &= ~0x2;
    stat = LIS2DE12_ReadSingle(handle, LIS_WHO_AM_I, &rdval);
  }
  if(stat != hal_status_ok){ return stat; }
  
  if(rdval != 0x33){
    return hal_status_initError;
  } 
  
  stat = LIS2DE12_WriteSingle(handle, LIS_TEMP_CFG_REG, 0xC0);
  if(stat != hal_status_ok){ return stat; }
  
  stat = LIS2DE12_WriteSingle(handle, LIS_CTRL_REG1, 0x5F);
  if(stat != hal_status_ok){ return stat; }
  
  // BDU en, FS = 4g
  stat = LIS2DE12_WriteSingle(handle, LIS_CTRL_REG4, 0x90);
  if(stat != hal_status_ok){ return stat; }
  
  return hal_status_ok;
}

/*******************************************************************************
 **
 ** @brief  Get accelerometer values (raw)
 **
 ******************************************************************************/
hal_status_t LIS2DE12_GetAccelRawValues(stc_accel_handle_t * const handle, \
  stc_accel_raw_values_t* const accelDataRaw)
{
  if(!handle->isInit){ return hal_status_notInitialized; }
  if(accelDataRaw == NULL){ return hal_status_badParam; }
  
  hal_status_t stat;
  uint8_t statusReg;
  do{
    stat = LIS2DE12_ReadSingle(handle, LIS_STATUS_REG, &statusReg);
    if(stat != hal_status_ok){ return stat; }
  } while((statusReg & 0x8) == 0); // wait for ZYXDA bit high
  
  uint8_t buf[6];
  stat = LIS2DE12_ReadMulti(handle, LIS_FIFO_READ_START, buf, 6);
  if(stat != hal_status_ok){ return stat; }
  
  accelDataRaw->x = *(int16_t*)&buf[0];
  accelDataRaw->y = *(int16_t*)&buf[2];
  accelDataRaw->z = *(int16_t*)&buf[4];
  
  return hal_status_ok;
}

/*******************************************************************************
 **
** @brief  Get accelerometer values (units: mg)
 **
 ******************************************************************************/
hal_status_t LIS2DE12_GetAccelMgValues(stc_accel_handle_t * const handle, \
  stc_accel_mg_values_t* const accelDataMg)
{
  if(!handle->isInit){ return hal_status_notInitialized; }
  if(accelDataMg == NULL){ return hal_status_badParam; }
  
  stc_accel_raw_values_t accelRaw;
  hal_status_t stat = LIS2DE12_GetAccelRawValues(handle, &accelRaw);
  if(stat != hal_status_ok){ return stat; }
  
  uint8_t ctrl4;
  stat = LIS2DE12_ReadSingle(handle, LIS_CTRL_REG4, &ctrl4);
  if(stat != hal_status_ok){ return stat; }
  
  switch((ctrl4 & 0x30) >> 4){
    case 0:
      // full scale 2g
      accelDataMg->x = lis2de12_from_fs2_to_mg(accelRaw.x);
      accelDataMg->y = lis2de12_from_fs2_to_mg(accelRaw.y);
      accelDataMg->z = lis2de12_from_fs2_to_mg(accelRaw.z);
      break;
    case 1:
      // full scale 4g
      accelDataMg->x = lis2de12_from_fs4_to_mg(accelRaw.x);
      accelDataMg->y = lis2de12_from_fs4_to_mg(accelRaw.y);
      accelDataMg->z = lis2de12_from_fs4_to_mg(accelRaw.z);
      break;
    case 2:
      // full scale 8g
      accelDataMg->x = lis2de12_from_fs8_to_mg(accelRaw.x);
      accelDataMg->y = lis2de12_from_fs8_to_mg(accelRaw.y);
      accelDataMg->z = lis2de12_from_fs8_to_mg(accelRaw.z);
      break;
    case 3:
      // full scale 16g
      accelDataMg->x = lis2de12_from_fs16_to_mg(accelRaw.x);
      accelDataMg->y = lis2de12_from_fs16_to_mg(accelRaw.y);
      accelDataMg->z = lis2de12_from_fs16_to_mg(accelRaw.z);
      break;
  }
  return hal_status_ok;
}

/*******************************************************************************
 **
 ** @brief  Get LIS2DE12 temperature
 **
 ******************************************************************************/
hal_status_t LIS2DE12_GetBoardTemp(stc_accel_handle_t * const handle, float* const temp)
{
  if(!handle->isInit){ return hal_status_notInitialized; }
  if(temp == NULL){ return hal_status_badParam; }
  
  hal_status_t stat;
  uint8_t statusReg;
  do{
    stat = LIS2DE12_ReadSingle(handle, LIS_STATUS_REG_AUX, &statusReg);
    if(stat != hal_status_ok){ return stat; }
  } while((statusReg & 0x4) == 0); // wait for TDA bit high
  
  uint8_t tempbuf[2];
  stat = LIS2DE12_ReadMulti(handle, LIS_OUT_TEMP_L, tempbuf, 2);
  if(stat != hal_status_ok){ return stat; }
  
  int16_t tempRaw;
  tempRaw = *(int16_t*)tempbuf;
  *temp = (((float)tempRaw) / 256.0f) + 25.0f;
  
  return hal_status_ok;
}

/*******************************************************************************
 **
 ** @brief  Un-init accelerometer and I2C port
 **
 ******************************************************************************/
hal_status_t LIS2DE12_UnInit(stc_accel_handle_t * const handle)
{
  if(true == handle->isInit){
    HAL_I2C_Reset(handle->i2c);
    HAL_I2C_Declock(handle->i2c);
    handle->isInit = false;
  }
  return hal_status_ok;
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
static hal_status_t LIS2DE12_WriteSingle(stc_accel_handle_t * const handle, uint8_t addr, uint8_t val)
{
  if(false == handle->isInit){ return hal_status_notInitialized; }
  char dat[2] = {addr, val};
  return HAL_I2CM_WriteB(handle->i2c, handle->i2cAddr, dat, 2);
}

/*******************************************************************************
 **
 ** @brief  Write multiple registers (disabled currently to suppress warning)
 **
 ******************************************************************************/
#if 0
static hal_status_t LIS2DE12_WriteMulti(stc_accel_handle_t * const handle, uint8_t* buf, uint32_t len)
{
  if(false == handle->isInit){ return hal_status_notInitialized; }
  if((buf == NULL) || (len == 0)){ return hal_status_badParam; }
  buf[0] |= 0x80; // set address auto increment
  return HAL_I2CM_WriteB(handle->i2c, handle->i2cAddr, (char*)buf, len);
}
#endif

/*******************************************************************************
 **
 ** @brief  Read single register
 **
 ******************************************************************************/
static hal_status_t LIS2DE12_ReadSingle(stc_accel_handle_t * const handle, uint8_t addr, uint8_t* val)
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
static hal_status_t LIS2DE12_ReadMulti(stc_accel_handle_t * const handle, uint8_t addr, uint8_t* buf, uint32_t len)
{
  if(false == handle->isInit){ return hal_status_notInitialized; }
  if((buf == NULL) || (len == 0)){ return hal_status_badParam; }
  addr |= 0x80; // set address auto increment
  return HAL_I2CM_WrReadB(handle->i2c, handle->i2cAddr, (char*)&addr, 1, (char*)buf, len);
}

static float lis2de12_from_fs2_to_mg(int16_t lsb)
{
  return ( (float)lsb / 256.0f ) * 16.0f;
}

static float lis2de12_from_fs4_to_mg(int16_t lsb)
{
  return ( (float)lsb / 256.0f ) * 32.0f;
}

static float lis2de12_from_fs8_to_mg(int16_t lsb)
{
  return ( (float)lsb / 256.0f ) * 64.0f;
}

static float lis2de12_from_fs16_to_mg(int16_t lsb)
{
  return ( (float)lsb / 256.0f ) * 192.0f;
}

/*****************************************************************************/ 
/* End of file                                                               */ 
/*****************************************************************************/
