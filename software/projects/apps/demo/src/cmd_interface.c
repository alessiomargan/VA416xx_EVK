/***************************************************************************************
 * @file     cmd_interface.c
 * @version  V2.05
 * @date     20 December 2023
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

#include <string.h> // for memcpy()
#include <stdlib.h>

#include "cmd_interface.h"
#include "board.h"
#include "hal_config.h"

#ifdef ENABLE_RTT
#include "segger_rtt.h"
#endif

#include "va416xx_hal_adc.h"
#include "va416xx_hal_clkgen.h"
#include "va416xx_hal_dac.h"
#include "va416xx_hal_dma.h"
#ifdef INC_I2C_TEST
#include "va416xx_hal_i2c.h"
#endif
#include "va416xx_hal_irqrouter.h"
#include "va416xx_hal_spi.h"
#include "va416xx_hal_timer.h"
#include "va416xx_hal_uart.h"
#include "va416xx_debug.h"

#include "spi_fram.h"
#ifdef INC_ACCEL_TEST
#include ACCEL_INCLUDE
#endif
#ifdef INC_CAN_TEST
#include "can_test.h"
#endif
#ifdef INC_PHY_TEST
#include "phy_test.h"
#endif
#ifdef INC_DAC_SINE
#include "dac_sine.h"
#endif

/*****************************************************************************/
/* Local pre-processor symbols/macros ('#define')                            */
/*****************************************************************************/

// RX command buffer size
#define RXCMD_BUFFERSZ (64)
#define TX_BUFFERSZ    (64)

// I2C test
#define I2C2_SLAVE_ADDR (i2caddr_shifted_t)(0x5E)

// spi2 dma test
#ifndef SPI_DMA_TX_CH
#define SPI_DMA_TX_CH    (2)
#endif
#ifndef SPI_DMA_RX_CH
#define SPI_DMA_RX_CH    (3)
#endif
#ifndef SPI_DMA_TX_ISR
#define SPI_DMA_TX_ISR   DMA_Done_2_IRQHandler
#endif
#ifndef SPI_DMA_RX_ISR
#define SPI_DMA_RX_ISR   DMA_Done_3_IRQHandler
#endif
#ifndef SPI_DMA_TX_IRQN
#define SPI_DMA_TX_IRQN  DMA_DONE2_IRQn
#endif
#ifndef SPI_DMA_RX_IRQN
#define SPI_DMA_RX_IRQN  DMA_DONE3_IRQn
#endif
#define CLKPRESCALE_VALUE (4)

#define ADC_DMA_CH        (3)
#define ADC_DMA_IRQN      DMA_DONE3_IRQn

/*****************************************************************************/
/* Global variable definitions (declared in header file with 'extern')       */
/*****************************************************************************/

TxDest_t txDest;

const char * CmdStrArray[(uint32_t)CmdEnum_End] =
{
  "",        //CmdEnum_None
  "readport",//CmdEnum_ReadPort
  "setport", //CmdEnum_SetPort
  "readadc", //CmdEnum_ReadADC
  "setdac",  //CmdEnum_SetDAC
  "rom1wr",  //CmdEnum_FramWrite
  "rom1rd",  //CmdEnum_FramRead
  "auxwr",   //CmdEnum_FramAuxWrite
  "auxrd",   //CmdEnum_FramAuxRead
  "ebiinit", //CmdEnum_EBIInit
  "ebiwrite",//CmdEnum_EBIWrite
  "ebiread", //CmdEnum_EBIRead
  "cantest", //CmdEnum_CanTest
  "phytest", //CmdEnum_PhyTest
  "spwtest", //CmdEnum_SpWTest
  "i2ctest", //CmdEnum_I2CTest
  "accel",   //CmdEnum_GetAccel
  "brdtmp",  //CmdEnum_GetBoardTemp
  "fwver",   //CmdEnum_FWVersion
  "srst",    //CmdEnum_SoftReset
  "clkpll",  //CmdEnum_ClkPLL
  "clkhbo",  //CmdEnum_ClkHBO
  "clkext",  //CmdEnum_ClkExt
  "clkxtal", //CmdEnum_ClkXtal
  "clk",     //CmdEnum_Clk
  "adctest", //CmdEnum_AdcTest
  "dacsine", //CmdEnum_DacSine
  "spiint",  //CmdEnum_SpiInt
  "spidma",  //CmdEnum_SpiDma
  "dietmp",  //CmdEnum_DieTemp
  "help",    //CmdEnum_Help
};

const char * UsageStrArray[(uint32_t)CmdEnum_End] =
{
  "",                               //CmdEnum_None
  "readport [port 0-6] [mask]",     //CmdEnum_ReadPort
  "setport [port 0-6] [mask] [val]",//CmdEnum_SetPort
  "readadc [ch 0-15]",              //CmdEnum_ReadADC
  "setdac [0 or 1] [0x0-0xfff]",    //CmdEnum_SetDAC
  "rom1wr [addr] [data32]",         //CmdEnum_FramWrite
  "rom1rd [addr]",                  //CmdEnum_FramRead
  "auxwr [addr] [data32]",          //CmdEnum_FramAuxWrite
  "auxrd [addr]",                   //CmdEnum_FramAuxRead
  "ebiinit",                        //CmdEnum_EBIInit
  "ebiwrite [addr] [data32]",       //CmdEnum_EBIWrite
  "ebiread [addr]",                 //CmdEnum_EBIRead
  "cantest",                        //CmdEnum_CanTest
  "phytest",                        //CmdEnum_PhyTest
  "spwtest",                        //CmdEnum_SpWTest
  "i2ctest",                        //CmdEnum_I2CTest
  "accel [(opt) 1: en every sec]",  //CmdEnum_GetAccel
  "brdtmp [(opt) 1: en every sec]", //CmdEnum_GetBoardTemp
  "fwver",                          //CmdEnum_FWVersion
  "srst",                           //CmdEnum_SoftReset
  "clkpll [Hz 7000000-100000000]",  //CmdEnum_ClkPLL
  "clkhbo",                         //CmdEnum_ClkHBO
  "clkext",                         //CmdEnum_ClkExt
  "clkxtal",                        //CmdEnum_ClkXtal
  "clk",                            //CmdEnum_Clk
  "adctest",                        //CmdEnum_AdcTest
  "dacsine [0 or 1] [freqHz]",      //CmdEnum_DacSine
  "spiint [bank] [len] [txword16]", //CmdEnum_SpiInt
  "spidma [bank] [len] [txword16]", //CmdEnum_SpiDma
  "dietmp [(opt) 1: en every sec]", //CmdEnum_DieTemp
  "help",                           //CmdEnum_Help
};

const char * DescStrArray[(uint32_t)CmdEnum_End] =
{
  "",                               //CmdEnum_None
  "Masked pins input, read IO port",//CmdEnum_ReadPort
  "Sets masked pins hi/lo",         //CmdEnum_SetPort
  "Read an ADC channel",            //CmdEnum_ReadADC
  "Set DAC0 or DAC1 output",        //CmdEnum_SetDAC
  "Write word to SPI boot FRAM",    //CmdEnum_FramWrite
  "Read word from SPI boot FRAM",   //CmdEnum_FramRead
  "Write word to aux FRAM",         //CmdEnum_FramAuxWrite
  "Read word from aux FRAM",        //CmdEnum_FramAuxRead
  "Initialize external mem interf", //CmdEnum_EBIInit
  "Write word to EBI",              //CmdEnum_EBIWrite
  "Read word from EBI",             //CmdEnum_EBIRead
  "CAN bus loopback test",          //CmdEnum_CanTest
  "Eth PHY loopback test",          //CmdEnum_PhyTest
  "SpaceWire test",                 //CmdEnum_SpWTest
  "I2C test I2C1(m)->I2C2(s)",      //CmdEnum_I2CTest
  "Get accelerometer data",         //CmdEnum_GetAccel
  "Get board temperature",          //CmdEnum_GetBoardTemp
  "Get firmware version",           //CmdEnum_FWVersion
  "Software reset",                 //CmdEnum_SoftReset
  "Set PLL freq., Clk src to PLL",  //CmdEnum_ClkPLL
  "Set clock source to HBO",        //CmdEnum_ClkHBO
  "Set clock source to XTAL in",    //CmdEnum_ClkExt
  "Set clock source to XTAL amp",   //CmdEnum_ClkXtal
  "Get current sysclk speed",       //CmdEnum_Clk
  "Enable adc sweep once/s",        //CmdEnum_AdcTest
  "Output a sine wave from DAC",    //CmdEnum_DacSine
  "SPI example using interrupts",   //CmdEnum_SpiInt
  "SPI example using DMA",          //CmdEnum_SpiDma
  "Read on chip die temp",          //CmdEnum_DieTemp
  "Print help",                     //CmdEnum_Help
};

const uint32_t CmdParamCountArray[(uint32_t)CmdEnum_End] =
{
  0, //CmdEnum_None
  2, //CmdEnum_ReadIO
  3, //CmdEnum_SetIO
  1, //CmdEnum_ReadADC
  2, //CmdEnum_SetDAC
  2, //CmdEnum_FramWrite
  1, //CmdEnum_FramRead
  2, //CmdEnum_FramAuxWrite
  1, //CmdEnum_FramAuxRead
  0, //CmdEnum_EBIInit
  2, //CmdEnum_EBIWrite
  1, //CmdEnum_EBIRead
  0, //CmdEnum_CanTest
  0, //CmdEnum_PhyTest
  0, //CmdEnum_SpWTest
  0, //CmdEnum_I2CTest
  0, //CmdEnum_GetAccel
  0, //CmdEnum_GetBoardTemp
  0, //CmdEnum_FWVersion
  0, //CmdEnum_SoftReset
  1, //CmdEnum_ClkPLL
  0, //CmdEnum_ClkHBO
  0, //CmdEnum_ClkExt
  0, //CmdEnum_ClkXtal
  0, //CmdEnum_Clk
  0, //CmdEnum_AdcTest
  2, //CmdEnum_DacSine
  3, //CmdEnum_SpiInt
  3, //CmdEnum_SpiDma
  0, //CmdEnum_DieTemp
  0, //CmdEnum_Help
};

/*****************************************************************************/
/* Local type definitions ('typedef')                                        */
/*****************************************************************************/

/*****************************************************************************/
/* Local variable definitions ('static')                                     */
/*****************************************************************************/

static char  rxCmdBuffer0[RXCMD_BUFFERSZ];
static char  rxCmdBuffer1[RXCMD_BUFFERSZ];
static uint32_t rxCmdBufferIndex0;
static uint32_t rxCmdBufferIndex1;
static char     txBuffer[TX_BUFFERSZ];
#if !defined(__VA41628__)
static bool     ebiInitialized = false;
#endif

// once a second tasks
static bool accelSecEn = false;
static bool brdTmpSecEn = false;
static bool dieTmpSecEn = false;
static bool adcTestEn = false;

// for SPI dma/interrupt examples
static hal_spi_handle_t spiHandle;
static volatile bool dmaTxDone = false;
static volatile bool dmaRxDone = false;

// Accelerometer handle
#ifdef INC_ACCEL_TEST
static stc_accel_handle_t accelHandle = {
  .i2c = ACCEL_I2C, \
  .i2cAddr = ACCEL_I2C_ADDR, \
  .serialType = en_accel_serial_i2c, \
  .isInit = false \
};
static const stc_accel_interf_t accelInterface = { \
  .init = &ACCEL_INIT, \
  .getRawValues = &ACCEL_GET_RAW_VALUES, \
  .getMgValues = &ACCEL_GET_MG_VALUES, \
  .getTemp = &ACCEL_GET_TEMP, \
  .unInit = &ACCEL_UNINIT \
};
#endif

/*****************************************************************************/
/* Local function prototypes ('static')                                      */
/*****************************************************************************/

static void txStr(const char *pcStr);
static void         processRxByte(uint8_t rxByte, char* cmdbuf, uint32_t* idxptr);
static RxCmdSrc_t   checkRxBuffers(void);
static bool runCommand(CmdEnum_t cmd, uint32_t params[], uint32_t paramCount);
static void flushUart(void);

static hal_status_t GPIO_Read(uint32_t port, uint32_t mask);
static hal_status_t GPIO_SetOutput(uint32_t port, uint32_t mask, uint32_t val);
static hal_status_t ADC_ReadAndPrint(uint32_t channel);
static hal_status_t ADC_TestEnable(void);
static hal_status_t ADC_PrintTemp(uint32_t enableSec);
static hal_status_t DAC_SetOutput(uint32_t dacNum, uint32_t val);
static hal_status_t FRAMBOOT_Write(uint32_t addr, uint32_t data);
static hal_status_t FRAMBOOT_ReadAndPrint(uint32_t addr);
static hal_status_t FRAMAUX_Write(uint32_t addr, uint32_t data);
static hal_status_t FRAMAUX_ReadAndPrint(uint32_t addr);
static hal_status_t EBI_Init(void);
static hal_status_t EBI_Write(uint32_t addr, uint32_t data);
static hal_status_t EBI_ReadAndPrint(uint32_t addr);
static hal_status_t CAN_Test(void);
static hal_status_t PHY_Test(void);
static hal_status_t SPW_Test(void);
static hal_status_t I2C_Test(void);
static hal_status_t I2C_GetAccelData(uint32_t enableSec);
static hal_status_t I2C_GetBoardTemp(uint32_t enableSec);
static hal_status_t Clk_PLL(uint32_t freq);
static hal_status_t Clk_HBO(void);
static hal_status_t Clk_Ext(void);
static hal_status_t Clk_Xtal(void);
static hal_status_t Spi_Int(uint32_t bank, uint16_t len, uint16_t txwd);
static hal_status_t Spi_Dma(uint32_t bank, uint16_t len, uint16_t txwd);
static void         Spi_Setup(hal_spi_handle_t* hspi, uint32_t bank);
static void         HELP_PrintHelp(void);

/*****************************************************************************/
/* Function implementation - global ('extern') and local ('static')          */
/*****************************************************************************/

/*******************************************************************************
 **
 ** @brief  Command Interface Main Task - processes commands and responds
 **
 ******************************************************************************/
void CommandInterfaceMainTask(void)
{
  RxCmdSrc_t rxSource;
  char cmdStr[6][21] = {{0}};
  uint32_t paramList[5] = {0};
  int32_t paramCount;
  CmdEnum_t cmdEnum = CmdEnum_None;

  rxSource = checkRxBuffers();
  switch(rxSource){
    case RxCmdSrc_Uart0:
      // Received command from UART0
      // Determine how many parameters the command was passed
      paramCount = sscanf((const char *)&rxCmdBuffer0[1], "%20s %20s %20s %20s %20s %20s", \
                              &cmdStr[0][0], &cmdStr[1][0], &cmdStr[2][0], \
                              &cmdStr[3][0], &cmdStr[4][0], &cmdStr[5][0]);
      txDest = TxDest_Uart0; // send response to Uart0
      break;

    case RxCmdSrc_Uart1:
      // Received command from UART1
      // Determine how many parameters the command was passed
      paramCount = sscanf((const char *)&rxCmdBuffer0[1], "%20s %20s %20s %20s %20s %20s", \
                              &cmdStr[0][0], &cmdStr[1][0], &cmdStr[2][0], \
                              &cmdStr[3][0], &cmdStr[4][0], &cmdStr[5][0]);
      txDest = TxDest_Uart1; // send response to Uart1
      break;

    case RxCmdSrc_Uart2:
      // Received command from UART2
      // Determine how many parameters the command was passed
      paramCount = sscanf((const char *)&rxCmdBuffer0[1], "%20s %20s %20s %20s %20s %20s", \
                              &cmdStr[0][0], &cmdStr[1][0], &cmdStr[2][0], \
                              &cmdStr[3][0], &cmdStr[4][0], &cmdStr[5][0]);
      txDest = TxDest_Uart2; // send response to Uart2
      break;

    case RxCmdSrc_RTT:
      // Received command from RTT
      // Determine how many parameters the command was passed
      paramCount = sscanf((const char *)&rxCmdBuffer1[1], "%20s %20s %20s %20s %20s %20s", \
                              &cmdStr[0][0], &cmdStr[1][0], &cmdStr[2][0], \
                              &cmdStr[3][0], &cmdStr[4][0], &cmdStr[5][0]);
      txDest = TxDest_RTT; // send response to RTT
      break;

    case RxCmdSrc_Error:
      ERRMSG_TERM();
      return;

    default:
      return;
  }

  // now process the received cmd
  if(paramCount < 0)
  {
    // sscanf failure condition
    ERRMSG_TERM();
    return;
  }
  for(uint32_t i = 1; i < paramCount; i++)
  {
    char* end;
    paramList[i-1] = strtoul(&cmdStr[i][0], &end, 0);
  }
  paramCount--;
  accelSecEn = false;
  brdTmpSecEn = false;
  dieTmpSecEn = false;
  adcTestEn = false;
  for(uint32_t i=0; i<(uint32_t)CmdEnum_End; i++)
  {
    if(strcmp((const char *)&cmdStr[0][0], CmdStrArray[i]) == 0)
    {
      cmdEnum = (CmdEnum_t)i;
    }
  }
  if(cmdEnum == CmdEnum_None)
  {
    ERRMSG();
    txStr("Unrecognized cmd. Type '$help' for help");
    TERMMSG();
  } else {
    runCommand(cmdEnum, paramList, paramCount);
  }
  return;
}

/*******************************************************************************
 **
 ** @brief  Do things that should be done once a second
 **
 ******************************************************************************/
void OnceEverySecondTasks(void)
{
  en_stdio_t dbgio = DBG_GetStdioOutput();
  switch(dbgio){
    case en_stdio_uart0:
      txDest = TxDest_Uart0;
      break;
    case en_stdio_uart1:
      txDest = TxDest_Uart1;
      break;
    case en_stdio_uart2:
      txDest = TxDest_Uart2;
      break;
    case en_stdio_rtt:
      txDest = TxDest_RTT;
      break;
    default:
      txDest = TxDest_None;
  }
  if(accelSecEn){ I2C_GetAccelData(0); }
  if(brdTmpSecEn){ I2C_GetBoardTemp(0); }
  if(dieTmpSecEn){ ADC_PrintTemp(0); }
  if(adcTestEn){
    for(uint8_t i=0; i<8; i++){
      uint16_t val;
      hal_status_t stat = HAL_ADC_ReadSingle(i, &val);
      if(stat == hal_status_ok){ sprintf(txBuffer, "%03x ",val); }
      else txStr("err ");
      txStr(txBuffer);
    }
    txStr("\n");
  }
}

/*******************************************************************************
 **
 ** @brief  Transmit a response string to the appropriate dest
 **
 ** @param pcStr - C string to transmit
 **
 ******************************************************************************/
static void txStr(const char *pcStr)
{
  switch(txDest){
    case TxDest_Uart0:
      HAL_Uart_TxStr(VOR_UART0, pcStr);
      break;
    case TxDest_Uart1:
      HAL_Uart_TxStr(VOR_UART1, pcStr);
      break;
    case TxDest_Uart2:
      HAL_Uart_TxStr(VOR_UART2, pcStr);
      break;
    case TxDest_RTT:
#ifdef ENABLE_RTT
      SEGGER_RTT_WriteString(0, pcStr);
#endif
      break;
    case TxDest_None:
      break;
  }
}

/*******************************************************************************
 **
 ** @brief  Wait for txDest's UART to finish transmitting, and clear RX
 **
 **
 ******************************************************************************/
static void flushUart(void)
{
  switch(txDest){
    case TxDest_Uart0:
      HAL_Uart_Flush(VOR_UART0);
      break;
    case TxDest_Uart1:
      HAL_Uart_Flush(VOR_UART1);
      break;
    case TxDest_Uart2:
      HAL_Uart_Flush(VOR_UART2);
      break;
    default:
      break;
  }
}

/*******************************************************************************
 **
 ** @brief  process and save received characters onto a buffer
 **
 ** @param rxByte the received character
 **
 ** @param cmdbuf the command buffer
 **
 ** @param idxptr pointer to a uint32_t that contains the current index in the buffer
 **
 ******************************************************************************/
static void processRxByte(uint8_t rxByte, char* cmdbuf, uint32_t* idxptr)
{
  uint32_t pos = *idxptr;
  switch(rxByte)
  {
    case CMD_START_CHAR: // $ - start of cmd
      pos = 0;
      cmdbuf[pos++] = rxByte;
      break;

    case 0x7F: // backspace key
      if (pos > 0)
        pos--;
      break;

    case CMD_TERMINATOR_CHAR_CR: // CR
      if (pos > 0)
      {
        cmdbuf[pos] = 0;
        pos = 0;
      }
      break;
    
    case CMD_TERMINATOR_CHAR_LF: // LF
      if (pos > 0)
      {
        cmdbuf[pos] = 0;
        pos = 0;
      }
      break;

    default:
      cmdbuf[pos++] = rxByte;
      if(pos == RXCMD_BUFFERSZ)
        pos = 0;
      break;
  }
  *idxptr = pos;
}

/*******************************************************************************
 **
 ** @brief  Handle the the RX buffers (UART and RTT)
 **
 ******************************************************************************/
static RxCmdSrc_t checkRxBuffers(void)
{
  uint32_t rxLen[HAL_NUM_UARTS] = {0,0,0};
  VOR_UART_Type* const uarts[HAL_NUM_UARTS] = {VOR_UART0, VOR_UART1, VOR_UART2};
  const RxCmdSrc_t uartReturns[HAL_NUM_UARTS] = {RxCmdSrc_Uart0, RxCmdSrc_Uart1, RxCmdSrc_Uart2};
  for(uint32_t i=0; i<HAL_NUM_UARTS; i++)
  {
    hal_status_t uart_stat = HAL_Uart_GetRxLen(uarts[i], &rxLen[i]);
    if(uart_stat != hal_status_ok)
      continue;
    while(rxLen[i] > 0){
      uint8_t rxByte;
      (void)HAL_Uart_RxByte(uarts[i], &rxByte);
      processRxByte(rxByte, rxCmdBuffer0, &rxCmdBufferIndex0);
      if((rxByte == CMD_TERMINATOR_CHAR_LF) || (rxByte == CMD_TERMINATOR_CHAR_CR))
        return uartReturns[i];
      rxLen[i]--;
    }
  }

#ifdef ENABLE_RTT
  while (SEGGER_RTT_HasKey()){
    uint8_t rxByte = (uint8_t)SEGGER_RTT_GetKey();
    processRxByte(rxByte, rxCmdBuffer1, &rxCmdBufferIndex1);
    if(RTT_ECHO == 1){ SEGGER_RTT_PutChar(0, rxByte); } // echo char
    if((rxByte == CMD_TERMINATOR_CHAR_LF) || (rxByte == CMD_TERMINATOR_CHAR_CR))
      return RxCmdSrc_RTT;
  }
#endif

  return RxCmdSrc_None;
}

/*******************************************************************************
 **
 ** @brief Run a received command (also sends the response)
 **
 ******************************************************************************/
static bool runCommand(CmdEnum_t cmd, uint32_t params[], uint32_t paramCount)
{
  hal_status_t cmdStatus = hal_status_ok;
  if(paramCount < CmdParamCountArray[(uint32_t)cmd]){
    ERRMSG();
    txStr("Usage: $");
    txStr(UsageStrArray[(uint32_t)cmd]);
    txStr(" - ");
    txStr(DescStrArray[(uint32_t)cmd]);
    TERMMSG();
    return false;
  }
  switch(cmd){
    case CmdEnum_ReadPort:
      cmdStatus = GPIO_Read(params[0], params[1]);
      if (hal_status_ok == cmdStatus) TERMMSG();
      break;

    case CmdEnum_SetPort:
      cmdStatus = GPIO_SetOutput(params[0], params[1], params[2]);
      if (hal_status_ok == cmdStatus) OKMSG_TERM();
      break;

    case CmdEnum_ReadADC:
      cmdStatus = ADC_ReadAndPrint(params[0]);
      if (hal_status_ok == cmdStatus) TERMMSG();
      break;

    case CmdEnum_AdcTest:
      cmdStatus = ADC_TestEnable();
      if (hal_status_ok == cmdStatus) OKMSG_TERM();
      break;

    case CmdEnum_DieTemp:
      cmdStatus = ADC_PrintTemp(params[0]);
      if (hal_status_ok == cmdStatus) TERMMSG();
      break;

    case CmdEnum_SetDAC:
      cmdStatus = DAC_SetOutput(params[0], params[1]);
      if (hal_status_ok == cmdStatus) OKMSG_TERM();
      break;

    case CmdEnum_FramWrite:
      cmdStatus = FRAMBOOT_Write(params[0], params[1]);
      if (hal_status_ok == cmdStatus) OKMSG_TERM();
      break;

    case CmdEnum_FramRead:
      cmdStatus = FRAMBOOT_ReadAndPrint(params[0]);
      if (hal_status_ok == cmdStatus) TERMMSG();
      break;

    case CmdEnum_FramAuxWrite:
      cmdStatus = FRAMAUX_Write(params[0], params[1]);
      if (hal_status_ok == cmdStatus) OKMSG_TERM();
      break;

    case CmdEnum_FramAuxRead:
      cmdStatus = FRAMAUX_ReadAndPrint(params[0]);
      if (hal_status_ok == cmdStatus) TERMMSG();
      break;

    case CmdEnum_EBIInit:
      cmdStatus = EBI_Init();
      if (hal_status_ok == cmdStatus) OKMSG_TERM();
      break;

    case CmdEnum_EBIWrite:
      cmdStatus = EBI_Write(params[0], params[1]);
      if (hal_status_ok == cmdStatus) OKMSG_TERM();
      break;

    case CmdEnum_EBIRead:
      cmdStatus = EBI_ReadAndPrint(params[0]);
      if (hal_status_ok == cmdStatus) TERMMSG();
      break;

    case CmdEnum_CanTest:
      cmdStatus = CAN_Test();
      if (hal_status_ok == cmdStatus) OKMSG_TERM();
      break;

    case CmdEnum_PhyTest:
      cmdStatus = PHY_Test();
      if (hal_status_ok == cmdStatus) OKMSG_TERM();
      break;

    case CmdEnum_SpWTest:
      cmdStatus = SPW_Test();
      if (hal_status_ok == cmdStatus) OKMSG_TERM();
      break;

    case CmdEnum_I2CTest:
      cmdStatus = I2C_Test();
      if (hal_status_ok == cmdStatus) OKMSG_TERM();
      break;

    case CmdEnum_GetAccel:
      cmdStatus = I2C_GetAccelData(params[0]);
      if (hal_status_ok == cmdStatus) TERMMSG();
      break;

    case CmdEnum_GetBoardTemp:
      cmdStatus = I2C_GetBoardTemp(params[0]);
      if (hal_status_ok == cmdStatus) TERMMSG();
      break;

    case CmdEnum_FWVersion:
      OKMSG();
      txStr(SOFTWARE_VERSION_STR);
      TERMMSG();
      break;

    case CmdEnum_SoftReset:
      OKMSG_TERM();
      NVIC_SystemReset();
      break;

    case CmdEnum_ClkPLL:
      txStr("\n");
      flushUart();
      cmdStatus = Clk_PLL(params[0]);
      if (hal_status_ok == cmdStatus) txStr("$OK\r\n");
      break;

    case CmdEnum_ClkHBO:
      txStr("\n");
      flushUart();
      cmdStatus = Clk_HBO();
      if (hal_status_ok == cmdStatus) txStr("$OK\r\n");
      break;

    case CmdEnum_ClkExt:
      txStr("\n");
      flushUart();
      cmdStatus = Clk_Ext();
      if (hal_status_ok == cmdStatus) txStr("$OK\r\n");
      break;

    case CmdEnum_ClkXtal:
      txStr("\n");
      flushUart();
      cmdStatus = Clk_Xtal();
      if (hal_status_ok == cmdStatus) txStr("$OK\r\n");
      break;

    case CmdEnum_Clk:
      sprintf(txBuffer, "%ld\n", SystemCoreClock);
      OKMSG();
      txStr(txBuffer);
      break;

    case CmdEnum_DacSine:
#ifdef INC_DAC_SINE
      txStr("\n");
      cmdStatus = DAC_SetupSineDma(params[0], params[1]); // uses DMA
      //cmdStatus = DAC_SetupSine(params[0], params[1]); // uses interrupts
      if (hal_status_ok == cmdStatus) txStr("$OK\r\n");
#else
		  cmdStatus = hal_status_notImplemented;
#endif
      break;

    case CmdEnum_SpiInt:
      txStr("\n");
			cmdStatus = Spi_Int(params[0], (uint16_t)params[1], (uint16_t)params[2]);
      if (hal_status_ok == cmdStatus) txStr("$OK\r\n");
		  break;

		case CmdEnum_SpiDma:
      txStr("\n");
			cmdStatus = Spi_Dma(params[0], (uint16_t)params[1], (uint16_t)params[2]);
      if (hal_status_ok == cmdStatus) txStr("$OK\r\n");
		  break;

    case CmdEnum_Help:
      HELP_PrintHelp();
      break;

    default:
      ERRMSG_TERM();
      return false;
  }
  if (hal_status_ok != cmdStatus){
    ERRMSG();
    if(hal_status_badParam == cmdStatus){
      txStr("Usage: $");
      txStr(UsageStrArray[(uint32_t)cmd]);
      txStr(" - ");
      txStr(DescStrArray[(uint32_t)cmd]);
    } else {
      txStr("cmd status: ");
      txStr(HAL_StatusToString(cmdStatus));
    }
    TERMMSG();
    return false;
  }
  return true;
}

/*******************************************************************************
 **
 ** @brief $readport [portnum] - Masked pins set to input, read IO port
 **
 ******************************************************************************/
static hal_status_t GPIO_Read(uint32_t port, uint32_t mask)
{
  uint16_t i;
  if(port > 6){ return hal_status_badParam; }
  if(port == 6){
    // PORTG
    mask &= 0x000000ff;
    VOR_GPIO->BANK[port].DIR &= ~mask;
    for(i=0; i<1000; i++);
    sprintf(txBuffer, "0x%02lx", VOR_GPIO->BANK[port].DATAIN);
  } else {
    // PORTA-F
    mask &= 0x0000ffff;
    VOR_GPIO->BANK[port].DIR &= ~mask;
    for(i=0; i<1000; i++);
    sprintf(txBuffer, "0x%04lx", VOR_GPIO->BANK[port].DATAIN);
  }
  OKMSG();
  txStr(txBuffer);
  return hal_status_ok;
}

/*******************************************************************************
 **
 ** @brief $setport [portnum] [mask] [val] - Sets masked pins hi/lo
 **
 ******************************************************************************/
static hal_status_t GPIO_SetOutput(uint32_t port, uint32_t mask, uint32_t val)
{
  if(port > 6){ return hal_status_badParam; }
  if(port == 6){
    // PORTG
    mask &= 0x000000ff;
    val &= 0x000000ff;
    VOR_GPIO->BANK[port].DIR |= mask;
    VOR_GPIO->BANK[port].DATAOUT &= ~mask;
    VOR_GPIO->BANK[port].DATAOUT |= val;
  } else {
    // PORTA-F
    mask &= 0x0000ffff;
    val &= 0x0000ffff;
    VOR_GPIO->BANK[port].DIR |= mask;
    VOR_GPIO->BANK[port].DATAOUT &= ~mask;
    VOR_GPIO->BANK[port].DATAOUT |= val;
  }
  return hal_status_ok;
}

#if 0
static hal_status_t ADC_timerTrigger(uint32_t timerNum, uint32_t channel, uint32_t freqHz)
{

  HAL_Irqrouter_SetAdcSel(timerNum);

  VOR_ADC->CTRL = 0;   
  VOR_ADC->CTRL |= (0x1UL << channel) << ADC_CTRL_CHAN_EN_Pos; // select channel
  VOR_ADC->FIFO_CLR = ADC_FIFO_CLR_FIFO_CLR_Msk;
  VOR_ADC->CTRL |= ADC_CTRL_EXT_TRIG_EN_Msk; // enable external convert

  hal_status_t stat = HAL_Timer_InitTimer(timerNum, 
    (stc_tim_cfg_t){
    .irq_en = true,
    .rst_value = (TIMER_CLK(timerNum)/freqHz)-1,
    .cnt_value = 0
    },
    true);
  return stat;
}
#endif

/*******************************************************************************
 **
 ** @brief $readadc [channel] - Read ADC channel
 **
 ******************************************************************************/
static hal_status_t ADC_ReadAndPrint(uint32_t channel)
{
#if !defined(__VA41628__) 
  uint16_t val;
  hal_status_t stat = hal_status_ok;
  if(channel > 15){ return hal_status_badParam; }
  stat = HAL_ADC_ReadSingle((uint8_t)channel, &val); // non-DMA example
  sprintf(txBuffer, "0x%03x, %ldmV", val, (val * ADC_VREF_MV) / ADC_MAX_COUNT);
  OKMSG();
  txStr(txBuffer);

  // timer trigger example
#if 0
  ADC_timerTrigger(8, channel, 10);
  while(1)
  {
    while(VOR_ADC->STATUS & ADC_STATUS_FIFO_ENTRY_CNT_Msk)
    {
      val = (uint16_t)VOR_ADC->FIFO_DATA;
      sprintf(txBuffer, "0x%03lx, %ldmV", val, (val * ADC_VREF_MV) / ADC_MAX_COUNT);
      OKMSG();
      txStr(txBuffer);
    }
    WDFEED();
  }
#endif

  // DMA example
#if 0
  uint32_t num_conv = 1; // one ADC conversion
  uint32_t dma_val;

  // set CTRL to 1 conversion, CONV_CNT=0 (n+1), CHAN_TAG_EN=0
  VOR_ADC->CTRL = 0;
  VOR_ADC->CTRL |= (0x1UL << channel) << ADC_CTRL_CHAN_EN_Pos; // select channel
  VOR_ADC->FIFO_CLR = ADC_FIFO_CLR_FIFO_CLR_Msk;

  // set RX trigger level to one, enable FIFO level interrupt
  VOR_ADC->RXFIFOIRQTRG = num_conv;
  VOR_ADC->IRQ_ENB = ADC_IRQ_ENB_FIFO_DEPTH_TRIG_Msk;

  // setup DMA
  HAL_Irqrouter_SetDmaSel(ADC_DMA_CH, en_irqr_dmasel_adc); // DMA triggers from ADC interrupt
  HAL_DMA_PeriphToSRAM32((uint32_t*)&VOR_ADC->FIFO_DATA, &dma_val, num_conv, ADC_DMA_CH);
  HAL_DMA_SetChannelEnable(ADC_DMA_CH, true);
  dmaRxDone = false; // set to true by DMA done ISR
  NVIC_EnableIRQ(ADC_DMA_IRQN);

  VOR_ADC->CTRL |= ADC_CTRL_MANUAL_TRIG_Msk; // start manual convert

  while(!dmaRxDone){} // wait for DMA xfer to complete

  if(hal_status_ok == stat){
    sprintf(txBuffer, "0x%03lx, %ldmV", dma_val, (dma_val * ADC_VREF_MV) / ADC_MAX_COUNT);
    OKMSG();
    txStr(txBuffer);
  }
#endif

  return stat;
#else
  return hal_status_notImplemented;
#endif
}

/*******************************************************************************
 **
 ** @brief $adctest - enable adc sweep 0-7 once/s
 **
 ******************************************************************************/
static hal_status_t ADC_TestEnable(void)
{
#if !defined(__VA41628__) 
  adcTestEn = true;
  return hal_status_ok;
#else
  return hal_status_notImplemented;
#endif
}

/*******************************************************************************
 **
 ** @brief $dietemp - read on-chip temp sensor
 **
 ******************************************************************************/
static hal_status_t ADC_PrintTemp(uint32_t enableSec)
{
  uint16_t tempRaw;
  HAL_ADC_ReadSingle(10, &tempRaw); // read channel 10
  float fDieTempC = ((((float)(tempRaw & 0xfff)/4095.0f)*(ADC_VREF)) - 1.5685f)/(-0.0032f);
  sprintf(txBuffer, "Die temp: %d*C", (int)fDieTempC);
  OKMSG();
  txStr(txBuffer);
  if(enableSec > 0)
  {
    dieTmpSecEn = true; //flag checked by OnceEverySecondTasks() to print temp every second
  }
  return hal_status_ok;
}

/*******************************************************************************
 **
 ** @brief $setdac [0 or 1] [0x0-0xfff] - Set DAC0 or DAC1 output
 **
 ******************************************************************************/
static hal_status_t DAC_SetOutput(uint32_t dacNum, uint32_t val)
{
#if !defined(__VA41628__) 
  if(dacNum > 1){ return hal_status_badParam; }
  if(val > 0xfff){ val = 0xfff; }
#ifdef INC_DAC_SINE
  DAC_StopDacLoop(dacNum);
#endif

  hal_status_t stat;
  if(0 == dacNum){ stat = HAL_DAC_ManualTrigger(VOR_DAC0, (uint16_t)val); }
  else           { stat = HAL_DAC_ManualTrigger(VOR_DAC1, (uint16_t)val); }
  return stat;
#else
  return hal_status_notImplemented;
#endif
}

/*******************************************************************************
 **
 ** @brief $rom1wr [addr] [data32] - Write 32 bits to the boot F-ram
 **
 ******************************************************************************/
static hal_status_t FRAMBOOT_Write(uint32_t addr, uint32_t data)
{
  if(addr > FRAM_LEN - 4){ return hal_status_badParam; }
  hal_status_t stat, errStat = hal_status_ok;
  stat = FRAM_Init(ROM_SPI_BANK, ROM_SPI_CSN);
  if(stat != hal_status_ok){ errStat = stat; }
  stat = FRAM_Write(ROM_SPI_BANK, addr, (uint8_t*)&data, 4);
  if(stat != hal_status_ok){ errStat = stat; }
  stat = FRAM_UnInit(ROM_SPI_BANK);
  if(stat != hal_status_ok){ errStat = stat; }
  return errStat;
}

/*******************************************************************************
 **
 ** @brief $rom1rd [addr] - Read 32 bits from the boot F-ram
 **
 ******************************************************************************/
static hal_status_t FRAMBOOT_ReadAndPrint(uint32_t addr)
{
  if(addr > FRAM_LEN - 4){ return hal_status_badParam; }
  hal_status_t stat, errStat = hal_status_ok;
  uint32_t readData = 0;
  stat = FRAM_Init(ROM_SPI_BANK, ROM_SPI_CSN);
  if(stat != hal_status_ok){ errStat = stat; }
  stat = FRAM_Read(ROM_SPI_BANK, addr, (uint8_t*)&readData, 4);
  if(stat != hal_status_ok){ errStat = stat; }
  stat = FRAM_UnInit(ROM_SPI_BANK);
  if(stat != hal_status_ok){ errStat = stat; }
  if(hal_status_ok == errStat){
    sprintf(txBuffer, "0x%08lx", readData);
    OKMSG();
    txStr(txBuffer);
  }
  return errStat;
}

/*******************************************************************************
 **
 ** @brief $auxwr [addr] [data32] - Write 32 bits to the aux F-ram
 **
 ******************************************************************************/
static hal_status_t FRAMAUX_Write(uint32_t addr, uint32_t data)
{
  if(addr > FRAM_LEN - 4){ return hal_status_badParam; }
  hal_status_t stat, errStat = hal_status_ok;
  stat = FRAM_Init(SPI1_BANK, 3);
  if(stat != hal_status_ok){ errStat = stat; }
  stat = FRAM_Write(SPI1_BANK, addr, (uint8_t*)&data, 4);
  if(stat != hal_status_ok){ errStat = stat; }
  stat = FRAM_UnInit(SPI1_BANK);
  if(stat != hal_status_ok){ errStat = stat; }
  return errStat;
}

/*******************************************************************************
 **
 ** @brief $auxrd [addr] - Read 32 bits from the aux F-ram
 **
 ******************************************************************************/
static hal_status_t FRAMAUX_ReadAndPrint(uint32_t addr)
{
#if !defined(__VA41628__) 
  if(addr > FRAM_LEN - 4){ return hal_status_badParam; }
  hal_status_t stat, errStat = hal_status_ok;
  uint32_t readData = 0;
  stat = FRAM_Init(FRAM_AUX_SPI_BANK, FRAM_AUX_SPI_CSN);
  if(stat != hal_status_ok){ errStat = stat; }
  stat = FRAM_Read(FRAM_AUX_SPI_BANK, addr, (uint8_t*)&readData, 4);
  if(stat != hal_status_ok){ errStat = stat; }
  stat = FRAM_UnInit(FRAM_AUX_SPI_BANK);
  if(stat != hal_status_ok){ errStat = stat; }
  if(hal_status_ok == errStat){
    sprintf(txBuffer, "0x%08lx", readData);
    OKMSG();
    txStr(txBuffer);
  }
  return errStat;
#else
  return hal_status_notImplemented;
#endif
}

/*******************************************************************************
 **
 ** @brief $ebiinit - Initialize the EBI
 **
 ******************************************************************************/
static hal_status_t EBI_Init(void)
{
#if !defined(__VA41628__) 
  hal_status_t stat = HAL_Iocfg_SetupPins(&ebiPinCfgArr[0]);
  if(hal_status_ok != stat){ return stat; }
  VOR_SYSCONFIG->PERIPHERAL_CLK_ENABLE |= CLK_ENABLE_EBI;
  VOR_SYSCONFIG->PERIPHERAL_RESET &= ~SYSCONFIG_PERIPHERAL_RESET_EBI_Msk;
  __NOP();
  __NOP();
  VOR_SYSCONFIG->PERIPHERAL_RESET |= SYSCONFIG_PERIPHERAL_RESET_EBI_Msk;

  //CFGxxxxCYCLE set to slowest for testing.  May work faster

  //512k Byte FRAM on CE0
  //addr[23:16] 0x0 to 0x7
  //0x6000_0000 -> 0x6007_ffff as Data Space using System Bus
  //0x1000_0000 -> 0x1007_ffff as Instruction Space using IBus for inst fetch, Data Bus for data
  // F-ram needs to use slowest settings only (100ns access time)
  VOR_SYSCONFIG->EBI_CFG0=(
     (0x00<<SYSCONFIG_EBI_CFG0_ADDRLOW0_Pos)
    |(0x07<<SYSCONFIG_EBI_CFG0_ADDRHIGH0_Pos)
    |(6<<SYSCONFIG_EBI_CFG0_CFGREADCYCLE_Pos)
    |(6<<SYSCONFIG_EBI_CFG0_CFGWRITECYCLE_Pos)
    |(6<<SYSCONFIG_EBI_CFG0_CFGTURNAROUNDCYCLE_Pos)
    |(SYSCONFIG_EBI_CFG0_CFGSIZE_Msk) //16 bit mode
  );

  //512k Byte SRAM on CE1
  //addr[23:16] 0x8 to 0xf
  //0x6008_0000 -> 0x600f_ffff as Data Space using System Bus
  //0x1008_0000 -> 0x100f_ffff as Instruction Space using IBus for inst fetch, Data Bus for data
  //fastest allowable read timing: 0x3
	VOR_SYSCONFIG->EBI_CFG1=(
     (0x08<<SYSCONFIG_EBI_CFG0_ADDRLOW0_Pos)
    |(0x0f<<SYSCONFIG_EBI_CFG0_ADDRHIGH0_Pos)
    |(3<<SYSCONFIG_EBI_CFG0_CFGREADCYCLE_Pos)
    |(0<<SYSCONFIG_EBI_CFG0_CFGWRITECYCLE_Pos)
    |(0<<SYSCONFIG_EBI_CFG0_CFGTURNAROUNDCYCLE_Pos)
    |(SYSCONFIG_EBI_CFG0_CFGSIZE_Msk) //16 bit mode
  );

  //not used, cannot disable, so map end of range
  VOR_SYSCONFIG->EBI_CFG2=(
     (0xfd<<SYSCONFIG_EBI_CFG0_ADDRLOW0_Pos)
    |(0xfd<<SYSCONFIG_EBI_CFG0_ADDRHIGH0_Pos)
    |(6<<SYSCONFIG_EBI_CFG0_CFGREADCYCLE_Pos)
    |(6<<SYSCONFIG_EBI_CFG0_CFGWRITECYCLE_Pos)
    |(6<<SYSCONFIG_EBI_CFG0_CFGTURNAROUNDCYCLE_Pos)
    |(SYSCONFIG_EBI_CFG0_CFGSIZE_Msk)  //16 bit mode
  );

  //not used, cannot disable, so map end of range
  VOR_SYSCONFIG->EBI_CFG3=(
     (0xff<<SYSCONFIG_EBI_CFG0_ADDRLOW0_Pos)
    |(0xff<<SYSCONFIG_EBI_CFG0_ADDRHIGH0_Pos)
    |(6<<SYSCONFIG_EBI_CFG0_CFGREADCYCLE_Pos)
    |(6<<SYSCONFIG_EBI_CFG0_CFGWRITECYCLE_Pos)
    |(6<<SYSCONFIG_EBI_CFG0_CFGTURNAROUNDCYCLE_Pos)
    |(SYSCONFIG_EBI_CFG0_CFGSIZE_Msk)  //16 bit mode
  );

  ebiInitialized = true;
  return stat;
#else
  return hal_status_notImplemented;
#endif
}

/*******************************************************************************
 **
 ** @brief $ebiwrite - Write a 32 bit word to EBI
 **
 ** @note  Requires EBI / ETH board installed to run correctly
 **
 ******************************************************************************/
static hal_status_t EBI_Write(uint32_t addr, uint32_t data)
{
#if !defined(__VA41628__) 
  if(false == ebiInitialized){ EBI_Init(); }
  if((addr < 0x60000000) || (addr >= 0x61000000)){
    if((addr < 0x10000000) || (addr >= 0x11000000)){ return hal_status_badParam; }
  }
  *(uint32_t*)addr = data;
  return hal_status_ok;
#else
  return hal_status_notImplemented;
#endif
}

/*******************************************************************************
 **
 ** @brief $ebiread - Read a 32 bit word from EBI
 **
 ** @note  Requires EBI / ETH board installed to run correctly
 **
 ******************************************************************************/
static hal_status_t EBI_ReadAndPrint(uint32_t addr)
{
#if !defined(__VA41628__) 
  if(false == ebiInitialized){ EBI_Init(); }
  if((addr < 0x60000000) || (addr >= 0x61000000)){
    if((addr < 0x10000000) || (addr >= 0x11000000)){ return hal_status_badParam; }
  }
  uint32_t data = *(uint32_t*)addr;
  sprintf(txBuffer, "0x%08lx", data);
  OKMSG();
  txStr(txBuffer);
  return hal_status_ok;
#else
  return hal_status_notImplemented;
#endif
}

/*******************************************************************************
 **
 ** @brief $cantest - CAN bus loopback test
 **
 ** @note  Req. CAN transceivers wired to CAN0 and CAN1, wired CANH-CANH and CANL-CANL
 **
 ******************************************************************************/
static hal_status_t CAN_Test(void)
{
#ifdef INC_CAN_TEST
  VOR_SYSCONFIG->PERIPHERAL_CLK_ENABLE |= CLK_ENABLE_CAN0 | CLK_ENABLE_CAN1;
  VOR_SYSCONFIG->PERIPHERAL_RESET &= ~(CLK_ENABLE_CAN0 | CLK_ENABLE_CAN1);
  __NOP();
  __NOP();
  VOR_SYSCONFIG->PERIPHERAL_RESET |= (CLK_ENABLE_CAN0 | CLK_ENABLE_CAN1);

  uint32_t* regPtr;
  for(regPtr=(uint32_t *)VOR_CAN0; regPtr<=(uint32_t *)&VOR_CAN0->CTMR; regPtr++){
    *regPtr = 0x0000;
  }
  for(regPtr=(uint32_t *)VOR_CAN1; regPtr<=(uint32_t *)&VOR_CAN1->CTMR; regPtr++){
    *regPtr = 0x0000;
  }

  uint32_t retStatus0 = can_test_loopback(VOR_CAN0);
  switch(retStatus0){
    case 0:
      break;
    case 1:
      return hal_status_timeout; // can0 tx timeout
    case 2:
      return hal_status_timeout; // can1 rx timeout
    case 3:
      return hal_status_rxError; // wrong data length
    case 4:
      return hal_status_rxError; // wrong data
  }
  return hal_status_ok;
#else
  return hal_status_notImplemented;
#endif
}

/*******************************************************************************
 **
 ** @brief $phytest - Ethernet PHY loopback test
 **
 ** @note  Requires EBI / ETH board installed to run
 **
 ******************************************************************************/
static hal_status_t PHY_Test(void)
{
#ifdef INC_PHY_TEST
  hal_status_t stat = HAL_Iocfg_SetupPins(&ethPinCfgArr[0]);
  if(hal_status_ok != stat){ return stat; }

  phytest_main();

  return hal_status_ok;
#else
  return hal_status_notImplemented;
#endif
}

/*******************************************************************************
 **
 ** @brief $spwtest - SpaceWire test
 **
 ******************************************************************************/
static hal_status_t SPW_Test(void)
{
  // Not yet implemented in this EVK software version

  return hal_status_notImplemented;
}

/*******************************************************************************
 **
 ** @brief $i2ctest - I2C test I2C1(ms)->I2C2(sl) - test writes and reads
 **
 ******************************************************************************/
static hal_status_t I2C_Test(void)
{
#ifdef INC_I2C_TEST
  stc_i2c_masterInit_t initm = {0};
  initm.speed = en_i2c_100k;

  hal_status_t stat = HAL_I2C_Reset(VOR_I2C1);
  if(stat != hal_status_ok){ return stat; }
  stat = HAL_I2CM_Init(VOR_I2C1, initm);
  if(stat != hal_status_ok){ return stat; }

  stc_i2c_slaveInit_t inits = {0};
  inits.slaveAddr = I2C2_SLAVE_ADDR;
  stat = HAL_I2C_Reset(VOR_I2C2);
  if(stat != hal_status_ok){ return stat; }
  stat = HAL_I2CS_Init(VOR_I2C2, inits);
  if(stat != hal_status_ok){ return stat; }

  // write test
  char i2cWrite = 0x55;
  char i2c_rx = 0;
  stat = HAL_I2CS_Listen(VOR_I2C2, &i2c_rx, 1, 1);
  if(stat != hal_status_ok){ return stat; }
  stat = HAL_I2CM_WriteB(VOR_I2C1, I2C2_SLAVE_ADDR, &i2cWrite, 1);
  if(stat != hal_status_ok){ return stat; }

  // check result
  if(i2c_rx != 0x55){
    return hal_status_rxError;
  }

  // read test
#define I2C_RDTEST_NUM_BYTES 32
  char i2csWrite[I2C_RDTEST_NUM_BYTES];
  char i2cRead[I2C_RDTEST_NUM_BYTES];

  // setup slave (tx 32 bytes)
  for(uint32_t i=0; i<I2C_RDTEST_NUM_BYTES; i++){
    i2csWrite[i] = 0x50+i;
    i2cRead[i] = 0;
  }
  stat = HAL_I2CS_Respond(VOR_I2C2, i2csWrite, I2C_RDTEST_NUM_BYTES);
  if(stat != hal_status_ok){ return stat; }

  // Perform read test
  stat = HAL_I2CM_ReadB(VOR_I2C1, I2C2_SLAVE_ADDR, i2cRead, I2C_RDTEST_NUM_BYTES);
  if(stat != hal_status_ok){ return stat; }

  // check result
  for(uint32_t i=0; i<I2C_RDTEST_NUM_BYTES; i++){
    if(i2cRead[i] != i2csWrite[i]) return hal_status_rxError;
  }

  return hal_status_ok;
#else
	return hal_status_notImplemented;
#endif
}

/*******************************************************************************
 **
 ** @brief $accel - Get accelerometer data
 **
 ******************************************************************************/
static hal_status_t I2C_GetAccelData(uint32_t enableSec)
{
#ifdef INC_ACCEL_TEST
  hal_status_t stat = (*accelInterface.init)(&accelHandle);
  if(stat != hal_status_ok){ return stat; }

  stc_accel_mg_values_t accelMg; // units: milli-G
  stat = (*accelInterface.getMgValues)(&accelHandle, &accelMg);
  if(stat != hal_status_ok){ return stat; }

  sprintf(txBuffer, "Accel data: X:%ld Y:%ld Z:%ld milli-g", (int32_t)accelMg.x, \
                                                     (int32_t)accelMg.y, \
                                                     (int32_t)accelMg.z);
  OKMSG();
  txStr(txBuffer);

  if(enableSec > 0){accelSecEn = true; }

  return hal_status_ok;
#else
  return hal_status_notImplemented;
#endif
}

/*******************************************************************************
 **
 ** @brief $brdtmp - Get board temperature from accelerometer chip
 **
 ******************************************************************************/
static hal_status_t I2C_GetBoardTemp(uint32_t enableSec)
{
#ifdef INC_ACCEL_TEST
  hal_status_t stat = (*accelInterface.init)(&accelHandle);
  if(stat != hal_status_ok){ return stat; }

  float temp;
  stat = (*accelInterface.getTemp)(&accelHandle, &temp);
  if(stat != hal_status_ok){ return stat; }

  sprintf(txBuffer, "Board temp: %ld*C", (uint32_t)temp);
  OKMSG();
  txStr(txBuffer);

  if(enableSec > 0){ brdTmpSecEn = true; }

  return hal_status_ok;
#else
  return hal_status_notImplemented;
#endif
}

/*******************************************************************************
 **
 ** @brief $clkpll - Clock to PLL and set frequency (7 - 100 MHz)
 **
 ******************************************************************************/
static hal_status_t Clk_PLL(uint32_t freq)
{
  if((freq < PLLOUT_MIN_HZ) || (freq > PLLOUT_MAX_HZ)){ return hal_status_badParam; }
  hal_status_t stat = HAL_Clkgen_Init((hal_clkgen_init_t){.xtalsel = gCurrentXtalsel, \
                                                      .clksel = hal_clksel_sys_pll, \
                                                      .pllcfg = hal_pllcfg_enabled, \
                                                      .clk_div_sel = hal_clk_div_1x, \
                                                      .lost_det_en = true, \
                                                      .pll_out_hz = freq});
  SystemCoreClockUpdate(); // calculate new SystemCoreClock
  OnSystemClockChanged(); // update peripherals
  return stat;
}

/*******************************************************************************
 **
 ** @brief $clkhbo - Clock to internal 20-22mhz oscillator
 **
 ******************************************************************************/
static hal_status_t Clk_HBO(void)
{
  hal_status_t stat = HAL_Clkgen_Init((hal_clkgen_init_t){.xtalsel = hal_xtalsel_none, \
                                                      .clksel = hal_clksel_sys_hbo, \
                                                      .pllcfg = hal_pllcfg_pwrdn, \
                                                      .clk_div_sel = hal_clk_div_1x, \
                                                      .lost_det_en = true});
  SystemCoreClockUpdate(); // calculate new SystemCoreClock
  OnSystemClockChanged(); // update peripherals
  return stat;
}

/*******************************************************************************
 **
 ** @brief $clkext - Clock to XTAL in (XTAL_N) - xtal_p not enabled
 **
 ******************************************************************************/
static hal_status_t Clk_Ext(void)
{
  gCurrentXtalsel = hal_xtalsel_xtal_n;
  hal_status_t stat = HAL_Clkgen_Init((hal_clkgen_init_t){.xtalsel = gCurrentXtalsel, \
                                                      .clksel = hal_clksel_sys_xtal_n, \
                                                      .pllcfg = hal_pllcfg_pwrdn, \
                                                      .clk_div_sel = hal_clk_div_1x, \
                                                      .lost_det_en = true});
  SystemCoreClockUpdate(); // calculate new SystemCoreClock
  OnSystemClockChanged(); // update peripherals
  return stat;
}

/*******************************************************************************
 **
 ** @brief $clkxtal - Clock to crystal amplifier (XTAL_P, XTAL_N)
 **
 ******************************************************************************/
static hal_status_t Clk_Xtal(void)
{
  gCurrentXtalsel = hal_xtalsel_xtal_osc;
  hal_status_t stat = HAL_Clkgen_Init((hal_clkgen_init_t){.xtalsel = gCurrentXtalsel, \
                                                      .clksel = hal_clksel_sys_xtal_osc, \
                                                      .pllcfg = hal_pllcfg_pwrdn, \
                                                      .clk_div_sel = hal_clk_div_1x, \
                                                      .lost_det_en = true});
  SystemCoreClockUpdate(); // calculate new SystemCoreClock
  OnSystemClockChanged(); // update peripherals
  return stat;
}

/*******************************************************************************
 **
 ** @brief Sets up the driver handle and peripheral for SPI int and DMA examples
 **
 ******************************************************************************/
static void Spi_Setup(hal_spi_handle_t* hspi, uint32_t bank)
{
  HAL_UNLOCK(hspi);
  if(bank == 0){
    hspi->spi = VOR_SPI0;
  } else if (bank == 1){
    hspi->spi = VOR_SPI1;
  } else if (bank == 2){
    hspi->spi = VOR_SPI2;
  } else {
    hspi->spi = VOR_SPI3;
  }
  hspi->init.blockmode = true;
  hspi->init.bmstall = true;
  hspi->init.clkDiv = 8;
  hspi->init.loopback = true;
  hspi->init.mdlycap = false;
  hspi->init.mode = hal_spi_clkmode_0;
  hspi->init.ms = hal_spi_ms_master;
  hspi->init.chipSelect = 0;
  hspi->init.wordLen = 16;
  HAL_Spi_Init(hspi);
  HAL_Spi_ConfigDMA(hspi, SPI_DMA_TX_CH, SPI_DMA_RX_CH);
}

/*******************************************************************************
 **
 ** @brief $spiint - SPI Interrupt example
 **
 ******************************************************************************/
static hal_status_t Spi_Int(uint32_t bank, uint16_t len, uint16_t txwd)
{
#ifdef INC_SPI_INT

	if(len>1024)
  {
    return hal_status_badParam;
  }
	uint16_t * txbuf = (uint16_t*)malloc(2*len);
	uint16_t * rxbuf = (uint16_t*)malloc(2*len);

  for(int i=0; i<len; i++)
  {
    txbuf[i] = txwd;
  }
  
  hal_status_t status;
  Spi_Setup(&spiHandle, bank);
  status = HAL_Spi_TransmitReceiveInt(&spiHandle, txbuf, rxbuf, len);
  if(status != hal_status_ok)
  {
    goto end;
  }

  while(spiHandle.state == hal_spi_state_busy); // wait for complete

  if(spiHandle.state == hal_spi_state_error)
  {
    status = hal_status_bufFull; // FIFO overrun error
  }
  else
  {
    for(int i=0; i<len; i++)
    {
      if(rxbuf[i] != txbuf[i]) status = hal_status_rxError; // receive data mismatch
    }
  }

end:
  free(txbuf);
	free(rxbuf);
  //HAL_Spi_DeInit(&spiHandle);

  return status;

#else
  return hal_status_notImplemented;
#endif
}

/*******************************************************************************
 **
 ** @brief $spidma - SPI DMA example
 **
 ******************************************************************************/
static hal_status_t Spi_Dma(uint32_t bank, uint16_t len, uint16_t txwd)
{
#ifdef INC_SPI_DMA
	uint16_t i;
	hal_status_t status;

	if(len>1024) return hal_status_badParam;
	uint32_t * txbuf = (uint32_t*)malloc(4*len);
	uint16_t * rxbuf = (uint16_t*)malloc(2*len);

  /* Set up SRAM */
  for(i=0; i<len; i++)
  {
    txbuf[i] = txwd;
    rxbuf[i] = 0;
  }
  txbuf[len-1] |= SPI_DATA_BMSTART_BMSTOP_Msk;

  /* SPI is set up here as the master */
  Spi_Setup(&spiHandle, bank);

  /* begin transfer */
  status = HAL_Spi_TransmitReceiveDMA(&spiHandle, txbuf, rxbuf, len);
  if(status != hal_status_ok)
  {
    goto end;
  }

  while(spiHandle.state == hal_spi_state_busy); // wait for complete
 
  /* check receive data */
  for(int i=0; i<len; i++)
  {
    if(rxbuf[i] != (uint16_t)txbuf[i]) status = hal_status_rxError; // receive data mismatch
  }

end:
  free(txbuf);
	free(rxbuf);
  //HAL_Spi_DeInit(&spiHandle);

  return status;
#else
	return hal_status_notImplemented;
#endif
}

/*******************************************************************************
 **
 ** @brief $help - Print help
 **
 ******************************************************************************/
static void HELP_PrintHelp(void)
{
  TERMMSG();
  for(uint32_t i=1; i<(uint32_t)CmdEnum_End; i++){
    txStr("$");
    txStr(UsageStrArray[i]);
    txStr(" - ");
    txStr(DescStrArray[i]);
    TERMMSG();
    if(((i%8)==0) && (txDest == TxDest_RTT)){
      // take a break, let the rtt catch up
      for(uint32_t j=0; j<200000; j++){ VOR_WATCH_DOG->WDOGINTCLR = 1; }
    }
  }
}

/*******************************************************************************
**                               ,----.
**                              ( WOW! )                         .-.
**                               `----' _                         \ \
**                                     (_)                         \ \
**                                         O                       | |
**                    |\ /\                  o                     | |
**    __              |,\(_\_                  . /\---/\   _,---._ | |
**   ( (              |\,`   `-^.               /^   ^  \,'       `. ;
**    \ \             :    `-'   )             ( O   O   )           ;
**     \ \             \        ;               `.=o=__,'            \
**      \ \             `-.   ,'                  /         _,--.__   \
**       \ \ ____________,'  (                   /  _ )   ,'   `-. `-. \
**        ; '                ;                  / ,' /  ,'        \ \ \ \
**        \                 /___,-.            / /  / ,'          (,_)(,_)
**         `,    ,_____|  ;'_____,'           (,;  (,,)      jrei
**       ,-" \  :      | :
**      ( .-" \ `.__   | |
**       \__)  `.__,'  |__)  SSt
**
**
**  VA416xx EVK Demo Software
**  Auth: TAlexander, DMiller
**
******************************************************************************/

/*****************************************************************************/
/* End of file                                                               */
/*****************************************************************************/
