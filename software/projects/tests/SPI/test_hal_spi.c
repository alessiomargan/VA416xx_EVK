/***************************************************************************************
 * @file     test_hal_spi.c
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
#include <stdlib.h>
#include "test_hal_spi.h"
#include "va416xx_hal_spi.h"
#include "va416xx_hal_dma.h"


/*****************************************************************************/
/* Local pre-processor symbols/macros ('#define')                            */
/*****************************************************************************/

/*****************************************************************************/
/* Global variable definitions (declared in header file with 'extern')       */
/*****************************************************************************/

/*****************************************************************************/
/* Local type definitions ('typedef')                                        */
/*****************************************************************************/

typedef enum
{
  errorType_none = 0,
  errorType_mismatch = 1,
  errorType_timeout = 2,
  errorType_rxovr = 3,
  errorType_param = 4
} errorType_t;

typedef enum
{
  transferType_blocking,
  transferType_interrupt,
  transferType_dma
} transferType_t;

typedef struct errorRecord
{
  struct errorRecord* next;
  VOR_SPI_Type* spi;
  transferType_t transferType;
  errorType_t errorType;
  uint16_t bufferLength;
  uint8_t wordLen;
  uint8_t clkDiv;
  bool internal;
  bool mdlycap;
} errorRecord_t;

/*****************************************************************************/
/* Local variable definitions ('static')                                     */
/*****************************************************************************/

/* Error recording linked list */
static errorRecord_t* errorRecord = NULL;

/* SPI interrupt driver test section */
static hal_spi_handle_t spiHandle;
static volatile bool complete;
static volatile hal_status_t spiStat;

/*****************************************************************************/
/* Local function prototypes ('static')                                      */
/*****************************************************************************/

/*****************************************************************************/
/* Function implementation - global ('extern') and local ('static')          */
/*****************************************************************************/

static void addErrorRecord(VOR_SPI_Type* spi, transferType_t transferType, \
                           errorType_t errorType, uint16_t bufferLength, uint8_t wordLen, \
                           uint8_t clkDiv, bool internal, bool mdlycap)
{
  errorRecord_t* newRecord = (errorRecord_t*)malloc(sizeof(errorRecord_t));
  newRecord->spi = spi;
  newRecord->transferType = transferType;
  newRecord->errorType = errorType;
  newRecord->bufferLength = bufferLength;
  newRecord->wordLen = wordLen;
  newRecord->clkDiv = clkDiv;
  newRecord->internal = internal;
  newRecord->mdlycap = mdlycap;
  newRecord->next = NULL;
  if(errorRecord == NULL) errorRecord = newRecord;
  else
  {
    // traverse to end of the linked list, add newRecord to the end
    errorRecord_t* current = errorRecord;
    while(current->next != NULL)
    {
      current = current->next;
    }
    current->next = newRecord;
  }
}

static void printEntireErrorRecordAndClear(void)
{
  if(errorRecord == NULL)
  {
    printf("\nError record empty\n\n");
    return;
  }
  errorRecord_t* current = errorRecord;
  errorRecord_t* toFree;
  printf("\nPrinting error record:\n\n");
  while(current != NULL)
  {
    switch((uint32_t)current->spi)
    {
      case (uint32_t)VOR_SPI0:
        printf("Error on SPI0\n");
        break;
      case (uint32_t)VOR_SPI1:
        printf("Error on SPI1\n");
        break;
      case (uint32_t)VOR_SPI2:
        printf("Error on SPI2\n");
        break;
      case (uint32_t)VOR_SPI3:
        printf("Error on SPI3\n");
        break;
      default:
        break;
    }
    printf("Transfer type: %d\n", (int)current->transferType);
    printf("Error type: %d\n", (int)current->errorType);
    printf("Buffer length: %u\n", current->bufferLength);
    printf("Word len: %u\n", current->wordLen);
    printf("Clkdiv: %u\n", current->clkDiv);
    printf("Internal LB: %s\n", current->internal?"true":"false");
    printf("Mdlycap: %s\n\n", current->mdlycap?"true":"false");

    toFree = current;
    current = current->next;
    free(toFree);
    HAL_WaitMs(20);
  }
}

static void zero(void* mem, size_t len)
{
  while(len)
  {
    len--;
    ((uint8_t*)mem)[len] = 0;
  }
}

static void setup_init()
{
  HAL_UNLOCK(&spiHandle);
  spiHandle.spi = VOR_SPI0;
  spiHandle.init.blockmode = true;
  spiHandle.init.bmstall = true;
  spiHandle.init.clkDiv = 8;
  spiHandle.init.loopback = true;
  spiHandle.init.mdlycap = false;
  spiHandle.init.mode = hal_spi_clkmode_0;
  spiHandle.init.ms = hal_spi_ms_master;
  spiHandle.init.chipSelect = 0;
  spiHandle.init.wordLen = 8;
}

static bool spi_loopback_test_blocking_8(hal_spi_handle_t* hspi, uint8_t* pattern_buf, uint16_t pattern_len)
{
  bool match = true;
  uint8_t* rxbuf = malloc((size_t)pattern_len);
  zero(rxbuf, (size_t)pattern_len);
  printf("Running 8-bit blocking loopback test\n");
  spiStat = HAL_Spi_TransmitReceive(&spiHandle, pattern_buf, rxbuf, 0, pattern_len, pattern_len, 500, true);
  if(spiStat != hal_status_ok)
  {
    printf("Error: HAL_Spi_TransmitReceive() status: %s\n", HAL_StatusToString(spiStat));
    free(rxbuf);
    HAL_Spi_DeInit(hspi);
    return false;
  }
  if(spiStat == hal_status_ok)
  {
    for(int i=0; i<pattern_len; i++)
    {
      if(rxbuf[i] != pattern_buf[i]) match = false;
    }
  } else {
    if(spiStat == hal_status_rxError) printf("Error: receive overrun\n");
    match = false;
  }
  free(rxbuf);
  if(match)
  {
    printf("Transfer OK\n");
    return true;
  }
  printf("Error: receive mismatch\n");
  addErrorRecord(hspi->spi, transferType_blocking, errorType_mismatch, pattern_len, hspi->init.wordLen, hspi->init.clkDiv, hspi->init.loopback, hspi->init.mdlycap);
  HAL_Spi_DeInit(hspi);
  return false;
}

static bool spi_loopback_test_blocking_16(hal_spi_handle_t* hspi, uint16_t* pattern_buf, uint16_t pattern_len)
{
  bool match = true;
  uint16_t* rxbuf = malloc((size_t)(pattern_len * 2));
  zero(rxbuf, (size_t)(pattern_len * 2));
  printf("Running 16-bit blocking loopback test\n");
  spiStat = HAL_Spi_TransmitReceive(&spiHandle, pattern_buf, rxbuf, 0, pattern_len, pattern_len, 500, true);
  if(spiStat != hal_status_ok)
  {
    printf("Error: HAL_Spi_TransmitReceive() status: %s\n", HAL_StatusToString(spiStat));
    free(rxbuf);
    HAL_Spi_DeInit(hspi);
    return false;
  }
  if(spiStat == hal_status_ok)
  {
    for(int i=0; i<pattern_len; i++)
    {
      if(rxbuf[i] != pattern_buf[i]) match = false;
    }
  } else {
    if(spiStat == hal_status_rxError) printf("Error: receive overrun\n");
    match = false;
  }
  free(rxbuf);
  if(match)
  {
    printf("Transfer OK\n");
    return true;
  }
  printf("Error: receive mismatch\n");
  addErrorRecord(hspi->spi, transferType_blocking, errorType_mismatch, pattern_len, hspi->init.wordLen, hspi->init.clkDiv, hspi->init.loopback, hspi->init.mdlycap);
  HAL_Spi_DeInit(hspi);
  return false;
}

static bool spi_loopback_test_int_8(hal_spi_handle_t* hspi, uint8_t* pattern_buf, uint16_t pattern_len)
{
  complete = false;
  bool match = true;
  uint8_t* rxbuf = malloc((size_t)pattern_len);
  zero(rxbuf, (size_t)pattern_len);
  printf("Running 8-bit interrupt loopback test\n");
  spiStat = HAL_Spi_TransmitReceiveInt(&spiHandle, pattern_buf, rxbuf, pattern_len);
  if(spiStat != hal_status_ok)
  {
    printf("Error: HAL_Spi_TransmitReceive_Int() status: %s\n", HAL_StatusToString(spiStat));
    free(rxbuf);
    HAL_Spi_DeInit(hspi);
    return false;
  }
  while(!complete);
  if(spiStat == hal_status_ok)
  {
    for(int i=0; i<pattern_len; i++)
    {
      if(rxbuf[i] != pattern_buf[i]) match = false;
    }
  } else {
    if(spiStat == hal_status_rxError) printf("Error: receive overrun\n");
    match = false;
  }
  free(rxbuf);
  if(match)
  {
    printf("Transfer OK\n");
    return true;
  }
  printf("Error: receive mismatch\n");
  addErrorRecord(hspi->spi, transferType_interrupt, errorType_mismatch, pattern_len, hspi->init.wordLen, hspi->init.clkDiv, hspi->init.loopback, hspi->init.mdlycap);
  HAL_Spi_DeInit(hspi);
  return false;
}

static bool spi_loopback_test_int_16(hal_spi_handle_t* hspi, uint16_t* pattern_buf, uint16_t pattern_len)
{
  complete = false;
  bool match = true;
  uint16_t* rxbuf = malloc((size_t)(pattern_len * 2));
  zero(rxbuf, (size_t)(pattern_len * 2));
  printf("Running 16-bit interrupt loopback test\n");
  spiStat = HAL_Spi_TransmitReceiveInt(&spiHandle, pattern_buf, rxbuf, pattern_len);
  if(spiStat != hal_status_ok)
  {
    printf("Error: HAL_Spi_TransmitReceive_Int() status: %s\n", HAL_StatusToString(spiStat));
    free(rxbuf);
    HAL_Spi_DeInit(hspi);
    return false;
  }
  while(!complete);
  if(spiStat == hal_status_ok)
  {
    for(int i=0; i<pattern_len; i++)
    {
      if(rxbuf[i] != pattern_buf[i]) match = false;
    }
  } else {
    if(spiStat == hal_status_rxError) printf("Error: receive overrun\n");
    match = false;
  }
  free(rxbuf);
  if(match)
  {
    printf("Transfer OK\n");
    return true;
  }
  printf("Error: receive mismatch\n");
  addErrorRecord(hspi->spi, transferType_interrupt, errorType_mismatch, pattern_len, hspi->init.wordLen, hspi->init.clkDiv, hspi->init.loopback, hspi->init.mdlycap);
  HAL_Spi_DeInit(hspi);
  return false;
}

bool spi_loopback_test_dma_8(hal_spi_handle_t* hspi, uint32_t* pattern_buf, uint16_t pattern_len)
{
  int timeout = 500; // ms
  complete = false;
  bool match = true;
  uint8_t* rxbuf = malloc((size_t)(pattern_len));
  zero(rxbuf, (size_t)pattern_len);
  printf("Running 8-bit DMA loopback test\n");
  spiStat = HAL_Spi_TransmitReceiveDMA(&spiHandle, pattern_buf, rxbuf, pattern_len);
  if(spiStat != hal_status_ok)
  {
    printf("Error: HAL_Spi_TransmitReceive_DMA() status: %s\n", HAL_StatusToString(spiStat));
    free(rxbuf);
    HAL_Spi_DeInit(hspi);
    return false;
  }
  while(!complete){
    if(--timeout == 0)
    {
      printf("Error: timeout\n");
      break;
    }
    HAL_WaitMs(1);
  }

  if(spiStat == hal_status_rxError) 
    printf("Error: receive overrun\n");

  for(uint16_t i=0; i<pattern_len; i++)
  {
    if(rxbuf[i] != (uint8_t)pattern_buf[i]) match = false;
  } 
  free(rxbuf);
  if(match)
  {
    printf("Transfer OK\n");
    return true;
  }
  printf("Error: receive mismatch\n");
  addErrorRecord(hspi->spi, transferType_dma, errorType_mismatch, pattern_len, hspi->init.wordLen, hspi->init.clkDiv, hspi->init.loopback, hspi->init.mdlycap);
  HAL_Spi_DeInit(hspi);
  return false;
}

bool spi_loopback_test_dma_16(hal_spi_handle_t* hspi, uint32_t* pattern_buf, uint16_t pattern_len)
{
  int timeout = 1000; // ms
  complete = false;
  bool match = true;
  uint16_t* rxbuf = malloc((size_t)(pattern_len * 2));
  zero(rxbuf, (size_t)(pattern_len * 2));
  printf("Running 16-bit DMA loopback test\n");
  spiStat = HAL_Spi_TransmitReceiveDMA(&spiHandle, pattern_buf, rxbuf, pattern_len);
  if(spiStat != hal_status_ok)
  {
    printf("Error: HAL_Spi_TransmitReceive_DMA() status: %s\n", HAL_StatusToString(spiStat));
    free(rxbuf);
    HAL_Spi_DeInit(hspi);
    return false;
  }
  while(!complete){
    if(--timeout == 0)
    {
      printf("Error: timeout\n");
      break;
    }
    HAL_WaitMs(1);
  }

  if(spiStat == hal_status_rxError) 
    printf("Error: receive overrun\n");

  for(uint16_t i=0; i<pattern_len; i++)
  {
    if(rxbuf[i] != (uint16_t)pattern_buf[i]) match = false;
  } 
  free(rxbuf);
  if(match)
  {
    printf("Transfer OK\n");
    return true;
  }
  printf("Error: receive mismatch\n");
  addErrorRecord(hspi->spi, transferType_dma, errorType_mismatch, pattern_len, hspi->init.wordLen, hspi->init.clkDiv, hspi->init.loopback, hspi->init.mdlycap);
  HAL_Spi_DeInit(hspi);
  return false;
}

static hal_status_t spi_clk_test(uint16_t pattern_len)
{
  uint8_t* txbuf = malloc((size_t)(pattern_len));
  uint16_t* txbuf16 = malloc((size_t)(pattern_len * 2));
  uint32_t* txbuf32 = malloc((size_t)(pattern_len * 4));

  printf("SPI interrupt driver clock speed test\n");
  printf("Buffer length = %u\n", pattern_len);

  for(uint32_t i=0; i<pattern_len; i++)
  {
    txbuf[i] = i;
    txbuf16[i] = i;
    txbuf32[i] = i;
  }
  txbuf32[pattern_len-1] |= SPI_DATA_BMSTART_BMSTOP_Msk;
  
  // walk through different clock rates
  for(uint16_t clk = 2; clk < 64; clk += 2)
  {
    printf("Testing clkdiv = %d\n", clk);


    if(spiHandle.locked == false)
    {
      spiHandle.init.clkDiv = clk;
      spiHandle.init.wordLen = 8;
    } else {
      printf("Error: SPI handle locked\n");
    }
    spiStat = HAL_Spi_Init(&spiHandle);
    if(spiStat != hal_status_ok)
    {
      printf("Error: HAL_Spi_Init() status: %s\n", HAL_StatusToString(spiStat));
      continue;
    }
    (void)spi_loopback_test_blocking_8(&spiHandle, txbuf, pattern_len);

    if(spiHandle.locked == false)
    {
      spiHandle.init.wordLen = 16;
    } else {
      printf("Error: SPI handle locked\n");
    }
    spiStat = HAL_Spi_Init(&spiHandle);
    if(spiStat != hal_status_ok)
    {
      printf("Error: HAL_Spi_Init() status: %s\n", HAL_StatusToString(spiStat));
      continue;
    }
    (void)spi_loopback_test_blocking_16(&spiHandle, txbuf16, pattern_len);

    if(spiHandle.locked == false)
    {
      spiHandle.init.clkDiv = clk;
      spiHandle.init.wordLen = 8;
    } else {
      printf("Error: SPI handle locked\n");
    }
    spiStat = HAL_Spi_Init(&spiHandle);
    if(spiStat != hal_status_ok)
    {
      printf("Error: HAL_Spi_Init() status: %s\n", HAL_StatusToString(spiStat));
      continue;
    }
    (void)spi_loopback_test_int_8(&spiHandle, txbuf, pattern_len);


    if(spiHandle.locked == false)
    {
      spiHandle.init.wordLen = 16;
    } else {
      printf("Error: SPI handle locked\n");
    }
    spiStat = HAL_Spi_Init(&spiHandle);
    if(spiStat != hal_status_ok)
    {
      printf("Error: HAL_Spi_Init() status: %s\n", HAL_StatusToString(spiStat));
      continue;
    }
    (void)spi_loopback_test_int_16(&spiHandle, txbuf16, pattern_len);

    if(spiHandle.locked == false)
    {
      spiHandle.init.clkDiv = clk;
      spiHandle.init.wordLen = 8;
    } else {
      printf("Error: SPI handle locked\n");
    }
    spiStat = HAL_Spi_Init(&spiHandle);
    if(spiStat != hal_status_ok)
    {
      printf("Error: HAL_Spi_Init() status: %s\n", HAL_StatusToString(spiStat));
      continue;
    }
    spiStat = HAL_Spi_ConfigDMA(&spiHandle, 0, 1);
    if(spiStat != hal_status_ok)
    {
      printf("Error: HAL_Spi_ConfigDMA() status: %s\n", HAL_StatusToString(spiStat));
      continue;
    }
    (void)spi_loopback_test_dma_8(&spiHandle, txbuf32, pattern_len);


    if(spiHandle.locked == false)
    {
      spiHandle.init.wordLen = 16;
    } else {
      printf("Error: SPI handle locked\n");
    }
    spiStat = HAL_Spi_Init(&spiHandle);
    if(spiStat != hal_status_ok)
    {
      printf("Error: HAL_Spi_Init() status: %s\n", HAL_StatusToString(spiStat));
      continue;
    }
    spiStat = HAL_Spi_ConfigDMA(&spiHandle, 0, 1);
    if(spiStat != hal_status_ok)
    {
      printf("Error: HAL_Spi_ConfigDMA() status: %s\n", HAL_StatusToString(spiStat));
      continue;
    }
    (void)spi_loopback_test_dma_16(&spiHandle, txbuf32, pattern_len);
  }
  free(txbuf);
  free(txbuf16);
  free(txbuf32);
  return hal_status_ok;
}

void HAL_Spi_Cmplt_Callback(hal_spi_handle_t* hspi)
{
  if(hspi == &spiHandle)
  {
    if(hspi->state != hal_spi_state_error)
    {
      spiStat = hal_status_ok;
      complete = true;
    } else {
      spiStat = hal_status_rxError; // receive overrun
      complete = true;
    }
  } 
}

static void clock_test_main(void)
{
  spiHandle.init.loopback = true;
  spiHandle.spi = VOR_SPI0;
  printf("Testing internal loopback on SPI0\n");
  spi_clk_test(127);
  spi_clk_test(512);
  HAL_Spi_DeInit(&spiHandle);
  HAL_WaitMs(100);
  spiHandle.init.loopback = false;
  spiHandle.spi = VOR_SPI1;
  printf("Testing external loopback on SPI1\n");
  spi_clk_test(127);
  spi_clk_test(512);
  HAL_WaitMs(100);
  printf("Test with mdlycap = true\n");
  spiHandle.init.mdlycap = true;
  spiHandle.init.loopback = true;
  spiHandle.spi = VOR_SPI0;
  printf("Testing internal loopback on SPI0\n");
  spi_clk_test(127);
  spi_clk_test(512);
  HAL_WaitMs(100);
  HAL_Spi_DeInit(&spiHandle);
  spiHandle.init.loopback = false;
  spiHandle.spi = VOR_SPI1;
  printf("Testing external loopback on SPI1\n");
  spi_clk_test(127);
  spi_clk_test(512);
  HAL_WaitMs(100);
}

void test_hal_spi(void)
{
  setup_init();

  // route SPI1 to some pins
  // for external loopback, bridge MOSI and MISO (PF4, PF5)
  HAL_Iocfg_PinMux(PORTF, 2, 1);
  HAL_Iocfg_PinMux(PORTF, 3, 1);
  HAL_Iocfg_PinMux(PORTF, 4, 1);
  HAL_Iocfg_PinMux(PORTF, 5, 1);
  
  clock_test_main();
  printEntireErrorRecordAndClear();

  //spiHandle.spi = VOR_SPI0;
  //spiHandle.init.loopback = true;
  //spiHandle.init.mdlycap = false;
}

/*****************************************************************************/
/* End of file                                                               */
/*****************************************************************************/