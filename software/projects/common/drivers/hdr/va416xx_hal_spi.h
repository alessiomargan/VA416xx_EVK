/***************************************************************************************
 * @file     va416xx_hal_spi.h
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
 
#ifndef __HAL_SPI_H
#define __HAL_SPI_H

/*****************************************************************************/ 
/* Include files                                                             */ 
/*****************************************************************************/

#include "va416xx_hal.h"
#include "hal_config.h"

/*****************************************************************************/ 
/* Global pre-processor symbols/macros ('#define')                           */ 
/*****************************************************************************/

#define HAL_SPI_VERSION     (0x00020005) // 2.05

#define SPI_MASTER_MSK      (0xFU) // SPI 0-3 support master mode
#define SPI_SLAVE_MSK       (0x7U) // SPI 0-2 support slave mode

#define SPI_NUM_BANKS       (4U)
#define SPI_MAX_BANKNUM     (SPI_NUM_BANKS-1)

#define SPI0_BANK           (0U)
#define SPI1_BANK           (1U)
#define SPI2_BANK           (2U)
#define SPI3_BANK           (3U)

/** SPI peripheral clock source */
#define SPI_CLK             (SystemCoreClock/2) // SPI peripherals reside on APB1

/** each SPI word can be between 4 and 16 bits long */
#define SPI_MIN_WORDLEN     (4U)
#define SPI_MAX_WORDLEN     (16U)

/* missing from some versions of va416xx.h */
/* ----------------------------------  SPI_DATA  --------------------------- */
#ifndef SPI_DATA_VALUE_Pos
#define SPI_DATA_VALUE_Pos   			 					0                                                      /*!< SPIB DATA: VALUE Position 							*/
#endif
#ifndef SPI_DATA_VALUE_Msk
#define SPI_DATA_VALUE_Msk   			 					(0x000000ffffUL << SPI_DATA_VALUE_Pos)   						 /*!< SPIB DATA: VALUE Mask   								*/
#endif
#ifndef SPI_DATA_BMSKIPDATA_Pos
#define SPI_DATA_BMSKIPDATA_Pos   	 					30                                                     /*!< SPIB DATA: BMSKIPDATA Position 					*/
#endif
#ifndef SPI_DATA_BMSKIPDATA_Msk
#define SPI_DATA_BMSKIPDATA_Msk   	 					(0x01UL << SPI_DATA_BMSKIPDATA_Pos)         					 /*!< SPIB DATA: BMSKIPDATA Mask   						*/
#endif
#ifndef SPI_DATA_BMSTART_BMSTOP_Pos
#define SPI_DATA_BMSTART_BMSTOP_Pos 					31                                                     /*!< SPIB DATA: BMSTART/BMSTOP Position 			*/
#endif
#ifndef SPI_DATA_BMSTART_BMSTOP_Msk
#define SPI_DATA_BMSTART_BMSTOP_Msk 					(0x01UL << SPI_DATA_BMSTART_BMSTOP_Pos)     					 /*!< SPIB DATA: BMSTART/BMSTOP Mask   				*/
#endif

#define HAL_SPI_SEND_BMSTOP(__HANDLE__)                                                    \
    do{                                                                                    \
      if((__HANDLE__)->init.blockmode)                                                     \
      {                                                                                    \
        while((hspi->spi->STATUS & SPI_STATUS_TNF_Msk) == 0)                               \
        {                                                                                  \
          if((HAL_GetTimeMs() - timestart) >= timeout)                                     \
          {                                                                                \
            goto timeout;                                                                  \
          }                                                                                \
        }                                                                                  \
        (__HANDLE__)->spi->DATA = SPI_DATA_BMSTART_BMSTOP_Msk | SPI_DATA_BMSKIPDATA_Msk;   \
      }                                                                                    \
    } while(0)

/*****************************************************************************/
/* Global type definitions ('typedef')                                       */ 
/*****************************************************************************/

/**
  * @brief  SPI clocking/phase mode enum definition (mode 0-3)
  */
typedef enum
{
  hal_spi_clkmode_0           = 0x00U,
  hal_spi_clkmode_1           = 0x01U,
  hal_spi_clkmode_2           = 0x02U,
  hal_spi_clkmode_3           = 0x03U
} hal_spi_clkmode_t;

/**
  * @brief  SPI master or slave enum definition
  */
typedef enum 
{
  hal_spi_ms_master           = 0x00U,
  hal_spi_ms_slave            = 0x01U
} hal_spi_ms_t;

/**
  * @brief  SPI state enum definition
  */
typedef enum {
  hal_spi_state_reset         = 0x00U,     /*!< The peripheral is not Initialized                  */
  hal_spi_state_ready         = 0x01U,     /*!< SPI peripheral is initialized and ready for use    */
  hal_spi_state_busy          = 0x02U,     /*!< an internal process is ongoing                     */
  hal_spi_state_error         = 0x03U      /*!< SPI is in an error state - recommend re-initialize */
} hal_spi_state_t;

/**
  * @brief  SPI initialization struct definition
  */
typedef struct
{
  hal_spi_ms_t ms;
  hal_spi_clkmode_t mode;
  uint16_t clkDiv;
  uint8_t wordLen;
  uint8_t chipSelect;
  bool loopback;
  bool blockmode;
  bool bmstall;
  bool mdlycap;
} hal_spi_init_t;

/**
  * @brief  SPI transfer struct definition
  */
typedef struct 
{
  void *txbuf;          // transmit buffer address, or NULL for rx only
  void *rxbuf;          // receive buffer address, or NULL for tx only
  uint16_t len;         // transfer length (must be nonzero)
  volatile uint16_t sent;
  volatile uint16_t received;
  uint8_t bmstop   : 1; // 1: eassert CSn on last word of the xfer if blockmode (when master)
} hal_spi_xfer_t;

/**
  * @brief  SPI handle struct definition
  */
typedef struct
{
  VOR_SPI_Type*    spi;       /* Pointer to SPI peripheral hardware */
  hal_spi_init_t   init;      /* Initialization settings struct */
  hal_spi_xfer_t   xfer;      /* Struct containing transfer information */
  IRQn_Type        txirqn;    /* TX irq number */
  IRQn_Type        rxirqn;    /* RX irq number */
  volatile hal_spi_state_t  state;     /* SPI state */
  volatile bool             locked;    /* Set to true when this driver is modifying this handle, do not modify handle at this time */
  uint8_t          txdma_ch;  /* Transmit DMA channel for DMA based transfers */
  uint8_t          rxdma_ch;  /* Receive DMA channel for DMA based transfers */
  uint8_t          bank;      /* SPI bank number 0-3 */
} hal_spi_handle_t;

/*****************************************************************************/
/* Global variable declarations ('extern', definition in C source)           */
/*****************************************************************************/

/*****************************************************************************/ 
/* Global function prototypes ('extern', definition in C source)             */ 
/*****************************************************************************/

#ifdef __HAL_SPI_MODULE_ENABLED

/** Init and deinit, config */
extern hal_status_t HAL_Spi_Init           (hal_spi_handle_t* const hspi);
extern hal_status_t HAL_Spi_DeInit         (hal_spi_handle_t* const hspi);
extern hal_status_t HAL_Spi_SetClkDiv      (hal_spi_handle_t* const hspi);
extern hal_status_t HAL_Spi_SetChipSelect  (hal_spi_handle_t* const hspi, uint8_t csNum);
extern hal_status_t HAL_Spi_ConfigDMA      (hal_spi_handle_t* const hspi, uint8_t txchannel, uint8_t rxchannel);

/** Master mode transfers */

/** Polling transfers (blocking functions) */
extern hal_status_t HAL_Spi_Transmit       (hal_spi_handle_t* const hspi, void* txbuf, uint16_t len, uint32_t timeout, bool bmstop);
extern hal_status_t HAL_Spi_Receive        (hal_spi_handle_t* const hspi, void* rxbuf, uint16_t len, uint32_t timeout);
extern hal_status_t HAL_Spi_TransmitReceive(hal_spi_handle_t* const hspi, void* txbuf, void* rxbuf, uint16_t rxstart, uint16_t txlen, uint16_t rxlen, uint32_t timeout, bool bmstop);

/** Interrupt based transfers, non-blocking */
extern hal_status_t HAL_Spi_TransmitInt       (hal_spi_handle_t* const hspi, void* txbuf, uint16_t len);
extern hal_status_t HAL_Spi_ReceiveInt        (hal_spi_handle_t* const hspi, void* rxbuf, uint16_t len);
extern hal_status_t HAL_Spi_TransmitReceiveInt(hal_spi_handle_t* const hspi, void* txbuf, void* rxbuf, uint16_t len);

/** DMA based transfers, non-blocking */
extern hal_status_t HAL_Spi_TransmitDMA       (hal_spi_handle_t* const hspi, uint32_t* txbuf, uint16_t len);
extern hal_status_t HAL_Spi_ReceiveDMA        (hal_spi_handle_t* const hspi, void* rxbuf, uint16_t len);
extern hal_status_t HAL_Spi_TransmitReceiveDMA(hal_spi_handle_t* const hspi, uint32_t* txbuf, void* rxbuf, uint16_t len);

#endif /* __HAL_SPI_MODULE_ENABLED */

/*****************************************************************************/ 
/* End of file                                                               */ 
/*****************************************************************************/
#endif /* __HAL_SPI_H */
