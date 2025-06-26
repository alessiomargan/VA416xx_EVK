/***************************************************************************************
 * @file     va416xx_hal_spi.c
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
#include "va416xx_hal_spi.h"
#include "va416xx_hal_dma.h"
#include "va416xx_hal_irqrouter.h"

#ifdef __HAL_SPI_MODULE_ENABLED

/*****************************************************************************/
/* Local pre-processor symbols/macros ('#define')                            */
/*****************************************************************************/

#define SPI_PERID           (0x021307E9)    /* Expected Peripheral ID */
#define SPI_FIFO_SZ         (16U)           /* Hardware FIFO length */

#define SPI_INVALID_BANKNUM (0xFFU)         /* flag for an invalid index */

#define SPI_TIMEOUT_MS      (100U)          /* default timeout in milliseconds */

#define SPI_PUMP            (0x00U)         /* word to be sent when in master RX only */

#define HAL_SPI_ENABLE(__HANDLEPTR__)  __HANDLEPTR__->spi->CTRL1 |= SPI_CTRL1_ENABLE_Msk
#define HAL_SPI_DISABLE(__HANDLEPTR__) __HANDLEPTR__->spi->CTRL1 &= ~SPI_CTRL1_ENABLE_Msk

/*****************************************************************************/
/* Global variable definitions (declared in header file with 'extern')       */
/*****************************************************************************/

/*****************************************************************************/
/* Local type definitions ('typedef')                                        */
/*****************************************************************************/

/*****************************************************************************/
/* Local variable definitions ('static')                                     */
/*****************************************************************************/

static const uint8_t spiNumChipSelects[SPI_NUM_BANKS] = {4, 8, 7, 1};
static hal_spi_handle_t* spiHandles[SPI_NUM_BANKS] = {NULL, NULL, NULL, NULL};
static uint32_t dummyWord;

/*****************************************************************************/
/* Local function prototypes ('static')                                      */
/*****************************************************************************/

__STATIC_INLINE void Spi_EnableRxIrq(hal_spi_handle_t* const hspi);
__STATIC_INLINE void Spi_EnableTxIrq(hal_spi_handle_t* const hspi); // disabled to remove not referenced warning
__STATIC_INLINE bool Spi_DisableRxIrq(hal_spi_handle_t* const hspi);
__STATIC_INLINE bool Spi_DisableTxIrq(hal_spi_handle_t* const hspi);
__STATIC_INLINE void Spi_ClearFifo(hal_spi_handle_t* const hspi);
__STATIC_INLINE void Spi_LoadTxFifo(hal_spi_handle_t* const hspi);
__STATIC_INLINE void Spi_Callback(hal_spi_handle_t* const hspi);
__STATIC_INLINE void Spi_StateMachine(hal_spi_handle_t* const hspi);

static void HAL_Spi_Rx_Dma_Callback(hal_spi_handle_t* const hspi);

/*****************************************************************************/
/* Function implementation - global ('extern') and local ('static')          */
/*****************************************************************************/

/*******************************************************************************
 **
 ** @brief  Enables a SPI's RX IRQ in the NVIC
 **
 ** @param  hspi - pointer to SPI handle
 **
 ******************************************************************************/
__STATIC_INLINE void Spi_EnableRxIrq(hal_spi_handle_t* const hspi)
{
  NVIC_EnableIRQ(hspi->rxirqn);
}

/*******************************************************************************
 **
 ** @brief  Enables a SPI's TX IRQ in the NVIC
 **
 ** @param  hspi - pointer to SPI handle
 **
 ******************************************************************************/
// Currently disabled because it is not used - suppresses not referenced warning
__STATIC_INLINE void Spi_EnableTxIrq(hal_spi_handle_t* const hspi)
{
  NVIC_EnableIRQ(hspi->txirqn);
}

/*******************************************************************************
 **
 ** @brief  Disables a SPI's RX IRQ in the NVIC
 **
 ** @param  hspi - pointer to SPI handle
 **
 ** @return bool - true if the RX IRQ for that bank was previously enabled, else false
 **
 ******************************************************************************/
__STATIC_INLINE bool Spi_DisableRxIrq(hal_spi_handle_t* const hspi)
{
  if(NVIC_GetEnableIRQ(hspi->rxirqn))
  {
    NVIC_DisableIRQ(hspi->rxirqn);
    return true;
  }
  return false;
}

/*******************************************************************************
 **
 ** @brief  Disables a SPI's TX IRQ in the NVIC
 **
 ** @param  hspi - pointer to SPI handle
 **
 ** @return bool - true if the TX IRQ for that bank was previously enabled, else false
 **
 ******************************************************************************/
__STATIC_INLINE bool Spi_DisableTxIrq(hal_spi_handle_t* const hspi)
{
  if(NVIC_GetEnableIRQ(hspi->txirqn))
  {
    NVIC_DisableIRQ(hspi->txirqn);
    return true;
  }
  return false;
}

/*******************************************************************************
 **
 ** @brief  Resets SPI RX and TX hardware FIFOs
 **
 ** @param  hspi - pointer to SPI handle
 **
 ******************************************************************************/
__STATIC_INLINE void Spi_ClearFifo(hal_spi_handle_t* const hspi)
{
  // Clear Tx & RX fifo
  hspi->spi->FIFO_CLR = SPI_FIFO_CLR_RXFIFO_Msk | SPI_FIFO_CLR_TXFIFO_Msk;
}

/*******************************************************************************
 **
 ** @brief  Loads the SPI TX hardware fifo until full or tx complete
 **
 ** @param  hspi - pointer to SPI handle
 **
 ******************************************************************************/
__STATIC_INLINE void Spi_LoadTxFifo(hal_spi_handle_t* const hspi)
{
  uint32_t wrWord = 0;
  uint8_t maxWrite = 8; // prevents RX overflow at high clock rates
  if(hspi->xfer.txbuf == NULL){
    wrWord = SPI_PUMP;
    while(hspi->xfer.sent < hspi->xfer.len)
    {
      if((hspi->spi->STATUS & SPI_STATUS_TNF_Msk) == 0) break;
      if((hspi->xfer.sent == (hspi->xfer.len - 1)) && (hspi->xfer.bmstop))
      {
        wrWord |= SPI_DATA_BMSTART_BMSTOP_Msk;
      }
      hspi->spi->DATA = wrWord;
      hspi->xfer.sent++;
      if(--maxWrite == 0) break;
    }
  } else {
    uint8_t*  txbuf8 =  (uint8_t*)((uint32_t)hspi->xfer.txbuf + hspi->xfer.sent);
    uint16_t* txbuf16 = (uint16_t*)(txbuf8 + hspi->xfer.sent);
    if(hspi->init.wordLen > 8)
    {
      while(hspi->xfer.sent < hspi->xfer.len)
      {
        if((hspi->spi->STATUS & SPI_STATUS_TNF_Msk) == 0) break;
        wrWord = *txbuf16++;
        if((hspi->xfer.sent == (hspi->xfer.len - 1)) && (hspi->xfer.bmstop)){
          wrWord |= SPI_DATA_BMSTART_BMSTOP_Msk;
        }
        hspi->spi->DATA = wrWord;
        hspi->xfer.sent++;
        if(--maxWrite == 0) break;
      }
    } else {
      while(hspi->xfer.sent < hspi->xfer.len)
      {
        if((hspi->spi->STATUS & SPI_STATUS_TNF_Msk) == 0) break;
        wrWord = *txbuf8++;
        if((hspi->xfer.sent == (hspi->xfer.len - 1)) && (hspi->xfer.bmstop)){
          wrWord |= SPI_DATA_BMSTART_BMSTOP_Msk;
        }
        hspi->spi->DATA = wrWord;
        hspi->xfer.sent++;
        if(--maxWrite == 0) break;
      }
    }
  }
}

/*******************************************************************************
 **
 ** @brief  This weak function is intended to be overridden in the user application
 **         to perform user defined SPI init, usually the setting up of the GPIO
 **         alternate function selects if they aren't already set to be SPI pins
 **
 ** @param  hspi - pointer to SPI handle structure
 **
 ** @return void
 **
 ******************************************************************************/
void __attribute__((weak)) HAL_Spi_UserInit(hal_spi_handle_t* const hspi)
{

}

/*******************************************************************************
 **
 ** @brief  This weak function is intended to be overridden in the user application
 **         as a transfer complete callback for non-blocking SPI transfers
 **         (interrupt and DMA)
 **
 ** @param  hspi - pointer to SPI handle structure
 **
 ** @return void
 **
 ******************************************************************************/
void __attribute__((weak)) HAL_Spi0_Cmplt_Callback(hal_spi_handle_t* const hspi)
{

}

/*******************************************************************************
 **
 ** @brief  This weak function is intended to be overridden in the user application
 **         as a transfer complete callback for non-blocking SPI transfers
 **         (interrupt and DMA)
 **
 ** @param  hspi - pointer to SPI handle structure
 **
 ** @return void
 **
 ******************************************************************************/
void __attribute__((weak)) HAL_Spi1_Cmplt_Callback(hal_spi_handle_t* const hspi)
{

}

/*******************************************************************************
 **
 ** @brief  This weak function is intended to be overridden in the user application
 **         as a transfer complete callback for non-blocking SPI transfers
 **         (interrupt and DMA)
 **
 ** @param  hspi - pointer to SPI handle structure
 **
 ** @return void
 **
 ******************************************************************************/
void __attribute__((weak)) HAL_Spi2_Cmplt_Callback(hal_spi_handle_t* const hspi)
{

}

/*******************************************************************************
 **
 ** @brief  This weak function is intended to be overridden in the user application
 **         as a transfer complete callback for non-blocking SPI transfers
 **         (interrupt and DMA)
 **
 ** @param  hspi - pointer to SPI handle structure
 **
 ** @return void
 **
 ******************************************************************************/
void __attribute__((weak)) HAL_Spi3_Cmplt_Callback(hal_spi_handle_t* const hspi)
{

}

/*******************************************************************************
 **
 ** @brief  This weak function is intended to be overridden in the user application
 **         as a transfer complete callback for non-blocking SPI transfers
 **         (interrupt and DMA). Use this one if you want a single callback for all
 **         SPI peripherals
 **
 ** @param  hspi - pointer to SPI handle structure
 **
 ** @return void
 **
 ******************************************************************************/
void __attribute__((weak)) HAL_Spi_Cmplt_Callback(hal_spi_handle_t* const hspi)
{

}

/*******************************************************************************
 **
 ** @brief  Used for DMA transfers, called by DMA RX channel DONE interrupt
 **
 ** @param  hspi - pointer to SPI handle structure
 **
 ** @return void
 **
 ******************************************************************************/
static void HAL_Spi_Rx_Dma_Callback(hal_spi_handle_t* const hspi)
{
  if(hspi == NULL)
  {
    return;
  }
  NVIC_DisableIRQ((IRQn_Type)(DMA_DONE0_IRQn+hspi->rxdma_ch));
  VOR_DMA->CHNL_ENABLE_CLR = (1UL << hspi->txdma_ch) | (1UL << hspi->rxdma_ch);
  HAL_Irqrouter_SetDmaSel(hspi->txdma_ch, en_irqr_dmasel_none);
  HAL_Irqrouter_SetDmaSel(hspi->rxdma_ch, en_irqr_dmasel_none);
  hspi->locked = 1;
  hspi->spi->IRQ_ENB = 0;
  hspi->state = hal_spi_state_ready;
  HAL_UNLOCK(hspi);
  Spi_Callback(hspi);
}

/*******************************************************************************
 **
 ** @brief  Calls SPI transfer complete callback function
 **
 ** @param  hspi - pointer to SPI handle
 **
 ******************************************************************************/
__STATIC_INLINE void Spi_Callback(hal_spi_handle_t* const hspi)
{
  /* call peripheral specific callbacks */
  switch((uint32_t)(hspi->spi)){
    case VOR_SPI0_BASE:
      HAL_Spi0_Cmplt_Callback(hspi);
      break;

    case VOR_SPI1_BASE:
      HAL_Spi1_Cmplt_Callback(hspi);
      break;

    case VOR_SPI2_BASE:
      HAL_Spi2_Cmplt_Callback(hspi);
      break;

    case VOR_SPI3_BASE:
      HAL_Spi3_Cmplt_Callback(hspi);
      break;
  }
  /* call generic callback */
  HAL_Spi_Cmplt_Callback(hspi);
}

/*******************************************************************************
 **
 ** @brief  Resets and initializes a SPI peripheral
 **
 ** @param  hspi - pointer to a hal_spi_handle_t structure that contains the
 **                configuration information for the SPI peripheral
 **
 ** @return
 **
 ******************************************************************************/
hal_status_t HAL_Spi_Init(hal_spi_handle_t* const hspi)
{
  hal_status_t status;
  if(hspi == NULL)
  {
    return hal_status_badParam;
  }
  HAL_LOCK(hspi);
  hspi->state = hal_spi_state_reset;
  hspi->txdma_ch = 0xff;
  hspi->rxdma_ch = 0xff;

  /* Enable clock and reset SPI */
  switch((uint32_t)(hspi->spi)){
    case VOR_SPI0_BASE:
      VOR_SYSCONFIG->PERIPHERAL_CLK_ENABLE |= CLK_ENABLE_SPI0;
      VOR_SYSCONFIG->PERIPHERAL_RESET &= ~SYSCONFIG_PERIPHERAL_RESET_SPI0_Msk;
      __NOP();
      __NOP();
      VOR_SYSCONFIG->PERIPHERAL_RESET |= SYSCONFIG_PERIPHERAL_RESET_SPI0_Msk;
      hspi->bank = SPI0_BANK;
      hspi->txirqn = SPI0_TX_IRQn;
      hspi->rxirqn = SPI0_RX_IRQn;
      break;

    case VOR_SPI1_BASE:
      VOR_SYSCONFIG->PERIPHERAL_CLK_ENABLE |= CLK_ENABLE_SPI1;
      VOR_SYSCONFIG->PERIPHERAL_RESET &= ~SYSCONFIG_PERIPHERAL_RESET_SPI1_Msk;
      __NOP();
      __NOP();
      VOR_SYSCONFIG->PERIPHERAL_RESET |= SYSCONFIG_PERIPHERAL_RESET_SPI1_Msk;
      hspi->bank = SPI1_BANK;
      hspi->txirqn = SPI1_TX_IRQn;
      hspi->rxirqn = SPI1_RX_IRQn;
      break;

    case VOR_SPI2_BASE:
      VOR_SYSCONFIG->PERIPHERAL_CLK_ENABLE |= CLK_ENABLE_SPI2;
      VOR_SYSCONFIG->PERIPHERAL_RESET &= ~SYSCONFIG_PERIPHERAL_RESET_SPI2_Msk;
      __NOP();
      __NOP();
      VOR_SYSCONFIG->PERIPHERAL_RESET |= SYSCONFIG_PERIPHERAL_RESET_SPI2_Msk;
      hspi->bank = SPI2_BANK;
      hspi->txirqn = SPI2_TX_IRQn;
      hspi->rxirqn = SPI2_RX_IRQn;
      break;

    case VOR_SPI3_BASE:
      VOR_SYSCONFIG->PERIPHERAL_CLK_ENABLE |= CLK_ENABLE_SPI3;
      VOR_SYSCONFIG->PERIPHERAL_RESET &= ~SYSCONFIG_PERIPHERAL_RESET_SPI3_Msk;
      __NOP();
      __NOP();
      VOR_SYSCONFIG->PERIPHERAL_RESET |= SYSCONFIG_PERIPHERAL_RESET_SPI3_Msk;
      hspi->bank = SPI3_BANK;
      hspi->txirqn = SPI3_TX_IRQn;
      hspi->rxirqn = SPI3_RX_IRQn;
      break;

    default:
      /* invalid spi address */
      hspi->state = hal_spi_state_error;
      HAL_UNLOCK(hspi);
      return hal_status_badParam;
  }

  /* Call user setup (GPIO init, etc) */
  HAL_Spi_UserInit(hspi);

  /* check peripheral ID */
  if(hspi->spi->PERID != SPI_PERID){
    hspi->state = hal_spi_state_error;
    HAL_UNLOCK(hspi);
    return hal_status_badPeriphID; /* not running on correct hw, or a register issue */
  }

  // Clear FIFOs
  Spi_ClearFifo(hspi);
  hspi->spi->CTRL0 = 0;
  hspi->spi->CTRL1 = 0;

  // SPI master (default) or slave operation
  if(hspi->init.ms == hal_spi_ms_slave)
  {
    if(hspi->spi == VOR_SPI3) return hal_status_badParam; // SPI3 cannot be a slave
    hspi->spi->CTRL1 |= SPI_CTRL1_MS_Msk;
  }

  // SPI clocking mode
  switch(hspi->init.mode)
  {
    case hal_spi_clkmode_0:
      break;
    case hal_spi_clkmode_1:
      hspi->spi->CTRL0 |= (SPI_CTRL0_SPH_Msk);
      break;
    case hal_spi_clkmode_2:
      hspi->spi->CTRL0 |= (SPI_CTRL0_SPO_Msk);
      break;
    case hal_spi_clkmode_3:
      hspi->spi->CTRL0 |= (SPI_CTRL0_SPO_Msk | SPI_CTRL0_SPH_Msk);
      break;
    default:
      hspi->state = hal_spi_state_error;
      HAL_UNLOCK(hspi);
      return hal_status_badParam;
  }

  // Clock divider
  status = HAL_Spi_SetClkDiv(hspi);
  if(status != hal_status_ok){
    hspi->state = hal_spi_state_error;
    HAL_UNLOCK(hspi);
    return status;
  }

  // Word length
  if((hspi->init.wordLen < SPI_MIN_WORDLEN) || (hspi->init.wordLen > SPI_MAX_WORDLEN)){
    hspi->state = hal_spi_state_error;
    HAL_UNLOCK(hspi);
    return hal_status_badParam;
  }
  hspi->spi->CTRL0 |= ((hspi->init.wordLen-1U) << SPI_CTRL0_SIZE_Pos);

  // Chip select pin
  if(hspi->init.chipSelect >= spiNumChipSelects[hspi->bank]){
    hspi->state = hal_spi_state_error;
    HAL_UNLOCK(hspi);
    return hal_status_badParam;
  }
  hspi->spi->CTRL1 |= (hspi->init.chipSelect << SPI_CTRL1_SS_Pos);

  // Loopback
  if(hspi->init.loopback){
    hspi->spi->CTRL1 |= SPI_CTRL1_LBM_Msk;
  }

  // Blockmode
  if(hspi->init.blockmode){
    hspi->spi->CTRL1 |= SPI_CTRL1_BLOCKMODE_Msk;
  }

  // BMStart (always true/enabled for now)
  hspi->spi->CTRL1 |= SPI_CTRL1_BMSTART_Msk;

  // BMStall
  if(hspi->init.bmstall){
    hspi->spi->CTRL1 |= SPI_CTRL1_BMSTALL_Msk;
  }

  // Master delayed capture (MDLYCAP)
  if(hspi->init.mdlycap){
    hspi->spi->CTRL1 |= SPI_CTRL1_MDLYCAP_Msk;
  }

  /* Enable SPI */
  HAL_SPI_ENABLE(hspi);
  hspi->state = hal_spi_state_ready;
  HAL_UNLOCK(hspi);
  return hal_status_ok;
}

/*******************************************************************************
 **
 ** @brief  De-initializes a SPI peripheral. Declocks and puts the SPI into reset
 **
 ** @param  hspi - pointer to SPI handle structure
 **
 ******************************************************************************/
hal_status_t HAL_Spi_DeInit(hal_spi_handle_t* const hspi)
{
  if(hspi == NULL)
  {
    return hal_status_badParam;
  }
  HAL_LOCK(hspi);

  Spi_DisableRxIrq(hspi);
  Spi_DisableTxIrq(hspi);
  HAL_DMA_SetChannelEnable(hspi->txdma_ch, false);
  HAL_DMA_SetChannelEnable(hspi->rxdma_ch, false);
  HAL_DMA_RegDoneCallback(hspi->txdma_ch, NULL, NULL);
  HAL_DMA_RegDoneCallback(hspi->rxdma_ch, NULL, NULL);

  // Put SPI into reset and then deassert clock
  switch((uint32_t)(hspi->spi)){
    case VOR_SPI0_BASE:
      VOR_SYSCONFIG->PERIPHERAL_RESET &= ~SYSCONFIG_PERIPHERAL_RESET_SPI0_Msk;
      __NOP();
      __NOP();
      VOR_SYSCONFIG->PERIPHERAL_CLK_ENABLE &= ~CLK_ENABLE_SPI0;
      break;

    case VOR_SPI1_BASE:
      VOR_SYSCONFIG->PERIPHERAL_RESET &= ~SYSCONFIG_PERIPHERAL_RESET_SPI1_Msk;
      __NOP();
      __NOP();
      VOR_SYSCONFIG->PERIPHERAL_CLK_ENABLE &= ~CLK_ENABLE_SPI1;
      break;

    case VOR_SPI2_BASE:
      VOR_SYSCONFIG->PERIPHERAL_RESET &= ~SYSCONFIG_PERIPHERAL_RESET_SPI2_Msk;
      __NOP();
      __NOP();
      VOR_SYSCONFIG->PERIPHERAL_CLK_ENABLE &= ~CLK_ENABLE_SPI2;
      break;

    case VOR_SPI3_BASE:
      VOR_SYSCONFIG->PERIPHERAL_RESET &= ~SYSCONFIG_PERIPHERAL_RESET_SPI3_Msk;
      __NOP();
      __NOP();
      VOR_SYSCONFIG->PERIPHERAL_CLK_ENABLE &= ~CLK_ENABLE_SPI3;
      break;

    default:
      // invalid spi address
      hspi->state = hal_spi_state_error;
      HAL_UNLOCK(hspi);
      return hal_status_badParam;
  }
  hspi->state = hal_spi_state_reset;
  HAL_UNLOCK(hspi);
  return hal_status_ok;
}

/*******************************************************************************
 **
 ** @brief  Set CLKPRESCALE and SCRDV bits according to div value (1-65024)
 **
 ** @param  hspi - pointer to SPI handle
 **
 ******************************************************************************/
hal_status_t HAL_Spi_SetClkDiv(hal_spi_handle_t* const hspi)
{
  uint32_t div = hspi->init.clkDiv;
  uint32_t scrdv_val = 0, prescale_val = 0;
  uint32_t i;

  if(div == 0){
    // illegal clkdiv
    return hal_status_badParam;
  }

  // find largest (even) prescale value that divides into div
  for(i=2; i<=0xfe; i+=2){
    if((div % i) == 0){
      prescale_val = i;
    }
  }
  if(prescale_val == 0){
    // divide value must be even
    return hal_status_badParam;
  }
  div /= prescale_val;

  // scrdv
  if(div <= 256){
    scrdv_val = div - 1;
  }else{
    return hal_status_badParam;
  }

  // set registers
  hspi->spi->CLKPRESCALE = prescale_val;
  hspi->spi->CTRL0 &= ~(SPI_CTRL0_SCRDV_Msk);
  hspi->spi->CTRL0 |= (scrdv_val << SPI_CTRL0_SCRDV_Pos);

  return hal_status_ok;
}

/*******************************************************************************
 **
 ** @brief  Sets the SPI chip select number to use
 **
 ** @param  hspi - pointer to SPI handle
 **
 ** @param csNum - the chip select number
 **
 ** @return hal_status_t driver call status
 **
 ******************************************************************************/
hal_status_t HAL_Spi_SetChipSelect(hal_spi_handle_t* const hspi, uint8_t csNum)
{
  if(hspi == NULL)
    return hal_status_badParam;
  if(csNum >= spiNumChipSelects[hspi->bank])
    return hal_status_badParam;

  HAL_LOCK(hspi);
  hspi->init.chipSelect = csNum;
  hspi->spi->CTRL1 = (hspi->spi->CTRL1 & ~SPI_CTRL1_SS_Msk) | (hspi->init.chipSelect << SPI_CTRL1_SS_Pos);
  HAL_UNLOCK(hspi);
  return hal_status_ok;
}

/*******************************************************************************
 **
 ** @brief  Setup DMA channels to use for DMA transfers. DMA peripheral must
 ** already be enabled by calling HAL_DMA_Init() (usually at software boot)
 **
 ** @param  hspi - pointer to SPI handle
 **
 ** @return hal_status_t driver call status
 **
 ******************************************************************************/
hal_status_t HAL_Spi_ConfigDMA(hal_spi_handle_t* const hspi, uint8_t txchannel, uint8_t rxchannel)
{
  if(hspi == NULL)
    return hal_status_badParam;
  if(txchannel >= DMA_NUM_CHNLS)
    return hal_status_badParam;
  if(rxchannel >= DMA_NUM_CHNLS)
    return hal_status_badParam;

  HAL_LOCK(hspi);
  hspi->txdma_ch = txchannel;
  hspi->rxdma_ch = rxchannel;

  stc_dma_control_blk_t* dma_blk = (stc_dma_control_blk_t*)VOR_DMA->CTRL_BASE_PTR;
  VOR_DMA->CHNL_ENABLE_CLR = (1UL << hspi->txdma_ch) | (1UL << hspi->rxdma_ch);
  VOR_DMA->CHNL_PRI_ALT_CLR = (1UL << txchannel) | (1UL << rxchannel);

  /* Tx channel setup */
  dma_blk->pri[txchannel].ctrl_raw = 0; // zero out ctrl
  dma_blk->pri[txchannel].ctrl.cycle_ctrl = DMA_CHNL_CFG_CYCLE_CTRL_BASIC;
  dma_blk->pri[txchannel].ctrl.src_size = DMA_CHNL_CFG_SIZE_WORD;
  dma_blk->pri[txchannel].dst = (uint32_t)(&hspi->spi->DATA);
  dma_blk->pri[txchannel].ctrl.dst_size = DMA_CHNL_CFG_SIZE_WORD;
  dma_blk->pri[txchannel].ctrl.dst_inc = DMA_CHNL_CFG_INC_NONE; // dest ptr increment
  dma_blk->pri[txchannel].ctrl.r_power = DMA_CHNL_CFG_R_POWER_EACH; // rearbitrate every 4 transfers

  /* Rx channel setup */
  dma_blk->pri[rxchannel].ctrl_raw = 0; // zero out ctrl
  dma_blk->pri[rxchannel].ctrl.cycle_ctrl = DMA_CHNL_CFG_CYCLE_CTRL_BASIC;
  dma_blk->pri[rxchannel].src = (uint32_t)(&hspi->spi->DATA);
  dma_blk->pri[rxchannel].ctrl.src_inc = DMA_CHNL_CFG_INC_NONE; // source ptr increment
  dma_blk->pri[rxchannel].ctrl.r_power = DMA_CHNL_CFG_R_POWER_EACH; // rearbitrate each

  /* Setup IRQ router */
  VOR_SYSCONFIG->PERIPHERAL_CLK_ENABLE |= CLK_ENABLE_IRQ;

  /* Register DMA RX done callback */
  HAL_DMA_RegDoneCallback(rxchannel, (void (*)(void *))HAL_Spi_Rx_Dma_Callback, hspi);

  HAL_UNLOCK(hspi);
  return hal_status_ok;
}

/*******************************************************************************
 **
 ** @brief  Transmits data in blocking mode
 **
 ** @param  hspi - pointer to SPI handle structure
 ** @param  txbuf - pointer to transmit buffer - byte array if wordlength 4-8 bits,
 **                 halfword array if wordlength is 9-16 bits
 ** @param  len - number of SPI words to transmit
 ** @param  timeout - timeout in ms. If zero, uses default timeout value
 ** @param  bmstop - if true, and in blockmode, send the blockmode stop cmd
 **
 ** @return hal_status_t - status of driver call
 **
 ******************************************************************************/
hal_status_t HAL_Spi_Transmit(hal_spi_handle_t* const hspi, void* txbuf, uint16_t len, uint32_t timeout, bool bmstop)
{
  return HAL_Spi_TransmitReceive(hspi, txbuf, NULL, len, len, 0, timeout, bmstop);
}

/*******************************************************************************
 **
 ** @brief  Receives data in blocking mode
 **
 ** @param  hspi - pointer to SPI handle structure
 ** @param  rxbuf - pointer to receive buffer - byte array if wordlength 4-8 bits,
 **                 halfword array if wordlength is 9-16 bits
 ** @param  len - number of SPI words to receive
 ** @param  timeout - timeout in ms. If zero, uses default timeout value
 **
 ** @return hal_status_t - status of driver call
 **
 ******************************************************************************/
hal_status_t HAL_Spi_Receive(hal_spi_handle_t* const hspi, void* rxbuf, uint16_t len, uint32_t timeout)
{
  return HAL_Spi_TransmitReceive(hspi, NULL, rxbuf, 0, 0, len, timeout, true);
}

/*******************************************************************************
 **
 ** @brief  Transmits and receives data in blocking mode
 **
 ** @param  hspi - pointer to SPI handle structure
 ** @param  txbuf - pointer to transmit buffer - byte array if wordlength 4-8 bits,
 **                 halfword array if wordlength is 9-16 bits
 ** @param  rxbuf - pointer to receive buffer - byte array if wordlength 4-8 bits,
 **                 halfword array if wordlength is 9-16 bits
 ** @param  rxstart how many received words to ignore before reading to buffer
 ** @param  txlen - number of SPI words to transmit from buffer. If (rxstart + rxlen) > txlen,
                    the SPI will transmit SPI_PUMP until (rxstart + rxlen) words are sent in total
 ** @param  rxlen - number of SPI words to receive
 ** @param  timeout - timeout in ms. If zero, uses default timeout value
 **
 ** @return hal_status_t - status of driver call
 **
 ******************************************************************************/
hal_status_t HAL_Spi_TransmitReceive(hal_spi_handle_t* const hspi, void* txbuf, void* rxbuf, uint16_t rxstart, uint16_t txlen, uint16_t rxlen, uint32_t timeout, bool bmstop)
{
  /* param checks */
  if(hspi == NULL)
  {
    return hal_status_badParam;
  }
  if((txbuf == NULL) && (txlen > 0))
  {
    return hal_status_badParam;
  }
  if((rxbuf == NULL) && (rxlen > 0))
  {
    return hal_status_badParam;
  }
  if(timeout == 0U)
  {
    timeout = SPI_TIMEOUT_MS; /* use default timeout value */
  }

  uint64_t timestart = HAL_GetTimeMs();
  uint16_t txcount = 0, rxcount = 0;
  uint32_t txtotal = (txlen > (rxlen + rxstart)) ? txlen : (rxlen + rxstart); // total number of words in total of the transfer
  uint32_t voidwrite = SPI_PUMP;
  volatile uint32_t voidread __attribute__((unused));
  switch(hspi->state)
  {
    case hal_spi_state_error:
    case hal_spi_state_reset:
      return hal_status_periphErr;
    case hal_spi_state_ready:
      HAL_LOCK(hspi);
      hspi->state = hal_spi_state_busy;
      Spi_ClearFifo(hspi);
      HAL_SPI_ENABLE(hspi);
      if(hspi->init.wordLen <= 8U)
      {
        // treat buffers 8 bits wide
        while((rxcount < rxlen) || (txcount < txlen))
        {
          if(hspi->spi->STATUS & SPI_STATUS_TNF_Msk)
          {
            if(txcount < txlen)
            {
              hspi->spi->DATA = *(uint8_t*)(txbuf);
              txbuf = (void*)((uint32_t)txbuf + sizeof(uint8_t));
              txcount++;
            }
            else if (txcount < txtotal)
            {
              hspi->spi->DATA = voidwrite;
              txcount++;
            }
            if((txcount == txtotal) && bmstop)
            {
              HAL_SPI_SEND_BMSTOP(hspi);
            }
          }
          if(hspi->spi->STATUS & SPI_STATUS_RNE_Msk)
          {
            if(rxstart)
            {
              voidread = hspi->spi->DATA;
              rxstart--;
            }
            else if(rxcount < rxlen)
            {
              *(uint8_t*)rxbuf = (uint8_t)(hspi->spi->DATA);
              rxbuf = (void*)((uint32_t)rxbuf + sizeof(uint8_t));
              rxcount++;
            }
            else
            {
              voidread = hspi->spi->DATA;
            }
          }
          if((HAL_GetTimeMs() - timestart) >= timeout)
          {
            goto timeout;
          }
        }
      }
      else
      {
        // treat buffers 16 bits wide
        while((rxcount < rxlen) || (txcount < txlen))
        {
          if(hspi->spi->STATUS & SPI_STATUS_TNF_Msk)
          {
            if(txcount < txlen)
            {
              hspi->spi->DATA = *(uint16_t*)(txbuf);
              txbuf = (void*)((uint32_t)txbuf + sizeof(uint16_t));
              txcount++;
            }
            else if (txcount < txtotal)
            {
              hspi->spi->DATA = voidwrite;
              txcount++;
            }
            if((txcount == txtotal) && bmstop)
            {
              HAL_SPI_SEND_BMSTOP(hspi);
            }
          }
          if(hspi->spi->STATUS & SPI_STATUS_RNE_Msk)
          {
            if(rxstart)
            {
              voidread = hspi->spi->DATA;
              rxstart--;
            }
            else if(rxcount < rxlen)
            {
              *(uint16_t*)rxbuf = (uint16_t)(hspi->spi->DATA);
              rxbuf = (void*)((uint32_t)rxbuf + sizeof(uint16_t));
              rxcount++;
            }
            else
            {
              voidread = hspi->spi->DATA;
            }
          }
          if((HAL_GetTimeMs() - timestart) >= timeout)
          {
            goto timeout;
          }
        }
      }
      hspi->state = hal_spi_state_ready;
      HAL_UNLOCK(hspi);
      break;
    case hal_spi_state_busy:
      return hal_status_busy;
      break;
  }
  return hal_status_ok;

timeout:
  HAL_SPI_DISABLE(hspi);
  Spi_ClearFifo(hspi);
  hspi->state = hal_spi_state_ready;
  HAL_SPI_ENABLE(hspi);
  HAL_UNLOCK(hspi);
  return hal_status_timeout;
}

/*******************************************************************************
 **
 ** @brief  SPI interrupt mode transmit - non blocking
 **
 ** @param  hspi - pointer to SPI handle
 **
 ** @return hal_status_t - status of driver call
 **
 ******************************************************************************/
hal_status_t HAL_Spi_TransmitInt(hal_spi_handle_t* const hspi, void* txbuf, uint16_t len)
{
  return HAL_Spi_TransmitReceiveInt(hspi, txbuf, NULL, len);
}

/*******************************************************************************
 **
 ** @brief  SPI interrupt mode receive - non blocking
 **
 ** @param  hspi - pointer to SPI handle
 **
 ** @return hal_status_t - status of driver call
 **
 ******************************************************************************/
hal_status_t HAL_Spi_ReceiveInt(hal_spi_handle_t* const hspi, void* rxbuf, uint16_t len)
{
  return HAL_Spi_TransmitReceiveInt(hspi, NULL, rxbuf, len);
}

/*******************************************************************************
 **
 ** @brief  SPI interrupt mode transmit/receive - non blocking
 **
 ** @param  hspi  - pointer to SPI handle
 ** @param  txbuf - pointer to transmit buffer - byte array if wordlength 4-8 bits,
 **                 halfword array if wordlength is 9-16 bits
 ** @param  rxbuf - pointer to receive buffer - byte array if wordlength 4-8 bits,
 **                 halfword array if wordlength is 9-16 bits
 ** @param  len   - number of SPI words in the transfer
 **
 ** @return hal_status_t - status of driver call
 **
 ******************************************************************************/
hal_status_t HAL_Spi_TransmitReceiveInt(hal_spi_handle_t* const hspi, void* txbuf, void* rxbuf, uint16_t len)
{
  // todo: fix spi receive overrun errors at high clock rates, if possible

  /* param checks */
  if(hspi == NULL)
    return hal_status_badParam;
  if((txbuf == NULL) && (rxbuf == NULL))
    return hal_status_badParam;
  if(len == 0)
    return hal_status_badParam;

  if(hspi->state == hal_spi_state_busy)
    return hal_status_busy;
  if((hspi->state == hal_spi_state_error) || (hspi->state == hal_spi_state_reset))
    return hal_status_periphErr;

  HAL_LOCK(hspi);
  spiHandles[hspi->bank] = hspi; // so that the SPI ISR can access the SPI handle via the bank number
  hspi->state = hal_spi_state_busy;

  /* setup transfer in handle struct */
  hspi->xfer.txbuf = txbuf;
  hspi->xfer.rxbuf = rxbuf;
  hspi->xfer.len = len;
  hspi->xfer.sent = 0;
  hspi->xfer.received = 0;
  if(hspi->init.blockmode)
    hspi->xfer.bmstop = 1;

  /* disable tx, clear fifos */
  HAL_SPI_DISABLE(hspi);
  Spi_ClearFifo(hspi);

  /* load tx fifo */
  Spi_LoadTxFifo(hspi);

  /* enable interrupts */
  hspi->spi->RXFIFOIRQTRG = 1;
  hspi->spi->IRQ_ENB |= SPI_IRQ_ENB_RXIM_Msk | SPI_IRQ_ENB_RORIM_Msk;
  Spi_EnableRxIrq(hspi);

  /* enable SPI */
  HAL_SPI_ENABLE(hspi);

  HAL_UNLOCK(hspi);
  return hal_status_ok;
}

/*******************************************************************************
 **
 ** @brief  SPI interrupt mode state machine - called by SPI ISRs
 **
 ** @param  hspi - pointer to SPI handle
 **
 ******************************************************************************/
__STATIC_INLINE void Spi_StateMachine(hal_spi_handle_t* const hspi)
{
  volatile uint32_t voidRead __attribute((unused));
  hspi->locked = 1;
  static uint16_t* rxptr16;
  static uint8_t*  rxptr8;
  if(hspi->xfer.received == 0)
  {
    rxptr16 = (uint16_t*)hspi->xfer.rxbuf;
    rxptr8 = (uint8_t*)rxptr16;
  }

  switch(hspi->state){
    case hal_spi_state_ready:
      // master: nothing to do

      // if slave, and received something when idle (no rxbuf), throw it away
      while(hspi->spi->STATUS & SPI_STATUS_RNE_Msk){
        voidRead = hspi->spi->DATA;
      }
      HAL_UNLOCK(hspi);
      return;

    case hal_spi_state_busy:

      // pull data off of the rx fifo
      if(hspi->xfer.rxbuf == NULL){
        while((hspi->spi->STATUS & SPI_STATUS_RNE_Msk) && \
          (hspi->xfer.received < hspi->xfer.len))
        {
          voidRead = hspi->spi->DATA;
          hspi->xfer.received++;
        }
      } else {
        if(hspi->init.wordLen <= 8)
        {
          // 8 bit buffer
          while((hspi->spi->STATUS & SPI_STATUS_RNE_Msk) && \
            (hspi->xfer.received < hspi->xfer.len))
          {
            *rxptr8++ = hspi->spi->DATA;
            hspi->xfer.received++;
          }
        } else {
          while((hspi->spi->STATUS & SPI_STATUS_RNE_Msk) &&
            (hspi->xfer.received < hspi->xfer.len))
          {
            *rxptr16++ = hspi->spi->DATA;
            hspi->xfer.received++;
          }
        }
      }

      // load tx fifo
      Spi_LoadTxFifo(hspi);

      // if not done
      if(hspi->xfer.received < hspi->xfer.len)
      {
        // check for receive FIFO overrun
        if(hspi->spi->IRQ_END & SPI_IRQ_END_RORIM_Msk)
        {
          // if overrun, abort the transfer
          hspi->state = hal_spi_state_error;
          break;
        }
        else
        {
          HAL_UNLOCK(hspi);
          return;
        }
      }

      // else, mark current xfer as done
      hspi->state = hal_spi_state_ready;
      break;

    default:
      // error - abort
      hspi->state = hal_spi_state_error;
      break;
  }

  Spi_ClearFifo(hspi);
  NVIC_DisableIRQ(hspi->rxirqn);
  hspi->spi->IRQ_ENB = 0;

  // execute callback
  Spi_Callback(hspi);
  HAL_UNLOCK(hspi);
}

#ifdef __HAL_SPI0_ISR_ENABLED
void SPI0_RX_IRQHandler(void)
{
  Spi_StateMachine(spiHandles[SPI0_BANK]);
}
#endif

#ifdef __HAL_SPI1_ISR_ENABLED
void SPI1_RX_IRQHandler(void)
{
  Spi_StateMachine(spiHandles[SPI1_BANK]);
}
#endif

#ifdef __HAL_SPI2_ISR_ENABLED
void SPI2_RX_IRQHandler(void)
{
  Spi_StateMachine(spiHandles[SPI2_BANK]);
}
#endif

#ifdef __HAL_SPI3_ISR_ENABLED
void SPI3_RX_IRQHandler(void)
{
  Spi_StateMachine(spiHandles[SPI3_BANK]);
}
#endif


/** DMA based transfers, non-blocking */

/*******************************************************************************
 **
 ** @brief  SPI DMA mode transmit - non blocking
 **
 ** @param  hspi  - pointer to SPI handle
 ** @param  txbuf - pointer to transmit buffer - 32-bit word array
 ** @param  len   - number of SPI words in the transfer
 **
 ** @return hal_status_t - status of driver call
 **
 ******************************************************************************/
hal_status_t HAL_Spi_TransmitDMA(hal_spi_handle_t* const hspi, uint32_t* txbuf, uint16_t len)
{
  return HAL_Spi_TransmitReceiveDMA(hspi, txbuf, NULL, len);
}

/*******************************************************************************
 **
 ** @brief  SPI DMA mode receive - non blocking
 **
 ** @param  hspi  - pointer to SPI handle
 ** @param  rxbuf - pointer to receive buffer - byte array if wordlength 4-8 bits,
 **                 halfword array if wordlength is 9-16 bits
 ** @param  len   - number of SPI words in the transfer
 **
 ** @return hal_status_t - status of driver call
 **
 ******************************************************************************/
hal_status_t HAL_Spi_ReceiveDMA(hal_spi_handle_t* const hspi, void* rxbuf, uint16_t len)
{
  return HAL_Spi_TransmitReceiveDMA(hspi, NULL, rxbuf, len);
}

/*******************************************************************************
 **
 ** @brief  SPI DMA mode transmit/receive - non blocking
 **
 ** @param  hspi  - pointer to SPI handle
 ** @param  txbuf - pointer to transmit buffer - 32-bit word array
 ** @param  rxbuf - pointer to receive buffer - byte array if wordlength 4-8 bits,
 **                 halfword array if wordlength is 9-16 bits
 ** @param  len   - number of SPI words in the transfer (1-1024)
 **
 ** @return hal_status_t - status of driver call
 **
 ******************************************************************************/
hal_status_t HAL_Spi_TransmitReceiveDMA(hal_spi_handle_t* const hspi, uint32_t* txbuf, void* rxbuf, uint16_t len)
{
  /* param checks */
  if(hspi == NULL)
    return hal_status_badParam; /* bad handle pointer */
  if((txbuf == NULL) && (rxbuf == NULL))
    return hal_status_badParam; /* at least one of these must not be NULL */
  if(len == 0)
    return hal_status_badParam; /* invalid length (does nothing) */
  if(len > 1024)
    return hal_status_badParam; /* maximum DMA transfer length is 1024 */

  if(hspi->state == hal_spi_state_busy)
    return hal_status_busy; /* must wait for previous txn to complete */
  if((hspi->state == hal_spi_state_error) || (hspi->state == hal_spi_state_reset))
    return hal_status_periphErr; /* SPI in error or reset state, re-run HAL_SPI_Init() */

  HAL_LOCK(hspi);
  hspi->state = hal_spi_state_busy;

  /* Setup SPI */
  hspi->spi->IRQ_ENB = 0;
  hspi->spi->TXFIFOIRQTRG = 8;
  hspi->spi->RXFIFOIRQTRG = 1;

  /* Setup IRQ router - SPI FIFO signals to DMA triggers */
  hal_status_t status;
  status = HAL_Irqrouter_SetDmaSel(hspi->txdma_ch, en_irqr_dmasel_spi0_tx+2*hspi->bank);
  if(status != hal_status_ok)
    return hal_status_notInitialized; // Need to call HAL_Spi_ConfigDMA()
  status = HAL_Irqrouter_SetDmaSel(hspi->rxdma_ch, en_irqr_dmasel_spi0_rx+2*hspi->bank);
  if(status != hal_status_ok)
    return hal_status_notInitialized; // Need to call HAL_Spi_ConfigDMA()

  /* set up TX DMA channel */
  stc_dma_control_blk_t* dma_blk = (stc_dma_control_blk_t*)VOR_DMA->CTRL_BASE_PTR;
  len--;
  uint32_t ch = hspi->txdma_ch;
  dma_blk->pri[ch].ctrl.n_minus_1 = len; // len already subtracted by one
  if(txbuf != NULL)
  {
    dma_blk->pri[ch].src = (uint32_t)(txbuf) + (4 * len);
    dma_blk->pri[ch].ctrl.src_inc = DMA_CHNL_CFG_INC_WORD; // source ptr increment
  }
  else
  {
    dummyWord = SPI_PUMP;
    dma_blk->pri[ch].src = (uint32_t)&dummyWord;
    dma_blk->pri[ch].ctrl.src_inc = DMA_CHNL_CFG_INC_NONE; // source ptr increment
  }

  /* set up RX DMA channel */
  ch = hspi->rxdma_ch;
  dma_blk->pri[ch].ctrl.n_minus_1 = len; // len already subtracted by one
  if(rxbuf != NULL)
  {
    if(hspi->init.wordLen > 8)
    {
      dma_blk->pri[ch].ctrl.src_size = DMA_CHNL_CFG_SIZE_HALFWORD;
      dma_blk->pri[ch].dst = (uint32_t)(rxbuf) + (2 * len);
      dma_blk->pri[ch].ctrl.dst_size = DMA_CHNL_CFG_SIZE_HALFWORD;
      dma_blk->pri[ch].ctrl.dst_inc = DMA_CHNL_CFG_INC_HALFWORD; // dest ptr increment
    }
    else
    {
      dma_blk->pri[ch].ctrl.src_size = DMA_CHNL_CFG_SIZE_BYTE;
      dma_blk->pri[ch].dst = (uint32_t)(rxbuf) + len;
      dma_blk->pri[ch].ctrl.dst_size = DMA_CHNL_CFG_SIZE_BYTE;
      dma_blk->pri[ch].ctrl.dst_inc = DMA_CHNL_CFG_INC_BYTE; // dest ptr increment
    }
  }
  else
  {
    dma_blk->pri[ch].ctrl.src_size = DMA_CHNL_CFG_SIZE_WORD;
    dma_blk->pri[ch].dst = (uint32_t)(&dummyWord);
    dma_blk->pri[ch].ctrl.dst_size = DMA_CHNL_CFG_SIZE_WORD;
    dma_blk->pri[ch].ctrl.dst_inc = DMA_CHNL_CFG_INC_NONE; // dest ptr increment
  }

  /* Enable, kick off transfer */
  VOR_DMA->CHNL_ENABLE_SET = (1UL << hspi->txdma_ch) | (1UL << hspi->rxdma_ch);
  hspi->spi->IRQ_ENB = (SPI_IRQ_ENB_TXIM_Msk | SPI_IRQ_ENB_RXIM_Msk);
  NVIC_EnableIRQ((IRQn_Type)(DMA_DONE0_IRQn + hspi->rxdma_ch));

  HAL_UNLOCK(hspi);
  return hal_status_ok;
}

#endif /* __HAL_SPI_MODULE_ENABLED */

/*****************************************************************************/
/* End of file                                                               */
/*****************************************************************************/
