/***************************************************************************************
 * @file     va416xx_hal_uart.c
 * @version  V2.05
 * @date     21 December 2023
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

#include "va416xx_hal_uart.h"
#include "va416xx_debug.h"
#include "hal_config.h"

/*****************************************************************************/ 
/* Local pre-processor symbols/macros ('#define')                            */ 
/*****************************************************************************/

// Fixed, do not change
#define UART_PERID   (0x021207E9)
#define UART_MAX_BANKNUM (HAL_NUM_UARTS-1) // UART0-2
#define UART_FIFO_SZ (16)
#define UART_INVALID_BANKNUM (0xff) // bad index

// These can be overridden in HAL_config.h, default settings here:
#ifndef UART_RXBUF_SZ
#define UART_RXBUF_SZ     (32)
#endif 

#ifndef UART_TXTIMEOUT
#define UART_TXTIMEOUT    (100000)
#endif 

#ifndef UART_TX_IRQ_PRI
#define UART_TX_IRQ_PRI   (1)
#endif 

#ifndef UART_RX_IRQ_PRI
#define UART_RX_IRQ_PRI   (1)
#endif 

/*****************************************************************************/ 
/* Global variable definitions (declared in header file with 'extern')       */ 
/*****************************************************************************/

/*****************************************************************************/ 
/* Local type definitions ('typedef')                                        */ 
/*****************************************************************************/

/*****************************************************************************/ 
/* Local variable definitions ('static')                                     */ 
/*****************************************************************************/

static bool aIsUartInitialized[HAL_NUM_UARTS] = {false, false, false};
static uint8_t aRxBuffer[HAL_NUM_UARTS][UART_RXBUF_SZ];
static uint32_t aRxBufLen[HAL_NUM_UARTS];
static uint8_t* apRxHead[HAL_NUM_UARTS];
static uint8_t* apRxTail[HAL_NUM_UARTS];
static uint8_t* apRxEnd[HAL_NUM_UARTS];

/*****************************************************************************/ 
/* Local function prototypes ('static')                                      */ 
/*****************************************************************************/

__STATIC_INLINE uint32_t GetBankNum(VOR_UART_Type* const uart);
__STATIC_INLINE void EnableRxIrq(uint32_t bank);
__STATIC_INLINE bool DisableRxIrq(uint32_t bank);

/*****************************************************************************/ 
/* Function implementation - global ('extern') and local ('static')          */ 
/*****************************************************************************/

/*****************************************************************************/
/*                                                                           */
/*****************************************************************************/ 
void __attribute__((weak)) uart0TxCallback(void)
{

}

/*****************************************************************************/
/*                                                                           */
/*****************************************************************************/ 
void __attribute__((weak)) uart0RxCallback(void)
{

}

/*****************************************************************************/
/*                                                                           */
/*****************************************************************************/ 
void __attribute__((weak)) uart1TxCallback(void)
{

}

/*****************************************************************************/
/*                                                                           */
/*****************************************************************************/ 
void __attribute__((weak)) uart1RxCallback(void)
{

}

/*****************************************************************************/
/*                                                                           */
/*****************************************************************************/ 
void __attribute__((weak)) uart2TxCallback(void)
{

}

/*****************************************************************************/
/*                                                                           */
/*****************************************************************************/ 
void __attribute__((weak)) uart2RxCallback(void)
{

}


/*******************************************************************************
 **
 ** @brief  Takes a pointer to a UART peripheral and returns its bank number
 **
 ** @param  uart - pointer to UART peripheral
 **
 ** @return uint32_t the bank number for this UART
 **
 ******************************************************************************/
__STATIC_INLINE uint32_t GetBankNum(VOR_UART_Type* const uart)
{
  if(uart == VOR_UART0)
  { return UART0_BANK; }
  else if(uart == VOR_UART1)
  { return UART1_BANK; }
  else if(uart == VOR_UART2)
  { return UART2_BANK; }
  else
  {
    // invalid uart bank
    return UART_INVALID_BANKNUM;
  }
}

/*******************************************************************************
 **
 ** @brief  Disables a UART's RX IRQ in the NVIC
 **
 ** @param  bank - the UART bank number (0-2)
 **
 ** @return bool - true if the RX IRQ for that bank was previously enabled, else false
 **
 ******************************************************************************/
__STATIC_INLINE bool DisableRxIrq(uint32_t bank)
{
  switch(bank){
    case UART0_BANK:
      if(NVIC_GetEnableIRQ(UART0_RX_IRQn) == true){
        NVIC_DisableIRQ(UART0_RX_IRQn);
        return true;
      }
      break;
    case UART1_BANK:
      if(NVIC_GetEnableIRQ(UART1_RX_IRQn) == true){
        NVIC_DisableIRQ(UART1_RX_IRQn);
        return true;
      }
      break;
    case UART2_BANK:
      if(NVIC_GetEnableIRQ(UART2_RX_IRQn) == true){
        NVIC_DisableIRQ(UART2_RX_IRQn);
        return true;
      }
      break;
  }
  return false;
}

/*******************************************************************************
 **
 ** @brief  Enables a UART's RX IRQ in the NVIC
 **
 ** @param  bank - the UART bank number (0-2)
 **
 ** @return void
 **
 ******************************************************************************/
__STATIC_INLINE void EnableRxIrq(uint32_t bank)
{
  switch(bank){
    case UART0_BANK:
      NVIC_EnableIRQ(UART0_RX_IRQn);
      break;
    case UART1_BANK:
      NVIC_EnableIRQ(UART1_RX_IRQn);
      break;
    case UART2_BANK:
      NVIC_EnableIRQ(UART2_RX_IRQn);
      break;
  }
}

/*******************************************************************************
 **
 ** @brief  Initialize a UART
 **
 ** @param  uart - The UART to init
 **
 ** @param  initCfg - configuration struct for how to init the UART (baud, etc)
 **
 ** @return hal_status_t Status of driver call
 **
 ******************************************************************************/
hal_status_t HAL_Uart_Init(VOR_UART_Type* const uart, const stc_uart_cfg_t initCfg)
{
  uint32_t bank = GetBankNum(uart);

  // Enable clock and reset UART
  switch(bank){
    case UART0_BANK:
      VOR_SYSCONFIG->PERIPHERAL_CLK_ENABLE |= CLK_ENABLE_UART0;
      VOR_SYSCONFIG->PERIPHERAL_RESET &= ~SYSCONFIG_PERIPHERAL_RESET_UART0_Msk;
      __NOP();
      __NOP();
      VOR_SYSCONFIG->PERIPHERAL_RESET |= SYSCONFIG_PERIPHERAL_RESET_UART0_Msk;
      break;
    
    case UART1_BANK:
      VOR_SYSCONFIG->PERIPHERAL_CLK_ENABLE |= CLK_ENABLE_UART1;
      VOR_SYSCONFIG->PERIPHERAL_RESET &= ~SYSCONFIG_PERIPHERAL_RESET_UART1_Msk;
      __NOP();
      __NOP();
      VOR_SYSCONFIG->PERIPHERAL_RESET |= SYSCONFIG_PERIPHERAL_RESET_UART1_Msk;
      break;
    
    case UART2_BANK:
      VOR_SYSCONFIG->PERIPHERAL_CLK_ENABLE |= CLK_ENABLE_UART2; 
      VOR_SYSCONFIG->PERIPHERAL_RESET &= ~SYSCONFIG_PERIPHERAL_RESET_UART2_Msk;
      __NOP();
      __NOP();
      VOR_SYSCONFIG->PERIPHERAL_RESET |= SYSCONFIG_PERIPHERAL_RESET_UART2_Msk;
      break;
    
    default:
      // invalid uart bank
      return hal_status_badParam;
  }
  aIsUartInitialized[bank] = false;

  // check peripheral ID
  if(uart->PERID != UART_PERID){ 
    return hal_status_badPeriphID; // not running on correct hw, or a register issue
  }

  // Initialize circular buffer variables
  apRxHead[bank] = &aRxBuffer[bank][0];
  apRxTail[bank] = &aRxBuffer[bank][0];
  apRxEnd[bank] = &aRxBuffer[bank][0] + UART_RXBUF_SZ;
  aRxBufLen[bank] = 0;

  // Baud rate
  // TODO: bounds check on min/max allowable baud rate
  if(initCfg.baudRate == 0){ 
    return hal_status_badParam;  // avoid divide by zero
  }
  if(uart == VOR_UART2){
    uart->CLKSCALE = UART_CALC_CLOCKSCALE(SystemCoreClock/2, initCfg.baudRate); // APB1 divide by 2
  } else {
    uart->CLKSCALE = UART_CALC_CLOCKSCALE(SystemCoreClock/4, initCfg.baudRate); // APB2 divide by 4
  }

  // Parity
  switch(initCfg.parity){
    case en_uart_parity_none:
      uart->CTRL &= ~UART_CTRL_PAREN_Msk;
      break;
    
    case en_uart_parity_even:
      uart->CTRL |= UART_CTRL_PAREN_Msk;
      uart->CTRL |= UART_CTRL_PAREVEN_Msk;
      break;
    
    case en_uart_parity_odd:
      uart->CTRL |= UART_CTRL_PAREN_Msk;
      uart->CTRL &= ~UART_CTRL_PAREVEN_Msk;
      break;
    
    default:
      return hal_status_badParam;
  }

  // Flow control
  switch(initCfg.flowctrl){
    case en_uart_flowctrl_none:
      uart->CTRL &= ~(UART_CTRL_AUTOCTS_Msk | 
                      UART_CTRL_AUTORTS_Msk);
      break;
    
    case en_uart_flowctrl_autoCtsRts:
      uart->CTRL |= (UART_CTRL_AUTOCTS_Msk | 
                     UART_CTRL_AUTORTS_Msk);
      break;
    
    default:
      return hal_status_badParam;
  }

  // Word size
  if((initCfg.wordSize < 5) || (initCfg.wordSize > 8)){ 
    return hal_status_badParam;
  }
  uart->CTRL &= ~UART_CTRL_WORDSIZE_Msk;
  uart->CTRL |= ((initCfg.wordSize - 5) << UART_CTRL_WORDSIZE_Pos);

  // Stop bits
  if((initCfg.stopBits < 1) || (initCfg.stopBits > 2)){ 
    return hal_status_badParam; 
  }
  uart->CTRL &= ~UART_CTRL_STOPBITS_Msk;
  uart->CTRL |= ((initCfg.stopBits - 1) << UART_CTRL_STOPBITS_Pos);

  // Loopback
  if(initCfg.loopback == true){ 
    uart->CTRL |= (UART_CTRL_LOOPBACK_Msk | UART_CTRL_LOOPBACKBLK_Msk); 
  } else { 
    uart->CTRL &= ~(UART_CTRL_LOOPBACK_Msk | UART_CTRL_LOOPBACKBLK_Msk); 
  }

  // Set up interrupts
  uart->IRQ_ENB = (uint32_t)initCfg.irq;
  uart->RXFIFOIRQTRG = 0x1;
  uart->TXFIFOIRQTRG = 0xF;
  if(((uint32_t)initCfg.irq & 0x0F) > 0){
    switch(bank){
      case UART0_BANK:
        NVIC_EnableIRQ(UART0_RX_IRQn);
        NVIC_SetPriority(UART0_RX_IRQn, UART_RX_IRQ_PRI);
        break;
      case UART1_BANK:
        NVIC_EnableIRQ(UART1_RX_IRQn);
        NVIC_SetPriority(UART1_RX_IRQn, UART_RX_IRQ_PRI);
        break;
      case UART2_BANK:
        NVIC_EnableIRQ(UART2_RX_IRQn);
        NVIC_SetPriority(UART2_RX_IRQn, UART_RX_IRQ_PRI);
        break;
    }
  }
  if(((uint32_t)initCfg.irq & 0xF0) > 0){
    switch(bank){
      case UART0_BANK:
        NVIC_EnableIRQ(UART0_TX_IRQn);
        NVIC_SetPriority(UART0_TX_IRQn, UART_TX_IRQ_PRI);
        break;
      case UART1_BANK:
        NVIC_EnableIRQ(UART1_TX_IRQn);
        NVIC_SetPriority(UART1_TX_IRQn, UART_TX_IRQ_PRI);
        break;
      case UART2_BANK:
        NVIC_EnableIRQ(UART2_TX_IRQn);
        NVIC_SetPriority(UART2_TX_IRQn, UART_TX_IRQ_PRI);
        break;
    }
  }

  // Enable
  uart->ENABLE = (UART_ENABLE_RXENABLE_Msk | 
                  UART_ENABLE_TXENABLE_Msk);

  aIsUartInitialized[bank] = true;
  return hal_status_ok;
}

/*******************************************************************************
 **
 ** @brief  De-initialize and reset a UART
 **
 ** @param  uart - The UART to de-init
 **
 ** @return hal_status_t Status of driver call
 **
 ******************************************************************************/
hal_status_t HAL_Uart_DeInit(VOR_UART_Type* const uart)
{
  uint32_t bank = GetBankNum(uart);
  
  switch(bank){
    case UART0_BANK:
      NVIC_DisableIRQ(UART0_TX_IRQn);
      NVIC_DisableIRQ(UART0_RX_IRQn);
			VOR_SYSCONFIG->PERIPHERAL_RESET &= ~SYSCONFIG_PERIPHERAL_RESET_UART0_Msk;
      __NOP();
      __NOP();
			VOR_SYSCONFIG->PERIPHERAL_CLK_ENABLE &= ~(CLK_ENABLE_UART0);
      break;   
    case UART1_BANK:
			NVIC_DisableIRQ(UART1_TX_IRQn);
      NVIC_DisableIRQ(UART1_RX_IRQn);
			VOR_SYSCONFIG->PERIPHERAL_RESET &= ~SYSCONFIG_PERIPHERAL_RESET_UART1_Msk;
      __NOP();
      __NOP();
			VOR_SYSCONFIG->PERIPHERAL_CLK_ENABLE &= ~(CLK_ENABLE_UART1);
      break;
    case UART2_BANK:
			NVIC_DisableIRQ(UART2_TX_IRQn);
      NVIC_DisableIRQ(UART2_RX_IRQn);
			VOR_SYSCONFIG->PERIPHERAL_RESET &= ~SYSCONFIG_PERIPHERAL_RESET_UART2_Msk;
      __NOP();
      __NOP();
			VOR_SYSCONFIG->PERIPHERAL_CLK_ENABLE &= ~(CLK_ENABLE_UART2);
      break;
    default:
      return hal_status_badParam; // invalid uart bank
  }
  aIsUartInitialized[bank] = false;
  return hal_status_ok;
}

/*******************************************************************************
 **
 ** @brief  Transmit a byte
 **
 ** @param  uart - The UART to operate on
 **
 ** @param  txByte - the byte to send
 **
 ** @return hal_status_t Status of driver call
 **
 ******************************************************************************/
hal_status_t HAL_Uart_TxByte(VOR_UART_Type* const uart, const uint8_t txByte)
{  
  uint32_t bank = GetBankNum(uart);
	uint32_t timeout = UART_TXTIMEOUT;
  
  if(bank > UART_MAX_BANKNUM){
    return hal_status_badParam; // invalid uart bank
  }
  if(aIsUartInitialized[bank] == false){
    return hal_status_notInitialized;
  }
	
	// Block until there is room on the FIFO to transmit a byte
	while((uart->TXSTATUS & UART_TXSTATUS_WRRDY_Msk) == 0){
		timeout--;
		if(timeout == 0){
      return hal_status_timeout;
    }
	}

	// transmit a character
	uart->DATA = txByte;
  
  return hal_status_ok;
}

/*******************************************************************************
 **
 ** @brief  Transmit a string, blocks until entire string has been written to 
 **         the TX FIFO
 **
 ** @param  uart - The UART to operate on
 **
 ** @param  pStr - constant pointer to the string to send
 **
 ** @return hal_status_t Status of driver call
 **
 ******************************************************************************/
hal_status_t HAL_Uart_TxStr(VOR_UART_Type* const uart, const char* pStr)
{
  uint32_t bank = GetBankNum(uart);
  uint32_t timeout = UART_TXTIMEOUT;

  if(bank > UART_MAX_BANKNUM){
    return hal_status_badParam; // invalid uart bank
  }
  if(aIsUartInitialized[bank] == false){
    return hal_status_notInitialized;
  }

  while(*pStr){
    // Block until there is room on the FIFO to transmit a byte
    timeout = UART_TXTIMEOUT;
    while((uart->TXSTATUS & UART_TXSTATUS_WRRDY_Msk) == 0){
      timeout--;
      if(timeout == 0){ 
        return hal_status_timeout;
      }
    }

    // transmit another character
    uart->DATA = *pStr++;
  }

  return hal_status_ok;
}

/*******************************************************************************
 **
 ** @brief  Transmits a serial break
 ** 
 ** @param  uart - The UART to operate on
 ** 
 ** @param  breakCount - number of character periods, >= 0x7F is a continuous break.
 **                    send another non-continuous break to exit continuous break.
 **
 ** @return hal_status_t Status of driver call
 **
 ******************************************************************************/
hal_status_t HAL_Uart_TxBreak(VOR_UART_Type* const uart, const uint32_t breakCount)
{
  uint32_t bank = GetBankNum(uart);
  
  if(bank > UART_MAX_BANKNUM){
    return hal_status_badParam; // invalid uart bank
  }
  if(aIsUartInitialized[bank] == false){
    return hal_status_notInitialized;
  }
  
  // Send break
  if (breakCount > 0x7F){ 
    uart->TXBREAK = 0x7F; 
  } else { 
    uart->TXBREAK = breakCount; 
  }
  
  return hal_status_ok;
}

/*******************************************************************************
 **
 ** @brief  Read a byte off of the specified UART's receive buffer. If the receive 
 **         buffer is empty, this will return 'hal_status_bufEmpty'.
 ** 
 ** @param  uart - The UART to operate on
 ** 
 ** @param  pRxByte - pointer to the byte to receive
 **
 ** @return hal_status_t Status of driver call
 **
 ******************************************************************************/
hal_status_t HAL_Uart_RxByte(VOR_UART_Type* const uart, uint8_t* const pRxByte)
{
  bool rxIrqEnabled;
  uint32_t bank = GetBankNum(uart);
  
  if(bank > UART_MAX_BANKNUM){
    return hal_status_badParam; // invalid uart bank
  }
  if(aIsUartInitialized[bank] == false){
    return hal_status_notInitialized;
  }
  
  rxIrqEnabled = DisableRxIrq(bank); // disable IRQ and save previous enable/disable status
  
  if(aRxBufLen[bank] == 0){
    // nothing to read
		*pRxByte = 0;
    
    // Re-enable UART RX interrupt (if it was previously enabled)
    if(rxIrqEnabled == true){
      EnableRxIrq(bank);
    }
    
		return hal_status_bufEmpty;
	}
  
  // pull byte from buffer
  *pRxByte = *apRxTail[bank];
  apRxTail[bank]++;
  aRxBufLen[bank]--;
  
  // loop tail pointer if needed
  if(apRxTail[bank] == apRxEnd[bank]){
    apRxTail[bank] = &aRxBuffer[bank][0];
  }
  
  // Re-enable UART RX interrupt (if it was previously enabled)
  if(rxIrqEnabled == true){
    EnableRxIrq(bank);
  }
  
  return hal_status_ok;
}

/*******************************************************************************
 **
 ** @brief  UART0-2 Get receive buffer length
 ** 
 ** @param  uart - The UART to operate on
 ** 
 ** @param  pRxLen - pointer to the desired receive buffer length
 **
 ** @return hal_status_t Status of driver call
 **
 ******************************************************************************/
hal_status_t HAL_Uart_GetRxLen(VOR_UART_Type* const uart, uint32_t* const pRxLen)
{
  uint32_t bank = GetBankNum(uart);

  if(bank > UART_MAX_BANKNUM){
    return hal_status_badParam; // invalid uart bank
  }
  if(aIsUartInitialized[bank] == false){
    return hal_status_notInitialized;
  }
  *pRxLen = aRxBufLen[bank];
  return hal_status_ok;
}

/*******************************************************************************
 **
 ** @brief  Waits for UART TX FIFO to empty, clears RX FIFO
 **
 ** @return hal_status_t Status of driver call
 **
 ******************************************************************************/
hal_status_t HAL_Uart_Flush(VOR_UART_Type* const uart)
{
  uint32_t bank = GetBankNum(uart);

  if(bank > UART_MAX_BANKNUM){
    return hal_status_badParam; // invalid uart bank
  }
  if(aIsUartInitialized[bank] == false){
    return hal_status_notInitialized;
  }

  while(uart->TXSTATUS & UART_TXSTATUS_TXBUSY_Msk); 
  uart->FIFO_CLR = UART_FIFO_CLR_RXFIFO_Msk;

  return hal_status_ok;
}

/*******************************************************************************
 **
 ** @brief  UART0-2 RX handler, polling mode
 **
 ** Reads characters received into RX buffer.  If the RX buffer is full, the
 ** contents of the RX buffer will be overwritten.
 **
 ** @return hal_status_t Status of driver call
 **
 ******************************************************************************/
hal_status_t HAL_Uart_RxHndlr(void)
{
  VOR_UART_Type* uart;
  uint32_t bank;
  uint32_t loopTerm = UART_FIFO_SZ;

  for(bank = UART0_BANK; bank < HAL_NUM_UARTS; bank++){
    switch(bank){
      case UART0_BANK:
        uart = VOR_UART0;
        if((VOR_UART0->IRQ_ENB & UART_IRQ_ENB_IRQ_RX_Msk) > 0){
          if(NVIC_GetEnableIRQ(UART0_RX_IRQn) == true){
            continue; // skip this bank if the RX IRQ is enabled
          }
        }
        break;
      case UART1_BANK: 
        uart = VOR_UART1;
        if((VOR_UART1->IRQ_ENB & UART_IRQ_ENB_IRQ_RX_Msk) > 0){
          if(NVIC_GetEnableIRQ(UART1_RX_IRQn) == true){
            continue; // skip this bank if the RX IRQ is enabled
          }
        }
        break;
      case UART2_BANK:
        uart = VOR_UART2;
        if((VOR_UART2->IRQ_ENB & UART_IRQ_ENB_IRQ_RX_Msk) > 0){
          if(NVIC_GetEnableIRQ(UART2_RX_IRQn) == true){
            continue; // skip this bank if the RX IRQ is enabled
          }
        }
        break;
      default:
        return hal_status_badParam; // invalid uart bank
    }
    if(aIsUartInitialized[bank] == true){
      // handle RX for this bank
      // put any received characters in the FIFO into the RXBUF
      // TODO: check for break condition
      while(((uart->RXSTATUS & UART_RXSTATUS_RDAVL_Msk) > 0) && (loopTerm > 0)){
        *apRxHead[bank] = uart->DATA;
        apRxHead[bank]++;
        aRxBufLen[bank]++;
        
        // Loop head pointer if needed
        if(apRxHead[bank] == apRxEnd[bank]){
          apRxHead[bank] = &aRxBuffer[bank][0];
        }
        loopTerm--;
      }
    }
  }
  return hal_status_ok;
}

// UART0-2 ISR(s)
#ifndef __HAL_DISABLE_UART0
void UART0_TX_IRQHandler(void)
{
    uart0TxCallback();
}

void UART0_RX_IRQHandler(void)
{
  uint32_t loopTerm = UART_FIFO_SZ;
  if((VOR_UART0->IRQ_END & UART_IRQ_END_IRQ_RX_Msk) > 0){
    // Read bytes into buffer as long as bytes are available
    while(((VOR_UART0->RXSTATUS & UART_RXSTATUS_RDAVL_Msk) > 0) && (loopTerm > 0)){
      *apRxHead[UART0_BANK] = VOR_UART0->DATA;
      apRxHead[UART0_BANK]++;
      aRxBufLen[UART0_BANK]++;
      
      // Loop head pointer if needed
      if(apRxHead[UART0_BANK] == apRxEnd[UART0_BANK]){
        apRxHead[UART0_BANK] = &aRxBuffer[UART0_BANK][0];
      }
      loopTerm--; // ensure loop terminates
    }
  }
  
  uart0RxCallback();
}
#endif

#ifndef __HAL_DISABLE_UART1
void UART1_TX_IRQHandler(void)
{
  uart1TxCallback();
}

void UART1_RX_IRQHandler(void)
{
  uint32_t loopTerm = UART_FIFO_SZ;
  if((VOR_UART1->IRQ_END & UART_IRQ_END_IRQ_RX_Msk) > 0){
    // Read bytes into buffer as long as bytes are available
    while(((VOR_UART1->RXSTATUS & UART_RXSTATUS_RDAVL_Msk) > 0) && (loopTerm > 0)){
      *apRxHead[UART1_BANK] = VOR_UART1->DATA;
      apRxHead[UART1_BANK]++;
      aRxBufLen[UART1_BANK]++;
      
      // Loop head pointer if needed
      if(apRxHead[UART1_BANK] == apRxEnd[UART1_BANK]){
        apRxHead[UART1_BANK] = &aRxBuffer[UART1_BANK][0];
      }
      loopTerm--; // ensure loop terminates
    }
  }
  
  uart1RxCallback();
}
#endif

#ifndef __HAL_DISABLE_UART2
void UART2_TX_IRQHandler(void)
{
  uart2TxCallback();
}

void UART2_RX_IRQHandler(void)
{
  uint32_t loopTerm = UART_FIFO_SZ;
  if((VOR_UART2->IRQ_END & UART_IRQ_END_IRQ_RX_Msk) > 0){
    // Read bytes into buffer as long as bytes are available
    while(((VOR_UART2->RXSTATUS & UART_RXSTATUS_RDAVL_Msk) > 0) && (loopTerm > 0)){
      *apRxHead[UART2_BANK] = VOR_UART2->DATA;
      apRxHead[UART2_BANK]++;
      aRxBufLen[UART2_BANK]++;
      
      // Loop head pointer if needed
      if(apRxHead[UART2_BANK] == apRxEnd[UART2_BANK]){
        apRxHead[UART2_BANK] = &aRxBuffer[UART2_BANK][0];
      }
      loopTerm--; // ensure loop terminates
    }
  }
  
  uart2RxCallback();
}
#endif

/*****************************************************************************/ 
/* End of file                                                               */ 
/*****************************************************************************/
