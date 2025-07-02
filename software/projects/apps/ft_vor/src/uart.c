/***************************************************************************************
 * @file     uart.c
 * @version  V2.05
 * @date     31 January 2024
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
/** @file */ 
/** \addtogroup UART
 *  @{
 */

#include "uart.h"

/* Variables for storing characters received from UART0. */
static uint8_t uart0RxBuffer[UART_RX_BUFFER_SIZE];
static uint8_t *uart0RxBufferHead;
static uint8_t *uart0RxBufferTail;
static uint8_t *uart0RxBufferEnd;

/* Variables for storing characters received from UART1. */
static uint8_t uart1RxBuffer[UART_RX_BUFFER_SIZE];
static uint8_t *uart1RxBufferHead;
static uint8_t *uart1RxBufferTail;
static uint8_t *uart1RxBufferEnd;

/* Variables for storing characters received from UART2. */
static uint8_t uart2RxBuffer[UART_RX_BUFFER_SIZE];
static uint8_t *uart2RxBufferHead;
static uint8_t *uart2RxBufferTail;
static uint8_t *uart2RxBufferEnd;

/*******************************************************************************
 **
 ** @brief Configures UART peripheral. Simple setup, 8N1 settings.
 **
 **  Initializes buffers and configures UART  
 **
 ** @param  uart - pointer to the UART peripheral to set up (VOR_UART0, etc)
 **
 ** @param  baudrate - the uart baud rate to set (ex. 115200)
 **
 ******************************************************************************/
void UartInit(VOR_UART_Type* uart, uint32_t baudrate)
{
  if(VOR_UART0 == uart){
    VOR_SYSCONFIG->PERIPHERAL_CLK_ENABLE |= CLK_ENABLE_UART0;
    /* Initialize variables. */
    uart0RxBufferHead = uart0RxBuffer;
    uart0RxBufferTail = uart0RxBuffer;
    uart0RxBufferEnd = uart0RxBuffer + UART_RX_BUFFER_SIZE;
    uart->CLKSCALE = UART_CALC_CLOCKSCALE(UART_CLK, baudrate);
  } else if(VOR_UART1 == uart){
    VOR_SYSCONFIG->PERIPHERAL_CLK_ENABLE |= CLK_ENABLE_UART1;
    /* Initialize variables. */
    uart1RxBufferHead = uart1RxBuffer;
    uart1RxBufferTail = uart1RxBuffer;
    uart1RxBufferEnd = uart1RxBuffer + UART_RX_BUFFER_SIZE;
    uart->CLKSCALE = UART_CALC_CLOCKSCALE(UART_CLK, baudrate);
  } else if(VOR_UART2 == uart){
    VOR_SYSCONFIG->PERIPHERAL_CLK_ENABLE |= CLK_ENABLE_UART2;
    /* Initialize variables. */
    uart2RxBufferHead = uart2RxBuffer;
    uart2RxBufferTail = uart2RxBuffer;
    uart2RxBufferEnd = uart2RxBuffer + UART_RX_BUFFER_SIZE;
    uart->CLKSCALE = UART_CALC_CLOCKSCALE(UART2_CLK, baudrate);
  } else {
    return;
  }
                     
  /* Configure word size and RTS behavior. */
  uart->CTRL = (3 << UART_CTRL_WORDSIZE_Pos) |
                      (UART_CTRL_DEFRTS_Msk);

  /* Enable CTS flow control IO, if needed. */
#ifdef configUART_CTS_FLOW_CONTROL
  uart->CTRL |= UART_CTRL_AUTOCTS_Msk;
#endif

  /* Enable RTS flow control IO, if needed. */
#ifdef configUART_RTS_FLOW_CONTROL
  uart->CTRL |= UART_CTRL_AUTORTS_Msk;
#endif

	/* Enable RX interupts as soon as a character is received. */
	uart->IRQ_ENB = UART_IRQ_ENB_IRQ_RX_Msk;
	uart->RXFIFOIRQTRG = 1;
  uart->TXFIFOIRQTRG = 8;
  
  if(VOR_UART0 == uart){  
	  NVIC_SetPriority(UART0_RX_IRQn, 1);
	  NVIC_EnableIRQ(UART0_RX_IRQn);
  } else if(VOR_UART1 == uart){  
    NVIC_SetPriority(UART1_RX_IRQn, 1);
	  NVIC_EnableIRQ(UART1_RX_IRQn);
  } else {
    NVIC_SetPriority(UART2_RX_IRQn, 1);
	  NVIC_EnableIRQ(UART2_RX_IRQn);
  }

  /* Enable UART. */
  uart->ENABLE = (UART_ENABLE_RXENABLE_Msk | 
                      UART_ENABLE_TXENABLE_Msk);
                      
	/* send a break to let rx state machine reset */
  uart->TXBREAK = 32;
}

/*******************************************************************************
 **
 ** @brief  Transmit a string across UART0.
 **
 ** Transmits a string across UART0, blocking until all characters are sent.
 **
 ** @param  pcStr is the pointer to the string that is transmitted across UART0.
 **
 ******************************************************************************/
void UART0TxStr(char *pcStr)
{
  if(pcStr == NULL) return;

	while(*pcStr){
    /* Block until UART is done transmitting data. */
    while((VOR_UART0->TXSTATUS & UART_TXSTATUS_WRRDY_Msk) == 0){}
        
    /* Transmit another character. */
    VOR_UART0->DATA = *pcStr++;
	}
}

/*******************************************************************************
 **
 ** @brief  TX one byte over UART0
 **
 ** @param  uint8_t b is the byte to send
 **
 ******************************************************************************/
void UART0TxByte(uint8_t b)
{
	/* Block until UART is done transmitting data. */
  while((VOR_UART0->TXSTATUS & UART_TXSTATUS_WRRDY_Msk) == 0){}
	VOR_UART0->DATA = b;
}

/*******************************************************************************
 **
 ** @brief  RX one byte over UART0, wait for a byte but timeout if no byte is received
 **
 ******************************************************************************/
uint8_t UART0RxByte(void)
{
	uint32_t timer = 1000000;
	uint8_t b =0;
	while(--timer){
		if(UART0Read(&b)){
#if UART_ECHO > 0
      UART0TxByte(b);
#endif
			break;
		}
	}
	return b;
}

/*******************************************************************************
 **
 ** @brief  Returns true if a byte is available on the RX buffer, else false
 **
 ******************************************************************************/
bool UART0IsRxByteAvailable(void)
{
	/* Disable UART RX interrupt. */
  NVIC_DisableIRQ(UART0_RX_IRQn);
    
  /* Verify that data is available. */
  if(uart0RxBufferTail == uart0RxBufferHead){
		/* Re-enable UART RX interrupt. */
    NVIC_EnableIRQ(UART0_RX_IRQn);
		return false;
	}
	/* Re-enable UART RX interrupt. */
  NVIC_EnableIRQ(UART0_RX_IRQn);
	return true;
}

/*******************************************************************************
 **
 ** @brief  Transmit a string across UART1.
 **
 ** Transmits a string across UART1, blocking until all characters are sent.
 **
 ** @param  pcStr is the pointer to the string that is transmitted across UART1.
 **
 ******************************************************************************/
void UART1TxStr(char *pcStr)
{
  if(pcStr == NULL) return;
	while(*pcStr){
    /* Block until UART is done transmitting data. */
    while((VOR_UART1->TXSTATUS & UART_TXSTATUS_WRRDY_Msk) == 0){}
        
    /* Transmit another character. */
    VOR_UART1->DATA = *pcStr++;
	}
}

/*******************************************************************************
 **
 ** @brief  TX one byte over UART1
 **
 ** @param  uint8_t b is the byte to send
 **
 ******************************************************************************/
void UART1TxByte(uint8_t b)
{
	/* Block until UART is done transmitting data. */
  while((VOR_UART1->TXSTATUS & UART_TXSTATUS_WRRDY_Msk) == 0){}
		
	VOR_UART1->DATA = b;
}

/*******************************************************************************
 **
 ** @brief  RX one byte over UART1, wait for a byte but timeout if no byte is received
 **
 ******************************************************************************/
uint8_t UART1RxByte(void)
{
	uint32_t timer = 1000000;
	uint8_t b =0;
	while(--timer){
		if(UART1Read(&b)){
#if UART_ECHO > 0
      UART1TxByte(b);
#endif
			break;
		}
	}
	return b;
}

/*******************************************************************************
 **
 ** @brief  Returns true if a byte is available on the RX buffer, else false
 **
 ******************************************************************************/
bool UART1IsRxByteAvailable(void)
{
	/* Disable UART RX interrupt. */
  NVIC_DisableIRQ(UART1_RX_IRQn);
    
  /* Verify that data is available. */
  if(uart1RxBufferTail == uart1RxBufferHead){
		/* Re-enable UART RX interrupt. */
    NVIC_EnableIRQ(UART1_RX_IRQn);
		return false;
	}
	/* Re-enable UART RX interrupt. */
  NVIC_EnableIRQ(UART1_RX_IRQn);
	return true;
}

/*******************************************************************************
 **
 ** @brief  Transmit a string across UART2.
 **
 ** Transmits a string across UART2, blocking until all characters are sent.
 **
 ** @param  pcStr is the pointer to the string that is transmitted across UART2.
 **
 ******************************************************************************/
void UART2TxStr(char *pcStr)
{
  if(pcStr == NULL) return;
	while(*pcStr){
    /* Block until UART is done transmitting data. */
    while((VOR_UART2->TXSTATUS & UART_TXSTATUS_WRRDY_Msk) == 0){}
        
    /* Transmit another character. */
    VOR_UART2->DATA = *pcStr++;
	}
}

/*******************************************************************************
 **
 ** @brief  TX one byte over UART2
 **
 ** @param  uint8_t b is the byte to send
 **
 ******************************************************************************/
void UART2TxByte(uint8_t b)
{
	/* Block until UART is done transmitting data. */
  while((VOR_UART2->TXSTATUS & UART_TXSTATUS_WRRDY_Msk) == 0){}
		
	VOR_UART2->DATA = b;
}

/*******************************************************************************
 **
 ** @brief  RX one byte over UART2, wait for a byte but timeout if no byte is received
 **
 ******************************************************************************/
uint8_t UART2RxByte(void)
{
	uint32_t timer = 1000000;
	uint8_t b =0;
	while(--timer){
		if(UART2Read(&b)){
#if UART_ECHO > 0
      UART2TxByte(b);
#endif
			break;
		}
	}
	return b;
}

/*******************************************************************************
 **
 ** @brief  Returns true if a byte is available on the RX buffer, else false
 **
 ******************************************************************************/
bool UART2IsRxByteAvailable(void)
{
	/* Disable UART RX interrupt. */
  NVIC_DisableIRQ(UART2_RX_IRQn);
    
  /* Verify that data is available. */
  if(uart2RxBufferTail == uart2RxBufferHead){
		/* Re-enable UART RX interrupt. */
    NVIC_EnableIRQ(UART2_RX_IRQn);
		return false;
	}
	/* Re-enable UART RX interrupt. */
  NVIC_EnableIRQ(UART2_RX_IRQn);
	return true;
}

/*******************************************************************************
 **
 ** @brief  Reads a character from the UART0 RX buffer.
 **
 ** @param  data is the pointer to store received data, if any exists.
 **
 ** @return Returns **true** if data was pulled from the RX buffer, otherwise
 **          returns **false** and zeros out data.
 **
 ******************************************************************************/
bool UART0Read(uint8_t *data)
{
  /* Disable UART RX interrupt. */
  NVIC_DisableIRQ(UART0_RX_IRQn);
    
  /* Verify that data is available. */
  if(uart0RxBufferTail == uart0RxBufferHead){
    /* Zero out data pointer. */
    *data = 0;
        
    /* Re-enable UART RX interrupt. */
    NVIC_EnableIRQ(UART0_RX_IRQn);
        
    return(false);
  }
    
  /* Pull byte from buffer. */
	*data = *uart0RxBufferTail++;
    
  /* Loop pointer if needed. */
	if(uart0RxBufferTail == uart0RxBufferEnd){
		uart0RxBufferTail = uart0RxBuffer;
  }
    
  /* Re-enable UART RX interrupt. */
	NVIC_EnableIRQ(UART0_RX_IRQn);
    
  return(true);
}

/*******************************************************************************
 **
 ** @brief  Reads a character from the UART1 RX buffer.
 **
 ** @param  data is the pointer to store received data, if any exists.
 **
 ** @return Returns **true** if data was pulled from the RX buffer, otherwise
 **          returns **false** and zeros out data.
 **
 ******************************************************************************/
bool UART1Read(uint8_t *data)
{
  /* Disable UART RX interrupt. */
  NVIC_DisableIRQ(UART1_RX_IRQn);
    
  /* Verify that data is available. */
  if(uart1RxBufferTail == uart1RxBufferHead){
    /* Zero out data pointer. */
    *data = 0;
        
    /* Re-enable UART RX interrupt. */
    NVIC_EnableIRQ(UART1_RX_IRQn);
        
    return(false);
  }
    
  /* Pull byte from buffer. */
	*data = *uart1RxBufferTail++;
    
  /* Loop pointer if needed. */
	if(uart1RxBufferTail == uart1RxBufferEnd){
		uart1RxBufferTail = uart1RxBuffer;
  }
    
  /* Re-enable UART RX interrupt. */
	NVIC_EnableIRQ(UART1_RX_IRQn);
  
  return(true);
}

/*******************************************************************************
 **
 ** @brief  Reads a character from the UART2 RX buffer.
 **
 ** @param  data is the pointer to store received data, if any exists.
 **
 ** @return Returns **true** if data was pulled from the RX buffer, otherwise
 **          returns **false** and zeros out data.
 **
 ******************************************************************************/
bool UART2Read(uint8_t *data)
{
  /* Disable UART RX interrupt. */
  NVIC_DisableIRQ(UART2_RX_IRQn);
    
  /* Verify that data is available. */
  if(uart2RxBufferTail == uart2RxBufferHead){
    /* Zero out data pointer. */
    *data = 0;
        
    /* Re-enable UART RX interrupt. */
    NVIC_EnableIRQ(UART2_RX_IRQn);
        
    return(false);
  }
    
  /* Pull byte from buffer. */
	*data = *uart2RxBufferTail++;
    
  /* Loop pointer if needed. */
	if(uart2RxBufferTail == uart2RxBufferEnd){
		uart2RxBufferTail = uart2RxBuffer;
  }
    
  /* Re-enable UART RX interrupt. */
	NVIC_EnableIRQ(UART2_RX_IRQn);
  
  return(true);
}

/*******************************************************************************
 **
 ** @brief  Reads from from the UART0 RX buffer
 **
 ** @param  data is the pointer to store received data, if any exists.
 **         limit prevents coping enough to overrun buffer in calling enviro
 **
 ** @return Returns num of char copied out.  Data may or may not not be 
 **         of string type
 **
 ******************************************************************************/
uint32_t UART0GetBuffer(uint8_t *data, uint32_t limit)
{   
	uint32_t count=0;
	/* Disable UART RX interrupt. */
	NVIC_DisableIRQ(UART0_RX_IRQn);
	
	/* Verify that data is available. */
	while((uart0RxBufferTail != uart0RxBufferHead) && limit){
		++count;
		--limit;
		/* Pull byte from buffer. */
		*data++ = *uart0RxBufferTail++;
		
	 /* Loop pointer if needed. */
		if(uart0RxBufferTail == uart0RxBufferEnd){
			uart0RxBufferTail = uart0RxBuffer;
		}
	}
    
  /* Re-enable UART RX interrupt. */
	NVIC_EnableIRQ(UART0_RX_IRQn);
    
  return(count);
}

/*******************************************************************************
 **
 ** @brief  Reads from from the UART1 RX buffer
 **
 ** @param  data is the pointer to store received data, if any exists.
 **         limit prevents coping enough to overrun buffer in calling enviro
 **
 ** @return Returns num of char copied out.  Data may or may not not be 
 **         of string type
 **
 ******************************************************************************/
uint32_t UART1GetBuffer(uint8_t *data, uint32_t limit)
{   
	uint32_t count=0;
  
	/* Disable UART RX interrupt. */
	NVIC_DisableIRQ(UART1_RX_IRQn);
	
	/* Verify that data is available. */
	while((uart1RxBufferTail != uart1RxBufferHead) && limit){
		++count;
		--limit;
		/* Pull byte from buffer. */
		*data++ = *uart1RxBufferTail++;
		
	 /* Loop pointer if needed. */
		if(uart1RxBufferTail == uart1RxBufferEnd){
			uart1RxBufferTail = uart1RxBuffer;
		}
	}
    
  /* Re-enable UART RX interrupt. */
	NVIC_EnableIRQ(UART1_RX_IRQn);
    
  return(count);
}

/*******************************************************************************
 **
 ** @brief  Reads from from the UART2 RX buffer
 **
 ** @param  data is the pointer to store received data, if any exists.
 **         limit prevents coping enough to overrun buffer in calling enviro
 **
 ** @return Returns num of char copied out.  Data may or may not not be 
 **         of string type
 **
 ******************************************************************************/
uint32_t UART2GetBuffer(uint8_t *data, uint32_t limit)
{   
	uint32_t count=0;
  
	/* Disable UART RX interrupt. */
	NVIC_DisableIRQ(UART2_RX_IRQn);
	
	/* Verify that data is available. */
	while((uart2RxBufferTail != uart2RxBufferHead) && limit){
		++count;
		--limit;
		/* Pull byte from buffer. */
		*data++ = *uart2RxBufferTail++;
		
	 /* Loop pointer if needed. */
		if(uart2RxBufferTail == uart2RxBufferEnd){
			uart2RxBufferTail = uart2RxBuffer;
		}
	}
    
  /* Re-enable UART RX interrupt. */
	NVIC_EnableIRQ(UART2_RX_IRQn);
    
  return(count);
}


/*******************************************************************************
 **
 ** @brief  UART0 RX interrupt handler.
 **
 ** Reads characters received into RX buffer.  If the RX buffer is full, the
 ** contents of the RX buffer will be overwritten.
 **
 ******************************************************************************/
void UART0_RX_IRQHandler(void)
{
  /* Read bytes into buffer as long as bytes are available. */
	while((VOR_UART0->RXSTATUS & UART_RXSTATUS_RDAVL_Msk) != 0){
		*uart0RxBufferHead++ = VOR_UART0->DATA;
		
    /* Loop pointer if needed. */
		if(uart0RxBufferHead == uart0RxBufferEnd){
      uart0RxBufferHead = uart0RxBuffer;
    }
	}
}

/*******************************************************************************
 **
 ** @brief  UART1 RX interrupt handler.
 **
 ** Reads characters received into RX buffer.  If the RX buffer is full, the
 ** contents of the RX buffer will be overwritten.
 **
 ******************************************************************************/
void UART1_RX_IRQHandler(void)
{
  /* Read bytes into buffer as long as bytes are available. */
	while((VOR_UART1->RXSTATUS & UART_RXSTATUS_RDAVL_Msk) != 0){
		*uart1RxBufferHead++ = VOR_UART1->DATA;
		
    /* Loop pointer if needed. */
		if(uart1RxBufferHead == uart1RxBufferEnd){
      uart1RxBufferHead = uart1RxBuffer;
    }
	}
}

/*******************************************************************************
 **
 ** @brief  UART2 RX interrupt handler.
 **
 ** Reads characters received into RX buffer.  If the RX buffer is full, the
 ** contents of the RX buffer will be overwritten.
 **
 ******************************************************************************/
void UART2_RX_IRQHandler(void)
{
  /* Read bytes into buffer as long as bytes are available. */
	while((VOR_UART2->RXSTATUS & UART_RXSTATUS_RDAVL_Msk) != 0){
		*uart2RxBufferHead++ = VOR_UART2->DATA;
		
    /* Loop pointer if needed. */
		if(uart2RxBufferHead == uart2RxBufferEnd){
      uart2RxBufferHead = uart2RxBuffer;
    }
	}
}

/*****************************************************************************/ 
/* End of file                                                               */ 
/*****************************************************************************/
