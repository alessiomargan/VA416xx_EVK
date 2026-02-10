/***************************************************************************************
 * @file     can.c
 * @version  V1.00
 * @date     10 February 2026
 *
 * @note
 * VORAGO Technologies
 *
 * @note
 * Copyright (c) 2013-2026 VORAGO Technologies.
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

#include "va416xx.h"
#include "board.h"
#include "can.h"
#include "can_protocol.h"
#include "va416xx_hal_canbus.h"

/*****************************************************************************/
/* Local variables                                                           */
/*****************************************************************************/

// CAN0 Message Buffer for RX (bootloader commands and data)
static volatile can_cmb_t * const can0_cmb_rx_cmd = (volatile can_cmb_t*)&VOR_CAN0->CNSTAT_CMB0;
static volatile can_cmb_t * const can0_cmb_rx_data = (volatile can_cmb_t*)&VOR_CAN0->CNSTAT_CMB1;

// CAN0 Message Buffer for TX (bootloader responses)
static volatile can_cmb_t * const can0_cmb_tx = (volatile can_cmb_t*)&VOR_CAN0->CNSTAT_CMB2;

/*****************************************************************************/
/* Function implementation                                                   */
/*****************************************************************************/

/*******************************************************************************
 **
 ** @brief  Initialize CAN0 for bootloader operation
 **
 ******************************************************************************/
void CAN_BootloaderInit(void)
{
  hal_can_config_t canConfig;
  
  // Enable CAN0 clock
  VOR_SYSCONFIG->PERIPHERAL_CLK_ENABLE |= CLK_ENABLE_CAN0;
  
  // Reset CAN0
  VOR_SYSCONFIG->PERIPHERAL_RESET &= ~SYSCONFIG_PERIPHERAL_RESET_CAN0_Msk;
  __NOP();
  __NOP();
  VOR_SYSCONFIG->PERIPHERAL_RESET |= SYSCONFIG_PERIPHERAL_RESET_CAN0_Msk;
  
  // Configure CAN0 at 250 kbps (assuming 80 MHz system clock)
  canConfig.clkdiv = 4;           // Divide by 4 -> 20 MHz
  canConfig.tseg1 = 13;           // Time segment 1
  canConfig.tseg2 = 2;            // Time segment 2
  canConfig.sjw = 1;              // Synchronization jump width
  canConfig.loopback = false;
  canConfig.listenonly = false;
  canConfig.bufflock = false;
  
  HAL_CanBus_Init(VOR_CAN0, &canConfig);
  
  // Configure RX buffer for command messages (ID 0x200)
  HAL_CanBus_ConfigBuffer11(VOR_CAN0, 0, CAN_BL_CMD_ID, 
                            hal_can_dir_rx, hal_can_bufmode_normal);
  
  // Configure RX buffer for data messages (ID 0x201)
  HAL_CanBus_ConfigBuffer11(VOR_CAN0, 1, CAN_BL_DATA_ID, 
                            hal_can_dir_rx, hal_can_bufmode_normal);
  
  // Configure TX buffer for response messages (ID 0x202)
  HAL_CanBus_ConfigBuffer11(VOR_CAN0, 2, CAN_BL_RESP_ID, 
                            hal_can_dir_tx, hal_can_bufmode_normal);
  
  // Enable CAN0 interrupts
  NVIC_EnableIRQ(CAN0_IRQn);
  NVIC_SetPriority(CAN0_IRQn, 2);
}

/*******************************************************************************
 **
 ** @brief  Check if a CAN message is available
 **
 ** @return True if message available, false otherwise
 **
 ******************************************************************************/
bool CAN_IsRxMsgAvailable(void)
{
  // Check if either RX buffer has a message
  return ((can0_cmb_rx_cmd->CNSTAT & CAN_CMB_CNSTAT_ST_Msk) == 0x3) ||
         ((can0_cmb_rx_data->CNSTAT & CAN_CMB_CNSTAT_ST_Msk) == 0x3);
}

/*******************************************************************************
 **
 ** @brief  Receive a CAN message
 **
 ** @param  pId   Pointer to store message ID
 ** @param  pData Pointer to store message data
 ** @param  pLen  Pointer to store message length
 **
 ** @return True if message received, false otherwise
 **
 ******************************************************************************/
bool CAN_ReceiveMessage(uint32_t *pId, uint8_t *pData, uint8_t *pLen)
{
  volatile can_cmb_t *cmb = NULL;
  
  // Check command buffer first
  if ((can0_cmb_rx_cmd->CNSTAT & CAN_CMB_CNSTAT_ST_Msk) == 0x3) {
    cmb = can0_cmb_rx_cmd;
    *pId = CAN_BL_CMD_ID;
  }
  // Check data buffer
  else if ((can0_cmb_rx_data->CNSTAT & CAN_CMB_CNSTAT_ST_Msk) == 0x3) {
    cmb = can0_cmb_rx_data;
    *pId = CAN_BL_DATA_ID;
  }
  else {
    return false;
  }
  
  // Get message length
  *pLen = (cmb->CNSTAT & CAN_CMB_CNSTAT_DLC_Msk) >> CAN_CMB_CNSTAT_DLC_Pos;
  
  // Read data bytes
  for (uint8_t i = 0; i < *pLen && i < 8; i++) {
    pData[i] = cmb->DATA[i];
  }
  
  // Clear the buffer to receive next message
  cmb->CNSTAT &= ~CAN_CMB_CNSTAT_ST_Msk;
  
  return true;
}

/*******************************************************************************
 **
 ** @brief  Send a CAN message
 **
 ** @param  id    Message ID
 ** @param  pData Pointer to message data
 ** @param  len   Message length
 **
 ** @return True if message sent, false otherwise
 **
 ******************************************************************************/
bool CAN_SendMessage(uint32_t id, uint8_t *pData, uint8_t len)
{
  // Check if TX buffer is available
  if ((can0_cmb_tx->CNSTAT & CAN_CMB_CNSTAT_ST_Msk) == 0x3) {
    // Buffer is full, can't send
    return false;
  }
  
  // Limit length to 8 bytes
  if (len > 8) {
    len = 8;
  }
  
  // Write data to buffer
  for (uint8_t i = 0; i < len; i++) {
    can0_cmb_tx->DATA[i] = pData[i];
  }
  
  // Set message length and trigger transmission
  can0_cmb_tx->CNSTAT = (can0_cmb_tx->CNSTAT & ~CAN_CMB_CNSTAT_DLC_Msk) | 
                        (len << CAN_CMB_CNSTAT_DLC_Pos);
  can0_cmb_tx->CNSTAT |= CAN_CMB_CNSTAT_ST_Msk; // Set to TX ready (0x3)
  
  return true;
}

/*******************************************************************************
 **
 ** @brief  CAN0 interrupt handler
 **
 ******************************************************************************/
void CAN0_IRQHandler(void)
{
  // Clear interrupt flags
  uint32_t status = VOR_CAN0->CANGIF;
  VOR_CAN0->CANGIF = status;
}

/*****************************************************************************/
/* End of file                                                               */
/*****************************************************************************/
