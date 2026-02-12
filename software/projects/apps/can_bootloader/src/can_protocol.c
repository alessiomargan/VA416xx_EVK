/***************************************************************************************
 * @file     can_protocol.c
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

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "va416xx.h"
#include "board.h"
#include "can.h"
#include "can_protocol.h"
#include "bootloader.h"
#include "main.h"
#include "crc.h"

/*****************************************************************************/
/* Local variables                                                           */
/*****************************************************************************/

static can_bl_state_t gBLState = CAN_BL_STATE_IDLE;
static uint8_t gCurrentSlot = CAN_BL_SLOT_A;
static uint32_t gBytesReceived = 0;
static uint32_t gExpectedBytes = 0;
static uint8_t *gpDestAddr = NULL;

/*****************************************************************************/
/* Function implementation                                                   */
/*****************************************************************************/

/*******************************************************************************
 **
 ** @brief  Initialize CAN protocol handler
 **
 ******************************************************************************/
void CANProtocol_Init(void)
{
  gBLState = CAN_BL_STATE_IDLE;
  gBytesReceived = 0;
  gExpectedBytes = 0;
  gpDestAddr = NULL;
}

/*******************************************************************************
 **
 ** @brief  Send a response message via CAN
 **
 ** @param  respCode Response code
 ** @param  pData    Optional data to send
 ** @param  len      Length of optional data
 **
 ******************************************************************************/
void CANProtocol_SendResponse(uint8_t respCode, uint8_t *pData, uint8_t len)
{
  uint8_t txData[8];
  
  txData[0] = respCode;
  
  // Copy optional data
  for (uint8_t i = 0; i < len && i < 7; i++) {
    txData[i + 1] = pData[i];
  }
  
  CAN_SendMessage(CAN_BL_RESP_ID, txData, len + 1);
}

/*******************************************************************************
 **
 ** @brief  Process received CAN command message
 **
 ** @param  pData Message data
 ** @param  len   Message length
 **
 ******************************************************************************/
static void ProcessCommand(uint8_t *pData, uint8_t len)
{
  uint8_t cmd = pData[0];
  
  switch (cmd) {
    case CAN_BL_CMD_PING:
      // Respond with ACK to indicate bootloader is alive
      CANProtocol_SendResponse(CAN_BL_RESP_ACK, NULL, 0);
      break;
      
    case CAN_BL_CMD_START_UL:
      if (len >= 5) {
        // pData[1] = slot (A or B)
        // pData[2-4] = expected length (24-bit, up to 16MB)
        gCurrentSlot = pData[1];
        gExpectedBytes = (uint32_t)pData[2] | ((uint32_t)pData[3] << 8) | 
                        ((uint32_t)pData[4] << 16);
        
        if (gCurrentSlot == CAN_BL_SLOT_A) {
          gpDestAddr = (uint8_t*)APP_A_START_ADDR;
        } else {
          gpDestAddr = (uint8_t*)APP_B_START_ADDR;
        }
        
        gBytesReceived = 0;
        gBLState = CAN_BL_STATE_READY;
        
        // Enable writes to code memory
        VOR_SYSCONFIG->ROM_PROT |= SYSCONFIG_ROM_PROT_WREN_Msk;
        
        CANProtocol_SendResponse(CAN_BL_RESP_READY, NULL, 0);
      } else {
        CANProtocol_SendResponse(CAN_BL_RESP_ERROR, NULL, 0);
      }
      break;
      
    case CAN_BL_CMD_GET_STATUS:
      {
        uint8_t statusData[4];
        statusData[0] = (uint8_t)gBLState;
        statusData[1] = (uint8_t)(gBytesReceived & 0xFF);
        statusData[2] = (uint8_t)((gBytesReceived >> 8) & 0xFF);
        statusData[3] = (uint8_t)((gBytesReceived >> 16) & 0xFF);
        CANProtocol_SendResponse(CAN_BL_RESP_ACK, statusData, 4);
      }
      break;
      
    case CAN_BL_CMD_RESET:
      CANProtocol_SendResponse(CAN_BL_RESP_ACK, NULL, 0);
      HAL_WaitMs(10);
      NVIC_SystemReset();
      break;
      
    default:
      CANProtocol_SendResponse(CAN_BL_RESP_NAK, NULL, 0);
      break;
  }
}

/*******************************************************************************
 **
 ** @brief  Process received CAN data message
 **
 ** @param  pData Message data
 ** @param  len   Message length
 **
 ******************************************************************************/
static void ProcessData(uint8_t *pData, uint8_t len)
{
  if (gBLState != CAN_BL_STATE_READY && gBLState != CAN_BL_STATE_RECEIVING) {
    CANProtocol_SendResponse(CAN_BL_RESP_ERROR, NULL, 0);
    return;
  }
  
  // First byte is sequence number, rest is data
  uint8_t seqNum = pData[0];
  uint8_t dataLen = len - 1;
  
  if (dataLen > 0 && gpDestAddr != NULL) {
    // Copy data to destination
    for (uint8_t i = 0; i < dataLen; i++) {
      if (gBytesReceived < gExpectedBytes) {
        gpDestAddr[gBytesReceived++] = pData[i + 1];
      }
    }
    
    gBLState = CAN_BL_STATE_RECEIVING;
    
    // Send ACK with sequence number
    CANProtocol_SendResponse(CAN_BL_RESP_ACK, &seqNum, 1);
    
    // Check if transfer is complete
    if (gBytesReceived >= gExpectedBytes) {
      // Disable writes to code memory
      VOR_SYSCONFIG->ROM_PROT &= ~SYSCONFIG_ROM_PROT_WREN_Msk;
      
      gBLState = CAN_BL_STATE_VERIFYING;
      CANProtocol_SendResponse(CAN_BL_RESP_CRC_OK, NULL, 0);
    }
  }
}

/*******************************************************************************
 **
 ** @brief  CAN protocol task - processes incoming messages
 **
 ******************************************************************************/
void CANProtocol_Task(void)
{
  uint32_t msgId;
  uint8_t msgData[8];
  uint8_t msgLen;
  
  // Check for incoming messages
  if (CAN_ReceiveMessage(&msgId, msgData, &msgLen)) {
    WDFEED(); // Feed watchdog
    
    if (msgId == CAN_BL_CMD_ID) {
      ProcessCommand(msgData, msgLen);
    } else if (msgId == CAN_BL_DATA_ID) {
      ProcessData(msgData, msgLen);
    }
  }
}

/*******************************************************************************
 **
 ** @brief  Receive a file via CAN protocol
 **
 ** @param  pDest       Destination address for received data
 ** @param  pCrcLenAddr Address to store CRC length
 **
 ** @return Number of bytes received, or 0 on error
 **
 ******************************************************************************/
size_t CANProtocol_ReceiveFile(uint8_t *pDest, uint8_t *pCrcLenAddr)
{
  uint64_t timeoutMs;
  
  gpDestAddr = pDest;
  gBytesReceived = 0;
  gBLState = CAN_BL_STATE_IDLE;
  
  // Send ready message
  CANProtocol_SendResponse(CAN_BL_RESP_READY, NULL, 0);
  
  // Wait for START_UL command
  timeoutMs = HAL_time_ms + CAN_BL_RX_TIMEOUT_MS;
  while (HAL_time_ms < timeoutMs && gBLState == CAN_BL_STATE_IDLE) {
    CANProtocol_Task();
    WDFEED();
  }
  
  if (gBLState != CAN_BL_STATE_READY) {
    return 0; // Timeout or error
  }
  
  // Receive data packets
  timeoutMs = HAL_time_ms + CAN_BL_RX_TIMEOUT_MS;
  while (gBLState == CAN_BL_STATE_READY || gBLState == CAN_BL_STATE_RECEIVING) {
    CANProtocol_Task();
    WDFEED();
    
    // Check for timeout
    if (HAL_time_ms > timeoutMs) {
      gBLState = CAN_BL_STATE_ERROR;
      return 0;
    }
    
    // Reset timeout on each packet
    if (gBLState == CAN_BL_STATE_RECEIVING) {
      timeoutMs = HAL_time_ms + CAN_BL_RX_TIMEOUT_MS;
    }
  }
  
  if (gBLState == CAN_BL_STATE_VERIFYING && gBytesReceived > 0) {
    gBLState = CAN_BL_STATE_IDLE;
    return gBytesReceived;
  }
  
  return 0;
}

/*****************************************************************************/
/* End of file                                                               */
/*****************************************************************************/
