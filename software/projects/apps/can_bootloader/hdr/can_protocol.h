/***************************************************************************************
 * @file     can_protocol.h
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
 
#ifndef __CAN_PROTOCOL_H
#define __CAN_PROTOCOL_H

/*****************************************************************************/
/* Include files                                                             */
/*****************************************************************************/

#include <stdint.h>
#include <stdbool.h>

/*****************************************************************************/
/* Global pre-processor symbols/macros ('#define')                          */
/*****************************************************************************/

/* CAN Bootloader Message IDs */
#define CAN_BL_CMD_ID           0x200  // Commands from host to bootloader
#define CAN_BL_DATA_ID          0x201  // Data packets from host to bootloader
#define CAN_BL_RESP_ID          0x202  // Responses from bootloader to host

/* CAN Bootloader Commands (in first byte of CMD message) */
#define CAN_BL_CMD_PING         0x01   // Ping bootloader
#define CAN_BL_CMD_START_UL     0x02   // Start upload (slot A or B)
#define CAN_BL_CMD_DATA         0x03   // Data packet follows
#define CAN_BL_CMD_END_UL       0x04   // End upload
#define CAN_BL_CMD_RUN_APP      0x05   // Run application
#define CAN_BL_CMD_GET_STATUS   0x06   // Get bootloader status
#define CAN_BL_CMD_RESET        0x07   // Reset device

/* CAN Bootloader Response Codes */
#define CAN_BL_RESP_ACK         0x01   // Command acknowledged
#define CAN_BL_RESP_NAK         0x02   // Command not acknowledged
#define CAN_BL_RESP_READY       0x03   // Ready for data
#define CAN_BL_RESP_BUSY        0x04   // Busy, try again
#define CAN_BL_RESP_ERROR       0x05   // Error occurred
#define CAN_BL_RESP_CRC_OK      0x06   // CRC check passed
#define CAN_BL_RESP_CRC_FAIL    0x07   // CRC check failed
#define CAN_BL_RESP_APP_VALID   0x08   // App is valid
#define CAN_BL_RESP_APP_INVALID 0x09   // App is invalid

/* Application slot selection */
#define CAN_BL_SLOT_A           0x00
#define CAN_BL_SLOT_B           0x01

/* CAN packet size for data transfer */
#define CAN_BL_DATA_SIZE        7      // 7 bytes of data per packet (1 byte for sequence)

/* Timeout values */
#define CAN_BL_RX_TIMEOUT_MS    5000   // 5 second timeout for receiving data

/*****************************************************************************/
/* Global type definitions ('typedef')                                       */
/*****************************************************************************/

/* CAN bootloader state machine states */
typedef enum {
  CAN_BL_STATE_IDLE = 0,
  CAN_BL_STATE_READY,
  CAN_BL_STATE_RECEIVING,
  CAN_BL_STATE_VERIFYING,
  CAN_BL_STATE_ERROR
} can_bl_state_t;

/*****************************************************************************/
/* Global function prototypes                                                */
/*****************************************************************************/

extern void CANProtocol_Init(void);
extern void CANProtocol_Task(void);
extern size_t CANProtocol_ReceiveFile(uint8_t *pDest, uint8_t *pCrcLenAddr);
extern void CANProtocol_SendResponse(uint8_t respCode, uint8_t *pData, uint8_t len);

/*****************************************************************************/
/* End of file                                                               */
/*****************************************************************************/
#endif /* __CAN_PROTOCOL_H */
