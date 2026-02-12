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
/* CAN configuration structure */
static can_config_t canConfig =
{
    /** 
     * Configure CAN general settings
    The CGCR is used to:
    - Enable/disable the CAN module
    - Configure the BUFFLOCK function for the message buffers 0 to 14
    - Enable/disable the time stamp synchronization
    - Set the logic levels of the CAN input/output pins
    - Choose the data storage direction
    - Select the error interrupt type
    - Enable/disable diagnostic functions
    **/
    .CGCR = 0x0
		| CAN_CGCR_CANEN_Msk        // CAN enable
//        | CAN_CGCR_CRX_Msk        // Control receive   
//        | CAN_CGCR_CTX_Msk        // Control transmit
        | CAN_CGCR_BUFFLOCK_Msk  
        | CAN_CGCR_TSTPEN_Msk       // Time sync enable
//        | CAN_CGCR_DDIR_Msk      
//        | CAN_CGCR_LO_Msk        
 		| CAN_CGCR_IGNACK_Msk       // Ignore ACK
//		| CAN_CGCR_LOOPBACK_Msk     //no loopback here
//        | CAN_CGCR_INTERNAL_Msk  
        | CAN_CGCR_DIAGEN_Msk    
        | CAN_CGCR_EIT_Msk,         // Error interrupt type
    /**
     * System clock = 100MHz, CAN peripheral input clock APB1 (System clock /2 ) => CKI = 50MHz
     * 50MHz / (PSC * (1+TSEG1+TSEG2))
     * 50MHz / (PSC * (1+3+6))
    **/
    .CTIM_TSEG2 = (6UL-1UL),  //6 time quanta,
    .CTIM_TSEG1 = (3UL-1UL),  //3 time quanta,
    .CTIM_SJW   = (1UL-1UL),  //1 time quanta,
    // Configure CAN timing for 500 kbps
    //.CTIM_PSC   = (10UL-2UL), //CAN PreScalar=10,
    // Configure CAN timing for 250 kbps
    .CTIM_PSC   = (20UL-2UL), //CAN PreScalar=20,
    // Configure CAN timing for 125 kbps
    //.CTIM_PSC   = (40UL-2UL), //CAN PreScalar=40,
    /**
     * CAN Interrupt Code Enable Register (CICEN) 
     * The CICEN register determines whether the interrupt pending flag in IPND must be translated
     * into the Interrupt Code field of the STPND register
     **/
    .CICEN = CAN_CICEN_ICEN_Msk | CAN_CICEN_EICEN_Msk // Enable error and interrupt conditions
};

static inline void HAL_Can_ClearBufferStatus(can_cmb_t *can_cmb)
{
    can_cmb->CNSTAT = en_can_cmb_cnstat_st_RX_NOT_ACTIVE;
}

static inline void HAL_Can_ClearAllBufferStatus(VOR_CAN_Type *myCAN)
{
    can_cmb_t *can_cmb = (can_cmb_t*)&myCAN->CNSTAT_CMB0;
    for (uint8_t i = 0; i <= 14; i++) {
        can_cmb[i].CNSTAT = en_can_cmb_cnstat_st_RX_NOT_ACTIVE;
    }
}

static inline void HAL_Can_Enable(VOR_CAN_Type *myCAN)
{
    uint32_t clk_enable;
    if (myCAN == VOR_CAN0) {
        clk_enable = CLK_ENABLE_CAN0;
    } else if (myCAN == VOR_CAN1) {
        clk_enable = CLK_ENABLE_CAN1;
    } else {
        return;
    }

    VOR_SYSCONFIG->PERIPHERAL_CLK_ENABLE |= clk_enable;
    VOR_SYSCONFIG->PERIPHERAL_RESET &= ~clk_enable;
    __NOP();
    __NOP();
    VOR_SYSCONFIG->PERIPHERAL_RESET |= clk_enable;

    uint32_t* regPtr;
    for(regPtr=(uint32_t *)myCAN; regPtr<=(uint32_t *)&myCAN->CTMR; regPtr++){
        *regPtr = 0x0000;
    }
}

/*******************************************************************************
 **
 ** @brief  Initialize CAN0 for bootloader operation
 **
 ******************************************************************************/
void CAN_BootloaderInit(void)
{
  HAL_Can_Enable(VOR_CAN0);
  /* Initialize CAN0 module with configuration and enable NVIC interrupt for CAN0 */
  HAL_Can_Setup(VOR_CAN0, &canConfig, CAN0_IRQn);
  
  /** CAN Interrupt Enable Register (CIEN) 
   * The CIEN register enables the transmit and receive interrupts of message buffers 0 through 14,
   * and the CAN error interrupt
   **/
  VOR_CAN0->CIEN = 0x0003; // bits 0-1 0X0003
  
  /** CAN Global Mask Extension Register (GMSKX)
   * The GMSKX register enables global masking for incoming identifier bits,
   * allowing the software to globally mask the incoming extended identifier bit XRTR
   **/
  VOR_CAN0->GMSKX = 0;
  /** CAN Global Mask Base Register (GMSKB)
   * The GMSKB register enables global masking for incoming identifier bits,
   * allowing the software to globally mask the identifier bits RTR and IDE.
   **/
  VOR_CAN0->GMSKB = 0x0010;  // ENABLE remote transmission request for standard frame
  
  /* Set up buffer-specific mask for buffer 14 */
  // Buffer 14 is configured as a catch-all for standard ID messages
  // Software: IRQ handler filters out IDs 0x200-0x205 (reserved for CAN1)
  VOR_CAN0->BMSKX = 0;            // unused for standard IDs
  VOR_CAN0->BMSKB = (0x7FF << 5)|(0x1 << 4) ;   // Mask = 0x7FF, accepts any ID (Buffer_ID = 0x0) + RTR bit
  
  // CAN Acceptance Rule: Message_Accepted = (Incoming_ID & MASK) == (Buffer_ID & MASK)
  // Example: (0x123 & 0x0) == (0x0 & 0x0) → TRUE, message accepted
  //          (0x456 & 0x0) == (0x0 & 0x0) → TRUE, message accepted
  
  /* Clear all CAN message buffers */
  HAL_Can_ClearAllBufferStatus(VOR_CAN0);

  // Configure RX buffer for command messages (ID 0x200)
  HAL_Can_ConfigCMB_Rx(CAN_BL_CMD_ID, en_can_cmb_msgtype_STD11, can0_cmb_rx_cmd);
  
  // Configure RX buffer for data messages (ID 0x201)
  HAL_Can_ConfigCMB_Rx(CAN_BL_DATA_ID, en_can_cmb_msgtype_STD11, can0_cmb_rx_data);
  
  // Configure TX buffer for response messages (ID 0x202)
  // Note: For TX buffers, the ID is set when sending the message, so we can configure with a default ID
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
  return ((can0_cmb_rx_cmd->CNSTAT & CAN_CNSTAT_CMB0_ST_Msk) == 0x3) ||
         ((can0_cmb_rx_data->CNSTAT & CAN_CNSTAT_CMB0_ST_Msk) == 0x3);
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
  can_pkt_t rxPkt;
  uint8_t *data8;
  // Check if message is available in either RX buffers cmd or data 
  if (HAL_Can_getCanPkt(can0_cmb_rx_cmd, &rxPkt) != 0) {
    if (HAL_Can_getCanPkt(can0_cmb_rx_data, &rxPkt) != 0) {
      return false; // No message available
    }
  }
  // Message received, extract ID, data, and length  
  *pId = rxPkt.id;
  *pLen = rxPkt.dataLengthBytes;
  data8 = (uint8_t *)rxPkt.data16;
  for (uint8_t i = 0; i < *pLen && i < 8; i++) {
    pData[i] = data8[i];
  }
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
  can_pkt_t respPkt;
  respPkt.id = id;
  respPkt.dataLengthBytes = len;
  respPkt.txPriorityCode = 0;   /* Highest priority */
  respPkt.msgType = en_can_cmb_msgtype_STD11;
  uint8_t *data8 = (uint8_t *)respPkt.data16;
  for (uint8_t i = 0; i < len && i < 8; i++) {
    data8[i] = pData[i];
  } 
  HAL_Can_sendCanPkt(can0_cmb_tx, &respPkt);
  return true;
}

/*******************************************************************************
 **
 ** @brief  CAN0 interrupt handler
 **
 ******************************************************************************/
void CAN0_IRQHandler(void)
{
}

/*****************************************************************************/
/* End of file                                                               */
/*****************************************************************************/
