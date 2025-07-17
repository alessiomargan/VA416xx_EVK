#include "board.h"
#include "peripherals.h"

#include "va416xx_hal_canbus.h"

/*
The CGCR is used to:
 - Enable/disable the CAN module
 - Configure the BUFFLOCK function for the message buffers 0 to 14
 - Enable/disable the time stamp synchronization
 - Set the logic levels of the CAN input/output pins
 - Choose the data storage direction
 - Select the error interrupt type
 - Enable/disable diagnostic functions
*/

 /**
 * @brief Configures and initializes the CAN0 peripheral
 * 
 * This function configures the CAN0 controller with default settings
 * using the VA416xx HAL CANBUS driver.
 */
void ConfigureCAN0(void)
{
    /* CAN configuration structure */
    can_config_t canConfig;
    
    /*
    System clock = 100MHz, CAN peripheral input clock, CKI = 50MHz
    50MHz / (PSC * (1+TSEG1+TSEG2))
    50MHz / (PSC * (1+3+6))
    */
    canConfig.CTIM_TSEG2 = (6UL-1UL);  //6 time quanta 
    canConfig.CTIM_TSEG1 = (3UL-1UL);  //3 time quanta
    canConfig.CTIM_SJW   = (1UL-1UL);  //1 time quanta
    //Configure CAN timing for 500 kbps 
    //canConfig.CTIM_PSC   = (10UL-2UL);  //CAN PreScalar=10
    //Configure CAN timing for 125 kbps 
    canConfig.CTIM_PSC   = (40UL-2UL);  //CAN PreScalar=10
    
    /*
     Configure CAN general settings
    The CGCR is used to:
    - Enable/disable the CAN module
    - Configure the BUFFLOCK function for the message buffers 0 to 14
    - Enable/disable the time stamp synchronization
    - Set the logic levels of the CAN input/output pins
    - Choose the data storage direction
    - Select the error interrupt type
    - Enable/disable diagnostic functions
    */
    canConfig.CGCR = 
		CAN_CGCR_CANEN_Msk          // CAN enable
//        | CAN_CGCR_CRX_Msk        // Control receive   
//        | CAN_CGCR_CTX_Msk        // Control transmit
        | CAN_CGCR_BUFFLOCK_Msk  
        | CAN_CGCR_TSTPEN_Msk    
//        | CAN_CGCR_DDIR_Msk      
//        | CAN_CGCR_LO_Msk        
// 		| CAN_CGCR_IGNACK_Msk
//		| CAN_CGCR_LOOPBACK_Msk  //no loopback here
//        | CAN_CGCR_INTERNAL_Msk  
        | CAN_CGCR_DIAGEN_Msk    
        | CAN_CGCR_EIT_Msk;

    /* Enable interrupt for all message buffers */
    canConfig.CICEN = CAN_CICEN_ICEN_Msk | CAN_CICEN_EICEN_Msk; /* Enable interrupt for all 15 buffers + errors */
    
    /* Initialize CAN0 module with configuration and enable NVIC interrupt for CAN0 */
    HAL_Can_Setup(VOR_CAN0, &canConfig, CAN0_IRQn);
    
    VOR_CAN0->CIEN = CAN_CIEN_IEN_Msk | CAN_CIEN_EIEN_Msk; /* Enable interrupt for all 15 buffers + errors */
    
    /* Set up global acceptance mask */
    HAL_Can_Setup_gMask(VOR_CAN0, HAL_Can_Make_maskBX(0, false, true));
    
    /* Set up buffer-specific mask for buffer 14 */
    HAL_Can_Setup_bMask(VOR_CAN0, HAL_Can_Make_maskBX(0, false, true));
    
    /* Clear all CAN message buffers */
    HAL_Can_ClearBuffer((can_cmb_t*)&VOR_CAN0->CNSTAT_CMB0 , 15);
  
    /* Configure receive message buffers (example for buffer 0) */
    /* ID 0x123, standard frame */
    HAL_Can_ConfigCMB_Rx(0x123, en_can_cmb_msgtype_STD11_REM, (can_cmb_t*)&VOR_CAN0->CNSTAT_CMB0);
    
#if 0
    /* ID 0x100, standard frame */
    HAL_Can_ConfigCMB_Rx(0x100, en_can_cmb_msgtype_STD11_REM, (can_cmb_t*)&VOR_CAN0->CNSTAT_CMB1);
    /* Set up buffer 1 for automatic RTR response */
    can_pkt_t autoRtrPkt;
    autoRtrPkt.msgType = en_can_cmb_msgtype_STD11_REM_RESP;  // This is the key setting
    autoRtrPkt.id = 0x100;  // ID that will trigger automatic response
    autoRtrPkt.dataLengthBytes = 8;
    autoRtrPkt.txPriorityCode = 0;  // Highest priority
    /* Prepare response data */
    autoRtrPkt.data16[0] = 0xAA55;
    autoRtrPkt.data16[1] = 0xBB66;
    autoRtrPkt.data16[2] = 0xCC77;
    autoRtrPkt.data16[3] = 0xDD88;
    /* Configure buffer 1 for auto-response */
    HAL_Can_sendCanPkt((can_cmb_t*)&VOR_CAN0->CNSTAT_CMB2, &autoRtrPkt);
#endif
    
    can_pkt_t testPkt;
    testPkt.msgType = en_can_cmb_msgtype_STD11;
    testPkt.id = 0x200; 
    testPkt.dataLengthBytes = 8;
    testPkt.data16[0] = 0x1234;
    testPkt.data16[1] = 0x5678;
    testPkt.data16[2] = 0x9ABC;
    testPkt.data16[3] = 0xDEF0;
    HAL_Can_sendCanPkt((can_cmb_t*)&VOR_CAN0->CNSTAT_CMB5, &testPkt);
 
}

/**
 * @brief CAN0 Interrupt handler to process received RTR frames
 * 
 * This function handles RTR frames that require software intervention
 * (not handled by automatic RTR response buffers)
 */
void CAN0_IRQHandler(void)
{
    can_pkt_t rxPkt;
    can_pkt_t respPkt;
    
    printf("%s\n",__FUNCTION__);

    /* Check if buffer 0 received a message */
    if (VOR_CAN0->CSTPND & 0x01) {
        
        /* Get the received packet from buffer 0 */
        HAL_Can_getCanPkt((can_cmb_t*)&VOR_CAN0->CNSTAT_CMB0, &rxPkt);
        
        /* Check if this is an RTR frame that needs manual handling */
        if ((rxPkt.msgType == en_can_cmb_msgtype_STD11_REM) || 
            (rxPkt.msgType == en_can_cmb_msgtype_EXT29_REM)) {
            
            /* Prepare response packet */
            respPkt.id = rxPkt.id;
            respPkt.dataLengthBytes = 8;  /* Set appropriate data length */
            respPkt.txPriorityCode = 0;   /* Highest priority */
            
            /* Set message type based on the request type */
            if (rxPkt.msgType == en_can_cmb_msgtype_STD11_REM) {
                respPkt.msgType = en_can_cmb_msgtype_STD11;
            } else {
                respPkt.msgType = en_can_cmb_msgtype_EXT29;
            }
            
            /* Set response data based on requested ID */
            switch (rxPkt.id) {
                case 0x123:  /* Example ID */
                    respPkt.data16[0] = 0x0102;
                    respPkt.data16[1] = 0x0304;
                    respPkt.data16[2] = 0x0506;
                    respPkt.data16[3] = 0x0708;
                    break;
                    
                default:
                    /* Default response for unknown RTR */
                    respPkt.data16[0] = 0xFFFF;
                    respPkt.data16[1] = 0xFFFF;
                    respPkt.data16[2] = 0xFFFF;
                    respPkt.data16[3] = 0xFFFF;
                    break;
            }
            
            /* Send the response using a free transmit buffer */
            HAL_Can_sendCanPkt((can_cmb_t*)&VOR_CAN0->CNSTAT_CMB3, &respPkt);
        }
        
        /* Clear the interrupt flag for buffer 0 */
        VOR_CAN0->CSTPND = 0x01;
    }
    
    /* Handle other buffer interrupts if needed */
}