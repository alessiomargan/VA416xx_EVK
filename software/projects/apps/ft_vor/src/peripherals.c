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
    
    /* Configure CAN timing for 500 kbps with 16MHz clock */
    canConfig.CTIM_PSC = 0;    /* Prescaler = 2 (value + 2) */
    canConfig.CTIM_SJW = 3;    /* Sync Jump Width = 4 (value + 1) */
    canConfig.CTIM_TSEG1 = 10; /* TSEG1 = 11 (value + 1) */
    canConfig.CTIM_TSEG2 = 3;  /* TSEG2 = 4 (value + 1) */
    
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
//        | CAN_CGCR_BUFFLOCK_Msk  
        | CAN_CGCR_TSTPEN_Msk    
//        | CAN_CGCR_DDIR_Msk      
//        | CAN_CGCR_LO_Msk        
// 		| CAN_CGCR_IGNACK_Msk
//		| CAN_CGCR_LOOPBACK_Msk  //no loopback here
//        | CAN_CGCR_INTERNAL_Msk  
        | CAN_CGCR_DIAGEN_Msk    
        | CAN_CGCR_EIT_Msk;

    /* Enable interrupt for all message buffers */
    canConfig.CICEN = 0x7FFF; /* Enable interrupt for all 15 message buffers */
    
    /* Initialize CAN0 module with configuration */
    HAL_Can_Setup(VOR_CAN0, &canConfig, CAN0_IRQn);
    
    /* Set up global acceptance mask - accept all messages initially */
    HAL_Can_Setup_gMask(VOR_CAN0, HAL_Can_Make_maskBX(0, true, true));
    
    /* Set up buffer-specific mask for buffer 14 */
    HAL_Can_Setup_bMask(VOR_CAN0, HAL_Can_Make_maskBX(0, true, true));
    
    /* Clear all CAN message buffers */
    HAL_Can_ClearBuffer((can_cmb_t*)&VOR_CAN0->CNSTAT_CMB0 , 15);
  
    /* Configure receive message buffers (example for buffer 0) */
    /* ID 0x123, standard frame */
    HAL_Can_ConfigCMB_Rx(0x123, en_can_cmb_msgtype_STD11, (can_cmb_t*)&VOR_CAN0->CNSTAT_CMB0);
    
    /* Configure another buffer for extended ID frames if needed */
    /* ID 0x18FF5500, extended frame */
    HAL_Can_ConfigCMB_Rx(0x18FF5500, en_can_cmb_msgtype_EXT29, (can_cmb_t*)&VOR_CAN0->CNSTAT_CMB1);
    
    /* Configure a buffer for RTR auto-response if needed */
    can_cmb_t* rxRtrBuf = (can_cmb_t*)&VOR_CAN0->CNSTAT_CMB2;
    can_pkt_t rtrRespPkt;
    
    rtrRespPkt.msgType = en_can_cmb_msgtype_STD11_REM_RESP;
    rtrRespPkt.id = 0x456; /* ID 0x456 */
    rtrRespPkt.dataLengthBytes = 8;
    rtrRespPkt.data16[0] = 0x1234;
    rtrRespPkt.data16[1] = 0x5678;
    rtrRespPkt.data16[2] = 0x9ABC;
    rtrRespPkt.data16[3] = 0xDEF0;
    
    HAL_Can_sendCanPkt(rxRtrBuf, &rtrRespPkt);
    
    /* Enable NVIC interrupt for CAN0 */
    NVIC_EnableIRQ(CAN0_IRQn);
}