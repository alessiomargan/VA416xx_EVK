#include "board.h"
#include "peripherals.h"

#include "va416xx_hal_canbus.h"
/**
 * @brief Redefine some HAL can functions 
 * 
 */
static inline void HAL_Can_ClearBufferStatus(can_cmb_t *can_cmb)
{
    can_cmb->CNSTAT = en_can_cmb_cnstat_st_RX_NOT_ACTIVE;
}

static inline void HAL_Can_ClearAllBufferStatus(void)
{
    can_cmb_t *can_cmb = (can_cmb_t*)&VOR_CAN0->CNSTAT_CMB0;
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

 /**
 * @brief Configures and initializes the CAN0 peripheral
 * 
 * This function configures the CAN0 controller with default settings
 * using the VA416xx HAL CANBUS driver.
 */
void ConfigureCAN0(void)
{
    hal_can_id29_or_11_t dontCareIDNone=0x0;//care about all=must be exact match
    /* CAN configuration structure */
    can_config_t canConfig;
    
#if 0  
    VOR_SYSCONFIG->PERIPHERAL_CLK_ENABLE |= CLK_ENABLE_CAN0 | CLK_ENABLE_CAN1;
    VOR_SYSCONFIG->PERIPHERAL_RESET &= ~(CLK_ENABLE_CAN0 | CLK_ENABLE_CAN1);
    __NOP();
    __NOP();
    VOR_SYSCONFIG->PERIPHERAL_RESET |= (CLK_ENABLE_CAN0 | CLK_ENABLE_CAN1);

    uint32_t* regPtr;
    for(regPtr=(uint32_t *)VOR_CAN0; regPtr<=(uint32_t *)&VOR_CAN0->CTMR; regPtr++){
        *regPtr = 0x0000;
    }
    for(regPtr=(uint32_t *)VOR_CAN1; regPtr<=(uint32_t *)&VOR_CAN1->CTMR; regPtr++){
        *regPtr = 0x0000;
    }
#endif
    HAL_Can_Enable(VOR_CAN0);
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
    canConfig.CGCR = 0x0
		| CAN_CGCR_CANEN_Msk          // CAN enable
//        | CAN_CGCR_CRX_Msk        // Control receive   
//        | CAN_CGCR_CTX_Msk        // Control transmit
        | CAN_CGCR_BUFFLOCK_Msk  
        | CAN_CGCR_TSTPEN_Msk    
//        | CAN_CGCR_DDIR_Msk      
//        | CAN_CGCR_LO_Msk        
 		| CAN_CGCR_IGNACK_Msk
//		| CAN_CGCR_LOOPBACK_Msk  //no loopback here
//        | CAN_CGCR_INTERNAL_Msk  
        | CAN_CGCR_DIAGEN_Msk    
        | CAN_CGCR_EIT_Msk;

    /* Enable interrupt for all message buffers */
    //canConfig.CICEN = CAN_CICEN_ICEN_Msk | CAN_CICEN_EICEN_Msk; /* Enable interrupt for all 15 buffers + errors */
    canConfig.CICEN = (BITMASK(ALLF,14,0)<<CAN_CICEN_ICEN_Pos) ;//| CAN_CICEN_EICEN_Msk;
    //canConfig.CICEN = 0x7FFF;

    /* Initialize CAN0 module with configuration and enable NVIC interrupt for CAN0 */
    HAL_Can_Setup(VOR_CAN0, &canConfig, CAN0_IRQn);
    
    //VOR_CAN0->CIEN = (BITMASK(ALLF,14,0)<<CAN_CIEN_IEN_Pos) ;//| CAN_CIEN_EIEN_Msk;
    VOR_CAN0->CIEN = 0x7 ; // only buffers 0 1 2 !!!
    
    /* Set up global acceptance mask */
    HAL_Can_Setup_gMask(VOR_CAN0, HAL_Can_Make_maskBX(dontCareIDNone, 1, 1));
    
    /* Set up buffer-specific mask for buffer 14 */
    HAL_Can_Setup_bMask(VOR_CAN0, HAL_Can_Make_maskBX(dontCareIDNone, 1, 1));
    
    /* Clear all CAN message buffers */
    //HAL_Can_ClearBuffer((can_cmb_t*)&VOR_CAN0->CNSTAT_CMB0 , 14); // 15 ?!?
    HAL_Can_ClearAllBufferStatus();

    /* Configure receive message buffer 0 */
    /* ID 0x123, standard frame */
    HAL_Can_ConfigCMB_Rx(0x123, en_can_cmb_msgtype_STD11, (can_cmb_t*)&VOR_CAN0->CNSTAT_CMB0);

    hal_can_id29_or_11_t rem_req_resp_id = 0x100;
    /* Configure rx msg buffer 1 */
    /* ID 0x100, Remote Request */
    HAL_Can_ConfigCMB_Rx(rem_req_resp_id, en_can_cmb_msgtype_STD11_REM, (can_cmb_t*)&VOR_CAN0->CNSTAT_CMB1);
    /* Configure tx msg buffer 1 */
    /* ID 0x100, Remote Transmit Response*/
    /* Set up buffer 2 for automatic RTR response */
    VOR_CAN0->CNSTAT_CMB2 = en_can_cmb_cnstat_st_TX_NOT_ACTIVE;
    VOR_CAN0->DATA0_CMB2 = 0xAA55;
    VOR_CAN0->DATA1_CMB2 = 0xBB66;
    VOR_CAN0->DATA2_CMB2 = 0xCC77;
    VOR_CAN0->DATA3_CMB2 = 0xDD88;
    VOR_CAN0->ID1_CMB2 = BITMASK_AND_SHIFTL(rem_req_resp_id,10,0,5); // | CAN_CMB_ID1_STD_RTR_Msk;
    VOR_CAN0->CNSTAT_CMB2 = en_can_cmb_cnstat_st_TX_RTR
                        | 8<<CAN_CNSTAT_CMB0_DLC_Pos 
                        | 0<<CAN_CNSTAT_CMB0_PRI_Pos;

    
    can_pkt_t testPkt;
    testPkt.msgType = en_can_cmb_msgtype_STD11;
    testPkt.id = 0x200; 
    testPkt.dataLengthBytes = 8;
    testPkt.data16[0] = 0x1234;
    testPkt.data16[1] = 0x5678;
    testPkt.data16[2] = 0x9ABC;
    testPkt.data16[3] = 0xDEF0;
    HAL_Can_sendCanPkt((can_cmb_t*)&VOR_CAN0->CNSTAT_CMB3, &testPkt);
 
}

/**
 * @brief CAN0 Interrupt handler
 * 
 * This function handles RTR frames that require software intervention
 * (not handled by automatic RTR response buffers)
 */
void CAN0_IRQHandler(void)
{
    can_pkt_t rxPkt;
    can_pkt_t respPkt;
    
    uint32_t    error_counter = VOR_CAN0->CANEC;
    uint32_t    status_pending = VOR_CAN0->CSTPND; 
    uint32_t    irq_pending = VOR_CAN0->CIPND; 
    
    printf("%s %lu %lu %lu\n",__FUNCTION__,irq_pending,status_pending, error_counter);

    /* Clear the interrupt flag for all buffers and error*/
    //VOR_CAN0->CICLR = 0xFFFF;

    /* Check if buffer 0 received a message */
    if (irq_pending & 0x0001) {
        
        /* Get the received packet from buffer 0 */
        /* NEED to review this function !!!!
         * - no difference btw remote request frame and data frame
         * - DATA0..DATA3 always copied , use dataLengthBytes ?!?!
         */
        if ( HAL_Can_getCanPkt((can_cmb_t*)&VOR_CAN0->CNSTAT_CMB0, &rxPkt) != 0) {            
            return;
        }
        /* Clear the interrupt flag for buffer 0 */
        VOR_CAN0->CICLR = 0x0001;
        VOR_CAN0->CNSTAT_CMB0 = en_can_cmb_cnstat_st_RX_READY;
        /* Prepare response packet */
        respPkt.id = 0x321;
        respPkt.dataLengthBytes = rxPkt.dataLengthBytes;
        respPkt.txPriorityCode = 0;   /* Highest priority */
        respPkt.msgType = en_can_cmb_msgtype_STD11;
        respPkt.data16[0] = rxPkt.data16[0];
        respPkt.data16[1] = rxPkt.data16[1];
        respPkt.data16[2] = rxPkt.data16[2];
        respPkt.data16[3] = rxPkt.data16[3];
        /* Send the response using a free transmit buffer */
        HAL_Can_sendCanPkt((can_cmb_t*)&VOR_CAN0->CNSTAT_CMB4, &respPkt);
    
    } else if (irq_pending & 0x0002) {
        
        //VOR_CAN0->DATA0_CMB2 = 0xCACA;
        /* Clear the interrupt flag for buffer 1 */
        VOR_CAN0->CICLR = 0x0002;
        VOR_CAN0->CNSTAT_CMB1 = en_can_cmb_cnstat_st_RX_READY;
    
    } else if (irq_pending & 0x0004) {
    
        VOR_CAN0->DATA0_CMB2 += 0x1111;
        VOR_CAN0->DATA1_CMB2 += 0x1111;
        VOR_CAN0->DATA2_CMB2 += 0x1111;
        VOR_CAN0->DATA3_CMB2 += 0x1111;
        /* Clear the interrupt flag for buffer 2 */
        VOR_CAN0->CICLR = 0x0004;
        //VOR_CAN0->CNSTAT_CMB2 = en_can_cmb_cnstat_st_RX_READY;
    
    }
    /* Handle other buffer interrupts if needed */
}