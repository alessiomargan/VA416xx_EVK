#include "board.h"
#include "peripherals.h"

#include "va416xx_hal_canbus.h"
/**
 * @brief Redefine some HAL can functions 
 * 
 */

 #define BIT(x) (1UL << (x))

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

static inline uint32_t HAL_Can_getCanPktIRQ(can_cmb_t *can_cmb, can_pkt_t *myPkt)
{
    myPkt->id = can_cmb->ID1 >> 5;
    
    myPkt->dataLengthBytes = can_cmb->CNSTAT >> CAN0_CNSTAT_CMB0_DLC_Pos; //12
    if(myPkt->dataLengthBytes>8){
        return 1; //illegal data length??
    }

    //excess bytes beyond dataLengthBytes is junk but not cleared here 
    myPkt->data16[0]=can_cmb->DATA0;
    myPkt->data16[1]=can_cmb->DATA1;
    myPkt->data16[2]=can_cmb->DATA2;
    myPkt->data16[3]=can_cmb->DATA3;

    myPkt->timestamp16=can_cmb->TSTP;

    can_cmb->CNSTAT = en_can_cmb_cnstat_st_TX_NOT_ACTIVE; //clr state to RX_NOT_ACTIVE, wipes DLC/PRI too
    
    return 0;
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

    /**
     * CAN Interrupt Code Enable Register (CICEN) 
     * The CICEN register determines whether the interrupt pending flag in IPND must be translated
     * into the Interrupt Code field of the STPND register
     **/
    canConfig.CICEN = CAN_CICEN_ICEN_Msk | CAN_CICEN_EICEN_Msk;
    //canConfig.CICEN = 0x7FFF | 0x8000;

    /* Initialize CAN0 module with configuration and enable NVIC interrupt for CAN0 */
    HAL_Can_Setup(VOR_CAN0, &canConfig, CAN0_IRQn);
    
    /** CAN Interrupt Enable Register (CIEN) 
     * The CIEN register enables the transmit and receive interrupts of message buffers 0 through 14,
     * and the CAN error interrupt
     **/
    VOR_CAN0->CIEN = BIT(0) | BIT(1) | BIT(2) | BIT(14); // bits 0,1,2 (0x7) and bit 14 (0x4000)

    /* Set up global acceptance mask to only accept standard frames */
    //HAL_Can_Setup_gMask(VOR_CAN0, HAL_Can_Make_maskBX(0x0, false, true));
    //VOR_CAN0->GMSKB &= ~CAN_GMSKB_IDE_Msk;  // Only accept standard frames (IDE=0) 
    //HAL_Can_Setup_gMask(VOR_CAN0, HAL_Can_Make_maskBX(dontCareIDNone, 1, 1));
    
    /** CAN Global Mask Extension Register (GMSKX)
     **/
    VOR_CAN0->GMSKX = 0;
    /** CAN Global Mask Base Register (GMSKB)
     * The GMSKB register enables global masking for incoming identifier bits,
     * allowing the software to globally mask the identifier bits RTR and IDE.
     **/
    VOR_CAN0->GMSKB = 0x0010;
    
    /* Set up buffer-specific mask for buffer 14 */
    // To make buffer 14 a true catch-all, set the mask to all 0s (don't care)
    VOR_CAN0->BMSKX = 0;
    VOR_CAN0->BMSKB = 0xFFF0; // 0x0010 + 0xFFE0
    
    /* Clear all CAN message buffers */
    HAL_Can_ClearAllBufferStatus();

    /* Configure buffer 14 as a catch-all for any standard ID message */
    HAL_Can_ConfigCMB_Rx(0x0, en_can_cmb_msgtype_STD11, (can_cmb_t*)&VOR_CAN0->CNSTAT_CMB14);
    
    /* Configure receive message buffer 0 */
    /* ID 0x123, standard frame */
    HAL_Can_ConfigCMB_Rx(0x123, en_can_cmb_msgtype_STD11, (can_cmb_t*)&VOR_CAN0->CNSTAT_CMB0);

    hal_can_id11_t rem_req_resp_id = 0x100;
    /* Configure rx msg buffer 1 */
    /* ID 0x100, Remote Request */
    HAL_Can_ConfigCMB_Rx(rem_req_resp_id, en_can_cmb_msgtype_STD11_REM, (can_cmb_t*)&VOR_CAN0->CNSTAT_CMB1);
    /* Configure tx msg buffer 2 */
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
 */
void CAN0_IRQHandler(void)
{
    can_pkt_t rxPkt;
    can_pkt_t respPkt;
    
    uint32_t    error_counter = VOR_CAN0->CANEC;
    uint32_t    status_pending = VOR_CAN0->CSTPND; 
    uint32_t    irq_pending = VOR_CAN0->CIPND; 
    
    printf("%s %lu %lu %lu\n",__FUNCTION__,irq_pending,status_pending,error_counter);

    /* Clear the interrupt flag for all buffers and error*/
    //VOR_CAN0->CICLR = 0xFFFF;

    /* Check if buffer 0 received a message */
    if (irq_pending & 0x0001) {
        /* Get the received packet from buffer 0 */
        if ( HAL_Can_getCanPktIRQ((can_cmb_t*)&VOR_CAN0->CNSTAT_CMB0, &rxPkt) != 0) {            
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
    
    } 
    
    if (irq_pending & 0x0002) {
        /* Get the received packet from buffer 1 */        
        
        /* Clear the interrupt flag for buffer 1 */
        VOR_CAN0->CICLR = 0x0002;
        VOR_CAN0->CNSTAT_CMB1 = en_can_cmb_cnstat_st_RX_READY;
    
    }
    
    if (irq_pending & 0x0004) {
    
        VOR_CAN0->DATA0_CMB2 += 0x1111;
        VOR_CAN0->DATA1_CMB2 += 0x1111;
        VOR_CAN0->DATA2_CMB2 += 0x1111;
        VOR_CAN0->DATA3_CMB2 += 0x1111;
        /* Clear the interrupt flag for buffer 2 */
        VOR_CAN0->CICLR = 0x0004;
        //VOR_CAN0->CNSTAT_CMB2 = en_can_cmb_cnstat_st_RX_READY;
    
    }
    
    if (irq_pending & 0x4000) {
        
        /* Get the received packet from buffer 14 */
        if (HAL_Can_getCanPktIRQ((can_cmb_t*)&VOR_CAN0->CNSTAT_CMB14, &rxPkt) == 0) {
#if 1
            printf("Buffer 14 received ID: 0x%X, DLC: %lu\n", 
                    (unsigned int)rxPkt.id, rxPkt.dataLengthBytes);
            printf("Data: ");
            for (int i = 0; i < (rxPkt.dataLengthBytes+1)/2; i++) {
                printf("%04X ", rxPkt.data16[i]);
            }
            printf("\n");
#endif
            }
        /* Clear the interrupt flag for buffer 14 */
        VOR_CAN0->CICLR = 0x4000;
        VOR_CAN0->CNSTAT_CMB14 = en_can_cmb_cnstat_st_RX_READY;
    }
    
}