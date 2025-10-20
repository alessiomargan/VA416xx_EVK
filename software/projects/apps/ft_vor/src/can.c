/**
 * @file can.c
 * @brief CAN Bus Driver Implementation for VA416xx
 *
 * This file implements the CAN bus interface for the VA416xx microcontroller,
 * with support for Remote Transmission Request (RTR) handling and automatic
 * response to RTR frames. It provides communication with external systems
 * to transfer ADC data over the CAN network.
 *
 * @author Alessio Margan
 * @date September 24, 2025
 *
 * @version 1.0
 *
 * @copyright Copyright (c) 2025 IIT
 *
 * @details
 * The driver implements:
 * - CAN bus initialization and configuration
 * - RTR message handling with automatic responses
 * - Standard frame support with 11-bit identifiers
 * - Message buffer management for efficient communication
 * - Integration with the ADS1278 ADC driver for data transfer
 *
 * @note This implementation requires the VA416xx HAL CANBUS driver
 */

#include "board.h"
#include "can.h"
#include "ads1278.h"

#include "va416xx_hal_canbus.h"

// CMB14 Rx msg buffer for catch-all any standard ID message
static can_cmb_t * const can_cmb_14 = (can_cmb_t*)&VOR_CAN0->CNSTAT_CMB14;
// CMB13 Tx msg buffer for standard ID message
static can_cmb_t * const can_cmb_13 = (can_cmb_t*)&VOR_CAN0->CNSTAT_CMB13;
// CMB12 Tx msg buffer for standard ID message
static can_cmb_t * const can_cmb_12 = (can_cmb_t*)&VOR_CAN0->CNSTAT_CMB12;


static hal_can_id11_t rtr_id_0 = 0x100;
// CMB0 Rx msg buffer Remote Transmission Request
static volatile can_cmb_t * const can_cmb_0 = (volatile can_cmb_t*)&VOR_CAN0->CNSTAT_CMB0;
// CMB1 TX msg buffer Remote Transmission Response
static volatile can_cmb_t * const can_cmb_1 = (volatile can_cmb_t*)&VOR_CAN0->CNSTAT_CMB1;

static hal_can_id11_t rtr_id_1 = 0x101;
// CMB2 Rx msg buffer Remote Transmission Request
static volatile can_cmb_t * const can_cmb_2 = (volatile can_cmb_t*)&VOR_CAN0->CNSTAT_CMB2;
// CMB3 TX msg buffer Remote Transmission Response
static volatile can_cmb_t * const can_cmb_3 = (volatile can_cmb_t*)&VOR_CAN0->CNSTAT_CMB3;

static hal_can_id11_t rtr_id_2 = 0x102;
// CMB4 Rx msg buffer Remote Transmission Request
static volatile can_cmb_t * const can_cmb_4 = (volatile can_cmb_t*)&VOR_CAN0->CNSTAT_CMB4;
// CMB5 TX msg buffer Remote Transmission Response
static volatile can_cmb_t * const can_cmb_5 = (volatile can_cmb_t*)&VOR_CAN0->CNSTAT_CMB5;

static hal_can_id11_t rtr_id_3 = 0x103;
// CMB6 Rx msg buffer Remote Transmission Request
static volatile can_cmb_t * const can_cmb_6 = (volatile can_cmb_t*)&VOR_CAN0->CNSTAT_CMB6;
// CMB7 TX msg buffer Remote Transmission Response
static volatile can_cmb_t * const can_cmb_7 = (volatile can_cmb_t*)&VOR_CAN0->CNSTAT_CMB7;

static hal_can_id11_t rtr_id_4 = 0x104;
// CMB8 Rx msg buffer Remote Transmission Request
static volatile can_cmb_t * const can_cmb_8 = (volatile can_cmb_t*)&VOR_CAN0->CNSTAT_CMB8;
// CMB9 TX msg buffer Remote Transmission Response
static volatile can_cmb_t * const can_cmb_9 = (volatile can_cmb_t*)&VOR_CAN0->CNSTAT_CMB9;

static hal_can_id11_t rtr_id_5 = 0x105;
// CMB10 Rx msg buffer Remote Transmission Request
static volatile can_cmb_t * const can_cmb_10 = (volatile can_cmb_t*)&VOR_CAN0->CNSTAT_CMB10;
// CMB11 TX msg buffer Remote Transmission Response
static volatile can_cmb_t * const can_cmb_11 = (volatile can_cmb_t*)&VOR_CAN0->CNSTAT_CMB11;

// Array of buffer pointers for RTR responses
volatile can_cmb_t * const cmb_RTR_resp[] = {
    can_cmb_1, can_cmb_3, can_cmb_5, can_cmb_7, can_cmb_9, can_cmb_11};


/**
 * @brief Redefine some HAL Can functions 
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

static inline uint32_t HAL_Can_getCanPktIRQ(volatile can_cmb_t *can_cmb, can_pkt_t *myPkt)
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

static inline void HAL_Can_ConfigCMBs_RTR(volatile can_cmb_t * can_cmb_rtr, 
    volatile can_cmb_t * can_cmb_rtr_resp, 
    const hal_can_id29_or_11_t rtr_id)
{
    //HAL_Can_ConfigCMB_Rx(rtr_id_0, en_can_cmb_msgtype_STD11_REM, (can_cmb_t *)can_cmb_0);
    can_cmb_rtr->CNSTAT = en_can_cmb_cnstat_st_RX_NOT_ACTIVE;
    can_cmb_rtr->ID1 = BITMASK_AND_SHIFTL(rtr_id,10,0,5) | CAN_CMB_ID1_STD_RTR_Msk;
    can_cmb_rtr->CNSTAT = en_can_cmb_cnstat_st_RX_READY;
    
    can_cmb_rtr_resp->CNSTAT = en_can_cmb_cnstat_st_TX_NOT_ACTIVE;
    can_cmb_rtr_resp->ID1 = BITMASK_AND_SHIFTL(rtr_id,10,0,5);
    can_cmb_rtr_resp->CNSTAT = en_can_cmb_cnstat_st_TX_RTR 
                                | 8<<CAN_CNSTAT_CMB0_DLC_Pos 
                                | 0<<CAN_CNSTAT_CMB0_PRI_Pos;
}


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
    
    HAL_Can_Enable(VOR_CAN0);
    /*
    System clock = 100MHz, CAN peripheral input clock APB1 (System clock /2 ) => CKI = 50MHz
    50MHz / (PSC * (1+TSEG1+TSEG2))
    50MHz / (PSC * (1+3+6))
    */
    canConfig.CTIM_TSEG2 = (6UL-1UL);  //6 time quanta 
    canConfig.CTIM_TSEG1 = (3UL-1UL);  //3 time quanta
    canConfig.CTIM_SJW   = (1UL-1UL);  //1 time quanta
    //Configure CAN timing for 500 kbps 
    //canConfig.CTIM_PSC   = (10UL-2UL);  //CAN PreScalar=10
    //Configure CAN timing for 250 kbps 
    canConfig.CTIM_PSC   = (20UL-2UL);  //CAN PreScalar=20
    //Configure CAN timing for 125 kbps 
    //canConfig.CTIM_PSC   = (40UL-2UL);  //CAN PreScalar=40
    
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
    //VOR_CAN0->CIEN = 0x40FF; // bits 0-7 0X00FF + bit 14 (0x4000)
    VOR_CAN0->CIEN = 0x4000; // bit 14 (0x4000)

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
    VOR_CAN0->GMSKB = 0x0010;  // ENABLE remote transmission request for standard frame
    
    /* Set up buffer-specific mask for buffer 14 */
    // To make buffer 14 a true catch-all, set the mask to all 0s (don't care)
    VOR_CAN0->BMSKX = 0;
    VOR_CAN0->BMSKB = 0xFFE0; // CMB14 DISABLE remote transmission request for standard frame
    
    /* Clear all CAN message buffers */
    HAL_Can_ClearAllBufferStatus();

    /* Configure Rx msg 14 as a catch-all for any standard ID message */
    HAL_Can_ConfigCMB_Rx(0x0, en_can_cmb_msgtype_STD11, can_cmb_14);
    
    /* Configure Rx msg 13 */
    /* ID 0x123, standard frame */
    //HAL_Can_ConfigCMB_Rx(0x123, en_can_cmb_msgtype_STD11, can_cmb_13);

    /* Configure rx CMB0 for Remote Transmission Request - standard frame */
    /* Configure tx CMB1 for Remote Transmission Request Response - standard frame */
    HAL_Can_ConfigCMBs_RTR(can_cmb_0, can_cmb_1, rtr_id_0);
    /* Configure rx CMB2 for Remote Transmission Request - standard frame */
    /* Configure tx CMB3 for Remote Transmission Request Response - standard frame */
    HAL_Can_ConfigCMBs_RTR(can_cmb_2, can_cmb_3, rtr_id_1);
    /* Configure rx CMB4 for Remote Transmission Request - standard frame */
    /* Configure tx CMB5 for Remote Transmission Request Response - standard frame */
    HAL_Can_ConfigCMBs_RTR(can_cmb_4, can_cmb_5, rtr_id_2);
    /* Configure rx CMB6 for Remote Transmission Request - standard frame */
    /* Configure tx CMB7 for Remote Transmission Request Response - standard frame */
    HAL_Can_ConfigCMBs_RTR(can_cmb_6, can_cmb_7, rtr_id_3);
    /* Configure rx CMB8 for Remote Transmission Request - standard frame */
    /* Configure tx CMB9 for Remote Transmission Request Response - standard frame */
    HAL_Can_ConfigCMBs_RTR(can_cmb_8, can_cmb_9, rtr_id_4);
    /* Configure rx CMB10 for Remote Transmission Request - standard frame */
    /* Configure tx CMB11 for Remote Transmission Request Response - standard frame */
    HAL_Can_ConfigCMBs_RTR(can_cmb_10, can_cmb_11, rtr_id_5);    

#if 0
    can_pkt_t testPkt;
    testPkt.msgType = en_can_cmb_msgtype_STD11;
    testPkt.id = 0x200; 
    testPkt.dataLengthBytes = 8;
    testPkt.data16[0] = 0x1234;
    testPkt.data16[1] = 0x5678;
    testPkt.data16[2] = 0x9ABC;
    testPkt.data16[3] = 0xDEF0;
    /* Send the response using a free transmit buffer */
    HAL_Can_sendCanPkt(can_cmb_13, &testPkt);
#endif 
}

/**
 * @brief CAN0 Interrupt handler
 * 
 */
void CAN0_IRQHandler(void)
{
    can_pkt_t rxPkt;
    can_pkt_t respPkt;
    
    volatile uint32_t    error_counter = VOR_CAN0->CANEC;
    volatile uint32_t    status_pending = VOR_CAN0->CSTPND; 
    volatile uint32_t    irq_pending = VOR_CAN0->CIPND; 
    
    printf("%s %lu %lu %lu\n",__FUNCTION__,irq_pending,status_pending,error_counter);

    /* CAN message buffers are configured for automatic transmission
       after reception of a Remote Transmission Request (RTR)
       No need if IRQ
    */

    // CMB14
    if (irq_pending & 0x4000) {
        /* Clear the interrupt flag for buffer 14 */
        VOR_CAN0->CICLR = 0x4000;
        /* Get the received packet from buffer 14 */
        if (HAL_Can_getCanPktIRQ(can_cmb_14, &rxPkt) != 0) {
            return;
        }

#if 0
        printf("Buffer 14 received ID: 0x%X, DLC: %lu\n", 
                (unsigned int)rxPkt.id, rxPkt.dataLengthBytes);
        printf("Data: ");
        for (int i = 0; i < (rxPkt.dataLengthBytes+1)/2; i++) {
            printf("%04X ", rxPkt.data16[i]);
        }
        printf("\n");
#endif

        switch (rxPkt.id)
        {
        case 0x123:
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
            HAL_Can_sendCanPkt(can_cmb_12, &respPkt);
            break;
        
        case 0x4FE:
            /* AFE request */
            AFE11612_ProcessRequest(&rxPkt, &respPkt);
            /* Send the response using a free transmit buffer */
            HAL_Can_sendCanPkt(can_cmb_12, &respPkt);
            break;
        
        default:
            respPkt.id = 0x0EE;
            respPkt.dataLengthBytes = rxPkt.dataLengthBytes;
            respPkt.txPriorityCode = 0;   /* Highest priority */
            respPkt.msgType = en_can_cmb_msgtype_STD11;
            respPkt.data16[0] = 0xE0E0;
            respPkt.data16[1] = 0xE0E0;;
            respPkt.data16[2] = 0xE0E0;;
            respPkt.data16[3] = 0xE0E0;;
            /* Send the response using a free transmit buffer */
            HAL_Can_sendCanPkt(can_cmb_13, &respPkt);
            break;
        }
    
        can_cmb_14->CNSTAT = en_can_cmb_cnstat_st_RX_READY;
    }
    
}