/***************************************************************************************
 * @file     va416xx_hal_spw.c
 * @version  V0.2
 * @date     17 June 2022
 *
 * @note
 * VORAGO Technologies
 *
 * @note
 * Copyright (c) 2013-2022 VORAGO Technologies.
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

#include "va416xx_hal_spw.h"
#include "va416xx_debug.h"

/*****************************************************************************/
/* Local pre-processor symbols/macros ('#define')                            */
/*****************************************************************************/

#define SPW_LINK_TIMEOUT_MS (100)
#define SPW_TX_TIMEOUT_MS   (100)
#define SPW_TIMEOUT (1000000)


/*****************************************************************************/
/* Global variable definitions (declared in header file with 'extern')       */
/*****************************************************************************/



// spacewire module is Big Endian Double Word
    uint32_t txBigEndianData[64];   // 256 bytes
    uint32_t rxBigEndianData[64];   // 256 bytes
    uint32_t txHdrBuffer[10];       // 40 bytes

    // transmit descriptor table this buffer address is loaded into 'DMATXDESC0' register
#if ((defined __ICCARM__) || (defined __GNUC__) || (defined __clang__))
    SpWtxDesc_t dmaTxDescTable[16] __attribute__((section(".ARM.__at_0x20007000"))) = {0};
#else
    SpWtxDesc_t dmaTxDescTable[16] __attribute__((section(".ARM.__at_0x20007000"), zero_init));
#endif

    // receive descriptor table this buffer address is loaded into 'DMARXDESC0' register
#if ((defined __ICCARM__) || (defined __GNUC__) || (defined __clang__))
    SpWrxDesc_t dmaRxDescTable[16] __attribute__((section(".ARM.__at_0x20007100"))) = {0};
#else
    SpWrxDesc_t dmaRxDescTable[16] __attribute__((section(".ARM.__at_0x20007100"), zero_init));
#endif

    SpWtxDesc_t *txdesc = &dmaTxDescTable[0];
    SpWrxDesc_t *rxdesc = &dmaRxDescTable[0];

    rmapTransaction_t rmapTrans;
    rmapTransaction_t *pRMAPt = &rmapTrans;

    uint32_t txDriverErrors;
    uint32_t rxDriverErrors;

/*****************************************************************************/
/* Local type definitions ('typedef')                                        */
/*****************************************************************************/

/*****************************************************************************/
/* Local variable definitions ('static')                                     */
/*****************************************************************************/
    
/** Table to compute RMAP CRC */
static const uint8_t RMAP_CRC_TABLE[256] = {
    0x00, 0x91, 0xe3, 0x72, 0x07, 0x96, 0xe4, 0x75,
    0x0e, 0x9f, 0xed, 0x7c, 0x09, 0x98, 0xea, 0x7b,
    0x1c, 0x8d, 0xff, 0x6e, 0x1b, 0x8a, 0xf8, 0x69,
    0x12, 0x83, 0xf1, 0x60, 0x15, 0x84, 0xf6, 0x67,
    0x38, 0xa9, 0xdb, 0x4a, 0x3f, 0xae, 0xdc, 0x4d,
    0x36, 0xa7, 0xd5, 0x44, 0x31, 0xa0, 0xd2, 0x43,
    0x24, 0xb5, 0xc7, 0x56, 0x23, 0xb2, 0xc0, 0x51,
    0x2a, 0xbb, 0xc9, 0x58, 0x2d, 0xbc, 0xce, 0x5f,
    0x70, 0xe1, 0x93, 0x02, 0x77, 0xe6, 0x94, 0x05,
    0x7e, 0xef, 0x9d, 0x0c, 0x79, 0xe8, 0x9a, 0x0b,
    0x6c, 0xfd, 0x8f, 0x1e, 0x6b, 0xfa, 0x88, 0x19,
    0x62, 0xf3, 0x81, 0x10, 0x65, 0xf4, 0x86, 0x17,
    0x48, 0xd9, 0xab, 0x3a, 0x4f, 0xde, 0xac, 0x3d,
    0x46, 0xd7, 0xa5, 0x34, 0x41, 0xd0, 0xa2, 0x33,
    0x54, 0xc5, 0xb7, 0x26, 0x53, 0xc2, 0xb0, 0x21,
    0x5a, 0xcb, 0xb9, 0x28, 0x5d, 0xcc, 0xbe, 0x2f,
    0xe0, 0x71, 0x03, 0x92, 0xe7, 0x76, 0x04, 0x95,
    0xee, 0x7f, 0x0d, 0x9c, 0xe9, 0x78, 0x0a, 0x9b,
    0xfc, 0x6d, 0x1f, 0x8e, 0xfb, 0x6a, 0x18, 0x89,
    0xf2, 0x63, 0x11, 0x80, 0xf5, 0x64, 0x16, 0x87,
    0xd8, 0x49, 0x3b, 0xaa, 0xdf, 0x4e, 0x3c, 0xad,
    0xd6, 0x47, 0x35, 0xa4, 0xd1, 0x40, 0x32, 0xa3,
    0xc4, 0x55, 0x27, 0xb6, 0xc3, 0x52, 0x20, 0xb1,
    0xca, 0x5b, 0x29, 0xb8, 0xcd, 0x5c, 0x2e, 0xbf,
    0x90, 0x01, 0x73, 0xe2, 0x97, 0x06, 0x74, 0xe5,
    0x9e, 0x0f, 0x7d, 0xec, 0x99, 0x08, 0x7a, 0xeb,
    0x8c, 0x1d, 0x6f, 0xfe, 0x8b, 0x1a, 0x68, 0xf9,
    0x82, 0x13, 0x61, 0xf0, 0x85, 0x14, 0x66, 0xf7,
    0xa8, 0x39, 0x4b, 0xda, 0xaf, 0x3e, 0x4c, 0xdd,
    0xa6, 0x37, 0x45, 0xd4, 0xa1, 0x30, 0x42, 0xd3,
    0xb4, 0x25, 0x57, 0xc6, 0xb3, 0x22, 0x50, 0xc1,
    0xba, 0x2b, 0x59, 0xc8, 0xbd, 0x2c, 0x5e, 0xcf,
};

/*****************************************************************************/
/* Local function prototypes ('static')                                      */
/*****************************************************************************/

//static uint8_t CalculateRmapCrc(const uint8_t *, uint32_t);


/*****************************************************************************/
/* Function implementation - global ('extern') and local ('static')          */
/*****************************************************************************/

/**@brief SpaceWire Transmit Driver
 *
 */
bool spwTransmit(SpWtxDesc_t *desc)
{
    bool error = false;
    uint32_t length = 0;
    uint32_t dwCount = 0;
    uint32_t dwIndex = 0;
    uint32_t *Src = NULL;

    uint32_t hdrLength = (desc->flagsHdrLen & SPW_TXDESC_HDRLEN_MASK);

    if ((VOR_SPW->DMACTRL0 & SPW_DMACTRL0_TE_Msk) == 0)
    {
        ///< test if there is a header in this message
        if (hdrLength)
        {
            dwCount = hdrLength/NUM_BYTES_WORD;
            if (hdrLength % NUM_BYTES_WORD)
            {
                dwCount++;
            }

            ///< convert header from little endian to big endian
            for (dwIndex = 0; dwIndex < dwCount; dwIndex++)
            {
                txHdrBuffer[dwIndex] = __REV(*(uint32_t *)desc->header++);
            }
        }

        desc->header = (uint8_t *)&txHdrBuffer[0];

        ///< convert data from little endian to big endian
        length = desc->dataLength;
        dwCount = length/NUM_BYTES_WORD;
        if (length % NUM_BYTES_WORD)
        {
            dwCount++;
        }

        Src = (uint32_t *)desc->data;
        for (dwIndex = 0; dwIndex < dwCount ; dwIndex++, Src++)
        {
            txBigEndianData[dwIndex] = __REV(*Src);     // convert to big endian
        }

        txdesc->data = (uint8_t *)&txBigEndianData[0];

        // enable TX descriptor, this should clear when tx is complete
        txdesc->flagsHdrLen |= TXDESC_CTRL_EN_Msk;

        // enable transmit in the state machine
        VOR_SPW->DMACTRL0 |= SPW_DMACTRL0_TE_Msk;
        
        uint32_t timeout = SPW_TIMEOUT;
        while (txdesc->flagsHdrLen & TXDESC_CTRL_EN_Msk) {
          if(--timeout == 0){
            error = true;
            break; 
          }
        };

        ///< save link error
        txDriverErrors = (txdesc->flagsHdrLen & 0x00008000);

        if (VOR_SPW->DMATXDESC0 == 0x20007100)
        {
            VOR_SPW->DMATXDESC0 = (uint32_t)&dmaTxDescTable[0];
        }

        txdesc = (SpWtxDesc_t *)VOR_SPW->DMATXDESC0;
    }
    return error;
}

/**@brief SpaceWire Receive
 *
 */
void spwReceive(uint32_t *saveBufr)
{
    uint32_t dwCount = 0;
    uint32_t packetLength = 0;
    uint32_t *inpacket = NULL;

    uint32_t ctrl = rxdesc->flagsPacketLen;

    if (( ctrl & RXDESC_CTRL_EN_Msk ) == 0)
    {

        packetLength = (ctrl & RXDESC_CTRL_PACKETLENGTH_Msk);

        if (packetLength)
        {
            dwCount = packetLength/NUM_BYTES_WORD;
            if (packetLength % NUM_BYTES_WORD)
            {
                dwCount++;
            }

            inpacket = rxdesc->packetAddr;

            for (uint32_t index = 0; index < dwCount; index++, inpacket++)
            {
                saveBufr[index] = __REV(*inpacket);   // convert
            }

        }

        if ( VOR_SPW->DMARXDESC0 == 0x20007180)
        {
            VOR_SPW->DMARXDESC0 = (uint32_t)&dmaRxDescTable[0];
        }

        rxdesc = (SpWrxDesc_t *)VOR_SPW->DMARXDESC0;
        rxdesc->packetAddr = (uint32_t *)&rxBigEndianData[0];
        rxdesc->flagsPacketLen = (RXDESC_INITENABLE | RXDESC_CTRL_EN_Msk);
    }
}




/**@brief Create RMAP CRC
 *
 */
uint8_t CalculateRmapCrc(const uint8_t *data, uint32_t dataLength)
{
    uint8_t crc = 0;
	for (uint32_t index = 0; index < dataLength; index++)
    {
		crc = RMAP_CRC_TABLE[(crc ^ data[index]) & BYTE_MASK];
	}
    return crc;
}

/**@brief
 *
 */
uint32_t createRmapTransaction(rmapTransaction_t *rtrans)
{
    uint8_t rmapOutBuffer[128];
    uint8_t scratch = 0;
    uint32_t crcStartlength = 0;
    uint32_t byteCount = 0;

    rmapOutBuffer[byteCount++] = rtrans->targetSpWaddr[0];
    rmapOutBuffer[byteCount++] = 0x0;
    rmapOutBuffer[byteCount++] = VOR_SPW->DEFADDR;
    crcStartlength = byteCount;
    rmapOutBuffer[byteCount++] = rtrans->targetLogicalAddress;
    rmapOutBuffer[byteCount++] = RMAP_PROTOCOL_ID;
    rmapOutBuffer[byteCount++] = rtrans->instructionField;
//    rmapOutBuffer[byteCount++] = rtrans->destinationKey;
    rmapOutBuffer[byteCount++] = (uint8_t)VOR_SPW->DKEY;

    switch (rtrans->replyAddressLength) {
        case REPLY_ADDR_LENGTH_12:
            rmapOutBuffer[byteCount++] = rtrans->replyAddress[scratch++];
            rmapOutBuffer[byteCount++] = rtrans->replyAddress[scratch++];
            rmapOutBuffer[byteCount++] = rtrans->replyAddress[scratch++];
            rmapOutBuffer[byteCount++] = rtrans->replyAddress[scratch++];
        case REPLY_ADDR_LENGTH_8:
            rmapOutBuffer[byteCount++] = rtrans->replyAddress[scratch++];
            rmapOutBuffer[byteCount++] = rtrans->replyAddress[scratch++];
            rmapOutBuffer[byteCount++] = rtrans->replyAddress[scratch++];
            rmapOutBuffer[byteCount++] = rtrans->replyAddress[scratch++];
        case REPLY_ADDR_LENGTH_4:
            rmapOutBuffer[byteCount++] = rtrans->replyAddress[scratch++];
            rmapOutBuffer[byteCount++] = rtrans->replyAddress[scratch++];
            rmapOutBuffer[byteCount++] = rtrans->replyAddress[scratch++];
            rmapOutBuffer[byteCount++] = rtrans->replyAddress[scratch++];
            break;
        case REPLY_ADDR_LENGTH_0:
            break;
    }

    rmapOutBuffer[byteCount++] = rtrans->initiatorLogicalAddr;
    rmapOutBuffer[byteCount++] = (uint8_t)(BYTE_SHIFT << rtrans->transactionIdent);
    rmapOutBuffer[byteCount++] = (uint8_t)rtrans->transactionIdent;
    rmapOutBuffer[byteCount++] = rtrans->extendedAddr;
    rmapOutBuffer[byteCount++] = (uint8_t)(SHIFT_24BITS << rtrans->memoryAddr);
    rmapOutBuffer[byteCount++] = (uint8_t)(SHIFT_16BITS << rtrans->memoryAddr);
    rmapOutBuffer[byteCount++] = (uint8_t)(BYTE_SHIFT << rtrans->memoryAddr);
    rmapOutBuffer[byteCount++] = (uint8_t)rtrans->memoryAddr;

    rmapOutBuffer[byteCount++] = (uint8_t)(SHIFT_16BITS << rtrans->dataLength);
    rmapOutBuffer[byteCount++] = (uint8_t)(BYTE_SHIFT << rtrans->dataLength);
    rmapOutBuffer[byteCount++] = (uint8_t)rtrans->dataLength;
    scratch = (byteCount - crcStartlength);
    rmapOutBuffer[byteCount] = CalculateRmapCrc(&rmapOutBuffer[crcStartlength], scratch);
    byteCount++;
    crcStartlength = byteCount;

    if ((rtrans->instructionField & INSTRUCTION_CMDCODE_MASK) == WRITE_CMDCODE)
    {
        // insert data field
        for (scratch = 0; scratch < rtrans->dataLength; scratch++)
        {
            rmapOutBuffer[byteCount++] = rtrans->writeData[scratch];
        }

        scratch = (byteCount - crcStartlength);
        rmapOutBuffer[byteCount] = CalculateRmapCrc(&rmapOutBuffer[crcStartlength], scratch);

    }
    else if ((rtrans->instructionField & INSTRUCTION_CMDCODE_MASK) == RMW_CMDCODE)
    {
        rmapOutBuffer[byteCount++] = (uint8_t)(SHIFT_24BITS << rtrans->readModWriteData);
        rmapOutBuffer[byteCount++] = (uint8_t)(SHIFT_16BITS << rtrans->readModWriteData);
        rmapOutBuffer[byteCount++] = (uint8_t)(BYTE_SHIFT << rtrans->readModWriteData);
        rmapOutBuffer[byteCount++] = (uint8_t)rtrans->readModWriteData;

        rmapOutBuffer[byteCount++] = (uint8_t)(SHIFT_24BITS << rtrans->readModWriteMask);
        rmapOutBuffer[byteCount++] = (uint8_t)(SHIFT_16BITS << rtrans->readModWriteMask);
        rmapOutBuffer[byteCount++] = (uint8_t)(BYTE_SHIFT << rtrans->readModWriteMask);
        rmapOutBuffer[byteCount++] = (uint8_t)rtrans->readModWriteMask;

        scratch = (byteCount - crcStartlength);
        rmapOutBuffer[byteCount] = CalculateRmapCrc(&rmapOutBuffer[crcStartlength], scratch);

    }


    return(0);
}

/**@brief Initialize the SpaceWire module
 *
 */
bool spaceWireInit(void)
{
    bool error = false;
  
    /* Enable clock */
    VOR_SYSCONFIG->PERIPHERAL_CLK_ENABLE |= SYSCONFIG_PERIPHERAL_CLK_ENABLE_SPW_Msk;

    /*reset the SpaceWire peripheral*/
    VOR_SYSCONFIG->PERIPHERAL_RESET &= ~SYSCONFIG_PERIPHERAL_RESET_SPW_Msk;
    __NOP();
    __NOP();
    VOR_SYSCONFIG->PERIPHERAL_RESET |= SYSCONFIG_PERIPHERAL_RESET_SPW_Msk;
    __NOP();
    __NOP();

    /* Enable spacewire pads */
    VOR_SYSCONFIG->SPW_M4_CTRL |= SYSCONFIG_SPW_M4_CTRL_SPW_PAD_EN_Msk;
    __NOP();
    __NOP();

    VOR_SPW->CTRL |= SPW_CTRL_RS_Msk;           ///< Reset the spacewire control register
    while (VOR_SPW->CTRL & SPW_CTRL_RS_Msk) {};  ///< wait for the reset to complete

    ///< set up the clock divisor start for 10Mhz
    VOR_SPW->CLKDIV = (0x00000700);

    SPW_STS_RESET;  ///< clear Status/Interrupt-source register

    VOR_SPW->DKEY = APP_DESTINATION_KEY;

    VOR_SPW->CTRL |= SPW_CTRL_IE_Msk |  ///< enable interrupts for Tick in and Link errors.
                     SPW_CTRL_LI_Msk |  // link error irq
                     SPW_CTRL_TQ_Msk;

    VOR_SPW->CTRL |= SPW_CTRL_RD_Msk;       // RMAP buffer disable,

    //enable time code transmit
//    VOR_SPW->CTRL |= SPW_CTRL_TT_Msk;
//    VOR_SPW->CTRL |= SPW_CTRL_TR_Msk;

    VOR_SPW->CTRL |= SPW_CTRL_LS_Msk;   ///< Link Start

    ///< set the maximum receive length to 256 bytes
    VOR_SPW->DMAMAXLEN0 = 256;

    ///< set pointer to receive descriptor table
    VOR_SPW->DMARXDESC0 = (uint32_t)&dmaRxDescTable[0];

    ///< enable first receive descriptor
    rxdesc->flagsPacketLen = (RXDESC_INITENABLE | RXDESC_CTRL_EN_Msk);
    rxdesc->packetAddr = (uint32_t *)&rxBigEndianData[0];

    ///< set pointer to transmit descriptor's table
    VOR_SPW->DMATXDESC0 = (uint32_t)&dmaTxDescTable[0];
    VOR_SPW->DMAADDR0 = VOR_SPW->DEFADDR;

    //DMA CTRL
    VOR_SPW->DMACTRL0 |= SPW_DMACTRL0_RE_Msk |  // receiver enable
                         SPW_DMACTRL0_RD_Msk;   // receiver descriptors available

    VOR_SPW->DMACTRL0 |= SPW_DMACTRL0_SA_Msk;   // strip out target address

    // use receive interrupt
    VOR_SPW->DMACTRL0 |= SPW_DMACTRL0_RI_Msk;   //bit 3 Receive interrupt enable


    volatile uint32_t timeout = SPW_TIMEOUT;
    while (GET_LINK_STATE != SPW_LINK_RUN){
      if(--timeout == 0){
        error = true;
        break; 
      }
    };   // wait until link is in run state

    NVIC_ClearPendingIRQ(SpW_IRQn);  // Clear pending interrupts
    NVIC_SetPriority(SpW_IRQn, 3);
    NVIC_EnableIRQ(SpW_IRQn);        // Enable SpaceWire IRQ
    
    return error;
}

/*****************************************************************************/
/* End of file                                                               */
/*****************************************************************************/
