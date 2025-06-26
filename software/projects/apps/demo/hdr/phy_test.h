/***************************************************************************************
 * @file     phy_test.h
 * @version  V0.1
 * @date     02 November 2020
 *
 * @note
 * VORAGO Technologies
 *
 * @note
 * Copyright (c) 2013-2020 VORAGO Technologies.
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
 
#ifndef __PHY_TEST_H
#define __PHY_TEST_H


/*****************************************************************************/ 
/* Debug message selection                                                   */ 
/*****************************************************************************/

//#define USE_LOCAL_CRC
//#define PRINT_MAC_DMA_REGS
//#define DEBUG_PHY_CONFIG
//#define MAC_DEBUG_REGISTER
//#define DEBUG_UDP_MESSAGE
//#define LOOPBACK_MAC

/*****************************************************************************/ 
/* Include files                                                             */ 
/*****************************************************************************/

#include "va416xx_hal.h"

/*****************************************************************************/ 
/* Global pre-processor symbols/macros ('#define')                           */ 
/*****************************************************************************/

#define MESSAGE_BUFFER_SIZE   127     // temp for debug
#define IPV4_TYPE             0x0800  //

// MAC definitions
#define NUMBER_TX_DESC   4  //Tx Descriptors needed in the Descriptor pool/queue
#define NUMBER_RX_DESC   4  //Rx Descriptors needed in the Descriptor pool/queue

#define FRAME_HEADER_SIZE        14	// 6 byte Dest addr, 6 byte Src addr, 2 byte length/type
#define FRAME_CRC                 4	// (bytes) Ethernet CRC
#define FRAME_VLAN                4 // 
#define FRAME_PADDING             2 // dword padding, DMA Buffers must be dword aligned
#define MIN_FRAME_PAYLOAD        46 // Minimum Ethernet payload size (bytes)
#define MAX_FRAME_PAYLOAD      1500 // Maximum Ethernet payload size (bytes)

#define IP_HEADER_LENGTH      40
#define UDP_HEADER_LENGTH      8

// DMA 'able memory (DWORD aligned)
#define TX_BUFR_SIZE ((FRAME_HEADER_SIZE + FRAME_CRC + MAX_FRAME_PAYLOAD + FRAME_VLAN + FRAME_PADDING)/ 4) // (/4) converts bytes to long
#define RX_BUFR_SIZE ((FRAME_HEADER_SIZE + FRAME_CRC + MAX_FRAME_PAYLOAD + FRAME_VLAN + FRAME_PADDING)/ 4) //   

#define ETH_DMA_OPERMODE_DISABLE_RXTX_Msk   0xffffdffd    // disable DMA Transmit & Receive

// timer 16 is the Link State 
#define linkStatusTimerClkEnable    0x00010000
#define linkStateTimerControl       0x10
#define linkStateTimerValue         550000

// timer 17
#define pingServerTimerClkEnable    0x00020000
#define pingServerTimerControl      0x10
#define pingServerTimerRstValue     900000
//jpwi #define pingServerTimerRstValue     16000000


// *****************************************************************************
// MAC and DMA Initialization
// *****************************************************************************
//  DMA Bus Mode Register
#define DmaInitialBusMode         0x00100800

// DMA Interrupt Enable Register
#define DmaEnableAllInterrupts    0x0001c7ff

// DMA Operation Mode Register
#define DmaInitialOperationMode   0x02300000

// MAC Config Register
//#define MacInitialConfig          0x00404c84
#define MacInitialConfig          0x00c0880c

// MAC Frame Filter
#define MacInitialFrameFilter     0x80000001

// MAC Flow Control
#define MacInitialFlowControl     0x00000006


//// **********Copied from va416xx_hal_ethernet.h for referance******************
//// GMII (PHY) status register (KSZ8041TL)  Physical is 16bit
//#define MIISTATUS_PHY_LINK      0x0004    // Link status 1=up
//// PHY Control register  (KSZ8041TL)  Physical is 16bit
//#define  MIIADDR_PHY_DUPLEX       0x0100    // Duplex mode
//#define  MIIADDR_PHY_AN           0x1000    // auto-negotiation
//#define  MIIADDR_PHY_SPEED        0x2000    // phy speed
//#define  MIIADDR_PHY_LOOPBACK     0x4000    // PHY Loopback

// ****************************************************************************
// dmaDescCtrlFlags definitions
#define phyLinkState  MIISTATUS_PHY_LINK     //  [2] 0x0004 PHY Link State, in Status Reg.
#define phyDuplex     MIIADDR_PHY_DUPLEX     //  [8] 0x0100 PHY Duplex mode
#define phyAutoNeg    MIIADDR_PHY_AN         // [12] 0x1000 PHY Auto-Negotiation state in Control Reg.
#define phyLinkSpeed  MIIADDR_PHY_SPEED      // [13] 0x2000 PHY Speed in Control Reg.
#define phyLoopBack   MIIADDR_PHY_LOOPBACK   // [14] 0x4000 PHY Loopback State in Control Reg.


// the following bits are set in the ISR and cleared in the handler
#define txInProcess     0x00008000  // [15] Transmit in process
#define txComplete      0x00010000  // [16] TX complete Normal Interrupt
#define txNoBuffer      0x00020000  // [17] TX buffer unavailable, Normal Interrupt
#define rxComplete      0x00040000  // [18] RX Complete, Normal Interrupt
#define earlyReceive    0x00080000  // [19] Early Receive, Normal Interrupt
#define txProcStopped   0x00100000  // [20] TX process stopped, Abnormal Interrupt
#define txJabberTO      0x00200000  // [21] TX Jabber Time Out, Abnormal Interrupt
#define rxFifoOver      0x00400000  // [22] RX FIFO Over flow, Abnormal Interrupt
#define txUnderFlow     0x00800000  // [23] TX Under Flow, Abnormal Interrupt
#define rxNoBuffer      0x01000000  // [24] RX Buffer Unavailable, Abnormal Interrupt
#define rxProcStopped   0x02000000  // [25] RX Process Stopped, Abnormal Interrupt
#define rxWatchTO       0x04000000  // [26] RX Watchdog Time Out, Abnormal Interrupt
#define earlyTransmit   0x08000000  // [27] Early Transmit, Abnormal Interrupt
#define fatalBusErr     0x10000000  // [28] Fatal Bus Error, Abnormal Interrupt
#define phyIntrp        0x20000000  // [29] PHY interupt Occured 
#define abnormalInterp  0x40000000  // [30] Abnormal Interrupt group
#define normalInterp    0x80000000  // [31] Normal Interrupt group

// End dmaDescCtrlFlags  ******************************************************



/**********************************************************
 * DMA Engine descriptors
 **********************************************************
  VOR_ETH->DMA_RXDESC_LISTADDR is the pointer to the first Rx Descriptors. The 
  Descriptor format in Little endian with a 32 bit Data bus as shown below. 
  
  VOR_ETH->DMA_TXDESC_LISTADDR is the pointer to the first Tx Descriptors. The
  Descriptor format in Little endian with a 32 bit Data bus as shown below.
               --------------------------------------------------------------------
  RDES0/TDES0  |OWN[31]| Status                                                    |
               --------------------------------------------------------------------
  RDES1/TDES1  | Control Bits | Byte Count Buffer 2 | Byte Count Buffer 1          |
               --------------------------------------------------------------------
  RDES2/TDES2  |  Buffer 1 Address                                                 |
               --------------------------------------------------------------------
  RDES3/TDES3  |  Buffer 2 Address / Next Descriptor Address                       |
               --------------------------------------------------------------------
*/
// STATUS word of DMA descriptor 
#define DescOwnByDma        0x80000000  // [31] (OWN)Descriptor is owned by DMA engine
#define DescDAFilterFail    0x40000000  // [30] (AFM)Rx - DA Filter Fail for the rx frame
#define DescFrameLengthMsk  0x3FFF0000  // [29:16] Rx (FL) Descriptor Frame Length               
#define DescFrameLengthPos  16          //
#define DescError           0x00008000  // [15] (ES)Error Summary - OR of following bits:
                                        //  DE || OE || IPC || LC || RWT || RE || CE 
#define DescRxTruncated     0x00004000  // [14] (DE)Rx - no more descriptors for Rx
#define DescSAFilterFail    0x00002000  // [13] (SAF)Rx - SA Filter Fail for the received frame
#define DescRxLengthError	  0x00001000  // [12] (LE)Rx - frame size not matching with len field
#define DescRxDamaged       0x00000800  // [11] (OE)Rx - frame was damaged due to buffer overflow 
#define DescRxVLANTag       0x00000400  // [10] (VLAN)Rx - received frame is a VLAN frame
#define DescRxFirst         0x00000200  //  [9] (FS)Rx - first descriptor of the frame
#define DescRxLast          0x00000100  //  [8] (LS)Rx - last descriptor of the frame
#define DescRxLongFrame     0x00000080  //  [7] (Giant Frame)Rx - frame is longer than 1518/1522
#define DescRxCollision     0x00000040  //  [6] (LC)Rx - late collision occurred during reception
#define DescRxFrameEther    0x00000020  //  [5] (FT)Rx - Frame type - Ethernet, otherwise 802.3
#define DescRxWatchdog      0x00000010  //  [4] (RWT)Rx - watchdog timer expired during reception
#define DescRxMiiError      0x00000008  //  [3] (RE)Rx - error reported by MII interface 
#define DescRxDribbling     0x00000004  //  [2] (DE)Rx - frame contains non int multiple of 8 bits
#define DescRxCrc           0x00000002  //  [1] (CE)Rx - CRC error
#define DescRxMacMatch      0x00000001  //  [0] (RX MAC Address) Rx mac address reg(1 to 15)match

//#define DescRxChkBit0   0x00000001  // ()  Rx - Rx Payload Checksum Error                  0
//#define DescRxChkBit7   0x00000080  // (IPC CS ERROR)Rx - Ipv4 header checksum error      7
//#define DescRxChkBit5		0x00000020  // (FT)Rx - Frame type - Ethernet, otherwise 802.3    5
  
#define DescTxIpv4ChkError    0x00010000  // (IHE) Tx Ip header error                           16
#define DescTxTimeout         0x00004000  // (JT)Tx - Transmit jabber timeout                   14
#define DescTxFrameFlushed    0x00002000  // (FF)Tx - DMA/MTL flushed the frame due to SW flush 13
#define DescTxPayChkError     0x00001000  // (PCE) Tx Payload checksum Error                    12
#define DescTxLostCarrier     0x00000800  // (LC)Tx - carrier lost during tramsmission          11
#define DescTxNoCarrier       0x00000400  // (NC)Tx - no carrier signal from the tranceiver     10
#define DescTxLateCollision   0x00000200  // (LC)Tx - transmission aborted due to collision      9
#define DescTxExcCollisions   0x00000100  // (EC)Tx - transmission aborted after 16 collisions   8
#define DescTxVLANFrame       0x00000080  // (VF)Tx - VLAN-type frame                            7
 
#define DescTxCollMask        0x00000078  // (CC)Tx - Collision count                           6:3
#define DescTxCollShift       3
  
#define DescTxExcDeferral     0x00000004  // (ED)Tx - excessive deferral                          2
#define DescTxUnderflow       0x00000002  // (UF)Tx - late data arrival from the memory           1
#define DescTxDeferred        0x00000001  // (DB)Tx - frame transmision deferred                  0
  
// ****************************************************************************
//	   This explains the RDES1/TDES1 bits layout
//			        ------------------------------------------------------------
//	RDES1/TDES1 | Control Bits | Byte Count Buffer 2 | Byte Count Buffer 1 |
//			        ------------------------------------------------------------
#define DescTxIntEnable       0x80000000  // (IC)Tx - interrupt on completion             [31]
#define DescTxLast            0x40000000  // (LS)Tx - Last segment of the frame           [30]
#define DescTxFirst           0x20000000  // (FS)Tx - First segment of the frame          [29]
#define DescTxCisMask         0x18000000  // Tx checksum offloading control mask          [28:27]
#define DescTxDisableCrc      0x04000000  // (DC)Tx - CRC disabled (first segment only)   [26]
#define TxDescEndOfRing       0x02000000  // (TER)End of descriptors ring                 [25]
#define TxDescChain           0x01000000  // (TCH)Second buffer address is chain address  [24]
#define DescTxDisablePadd     0x00800000  // (DP)disable padding                          [23]
#define DescTxTimeStampEnable 0x00400000  // (TTSE) Transmit Timestamp Enable             [22]

#define RxDisIntCompl         0x80000000  // (Disable Rx int on completion) 			31
#define RxDescEndOfRing       0x02000000  // (TER)End of descriptors ring
#define RxDescChain           0x01000000  // (TCH)Second buffer address is chain address         24
  
#define DescSize2Mask         0x003FF800  // (TBS2) Buffer 2 size   [21:11]
#define DescSize2Shift        11
#define DescSize1Mask         0x000007FF  // (TBS1) Buffer 1 size   [10:0]
#define DescSize1Shift        0

#define DescTxCisBypass       0x00000000  // Checksum bypass
#define DescTxCisIpv4HdrCs	  0x08000000  // IPv4 header checksum
#define DescTxCisTcpOnlyCs    0x10000000  // TCP/UDP/ICMP checksum. Pseudo header checksum is assumed to be present
#define DescTxCisTcpPseudoCs  0x18000000  // TCP/UDP/ICMP checksum fully in hardware including pseudo header


/*****************************************************************************/
/* Global type definitions ('typedef')                                       */ 
/*****************************************************************************/

typedef struct _DmaDescStruct    
{                               
  uint32_t   status;  // Status
  uint32_t   length;  // Control bits, Buffer 1 and Buffer 2 length
  uint32_t   buffer1; // Network Buffer 1 pointer (Dma-able)
  uint32_t   buffer2; // Network Buffer 2 pointer or next descriptor pointer (Dma-able)in chain structure
} stc_DmaDesc_t;


typedef struct _ethDMAcontrol
{
  uint32_t dmaDescCtrlFlags;        // note definitions above
  stc_DmaDesc_t txDmaDesc[NUMBER_TX_DESC];
  stc_DmaDesc_t rxDmaDesc[NUMBER_RX_DESC];
  uint32_t rxNext;                    // index to next available RX descriptor
  uint32_t txNext;
}stc_DmaDescCtrlReg_t;

/*****************************************************************************/
/* Global variable declarations ('extern', definition in C source)           */
/*****************************************************************************/

extern uint16_t phy_regs[16];

/*****************************************************************************/ 
/* Global function prototypes ('extern', definition in C source)             */ 
/*****************************************************************************/

extern int phytest_main(void);

/*****************************************************************************/ 
/* End of file                                                               */ 
/*****************************************************************************/
#endif /* __PHY_TEST_H */
