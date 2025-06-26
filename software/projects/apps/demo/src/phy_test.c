/***************************************************************************************
 * @file     phy_test.c
 * @version  V0.1
 * @date     05 Novmeber 2020
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


#include "board.h"

#include "va416xx_hal_ioconfig.h"
#include "va416xx_hal_ethernet.h"

#include "phy_test.h"
#include "phytest_cfg.h"

#ifdef INC_PHY_TEST

#ifdef __clang__
#define __UNUSED 
#elif defined ( __GNUC__ )
#define __UNUSED __unused
#else
#define __UNUSED
#endif

/*****************************************************************************/ 
/* Local type definitions ('typedef')                                        */ 
/*****************************************************************************/

  typedef struct _udp_header
  {
    __PACKED uint32_t src_port  : 16;
    __PACKED uint32_t dest_port : 16;
    __PACKED uint32_t length    : 16;
    __PACKED uint32_t chksum    : 16;
  }stc_udpheader_t;

  typedef struct _ip_header
  {
    __PACKED uint32_t version       :  4;
    __PACKED uint32_t headlen       :  4;
    __PACKED uint32_t diff_services :  8;
    __PACKED uint32_t total_length  : 16;
    __PACKED uint32_t ident_frag    : 16;
    __PACKED uint32_t flags         :  3;
    __PACKED uint32_t frag_offset   : 13;
    __PACKED uint32_t timetolive    :  8;
    __PACKED uint32_t protocol      :  8;
    __PACKED uint32_t header_chksum : 16;
  }stc_ipheader_t;
  
  typedef struct _ipv4_datagram
  {
    stc_ipheader_t    iphead;
    uint32_t          srcaddr;
    uint32_t          destaddr;
    uint32_t          options;
    stc_udpheader_t   udphead;
  }stc_ipv4_header_t;

  // typedef struct _ethframe
  // {
  //   __packed uint8_t eth_frame_dest[6];
  //   __packed uint8_t eth_frame_src[6];
  //   __packed uint8_t eth_type[2];
  //   __packed uint8_t eth_data[MAX_FRAME_PAYLOAD];
  //   uint32_t eth_fcs;
  // }stc_eth_frame_header_t;

    typedef struct _ethframe
  {
    uint8_t eth_frame_dest[6];
    uint8_t eth_frame_src[6];
    uint8_t eth_type[2];
    uint8_t eth_data[MAX_FRAME_PAYLOAD];
    uint32_t eth_fcs;
  }stc_eth_frame_header_t;

  
/*****************************************************************************/ 
/* Local variable definitions                                                */ 
/*****************************************************************************/

  const char testMsg[] = "Hello from VORAGO VA416x0"; 
  //const char bannerPOR[] =  "\033[0H\033[2J\r\n\r\n************************"" Ethernet STARTUP ""****************************\r\n"; 
  const char rxFifoReadController[4][32] = { "Idle", "Reading frame data", "Reading frame status", "Flushing frame data and status"};
  const char rxFifoFillLevel[4][40] = { "Empty", "below flow-control deactivate threshold", "above flow-control activate threshold", "Full"};
  const char macTxFrameCtrlr[4][40] = { "Idle", "Waiting prev frame, IFG, backoff period", "Tx Pause frame (full-duplex)", "Xferring input frame for tx"};
  const char txFifoReadController[4][50] = { "Idle", "READ state (Xferring data->MAC tx)", "Waiting TxStatus from MAC tx", "Writing received TxStatus or flushing Tx FIFO"};

  //char message[MESSAGE_BUFFER_SIZE + 1];    // debug message's on uart0
  
  uint32_t checkPhyLink;
  uint32_t pingTheUDPServer;
  uint32_t prevMacDebug;
  uint32_t txUdpMessageNumber;
  uint32_t rxUdpMessageNumber;
  uint32_t txSuspendRestarts;
  uint32_t sysTicker;
  uint32_t msgNumber;
  
  uint32_t macDebugSnapShot;
  uint32_t timeTestLoop;
//****************************************************
  uint8_t bCastMacAddr[6] = {255,255,255,255,255,255};
  uint8_t bCastIP[4] = {255,255,255,255};
//****************************************************
// Destination mac address
  uint8_t destMacAddr[6] = {0x54,0xbf,0x64,0x17,0x36,0x41};
  uint8_t destIpAddr [4] = {10,8,7,100};
//*********************************************************
// Source mac address
  uint8_t macAddr0[6] = {0xbf, 0xa5, 0x24, 0x3c, 0x61, 0x00}; // 00-61-3c-24-a5-bf (lsb first)
		//	arp -s 10.8.7.25 00-61-3c-24-a5-bf <- for Win10 to understand
	uint8_t srcIpAddr[4] = {10,8,7,25};   // our IP address
//*********************************************************


  uint8_t txBufr[MAX_FRAME_PAYLOAD];
  uint8_t receiveBuffr[MAX_FRAME_PAYLOAD];
  
  uint16_t phyPrevStatusReg;

  // DMA buffers
  uint32_t txBuffer[NUMBER_TX_DESC][TX_BUFR_SIZE];
  uint32_t *currTxBufr;
  uint32_t txCurrIndex;
  
  uint32_t rxBuffer[NUMBER_RX_DESC][RX_BUFR_SIZE];
  uint32_t *currRxBufr;
  uint32_t rxCurrIndex;
  
  stc_DmaDescCtrlReg_t dmaCtrl;
	
/// A  time reference incremented every 1ms since systick start
//static volatile uint32_t u32SystemTime = 0;
/// A  time reference incremented every 1ms resetable by the application programmer
//static volatile uint32_t HAL_time_ms = 0;

///**************************************************************************************************
// *  PORTA[15:8] AND PORTB[10:0] used for the Ethernet Peripheral as FUNselect 1
// */  
//const stc_iocfg_pin_cfg_t ethernetPortconfig[] = 
//{
//{VOR_PORTA, 0,en_iocfg_dir__input, {{.fltclk=0,.invinp=0,.iewo=0,.opendrn=0,.invout=0,.plevel=0,.pen=0,.pwoa=0,.funsel=0,.iodis=0}}},
//{VOR_PORTA, 1,en_iocfg_dir__input, {{.fltclk=0,.invinp=0,.iewo=0,.opendrn=0,.invout=0,.plevel=0,.pen=0,.pwoa=0,.funsel=0,.iodis=0}}},
//{VOR_PORTA, 2,en_iocfg_dir_dncare, {{.fltclk=0,.invinp=0,.iewo=0,.opendrn=0,.invout=0,.plevel=0,.pen=0,.pwoa=0,.funsel=3,.iodis=0}}},
//{VOR_PORTA, 3,en_iocfg_dir_dncare, {{.fltclk=0,.invinp=0,.iewo=0,.opendrn=0,.invout=0,.plevel=0,.pen=0,.pwoa=0,.funsel=3,.iodis=0}}},
//{VOR_PORTA, 4,en_iocfg_dir__input, {{.fltclk=0,.invinp=0,.iewo=0,.opendrn=0,.invout=0,.plevel=0,.pen=0,.pwoa=0,.funsel=0,.iodis=0}}},
//{VOR_PORTA, 5,en_iocfg_dir__input, {{.fltclk=0,.invinp=0,.iewo=0,.opendrn=0,.invout=0,.plevel=0,.pen=0,.pwoa=0,.funsel=0,.iodis=0}}},
//{VOR_PORTA, 6,en_iocfg_dir__input, {{.fltclk=0,.invinp=0,.iewo=0,.opendrn=0,.invout=0,.plevel=0,.pen=0,.pwoa=0,.funsel=0,.iodis=0}}},
//{VOR_PORTA, 7,en_iocfg_dir__input, {{.fltclk=0,.invinp=0,.iewo=0,.opendrn=0,.invout=0,.plevel=0,.pen=0,.pwoa=0,.funsel=0,.iodis=0}}},  
//{VOR_PORTA, 8,en_iocfg_dir_dncare, {{.fltclk=0,.invinp=0,.iewo=0,.opendrn=0,.invout=0,.plevel=0,.pen=0,.pwoa=0,.funsel=1,.iodis=0}}},
//{VOR_PORTA, 9,en_iocfg_dir_dncare, {{.fltclk=0,.invinp=0,.iewo=0,.opendrn=0,.invout=0,.plevel=0,.pen=0,.pwoa=0,.funsel=1,.iodis=0}}},
//{VOR_PORTA,10,en_iocfg_dir_dncare, {{.fltclk=0,.invinp=0,.iewo=0,.opendrn=0,.invout=0,.plevel=0,.pen=0,.pwoa=0,.funsel=1,.iodis=0}}},
//{VOR_PORTA,11,en_iocfg_dir_dncare, {{.fltclk=0,.invinp=0,.iewo=0,.opendrn=0,.invout=0,.plevel=0,.pen=0,.pwoa=0,.funsel=1,.iodis=0}}},
//{VOR_PORTA,12,en_iocfg_dir_dncare, {{.fltclk=0,.invinp=0,.iewo=0,.opendrn=0,.invout=0,.plevel=0,.pen=0,.pwoa=0,.funsel=1,.iodis=0}}},
//{VOR_PORTA,13,en_iocfg_dir_dncare, {{.fltclk=0,.invinp=0,.iewo=0,.opendrn=0,.invout=0,.plevel=0,.pen=0,.pwoa=0,.funsel=1,.iodis=0}}},
//{VOR_PORTA,14,en_iocfg_dir_dncare, {{.fltclk=0,.invinp=0,.iewo=0,.opendrn=0,.invout=0,.plevel=0,.pen=0,.pwoa=0,.funsel=1,.iodis=0}}},
//{VOR_PORTA,15,en_iocfg_dir_dncare, {{.fltclk=0,.invinp=0,.iewo=0,.opendrn=0,.invout=0,.plevel=0,.pen=0,.pwoa=0,.funsel=1,.iodis=0}}},
//{VOR_PORTB, 0,en_iocfg_dir_dncare, {{.fltclk=0,.invinp=0,.iewo=0,.opendrn=0,.invout=0,.plevel=0,.pen=0,.pwoa=0,.funsel=1,.iodis=0}}},
//{VOR_PORTB, 1,en_iocfg_dir_dncare, {{.fltclk=0,.invinp=0,.iewo=0,.opendrn=0,.invout=0,.plevel=0,.pen=0,.pwoa=0,.funsel=1,.iodis=0}}},
//{VOR_PORTB, 2,en_iocfg_dir_dncare, {{.fltclk=0,.invinp=0,.iewo=0,.opendrn=0,.invout=0,.plevel=0,.pen=0,.pwoa=0,.funsel=1,.iodis=0}}},
//{VOR_PORTB, 3,en_iocfg_dir_dncare, {{.fltclk=0,.invinp=0,.iewo=0,.opendrn=0,.invout=0,.plevel=0,.pen=0,.pwoa=0,.funsel=1,.iodis=0}}},
//{VOR_PORTB, 4,en_iocfg_dir_dncare, {{.fltclk=0,.invinp=0,.iewo=0,.opendrn=0,.invout=0,.plevel=0,.pen=0,.pwoa=0,.funsel=1,.iodis=0}}},
//{VOR_PORTB, 5,en_iocfg_dir_dncare, {{.fltclk=0,.invinp=0,.iewo=0,.opendrn=0,.invout=0,.plevel=0,.pen=0,.pwoa=0,.funsel=1,.iodis=0}}},
//{VOR_PORTB, 6,en_iocfg_dir_dncare, {{.fltclk=0,.invinp=0,.iewo=0,.opendrn=0,.invout=0,.plevel=0,.pen=0,.pwoa=0,.funsel=1,.iodis=0}}},
//{VOR_PORTB, 7,en_iocfg_dir_dncare, {{.fltclk=0,.invinp=0,.iewo=0,.opendrn=0,.invout=0,.plevel=0,.pen=0,.pwoa=0,.funsel=1,.iodis=0}}},
//{VOR_PORTB, 8,en_iocfg_dir_dncare, {{.fltclk=0,.invinp=0,.iewo=0,.opendrn=0,.invout=0,.plevel=0,.pen=0,.pwoa=0,.funsel=1,.iodis=0}}},
//{VOR_PORTB, 9,en_iocfg_dir_dncare, {{.fltclk=0,.invinp=0,.iewo=0,.opendrn=0,.invout=0,.plevel=0,.pen=0,.pwoa=0,.funsel=1,.iodis=0}}},
//{VOR_PORTB,10,en_iocfg_dir_dncare, {{.fltclk=0,.invinp=0,.iewo=0,.opendrn=0,.invout=0,.plevel=0,.pen=0,.pwoa=0,.funsel=1,.iodis=0}}},
//{VOR_PORTB,11,en_iocfg_dir__input, {{.fltclk=0,.invinp=0,.iewo=0,.opendrn=0,.invout=0,.plevel=0,.pen=0,.pwoa=0,.funsel=0,.iodis=0}}},
//{VOR_PORTB,12,en_iocfg_dir__input, {{.fltclk=0,.invinp=0,.iewo=0,.opendrn=0,.invout=0,.plevel=0,.pen=0,.pwoa=0,.funsel=0,.iodis=0}}},
//{VOR_PORTB,13,en_iocfg_dir__input, {{.fltclk=0,.invinp=0,.iewo=0,.opendrn=0,.invout=0,.plevel=0,.pen=0,.pwoa=0,.funsel=0,.iodis=0}}},
//{VOR_PORTB,14,en_iocfg_dir_dncare, {{.fltclk=0,.invinp=0,.iewo=0,.opendrn=0,.invout=0,.plevel=0,.pen=0,.pwoa=0,.funsel=3,.iodis=0}}},
//{VOR_PORTB,15,en_iocfg_dir_dncare, {{.fltclk=0,.invinp=0,.iewo=0,.opendrn=0,.invout=0,.plevel=0,.pen=0,.pwoa=0,.funsel=3,.iodis=0}}},
////{VOR_PORTG, 0,en_iocfg_dir__input, {{.fltclk=0,.invinp=0,.iewo=0,.opendrn=0,.invout=0,.plevel=0,.pen=0,.pwoa=0,.funsel=0,.iodis=0}}},
////{VOR_PORTG, 1,en_iocfg_dir__input, {{.fltclk=0,.invinp=0,.iewo=0,.opendrn=0,.invout=0,.plevel=0,.pen=0,.pwoa=0,.funsel=0,.iodis=0}}},
//// jpwi - Should not do this but both PA3,PA4 and PG0,PG1 are connect to UART0
//  {VOR_PORTG, 0,en_iocfg_dir_dncare, {{.fltclk=0,.invinp=0,.iewo=0,.opendrn=0,.invout=0,.plevel=0,.pen=0,.pwoa=0,.funsel=1,.iodis=0}}},
//  {VOR_PORTG, 1,en_iocfg_dir__input, {{.fltclk=0,.invinp=0,.iewo=0,.opendrn=0,.invout=0,.plevel=0,.pen=0,.pwoa=0,.funsel=1,.iodis=0}}},
//{VOR_PORTG, 2,en_iocfg_dir__input, {{.fltclk=0,.invinp=0,.iewo=0,.opendrn=0,.invout=0,.plevel=0,.pen=0,.pwoa=0,.funsel=0,.iodis=0}}},
//{VOR_PORTG, 3,en_iocfg_dir__input, {{.fltclk=0,.invinp=0,.iewo=0,.opendrn=0,.invout=0,.plevel=0,.pen=0,.pwoa=0,.funsel=0,.iodis=0}}},
//{VOR_PORTG, 4,en_iocfg_dir__input, {{.fltclk=0,.invinp=0,.iewo=0,.opendrn=0,.invout=0,.plevel=0,.pen=0,.pwoa=0,.funsel=0,.iodis=0}}},
//{VOR_PORTG, 5,en_iocfg_dir_output, {{.fltclk=0,.invinp=0,.iewo=0,.opendrn=0,.invout=0,.plevel=0,.pen=0,.pwoa=0,.funsel=0,.iodis=0}}},
//{VOR_PORTG, 6,en_iocfg_dir__input, {{.fltclk=0,.invinp=0,.iewo=0,.opendrn=0,.invout=0,.plevel=0,.pen=0,.pwoa=0,.funsel=0,.iodis=0}}},
//{VOR_PORTG, 7,en_iocfg_dir__input, {{.fltclk=0,.invinp=0,.iewo=0,.opendrn=0,.invout=0,.plevel=0,.pen=0,.pwoa=0,.funsel=0,.iodis=0}}},

//IOCFG_PINCFG_END
//};
  
/*****************************************************************************/ 
/* Local function prototypes ('static')                                      */ 
/*****************************************************************************/
  
void TIM16_IRQHandler(void);
void TIM17_IRQHandler(void);
void SysTick_Handler(void);
void Delay (uint32_t dlyTicks);
void PA0_IRQHandler(void);
void Ethernet_IRQHandler(void);
//int32_t _puts_uart0(const char *);
uint32_t crc32(uint32_t, uint8_t *);
void printMacAddress(void);
void initLinkUpDownTimer(void);
void initPingServerTimer(void);
void initRxQueueChain(void);
void initTxQueueChain(void);
hal_status_t initialEtherCore(void);
hal_status_t handlePhyLinkState(void);
hal_status_t savePHYstate(void);
void dmaQueueHandler(void);
void parseReceivedMessage(void);
void _sendUDPMessage(void);
bool dumpPHY_Registers(void);
void printLinkAttributes(void);
void restartAutoNegotiation(void);
void msTimeStamp(void);


/*****************************************************************************/ 
/* Function implementation - global ('extern') and local ('static')          */ 
/*****************************************************************************/


/*******************************************************************************
 ** @brief  One second interrupt, used to check Ethernet PHY link state.
 ** @param
 ** @return 
 ******************************************************************************/ 
void TIM16_IRQHandler(void)
{
  // time to check the PHY Link State, cleared by the handler
  checkPhyLink = 1;
}

/*******************************************************************************
 **
 ** @brief interrupt
 **
 ** @param
 **
 ** @return
 **
 ******************************************************************************/ 
void TIM17_IRQHandler(void)
{
  sysTicker++;
  pingTheUDPServer = 1;
}


/*******************************************************************************
 ** @brief Interrupt service routine for the PHY
 ** @param none
 ** @return none
 ** @note
 ******************************************************************************/
void PA0_IRQHandler(void)
{
  dmaCtrl.dmaDescCtrlFlags |= phyIntrp; 
}

 /******************************************************************************
 ** This function is triggered by the SysTick interrupt.
 **
 *****************************************************************************/
//void SysTick_Handler(void) // 
//{
//  u32SystemTime++;
//  HAL_time_ms++;
//} // SysTick_Handler

/* ---------------------------------------------------------
 * Delay: delays a number of Systicks
   ---------------------------------------------------------*/
void Delay (uint32_t dlyTicks) {
  uint64_t curTicks;

  curTicks = HAL_time_ms;
  while ((HAL_time_ms - curTicks) < dlyTicks) { __NOP(); }
}

/* ---------------------------------------------------------
 * Restart Auto-negotiation process
   ---------------------------------------------------------*/
void restartAutoNegotiation(void){
  // force start auto-neg
	uint16_t temp;
	
	if ( (HAL_ReadPhyReg(PHY_CONTROL_REG, &temp) == 0)  )   // read phy control resister
  {
    if ( (HAL_ReadPhyReg(PHY_CONTROL_REG, &temp) == 0)  )   // read phy control resister
    {
      temp |= 0x0200;   // restart AN
      
      HAL_WritePhyReg(PHY_CONTROL_REG, temp);
    }
	}
}

/* ---------------------------------------------------------
 * Function is to read out the PHY registers for debug
   ---------------------------------------------------------*/
bool dumpPHY_Registers(void){

  uint16_t phyStatusReg;

  // table header
	printf( "\r\n       ");  
	////_puts_uart0(message);

  for (uint8_t i = 0; i < 8; i++) {
    printf( "%2x    ", i);
		////_puts_uart0(message);
  }
		for (uint16_t j = 0; j < 32; j++)
		{
			if (((j) % 8) == 0){
			printf( "\r\n0x%02X: ", (j));
			////_puts_uart0(message);
			}
			if ((HAL_ReadPhyReg(j<<6, &phyStatusReg) == hal_status_ok ))
			{
				if ((HAL_ReadPhyReg(j<<6, &phyStatusReg) == hal_status_ok ))
				{
          printf( "%04X  ",phyStatusReg);
          ////_puts_uart0(message);
				}
			}
	}
	printf("\r\n\n");
  ////_puts_uart0(message);
return true;
}

/*******************************************************************************
 ** @brief Interrupt Service Routine for the Ethernet
 ** @param none
 ** @return none
 ** @note
 ******************************************************************************/ 
void Ethernet_IRQHandler(void)
{
  // check for Normal interupt
  if (VOR_ETH->DMA_STATUS & ETH_DMA_STATUS_NIS_Msk)
  {
    /**************************************************************************
     * Transmit complete interrupt
     *
     */
    if (VOR_ETH->DMA_STATUS &  ETH_DMA_STATUS_TI_Msk)    // [0] Tx Completed
    {
      VOR_ETH->DMA_OPER_MODE &= ~(ETH_DMA_OPER_MODE_ST_Msk); 
      dmaCtrl.dmaDescCtrlFlags |= txComplete;
      dmaCtrl.dmaDescCtrlFlags &= ~(txInProcess);  // flag tx complete
      VOR_ETH->DMA_STATUS |= ETH_DMA_STATUS_TI_Msk;     // (R_SS_WC)  Read, Write To Clear
    }
    
    /**************************************************************************
     * Transmit buffer unavailable
     *
     */
    if (VOR_ETH->DMA_STATUS & ETH_DMA_STATUS_TU_Msk)  // [2] TX No Buffer
    {
      dmaCtrl.dmaDescCtrlFlags |= txNoBuffer;       // flag the handler
      VOR_ETH->DMA_STATUS |= ETH_DMA_STATUS_TU_Msk; // (R_SS_WC)  Read, Write To Clear
    }//**

    /**************************************************************************
     * Receive Complete interrupt
     *
     */    
    if (VOR_ETH->DMA_STATUS & ETH_DMA_STATUS_RI_Msk)    // [6] Receive Complete
    {
      currRxBufr = (uint32_t *) VOR_ETH->DMA_CURR_RX_BUFR_ADDR;
      rxCurrIndex = dmaCtrl.rxNext;
      
      // point to next rx descriptor
      if (dmaCtrl.rxNext < NUMBER_RX_DESC - 1)
      {
        dmaCtrl.rxNext++;   // next descriptor
      }
      else
      {
        dmaCtrl.rxNext = 0; // top of chain
      }

      dmaCtrl.rxDmaDesc[dmaCtrl.rxNext].status |= DescOwnByDma;   // set the ownership to DMA
      dmaCtrl.dmaDescCtrlFlags |= rxComplete;                     // set flag
      VOR_ETH->DMA_STATUS |= ETH_DMA_STATUS_RI_Msk;   // (R_SS_WC)  Read, Write To Clear Interrupt
    }//**

    /**************************************************************************
     *
     */
    if ( VOR_ETH->DMA_STATUS & ETH_DMA_STATUS_ERI_Msk)   // [14] Early Receive 
    {
      dmaCtrl.dmaDescCtrlFlags |= earlyReceive;
      VOR_ETH->DMA_STATUS |= ETH_DMA_STATUS_ERI_Msk;     // (R_SS_WC)  Read, Write To Clear
    }
    
    dmaCtrl.dmaDescCtrlFlags |= normalInterp;
    VOR_ETH->DMA_STATUS |= ETH_DMA_STATUS_NIS_Msk;        // (R_SS_WC)  Read, Write To Clear Normal Interrupt
    //**
  }   // **************** end normal interrupt's
  
  /**************************************************************************
   *  check for abnormal interrupts
   */
  if (VOR_ETH->DMA_STATUS & ETH_DMA_STATUS_AIS_Msk)
  {
    if (VOR_ETH->DMA_STATUS & ETH_DMA_STATUS_TPS_Msk)    // Transmit process stopped (Abnormal) [1]
    {
      dmaCtrl.dmaDescCtrlFlags |= txProcStopped;
      VOR_ETH->DMA_STATUS |=  ETH_DMA_STATUS_TPS_Msk;   // (R_SS_WC)  Read, Write To Clear
    }

    if (VOR_ETH->DMA_STATUS & ETH_DMA_STATUS_TJT_Msk)   // Transmit Jabber Timeout (Abnormal) [3]
    {
      dmaCtrl.dmaDescCtrlFlags |= txJabberTO;
      VOR_ETH->DMA_STATUS |= ETH_DMA_STATUS_TJT_Msk;   // (R_SS_WC)  Read, Write To Clear
    }
    
    if (VOR_ETH->DMA_STATUS & ETH_DMA_STATUS_OVF_Msk)  // Receive Buffer overflow (Abnormal) [4]
    {
      dmaCtrl.dmaDescCtrlFlags |= rxFifoOver;
      VOR_ETH->DMA_STATUS |= ETH_DMA_STATUS_OVF_Msk;   // (R_SS_WC)  Read, Write To Clear
    }
    
    if (VOR_ETH->DMA_STATUS & ETH_DMA_STATUS_UNF_Msk )   // Transmit underflow (Abnormal)  [5]
    {
      dmaCtrl.dmaDescCtrlFlags |= txUnderFlow;
      VOR_ETH->DMA_STATUS |= ETH_DMA_STATUS_UNF_Msk;    // (R_SS_WC)  Read, Write To Clear
    }
    
    /*************************************************************
     * Receive buffer unavailable (Abnormal)  [7]
     */
    if (VOR_ETH->DMA_STATUS & ETH_DMA_STATUS_RU_Msk )
    {
      // is the current RX descriptor owned by DMA 
      if ( (VOR_ETH->DMA_CURR_RX_DESC & DescOwnByDma) == 0)
      {
        VOR_ETH->DMA_CURR_RX_DESC |= DescOwnByDma;   // set the descriptor to DMA ownership
        VOR_ETH->DMA_RX_POLL_DEMAND = 1;             // poll descriptors to set
      }
      dmaCtrl.dmaDescCtrlFlags |= rxNoBuffer;
      VOR_ETH->DMA_STATUS |= ETH_DMA_STATUS_RU_Msk;    // (R_SS_WC)  Read, Write To Clear
    }
    
    // Receive process stopped (Abnormal)     [8]
    if (VOR_ETH->DMA_STATUS & ETH_DMA_STATUS_RPS_Msk)
    {
      dmaCtrl.dmaDescCtrlFlags |= rxProcStopped;
      VOR_ETH->DMA_STATUS |= ETH_DMA_STATUS_RPS_Msk;   // (R_SS_WC)  Read, Write To Clear
    }
    
    // Receive Watchdog Timeout (Abnormal)    [9]
    if (VOR_ETH->DMA_STATUS & ETH_DMA_STATUS_RWT_Msk)
    {
      dmaCtrl.dmaDescCtrlFlags |= rxWatchTO;
      VOR_ETH->DMA_STATUS |= ETH_DMA_STATUS_RWT_Msk;    // (R_SS_WC)  Read, Write To Clear
    }
    
    // Early transmit interrupt (Abnormal)    [10]
    if (VOR_ETH->DMA_STATUS & ETH_DMA_STATUS_ETI_Msk)
    {
      dmaCtrl.dmaDescCtrlFlags |= earlyTransmit;
      VOR_ETH->DMA_STATUS |= ETH_DMA_STATUS_ETI_Msk;   // (R_SS_WC)  Read, Write To Clear
    }
    
    // Fatal bus error (Abnormal)             [13]
    if (VOR_ETH->DMA_STATUS & ETH_DMA_STATUS_FBI_Msk)
    {
      dmaCtrl.dmaDescCtrlFlags |= fatalBusErr;
      VOR_ETH->DMA_STATUS |= ETH_DMA_STATUS_FBI_Msk;    // (R_SS_WC)  Read, Write To Clear
    }
    
    dmaCtrl.dmaDescCtrlFlags |= abnormalInterp;
    VOR_ETH->DMA_STATUS |= ETH_DMA_STATUS_AIS_Msk;    // (R_SS_WC)  Read, Write To Clear Abnormal interrupt
    
  } // end abnormal interrupts

}

/*******************************************************************************
 **
 ** @brief
 **
 ** @param
 **
 ** @return
 **
 ******************************************************************************/ 
//int32_t //_puts_uart0(const char *pMsg)
//{
//  uint32_t timeout = 100000;
//  uint32_t charCount = 0;
//  
//  while (*pMsg)
//  {
//    // Block until there is room on the FIFO to transmit a byte
//    while (( VOR_UART0->TXSTATUS & UART_TXSTATUS_WRRDY_Msk) == 0)
//    {
//      timeout--;
//      if(timeout == 0)
//      { 
//        return(-1);   // return error
//      }
//    }
//    VOR_UART0->DATA = *pMsg++;
//    charCount++;
//  }
//  return charCount;
//  return 0;
//}
#ifdef MAC_DEBUG_REGISTER
/*******************************************************************************
 ** @brief  Ouput Mac Debug register (short)
 ** @param  entry 
 ** @return result
 **
 ******************************************************************************/
void printMacDebugReg(void)
{
  printf( "\r\n ------ MAC Debug: 0x%.8lX ------\r\n", macDebugSnapShot); // prevMacDebug);
  //_puts_uart0(message);
}    
#endif
#ifdef MAC_DEBUG_VERBOSE
/*******************************************************************************
 **
 ** @brief  
 **
 ** @param  entry 
 **
 ** @return result
 **
 ******************************************************************************/
void printMacDebugVerbose(void)
{
  uint32_t temp;

  printf( "\r\n ------ MAC Debug: 0x%.8X ------\r\n", macDebugSnapShot); // prevMacDebug);
  //_puts_uart0(message);
    
  printf("[24]MTL Tx FIFO%sEmpty\r\n", \
          (macDebugSnapShot & ETH_MAC_DEBUG_TXFSTS_Msk) ? " Not " : " is " );
  //_puts_uart0(message);
    
  printf("[22]MTL Tx FIFO Write Ctrlr is %s\r\n", \
          (macDebugSnapShot & ETH_MAC_DEBUG_TWCSTS_Msk) ? "Active" : "Idle");
  //_puts_uart0(message);
    
  // MTL Tx FIFO Read Controller Status
  temp = ((macDebugSnapShot & ETH_MAC_DEBUG_TRCSTS_Msk)>>20);
  //_puts_uart0("[21:20]MTL Tx FIFO Read Controller: ");
  //_puts_uart0(&txFifoReadController[temp][0]);
  //_puts_uart0("\r\n");

  printf("[19]MAC Transmitter is %s\r\n", \
          (macDebugSnapShot & ETH_MAC_DEBUG_TXPAUSED_Msk) ? "Paused" : "Active");
  //_puts_uart0(message);
  
  // MAC Transmit Frame Controller Status
  temp = ((macDebugSnapShot & ETH_MAC_DEBUG_TFCSTS_Msk)>>17);
  //_puts_uart0("[18:17]MAC Transmit Frame Controller: ");
  //_puts_uart0(&macTxFrameCtrlr[temp][0]);
  //_puts_uart0("\r\n");
  
  printf("[16]MAC MII Tx Protocol Engine is %s\r\n", \
          (macDebugSnapShot & ETH_MAC_DEBUG_TPESTS_Msk) ? "Active" : "Idle");
  //_puts_uart0(message);
  
  // get MTL RxFIFO Fill-Level Status
  temp = ((macDebugSnapShot & ETH_MAC_DEBUG_RXFSTS_Msk)>>8);
  //_puts_uart0("[9:8]MTL RxFIFO Fill-Level: ");
  //_puts_uart0(&rxFifoFillLevel[temp][0]);
  //_puts_uart0("\r\n");
  
  // get the MTL RxFIFO Read Controller State
  temp = ((macDebugSnapShot & ETH_MAC_DEBUG_RRCSTS_Msk)>>5);
  //_puts_uart0("[6:5]MTL RxFIFO Read Controller: ");
  //_puts_uart0(&rxFifoReadController[temp][0]);
  //_puts_uart0("\r\n");

  printf("[4]MTL Rx FIFO Write Controller is %s\r\n", \
          (macDebugSnapShot & ETH_MAC_DEBUG_RWCSTS_Msk) ? "Active" : "Idle");
  //_puts_uart0(message);

  printf("[2]MAC Rx Frame Small FIFO Read Controller is %s\r\n", \
          (macDebugSnapShot & ETH_MAC_DEBUG_RFSFRCSTS_Msk) ? "Active" : "Idle");
  //_puts_uart0(message);
  
  printf("[1]MAC Rx Frame FIFO Write Controller is %s\r\n", \
          (macDebugSnapShot & ETH_MAC_DEBUG_RFSFWCSTS_Msk) ? "Active" : "Idle");
  //_puts_uart0(message);
  
  printf("[0]MAC MII Receive Protocol Engine is %s\r\n", \
          (macDebugSnapShot & ETH_MAC_DEBUG_RPESTS_Msk) ? "Active" : "Idle");
  //_puts_uart0(message);
  //_puts_uart0("\r\n");
}
#endif  //MAC_DEBUG_VERBOSE

#ifdef PRINT_MAC_DMA_REGS
/*******************************************************************************
 **
 ** @brief  
 **
 ** @param  entry 
 **
 ** @return result
 **
 ******************************************************************************/
void print_macRegisters(void)
{
  //_puts_uart0("\r\n  Initial MAC and DMA regiater settings");
  //_puts_uart0("\r\n+ MAC ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++\r\n");
  printf("\tCSR  0 -MAC Configuration:\t\t0x%.8X\r\n", VOR_ETH->MAC_CONFIG);
  //_puts_uart0(message);
  printf("\tCSR  1 -MAC Frame Filter:\t\t0x%.8X\r\n", VOR_ETH->MAC_FRAME_FLTR);
  //_puts_uart0(message);
  printf("\tCSR  6 -MAC Flow Control:\t\t0x%.8X\r\n", VOR_ETH->MAC_FLOW_CTRL);
  //_puts_uart0(message);
  printf("\tCSR  7 -MAC VLAN:\t\t\t0x%.8X\r\n", VOR_ETH->MAC_VLAN_TAG); 
  //_puts_uart0(message);
  printf("\tCSR  9 -MAC Debug:\t\t\t0x%.8X\r\n", VOR_ETH->MAC_DEBUG); 
  //_puts_uart0(message);
  printf("\tCSR 14 -MAC Interrupt Status:\t\t0x%.8X\r\n", VOR_ETH->MAC_INTR_STAT); 
  //_puts_uart0(message);
  printf("\tCSR 15 -MAC Interrupt Mask:\t\t0x%.8X\r\n", VOR_ETH->MAC_INTR_MASK);
  //_puts_uart0(message);  
  //_puts_uart0("+ DMA ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++\r\n");
  printf("\tCSR  0 -DMA Bus Mode:\t\t\t0x%.8X\r\n", VOR_ETH->DMA_BUS_MODE);
  //_puts_uart0(message);  
  printf("\tCSR  3 -DMA RxBaseAddr:\t\t\t0x%.8X\r\n", VOR_ETH->DMA_RXDESC_LISTADDR);
  //_puts_uart0(message);
  printf("\tCSR  4 -DMA Tx Desc Base Addr:\t\t0x%.8X\r\n", VOR_ETH->DMA_TXDESC_LISTADDR);
  //_puts_uart0(message);
  printf("\tCSR  5 -DMA Status:\t\t\t0x%.8X\r\n", VOR_ETH->DMA_STATUS);
  //_puts_uart0(message);
  printf("\tCSR  6 -DMA Operation Mode:\t\t0x%.8X\r\n", VOR_ETH->DMA_OPER_MODE);
  //_puts_uart0(message);
  printf("\tCSR  7 -DMA Interrupt Enable:\t\t0x%.8X\r\n", VOR_ETH->DMA_INTR_EN);
  //_puts_uart0(message);
  printf("\tCSR  8 -DMA Missed Over Frame Count:\t0x%.8X\r\n", VOR_ETH->DMA_MISSOVER_COUNT);
  //_puts_uart0(message);
  //_puts_uart0("+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++\r\n");  
}
#endif //PRINT_MAC_DMA_REGS

/*******************************************************************************
 **
 ** @brief Prints MAC address on uart0
 **
 ** @param  none
 **
 ** @return none
 **
 ******************************************************************************/
void print_MACaddress(void)
{
  uint32_t cnt;
  uint8_t addroc[6];
 
  addroc[5] = (VOR_ETH->MAC_ADDR_L & 0xff);
  addroc[4] = (VOR_ETH->MAC_ADDR_L >> 8) & 0xff;
  addroc[3] = (VOR_ETH->MAC_ADDR_L >> 16) & 0xff;
  addroc[2] = (VOR_ETH->MAC_ADDR_L >> 24) & 0xff;
  addroc[1] = (VOR_ETH->MAC_ADDR_H & 0xff);
  addroc[0] = (VOR_ETH->MAC_ADDR_H >> 8) & 0xff;

   msTimeStamp();
					  
  printf("MAC Addr: ");
  
  for (cnt = 0; cnt < 5; cnt++)
  {
    printf("%.2x:", addroc[cnt]);
    //_puts_uart0(message);
  }
  printf("%.2x\r\n", addroc[cnt]);
  //_puts_uart0(message);
 
}

#ifdef DEBUG_PHY_CONFIG
/*******************************************************************************
 **
 ** @brief Prints Physical configuration from a local structure 
 **
 ** @param none
 **
 ** @return none
 **
 ******************************************************************************/
void print_PHYconfig(void)
{
  printf("\n\r  Advertised link attributes:\r\n");  
  //_puts_uart0(message);
  printf(
          "  PHY Speed:\t\t%s\r\n", (dmaCtrl.dmaDescCtrlFlags & MIIADDR_PHY_SPEED)  ? "100Mbps" : "10Mbps");  
  //_puts_uart0(message);
  printf(
          "  PHY Auto-negotiation:\t%s\r\n", (dmaCtrl.dmaDescCtrlFlags & MIIADDR_PHY_AN) ? "Enabled" : "Disabled");  
  //_puts_uart0(message);
  printf(
          "  PHY Loopback Mode:\t%s\r\n", (dmaCtrl.dmaDescCtrlFlags & MIIADDR_PHY_LOOPBACK) ? "ON" : "OFF");  
  //_puts_uart0(message);
  printf(
          "  PHY Duplex Mode:\t%s\r\n\n", (dmaCtrl.dmaDescCtrlFlags & MIIADDR_PHY_DUPLEX) ? "Full" : "Half");  
  //_puts_uart0(message);
//  printf(
//          "  PHY Link Status:\t%s\r\n\n", (dmaCtrl.dmaDescCtrlFlags & MIISTATUS_PHY_LINK) ? "Up" : "Down");  
//  //_puts_uart0(message);
}
#endif
/*******************************************************************************
 **
 ** @brief Prints link attributes 
 **
 ** @param none
 **
 ** @return none
 **
 ******************************************************************************/
void printLinkAttributes(void)
{
  uint16_t phyRegBufr;
  uint16_t temp;

  msTimeStamp();
					  
  printf(" ->""Negotiated link attributes: ");  
  //_puts_uart0(message);
  if ((HAL_ReadPhyReg(PHY_CONTROL_TWO, &phyRegBufr) == hal_status_ok ))
  {
			do{
				HAL_ReadPhyReg(PHY_CONTROL_TWO, &phyRegBufr);
				temp = ((phyRegBufr>>2) & 0x0007);
			}while (temp == 0);

			phyRegBufr = (phyRegBufr >> 3) & 0x0003;
			switch(phyRegBufr){
				case 0 :
					printf("10BASE-T Half-duplex\n\n\r");	
					break;
				case 1 :
					printf("100BASE-T Half-duplex\n\n\r");	
					break;
				case 2 :
					printf("10BASE-T Full-duplex\n\n\n\r");	
					break;
				case 3 :
					printf("100BASE-T Full-duplex\n\n\n\n\r");	
					break;
				default:
					break;
				}
		}
  //_puts_uart0(message);
}

#ifdef LOOPBACK_MAC
/*******************************************************************************
 ** @brief Create a ping response packet
 ** @param none
 ** @return none
 ******************************************************************************/
void createPingResponsePacket(void)
{
  uint32_t index;
   __packed uint8_t *pd;
  const char *testLB = "Hello World";
  
  
  // DMA is packed data, point to the tx buffer in the NEXT descriptor
  pd = (uint8_t *)dmaCtrl.txDmaDesc[dmaCtrl.txNext].buffer1;

  // create a message
  uint32_t payloadlength = sprintf((char *)txBufr,"%s -Message Number: %d", testLB, msgNumber);
	// ping reply
//03  uint32_t payloadlength = sprintf((char *)txBufr,"abcdefghijklmnopqrstuvwabcdefghi");

//  uint32_t totalpacketlength = IP_HEADER_LENGTH + UDP_HEADER_LENGTH + payloadlength;
  

	 // destination address
  *pd++ = destMacAddr[0];
  *pd++ = destMacAddr[1];
  *pd++ = destMacAddr[2];
  *pd++ = destMacAddr[3];
  *pd++ = destMacAddr[4];
  *pd++ = destMacAddr[5];


  // source address
  *pd++ = (VOR_ETH->MAC_ADDR_H >> 8) & 0xff;
  *pd++ = (VOR_ETH->MAC_ADDR_H & 0xff);
  *pd++ = (VOR_ETH->MAC_ADDR_L >> 24) & 0xff;
  *pd++ = (VOR_ETH->MAC_ADDR_L >> 16) & 0xff;
  *pd++ = (VOR_ETH->MAC_ADDR_L >> 8) & 0xff;
  *pd++ = (VOR_ETH->MAC_ADDR_L & 0xff);

  // type/length
  *pd++ = 0x08; // set to ipv4   //was  *pd++ = 0x00;
  *pd++ = 0x00;
  
  // IPV4/5= 20byte header
  *pd++ = 0x45;
  *pd++ = 0x00;
  
  // Total length
  *pd++ = 0x00;
  *pd++ = 0x3C;
  
  // Identification 
  *pd++ = 0xBE;
  *pd++ = 0xEF;
  
  // Flags 
  *pd++ = 0x00;
  *pd++ = 0x00;
  
  *pd++ = 0x40; // Time to live
  *pd++ = 0x01; // protocol ICMP
  
  // Header Checksum 
  *pd++ = 0x00;
  *pd++ = 0x00;
	
  // Source IP 
  *pd++ = srcIpAddr[0];
  *pd++ = srcIpAddr[1];
  *pd++ = srcIpAddr[2];
  *pd++ = srcIpAddr[3];
  
  // Destination IP 
  *pd++ = destIpAddr[0];
  *pd++ = destIpAddr[1];
  *pd++ = destIpAddr[2];
  *pd++ = destIpAddr[3];
  
  // ICMP 
  *pd++ = 0x00;  // type (echo/ping)
  *pd++ = 0x00;  // code
  
   // ICMP checksum 
  *pd++ = 0x00;
  *pd++ = 0x00;
  
  // Identifier 
  *pd++ = 0x00;
  *pd++ = 0x03;
  
   // Sequence number 
  *pd++ = 0xde;
  *pd++ = 0xad;
  
  // copy payload 
  for ( index = 0; index < payloadlength; index++)
  {
    *pd++ = txBufr[index];
  }  
  
  // set TX descriptor owned by DMA (TXDES0)
  dmaCtrl.txDmaDesc[dmaCtrl.txNext].status |= DescOwnByDma;  // pass ownership to the DMA
  
  // insert length into Descriptor (TXDES1 [10:0])
  dmaCtrl.txDmaDesc[dmaCtrl.txNext].length |= payloadlength;

  // start DMA transmit
  dmaCtrl.txDmaDesc[dmaCtrl.txNext].length |= DescTxDisableCrc | DescTxFirst \
                                            | DescTxLast | DescTxIntEnable;
  printf("New Tx Message Q%d : %d bytes\r\n",
          dmaCtrl.txNext, (dmaCtrl.txDmaDesc[dmaCtrl.txNext].length & 0x7ff));
  //_puts_uart0(message);

  if (dmaCtrl.txNext < NUMBER_TX_DESC - 1)
  {
    dmaCtrl.txNext++;   // next descriptor
  }
  else
  {
    dmaCtrl.txNext = 0; // top of chain
  }

  dmaCtrl.dmaDescCtrlFlags |= txInProcess;              // flag tx in process
  VOR_ETH->DMA_OPER_MODE |= ETH_DMA_OPER_MODE_ST_Msk;   // enable DMA Transmit
  
  msgNumber++;
  
}
#endif

/*******************************************************************************
 ** @brief  Calculates the CRC of the buffer
 ** @param  Lenth of message (Byte count)
 ** @param  pointer to message buffer
 ** @return CRC32 result
 ******************************************************************************/  
uint32_t crc32(uint32_t length, uint8_t *message)
{
  uint32_t index;
  int32_t cnt;
  uint32_t byte;
  uint32_t crc;
  uint32_t mask;

  index = 0;
  crc = 0xffffffff;   // seed
  
  while ( index < length)
  {
    byte = *message++;    // Get next byte
    crc = crc ^ byte;
    for (cnt = 7; cnt >= 0; cnt--)  // Do eight times.
    {
      mask = -(crc & 1);
      crc = (crc >> 1) ^ (0xedb88320 & mask);
    }
    index++;  // next byte
  }
   return ~crc;
}

/*******************************************************************************
 * @brief Initialize and start One second timer to check PHY link state
 * @param none
 * @return none
 * @note Using Timer 16
 ******************************************************************************/
void initLinkUpDownTimer(void)
{
  VOR_SYSCONFIG->TIM_CLK_ENABLE |= linkStatusTimerClkEnable; // enable clock for timer 16
  VOR_TIM16->RST_VALUE = linkStateTimerValue;
  VOR_TIM16->CTRL |= linkStateTimerControl;
  // enable irq in NVIC and enable timer
  NVIC_SetPriority(TIM16_IRQn, 6);  //jpwi was 3 
  NVIC_EnableIRQ(TIM16_IRQn);
  VOR_TIM16->ENABLE = 1;
}

/*******************************************************************************
 ** @brief intialize and start a 3 second timer
 ** @param none
 ** @return none
 ** @note Using Timer 17
 ******************************************************************************/
void initPingServerTimer(void)
{
  VOR_SYSCONFIG->TIM_CLK_ENABLE |= pingServerTimerClkEnable; // enable clock for timer 17
  VOR_TIM17->RST_VALUE = pingServerTimerRstValue; 
  VOR_TIM17->CTRL |= pingServerTimerControl;
  // enable irq in NVIC and enable timer
  NVIC_SetPriority(TIM17_IRQn, 4);
  NVIC_EnableIRQ(TIM17_IRQn);
  VOR_TIM17->ENABLE = 1;
}

/*******************************************************************************
 ** @brief  Intialize the Receive Queue (CHAIN)
 ** @param  none
 ** @return none
 ******************************************************************************/
void initRxQueueChain(void)
{ 
  // Receive Descriptor List Address Register
  // Points the DMA to the start of the Receive Descriptor list.
  VOR_ETH->DMA_RX_DESC_LIST_ADDR = (uint32_t)&dmaCtrl.rxDmaDesc[0].status;
  
  for (uint32_t index = 0; index < NUMBER_RX_DESC; index++)
  {
    dmaCtrl.rxDmaDesc[index].status = DescOwnByDma; // all RX descriptors
    dmaCtrl.rxDmaDesc[index].length = RxDescChain | RX_BUFR_SIZE*4;
    dmaCtrl.rxDmaDesc[index].buffer1 = (uint32_t)&rxBuffer[index][0];

    HAL_setBuffer(&rxBuffer[index][0], 0x0, RX_BUFR_SIZE*4); //jpwi added *4

    if (index < (NUMBER_RX_DESC - 1))
    {
      dmaCtrl.rxDmaDesc[index].buffer2 = (uint32_t)&dmaCtrl.rxDmaDesc[index + 1].status;
    }
    else
    {
      dmaCtrl.rxDmaDesc[index].buffer2 = VOR_ETH->DMA_RX_DESC_LIST_ADDR;
    }
  }
  dmaCtrl.rxNext = 0;
}

/*******************************************************************************
 ** @brief  Initialize the Transmit Queue (CHAIN)
 ** @param  none
 ** @return none
 ******************************************************************************/
void initTxQueueChain(void)
{
  uint32_t index;
  
  // DMA Transmit Descriptor List Address Register
  // Points the DMA to the start of the Transmit Descriptor list. 
  VOR_ETH->DMA_TX_DESC_LIST_ADDR = (uint32_t)&dmaCtrl.txDmaDesc[0].status;
  
  for (index = 0; index < NUMBER_TX_DESC; index++)
  {
    dmaCtrl.txDmaDesc[index].status = 0;
    dmaCtrl.txDmaDesc[index].length = TxDescChain; // (TCH) 2nd buffer address is chain address  [24]
    dmaCtrl.txDmaDesc[index].buffer1 = (uint32_t)&txBuffer[index][0];
    
    HAL_setBuffer(&txBuffer[index][0], 0x0, TX_BUFR_SIZE);
    
    // initialize buffer 2 , point to next descriptor in chain
    if (index < (NUMBER_TX_DESC - 1))
    {
      dmaCtrl.txDmaDesc[index].buffer2 = (uint32_t)&dmaCtrl.txDmaDesc[index + 1].status;
    }
    else
    {
      dmaCtrl.txDmaDesc[index].buffer2 = VOR_ETH->DMA_TX_DESC_LIST_ADDR;
    }
  }
  dmaCtrl.txNext = 0;
}

/*******************************************************************************
 ** @brief  Initialize Ethernet MAC 10/100
 ** @param none
 ** @return result
 ******************************************************************************/
hal_status_t initialEtherCore(void)
{
  hal_status_t status;
  uint16_t phyRegBufr;
  
  // enable ethernet peripheral clock
  VOR_SYSCONFIG->PERIPHERAL_CLK_ENABLE |= CLK_ENABLE_ETH;
  
  // reset the IP core
  status = HAL_CoreReset();
  if (status != hal_status_ok)
  {
    return status;        // can't reset, exit with error
  }
  
// ****************************************************************************
//  Initialize DMA
  
  // DMA Bus Mode Register
  VOR_ETH->DMA_BUS_MODE = DmaInitialBusMode;

  // ********* initialize RX descriptors  *********************
  initRxQueueChain();
  
  // *********** initialize TX descriptors   ******************
  initTxQueueChain();
  
  // DMA Interrupt Enable Register
  // enable normal interrupts
  VOR_ETH->DMA_INTR_EN = DmaEnableAllInterrupts;
  
  // DMA Operation Mode Register
  VOR_ETH->DMA_OPER_MODE = DmaInitialOperationMode;
  
// ***************************************************************
// Initialize the MAC interface

  // MAC Config Register
  VOR_ETH->MAC_CONFIG = MacInitialConfig;
  
  // MAC Frame Filter
  VOR_ETH->MAC_FRAME_FLTR = MacInitialFrameFilter;
  
  // MAC Flow Control
  VOR_ETH->MAC_FLOW_CTRL = MacInitialFlowControl;
    
  // ******************************************************************
  // Initialize PHY 
  status = HAL_ResetPHY();
  if ( status != hal_status_ok)   // TODO: refactor return actual PHY status
  {
    return status;
  }
  
  // MII CSR configuration MDC/MDIO clock
  if((SystemCoreClock >= 20000000)&&(SystemCoreClock < 35000000))
  {
    VOR_ETH->MAC_GMII_ADDR |= (2 << PHY_MACMII_CR_DIV16);
  }
  else if((SystemCoreClock >= 35000000)&&(SystemCoreClock < 60000000))
  {
    VOR_ETH->MAC_GMII_ADDR |= (2 << PHY_MACMII_CR_DIV26); // 35-60MHZ
  }  
  else 
  {
    VOR_ETH->MAC_GMII_ADDR |= (2 << PHY_MACMII_CR_DIV42); // 60-100MHZ
  }

  // Set Physical Layer Address - Register 4 ( GMII Address Register [15:11] )
  // This address is set by straps on the KSZ8041TL upon power-up
  VOR_ETH->MAC_GMII_ADDR |= MIIADDR_PHY_LAYR_ADDR;

  // On the VA416-EVK-B4 board, PHY's power-up straps are (default) set for:
  //  Half Duplex
  //  Auto-Negotiate enabled,
  //  Speed = 100Mbps,
  //  Loopback = OFF
  if (HAL_SetPhyDuplex(PHYFULLDUPLEX) != hal_status_ok)  // Set the PHY to Full Duplex mode
  {
    return status;
  }
	
#if 0 

  // The PHY default to :
  //  LED1: Speed; LED0: Link/Activity
  if (HAL_SetPhyLeds(false) != hal_status_ok)  // Set the PHY Leds to Speed/Activity mode
  {
    return status;
  }
#endif
  
  dmaCtrl.dmaDescCtrlFlags = 0;
  
  // save a local copy of the PHY State's of DUPLEX, SPEED, AUTO-NEGOTIATE AND LOOPBACK
  savePHYstate();
  
  // PHY interrupt enable
  phyRegBufr = PHY_INTER_ENABLE_ALL;
  
  if (HAL_WritePhyReg(PHY_INTR_CTRL_STATUS, phyRegBufr) != 0)
  {
    //_puts_uart0("Error could not enable PHY interrupts\r\n");
  }
  // DMA and MAC interrupts are enabled in the main loop after the link is up
  return status;
}

/*******************************************************************************
 **
 ** @brief  Checks for and handles change in the Physical Link state
 **
 ** @param  entry: nothing 
 **
 ** @return hal_status_ok if successfull, else error code
 **
 ******************************************************************************/
hal_status_t handlePhyLinkState(void)
{
  uint16_t phyStatusReg;
  
  // reading the PHY is slow (6us)
  if ((HAL_ReadPhyReg(PHY_STATUS_REG, &phyStatusReg) == hal_status_ok ))
  {		// and PHY needs read twice for correct result
    if ((HAL_ReadPhyReg(PHY_STATUS_REG, &phyStatusReg) == hal_status_ok ))
    {
      dmaCtrl.dmaDescCtrlFlags &= ~(phyLinkState);                // clear the old state
      dmaCtrl.dmaDescCtrlFlags |= (phyStatusReg & phyLinkState);  // save the present state

      // check for a Link State change
      if ((phyPrevStatusReg & phyLinkState) != (phyStatusReg & phyLinkState))
      {
        // there is a change in state, check to see if the link just came up
        if (dmaCtrl.dmaDescCtrlFlags & phyLinkState)
        {
          // Link just came up, enable DMA and Interrupts
          VOR_ETH->DMA_INTR_EN = DmaEnableAllInterrupts;

          // enable Ethernet Interrupt's in M4
          NVIC_SetPriority(Ethernet_IRQn, 3);   //jpwi was 4, but so was TIM17
          NVIC_EnableIRQ(Ethernet_IRQn);
          VOR_ETH->DMA_OPER_MODE |= ETH_DMA_OPER_MODE_SR_Msk;  //enable DMA Receive
        }
        else  // link just went down
        {
          // disable DMA Transmit & Receive
          VOR_ETH->DMA_OPER_MODE &= ETH_DMA_OPERMODE_DISABLE_RXTX_Msk;
          NVIC_DisableIRQ(Ethernet_IRQn); // disable Ethernet Interrupt
        }
        // print link state
				msTimeStamp();
					  
        printf(" ->""PHY Link Stat:\t%s\r\n", (dmaCtrl.dmaDescCtrlFlags & phyLinkState) ? "Up" : "Down");  
        //_puts_uart0(message);

				printLinkAttributes();
        phyPrevStatusReg = phyStatusReg;
      }
      checkPhyLink = 0;   // clear for next interrupt
    }
    else    // First MDIO read of PHY failed
    {
      //_puts_uart0(" ** 1st Error.. PHY Status Read Failed\r\n");
    }
  }
  else
  {
    //_puts_uart0(" **  2nd Error.. PHY Status Read Failed\r\n");
  }

	#if LOOPBACKCASE == 1	//If set for MAC loopback, force phy link to 1
	dmaCtrl.dmaDescCtrlFlags &= ~(phyLinkState);                // clear the old state
	dmaCtrl.dmaDescCtrlFlags |= phyLinkState; 
	#endif
	
	return hal_status_ok;
	}

/*******************************************************************************
 ** @brief Save the Physical State's of DUPLEX, SPEED, AUTO-NEGOTIATE AND LOOPBACK
 ** @param none
 ** @return hal_status_ok if successful, else error code

 ** Note: These are requeasted setting NOT the actual link attributes !!
 ******************************************************************************/
hal_status_t savePHYstate(void)
{
  uint16_t phyRegBufr;

  if ((HAL_ReadPhyReg(PHY_CONTROL_REG, &phyRegBufr) == hal_status_ok))
  {
    if ((HAL_ReadPhyReg(PHY_CONTROL_REG, &phyRegBufr) == hal_status_ok))
    {
      dmaCtrl.dmaDescCtrlFlags |= phyRegBufr;
    }
    else
    {
      return hal_status_timeout;
    }
  }
  else
  {
    return hal_status_timeout;
  }
  
  if ((HAL_ReadPhyReg(PHY_STATUS_REG, &phyRegBufr) == hal_status_ok))
  {
    if ((HAL_ReadPhyReg(PHY_STATUS_REG, &phyRegBufr) == hal_status_ok))
    {
      dmaCtrl.dmaDescCtrlFlags |= (phyRegBufr & phyLinkState);
      phyPrevStatusReg = phyRegBufr;
    }
    else
    {
      return hal_status_timeout;
    }
  }
  else
  {
    return hal_status_timeout;
  }
  return hal_status_ok;
}

/*******************************************************************************
 ** @brief Handle DMA queuss
 ** @param none
 ** @return none
 ******************************************************************************/  
void dmaQueueHandler(void)
{
  // ****** check if MAC debug register has changed ***************************
  if (macDebugSnapShot != VOR_ETH->MAC_DEBUG)
  {
    macDebugSnapShot = VOR_ETH->MAC_DEBUG;  // snap shot register

#ifdef MAC_DEBUG_REGISTER
//    printMacDebugReg();       // print MAC DEBUG
#endif

  }
	
  if (((VOR_ETH->DMA_STATUS & ETH_DMA_STATUS_TS_Msk) >> ETH_DMA_STATUS_TS_Pos) == 0x06)
  {
		txSuspendRestarts++;
		VOR_ETH->DMA_TX_POLL_DEMAND = 0x1;        // force poll demand
	}


  // **************************************************************************
  // **********   parse normal interrupts  ************************************
  if (dmaCtrl.dmaDescCtrlFlags & normalInterp)
  {
    // ***********   TX No Buffer  ************************ 
    if (dmaCtrl.dmaDescCtrlFlags & txNoBuffer)
    {
			msTimeStamp();
					  
      printf(" ->""Tx ""No Buffer"" Q""%ld"", txNext %ld, ""0x%.8lX - 0x%.8lX\r\n\n",
                txCurrIndex, dmaCtrl.txNext, dmaCtrl.txDmaDesc[txCurrIndex].status, *(uint32_t *)VOR_ETH->DMA_CURR_TX_DESC);
      //_puts_uart0(message);
      
#if 0     
			if ((dmaCtrl.txDmaDesc[txCurrIndex].status & DescOwnByDma) == 0)
      {
        dmaCtrl.txDmaDesc[txCurrIndex].status |= DescOwnByDma;
        VOR_ETH->DMA_TXPOLL_DEMAND = 0x1;                      // poll demand
      }
#endif
      

      if ((*(uint32_t *) VOR_ETH->DMA_CURR_TX_DESC & DescOwnByDma) == 0)
      {
        *(uint32_t *)VOR_ETH->DMA_CURR_TX_DESC |= DescOwnByDma;   // set descriptor ownership to DMA
        VOR_ETH->DMA_TX_POLL_DEMAND = 0x1;                      // poll demand
//				printf("Re trying\r\n\n");
//				//_puts_uart0(message);
      
      }
      dmaCtrl.dmaDescCtrlFlags &= ~(txNoBuffer); 
    }

    // ************ TX complete  *************************
    if (dmaCtrl.dmaDescCtrlFlags & txComplete)
    {
			msTimeStamp();
#ifdef BUFFER_MESSAGES
      printf(" ->""Tx ""Q""%ld"" Complete - TxBufr ""0x%.8lX\r\n""\r\n", \
                txCurrIndex, dmaCtrl.txDmaDesc[txCurrIndex].buffer1); 
      //_puts_uart0(message);
#endif
      dmaCtrl.dmaDescCtrlFlags &= ~(txComplete);
      
      VOR_ETH->DMA_OPER_MODE &= ~(ETH_DMA_OPER_MODE_ST_Msk);   // disable DMA Transmit
      HAL_setBuffer(&txBuffer[txCurrIndex][0], 0x0, TX_BUFR_SIZE);
    }

    // ************* RX complete ******************************
    if (dmaCtrl.dmaDescCtrlFlags & rxComplete)
    {
			msTimeStamp();
#ifdef BUFFER_MESSAGES
      printf(" ->""Rx ""Q""%ld"" - RxBufr ""0x%.8lX\n\r",		\
              rxCurrIndex, dmaCtrl.rxDmaDesc[rxCurrIndex].buffer1);
      //_puts_uart0(message);
#endif
      dmaCtrl.dmaDescCtrlFlags &= ~(rxComplete);  // clear flag
      parseReceivedMessage();
    }

    // ************** Early Receive
    if (dmaCtrl.dmaDescCtrlFlags & earlyReceive)
    {
      //_puts_uart0("Early Receive\r\n");
      dmaCtrl.dmaDescCtrlFlags &= ~(earlyReceive); 
    }
    dmaCtrl.dmaDescCtrlFlags &= ~(normalInterp);
  }
    
  // parse abnormal interrupts
  if (dmaCtrl.dmaDescCtrlFlags & abnormalInterp)
  {  
    if (dmaCtrl.dmaDescCtrlFlags & txProcStopped)
    {
      //_puts_uart0("Tx Process Stopped\r\n");
      dmaCtrl.dmaDescCtrlFlags &= ~(txProcStopped); 
    }
    
    if (dmaCtrl.dmaDescCtrlFlags & txJabberTO)
    {
      //_puts_uart0("Tx Jabber Time Out\r\n");
      dmaCtrl.dmaDescCtrlFlags &= ~(txJabberTO); 
    }
    
    if (dmaCtrl.dmaDescCtrlFlags & rxFifoOver)
    {
      //_puts_uart0("Rx FIFO Over\r\n");
      dmaCtrl.dmaDescCtrlFlags &= ~(rxFifoOver); 
    }
    
    if (dmaCtrl.dmaDescCtrlFlags & txUnderFlow)
    {
      //_puts_uart0("Tx Under flow\r\n");
      dmaCtrl.dmaDescCtrlFlags &= ~(txUnderFlow); 
    }
    
    // **********  RX No Buffer
    if (dmaCtrl.dmaDescCtrlFlags & rxNoBuffer)
    {
			msTimeStamp();
					  
      printf(" ->""Rx ""No Buffer""(RDES0) ""0x%.8lX"" , rxNext: %ld\r\n\n",
                *(uint32_t *)VOR_ETH->DMA_CURR_RX_DESC, dmaCtrl.rxNext);
      //_puts_uart0(message);

      // NO RECOVERY, FIX!!
      if ( (*(uint32_t *)VOR_ETH->DMA_CURR_RX_DESC & DescOwnByDma) == 0 )
      {
        *(uint32_t *)VOR_ETH->DMA_CURR_RX_DESC |= DescOwnByDma;   // set descriptor ownership to DMA
        VOR_ETH->DMA_TX_POLL_DEMAND = 0x1;                      // poll demand
      }
      
      dmaCtrl.dmaDescCtrlFlags &= ~(rxNoBuffer); 
    }
    
    if (dmaCtrl.dmaDescCtrlFlags & rxProcStopped)
    {
      //_puts_uart0("Rx Process Stopped\r\n");
      dmaCtrl.dmaDescCtrlFlags &= ~(rxProcStopped); 
    }
    
    if (dmaCtrl.dmaDescCtrlFlags & rxWatchTO)
    {
      //_puts_uart0("Rx Watchdog time out\r\n");
      dmaCtrl.dmaDescCtrlFlags &= ~(rxWatchTO); 
    }
    
    if (dmaCtrl.dmaDescCtrlFlags & earlyTransmit)
    {
      dmaCtrl.dmaDescCtrlFlags &= ~(earlyTransmit); 
    }
    
    if (dmaCtrl.dmaDescCtrlFlags & fatalBusErr)
    {
      //_puts_uart0("Fatel Bus Error\r\n");
      dmaCtrl.dmaDescCtrlFlags &= ~(fatalBusErr); 
    }
    
    dmaCtrl.dmaDescCtrlFlags &= ~(abnormalInterp);
  } 
}

/*******************************************************************************
 **
 ** @brief Parse BroadCast Destination frame.  
 **
 ** @param none
 **
 ** @return none
 **
 ** @note Test for ARP (RFC  or ICMP (RFC 792), most common network level
 **
 ******************************************************************************/
void parseBroadCast(void)
{
  
}

/*******************************************************************************
 **
 ** @brief 
 **
 ** @param none
 **
 ** @return none
 **
 ** @note This should be a UDP datagram (RFC 768)
 **
 ******************************************************************************/
void parseReceivedMessage(void)
{
  uint8_t *pdata;
  uint8_t macAddr[6];
  uint32_t index;
  uint32_t totalLen;
//jpwi  char cg_temp[256];
  volatile char cg_temp[MAX_FRAME_PAYLOAD] __UNUSED; // should implement zero copy
 
  // DMA is packed data, point to the RX buffer
  pdata = (uint8_t *)dmaCtrl.rxDmaDesc[rxCurrIndex].buffer1;
  
  // look at destination address
  HAL_GetMacAddr(macAddr);    // we only parse our address
  for (index = 0; (macAddr[index] == *pdata) && (index < 6); index++, pdata++);
  if (index == 6)  // exact match
  {
    pdata += 10;    // point to Total Length of IP packet
    totalLen = (*pdata);
    totalLen = (*pdata++ << 8);
    totalLen |= *pdata++;
#ifdef DEBUG_UDP_MESSAGE  
  printf("\nRx Packet Len ""%ld"" b\r\n", totalLen);
  //_puts_uart0(message);
#endif
    totalLen -= 28;
#ifdef DEBUG_UDP_MESSAGE  
  printf("Rx Data Len ""%ld"" b\r\n", totalLen);
  //_puts_uart0(message);
#endif
    pdata += 24;
    for (index = 0; index < totalLen; )
    {
      cg_temp[index++] = *pdata++;
    }
    cg_temp[index] = 0x0;
    
		printf("\n\r");
		//_puts_uart0(message);
		//_puts_uart0(cg_temp);
		printf("\r\n\n");
		//_puts_uart0(message);
		
#ifdef LOOPBACK_MAC
//		createPingResponsePacket();
#endif
  }
  else{
		
		// Point back to the RX buffer   
  pdata	= (uint8_t *)dmaCtrl.rxDmaDesc[rxCurrIndex].buffer1;

#ifdef SHOWALLIP_MESSAGES
	printf("\r\nRx Packet for ");
  //_puts_uart0(message);
		for (index = 0; index < 5; )
    {
														   
			printf("%.2X:", *pdata);
			//_puts_uart0(message);
			cg_temp[index++] = *pdata++;
    }
  printf("%.2X (", *pdata);
  //_puts_uart0(message);

    pdata += 25; // skip forward to Dest. IP addr.
		for (index = 0; index < 3; )
    {
		printf("%.3d.", *pdata);
														   
		//_puts_uart0(message);
		cg_temp[index++] = *pdata++;
    }
		printf("%.3d)\r\n\n", *pdata);
															   
  //_puts_uart0(message);
#endif
	}
  rxUdpMessageNumber++;
}

/*******************************************************************************
 ** 
 ** @brief  
 **
 ** @param  entry 
 **
 ** @return result
 **
 ******************************************************************************/
void _sendUDPMessage(void)
{
  uint32_t index;
  uint32_t len;
  uint8_t *pd;

#ifdef USE_LOCAL_CRC  
  uint32_t crc; 
  uint8_t *pd2;
#endif
  
  // DMA is packed data, point to the tx buffer in the NEXT descriptor
  pd = (uint8_t *)dmaCtrl.txDmaDesc[dmaCtrl.txNext].buffer1;
#ifdef USE_LOCAL_CRC
  pd2 = pd; // save a pointer for CRC calculation
#endif

  // create a message
  uint32_t payloadlength = sprintf((char *)txBufr,"%s", testMsg);
  uint32_t totalpacketlength = IP_HEADER_LENGTH + UDP_HEADER_LENGTH + payloadlength;
#ifdef DEBUG_UDP_MESSAGE
  printf("\r\nTx Payload Len: ""%ld"", Total Pkt Len: ""%ld""\r\n", \
            payloadlength, totalpacketlength);
  //_puts_uart0(message);
#endif
  // Ethernet header 
  // destination address
  for (index = 0; index < 6; index++, pd++)
  {
    *pd = destMacAddr[index];
  }

  // source address
  *pd++ = (VOR_ETH->MAC_ADDR_H >> 8) & 0xff;
  *pd++ = (VOR_ETH->MAC_ADDR_H & 0xff);
  *pd++ = (VOR_ETH->MAC_ADDR_L >> 24) & 0xff;
  *pd++ = (VOR_ETH->MAC_ADDR_L >> 16) & 0xff;
  *pd++ = (VOR_ETH->MAC_ADDR_L >> 8) & 0xff;
  *pd++ = (VOR_ETH->MAC_ADDR_L & 0xff);

  // type/length
  *pd++ = 0x08;   // ip protocol
  *pd++ = 0x00;
  
  // IP Header
  *pd++ = 0x45; // IPv4, Header length
  *pd++ = 0;            // type of service
  *pd++ = (uint8_t)(totalpacketlength >> 8);
  *pd++ = (uint8_t)(totalpacketlength & 255);
  
  *pd++ = 0;
  *pd++ = 0;
  *pd++ = (1 << 6);
  *pd++ = 0;
  
  *pd++ = 255;    // Time To Live
  *pd++ = 17;     // UDP protocol
  *pd++ = 0x00;   // header checksum 
  *pd++ = 0x00;   // IP will calculate Checksum
  
  *pd++ = srcIpAddr[0];   // source address
  *pd++ = srcIpAddr[1];
  *pd++ = srcIpAddr[2];
  *pd++ = srcIpAddr[3];
  
  *pd++ = destIpAddr[0];   // destination address
  *pd++ = destIpAddr[1];
  *pd++ = destIpAddr[2];
  *pd++ = destIpAddr[3];
   
  // UDP header
  *pd++ = 0x00;   // source port (15) current unassigned 
  *pd++ = 0x0F;
  *pd++ = 0x02;   // destination port (625) current unassigned
  *pd++ = 0x71;
  
  uint16_t udpheaderlength = UDP_HEADER_LENGTH + payloadlength;

#ifdef DEBUG_UDP_MESSAGE  
  printf("Tx UDP hdr len ""%d"" b,  ", udpheaderlength);
  //_puts_uart0(message);
#endif

  *pd++ = (uint8_t)((udpheaderlength >> 8) & 255);  // todo:
  *pd++ = (uint8_t)(udpheaderlength & 255);
  *pd++ = 0x00; // hardware will calculate header checksum
  *pd++ = 0x00; // just set aside space
  
  // copy payload (The Message)
  for ( index = 0; index < payloadlength; index++)
  {
    *pd++ = txBufr[index];
  }  
  
  len = FRAME_HEADER_SIZE + totalpacketlength;

#ifdef DEBUG_UDP_MESSAGE
  printf("CRC len ""%ld"" bytes\r\n", len);
  //_puts_uart0(message);
#endif

#ifdef USE_LOCAL_CRC
  // insert CRC 
  crc = crc32( len, pd2);    
  pd2 = (uint8_t *)&crc;
  for (index = 0; index < FRAME_CRC; index++)
  {
    *pd++ = *pd2++;
  }

  // insert length into Descriptor (TXDES1 [10:0])
  dmaCtrl.txDmaDesc[dmaCtrl.txNext].length |= ((len + FRAME_CRC) & 0x7ff); 
#else
  // insert length into Descriptor (TXDES1 [10:0])
  dmaCtrl.txDmaDesc[dmaCtrl.txNext].length |= (len & 0x7ff); 
#endif

  // set TX descriptor owned by DMA (TXDES0)
  dmaCtrl.txDmaDesc[dmaCtrl.txNext].status |= DescOwnByDma;  // pass ownership to the DMA
  
  
  // start DMA transmit
	//  dmaCtrl.txDmaDesc[dmaCtrl.txNext].length |= DescTxDisableCrc | DescTxFirst | DescTxLast | DescTxIntEnable;
  dmaCtrl.txDmaDesc[dmaCtrl.txNext].length |= DescTxFirst | DescTxLast | DescTxIntEnable;


  printf("Tx Msg ""Q""%ld"" : ""%ld"" bytes TxBufr ""0x%.8lX\r\n\n",
          dmaCtrl.txNext, (dmaCtrl.txDmaDesc[dmaCtrl.txNext].length & 0x7ff), dmaCtrl.txDmaDesc[dmaCtrl.txNext].buffer1);
  //_puts_uart0(message);

  txCurrIndex = dmaCtrl.txNext;
  
  if (dmaCtrl.txNext < NUMBER_TX_DESC - 1)
  {
    dmaCtrl.txNext++;   // next descriptor
  }
  else
  {
    dmaCtrl.txNext = 0; // top of chain
  }
  dmaCtrl.dmaDescCtrlFlags |= txInProcess;              // flag tx in process
  VOR_ETH->DMA_OPER_MODE |= ETH_DMA_OPER_MODE_ST_Msk;   // enable DMA Transmit
  txUdpMessageNumber++;   // next message number
}

/*******************************************************************************
 ** @brief timeStamp 
 ** @param none
 ** @return result
 ******************************************************************************/
void msTimeStamp(void)
{
  #ifdef TIMESTAMP_MESSAGE
    printf( "%.10ld ms  ",(uint32_t)HAL_time_ms);
    //_puts_uart0(message);
  #endif
}

uint8_t ether_status;

/*******************************************************************************
 ** @brief Ethernet entry 
 ** @param none
 ** @return result
 ******************************************************************************/
#define PORT_TEST PORTA
#define PORTPIN_TEST (0)

void testFunc(){
  PORT_TEST->SETOUT=1UL<<PORTPIN_TEST;
  return;
}

int phytest_main(void)
{
	uint32_t count;
	
  // initialize fault exceptions
	SCB->SHCSR |= SCB_SHCSR_USGFAULTENA_Msk
						 | SCB_SHCSR_BUSFAULTENA_Msk
						 | SCB_SHCSR_MEMFAULTENA_Msk; //enable Usage-/Bus-/MPU Fault

	// initialize Ethernet
	count = 0;
	do{
		ether_status = initialEtherCore();
    VOR_WATCH_DOG->WDOGINTCLR = 1;
		for(uint32_t i=0; i<10000UL; i++)
		{
			__NOP();
		}
		if (count++ == 10000){
			printf( "\r\nInit Failed\r\n"); 
			////_puts_uart0(message);
			break;
		}
	} while(ether_status);
	
	printf( "\n%.10ld ms  "" ->""Init time  \n\r",(uint32_t)HAL_time_ms);
	////_puts_uart0(message);

  restartAutoNegotiation();
	Delay(1000);  //Delay for AN process
	
#ifdef DEBUG_PHY_CONFIG
  print_PHYconfig();
//	dumpPHY_Registers();
#endif

#if LOOPBACKCASE == 1
	VOR_ETH->MAC_CONFIG |= ETH_MAC_CONFIG_LM_Msk;  //enable MAC loopback (requires PHY clock)
#endif
 
#if LOOPBACKCASE == 2
	HAL_SetPhyLoopBack(true);
#endif

  HAL_SetMacAddr(macAddr0);   // set our MAC address
  
#ifdef PRINT_MAC_DMA_REGS
  print_macRegisters();
#endif
  
  print_MACaddress();
    
  prevMacDebug = VOR_ETH->MAC_DEBUG;
    
  txUdpMessageNumber = 1;
	rxUdpMessageNumber = 1;
	msgNumber = 1;
  txSuspendRestarts =0;

  sysTicker = 0;
  
  initLinkUpDownTimer();  // start ONE second timer to check PHY
  initPingServerTimer();	// start timer to periodicall send UDP packets
	
  // Set up LED on CPU card as heartbeat
  VOR_GPIO->BANK[6].DIR&=~(uint32_t)1>>5;
  VOR_GPIO->BANK[6].DATAOUT=0x00000020;
	
	
  for (timeTestLoop = 1000000; timeTestLoop > 0; timeTestLoop --)  // loop until power off
  {
		if (checkPhyLink)   // Check PHY link status (updated exery 1 sec)
		{   
      handlePhyLinkState();   // probe PHY link state
		  PORTG->TOGOUT = 1<<5;   // PORTG.5 is EVK MCU Board LED.
		}
    VOR_WATCH_DOG->WDOGINTCLR = 1;
    
    // PHY link needs to be Active
    if (dmaCtrl.dmaDescCtrlFlags & phyLinkState)
		{
			dmaQueueHandler();  // handle DMA Queue
		}
    
    if (pingTheUDPServer)
    {
      pingTheUDPServer = 0;
      if (dmaCtrl.dmaDescCtrlFlags & phyLinkState)   // PHY link needs to be UP
      { 
        // tx Descriptor must be owned by host
        if (((dmaCtrl.txDmaDesc[dmaCtrl.txNext].status & DescOwnByDma) == 0) && \
           ((dmaCtrl.dmaDescCtrlFlags & txInProcess) == 0))
        {

#ifdef LOOPBACK_MAC
//					createLoopbackTest();
#endif
          _sendUDPMessage();
        }
      }
    } 

    if (VOR_ETH->MAC_DEBUG != prevMacDebug)
    {
      macDebugSnapShot = VOR_ETH->MAC_DEBUG;
#ifdef MAC_DEBUG_REGISTER
 //     printMacDebugReg();
#endif
      prevMacDebug = VOR_ETH->MAC_DEBUG;
    }
    
  }   // *****************  END of forever loop

	printf("\n\nMAC loopback test done.\n");
  return 0;
}

#endif // INC_PHY_TEST

/*****************************************************************************/ 
/* End of file                                                               */ 
/*****************************************************************************/
