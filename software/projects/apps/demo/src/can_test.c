/***************************************************************************************
 * @file     can_test.c
 * @version  V1.0
 * @date     15 November 2019
 *
 * @note
 * VORAGO Technologies
 *
 * @note
 * Copyright (c) 2013-2019 VORAGO Technologies.
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

#include "can_test.h"
#include "board.h"

//#include "va416xx_hal_ioconfg.h"

/*****************************************************************************/ 
/* Function implementation - global ('extern') and local ('static')          */ 
/*****************************************************************************/

uint32_t can_test_loopback(VOR_CAN_Type *myCAN)
{
  can_config_t myCANCfg;
  hal_can_id29_or_11_t dontCareIDNone=0x0;//care about all=must be exact match
  hal_can_id29_or_11_t id29Bits[14]={0xdead,0xbeef,0x1234};
  can_pkt_t myTxPkt[3], myRxPkt[3];
  uint32_t i;
  volatile uint32_t timeout;
  

  
	myCANCfg.CGCR = 0x0
//				CAN_CGCR_BUFFLOCK_Msk  
			| CAN_CGCR_TSTPEN_Msk    
//			| CAN_CGCR_DDIR_Msk      
//			| CAN_CGCR_LO_Msk        
//			| CAN_CGCR_LOOPBACK_Msk  //yes loopback here?
//			| CAN_CGCR_INTERNAL_Msk  
			| CAN_CGCR_DIAGEN_Msk    
			| CAN_CGCR_EIT_Msk
      | CAN_CGCR_CANEN_Msk     
 // 		| CAN_CGCR_CRX_Msk       
 // 		| CAN_CGCR_CTX_Msk;  
// 		| CAN_CGCR_IGNACK_Msk
//  uint32_t CICEN; //EICEN|ICEN[14:0] //disable ISR mode for now
 ;
  myCANCfg.CTIM_TSEG2 = (1UL-1UL);  //1 time quanta 
  myCANCfg.CTIM_TSEG1 = (2UL-1UL);  //2 time quanta
  myCANCfg.CTIM_SJW   = (1UL-1UL);  //1 time quanta
  myCANCfg.CTIM_PSC   = (20UL-2UL); //CAN PreScalar=20
  myCANCfg.CICEN      = (BITMASK(ALLF,14,0)<<CAN_CICEN_ICEN_Pos)
                       |(CAN_CICEN_EICEN_Msk);
                      
  
  //set everyone to rx_not_ready
  HAL_Can_ClearBuffer((can_cmb_t *)&VOR_CAN0->CNSTAT_CMB0 , 14);
  HAL_Can_ClearBuffer((can_cmb_t *)&VOR_CAN1->CNSTAT_CMB0 , 14);

  
  //config some rx buf here
  HAL_Can_Setup(VOR_CAN0, (const can_config_t *)&myCANCfg, CAN0_IRQn);
  HAL_Can_Setup(VOR_CAN1, (const can_config_t *)&myCANCfg, CAN1_IRQn);

  HAL_Can_Setup_gMask(VOR_CAN0,HAL_Can_Make_maskBX(dontCareIDNone, 1, 1));
  HAL_Can_Setup_bMask(VOR_CAN0,HAL_Can_Make_maskBX(dontCareIDNone, 1, 1));

  HAL_Can_Setup_gMask(VOR_CAN1,HAL_Can_Make_maskBX(dontCareIDNone, 1, 1));
  HAL_Can_Setup_bMask(VOR_CAN1,HAL_Can_Make_maskBX(dontCareIDNone, 1, 1));



 
  //set up RX buf0-2 on can0
  HAL_Can_ConfigCMB_Rx(id29Bits[0], en_can_cmb_msgtype_EXT29, (can_cmb_t *)&VOR_CAN1->CNSTAT_CMB0);
  HAL_Can_ConfigCMB_Rx(id29Bits[1], en_can_cmb_msgtype_EXT29, (can_cmb_t *)&VOR_CAN1->CNSTAT_CMB1);
  HAL_Can_ConfigCMB_Rx(id29Bits[2], en_can_cmb_msgtype_EXT29, (can_cmb_t *)&VOR_CAN1->CNSTAT_CMB2);

  myTxPkt[0].id=id29Bits[0];
  myTxPkt[1].id=id29Bits[1]; 
  myTxPkt[2].id=id29Bits[2];
  
  //all extended, not Remote req
  myTxPkt[0].msgType=en_can_cmb_msgtype_EXT29;
  myTxPkt[1].msgType=en_can_cmb_msgtype_EXT29;
  myTxPkt[2].msgType=en_can_cmb_msgtype_EXT29;
  
  //max data
  myTxPkt[0].dataLengthBytes=8;
  myTxPkt[1].dataLengthBytes=8;
  myTxPkt[2].dataLengthBytes=8;
  
  myTxPkt[0].txPriorityCode=0x3;
  myTxPkt[1].txPriorityCode=0x2;
  myTxPkt[2].txPriorityCode=0x1;  
 
  for(i=0;i<4;i++){
    myTxPkt[0].data16[i]=0xa510+i;
    myTxPkt[1].data16[i]=0xa520+i;
    myTxPkt[2].data16[i]=0xa530+i;
  }
  
  //set up tX buf3-5 on can, does not wait for reception
  //loopback should drop in rx buf0-2 
  //does not block
  HAL_Can_sendCanPkt((can_cmb_t *)&VOR_CAN0->CNSTAT_CMB3, (can_pkt_t *)&myTxPkt[0]);
  HAL_Can_sendCanPkt((can_cmb_t *)&VOR_CAN0->CNSTAT_CMB4, (can_pkt_t *)&myTxPkt[1]);
  HAL_Can_sendCanPkt((can_cmb_t *)&VOR_CAN0->CNSTAT_CMB5, (can_pkt_t *)&myTxPkt[2]);
  
  //wait for tx status to indicate it's sent
  //just checking one of the 3 transaction
  timeout=50000;
  while((VOR_CAN0->CNSTAT_CMB3 & CAN_CNSTAT_CMB0_ST_Msk)!=en_can_cmb_cnstat_st_TX_NOT_ACTIVE){
    --timeout;
    if(timeout==0){
      return 1;    
    }      
  }
  
  timeout=50000;
  //wait for rx status to indicate it's received
  while((VOR_CAN1->CNSTAT_CMB0 & CAN_CNSTAT_CMB0_ST_Msk)!=en_can_cmb_cnstat_st_RX_FULL){
    --timeout;
    if(timeout==0){
      return 2;
    }      
  }
  //just check CMB0
  
  HAL_Can_getCanPkt((can_cmb_t *)&VOR_CAN1->CNSTAT_CMB0, (can_pkt_t *)&myRxPkt[0]);
  if(myRxPkt[0].dataLengthBytes!=8){
    return 3;//wrong data length
  }
  for(i=0;i<4;i++){
    if((myRxPkt[0].data16[i])!=myTxPkt[0].data16[i]){
      return 4;//wrong data
    }
  } 
  return 0;//a-ok
}

uint32_t can_test_connect(VOR_CAN_Type *myCAN)
{
  can_config_t myCANCfg;
  hal_can_id29_or_11_t dontCareIDNone=0x0;//care about all 
  hal_can_id29_or_11_t id29Bits[14]={0xdead,0xbeef,0x1234};
  can_pkt_t myTxPkt[3], myRxPkt[3];
  uint32_t i;
  volatile uint32_t timeout;
  volatile uint32_t voidRead __attribute((unused));

	myCANCfg.CGCR = 
				CAN_CGCR_BUFFLOCK_Msk  
			| CAN_CGCR_TSTPEN_Msk    
			| CAN_CGCR_DDIR_Msk      
			| CAN_CGCR_LO_Msk        
//			| CAN_CGCR_LOOPBACK_Msk  //no loopback here
			| CAN_CGCR_INTERNAL_Msk  
			| CAN_CGCR_DIAGEN_Msk    
			| CAN_CGCR_EIT_Msk
      | CAN_CGCR_CANEN_Msk     
  		| CAN_CGCR_CRX_Msk       
  		| CAN_CGCR_CTX_Msk;  
// 		| CAN_CGCR_IGNACK_Msk
//  uint32_t CICEN; //EICEN|ICEN[14:0] //disable ISR mode for now
 
  myCANCfg.CTIM_TSEG2 = (6UL-1UL);  //6 time quanta 
  myCANCfg.CTIM_TSEG1 = (4UL-1UL);  //4 time quanta
  myCANCfg.CTIM_SJW   = (4UL-1UL);  //4 time quanta
  myCANCfg.CTIM_PSC   = (2UL-1UL);  //CAN PreScalar=2
  myCANCfg.CICEN      = (BITMASK(ALLF,14,0)<<CAN_CICEN_ICEN_Pos)
                       |(CAN_CICEN_EICEN_Msk);

//tx on CAN0, RX on CAN1
  HAL_Can_Setup_gMask(VOR_CAN0,HAL_Can_Make_maskBX(dontCareIDNone, 1, 1));
  HAL_Can_Setup_bMask(VOR_CAN0,HAL_Can_Make_maskBX(dontCareIDNone, 1, 1));
  
  HAL_Can_Setup_gMask(VOR_CAN1,HAL_Can_Make_maskBX(dontCareIDNone, 1, 1));
  HAL_Can_Setup_bMask(VOR_CAN1,HAL_Can_Make_maskBX(dontCareIDNone, 1, 1));
  
  //set everyone to rx_not_ready
  HAL_Can_ClearBuffer((can_cmb_t *)&VOR_CAN0->CNSTAT_CMB0 , 14);
  HAL_Can_ClearBuffer((can_cmb_t *)&VOR_CAN1->CNSTAT_CMB0 , 14);
  
  //config some rx buf here
  HAL_Can_Setup(VOR_CAN0, (const can_config_t *)&myCANCfg, CAN0_IRQn);
  HAL_Can_Setup(VOR_CAN1, (const can_config_t *)&myCANCfg, CAN1_IRQn);
  
  //set up RX buf0-2 on can0
  HAL_Can_ConfigCMB_Rx(id29Bits[0], en_can_cmb_msgtype_EXT29, (can_cmb_t *)&VOR_CAN0->CNSTAT_CMB0);
  HAL_Can_ConfigCMB_Rx(id29Bits[1], en_can_cmb_msgtype_EXT29, (can_cmb_t *)&VOR_CAN0->CNSTAT_CMB1);
  HAL_Can_ConfigCMB_Rx(id29Bits[2], en_can_cmb_msgtype_EXT29, (can_cmb_t *)&VOR_CAN0->CNSTAT_CMB2);

  HAL_Can_ConfigCMB_Rx(id29Bits[0], en_can_cmb_msgtype_EXT29, (can_cmb_t *)&VOR_CAN1->CNSTAT_CMB0);
  HAL_Can_ConfigCMB_Rx(id29Bits[1], en_can_cmb_msgtype_EXT29, (can_cmb_t *)&VOR_CAN1->CNSTAT_CMB1);
  HAL_Can_ConfigCMB_Rx(id29Bits[2], en_can_cmb_msgtype_EXT29, (can_cmb_t *)&VOR_CAN1->CNSTAT_CMB2);

  myTxPkt[0].id=id29Bits[0];
  myTxPkt[1].id=id29Bits[1]; 
  myTxPkt[2].id=id29Bits[2];
  
  //all extended, not Remote req
  myTxPkt[0].msgType=en_can_cmb_msgtype_EXT29;
  myTxPkt[1].msgType=en_can_cmb_msgtype_EXT29;
  myTxPkt[2].msgType=en_can_cmb_msgtype_EXT29;
  
  //max data
  myTxPkt[0].dataLengthBytes=8;
  myTxPkt[1].dataLengthBytes=8;
  myTxPkt[2].dataLengthBytes=8;
  
  for(i=0;i<4;i++){
    myTxPkt[0].data16[i]=0xa510+i;
    myTxPkt[1].data16[i]=0xa520+i;
    myTxPkt[2].data16[i]=0xa530+i;
  }

  //set up tX buf3-5 on can0, does not wait for reception
  //loopback should drop in rx buf0-2 
  HAL_Can_sendCanPkt((can_cmb_t *)&VOR_CAN0->CNSTAT_CMB3, (can_pkt_t *)&myTxPkt[0]);

    timeout=50000;
  while(((VOR_CAN0->CNSTAT_CMB3|0xff)&CAN_CNSTAT_CMB0_ST_Msk)!=en_can_cmb_cnstat_st_TX_NOT_ACTIVE){
    --timeout;
    if(timeout==0){
      return 1;    
    }      
  }
  
  timeout=50000;
  //wait for rx status to indicate it's received
  while(((VOR_CAN0->CNSTAT_CMB0|0xff)&CAN_CNSTAT_CMB0_ST_Msk)!=en_can_cmb_cnstat_st_RX_FULL){
    --timeout;
    if(timeout==0){
      return 2;
    }      
  }
  HAL_Can_getCanPkt((can_cmb_t *)&VOR_CAN0->CNSTAT_CMB0, (can_pkt_t *)&myRxPkt[0]);
  if(myRxPkt[0].dataLengthBytes!=8){
    return 3;//wrong data length
  }
  if((myRxPkt[0].data16[i]&0xffff)!=0xa510){
    return 4;//wrong data
  } 
  return 0;//a-ok
}  

/*****************************************************************************/ 
/* End of file                                                               */ 
/*****************************************************************************/
