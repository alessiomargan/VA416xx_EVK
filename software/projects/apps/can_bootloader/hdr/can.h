/***************************************************************************************
 * @file     can.h
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
 
#ifndef __CAN_H
#define __CAN_H

/*****************************************************************************/
/* Include files                                                             */
/*****************************************************************************/

#include "va416xx_hal_canbus.h"

/*****************************************************************************/
/* Global function prototypes                                                */
/*****************************************************************************/

extern void CAN_BootloaderInit(void);
extern bool CAN_IsRxMsgAvailable(void);
extern bool CAN_ReceiveMessage(uint32_t *pId, uint8_t *pData, uint8_t *pLen);
extern bool CAN_SendMessage(uint32_t id, uint8_t *pData, uint8_t len);

/*****************************************************************************/
/* End of file                                                               */
/*****************************************************************************/
#endif /* __CAN_H */
