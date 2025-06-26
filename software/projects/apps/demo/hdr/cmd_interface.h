/***************************************************************************************
 * @file     cmd_interface.h
 * @version  V2.05
 * @date     10 January 2024
 *
 * @note
 * VORAGO Technologies
 *
 * @note
 * Copyright (c) 2013-2024 VORAGO Technologies.
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
 
#ifndef __CMD_INTERF_H
#define __CMD_INTERF_H

/*****************************************************************************/ 
/* Include files                                                             */ 
/*****************************************************************************/

#include "va416xx_hal.h"

/*****************************************************************************/ 
/* Global pre-processor symbols/macros ('#define')                           */ 
/*****************************************************************************/

#define CMD_PARAM_MAX (4)
#define RESP_PARAM_MAX (4)

#define RESP_PARAM_SIZE_UNDEFINED (0xff)
//#define CMD_MAX_STR_SIZE (128)
//#define RESP_MAX_STR_SIZE (128)

#define CMD_START_CHAR          '$'
#define CMD_TERMINATOR_STR      "\r\n"
#define CMD_TERMINATOR_CHAR_CR  '\r'
#define CMD_TERMINATOR_CHAR_LF  '\n'
#define RESP_TERMINATOR_CHAR    '\n'

#define OKMSG() txStr("\n$OK ")
#define OKMSG_TERM() txStr("\n$OK\r\n")
#define ERRMSG() txStr("\n$ERROR ")
#define ERRMSG_TERM() txStr("\n$ERROR\r\n")
#define TERMMSG() txStr(CMD_TERMINATOR_STR)

// 1: Echoes RTT input to output buffer. 0: Does not echo
#define RTT_ECHO (1)

/*****************************************************************************/
/* Global type definitions ('typedef')                                       */ 
/*****************************************************************************/

typedef enum TxDest
{
  TxDest_Uart0 = 0,
  TxDest_Uart1 = 1,
  TxDest_Uart2 = 2,
  TxDest_RTT   = 3,
  TxDest_None  = 4
} TxDest_t;

typedef enum RxCmdSrc
{
	RxCmdSrc_Uart0 = 0,
  RxCmdSrc_Uart1 = 1,
  RxCmdSrc_Uart2 = 2,
  RxCmdSrc_RTT   = 3,
  RxCmdSrc_None  = 4,
  RxCmdSrc_Error = 5
} RxCmdSrc_t;

typedef enum CmdEnum
{
  CmdEnum_None          =0,
  CmdEnum_ReadPort      =1,
  CmdEnum_SetPort       =2,
  CmdEnum_ReadADC       =3,
  CmdEnum_SetDAC        =4,
  CmdEnum_FramWrite     =5,
  CmdEnum_FramRead      =6,
  CmdEnum_FramAuxWrite  =7,
  CmdEnum_FramAuxRead   =8,
  CmdEnum_EBIInit       =9,
  CmdEnum_EBIWrite      =10,
  CmdEnum_EBIRead       =11,
  CmdEnum_CanTest       =12,
  CmdEnum_PhyTest       =13,
  CmdEnum_SpWTest       =14,
  CmdEnum_I2CTest       =15,
  CmdEnum_GetAccel      =16,
  CmdEnum_GetBoardTemp  =17,
  CmdEnum_FWVersion     =18,
  CmdEnum_SoftReset     =19,
  CmdEnum_ClkPLL        =20,
  CmdEnum_ClkHBO        =21,
  CmdEnum_ClkExt        =22,
  CmdEnum_ClkXtal       =23,
  CmdEnum_Clk           =24,
  CmdEnum_AdcTest       =25,
  CmdEnum_DacSine       =26,
  CmdEnum_SpiInt        =27,
	CmdEnum_SpiDma        =28,
  CmdEnum_DieTemp       =29,
  CmdEnum_Help          =30,
  CmdEnum_End           =31
} CmdEnum_t;

/*****************************************************************************/
/* Global variable declarations ('extern', definition in C source)           */
/*****************************************************************************/

extern const char *          CmdStrArray[(uint32_t)CmdEnum_End];
extern const char *        UsageStrArray[(uint32_t)CmdEnum_End];
extern const char *         DescStrArray[(uint32_t)CmdEnum_End];
extern const uint32_t CmdParamCountArray[(uint32_t)CmdEnum_End];

/*****************************************************************************/ 
/* Global function prototypes ('extern', definition in C source)             */ 
/*****************************************************************************/

extern void CommandInterfaceMainTask(void);
extern void OnceEverySecondTasks(void);

/*****************************************************************************/ 
/* End of file                                                               */ 
/*****************************************************************************/
#endif /* __CMD_INTERF_H */
