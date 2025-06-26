/***************************************************************************************
 * @file     va416xx_hal_spw.h
 * @version  V0.1
 * @date     20 July 2021
 *
 * @note
 * VORAGO Technologies
 *
 * @note
 * Copyright (c) 2013-2021 VORAGO Technologies.
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
 
#ifndef __VA416XX_HAL_SPW_H
#define __VA416XX_HAL_SPW_H

/*****************************************************************************/ 
/* Include files                                                             */ 
/*****************************************************************************/
#include <stdbool.h>    // for the bool type
#include <stdint.h>     // for the numeric integer types
#include <string.h>     // for memcpy and memset functions
#include <stdio.h>      // for the definition of NULL

#include "va416xx_hal.h"

/*****************************************************************************/ 
/* Global pre-processor symbols/macros ('#define')                           */ 
/*****************************************************************************/

#define MAX_PAYLOAD 254


/*definitions for indices for extract length from RP command packet*/
//#define GRND_CMD_HEADER_LENGTH_MSB_INDEX 4
//#define GRND_CMD_HEADER_LENGTH_LSB_INDEX 5

#define SPW_STS_RESET (VOR_SPW->STS |= (0x000003df))  // Reset spacewire status register
#define GET_LINK_STATE ((LinkState_t)((VOR_SPW->STS & SPW_STS_LS_Msk) >> SPW_STS_LS_Pos))

#define MAX_SPACEWIRE_DWORD_BUFFER_SIZE     44
#define MAX_RMAP_DATA_SIZE                  128  // bytes


#define SPW_TXDESC_HDRLEN_MASK      (0xff)
#define SPW_TXDESC_HEADER_SIZE      40

/** Max number of bytes to use for Reply Address field */
#define RMAP_MAX_REPLY_ADDRESS_BYTES        12  // 12 bytes, as per ECSS-E-ST-50-52C

/** Max Data Bytes in a RMAP command or Reply packet. This is application specific. Can be tuned. */ 
#define RMAP_PACKET_MAX_DATA_BYTES          64   

/** Max Packet size (header + data). Application specific */
#define RMAP_MAX_PACKET_SIZE (RMAP_PACKET_MAX_DATA_BYTES + 64)

/** RMAP Protocol ID */
#define RMAP_PROTOCOL_ID            (0x01)

#define INSTRUCTION_CMDCODE_MASK    0x3c
#define RMW_CMDCODE                 0x1c

#define APP_DESTINATION_KEY     (0x5a)

/** Max Data Length value for a Read-Modify-Write command */
#define RMAP_MAX_RMW_DATA_LENGTH          8  

/** Min Packet length of RMAP Reply packet */
#define RMAP_REPLY_MIN_PACKET_LENGTH      8  // (including header CRC) 

/* RMAP Reply Bytes */
#define RMAP_REPLY_SRC_ADDRESS       0
#define RMAP_REPLY_PID               1
#define RMAP_REPLY_INSTRUCTION       2
#define RMAP_REPLY_STATUS            3
#define RMAP_REPLY_TARGET_ADDRESS    4
#define RMAP_REPLY_TRANS_ID_0        5
#define RMAP_REPLY_TRANS_ID_1        6
#define RMAP_REPLY_RESERVED_HDR_CRC  7  // Header CRC for Write reply, Reserved for other
#define RMAP_REPLY_DATA_LENGTH_0     8
#define RMAP_REPLY_DATA_LENGTH_1     9
#define RMAP_REPLY_DATA_LENGTH_2     10
#define RMAP_REPLY_HEADER_CRC        11  // Header CRC for Read or RMW replies
#define RMAP_REPLY_DATA_START        12


// Instruction field
#define COMMAND_PACKET  0x01
#define REPLY_PACKET    0x00
#define INCREMENT_ADDR  0x04
#define REPLY_CMDCODE   0x08
#define VERIFY_BEFWRT   0x10
#define WRITE_CMDCODE   0X20

#define REPLY_ADDR_LENGTH_0     0x0
#define REPLY_ADDR_LENGTH_4     0x1
#define REPLY_ADDR_LENGTH_8     0x2
#define REPLY_ADDR_LENGTH_12    0x3

#define RMAP_RMW_DATA_LENGTH_2  2
#define RMAP_RMW_DATA_LENGTH_4  4
#define RMAP_RMW_DATA_LENGTH_8  8


#define RMAP_MSG_OPTIONS_MASK    (RMAP_MSG_OPTIONS_INCREMENT |  \
                                  RMAP_MSG_OPTIONS_REPLY     |  \
                                  RMAP_MSG_OPTIONS_VERIFY) 


#define MASK_16_BITS        0xFFFF  // MASK for 16 bits
#define BYTE_MASK           0xFF    // Mask for 8 bits
#define NIBBLE_MASK         0xF     // Mask for 4 bits
#define MASK_2BITS          0x3     // Mask for 2 bit
#define BIT_MASK            0x1     // Mask for 1 bit

#define SHIFT_24BITS        24      // Shift for 24 bits
#define SHIFT_16BITS        16      // Shift for 16 bits
#define BYTE_SHIFT          8       // Shift for 8 bits
#define FOUR_BIT_SHIFT      4       // Shift for 4 bits
#define TWO_BIT_SHIFT       2       // shift for 2 bits
#define NUM_BYTES_WORD      4       // number of bytes in a word
#define NUM_BYTES_HALFWORD  2       // Number of bytes in a halfword (16-bits)


#define RXDESC_CTRL_TR_Msk      (0x80000000UL)
#define RXDESC_CTRL_TR_Pos      (31UL)
#define RXDESC_CTRL_DC_Msk      (0x40000000UL)
#define RXDESC_CTRL_DC_Pos      (30UL)
#define RXDESC_CTRL_HC_Msk      (0x20000000UL)
#define RXDESC_CTRL_HC_Pos      (29UL)
#define RXDESC_CTRL_EP_Msk      (0x10000000UL)
#define RXDESC_CTRL_EP_Pos      (28UL)
#define RXDESC_CTRL_IE_Msk      (0x08000000UL)
#define RXDESC_CTRL_IE_Pos      (27UL)
#define RXDESC_CTRL_WR_Msk      (0x04000000UL)
#define RXDESC_CTRL_WR_Pos      (26UL)
#define RXDESC_CTRL_EN_Msk      (0x02000000UL)
#define RXDESC_CTRL_EN_Pos      (25UL)

#define RXDESC_CTRL_PACKETLENGTH_Msk    (0x01FFFFFFUL)

// this enables a receive descriptor and enables its interrupt
#define RXDESC_INITENABLE   (RXDESC_CTRL_EN_Msk | RXDESC_CTRL_IE_Msk)

// Transmit Descriptor Control register
#define TXDESC_CTRL_CRCT_Msk        (0x000C0000UL)
#define TXDESC_CTRL_DC_Pos          (17UL)
#define TXDESC_CTRL_DC_Msk          (0x00020000UL)
#define TXDESC_CTRL_HC_Pos          (16UL)
#define TXDESC_CTRL_HC_Msk          (0x00010000UL)
#define TXDESC_CTRL_LE_Pos          (15UL)
#define TXDESC_CTRL_LE_Msk          (0x00008000UL)
#define TXDESC_CTRL_IE_Pos          (14UL)
#define TXDESC_CTRL_IE_Msk          (0x00004000UL)
#define TXDESC_CTRL_WR_Pos          (13UL)
#define TXDESC_CTRL_WR_Msk          (0x00002000UL)
#define TXDESC_CTRL_EN_Pos          (12UL)
#define TXDESC_CTRL_EN_Msk          (0x00001000UL)
#define TXDESC_CTRL_NONCRCLEN_Msk   (0x00000F00UL)
#define TXDESC_CTRL_HEADERLEN_Msk   (0x000000FFUL)

/*****************************************************************************/
/* Global type definitions ('typedef')                                       */ 
/*****************************************************************************/

// space wire link status
typedef enum
{
    SPW_LINK_ERROR_RESET,
    SPW_LINK_ERROR_WAIT,
    SPW_LINK_READY,
    SPW_LINK_STARTED,
    SPW_LINK_CONNECTING,
    SPW_LINK_RUN,
    SPW_LINK_UNDEFINED1,
    SPW_LINK_UNDEFINED2,
}LinkState_t;

/** RMAP Transaction Command Message */
typedef struct _rmapTransaction {
    
    uint8_t targetSpWaddr[3];
    uint8_t targetLogicalAddress;
    
    uint8_t instructionField;
 	uint8_t	destKey;         // Destination key
    uint16_t transactionIdent;
    uint8_t initiatorLogicalAddr;

   /** Length of the Source Reply address (in number of bytes). 
    * Note: As per ECSS-E-ST-50-52C, this value should be 0, 4, 8 or 12*/
    uint32_t replyAddressLength; 

    /** Reply Address */
    uint8_t replyAddress[RMAP_MAX_REPLY_ADDRESS_BYTES];

    /** Initiator Logical Address */
    uint8_t sourceLogicalAddress;
    uint8_t extendedAddr;
    uint32_t memoryAddr;

    /** Data Length, 24-bits. Note that for a read-modify-write command, 
      only the values 0x00, 0x02, 0x04, 0x06 and 0x08 are valid, and the
      length shall include the size of the mask field */
    uint32_t dataLength;	

    /** Data bytes for a write command. */
    uint8_t writeData[RMAP_PACKET_MAX_DATA_BYTES];	

    /** 1, 2 or 4 bytes data for a read-modify-write command. 
    *  The length of the data is dataLength/2 */
    uint32_t readModWriteData;	

    /** 1, 2 or 4 bytes data mask for a read-modify-write command. 
    *  The length of the mask is dataLength/2 */
	uint32_t readModWriteMask;
    
    volatile uint32_t* timerAddress;  /** Address of the decrementing timer */
    uint32_t timeout;                 /** Wait timeout value in timer counts */

} rmapTransaction_t;

/**@brief Spacewire Transmit descriptors
 *
 */
typedef struct _SpWtransmitDescriptor {    
    uint32_t flagsHdrLen;   // word 0 offset 0x0
    const uint8_t *header;  // word 1 offset 0x4
    uint32_t dataLength;    // word 2 offset 0x8
    const uint8_t *data;    // word 3 offset 0xc
} SpWtxDesc_t;


typedef struct _SpWrecieveDescriptor {
    uint32_t flagsPacketLen;    // word 0 offset 0x0
    uint32_t *packetAddr;       // word 1 offset 0x4
}SpWrxDesc_t;



/*****************************************************************************/
/* Global variable declarations ('extern', definition in C source)           */
/*****************************************************************************/


/*****************************************************************************/ 
/* Global function prototypes ('extern', definition in C source)             */ 
/*****************************************************************************/

extern bool spwTransmit(SpWtxDesc_t *);
extern void spwReceive(uint32_t *);
extern uint32_t createRmapTransaction(rmapTransaction_t *);
extern bool spaceWireInit(void);





/*****************************************************************************/ 
/* End of file                                                               */ 
/*****************************************************************************/
#endif /* __VA416XX_HAL_SPW_H */
