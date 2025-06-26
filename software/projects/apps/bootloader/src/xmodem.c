/***************************************************************************************
 * @file     xmodem.c
 * @version  V1.41
 * @date     31 January 2022
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

#include <stdio.h>

#include "va416xx.h"
#include "va416xx_hal.h"

#include "xmodem.h"
#include "uart.h"
#include "crc.h"
#include "board.h"
#include "main.h"

/*****************************************************************************/
/* Local pre-processor symbols/macros ('#define')                            */
/*****************************************************************************/

/** byte alignment macro */
#define ALIGNMENT(sz,align) (((sz)+((align)-1))&(~((align)-1)))

/** value to indicate packet verify failure */
#define XMODEM_FAIL_VALUE -1

/** value to indicate packet verify success */
#define XMODEM_PASS_VALUE 0

/** value in milliseconds between repeated sends of start transfer packet ('C') */
#define START_TRANSMISSION_REPEAT_MSEC 1000

/*****************************************************************************/
/* Local variable definitions ('static')                                     */
/*****************************************************************************/

// Packet storage. Double buffered version
static uint8_t m_au8RawPacket[2][ALIGNMENT(sizeof(stc_xmodem_packet_t),4)];

/*****************************************************************************/
/* Local function prototypes ('static')                                      */
/*****************************************************************************/

__INLINE static int32_t VerifyPacketChecksum(const stc_xmodem_packet_t* const pstcPacket, \
                                             const uint32_t u32SequenceNumber);

static void    send(const uint8_t uart, const uint8_t b);
static uint8_t receive(const uint8_t u8Uart);
static void    memcpy16(uint16_t* to, const uint16_t* from, size_t ln);

/*****************************************************************************/
/* Function implementation - global ('extern') and local ('static')          */
/*****************************************************************************/

/******************************************************************************
 * @brief Verifies checksum, packet numbering and
 * @param pkt The packet to verify
 * @param sequenceNumber The current sequence number.
 * @returns -1 on packet error, 0 otherwise
 *****************************************************************************/
__INLINE static int32_t VerifyPacketChecksum(const stc_xmodem_packet_t* const pstcPacket, \
                                             const uint32_t u32SequenceNumber)
{
  uint16_t u16PacketCRC;
  uint16_t u16CalculatedCRC;

  // Check the packet number integrity
  if (pstcPacket->u8PacketNumber + pstcPacket->u8PacketNumberC != 255)
  {
    return XMODEM_FAIL_VALUE;
  }

  // Check that the packet number matches the excpected packet number
  if (pstcPacket->u8PacketNumber != (u32SequenceNumber % 256))
  {
    return XMODEM_FAIL_VALUE;
  }

	// Calculate the CRC
  u16CalculatedCRC = CalculateCRC16((uint8_t *) pstcPacket->au8Data, (uint8_t *) &(pstcPacket->u8CrcHigh));
  u16PacketCRC = pstcPacket->u8CrcHigh << 8 | pstcPacket->u8CrcLow;

  // Check the CRC
  if (u16CalculatedCRC != u16PacketCRC)
  {
    return XMODEM_FAIL_VALUE;
  }
  return XMODEM_PASS_VALUE;
}

/******************************************************************************
 * @brief sends a byte over the specified uart
 * @param uart 0 = uart0, 1 = uart1
 * @param b the byte to send
 * @returns void
 *****************************************************************************/
static void send(const uint8_t uart, const uint8_t b)
{
  if(uart == 0){
    UART0TxByte(b);
  } else if(uart == 1){
    UART1TxByte(b);
  }
}

/******************************************************************************
 * @brief receives a byte over the specified uart
 * @param u8uart 0 = uart0, 1 = uart1
 * @returns the received byte, or 0x0 if timed out
 *****************************************************************************/
static uint8_t receive(const uint8_t u8Uart)
{
  if(u8Uart == 0){
    return UART0RxByte();
  } else if(u8Uart == 1){
    return UART1RxByte();
  }
	return 0;
}

/******************************************************************************
 * @brief copies memory in 16 bit chunks (necessary for IRAM)
 * @param to - the destination
 * @param from - the source
 * @param ln - size in bytes
 * @returns the received byte, or 0x0 if timed out
 *****************************************************************************/
static void memcpy16(uint16_t* to, const uint16_t* from, size_t ln)
{
  ln++;
  ln/=2;
  while(ln){
    *(to++)=*(from++);
    ln--;
  }
}

/******************************************************************************
 * @brief Starts a XMODEM download
 *
 * @param uint8_t* pu8BaseAddress
 *   The code RAM address to start writing to (usually the start of user app space)
 *
 * @param uint8_t* pu8EndAddress
 *   The last address - used to make sure we don't overrun our available space
 *   Usually the end of code RAM space, 0x20000
 *
 * @return size_t - the number of bytes received, or zero if there was an error
 *
 *****************************************************************************/
size_t XMODEM_ReceiveFile(uint8_t* pu8BaseAddress, uint8_t* pu8EndAddress, uint8_t u8Uart)
{
  stc_xmodem_packet_t* pstcPacket;
  uint32_t u32Idx;
  uint32_t u32SequenceNumber = 1;
	uint8_t* pu8CheckAddress;
	uint64_t u64Now;

  /* Send a start transmission packet. Wait for a response.
	   In PC_MODE, keep sending start packets until the transmission
     starts. In MCU<->MCU mode, send the start once.	*/
  while (1)
  {
    send(u8Uart, XMODEM_NCG);
		u64Now = HAL_time_ms;
    while(HAL_time_ms < (u64Now + START_TRANSMISSION_REPEAT_MSEC))
    {
      if(0 == u8Uart){
        if (UART0IsRxByteAvailable())
        {
          // Goto is evil, but necessary to break from outer loop
          goto xmodem_transfer;
        }
      } else if(1 == u8Uart){
        if (UART1IsRxByteAvailable())
        {
          // Goto is evil, but necessary to break from outer loop
          goto xmodem_transfer;
        }
      }
			// feed watchdog
			WDFEED();
    }
  }
xmodem_transfer:
	// First, enable writing to code RAM space
	VOR_SYSCONFIG->ROM_PROT |= SYSCONFIG_ROM_PROT_WREN_Msk;
  while (1)
  {
    // Swap buffer for packet buffer.
    pstcPacket = (stc_xmodem_packet_t *)m_au8RawPacket[u32SequenceNumber & 1];

    // Fetch the first byte of the packet explicitly, as it defines the
    // rest of the packet.
    pstcPacket->u8Header = receive(u8Uart);

    // Check for end of transfer.
    if (pstcPacket->u8Header == XMODEM_EOT)
    {
      // Acknowledge End of transfer.
      send(u8Uart, XMODEM_ACK);
      break;
    }

    // If the header is not a start of header (SOH), then NAK
    if (pstcPacket->u8Header != XMODEM_SOH)
    {
      send(u8Uart, XMODEM_NAK);
      continue;
    }

    // Fill the remaining bytes packet.
    // Byte 0 is padding, byte 1 is header.
    for (u32Idx = 2; u32Idx < sizeof(stc_xmodem_packet_t); u32Idx++)
    {
      *(((uint8_t *) pstcPacket) + u32Idx) = receive(u8Uart);
      WDFEED();
    }

    if (VerifyPacketChecksum(pstcPacket, u32SequenceNumber) != 0)
    {
      // On a malformed packet, we send a NAK, and start over.
      send(u8Uart, XMODEM_NAK);
      continue;
    }

		// Write data to code RAM
		pu8CheckAddress = pu8BaseAddress + ((u32SequenceNumber) * XMODEM_DATA_SIZE);
		if (pu8CheckAddress <= pu8EndAddress)
		{
		  memcpy16((uint16_t*)(pu8BaseAddress + ((u32SequenceNumber - 1) * XMODEM_DATA_SIZE)),(const uint16_t*)pstcPacket->au8Data,XMODEM_DATA_SIZE);
		}

    u32SequenceNumber++;
    // Send ACK.
    send(u8Uart, XMODEM_ACK);

		// feed watchdog
	  WDFEED();
  }

	// We're done, so disable writing to code RAM space
	VOR_SYSCONFIG->ROM_PROT &= ~SYSCONFIG_ROM_PROT_WREN_Msk;
	return (u32SequenceNumber - 1) * XMODEM_DATA_SIZE;
}

/******************************************************************************
 * @brief Starts a XMODEM upload
 *
 * @param uint8_t* pu8BaseAddress
 *   The code RAM address to start sending from (usually the start of user app space)
 *
 * @param size_t szLength
 *   The length in bytes of the file to send
 *
 * @return size_t - the number of bytes sent, or zero if there was an error
 *
 *****************************************************************************/
size_t XMODEM_SendFile(uint8_t* pu8BaseAddress, size_t szLength, uint8_t u8Uart)
{
	// TODO: implement
	return 0;
}

/*****************************************************************************/
/* End of file                                                               */
/*****************************************************************************/
