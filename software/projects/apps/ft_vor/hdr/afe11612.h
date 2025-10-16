/**
 * @file afe11612.h
 * @brief AFE11612-SEP Radiation-Tolerant, Analog Monitor and Controller
 * with Multichannel ADC, DACs, and Temperature Sensors
 *
 * This file defines the interface to the AFE11612
 *
 * @author Alessio Margan
 * @date October 13, 2025
 *
 * @version 1.0
 *
 * @copyright Copyright (c) 2025 IIT
 *
 * @note This implementation requires the VA416xx HAL SPI, and CAN drivers
 */

#ifndef __AFE11612_H__
#define __AFE11612_H__

#include <stdint.h>

#include <afe11612_regs.h>

#define SPI_WORDLEN         (8u)
#define SPI_WORDS           (3u)

/*
 * The device is entirely controlled by registers.
 * Reading from and writing to these registers is accomplished by issuing a 24-bit operation
 * bit 23 : R/W Indicates a read from or a write to the addressed register
 *  - bit 0 = The write operation is set and data are written to the specified register
 *  - bit 1 = A read operation where bits Addr[6:0] select the register to be read. The remaining bits are don't care. Data read from
 *    the selected register appear on the SDO pin in the next SPI cycle
 * bits [22:16] : Addr[6:0] Register address; specifies which register is accessed
 * bits [15:0] : data
 */
/**
 * SPI command packed as 3 bytes (1 byte address + 2 bytes data).
 * Access as named fields or as a raw byte array for SPI transfers.
 * Note: The uint16_t 'data' layout in bytes depends on target endianness.
 */
typedef union {
    uint8_t spiword[SPI_WORDS];        /* Raw bytes for SPI (MSB/LSB ordering depends on endianness) */
} afe11612_spi_cmd_t;


void ConfigureAFE11612(void);
/**
 * Read a 16-bit register from the AFE11612.
 * @param reg 7-bit register address (0x00..0x7F). The read bit will be set by the function.
 * @param out pointer to uint16_t where the register value will be written (MSB: high byte).
 * @return true on success, false on SPI error.
 */
bool AFE11612_ReadReg(uint8_t reg, uint16_t *out);

/**
 * Write a 16-bit register in the AFE11612.
 * @param reg 7-bit register address (0x00..0x7F). The read bit will be cleared by the function.
 * @param value 16-bit value to write (MSB: high byte).
 * @return true on success, false on SPI error.
 */
bool AFE11612_WriteReg(uint8_t reg, uint16_t value);

/**
 * Test function to read and print the AFE11612 Device ID register.
 * Expected value is 0x1220.
 */
void AFE11612_testDeviceId(void);

#endif