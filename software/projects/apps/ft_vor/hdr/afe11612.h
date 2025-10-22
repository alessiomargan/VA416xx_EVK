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

#include "va416xx_hal_canbus.h"

#define SPI_WORDLEN         (8u)
#define SPI_WORDS           (3u)

typedef enum {
    TXFER_IDLE,
    TXFER_RD_PROGRESS_1,
    TXFER_RD_PROGRESS_2,
    TXFER_RD_COMPLETE,
    TXFER_WR_PROGRESS_1,
    TXFER_WR_COMPLETE,
    TXFER_ERROR
} transferState_t;

typedef struct {
    uint8_t reg;
    uint16_t value;
} afe11612_reg_t;

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
 * Test function to read and verify the AFE11612 Device ID register.
 * Expected value is 0x1220.
 * @return true if Device ID matches 0x1220, false otherwise or on SPI error.
 */
bool AFE11612_testDeviceId(void);

/**
 * Write all configuration registers from afe11612_config[] array.
 * @return Number of successfully written registers.
 */
uint8_t AFE11612_WriteConfig(void);

/**
 * Read all runtime value registers into afe11612_values[] array.
 * Reads temperatures, ADC inputs, and DAC outputs.
 * @return Number of successfully read registers.
 */
uint8_t AFE11612_ReadValues(void);

/**
 * Read selected runtime value registers by array index.
 * Only reads the specified indices from afe11612_values[] array.
 * @param indices Pointer to array of indices into afe11612_values[] (0..AFE11612_VALUES_COUNT-1).
 *                Array must be terminated with a negative value (e.g., -1).
 * @return Number of successfully read registers.
 * 
 * Example usage:
 *   int8_t indices[] = {0, 1, 2, 23, -1};  // Read temps and STATUS
 *   AFE11612_ReadValuesByIndex(indices);
 */
uint8_t AFE11612_ReadValuesByIndex(const int8_t *indices);

/**
 * Convert AFE11612 temperature register value to centidegrees Celsius (hundredths of a degree).
 * Temperature is stored as 12-bit 2's complement in bits [15:4], bits [3:0] are reserved.
 * Resolution is 0.125°C/LSB (12.5 centidegrees/LSB). Result scaled by 100 for full precision.
 * Range: -4500 to +15500 centidegrees (-45°C to +155°C).
 * @param raw_value Raw 16-bit value from temperature register (bits [15:4] contain data)
 * @return Temperature in centidegrees Celsius (e.g., 2675 = 26.75°C, -4500 = -45.00°C)
 * 
 * Example usage:
 *   int16_t temp_cdeg = AFE11612_ConvertTemp(afe11612_values[0].value);  // Convert LT__TEMP
 *   printf("Temperature: %d.%02d°C\n", temp_cdeg / 100, abs(temp_cdeg % 100));
 */
int16_t AFE11612_ConvertTemp(uint16_t raw_value);

/**
 * @brief Process a CAN request for the AFE11612.
 *
 * @param rxPkt Pointer to the received CAN packet.
 * @param respPkt Pointer to the response CAN packet.
 */
void AFE11612_ProcessRequest(const can_pkt_t *rxPkt, can_pkt_t *respPkt);

#endif