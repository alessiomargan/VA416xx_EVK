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

/* Configuration array indices - use these instead of magic numbers */
typedef enum {
    AFE_CONFIG_IDX_TEMP__CONFIG = 0,        /* 0x0A - Temperature sensor config */
    AFE_CONFIG_IDX_TEMP__CONV__RATE,        /* 0x0B - Temperature conversion rate */
    AFE_CONFIG_IDX_D1_N__ADJUST,            /* 0x21 - D1_N adjustment */
    AFE_CONFIG_IDX_D2_N__ADJUST,            /* 0x22 - D2_N adjustment */
    AFE_CONFIG_IDX_DAC_0__CLR,              /* 0x3F - DAC 0 clear value */
    AFE_CONFIG_IDX_DAC_1__CLR,              /* 0x40 - DAC 1 clear value */
    AFE_CONFIG_IDX_DAC_2__CLR,              /* 0x41 - DAC 2 clear value */
    AFE_CONFIG_IDX_DAC_3__CLR,              /* 0x42 - DAC 3 clear value */
    AFE_CONFIG_IDX_DAC_4__CLR,              /* 0x43 - DAC 4 clear value */
    AFE_CONFIG_IDX_DAC_5__CLR,              /* 0x44 - DAC 5 clear value */
    AFE_CONFIG_IDX_DAC_6__CLR,              /* 0x45 - DAC 6 clear value */
    AFE_CONFIG_IDX_DAC_7__CLR,              /* 0x46 - DAC 7 clear value */
    AFE_CONFIG_IDX_DAC_8__CLR,              /* 0x47 - DAC 8 clear value */
    AFE_CONFIG_IDX_DAC_9__CLR,              /* 0x48 - DAC 9 clear value */
    AFE_CONFIG_IDX_DAC_10__CLR,             /* 0x49 - DAC 10 clear value */
    AFE_CONFIG_IDX_DAC_11__CLR,             /* 0x4A - DAC 11 clear value */
    AFE_CONFIG_IDX_AFE__CONFIG_0,           /* 0x4C - AFE Configuration Register 0 */
    AFE_CONFIG_IDX_AFE__CONFIG_1,           /* 0x4D - AFE Configuration Register 1 */
    AFE_CONFIG_IDX_ALR_CTRL,                /* 0x4E - Alarm Control Register */
    AFE_CONFIG_IDX_ADC_CH0,                 /* 0x50 - ADC Channel 0 configuration */
    AFE_CONFIG_IDX_ADC_CH1,                 /* 0x51 - ADC Channel 1 configuration */
    AFE_CONFIG_IDX_ADC_GAIN,                /* 0x52 - ADC Gain */
    AFE_CONFIG_IDX_AUTO_DAC__CLR__SOURCE,   /* 0x53 - Auto DAC Clear Source */
    AFE_CONFIG_IDX_AUTO_DAC__CLR_EN,        /* 0x54 - Auto DAC Clear Enable */
    AFE_CONFIG_IDX_SW_DAC__CLR,             /* 0x55 - Software DAC Clear */
    AFE_CONFIG_IDX_HW_DAC__CLR_EN_0,        /* 0x56 - Hardware DAC Clear Enable 0 */
    AFE_CONFIG_IDX_HW_DAC__CLR_EN_1,        /* 0x57 - Hardware DAC Clear Enable 1 */
    AFE_CONFIG_IDX_DAC_CONFIG,              /* 0x58 - DAC Configuration */
    AFE_CONFIG_IDX_DAC_GAIN,                /* 0x59 - DAC Gain */
    AFE_CONFIG_IDX_IN_0__HIGH__THRESHOLD,   /* 0x5A - IN_0 High Threshold */
    AFE_CONFIG_IDX_IN_0__LOW__THRESHOLD,    /* 0x5B - IN_0 Low Threshold */
    AFE_CONFIG_IDX_IN_1__HIGH__THRESHOLD,   /* 0x5C - IN_1 High Threshold */
    AFE_CONFIG_IDX_IN_1__LOW__THRESHOLD,    /* 0x5D - IN_1 Low Threshold */
    AFE_CONFIG_IDX_IN_2__HIGH__THRESHOLD,   /* 0x5E - IN_2 High Threshold */
    AFE_CONFIG_IDX_IN_2__LOW__THRESHOLD,    /* 0x5F - IN_2 Low Threshold */
    AFE_CONFIG_IDX_IN_3__HIGH__THRESHOLD,   /* 0x60 - IN_3 High Threshold */
    AFE_CONFIG_IDX_IN_3__LOW__THRESHOLD,    /* 0x61 - IN_3 Low Threshold */
    AFE_CONFIG_IDX_LT__HIGH__THRESHOLD,     /* 0x62 - Local Temp High Threshold */
    AFE_CONFIG_IDX_LT__LOW__THRESHOLD,      /* 0x63 - Local Temp Low Threshold */
    AFE_CONFIG_IDX_D1__HIGH__THRESHOLD,     /* 0x64 - D1 Temp High Threshold */
    AFE_CONFIG_IDX_D1__LOW__THRESHOLD,      /* 0x65 - D1 Temp Low Threshold */
    AFE_CONFIG_IDX_D2__HIGH__THRESHOLD,     /* 0x66 - D2 Temp High Threshold */
    AFE_CONFIG_IDX_D2__LOW__THRESHOLD,      /* 0x67 - D2 Temp Low Threshold */
    AFE_CONFIG_IDX_HYST_0,                  /* 0x68 - Hysteresis 0 */
    AFE_CONFIG_IDX_HYST_1,                  /* 0x69 - Hysteresis 1 */
    AFE_CONFIG_IDX_HYST_2,                  /* 0x6A - Hysteresis 2 */
    AFE_CONFIG_IDX_PWR_DOWN,                /* 0x6B - Power Down */
    AFE_CONFIG_IDX_SW_RST                   /* 0x7C - Software Reset */
} afe11612_config_idx_t;

/* Runtime value array indices - use these instead of magic numbers */
typedef enum {
    AFE_VALUES_IDX_LT__TEMP = 0,            /* 0x00 - Local temperature */
    AFE_VALUES_IDX_D1__TEMP,                /* 0x01 - Remote temperature D1 */
    AFE_VALUES_IDX_D2__TEMP,                /* 0x02 - Remote temperature D2 */
    AFE_VALUES_IDX_ADC_0,                   /* 0x23 - ADC input 0 */
    AFE_VALUES_IDX_ADC_1,                   /* 0x24 - ADC input 1 */
    AFE_VALUES_IDX_ADC_2,                   /* 0x25 - ADC input 2 */
    AFE_VALUES_IDX_ADC_3,                   /* 0x26 - ADC input 3 */
    AFE_VALUES_IDX_ADC_4,                   /* 0x27 - ADC input 4 */
    AFE_VALUES_IDX_ADC_5,                   /* 0x28 - ADC input 5 */
    AFE_VALUES_IDX_ADC_6,                   /* 0x29 - ADC input 6 */
    AFE_VALUES_IDX_ADC_7,                   /* 0x2A - ADC input 7 */
    AFE_VALUES_IDX_DAC_0,                   /* 0x33 - DAC 0 output */
    AFE_VALUES_IDX_DAC_1,                   /* 0x34 - DAC 1 output */
    AFE_VALUES_IDX_DAC_2,                   /* 0x35 - DAC 2 output */
    AFE_VALUES_IDX_DAC_3,                   /* 0x36 - DAC 3 output */
    AFE_VALUES_IDX_DAC_4,                   /* 0x37 - DAC 4 output */
    AFE_VALUES_IDX_DAC_5,                   /* 0x38 - DAC 5 output */
    AFE_VALUES_IDX_DAC_6,                   /* 0x39 - DAC 6 output */
    AFE_VALUES_IDX_DAC_7,                   /* 0x3A - DAC 7 output */
    AFE_VALUES_IDX_DAC_8,                   /* 0x3B - DAC 8 output */
    AFE_VALUES_IDX_DAC_9,                   /* 0x3C - DAC 9 output */
    AFE_VALUES_IDX_DAC_10,                  /* 0x3D - DAC 10 output */
    AFE_VALUES_IDX_DAC_11,                  /* 0x3E - DAC 11 output */
    AFE_VALUES_IDX_STATUS,                  /* 0x4F - Status register */
    AFE_VALUES_IDX_DEVICE_ID                /* 0x6C - Device ID */
} afe11612_values_idx_t;

extern afe11612_reg_t afe11612_config[];
extern afe11612_reg_t afe11612_values[];

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
 *   int8_t indices[] = {AFE_VALUES_IDX_LT__TEMP, AFE_VALUES_IDX_D1__TEMP, 
 *                       AFE_VALUES_IDX_D2__TEMP, AFE_VALUES_IDX_STATUS, -1};
 *   AFE11612_ReadValuesByIndex(indices);
 */
uint8_t AFE11612_ReadValuesByIndex(const int8_t *indices);

/**
 * Write selected runtime value registers by array index.
 * Only writes the specified indices from afe11612_values[] array.
 * The values to write must already be set in the afe11612_values[] array.
 * @param indices Pointer to array of indices into afe11612_values[] (0..AFE11612_VALUES_COUNT-1).
 *                Array must be terminated with a negative value (e.g., -1).
 * @return Number of successfully written registers.
 * 
 * Example usage:
 *   afe11612_values[AFE_VALUES_IDX_DAC_0].value = 0x0800;  // Set DAC_0 to mid-scale
 *   afe11612_values[AFE_VALUES_IDX_DAC_1].value = 0x0FFF;  // Set DAC_1 to max
 *   int8_t indices[] = {AFE_VALUES_IDX_DAC_0, AFE_VALUES_IDX_DAC_1, -1};
 *   AFE11612_WriteValuesByIndex(indices);
 */
uint8_t AFE11612_WriteValuesByIndex(const int8_t *indices);

/**
 * Convert AFE11612 temperature register value to centidegrees Celsius (hundredths of a degree).
 * Temperature is stored as 12-bit 2's complement in bits [15:4], bits [3:0] are reserved.
 * Resolution is 0.125°C/LSB (12.5 centidegrees/LSB). Result scaled by 100 for full precision.
 * Range: -4500 to +15500 centidegrees (-45°C to +155°C).
 * @param raw_value Raw 16-bit value from temperature register (bits [15:4] contain data)
 * @return Temperature in centidegrees Celsius (e.g., 2675 = 26.75°C, -4500 = -45.00°C)
 * 
 * Example usage:
 *   int16_t temp_cdeg = AFE11612_ConvertTemp(afe11612_values[AFE_VALUES_IDX_LT__TEMP].value);
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

/**
 * @brief Set the ILDAC pin high.  
 * 
 */
void AFE11612_SetILDAC(void);

/**
 * @brief Clear the ILDAC pin.
 * 
 */
void AFE11612_ClearILDAC(void);

#endif