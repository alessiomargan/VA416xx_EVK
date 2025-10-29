/**
 * @file sine_table.h
 * @brief Sine lookup table for waveform generation
 *
 * This file contains a precomputed sine lookup table for efficient
 * sine wave generation without requiring floating point math.
 *
 * @author Alessio Margan
 * @date October 29, 2025
 *
 * @version 1.0
 *
 * @copyright Copyright (c) 2025 IIT
 */

#ifndef __SINE_TABLE_H__
#define __SINE_TABLE_H__

#include <stdint.h>

/**
 * @brief Number of samples in the sine lookup table
 * 
 * 256 samples provides good resolution while fitting nicely with 8-bit indexing.
 * Covers one complete period (0 to 2π).
 */
#define SINE_TABLE_SIZE 256

/**
 * @brief Sine lookup table
 * 
 * Contains 256 samples of a sine wave over one complete period.
 * Values are scaled to 12-bit range (0-4095) for DAC compatibility.
 * Offset to 2048 for bipolar output (sine values range from 0 to 4095).
 * 
 * Formula: sine_table[i] = (uint16_t)(2048 + 2047 * sin(2 * PI * i / 256))
 * 
 * Usage example:
 *   // Generate sine wave at specific phase
 *   uint8_t phase = 0;
 *   uint16_t dac_value = sine_table[phase];
 *   afe11612_values[AFE_VALUES_IDX_DAC_0].value = dac_value;
 *   
 *   // Increment phase for next sample
 *   phase++;  // Automatically wraps at 256
 */
extern const uint16_t sine_table[SINE_TABLE_SIZE];

/**
 * @brief Get sine value at specified phase
 * 
 * @param phase Phase index (0-255), wraps automatically with 8-bit overflow
 * @return Sine value scaled for 12-bit DAC (0-4095, centered at 2048)
 */
static inline uint16_t sine_lookup(uint8_t phase)
{
    return sine_table[phase];
}

#endif /* __SINE_TABLE_H__ */
