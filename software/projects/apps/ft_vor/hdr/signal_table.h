/**
 * @file signal_table.h
 * @brief Sine and triangle lookup tables
 *
 * @author Alessio Margan
 * @date February 4, 2026
 *
 * @version 1.0
 *
 * @copyright Copyright (c) 2026 IIT
 */

#ifndef SIGNAL_TABLE_H
#define SIGNAL_TABLE_H

#include <stdint.h>

#define SIGNAL_TABLE_SIZE 256

extern const uint16_t sine_table[SIGNAL_TABLE_SIZE];
extern const uint16_t triangle_table[SIGNAL_TABLE_SIZE];

#endif // SIGNAL_TABLE_H