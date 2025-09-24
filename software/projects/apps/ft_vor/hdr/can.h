/**
 * @file can.h
 * @brief CAN Bus Driver Interface for VA416xx
 *
 * This file defines the interface for the CAN bus driver used with the
 * VA416xx microcontroller, providing functions to initialize and
 * communicate over the CAN network with support for RTR handling.
 *
 * @author Alessio Margan
 * @date September 24, 2025
 *
 * @version 1.0
 *
 * @copyright Copyright (c) 2025 IIT
 *
 * @note This implementation requires the VA416xx HAL CANBUS driver
 */

#ifndef __CAN_H__
#define __CAN_H__

#include "va416xx_hal_canbus.h"

// Function to initialize and configure CAN0 peripheral
void ConfigureCAN0(void);

// Global array of CAN message buffers for RTR responses
extern volatile can_cmb_t * const cmb_RTR_resp[];

#endif /* __CAN_H__ */