#ifndef __HAL_UTILS_H__
#define __HAL_UTILS_H__

#include <stdbool.h>
#include <stdint.h>
#include "va416xx.h"


#define DBG1_PIN     10
#define DBG2_PIN     11
#define DBG3_PIN     12
#define DBG_PORT    PORTF


// Simple GPIO pin manipulation helpers
// Usage: Pin_on(PORTA, 5); Pin_off(PORTB, 3);

static inline void Pin_off(VOR_GPIO_Type * const GPIO_PORT, uint8_t pin) { GPIO_PORT->CLROUT = 1UL<<pin; }
static inline void Pin_on (VOR_GPIO_Type * const GPIO_PORT, uint8_t pin) { GPIO_PORT->SETOUT = 1UL<<pin; }
static inline void Pin_tgl(VOR_GPIO_Type * const GPIO_PORT, uint8_t pin) { GPIO_PORT->TOGOUT = 1UL<<pin; }
static inline void Pin_set(VOR_GPIO_Type * const GPIO_PORT, uint8_t pin, bool val) {
    val ? (GPIO_PORT->SETOUT = 1UL<<pin) : (GPIO_PORT->CLROUT = 1UL<<pin);
}

#endif