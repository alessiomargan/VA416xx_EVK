#ifndef __CAN_TEST_H
#define __CAN_TEST_H

#include "va416xx_hal_canbus.h"

extern uint32_t can_test_loopback(VOR_CAN_Type *myCAN);
extern uint32_t can_test_connect(VOR_CAN_Type *myCAN);

#endif
