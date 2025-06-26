/******************************************************************************
 * @file     HardFault_Handler.c
 * @brief    HardFault handler example
 * @version  V2.00
 * @date     9 Sept 2022
 ******************************************************************************/
/*
 * Copyright (c) 2017 ARM Limited. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "va416xx_hal.h"
#include "va416xx_debug.h"

#ifndef __ICCARM__
#ifdef __clang__
  extern void hfhandler_asm(void);
#endif

void HardFault_Handler(void)
{
#ifdef __clang__
  hfhandler_asm();
#else
  asm("TST    LR, #4");
  asm("ITE    EQ");
  asm("MRSEQ  R0, MSP");
  asm("MRSNE  R0, PSP");
	asm("MOV    R1, LR");
	asm("B      HardFault_Handler_C");
#endif
}
#endif

// HardFault handler in C, with stack frame location and LR value extracted
// from the assembly wrapper as input parameters. Called by HardFault_Handler()
void HardFault_Handler_C(uint32_t * hardfault_args, uint32_t lr_value)
{
  volatile uint32_t stacked_r0;
  volatile uint32_t stacked_r1;
  volatile uint32_t stacked_r2;
  volatile uint32_t stacked_r3;
  volatile uint32_t stacked_r12;
  volatile uint32_t stacked_lr;
  volatile uint32_t stacked_pc;
  volatile uint32_t stacked_psr;
  volatile uint32_t cfsr;
  volatile uint32_t bus_fault_address;
  volatile uint32_t memmanage_fault_address;

  bus_fault_address       = SCB->BFAR;
  memmanage_fault_address = SCB->MMFAR;
  cfsr                    = SCB->CFSR;

  stacked_r0  = ((uint32_t) hardfault_args[0]);
  stacked_r1  = ((uint32_t) hardfault_args[1]);
  stacked_r2  = ((uint32_t) hardfault_args[2]);
  stacked_r3  = ((uint32_t) hardfault_args[3]);
  stacked_r12 = ((uint32_t) hardfault_args[4]);
  stacked_lr  = ((uint32_t) hardfault_args[5]);
  stacked_pc  = ((uint32_t) hardfault_args[6]);
  stacked_psr = ((uint32_t) hardfault_args[7]);

  dbgprintf ("[HardFault]\n");
  dbgprintf ("- Stack frame:\n");
  dbgprintf (" R0   = %x\n", stacked_r0);
  dbgprintf (" R1   = %x\n", stacked_r1);
  dbgprintf (" R2   = %x\n", stacked_r2);
  dbgprintf (" R3   = %x\n", stacked_r3);
  dbgprintf (" R12  = %x\n", stacked_r12);
  dbgprintf (" LR   = %x\n", stacked_lr);
  dbgprintf (" PC   = %x\n", stacked_pc);
  dbgprintf (" PSR  = %x\n", stacked_psr);
  dbgprintf ("- FSR/FAR:\n");
  dbgprintf (" CFSR = %x\n", cfsr);
  dbgprintf (" HFSR = %x\n", SCB->HFSR);
  dbgprintf (" DFSR = %x\n", SCB->DFSR);
  dbgprintf (" AFSR = %x\n", SCB->AFSR);
  if (cfsr & 0x0080) dbgprintf (" MMFAR= %x\n", memmanage_fault_address);
  if (cfsr & 0x8000) dbgprintf (" BFAR = %x\n", bus_fault_address);
  dbgprintf ("- Misc\n");
  dbgprintf (" LR/EXC_RETURN= %x\n", lr_value);

  uint32_t u32RomSbe = VOR_SYSCONFIG->ROM_SBE;
  uint32_t u32Ram0Sbe = VOR_SYSCONFIG->RAM0_SBE;
  uint32_t u32Ram1Sbe = VOR_SYSCONFIG->RAM1_SBE;
  dbgprintf("irom ram0 ram1 sbec, %d, %d, %d\n", u32RomSbe, \
    u32Ram0Sbe, u32Ram1Sbe);
  uint32_t u32RomMbe = VOR_SYSCONFIG->ROM_MBE;
  uint32_t u32Ram0Mbe = VOR_SYSCONFIG->RAM0_MBE;
  uint32_t u32Ram1Mbe = VOR_SYSCONFIG->RAM1_MBE;
  dbgprintf("irom ram0 ram1 mbec, %d, %d, %d\n", u32RomMbe, \
    u32Ram0Mbe, u32Ram1Mbe);

  dbgprintf("Resetting...\n");
  VOR_WATCH_DOG->WDOGINTCLR = 1;
  for(volatile int i=0; i<1000000; i++){} // wait for print to complete
  NVIC_SystemReset(); // reset
  //while(1){}
}
