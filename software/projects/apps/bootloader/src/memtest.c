/***************************************************************************************
 * @file     memtest.c
 * @version  V1.1
 * @date     24 June 2021
 *
 * @note
 * VORAGO Technologies
 *
 * @note
 * Copyright (c) 2013-2021 VORAGO Technologies.
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

#include "board.h"
#include "memtest.h"

/*****************************************************************************/
/* Local pre-processor symbols/macros ('#define')                            */
/*****************************************************************************/

// memory test bounds
#define ROM_CHK_START  (uint32_t*)(0x4000)
#define ROM_CHK_END    (uint32_t*)(0x40000)
#define RAM0_CHK_START (uint32_t*)(0x1FFF8000)
#define RAM0_CHK_END   (uint32_t*)(0x1FFFD000)
#define RAM1_CHK_START (uint32_t*)(0x20000000)
#define RAM1_CHK_END   (uint32_t*)(0x20008000)

// memory test pattern
#define MEM_CHECKERBOARD0 (0xaaaaaaaa)
#define MEM_CHECKERBOARD1 (0x55555555)

/*****************************************************************************/
/* Global variable definitions (declared in header file with 'extern')       */
/*****************************************************************************/

bool chkInv = false; // checkerboard inverse (0xaa / 0x55)

/*****************************************************************************/
/* Local type definitions ('typedef')                                        */
/*****************************************************************************/

/*****************************************************************************/
/* Local variable definitions ('static')                                     */
/*****************************************************************************/

static uint32_t sramErrors = 0;

/*****************************************************************************/
/* Local function prototypes ('static')                                      */
/*****************************************************************************/

static void checkerboard(uint32_t *start, uint32_t *end);

/*****************************************************************************/
/* Function implementation - global ('extern') and local ('static')          */
/*****************************************************************************/

static void checkerboard(uint32_t *start, uint32_t *end)
{
  // start and end must be aligned on 8 byte (double word) boundary

  if(chkInv){
    while(start < end){
      *(start++) = MEM_CHECKERBOARD1;
      *(start++) = MEM_CHECKERBOARD0;
    }
  } else {
    while(start < end){
      *(start++) = MEM_CHECKERBOARD0;
      *(start++) = MEM_CHECKERBOARD1;
    }
  }
}

void sramTest_init(void)
{
  // set up memory checkerboards
  VOR_SYSCONFIG->ROM_PROT = 1; // enable IROM writes
  checkerboard(ROM_CHK_START, ROM_CHK_END);
  VOR_SYSCONFIG->ROM_PROT = 0; // disable IROM writes
  checkerboard(RAM0_CHK_START, RAM0_CHK_END);
  checkerboard(RAM1_CHK_START, RAM1_CHK_END);
  WDFEED();
}

/** memory error logging (part of SRAM test) */
static void logMemErr(uint32_t *addr, uint32_t val, uint32_t region)
{
  uint32_t u32Irqraw =  VOR_SYSCONFIG->IRQ_RAW;
  uint32_t u32RomSbe =  VOR_SYSCONFIG->ROM_SBE;
  uint32_t u32Ram0Sbe = VOR_SYSCONFIG->RAM0_SBE;
  uint32_t u32Ram1Sbe = VOR_SYSCONFIG->RAM1_SBE;
  uint32_t u32RomMbe =  VOR_SYSCONFIG->ROM_MBE;
  uint32_t u32Ram0Mbe = VOR_SYSCONFIG->RAM0_MBE;
  uint32_t u32Ram1Mbe = VOR_SYSCONFIG->RAM1_MBE;
  switch(region)
  {
    case 0:
      // IRAM
      if(u32Irqraw & SYSCONFIG_IRQ_RAW_ROMSBE_Msk){
        printf("ROM SBE, 0x%lx, 0x%08lx, 0x%lx, %ld, %ld\n", (uint32_t)addr, val,
                u32Irqraw, u32RomSbe, u32RomMbe);
      } else if(u32Irqraw & SYSCONFIG_IRQ_RAW_ROMMBE_Msk){
        printf( "ROM MBE, 0x%lx, 0x%08lx, 0x%lx, %ld, %ld\n", (uint32_t)addr, val,
                u32Irqraw, u32RomSbe, u32RomMbe);
      } else {
        printf("ROM Mismatch or IRQ_RAW, 0x%lx, 0x%08lx, 0x%lx, %ld, %ld\n", (uint32_t)addr, val,
                u32Irqraw, u32RomSbe, u32RomMbe);
      }
      break;

    case 1:
      // RAM0
      if(u32Irqraw & SYSCONFIG_IRQ_RAW_RAM0SBE_Msk){
        printf("RAM0 SBE, 0x%lx, 0x%08lx, 0x%lx, %ld, %ld\n", (uint32_t)addr, val,
                u32Irqraw, u32Ram0Sbe, u32Ram0Mbe);
      } else if(u32Irqraw & SYSCONFIG_IRQ_RAW_RAM0MBE_Msk){
        printf("RAM0 MBE, 0x%lx, 0x%08lx, 0x%lx, %ld, %ld\n", (uint32_t)addr, val,
                u32Irqraw, u32Ram0Sbe, u32Ram0Mbe);
      } else {
        printf("RAM0 Mismatch or IRQ_RAW, 0x%lx, 0x%08lx, 0x%lx, %ld, %ld\n", (uint32_t)addr, val,
                u32Irqraw, u32Ram0Sbe, u32Ram0Mbe);
      }
      break;

    case 2:
      // RAM1
      if(u32Irqraw & SYSCONFIG_IRQ_RAW_RAM1SBE_Msk){
        printf("RAM1 SBE, 0x%lx, 0x%08lx, 0x%lx, %ld, %ld\n", (uint32_t)addr, val,
                u32Irqraw, u32Ram1Sbe, u32Ram1Mbe);
      } else if(u32Irqraw & SYSCONFIG_IRQ_RAW_RAM1MBE_Msk){
        printf("RAM1 MBE, 0x%lx, 0x%08lx, 0x%lx, %ld, %ld\n", (uint32_t)addr, val,
                u32Irqraw, u32Ram1Sbe, u32Ram1Mbe);
      } else {
        printf("RAM1 Mismatch or IRQ_RAW, 0x%lx, 0x%08lx, 0x%lx, %ld, %ld\n", (uint32_t)addr, val,
                u32Irqraw, u32Ram1Sbe, u32Ram1Mbe);
      }
      break;
  }
  WDFEED();
}

/** SRAM test, checking for SBE/MBE */
uint32_t sramTest(void)
{
  uint32_t *ramPtr;
  uint32_t rVal;
  uint32_t chkVal;
  bool log;

  VOR_SYSCONFIG->IRQ_ENB = 0x0;
  VOR_SYSCONFIG->IRQ_CLR = 0xff; // clear all sbe/mbe flags
  WDFEED();

  // 1. IRAM check
  ramPtr = ROM_CHK_START;
  log = false;
  while(ramPtr < ROM_CHK_END){
    rVal = *ramPtr;
    chkVal = chkInv ? MEM_CHECKERBOARD1 : MEM_CHECKERBOARD0;
    if(rVal != chkVal){
      log = true;  // data mismatch (MBE)
      VOR_SYSCONFIG->ROM_PROT = 1; // enable IROM writes
      *ramPtr = chkVal; // fix MBE
      VOR_SYSCONFIG->ROM_PROT = 0; // disable IROM writes
    }
    if(VOR_SYSCONFIG->IRQ_RAW){ log = true; } // SBE or MBE
    if(log){
      logMemErr(ramPtr, rVal, 0);
      log = false;
      VOR_SYSCONFIG->IRQ_CLR = 0xff; // clear all sbe/mbe flags
      sramErrors++;
    }
    ramPtr++;
    rVal = *ramPtr;
    chkVal = chkInv ? MEM_CHECKERBOARD0 : MEM_CHECKERBOARD1;
    if(rVal != chkVal){
      log = true;  // data mismatch (MBE)
      *ramPtr = chkVal; // fix MBE
    }
    if(VOR_SYSCONFIG->IRQ_RAW){ log = true; } // SBE or MBE
    if(log){
      logMemErr(ramPtr, rVal, 0);
      log = false;
      VOR_SYSCONFIG->IRQ_CLR = 0xff; // clear all sbe/mbe flags
      sramErrors++;
    }
    ramPtr++;
    WDFEED();
  }

  // 1. RAM0 check
  ramPtr = RAM0_CHK_START;
  log = false;
  while(ramPtr < RAM0_CHK_END){
    rVal = *ramPtr;
    chkVal = chkInv ? MEM_CHECKERBOARD1 : MEM_CHECKERBOARD0;
    if(rVal != chkVal){
      log = true;  // data mismatch (MBE)
      *ramPtr = chkVal; // fix MBE
    }
    if(VOR_SYSCONFIG->IRQ_RAW){ log = true; } // SBE or MBE
    if(log){
      logMemErr(ramPtr, rVal, 1);
      log = false;
      VOR_SYSCONFIG->IRQ_CLR = 0xff; // clear all sbe/mbe flags
      sramErrors++;
    }
    ramPtr++;
    rVal = *ramPtr;
    chkVal = chkInv ? MEM_CHECKERBOARD0 : MEM_CHECKERBOARD1;
    if(rVal != chkVal){
      log = true;  // data mismatch (MBE)
      *ramPtr = chkVal; // fix MBE
    }
    if(VOR_SYSCONFIG->IRQ_RAW){ log = true; } // SBE or MBE
    if(log){
      logMemErr(ramPtr, rVal, 1);
      log = false;
      VOR_SYSCONFIG->IRQ_CLR = 0xff; // clear all sbe/mbe flags
      sramErrors++;
    }
    ramPtr++;
    WDFEED();
  }


  // 1. RAM1 check
  ramPtr = RAM1_CHK_START;
  log = false;
  while(ramPtr < RAM1_CHK_END){
    rVal = *ramPtr;
    chkVal = chkInv ? MEM_CHECKERBOARD1 : MEM_CHECKERBOARD0;
    if(rVal != chkVal){
      log = true;  // data mismatch (MBE)
      *ramPtr = chkVal; // fix MBE
    }
    if(VOR_SYSCONFIG->IRQ_RAW){ log = true; } // SBE or MBE
    if(log){
      logMemErr(ramPtr, rVal, 2);
      log = false;
      VOR_SYSCONFIG->IRQ_CLR = 0xff; // clear all sbe/mbe flags
      sramErrors++;
    }
    ramPtr++;
    rVal = *ramPtr;
    chkVal = chkInv ? MEM_CHECKERBOARD0 : MEM_CHECKERBOARD1;
    if(rVal != chkVal){
      log = true;  // data mismatch (MBE)
      *ramPtr = chkVal; // fix MBE
    }
    if(VOR_SYSCONFIG->IRQ_RAW){ log = true; } // SBE or MBE
    if(log){
      logMemErr(ramPtr, rVal, 2);
      log = false;
      VOR_SYSCONFIG->IRQ_CLR = 0xff; // clear all sbe/mbe flags
      sramErrors++;
    }
    ramPtr++;
    WDFEED();
  }

  VOR_SYSCONFIG->IRQ_ENB = 0x3F;

  return sramErrors;
}

/*****************************************************************************/
/* End of file                                                               */
/*****************************************************************************/
