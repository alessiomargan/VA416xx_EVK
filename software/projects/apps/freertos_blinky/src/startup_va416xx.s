/***************************************************************************************
 * @file     startup_va416xx.s
 * @version  V1.12
 * @date     18 August 2022
 *
 * @note
 * VORAGO Technologies
 *
 * @note
 * Copyright (c) 2013-2022 VORAGO Technologies.
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
  .syntax unified
  .cpu cortex-m4
  .thumb

.global g_pfnVectors
.global Default_Handler

/* start address for the initialization values of the .data section.
defined in linker script */
.word _sidata
/* start address for the .data section. defined in linker script */
.word _sdata
/* end address for the .data section. defined in linker script */
.word _edata
/* start address for the .bss section. defined in linker script */
.word _sbss
/* end address for the .bss section. defined in linker script */
.word _ebss
/* stack used for SystemInit_ExtMemCtl@ always internal RAM used */

/**
 * @brief  This is the code that gets called when the processor first
 *          starts execution following a reset event. 
 * Only the absolutely necessary set is performed, 
 * after which the application supplied main() routine is called.
 * @param  None
 * @retval : None
*/

  .section .text
  .weak Reset_Handler
  .type Reset_Handler, %function
Reset_Handler:
  ldr   sp, =_estack  /* Set stack pointer */

/* Copy the data segment initializers from flash to SRAM */
  movs r1, #0
  b LoopCopyDataInit

CopyDataInit:
  ldr r3, =_sidata
  ldr r3, [r3, r1]
  str r3, [r0, r1]
  adds r1, r1, #4

LoopCopyDataInit:
  ldr r0, =_sdata
  ldr r3, =_edata
  adds r2, r0, r1
  cmp r2, r3
  bcc CopyDataInit
  ldr r2, =_sbss
  b LoopFillZerobss
/* Zero fill the bss segment. */
FillZerobss:
  movs r3, #0
  str r3, [r2], #4

LoopFillZerobss:
  ldr r3, = _ebss
  cmp r2, r3
  bcc FillZerobss

/* Call the clock system intitialization function.*/
  bl  SystemInit
/* Call static constructors */
  bl __libc_init_array
/* Call the application's entry point.*/
  bl main
  bx lr
  .size Reset_Handler, .-Reset_Handler

/**
* @brief  This is the code that gets called when the processor receives an
*         unexpected interrupt.  This simply enters an infinite loop, preserving
*         the system state for examination by a debugger.
*
* @param  None
* @retval : None
*/
  .section .text.Default_Handler,"ax",%progbits
Default_Handler:
Infinite_Loop:
  b Infinite_Loop
  .size Default_Handler, .-Default_Handler
/******************************************************************************
*
* The minimal vector table for a cortex-m4.  Note that the proper constructs
* must be placed on this to ensure that it ends up at physical address
* 0x0000.0000.
*
******************************************************************************/
  .section .isr_vector,"a",%progbits
  .type g_pfnVectors, %object
  .size g_pfnVectors, .-g_pfnVectors

g_pfnVectors:
  .word 	_estack
  .word 	Reset_Handler
  .word 	NMI_Handler
  .word 	HardFault_Handler
  .word 	MemManage_Handler
  .word 	BusFault_Handler
  .word 	UsageFault_Handler
  .word 	0
  .word 	0
  .word 	0
  .word 	0
  .word 	SVC_Handler
  .word 	DebugMon_Handler
  .word 	0
  .word 	PendSV_Handler
  .word 	SysTick_Handler
@  External Interrupts
  .word 	OC0_IRQHandler             @  0: Always 0
  .word 	OC1_IRQHandler             @  1: Always 0
  .word 	OC2_IRQHandler             @  2: Always 0
  .word 	OC3_IRQHandler             @  3: Always 0
  .word 	OC4_IRQHandler             @  4: Always 0
  .word 	OC5_IRQHandler             @  5: Always 0
  .word 	OC6_IRQHandler             @  6: Always 0
  .word 	OC7_IRQHandler             @  7: Always 0
  .word 	OC8_IRQHandler             @  8: Always 0
  .word 	OC9_IRQHandler             @  9: Always 0
  .word 	OC10_IRQHandler            @ 10: Always 0
  .word 	OC11_IRQHandler            @ 11: Always 0
  .word 	OC12_IRQHandler            @ 12: Always 0
  .word 	OC13_IRQHandler            @ 13: Always 0
  .word 	OC14_IRQHandler            @ 14: Always 0
  .word 	OC15_IRQHandler            @ 15: Always 0
  .word 	SPI0_TX_IRQHandler         @ 16: SPI0 TX
  .word 	SPI0_RX_IRQHandler         @ 17: SPI0 RX
  .word 	SPI1_TX_IRQHandler         @ 18: SPI1 TX
  .word 	SPI1_RX_IRQHandler         @ 19: SPI1 RX
  .word 	SPI2_TX_IRQHandler         @ 20: SPI2 TX
  .word 	SPI2_RX_IRQHandler         @ 21: SPI2 RX
  .word 	SPI3_TX_IRQHandler         @ 22: SPI3 TX
  .word 	SPI3_RX_IRQHandler         @ 23: SPI3 RX
  .word 	UART0_TX_IRQHandler        @ 24: UART0 TX
  .word 	UART0_RX_IRQHandler        @ 25: UART0 RX
  .word 	UART1_TX_IRQHandler        @ 26: UART1 TX
  .word 	UART1_RX_IRQHandler        @ 27: UART1 RX
  .word 	UART2_TX_IRQHandler        @ 28: UART2 TX
  .word 	UART2_RX_IRQHandler        @ 29: UART2 RX
  .word 	I2C0_MS_IRQHandler         @ 30: I2C0_MS
  .word 	I2C0_SL_IRQHandler         @ 31: I2C0_SL
  .word 	I2C1_MS_IRQHandler         @ 32: I2C1_MS
  .word 	I2C1_SL_IRQHandler         @ 33: I2C1_SL
  .word 	I2C2_MS_IRQHandler         @ 34: I2C2_MS
  .word 	I2C2_SL_IRQHandler         @ 35: I2C2_SL
  .word 	Ethernet_IRQHandler        @ 36: Ethernet TX
  .word 	OC37_IRQHandler            @ 37: Always 0
  .word 	SpW_IRQHandler             @ 38: Space Wire
  .word 	OC39_IRQHandler            @ 39: Always 0
  .word 	DAC0_IRQHandler            @ 40: DAC 0
  .word 	DAC1_IRQHandler            @ 41: DAC 1
  .word 	TRNG_IRQHandler            @ 42: Random Number Generator
  .word 	DMA_Error_IRQHandler       @ 43: DMA error
  .word 	ADC_IRQHandler             @ 44: ADC
  .word 	LoCLK_IRQHandler           @ 45: LoCLK
  .word 	LVD_IRQHandler             @ 46: LVD
  .word 	WDT_IRQHandler             @ 47: Watchdog
  .word 	TIM0_IRQHandler            @ 48: Timer 0
  .word 	TIM1_IRQHandler            @ 49: Timer 1
  .word 	TIM2_IRQHandler            @ 50: Timer 2
  .word 	TIM3_IRQHandler            @ 51: Timer 3
  .word 	TIM4_IRQHandler            @ 52: Timer 4
  .word 	TIM5_IRQHandler            @ 53: Timer 5
  .word 	TIM6_IRQHandler            @ 54: Timer 6
  .word 	TIM7_IRQHandler            @ 55: Timer 7
  .word 	TIM8_IRQHandler            @ 56: Timer 8
  .word 	TIM9_IRQHandler            @ 57: Timer 9
  .word 	TIM10_IRQHandler           @ 58: Timer 10
  .word 	TIM11_IRQHandler           @ 59: Timer 11
  .word 	TIM12_IRQHandler           @ 60: Timer 12
  .word 	TIM13_IRQHandler           @ 61: Timer 13
  .word 	TIM14_IRQHandler           @ 62: Timer 14
  .word 	TIM15_IRQHandler           @ 63: Timer 15
  .word 	TIM16_IRQHandler           @ 64: Timer 16
  .word 	TIM17_IRQHandler           @ 65: Timer 17
  .word 	TIM18_IRQHandler           @ 66: Timer 18
  .word 	TIM19_IRQHandler           @ 67: Timer 19
  .word 	TIM20_IRQHandler           @ 68: Timer 20
  .word 	TIM21_IRQHandler           @ 69: Timer 21
  .word 	TIM22_IRQHandler           @ 70: Timer 22
  .word 	TIM23_IRQHandler           @ 71: Timer 23
  .word 	CAN0_IRQHandler            @ 72: CAN 0
  .word 	OC73_IRQHandler            @ 73: Always 0
  .word 	CAN1_IRQHandler            @ 74: CAN 1
  .word 	OC75_IRQHandler            @ 75: Always 0
  .word 	EDAC_MBE_IRQHandler        @ 76: EDAC Multi Bit Error
  .word 	EDAC_SBE_IRQHandler        @ 77: EDAC Single Bit Error
  .word 	PA0_IRQHandler             @ 78: PORTA 0
  .word 	PA1_IRQHandler             @ 79: PORTA 1
  .word 	PA2_IRQHandler             @ 80: PORTA 2
  .word 	PA3_IRQHandler             @ 81: PORTA 3
  .word 	PA4_IRQHandler             @ 82: PORTA 4
  .word 	PA5_IRQHandler             @ 83: PORTA 5
  .word 	PA6_IRQHandler             @ 84: PORTA 6
  .word 	PA7_IRQHandler             @ 85: PORTA 7
  .word 	PA8_IRQHandler             @ 86: PORTA 8
  .word 	PA9_IRQHandler             @ 87: PORTA 9
  .word 	PA10_IRQHandler            @ 88: PORTA 10
  .word 	PA11_IRQHandler            @ 89: PORTA 11
  .word 	PA12_IRQHandler            @ 90: PORTA 12
  .word 	PA13_IRQHandler            @ 91: PORTA 13
  .word 	PA14_IRQHandler            @ 92: PORTA 14
  .word 	PA15_IRQHandler            @ 93: PORTA 15
  .word 	PB0_IRQHandler             @ 94: PORTB 0
  .word 	PB1_IRQHandler             @ 95: PORTB 1
  .word 	PB2_IRQHandler             @ 96: PORTB 2
  .word 	PB3_IRQHandler             @ 97: PORTB 3
  .word 	PB4_IRQHandler             @ 98: PORTB 4
  .word 	PB5_IRQHandler             @ 99: PORTB 5
  .word 	PB6_IRQHandler             @ 100: PORTB 6
  .word 	PB7_IRQHandler             @ 101: PORTB 7
  .word 	PB8_IRQHandler             @ 102: PORTB 8
  .word 	PB9_IRQHandler             @ 103: PORTB 9
  .word 	PB10_IRQHandler            @ 104: PORTB 10
  .word 	PB11_IRQHandler            @ 105: PORTB 11
  .word 	PB12_IRQHandler            @ 106: PORTB 12
  .word 	PB13_IRQHandler            @ 107: PORTB 13
  .word 	PB14_IRQHandler            @ 108: PORTB 14
  .word 	PB15_IRQHandler            @ 109: PORTB 15
  .word 	PC0_IRQHandler             @ 110: PORTC 0
  .word 	PC1_IRQHandler             @ 111: PORTC 1
  .word 	PC2_IRQHandler             @ 112: PORTC 2
  .word 	PC3_IRQHandler             @ 113: PORTC 3
  .word 	PC4_IRQHandler             @ 114: PORTC 4
  .word 	PC5_IRQHandler             @ 115: PORTC 5
  .word 	PC6_IRQHandler             @ 116: PORTC 6
  .word 	PC7_IRQHandler             @ 117: PORTC 7
  .word 	PC8_IRQHandler             @ 118: PORTC 8
  .word 	PC9_IRQHandler             @ 119: PORTC 9
  .word 	PC10_IRQHandler            @ 120: PORTC 10
  .word 	PC11_IRQHandler            @ 121: PORTC 11
  .word 	PC12_IRQHandler            @ 122: PORTC 12
  .word 	PC13_IRQHandler            @ 123: PORTC 13
  .word 	PC14_IRQHandler            @ 124: PORTC 14
  .word 	PC15_IRQHandler            @ 125: PORTC 15
  .word 	PD0_IRQHandler             @ 126: PORTD 0
  .word 	PD1_IRQHandler             @ 127: PORTD 1
  .word 	PD2_IRQHandler             @ 128: PORTD 2
  .word 	PD3_IRQHandler             @ 129: PORTD 3
  .word 	PD4_IRQHandler             @ 130: PORTD 4
  .word 	PD5_IRQHandler             @ 131: PORTD 5
  .word 	PD6_IRQHandler             @ 132: PORTD 6
  .word 	PD7_IRQHandler             @ 133: PORTD 7
  .word 	PD8_IRQHandler             @ 134: PORTD 8
  .word 	PD9_IRQHandler             @ 135: PORTD 9
  .word 	PD10_IRQHandler            @ 136: PORTD 10
  .word 	PD11_IRQHandler            @ 137: PORTD 11
  .word 	PD12_IRQHandler            @ 138: PORTD 12
  .word 	PD13_IRQHandler            @ 139: PORTD 13
  .word 	PD14_IRQHandler            @ 140: PORTD 14
  .word 	PD15_IRQHandler            @ 141: PORTD 15
  .word 	PE0_IRQHandler             @ 142: PORTE 0
  .word 	PE1_IRQHandler             @ 143: PORTE 1
  .word 	PE2_IRQHandler             @ 144: PORTE 2
  .word 	PE3_IRQHandler             @ 145: PORTE 3
  .word 	PE4_IRQHandler             @ 146: PORTE 4
  .word 	PE5_IRQHandler             @ 147: PORTE 5
  .word 	PE6_IRQHandler             @ 148: PORTE 6
  .word 	PE7_IRQHandler             @ 149: PORTE 7
  .word 	PE8_IRQHandler             @ 150: PORTE 8
  .word 	PE9_IRQHandler             @ 151: PORTE 9
  .word 	PE10_IRQHandler            @ 152: PORTE 10
  .word 	PE11_IRQHandler            @ 153: PORTE 11
  .word 	PE12_IRQHandler            @ 154: PORTE 12
  .word 	PE13_IRQHandler            @ 155: PORTE 13
  .word 	PE14_IRQHandler            @ 156: PORTE 14
  .word 	PE15_IRQHandler            @ 157: PORTE 15
  .word 	PF0_IRQHandler             @ 158: PORTF 0
  .word 	PF1_IRQHandler             @ 159: PORTF 1
  .word 	PF2_IRQHandler             @ 160: PORTF 2
  .word 	PF3_IRQHandler             @ 161: PORTF 3
  .word 	PF4_IRQHandler             @ 162: PORTF 4
  .word 	PF5_IRQHandler             @ 163: PORTF 5
  .word 	PF6_IRQHandler             @ 164: PORTF 6
  .word 	PF7_IRQHandler             @ 165: PORTF 7
  .word 	PF8_IRQHandler             @ 166: PORTF 8
  .word 	PF9_IRQHandler             @ 167: PORTF 9
  .word 	PF10_IRQHandler            @ 168: PORTF 10
  .word 	PF11_IRQHandler            @ 169: PORTF 11
  .word 	PF12_IRQHandler            @ 170: PORTF 12
  .word 	PF13_IRQHandler            @ 171: PORTF 13
  .word 	PF14_IRQHandler            @ 172: PORTF 14
  .word 	PF15_IRQHandler            @ 173: PORTF 15
  .word 	DMA_Active_0_IRQHandler    @ 174: DMA Active 0
  .word 	DMA_Active_1_IRQHandler    @ 175: DMA Active 1
  .word 	DMA_Active_2_IRQHandler    @ 176: DMA Active 2
  .word 	DMA_Active_3_IRQHandler    @ 177: DMA Active 3
  .word 	DMA_Done_0_IRQHandler      @ 178: DMA Done 0
  .word 	DMA_Done_1_IRQHandler      @ 179: DMA Done 1
  .word 	DMA_Done_2_IRQHandler      @ 180: DMA Done 2
  .word 	DMA_Done_3_IRQHandler      @ 181: DMA Done 3
  .word 	I2C0_MS_RX_IRQHandler      @ 182: I2C0 Master RX
  .word 	I2C0_MS_TX_IRQHandler      @ 183: I2C0 Master TX
  .word 	I2C0_SL_RX_IRQHandler      @ 184: I2C0 Slave RX
  .word 	I2C0_SL_TX_IRQHandler      @ 185: I2C0 Slave TX
  .word 	I2C1_MS_RX_IRQHandler      @ 186: I2C1 Master RX
  .word 	I2C1_MS_TX_IRQHandler      @ 187: I2C1 Master TX
  .word 	I2C1_SL_RX_IRQHandler      @ 188: I2C1 Slave RX
  .word 	I2C1_SL_TX_IRQHandler      @ 189: I2C1 Slave TX
  .word 	I2C2_MS_RX_IRQHandler      @ 190: I2C2 Master RX
  .word 	I2C2_MS_TX_IRQHandler      @ 191: I2C2 Master TX
  .word 	I2C2_SL_RX_IRQHandler      @ 192: I2C2 Slave RX
  .word 	I2C2_SL_TX_IRQHandler      @ 193: I2C2 Slave TX
  .word 	FPU_IRQHandler             @ 194: FPU
  .word 	TXEV_IRQHandler            @ 195: TXEV

/*******************************************************************************
*
* Provide weak aliases for each Exception handler to the Default_Handler.
* As they are weak aliases, any function with the same name will override
* this definition.
*
*******************************************************************************/
  .weak      NMI_Handler
  .thumb_set NMI_Handler,Default_Handler

  .weak      HardFault_Handler
  .thumb_set HardFault_Handler,Default_Handler

  .weak      MemManage_Handler
  .thumb_set MemManage_Handler,Default_Handler

  .weak      BusFault_Handler
  .thumb_set BusFault_Handler,Default_Handler

  .weak      UsageFault_Handler
  .thumb_set UsageFault_Handler,Default_Handler

  .weak      SVC_Handler
  .thumb_set SVC_Handler,Default_Handler

  .weak      DebugMon_Handler
  .thumb_set DebugMon_Handler,Default_Handler

  .weak      PendSV_Handler
  .thumb_set PendSV_Handler,Default_Handler

  .weak      SysTick_Handler
  .thumb_set SysTick_Handler,Default_Handler

@  External Interrupts
  .weak      OC0_IRQHandler
  .thumb_set OC0_IRQHandler,Default_Handler

  .weak      OC1_IRQHandler
  .thumb_set OC1_IRQHandler,Default_Handler

  .weak      OC2_IRQHandler
  .thumb_set OC2_IRQHandler,Default_Handler

  .weak      OC3_IRQHandler
  .thumb_set OC3_IRQHandler,Default_Handler

  .weak      OC4_IRQHandler
  .thumb_set OC4_IRQHandler,Default_Handler

  .weak      OC5_IRQHandler
  .thumb_set OC5_IRQHandler,Default_Handler

  .weak      OC6_IRQHandler
  .thumb_set OC6_IRQHandler,Default_Handler

  .weak      OC7_IRQHandler
  .thumb_set OC7_IRQHandler,Default_Handler

  .weak      OC8_IRQHandler
  .thumb_set OC8_IRQHandler,Default_Handler

  .weak      OC9_IRQHandler
  .thumb_set OC9_IRQHandler,Default_Handler

  .weak      OC10_IRQHandler
  .thumb_set OC10_IRQHandler,Default_Handler

  .weak      OC11_IRQHandler
  .thumb_set OC11_IRQHandler,Default_Handler

  .weak      OC12_IRQHandler
  .thumb_set OC12_IRQHandler,Default_Handler

  .weak      OC13_IRQHandler
  .thumb_set OC13_IRQHandler,Default_Handler

  .weak      OC14_IRQHandler
  .thumb_set OC14_IRQHandler,Default_Handler

  .weak      OC15_IRQHandler
  .thumb_set OC15_IRQHandler,Default_Handler

  .weak      SPI0_TX_IRQHandler
  .thumb_set SPI0_TX_IRQHandler,Default_Handler

  .weak      SPI0_RX_IRQHandler
  .thumb_set SPI0_RX_IRQHandler,Default_Handler

  .weak      SPI1_TX_IRQHandler
  .thumb_set SPI1_TX_IRQHandler,Default_Handler

  .weak      SPI1_RX_IRQHandler
  .thumb_set SPI1_RX_IRQHandler,Default_Handler

  .weak      SPI2_TX_IRQHandler
  .thumb_set SPI2_TX_IRQHandler,Default_Handler

  .weak      SPI2_RX_IRQHandler
  .thumb_set SPI2_RX_IRQHandler,Default_Handler

  .weak      SPI3_TX_IRQHandler
  .thumb_set SPI3_TX_IRQHandler,Default_Handler

  .weak      SPI3_RX_IRQHandler
  .thumb_set SPI3_RX_IRQHandler,Default_Handler

  .weak      UART0_TX_IRQHandler
  .thumb_set UART0_TX_IRQHandler,Default_Handler

  .weak      UART0_RX_IRQHandler
  .thumb_set UART0_RX_IRQHandler,Default_Handler

  .weak      UART1_TX_IRQHandler
  .thumb_set UART1_TX_IRQHandler,Default_Handler

  .weak      UART1_RX_IRQHandler
  .thumb_set UART1_RX_IRQHandler,Default_Handler

  .weak      UART2_TX_IRQHandler
  .thumb_set UART2_TX_IRQHandler,Default_Handler

  .weak      UART2_RX_IRQHandler
  .thumb_set UART2_RX_IRQHandler,Default_Handler

  .weak      I2C0_MS_IRQHandler
  .thumb_set I2C0_MS_IRQHandler,Default_Handler

  .weak      I2C0_SL_IRQHandler
  .thumb_set I2C0_SL_IRQHandler,Default_Handler

  .weak      I2C1_MS_IRQHandler
  .thumb_set I2C1_MS_IRQHandler,Default_Handler

  .weak      I2C1_SL_IRQHandler
  .thumb_set I2C1_SL_IRQHandler,Default_Handler

  .weak      I2C2_MS_IRQHandler
  .thumb_set I2C2_MS_IRQHandler,Default_Handler

  .weak      I2C2_SL_IRQHandler
  .thumb_set I2C2_SL_IRQHandler,Default_Handler

  .weak      Ethernet_IRQHandler
  .thumb_set Ethernet_IRQHandler,Default_Handler

  .weak      OC37_IRQHandler
  .thumb_set OC37_IRQHandler,Default_Handler

  .weak      SpW_IRQHandler
  .thumb_set SpW_IRQHandler,Default_Handler

  .weak      OC39_IRQHandler
  .thumb_set OC39_IRQHandler,Default_Handler

  .weak      DAC0_IRQHandler
  .thumb_set DAC0_IRQHandler,Default_Handler

  .weak      DAC1_IRQHandler
  .thumb_set DAC1_IRQHandler,Default_Handler

  .weak      TRNG_IRQHandler
  .thumb_set TRNG_IRQHandler,Default_Handler

  .weak      DMA_Error_IRQHandler
  .thumb_set DMA_Error_IRQHandler,Default_Handler

  .weak      ADC_IRQHandler
  .thumb_set ADC_IRQHandler,Default_Handler

  .weak      LoCLK_IRQHandler
  .thumb_set LoCLK_IRQHandler,Default_Handler

  .weak      LVD_IRQHandler
  .thumb_set LVD_IRQHandler,Default_Handler

  .weak      WDT_IRQHandler
  .thumb_set WDT_IRQHandler,Default_Handler

  .weak      TIM0_IRQHandler
  .thumb_set TIM0_IRQHandler,Default_Handler

  .weak      TIM1_IRQHandler
  .thumb_set TIM1_IRQHandler,Default_Handler

  .weak      TIM2_IRQHandler
  .thumb_set TIM2_IRQHandler,Default_Handler

  .weak      TIM3_IRQHandler
  .thumb_set TIM3_IRQHandler,Default_Handler

  .weak      TIM4_IRQHandler
  .thumb_set TIM4_IRQHandler,Default_Handler

  .weak      TIM5_IRQHandler
  .thumb_set TIM5_IRQHandler,Default_Handler

  .weak      TIM6_IRQHandler
  .thumb_set TIM6_IRQHandler,Default_Handler

  .weak      TIM7_IRQHandler
  .thumb_set TIM7_IRQHandler,Default_Handler

  .weak      TIM8_IRQHandler
  .thumb_set TIM8_IRQHandler,Default_Handler

  .weak      TIM9_IRQHandler
  .thumb_set TIM9_IRQHandler,Default_Handler

  .weak      TIM10_IRQHandler
  .thumb_set TIM10_IRQHandler,Default_Handler

  .weak      TIM11_IRQHandler
  .thumb_set TIM11_IRQHandler,Default_Handler

  .weak      TIM12_IRQHandler
  .thumb_set TIM12_IRQHandler,Default_Handler

  .weak      TIM13_IRQHandler
  .thumb_set TIM13_IRQHandler,Default_Handler

  .weak      TIM14_IRQHandler
  .thumb_set TIM14_IRQHandler,Default_Handler

  .weak      TIM15_IRQHandler
  .thumb_set TIM15_IRQHandler,Default_Handler

  .weak      TIM16_IRQHandler
  .thumb_set TIM16_IRQHandler,Default_Handler

  .weak      TIM17_IRQHandler
  .thumb_set TIM17_IRQHandler,Default_Handler

  .weak      TIM18_IRQHandler
  .thumb_set TIM18_IRQHandler,Default_Handler

  .weak      TIM19_IRQHandler
  .thumb_set TIM19_IRQHandler,Default_Handler

  .weak      TIM20_IRQHandler
  .thumb_set TIM20_IRQHandler,Default_Handler

  .weak      TIM21_IRQHandler
  .thumb_set TIM21_IRQHandler,Default_Handler

  .weak      TIM22_IRQHandler
  .thumb_set TIM22_IRQHandler,Default_Handler

  .weak      TIM23_IRQHandler
  .thumb_set TIM23_IRQHandler,Default_Handler

  .weak      CAN0_IRQHandler
  .thumb_set CAN0_IRQHandler,Default_Handler

  .weak      OC73_IRQHandler
  .thumb_set OC73_IRQHandler,Default_Handler

  .weak      CAN1_IRQHandler
  .thumb_set CAN1_IRQHandler,Default_Handler

  .weak      OC75_IRQHandler
  .thumb_set OC75_IRQHandler,Default_Handler

  .weak      EDAC_MBE_IRQHandler
  .thumb_set EDAC_MBE_IRQHandler,Default_Handler

  .weak      EDAC_SBE_IRQHandler
  .thumb_set EDAC_SBE_IRQHandler,Default_Handler

  .weak      PA0_IRQHandler
  .thumb_set PA0_IRQHandler,Default_Handler

  .weak      PA1_IRQHandler
  .thumb_set PA1_IRQHandler,Default_Handler

  .weak      PA2_IRQHandler
  .thumb_set PA2_IRQHandler,Default_Handler

  .weak      PA3_IRQHandler
  .thumb_set PA3_IRQHandler,Default_Handler

  .weak      PA4_IRQHandler
  .thumb_set PA4_IRQHandler,Default_Handler

  .weak      PA5_IRQHandler
  .thumb_set PA5_IRQHandler,Default_Handler

  .weak      PA6_IRQHandler
  .thumb_set PA6_IRQHandler,Default_Handler

  .weak      PA7_IRQHandler
  .thumb_set PA7_IRQHandler,Default_Handler

  .weak      PA8_IRQHandler
  .thumb_set PA8_IRQHandler,Default_Handler

  .weak      PA9_IRQHandler
  .thumb_set PA9_IRQHandler,Default_Handler

  .weak      PA10_IRQHandler
  .thumb_set PA10_IRQHandler,Default_Handler

  .weak      PA11_IRQHandler
  .thumb_set PA11_IRQHandler,Default_Handler

  .weak      PA12_IRQHandler
  .thumb_set PA12_IRQHandler,Default_Handler

  .weak      PA13_IRQHandler
  .thumb_set PA13_IRQHandler,Default_Handler

  .weak      PA14_IRQHandler
  .thumb_set PA14_IRQHandler,Default_Handler

  .weak      PA15_IRQHandler
  .thumb_set PA15_IRQHandler,Default_Handler

  .weak      PB0_IRQHandler
  .thumb_set PB0_IRQHandler,Default_Handler

  .weak      PB1_IRQHandler
  .thumb_set PB1_IRQHandler,Default_Handler

  .weak      PB2_IRQHandler
  .thumb_set PB2_IRQHandler,Default_Handler

  .weak      PB3_IRQHandler
  .thumb_set PB3_IRQHandler,Default_Handler

  .weak      PB4_IRQHandler
  .thumb_set PB4_IRQHandler,Default_Handler

  .weak      PB5_IRQHandler
  .thumb_set PB5_IRQHandler,Default_Handler

  .weak      PB6_IRQHandler
  .thumb_set PB6_IRQHandler,Default_Handler

  .weak      PB7_IRQHandler
  .thumb_set PB7_IRQHandler,Default_Handler

  .weak      PB8_IRQHandler
  .thumb_set PB8_IRQHandler,Default_Handler

  .weak      PB9_IRQHandler
  .thumb_set PB9_IRQHandler,Default_Handler

  .weak      PB10_IRQHandler
  .thumb_set PB10_IRQHandler,Default_Handler

  .weak      PB11_IRQHandler
  .thumb_set PB11_IRQHandler,Default_Handler

  .weak      PB12_IRQHandler
  .thumb_set PB12_IRQHandler,Default_Handler

  .weak      PB13_IRQHandler
  .thumb_set PB13_IRQHandler,Default_Handler

  .weak      PB14_IRQHandler
  .thumb_set PB14_IRQHandler,Default_Handler

  .weak      PB15_IRQHandler
  .thumb_set PB15_IRQHandler,Default_Handler

  .weak      PC0_IRQHandler
  .thumb_set PC0_IRQHandler,Default_Handler

  .weak      PC1_IRQHandler
  .thumb_set PC1_IRQHandler,Default_Handler

  .weak      PC2_IRQHandler
  .thumb_set PC2_IRQHandler,Default_Handler

  .weak      PC3_IRQHandler
  .thumb_set PC3_IRQHandler,Default_Handler

  .weak      PC4_IRQHandler
  .thumb_set PC4_IRQHandler,Default_Handler

  .weak      PC5_IRQHandler
  .thumb_set PC5_IRQHandler,Default_Handler

  .weak      PC6_IRQHandler
  .thumb_set PC6_IRQHandler,Default_Handler

  .weak      PC7_IRQHandler
  .thumb_set PC7_IRQHandler,Default_Handler

  .weak      PC8_IRQHandler
  .thumb_set PC8_IRQHandler,Default_Handler

  .weak      PC9_IRQHandler
  .thumb_set PC9_IRQHandler,Default_Handler

  .weak      PC10_IRQHandler
  .thumb_set PC10_IRQHandler,Default_Handler

  .weak      PC11_IRQHandler
  .thumb_set PC11_IRQHandler,Default_Handler

  .weak      PC12_IRQHandler
  .thumb_set PC12_IRQHandler,Default_Handler

  .weak      PC13_IRQHandler
  .thumb_set PC13_IRQHandler,Default_Handler

  .weak      PC14_IRQHandler
  .thumb_set PC14_IRQHandler,Default_Handler

  .weak      PC15_IRQHandler
  .thumb_set PC15_IRQHandler,Default_Handler

  .weak      PD0_IRQHandler
  .thumb_set PD0_IRQHandler,Default_Handler

  .weak      PD1_IRQHandler
  .thumb_set PD1_IRQHandler,Default_Handler

  .weak      PD2_IRQHandler
  .thumb_set PD2_IRQHandler,Default_Handler

  .weak      PD3_IRQHandler
  .thumb_set PD3_IRQHandler,Default_Handler

  .weak      PD4_IRQHandler
  .thumb_set PD4_IRQHandler,Default_Handler

  .weak      PD5_IRQHandler
  .thumb_set PD5_IRQHandler,Default_Handler

  .weak      PD6_IRQHandler
  .thumb_set PD6_IRQHandler,Default_Handler

  .weak      PD7_IRQHandler
  .thumb_set PD7_IRQHandler,Default_Handler

  .weak      PD8_IRQHandler
  .thumb_set PD8_IRQHandler,Default_Handler

  .weak      PD9_IRQHandler
  .thumb_set PD9_IRQHandler,Default_Handler

  .weak      PD10_IRQHandler
  .thumb_set PD10_IRQHandler,Default_Handler

  .weak      PD11_IRQHandler
  .thumb_set PD11_IRQHandler,Default_Handler

  .weak      PD12_IRQHandler
  .thumb_set PD12_IRQHandler,Default_Handler

  .weak      PD13_IRQHandler
  .thumb_set PD13_IRQHandler,Default_Handler

  .weak      PD14_IRQHandler
  .thumb_set PD14_IRQHandler,Default_Handler

  .weak      PD15_IRQHandler
  .thumb_set PD15_IRQHandler,Default_Handler

  .weak      PE0_IRQHandler
  .thumb_set PE0_IRQHandler,Default_Handler

  .weak      PE1_IRQHandler
  .thumb_set PE1_IRQHandler,Default_Handler

  .weak      PE2_IRQHandler
  .thumb_set PE2_IRQHandler,Default_Handler

  .weak      PE3_IRQHandler
  .thumb_set PE3_IRQHandler,Default_Handler

  .weak      PE4_IRQHandler
  .thumb_set PE4_IRQHandler,Default_Handler

  .weak      PE5_IRQHandler
  .thumb_set PE5_IRQHandler,Default_Handler

  .weak      PE6_IRQHandler
  .thumb_set PE6_IRQHandler,Default_Handler

  .weak      PE7_IRQHandler
  .thumb_set PE7_IRQHandler,Default_Handler

  .weak      PE8_IRQHandler
  .thumb_set PE8_IRQHandler,Default_Handler

  .weak      PE9_IRQHandler
  .thumb_set PE9_IRQHandler,Default_Handler

  .weak      PE10_IRQHandler
  .thumb_set PE10_IRQHandler,Default_Handler

  .weak      PE11_IRQHandler
  .thumb_set PE11_IRQHandler,Default_Handler

  .weak      PE12_IRQHandler
  .thumb_set PE12_IRQHandler,Default_Handler

  .weak      PE13_IRQHandler
  .thumb_set PE13_IRQHandler,Default_Handler

  .weak      PE14_IRQHandler
  .thumb_set PE14_IRQHandler,Default_Handler

  .weak      PE15_IRQHandler
  .thumb_set PE15_IRQHandler,Default_Handler

  .weak      PF0_IRQHandler
  .thumb_set PF0_IRQHandler,Default_Handler

  .weak      PF1_IRQHandler
  .thumb_set PF1_IRQHandler,Default_Handler

  .weak      PF2_IRQHandler
  .thumb_set PF2_IRQHandler,Default_Handler

  .weak      PF3_IRQHandler
  .thumb_set PF3_IRQHandler,Default_Handler

  .weak      PF4_IRQHandler
  .thumb_set PF4_IRQHandler,Default_Handler

  .weak      PF5_IRQHandler
  .thumb_set PF5_IRQHandler,Default_Handler

  .weak      PF6_IRQHandler
  .thumb_set PF6_IRQHandler,Default_Handler

  .weak      PF7_IRQHandler
  .thumb_set PF7_IRQHandler,Default_Handler

  .weak      PF8_IRQHandler
  .thumb_set PF8_IRQHandler,Default_Handler

  .weak      PF9_IRQHandler
  .thumb_set PF9_IRQHandler,Default_Handler

  .weak      PF10_IRQHandler
  .thumb_set PF10_IRQHandler,Default_Handler

  .weak      PF11_IRQHandler
  .thumb_set PF11_IRQHandler,Default_Handler

  .weak      PF12_IRQHandler
  .thumb_set PF12_IRQHandler,Default_Handler

  .weak      PF13_IRQHandler
  .thumb_set PF13_IRQHandler,Default_Handler

  .weak      PF14_IRQHandler
  .thumb_set PF14_IRQHandler,Default_Handler

  .weak      PF15_IRQHandler
  .thumb_set PF15_IRQHandler,Default_Handler

  .weak      DMA_Active_0_IRQHandler
  .thumb_set DMA_Active_0_IRQHandler,Default_Handler

  .weak      DMA_Active_1_IRQHandler
  .thumb_set DMA_Active_1_IRQHandler,Default_Handler

  .weak      DMA_Active_2_IRQHandler
  .thumb_set DMA_Active_2_IRQHandler,Default_Handler

  .weak      DMA_Active_3_IRQHandler
  .thumb_set DMA_Active_3_IRQHandler,Default_Handler

  .weak      DMA_Done_0_IRQHandler
  .thumb_set DMA_Done_0_IRQHandler,Default_Handler

  .weak      DMA_Done_1_IRQHandler
  .thumb_set DMA_Done_1_IRQHandler,Default_Handler

  .weak      DMA_Done_2_IRQHandler
  .thumb_set DMA_Done_2_IRQHandler,Default_Handler

  .weak      DMA_Done_3_IRQHandler
  .thumb_set DMA_Done_3_IRQHandler,Default_Handler

  .weak      I2C0_MS_RX_IRQHandler
  .thumb_set I2C0_MS_RX_IRQHandler,Default_Handler

  .weak      I2C0_MS_TX_IRQHandler
  .thumb_set I2C0_MS_TX_IRQHandler,Default_Handler

  .weak      I2C0_SL_RX_IRQHandler
  .thumb_set I2C0_SL_RX_IRQHandler,Default_Handler

  .weak      I2C0_SL_TX_IRQHandler
  .thumb_set I2C0_SL_TX_IRQHandler,Default_Handler

  .weak      I2C1_MS_RX_IRQHandler
  .thumb_set I2C1_MS_RX_IRQHandler,Default_Handler

  .weak      I2C1_MS_TX_IRQHandler
  .thumb_set I2C1_MS_TX_IRQHandler,Default_Handler

  .weak      I2C1_SL_RX_IRQHandler
  .thumb_set I2C1_SL_RX_IRQHandler,Default_Handler

  .weak      I2C1_SL_TX_IRQHandler
  .thumb_set I2C1_SL_TX_IRQHandler,Default_Handler

  .weak      I2C2_MS_RX_IRQHandler
  .thumb_set I2C2_MS_RX_IRQHandler,Default_Handler

  .weak      I2C2_MS_TX_IRQHandler
  .thumb_set I2C2_MS_TX_IRQHandler,Default_Handler

  .weak      I2C2_SL_RX_IRQHandler
  .thumb_set I2C2_SL_RX_IRQHandler,Default_Handler

  .weak      I2C2_SL_TX_IRQHandler
  .thumb_set I2C2_SL_TX_IRQHandler,Default_Handler

  .weak      FPU_IRQHandler
  .thumb_set FPU_IRQHandler,Default_Handler

  .weak      TXEV_IRQHandler
  .thumb_set TXEV_IRQHandler,Default_Handler
