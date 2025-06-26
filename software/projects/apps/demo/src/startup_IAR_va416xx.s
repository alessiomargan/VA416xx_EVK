;/******************** (C) COPYRIGHT 2020 VORAG0 Technologies ********************
;* File Name          : startup_IAR_va416xx.s
;* Date               : December 3, 2020
;* Description        : VA416xx devices vector table for EWARM toolchain.
;*                      This module performs:
;*                      - Set the initial SP
;*                      - Set the initial PC == _iar_program_start,
;*                      - Set the vector table entries with the exceptions ISR 
;*                        address.
;*                      - Branches to main in the C library (which eventually
;*                        calls main()).
;********************************************************************************
;

#pragma language=extended
#pragma segment="CSTACK"

//  EXTERN void __iar_program_start( void );

//  EXTERN void NMI_Handler( void );
//  EXTERN void HardFault_Handler( void );
//  EXTERN void MemManage_Handler( void );
//  EXTERN void BusFault_Handler( void );
//  EXTERN void UsageFault_Handler( void );
//  EXTERN void SVC_Handler( void );
//  EXTERN void DebugMon_Handler( void );
//  EXTERN void PendSV_Handler( void );
//  EXTERN void SysTick_Handler( void );

//  typedef void( *intfunc )( void );
//  typedef union { intfunc __fun; void * __ptr; } intvec_elem;

; The modules in this file are included in the libraries, and may be replaced
; by any user-defined modules that define the PUBLIC symbol _program_start or
; a user defined start symbol.
; To override the cstartup defined in the library, simply add your modified
; version to the workbench project.
;
; The vector table is normally located at address 0.
; When debugging in RAM, it can be located in RAM, aligned to at least 2^6.
; The name "__vector_table" has special meaning for C-SPY, which
; is where to find the SP start value.
; If vector table is not located at address 0, the user has to initialize the  NVIC vector
; table register (VTOR) before using interrupts.
;
; Cortex-M version
;

        MODULE  ?cstartup

        ;; Forward declaration of sections.
        SECTION CSTACK:DATA:NOROOT(3)

        SECTION .intvec:CODE:NOROOT(2)

        EXTERN  __iar_program_start
        EXTERN  SystemInit
        EXTERN  HardFault_Handler_C
        PUBLIC  __vector_table

        DATA
__vector_table
        DCD     sfe(CSTACK)
        DCD     Reset_Handler             ; Reset Handler

        DCD     NMI_Handler               ; NMI Handler
        DCD     HardFault_Handler         ; Hard Fault Handler
        DCD     MemManage_Handler         ; MPU Fault Handler
        DCD     BusFault_Handler          ; Bus Fault Handler
        DCD     UsageFault_Handler        ; Usage Fault Handler
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     SVC_Handler               ; SVCall Handler
        DCD     DebugMon_Handler          ; Debug Monitor Handler
        DCD     0                         ; Reserved
        DCD     PendSV_Handler            ; PendSV Handler
        DCD     SysTick_Handler           ; SysTick Handler

    ; External Interrupts
        DCD     OC0_IRQHandler   	        ;  0: TIM 0 ISR
        DCD     OC1_IRQHandler             ;  1: Always 0
        DCD     OC2_IRQHandler             ;  2: Always 0
        DCD     OC3_IRQHandler             ;  3: Always 0
        DCD     OC4_IRQHandler             ;  4: Always 0
        DCD     OC5_IRQHandler             ;  5: Always 0
        DCD     OC6_IRQHandler             ;  6: Always 0
        DCD     OC7_IRQHandler             ;  7: Always 0
        DCD     OC8_IRQHandler             ;  8: Always 0
        DCD     OC9_IRQHandler             ;  9: Always 0
        DCD     OC10_IRQHandler            ; 10: Always 0
        DCD     OC11_IRQHandler            ; 11: Always 0
        DCD     OC12_IRQHandler            ; 12: Always 0
        DCD     OC13_IRQHandler            ; 13: Always 0
        DCD     OC14_IRQHandler            ; 14: Always 0
        DCD     OC15_IRQHandler            ; 15: Always 0
        DCD     SPI0_TX_IRQHandler         ; 16: SPI0 TX
        DCD     SPI0_RX_IRQHandler         ; 17: SPI0 RX
        DCD     SPI1_TX_IRQHandler         ; 18: SPI1 TX
        DCD     SPI1_RX_IRQHandler         ; 19: SPI1 RX
        DCD     SPI2_TX_IRQHandler         ; 20: SPI2 TX
        DCD     SPI2_RX_IRQHandler         ; 21: SPI2 RX
        DCD     SPI3_TX_IRQHandler         ; 22: SPI3 TX
        DCD     SPI3_RX_IRQHandler         ; 23: SPI3 RX
        DCD     UART0_TX_IRQHandler        ; 24: UART0 TX
        DCD     UART0_RX_IRQHandler        ; 25: UART0 RX
        DCD     UART1_TX_IRQHandler        ; 26: UART1 TX
        DCD     UART1_RX_IRQHandler        ; 27: UART1 RX
        DCD     UART2_TX_IRQHandler        ; 28: UART2 TX
        DCD     UART2_RX_IRQHandler        ; 29: UART2 RX
        DCD     I2C0_MS_IRQHandler         ; 30: I2C0_MS
        DCD     I2C0_SL_IRQHandler         ; 31: I2C0_SL
        DCD     I2C1_MS_IRQHandler         ; 32: I2C1_MS
        DCD     I2C1_SL_IRQHandler         ; 33: I2C1_SL
        DCD     I2C2_MS_IRQHandler         ; 34: I2C2_MS
        DCD     I2C2_SL_IRQHandler         ; 35: I2C2_SL
        DCD     Ethernet_IRQHandler        ; 36: Ethernet TX
        DCD     OC37_IRQHandler            ; 37: Always 0
        DCD     SpW_IRQHandler             ; 38: Space Wire
        DCD     OC39_IRQHandler            ; 39: Always 0
        DCD     DAC0_IRQHandler            ; 40: DAC 0
        DCD     DAC1_IRQHandler            ; 41: DAC 1
        DCD     TRNG_IRQHandler            ; 42: Random Number Generator
        DCD     DMA_Error_IRQHandler       ; 43: DMA error
        DCD     ADC_IRQHandler             ; 44: ADC
        DCD     LoCLK_IRQHandler           ; 45: LoCLK
        DCD     LVD_IRQHandler             ; 46: LVD
        DCD     WDT_IRQHandler             ; 47: Watchdog
        DCD     TIM0_IRQHandler            ; 48: Timer 0
        DCD     TIM1_IRQHandler            ; 49: Timer 1
        DCD     TIM2_IRQHandler            ; 50: Timer 2
        DCD     TIM3_IRQHandler            ; 51: Timer 3
        DCD     TIM4_IRQHandler            ; 52: Timer 4
        DCD     TIM5_IRQHandler            ; 53: Timer 5
        DCD     TIM6_IRQHandler            ; 54: Timer 6
        DCD     TIM7_IRQHandler            ; 55: Timer 7
        DCD     TIM8_IRQHandler            ; 56: Timer 8
        DCD     TIM9_IRQHandler            ; 57: Timer 9
        DCD     TIM10_IRQHandler           ; 58: Timer 10
        DCD     TIM11_IRQHandler           ; 59: Timer 11
        DCD     TIM12_IRQHandler           ; 60: Timer 12
        DCD     TIM13_IRQHandler           ; 61: Timer 13
        DCD     TIM14_IRQHandler           ; 62: Timer 14
        DCD     TIM15_IRQHandler           ; 63: Timer 15
        DCD     TIM16_IRQHandler           ; 64: Timer 16
        DCD     TIM17_IRQHandler           ; 65: Timer 17
        DCD     TIM18_IRQHandler           ; 66: Timer 18
        DCD     TIM19_IRQHandler           ; 67: Timer 19
        DCD     TIM20_IRQHandler           ; 68: Timer 20
        DCD     TIM21_IRQHandler           ; 69: Timer 21
        DCD     TIM22_IRQHandler           ; 70: Timer 22
        DCD     TIM23_IRQHandler           ; 71: Timer 23
        DCD     CAN0_IRQHandler            ; 72: CAN 0
        DCD     OC73_IRQHandler            ; 73: Always 0
        DCD     CAN1_IRQHandler            ; 74: CAN 1
        DCD     OC75_IRQHandler            ; 75: Always 0
        DCD     EDAC_MBE_IRQHandler        ; 76: EDAC Multi Bit Error
        DCD     EDAC_SBE_IRQHandler        ; 77: EDAC Single Bit Error
        DCD     PA0_IRQHandler             ; 78: PORTA 0
        DCD     PA1_IRQHandler             ; 79: PORTA 1
        DCD     PA2_IRQHandler             ; 80: PORTA 2
        DCD     PA3_IRQHandler             ; 81: PORTA 3
        DCD     PA4_IRQHandler             ; 82: PORTA 4
        DCD     PA5_IRQHandler             ; 83: PORTA 5
        DCD     PA6_IRQHandler             ; 84: PORTA 6
        DCD     PA7_IRQHandler             ; 85: PORTA 7
        DCD     PA8_IRQHandler             ; 86: PORTA 8
        DCD     PA9_IRQHandler             ; 87: PORTA 9
        DCD     PA10_IRQHandler            ; 88: PORTA 10
        DCD     PA11_IRQHandler            ; 89: PORTA 11
        DCD     PA12_IRQHandler            ; 90: PORTA 12
        DCD     PA13_IRQHandler            ; 91: PORTA 13
        DCD     PA14_IRQHandler            ; 92: PORTA 14
        DCD     PA15_IRQHandler            ; 93: PORTA 15
        DCD     PB0_IRQHandler             ; 94: PORTB 0
        DCD     PB1_IRQHandler             ; 95: PORTB 1
        DCD     PB2_IRQHandler             ; 96: PORTB 2
        DCD     PB3_IRQHandler             ; 97: PORTB 3
        DCD     PB4_IRQHandler             ; 98: PORTB 4
        DCD     PB5_IRQHandler             ; 99: PORTB 5
        DCD     PB6_IRQHandler             ; 100: PORTB 6
        DCD     PB7_IRQHandler             ; 101: PORTB 7
        DCD     PB8_IRQHandler             ; 102: PORTB 8
        DCD     PB9_IRQHandler             ; 103: PORTB 9
        DCD     PB10_IRQHandler            ; 104: PORTB 10
        DCD     PB11_IRQHandler            ; 105: PORTB 11
        DCD     PB12_IRQHandler            ; 106: PORTB 12
        DCD     PB13_IRQHandler            ; 107: PORTB 13
        DCD     PB14_IRQHandler            ; 108: PORTB 14
        DCD     PB15_IRQHandler            ; 109: PORTB 15
        DCD     PC0_IRQHandler             ; 110: PORTC 0
        DCD     PC1_IRQHandler             ; 111: PORTC 1
        DCD     PC2_IRQHandler             ; 112: PORTC 2
        DCD     PC3_IRQHandler             ; 113: PORTC 3
        DCD     PC4_IRQHandler             ; 114: PORTC 4
        DCD     PC5_IRQHandler             ; 115: PORTC 5
        DCD     PC6_IRQHandler             ; 116: PORTC 6
        DCD     PC7_IRQHandler             ; 117: PORTC 7
        DCD     PC8_IRQHandler             ; 118: PORTC 8
        DCD     PC9_IRQHandler             ; 119: PORTC 9
        DCD     PC10_IRQHandler            ; 120: PORTC 10
        DCD     PC11_IRQHandler            ; 121: PORTC 11
        DCD     PC12_IRQHandler            ; 122: PORTC 12
        DCD     PC13_IRQHandler            ; 123: PORTC 13
        DCD     PC14_IRQHandler            ; 124: PORTC 14
        DCD     PC15_IRQHandler            ; 125: PORTC 15
        DCD     PD0_IRQHandler             ; 126: PORTD 0
        DCD     PD1_IRQHandler             ; 127: PORTD 1
        DCD     PD2_IRQHandler             ; 128: PORTD 2
        DCD     PD3_IRQHandler             ; 129: PORTD 3
        DCD     PD4_IRQHandler             ; 130: PORTD 4
        DCD     PD5_IRQHandler             ; 131: PORTD 5
        DCD     PD6_IRQHandler             ; 132: PORTD 6
        DCD     PD7_IRQHandler             ; 133: PORTD 7
        DCD     PD8_IRQHandler             ; 134: PORTD 8
        DCD     PD9_IRQHandler             ; 135: PORTD 9
        DCD     PD10_IRQHandler            ; 136: PORTD 10
        DCD     PD11_IRQHandler            ; 137: PORTD 11
        DCD     PD12_IRQHandler            ; 138: PORTD 12
        DCD     PD13_IRQHandler            ; 139: PORTD 13
        DCD     PD14_IRQHandler            ; 140: PORTD 14
        DCD     PD15_IRQHandler            ; 141: PORTD 15
        DCD     PE0_IRQHandler             ; 142: PORTE 0
        DCD     PE1_IRQHandler             ; 143: PORTE 1
        DCD     PE2_IRQHandler             ; 144: PORTE 2
        DCD     PE3_IRQHandler             ; 145: PORTE 3
        DCD     PE4_IRQHandler             ; 146: PORTE 4
        DCD     PE5_IRQHandler             ; 147: PORTE 5
        DCD     PE6_IRQHandler             ; 148: PORTE 6
        DCD     PE7_IRQHandler             ; 149: PORTE 7
        DCD     PE8_IRQHandler             ; 150: PORTE 8
        DCD     PE9_IRQHandler             ; 151: PORTE 9
        DCD     PE10_IRQHandler            ; 152: PORTE 10
        DCD     PE11_IRQHandler            ; 153: PORTE 11
        DCD     PE12_IRQHandler            ; 154: PORTE 12
        DCD     PE13_IRQHandler            ; 155: PORTE 13
        DCD     PE14_IRQHandler            ; 156: PORTE 14
        DCD     PE15_IRQHandler            ; 157: PORTE 15
        DCD     PF0_IRQHandler             ; 158: PORTF 0
        DCD     PF1_IRQHandler             ; 159: PORTF 1
        DCD     PF2_IRQHandler             ; 160: PORTF 2
        DCD     PF3_IRQHandler             ; 161: PORTF 3
        DCD     PF4_IRQHandler             ; 162: PORTF 4
        DCD     PF5_IRQHandler             ; 163: PORTF 5
        DCD     PF6_IRQHandler             ; 164: PORTF 6
        DCD     PF7_IRQHandler             ; 165: PORTF 7
        DCD     PF8_IRQHandler             ; 166: PORTF 8
        DCD     PF9_IRQHandler             ; 167: PORTF 9
        DCD     PF10_IRQHandler            ; 168: PORTF 10
        DCD     PF11_IRQHandler            ; 169: PORTF 11
        DCD     PF12_IRQHandler            ; 170: PORTF 12
        DCD     PF13_IRQHandler            ; 171: PORTF 13
        DCD     PF14_IRQHandler            ; 172: PORTF 14
        DCD     PF15_IRQHandler            ; 173: PORTF 15
        DCD     DMA_Active_0_IRQHandler    ; 174: DMA Active 0
        DCD     DMA_Active_1_IRQHandler    ; 175: DMA Active 1
        DCD     DMA_Active_2_IRQHandler    ; 176: DMA Active 2
        DCD     DMA_Active_3_IRQHandler    ; 177: DMA Active 3
        DCD     DMA_Done_0_IRQHandler      ; 178: DMA Done 0
        DCD     DMA_Done_1_IRQHandler      ; 179: DMA Done 1
        DCD     DMA_Done_2_IRQHandler      ; 180: DMA Done 2
        DCD     DMA_Done_3_IRQHandler      ; 181: DMA Done 3
        DCD     OC182_IRQHandler           ; 182: Always 0
        DCD     OC183_IRQHandler           ; 183: Always 0
        DCD     OC184_IRQHandler           ; 184: Always 0
        DCD     OC185_IRQHandler           ; 185: Always 0
        DCD     OC186_IRQHandler           ; 186: Always 0
        DCD     OC187_IRQHandler           ; 187: Always 0
        DCD     OC188_IRQHandler           ; 188: Always 0
        DCD     OC189_IRQHandler           ; 189: Always 0
        DCD     OC190_IRQHandler           ; 190: Always 0
        DCD     OC191_IRQHandler           ; 191: Always 0
        DCD     OC192_IRQHandler           ; 192: Always 0
        DCD     OC193_IRQHandler           ; 193: Always 0
        DCD     FPU_IRQHandler             ; 194: FPU
        DCD     TXEV_IRQHandler            ; 195: TXEV

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;
;; Default interrupt handlers.
;;
        THUMB
        PUBWEAK Reset_Handler
        SECTION .text:CODE:REORDER:NOROOT(2)
Reset_Handler

        LDR     R0, =SystemInit
        BLX     R0
        LDR     R0, =__iar_program_start
        BX      R0

        PUBWEAK NMI_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
NMI_Handler
        B NMI_Handler

        PUBWEAK HardFault_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
HardFault_Handler

        REQUIRE8              ; For API compatability used 8 byte stack frame
        PRESERVE8  
        MOVS    r3, #127	
        PUSH    {r3-r7,lr}    ; Save contents of other registers for display (R3 included to meet REQUIRE8)e
        MOV     R0,SP         ; Pass current SP as parameter
        MOV     R1,LR
        BL      HardFault_Handler_C
        POP     {r3-r7,pc}    ; Restore all registers
//        B HardFault_Handler

        PUBWEAK MemManage_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
MemManage_Handler
        B MemManage_Handler

        PUBWEAK BusFault_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
BusFault_Handler
        B BusFault_Handler

        PUBWEAK UsageFault_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
UsageFault_Handler
        B UsageFault_Handler

        PUBWEAK SVC_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
SVC_Handler
        B SVC_Handler

        PUBWEAK DebugMon_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
DebugMon_Handler
        B DebugMon_Handler

        PUBWEAK PendSV_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
PendSV_Handler
        B PendSV_Handler

        PUBWEAK SysTick_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
SysTick_Handler
        B SysTick_Handler
        
 ; externaal interrupts start here   *****************
 
        PUBWEAK OC0_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
OC0_IRQHandler  
        B OC0_IRQHandler
        
        PUBWEAK OC1_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
OC1_IRQHandler  
        B OC1_IRQHandler
                 
        PUBWEAK OC2_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
OC2_IRQHandler  
        B OC2_IRQHandler
        
        PUBWEAK OC3_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
OC3_IRQHandler  
        B OC3_IRQHandler
        
        PUBWEAK OC4_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
OC4_IRQHandler  
        B OC4_IRQHandler
        
        PUBWEAK OC5_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
OC5_IRQHandler  
        B OC5_IRQHandler
        
        PUBWEAK OC6_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
OC6_IRQHandler  
        B OC6_IRQHandler
        
        PUBWEAK OC7_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
OC7_IRQHandler  
        B OC7_IRQHandler
        
        PUBWEAK OC8_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
OC8_IRQHandler  
        B OC8_IRQHandler
        
        PUBWEAK OC9_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
OC9_IRQHandler  
        B OC9_IRQHandler
        
        PUBWEAK OC10_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
OC10_IRQHandler  
        B OC10_IRQHandler
        
        PUBWEAK OC11_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
OC11_IRQHandler  
        B OC11_IRQHandler
        
        PUBWEAK OC12_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
OC12_IRQHandler  
        B OC12_IRQHandler
        
        PUBWEAK OC13_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
OC13_IRQHandler  
        B OC13_IRQHandler
        
        PUBWEAK OC14_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
OC14_IRQHandler  
        B OC14_IRQHandler
        
        PUBWEAK OC15_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
OC15_IRQHandler  
        B OC15_IRQHandler
        
        PUBWEAK SPI0_TX_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
SPI0_TX_IRQHandler  
        B SPI0_TX_IRQHandler
        
        PUBWEAK SPI0_RX_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
SPI0_RX_IRQHandler  
        B SPI0_RX_IRQHandler
        
        PUBWEAK SPI1_TX_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
SPI1_TX_IRQHandler  
        B SPI1_TX_IRQHandler
        
        PUBWEAK SPI1_RX_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
SPI1_RX_IRQHandler  
        B SPI1_RX_IRQHandler
        
        PUBWEAK SPI2_TX_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
SPI2_TX_IRQHandler  
        B SPI2_TX_IRQHandler
        
        PUBWEAK SPI2_RX_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
SPI2_RX_IRQHandler  
        B SPI2_RX_IRQHandler
        
        PUBWEAK SPI3_TX_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
SPI3_TX_IRQHandler  
        B SPI3_TX_IRQHandler
        
        PUBWEAK SPI3_RX_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
SPI3_RX_IRQHandler  
        B SPI3_RX_IRQHandler
        
        PUBWEAK UART0_TX_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
UART0_TX_IRQHandler  
        B UART0_TX_IRQHandler
        
        PUBWEAK UART0_RX_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
UART0_RX_IRQHandler  
        B UART0_RX_IRQHandler
        
        PUBWEAK UART1_TX_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
UART1_TX_IRQHandler  
        B UART1_TX_IRQHandler
        
        PUBWEAK UART1_RX_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
UART1_RX_IRQHandler  
        B UART1_RX_IRQHandler
        
        PUBWEAK UART2_TX_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
UART2_TX_IRQHandler  
        B UART2_TX_IRQHandler
        
        PUBWEAK UART2_RX_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
UART2_RX_IRQHandler  
        B UART2_RX_IRQHandler
        
        PUBWEAK I2C0_MS_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
I2C0_MS_IRQHandler  
        B I2C0_MS_IRQHandler
        
        PUBWEAK I2C0_SL_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
I2C0_SL_IRQHandler  
        B I2C0_SL_IRQHandler
        
        PUBWEAK I2C1_MS_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
I2C1_MS_IRQHandler  
        B I2C1_MS_IRQHandler
        
        PUBWEAK I2C1_SL_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
I2C1_SL_IRQHandler  
        B I2C1_SL_IRQHandler
        
        PUBWEAK I2C2_MS_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
I2C2_MS_IRQHandler  
        B I2C2_MS_IRQHandler
        
        PUBWEAK I2C2_SL_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
I2C2_SL_IRQHandler  
        B I2C2_SL_IRQHandler
        
        PUBWEAK Ethernet_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
Ethernet_IRQHandler  
        B Ethernet_IRQHandler
        
        PUBWEAK OC37_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
OC37_IRQHandler  
        B OC37_IRQHandler
        
        PUBWEAK SpW_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
SpW_IRQHandler  
        B SpW_IRQHandler
        
        PUBWEAK OC39_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
OC39_IRQHandler  
        B OC39_IRQHandler
        
        PUBWEAK DAC0_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
DAC0_IRQHandler  
        B DAC0_IRQHandler
        
        PUBWEAK DAC1_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
DAC1_IRQHandler  
        B DAC1_IRQHandler
        
        PUBWEAK TRNG_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
TRNG_IRQHandler  
        B TRNG_IRQHandler
        
        PUBWEAK DMA_Error_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
DMA_Error_IRQHandler  
        B DMA_Error_IRQHandler
        
        PUBWEAK ADC_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
ADC_IRQHandler  
        B ADC_IRQHandler
        
        PUBWEAK LoCLK_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
LoCLK_IRQHandler  
        B LoCLK_IRQHandler
        
        PUBWEAK LVD_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
LVD_IRQHandler  
        B LVD_IRQHandler
        
        PUBWEAK WDT_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
WDT_IRQHandler  
        B WDT_IRQHandler
        
        PUBWEAK TIM0_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
TIM0_IRQHandler  
        B TIM0_IRQHandler
        
        PUBWEAK TIM1_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
TIM1_IRQHandler  
        B TIM1_IRQHandler
        
        PUBWEAK TIM2_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
TIM2_IRQHandler  
        B TIM2_IRQHandler
        
        PUBWEAK TIM3_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
TIM3_IRQHandler  
        B TIM3_IRQHandler
        
        PUBWEAK TIM4_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
TIM4_IRQHandler  
        B TIM4_IRQHandler
        
        PUBWEAK TIM5_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
TIM5_IRQHandler  
        B TIM5_IRQHandler
        
        PUBWEAK TIM6_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
TIM6_IRQHandler  
        B TIM6_IRQHandler
        
        PUBWEAK TIM7_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
TIM7_IRQHandler  
        B TIM7_IRQHandler
        
        PUBWEAK TIM8_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
TIM8_IRQHandler  
        B TIM8_IRQHandler
        
        PUBWEAK TIM9_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
TIM9_IRQHandler  
        B TIM9_IRQHandler
        
        PUBWEAK TIM10_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
TIM10_IRQHandler  
        B TIM10_IRQHandler
        
        PUBWEAK TIM11_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
TIM11_IRQHandler  
        B TIM11_IRQHandler
        
        PUBWEAK TIM12_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
TIM12_IRQHandler  
        B TIM12_IRQHandler
        
        PUBWEAK TIM13_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
TIM13_IRQHandler  
        B TIM13_IRQHandler
        
        PUBWEAK TIM14_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
TIM14_IRQHandler  
        B TIM14_IRQHandler
        
        PUBWEAK TIM15_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
TIM15_IRQHandler  
        B TIM15_IRQHandler
        
        PUBWEAK TIM16_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
TIM16_IRQHandler  
        B TIM16_IRQHandler
        
        PUBWEAK TIM17_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
TIM17_IRQHandler  
        B TIM17_IRQHandler
        
        PUBWEAK TIM18_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
TIM18_IRQHandler  
        B TIM18_IRQHandler
        
        PUBWEAK TIM19_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
TIM19_IRQHandler  
        B TIM19_IRQHandler
        
        PUBWEAK TIM20_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
TIM20_IRQHandler  
        B TIM20_IRQHandler
        
        PUBWEAK TIM21_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
TIM21_IRQHandler  
        B TIM21_IRQHandler
        
        PUBWEAK TIM22_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
TIM22_IRQHandler  
        B TIM22_IRQHandler
        
        PUBWEAK TIM23_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
TIM23_IRQHandler  
        B TIM23_IRQHandler
        
        PUBWEAK CAN0_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
CAN0_IRQHandler  
        B CAN0_IRQHandler
        
        PUBWEAK OC73_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
OC73_IRQHandler  
        B OC73_IRQHandler
        
        PUBWEAK CAN1_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
CAN1_IRQHandler  
        B CAN1_IRQHandler
        
        PUBWEAK OC75_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
OC75_IRQHandler  
        B OC75_IRQHandler
        
        PUBWEAK EDAC_MBE_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
EDAC_MBE_IRQHandler  
        B EDAC_MBE_IRQHandler
        
        PUBWEAK EDAC_SBE_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
EDAC_SBE_IRQHandler  
        B EDAC_SBE_IRQHandler
        
        PUBWEAK PA0_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
PA0_IRQHandler  
        B PA0_IRQHandler
        
        PUBWEAK PA1_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
PA1_IRQHandler  
        B PA1_IRQHandler
        
        PUBWEAK PA2_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
PA2_IRQHandler  
        B PA2_IRQHandler
        
        PUBWEAK PA3_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
PA3_IRQHandler  
        B PA3_IRQHandler
        
        PUBWEAK PA4_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
PA4_IRQHandler  
        B PA4_IRQHandler
        
        PUBWEAK PA5_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
PA5_IRQHandler  
        B PA5_IRQHandler
        
        PUBWEAK PA6_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
PA6_IRQHandler  
        B PA6_IRQHandler
        
        PUBWEAK PA7_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
PA7_IRQHandler  
        B PA7_IRQHandler
        
        PUBWEAK PA8_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
PA8_IRQHandler  
        B PA8_IRQHandler
        
        PUBWEAK PA9_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
PA9_IRQHandler  
        B PA9_IRQHandler
        
        PUBWEAK PA10_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
PA10_IRQHandler  
        B PA10_IRQHandler
        
        PUBWEAK PA11_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
PA11_IRQHandler  
        B PA11_IRQHandler
        
        PUBWEAK PA12_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
PA12_IRQHandler  
        B PA12_IRQHandler
        
        PUBWEAK PA13_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
PA13_IRQHandler  
        B PA13_IRQHandler
        
        PUBWEAK PA14_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
PA14_IRQHandler  
        B PA14_IRQHandler
        
        PUBWEAK PA15_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
PA15_IRQHandler  
        B PA15_IRQHandler
        
        PUBWEAK PB0_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
PB0_IRQHandler  
        B PB0_IRQHandler
        
        PUBWEAK PB1_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
PB1_IRQHandler  
        B PB1_IRQHandler
        
        PUBWEAK PB2_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
PB2_IRQHandler  
        B PB2_IRQHandler
        
        PUBWEAK PB3_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
PB3_IRQHandler  
        B PB3_IRQHandler
        
        PUBWEAK PB4_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
PB4_IRQHandler  
        B PB4_IRQHandler
        
        PUBWEAK PB5_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
PB5_IRQHandler  
        B PB5_IRQHandler
        
        PUBWEAK PB6_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
PB6_IRQHandler  
        B PB6_IRQHandler
        
        PUBWEAK PB7_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
PB7_IRQHandler  
        B PB7_IRQHandler
        
        PUBWEAK PB8_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
PB8_IRQHandler  
        B PB8_IRQHandler
        
        PUBWEAK PB9_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
PB9_IRQHandler  
        B PB9_IRQHandler
        
        PUBWEAK PB10_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
PB10_IRQHandler  
        B PB10_IRQHandler
        
        PUBWEAK PB11_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
PB11_IRQHandler  
        B PB11_IRQHandler
        
        PUBWEAK PB12_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
PB12_IRQHandler  
        B PB12_IRQHandler
        
        PUBWEAK PB13_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
PB13_IRQHandler  
        B PB13_IRQHandler
        
        PUBWEAK PB14_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
PB14_IRQHandler  
        B PB14_IRQHandler
        
        PUBWEAK PB15_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
PB15_IRQHandler  
        B PB15_IRQHandler
        
        PUBWEAK PC0_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
PC0_IRQHandler  
        B PC0_IRQHandler
        
        PUBWEAK PC1_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
PC1_IRQHandler  
        B PC1_IRQHandler
        
        PUBWEAK PC2_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
PC2_IRQHandler  
        B PC2_IRQHandler
        
        PUBWEAK PC3_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
PC3_IRQHandler  
        B PC3_IRQHandler
        
        PUBWEAK PC4_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
PC4_IRQHandler  
        B PC4_IRQHandler
        
        PUBWEAK PC5_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
PC5_IRQHandler  
        B PC5_IRQHandler
        
        PUBWEAK PC6_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
PC6_IRQHandler  
        B PC6_IRQHandler
        
        PUBWEAK PC7_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
PC7_IRQHandler  
        B PC7_IRQHandler
        
        PUBWEAK PC8_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
PC8_IRQHandler  
        B PC8_IRQHandler
        
        PUBWEAK PC9_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
PC9_IRQHandler  
        B PC9_IRQHandler
        
        PUBWEAK PC10_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
PC10_IRQHandler  
        B PC10_IRQHandler
        
        PUBWEAK PC11_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
PC11_IRQHandler  
        B PC11_IRQHandler
        
        PUBWEAK PC12_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
PC12_IRQHandler  
        B PC12_IRQHandler
        
        PUBWEAK PC13_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
PC13_IRQHandler  
        B PC13_IRQHandler
        
        PUBWEAK PC14_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
PC14_IRQHandler  
        B PC14_IRQHandler
        
        PUBWEAK PC15_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
PC15_IRQHandler  
        B PC15_IRQHandler
        
        PUBWEAK PD0_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
PD0_IRQHandler  
        B PD0_IRQHandler
        
        PUBWEAK PD1_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
PD1_IRQHandler  
        B PD1_IRQHandler
        
        PUBWEAK PD2_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
PD2_IRQHandler  
        B PD2_IRQHandler
        
        PUBWEAK PD3_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
PD3_IRQHandler  
        B PD3_IRQHandler
        
        PUBWEAK PD4_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
PD4_IRQHandler  
        B PD4_IRQHandler
        
        PUBWEAK PD5_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
PD5_IRQHandler  
        B PD5_IRQHandler
        
        PUBWEAK PD6_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
PD6_IRQHandler  
        B PD6_IRQHandler
        
        PUBWEAK PD7_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
PD7_IRQHandler  
        B PD7_IRQHandler
        
        PUBWEAK PD8_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
PD8_IRQHandler  
        B PD8_IRQHandler
        
        PUBWEAK PD9_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
PD9_IRQHandler  
        B PD9_IRQHandler
        
        PUBWEAK PD10_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
PD10_IRQHandler  
        B PD10_IRQHandler
        
        PUBWEAK PD11_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
PD11_IRQHandler  
        B PD11_IRQHandler
        
        PUBWEAK PD12_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
PD12_IRQHandler  
        B PD12_IRQHandler
        
        PUBWEAK PD13_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
PD13_IRQHandler  
        B PD13_IRQHandler
        
        PUBWEAK PD14_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
PD14_IRQHandler  
        B PD14_IRQHandler
        
        PUBWEAK PD15_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
PD15_IRQHandler  
        B PD15_IRQHandler
        
        PUBWEAK PE0_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
PE0_IRQHandler  
        B PE0_IRQHandler
        
        PUBWEAK PE1_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
PE1_IRQHandler  
        B PE1_IRQHandler
        
        PUBWEAK PE2_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
PE2_IRQHandler  
        B PE2_IRQHandler
        
        PUBWEAK PE3_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
PE3_IRQHandler  
        B PE3_IRQHandler
        
        PUBWEAK PE4_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
PE4_IRQHandler  
        B PE4_IRQHandler
        
        PUBWEAK PE5_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
PE5_IRQHandler  
        B PE5_IRQHandler
        
        PUBWEAK PE6_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
PE6_IRQHandler  
        B PE6_IRQHandler
        
        PUBWEAK PE7_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
PE7_IRQHandler  
        B PE7_IRQHandler
        
        PUBWEAK PE8_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
PE8_IRQHandler  
        B PE8_IRQHandler
        
        PUBWEAK PE9_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
PE9_IRQHandler  
        B PE9_IRQHandler
        
        PUBWEAK PE10_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
PE10_IRQHandler  
        B PE10_IRQHandler
        
        PUBWEAK PE11_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
PE11_IRQHandler  
        B PE11_IRQHandler
        
        PUBWEAK PE12_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
PE12_IRQHandler  
        B PE12_IRQHandler
        
        PUBWEAK PE13_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
PE13_IRQHandler  
        B PE13_IRQHandler
        
        PUBWEAK PE14_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
PE14_IRQHandler  
        B PE14_IRQHandler
        
        PUBWEAK PE15_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
PE15_IRQHandler  
        B PE15_IRQHandler
        
        PUBWEAK PF0_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
PF0_IRQHandler  
        B PF0_IRQHandler
        
        PUBWEAK PF1_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
PF1_IRQHandler  
        B PF1_IRQHandler
        
        PUBWEAK PF2_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
PF2_IRQHandler  
        B PF2_IRQHandler
        
        PUBWEAK PF3_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
PF3_IRQHandler  
        B PF3_IRQHandler
        
        PUBWEAK PF4_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
PF4_IRQHandler  
        B PF4_IRQHandler
        
        PUBWEAK PF5_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
PF5_IRQHandler  
        B PF5_IRQHandler
        
        PUBWEAK PF6_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
PF6_IRQHandler  
        B PF6_IRQHandler
        
        PUBWEAK PF7_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
PF7_IRQHandler  
        B PF7_IRQHandler
        
        PUBWEAK PF8_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
PF8_IRQHandler  
        B PF8_IRQHandler
        
        PUBWEAK PF9_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
PF9_IRQHandler  
        B PF9_IRQHandler
        
        PUBWEAK PF10_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
PF10_IRQHandler  
        B PF10_IRQHandler
        
        PUBWEAK PF11_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
PF11_IRQHandler  
        B PF11_IRQHandler
        
        PUBWEAK PF12_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
PF12_IRQHandler  
        B PF12_IRQHandler
        
        PUBWEAK PF13_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
PF13_IRQHandler  
        B PF13_IRQHandler
        
        PUBWEAK PF14_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
PF14_IRQHandler  
        B PF14_IRQHandler
        
        PUBWEAK PF15_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
PF15_IRQHandler  
        B PF15_IRQHandler
        
        PUBWEAK DMA_Active_0_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
DMA_Active_0_IRQHandler  
        B DMA_Active_0_IRQHandler
        
        PUBWEAK DMA_Active_1_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
DMA_Active_1_IRQHandler  
        B DMA_Active_1_IRQHandler
        
        PUBWEAK DMA_Active_2_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
DMA_Active_2_IRQHandler  
        B DMA_Active_2_IRQHandler
        
        PUBWEAK DMA_Active_3_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
DMA_Active_3_IRQHandler  
        B DMA_Active_3_IRQHandler
        
        PUBWEAK DMA_Done_0_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
DMA_Done_0_IRQHandler  
        B DMA_Done_0_IRQHandler
        
        PUBWEAK DMA_Done_1_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
DMA_Done_1_IRQHandler  
        B DMA_Done_1_IRQHandler
        
        PUBWEAK DMA_Done_2_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
DMA_Done_2_IRQHandler  
        B DMA_Done_2_IRQHandler
        
        PUBWEAK DMA_Done_3_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
DMA_Done_3_IRQHandler  
        B DMA_Done_3_IRQHandler
        
        PUBWEAK OC182_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
OC182_IRQHandler  
        B OC182_IRQHandler
        
        PUBWEAK OC183_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
OC183_IRQHandler  
        B OC183_IRQHandler
        
        PUBWEAK OC184_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
OC184_IRQHandler  
        B OC184_IRQHandler
        
        PUBWEAK OC185_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
OC185_IRQHandler  
        B OC185_IRQHandler
        
        PUBWEAK OC186_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
OC186_IRQHandler  
        B OC186_IRQHandler
        
        PUBWEAK OC187_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
OC187_IRQHandler  
        B OC187_IRQHandler
        
        PUBWEAK OC188_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
OC188_IRQHandler  
        B OC188_IRQHandler
        
        PUBWEAK OC189_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
OC189_IRQHandler  
        B OC189_IRQHandler
        
        PUBWEAK OC190_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
OC190_IRQHandler  
        B OC190_IRQHandler
        
        PUBWEAK OC191_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
OC191_IRQHandler  
        B OC191_IRQHandler
        
        PUBWEAK OC192_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
OC192_IRQHandler  
        B OC192_IRQHandler
        
        PUBWEAK OC193_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
OC193_IRQHandler  
        B OC193_IRQHandler
        
        PUBWEAK FPU_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
FPU_IRQHandler  
        B FPU_IRQHandler
        
        PUBWEAK TXEV_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
TXEV_IRQHandler  
        B TXEV_IRQHandler

        END
        
        
/****END OF FILE****/
