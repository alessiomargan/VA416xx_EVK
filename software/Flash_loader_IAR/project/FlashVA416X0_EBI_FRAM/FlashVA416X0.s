/**************************************************
 *
 * Vorago VA416X0 flashloader low level code.
 *
 * Copyright 2020 VORAGO. All rights reserved.
 *
 * $Revision: 1 $
 *
 **************************************************/

        MODULE  ?vector_table_vorago

        AAPCS INTERWORK, RWPI_COMPATIBLE
        PRESERVE8
        REQUIRE8

        SECTION CSTACK:DATA:NOROOT(3)
        SECTION .intvec_vorago:CODE:ROOT(2)

        PUBLIC  __vector_table_vorago
        EXTERN  FlashInitEntry

        DATA

__vector_table_vorago
        DCD     sfe(CSTACK)
        DCD     FlashInitEntry

        DCD     0
        DCD     HardFault_Handler
        DCD     0
        DCD     0
        DCD     0
        DCD     0
        DCD     0
        DCD     0
        DCD     0
        DCD     0
        DCD     0
        DCD     0
        DCD     0
        DCD     0


        SECTION .text:CODE:REORDER:NOROOT(2)
        EXTERN __HardFault_Handler
        THUMB

HardFault_Handler:
        REQUIRE8
        PRESERVE8
        PUSH    {R3-R7,LR}
        BL      __HardFault_Handler
        POP     {R3-R7,PC}
        BX      LR

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;
;; Safe APB access.
;;

        PUBWEAK WRITE_MEM
        SECTION .text:CODE:REORDER:NOROOT(5)
        THUMB

WRITE_MEM:
        STR     R1, [R0]
        BX      LR

        PUBWEAK READ_MEM
        SECTION .text:CODE:REORDER:NOROOT(5)
        THUMB

READ_MEM:
        LDR     R0, [R0]
        BX      LR

        END
