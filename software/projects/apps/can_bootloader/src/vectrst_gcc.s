;/***************************************************************************/
;/***************************************************************************/
;/* vector_reset()
; * Jump to reset vector
; */

  .section .text
  .weak vector_reset
  .type vector_reset, %function

vector_reset:
              ldr r0,=0xe000ed08 /* Set R0 to VTOR address                */
              ldr r1,[r0]        /* Load VTOR                             */ 
              ldr r0,[r1]        /* Load initial MSP value                */
              msr msp,r0         /* Set SP value (assume MSP is selected) */
              ldr r0,[r1, #4]    /*  Load reset vector                    */
              bx r0              /* Branch to reset handler               */  
              nop
