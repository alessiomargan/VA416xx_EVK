
DESCRIPTION
===========
Flash loader for Vorago VA416X0 external parallel 256K x 16 FRAM memory (Cypress FM22L16 and similar)

CONFIGURATION
=============
The project contains configuration:

- FlashVA416X0_EBI_FRAM
Builds a flash loader for Vorago VA416X0 external parallel FRAM memory.
Output file: FlashVA416X0_EBI_FRAM.out
Macro file: FlashVA416X0_reset.mac

USAGE
=====
The flash loader expects program code to be linked in the range 0x00000000 - 0x0003FFFF. 
Only the first half (256KB) of this memory is booted (copied to internal RAM) by the 
VA416x0 on startup. The pin 'EBIBOOT' must be a logic '1' for the VA416x0 to boot from this 
memory source out of a reset.
