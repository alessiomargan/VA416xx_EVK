
DESCRIPTION
===========
Flash loader for Vorago VA416X0 external/internal SPI FRAM memory (Cypress FM25V20A and similar)

CONFIGURATION
=============
The project contains configuration:

- FlashVA416X0_SPI_FRAM
Builds a flash loader for Vorago VA416X0 external FRAM memory.
Output file: FlashVA416X0_SPI_FRAM.out
Macro file: FlashVA416X0_reset.mac

USAGE
=====
The flash loader expects program code to be linked
in the range 0x00000000 - 0x0003FFFF.
