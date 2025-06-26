#ifndef ERASER_H
#define ERASER_H

#define BOOT_FROM_SPI3                              // Chip is booting from SPI3
#define SZDEV                       (0x00040000)    // Device Size in Bytes (256KB)
#define SPIC_DATA_BMSTOP            (0x80000000)    // SPIC DATA: BMSTOP

#ifdef BOOT_FROM_SPI0
    #define BOOT_SPI                VOR_SPI0
    #define BOOT_SPI_RESET          SYSCONFIG_PERIPHERAL_RESET_SPI0_Msk
    #define BOOT_SPI_CLK_ENABLE     CLK_ENABLE_SPI0
#endif

#ifdef BOOT_FROM_SPI1
    #define BOOT_SPI                VOR_SPI1
    #define BOOT_SPI_RESET          SYSCONFIG_PERIPHERAL_RESET_SPI1_Msk
    #define BOOT_SPI_CLK_ENABLE     CLK_ENABLE_SPI1
#endif

#ifdef BOOT_FROM_SPI2
    #define BOOT_SPI                VOR_SPI2
    #define BOOT_SPI_RESET          SYSCONFIG_PERIPHERAL_RESET_SPI2_Msk
    #define BOOT_SPI_CLK_ENABLE     CLK_ENABLE_SPI2
#endif

#ifdef BOOT_FROM_SPI3
    #define BOOT_SPI                VOR_SPI3
    #define BOOT_SPI_RESET          SYSCONFIG_PERIPHERAL_RESET_SPI3_Msk
    #define BOOT_SPI_CLK_ENABLE     CLK_ENABLE_SPI3
#endif

// FM25V20A Commands
#define WREN                        (0x06)
#define WRDI                        (0x04)
#define RDSR                        (0x05)
#define WRSR                        (0x01)
#define READ                        (0x03)
#define WRITE                       (0x02)
#define RDID                        (0x9F) 
#define TTZ2564_SE                  (0x20) 
#define TTZ2564_BE                  (0x52)
#define TTZ2564_CE                  (0x60) 

#endif  // ERASER_H
