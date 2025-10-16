/**
 * @file afe11612.h
 * @brief AFE11612-SEP Radiation-Tolerant, Analog Monitor and Controller
 * with Multichannel ADC, DACs, and Temperature Sensors
 *
 * This file defines the interface to the AFE11612
 *
 * @author Alessio Margan
 * @date October 13, 2025
 *
 * @version 1.0
 *
 * @copyright Copyright (c) 2025 IIT
 *
 * @note This implementation requires the VA416xx HAL SPI, and CAN drivers
 */

#include "board.h"
#include "afe11612.h"
#include "hal_utils.h"

#include "va416xx_hal.h"
#include "va416xx_hal_spi.h"
#include "va416xx_hal_dma.h"
#include "va416xx_hal_canbus.h"

#include <inttypes.h>

#define ALARM_PIN    0
#define ALARM_PORT   PORTA

#define RESET_PIN    1
#define RESET_PORT   PORTA

static hal_spi_handle_t hspi;
static volatile hal_status_t spiStat;
static volatile bool rxDmaDone = true;

void ConfigureAFE11612(void) {
    
    // config SPI
    HAL_UNLOCK(&hspi);
    hspi.spi = VOR_SPI2;
    hspi.init.blockmode = true;
    hspi.init.bmstall = false;
    hspi.init.clkDiv = 10;   
    hspi.init.loopback = false;
    hspi.init.mdlycap = false;
    hspi.init.mode = hal_spi_clkmode_1;
    hspi.init.ms = hal_spi_ms_master;
    hspi.init.chipSelect = 0;
    hspi.init.wordLen = SPI_WORDLEN;
    spiStat = HAL_Spi_Init(&hspi);
    if(spiStat != hal_status_ok) {
        printf("Error: HAL_Spi_Init() status: %s\n", HAL_StatusToString(spiStat));
        return;
    }
    
    // keep RESET low for a while
    Pin_off(RESET_PORT, RESET_PIN);
    for(volatile int i=0; i<10000; i++) { asm("nop"); }
    // then release it
    Pin_on(RESET_PORT, RESET_PIN);
}


void AFE11612_testDeviceId(void) {
    uint16_t val;
    bool ret;

    //__disable_irq();
    ret = AFE11612_ReadReg(AFE11612_REG_DEVICE_ID, &val);
    //__enable_irq();

    if ( ! ret) {
        printf("AFE11612 read failed\n");
        return;
    }
  
    if ( val != 0x1220 ) {
        printf("Error: AFE11612 Device ID mismatch: 0x%04X\n", val);
    } else {
        printf("AFE11612 Device ID match: 0x%04X\n", val);
    }
}


bool AFE11612_ReadReg(uint8_t reg, uint16_t *out) {

    volatile afe11612_spi_cmd_t cmd;

    if (out == NULL) return false;

    /* Set read bit (bit7 = 1) and place 7-bit address in bits[6:0]
     * The device expects a 24-bit transaction: [R/W + Addr(7)] [MSB data] [LSB data]
     * First transfer: send the read command and receive don't-care
     * Second transfer: send 3 dummy bytes and receive the 3 response bytes (address + data)
     */
    cmd.spiword[0] = (1u << 7) | (reg & 0x7F);
    cmd.spiword[1] = 0x00;
    cmd.spiword[2] = 0x00;

    /* Send read command (3 bytes) */
    //spiStat = HAL_Spi_Transmit(&hspi, (void*)cmd.spiword, SPI_WORDS, 0, false);
    spiStat = HAL_Spi_TransmitInt(&hspi, (void*)cmd.spiword, SPI_WORDS);
    if (spiStat != hal_status_ok) {
        return false;
    }
    
    while(hspi.state == hal_spi_state_busy); // wait for complete
    /* need for CS transition*/
    //for(volatile int i=0; i<10; i++) { asm("nop"); }
    
    /* Receive response (3 bytes) */
    //spiStat = HAL_Spi_Receive(&hspi, (void*)cmd.spiword, SPI_WORDS, 0);
    spiStat = HAL_Spi_ReceiveInt(&hspi, (void*)cmd.spiword, SPI_WORDS);
    if (spiStat != hal_status_ok) {
        return false;
    }

    while(hspi.state == hal_spi_state_busy); // wait for complete

    /* Assemble 16-bit data (MSB first) */
    *out = (uint16_t)((cmd.spiword[1] << 8) | cmd.spiword[2]);

    return true;
}

bool AFE11612_WriteReg(uint8_t reg, uint16_t value) {

    volatile afe11612_spi_cmd_t cmd;

    /* Set read bit (bit7 = 1) and place 7-bit address in bits[6:0]
     * The device expects a 24-bit transaction: [R/W + Addr(7)] [MSB data] [LSB data]
     * First transfer: send the read command and receive don't-care
     * Second transfer: send 3 dummy bytes and receive the 3 response bytes (address + data)
     */
    cmd.spiword[0] = reg & 0x7F;
    cmd.spiword[1] = (value >> 8) & 0xFF;
    cmd.spiword[2] = value & 0xFF;

    /* Send write command (3 bytes) */
    //spiStat = HAL_Spi_Transmit(&hspi, (void*)cmd.spiword, SPI_WORDS, 0, true);
    spiStat = HAL_Spi_TransmitInt(&hspi, (void*)cmd.spiword, SPI_WORDS);
    if (spiStat != hal_status_ok) {
        return false;
    }
    /* need for CS transition*/
    //for(volatile int i=0; i<100; i++) { asm("nop"); }
    while(hspi.state == hal_spi_state_busy); // wait for complete
    
     return true;
}
