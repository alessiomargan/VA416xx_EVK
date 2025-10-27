/**
 * @file afe1112.c
 * @brief AFE11612-SEP Radiation-Tolerant, Analog Monitor and Controller
 * with Multichannel ADC, DACs, and Temperature Sensors
 *
 * This file implements the interface to the AFE11612
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
static volatile bool transferDone = true;
static volatile transferState_t transferState;

/* Configuration registers - written during initialization (ordered by register address) */
afe11612_reg_t afe11612_config[] = {
    { AFE11612_REG_TEMP__CONFIG,        0x003C },  /* 0x0A - Temperature sensor config */
    { AFE11612_REG_TEMP__CONV__RATE,    0x0007 },  /* 0x0B - Temperature conversion rate */
    { AFE11612_REG_D1_N__ADJUST,        0x0000 },  /* 0x21 - D1_N adjustment */
    { AFE11612_REG_D2_N__ADJUST,        0x0000 },  /* 0x22 - D2_N adjustment */
    { AFE11612_REG_DAC_0__CLR,          0x0000 },  /* 0x3F - DAC 0 clear value */
    { AFE11612_REG_DAC_1__CLR,          0x0000 },  /* 0x40 - DAC 1 clear value */
    { AFE11612_REG_DAC_2__CLR,          0x0000 },  /* 0x41 - DAC 2 clear value */
    { AFE11612_REG_DAC_3__CLR,          0x0000 },  /* 0x42 - DAC 3 clear value */
    { AFE11612_REG_DAC_4__CLR,          0x0000 },  /* 0x43 - DAC 4 clear value */
    { AFE11612_REG_DAC_5__CLR,          0x0000 },  /* 0x44 - DAC 5 clear value */
    { AFE11612_REG_DAC_6__CLR,          0x0000 },  /* 0x45 - DAC 6 clear value */
    { AFE11612_REG_DAC_7__CLR,          0x0000 },  /* 0x46 - DAC 7 clear value */
    { AFE11612_REG_DAC_8__CLR,          0x0000 },  /* 0x47 - DAC 8 clear value */
    { AFE11612_REG_DAC_9__CLR,          0x0000 },  /* 0x48 - DAC 9 clear value */
    { AFE11612_REG_DAC_10__CLR,         0x0000 },  /* 0x49 - DAC 10 clear value */
    { AFE11612_REG_DAC_11__CLR,         0x0000 },  /* 0x4A - DAC 11 clear value */
    { AFE11612_REG_AFE__CONFIG_0,       0x2000 },  /* 0x4C - AFE Configuration Register 0 */
    { AFE11612_REG_AFE__CONFIG_1,       0x0070 },  /* 0x4D - AFE Configuration Register 1 */
    { AFE11612_REG_ALR_CTRL,            0x0000 },  /* 0x4E - Alarm Control Register */
    { AFE11612_REG_ADC_CH0,             0x0000 },  /* 0x50 - ADC Channel 0 configuration */
    { AFE11612_REG_ADC_CH1,             0x0000 },  /* 0x51 - ADC Channel 1 configuration */
    { AFE11612_REG_ADC_GAIN,            0xFFFF },  /* 0x52 - ADC Gain */
    { AFE11612_REG_AUTO_DAC__CLR__SOURCE, 0x0004 }, /* 0x53 - Auto DAC Clear Source */
    { AFE11612_REG_AUTO_DAC__CLR_EN,    0x0000 },  /* 0x54 - Auto DAC Clear Enable */
    { AFE11612_REG_SW_DAC__CLR,         0x0000 },  /* 0x55 - Software DAC Clear */
    { AFE11612_REG_HW_DAC__CLR_EN_0,    0x0000 },  /* 0x56 - Hardware DAC Clear Enable 0 */
    { AFE11612_REG_HW_DAC__CLR_EN_1,    0x0000 },  /* 0x57 - Hardware DAC Clear Enable 1 */
    { AFE11612_REG_DAC_CONFIG,          0x0000 },  /* 0x58 - DAC Configuration */
    { AFE11612_REG_DAC_GAIN,            0x0000 },  /* 0x59 - DAC Gain */
    { AFE11612_REG_IN_0__HIGH__THRESHOLD, 0x0FFF }, /* 0x5A - IN_0 High Threshold */
    { AFE11612_REG_IN_0__LOW__THRESHOLD,  0x0000 }, /* 0x5B - IN_0 Low Threshold */
    { AFE11612_REG_IN_1__HIGH__THRESHOLD, 0x0FFF }, /* 0x5C - IN_1 High Threshold */
    { AFE11612_REG_IN_1__LOW__THRESHOLD,  0x0000 }, /* 0x5D - IN_1 Low Threshold */
    { AFE11612_REG_IN_2__HIGH__THRESHOLD, 0x0FFF }, /* 0x5E - IN_2 High Threshold */
    { AFE11612_REG_IN_2__LOW__THRESHOLD,  0x0000 }, /* 0x5F - IN_2 Low Threshold */
    { AFE11612_REG_IN_3__HIGH__THRESHOLD, 0x0FFF }, /* 0x60 - IN_3 High Threshold */
    { AFE11612_REG_IN_3__LOW__THRESHOLD,  0x0000 }, /* 0x61 - IN_3 Low Threshold */
    { AFE11612_REG_LT__HIGH__THRESHOLD,   0x07FF }, /* 0x62 - Local Temp High Threshold */
    { AFE11612_REG_LT__LOW__THRESHOLD,    0x0800 }, /* 0x63 - Local Temp Low Threshold */
    { AFE11612_REG_D1__HIGH__THRESHOLD,   0x07FF }, /* 0x64 - D1 Temp High Threshold */
    { AFE11612_REG_D1__LOW__THRESHOLD,    0x0800 }, /* 0x65 - D1 Temp Low Threshold */
    { AFE11612_REG_D2__HIGH__THRESHOLD,   0x0000 }, /* 0x66 - D2 Temp High Threshold */
    { AFE11612_REG_D2__LOW__THRESHOLD,    0x0000 }, /* 0x67 - D2 Temp Low Threshold */
    { AFE11612_REG_HYST_0,                0x0810 }, /* 0x68 - Hysteresis 0 */
    { AFE11612_REG_HYST_1,                0x0810 }, /* 0x69 - Hysteresis 1 */
    { AFE11612_REG_HYST_2,                0x2108 }, /* 0x6A - Hysteresis 2 */
    { AFE11612_REG_PWR_DOWN,              0x0000 }, /* 0x6B - Power Down */
    { AFE11612_REG_SW_RST,                0x0000 }  /* 0x7C - Software Reset */
};
/**
 * Get the number of configuration registers.
 */
#define AFE11612_CONFIG_COUNT (sizeof(afe11612_config) / sizeof(afe11612_reg_t))

/* Runtime value registers - temp sensors, ADC inputs, DAC outputs (ordered by register address) */
afe11612_reg_t afe11612_values[] = {
    { AFE11612_REG_LT__TEMP,            0x0000 },  /* 0x00 - Local temperature */
    { AFE11612_REG_D1__TEMP,            0x0000 },  /* 0x01 - Remote temperature D1 */
    { AFE11612_REG_D2__TEMP,            0x0000 },  /* 0x02 - Remote temperature D2 */
    { AFE11612_REG_ADC_0,               0x0000 },  /* 0x23 - ADC input 0 */
    { AFE11612_REG_ADC_1,               0x0000 },  /* 0x24 - ADC input 1 */
    { AFE11612_REG_ADC_2,               0x0000 },  /* 0x25 - ADC input 2 */
    { AFE11612_REG_ADC_3,               0x0000 },  /* 0x26 - ADC input 3 */
    { AFE11612_REG_ADC_4,               0x0000 },  /* 0x27 - ADC input 4 */
    { AFE11612_REG_ADC_5,               0x0000 },  /* 0x28 - ADC input 5 */
    { AFE11612_REG_ADC_6,               0x0000 },  /* 0x29 - ADC input 6 */
    { AFE11612_REG_ADC_7,               0x0000 },  /* 0x2A - ADC input 7 */
    { AFE11612_REG_DAC_0,               0x0000 },  /* 0x33 - DAC 0 output */
    { AFE11612_REG_DAC_1,               0x0000 },  /* 0x34 - DAC 1 output */
    { AFE11612_REG_DAC_2,               0x0000 },  /* 0x35 - DAC 2 output */
    { AFE11612_REG_DAC_3,               0x0000 },  /* 0x36 - DAC 3 output */
    { AFE11612_REG_DAC_4,               0x0000 },  /* 0x37 - DAC 4 output */
    { AFE11612_REG_DAC_5,               0x0000 },  /* 0x38 - DAC 5 output */
    { AFE11612_REG_DAC_6,               0x0000 },  /* 0x39 - DAC 6 output */
    { AFE11612_REG_DAC_7,               0x0000 },  /* 0x3A - DAC 7 output */
    { AFE11612_REG_DAC_8,               0x0000 },  /* 0x3B - DAC 8 output */
    { AFE11612_REG_DAC_9,               0x0000 },  /* 0x3C - DAC 9 output */
    { AFE11612_REG_DAC_10,              0x0000 },  /* 0x3D - DAC 10 output */
    { AFE11612_REG_DAC_11,              0x0000 },  /* 0x3E - DAC 11 output */
    { AFE11612_REG_STATUS,              0x0000 },  /* 0x4F - Status register (alarms/faults) */
    { AFE11612_REG_DEVICE_ID,           0x0000 }   /* 0x6C - Device ID */
};
/**
 * Get the number of runtime value registers.
 */
#define AFE11612_VALUES_COUNT (sizeof(afe11612_values) / sizeof(afe11612_reg_t))

/**
 * Convert AFE11612 temperature register value to centidegrees Celsius (hundredths of a degree).
 * Temperature is stored as 12-bit 2's complement in bits [15:4], bits [3:0] are reserved.
 * Resolution is 0.125°C/LSB (12.5 centidegrees/LSB).
 * Result is scaled by 100 to preserve full 0.125°C precision.
 * @param raw_value Raw 16-bit value from temperature register
 * @return Temperature in centidegrees Celsius (e.g., 2675 = 26.75°C, -4500 = -45.00°C)
 */
int16_t AFE11612_ConvertTemp(uint16_t raw_value) {
    // Shift right by 4 to get 12-bit value from bits [15:4]
    int16_t value_12bit = (raw_value >> 4) & 0x0FFF;
    // Convert to signed 12-bit (2's complement)
    if (value_12bit & 0x0800) {  // Check if bit 11 is set (negative)
        value_12bit |= 0xF000;   // Sign-extend to 16-bit
    }
    // 0.125°C/LSB = 12.5 centidegrees/LSB
    // To get centidegrees: (value * 0.125 * 100) = (value * 12.5) = (value * 25) / 2
    return (value_12bit * 25) / 2;
}


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

    for(volatile int i=0; i<1000; i++) { asm("nop"); }
    if ( ! AFE11612_testDeviceId() ) {
        // TODO: handle error
        return;
    }
  
    AFE11612_WriteConfig();
}

/* declared weak in va416xx_hal_spi.c
 * Used for non-blocking SPI transfers (interrupt and DMA)
 * called by Spi_Callback - HAL_Spi_Rx_Dma_Callback - DMA RX channel DONE interrupt
 * called by Spi_Callback - Spi_StateMachine - SPIn_RX_IRQHandler
 */
 void HAL_Spi2_Cmplt_Callback(hal_spi_handle_t* const hspi)
{
    if(hspi->state == hal_spi_state_error) {
        transferState = TXFER_ERROR;
        spiStat = hal_status_rxError; // receive overrun
        goto exit_cb;
    }
    
    switch (transferState)
    {
    case TXFER_RD_PROGRESS_1:
        transferState = TXFER_RD_PROGRESS_2;
        break;
    
    case TXFER_RD_PROGRESS_2:
        transferState = TXFER_RD_COMPLETE;
        break;

    case TXFER_WR_PROGRESS_1:
        transferState = TXFER_WR_COMPLETE;
        break;

    default:
        transferState = TXFER_IDLE;
        break;
    }

exit_cb :
    transferDone = true;
    Pin_set(DBG_PORT, DBG2_PIN, !transferDone);
        
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

    transferState = TXFER_RD_PROGRESS_1;
    /* Send read command (3 bytes) */
    //spiStat = HAL_Spi_Transmit(&hspi, (void*)cmd.spiword, SPI_WORDS, 0, false);
    spiStat = HAL_Spi_TransmitInt(&hspi, (void*)cmd.spiword, SPI_WORDS);
    if (spiStat != hal_status_ok) {
        transferState = TXFER_ERROR;
        return false;
    }
    
    while(transferState == TXFER_RD_PROGRESS_1) { asm("nop"); };
    /* need for CS transition in case of blocking SPI transfers*/
    //for(volatile int i=0; i<10; i++) { asm("nop"); }
    
    /* Receive response (3 bytes) */
    //spiStat = HAL_Spi_Receive(&hspi, (void*)cmd.spiword, SPI_WORDS, 0);
    spiStat = HAL_Spi_ReceiveInt(&hspi, (void*)cmd.spiword, SPI_WORDS);
    if (spiStat != hal_status_ok) {
        return false;
    }

    while(transferState == TXFER_RD_PROGRESS_2) { asm("nop"); };

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

    transferState = TXFER_WR_PROGRESS_1;
    /* Send write command (3 bytes) */
    //spiStat = HAL_Spi_Transmit(&hspi, (void*)cmd.spiword, SPI_WORDS, 0, true);
    spiStat = HAL_Spi_TransmitInt(&hspi, (void*)cmd.spiword, SPI_WORDS);
    if (spiStat != hal_status_ok) {
        return false;
    }

    while(transferState == TXFER_WR_PROGRESS_1) { asm("nop"); };
    /* need for CS transition in case of blocking SPI transfers*/
    //for(volatile int i=0; i<100; i++) { asm("nop"); }

    return true;
}

bool AFE11612_testDeviceId(void) {
    uint16_t val;
    bool ret;

    //__disable_irq();
    ret = AFE11612_ReadReg(AFE11612_REG_DEVICE_ID, &val);
    //__enable_irq();

    if ( ! ret) {
        printf("AFE11612 read failed\n");
        return false;
    }
  
    if ( val != 0x1220 ) {
        printf("Error: AFE11612 Device ID mismatch: 0x%04X (expected 0x1220)\n", val);
        return false;
    } else {
        printf("AFE11612 Device ID match: 0x%04X\n", val);
        return true;    
    }

}

uint8_t AFE11612_WriteConfig(void) {
    uint8_t success_count = 0;
    
    for (uint8_t i = 0; i < AFE11612_CONFIG_COUNT; i++) {
        if (AFE11612_WriteReg(afe11612_config[i].reg, afe11612_config[i].value)) {
            success_count++;
        } else {
            printf("Error: AFE11612_WriteConfig failed at register 0x%02X\n", 
                   afe11612_config[i].reg);
        }
    }
    
    printf("AFE11612_WriteConfig: %d/%d registers written\n", 
           success_count, AFE11612_CONFIG_COUNT);
    return success_count;
}

uint8_t AFE11612_ReadValues(void) {
    uint8_t success_count = 0;
    
    for (uint8_t i = 0; i < AFE11612_VALUES_COUNT; i++) {
        if (AFE11612_ReadReg(afe11612_values[i].reg, &afe11612_values[i].value)) {
            success_count++;
        } else {
            printf("Error: AFE11612_ReadValues failed at register 0x%02X\n", 
                   afe11612_values[i].reg);
        }
    }
    
    return success_count;
}

uint8_t AFE11612_ReadValuesByIndex(const int8_t *indices) {
    uint8_t success_count = 0;
    
    if (indices == NULL) {
        return 0;
    }
    
    // Process indices until we hit a negative marker
    for (uint8_t i = 0; indices[i] >= 0; i++) {
        int8_t idx = indices[i];
        
        // Bounds check
        if (idx >= AFE11612_VALUES_COUNT) {
            printf("Error: AFE11612_ReadValuesByIndex invalid index %d (max %d)\n", 
                   idx, AFE11612_VALUES_COUNT - 1);
            continue;
        }
        
        // Read the register at this index
        if (AFE11612_ReadReg(afe11612_values[idx].reg, &afe11612_values[idx].value)) {
            success_count++;
        } else {
            printf("Error: AFE11612_ReadValuesByIndex failed at index %d (register 0x%02X)\n", 
                   idx, afe11612_values[idx].reg);
        }
    }
    
    return success_count;
}

void AFE11612_ProcessRequest(const can_pkt_t *rxPkt, can_pkt_t *respPkt) {

    respPkt->id = 0x5FE;  /* AFE response ID */
    respPkt->dataLengthBytes = 6; //rxPkt->dataLengthBytes;
    respPkt->txPriorityCode = 0;   /* Highest priority */
    respPkt->msgType = en_can_cmb_msgtype_STD11;
#if 0
    respPkt->data16[0] = afe11612_values[AFE11612_REG_LT__TEMP].value;
    respPkt->data16[1] = afe11612_values[AFE11612_REG_D1__TEMP].value;
    respPkt->data16[2] = afe11612_values[AFE11612_REG_D2__TEMP].value;
#else
    respPkt->data16[0] = AFE11612_ConvertTemp(afe11612_values[AFE11612_REG_LT__TEMP].value);
    respPkt->data16[1] = AFE11612_ConvertTemp(afe11612_values[AFE11612_REG_D1__TEMP].value);
    respPkt->data16[2] = AFE11612_ConvertTemp(afe11612_values[AFE11612_REG_D2__TEMP].value);
#endif
}