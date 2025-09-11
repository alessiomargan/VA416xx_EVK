#ifndef __ADS1278_H__
#define __ADS1278_H__

#include <stdint.h>

#define SPI_WORDLEN_X_CH    3
#define MAX_SMPL_POW2		8 	// 2^8 = 256
#define MAX_RAW_SMPL		(1 << MAX_SMPL_POW2)  	// 1<<8 = 256
#define ADC_CH_NUM          8
#define V_TICK              0.0000003f
#define mV_TICK             0.0003f
#define uV_TICK             0.3f


// SPI receive buffer: 12 uint16_t values (1.5 words per channel × 8 channels)
typedef struct {
    uint8_t spiword[SPI_WORDLEN_X_CH*ADC_CH_NUM];    // Raw SPI data (24 words total)
} ads1278_spi_data_t;

// Final data structure with both raw and voltage values
typedef struct {
    int32_t ch[ADC_CH_NUM];     // Raw 24-bit values (sign-extended)
    uint32_t timestamp;         // Optional: add timestamp for data tracking
} ads1278_raw_data_t;

typedef struct {
    int32_t     microVolts[ADC_CH_NUM];     // Voltage values
    uint32_t    timestamp;          // Optional: add timestamp for data tracking
} ads1278_adc_data_t;

// Function to convert raw SPI data to channel values
static inline void ads1278_process_data(const ads1278_spi_data_t* raw_data, ads1278_raw_data_t* proc_data) {
    for (int i = 0; i < ADC_CH_NUM; i++) {
        // Combine 3 bytes into a 24-bit value
        int32_t value = ((int32_t)raw_data->spiword[i*3] << 16) |     // MSB (bits 23-16)
                        ((int32_t)raw_data->spiword[i*3 + 1] << 8) |  // Middle byte (bits 15-8)
                        (raw_data->spiword[i*3 + 2]);                 // LSB (bits 7-0)
        // Sign extension for negative values (if bit 23 is set)
        if (value & 0x800000) { 
            value |= 0xFF000000;
        }
        proc_data->ch[i] = value;
    }
}

void ConfigureADS1278(void);
void ADS1278_getADCs(ads1278_adc_data_t* data);

#endif