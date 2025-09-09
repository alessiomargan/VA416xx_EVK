#ifndef __ADS1278_H__
#define __ADS1278_H__

#include <stdint.h>

#define SPI_WORDLEN_X_CH  3
#define MAX_SMPL_POW2		9 	// 2^9 = 512
#define MAX_SMPL_NUM		(1 << MAX_SMPL_POW2)  	// 1<<(9-1) = 512
#define ADC_RAW_BUFF_SIZE   MAX_SMPL_NUM
#define ADC_CH_NUM          4


// SPI receive buffer: 12 uint16_t values (1.5 words per channel × 8 channels)
typedef struct {
    uint8_t raw[SPI_WORDLEN_X_CH*ADC_CH_NUM];    // Raw SPI data (24 words total)
} ads1278_spi_data_t;

// Final data structure with 24-bit values
typedef struct {
    int32_t ch[ADC_CH_NUM];     // Processed 24-bit values (sign-extended to 32-bit)
} ads1278_data_t;

// Function to convert raw SPI data to channel values
static inline void ads1278_process_data(const ads1278_spi_data_t* raw_data, ads1278_data_t* proc_data) {
    for (int i = 0; i < ADC_CH_NUM; i++) {
        // Each channel uses 3 consecutive bytes
        uint8_t byte0 = raw_data->raw[i*3];     // MSB (bits 23-16)
        uint8_t byte1 = raw_data->raw[i*3 + 1]; // Middle byte (bits 15-8)
        uint8_t byte2 = raw_data->raw[i*3 + 2]; // LSB (bits 7-0)
        
        // Combine into 24-bit value
        int32_t value = ((int32_t)byte0 << 16) | 
                        ((int32_t)byte1 << 8) | 
                        byte2;
        
        // Sign extension
        if (value & 0x800000) {
            value |= 0xFF000000;
        }
        
        proc_data->ch[i] = value;
    }
}

#if 0
// Function to convert raw SPI data to channel values
static inline void ads1278_process_data(const ads1278_spi_data_t* raw_data, ads1278_data_t* proc_data) {
    for (int i = 0; i < 8; i++) {
        // Determine which words contain this channel's data
        uint16_t msw = raw_data->raw[i];                // First word (16 bits)
        uint16_t lsw_idx = 8 + (i / 2);                 // Index for word with LSBs
        uint16_t lsw = raw_data->raw[lsw_idx];          // Word containing LSBs
        uint8_t  shift = (i % 2) ? 0 : 8;               // Shift amount (0 or 8)
        
        // Extract the 8 LSBs based on even/odd channel
        uint8_t lsb = (lsw >> shift) & 0xFF;
        
        // Combine into 24-bit value
        int32_t value = ((int32_t)msw << 8) | lsb;
        
        // Sign extension
        if (value & 0x800000) {
            value |= 0xFF000000;
        }
        
        proc_data->ch[i] = value;
    }
}
#endif

void ConfigureADS1278(void);
void ADS1278_ReadAllChannels(ads1278_data_t* data);

#endif