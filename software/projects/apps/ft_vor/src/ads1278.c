#include "board.h"
#include "ads1278.h"

#include "va416xx_hal.h"
#include "va416xx_hal_spi.h"
#include "va416xx_hal_dma.h"

#include <inttypes.h>

#define DRDY_PIN    0
#define DRDY_PORT   PORTF
#define DBG_PIN     10
#define DBG_PORT    PORTF

#define USE_DMA
//#define TEST_DMA

// Define dummy system calls to suppress warnings
void _close(void)  {}
void _lseek(void)  {}
void _read(void)   {}
void _fstat(void)  {}
void _isatty(void) {}

static hal_spi_handle_t hspi;
static volatile hal_status_t spiStat;
static volatile bool rxDmaDone = true;
static volatile uint32_t missedSamples;
static volatile uint32_t sampleCount;
        
static ads1278_spi_data_t   spi_rx_data;                            // raw SPI data
static ads1278_raw_data_t   adc_raw_data[ADC_RAW_BUFF_SIZE] = {0};  // adc raw counts
//static ads1278_data_t       adc_data[ADC_RAW_BUFF_SIZE] = {0};      // adc values
static ads1278_raw_data_t   adc_raw_test;   // adc raw counts
static uint16_t adc_raw_idx = 0;                // adc raw last idx 
static int32_t  adc_raw_sum[ADC_CH_NUM] = {0};
static const uint8_t adc_ch_num = ADC_CH_NUM;

static inline void Pin_off(VOR_GPIO_Type * const GPIO_PORT, uint8_t pin) { GPIO_PORT->CLROUT = 1UL<<pin; }
static inline void Pin_on (VOR_GPIO_Type * const GPIO_PORT, uint8_t pin) { GPIO_PORT->SETOUT = 1UL<<pin; }
static inline void Pin_tgl(VOR_GPIO_Type * const GPIO_PORT, uint8_t pin) { GPIO_PORT->TOGOUT = 1UL<<pin; }
static inline void Pin_set(VOR_GPIO_Type * const GPIO_PORT, uint8_t pin, bool val) {
    val ? (GPIO_PORT->SETOUT = 1UL<<pin) : (GPIO_PORT->CLROUT = 1UL<<pin);
}

void ConfigureADS1278(void) {
    
    HAL_DMA_Init(NULL, false, false, false);
    
    // config SPI
    HAL_UNLOCK(&hspi);
    hspi.spi = VOR_SPI1;
    hspi.init.blockmode = true;
    hspi.init.bmstall = false;
    hspi.init.clkDiv = 10;   // could be less but wires probes make clk suck
    hspi.init.loopback = false;
    hspi.init.mdlycap = false;
    // for ads1278 don't care ?!?
    hspi.init.mode = hal_spi_clkmode_0;
    hspi.init.ms = hal_spi_ms_master;
    hspi.init.chipSelect = 0;
    hspi.init.wordLen = 8;  //16;
    spiStat = HAL_Spi_Init(&hspi);
    if(spiStat != hal_status_ok) {
        printf("Error: HAL_Spi_Init() status: %s\n", HAL_StatusToString(spiStat));
        return;
    }
    // Initialize ~DRDY pin as an input interrupt to trigger the read.
    // Configure as input
    DRDY_PORT->DIR &= ~(1 << DRDY_PIN);
    // Configure falling edge interrupt
    DRDY_PORT->IRQ_SEN  &= ~(1 << DRDY_PIN);  // Edge sensitive
    DRDY_PORT->IRQ_EDGE &= ~(1 << DRDY_PIN);  // Falling edge
    DRDY_PORT->IRQ_EVT  &= ~(1 << DRDY_PIN);
    DRDY_PORT->IRQ_ENB  |= (1 << DRDY_PIN);
    // Enable NVIC interrupt for GPIO Bank B
    NVIC_EnableIRQ(PORTF0_IRQn);
    NVIC_SetPriority(PORTF0_IRQn, 3);  // Set appropriate priority

}


void PF0_IRQHandler(void) {

    //Pin_tgl(DBG_PORT, DBG_PIN);
        
#ifdef USE_DMA
    if ( rxDmaDone ) {
        rxDmaDone = false;
        Pin_set(DBG_PORT, DBG_PIN, !rxDmaDone);
        // !!!! DO IT EVERY TIME ....
        spiStat = HAL_Spi_ConfigDMA(&hspi, 0, 1);
        spiStat = HAL_Spi_ReceiveDMA(&hspi, spi_rx_data.raw, SPI_WORDLEN_X_CH*ADC_CH_NUM);
        if(spiStat != hal_status_ok) {
            printf("Error: HAL_Spi_ReceiveDMA() status: %s\n", HAL_StatusToString(spiStat));
        }
    } else {
        // This means we missed a sample because DMA wasn't ready
        if ( ++missedSamples % 1000 == 0) {
            printf("Warning: Missed %lu samples due to DMA not ready\n", missedSamples);
        }
    }
#else
    // Receive 12 uint16_t words
    spiStat = HAL_Spi_Receive(&hspi, spi_rx_data.raw, 3*ADC_CH_NUM, 100);
    if(spiStat != hal_status_ok) {
        printf("Error: HAL_Spi_Receive() status: %s\n", HAL_StatusToString(spiStat));
        return;
    }
    // Process the raw SPI data into channel values
    ads1278_process_data(&spi_rx_data, &processed_data);    
    // Now you can use processed_data.ch[0] through processed_data.ch[7]
    // Example: print channel 1 value
    //printf("CH1: %ld\n", processed_data.ch[0]);
#endif
}

// declared weak in va416xx_hal_spi.c
// Used for DMA transfers, called by DMA RX channel DONE interrupt
void HAL_Spi_Cmplt_Callback(hal_spi_handle_t* hdl)
{
    if(hdl != &hspi) { return; }
    
    if(hdl->state == hal_spi_state_error) {

        spiStat = hal_status_rxError; // receive overrun
        goto exit_cb;
    }
    
    spiStat = hal_status_ok;
#if 1
    // START Protect data sum calculation from interruptions
    __disable_irq();
    //
    #pragma GCC unroll adc_ch_num
    for (int i = 0; i < adc_ch_num; i++) {
        adc_raw_sum[i] -= adc_raw_data[adc_raw_idx].ch[i];
    }
    // Process the raw SPI data into channel values
    ads1278_process_data(&spi_rx_data, &adc_raw_data[adc_raw_idx]);
    //ads1278_process_data(&spi_rx_data, &adc_raw_test);
    sampleCount++;
    //
    #pragma GCC unroll adc_ch_num
    for (int i = 0; i < adc_ch_num; i++) {
        adc_raw_sum[i] += adc_raw_data[adc_raw_idx].ch[i];
    }
    //
    adc_raw_idx = (adc_raw_idx+1) % ADC_RAW_BUFF_SIZE;
    // END Protect data sum calculation from interruptions
    __enable_irq();
#else
    ads1278_process_data(&spi_rx_data, &adc_raw_test);
#endif

exit_cb :    
    rxDmaDone = true;
    Pin_set(DBG_PORT, DBG_PIN, !rxDmaDone);
        
}

// New function to allow external access to the data
void ADS1278_ReadAllChannels(ads1278_data_t* data) {
    
#if 1
    __disable_irq();
    // Copy the processed data to the provided structure
    #pragma GCC unroll adc_ch_num
    for (int i = 0; i < adc_ch_num; i++) {
        // divide by SMPL_NUM using shift x / 2^n == x >> n
	    // SMPL_NUM 64 = 2^6 ==> x >> 6
	    data->ch[i] = (adc_raw_sum[i] >> MAX_SMPL_POW2) * V_TICK;  // Equivalent to division by 512
    }
    //printf("latest means: %u %ld %ld\n", 
    //        sampleCount, adc_raw_data[adc_raw_idx].ch[0], data->ch[0]);
    printf("%u samples - means %d mV\n", 
            sampleCount, (uint32_t)(data->ch[0]*1000));
    sampleCount = 0;
    __enable_irq();
#else
    for (int i = 0; i < adc_ch_num; i++) {
        data->ch[i] = adc_raw_test.ch[i];  
    }
    printf("latest : %ld\n", data->ch[0]);
#endif
    
   

}