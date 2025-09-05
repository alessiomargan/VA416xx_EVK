#include "board.h"
#include "ads1278.h"

#include "va416xx_hal.h"
#include "va416xx_hal_spi.h"
#include "va416xx_hal_dma.h"

#define DRDY_PIN    0
#define DRDY_PORT   PORTF
#define CH_NUM      4
#define USE_DMA
//#define TEST_DMA

static hal_spi_handle_t hspi;
static volatile hal_status_t spiStat;
static volatile bool rxDmaDone = true;
static volatile uint32_t missedSamples;
static volatile uint32_t sampleCount;
        
static ads1278_spi_data_t spi_rx_data;  // Raw SPI data
static ads1278_data_t processed_data;   // Processed channel data

void ConfigureADS1278(void) {
    
    HAL_DMA_Init(NULL, false, false, false);
    
    // config SPI0
    HAL_UNLOCK(&hspi);
    hspi.spi = VOR_SPI1;
    hspi.init.blockmode = true;
    hspi.init.bmstall = false;
    hspi.init.clkDiv = 6;   // could be less but wires probes make clk suck
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
    // Enable clock to GPIO bank
    // Configure as input
    DRDY_PORT->DIR &= ~(1 << DRDY_PIN);
    // Enable pull-up (if needed - ADS1278 DRDY is typically open-drain)
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

#ifdef USE_DMA
    if ( rxDmaDone ) {
        rxDmaDone = false;
        // !!!! DO IT EVERY TIME .... 
        spiStat = HAL_Spi_ConfigDMA(&hspi, 0, 1);
        spiStat = HAL_Spi_ReceiveDMA(&hspi, spi_rx_data.raw, 3*CH_NUM);
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
    spiStat = HAL_Spi_Receive(&hspi, spi_rx_data.raw, 3*CH_NUM, 100);
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

// declared weak 
void HAL_Spi_Cmplt_Callback(hal_spi_handle_t* hdl)
{
    if(hdl != &hspi) { return; }
    
    if(hdl->state != hal_spi_state_error) {
        spiStat = hal_status_ok;
        // Process the raw SPI data into channel values
        ads1278_process_data(&spi_rx_data, &processed_data);
        // Consider adding a timestamp or sequence number to track data flow
        if ( ++sampleCount % 10000 == 0) {
            printf("%lu #samples, latest : %ld %ld %ld %ld\n", 
                   sampleCount,
                   processed_data.ch[0], processed_data.ch[1],
                   processed_data.ch[2], processed_data.ch[3]);
        }
    } else {
        spiStat = hal_status_rxError; // receive overrun
    }
 
    rxDmaDone = true;

}

// New function to allow external access to the data
void ADS1278_ReadAllChannels(ads1278_data_t* data) {
    // Copy the processed data to the provided structure
    for (int i = 0; i < 8; i++) {
        data->ch[i] = processed_data.ch[i];
    }
}