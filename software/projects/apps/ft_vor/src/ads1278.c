/**
 * @file ads1278.c
 * @brief ADS1278 ADC Driver Implementation with DMA and Ring Buffer
 *
 * This file implements the interface to the ADS1278 8-channel, 24-bit
 * delta-sigma ADC with SPI communication and DMA-based data acquisition.
 * The driver provides efficient ring buffer management with continuous 
 * averaging of the sampled data for noise reduction.
 *
 * @author Alessio Margan
 * @date September 24, 2025
 *
 * @version 1.0
 *
 * @copyright Copyright (c) 2025 IIT
 *
 * @details
 * The driver implements:
 * - SPI communication with the ADS1278 ADC
 * - DMA-based data acquisition triggered by DRDY falling edge
 * - 256-sample ring buffer with running sum for efficient averaging
 * - Automatic CAN message buffer updates for RTR responses
 * - Voltage conversion with configurable scaling factors
 * - Debug pin for oscilloscope monitoring of DMA transfers
 *
 * @note This implementation requires the VA416xx HAL SPI, DMA, and CAN drivers
 */

#include "board.h"
#include "ads1278.h"
#include "hal_utils.h"

#include "va416xx_hal.h"
#include "va416xx_hal_spi.h"
#include "va416xx_hal_dma.h"
#include "va416xx_hal_canbus.h"

#include <inttypes.h>

//#define USE_DMA
#define USE_IRQ

#define DRDY_PIN    0
#define DRDY_PORT   PORTF


calib_t calMat[10];
float   tempVec[12][12];

static hal_spi_handle_t hspi;
static volatile hal_status_t spiStat;
static volatile bool transferDone = true;
static volatile uint32_t missedSamples;
static volatile uint32_t sampleCount;
        
static ads1278_spi_data_t   spi_rx_data = {0};                        // raw SPI data
static ads1278_raw_data_t   adc_raw_data[MAX_RAW_SMPL] = {0};   // adc raw counts
//static ads1278_adc_data_t   adc_raw_test;                       // adc raw counts
static uint16_t adc_raw_idx = 0;                                // adc raw last idx 
static int32_t  adc_raw_sum[ADC_CH_NUM] = {0};
static const uint8_t adc_ch_num = ADC_CH_NUM;


extern can_cmb_t * cmb_RTR_resp[]; 
//
static inline void set_cmb_data(void) {

    static const uint8_t rtr_resp_num = 4;
    //assert((rtr_resp_num*2)==ADC_CH_NUM);
    can_cmb_t * can_cmb;
    #pragma GCC unroll rtr_resp_num
    for (int i = 0; i < rtr_resp_num; i++) {
        can_cmb = cmb_RTR_resp[i];
        can_cmb->DATA0 = (int32_t)((adc_raw_sum[i*2] >> MAX_SMPL_POW2) * uV_TICK) >> 16;    // MSB Upper 16 bits
        can_cmb->DATA1 = (int32_t)((adc_raw_sum[i*2] >> MAX_SMPL_POW2) * uV_TICK);
        can_cmb->DATA2 = (int32_t)((adc_raw_sum[(i*2)+1] >> MAX_SMPL_POW2) * uV_TICK) >> 16;    // MSB Upper 16 bits
        can_cmb->DATA3 = (int32_t)((adc_raw_sum[(i*2)+1] >> MAX_SMPL_POW2) * uV_TICK);
    }
}

static inline void process_spi_data(void) {
    
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
    set_cmb_data();
    // next adc_raw_data buffer idx
    adc_raw_idx = (adc_raw_idx+1) % MAX_RAW_SMPL;
    // END Protect data sum calculation from interruptions
    __enable_irq();

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
    hspi.init.wordLen = SPI_WORDLEN;
    spiStat = HAL_Spi_Init(&hspi);
    if(spiStat != hal_status_ok) {
        printf("Error: HAL_Spi_Init() status: %s\n", HAL_StatusToString(spiStat));
        return;
    }
#ifdef USE_DMA
    spiStat = HAL_Spi_ConfigDMA(&hspi, 0, 1);
    if(spiStat != hal_status_ok) {
        printf("Error: HAL_Spi_ConfigDMA() status: %s\n", HAL_StatusToString(spiStat));
        return;
    }
#endif
        
    // Initialize ~DRDY pin as an input interrupt to trigger the read.
    // Configure as input
    DRDY_PORT->DIR &= ~(1 << DRDY_PIN);
    // Configure falling edge interrupt
    DRDY_PORT->IRQ_SEN  &= ~(1 << DRDY_PIN);  // Edge sensitive
    DRDY_PORT->IRQ_EDGE &= ~(1 << DRDY_PIN);  // Falling edge
    DRDY_PORT->IRQ_EVT  &= ~(1 << DRDY_PIN);
    DRDY_PORT->IRQ_ENB  |= (1 << DRDY_PIN);
    // Enable NVIC interrupt for GPIO Bank F
    NVIC_EnableIRQ(PORTF0_IRQn);
    NVIC_SetPriority(PORTF0_IRQn, 3);  // Set appropriate priority

}

/**
 * @brief GPIO Port F Pin 0 Interrupt Handler
 * 
 */
void PF0_IRQHandler(void) {

#if defined(USE_DMA) || defined(USE_IRQ)
    if ( transferDone ) {
        transferDone = false;
        Pin_set(DBG_PORT, DBG1_PIN, !transferDone);
#if defined(USE_DMA)
        HAL_Spi_ConfigDMA(&hspi, 0, 1);
        spiStat = HAL_Spi_ReceiveDMA(&hspi, spi_rx_data.spiword, SPI_WORDS_X_CH*ADC_CH_NUM);
#endif
#if defined(USE_IRQ)
        spiStat = HAL_Spi_ReceiveInt(&hspi, spi_rx_data.spiword, SPI_WORDS_X_CH*ADC_CH_NUM);
#endif
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
    Pin_on(DBG_PORT, DBG1_PIN);
    spiStat = HAL_Spi_Receive(&hspi, spi_rx_data.spiword, SPI_WORDLEN_X_CH*ADC_CH_NUM, 0);
    if(spiStat != hal_status_ok) {
        printf("Error: HAL_Spi_Receive() status: %s\n", HAL_StatusToString(spiStat));
    }
    process_spi_data();
    Pin_off(DBG_PORT, DBG1_PIN);
#endif
}

/* declared weak in va416xx_hal_spi.c
 * Used for non-blocking SPI transfers (interrupt and DMA)
 * called by Spi_Callback - HAL_Spi_Rx_Dma_Callback - DMA RX channel DONE interrupt
 * called by Spi_Callback - Spi_StateMachine - SPIn_RX_IRQHandler
 */
 void HAL_Spi1_Cmplt_Callback(hal_spi_handle_t* const hspi)
{
    if(hspi->state == hal_spi_state_error) {
        spiStat = hal_status_rxError; // receive overrun
        goto exit_cb;
    }
    
    spiStat = hal_status_ok;
    // Process the received raw data and copy to CAN buffers
    process_spi_data();

exit_cb :    
    transferDone = true;
    Pin_set(DBG_PORT, DBG1_PIN, !transferDone);

}

// function to allow external access to the data
void ADS1278_getADCs(ads1278_adc_data_t* data) {
    
    uint32_t cnt;
    __disable_irq();
    // Copy the processed data to the provided structure
    #pragma GCC unroll adc_ch_num
    for (int i = 0; i < adc_ch_num; i++) {
        // use shift operator to divide
        data->microVolts[i] = (adc_raw_sum[i] >> MAX_SMPL_POW2) * uV_TICK;
    }
    cnt = sampleCount;
    sampleCount = 0;
    __enable_irq();
    //printf("latest means: %u %ld %ld\n", 
    //        sampleCount, adc_raw_data[adc_raw_idx].ch[0], data->ch[0]);
    printf("%u samples - means %d uV\n", 
            cnt, (data->microVolts[0]));
    
}