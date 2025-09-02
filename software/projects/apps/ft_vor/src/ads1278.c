#include "board.h"
#include "ads1278.h"

#include "va416xx_hal.h"
#include "va416xx_hal_spi.h"

#define DRDY_PIN    0
#define DRDY_PORT   PORTF

static hal_spi_handle_t hspi;
static volatile hal_status_t spiStat;
static volatile bool rxDmaDone;
static uint16_t rxBuf[12];
static uint32_t rxDmaBuf[24];
static uint32_t txDmaBuf[24];
static uint32_t dummy = SPI_DATA_BMSTART_BMSTOP_Msk;

void ConfigureADS1278(void) {
    
    HAL_DMA_Init(NULL, false, false, false);
    
    // config SPI0
    HAL_UNLOCK(&hspi);
    hspi.spi = VOR_SPI1;
    hspi.init.blockmode = true;
    hspi.init.bmstall = true;
    hspi.init.clkDiv = 2;
    hspi.init.loopback = false;
    hspi.init.mdlycap = false;
    hspi.init.mode = hal_spi_clkmode_0;
    hspi.init.ms = hal_spi_ms_master;
    hspi.init.chipSelect = 0;
    hspi.init.wordLen = 16;
    spiStat = HAL_Spi_Init(&hspi);
    if(spiStat != hal_status_ok) {
        printf("Error: HAL_Spi_Init() status: %s\n", HAL_StatusToString(spiStat));
        return;
    }
    spiStat = HAL_Spi_ConfigDMA(&hspi, 0, 1);
    if(spiStat != hal_status_ok)
    {
      printf("Error: HAL_Spi_ConfigDMA() status: %s\n", HAL_StatusToString(spiStat));
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

    rxDmaDone = false;
    //spiStat = HAL_Spi_Receive(&hspi, rxBuf, 12, 100);
    spiStat = HAL_Spi_ReceiveDMA(&hspi, rxDmaBuf, 12);
    if(spiStat != hal_status_ok) {
        printf("Error: HAL_Spi_Receive() status: %s\n", HAL_StatusToString(spiStat));
    }
#if 0
    int timeout = 1000; // ms
      while(!rxDmaDone){
        if(--timeout == 0) {
            printf("Error: timeout\n");
            break;
        }
        //HAL_WaitMs(1);
    }
    if(spiStat == hal_status_rxError) {
        printf("Error: receive overrun\n");
    }
#endif
}

void HAL_Spi_Cmplt_Callback(hal_spi_handle_t* hdl)
{
    if(hdl != &hspi) {
        return;
    }
    
    if(hdl->state != hal_spi_state_error) {
        spiStat = hal_status_ok;
    } else {
        spiStat = hal_status_rxError; // receive overrun
    }
 
    rxDmaDone = true;

}