# CAN Message Table Documentation

## Overview
This document describes the CAN message IDs and their associated data structures used in the VA416xx FT_VOR application. The system uses two CAN buses (CAN0 and CAN1) with Remote Transmission Request (RTR) handling for ADC data transfer.

## CAN Message Structure

### Message Entry Definition
```c
typedef struct {
    uint32_t can_id;           // CAN Message ID (11-bit or 29-bit)
    uint8_t data_length;       // Data length in bytes (0-8)
    uint8_t data[8];           // CAN message data payload
    uint32_t frequency_ms;     // Message transmission frequency (milliseconds)
    const char *description;   // Description of the CAN message
} can_message_t;
```

## CAN0 Message Table

CAN0 is configured for RTR (Remote Transmission Request) handling with automatic responses. Each RTR channel has a request buffer and a response buffer. **Only 4 RTR response buffers are actively used** by the ADS1278 driver (set_cmb_data function).

| ID (hex) | Channel | Buffer RX | Buffer TX | ADC Channels | Data Length | Frequency (ms) | Description |
|----------|---------|-----------|-----------|--------------|-------------|----------------|-------------|
| 0x100    | 0       | CMB0      | CMB1      | Ch0, Ch1     | 8 bytes     | On-demand      | ADS1278 ADC Channels 0-1 |
| 0x101    | 1       | CMB2      | CMB3      | Ch2, Ch3     | 8 bytes     | On-demand      | ADS1278 ADC Channels 2-3 |
| 0x102    | 2       | CMB4      | CMB5      | Ch4, Ch5     | 8 bytes     | On-demand      | ADS1278 ADC Channels 4-5 |
| 0x103    | 3       | CMB6      | CMB7      | Ch6, Ch7     | 8 bytes     | On-demand      | ADS1278 ADC Channels 6-7 |
| 0x104    | 4       | CMB8      | CMB9      | Unused       | 8 bytes     | -              | RTR Response Configured (Not Used) |
| 0x105    | 5       | CMB10     | CMB11     | Unused       | 8 bytes     | -              | RTR Response Configured (Not Used) |

### CAN0 Standard Messages
| ID (hex) | Buffer | Data Length | Purpose | Description |
|----------|--------|-------------|---------|-------------|
| Various  | CMB12  | 8 bytes     | TX      | AFE Data Responses |
| Various  | CMB13  | 8 bytes     | TX      | Error Responses |
| Various  | CMB14  | 8 bytes     | RX      | Catch-all for standard ID messages |

## CAN1 Message Table

CAN1 is configured for RTR handling with extended message support. Similar to CAN0, each RTR channel has paired request/response buffers.

| ID (hex) | Channel | Buffer RX | Buffer TX | Data Length | Frequency (ms) | Description |
|----------|---------|-----------|-----------|-------------|----------------|-------------|
| 0x200    | 0       | CMB0      | CMB1      | 8 bytes     | 100            | CAN1 RTR Channel 0 - ADC Data Request/Response |
| 0x201    | 1       | CMB2      | CMB3      | 8 bytes     | 100            | CAN1 RTR Channel 1 - ADC Data Request/Response |
| 0x202    | 2       | CMB4      | CMB5      | 8 bytes     | 100            | CAN1 RTR Channel 2 - ADC Data Request/Response |
| 0x203    | 3       | CMB6      | CMB7      | 8 bytes     | 100            | CAN1 RTR Channel 3 - ADC Data Request/Response |
| 0x204    | 4       | CMB8      | CMB9      | 8 bytes     | 100            | CAN1 RTR Channel 4 - ADC Data Request/Response |
| 0x205    | 5       | CMB10     | CMB11     | 8 bytes     | 100            | CAN1 RTR Channel 5 - ADC Data Request/Response |
| 0x206    | 6       | CMB12     | CMB13     | 8 bytes     | 100            | CAN1 RTR Channel 6 - ADC Data Request/Response |

### CAN1 Extended Messages
| ID (hex) | Buffer | Data Length | Purpose | Description |
|----------|--------|-------------|---------|-------------|
| Various  | CMB14  | 8 bytes     | RX      | Catch-all for extended ID messages |

## Configuration Parameters

### CAN Bus Timing
- **System Clock**: 100 MHz
- **Peripheral Clock (APB1)**: 50 MHz
- **Baud Rate**: 250 kbps (default)
- **Time Quanta**:
  - TSEG1 = 3 time quanta
  - TSEG2 = 6 time quanta
  - SJW = 1 time quanta
  - PSC (Prescaler) = 20

### Alternative Baud Rates
- **500 kbps**: PSC = 10
- **250 kbps**: PSC = 20 (default)
- **125 kbps**: PSC = 40

## Data Payload Examples

### ADS1278 ADC Data Format (CAN0: 0x100-0x103)
Each CAN message contains data from **2 ADC channels**. The ADC values are 32-bit signed integers in **microvolts (µV)**, split across the 8-byte payload as follows:

#### Byte Layout
| Byte Offset | CAN Register | Content | Description |
|-------------|--------------|---------|-------------|
| 0-1         | DATA0        | MSB     | ADC Channel N - Upper 16 bits (microvolts) |
| 2-3         | DATA1        | LSB     | ADC Channel N - Lower 16 bits (microvolts) |
| 4-5         | DATA2        | MSB     | ADC Channel N+1 - Upper 16 bits (microvolts) |
| 6-7         | DATA3        | LSB     | ADC Channel N+1 - Lower 16 bits (microvolts) |

#### Mapping of CAN IDs to ADC Channels
```c
// From set_cmb_data() in ads1278.c
CAN ID 0x100 (CMB1): ADC Ch0 (DATA0:DATA1) + Ch1 (DATA2:DATA3)
CAN ID 0x101 (CMB3): ADC Ch2 (DATA0:DATA1) + Ch3 (DATA2:DATA3)
CAN ID 0x102 (CMB5): ADC Ch4 (DATA0:DATA1) + Ch5 (DATA2:DATA3)
CAN ID 0x103 (CMB7): ADC Ch6 (DATA0:DATA1) + Ch7 (DATA2:DATA3)
```

#### Data Calculation
```c
// Reconstructing the 32-bit value from CAN message
int32_t adc_ch0_uV = (DATA0 << 16) | DATA1;  // Channel 0 in microvolts
int32_t adc_ch1_uV = (DATA2 << 16) | DATA3;  // Channel 1 in microvolts

// Original calculation in set_cmb_data():
// Each value = (averaged_raw_counts * uV_TICK)
// Where averaged_raw_counts = (adc_raw_sum[ch] >> MAX_SMPL_POW2)
```

#### Example Values
For CAN ID 0x100 with ADC Ch0 = +5,000,000 µV (5V) and Ch1 = -1,000,000 µV (-1V):
```
DATA0 = 0x004C     // MSB of 5,000,000 µV
DATA1 = 0x4B40     // LSB of 5,000,000 µV
DATA2 = 0xFFF0     // MSB of -1,000,000 µV  
DATA3 = 0xBDC0     // LSB of -1,000,000 µV
```

### Error Response Format
- Byte 0: Error code
- Byte 1: Channel/Module identifier
- Bytes 2-7: Additional error information

## RTR Handling

### Remote Transmission Request (RTR) Process
1. External system sends RTR message with matching ID (0x100-0x105 for CAN0, 0x200-0x206 for CAN1)
2. The corresponding RX buffer detects the RTR request
3. The paired TX buffer automatically transmits the response with current ADC data
4. No software intervention required for automatic responses

### Message Buffer Assignment
- **Even Buffer Numbers (0, 2, 4, ...)**: RX buffers for RTR requests
- **Odd Buffer Numbers (1, 3, 5, ...)**: TX buffers for RTR responses
- **CMB12/CMB13 (CAN0)**: Standard message transmission
- **CMB14 (CAN0/CAN1)**: Catch-all RX buffer for non-RTR messages

## Hardware Interrupts

### CAN0 Interrupt
- **IRQ Handler**: CAN0_RxMsg_IRQHandler()
- **Priority Level**: Configured via NVIC
- **Trigger Sources**:
  - Message received in RX buffers
  - Error conditions (if enabled)

### CAN1 Interrupt
- **IRQ Handler**: CAN1_RxMsg_IRQHandler()
- **Priority Level**: Configured via NVIC
- **Trigger Sources**:
  - Message received in RX buffers (extended IDs)
  - Error conditions (if enabled)

## Notes

- All RTR channels are set to automatic response mode with BUFFLOCK enabled
- Time stamping is enabled for all received messages
- Diagnostic functions are enabled for error tracking and debugging
- The CAN buses operate independently with their own clock domains
- ACK ignoring is enabled for loopback testing if needed
