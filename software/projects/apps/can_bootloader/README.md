# CAN Bootloader for VA416xx

## Overview

This is a CAN bus-based bootloader for the VA416xx microcontroller family. It allows firmware updates to be sent over the CAN bus, providing an alternative to the UART/XMODEM-based bootloader.

## Features

- **CAN-based firmware upload**: Receive firmware binary files over CAN bus
- **Dual app slots**: Supports two application slots (A and B) for firmware redundancy
- **CRC validation**: Verifies firmware integrity using CRC16
- **F-RAM storage**: Stores firmware in non-volatile F-RAM
- **Standard CAN frames**: Uses 11-bit standard CAN identifiers
- **Automatic app execution**: Boots valid applications automatically on power-up

## CAN Protocol Specification

### CAN Message IDs

| ID    | Direction        | Purpose                           |
|-------|------------------|-----------------------------------|
| 0x200 | Host → BL        | Bootloader commands               |
| 0x201 | Host → BL        | Firmware data packets             |
| 0x202 | BL → Host        | Bootloader responses              |

### Command Messages (ID 0x200)

Commands are sent in the first byte of the message payload.

| Command | Code | Data Bytes | Description                          |
|---------|------|------------|--------------------------------------|
| PING    | 0x01 | -          | Check if bootloader is active        |
| START_UL| 0x02 | 4 bytes    | Start firmware upload                |
|         |      |            | [1]: Slot (0=A, 1=B)                 |
|         |      |            | [2-4]: Size (24-bit, little-endian)  |
| GET_STATUS | 0x06 | -      | Request bootloader status            |
| RUN_APP | 0x05 | -          | Run the application                  |
| RESET   | 0x07 | -          | Reset the device                     |

### Data Messages (ID 0x201)

Data packets contain firmware binary data. Each message contains:
- Byte 0: Sequence number (0-255, wraps around)
- Bytes 1-7: Up to 7 bytes of firmware data

### Response Messages (ID 0x202)

The bootloader sends responses with the following codes in the first byte:

| Response     | Code | Description                    |
|--------------|------|--------------------------------|
| ACK          | 0x01 | Command acknowledged           |
| NAK          | 0x02 | Command not acknowledged       |
| READY        | 0x03 | Ready to receive data          |
| BUSY         | 0x04 | Busy, try again                |
| ERROR        | 0x05 | Error occurred                 |
| CRC_OK       | 0x06 | CRC check passed               |
| CRC_FAIL     | 0x07 | CRC check failed               |
| APP_VALID    | 0x08 | Application is valid           |
| APP_INVALID  | 0x09 | Application is invalid         |

## Firmware Upload Procedure

1. **Initiate bootloader entry**:
   - Send PING command (ID 0x200, byte 0 = 0x01)
   - Wait for ACK response (ID 0x202, byte 0 = 0x01)

2. **Start upload**:
   - Send START_UL command with slot and size
   - Example: [0x02, 0x00, 0x00, 0x10, 0x00] for slot A, 4096 bytes
   - Wait for READY response (0x03)

3. **Transfer firmware**:
   - Send data packets (ID 0x201)
   - Format: [seq, data0, data1, ..., data6]
   - Wait for ACK with sequence number after each packet
   - Increment sequence number for each packet (wraps at 256)

4. **Complete transfer**:
   - When all bytes sent, bootloader sends CRC_OK (0x06)
   - Firmware is automatically written to F-RAM and verified

5. **Run application** (optional):
   - Send RUN_APP command (0x05)
   - Or reset device - bootloader will auto-run valid app

## CAN Bus Configuration

- **Baud rate**: 250 kbps (configurable in can.c)
- **Bit timing**: Configured for 80 MHz system clock
  - TSEG1 = 13
  - TSEG2 = 2
  - SJW = 1
  - Clock divider = 4
- **Message buffers**:
  - CMB0: RX for command messages (ID 0x200)
  - CMB1: RX for data messages (ID 0x201)
  - CMB2: TX for response messages (ID 0x202)

## Memory Map

Same as standard bootloader:

```
0x00000 - 0x03FFE  Bootloader code
0x03FFE - 0x03FFF  Bootloader CRC
0x04000 - 0x21FF7  Application A code space (~120KB)
0x21FF8 - 0x21FFB  Application A CRC length
0x21FFC - 0x21FFF  Application A CRC value
0x22000 - 0x3FFF7  Application B code space (~120KB)
0x3FFF8 - 0x3FFFB  Application B CRC length
0x3FFFC - 0x3FFFF  Application B CRC value
```

## Building

```bash
cd software/projects
make init
make can_bootloader
```

The output files will be in `build/`:
- `can_bootloader.elf` - ELF executable
- `can_bootloader.hex` - Intel HEX format
- `can_bootloader.map` - Memory map

## Pinout

CAN pins (configured in board.c):
- CAN0_TX: Configurable via IOCFG
- CAN0_RX: Configurable via IOCFG

LED:
- PG5: Status LED (blinks at 1 Hz)

## Usage

1. Flash the CAN bootloader to the VA416xx device
2. On power-up, bootloader waits 5 seconds for PING command
3. If no PING received, attempts to run valid application
4. Send firmware via CAN using the protocol above
5. After successful upload, application runs automatically

## Timeouts

- **Bootloader entry**: 5 seconds (configurable via BOOTLOADER_MSEC_TIMEOUT)
- **Data receive**: 5 seconds between packets (CAN_BL_RX_TIMEOUT_MS)

## Error Handling

- If CRC check fails, bootloader returns to idle state
- If upload interrupted, send START_UL again to restart
- Use GET_STATUS command to check bootloader state and bytes received

## Differences from UART Bootloader

| Feature              | UART Bootloader | CAN Bootloader    |
|----------------------|-----------------|-------------------|
| Interface            | UART0/UART1     | CAN0              |
| Protocol             | XMODEM-CRC      | Custom CAN        |
| Packet size          | 128 bytes       | 7 bytes           |
| Baud rate            | 115200 bps      | 250 kbps          |
| Interactive mode     | Yes (PC_MODE)   | No (CAN only)     |
| Slot selection       | Interactive     | In START_UL cmd   |

## Future Enhancements

Possible improvements:
- Add support for CAN29 (extended identifiers)
- Implement flow control for faster transfers
- Add encryption/authentication
- Support for bootloader self-update
- Add CAN1 support for redundancy

## License

Copyright (c) 2013-2026 VORAGO Technologies
