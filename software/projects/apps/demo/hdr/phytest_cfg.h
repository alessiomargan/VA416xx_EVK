/***************************************************************************************
 * @file     ethtest_configuration.h
 * @brief    Configuration file for the Ethernet test project
 * @version  V0.1
 * @date     27 July 2020
 *
 * @note
 * VORAGO Technologies
 *
 * @note
 * Copyright (c) 2013-2020 VORAGO Technologies.
 *
 * @par
 * BY DOWNLOADING, INSTALLING OR USING THIS SOFTWARE, YOU AGREE TO BE BOUND BY
 * ALL THE TERMS AND CONDITIONS OF THE VORAGO TECHNOLOGIES END USER LICENSE AGREEMENT.
 * THIS SOFTWARE IS PROVIDED "AS IS". NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 * OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY
 * AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. VORAGO TECHNOLOGIES
 * SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
 * DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 ****************************************************************************************/
#ifndef __PROJECT_CONFIGURATION_H
#define __PROJECT_CONFIGURATION_H

#include "stdint.h"
//unsigned long pingServerTimerRstValue;

//-------- <<< Use Configuration Wizard in Context Menu>>> -----------------
//
/*********************************************************/
/*            Ethernet configurations                    */
/*********************************************************/
// <h>Ethernet configurations
// <i>Group of all Ethernet test switches

		/*********************************************************/
		/*            LoopBack configurations                    */
		/*********************************************************/
		// <h> Ethernet loopback configuration
		// <i> Select loopback method.
		//
		// <o>Type of loopback:
		//   <0=> None
		//   <1=> MAC Loopback
		//   <2=> PHY Loopback
		// <i> Test cases available:
		// <i> Normal (default)
		// <i> MAC Loopback
		// <i> PHY Loopback
		#ifndef LOOPBACKCASE
			#define LOOPBACKCASE 2
		#endif

		/*         LOOPBACK_MAC                 */
		// <c1> Enable loopback (LOOPBACK_MAC)
		// <i> defines LOOPBACK_MAC
		//#ifndef LOOPBACK_MAC
			//#define LOOPBACK_MAC     
		//#endif
		// </c>
		//

		// </h> ETHERNET LOOPBACK CONFIGURATION
		/*********************************************************/
		/*         END LOOPBACK CONFIGURATIONS                   */
		/*********************************************************/
		//
		/*********************************************************/
		/*         Ethernet macro configurations                 */
		/*********************************************************/
		// <h>Ethernet macro configurations
		//
				/*********************************************************/
				/*         Common configurations                         */
				/*********************************************************/
				// <h>Common configurations
				//
				/*         SHOWALLIP_MESSAGES                 */
				// <c1> Output all UDP messages (SHOWALLIP_MESSAGES)
				// <i> Defines ONLYLOCALIP_MESSAGES
				#ifndef SHOWALLIP_MESSAGES
					#define SHOWALLIP_MESSAGES
				#endif
				// </c>
				//<i> Output both local and nonlocal packets information
				//
				/*         BUFFER_MESSAGES                 */
				// <c1> Output Rx/Tx buffer usage (BUFFER_MESSAGES)
				// <i> Defines BUFFER_MESSAGES
				#ifndef BUFFER_MESSAGES
					#define BUFFER_MESSAGES
				#endif
				// </c>
				//
				/*         TIMESTAMP_MESSAGE                 */
				// <c1> Output Timestamp in ms (TIMESTAMP_MESSAGE)
				// <i> Defines TIMESTAMP_MESSAGE
				#ifndef TIMESTAMP_MESSAGE
					#define TIMESTAMP_MESSAGE
				#endif
				// </c>
				//
				/*         DEBUG_UDP_MESSAGE                 */
				// <c1> Output debug info for each UDP msgs (DEBUG_UDP_MESSAGE)
				// <i> Defines DEBUG_UDP_MESSAGE
				#ifndef DEBUG_UDP_MESSAGE
					#define DEBUG_UDP_MESSAGE
				#endif
				// </c>
				//
				/*-----------DEBUG_PHY_CONFIG---------------------*/
				// <c1> Output Advertised link attribute on startup (DEBUG_PHY_CONFIG)
				//<i> Advertised link attributes:
				#ifndef DEBUG_PHY_CONFIG
					#define DEBUG_PHY_CONFIG
				#endif
				// </c>
				//<i> ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
				//<i> Advertised link attributes:
				//<i> PHY Speed:            100Mbps
				//<i> PHY Auto-negotiation: Enabled
				//<i> PHY Loopback Mode:    OFF
				//<i> PHY Duplex Mode:      Full
				//<i> ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
				//
				/*-----------PRINT_MAC_DMA_REGS ----------------*/
				// <c1> Output MAC and DMA regsiters on startup (PRINT_MAC_DMA_REGS)
				// <i> PRINT_MAC_DMA_REGS
				//#ifndef PRINT_MAC_DMA_REGS
					//#define PRINT_MAC_DMA_REGS
				//#endif
				// </c>
				// <i> + MAC ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
				// <i>         CSR  0 -MAC Configuration:              0x00C0880C
				// <i>         CSR  1 -MAC Frame Filter:               0x80000001
				// <i>         CSR  6 -MAC Flow Control:               0x00000006
				// <i>         CSR  7 -MAC VLAN:                       0x00000000
				// <i>         CSR  9 -MAC Debug:                      0x00000120
				// <i>         CSR 14 -MAC Interrupt Status:           0x00000000
				// <i>         CSR 15 -MAC Interrupt Mask:             0x00000000
				// <i> + DMA ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
				// <i>        CSR  0 -DMA Bus Mode:                   0x00100800
				// <i>        CSR  3 -DMA RxBaseAddr:                 0x1FFFB640
				// <i>        CSR  4 -DMA Tx Desc Base Addr:          0x1FFFB600
				// <i>        CSR  5 -DMA Status:                     0x00000000
				// <i>        CSR  6 -DMA Operation Mode:             0x02200000
				// <i>        CSR  7 -DMA Interrupt Enable:           0x0001C7FF
				// <i>        CSR  8 -DMA Missed Over Frame Count:    0x00000000
				// <i>

				// </h>COMMON CONFIGURATION
				/*********************************************************/
				/*         COMMON CONFIGURATIONS                         */
				/*********************************************************/
// </h>
				/*********************************************************/
				/*         Less Common Configurations                    */
				/*********************************************************/
				// <h>Less Common configurations
				//
				// <c1> Output Ethernet MAC Debug register for each Rx/Tx (MAC_DEBUG_REGISTER)
				// <i> Enables MAC Debug register for each packet
				#ifndef MAC_DEBUG_REGISTER
					#define MAC_DEBUG_REGISTER
				#endif
				// </c>
				//<i> (verbose) This will generate a lot of output
				// <c1> Output Ethernet MAC Debug register (verbose) for each Rx/Tx (MAC_DEBUG_VERBOSE)
				// <i> Enables MAC Debug register for each packet
				//#ifndef MAC_DEBUG_VERBOSE
					//#define MAC_DEBUG_VERBOSE
				//#endif
				// </c>
				//<i> (verbose) This will generate a lot of output
				//<i> ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
				//<i> For Example:  
				//<i> ------ MAC Debug: 0x00000123 ------
				//<i>[24]    MTL Tx FIFO is Empty
				//<i>[22]    MTL Tx FIFO Write Ctrlr is Idle
				//<i>[21:20] MTL Tx FIFO Read Controller: Idle
				//<i>[19]    MAC Transmitter is Active
				//<i>[18:17] MAC Transmit Frame Controller: Idle
				//<i>[16]    MAC MII Tx Protocol Engine is Idle
				//<i>[9:8]   MTL RxFIFO Fill-Level: below flow-control deactivate threshold
				//<i>[6:5]   MTL RxFIFO Read Controller: Reading frame data
				//<i>[4]     MTL Rx FIFO Write Controller is Idle
				//<i>[2]     MAC Rx Frame Small FIFO Read Controller is Idle
				//<i>[1]     MAC Rx Frame FIFO Write Controller is Active
				//<i>[0]     MAC MII Receive Protocol Engine is Active
				//<i> ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
				//
				//
				//<o0.1..31> pingServerTimerRstValue <0x0-0xFFFFFFFF:0x100000><#/0x100000>
				//<i> Time delay between Tx packets
				 unsigned long pingServerTimerRstValue1 = 0x80000009;   // pingServerTimerRstValue

				

				// <o>Select source for CRC (USE_LOCAL_CRC):
				//   <0=> Hardware
				//   <1=> Software
				// <i> Sets USE_LOCAL_CRC
				#ifndef USE_LOCAL_CRC
					#define USE_LOCAL_CRC	0
				#endif
				//
				// <o>Select UART for Serial Comms:  (UARTX_ACTIVE)
				//  <0=> UART0
				// <i> UARTs Available (currently PA3,PA4 and PG0,PG1 are BOTH connected to UART0)
				// <i> UART0 (default)
				#ifndef UART0_ACTIVE
					#define UART0_ACTIVE
				#endif
				//
				//
				//</h> LESS COMMON CONFIGURATIONS
				/*********************************************************/
				/*         LESS COMMON CONFIGURATIONS                    */
				/*********************************************************/
				//

		// </h>ETHERNET MACRO CONFIGURATIONS
		/*********************************************************/
		/*         ETHERNET MACRO CONFIGURATIONS                 */
		/*********************************************************/
// </h>ETHERNET CONFIGURATIONS
/*********************************************************/
/*         ETHERNET CONFIGURATIONS                       */
/*********************************************************/
//------------- <<< end of configuration section>>> -----------------------

#endif // __PROJECT_CONFIGURATION_H
