/**
 * \file
 *
 * \brief Autogenerated API include file for the Atmel Software Framework (ASF)
 *
 * Copyright (c) 2012 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 *    Atmel microcontroller product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 */

#ifndef ASF_H
#define ASF_H

/*
 * This file includes all API header files for the selected drivers from ASF.
 * Note: There might be duplicate includes required by more than one driver.
 *
 * The file is automatically generated and will be re-written when
 * running the ASF driver selector tool. Any changes will be discarded.
 */

// From module: CPU Cycle Counter
#include <cycle_counter.h>

// From module: Compiler abstraction layer and code utilities
#include <compiler.h>
#include <status_codes.h>

// From module: EVK1100 board support
#include <led.h>

// From module: Ethernet Physical (DP83848)
#include <dp83848.h>

// From module: Ethernet Physical Device Manager
#include <ethernet_phy.h>

// From module: FLASHC - Flash Controller
#include <flashc.h>

// From module: FreeRTOS mini Real-Time Kernel
#include <FreeRTOS.h>
#include <StackMacros.h>
#include <croutine.h>
#include <list.h>
#include <mpu_wrappers.h>
#include <portable.h>
#include <projdefs.h>
#include <queue.h>
#include <semphr.h>
#include <task.h>
#include <timers.h>

// From module: GPIO - General-Purpose Input/Output
#include <gpio.h>

// From module: Generic board support
#include <board.h>

// From module: INTC - Interrupt Controller
#include <intc.h>

// From module: IPv4 support in lwIP
#include <lwip/autoip.h>
#include <lwip/icmp.h>
#include <lwip/igmp.h>
#include <lwip/inet.h>
#include <lwip/inet_chksum.h>
#include <lwip/ip.h>
#include <lwip/ip_addr.h>
#include <lwip/ip_frag.h>

// From module: Interrupt management - UC3 implementation
#include <interrupt.h>

// From module: LCD Display - DIP204B-4ORT01
#include <dip204.h>

// From module: MACB - Ethernet MAC
#include <macb.h>

// From module: PM Power Manager- UC3 A0/A1/A3/A4/B0/B1 implementation
#include <power_clocks_lib.h>
#include <sleep.h>

// From module: PWM - UC3 A/B implementation
#include <pwm.h>

// From module: SPI - Serial Peripheral Interface
#include <spi.h>

// From module: System Clock Control - UC3 A implementation
#include <sysclk.h>

// From module: TC - Timer/Counter
#include <tc.h>

// From module: TIMING - Delay routines
#include <delay.h>

// From module: USART - Universal Synchronous/Asynchronous Receiver/Transmitter
#include <usart.h>

// From module: lwIP TCP/IP Stack
#include <lwip/api.h>
#include <lwip/api_msg.h>
#include <lwip/arch.h>
#include <lwip/debug.h>
#include <lwip/def.h>
#include <lwip/dhcp.h>
#include <lwip/dns.h>
#include <lwip/err.h>
#include <lwip/init.h>
#include <lwip/mem.h>
#include <lwip/memp.h>
#include <lwip/memp_std.h>
#include <lwip/netbuf.h>
#include <lwip/netdb.h>
#include <lwip/netif.h>
#include <lwip/netifapi.h>
#include <lwip/opt.h>
#include <lwip/pbuf.h>
#include <lwip/raw.h>
#include <lwip/sio.h>
#include <lwip/snmp.h>
#include <lwip/snmp_asn1.h>
#include <lwip/snmp_msg.h>
#include <lwip/snmp_structs.h>
#include <lwip/sockets.h>
#include <lwip/stats.h>
#include <lwip/sys.h>
#include <lwip/tcp.h>
#include <lwip/tcpip.h>
#include <lwip/udp.h>

// From module: lwIP port to FreeRTOS on AVR UC3 MACB hardware module.
#include <arch/cc.h>
#include <arch/perf.h>
#include <arch/sys_arch.h>
#include <netif/ethernetif.h>

#endif // ASF_H
