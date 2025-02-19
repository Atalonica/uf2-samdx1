/* ----------------------------------------------------------------------------
 *         SAM Software Package License
 * ----------------------------------------------------------------------------
 * Copyright (c) 2011-2014, Atmel Corporation
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following condition is met:
 *
 * Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the disclaimer below.
 *
 * Atmel's name may not be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * DISCLAIMER: THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * ----------------------------------------------------------------------------
 */

#ifndef _MAIN_H_
#define _MAIN_H_

#define CPU_FREQUENCY 48000000

#if defined(SAML21)
#define FLASH_WAIT_STATES 2
#else
#define FLASH_WAIT_STATES 1
#endif


#ifndef BOOT_USART_MODULE

  #if defined(SAML21)
    #define BOOT_USART_MODULE       SERCOM0
    #define BOOT_USART_PAD_SETTINGS UART_RX_PAD3_TX_PAD2
    #define BOOT_USART_PAD3         PINMUX_PA11C_SERCOM0_PAD3
    #define BOOT_USART_PAD2         PINMUX_PA10C_SERCOM0_PAD2
    #define BOOT_USART_PAD1         PINMUX_UNUSED
    #define BOOT_USART_PAD0         PINMUX_UNUSED
  #else
    #define BOOT_USART_MODULE       SERCOM3
    #define BOOT_USART_PAD_SETTINGS UART_RX_PAD1_TX_PAD0
    #define BOOT_USART_PAD3         PINMUX_UNUSED
    #define BOOT_USART_PAD2         PINMUX_UNUSED
    #define BOOT_USART_PAD1         PINMUX_PA23C_SERCOM3_PAD1
    #define BOOT_USART_PAD0         PINMUX_PA22C_SERCOM3_PAD0
  #endif

#endif

#endif // _MAIN_H_
