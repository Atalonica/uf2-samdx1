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

#ifndef UART_DRIVER_H
#define UART_DRIVER_H
#include "sam.h"
#include <stdbool.h>
#include <stdio.h>

#define PINMUX_UNUSED 0xFFFFFFFF

#ifdef SAML21
#define GCLK_CLKCTRL_ID_SERCOM0_CORE	0x18
#define GCLK_CLKCTRL_ID_SERCOM1_CORE	0x19
#define GCLK_CLKCTRL_ID_SERCOM2_CORE	0x20
#define GCLK_CLKCTRL_ID_SERCOM3_CORE	0x21
#define GCLK_CLKCTRL_ID_SERCOM4_CORE	0x22
#define GCLK_CLKCTRL_ID_SERCOM5_CORE	0x24
#endif

#ifdef SAMD21
#define GCLK_CLKCTRL_ID_SERCOM0_CORE	0x14
#define GCLK_CLKCTRL_ID_SERCOM1_CORE	0x15
#define GCLK_CLKCTRL_ID_SERCOM2_CORE	0x16
#define GCLK_CLKCTRL_ID_SERCOM3_CORE	0x17
#define GCLK_CLKCTRL_ID_SERCOM4_CORE	0x18
#define GCLK_CLKCTRL_ID_SERCOM5_CORE	0x19
#endif

#ifdef SAMD51
#define GCLK_CLKCTRL_ID_SERCOM0_CORE    0x7
#define GCLK_CLKCTRL_ID_SERCOM1_CORE    0x8
#define GCLK_CLKCTRL_ID_SERCOM2_CORE    0x23
#define GCLK_CLKCTRL_ID_SERCOM3_CORE    0x24
#define GCLK_CLKCTRL_ID_SERCOM4_CORE    0x34
#define GCLK_CLKCTRL_ID_SERCOM5_CORE    0x35
#define GCLK_CLKCTRL_ID_SERCOM6_CORE    0x36
#define GCLK_CLKCTRL_ID_SERCOM7_CORE    0x37
#endif

/* SERCOM UART available pad settings */
enum uart_pad_settings {
    UART_RX_PAD0_TX_PAD2 = SERCOM_USART_CTRLA_RXPO(0) | SERCOM_USART_CTRLA_TXPO(1),
    UART_RX_PAD1_TX_PAD2 = SERCOM_USART_CTRLA_RXPO(1) | SERCOM_USART_CTRLA_TXPO(1),
    UART_RX_PAD2_TX_PAD0 = SERCOM_USART_CTRLA_RXPO(2),
    UART_RX_PAD3_TX_PAD0 = SERCOM_USART_CTRLA_RXPO(3),
    UART_RX_PAD1_TX_PAD0 = SERCOM_USART_CTRLA_RXPO(1),
    UART_RX_PAD3_TX_PAD2 = SERCOM_USART_CTRLA_RXPO(3) | SERCOM_USART_CTRLA_TXPO(1),
};

/**
 * \brief Gets the index of the provided SERCOM instance
 *
 * \param Pointer to SERCOM instance
 * \return Index of the SERCOM module
 */
uint32_t uart_get_sercom_index(Sercom *sercom_instance);

/**
 * \brief Initializes the UART
 *
 * \param Pointer to SERCOM instance
 * \param Baud value corresponding to the desired baudrate
 * \param SERCOM pad settings
 */
void uart_basic_init(Sercom *sercom, uint16_t baud_val, enum uart_pad_settings pad_conf);

/**
 * \brief Disables UART interface
 *
 * \param Pointer to SERCOM instance
 */
void uart_disable(Sercom *sercom);

/**
 * \brief Sends a single byte through UART interface
 *
 * \param Pointer to SERCOM instance
 * \param Data to send
 */
void uart_write_byte(Sercom *sercom, uint8_t data);

/**
 * \brief Reads a single character from UART interface
 *
 * \param Pointer to SERCOM instance
 * \return Data byte read
 */
uint8_t uart_read_byte(Sercom *sercom);

/**
 * \brief Sends buffer on UART interface
 *
 * \param Pointer to SERCOM instance
 * \param Pointer to data to send
 * \param Number of bytes to send
 */
void uart_write_buffer_polled(Sercom *sercom, uint8_t *ptr, uint16_t length);

/**
 * \brief Reads data on UART interface
 *
 * \param Pointer to SERCOM instance
 * \param Pointer to store read data
 * \param Number of bytes to read
 */
void uart_read_buffer_polled(Sercom *sercom, uint8_t *ptr, uint16_t length);

#endif
