/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * ESP32-S3 UART pure-logic helpers (shared with KUnit).
 *
 * Divider encoding per the official uart_ll formula:
 *   clk_div = (sclk << 4) / (baud * sclk_div)
 * CLKDIV = integer part + 4-bit fraction, sclk_div = 1 (APB 80 MHz).
 */
#ifndef _ESP32S3_UART_H
#define _ESP32S3_UART_H

#include <linux/bits.h>
#include <linux/bitfield.h>
#include <linux/errno.h>
#include <linux/math64.h>
#include <linux/types.h>

#define ESP32S3_UART_STATUS_RXFIFO_CNT		GENMASK(9, 0)
#define ESP32S3_UART_STATUS_TXFIFO_CNT		GENMASK(25, 16)
#define ESP32S3_UART_CONF0_PARITY		BIT(0)
#define ESP32S3_UART_CONF0_PARITY_EN		BIT(1)
#define ESP32S3_UART_CONF0_BIT_NUM		GENMASK(3, 2)
#define ESP32S3_UART_CONF0_STOP_BIT_NUM		GENMASK(5, 4)
#define ESP32S3_UART_CONF1_RXFIFO_FULL_THRHD	GENMASK(9, 0)
#define ESP32S3_UART_CONF1_TXFIFO_EMPTY_THRHD	GENMASK(19, 10)
#define ESP32S3_UART_CONF1_RX_TOUT_EN		BIT(23)
#define ESP32S3_UART_CLKDIV_DIV			GENMASK(11, 0)
#define ESP32S3_UART_CLKDIV_FRAG			GENMASK(23, 20)
#define ESP32S3_UART_MEM_CONF_RX_TOUT_THRHD	GENMASK(26, 17)
#define ESP32S3_UART_CLK_CONF_SCLK_DIV_NUM	GENMASK(19, 12)
#define ESP32S3_UART_CLK_CONF_SCLK_SEL		GENMASK(21, 20)
#define ESP32S3_UART_CLK_CONF_SCLK_EN		BIT(22)
#define ESP32S3_UART_CLK_CONF_TX_SCLK_EN	BIT(24)
#define ESP32S3_UART_CLK_CONF_RX_SCLK_EN	BIT(25)

static inline int esp32s3_uart_clk_div(unsigned long sclk, unsigned int baud,
				       u32 *clk_div, u8 *sclk_div_num)
{
	u64 denominator;
	u32 sclk_div;
	u32 div;

	if (!baud)
		return -EINVAL;

	denominator = (u64)FIELD_MAX(ESP32S3_UART_CLKDIV_DIV) * baud;
	sclk_div = DIV_ROUND_UP_ULL(sclk, denominator);
	if (!sclk_div || sclk_div >
	    FIELD_MAX(ESP32S3_UART_CLK_CONF_SCLK_DIV_NUM) + 1)
		return -EINVAL;

	denominator = (u64)baud * sclk_div;
	div = div_u64((u64)sclk << 4, denominator);
	if (div < 16 || (div >> 4) > FIELD_MAX(ESP32S3_UART_CLKDIV_DIV))
		return -EINVAL;

	*clk_div = div;
	*sclk_div_num = sclk_div - 1;
	return 0;
}

#endif /* _ESP32S3_UART_H */
