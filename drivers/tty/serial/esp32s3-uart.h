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

#include <linux/errno.h>
#include <linux/math.h>
#include <linux/types.h>

static inline int esp32s3_uart_clk_div(unsigned long sclk, unsigned int baud,
				       u32 *clk_div)
{
	u32 div;

	if (!baud)
		return -EINVAL;

	div = DIV_ROUND_CLOSEST(sclk << 4, baud);
	if (div < 16)
		return -EINVAL;

	*clk_div = div;
	return 0;
}

#endif /* _ESP32S3_UART_H */
