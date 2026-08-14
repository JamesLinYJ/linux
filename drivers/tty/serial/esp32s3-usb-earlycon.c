// SPDX-License-Identifier: GPL-2.0-only
/*
 * Minimal early console for the ESP32-S3 USB Serial/JTAG peripheral.
 *
 * This intentionally implements output only. The eventual runtime TTY
 * driver owns interrupts, receive handling and the complete device lifecycle.
 */

#include <linux/console.h>
#include <linux/io.h>
#include <linux/serial_core.h>

#include "esp32s3-usb-serial.h"

static bool esp32s3_usb_earlycon_tx_ready(struct uart_port *port)
{
	unsigned int retry;

	for (retry = 0; retry < ESP32S3_USB_SERIAL_POLL_LIMIT; retry++) {
		if (readl_relaxed(port->membase +
				  ESP32S3_USB_SERIAL_EP1_CONF) &
		    ESP32S3_USB_SERIAL_TX_FREE)
			return true;
		cpu_relax();
	}

	return false;
}

static bool esp32s3_usb_earlycon_putc(struct uart_port *port,
				      unsigned char ch)
{
	if (!esp32s3_usb_earlycon_tx_ready(port)) {
		writel(ESP32S3_USB_SERIAL_WR_DONE,
		       port->membase + ESP32S3_USB_SERIAL_EP1_CONF);
		if (!esp32s3_usb_earlycon_tx_ready(port))
			return false;
	}

	writel(ch, port->membase + ESP32S3_USB_SERIAL_EP1);
	return true;
}

static void esp32s3_usb_earlycon_write(struct console *console,
				       const char *string,
				       unsigned int count)
{
	struct earlycon_device *device = console->data;
	struct uart_port *port = &device->port;
	unsigned int bytes = 0;
	unsigned int index;
	unsigned int retry;

	for (index = 0; index < count; index++) {
		if (string[index] == '\n') {
			if (!esp32s3_usb_earlycon_putc(port, '\r'))
				break;
			bytes++;
		}
		if (!esp32s3_usb_earlycon_putc(port, string[index]))
			break;
		bytes++;
	}

	if (bytes) {
		writel(ESP32S3_USB_SERIAL_WR_DONE,
		       port->membase + ESP32S3_USB_SERIAL_EP1_CONF);
		if (!(bytes % ESP32S3_USB_SERIAL_FIFO_SIZE)) {
			for (retry = 0; retry < ESP32S3_USB_SERIAL_POLL_LIMIT;
			     retry++) {
				if (readl_relaxed(port->membase +
						  ESP32S3_USB_SERIAL_EP1_CONF) &
				    ESP32S3_USB_SERIAL_TX_FREE) {
					writel(ESP32S3_USB_SERIAL_WR_DONE,
					       port->membase +
					       ESP32S3_USB_SERIAL_EP1_CONF);
					break;
				}
				cpu_relax();
			}
		}
	}
}

static int __init
esp32s3_usb_earlycon_setup(struct earlycon_device *device,
			   const char *options)
{
	(void)options;
	if (!device->port.membase)
		return -ENODEV;

	device->con->write = esp32s3_usb_earlycon_write;
	return 0;
}

OF_EARLYCON_DECLARE(esp32s3_usb,
		    "esp,esp32s3-usb-serial-jtag",
		    esp32s3_usb_earlycon_setup);
