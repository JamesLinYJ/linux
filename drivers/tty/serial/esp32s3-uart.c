// SPDX-License-Identifier: GPL-2.0-only
/*
 * ESP32-S3 UART controller driver (uart_port API)
 *
 * The Waveshare V2 exposes UART0 on the GPIO43/44 header. Register facts
 * (offsets, status/interrupt bits, CONF0/CONF1 fields, the baud divider
 * encoding) are verified against the locked ESP-IDF v5.5.3 sources
 * (components/soc/esp32s3/register/soc/uart_reg.h,
 * components/hal/esp32s3/include/hal/uart_ll.h) and are cited inline.
 * The mainline binding esp,esp32s3-uart (Documentation/devicetree/
 * bindings/serial/esp,esp32-uart.yaml) has no mainline driver; this file
 * fills that gap. No device has exercised this driver yet.
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/serial_core.h>
#include <linux/tty_flip.h>

#include "esp32s3-uart.h"

#define ESP32S3_UART_FIFO			0x00
#define ESP32S3_UART_INT_ENA			0x0c
#define ESP32S3_UART_INT_CLR			0x10
#define ESP32S3_UART_CLKDIV			0x14
#define ESP32S3_UART_STATUS			0x1c
#define ESP32S3_UART_CONF0			0x20
#define ESP32S3_UART_CONF1			0x24
#define ESP32S3_UART_RXD_CNT			0x30
#define ESP32S3_UART_IDLE_CONF			0x48
#define ESP32S3_UART_CLK_CONF			0x78

/* Interrupt sources (uart_reg.h bit positions) */
#define ESP32S3_UART_INT_RXFIFO_FULL		BIT(0)
#define ESP32S3_UART_INT_TXFIFO_EMPTY		BIT(1)
#define ESP32S3_UART_INT_RXFIFO_TOUT		BIT(8)

/* STATUS register fields */
#define ESP32S3_UART_STATUS_RXFIFO_CNT		GENMASK(15, 8)
#define ESP32S3_UART_STATUS_TXFIFO_CNT		GENMASK(23, 16)

/* CONF0 fields (bit positions per uart_struct.h) */
#define ESP32S3_UART_CONF0_BIT_NUM		GENMASK(3, 2)
#define ESP32S3_UART_CONF0_PARITY_EN		BIT(18)
#define ESP32S3_UART_CONF0_PARITY		BIT(19)
#define ESP32S3_UART_CONF0_STOP_BIT_NUM		GENMASK(6, 4)

/* CONF1 fields */
#define ESP32S3_UART_CONF1_RXFIFO_FULL_THRHD	GENMASK(15, 8)
#define ESP32S3_UART_CONF1_TXFIFO_EMPTY_THRHD	GENMASK(23, 16)

/* CLKDIV fields */
#define ESP32S3_UART_CLKDIV_FRAG		GENMASK(23, 20)
#define ESP32S3_UART_CLKDIV_DIV		GENMASK(19, 0)

#define ESP32S3_UART_FIFO_DEPTH		128

static struct uart_driver esp32s3_uart_driver;

struct esp32s3_uart {
	struct uart_port port;
	struct clk *clk;
};

static inline void esp32s3_uart_write(struct uart_port *port,
				      unsigned int offset, u32 value)
{
	writel(value, port->membase + offset);
}

static inline u32 esp32s3_uart_read(struct uart_port *port,
				    unsigned int offset)
{
	return readl(port->membase + offset);
}

/*
 * Baud divider (official uart_ll encoding): clk_div = (sclk << 4) /
 * (baud * sclk_div); CLKDIV holds the integer part plus a 4-bit fraction.
 */
static int esp32s3_uart_set_baud(struct uart_port *port, unsigned int baud)
{
	u32 clk_div;
	int ret;

	/* sclk_div fixed at 1 (APB 80 MHz source) */
	ret = esp32s3_uart_clk_div(port->uartclk, baud, &clk_div);
	if (ret)
		return ret;

	esp32s3_uart_write(port, ESP32S3_UART_CLKDIV,
			   FIELD_PREP(ESP32S3_UART_CLKDIV_FRAG, clk_div & 0xf) |
			   FIELD_PREP(ESP32S3_UART_CLKDIV_DIV, clk_div >> 4));
	return 0;
}

static void esp32s3_uart_set_termios(struct uart_port *port,
				     struct ktermios *termios,
				     const struct ktermios *old)
{
	u32 conf0 = esp32s3_uart_read(port, ESP32S3_UART_CONF0);
	unsigned int baud;
	u8 bits;

	baud = uart_get_baud_rate(port, termios, old, 110, 4000000);
	esp32s3_uart_set_baud(port, baud);

	switch (termios->c_cflag & CSIZE) {
	case CS5:
		bits = 0;
		break;
	case CS6:
		bits = 1;
		break;
	case CS7:
		bits = 2;
		break;
	default:
		bits = 3;
		break;
	}
	conf0 &= ~ESP32S3_UART_CONF0_BIT_NUM;
	conf0 |= FIELD_PREP(ESP32S3_UART_CONF0_BIT_NUM, bits);

	if (termios->c_cflag & CSTOPB)
		conf0 |= 0x3 << __ffs(ESP32S3_UART_CONF0_STOP_BIT_NUM);
	else
		conf0 |= 0x1 << __ffs(ESP32S3_UART_CONF0_STOP_BIT_NUM);

	if (termios->c_cflag & PARENB) {
		conf0 |= ESP32S3_UART_CONF0_PARITY_EN;
		if (termios->c_cflag & PARODD)
			conf0 |= ESP32S3_UART_CONF0_PARITY;
		else
			conf0 &= ~ESP32S3_UART_CONF0_PARITY;
	} else {
		conf0 &= ~(ESP32S3_UART_CONF0_PARITY_EN |
			   ESP32S3_UART_CONF0_PARITY);
	}

	esp32s3_uart_write(port, ESP32S3_UART_CONF0, conf0);
	uart_update_timeout(port, termios->c_cflag, baud);
}

static void esp32s3_uart_start_tx(struct uart_port *port)
{
	u32 ena;

	ena = esp32s3_uart_read(port, ESP32S3_UART_INT_ENA);
	esp32s3_uart_write(port, ESP32S3_UART_INT_ENA,
			   ena | ESP32S3_UART_INT_TXFIFO_EMPTY);
}

static void esp32s3_uart_stop_tx(struct uart_port *port)
{
	u32 ena;

	ena = esp32s3_uart_read(port, ESP32S3_UART_INT_ENA);
	esp32s3_uart_write(port, ESP32S3_UART_INT_ENA,
			   ena & ~ESP32S3_UART_INT_TXFIFO_EMPTY);
}

static void esp32s3_uart_stop_rx(struct uart_port *port)
{
	u32 ena;

	ena = esp32s3_uart_read(port, ESP32S3_UART_INT_ENA);
	esp32s3_uart_write(port, ESP32S3_UART_INT_ENA,
			   ena & ~(ESP32S3_UART_INT_RXFIFO_FULL |
				   ESP32S3_UART_INT_RXFIFO_TOUT));
}

static unsigned int esp32s3_uart_tx_empty(struct uart_port *port)
{
	u32 status = esp32s3_uart_read(port, ESP32S3_UART_STATUS);

	return FIELD_GET(ESP32S3_UART_STATUS_TXFIFO_CNT, status) ?
	       0 : TIOCSER_TEMT;
}

static void esp32s3_uart_receive(struct uart_port *port)
{
	u32 status = esp32s3_uart_read(port, ESP32S3_UART_STATUS);
	unsigned int count = FIELD_GET(ESP32S3_UART_STATUS_RXFIFO_CNT, status);

	while (count--) {
		u8 ch = readb(port->membase + ESP32S3_UART_FIFO);

		uart_insert_char(port, 0, 0, ch, TTY_NORMAL);
	}

	tty_flip_buffer_push(&port->state->port);
}

static void esp32s3_uart_transmit(struct uart_port *port)
{
	struct tty_port *tport = &port->state->port;
	unsigned int pending = ESP32S3_UART_FIFO_DEPTH / 2;
	u8 ch;

	while (pending-- && !kfifo_is_empty(&tport->xmit_fifo) &&
	       !uart_tx_stopped(port)) {
		if (kfifo_get(&tport->xmit_fifo, &ch) != 1)
			break;
		writeb(ch, port->membase + ESP32S3_UART_FIFO);
	}

	if (kfifo_len(&tport->xmit_fifo) < WAKEUP_CHARS)
		uart_write_wakeup(port);
}

static irqreturn_t esp32s3_uart_irq(int irq, void *dev_id)
{
	struct uart_port *port = dev_id;
	u32 status;

	/* S3 UART shares one interrupt line; read and clear the sources */
	status = esp32s3_uart_read(port, ESP32S3_UART_INT_ENA);
	if (status & (ESP32S3_UART_INT_RXFIFO_FULL |
		      ESP32S3_UART_INT_RXFIFO_TOUT)) {
		esp32s3_uart_write(port, ESP32S3_UART_INT_CLR,
				   ESP32S3_UART_INT_RXFIFO_FULL |
				   ESP32S3_UART_INT_RXFIFO_TOUT);
		esp32s3_uart_receive(port);
	}
	if (status & ESP32S3_UART_INT_TXFIFO_EMPTY) {
		esp32s3_uart_write(port, ESP32S3_UART_INT_CLR,
				   ESP32S3_UART_INT_TXFIFO_EMPTY);
		esp32s3_uart_transmit(port);
	}

	return IRQ_HANDLED;
}

static int esp32s3_uart_startup(struct uart_port *port)
{
	u32 conf1;
	int ret;

	/* RX FIFO full threshold at 120/128, TX empty threshold at 64 */
	conf1 = esp32s3_uart_read(port, ESP32S3_UART_CONF1);
	conf1 &= ~(ESP32S3_UART_CONF1_RXFIFO_FULL_THRHD |
		   ESP32S3_UART_CONF1_TXFIFO_EMPTY_THRHD);
	conf1 |= FIELD_PREP(ESP32S3_UART_CONF1_RXFIFO_FULL_THRHD, 120) |
		 FIELD_PREP(ESP32S3_UART_CONF1_TXFIFO_EMPTY_THRHD, 64);
	esp32s3_uart_write(port, ESP32S3_UART_CONF1, conf1);

	/* RX timeout after 12 idle bit times; enable the timeout source */
	esp32s3_uart_write(port, ESP32S3_UART_IDLE_CONF, 12);
	conf1 = esp32s3_uart_read(port, ESP32S3_UART_CONF1);
	esp32s3_uart_write(port, ESP32S3_UART_CONF1, conf1 | BIT(23) /* RX_TOUT_EN */);

	ret = request_irq(port->irq, esp32s3_uart_irq, IRQF_SHARED,
			  dev_name(port->dev), port);
	if (ret)
		return ret;

	esp32s3_uart_write(port, ESP32S3_UART_INT_ENA,
			   ESP32S3_UART_INT_RXFIFO_FULL |
			   ESP32S3_UART_INT_RXFIFO_TOUT);

	return 0;
}

static void esp32s3_uart_shutdown(struct uart_port *port)
{
	esp32s3_uart_write(port, ESP32S3_UART_INT_ENA, 0);
	free_irq(port->irq, port);
}

static const char *esp32s3_uart_type(struct uart_port *port)
{
	return "esp32s3-uart";
}

/* S3 UART has no modem control lines */
static void esp32s3_uart_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
}

static unsigned int esp32s3_uart_get_mctrl(struct uart_port *port)
{
	return TIOCM_CTS | TIOCM_DSR | TIOCM_CAR;
}

static const struct uart_ops esp32s3_uart_ops = {
	.tx_empty = esp32s3_uart_tx_empty,
	.set_mctrl = esp32s3_uart_set_mctrl,
	.get_mctrl = esp32s3_uart_get_mctrl,
	.stop_tx = esp32s3_uart_stop_tx,
	.start_tx = esp32s3_uart_start_tx,
	.stop_rx = esp32s3_uart_stop_rx,
	.startup = esp32s3_uart_startup,
	.shutdown = esp32s3_uart_shutdown,
	.set_termios = esp32s3_uart_set_termios,
	.type = esp32s3_uart_type,
};

static int esp32s3_uart_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct esp32s3_uart *esp;
	struct uart_port *port;
	unsigned long rate;
	int ret;

	esp = devm_kzalloc(dev, sizeof(*esp), GFP_KERNEL);
	if (!esp)
		return -ENOMEM;

	port = &esp->port;
	port->dev = dev;
	port->membase = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(port->membase))
		return PTR_ERR(port->membase);

	port->irq = platform_get_irq(pdev, 0);
	if (port->irq < 0)
		return port->irq;

	esp->clk = devm_clk_get(dev, NULL);
	if (IS_ERR(esp->clk))
		return dev_err_probe(dev, PTR_ERR(esp->clk),
				     "failed to get UART clock\n");
	ret = clk_prepare_enable(esp->clk);
	if (ret)
		return dev_err_probe(dev, ret, "failed to enable UART clock\n");
	rate = clk_get_rate(esp->clk);
	if (!rate)
		return dev_err_probe(dev, -EINVAL, "invalid UART clock rate\n");

	port->type = PORT_GENERIC;
	port->iotype = UPIO_MEM;
	port->uartclk = rate;
	port->fifosize = ESP32S3_UART_FIFO_DEPTH;
	port->ops = &esp32s3_uart_ops;
	port->line = of_alias_get_id(dev->of_node, "serial");
	if (port->line < 0)
		port->line = 0;

	/* APB clock source with sclk_div = 1 (official default) */
	esp32s3_uart_write(port, ESP32S3_UART_CLK_CONF, 0);

	platform_set_drvdata(pdev, esp);

	return uart_add_one_port(&esp32s3_uart_driver, port);
}

static void esp32s3_uart_remove(struct platform_device *pdev)
{
	struct esp32s3_uart *esp = platform_get_drvdata(pdev);

	uart_remove_one_port(&esp32s3_uart_driver, &esp->port);
	clk_disable_unprepare(esp->clk);
}

static const struct of_device_id esp32s3_uart_of_match[] = {
	{ .compatible = "esp,esp32s3-uart" },
	{ }
};
MODULE_DEVICE_TABLE(of, esp32s3_uart_of_match);

static struct platform_driver esp32s3_uart_platform_driver = {
	.driver = {
		.name = "esp32s3-uart",
		.of_match_table = esp32s3_uart_of_match,
	},
	.probe = esp32s3_uart_probe,
	.remove = esp32s3_uart_remove,
};

static int __init esp32s3_uart_init(void)
{
	int ret;

	esp32s3_uart_driver.owner = THIS_MODULE;
	esp32s3_uart_driver.driver_name = "esp32s3-uart";
	esp32s3_uart_driver.dev_name = "ttyS";
	esp32s3_uart_driver.major = 0;
	esp32s3_uart_driver.minor = 0;
	esp32s3_uart_driver.nr = 1;

	ret = uart_register_driver(&esp32s3_uart_driver);
	if (ret)
		return ret;

	ret = platform_driver_register(&esp32s3_uart_platform_driver);
	if (ret)
		uart_unregister_driver(&esp32s3_uart_driver);

	return ret;
}

static void __exit esp32s3_uart_exit(void)
{
	platform_driver_unregister(&esp32s3_uart_platform_driver);
	uart_unregister_driver(&esp32s3_uart_driver);
}

module_init(esp32s3_uart_init);
module_exit(esp32s3_uart_exit);

MODULE_DESCRIPTION("ESP32-S3 UART driver");
MODULE_AUTHOR("ESP32-S3 Linux project");
MODULE_LICENSE("GPL");
