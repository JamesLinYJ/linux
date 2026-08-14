// SPDX-License-Identifier: GPL-2.0-only
/*
 * ESP32-S3 USB Serial/JTAG runtime serial driver
 *
 * The register layout and interrupt definitions come from Espressif's
 * official ESP-IDF v5.5.3 register headers. This driver only owns the serial
 * endpoint. It deliberately leaves USB PHY selection and JTAG routing alone.
 */

#include <linux/console.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/serial.h>
#include <linux/serial_core.h>
#include <linux/tty_flip.h>

#include "esp32s3-usb-serial.h"

#define ESP32S3_USB_SERIAL_NR_PORTS	1
#define ESP32S3_USB_SERIAL_BAUD		115200

struct esp32s3_usb_serial_port {
	struct uart_port port;
	u32 irq_mask;
	bool need_zlp;
};

static inline struct esp32s3_usb_serial_port *
to_esp32s3_usb_serial(struct uart_port *port)
{
	return container_of(port, struct esp32s3_usb_serial_port, port);
}

static struct esp32s3_usb_serial_port *esp32s3_usb_ports[ESP32S3_USB_SERIAL_NR_PORTS];

#ifdef CONFIG_SERIAL_ESP32S3_USB_CONSOLE
static struct console esp32s3_usb_serial_console;
#endif

static struct uart_driver esp32s3_usb_serial_driver = {
	.owner		= THIS_MODULE,
	.driver_name	= "esp32s3-usb-serial",
	.dev_name	= "ttyESJ",
	.major		= 0,
	.minor		= 0,
	.nr		= ESP32S3_USB_SERIAL_NR_PORTS,
#ifdef CONFIG_SERIAL_ESP32S3_USB_CONSOLE
	.cons		= &esp32s3_usb_serial_console,
#endif
};

static bool esp32s3_usb_serial_tx_ready(struct uart_port *port)
{
	return readl_relaxed(port->membase + ESP32S3_USB_SERIAL_EP1_CONF) &
		ESP32S3_USB_SERIAL_TX_FREE;
}

/* port->lock must be held. */
static void esp32s3_usb_update_irq(struct uart_port *port, bool enable,
				   u32 mask)
{
	struct esp32s3_usb_serial_port *uart =
		to_esp32s3_usb_serial(port);

	if (enable)
		uart->irq_mask |= mask;
	else
		uart->irq_mask &= ~mask;

	writel(uart->irq_mask,
	       port->membase + ESP32S3_USB_SERIAL_INT_ENA);
	/* Complete the interrupt-mask update before dropping port->lock. */
	readl(port->membase + ESP32S3_USB_SERIAL_INT_ENA);
}

static void esp32s3_usb_serial_stop_tx(struct uart_port *port)
{
	esp32s3_usb_update_irq(port, false, ESP32S3_USB_SERIAL_INT_TX);
}

/* port->lock must be held. */
static void esp32s3_usb_serial_tx_chars(struct uart_port *port)
{
	struct esp32s3_usb_serial_port *uart =
		to_esp32s3_usb_serial(port);
	unsigned int before = port->icount.tx;
	unsigned int pending;
	unsigned int written;
	u8 ch;

	/*
	 * A full 64-byte USB transaction needs a later zero-length packet when
	 * no following data exists. Wait for the endpoint to become writable
	 * before issuing that termination packet.
	 */
	if (uart->need_zlp) {
		if (!esp32s3_usb_serial_tx_ready(port)) {
			esp32s3_usb_update_irq(port, true, ESP32S3_USB_SERIAL_INT_TX);
			return;
		}
		writel(ESP32S3_USB_SERIAL_WR_DONE,
		       port->membase + ESP32S3_USB_SERIAL_EP1_CONF);
		uart->need_zlp = false;
	}

	pending = uart_port_tx_limited(port, ch, ESP32S3_USB_SERIAL_FIFO_SIZE,
				       esp32s3_usb_serial_tx_ready(port),
				       writel(ch, port->membase +
					      ESP32S3_USB_SERIAL_EP1),
				       ({}));
	written = port->icount.tx - before;

	if (written) {
		writel(ESP32S3_USB_SERIAL_WR_DONE,
		       port->membase + ESP32S3_USB_SERIAL_EP1_CONF);
		uart->need_zlp = written == ESP32S3_USB_SERIAL_FIFO_SIZE &&
				 !pending && !port->x_char;
	}

	if (pending || port->x_char || uart->need_zlp)
		esp32s3_usb_update_irq(port, true, ESP32S3_USB_SERIAL_INT_TX);
	else
		esp32s3_usb_update_irq(port, false, ESP32S3_USB_SERIAL_INT_TX);
}

static void esp32s3_usb_serial_start_tx(struct uart_port *port)
{
	esp32s3_usb_serial_tx_chars(port);
}

static void esp32s3_usb_serial_stop_rx(struct uart_port *port)
{
	esp32s3_usb_update_irq(port, false, ESP32S3_USB_SERIAL_INT_RX);
}

/* port->lock must be held. */
static void esp32s3_usb_serial_rx_chars(struct uart_port *port)
{
	unsigned int count;
	u8 ch;

	for (count = 0; count < ESP32S3_USB_SERIAL_FIFO_SIZE; count++) {
		if (!(readl_relaxed(port->membase +
				    ESP32S3_USB_SERIAL_EP1_CONF) &
		      ESP32S3_USB_SERIAL_RX_AVAIL))
			break;

		ch = readl_relaxed(port->membase + ESP32S3_USB_SERIAL_EP1);
		port->icount.rx++;
		if (!uart_handle_sysrq_char(port, ch))
			uart_insert_char(port, 0, 0, ch, TTY_NORMAL);
	}

	tty_flip_buffer_push(&port->state->port);
}

static irqreturn_t esp32s3_usb_serial_interrupt(int irq, void *data)
{
	struct esp32s3_usb_serial_port *uart = data;
	struct uart_port *port = &uart->port;
	u32 status;

	uart_port_lock(port);
	status = readl(port->membase + ESP32S3_USB_SERIAL_INT_ST) &
		uart->irq_mask;
	if (!status) {
		uart_port_unlock(port);
		return IRQ_NONE;
	}

	/* Ack first so a packet arriving while it is drained can retrigger. */
	writel(status, port->membase + ESP32S3_USB_SERIAL_INT_CLR);
	if (status & ESP32S3_USB_SERIAL_INT_RX)
		esp32s3_usb_serial_rx_chars(port);
	if (status & ESP32S3_USB_SERIAL_INT_TX)
		esp32s3_usb_serial_tx_chars(port);
	uart_port_unlock(port);

	return IRQ_HANDLED;
}

static unsigned int esp32s3_usb_serial_tx_empty(struct uart_port *port)
{
	struct esp32s3_usb_serial_port *uart =
		to_esp32s3_usb_serial(port);

	if (esp32s3_usb_serial_tx_ready(port) && !uart->need_zlp)
		return TIOCSER_TEMT;

	return 0;
}

static void esp32s3_usb_serial_set_mctrl(struct uart_port *port,
					 unsigned int mctrl)
{
}

static unsigned int esp32s3_usb_serial_get_mctrl(struct uart_port *port)
{
	return TIOCM_CTS | TIOCM_DSR | TIOCM_CAR;
}

static int esp32s3_usb_serial_startup(struct uart_port *port)
{
	struct esp32s3_usb_serial_port *uart =
		to_esp32s3_usb_serial(port);
	unsigned long flags;
	int ret;

	ret = request_irq(port->irq, esp32s3_usb_serial_interrupt, 0,
			  dev_name(port->dev), uart);
	if (ret)
		return ret;

	uart_port_lock_irqsave(port, &flags);
	uart->irq_mask = 0;
	uart->need_zlp = false;
	writel(0, port->membase + ESP32S3_USB_SERIAL_INT_ENA);
	writel(ESP32S3_USB_SERIAL_INT_TX,
	       port->membase + ESP32S3_USB_SERIAL_INT_CLR);
	esp32s3_usb_update_irq(port, true, ESP32S3_USB_SERIAL_INT_RX);
	uart_port_unlock_irqrestore(port, flags);

	return 0;
}

static void esp32s3_usb_serial_shutdown(struct uart_port *port)
{
	struct esp32s3_usb_serial_port *uart =
		to_esp32s3_usb_serial(port);
	unsigned long flags;

	uart_port_lock_irqsave(port, &flags);
	uart->irq_mask = 0;
	uart->need_zlp = false;
	writel(0, port->membase + ESP32S3_USB_SERIAL_INT_ENA);
	writel(ESP32S3_USB_SERIAL_INT_MASK,
	       port->membase + ESP32S3_USB_SERIAL_INT_CLR);
	readl(port->membase + ESP32S3_USB_SERIAL_INT_ENA);
	uart_port_unlock_irqrestore(port, flags);

	free_irq(port->irq, uart);
}

static void esp32s3_usb_serial_set_termios(struct uart_port *port,
					   struct ktermios *termios,
					   const struct ktermios *old)
{
	unsigned long flags;

	termios->c_cflag &= ~(CSIZE | CSTOPB | PARENB | CRTSCTS);
	termios->c_cflag |= CS8 | CREAD | CLOCAL;
	tty_termios_encode_baud_rate(termios, ESP32S3_USB_SERIAL_BAUD,
				     ESP32S3_USB_SERIAL_BAUD);

	uart_port_lock_irqsave(port, &flags);
	uart_update_timeout(port, termios->c_cflag,
			    ESP32S3_USB_SERIAL_BAUD);
	uart_port_unlock_irqrestore(port, flags);
}

static const char *esp32s3_usb_serial_type(struct uart_port *port)
{
	return port->type == PORT_GENERIC ? "ESP32-S3 USB Serial/JTAG" : NULL;
}

static void esp32s3_usb_serial_config_port(struct uart_port *port, int flags)
{
	if (flags & UART_CONFIG_TYPE)
		port->type = PORT_GENERIC;
}

static int esp32s3_usb_serial_verify_port(struct uart_port *port,
					  struct serial_struct *ser)
{
	if (ser->type != PORT_UNKNOWN && ser->type != PORT_GENERIC)
		return -EINVAL;

	return 0;
}

static const struct uart_ops esp32s3_usb_serial_ops = {
	.tx_empty	= esp32s3_usb_serial_tx_empty,
	.set_mctrl	= esp32s3_usb_serial_set_mctrl,
	.get_mctrl	= esp32s3_usb_serial_get_mctrl,
	.stop_tx	= esp32s3_usb_serial_stop_tx,
	.start_tx	= esp32s3_usb_serial_start_tx,
	.stop_rx	= esp32s3_usb_serial_stop_rx,
	.startup	= esp32s3_usb_serial_startup,
	.shutdown	= esp32s3_usb_serial_shutdown,
	.set_termios	= esp32s3_usb_serial_set_termios,
	.type		= esp32s3_usb_serial_type,
	.config_port	= esp32s3_usb_serial_config_port,
	.verify_port	= esp32s3_usb_serial_verify_port,
};

static int esp32s3_usb_serial_probe(struct platform_device *pdev)
{
	struct esp32s3_usb_serial_port *uart;
	struct resource *resource;
	struct uart_port *port;
	int alias;
	int ret;

	uart = devm_kzalloc(&pdev->dev, sizeof(*uart), GFP_KERNEL);
	if (!uart)
		return -ENOMEM;

	port = &uart->port;
	port->membase = devm_platform_get_and_ioremap_resource(pdev, 0, &resource);
	if (IS_ERR(port->membase))
		return PTR_ERR(port->membase);

	ret = platform_get_irq(pdev, 0);
	if (ret < 0)
		return ret;
	port->irq = ret;

	alias = of_alias_get_id(pdev->dev.of_node, "serial");
	if (alias < 0)
		alias = 0;
	if (alias >= ESP32S3_USB_SERIAL_NR_PORTS)
		return dev_err_probe(&pdev->dev, -EINVAL,
				     "serial alias %d is out of range\n", alias);

	port->dev = &pdev->dev;
	port->mapbase = resource->start;
	port->iotype = UPIO_MEM;
	port->flags = UPF_BOOT_AUTOCONF;
	port->ops = &esp32s3_usb_serial_ops;
	port->fifosize = ESP32S3_USB_SERIAL_FIFO_SIZE;
	port->type = PORT_UNKNOWN;
	port->line = alias;
	spin_lock_init(&port->lock);

	platform_set_drvdata(pdev, uart);
	WRITE_ONCE(esp32s3_usb_ports[alias], uart);
	ret = uart_add_one_port(&esp32s3_usb_serial_driver, port);
	if (ret)
		WRITE_ONCE(esp32s3_usb_ports[alias], NULL);

	return ret;
}

static void esp32s3_usb_serial_remove(struct platform_device *pdev)
{
	struct esp32s3_usb_serial_port *uart = platform_get_drvdata(pdev);
	unsigned int line = uart->port.line;

	uart_remove_one_port(&esp32s3_usb_serial_driver, &uart->port);
	WRITE_ONCE(esp32s3_usb_ports[line], NULL);
}

static const struct of_device_id esp32s3_usb_serial_of_match[] = {
	{ .compatible = "esp,esp32s3-usb-serial-jtag" },
	{}
};
MODULE_DEVICE_TABLE(of, esp32s3_usb_serial_of_match);

static struct platform_driver esp32s3_usb_serial_platform_driver = {
	.probe = esp32s3_usb_serial_probe,
	.remove = esp32s3_usb_serial_remove,
	.driver = {
		.name = "esp32s3-usb-serial",
		.of_match_table = esp32s3_usb_serial_of_match,
	},
};

#ifdef CONFIG_SERIAL_ESP32S3_USB_CONSOLE
static bool esp32s3_usb_serial_console_putc(struct uart_port *port, u8 ch)
{
	unsigned int retry;

	for (retry = 0; retry < ESP32S3_USB_SERIAL_POLL_LIMIT; retry++) {
		if (esp32s3_usb_serial_tx_ready(port)) {
			writel(ch, port->membase + ESP32S3_USB_SERIAL_EP1);
			return true;
		}
		cpu_relax();
	}

	return false;
}

static void esp32s3_usb_console_write(struct console *console,
				      const char *string, unsigned int count)
{
	struct esp32s3_usb_serial_port *uart;
	struct uart_port *port;
	unsigned long flags;
	unsigned int bytes = 0;
	unsigned int index;
	unsigned int retry;

	if (console->index < 0 ||
	    console->index >= ESP32S3_USB_SERIAL_NR_PORTS)
		return;
	uart = READ_ONCE(esp32s3_usb_ports[console->index]);
	if (!uart)
		return;
	port = &uart->port;

	uart_port_lock_irqsave(port, &flags);
	for (index = 0; index < count; index++) {
		if (string[index] == '\n') {
			if (!esp32s3_usb_serial_console_putc(port, '\r'))
				break;
			bytes++;
		}
		if (!esp32s3_usb_serial_console_putc(port, string[index]))
			break;
		bytes++;
	}
	if (bytes) {
		writel(ESP32S3_USB_SERIAL_WR_DONE,
		       port->membase + ESP32S3_USB_SERIAL_EP1_CONF);
		if (!(bytes % ESP32S3_USB_SERIAL_FIFO_SIZE)) {
			for (retry = 0; retry < ESP32S3_USB_SERIAL_POLL_LIMIT;
			     retry++) {
				if (esp32s3_usb_serial_tx_ready(port)) {
					writel(ESP32S3_USB_SERIAL_WR_DONE,
					       port->membase +
					       ESP32S3_USB_SERIAL_EP1_CONF);
					break;
				}
				cpu_relax();
			}
		}
	}
	uart_port_unlock_irqrestore(port, flags);
}

static int esp32s3_usb_console_setup(struct console *console, char *options)
{
	struct esp32s3_usb_serial_port *uart;
	int baud = ESP32S3_USB_SERIAL_BAUD;
	int parity = 'n';
	int bits = 8;
	int flow = 'n';

	if (console->index < 0)
		console->index = 0;
	if (console->index >= ESP32S3_USB_SERIAL_NR_PORTS)
		return -EINVAL;
	uart = READ_ONCE(esp32s3_usb_ports[console->index]);
	if (!uart || !uart->port.membase)
		return -ENODEV;

	if (options)
		uart_parse_options(options, &baud, &parity, &bits, &flow);

	return uart_set_options(&uart->port, console, baud, parity, bits, flow);
}

static struct console esp32s3_usb_serial_console = {
	.name	= "ttyESJ",
	.write	= esp32s3_usb_console_write,
	.device	= uart_console_device,
	.setup	= esp32s3_usb_console_setup,
	.flags	= CON_PRINTBUFFER,
	.index	= -1,
	.data	= &esp32s3_usb_serial_driver,
};

static int __init esp32s3_usb_serial_console_init(void)
{
	register_console(&esp32s3_usb_serial_console);
	return 0;
}
console_initcall(esp32s3_usb_serial_console_init);
#endif

static int __init esp32s3_usb_serial_init(void)
{
	int ret;

	ret = uart_register_driver(&esp32s3_usb_serial_driver);
	if (ret)
		return ret;

	ret = platform_driver_register(&esp32s3_usb_serial_platform_driver);
	if (ret)
		uart_unregister_driver(&esp32s3_usb_serial_driver);

	return ret;
}
module_init(esp32s3_usb_serial_init);

static void __exit esp32s3_usb_serial_exit(void)
{
	platform_driver_unregister(&esp32s3_usb_serial_platform_driver);
	uart_unregister_driver(&esp32s3_usb_serial_driver);
}
module_exit(esp32s3_usb_serial_exit);

MODULE_AUTHOR("ESP32-S3 Linux project");
MODULE_DESCRIPTION("ESP32-S3 USB Serial/JTAG TTY driver");
MODULE_LICENSE("GPL");
