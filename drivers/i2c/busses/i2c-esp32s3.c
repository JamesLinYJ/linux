// SPDX-License-Identifier: GPL-2.0-only
/* Espressif ESP32-S3 I2C controller driver */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/reset.h>

#define ESP32S3_I2C_SCL_LOW_PERIOD	0x00
#define ESP32S3_I2C_CTR			0x04
#define ESP32S3_I2C_SR			0x08
#define ESP32S3_I2C_TO			0x0c
#define ESP32S3_I2C_FIFO_CONF		0x18
#define ESP32S3_I2C_DATA			0x1c
#define ESP32S3_I2C_INT_CLR		0x24
#define ESP32S3_I2C_INT_ENA		0x28
#define ESP32S3_I2C_INT_STATUS		0x2c
#define ESP32S3_I2C_SDA_HOLD		0x30
#define ESP32S3_I2C_SDA_SAMPLE		0x34
#define ESP32S3_I2C_SCL_HIGH_PERIOD	0x38
#define ESP32S3_I2C_SCL_START_HOLD	0x40
#define ESP32S3_I2C_SCL_RSTART_SETUP	0x44
#define ESP32S3_I2C_SCL_STOP_HOLD	0x48
#define ESP32S3_I2C_SCL_STOP_SETUP	0x4c
#define ESP32S3_I2C_FILTER_CFG		0x50
#define ESP32S3_I2C_CLK_CONF		0x54
#define ESP32S3_I2C_COMMAND(n)		(0x58 + (n) * 4)

#define ESP32S3_I2C_CTR_SDA_FORCE_OUT	BIT(0)
#define ESP32S3_I2C_CTR_SCL_FORCE_OUT	BIT(1)
#define ESP32S3_I2C_CTR_MS_MODE		BIT(4)
#define ESP32S3_I2C_CTR_TRANS_START	BIT(5)
#define ESP32S3_I2C_CTR_CLK_EN		BIT(8)
#define ESP32S3_I2C_CTR_ARBITRATION_EN	BIT(9)
#define ESP32S3_I2C_CTR_FSM_RST		BIT(10)
#define ESP32S3_I2C_CTR_CONF_UPGATE	BIT(11)

#define ESP32S3_I2C_SR_RXFIFO_CNT	GENMASK(13, 8)

#define ESP32S3_I2C_TO_VALUE		GENMASK(4, 0)
#define ESP32S3_I2C_TO_EN		BIT(5)

#define ESP32S3_I2C_FIFO_RX_RST		BIT(12)
#define ESP32S3_I2C_FIFO_TX_RST		BIT(13)
#define ESP32S3_I2C_FIFO_PRT_EN		BIT(14)

#define ESP32S3_I2C_INT_RXFIFO_OVF	BIT(2)
#define ESP32S3_I2C_INT_END_DETECT	BIT(3)
#define ESP32S3_I2C_INT_ARB_LOST	BIT(5)
#define ESP32S3_I2C_INT_TXFIFO_UDF	BIT(6)
#define ESP32S3_I2C_INT_COMPLETE	BIT(7)
#define ESP32S3_I2C_INT_TIMEOUT		BIT(8)
#define ESP32S3_I2C_INT_NACK		BIT(10)
#define ESP32S3_I2C_INT_TXFIFO_OVF	BIT(11)
#define ESP32S3_I2C_INT_RXFIFO_UDF	BIT(12)
#define ESP32S3_I2C_INT_SCL_TIMEOUT	(BIT(13) | BIT(14))
#define ESP32S3_I2C_INT_ERRORS		(ESP32S3_I2C_INT_RXFIFO_OVF | \
					 ESP32S3_I2C_INT_ARB_LOST | \
					 ESP32S3_I2C_INT_TXFIFO_UDF | \
					 ESP32S3_I2C_INT_TIMEOUT | \
					 ESP32S3_I2C_INT_NACK | \
					 ESP32S3_I2C_INT_TXFIFO_OVF | \
					 ESP32S3_I2C_INT_RXFIFO_UDF | \
					 ESP32S3_I2C_INT_SCL_TIMEOUT)
#define ESP32S3_I2C_INT_MASK		(ESP32S3_I2C_INT_ERRORS | \
					 ESP32S3_I2C_INT_END_DETECT | \
					 ESP32S3_I2C_INT_COMPLETE)

#define ESP32S3_I2C_CMD_ACK_EN		BIT(8)
#define ESP32S3_I2C_CMD_ACK_VAL		BIT(10)
#define ESP32S3_I2C_CMD_OPCODE		GENMASK(13, 11)

#define ESP32S3_I2C_CMD_WRITE		1
#define ESP32S3_I2C_CMD_STOP		2
#define ESP32S3_I2C_CMD_READ		3
#define ESP32S3_I2C_CMD_END		4
#define ESP32S3_I2C_CMD_RESTART		6

#define ESP32S3_I2C_FIFO_SIZE		32
#define ESP32S3_I2C_COMMANDS		8

struct esp32s3_i2c {
	void __iomem *base;
	struct clk *clk;
	struct completion complete;
	struct i2c_adapter adapter;
	int irq;
	int result;
};

static u32 esp32s3_i2c_command(u8 opcode, u8 length, bool ack_en,
			       bool ack_val)
{
	u32 command = FIELD_PREP(ESP32S3_I2C_CMD_OPCODE, opcode) | length;

	if (ack_en)
		command |= ESP32S3_I2C_CMD_ACK_EN;
	if (ack_val)
		command |= ESP32S3_I2C_CMD_ACK_VAL;

	return command;
}

static u32 esp32s3_i2c_simple_command(u8 opcode)
{
	return esp32s3_i2c_command(opcode, 0, false, false);
}

static u32 esp32s3_i2c_write_command(u8 length)
{
	return esp32s3_i2c_command(ESP32S3_I2C_CMD_WRITE, length, true, false);
}

static u32 esp32s3_i2c_read_command(u8 length, bool nack)
{
	return esp32s3_i2c_command(ESP32S3_I2C_CMD_READ, length, false, nack);
}

static void esp32s3_i2c_reset_fifos(struct esp32s3_i2c *i2c)
{
	u32 value = readl(i2c->base + ESP32S3_I2C_FIFO_CONF);

	value |= ESP32S3_I2C_FIFO_RX_RST | ESP32S3_I2C_FIFO_TX_RST;
	writel(value, i2c->base + ESP32S3_I2C_FIFO_CONF);
	value &= ~(ESP32S3_I2C_FIFO_RX_RST | ESP32S3_I2C_FIFO_TX_RST);
	value |= ESP32S3_I2C_FIFO_PRT_EN;
	writel(value, i2c->base + ESP32S3_I2C_FIFO_CONF);
}

static void esp32s3_i2c_reset_fsm(struct esp32s3_i2c *i2c)
{
	u32 value = readl(i2c->base + ESP32S3_I2C_CTR);

	writel(value | ESP32S3_I2C_CTR_FSM_RST,
	       i2c->base + ESP32S3_I2C_CTR);
}

static irqreturn_t esp32s3_i2c_irq(int irq, void *data)
{
	struct esp32s3_i2c *i2c = data;
	u32 status = readl(i2c->base + ESP32S3_I2C_INT_STATUS);

	if (!status)
		return IRQ_NONE;

	writel(status, i2c->base + ESP32S3_I2C_INT_CLR);

	if (status & ESP32S3_I2C_INT_NACK)
		i2c->result = -ENXIO;
	else if (status & ESP32S3_I2C_INT_ARB_LOST)
		i2c->result = -EAGAIN;
	else if (status & (ESP32S3_I2C_INT_TIMEOUT |
			   ESP32S3_I2C_INT_SCL_TIMEOUT))
		i2c->result = -ETIMEDOUT;
	else if (status & ESP32S3_I2C_INT_ERRORS)
		i2c->result = -EIO;
	else if (!(status & (ESP32S3_I2C_INT_END_DETECT |
			    ESP32S3_I2C_INT_COMPLETE)))
		return IRQ_HANDLED;

	writel(0, i2c->base + ESP32S3_I2C_INT_ENA);
	complete(&i2c->complete);

	return IRQ_HANDLED;
}

static int esp32s3_i2c_execute(struct esp32s3_i2c *i2c,
			       const u32 *commands, unsigned int count)
{
	unsigned long timeout;
	unsigned int i;
	u32 value;

	for (i = 0; i < ESP32S3_I2C_COMMANDS; i++)
		writel(esp32s3_i2c_simple_command(ESP32S3_I2C_CMD_END),
		       i2c->base + ESP32S3_I2C_COMMAND(i));
	for (i = 0; i < count; i++)
		writel(commands[i], i2c->base + ESP32S3_I2C_COMMAND(i));

	reinit_completion(&i2c->complete);
	i2c->result = 0;
	writel(ESP32S3_I2C_INT_MASK, i2c->base + ESP32S3_I2C_INT_CLR);
	writel(ESP32S3_I2C_INT_MASK, i2c->base + ESP32S3_I2C_INT_ENA);

	value = readl(i2c->base + ESP32S3_I2C_CTR);
	value |= ESP32S3_I2C_CTR_CONF_UPGATE;
	writel(value, i2c->base + ESP32S3_I2C_CTR);
	writel(value | ESP32S3_I2C_CTR_TRANS_START,
	       i2c->base + ESP32S3_I2C_CTR);

	timeout = wait_for_completion_timeout(&i2c->complete,
					      i2c->adapter.timeout);
	if (!timeout) {
		writel(0, i2c->base + ESP32S3_I2C_INT_ENA);
		synchronize_irq(i2c->irq);
		esp32s3_i2c_reset_fsm(i2c);
		return -ETIMEDOUT;
	}

	if (i2c->result)
		esp32s3_i2c_reset_fsm(i2c);

	return i2c->result;
}

static void esp32s3_i2c_write_fifo(struct esp32s3_i2c *i2c, const u8 *buffer,
				   size_t length)
{
	while (length--) {
		writel(*buffer, i2c->base + ESP32S3_I2C_DATA);
		buffer++;
	}
}

static int esp32s3_i2c_read_fifo(struct esp32s3_i2c *i2c, u8 *buffer,
				 size_t length)
{
	u32 count = FIELD_GET(ESP32S3_I2C_SR_RXFIFO_CNT,
			      readl(i2c->base + ESP32S3_I2C_SR));

	if (count < length)
		return -EIO;

	while (length--) {
		*buffer = readl(i2c->base + ESP32S3_I2C_DATA);
		buffer++;
	}

	return 0;
}

static int esp32s3_i2c_write_msg(struct esp32s3_i2c *i2c,
				 struct i2c_msg *msg, bool last_msg)
{
	const u8 *buffer = msg->buf;
	size_t remaining = msg->len;
	bool first = true;

	do {
		u32 commands[4];
		unsigned int count = 0;
		size_t limit = first ? ESP32S3_I2C_FIFO_SIZE - 1 :
				       ESP32S3_I2C_FIFO_SIZE;
		size_t length = min(remaining, limit);
		bool last_chunk = length == remaining;
		u8 terminal = last_chunk && last_msg ? ESP32S3_I2C_CMD_STOP :
						    ESP32S3_I2C_CMD_END;
		int ret;

		if (first) {
			u8 address = i2c_8bit_addr_from_msg(msg);

			commands[count++] =
				esp32s3_i2c_simple_command(ESP32S3_I2C_CMD_RESTART);
			writel(address, i2c->base + ESP32S3_I2C_DATA);
		}
		esp32s3_i2c_write_fifo(i2c, buffer, length);
		commands[count++] = esp32s3_i2c_write_command(length + first);
		commands[count++] = esp32s3_i2c_simple_command(terminal);

		ret = esp32s3_i2c_execute(i2c, commands, count);
		if (ret)
			return ret;

		buffer += length;
		remaining -= length;
		first = false;
	} while (remaining);

	return 0;
}

static int esp32s3_i2c_read_msg(struct esp32s3_i2c *i2c,
				struct i2c_msg *msg, bool last_msg)
{
	u8 *buffer = msg->buf;
	size_t remaining = msg->len;
	bool first = true;

	do {
		u32 commands[5];
		unsigned int count = 0;
		size_t length = min_t(size_t, remaining,
				      ESP32S3_I2C_FIFO_SIZE);
		bool last_chunk = length == remaining;
		u8 terminal = last_chunk && last_msg ? ESP32S3_I2C_CMD_STOP :
						    ESP32S3_I2C_CMD_END;
		int ret;

		if (first) {
			u8 address = i2c_8bit_addr_from_msg(msg);

			commands[count++] =
				esp32s3_i2c_simple_command(ESP32S3_I2C_CMD_RESTART);
			writel(address, i2c->base + ESP32S3_I2C_DATA);
			commands[count++] = esp32s3_i2c_write_command(1);
		}

		if (length && last_chunk) {
			if (length > 1)
				commands[count++] =
					esp32s3_i2c_read_command(length - 1, false);
			commands[count++] = esp32s3_i2c_read_command(1, true);
		} else if (length) {
			commands[count++] = esp32s3_i2c_read_command(length, false);
		}

		commands[count++] = esp32s3_i2c_simple_command(terminal);
		ret = esp32s3_i2c_execute(i2c, commands, count);
		if (ret)
			return ret;
		ret = esp32s3_i2c_read_fifo(i2c, buffer, length);
		if (ret)
			return ret;

		buffer += length;
		remaining -= length;
		first = false;
	} while (remaining);

	return 0;
}

static int esp32s3_i2c_master_xfer(struct i2c_adapter *adapter,
				   struct i2c_msg *messages, int count)
{
	struct esp32s3_i2c *i2c = i2c_get_adapdata(adapter);
	int i;

	esp32s3_i2c_reset_fifos(i2c);
	for (i = 0; i < count; i++) {
		struct i2c_msg *msg = &messages[i];
		int ret;

		if (msg->flags & ~(I2C_M_RD | I2C_M_DMA_SAFE))
			return -EOPNOTSUPP;

		if (msg->flags & I2C_M_RD)
			ret = esp32s3_i2c_read_msg(i2c, msg, i == count - 1);
		else
			ret = esp32s3_i2c_write_msg(i2c, msg, i == count - 1);
		if (ret)
			return ret;
	}

	return count;
}

static u32 esp32s3_i2c_functionality(struct i2c_adapter *adapter)
{
	return I2C_FUNC_I2C;
}

static const struct i2c_algorithm esp32s3_i2c_algorithm = {
	.master_xfer = esp32s3_i2c_master_xfer,
	.functionality = esp32s3_i2c_functionality,
};

static int esp32s3_i2c_init_hardware(struct esp32s3_i2c *i2c,
				     u32 bus_frequency)
{
	unsigned long source_frequency = clk_get_rate(i2c->clk);
	u32 clock_divider;
	u32 half_cycle;
	u32 wait_high;
	u32 high;
	u32 timeout;
	u32 value;

	if (!source_frequency || bus_frequency < 10000 ||
	    bus_frequency > 1000000)
		return -EINVAL;

	clock_divider = source_frequency / (bus_frequency * 1024UL) + 1;
	half_cycle = source_frequency / clock_divider / bus_frequency / 2;
	if (clock_divider > 256 || half_cycle < 8 || half_cycle > 511)
		return -EINVAL;

	wait_high = bus_frequency >= 80000 ? half_cycle / 2 - 2 :
					       half_cycle / 4;
	high = half_cycle - wait_high;
	if (wait_high > 127 || high > 511)
		return -EINVAL;

	timeout = min_t(u32, fls(5 * half_cycle) + 2, 31);
	writel(half_cycle - 1, i2c->base + ESP32S3_I2C_SCL_LOW_PERIOD);
	writel(high | (wait_high << 9),
	       i2c->base + ESP32S3_I2C_SCL_HIGH_PERIOD);
	writel(half_cycle / 4 - 1, i2c->base + ESP32S3_I2C_SDA_HOLD);
	writel(half_cycle / 2 - 1, i2c->base + ESP32S3_I2C_SDA_SAMPLE);
	writel(half_cycle - 1, i2c->base + ESP32S3_I2C_SCL_RSTART_SETUP);
	writel(half_cycle - 1, i2c->base + ESP32S3_I2C_SCL_STOP_SETUP);
	writel(half_cycle - 1, i2c->base + ESP32S3_I2C_SCL_START_HOLD);
	writel(half_cycle - 1, i2c->base + ESP32S3_I2C_SCL_STOP_HOLD);
	writel(FIELD_PREP(ESP32S3_I2C_TO_VALUE, timeout) |
	       ESP32S3_I2C_TO_EN, i2c->base + ESP32S3_I2C_TO);
	writel((clock_divider - 1) | BIT(21),
	       i2c->base + ESP32S3_I2C_CLK_CONF);
	writel(7 | (7 << 4) | BIT(8) | BIT(9),
	       i2c->base + ESP32S3_I2C_FILTER_CFG);

	value = ESP32S3_I2C_CTR_SDA_FORCE_OUT |
		ESP32S3_I2C_CTR_SCL_FORCE_OUT |
		ESP32S3_I2C_CTR_MS_MODE |
		ESP32S3_I2C_CTR_CLK_EN |
		ESP32S3_I2C_CTR_ARBITRATION_EN;
	writel(value | ESP32S3_I2C_CTR_CONF_UPGATE,
	       i2c->base + ESP32S3_I2C_CTR);
	esp32s3_i2c_reset_fifos(i2c);
	writel(ESP32S3_I2C_INT_MASK, i2c->base + ESP32S3_I2C_INT_CLR);

	return 0;
}

static int esp32s3_i2c_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct i2c_timings timings;
	struct reset_control *reset;
	struct esp32s3_i2c *i2c;
	int ret;

	i2c = devm_kzalloc(dev, sizeof(*i2c), GFP_KERNEL);
	if (!i2c)
		return -ENOMEM;
	i2c->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(i2c->base))
		return PTR_ERR(i2c->base);

	i2c->clk = devm_clk_get_enabled(dev, NULL);
	if (IS_ERR(i2c->clk))
		return dev_err_probe(dev, PTR_ERR(i2c->clk),
				     "failed to enable clock\n");
	reset = devm_reset_control_get_exclusive(dev, NULL);
	if (IS_ERR(reset))
		return dev_err_probe(dev, PTR_ERR(reset),
				     "failed to get reset\n");
	ret = reset_control_reset(reset);
	if (ret)
		return dev_err_probe(dev, ret, "failed to reset controller\n");

	memset(&timings, 0, sizeof(timings));
	i2c_parse_fw_timings(dev, &timings, true);
	ret = esp32s3_i2c_init_hardware(i2c, timings.bus_freq_hz);
	if (ret)
		return dev_err_probe(dev, ret, "invalid bus timing\n");

	init_completion(&i2c->complete);
	i2c->irq = platform_get_irq(pdev, 0);
	if (i2c->irq < 0)
		return i2c->irq;
	ret = devm_request_irq(dev, i2c->irq, esp32s3_i2c_irq, 0,
			       dev_name(dev), i2c);
	if (ret)
		return dev_err_probe(dev, ret, "failed to request interrupt\n");

	i2c->adapter.owner = THIS_MODULE;
	i2c->adapter.algo = &esp32s3_i2c_algorithm;
	i2c->adapter.dev.parent = dev;
	i2c->adapter.dev.of_node = dev->of_node;
	i2c->adapter.timeout = HZ;
	strscpy(i2c->adapter.name, "ESP32-S3 I2C adapter",
		sizeof(i2c->adapter.name));
	i2c_set_adapdata(&i2c->adapter, i2c);
	platform_set_drvdata(pdev, i2c);

	ret = devm_i2c_add_adapter(dev, &i2c->adapter);
	if (ret)
		return dev_err_probe(dev, ret, "failed to add adapter\n");

	return 0;
}

static const struct of_device_id esp32s3_i2c_of_match[] = {
	{ .compatible = "esp,esp32s3-i2c" },
	{ }
};
MODULE_DEVICE_TABLE(of, esp32s3_i2c_of_match);

static struct platform_driver esp32s3_i2c_driver = {
	.probe = esp32s3_i2c_probe,
	.driver = {
		.name = "esp32s3-i2c",
		.of_match_table = esp32s3_i2c_of_match,
	},
};
module_platform_driver(esp32s3_i2c_driver);

MODULE_DESCRIPTION("Espressif ESP32-S3 I2C controller driver");
MODULE_LICENSE("GPL");
