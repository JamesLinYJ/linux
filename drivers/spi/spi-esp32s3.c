// SPDX-License-Identifier: GPL-2.0-only
/* Espressif ESP32-S3 general-purpose SPI controller driver */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/dmaengine.h>
#include <linux/iopoll.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/reset.h>
#include <linux/spi/spi.h>

#define ESP32S3_SPI_CMD			0x00
#define ESP32S3_SPI_CTRL		0x08
#define ESP32S3_SPI_CLOCK		0x0c
#define ESP32S3_SPI_USER		0x10
#define ESP32S3_SPI_MS_DLEN		0x1c
#define ESP32S3_SPI_MISC		0x20
#define ESP32S3_SPI_DMA_CONF		0x30
#define ESP32S3_SPI_INT_ENA		0x34
#define ESP32S3_SPI_INT_CLR		0x38
#define ESP32S3_SPI_INT_RAW		0x3c
#define ESP32S3_SPI_W(n)		(0x98 + (n) * 4)
#define ESP32S3_SPI_SLAVE		0xe0
#define ESP32S3_SPI_CLK_GATE		0xe8

#define ESP32S3_SPI_CMD_USR		BIT(24)
#define ESP32S3_SPI_CMD_UPDATE		BIT(23)

#define ESP32S3_SPI_CTRL_FREAD_QUAD	BIT(15)

#define ESP32S3_SPI_CLOCK_EQU_SYSCLK	BIT(31)
#define ESP32S3_SPI_CLOCK_PRE		GENMASK(21, 18)
#define ESP32S3_SPI_CLOCK_N		GENMASK(17, 12)
#define ESP32S3_SPI_CLOCK_H		GENMASK(11, 6)
#define ESP32S3_SPI_CLOCK_L		GENMASK(5, 0)

#define ESP32S3_SPI_USER_MISO		BIT(28)
#define ESP32S3_SPI_USER_MOSI		BIT(27)
#define ESP32S3_SPI_USER_FWRITE_QUAD	BIT(13)
#define ESP32S3_SPI_USER_CK_OUT_EDGE	BIT(9)
#define ESP32S3_SPI_USER_DOUTDIN	BIT(0)

#define ESP32S3_SPI_MISC_CS_KEEP_ACTIVE	BIT(30)
#define ESP32S3_SPI_MISC_CK_IDLE_EDGE	BIT(29)
#define ESP32S3_SPI_MISC_CS0_POL	BIT(7)
#define ESP32S3_SPI_MISC_CS2_DIS	BIT(2)
#define ESP32S3_SPI_MISC_CS1_DIS	BIT(1)
#define ESP32S3_SPI_MISC_CS0_DIS	BIT(0)

#define ESP32S3_SPI_INT_TRANS_DONE	BIT(12)
#define ESP32S3_SPI_INT_DMA_ERRORS	(BIT(0) | BIT(1) | BIT(17) | BIT(18))

#define ESP32S3_SPI_DMA_TX_AFIFO_RST	BIT(31)
#define ESP32S3_SPI_DMA_RX_AFIFO_RST	BIT(29)
#define ESP32S3_SPI_DMA_TX_EN		BIT(28)
#define ESP32S3_SPI_DMA_RX_EN		BIT(27)

#define ESP32S3_SPI_CLK_GATE_EN		BIT(0)
#define ESP32S3_SPI_CLK_GATE_ACTIVE	BIT(1)
#define ESP32S3_SPI_CLK_GATE_PLL	BIT(2)

#define ESP32S3_SPI_FIFO_SIZE		64
#define ESP32S3_SPI_DMA_MAX_SIZE		32768
#define ESP32S3_SPI_UPDATE_TIMEOUT_US	10000
#define ESP32S3_SPI_TRANSFER_TIMEOUT_MS	1000

struct esp32s3_spi {
	void __iomem *base;
	struct completion done;
	struct completion dma_rx_done;
	struct completion dma_tx_done;
	struct reset_control *reset;
	phys_addr_t phys_base;
	unsigned long clk_rate;
	enum dmaengine_tx_result dma_rx_result;
	enum dmaengine_tx_result dma_tx_result;
	int error;
	int irq;
};

static int esp32s3_spi_update(struct esp32s3_spi *espi)
{
	u32 value;

	writel(ESP32S3_SPI_CMD_UPDATE, espi->base + ESP32S3_SPI_CMD);

	return readl_poll_timeout_atomic(espi->base + ESP32S3_SPI_CMD, value,
					 !(value & ESP32S3_SPI_CMD_UPDATE), 1,
					 ESP32S3_SPI_UPDATE_TIMEOUT_US);
}

static void esp32s3_spi_hw_init(struct esp32s3_spi *espi)
{
	writel(ESP32S3_SPI_CLK_GATE_EN | ESP32S3_SPI_CLK_GATE_ACTIVE |
	       ESP32S3_SPI_CLK_GATE_PLL,
	       espi->base + ESP32S3_SPI_CLK_GATE);
	writel(0, espi->base + ESP32S3_SPI_SLAVE);
	writel(0, espi->base + ESP32S3_SPI_CTRL);
	writel(0, espi->base + ESP32S3_SPI_USER);
	writel(0, espi->base + ESP32S3_SPI_DMA_CONF);
	writel(ESP32S3_SPI_MISC_CS1_DIS | ESP32S3_SPI_MISC_CS2_DIS,
	       espi->base + ESP32S3_SPI_MISC);
	writel(0, espi->base + ESP32S3_SPI_INT_ENA);
	writel(ESP32S3_SPI_INT_TRANS_DONE | ESP32S3_SPI_INT_DMA_ERRORS,
	       espi->base + ESP32S3_SPI_INT_CLR);
}

static int esp32s3_spi_recover(struct esp32s3_spi *espi)
{
	int ret;

	writel(0, espi->base + ESP32S3_SPI_INT_ENA);
	synchronize_irq(espi->irq);

	ret = reset_control_reset(espi->reset);
	if (ret)
		return ret;

	esp32s3_spi_hw_init(espi);
	return 0;
}

static irqreturn_t esp32s3_spi_irq(int irq, void *data)
{
	struct esp32s3_spi *espi = data;
	u32 status = readl(espi->base + ESP32S3_SPI_INT_RAW);
	u32 handled = status & (ESP32S3_SPI_INT_TRANS_DONE |
				ESP32S3_SPI_INT_DMA_ERRORS);

	if (!handled)
		return IRQ_NONE;

	writel(handled, espi->base + ESP32S3_SPI_INT_CLR);
	writel(0, espi->base + ESP32S3_SPI_INT_ENA);
	if (status & ESP32S3_SPI_INT_DMA_ERRORS)
		WRITE_ONCE(espi->error, -EIO);
	complete(&espi->done);

	return IRQ_HANDLED;
}

static u32 esp32s3_spi_clock(struct esp32s3_spi *espi, u32 requested,
			     u32 *actual)
{
	unsigned long best_rate = 0;
	u32 best_pre = 1;
	u32 best_n = 1;
	u32 pre;
	u32 n;

	if (requested >= espi->clk_rate) {
		*actual = espi->clk_rate;
		return ESP32S3_SPI_CLOCK_EQU_SYSCLK;
	}

	for (pre = 1; pre <= 16; pre++) {
		for (n = 2; n <= 64; n++) {
			unsigned long rate = espi->clk_rate / (pre * n);

			if (rate <= requested && rate > best_rate) {
				best_rate = rate;
				best_pre = pre;
				best_n = n;
			}
		}
	}

	if (!best_rate) {
		best_pre = 16;
		best_n = 64;
		best_rate = espi->clk_rate / (best_pre * best_n);
	}

	*actual = best_rate;
	return FIELD_PREP(ESP32S3_SPI_CLOCK_PRE, best_pre - 1) |
	       FIELD_PREP(ESP32S3_SPI_CLOCK_N, best_n - 1) |
	       FIELD_PREP(ESP32S3_SPI_CLOCK_H, DIV_ROUND_UP(best_n, 2) - 1) |
	       FIELD_PREP(ESP32S3_SPI_CLOCK_L, best_n - 1);
}

static int esp32s3_spi_bus_width(const struct spi_transfer *xfer,
				 bool transmit)
{
	u8 width = transmit ? xfer->tx_nbits : xfer->rx_nbits;

	if (!width)
		width = SPI_NBITS_SINGLE;
	if (width != SPI_NBITS_SINGLE && width != SPI_NBITS_QUAD)
		return -EINVAL;

	return width;
}

static int esp32s3_spi_configure(struct esp32s3_spi *espi,
				 struct spi_device *spi,
				  struct spi_transfer *xfer,
				  size_t length, bool keep_cs)
{
	u32 clock;
	u32 control = 0;
	u32 misc = ESP32S3_SPI_MISC_CS1_DIS | ESP32S3_SPI_MISC_CS2_DIS;
	u32 user = 0;
	int rx_width = SPI_NBITS_SINGLE;
	int tx_width = SPI_NBITS_SINGLE;

	if (xfer->tx_buf) {
		tx_width = esp32s3_spi_bus_width(xfer, true);
		if (tx_width < 0)
			return tx_width;
		user |= ESP32S3_SPI_USER_MOSI;
		if (tx_width == SPI_NBITS_QUAD)
			user |= ESP32S3_SPI_USER_FWRITE_QUAD;
	}

	if (xfer->rx_buf) {
		rx_width = esp32s3_spi_bus_width(xfer, false);
		if (rx_width < 0)
			return rx_width;
		user |= ESP32S3_SPI_USER_MISO;
		if (rx_width == SPI_NBITS_QUAD)
			control |= ESP32S3_SPI_CTRL_FREAD_QUAD;
	}

	if (xfer->tx_buf && xfer->rx_buf) {
		if (tx_width != SPI_NBITS_SINGLE || rx_width != SPI_NBITS_SINGLE)
			return -EINVAL;
		user |= ESP32S3_SPI_USER_DOUTDIN;
	}

	if (!!(spi->mode & SPI_CPOL) ^ !!(spi->mode & SPI_CPHA))
		user |= ESP32S3_SPI_USER_CK_OUT_EDGE;
	if (spi->mode & SPI_CPOL)
		misc |= ESP32S3_SPI_MISC_CK_IDLE_EDGE;
	if (spi->mode & SPI_CS_HIGH)
		misc |= ESP32S3_SPI_MISC_CS0_POL;
	if (keep_cs)
		misc |= ESP32S3_SPI_MISC_CS_KEEP_ACTIVE;
	if (xfer->cs_off)
		misc |= ESP32S3_SPI_MISC_CS0_DIS;

	clock = esp32s3_spi_clock(espi, xfer->speed_hz,
				  &xfer->effective_speed_hz);
	writel(clock, espi->base + ESP32S3_SPI_CLOCK);
	writel(control, espi->base + ESP32S3_SPI_CTRL);
	writel(user, espi->base + ESP32S3_SPI_USER);
	writel(misc, espi->base + ESP32S3_SPI_MISC);
	writel(length * 8 - 1, espi->base + ESP32S3_SPI_MS_DLEN);

	return esp32s3_spi_update(espi);
}

static void esp32s3_spi_write_fifo(struct esp32s3_spi *espi,
				   const u8 *buffer, size_t length)
{
	unsigned int word;

	for (word = 0; word < DIV_ROUND_UP(length, sizeof(u32)); word++) {
		u32 value = 0;
		unsigned int byte;

		for (byte = 0; byte < sizeof(value); byte++) {
			size_t offset = word * sizeof(value) + byte;

			if (offset < length)
				value |= (u32)buffer[offset] << (byte * 8);
		}
		writel(value, espi->base + ESP32S3_SPI_W(word));
	}
}

static void esp32s3_spi_read_fifo(struct esp32s3_spi *espi, u8 *buffer,
				  size_t length)
{
	unsigned int word;

	for (word = 0; word < DIV_ROUND_UP(length, sizeof(u32)); word++) {
		u32 value = readl(espi->base + ESP32S3_SPI_W(word));
		unsigned int byte;

		for (byte = 0; byte < sizeof(value); byte++) {
			size_t offset = word * sizeof(value) + byte;

			if (offset < length)
				buffer[offset] = value >> (byte * 8);
		}
	}
}

static bool esp32s3_spi_can_dma(struct spi_controller *host,
				struct spi_device *spi,
				struct spi_transfer *xfer)
{
	if (host->fallback || !host->dma_rx || !host->dma_tx)
		return false;
	if (xfer->len <= ESP32S3_SPI_FIFO_SIZE ||
	    xfer->len > ESP32S3_SPI_DMA_MAX_SIZE || !IS_ALIGNED(xfer->len, 4))
		return false;
	if (xfer->tx_buf && !IS_ALIGNED((uintptr_t)xfer->tx_buf, 4))
		return false;
	if (xfer->rx_buf && !IS_ALIGNED((uintptr_t)xfer->rx_buf, 4))
		return false;

	return true;
}

static void esp32s3_spi_dma_rx_callback(void *data,
					const struct dmaengine_result *result)
{
	struct esp32s3_spi *espi = data;

	WRITE_ONCE(espi->dma_rx_result, result->result);
	if (result->result != DMA_TRANS_NOERROR) {
		WRITE_ONCE(espi->error, -EIO);
		complete(&espi->done);
	}
	complete(&espi->dma_rx_done);
}

static void esp32s3_spi_dma_tx_callback(void *data,
					const struct dmaengine_result *result)
{
	struct esp32s3_spi *espi = data;

	WRITE_ONCE(espi->dma_tx_result, result->result);
	if (result->result != DMA_TRANS_NOERROR) {
		WRITE_ONCE(espi->error, -EIO);
		complete(&espi->done);
	}
	complete(&espi->dma_tx_done);
}

static struct dma_async_tx_descriptor *
esp32s3_spi_dma_prep(struct esp32s3_spi *espi,
		     struct dma_chan *chan,
		     struct sg_table *sgt,
		     enum dma_transfer_direction direction)
{
	struct dma_slave_config config = {
		.direction = direction,
	};

	if (direction == DMA_MEM_TO_DEV) {
		config.dst_addr = espi->phys_base + ESP32S3_SPI_W(0);
		config.dst_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
		config.dst_maxburst = 1;
	} else {
		config.src_addr = espi->phys_base + ESP32S3_SPI_W(0);
		config.src_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
		config.src_maxburst = 1;
	}

	if (dmaengine_slave_config(chan, &config))
		return NULL;

	return dmaengine_prep_slave_sg(chan, sgt->sgl, sgt->nents,
				       direction,
				       DMA_PREP_INTERRUPT | DMA_CTRL_ACK);
}

static bool esp32s3_spi_wait_until(struct completion *done,
				   unsigned long deadline)
{
	unsigned long timeout;

	if (time_after_eq(jiffies, deadline))
		return false;
	timeout = deadline - jiffies;

	return wait_for_completion_timeout(done, timeout) != 0;
}

static void esp32s3_spi_dma_terminate(struct spi_controller *host)
{
	dmaengine_terminate_sync(host->dma_rx);
	dmaengine_terminate_sync(host->dma_tx);
}

static int esp32s3_spi_transfer_dma(struct spi_controller *host,
				    struct spi_device *spi,
				    struct spi_transfer *xfer,
				    bool keep_cs)
{
	struct esp32s3_spi *espi = spi_controller_get_devdata(host);
	struct dma_async_tx_descriptor *rxdesc = NULL;
	struct dma_async_tx_descriptor *txdesc = NULL;
	bool rx_submitted = false;
	bool tx_submitted = false;
	unsigned long deadline;
	dma_cookie_t cookie;
	u32 dma_conf = 0;
	int recover_ret;
	int ret;

	ret = esp32s3_spi_configure(espi, spi, xfer, xfer->len, keep_cs);
	if (ret)
		return ret;

	if (xfer->rx_buf) {
		rxdesc = esp32s3_spi_dma_prep(espi, host->dma_rx, &xfer->rx_sg,
					      DMA_DEV_TO_MEM);
		if (!rxdesc)
			goto no_start;
		rxdesc->callback_result = esp32s3_spi_dma_rx_callback;
		rxdesc->callback_param = espi;
	}
	if (xfer->tx_buf) {
		txdesc = esp32s3_spi_dma_prep(espi, host->dma_tx, &xfer->tx_sg,
					      DMA_MEM_TO_DEV);
		if (!txdesc)
			goto no_start;
		txdesc->callback_result = esp32s3_spi_dma_tx_callback;
		txdesc->callback_param = espi;
	}

	if (rxdesc) {
		cookie = dmaengine_submit(rxdesc);
		ret = dma_submit_error(cookie);
		if (ret)
			goto no_start;
		rx_submitted = true;
	}
	if (txdesc) {
		cookie = dmaengine_submit(txdesc);
		ret = dma_submit_error(cookie);
		if (ret)
			goto no_start;
		tx_submitted = true;
	}

	reinit_completion(&espi->done);
	reinit_completion(&espi->dma_rx_done);
	reinit_completion(&espi->dma_tx_done);
	WRITE_ONCE(espi->error, 0);
	WRITE_ONCE(espi->dma_rx_result, DMA_TRANS_NOERROR);
	WRITE_ONCE(espi->dma_tx_result, DMA_TRANS_NOERROR);
	if (rxdesc)
		dma_conf |= ESP32S3_SPI_DMA_RX_EN;
	if (txdesc)
		dma_conf |= ESP32S3_SPI_DMA_TX_EN;
	writel(ESP32S3_SPI_DMA_RX_AFIFO_RST |
	       ESP32S3_SPI_DMA_TX_AFIFO_RST,
	       espi->base + ESP32S3_SPI_DMA_CONF);
	writel(dma_conf, espi->base + ESP32S3_SPI_DMA_CONF);

	if (rxdesc)
		dma_async_issue_pending(host->dma_rx);
	if (txdesc)
		dma_async_issue_pending(host->dma_tx);
	writel(ESP32S3_SPI_INT_TRANS_DONE | ESP32S3_SPI_INT_DMA_ERRORS,
	       espi->base + ESP32S3_SPI_INT_CLR);
	writel(ESP32S3_SPI_INT_TRANS_DONE | ESP32S3_SPI_INT_DMA_ERRORS,
	       espi->base + ESP32S3_SPI_INT_ENA);
	writel(ESP32S3_SPI_CMD_USR, espi->base + ESP32S3_SPI_CMD);

	deadline = jiffies +
		msecs_to_jiffies(spi_controller_xfer_timeout(host, xfer));
	ret = -ETIMEDOUT;
	if (!esp32s3_spi_wait_until(&espi->done, deadline))
		goto transfer_failed;
	if (READ_ONCE(espi->error)) {
		ret = READ_ONCE(espi->error);
		goto transfer_failed;
	}
	if (rxdesc && !esp32s3_spi_wait_until(&espi->dma_rx_done, deadline))
		goto transfer_failed;
	if (txdesc && !esp32s3_spi_wait_until(&espi->dma_tx_done, deadline))
		goto transfer_failed;
	if (READ_ONCE(espi->dma_rx_result) != DMA_TRANS_NOERROR ||
	    READ_ONCE(espi->dma_tx_result) != DMA_TRANS_NOERROR) {
		ret = -EIO;
		goto transfer_failed;
	}

	writel(0, espi->base + ESP32S3_SPI_DMA_CONF);
	return 0;

transfer_failed:
	writel(0, espi->base + ESP32S3_SPI_DMA_CONF);
	esp32s3_spi_dma_terminate(host);
	recover_ret = esp32s3_spi_recover(espi);
	return recover_ret ? recover_ret : ret;

no_start:
	if (rx_submitted)
		dmaengine_terminate_sync(host->dma_rx);
	else if (rxdesc)
		dmaengine_desc_free(rxdesc);
	if (tx_submitted)
		dmaengine_terminate_sync(host->dma_tx);
	else if (txdesc)
		dmaengine_desc_free(txdesc);
	xfer->error |= SPI_TRANS_FAIL_NO_START;
	return -EAGAIN;
}

static int esp32s3_spi_transfer_chunk(struct esp32s3_spi *espi,
				      struct spi_device *spi,
				       struct spi_transfer *xfer,
				       size_t offset, size_t length,
				       bool keep_cs)
{
	unsigned long timeout;
	int ret;

	ret = esp32s3_spi_configure(espi, spi, xfer, length, keep_cs);
	if (ret == -ETIMEDOUT) {
		int recover_ret = esp32s3_spi_recover(espi);

		return recover_ret ? recover_ret : ret;
	}
	if (ret)
		return ret;

	if (xfer->tx_buf)
		esp32s3_spi_write_fifo(espi,
				       (const u8 *)xfer->tx_buf + offset, length);

	reinit_completion(&espi->done);
	writel(ESP32S3_SPI_INT_TRANS_DONE,
	       espi->base + ESP32S3_SPI_INT_CLR);
	writel(ESP32S3_SPI_INT_TRANS_DONE,
	       espi->base + ESP32S3_SPI_INT_ENA);
	writel(ESP32S3_SPI_CMD_USR, espi->base + ESP32S3_SPI_CMD);

	timeout = wait_for_completion_timeout(&espi->done,
					      msecs_to_jiffies(ESP32S3_SPI_TRANSFER_TIMEOUT_MS));
	if (!timeout) {
		ret = esp32s3_spi_recover(espi);
		return ret ? ret : -ETIMEDOUT;
	}

	if (xfer->rx_buf)
		esp32s3_spi_read_fifo(espi, (u8 *)xfer->rx_buf + offset,
				      length);

	return 0;
}

static int esp32s3_spi_transfer_one(struct spi_controller *host,
				    struct spi_device *spi,
				     struct spi_transfer *xfer)
{
	struct esp32s3_spi *espi = spi_controller_get_devdata(host);
	bool last = spi_transfer_is_last(host, xfer);
	bool keep_after = last ? xfer->cs_change : !xfer->cs_change;
	size_t offset = 0;

	if (!xfer->tx_buf && !xfer->rx_buf)
		return -EINVAL;
	if (esp32s3_spi_can_dma(host, spi, xfer))
		return esp32s3_spi_transfer_dma(host, spi, xfer, keep_after);

	while (offset < xfer->len) {
		size_t length = min_t(size_t, xfer->len - offset,
				      ESP32S3_SPI_FIFO_SIZE);
		bool keep_cs = offset + length < xfer->len || keep_after;
		int ret;

		ret = esp32s3_spi_transfer_chunk(espi, spi, xfer, offset,
						 length, keep_cs);
		if (ret)
			return ret;
		offset += length;
	}

	return 0;
}

static void esp32s3_spi_set_cs(struct spi_device *spi, bool enable)
{
	struct esp32s3_spi *espi = spi_controller_get_devdata(spi->controller);
	u32 misc;

	if (enable)
		return;

	misc = readl(espi->base + ESP32S3_SPI_MISC);
	if (!(misc & ESP32S3_SPI_MISC_CS_KEEP_ACTIVE))
		return;

	writel(misc & ~ESP32S3_SPI_MISC_CS_KEEP_ACTIVE,
	       espi->base + ESP32S3_SPI_MISC);
	if (esp32s3_spi_update(espi))
		dev_err(&spi->dev, "failed to release chip select\n");
}

static void esp32s3_spi_release_dma(void *data)
{
	struct spi_controller *host = data;

	dma_release_channel(host->dma_rx);
	host->dma_rx = NULL;
	dma_release_channel(host->dma_tx);
	host->dma_tx = NULL;
}

static int esp32s3_spi_request_dma(struct device *dev,
				   struct spi_controller *host)
{
	struct dma_chan *rx;
	struct dma_chan *tx;
	int ret;

	rx = dma_request_chan(dev, "rx");
	if (IS_ERR(rx)) {
		ret = PTR_ERR(rx);
		if (ret == -EPROBE_DEFER)
			return ret;
		dev_warn(dev, "RX DMA unavailable, using PIO: %pe\n", rx);
		return 0;
	}

	tx = dma_request_chan(dev, "tx");
	if (IS_ERR(tx)) {
		ret = PTR_ERR(tx);
		dma_release_channel(rx);
		if (ret == -EPROBE_DEFER)
			return ret;
		dev_warn(dev, "TX DMA unavailable, using PIO: %pe\n", tx);
		return 0;
	}

	host->dma_rx = rx;
	host->dma_tx = tx;
	ret = devm_add_action_or_reset(dev, esp32s3_spi_release_dma, host);
	if (ret)
		return ret;

	return 0;
}

static int esp32s3_spi_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct spi_controller *host;
	struct esp32s3_spi *espi;
	struct resource *res;
	struct clk *clk;
	int ret;

	host = devm_spi_alloc_host(dev, sizeof(*espi));
	if (!host)
		return -ENOMEM;

	espi = spi_controller_get_devdata(host);
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -EINVAL;

	espi->base = devm_ioremap_resource(dev, res);
	if (IS_ERR(espi->base))
		return PTR_ERR(espi->base);
	espi->phys_base = res->start;

	clk = devm_clk_get_enabled(dev, NULL);
	if (IS_ERR(clk))
		return dev_err_probe(dev, PTR_ERR(clk), "failed to enable clock\n");
	espi->clk_rate = clk_get_rate(clk);
	if (!espi->clk_rate)
		return dev_err_probe(dev, -EINVAL, "clock has zero rate\n");

	espi->reset = devm_reset_control_get_exclusive_deasserted(dev, NULL);
	if (IS_ERR(espi->reset))
		return dev_err_probe(dev, PTR_ERR(espi->reset),
				     "failed to deassert reset\n");

	espi->irq = platform_get_irq(pdev, 0);
	if (espi->irq < 0)
		return espi->irq;

	init_completion(&espi->done);
	init_completion(&espi->dma_rx_done);
	init_completion(&espi->dma_tx_done);
	ret = devm_request_irq(dev, espi->irq, esp32s3_spi_irq, 0,
			       dev_name(dev), espi);
	if (ret)
		return dev_err_probe(dev, ret, "failed to request interrupt\n");

	host->mode_bits = SPI_CPOL | SPI_CPHA | SPI_CS_HIGH |
			  SPI_TX_QUAD | SPI_RX_QUAD;
	host->bits_per_word_mask = SPI_BPW_MASK(8);
	host->num_chipselect = 1;
	host->min_speed_hz = DIV_ROUND_UP(espi->clk_rate, 16 * 64);
	host->max_speed_hz = espi->clk_rate;
	host->max_dma_len = ESP32S3_SPI_DMA_MAX_SIZE;
	host->dma_alignment = 4;
	host->set_cs = esp32s3_spi_set_cs;
	host->can_dma = esp32s3_spi_can_dma;
	host->transfer_one = esp32s3_spi_transfer_one;
	ret = esp32s3_spi_request_dma(dev, host);
	if (ret)
		return ret;

	esp32s3_spi_hw_init(espi);
	platform_set_drvdata(pdev, host);

	return devm_spi_register_controller(dev, host);
}

static const struct of_device_id esp32s3_spi_of_match[] = {
	{ .compatible = "esp,esp32s3-spi" },
	{ }
};
MODULE_DEVICE_TABLE(of, esp32s3_spi_of_match);

static struct platform_driver esp32s3_spi_driver = {
	.probe = esp32s3_spi_probe,
	.driver = {
		.name = "esp32s3-spi",
		.of_match_table = esp32s3_spi_of_match,
	},
};
module_platform_driver(esp32s3_spi_driver);

MODULE_DESCRIPTION("Espressif ESP32-S3 general-purpose SPI controller");
MODULE_LICENSE("GPL");
