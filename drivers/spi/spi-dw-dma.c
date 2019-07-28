
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/types.h>

#include "spi-dw.h"

#include <linux/platform_data/dma-dw.h>
#include <linux/delay.h>

#define TX_BURST_LEVEL 4
#define RX_BURST_LEVEL 4
#define MAX_DMA_LEN 4095

#define RX_BUSY     0
#define TX_BUSY     1

static int dw_spi_dma_init(struct dw_spi *dws)
{
	struct device *dev = &(dws->master->dev);
	int ret;

	/*
	 * Set max DMA len to be of maximum DW DMA block-size, so the SPI
	 * subsystem would split the transers up accordingly.
	 */
	dws->master->max_dma_len = MAX_DMA_LEN;

	/* Request exclusive access to the rx channel. */
	dws->rxchan = dma_request_slave_channel_reason(dev->parent, "rx");
	if(IS_ERR(dws->rxchan)) {
		dev_dbg(dev->parent, "Failed to get Rx DMA channel\n");
		return PTR_ERR(dws->rxchan);
	}
	dws->master->dma_rx = dws->rxchan;

	/* Request exclusive access to the tx channel. */
	dws->txchan = dma_request_slave_channel_reason(dev->parent, "tx");
	if(IS_ERR(dws->txchan)) {
		ret = PTR_ERR(dws->txchan);
		dev_dbg(dev->parent, "Failed to get Tx DMA channel\n");
		goto err_dma_release_rxchan;
	}
	dws->master->dma_tx = dws->txchan;

	dws->dma_inited = 1;
	return 0;

err_dma_release_rxchan:
	dma_release_channel(dws->rxchan);

	return ret;
}

static void dw_spi_dma_exit(struct dw_spi *dws)
{
	if (!dws->dma_inited)
		return;

	dmaengine_terminate_all(dws->txchan);
	dma_release_channel(dws->txchan);

	dmaengine_terminate_all(dws->rxchan);
	dma_release_channel(dws->rxchan);
}

static bool dw_spi_dma_can(struct spi_master *master, struct spi_device *spi,
			   struct spi_transfer *xfer)
{
	struct dw_spi *dws = spi_master_get_devdata(master);

	return (dws->dma_inited) && (xfer->len > dws->fifo_len);
}

static enum dma_slave_buswidth dw_spi_dma_convert_width(u32 dma_width) {
	if (dma_width == 1)
		return DMA_SLAVE_BUSWIDTH_1_BYTE;
	else if (dma_width == 2)
		return DMA_SLAVE_BUSWIDTH_2_BYTES;

	return DMA_SLAVE_BUSWIDTH_UNDEFINED;
}

static inline int dw_spi_dma_tx_busy(struct dw_spi *dws)
{
	return (dw_readl(dws, DW_SPI_SR) & (SR_BUSY | SR_TF_EMPT)) ^ SR_TF_EMPT;
}

static void dw_spi_dma_wait_tx_done(struct dw_spi *dws)
{
	unsigned int us = (dw_readl(dws, DW_SPI_TXFLR) + 1) * 1000;

	while (dw_spi_dma_tx_busy(dws) && --us)
		udelay(1);

	if(!us) {
		dev_err(&dws->master->cur_msg->spi->dev, "Tx hanged up\n");
		dws->master->cur_msg->status = -EIO;
	}
}

static inline int dw_spi_dma_rx_busy(struct dw_spi *dws)
{
	return dw_readl(dws, DW_SPI_SR) & (SR_BUSY | SR_RF_NOT_EMPT);
}

static void dw_spi_dma_wait_rx_done(struct dw_spi *dws)
{
	unsigned int us = (dw_readl(dws, DW_SPI_RXFLR) + 1) * 1000;

	while (dw_spi_dma_rx_busy(dws) && --us)
		udelay(1);

	if(!us) {
		dev_err(&dws->master->cur_msg->spi->dev, "Rx hanged up\n");
		dws->master->cur_msg->status = -EIO;
	}
}

static void dw_spi_dma_tx_done(void *arg)
{
	struct dw_spi *dws = arg;

	dw_spi_dma_wait_tx_done(dws);
	clear_bit(TX_BUSY, &dws->dma_chan_busy);
	if (test_bit(RX_BUSY, &dws->dma_chan_busy))
		return;

	dw_writel(dws, DW_SPI_DMACR, 0);
	spi_finalize_current_transfer(dws->master);
}

static void dw_spi_dma_rx_done(void *arg)
{
	struct dw_spi *dws = arg;

	dw_spi_dma_wait_rx_done(dws);
	clear_bit(RX_BUSY, &dws->dma_chan_busy);
	if (test_bit(TX_BUSY, &dws->dma_chan_busy))
		return;

	dw_writel(dws, DW_SPI_DMACR, 0);
	spi_finalize_current_transfer(dws->master);
}

static struct dma_async_tx_descriptor *
dw_spi_dma_prepare_tx(struct dw_spi *dws, struct spi_transfer *xfer)
{
	struct dma_async_tx_descriptor *desc;
	struct dma_slave_config txconf = {0};
	int ret;

	/* Tx slave DMA channel config. */
	txconf.direction      = DMA_MEM_TO_DEV;
	txconf.dst_addr       = dws->dma_addr;
	txconf.dst_maxburst   = TX_BURST_LEVEL;
	txconf.src_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
	txconf.dst_addr_width = dw_spi_dma_convert_width(dws->dma_width);
	txconf.device_fc      = false;

	ret = dmaengine_slave_config(dws->txchan, &txconf);
	if (ret) {
		dev_err(&dws->master->dev, "Failed to set Tx DMA config\n");
		return ERR_PTR(ret);
	}

	/* Get Tx block descriptor. */
	desc = dmaengine_prep_slave_sg(dws->txchan,
		xfer->tx_sg.sgl, xfer->tx_sg.nents,
		DMA_MEM_TO_DEV, DMA_PREP_INTERRUPT | DMA_CTRL_ACK);
	if (!desc) {
		dev_err(&dws->master->dev, "Failed to prepare Tx DMA block\n");
		return ERR_PTR(-EIO);
	}

	desc->callback = dw_spi_dma_tx_done;
	desc->callback_param = dws;

	return desc;
}

static int dw_spi_dma_submit_tx(struct dw_spi *dws,
				struct dma_async_tx_descriptor *desc)
{
	int ret;

	/*
	 * Submit DMA tranfer, then set flag of the channel being busy and
	 * launch the transfer.
	 */
	ret = dma_submit_error(dmaengine_submit(desc));
	if (ret) {
		dev_err(&dws->master->dev, "Failed to submit Tx DMA block\n");
		return ret;
	}
	set_bit(TX_BUSY, &dws->dma_chan_busy);
	dma_async_issue_pending(dws->txchan);

	return 0;
}

static struct dma_async_tx_descriptor *
dw_spi_dma_prepare_rx(struct dw_spi *dws, struct spi_transfer *xfer)
{
	struct dma_async_tx_descriptor *desc;
	struct dma_slave_config rxconf = {0};
	int ret;

	/* Tx slave DMA channel config. */
	rxconf.direction      = DMA_DEV_TO_MEM;
	rxconf.device_fc      = false;
	rxconf.src_addr_width = dw_spi_dma_convert_width(dws->dma_width);
	rxconf.dst_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
	rxconf.src_maxburst   = RX_BURST_LEVEL;
	rxconf.src_addr       = dws->dma_addr;

	ret = dmaengine_slave_config(dws->rxchan, &rxconf);
	if (ret) {
		dev_err(&dws->master->dev, "Failed to set Rx DMA config\n");
		return ERR_PTR(ret);
	}

	/* Get Rx block descriptor. */
	desc = dmaengine_prep_slave_sg(dws->rxchan,
		xfer->rx_sg.sgl, xfer->rx_sg.nents,
		DMA_DEV_TO_MEM, DMA_PREP_INTERRUPT | DMA_CTRL_ACK);
	if (!desc) {
		dev_err(&dws->master->dev, "Failed to prepare Rx DMA block\n");
		return ERR_PTR(-EIO);
	}

	desc->callback = dw_spi_dma_rx_done;
	desc->callback_param = dws;

	return desc;
}

static int dw_spi_dma_submit_rx(struct dw_spi *dws,
				struct dma_async_tx_descriptor *desc)
{
	int ret;

	/*
	 * Submit DMA tranfer, then set flag of the channel being busy and
	 * launch the transfer.
	 */
	ret = dma_submit_error(dmaengine_submit(desc));
	if (ret) {
		dev_err(&dws->master->dev, "Failed to submit Rx DMA block\n");
		return ret;
	}
	set_bit(RX_BUSY, &dws->dma_chan_busy);
	dma_async_issue_pending(dws->rxchan);

	/* Write dummy data to start read-only mode. */
	if (!dws->tx)
		dw_writel(dws, DW_SPI_DR, 0);

	return 0;
}

static irqreturn_t dw_spi_dma_handler(struct dw_spi *dws)
{
	u16 irq_status = dw_readl(dws, DW_SPI_ISR);

	if (!irq_status)
		return IRQ_NONE;

	dw_readl(dws, DW_SPI_ICR);
	spi_reset_chip(dws);

	dev_err(&dws->master->dev, "%s: FIFO overrun/underrun (ISR 0x%02x)\n", __func__, irq_status);

	dws->master->cur_msg->status = -EIO;
	spi_finalize_current_transfer(dws->master);

	return IRQ_HANDLED;
}

static int dw_spi_dma_setup(struct dw_spi *dws, struct spi_transfer *xfer)
{
	u16 imr = 0, dma_ctrl = 0;

	dw_writel(dws, DW_SPI_DMATDLR, dws->fifo_len - TX_BURST_LEVEL);
	dw_writel(dws, DW_SPI_DMARDLR, RX_BURST_LEVEL - 1);

	if(xfer->tx_buf) {
		dma_ctrl |= SPI_DMA_TDMAE;
		imr |= SPI_INT_TXOI;
	}
	if(xfer->rx_buf) {
		dma_ctrl |= SPI_DMA_RDMAE;
		imr |= SPI_INT_RXOI | SPI_INT_RXUI;
	}
	dw_writel(dws, DW_SPI_DMACR, dma_ctrl);
	spi_umask_intr(dws, imr);

	dws->transfer_handler = dw_spi_dma_handler;

	return 0;
}

static int dw_spi_dma_transfer(struct dw_spi *dws, struct spi_transfer *xfer)
{
	struct dma_async_tx_descriptor *txdesc, *rxdesc;
	int ret;

	/*
	 * Setup channels to work with the passed transfer and get the DMA Tx
	 * descripor for the transfer SG-list. We also need to submit rx before
	 * tx due to spi instinct.
	 */
	if (xfer->rx_buf) {
		rxdesc = dw_spi_dma_prepare_rx(dws, xfer);
		if (IS_ERR(rxdesc))
			return PTR_ERR(rxdesc);

		ret = dw_spi_dma_submit_rx(dws, rxdesc);
		if (ret)
			return ret;
	} else {
		clear_bit(RX_BUSY, &dws->dma_chan_busy);
	}

	/*
	 * Don't warry about rxdesc resource deallocation if the next code
	 * fails, dma_stop will get called by the handle_err callback of the
	 * generic DW SPI driver.
	 */
	if (xfer->tx_buf) {
		txdesc = dw_spi_dma_prepare_tx(dws, xfer);
		if (IS_ERR(txdesc))
			return PTR_ERR(txdesc);

		ret = dw_spi_dma_submit_tx(dws, txdesc);
		if (ret)
			return ret;
	} else {
		clear_bit(TX_BUSY, &dws->dma_chan_busy);
	}

	/* Enable SPI data transfer by setting the corresponding hardware CS */
	dw_writel(dws, DW_SPI_SER, BIT(dws->master->cur_msg->spi->chip_select));

	return 0;
}

static void dw_spi_dma_stop(struct dw_spi *dws)
{
	if (test_bit(TX_BUSY, &dws->dma_chan_busy)) {
		dmaengine_terminate_all(dws->txchan);
		clear_bit(TX_BUSY, &dws->dma_chan_busy);
	}
	if (test_bit(RX_BUSY, &dws->dma_chan_busy)) {
		dmaengine_terminate_all(dws->rxchan);
		clear_bit(RX_BUSY, &dws->dma_chan_busy);
	}

	dw_writel(dws, DW_SPI_DMACR, 0);
}

static struct dw_spi_dma_ops  dma_ops = {
	.dma_init     = dw_spi_dma_init,
	.dma_exit     = dw_spi_dma_exit,
	.can_dma      = dw_spi_dma_can,
	.dma_setup    = dw_spi_dma_setup,
	.dma_transfer = dw_spi_dma_transfer,
	.dma_stop     = dw_spi_dma_stop,
};

void dw_spi_mmio_dma_init(struct dw_spi *dws)
{
	dws->dma_ops = &dma_ops;
}
