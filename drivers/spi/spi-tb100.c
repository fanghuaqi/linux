/*
 * Abilis Systems: TB100 SPI controller driver
 * Copyright (C) 2012-2013 Pierrick Hascoet <pierrick.hascoet@abilis.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/interrupt.h>
#include <linux/dma-mapping.h>
#include <linux/dmapool.h>

#include <linux/spi/spi.h>

#ifdef CONFIG_DEBUG_FS
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#endif

#define DRIVER_NAME "tb100_spi"

/* TODO: remove temporary internal debug flags */
#define TB100_SPI_DEBUG		0
#define TB100_SPI_DUMP_XFER	0
#define TB100_SPI_DUMP_LLI	0

/* control0 register */
#define	SPI_REG_CTRL0			0x00
#define		SPI_ENABLE		(1 << 0)
#define		SPI_MODE(x)		(((x) & 0xf) << 4)
#define		SPI_TMOD(x)		(((x) & 0xf) << 8)
#define		SPI_TMOD_FULL_DUPLEX	SPI_TMOD(0)
#define		SPI_TMOD_HALF_DUPLEX	SPI_TMOD(3)
#define		SPI_SSTGCPH1		(1 << 6)
#define		SPI_DMA_EN		(0 << 10)
#define		SPI_PIO_EN		(1 << 10)
#define		SPI_DFS(x)		((x - 1) << 12)
#define		SPI_LOOP_EN		(1 << 28)
#define		SPI_RESET		(1 << 31)

/* control1 register */
#define SPI_REG_CTRL1			0x04
#define		SPI_STARTXFR		(1 << 0)
#define		SPI_TSCD(x)		(((x) & 0xf) << 16)
#define		SPI_TIDL(x)		(((x) & 0xf) << 20)

#define	SPI_REG_TX_CTRL			0x08
#define	SPI_REG_RX_CTRL			0x0c

#define SPI_REG_SER			0x10
#define		SPI_SS_EN(x)		(1 << x)

#define SPI_REG_BAUDRATE		0x18
#define SPI_REG_STATUS			0x1c

#define	SPI_REG_INTR_STATUS		0x20
#define	SPI_REG_INTR_ENABLE		0x24
#define		SPI_STAT_ENDOFTRANS	(1 << 4)
#define		SPI_STAT_XFERR		(1 << 5)
#define		SPI_STAT_RXERR		(1 << 6)
#define		SPI_STAT_TXERR		(1 << 7)
#define		SPI_STAT_RXFNE		(1 << 8)

#define	SPI_REG_TX_DMA_START		0x28
#define	SPI_REG_TX_DMA_LLP		0x2c
#define	SPI_REG_TX_DMA_CTRL		0x30
#define SPI_REG_TX_DMA_LAST_ADDR	0x38
#define SPI_REG_TX_DMA_LAST_DATA	0x3c

/* single dma control register */
#define	SPI_REG_RX_DMA_START		0x40
/* link list dma control registers */
#define	SPI_REG_RX_DMA_LLP		0x44
#define	SPI_REG_RX_DMA_CTRL		0x48
#define SPI_REG_RX_DMA_LAST_ADDR	0x50
#define SPI_REG_RX_DMA_LAST_DATA	0x54

#define SPI_REG_TX_DATA			0x58
#define SPI_REG_RX_DATA			0x5c

#define SPI_REG_ID			0x68
#define SPI_REG_PARAM1			0x6c
#define SPI_REG_PARAM2			0x70

struct tb100_spi_hw_info {
	/* spi hw version */
	uint16_t version;
	/* spi hw fifo size */
	uint8_t rx_ff_size;
	uint8_t tx_ff_size;
	/* spi hw threshold */
	uint8_t tx_af_level;
	uint8_t tx_ae_level;
	uint8_t rx_af_level;
	/* pad */
	uint8_t pad;
};

struct tb100_spi_lli_desc {
	/* dma target address (src or dst) */
	uint32_t dmastart;
	uint32_t llp;
	uint32_t dmactrl;
	uint32_t used;
	/* internal data */
	struct list_head node;
	/* lli dma address */
	dma_addr_t dma_addr;
};

struct tb100_spi_xfer {
	struct spi_message *m;

	/* xfer len */
	uint32_t tx_len;
	uint32_t rx_len;

	/* dma lli tx queue */
	struct list_head tx_queue;
	struct sg_table sgt_tx;
	uint32_t sgt_tx_nelem;

	/* dma lli rx queue */
	struct list_head rx_queue;
	struct sg_table sgt_rx;
	uint32_t sgt_rx_nelem;

	struct list_head node;
};

#define MAX_LLI	16
struct tb100_pms_spi {
	struct spi_master *master;
	struct platform_device *pdev;

	struct tb100_spi_hw_info hw_info;
	spinlock_t lock;

	/* tb100 spi xfer queue */
	struct list_head x_queue;

	/* linked list item dma pool */
	struct dma_pool *lli_pool;

	/* spi dma descriptors (lli desc) */
	struct tb100_spi_lli_desc *lli_desc[MAX_LLI];

	/* spi root clock */
	struct clk *clk;

	/* spi source clock divider */
	uint32_t sclkdiv;
	uint32_t max_speed;
	uint32_t cur_speed;

	void __iomem *iomem;
	int irq;

#ifdef CONFIG_DEBUG_FS
	struct dentry *debugfs;
#endif
};

static void arc_halt(void)
{
	printk("%s entered !!! please debug !!! \n", DRIVER_NAME);
	/* Halt the processor */
	__asm__ __volatile__("flag  1\n");
}

static inline uint32_t tb100_spi_readl(struct tb100_pms_spi *dev,
		uint32_t reg)
{
	return readl(dev->iomem + reg);
}

static inline void tb100_spi_writel(struct tb100_pms_spi *dev,
		uint32_t val, uint32_t reg)
{
	return writel(val, (dev->iomem + reg));
}

static void tb100_spi_dump_registers(struct tb100_pms_spi *dev)
{
	int i;

	for (i = 0; i < 0x16; i++) {
		printk("%02x: %08x\n", (i*4), tb100_spi_readl(dev, (i*4)));
	}
}

static int tb100_spi_set_speed(struct tb100_pms_spi *dev,
		uint32_t speed_hz)
{
	unsigned long bus_speed_hz = clk_get_rate(dev->clk);

	dev->max_speed = speed_hz;
	dev->sclkdiv = DIV_ROUND_UP(bus_speed_hz, speed_hz);
	tb100_spi_writel(dev, dev->sclkdiv, SPI_REG_BAUDRATE);

	dev->cur_speed = (bus_speed_hz / dev->sclkdiv);

	return dev->cur_speed;
}

static int tb100_spi_setup(struct spi_device *spi)
{
	struct tb100_pms_spi *dev = spi_master_get_devdata(spi->master);
	uint32_t reg;
	int ret;

	tb100_spi_writel(dev, SPI_RESET, SPI_REG_CTRL0);

	reg = tb100_spi_readl(dev, SPI_REG_CTRL0);

	reg |= SPI_MODE(spi->mode);
	reg |= SPI_DFS(spi->bits_per_word);
	/* enable spi port */
	reg |= (SPI_ENABLE | SPI_TMOD_HALF_DUPLEX);

	tb100_spi_writel(dev, reg, SPI_REG_CTRL0);

	/* select slave select */
	reg = SPI_SS_EN(spi->chip_select);
	tb100_spi_writel(dev, reg, SPI_REG_SER);

	/* enable XFRREN */
	tb100_spi_writel(dev,
			(SPI_STAT_XFERR | SPI_STAT_TXERR | SPI_STAT_RXERR),
			SPI_REG_INTR_ENABLE);

	ret = tb100_spi_set_speed(dev, spi->max_speed_hz);

	return ret;
}

static void tb100_spi_xfer_start(struct tb100_pms_spi *dev)
{
	uint32_t reg = tb100_spi_readl(dev, SPI_REG_INTR_ENABLE);

	if (!(reg & SPI_STAT_ENDOFTRANS)) {
		tb100_spi_writel(dev, (reg | SPI_STAT_ENDOFTRANS),
				SPI_REG_INTR_ENABLE);
	}

	// TODO
	//tb100_spi_dump_registers(dev);

	reg = tb100_spi_readl(dev, SPI_REG_CTRL1);
	reg |= (SPI_STARTXFR | SPI_TSCD(2)) ;

	tb100_spi_writel(dev, reg, SPI_REG_CTRL1);
}

static void tb100_spi_reset_lli(struct tb100_pms_spi *dev,
		struct tb100_spi_lli_desc *desc)
{
	/* TODO: lock */
	if (desc->used) {
		// TODO: replace deadbeef -> 0
		desc->dmastart = 0xdeadbeef;
		desc->llp = 0xdeadbeef;
		desc->dmactrl = 0xdeadbeef;
		desc->used = 0;
	}
	/* TODO: unlock */
}

static void tb100_spi_flush_xfer_queues(struct tb100_pms_spi *dev,
		struct tb100_spi_xfer *xfer)
{
	struct tb100_spi_lli_desc *lli;

	if (list_empty(&xfer->rx_queue) && list_empty(&xfer->tx_queue))
		return;

	list_for_each_entry(lli, &xfer->tx_queue, node) {
		tb100_spi_reset_lli(dev, lli);
	}
	list_del_init(&xfer->tx_queue);

	list_for_each_entry(lli, &xfer->rx_queue, node) {
		tb100_spi_reset_lli(dev, lli);
	}
	list_del_init(&xfer->rx_queue);
}

static struct tb100_spi_lli_desc *tb100_spi_get_lli(
		struct tb100_pms_spi *dev)
{
	struct tb100_spi_lli_desc *desc;
	int i;

	// TODO: lock

	for (i = 0; i < MAX_LLI; i++) {
		desc = dev->lli_desc[i];
		if (!desc->used) {
			desc->used = 1;
			/* init dma control */
			desc->dmactrl = 0x438; // TODO: FIXME with define values
			desc->llp = 0;

			/* TODO: unlock */

			return desc;
		}
	}

	// TODO: unlock

	dev_err(&dev->master->dev, "%s: not enough lli descriptors\n",
			__func__);
	return NULL;
}

static void tb100_spi_dump_lli_queue(char *nl, int len, struct list_head *queue)
{
#if TB100_SPI_DUMP_LLI
	struct tb100_spi_lli_desc *lli;
	int not_single = 0;

	not_single = !list_is_singular(queue);
	list_for_each_entry(lli, queue, node) {
		u8 not_aligned = 0;

		if ((lli->dmastart % 4)) {
			not_aligned = 1;
		}
		printk("%s: %s lli@%p (dma: %08x) : dmastart: %08x %s; llp: %08x ; dmactrl: %08x ; len: %d\n",
			__func__,
			nl,
			lli,
			lli->dma_addr,
			lli->dmastart,
			(not_aligned && not_single) ? "** NOT ALIGNED **": "",
			lli->llp, lli->dmactrl,
			len);
	}
#endif
}

static void tb100_spi_lli_chain(struct tb100_spi_lli_desc *lli)
{
	struct tb100_spi_lli_desc *prev;

	prev = container_of(lli->node.prev, struct tb100_spi_lli_desc, node);

	/* previous llp is me */
	prev->llp = lli->dma_addr;
	/* activate chaining mode */
	prev->dmactrl |= 0x01;
}

static int tb100_spi_configure_dma_xfer(struct tb100_pms_spi *dev,
		struct tb100_spi_xfer *x)
{
	struct scatterlist *sg;
	struct tb100_spi_lli_desc *lli;
	int i;

	for_each_sg(x->sgt_tx.sgl, sg, x->sgt_tx_nelem, i) {
		lli = tb100_spi_get_lli(dev);
		if (!lli)
			return -ENOMEM;

		lli->dmastart = sg_dma_address(sg);
		lli->dmactrl |= (sg_dma_len(sg) << 12);

		/* enqueue tx lli */
		list_add_tail(&lli->node, &x->tx_queue);

		if (i > 0)
			tb100_spi_lli_chain(lli);
	}

	for_each_sg(x->sgt_rx.sgl, sg, x->sgt_rx_nelem, i) {
		lli = tb100_spi_get_lli(dev);
		if (!lli)
			return -ENOMEM;

		lli->dmastart = sg_dma_address(sg);
		lli->dmactrl |= (sg_dma_len(sg) << 12);

		/* enqueue rx lli */
		list_add_tail(&lli->node, &x->rx_queue);

		/* chain our dma lli if the sg table is not singular */
		if (i > 0)
			tb100_spi_lli_chain(lli);
	}

	return 0;
}

static int tb100_spi_dma_xfer(struct tb100_pms_spi *dev, struct tb100_spi_xfer *xfer)
{
	struct tb100_spi_lli_desc *lli = NULL;

	/* setup spi mode for xfer */
	tb100_spi_setup(xfer->m->spi);

	/* always set SPI_REG_TX_CTRL */
	tb100_spi_writel(dev, xfer->tx_len, SPI_REG_TX_CTRL);

	/* set SPI_REG_TX_DMA_LLP if needed */
	if (xfer->tx_len) {
		lli = list_first_entry(&xfer->tx_queue,
				struct tb100_spi_lli_desc,
				node);

		/* we don't use dma chaining mode for only one tx_queue entry */
		if (list_is_singular(&xfer->tx_queue)) {
			tb100_spi_writel(dev, lli->dmastart,
					SPI_REG_TX_DMA_START);
		} else {
			tb100_spi_writel(dev, (uint32_t) lli->dma_addr,
				SPI_REG_TX_DMA_LLP);
			tb100_spi_writel(dev, 0x439, SPI_REG_TX_DMA_CTRL);
		}
	}

	tb100_spi_dump_lli_queue("TX", xfer->tx_len, &xfer->tx_queue);

	/* always set SPI_REG_RX_CTRL */
	tb100_spi_writel(dev, xfer->rx_len, SPI_REG_RX_CTRL);

	if (xfer->rx_len) {
		lli = list_first_entry(&xfer->rx_queue,
				struct tb100_spi_lli_desc,
				node);

		/* we don't use dma chaining mode for only one rx_queue entry */
		if (list_is_singular(&xfer->rx_queue)) {
			/* set SPI_REG_RX_DMA_START, with dmastart address */
			tb100_spi_writel(dev, lli->dmastart,
					SPI_REG_RX_DMA_START);
		} else {
			/* set SPI_REG_RX_DMA_LLP if needed */
			tb100_spi_writel(dev, (uint32_t) lli->dma_addr,
				SPI_REG_RX_DMA_LLP);
			tb100_spi_writel(dev, 0x439, SPI_REG_RX_DMA_CTRL);
		}
	}

	tb100_spi_dump_lli_queue("RX", xfer->rx_len, &xfer->rx_queue);

	dma_sync_sg_for_device(&dev->master->dev, xfer->sgt_tx.sgl,
			xfer->sgt_tx_nelem, DMA_TO_DEVICE);

	tb100_spi_xfer_start(dev);
	return 0;
}

static void setup_dma_scatter(struct tb100_pms_spi *dev, struct sg_table *sgt,
		const void *buffer, int len)
{
	void *buf = (void *) buffer;
	struct scatterlist *sg;
	int bytesleft = len, mapbytes = 0;
	int i = 0;

	for_each_sg(sgt->sgl, sg, sgt->nents, i) {
		struct page *page = NULL;

		BUG_ON(sg == NULL);

		if (sg_dma_len(sg))
			continue;

		if (bytesleft > 0) {
			if (bytesleft < (PAGE_SIZE - offset_in_page(buf)))
				mapbytes = bytesleft;
			else
				mapbytes = PAGE_SIZE - offset_in_page(buf);

			if (is_vmalloc_addr(buf))
				page = vmalloc_to_page(buf);
			else
				page = virt_to_page(buf);

			/* fill sg entry */
			sg_set_page(sg, page, mapbytes, offset_in_page(buf));

			bytesleft -= mapbytes;
#if TB100_SPI_DEBUG
			printk("%s: i = %d, mapped %d bytes from %p, bytes left: %d\n",
					__func__, i , mapbytes, buf, bytesleft);
#endif
			buf += mapbytes;
		}
	}

	if (bytesleft > 0) {
		printk("%s: i = %d, mapped %d bytes from %p, bytes left: %d\n",
				__func__, i , mapbytes, buf, bytesleft);
		arc_halt();
	}
}

static void tb100_spi_free_dma_sg_list(struct tb100_pms_spi *dev,
		struct tb100_spi_xfer *x)
{
	if (x->sgt_tx_nelem) {
		dma_unmap_sg(&dev->master->dev, x->sgt_tx.sgl,
				x->sgt_tx_nelem, DMA_TO_DEVICE);
		sg_free_table(&x->sgt_tx);
	}

	if (x->sgt_rx_nelem) {
		dma_unmap_sg(&dev->master->dev, x->sgt_rx.sgl,
				x->sgt_rx_nelem, DMA_FROM_DEVICE);
		sg_free_table(&x->sgt_rx);
	}
}

static int tb100_spi_setup_dma_sg(struct tb100_pms_spi *dev,
		struct tb100_spi_xfer *x)
{
	struct spi_transfer *t = NULL;
	int ret;

	if (x->sgt_tx_nelem) {
		ret = sg_alloc_table(&x->sgt_tx, x->sgt_tx_nelem, GFP_KERNEL);
		if (ret)
			goto failed;
	}

	if (x->sgt_rx_nelem) {
		ret = sg_alloc_table(&x->sgt_rx, x->sgt_rx_nelem, GFP_KERNEL);
		if (ret)
			goto failed;
	}

	list_for_each_entry(t, &x->m->transfers, transfer_list) {
		if (t->tx_buf) {
			setup_dma_scatter(dev, &x->sgt_tx, t->tx_buf, t->len);
		}

		if (t->rx_buf) {
			setup_dma_scatter(dev, &x->sgt_rx, t->rx_buf, t->len);
		}
	}

	if (x->sgt_tx_nelem) {
		ret = dma_map_sg(&dev->master->dev, x->sgt_tx.sgl,
				x->sgt_tx_nelem, DMA_TO_DEVICE);
		if(ret != x->sgt_tx_nelem)
			goto failed;
	}

	if (x->sgt_rx_nelem) {
		ret = dma_map_sg(&dev->master->dev, x->sgt_rx.sgl,
				x->sgt_rx_nelem, DMA_FROM_DEVICE);
		if(ret != x->sgt_rx_nelem)
			goto failed;
	}

	return 0;

failed:
	tb100_spi_free_dma_sg_list(dev, x);
	return -ENOMEM;
}

static inline void tb100_spi_fix_dma_xfer(struct tb100_pms_spi *dev)
{
	s8 i;
	u32 data = tb100_spi_readl(dev, SPI_REG_RX_DATA);
	u32 addr = tb100_spi_readl(dev, SPI_REG_RX_DMA_LAST_ADDR);

#if 0 //def TB100_SPI_DEBUG
	printk("%s: fifo data: 0x%08x ; (addr %% 4) = %d\n",
			__func__, data, addr % 4);
#endif

	for (i = (addr % 4); i >= 0; i--) {
		u32 dst = (addr - (addr % 4) + i);
		u8 d = (data >> (i*8));
#if 1 //def TB100_SPI_DEBUG
		printk("%s: write8(0x%08x, 0x%02x)\n", __func__, dst, d);
#endif
		writeb(d, (void *) dst);
#if 0 //def TB100_SPI_DEBUG
		printk("%s:  read8(0x%08x): 0x%02x\n", __func__, dst, readb((void *) dst));
#endif
	}
}

static irqreturn_t tb100_spi_interrupt(int irq, void *ctx)
{
	struct tb100_pms_spi *dev = NULL;
	struct tb100_spi_xfer *x = NULL;
	struct spi_master *master = NULL;

	uint32_t sts;

	if (unlikely(!ctx)) {
		pr_err("%s: invalid dev pointer\n", __func__);
		return IRQ_NONE;
	}

	master = (struct spi_master *) ctx;
	dev = spi_master_get_devdata(master);

	/* read status */
	sts = tb100_spi_readl(dev, SPI_REG_INTR_STATUS);

	if (!list_is_singular(&dev->x_queue)) {
		printk("%s: x_queue list is not singular !!\n", __func__);
		arc_halt();
	}

	if (tb100_spi_readl(dev, SPI_REG_STATUS) == 0x330) {
		printk("spi_it: detected sts = 0x330\n");
		//tb100_spi_dump_registers(dev);
		tb100_spi_fix_dma_xfer(dev);
	}

	if (!list_empty(&dev->x_queue)) {
		x = list_first_entry(&dev->x_queue,
				struct tb100_spi_xfer,
				node);
		if (x == NULL) {
			printk("%s: m is NULL !!\n", __func__);
			arc_halt();
		}

		if (x)
			list_del(&x->node);

	} else {
		printk("%s: list is empty !!\n", __func__);
		arc_halt();
	}

	if (sts & SPI_STAT_XFERR) {
		x->m->status = -EIO;
		x->m->actual_length = 0;
		printk("%s: detected SPI_STAT_XFERR (BUS_ERROR @ 0x%08x)\n",
				__func__,
				tb100_spi_readl(dev, SPI_REG_RX_DMA_LAST_ADDR));
		 //TODO: remove me !
		tb100_spi_dump_registers(dev);
		arc_halt();
	}

	if (sts & SPI_STAT_ENDOFTRANS) {
		x->m->status = 0;
		x->m->actual_length = (x->rx_len + x->tx_len);

		dma_sync_sg_for_cpu(&dev->master->dev, x->sgt_rx.sgl,
				x->sgt_rx_nelem, DMA_FROM_DEVICE);

	}

	/* unlock waiting process */
	x->m->complete(x->m->context);

	/* flush internals xfer queues */
	tb100_spi_flush_xfer_queues(dev, x);

	/* free dma sg list */
	tb100_spi_free_dma_sg_list(dev, x);

	if (x)
		kfree(x);

	/* clear status */
	tb100_spi_writel(dev, sts, SPI_REG_INTR_STATUS);

	return IRQ_HANDLED;
}

static int tb100_spi_transfer(struct spi_device *spi, struct spi_message *m)
{
	struct tb100_pms_spi *dev = NULL;
	struct tb100_spi_xfer *x = NULL;
	struct spi_transfer *t = NULL;
	int i = 0;

	m->actual_length = 0;
	m->status = -EINPROGRESS;

	dev = spi_master_get_devdata(spi->master);

#if TB100_SPI_DEBUG
	printk(">>> %s <<<\n", __func__);
#endif

	/* reject invalid messages and transfers */
	if (list_empty(&m->transfers) || !m->complete)
		return -EINVAL;

	/* allocate spi xfer */
	x = kzalloc(sizeof(*x), GFP_KERNEL);
	if (!x)
		return -ENOMEM;

	/* init internal spi queue */
	INIT_LIST_HEAD(&x->tx_queue);
	INIT_LIST_HEAD(&x->rx_queue);
	/* store spi_message handle */
	x->m = m;

	list_for_each_entry(t, &m->transfers, transfer_list) {

		if (t->bits_per_word < 0 || t->bits_per_word > 32)
			goto failed;

		if (t->len == 0)
			goto failed;

		if (!t->rx_buf && !t->tx_buf)
			goto failed;

#if TB100_SPI_DEBUG
		printk("t[%02d]: tx_buf: %p ; rx_buf: %p ; len: %d",
				i++, t->tx_buf, t->rx_buf, t->len);

#if TB100_SPI_DUMP_XFER
#ifndef MIN
#define MIN(_a, _b) ((_a) < (_b) ? (_a) : (_b))
#endif
		if (t->tx_buf) {
			int j, limit = 5, n;
			u8 *buf = (u8 *) t->tx_buf;
			n = MIN(limit, t->len);
			printk(" >>>");
			for (j = 0; j < n; j++)
				printk(" %02x",  buf[j] );
		}
#endif
		printk("\n");
#endif

		if (t->tx_buf) {
			x->sgt_tx_nelem +=
				DIV_ROUND_UP(
					offset_in_page(t->tx_buf) + t->len,
					PAGE_SIZE);
			x->tx_len += t->len;
		}

		if (t->rx_buf) {
			x->sgt_rx_nelem +=
				DIV_ROUND_UP(
					offset_in_page(t->rx_buf) + t->len,
					PAGE_SIZE);
			x->rx_len += t->len;
		}

	}

	if (!m->is_dma_mapped) {
		if (tb100_spi_setup_dma_sg(dev, x) < 0)
			goto failed;
	} else {
		printk("%s: spi_message is_dma_mapped ! will halt !\n", __func__);
		BUG(); arc_halt();
	}

	if (tb100_spi_configure_dma_xfer(dev, x) < 0)
		goto failed;

	// TODO: lock
	list_add_tail(&x->node, &dev->x_queue);
	// TODO: unlock

	// TODO: replace by workqueue
	if (tb100_spi_dma_xfer(dev, x) == 0)
		return 0;

failed:
	m->status = -EFAULT;
	tb100_spi_flush_xfer_queues(dev, x);

	if (x)
		kfree(x);

	return -EFAULT;
}

#ifdef CONFIG_DEBUG_FS
static int debugfs_lli_desc_dump(struct seq_file *s, void *args)
{
	struct tb100_pms_spi *dev = (struct tb100_pms_spi *) s->private;
	struct tb100_spi_lli_desc *lli;
	int i;

	seq_printf(s, "DMA lli descriptors dump for spi@%p\n", dev->iomem);
	seq_printf(s, "-------------------------------------------------------\n");

	for (i = 0; i < MAX_LLI; i++) {
		lli = dev->lli_desc[i];
		seq_printf(s, "lli[%02d]: cpu_addr: 0x%p ; dma_addr: 0x%08x ; used: %d\n",
				i, lli, lli->dma_addr, lli->used);
	}

	return 0;
}

static int debugfs_show_lli_desc_open(struct inode *inode, struct file *file)
{
	return single_open(file, debugfs_lli_desc_dump, inode->i_private);
}

static const struct file_operations tb100_spi_lli_ops = {
	.open = debugfs_show_lli_desc_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
	.owner = THIS_MODULE,
};

static int tb100_spi_debugfs_init(struct tb100_pms_spi *tspi)
{
	tspi->debugfs = debugfs_create_dir(dev_name(&tspi->master->dev),
			NULL);
	if (!tspi->debugfs)
		return -ENOMEM;

	debugfs_create_file("lli_desc", S_IFREG | S_IRUGO,
		tspi->debugfs, (void *)tspi, &tb100_spi_lli_ops);
	return 0;
}

static void tb100_spi_debugfs_remove(struct tb100_pms_spi *tspi)
{
	if (tspi->debugfs)
		debugfs_remove_recursive(tspi->debugfs);
}
#endif

static int tb100_spi_lli_desc_init(struct tb100_pms_spi *tspi)
{
	int i;

	for (i = 0; i < MAX_LLI; i++) {
		dma_addr_t dma_addr;
		/* alloc lli descriptor */
		tspi->lli_desc[i] = (struct tb100_spi_lli_desc *)
			dma_pool_alloc(tspi->lli_pool, GFP_KERNEL, &dma_addr);

		BUG_ON(tspi->lli_desc[i] == NULL);

		/* store dma address in lli_desc */
		if (tspi->lli_desc[i])
			tspi->lli_desc[i]->dma_addr = dma_addr;
	}

	return 0;
}

static void tb100_spi_lli_desc_free(struct tb100_pms_spi *tspi)
{
	int i;

	for (i = 0; i < MAX_LLI; i++)
		dma_pool_free(tspi->lli_pool,
				tspi->lli_desc[i],
				tspi->lli_desc[i]->dma_addr);
}

static int tb100_spi_probe(struct platform_device *pdev)
{
	struct spi_master *master;
	struct tb100_pms_spi *tspi;
	struct resource *mem;
	static int bus_num = 0;
	uint32_t reg;
	int ret;

	master = spi_alloc_master(&pdev->dev, sizeof(*tspi));
	if (!master) {
		dev_err(&pdev->dev, "spi_alloc_master failed\n");
		return -ENOMEM;
	}

	/* set spi mode capabilities */
	master->mode_bits = SPI_CPOL | SPI_CPHA | SPI_LSB_FIRST |
		SPI_LOOP | SPI_CS_HIGH;

	master->setup = tb100_spi_setup;
	master->transfer = tb100_spi_transfer;

	tspi = spi_master_get_devdata(master);
	dev_set_drvdata(&pdev->dev, tspi);

	tspi->master = master;
	tspi->pdev = pdev;

	tspi->lli_pool = dma_pool_create(dev_name(&pdev->dev), &pdev->dev,
			sizeof(struct tb100_spi_lli_desc), 4, 0);
	if (!tspi->lli_pool) {
		dev_err(&pdev->dev, "dma_pool_create() failed\n");
		ret = -ENOMEM;
		goto err_kfree;
	}

	ret = tb100_spi_lli_desc_init(tspi);
	if (ret)
		goto err_kfree;

	tspi->clk = clk_get(&pdev->dev, NULL);
	if (IS_ERR(tspi->clk)) {
		ret = -ENODEV;
		goto err_kfree;
	}
	clk_enable(tspi->clk);

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!mem) {
		dev_err(&pdev->dev, "SPI memory resources not defined\n");
		ret = -EINVAL;
		goto err_kfree;
	}

	if (!request_mem_region(mem->start, resource_size(mem), DRIVER_NAME)) {
		dev_err(&pdev->dev, "SPI region already claimed\n");
		ret = -EBUSY;
		goto err_kfree;
	}

	tspi->iomem = ioremap_nocache(mem->start, resource_size(mem));
	if (!tspi->iomem) {
		dev_err(&pdev->dev, "SPI region already mapped\n");
		ret = -ENOMEM;
		goto err_free_mem;
	}

	tspi->irq = platform_get_irq(pdev, 0);
	if (tspi->irq < 0) {
		dev_err(&pdev->dev, "SPI irq not defined\n");
		ret = -ENXIO;
		goto err_iounmap;
	}

	ret = request_irq(tspi->irq, tb100_spi_interrupt, 0,
			dev_name(&pdev->dev), &pdev->dev);
	if (unlikely(ret < 0)) {
		dev_err(&pdev->dev, "request IRQ %d failed (error: %d)\n",
		       tspi->irq, ret);
		goto err_iounmap;
	}

	/* init tb100_spi_xfer queue */
	INIT_LIST_HEAD(&tspi->x_queue);

	/* read hw version */
	tspi->hw_info.version = tb100_spi_readl(tspi, SPI_REG_ID);

	/* read hw fifo size */
	reg = tb100_spi_readl(tspi, SPI_REG_PARAM1);
	tspi->hw_info.rx_ff_size = tspi->hw_info.tx_ff_size =
		((reg & 0xFF00) >> 8) * ((reg & 0xFF) / 8);

	/* read hw threshold */
	reg = tb100_spi_readl(tspi, SPI_REG_PARAM2);
	tspi->hw_info.tx_af_level =  ((reg >> 16) & 0xFF);
	tspi->hw_info.tx_ae_level =  ((reg >> 8) & 0xFF);
	tspi->hw_info.rx_af_level =  (reg & 0xFF);

	pr_info("%s enabled @%p (ver: %d.%d.%d)\n", DRIVER_NAME,
			tspi->iomem,
			(tspi->hw_info.version & 0xF00) >> 8,
			(tspi->hw_info.version & 0xF0) >> 4,
			(tspi->hw_info.version & 0x0F));

	/* detect available slave select */
	tb100_spi_writel(tspi, 0xF, SPI_REG_SER);
	reg = tb100_spi_readl(tspi, SPI_REG_SER);
	for_each_set_bit(ret, (const long unsigned int *) &reg, 0xF)
		master->num_chipselect++;

	master->bus_num = bus_num++;

	master->dev.of_node = pdev->dev.of_node;

	/* ready to register */
	ret = spi_register_master(master);
	if (ret < 0) {
		pr_info("spi_register_master failed err: %d\n", ret);
		goto err_irq;
	}

#ifdef CONFIG_DEBUG_FS
	tb100_spi_debugfs_init(tspi);
#endif

	return ret;

err_irq:
	free_irq(tspi->irq, tspi);
err_iounmap:
	iounmap(tspi->iomem);
err_free_mem:
	release_mem_region(mem->start, resource_size(mem));
err_kfree:
	if (tspi->lli_pool) {
		tb100_spi_lli_desc_free(tspi);
		dma_pool_destroy(tspi->lli_pool);
	}
	kfree(master);

	return ret;
}

static int __exit tb100_spi_remove(struct platform_device *pdev)
{
	struct tb100_pms_spi *dev = platform_get_drvdata(pdev);
	struct spi_master *master;
	struct resource *mem;

	printk("%s: enter\n", __func__);

	platform_set_drvdata(pdev, NULL);

	/* clock release */
	clk_disable(dev->clk);
	clk_put(dev->clk);
	dev->clk = NULL;

#ifdef CONFIG_DEBUG_FS
	tb100_spi_debugfs_remove(dev);
#endif

	master = dev->master;
	if (master) {
		spi_unregister_master(dev->master);
		kfree(master);
	}

	free_irq(dev->irq, dev);
	iounmap(dev->iomem);

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	release_mem_region(mem->start, resource_size(mem));

	tb100_spi_lli_desc_free(dev);
	dma_pool_destroy(dev->lli_pool);
	kfree(dev);

	printk("%s: exit\n", __func__);
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id tb100_spi_of_match_table[] = {
	{ .compatible = "abilis,tb100-spi", },
	{}
};
MODULE_DEVICE_TABLE(of, tb100_spi_of_match_table);
#else /* CONFIG_OF */
#define tb100_spi_of_match_table NULL
#endif /* CONFIG_OF */

static struct platform_driver tb100_spi_driver = {
	.driver		= {
		.name	= DRIVER_NAME,
		.owner	= THIS_MODULE,
		.of_match_table = tb100_spi_of_match_table,
	},
	.probe		= tb100_spi_probe,
	.remove		= __exit_p(tb100_spi_remove),
};
module_platform_driver(tb100_spi_driver);

/* modinfo details */
MODULE_ALIAS("platform:tb100-spi");
MODULE_DESCRIPTION("Abilis Systems TB100 SPI controller driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Pierrick Hascoet <pierrick.hascoet@abilis.com>");

/* EOF - vim: set textwidth=80 ts=8 sw=8 sts=8 noet: */
