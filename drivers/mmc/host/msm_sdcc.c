/*
 *  linux/drivers/mmc/host/msm_sdcc.c - Qualcomm MSM 7X00A SDCC Driver
 *
 *  Copyright (C) 2007 Google Inc,
 *  Copyright (C) 2003 Deep Blue Solutions, Ltd, All Rights Reserved.
 *  Copyright (C) 2009, Code Aurora Forum. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Based on mmci.c
 *
 * Author: San Mehat (san@android.com)
 *
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/highmem.h>
#include <linux/log2.h>
#include <linux/mmc/host.h>
#include <linux/mmc/card.h>
#include <linux/mmc/sdio.h>
#include <linux/clk.h>
#include <linux/scatterlist.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/debugfs.h>
#include <linux/io.h>
#include <linux/memory.h>
#include <linux/wakelock.h>

#include <asm/cacheflush.h>
#include <asm/div64.h>
#include <asm/sizes.h>
#include <asm/gpio.h>

#include <asm/mach/mmc.h>
#include <mach/msm_iomap.h>
#include <mach/dma.h>
#include <mach/htc_pwrsink.h>

#include "msm_sdcc.h"
#include "../core/core.h"

#define DRIVER_NAME "msm-sdcc"

#define DBG(host, fmt, args...)	\
	pr_debug("%s: %s: " fmt, mmc_hostname(host->mmc), __func__ , args)

// Andrew 0224
#define IRQ_DEBUG 0
// #define IRQ_DEBUG 1

#if defined(CONFIG_DEBUG_FS)
static void msmsdcc_dbg_createhost(struct msmsdcc_host *);
static struct dentry *debugfs_dir;
#endif

#define BUSCLK_TIMEOUT (HZ)
static unsigned int msmsdcc_fmin = 144000;
static unsigned int msmsdcc_fmax = 50000000;
static unsigned int msmsdcc_4bit = 1;
static unsigned int msmsdcc_pwrsave = 1;
static unsigned int msmsdcc_sdioirq = 1;
static unsigned int msmsdcc_piopoll = 0;
static unsigned long msmsdcc_irqtime = 0;

#define PIO_SPINMAX 30
#define CMD_SPINMAX 20

#define VERBOSE_COMMAND_TIMEOUTS	1

#ifdef CONFIG_WIMAX
extern int supersonic_wimax_get_status(); 
#endif

#if IRQ_DEBUG == 1
static char *irq_status_bits[] = { "cmdcrcfail", "datcrcfail", "cmdtimeout",
				   "dattimeout", "txunderrun", "rxoverrun",
				   "cmdrespend", "cmdsent", "dataend", NULL,
				   "datablkend", "cmdactive", "txactive",
				   "rxactive", "txhalfempty", "rxhalffull",
				   "txfifofull", "rxfifofull", "txfifoempty",
				   "rxfifoempty", "txdataavlbl", "rxdataavlbl",
				   "sdiointr", "progdone", "atacmdcompl",
				   "sdiointrope", "ccstimeout", NULL, NULL,
				   NULL, NULL, NULL };

static void
msmsdcc_print_status(struct msmsdcc_host *host, char *hdr, uint32_t status)
{
	int i;

	printk(KERN_DEBUG "%s-%s ", mmc_hostname(host->mmc), hdr);
	for (i = 0; i < 32; i++) {
		if (status & (1 << i))
			printk("%s ", irq_status_bits[i]);
	}
	printk("\n");
}
#endif

static inline void
msmsdcc_disable_clocks(struct msmsdcc_host *host, int deferr)
{
	WARN_ON(!host->clks_on);

	BUG_ON(host->curr.mrq);

	if (deferr) {
		mod_timer(&host->busclk_timer, jiffies + BUSCLK_TIMEOUT);
	} else {
		del_timer_sync(&host->busclk_timer);
		/* dev_info(mmc_dev(host->mmc), "Immediate clock shutdown\n"); */
		if (host->clks_on) {
			clk_disable(host->clk);
			clk_disable(host->pclk);
			host->clks_on = 0;
		}
	}
}

static inline int
msmsdcc_enable_clocks(struct msmsdcc_host *host)
{
	int rc;

	del_timer_sync(&host->busclk_timer);

	if (!host->clks_on) {
		rc = clk_enable(host->pclk);
		if (rc)
			return rc;
		rc = clk_enable(host->clk);
		if (rc) {
			clk_disable(host->pclk);
			return rc;
		}
		udelay(1 + ((3 * USEC_PER_SEC) /
		       (host->clk_rate ? host->clk_rate : msmsdcc_fmin)));
		host->clks_on = 1;
	}
	return 0;
}

static inline unsigned int
msmsdcc_readl(struct msmsdcc_host *host, unsigned int reg)
{
	return readl(host->base + reg);
}

static inline void
msmsdcc_writel(struct msmsdcc_host *host, u32 data, unsigned int reg)
{
	writel(data, host->base + reg);
	/* 3 clk delay required! */
	udelay(1 + ((3 * USEC_PER_SEC) /
	       (host->clk_rate ? host->clk_rate : msmsdcc_fmin)));
}

static void
msmsdcc_start_command(struct msmsdcc_host *host, struct mmc_command *cmd,
		      u32 c);

static int is_sd_platform(struct mmc_platform_data *plat)
{
	if (plat->slot_type)
		if (*plat->slot_type == MMC_TYPE_SD)
			return 1;

	return 0;
}

static int is_mmc_platform(struct mmc_platform_data *plat)
{
	if (plat->slot_type)
		if (*plat->slot_type == MMC_TYPE_MMC)
			return 1;

	return 0;
}

static char *mmc_type_str(unsigned int slot_type)
{
	switch (slot_type) {
	case MMC_TYPE_MMC:
		return "MMC";

	case MMC_TYPE_SD:
		return "SD";

	case MMC_TYPE_SDIO:
		return "SDIO";

	default:
		return "Unknown type";
	}
}

static void
msmsdcc_request_end(struct msmsdcc_host *host, struct mmc_request *mrq)
{
	BUG_ON(host->curr.data);

	host->curr.mrq = NULL;
	host->curr.cmd = NULL;

	if (mrq->data)
		mrq->data->bytes_xfered = host->curr.data_xfered;
	if (mrq->cmd->error == -ETIMEDOUT)
		mdelay(5);

#ifdef CONFIG_MMC_BUSCLK_PWRSAVE
#ifdef CONFIG_WIMAX
	if (!supersonic_wimax_get_status())              
#endif
	       msmsdcc_disable_clocks(host, 1);
#endif

	/*
	 * Need to drop the host lock here; mmc_request_done may call
	 * back into the driver...
	 */
	spin_unlock(&host->lock);
	mmc_request_done(host->mmc, mrq);
	spin_lock(&host->lock);
}

static void
msmsdcc_stop_data(struct msmsdcc_host *host)
{
	host->curr.data = NULL;
	host->curr.got_dataend = host->curr.got_datablkend = 0;
}

uint32_t msmsdcc_fifo_addr(struct msmsdcc_host *host)
{
	if (host->pdev_id == 1)
		return MSM_SDC1_PHYS + MMCIFIFO;
	else if (host->pdev_id == 2)
		return MSM_SDC2_PHYS + MMCIFIFO;
	else if (host->pdev_id == 3)
		return MSM_SDC3_PHYS + MMCIFIFO;
	else if (host->pdev_id == 4)
		return MSM_SDC4_PHYS + MMCIFIFO;
	else
		BUG();
	return 0;
}

static inline void
msmsdcc_start_command_exec(struct msmsdcc_host *host, u32 arg, u32 c) {
       msmsdcc_writel(host, arg, MMCIARGUMENT);
       msmsdcc_writel(host, c, MMCICOMMAND);
}

static void
msmsdcc_dma_exec_func(struct msm_dmov_cmd *cmd)
{
	struct msmsdcc_host *host = (struct msmsdcc_host *)cmd->data;

	msmsdcc_writel(host, host->cmd_timeout, MMCIDATATIMER);
	msmsdcc_writel(host, (unsigned int)host->curr.xfer_size,
		       MMCIDATALENGTH);
	msmsdcc_writel(host, host->cmd_pio_irqmask, MMCIMASK1);
	msmsdcc_writel(host, host->cmd_datactrl, MMCIDATACTRL);

	if (host->cmd_cmd) {
		msmsdcc_start_command_exec(host,
					   (u32) host->cmd_cmd->arg,
					   (u32) host->cmd_c);
	}
	host->dma.active = 1;
}

static void
msmsdcc_dma_complete_func(struct msm_dmov_cmd *cmd,
			  unsigned int result,
			  struct msm_dmov_errdata *err)
{
	struct msmsdcc_dma_data	*dma_data =
		container_of(cmd, struct msmsdcc_dma_data, hdr);
	struct msmsdcc_host	*host = dma_data->host;
	unsigned long		flags;
	struct mmc_request	*mrq;

	spin_lock_irqsave(&host->lock, flags);
	host->dma.active = 0;

	mrq = host->curr.mrq;
	BUG_ON(!mrq);
	WARN_ON(!mrq->data);

	if (!(result & DMOV_RSLT_VALID)) {
		printk(KERN_ERR "msmsdcc: Invalid DataMover result\n");
		goto out;
	}

	if (result & DMOV_RSLT_DONE) {
		host->curr.data_xfered = host->curr.xfer_size;
	} else {
		/* Error or flush  */
		if (result & DMOV_RSLT_ERROR)
			printk(KERN_ERR "%s: DMA error (0x%.8x)\n",
			       mmc_hostname(host->mmc), result);
		if (result & DMOV_RSLT_FLUSH)
			printk(KERN_ERR "%s: DMA channel flushed (0x%.8x)\n",
			       mmc_hostname(host->mmc), result);
		if (err)
			printk(KERN_ERR
			       "Flush data: %.8x %.8x %.8x %.8x %.8x %.8x\n",
			       err->flush[0], err->flush[1], err->flush[2],
			       err->flush[3], err->flush[4], err->flush[5]);
		if (!mrq->data->error)
			mrq->data->error = -EIO;
	}
	dma_unmap_sg(mmc_dev(host->mmc), host->dma.sg, host->dma.num_ents,
		     host->dma.dir);

	if (host->curr.user_pages) {
		struct scatterlist *sg = host->dma.sg;
		int i;

		for (i = 0; i < host->dma.num_ents; i++, sg++)
			flush_dcache_page(sg_page(sg));
	}

	host->dma.sg = NULL;
	host->dma.busy = 0;

	if ((host->curr.got_dataend && host->curr.got_datablkend)
             || mrq->data->error) {

		/*
		 * If we've already gotten our DATAEND / DATABLKEND
		 * for this request, then complete it through here.
		 */
		msmsdcc_stop_data(host);

		if (!mrq->data->error)
			host->curr.data_xfered = host->curr.xfer_size;
		if (!mrq->data->stop || mrq->cmd->error) {
			host->curr.mrq = NULL;
			host->curr.cmd = NULL;
			mrq->data->bytes_xfered = host->curr.data_xfered;

			spin_unlock_irqrestore(&host->lock, flags);
#ifdef CONFIG_MMC_BUSCLK_PWRSAVE
#ifdef CONFIG_WIMAX
			if (!supersonic_wimax_get_status())              
#endif
			   msmsdcc_disable_clocks(host, 1);
#endif
			mmc_request_done(host->mmc, mrq);
			return;
		} else
			msmsdcc_start_command(host, mrq->data->stop, 0);
	}

out:
	spin_unlock_irqrestore(&host->lock, flags);
	return;
}

static int validate_dma(struct msmsdcc_host *host, struct mmc_data *data)
{
	if (host->dma.channel == -1)
		return -ENOENT;

	if ((data->blksz * data->blocks) < MCI_FIFOSIZE)
		return -EINVAL;
	if ((data->blksz * data->blocks) % MCI_FIFOSIZE)
		return -EINVAL;
	return 0;
}

static int msmsdcc_config_dma(struct msmsdcc_host *host, struct mmc_data *data)
{
	struct msmsdcc_nc_dmadata *nc;
	dmov_box *box;
	uint32_t rows;
	uint32_t crci;
	unsigned int n;
	int i, rc;
	struct scatterlist *sg = data->sg;

	rc = validate_dma(host, data);
	if (rc)
		return rc;

	host->dma.sg = data->sg;
	host->dma.num_ents = data->sg_len;

	BUG_ON(host->dma.num_ents > NR_SG); /* Prevent memory corruption */

	nc = host->dma.nc;

	if (host->pdev_id == 1)
		crci = MSMSDCC_CRCI_SDC1;
	else if (host->pdev_id == 2)
		crci = MSMSDCC_CRCI_SDC2;
	else if (host->pdev_id == 3)
		crci = MSMSDCC_CRCI_SDC3;
	else if (host->pdev_id == 4)
		crci = MSMSDCC_CRCI_SDC4;
	else {
		host->dma.sg = NULL;
		host->dma.num_ents = 0;
		return -ENOENT;
	}

	if (data->flags & MMC_DATA_READ)
		host->dma.dir = DMA_FROM_DEVICE;
	else
		host->dma.dir = DMA_TO_DEVICE;

	/* host->curr.user_pages = (data->flags & MMC_DATA_USERPAGE); */
	host->curr.user_pages = 0;

	box = &nc->cmd[0];
	for (i = 0; i < host->dma.num_ents; i++) {
		box->cmd = CMD_MODE_BOX;

		/* Initialize sg dma address */
		sg->dma_address = page_to_dma(mmc_dev(host->mmc), sg_page(sg))
							+ sg->offset;

		if (i == (host->dma.num_ents - 1))
			box->cmd |= CMD_LC;
		rows = (sg_dma_len(sg) % MCI_FIFOSIZE) ?
			(sg_dma_len(sg) / MCI_FIFOSIZE) + 1 :
			(sg_dma_len(sg) / MCI_FIFOSIZE) ;

		if (data->flags & MMC_DATA_READ) {
			box->src_row_addr = msmsdcc_fifo_addr(host);
			box->dst_row_addr = sg_dma_address(sg);

			box->src_dst_len = (MCI_FIFOSIZE << 16) |
					   (MCI_FIFOSIZE);
			box->row_offset = MCI_FIFOSIZE;

			box->num_rows = rows * ((1 << 16) + 1);
			box->cmd |= CMD_SRC_CRCI(crci);
		} else {
			box->src_row_addr = sg_dma_address(sg);
			box->dst_row_addr = msmsdcc_fifo_addr(host);

			box->src_dst_len = (MCI_FIFOSIZE << 16) |
					   (MCI_FIFOSIZE);
			box->row_offset = (MCI_FIFOSIZE << 16);

			box->num_rows = rows * ((1 << 16) + 1);
			box->cmd |= CMD_DST_CRCI(crci);
		}
		box++;
		sg++;
	}

	/* location of command block must be 64 bit aligned */
	BUG_ON(host->dma.cmd_busaddr & 0x07);

	nc->cmdptr = (host->dma.cmd_busaddr >> 3) | CMD_PTR_LP;
	host->dma.hdr.cmdptr = DMOV_CMD_PTR_LIST |
			       DMOV_CMD_ADDR(host->dma.cmdptr_busaddr);
	host->dma.hdr.complete_func = msmsdcc_dma_complete_func;

	n = dma_map_sg(mmc_dev(host->mmc), host->dma.sg,
			host->dma.num_ents, host->dma.dir);
	/* dsb inside dma_map_sg will write nc out to mem as well */

	if (n != host->dma.num_ents) {
		printk(KERN_ERR "%s: Unable to map in all sg elements\n",
			mmc_hostname(host->mmc));
		host->dma.sg = NULL;
		host->dma.num_ents = 0;
		return -ENOMEM;
	}

	return 0;
}

static int
snoop_cccr_abort(struct mmc_command *cmd)
{
	if ((cmd->opcode == 52) &&
	    (cmd->arg & 0x80000000) &&
	    (((cmd->arg >> 9) & 0x1ffff) == SDIO_CCCR_ABORT))
		return 1;
	return 0;
}

static void
msmsdcc_start_command_deferred(struct msmsdcc_host *host,
				struct mmc_command *cmd, u32 *c)
{
	DBG(host, "op %02x arg %08x flags %08x\n",
		cmd->opcode, cmd->arg, cmd->flags);

	*c |= (cmd->opcode | MCI_CPSM_ENABLE);

	if (cmd->flags & MMC_RSP_PRESENT) {
		if (cmd->flags & MMC_RSP_136)
			*c |= MCI_CPSM_LONGRSP;
		*c |= MCI_CPSM_RESPONSE;
	}

	if (/*interrupt*/0)
		*c |= MCI_CPSM_INTERRUPT;

	if ((((cmd->opcode == 17) || (cmd->opcode == 18))  ||
	     ((cmd->opcode == 24) || (cmd->opcode == 25))) ||
	      (cmd->opcode == 53))
		*c |= MCI_CSPM_DATCMD;

#ifdef CONFIG_MMC_BLOCK_PROG_ENA
	if (host->prog_scan && (cmd->opcode == 12)) {
		*c |= MCI_CPSM_PROGENA;
		host->prog_enable = 1;
	}
#endif

	if (cmd == cmd->mrq->stop)
		*c |= MCI_CSPM_MCIABORT;

	if (snoop_cccr_abort(cmd))
		*c |= MCI_CSPM_MCIABORT;

	host->curr.cmd = cmd;
}

static void
msmsdcc_start_data(struct msmsdcc_host *host, struct mmc_data *data,
			struct mmc_command *cmd, u32 c)
{
	unsigned int datactrl, timeout;
	unsigned long long clks;
	unsigned int pio_irqmask = 0;

	host->curr.data = data;
	host->curr.xfer_size = data->blksz * data->blocks;
	host->curr.xfer_remain = host->curr.xfer_size;
	host->curr.data_xfered = 0;
	host->curr.got_dataend = 0;
	host->curr.got_datablkend = 0;

	memset(&host->pio, 0, sizeof(host->pio));

	datactrl = MCI_DPSM_ENABLE | (data->blksz << 4);

	if (!msmsdcc_config_dma(host, data))
		datactrl |= MCI_DPSM_DMAENABLE;
	else {
		host->pio.sg = data->sg;
		host->pio.sg_len = data->sg_len;
		host->pio.sg_off = 0;

		if (data->flags & MMC_DATA_READ) {
			pio_irqmask = MCI_RXFIFOHALFFULLMASK;
			if (host->curr.xfer_remain < MCI_FIFOSIZE)
				pio_irqmask |= MCI_RXDATAAVLBLMASK;
		} else
			pio_irqmask = MCI_TXFIFOHALFEMPTYMASK;
	}

	if (data->flags & MMC_DATA_READ)
		datactrl |= MCI_DPSM_DIRECTION;

	clks = (unsigned long long)data->timeout_ns * host->clk_rate;
	do_div(clks, 1000000000UL);
	timeout = data->timeout_clks + (unsigned int)clks*2 ;

	if (datactrl & MCI_DPSM_DMAENABLE) {
		/* Save parameters for the exec function */
		host->cmd_timeout = timeout;
		host->cmd_pio_irqmask = pio_irqmask;
		host->cmd_datactrl = datactrl;
		host->cmd_cmd = cmd;

		host->dma.hdr.execute_func = msmsdcc_dma_exec_func;
		host->dma.hdr.data = (void *)host;
		host->dma.busy = 1;

		if (cmd) {
			msmsdcc_start_command_deferred(host, cmd, &c);
			host->cmd_c = c;
		}
		msm_dmov_enqueue_cmd(host->dma.channel, &host->dma.hdr);
		if (data->flags & MMC_DATA_WRITE)
			host->prog_scan = 1;
	} else {
		msmsdcc_writel(host, timeout, MMCIDATATIMER);

		msmsdcc_writel(host, host->curr.xfer_size, MMCIDATALENGTH);

		msmsdcc_writel(host, pio_irqmask, MMCIMASK1);
		msmsdcc_writel(host, datactrl, MMCIDATACTRL);

		if (cmd) {
			/* Daisy-chain the command if requested */
			msmsdcc_start_command(host, cmd, c);
		}
	}
}

static void
msmsdcc_start_command(struct msmsdcc_host *host, struct mmc_command *cmd, u32 c)
{
	/* Workaround for old firmware of PHISON's controller */
	if ((cmd->opcode == 12) &&
	    (cmd->mrq->data->flags & MMC_DATA_WRITE) &&
	    (cmd->mrq->data->blocks > 64)) {
		int count = 0;
		while (++count < WAIT_DAT0_HIGH_MAX) {
			if (gpio_get_value(67))  /* DAT0 */
				break;
			else
				udelay(300);
		}
	}

	host->curr.cmd = cmd;

	host->stats.cmds++;

	msmsdcc_start_command_deferred(host, cmd, &c);
	msmsdcc_start_command_exec(host, cmd->arg, c);
}

static void
msmsdcc_data_err(struct msmsdcc_host *host, struct mmc_data *data,
		 unsigned int status)
{
	if (status & MCI_DATACRCFAIL) {
		printk(KERN_ERR "%s: Data CRC error\n",
		       mmc_hostname(host->mmc));
		printk(KERN_ERR "%s: opcode 0x%.8x\n", __func__,
		       data->mrq->cmd->opcode);
		printk(KERN_ERR "%s: blksz %d, blocks %d\n", __func__,
		       data->blksz, data->blocks);
		data->error = -EILSEQ;
	} else if (status & MCI_DATATIMEOUT) {
		printk(KERN_ERR "%s: Data timeout (cmd=%2u mci_st=%08x)\n",
		       mmc_hostname(host->mmc), data->mrq->cmd->opcode, status);
		data->error = -ETIMEDOUT;
	} else if (status & MCI_RXOVERRUN) {
		printk(KERN_ERR "%s: RX overrun\n", mmc_hostname(host->mmc));
		data->error = -EIO;
	} else if (status & MCI_TXUNDERRUN) {
		printk(KERN_ERR "%s: TX underrun\n", mmc_hostname(host->mmc));
		data->error = -EIO;
	} else {
		printk(KERN_ERR "%s: Unknown error (0x%.8x)\n",
		      mmc_hostname(host->mmc), status);
		data->error = -EIO;
	}
}


static int
msmsdcc_pio_read(struct msmsdcc_host *host, char *buffer, unsigned int remain)
{
	uint32_t	*ptr = (uint32_t *) buffer;
	int		count = 0;

#ifdef CONFIG_MACH_SUPERSONIC
	unsigned int val = 0; //For 2 bytes data access and consider normal 4 bytes SDIO alignment

	/* For 2 bytes data access and consider normal 4 bytes SDIO alignment */
	//if ( (machine_is_supersonic()) && (remain < 4)){
	if ( (remain < 4) ) {
		val = msmsdcc_readl(host, MMCIFIFO);
		memcpy(ptr, &val, remain);
		count += remain;
	}else
#endif
		while (msmsdcc_readl(host, MMCISTATUS) & MCI_RXDATAAVLBL) {
			*ptr = msmsdcc_readl(host, MMCIFIFO + (count % MCI_FIFOSIZE));
			ptr++;
			count += sizeof(uint32_t);

			remain -=  sizeof(uint32_t);
			if (remain == 0)
				break;
		}

	return count;
}

static int
msmsdcc_pio_write(struct msmsdcc_host *host, char *buffer,
		  unsigned int remain, u32 status)
{
	void __iomem *base = host->base;
	char *ptr = buffer;

#ifdef CONFIG_MACH_SUPERSONIC
	unsigned int val = 0; //For 2 bytes data access and consider normal 4 bytes SDIO alignment

	/* For 2 bytes data access and consider normal 4 bytes SDIO alignment */
	//if ((machine_is_supersonic()) && (remain < 4)) {
	if ( (remain < 4) ) {
		memcpy(&val, ptr, remain);
		writel(val, base + MMCIFIFO);
		// check the data end
		do {
			status = readl(base + MMCISTATUS);
		}while (!(status & MCI_DATABLOCKEND));

		return remain;
	} else {
#endif
		do {
			unsigned int count, maxcnt;

			maxcnt = status & MCI_TXFIFOEMPTY ? MCI_FIFOSIZE :
							MCI_FIFOHALFSIZE;
			count = min(remain, maxcnt);

			writesl(base + MMCIFIFO, ptr, count >> 2);
			ptr += count;
			remain -= count;

			if (remain == 0)
				break;

			status = msmsdcc_readl(host, MMCISTATUS);
		} while (status & MCI_TXFIFOHALFEMPTY);

		return ptr - buffer;
#ifdef CONFIG_MACH_SUPERSONIC
	}
#endif
}

static int
msmsdcc_spin_on_status(struct msmsdcc_host *host, uint32_t mask, int maxspin)
{
	while (maxspin) {
		if ((msmsdcc_readl(host, MMCISTATUS) & mask))
			return 0;
		udelay(1);
		--maxspin;
	}
	return -ETIMEDOUT;
}

static int
msmsdcc_pio_irq(int irq, void *dev_id)
{
	struct msmsdcc_host	*host = dev_id;
	uint32_t		status;

	status = msmsdcc_readl(host, MMCISTATUS);
#if IRQ_DEBUG
	msmsdcc_print_status(host, "irq1-r", status);
#endif

	/* Workaround when we found sg is NULL (SST) */
	if (host->pio.sg == NULL) {
		printk(KERN_INFO "%s: pio scatter list is null - ", mmc_hostname(host->mmc));

		if (status & MCI_RXACTIVE) {
			int read_cnt = 0;
			while (msmsdcc_readl(host, MMCISTATUS) & MCI_RXDATAAVLBL) {
				msmsdcc_readl(host, MMCIFIFO + (read_cnt % MCI_FIFOSIZE));
				read_cnt += sizeof(uint32_t);
				if ((read_cnt) > MCI_FIFOSIZE)
					break;
			}
			msmsdcc_writel(host, MCI_RXDATAAVLBLMASK, MMCIMASK1);
			printk("RX\n");
		}
		if (status & MCI_TXACTIVE) {
			struct mmc_request *mrq;

			msmsdcc_writel(host, 0, MMCIMASK1);
			mrq = host->curr.mrq;
			if (mrq) {
				mrq->data->error = -EIO;
				if (mrq->done)
					mrq->done(mrq);
				host->curr.mrq = NULL;
			}
			printk("TX\n");
		}

		return IRQ_HANDLED;
	}

	do {
		unsigned long flags;
		unsigned int remain, len;
		char *buffer;

		if (!(status & (MCI_TXFIFOHALFEMPTY | MCI_RXDATAAVLBL))) {
			if (host->curr.xfer_remain == 0 || !msmsdcc_piopoll)
				break;

			if (msmsdcc_spin_on_status(host,
						   (MCI_TXFIFOHALFEMPTY |
						   MCI_RXDATAAVLBL),
						   PIO_SPINMAX)) {
				break;
			}
		}

		/* Map the current scatter buffer */
		local_irq_save(flags);
		buffer = kmap_atomic(sg_page(host->pio.sg),
				     KM_BIO_SRC_IRQ) + host->pio.sg->offset;
		buffer += host->pio.sg_off;
		remain = host->pio.sg->length - host->pio.sg_off;
		len = 0;
		if (status & MCI_RXACTIVE)
			len = msmsdcc_pio_read(host, buffer, remain);
		if (status & MCI_TXACTIVE)
			len = msmsdcc_pio_write(host, buffer, remain, status);

		/* Unmap the buffer */
		kunmap_atomic(buffer, KM_BIO_SRC_IRQ);
		local_irq_restore(flags);

		host->pio.sg_off += len;
		host->curr.xfer_remain -= len;
		host->curr.data_xfered += len;
		remain -= len;

		if (remain == 0) {
			/* This sg page is full - do some housekeeping */
			if (status & MCI_RXACTIVE && host->curr.user_pages)
				flush_dcache_page(sg_page(host->pio.sg));

			if (!--host->pio.sg_len) {
				memset(&host->pio, 0, sizeof(host->pio));
				break;
			}

			/* Advance to next sg */
			host->pio.sg++;
			host->pio.sg_off = 0;
		}

		status = msmsdcc_readl(host, MMCISTATUS);
	} while (1);

	if (status & MCI_RXACTIVE && host->curr.xfer_remain < MCI_FIFOSIZE)
		msmsdcc_writel(host, MCI_RXDATAAVLBLMASK, MMCIMASK1);

	if (!host->curr.xfer_remain)
		msmsdcc_writel(host, 0, MMCIMASK1);

	return IRQ_HANDLED;
}

static void msmsdcc_do_cmdirq(struct msmsdcc_host *host, uint32_t status)
{
	struct mmc_command *cmd = host->curr.cmd;

	host->curr.cmd = NULL;
	cmd->resp[0] = msmsdcc_readl(host, MMCIRESPONSE0);
	cmd->resp[1] = msmsdcc_readl(host, MMCIRESPONSE1);
	cmd->resp[2] = msmsdcc_readl(host, MMCIRESPONSE2);
	cmd->resp[3] = msmsdcc_readl(host, MMCIRESPONSE3);

	if (status & MCI_CMDTIMEOUT) {
#if VERBOSE_COMMAND_TIMEOUTS
		printk(KERN_ERR "%s: Command timeout (cmd=%2u mci_st=%08x)\n",
		       mmc_hostname(host->mmc), cmd->opcode, status);
#endif
		cmd->error = -ETIMEDOUT;
	} else if (status & MCI_CMDCRCFAIL &&
		   cmd->flags & MMC_RSP_CRC) {
		printk(KERN_ERR "%s: Command CRC error (cmd=%2u mci_st=%08x)\n",
		       mmc_hostname(host->mmc), cmd->opcode, status);
		cmd->error = -EILSEQ;
	}

	if (!cmd->data || cmd->error) {
		if (host->curr.data && host->dma.sg)
			msm_dmov_stop_cmd(host->dma.channel,
					  &host->dma.hdr, 0);
		else if (host->curr.data) { /* Non DMA */
			msmsdcc_stop_data(host);
			msmsdcc_request_end(host, cmd->mrq);
		} else { /* host->data == NULL */
			if (!cmd->error && host->prog_enable) {
				if (status & MCI_PROGDONE) {
					host->prog_scan = 0;
					host->prog_enable = 0;
					msmsdcc_request_end(host, cmd->mrq);
				} else
					host->curr.cmd = cmd;
			} else {
				if (host->prog_enable) {
					host->prog_scan = 0;
					host->prog_enable = 0;
				}
				msmsdcc_request_end(host, cmd->mrq);
			}
		}
	} else if (cmd->data)
		if (!(cmd->data->flags & MMC_DATA_READ))
			msmsdcc_start_data(host, cmd->data, NULL, 0);
}

static irqreturn_t
msmsdcc_irq(int irq, void *dev_id)
{
	struct msmsdcc_host	*host = dev_id;
	u32			status;
	int			ret = 0;
	int			cardint = 0;

	spin_lock(&host->lock);

	do {
		struct mmc_data *data;
		status = msmsdcc_readl(host, MMCISTATUS);

#if IRQ_DEBUG
		msmsdcc_print_status(host, "irq0-r", status);
#endif

		status &= (msmsdcc_readl(host, MMCIMASK0) |
					      MCI_DATABLOCKENDMASK);
		msmsdcc_writel(host, status, MMCICLEAR);

		if (status & MCI_SDIOINTR)
			status &= ~MCI_SDIOINTR;
#if IRQ_DEBUG
		msmsdcc_print_status(host, "irq0-p", status);
#endif
		if (!status)
			break;

		data = host->curr.data;
		if (status & (MCI_CMDSENT | MCI_CMDRESPEND | MCI_CMDCRCFAIL |
			      MCI_CMDTIMEOUT | MCI_PROGDONE) && host->curr.cmd) {
			msmsdcc_do_cmdirq(host, status);
		}

		if (data) {
			/* Check for data errors */
			if (status & (MCI_DATACRCFAIL|MCI_DATATIMEOUT|
				      MCI_TXUNDERRUN|MCI_RXOVERRUN)) {
				msmsdcc_data_err(host, data, status);
				host->curr.data_xfered = 0;
				if (host->dma.sg)
					msm_dmov_stop_cmd(host->dma.channel,
							  &host->dma.hdr, 0);
				else {
					if (host->curr.data)
						msmsdcc_stop_data(host);
					if (!data->stop)
						msmsdcc_request_end(host,
								    data->mrq);
					else
						msmsdcc_start_command(host,
								     data->stop,
								     0);
				}
			}

			/* Check for data done */
			if (!host->curr.got_dataend && (status & MCI_DATAEND))
				host->curr.got_dataend = 1;

			if (!host->curr.got_datablkend &&
			    (status & MCI_DATABLOCKEND)) {
				host->curr.got_datablkend = 1;
			}

			if (host->curr.got_dataend &&
			    host->curr.got_datablkend) {
				/*
				 * If DMA is still in progress, we complete
				 * via the completion handler
				 */
				if (!host->dma.busy) {
					/*
					 * There appears to be an issue in the
					 * controller where if you request a
					 * small block transfer (< fifo size),
					 * you may get your DATAEND/DATABLKEND
					 * irq without the PIO data irq.
					 *
					 * Check to see if theres still data
					 * to be read, and simulate a PIO irq.
					 */
					if (msmsdcc_readl(host, MMCISTATUS) &
							       MCI_RXDATAAVLBL)
						msmsdcc_pio_irq(1, host);

					msmsdcc_stop_data(host);
					if (!data->error)
						host->curr.data_xfered =
							host->curr.xfer_size;

					if (!data->stop)
						msmsdcc_request_end(host,
								    data->mrq);
					else
						msmsdcc_start_command(host,
							      data->stop, 0);
				}
			}
		}

		if (status & MCI_SDIOINTOPER) {
			cardint = 1;
			status &= ~MCI_SDIOINTOPER;
		}
		ret = 1;
	} while (status);

	spin_unlock(&host->lock);

	/*
	 * We have to delay handling the card interrupt as it calls
	 * back into the driver.
	 */
	if (cardint)
		mmc_signal_sdio_irq(host->mmc);

	return IRQ_RETVAL(ret);
}

static void
msmsdcc_request(struct mmc_host *mmc, struct mmc_request *mrq)
{
	struct msmsdcc_host *host = mmc_priv(mmc);
	unsigned long		flags;

	WARN_ON(host->curr.mrq != NULL);

	WARN_ON(host->pwr == 0);

	spin_lock_irqsave(&host->lock, flags);

	host->stats.reqs++;

	if (host->eject) {
		if (mrq->data && !(mrq->data->flags & MMC_DATA_READ)) {
			mrq->cmd->error = 0;
			mrq->data->bytes_xfered = mrq->data->blksz *
						  mrq->data->blocks;
		} else
			mrq->cmd->error = -ENOMEDIUM;

		spin_unlock_irqrestore(&host->lock, flags);
		mmc_request_done(mmc, mrq);
		return;
	}

	msmsdcc_enable_clocks(host);

	host->curr.mrq = mrq;

	if (mrq->data && mrq->data->flags & MMC_DATA_READ)
		/* Queue/read data, daisy-chain command when data starts */
		msmsdcc_start_data(host, mrq->data, mrq->cmd, 0);
	else
		msmsdcc_start_command(host, mrq->cmd, 0);

	if (host->cmdpoll && !msmsdcc_spin_on_status(host,
				MCI_CMDRESPEND|MCI_CMDCRCFAIL|MCI_CMDTIMEOUT,
				CMD_SPINMAX)) {
		uint32_t status = msmsdcc_readl(host, MMCISTATUS);
		msmsdcc_do_cmdirq(host, status);
		msmsdcc_writel(host,
			       MCI_CMDRESPEND | MCI_CMDCRCFAIL | MCI_CMDTIMEOUT,
			       MMCICLEAR);
		host->stats.cmdpoll_hits++;
	} else {
		host->stats.cmdpoll_misses++;
	}
	spin_unlock_irqrestore(&host->lock, flags);
}

static void
msmsdcc_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct msmsdcc_host *host = mmc_priv(mmc);
	u32 clk = 0, pwr = 0;
	int rc;
	unsigned long flags;

	spin_lock_irqsave(&host->lock, flags);

#ifdef CONFIG_MMC_BUSCLK_PWRSAVE
	if (host->clks_on)
#ifdef CONFIG_WIMAX
			if (!supersonic_wimax_get_status())              
#endif
		      del_timer_sync(&host->busclk_timer);
#endif

	if (ios->clock) {
		if (!host->clks_on)
			msmsdcc_enable_clocks(host);

		if (ios->clock != host->clk_rate) {
			rc = clk_set_rate(host->clk, ios->clock);
			if (rc < 0)
				printk(KERN_ERR
				       "Error setting clock rate (%d)\n", rc);
			else
				host->clk_rate = ios->clock;
		}
		clk |= MCI_CLK_ENABLE;
	}

	if (ios->bus_width == MMC_BUS_WIDTH_4)
		clk |= (2 << 10); /* Set WIDEBUS */

	if (ios->bus_width == MMC_BUS_WIDTH_8)
		clk |= (3 << 10); /* Set WIDEBUS */

	if (ios->clock > 400000 && msmsdcc_pwrsave)
		clk |= (1 << 9); /* PWRSAVE */

	clk |= (1 << 12); /* FLOW_ENA */
	clk |= (1 << 15); /* feedback clock */

	if (host->plat->translate_vdd)
		pwr |= host->plat->translate_vdd(mmc_dev(mmc), ios->vdd);

	switch (ios->power_mode) {
	case MMC_POWER_OFF:
		htc_pwrsink_set(PWRSINK_SDCARD, 0);
		break;
	case MMC_POWER_UP:
		pwr |= MCI_PWR_UP;
		break;
	case MMC_POWER_ON:
		htc_pwrsink_set(PWRSINK_SDCARD, 100);
		pwr |= MCI_PWR_ON;
		break;
	}

	if (ios->bus_mode == MMC_BUSMODE_OPENDRAIN)
		pwr |= MCI_OD;

	msmsdcc_writel(host, clk, MMCICLOCK);

	if (host->pwr != pwr) {
		host->pwr = pwr;
		msmsdcc_writel(host, pwr, MMCIPOWER);
	}

	if (!(clk & MCI_CLK_ENABLE) && host->clks_on)
		msmsdcc_disable_clocks(host, 0);

#ifdef CONFIG_MMC_BUSCLK_PWRSAVE
	if (host->clks_on)
#ifdef CONFIG_WIMAX
		if (!supersonic_wimax_get_status())              
#endif
		     msmsdcc_disable_clocks(host, 1);
#endif

	spin_unlock_irqrestore(&host->lock, flags);
}

static void msmsdcc_enable_sdio_irq(struct mmc_host *mmc, int enable)
{
	struct msmsdcc_host *host = mmc_priv(mmc);
	unsigned long flags;
	u32 status;

	/* printk("%s: %d\n", __FUNCTION__, enable); */
	spin_lock_irqsave(&host->lock, flags);
	if (msmsdcc_sdioirq == 1) {
		status = msmsdcc_readl(host, MMCIMASK0);
		if (enable)
			status |= MCI_SDIOINTOPERMASK;
		else
			status &= ~MCI_SDIOINTOPERMASK;
		host->saved_irq0mask = status;
		msmsdcc_writel(host, status, MMCIMASK0);
	}
	spin_unlock_irqrestore(&host->lock, flags);
}

/*
 * This callback function is for mmc/core/core.c mmc_rescan.
 * Since Wifi has concern, we disable it currently.
 */
/*
static int msmsdcc_get_cd(struct mmc_host *mmc)
{
	struct msmsdcc_host *host = mmc_priv(mmc);

	if (host->plat->status)
		return host->plat->status(mmc_dev(mmc));

	return 1;
}
*/

static const struct mmc_host_ops msmsdcc_ops = {
	.request	= msmsdcc_request,
	.set_ios	= msmsdcc_set_ios,
	.enable_sdio_irq = msmsdcc_enable_sdio_irq,
	/* .get_cd		= msmsdcc_get_cd, */
};

static void
msmsdcc_check_status(unsigned long data)
{
	struct msmsdcc_host *host = (struct msmsdcc_host *)data;
	unsigned int status;
	unsigned long duration;
	int sdcard = is_sd_platform(host->plat);

	if (!host->plat->status) {
		mmc_detect_change(host->mmc, 0);
		goto out;
	}

	status = host->plat->status(mmc_dev(host->mmc));
	host->eject = !status;
	if (status ^ host->oldstat) {
		printk(KERN_INFO
		       "%s: Slot status change detected (%d -> %d)\n",
		       mmc_hostname(host->mmc), host->oldstat, status);
		duration = jiffies - msmsdcc_irqtime;

		if (status) {
			if (sdcard) {
				if (duration < (7 * HZ))
					duration = (7 * HZ) - duration;
				else
					duration = 10;
			} else
				duration = (5 * HZ) / 2;
		} else
			duration = 0;

		mmc_detect_change(host->mmc, duration);

		if (sdcard)
			msmsdcc_irqtime = jiffies;
	}

	host->oldstat = status;

out:
	if (host->timer.function)
		mod_timer(&host->timer, jiffies + HZ);
}

static irqreturn_t
msmsdcc_platform_status_irq(int irq, void *dev_id)
{
	struct msmsdcc_host *host = dev_id;

	printk(KERN_DEBUG "%s: %d\n", __func__, irq);
	msmsdcc_check_status((unsigned long) host);
	return IRQ_HANDLED;
}

static void
msmsdcc_status_notify_cb(int card_present, void *dev_id)
{
	struct msmsdcc_host *host = dev_id;

	printk(KERN_DEBUG "%s: card_present %d\n", mmc_hostname(host->mmc),
	       card_present);
	msmsdcc_check_status((unsigned long) host);
}

static void
msmsdcc_busclk_expired(unsigned long _data)
{
	struct msmsdcc_host	*host = (struct msmsdcc_host *) _data;
	unsigned long		flags;

	/* dev_info(mmc_dev(host->mmc), "Bus clock timer expired - S\n"); */
	spin_lock_irqsave(&host->lock, flags);
	if (host->clks_on) {
		msmsdcc_disable_clocks(host, 0);
	}
	spin_unlock_irqrestore(&host->lock, flags);
}

static int
msmsdcc_init_dma(struct msmsdcc_host *host)
{
	memset(&host->dma, 0, sizeof(struct msmsdcc_dma_data));
	host->dma.host = host;
	host->dma.channel = -1;

	if (!host->dmares)
		return -ENODEV;

	host->dma.nc = dma_alloc_coherent(NULL,
					  sizeof(struct msmsdcc_nc_dmadata),
					  &host->dma.nc_busaddr,
					  GFP_KERNEL);
	if (host->dma.nc == NULL) {
		printk(KERN_ERR "Unable to allocate DMA buffer\n");
		return -ENOMEM;
	}
	memset(host->dma.nc, 0x00, sizeof(struct msmsdcc_nc_dmadata));
	host->dma.cmd_busaddr = host->dma.nc_busaddr;
	host->dma.cmdptr_busaddr = host->dma.nc_busaddr +
				offsetof(struct msmsdcc_nc_dmadata, cmdptr);
	host->dma.channel = host->dmares->start;

	return 0;
}

#ifdef CONFIG_MMC_MSM7X00A_RESUME_IN_WQ
static void
do_resume_work(struct work_struct *work)
{
	struct msmsdcc_host *host =
		container_of(work, struct msmsdcc_host, resume_task);
	struct mmc_host	*mmc = host->mmc;

	if (mmc) {
		mmc_resume_host(mmc);
		if (host->stat_irq)
			enable_irq(host->stat_irq);
	}
}
#endif

static int
msmsdcc_probe(struct platform_device *pdev)
{
	struct mmc_platform_data *plat = pdev->dev.platform_data;
	struct msmsdcc_host *host;
	struct mmc_host *mmc;
	struct resource *cmd_irqres = NULL;
	struct resource *pio_irqres = NULL;
	struct resource *stat_irqres = NULL;
	struct resource *memres = NULL;
	struct resource *dmares = NULL;
	int ret;

	/* must have platform data */
	if (!plat) {
		printk(KERN_ERR "%s: Platform data not available\n", __func__);
		ret = -EINVAL;
		goto out;
	}

	if (pdev->id < 1 || pdev->id > 4)
		return -EINVAL;

	if (pdev->resource == NULL || pdev->num_resources < 2) {
		printk(KERN_ERR "%s: Invalid resource\n", __func__);
		return -ENXIO;
	}

	memres = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	dmares = platform_get_resource(pdev, IORESOURCE_DMA, 0);
	cmd_irqres = platform_get_resource_byname(pdev, IORESOURCE_IRQ,
						  "cmd_irq");
	pio_irqres = platform_get_resource_byname(pdev, IORESOURCE_IRQ,
						  "pio_irq");
	stat_irqres = platform_get_resource_byname(pdev, IORESOURCE_IRQ,
						   "status_irq");

	if (!cmd_irqres || !pio_irqres || !memres) {
		printk(KERN_ERR "%s: Invalid resource\n", __func__);
		return -ENXIO;
	}

	/*
	 * Setup our host structure
	 */

	mmc = mmc_alloc_host(sizeof(struct msmsdcc_host), &pdev->dev);
	if (!mmc) {
		ret = -ENOMEM;
		goto out;
	}

	host = mmc_priv(mmc);
	host->pdev_id = pdev->id;
	host->plat = plat;
	host->mmc = mmc;
	host->curr.cmd = NULL;

	host->cmdpoll = 1;

	host->base = ioremap(memres->start, PAGE_SIZE);
	if (!host->base) {
		ret = -ENOMEM;
		goto out;
	}

	host->cmd_irqres = cmd_irqres;
	host->pio_irqres = pio_irqres;
	host->memres = memres;
	host->dmares = dmares;
	spin_lock_init(&host->lock);

#ifdef CONFIG_MMC_EMBEDDED_SDIO
	if (plat->embedded_sdio)
		mmc_set_embedded_sdio_data(mmc,
					   &plat->embedded_sdio->cis,
					   &plat->embedded_sdio->cccr,
					   plat->embedded_sdio->funcs,
					   plat->embedded_sdio->num_funcs);
#endif

#ifdef CONFIG_MMC_MSM7X00A_RESUME_IN_WQ
	INIT_WORK(&host->resume_task, do_resume_work);
#endif

	/*
	 * Setup DMA
	 */
	msmsdcc_init_dma(host);

	/* Get our clocks */
	host->pclk = clk_get(&pdev->dev, "sdc_pclk");
	if (IS_ERR(host->pclk)) {
		ret = PTR_ERR(host->pclk);
		goto host_free;
	}

	/*
	 * Setup SDC MMC clock
	 */
	host->clk = clk_get(&pdev->dev, "sdc_clk");
	if (IS_ERR(host->clk)) {
		ret = PTR_ERR(host->clk);
		goto pclk_put;
	}

	/* Enable clocks */
	ret = msmsdcc_enable_clocks(host);
	if (ret)
		goto clk_put;

	ret = clk_set_rate(host->clk, msmsdcc_fmin);
	if (ret) {
		printk(KERN_ERR "%s: Clock rate set failed (%d)\n",
		       __func__, ret);
		goto clk_disable;
	}

	host->pclk_rate = clk_get_rate(host->pclk);
	host->clk_rate = clk_get_rate(host->clk);

	/*
	 * Setup MMC host structure
	 */
	mmc->ops = &msmsdcc_ops;
	mmc->f_min = msmsdcc_fmin;
	mmc->f_max = msmsdcc_fmax;
	mmc->ocr_avail = plat->ocr_mask;

	if (is_mmc_platform(plat))
		mmc->caps |= MMC_CAP_8_BIT_DATA;
	else if (msmsdcc_4bit)
		mmc->caps |= MMC_CAP_4_BIT_DATA;
	if (msmsdcc_sdioirq)
		mmc->caps |= MMC_CAP_SDIO_IRQ;
	mmc->caps |= MMC_CAP_MMC_HIGHSPEED | MMC_CAP_SD_HIGHSPEED;

	mmc->max_phys_segs = NR_SG;
	mmc->max_hw_segs = NR_SG;
	mmc->max_blk_size = 4096;	/* MCI_DATA_CTL BLOCKSIZE up to 4096 */
	mmc->max_blk_count = 65536;

	mmc->max_req_size = 33554432;	/* MCI_DATA_LENGTH is 25 bits */
	mmc->max_seg_size = mmc->max_req_size;

	msmsdcc_writel(host, 0, MMCIMASK0);
	msmsdcc_writel(host, 0x5e007ff, MMCICLEAR);

	msmsdcc_writel(host, MCI_IRQENABLE, MMCIMASK0);
	host->saved_irq0mask = MCI_IRQENABLE;

	/*
	 * Setup card detect change
	 */

	memset(&host->timer, 0, sizeof(host->timer));

	if (stat_irqres && !(stat_irqres->flags & IORESOURCE_DISABLED)) {
		unsigned long irqflags = IRQF_SHARED |
			(stat_irqres->flags & IRQF_TRIGGER_MASK);

		host->stat_irq = stat_irqres->start;
		ret = request_irq(host->stat_irq,
				  msmsdcc_platform_status_irq,
				  irqflags,
				  DRIVER_NAME " (slot)",
				  host);
		if (ret) {
			printk(KERN_ERR "Unable to get slot IRQ %d (%d)\n",
			       host->stat_irq, ret);
			goto clk_disable;
		}
	} else if (plat->register_status_notify) {
		plat->register_status_notify(msmsdcc_status_notify_cb, host);
	} else if (!plat->status)
		printk(KERN_ERR "%s: No card detect facilities available\n",
		       mmc_hostname(mmc));
	else {
		init_timer(&host->timer);
		host->timer.data = (unsigned long)host;
		host->timer.function = msmsdcc_check_status;
		host->timer.expires = jiffies + HZ;
		add_timer(&host->timer);
	}

	if (plat->status) {
		host->oldstat = host->plat->status(mmc_dev(host->mmc));
		host->eject = !host->oldstat;
	}

	init_timer(&host->busclk_timer);
	host->busclk_timer.data = (unsigned long) host;
	host->busclk_timer.function = msmsdcc_busclk_expired;
#ifdef CONFIG_MMC_BUSCLK_PWRSAVE
	printk(KERN_INFO "%s: MMC_BUSCLK_PWRSAVE is enabled\n", mmc_hostname(mmc));
#endif

	ret = request_irq(cmd_irqres->start, msmsdcc_irq, IRQF_SHARED,
			  DRIVER_NAME " (cmd)", host);
	if (ret)
		goto stat_irq_free;

	ret = request_irq(pio_irqres->start, msmsdcc_pio_irq, IRQF_SHARED,
			  DRIVER_NAME " (pio)", host);
	if (ret)
		goto cmd_irq_free;

	mmc_set_drvdata(pdev, mmc);
	if (is_mmc_platform(plat)) {
		ret = device_add(&mmc->class_dev);
		if (ret)
			goto cmd_irq_free;

		mmc_power_off(mmc);
		mmc->detect.work.func(&mmc->detect.work);
	} else
		mmc_add_host(mmc);

	printk(KERN_INFO
	       "%s: Qualcomm MSM SDCC at 0x%016llx irq %d,%d dma %d\n",
	       mmc_hostname(mmc), (unsigned long long)memres->start,
	       (unsigned int) cmd_irqres->start,
	       (unsigned int) host->stat_irq, host->dma.channel);
	printk(KERN_INFO "%s: Platform slot type: %s\n", mmc_hostname(mmc),
		(plat->slot_type) ? mmc_type_str(*plat->slot_type) : "Not defined");
	printk(KERN_INFO "%s: 4 bit data mode %s\n", mmc_hostname(mmc),
	       (mmc->caps & MMC_CAP_4_BIT_DATA ? "enabled" : "disabled"));
	printk(KERN_INFO "%s: MMC clock %u -> %u Hz, PCLK %u Hz\n",
	       mmc_hostname(mmc), msmsdcc_fmin, msmsdcc_fmax, host->pclk_rate);
	printk(KERN_INFO "%s: Slot eject status = %d\n", mmc_hostname(mmc),
	       host->eject);
	printk(KERN_INFO "%s: Power save feature enable = %d\n",
	       mmc_hostname(mmc), msmsdcc_pwrsave);

	if (host->dma.channel != -1) {
		printk(KERN_INFO
		       "%s: DM non-cached buffer at %p, dma_addr 0x%.8x\n",
		       mmc_hostname(mmc), host->dma.nc, host->dma.nc_busaddr);
		printk(KERN_INFO
		       "%s: DM cmd busaddr 0x%.8x, cmdptr busaddr 0x%.8x\n",
		       mmc_hostname(mmc), host->dma.cmd_busaddr,
		       host->dma.cmdptr_busaddr);
	} else
		printk(KERN_INFO
		       "%s: PIO transfer enabled\n", mmc_hostname(mmc));
	if (host->timer.function)
		printk(KERN_INFO "%s: Polling status mode enabled\n",
		       mmc_hostname(mmc));

#if defined(CONFIG_DEBUG_FS)
	msmsdcc_dbg_createhost(host);
#endif
#ifdef CONFIG_MMC_BUSCLK_PWRSAVE
	if (host->clks_on) {
#ifdef CONFIG_WIMAX
			if (!supersonic_wimax_get_status())              
#endif			
		        msmsdcc_disable_clocks(host, 1);
	}	
#endif
	return 0;
 cmd_irq_free:
	free_irq(cmd_irqres->start, host);
 stat_irq_free:
	if (host->stat_irq)
		free_irq(host->stat_irq, host);
 clk_disable:
	msmsdcc_disable_clocks(host, 0);
 clk_put:
	clk_put(host->clk);
 pclk_put:
	clk_put(host->pclk);
 host_free:
	mmc_free_host(mmc);
 out:
	return ret;
}

static int
msmsdcc_suspend(struct platform_device *dev, pm_message_t state)
{
	struct mmc_host *mmc = mmc_get_drvdata(dev);
	int rc = 0;

	if (mmc) {
		struct msmsdcc_host *host = mmc_priv(mmc);

		if (host->stat_irq)
			disable_irq(host->stat_irq);

		if (mmc->card && mmc->card->type != MMC_TYPE_SDIO)
			rc = mmc_suspend_host(mmc, state);
		if (!rc)
			msmsdcc_writel(host, 0, MMCIMASK0);
		if (host->clks_on)
			msmsdcc_disable_clocks(host, 0);
	}
	return rc;
}

static int
msmsdcc_resume(struct platform_device *dev)
{
	struct mmc_host *mmc = mmc_get_drvdata(dev);

	if (mmc) {
		struct msmsdcc_host *host = mmc_priv(mmc);

		msmsdcc_enable_clocks(host);

		msmsdcc_writel(host, host->saved_irq0mask, MMCIMASK0);

		if (mmc->card && mmc->card->type != MMC_TYPE_SDIO)
#ifdef CONFIG_MMC_MSM7X00A_RESUME_IN_WQ
			schedule_work(&host->resume_task);
#else
			mmc_resume_host(mmc);
			if (host->stat_irq)
				enable_irq(host->stat_irq);
#endif
		else if (host->stat_irq)
			enable_irq(host->stat_irq);
#ifdef CONFIG_MMC_BUSCLK_PWRSAVE
		if (host->clks_on)
		{
#ifdef CONFIG_WIMAX
			if (!supersonic_wimax_get_status())              
#endif
            {				
				printk(KERN_INFO "%s msmsdcc_disable_clocks\n", __func__);
				msmsdcc_disable_clocks(host, 1);
			}
		}

#endif
	}
	return 0;
}

static struct platform_driver msmsdcc_driver = {
	.probe		= msmsdcc_probe,
	.suspend	= msmsdcc_suspend,
	.resume		= msmsdcc_resume,
	.driver		= {
		.name	= "msm_sdcc",
	},
};

static int __init msmsdcc_init(void)
{
	return platform_driver_register(&msmsdcc_driver);
}

static void __exit msmsdcc_exit(void)
{
	platform_driver_unregister(&msmsdcc_driver);
}

static int __init msmsdcc_piopoll_setup(char *__unused)
{
	msmsdcc_piopoll = 1;
	return 1;
}

static int __init msmsdcc_nopiopoll_setup(char *__unused)
{
	msmsdcc_piopoll = 0;
	return 1;
}

static int __init msmsdcc_pwrsave_setup(char *__unused)
{
	msmsdcc_pwrsave = 1;
	return 1;
}

static int __init msmsdcc_nopwrsave_setup(char *__unused)
{
	msmsdcc_pwrsave = 0;
	return 1;
}

static int __init msmsdcc_sdioirq_setup(char *__unused)
{
	msmsdcc_sdioirq = 1;
	return 1;
}

static int __init msmsdcc_nosdioirq_setup(char *__unused)
{
	msmsdcc_sdioirq = 0;
	return 1;
}

static int __init msmsdcc_4bit_setup(char *__unused)
{
	msmsdcc_4bit = 1;
	return 1;
}

static int __init msmsdcc_1bit_setup(char *__unused)
{
	msmsdcc_4bit = 0;
	return 1;
}

static int __init msmsdcc_fmin_setup(char *str)
{
	unsigned int n;

	if (!get_option(&str, &n))
		return 0;
	msmsdcc_fmin = n;
	return 1;
}

static int __init msmsdcc_fmax_setup(char *str)
{
	unsigned int n;

	if (!get_option(&str, &n))
		return 0;
	msmsdcc_fmax = n;
	return 1;
}

__setup("msmsdcc_4bit", msmsdcc_4bit_setup);
__setup("msmsdcc_1bit", msmsdcc_1bit_setup);
__setup("msmsdcc_pwrsave", msmsdcc_pwrsave_setup);
__setup("msmsdcc_nopwrsave", msmsdcc_nopwrsave_setup);
__setup("msmsdcc_sdioirq", msmsdcc_sdioirq_setup);
__setup("msmsdcc_nosdioirq", msmsdcc_nosdioirq_setup);
__setup("msmsdcc_fmin=", msmsdcc_fmin_setup);
__setup("msmsdcc_fmax=", msmsdcc_fmax_setup);
__setup("msmsdcc_piopoll", msmsdcc_piopoll_setup);
__setup("msmsdcc_nopiopoll", msmsdcc_nopiopoll_setup);

module_init(msmsdcc_init);
module_exit(msmsdcc_exit);
module_param(msmsdcc_fmin, uint, 0444);
module_param(msmsdcc_fmax, uint, 0444);
module_param(msmsdcc_4bit, uint, 0444);

MODULE_DESCRIPTION("Qualcomm MSM 7X00A Multimedia Card Interface driver");
MODULE_LICENSE("GPL");

#if defined(CONFIG_DEBUG_FS)

static int
msmsdcc_dbg_state_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static ssize_t
msmsdcc_dbg_state_read(struct file *file, char __user *ubuf,
		       size_t count, loff_t *ppos)
{
	struct msmsdcc_host *host = (struct msmsdcc_host *) file->private_data;
	char buf[1024];
	int max, i;

	i = 0;
	max = sizeof(buf) - 1;

	if (host->curr.cmd) {
		struct mmc_command *cmd = host->curr.cmd;

		i += scnprintf(buf + i, max - i,
			       "CurrentCmd : opcode %d, arg 0x%.8x,"
			       " flags 0x%.8x\n",
			       cmd->opcode, cmd->arg, cmd->flags);
	}

	if (host->curr.data) {
		struct mmc_data *data = host->curr.data;
		i += scnprintf(buf + i, max - i,
			      "DAT0: %.8x %.8x %.8x %.8x %.8x %.8x\n",
			      data->timeout_ns, data->timeout_clks,
			      data->blksz, data->blocks, data->error,
			      data->flags);
		i += scnprintf(buf + i, max - i, "DAT1: %.8x %.8x %.8x %p\n",
			      host->curr.xfer_size, host->curr.xfer_remain,
			      host->curr.data_xfered, host->dma.sg);
	}

	i += scnprintf(buf + i, max - i, "Requests : %u\n", host->stats.reqs);
	i += scnprintf(buf + i, max - i, "Commands : %u\n", host->stats.cmds);
	i += scnprintf(buf + i, max - i, "CmdPoll  : %d\n", host->cmdpoll);
	i += scnprintf(buf + i, max - i, "PollHit  : %u\n",
		       host->stats.cmdpoll_hits);
	i += scnprintf(buf + i, max - i, "PollMiss : %u\n\n",
		       host->stats.cmdpoll_misses);

	i += scnprintf(buf + i, max - i, "DmaBusy    : %d\n", host->dma.busy);
	i += scnprintf(buf + i, max - i, "DmaActive  : %d\n", host->dma.active);
	i += scnprintf(buf + i, max - i, "DmCh8Status : 0x%x\n",
		       readl(DMOV_STATUS(8)));
	i += scnprintf(buf + i, max - i, "ClocksOn   : %d\n", host->clks_on);
	if (host->clks_on) {
		i += scnprintf(buf + i, max - i, "MmciStatus : 0x%x\n",
			       msmsdcc_readl(host, MMCISTATUS));
		i += scnprintf(buf + i, max - i, "DataCtrl   : 0x%x\n",
			       msmsdcc_readl(host, MMCIDATACTRL));
		i += scnprintf(buf + i, max - i, "DataLength : 0x%x\n",
			       msmsdcc_readl(host, MMCIDATALENGTH));
	}

	return simple_read_from_buffer(ubuf, count, ppos, buf, i);
}

static const struct file_operations msmsdcc_dbg_state_ops = {
	.read	= msmsdcc_dbg_state_read,
	.open	= msmsdcc_dbg_state_open,
};

static void msmsdcc_dbg_createhost(struct msmsdcc_host *host)
{
	if (debugfs_dir) {
		debugfs_create_file(mmc_hostname(host->mmc), 0644, debugfs_dir,
				    host, &msmsdcc_dbg_state_ops);
	}
}

static int __init msmsdcc_dbg_init(void)
{
	int err;

	debugfs_dir = debugfs_create_dir("msmsdcc", 0);
	if (IS_ERR(debugfs_dir)) {
		err = PTR_ERR(debugfs_dir);
		debugfs_dir = NULL;
		return err;
	}

	return 0;
}

device_initcall(msmsdcc_dbg_init);
#endif
