// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2024 Broadcom Corporation
 */
#include <linux/iopoll.h>
#include "stmmac.h"
#include "dwxgmac2.h"
#include "dw25gmac.h"

u32 dw25gmac_decode_vdma_count(u32 regval)
{
	/* compressed encoding for vdma count */
	if (regval < 16) /* Direct mapping */
		return regval + 1;
	else if (regval < 20) /* 20, 24, 28, 32 */
		return 20 + (regval - 16) * 4;
	else if (regval < 24) /* 40, 48, 56, 64 */
		return 40 + (regval - 20) * 8;
	else if (regval < 28) /* 80, 96, 112, 128 */
		return 80 + (regval - 24) * 16;
	else  /* not defined */
		return 0;
}

static int rd_dma_ch_ind(void __iomem *ioaddr, u8 mode, u32 channel)
{
	u32 reg_val = 0;

	reg_val |= FIELD_PREP(XXVGMAC_MODE_SELECT, mode);
	reg_val |= FIELD_PREP(XXVGMAC_ADDR_OFFSET, channel);
	reg_val |= XXVGMAC_CMD_TYPE | XXVGMAC_OB;
	writel(reg_val, ioaddr + XXVGMAC_DMA_CH_IND_CONTROL);
	return readl(ioaddr + XXVGMAC_DMA_CH_IND_DATA);
}

static void wr_dma_ch_ind(void __iomem *ioaddr, u8 mode, u32 channel, u32 val)
{
	u32 reg_val = 0;

	writel(val, ioaddr + XXVGMAC_DMA_CH_IND_DATA);
	reg_val |= FIELD_PREP(XXVGMAC_MODE_SELECT, mode);
	reg_val |= FIELD_PREP(XXVGMAC_ADDR_OFFSET, channel);
	reg_val |= XGMAC_OB;
	writel(reg_val, ioaddr + XXVGMAC_DMA_CH_IND_CONTROL);
}

void dw25gmac_dma_init(void __iomem *ioaddr,
		       struct stmmac_dma_cfg *dma_cfg)
{
	u32 tx_pdmas, rx_pdmas;
	u32 hw_cap;
	u32 value;
	u32 i;

	/*
	 * The 25GMAC initialises its internal SPRAM (which holds the per-VDMA
	 * register file) after reset. Poll HMIC until the hardware signals that
	 * the memory init is complete before touching any DMA configuration.
	 */
	if (readl_poll_timeout(ioaddr + XGMAC_DMA_MODE, value,
			       value & XXVGMAC_HMIC, 10, 200000))
		pr_warn("dw25gmac: timeout waiting for HMIC\n");

	value = readl(ioaddr + XGMAC_DMA_SYSBUS_MODE);
	value &= ~(XGMAC_AAL | XGMAC_EAME);
	if (dma_cfg->aal)
		value |= XGMAC_AAL;
	if (dma_cfg->eame)
		value |= XGMAC_EAME;
	writel(value, ioaddr + XGMAC_DMA_SYSBUS_MODE);

	/* Get PDMA counts from HW */
	hw_cap = readl(ioaddr + XGMAC_HW_FEATURE2);
	tx_pdmas = FIELD_GET(XGMAC_HWFEAT_TXQCNT, hw_cap) + 1;
	rx_pdmas = FIELD_GET(XGMAC_HWFEAT_RXQCNT, hw_cap) + 1;

	/* Initialize all PDMAs with burst length fields */
	for (i = 0; i < tx_pdmas; i++) {
		value = rd_dma_ch_ind(ioaddr, MODE_TXEXTCFG, i);
		value &= ~(XXVGMAC_TXPBL | XXVGMAC_TPBLX8_MODE);
		if (dma_cfg->pblx8)
			value |= XXVGMAC_TPBLX8_MODE;
		value |= FIELD_PREP(XXVGMAC_TXPBL, dma_cfg->pbl);
		wr_dma_ch_ind(ioaddr, MODE_TXEXTCFG, i, value);
	}

	for (i = 0; i < rx_pdmas; i++) {
		value = rd_dma_ch_ind(ioaddr, MODE_RXEXTCFG, i);
		value &= ~(XXVGMAC_RXPBL | XXVGMAC_RPBLX8_MODE);
		if (dma_cfg->pblx8)
			value |= XXVGMAC_RPBLX8_MODE;
		value |= FIELD_PREP(XXVGMAC_RXPBL, dma_cfg->pbl);
		wr_dma_ch_ind(ioaddr, MODE_RXEXTCFG, i, value);
	}
}

void dw25gmac_dma_init_tx_chan(struct stmmac_priv *priv,
			       void __iomem *ioaddr,
			       struct stmmac_dma_cfg *dma_cfg,
			       dma_addr_t dma_addr, u32 chan)
{
	const struct dwxgmac_addrs *addrs = priv->plat->dwxgmac_addrs;
	u32 pdma, tc, value;

	/* Descriptor cache size and prefetch threshold size. */
	value = rd_dma_ch_ind(ioaddr, MODE_TXDESCCTRL, chan);
	value &= ~XXVGMAC_TXDCSZ;
	value |= FIELD_PREP(XXVGMAC_TXDCSZ,
			    dma_cfg->txdcsz ? dma_cfg->txdcsz
					    : XXVGMAC_TXDCSZ_256BYTES);
	value &= ~XXVGMAC_TDPS;
	value |= FIELD_PREP(XXVGMAC_TDPS,
			    dma_cfg->tdps ? dma_cfg->tdps : XXVGMAC_TDPS_HALF);
	wr_dma_ch_ind(ioaddr, MODE_TXDESCCTRL, chan, value);

	/* Use custom VDMA→TC / VDMA→PDMA maps if provided, else 1:1 default. */
	pdma = (dma_cfg->tx_vdma_pdma_map && chan < dma_cfg->total_tx_vdma)
	       ? dma_cfg->tx_vdma_pdma_map[chan] : chan;
	tc   = (dma_cfg->tx_vdma_tc_map && chan < dma_cfg->total_tx_vdma)
	       ? dma_cfg->tx_vdma_tc_map[chan]   : chan;

	/* PDMA assignment; optionally set AXI outstanding reads */
	value = rd_dma_ch_ind(ioaddr, MODE_TXEXTCFG, chan);
	value &= ~XXVGMAC_TP2TCMP;
	value |= FIELD_PREP(XXVGMAC_TP2TCMP, pdma);
	if (dma_cfg->orrq)
		value |= FIELD_PREP(XXVGMAC_ORRQ, dma_cfg->orrq);
	wr_dma_ch_ind(ioaddr, MODE_TXEXTCFG, chan, value);

	/* VDMA to TC mapping */
	value = readl(ioaddr + XGMAC_DMA_CH_TX_CONTROL(addrs, chan));
	value &= ~XXVGMAC_TVDMA2TCMP;
	value |= FIELD_PREP(XXVGMAC_TVDMA2TCMP, tc);
	writel(value, ioaddr + XGMAC_DMA_CH_TX_CONTROL(addrs, chan));

	writel(upper_32_bits(dma_addr),
	       ioaddr + XGMAC_DMA_CH_TxDESC_HADDR(addrs, chan));
	writel(lower_32_bits(dma_addr),
	       ioaddr + XGMAC_DMA_CH_TxDESC_LADDR(addrs, chan));
}

void dw25gmac_dma_init_rx_chan(struct stmmac_priv *priv,
			       void __iomem *ioaddr,
			       struct stmmac_dma_cfg *dma_cfg,
			       dma_addr_t dma_addr, u32 chan)
{
	const struct dwxgmac_addrs *addrs = priv->plat->dwxgmac_addrs;
	u32 pdma, tc, value;

	/* Descriptor cache size and prefetch threshold size. */
	value = rd_dma_ch_ind(ioaddr, MODE_RXDESCCTRL, chan);
	value &= ~XXVGMAC_RXDCSZ;
	value |= FIELD_PREP(XXVGMAC_RXDCSZ,
			    dma_cfg->rxdcsz ? dma_cfg->rxdcsz
					    : XXVGMAC_RXDCSZ_256BYTES);
	value &= ~XXVGMAC_RDPS;
	value |= FIELD_PREP(XXVGMAC_RDPS,
			    dma_cfg->rdps ? dma_cfg->rdps : XXVGMAC_RDPS_HALF);
	wr_dma_ch_ind(ioaddr, MODE_RXDESCCTRL, chan, value);

	/* Use custom VDMA→TC / VDMA→PDMA maps if provided, else 1:1 default. */
	pdma = (dma_cfg->rx_vdma_pdma_map && chan < dma_cfg->total_rx_vdma)
	       ? dma_cfg->rx_vdma_pdma_map[chan] : chan;
	tc   = (dma_cfg->rx_vdma_tc_map && chan < dma_cfg->total_rx_vdma)
	       ? dma_cfg->rx_vdma_tc_map[chan]   : chan;

	/* PDMA assignment; enable RX PDMA; optionally set AXI outstanding writes */
	value = rd_dma_ch_ind(ioaddr, MODE_RXEXTCFG, chan);
	value &= ~XXVGMAC_RP2TCMP;
	value |= FIELD_PREP(XXVGMAC_RP2TCMP, pdma);
	if (dma_cfg->owrq)
		value |= FIELD_PREP(XXVGMAC_OWRQ, dma_cfg->owrq);
	value |= XXVGMAC_RXPEN;
	wr_dma_ch_ind(ioaddr, MODE_RXEXTCFG, chan, value);

	/* VDMA to TC mapping */
	value = readl(ioaddr + XGMAC_DMA_CH_RX_CONTROL(addrs, chan));
	value &= ~XXVGMAC_RVDMA2TCMP;
	value |= FIELD_PREP(XXVGMAC_RVDMA2TCMP, tc);
	writel(value, ioaddr + XGMAC_DMA_CH_RX_CONTROL(addrs, chan));

	writel(upper_32_bits(dma_addr),
	       ioaddr + XGMAC_DMA_CH_RxDESC_HADDR(addrs, chan));
	writel(lower_32_bits(dma_addr),
	       ioaddr + XGMAC_DMA_CH_RxDESC_LADDR(addrs, chan));
}

void dw25gmac_dma_map_tx_offline_chan(struct stmmac_priv *priv,
				      void __iomem *ioaddr,
				      struct stmmac_dma_cfg *dma_cfg, u32 chan)
{
	const struct dwxgmac_addrs *addrs = priv->plat->dwxgmac_addrs;
	u32 pdma = (dma_cfg->tx_vdma_pdma_map && chan < dma_cfg->total_tx_vdma)
		   ? dma_cfg->tx_vdma_pdma_map[chan]
		   : chan % priv->plat->tx_queues_to_use;
	u32 tc   = (dma_cfg->tx_vdma_tc_map && chan < dma_cfg->total_tx_vdma)
		   ? dma_cfg->tx_vdma_tc_map[chan]
		   : chan % priv->plat->tx_queues_to_use;
	u32 value;

	/* PDMA assignment for offline channel */
	value = rd_dma_ch_ind(ioaddr, MODE_TXEXTCFG, chan);
	value &= ~XXVGMAC_TP2TCMP;
	value |= FIELD_PREP(XXVGMAC_TP2TCMP, pdma);
	wr_dma_ch_ind(ioaddr, MODE_TXEXTCFG, chan, value);

	/* VDMA to TC mapping for offline channel */
	value = readl(ioaddr + XGMAC_DMA_CH_TX_CONTROL(addrs, chan));
	value &= ~XXVGMAC_TVDMA2TCMP;
	value |= FIELD_PREP(XXVGMAC_TVDMA2TCMP, tc);
	writel(value, ioaddr + XGMAC_DMA_CH_TX_CONTROL(addrs, chan));
}

void dw25gmac_dma_map_rx_offline_chan(struct stmmac_priv *priv,
				      void __iomem *ioaddr,
				      struct stmmac_dma_cfg *dma_cfg, u32 chan)
{
	const struct dwxgmac_addrs *addrs = priv->plat->dwxgmac_addrs;
	u32 pdma = (dma_cfg->rx_vdma_pdma_map && chan < dma_cfg->total_rx_vdma)
		   ? dma_cfg->rx_vdma_pdma_map[chan]
		   : chan % priv->plat->rx_queues_to_use;
	u32 tc   = (dma_cfg->rx_vdma_tc_map && chan < dma_cfg->total_rx_vdma)
		   ? dma_cfg->rx_vdma_tc_map[chan]
		   : chan % priv->plat->rx_queues_to_use;
	u32 value;

	/* PDMA assignment for offline channel */
	value = rd_dma_ch_ind(ioaddr, MODE_RXEXTCFG, chan);
	value &= ~XXVGMAC_RP2TCMP;
	value |= FIELD_PREP(XXVGMAC_RP2TCMP, pdma);
	if (dma_cfg->owrq)
		value |= FIELD_PREP(XXVGMAC_OWRQ, dma_cfg->owrq);
	wr_dma_ch_ind(ioaddr, MODE_RXEXTCFG, chan, value);

	/* VDMA to TC mapping for offline channel */
	value = readl(ioaddr + XGMAC_DMA_CH_RX_CONTROL(addrs, chan));
	value &= ~XXVGMAC_RVDMA2TCMP;
	value |= FIELD_PREP(XXVGMAC_RVDMA2TCMP, tc);
	writel(value, ioaddr + XGMAC_DMA_CH_RX_CONTROL(addrs, chan));
}

void dw25gmac_desc_cache_compute(void __iomem *ioaddr)
{
	u32 value;

	value = readl(ioaddr + XGMAC_DMA_MODE);
	value |= XXVGMAC_DSCB;
	writel(value, ioaddr + XGMAC_DMA_MODE);

	if (readl_poll_timeout(ioaddr + XGMAC_DMA_MODE, value,
			       !(value & XXVGMAC_DSCB), 10, 200))
		pr_warn("dw25gmac: timeout waiting for DSCB completion\n");
}
