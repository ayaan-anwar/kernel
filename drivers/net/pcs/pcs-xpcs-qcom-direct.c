// SPDX-License-Identifier: GPL-2.0
/*
 * Qualcomm XPCS standalone platform driver for 10GBASE-R bring-up.
 *
 * Implements the downstream pcs-xpcs-qcom.c link-up sequence directly via
 * MMIO, without delegating to the upstream pcs-xpcs.c core.
 *
 * The upstream thin wrapper (pcs-xpcs-qcom.c) only clears USXGMII_EN in
 * pcs_link_up which is insufficient: the XPCS achieves RX block-lock but
 * does not drive the SerDes TX, so no frames reach the switch.
 *
 * This file implements the equivalent of the downstream
 * qcom_xpcs_link_up_usxgmii() + qcom_xpcs_reset_usxgmii() sequence for
 * 10GBASE-R, matching the downstream register accesses write-for-write.
 * Differences from downstream:
 *   1. SR_XS_PCS_CTRL2 = BASE-R written at start (§7.5.4 step 1, safety).
 *   2. qcom_xpcs_serdes_reset() skipped — SerDes reset is handled by the
 *      phy-qcom-usxgmii-eth.c driver via phy_power_off/on.
 *   3. Clock voting done explicitly in probe (downstream uses SCMI/RPMh).
 *
 * APB byte offsets (IPCAT nordschleife_2.0, DWC_PORT1_DWC_XPCS):
 *   0x0000  SR_XS_PCS_CTRL1     MDIO_MMD_PCS reg 0
 *   0x0004  SR_XS_PCS_STS1      MDIO_MMD_PCS reg 1   (bit 2 = RLS)
 *   0x001C  SR_XS_PCS_CTRL2     MDIO_MMD_PCS reg 7   (bits[3:0] = PCS type)
 *   0x0020  SR_XS_PCS_STS2      MDIO_MMD_PCS reg 8   (bit 11 = block-lock)
 *   0x2000  VR_XS_PCS_DIG_CTRL1 MDIO_MMD_PCS vnd 0x8000
 *              bit[15]=VR_RST  bit[10]=USXGMII_RST  bit[9]=USXGMII_EN
 *   0x4000  SR_MII_CTRL         MDIO_MMD_VEND2 reg 0 (speed sel bits)
 *   0x5000  VR_MII_DIG_CTRL1    MDIO_MMD_VEND2 vnd 0x8000
 *              bit[15]=VR_RST (SOFT_RST)
 *
 * Copyright (C) 2024 Qualcomm Technologies, Inc.
 */

#include <linux/bits.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/mii.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/phylink.h>
#include <linux/pcs/pcs-xpcs-qcom-direct.h>

/* APB byte offsets from XPCS base */
#define QXPCS_SR_XS_PCS_STS1		0x0004
#define QXPCS_SR_XS_PCS_CTRL2		0x001C
#define QXPCS_SR_XS_PCS_STS2		0x0020
#define QXPCS_VR_XS_PCS_DIG_CTRL1	0x2000	/* DW_VR_MII_PCS_DIG_CTRL1 */
#define QXPCS_SR_MII_CTRL		0x4000	/* DW_SR_MII_MMD_CTRL */
#define QXPCS_VR_MII_DIG_CTRL1		0x5000	/* DW_VR_MII_DIG_CTRL1 */

/* VR_XS_PCS_DIG_CTRL1 / VR_MII_DIG_CTRL1 bit[15] — vendor soft reset */
#define QXPCS_VR_RST		BIT(15)	/* SOFT_RST, SW_RST_BIT_STATUS = 0x8000 */

/* VR_XS_PCS_DIG_CTRL1 bit[10] — USXGMII state-machine restart */
#define QXPCS_USXGMII_RST	BIT(10)	/* USXG_RST_BIT_STATUS = 0x0400 */

/* VR_XS_PCS_DIG_CTRL1 bit[9] — USXGMII mode enable */
#define QXPCS_USXGMII_EN	BIT(9)

/* SR_XS_PCS_STS1 bit[2] — receive link status (DW_SR_XS_PCS_STS1) */
#define QXPCS_STS1_RLS		BIT(2)

/* SR_XS_PCS_STS2 bit[11] — 10GBASE-R block lock (non-latching) */
#define QXPCS_STS2_BLK_LOCK	BIT(11)

/* SR_XS_PCS_CTRL2 bits[3:0] — PCS type selection */
#define QXPCS_CTRL2_TYPE_MASK	GENMASK(3, 0)
#define QXPCS_CTRL2_10GBR	0x0	/* BASE-R type */

/* SR_MII_CTRL speed selection (DW_USXGMII_SS_MASK / DW_USXGMII_10000 etc.) */
#define QXPCS_SS_MASK		(BIT(13) | BIT(6) | BIT(5))
#define QXPCS_SS_10000		(BIT(13) | BIT(6))
#define QXPCS_SS_5000		(BIT(13) | BIT(5))
#define QXPCS_FULL_DUPLEX	BIT(8)

struct qcom_xpcs_direct {
	struct phylink_pcs pcs;
	void __iomem *base;
	struct device *dev;
	int n_clks;
	struct clk_bulk_data *clks;
};

#define to_qxd(p)	container_of((p), struct qcom_xpcs_direct, pcs)

/* Helpers - read/write 16-bit XPCS APB register */
static u32 qxd_rd(struct qcom_xpcs_direct *q, u32 off)
{
	return readl(q->base + off) & 0xffff;
}

static void qxd_wr(struct qcom_xpcs_direct *q, u32 off, u32 val)
{
	writel(val & 0xffff, q->base + off);
}

static void qxd_rmw(struct qcom_xpcs_direct *q, u32 off, u32 mask, u32 set)
{
	qxd_wr(q, off, (qxd_rd(q, off) & ~mask) | set);
}

/* Poll register 'off' until bit 'b' clears (self-clearing bits). */
static int qxd_poll_clr(struct qcom_xpcs_direct *q, u32 off, u32 b)
{
	int i;

	for (i = 0; i < 2000; i++) {
		if (!(qxd_rd(q, off) & b))
			return 0;
		udelay(1);
	}
	dev_warn(q->dev, "poll-clr timeout: off=0x%03x bit=0x%04x\n", off, b);
	return -ETIMEDOUT;
}

/* Poll register 'off' until bit 'b' sets. Non-fatal for fixed-link. */
static int qxd_poll_set(struct qcom_xpcs_direct *q, u32 off, u32 b)
{
	int i;

	for (i = 0; i < 2000; i++) {
		if (qxd_rd(q, off) & b)
			return 0;
		udelay(1);
	}
	dev_info(q->dev, "poll-set timeout: off=0x%03x bit=0x%04x (non-fatal)\n",
		 off, b);
	return -ETIMEDOUT;
}

/*
 * qxd_10g_link_up - downstream link-up sequence for 10GBASE-R.
 *
 * Matches qcom_xpcs_link_up_usxgmii() + qcom_xpcs_reset_usxgmii() for the
 * PHY_INTERFACE_MODE_10GBASER case, register access by register access.
 *
 * Called after USXGMII_EN has already been cleared (so the PCS runs in
 * 10GBASE-R 64B/66B mode).  The sequence arms the XPCS TX encoder toward
 * the SerDes; without it the XPCS receives (block-lock OK) but does not
 * drive any TX signal.
 */
static void qxd_10g_link_up(struct qcom_xpcs_direct *q, int speed)
{
	u32 speed_sel, mii;

	/* -- qcom_xpcs_link_up_usxgmii() part -- */

	/* Step 1: VR_RST in VR_XS_PCS_DIG_CTRL1 (DW_VR_MII_PCS_DIG_CTRL1) */
	qxd_rmw(q, QXPCS_VR_XS_PCS_DIG_CTRL1, QXPCS_VR_RST, QXPCS_VR_RST);
	qxd_poll_clr(q, QXPCS_VR_XS_PCS_DIG_CTRL1, QXPCS_VR_RST);

	/* Step 2: speed selection in SR_MII_CTRL (DW_SR_MII_MMD_CTRL) */
	speed_sel = (speed == SPEED_5000) ? QXPCS_SS_5000 : QXPCS_SS_10000;
	mii = qxd_rd(q, QXPCS_SR_MII_CTRL);
	mii = (mii & ~QXPCS_SS_MASK) | speed_sel | QXPCS_FULL_DUPLEX;
	qxd_wr(q, QXPCS_SR_MII_CTRL, mii);

	/* -- qcom_xpcs_reset_usxgmii() part -- */

	/* Step 3: VR_RST in VR_MII_DIG_CTRL1 (DW_VR_MII_DIG_CTRL1) */
	qxd_rmw(q, QXPCS_VR_MII_DIG_CTRL1, QXPCS_VR_RST, QXPCS_VR_RST);
	qxd_poll_clr(q, QXPCS_VR_MII_DIG_CTRL1, QXPCS_VR_RST);

	/* Step 4: USXGMII_RST in VR_XS_PCS_DIG_CTRL1 — arms TX encoder */
	qxd_rmw(q, QXPCS_VR_XS_PCS_DIG_CTRL1, QXPCS_USXGMII_RST, QXPCS_USXGMII_RST);
	qxd_poll_clr(q, QXPCS_VR_XS_PCS_DIG_CTRL1, QXPCS_USXGMII_RST);

	/* Step 5 (10GBASER case): clear latching STS1 bits then poll RLS */
	qxd_rd(q, QXPCS_SR_XS_PCS_STS1);
	if (qxd_poll_set(q, QXPCS_SR_XS_PCS_STS1, QXPCS_STS1_RLS))
		dev_info(q->dev, "RLS not yet set (fixed-link, continuing)\n");
	else
		dev_info(q->dev, "10G RLS=1: PCS TX link established\n");
}

/* ---- phylink_pcs_ops ---------------------------------------------------- */

static int qxd_pcs_config(struct phylink_pcs *pcs, unsigned int neg_mode,
			  phy_interface_t interface,
			  const unsigned long *advertising,
			  bool permit_pause_to_mac)
{
	return 0;
}

static void qxd_pcs_get_state(struct phylink_pcs *pcs,
			      unsigned int neg_mode,
			      struct phylink_link_state *state)
{
	struct qcom_xpcs_direct *q = to_qxd(pcs);
	u32 sts2 = qxd_rd(q, QXPCS_SR_XS_PCS_STS2);

	state->link = !!(sts2 & QXPCS_STS2_BLK_LOCK);
	if (state->link) {
		state->speed  = SPEED_10000;
		state->duplex = DUPLEX_FULL;
	}
}

static void qxd_pcs_link_up(struct phylink_pcs *pcs, unsigned int neg_mode,
			    phy_interface_t interface, int speed, int duplex)
{
	struct qcom_xpcs_direct *q = to_qxd(pcs);

	/*
	 * Corresponds to qcom_xpcs_link_up() preamble for 10GBASER:
	 *   qcom_xpcs_loopback(false) → clear BMCR_LOOPBACK in SR_MII_CTRL
	 *   qcom_xpcs_serdes_reset()  → skip (phy driver handles SerDes)
	 *   clear USXGMII_EN          → 10GBASE-R mode
	 */

	/* SR_XS_PCS_CTRL2 bits[3:0] = BASE-R (§7.5.4 step 1) */
	qxd_rmw(q, QXPCS_SR_XS_PCS_CTRL2, QXPCS_CTRL2_TYPE_MASK, QXPCS_CTRL2_10GBR);

	/* Disable loopback in SR_MII_CTRL (qcom_xpcs_loopback false) */
	qxd_rmw(q, QXPCS_SR_MII_CTRL, BMCR_LOOPBACK, 0);

	/* Clear USXGMII_EN — PCS now in 10GBASE-R 64B/66B mode */
	qxd_rmw(q, QXPCS_VR_XS_PCS_DIG_CTRL1, QXPCS_USXGMII_EN, 0);

	/* Run downstream restart sequence */
	qxd_10g_link_up(q, speed);

	dev_info(q->dev, "pcs_link_up done: USXGMII_EN=0, TX encoder armed\n");
}

static const struct phylink_pcs_ops qxd_ops = {
	.pcs_config   = qxd_pcs_config,
	.pcs_get_state = qxd_pcs_get_state,
	.pcs_link_up  = qxd_pcs_link_up,
};

/**
 * qcom_xpcs_direct_get_pcs() - return the phylink_pcs for the given XPCS pdev.
 *
 * Called from the ethqos pcs_init callback after finding the XPCS platform
 * device via the pcs-handle DT phandle.
 */
struct phylink_pcs *qcom_xpcs_direct_get_pcs(struct platform_device *pdev)
{
	struct qcom_xpcs_direct *q = platform_get_drvdata(pdev);

	if (!q)
		return ERR_PTR(-EPROBE_DEFER);
	return &q->pcs;
}
EXPORT_SYMBOL_GPL(qcom_xpcs_direct_get_pcs);

static int qxd_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct qcom_xpcs_direct *q;
	int ret;

	dev_info(dev, "probing QCOM XPCS (direct/downstream link-up)\n");

	q = devm_kzalloc(dev, sizeof(*q), GFP_KERNEL);
	if (!q)
		return -ENOMEM;

	q->dev  = dev;
	q->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(q->base))
		return PTR_ERR(q->base);

	dev_info(dev, "XPCS APB mapped at %px\n", q->base);

	q->pcs.ops      = &qxd_ops;

	/*
	 * Enable RPCS/XGXS clocks listed in the DT node.  The downstream
	 * driver relies on SCMI/RPMh to enable these; in upstream Linux we
	 * must do it explicitly.  devm_clk_bulk_get_all() is a no-op when
	 * the node has no clocks.
	 */
	q->n_clks = devm_clk_bulk_get_all(dev, &q->clks);
	if (q->n_clks < 0)
		return dev_err_probe(dev, q->n_clks, "failed to get clocks\n");
	dev_info(dev, "enabling %d XPCS clocks\n", q->n_clks);
	if (q->n_clks > 0) {
		ret = clk_bulk_prepare_enable(q->n_clks, q->clks);
		if (ret)
			return dev_err_probe(dev, ret, "failed to enable clocks\n");
	}

	platform_set_drvdata(pdev, q);
	pm_runtime_enable(dev);

	dev_info(dev, "QCOM XPCS direct probe complete\n");
	return 0;
}

static void qxd_remove(struct platform_device *pdev)
{
	pm_runtime_disable(&pdev->dev);
}

static const struct of_device_id qxd_of_ids[] = {
	{ .compatible = "qcom,xpcs" },
	{ }
};
MODULE_DEVICE_TABLE(of, qxd_of_ids);

static struct platform_driver qxd_driver = {
	.probe	= qxd_probe,
	.remove	= qxd_remove,
	.driver	= {
		.name		= "qcom-xpcs",
		.of_match_table	= qxd_of_ids,
	},
};
module_platform_driver(qxd_driver);

MODULE_DESCRIPTION("Qualcomm DW XPCS - standalone with downstream 10GBASE-R link-up");
MODULE_LICENSE("GPL");
