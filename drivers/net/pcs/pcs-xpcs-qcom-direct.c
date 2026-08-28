// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Qualcomm DW XPCS standalone platform driver — adapted from downstream 6.6
 * pcs-xpcs-qcom.c for the upstream kernel.
 *
 * Changes from the downstream driver:
 *   - All interrupt handling removed (FUSA ISR, AN ISR, uevent work)
 *   - SerDes reset removed (handled by phy-qcom-usxgmii-eth.c via phy_power_off/on)
 *   - devm-based probe (no manual kzalloc/kfree)
 *   - pcs_link_up added to phylink_pcs_ops (downstream exported it standalone)
 *   - pcs_config / pcs_get_state gain the upstream neg_mode parameter
 *   - qcom_xpcs_select_mode() called from pcs_config() (not from a create())
 *   - qcom_xpcs_direct_get_pcs() exported instead of qcom_xpcs_create()
 *   - Register definitions and structs inlined (no private downstream headers)
 *
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/phylink.h>
#include <linux/pcs/pcs-xpcs-qcom-direct.h>

/* ---- driver identity ---------------------------------------------------- */

#define DRV_NAME "qcom-xpcs"

#define XPCSINFO(fmt, args...)	pr_info(DRV_NAME " %s:%d " fmt, __func__, __LINE__, ## args)
#define XPCSERR(fmt, args...)	pr_err(DRV_NAME " %s:%d " fmt, __func__, __LINE__, ## args)
#define XPCSDBG(fmt, args...)	pr_debug(DRV_NAME " %s:%d " fmt, __func__, __LINE__, ## args)

/* ---- XPCS device ID ---------------------------------------------------- */

#define SYNOPSYS_XPCS_ID	0x7996ced0
#define SYNOPSYS_XPCS_MASK	0xffffffff

/* ---- AN mode constants -------------------------------------------------- */

#define DW_AN_C37_USXGMII	1
#define DW_10GBASER		5

/* ---- APB byte offsets (DW_SR_MII_* = SR_XS_PCS, DW_VR_MII_* = VR spaces) */

/* SR_XS_PCS standard registers */
#define DW_SR_MII_PCS_CTRL1		0x0000	/* SR_XS_PCS_CTRL1  — bit[15]=SOFT_RST, bit[11]=LPM_EN */
#define DW_SR_MII_PCS_STS1		0x0004	/* SR_XS_PCS_STS1   — bit[2]=RLS */
#define DW_SR_MII_PCS_DEV_ID1		0x0008
#define DW_SR_MII_PCS_DEV_ID2		0x000c
#define DW_SR_MII_PCS_CTRL2		0x001c	/* SR_XS_PCS_CTRL2  — bits[3:0] = PCS type */

/* VR_XS_PCS vendor registers */
#define DW_VR_MII_PCS_DIG_CTRL1	0x2000	/* bit[15]=VR_RST, bit[10]=USXGMII_RST, bit[9]=USXGMII_EN */
#define DW_VR_MII_PCS_KR_CTRL		0x201c	/* bits[12:10]=USXG_MODE_SEL */

/* SR_MII (VEND2 standard) registers */
#define DW_SR_MII_MMD_CTRL		0x4000	/* SR_MII_CTRL — speed sel, loopback, AN enable */
#define DW_SR_MII_MMD_STS		0x4004	/* SR_MII_STS  — bit[2]=link */

/* VR_MII (VEND2 vendor) registers */
#define DW_VR_MII_DIG_CTRL1		0x5000	/* bit[15]=VR_RST */
#define DW_VR_MII_AN_CTRL		0x5004
#define DW_VR_MII_AN_INTR_STS		0x5008

/* VSMMD registers */
#define DW_SR_MII_VSMMD_CTRL		0x3024

/* ---- Bit definitions ---------------------------------------------------- */

/* Soft / VR reset (bit 15 in both SR_MII_PCS_CTRL1 and VR_MII_DIG_CTRL1) */
#define SOFT_RST			BIT(15)
#define SW_RST_BIT_STATUS		0x8000
#define USXG_RST_BIT_STATUS		0x0400

/* SR_MII_PCS_CTRL1 */
#define LPM_EN				BIT(11)

/* SR_MII_PCS_CTRL2 */
#define PCS_TYPE_SEL_10GBR		GENMASK(3, 0)

/* VR_XS_PCS_DIG_CTRL1 */
#define DW_USXGMII_RST			BIT(10)
#define DW_USXGMII_EN			BIT(9)

/* VR_MII_PCS_KR_CTRL */
#define USXG_MODE_SEL			GENMASK(12, 10)
#define USXGMII_5G			BIT(10)

/* SR_MII_MMD_CTRL speed-selection */
#define DW_USXGMII_SS_MASK		(BIT(13) | BIT(6) | BIT(5))
#define DW_USXGMII_10000		(BIT(13) | BIT(6))
#define DW_USXGMII_5000			(BIT(13) | BIT(5))
#define DW_GMII_2500			(BIT(5))
#define DW_GMII_1000			(BIT(6))
#define DW_GMII_100			(BIT(13))
#define DW_LBE				BIT(14)		/* BMCR loopback */

/* SR_MII_MMD_CTRL — Clause 37 AN enable */
#define AN_CL37_EN			BIT(12)

/* SR_MII_MMD_STS */
#define DW_SR_MII_STS_LINK_STS		BIT(2)

/* SR_XS_PCS_STS1 */
#define DW_SR_XS_PCS_STS1		BIT(2)

/* VR_MII_AN_CTRL */
#define DW_VR_MII_AN_CTRL_TX_CONFIG_SHIFT	3
#define DW_VR_MII_TX_CONFIG_PHY_SIDE		0x1
#define DW_VR_MII_SGMII_LINK_STS		BIT(4)

/* VR_MII_AN_INTR_STS */
#define DW_VR_MII_USXG_ANSGM_SP_SHIFT	10
#define DW_VR_MII_USXG_ANSGM_SP		GENMASK(12, 10)
#define DW_VR_MII_USXG_ANSGM_SP_10G	0x3
#define DW_VR_MII_USXG_ANSGM_SP_5G	0x5
#define DW_VR_MII_USXG_ANSGM_SP_2P5G	0x4
#define DW_VR_MII_USXG_ANSGM_SP_1000	0x2
#define DW_VR_MII_USXG_ANSGM_SP_100	0x1
#define DW_VR_MII_USXG_ANSGM_SP_10	0x0
#define DW_VR_MII_USXG_ANSGM_SP_LNKSTS	BIT(14)
#define DW_VR_MII_ANCMPLT_INTR		BIT(0)

/* VSMMD power-down */
#define PD_CTRL				BIT(5)

/* ---- xpcs_compat / xpcs_id ----------------------------------------------- */

struct xpcs_compat {
	const int *supported;
	const phy_interface_t *interface;
	int num_interfaces;
	int an_mode;
};

struct xpcs_id {
	u32 id;
	u32 mask;
	const struct xpcs_compat *compat;
};

/* ---- dw_xpcs_qcom struct ------------------------------------------------ */

struct dw_xpcs_qcom {
	const struct xpcs_id *id;
	struct phylink_pcs pcs;
	void __iomem *addr;
	struct device *dev;
	bool needs_aneg;
	phy_interface_t phy_interface;
	int n_clks;
	struct clk_bulk_data *clks;
};

#define phylink_pcs_to_xpcs(pl_pcs) \
	container_of((pl_pcs), struct dw_xpcs_qcom, pcs)

/* ---- supported feature lists -------------------------------------------- */

static const int xpcs_usxgmii_features[] = {
	ETHTOOL_LINK_MODE_Pause_BIT,
	ETHTOOL_LINK_MODE_Asym_Pause_BIT,
	ETHTOOL_LINK_MODE_Autoneg_BIT,
	ETHTOOL_LINK_MODE_10baseT_Full_BIT,
	ETHTOOL_LINK_MODE_100baseT_Full_BIT,
	ETHTOOL_LINK_MODE_1000baseKX_Full_BIT,
	ETHTOOL_LINK_MODE_1000baseT_Full_BIT,
	ETHTOOL_LINK_MODE_2500baseX_Full_BIT,
	ETHTOOL_LINK_MODE_2500baseT_Full_BIT,
	ETHTOOL_LINK_MODE_5000baseT_Full_BIT,
	ETHTOOL_LINK_MODE_10000baseKX4_Full_BIT,
	ETHTOOL_LINK_MODE_10000baseKR_Full_BIT,
	ETHTOOL_LINK_MODE_10000baseT_Full_BIT,
	__ETHTOOL_LINK_MODE_MASK_NBITS,
};

static const int xpcs_10gbaser_features[] = {
	ETHTOOL_LINK_MODE_Pause_BIT,
	ETHTOOL_LINK_MODE_Asym_Pause_BIT,
	ETHTOOL_LINK_MODE_10000baseSR_Full_BIT,
	ETHTOOL_LINK_MODE_10000baseLR_Full_BIT,
	ETHTOOL_LINK_MODE_10000baseLRM_Full_BIT,
	ETHTOOL_LINK_MODE_10000baseER_Full_BIT,
	__ETHTOOL_LINK_MODE_MASK_NBITS,
};

static const int xpcs_usx5g_features[] = {
	ETHTOOL_LINK_MODE_Pause_BIT,
	ETHTOOL_LINK_MODE_Asym_Pause_BIT,
	ETHTOOL_LINK_MODE_Autoneg_BIT,
	ETHTOOL_LINK_MODE_10baseT_Half_BIT,
	ETHTOOL_LINK_MODE_10baseT_Full_BIT,
	ETHTOOL_LINK_MODE_100baseT_Half_BIT,
	ETHTOOL_LINK_MODE_100baseT_Full_BIT,
	ETHTOOL_LINK_MODE_1000baseT_Half_BIT,
	ETHTOOL_LINK_MODE_1000baseT_Full_BIT,
	ETHTOOL_LINK_MODE_2500baseT_Full_BIT,
	ETHTOOL_LINK_MODE_5000baseT_Full_BIT,
	__ETHTOOL_LINK_MODE_MASK_NBITS,
};

static const phy_interface_t xpcs_usxgmii_interfaces[] = {
	PHY_INTERFACE_MODE_USXGMII,
};

static const phy_interface_t xpcs_10gbaser_interfaces[] = {
	PHY_INTERFACE_MODE_10GBASER,
};

static const phy_interface_t xpcs_usx5g_interfaces[] = {
	PHY_INTERFACE_MODE_5GBASER,
};

enum {
	DW_XPCS_USXGMII,
	DW_XPCS_10GBASER,
	DW_XPCS_USX5G,
	DW_XPCS_INTERFACE_MAX,
};

/* ---- compat lookup ------------------------------------------------------ */

static const struct xpcs_compat *xpcs_find_compat(const struct xpcs_id *id,
						   phy_interface_t interface)
{
	int i, j;

	for (i = 0; i < DW_XPCS_INTERFACE_MAX; i++) {
		const struct xpcs_compat *compat = &id->compat[i];

		for (j = 0; j < compat->num_interfaces; j++)
			if (compat->interface[j] == interface)
				return compat;
	}

	return NULL;
}

/* ---- register helpers --------------------------------------------------- */

static int qcom_xpcs_read(struct dw_xpcs_qcom *qxpcs, u32 reg)
{
	return readl(qxpcs->addr + reg);
}

static void qcom_xpcs_write(struct dw_xpcs_qcom *qxpcs, u32 reg, u16 val)
{
	writel(val, qxpcs->addr + reg);
}

static int qcom_xpcs_poll_reset(struct dw_xpcs_qcom *qxpcs, unsigned int offset,
				unsigned int field)
{
	unsigned int retries = 32;
	int ret;

	do {
		usleep_range(1000, 2000);
		ret = qcom_xpcs_read(qxpcs, offset);
		if (ret < 0)
			return ret;
	} while (ret & field && --retries);

	return (ret & field) ? -ETIMEDOUT : 0;
}

static int qcom_xpcs_poll_bit_set(struct dw_xpcs_qcom *qxpcs, unsigned int offset,
				  unsigned int field)
{
	unsigned int retries = 256;
	int ret;

	do {
		usleep_range(1000, 2000);
		ret = qcom_xpcs_read(qxpcs, offset);
		if (ret & field)
			return 0;
	} while (--retries);

	return -ETIMEDOUT;
}

/* ---- soft reset (programs SR_XS_PCS_CTRL1 bit[15]) --------------------- */

static int xpcs_soft_reset(struct dw_xpcs_qcom *qxpcs)
{
	int ret;

	ret = qcom_xpcs_read(qxpcs, DW_SR_MII_PCS_CTRL1);
	if (ret < 0)
		return ret;

	qcom_xpcs_write(qxpcs, DW_SR_MII_PCS_CTRL1, ret | SOFT_RST);

	ret = qcom_xpcs_poll_reset(qxpcs, DW_SR_MII_PCS_CTRL1, SW_RST_BIT_STATUS);
	if (ret < 0)
		return ret;

	ret = qcom_xpcs_read(qxpcs, DW_VR_MII_DIG_CTRL1);
	if (ret < 0)
		return ret;

	qcom_xpcs_write(qxpcs, DW_VR_MII_DIG_CTRL1, ret | SOFT_RST);

	return qcom_xpcs_poll_reset(qxpcs, DW_VR_MII_DIG_CTRL1, SW_RST_BIT_STATUS);
}

/* ---- phylink_pcs ops ---------------------------------------------------- */

static int qcom_xpcs_validate(struct phylink_pcs *pcs, unsigned long *supported,
			      const struct phylink_link_state *state)
{
	__ETHTOOL_DECLARE_LINK_MODE_MASK(xpcs_supported);
	struct dw_xpcs_qcom *qxpcs = phylink_pcs_to_xpcs(pcs);
	const struct xpcs_compat *compat;
	int i;

	if (state->interface == PHY_INTERFACE_MODE_NA)
		return -EINVAL;

	bitmap_zero(xpcs_supported, __ETHTOOL_LINK_MODE_MASK_NBITS);

	compat = xpcs_find_compat(qxpcs->id, state->interface);

	if (compat)
		for (i = 0; compat->supported[i] != __ETHTOOL_LINK_MODE_MASK_NBITS; i++)
			set_bit(compat->supported[i], xpcs_supported);

	linkmode_and(supported, supported, xpcs_supported);
	/* state->advertising is const in pcs_validate — cannot modify */

	return 0;
}

/* ---- USXGMII reset sequence --------------------------------------------- */

static int qcom_xpcs_reset_usxgmii(struct dw_xpcs_qcom *qxpcs)
{
	int ret;

	ret = qcom_xpcs_read(qxpcs, DW_VR_MII_DIG_CTRL1);
	if (ret < 0)
		return ret;

	qcom_xpcs_write(qxpcs, DW_VR_MII_DIG_CTRL1, ret | SOFT_RST);

	ret = qcom_xpcs_poll_reset(qxpcs, DW_VR_MII_DIG_CTRL1, SW_RST_BIT_STATUS);
	if (ret < 0)
		return ret;

	ret = qcom_xpcs_read(qxpcs, DW_VR_MII_PCS_DIG_CTRL1);
	if (ret < 0)
		return ret;

	qcom_xpcs_write(qxpcs, DW_VR_MII_PCS_DIG_CTRL1, ret | DW_USXGMII_RST);

	ret = qcom_xpcs_poll_reset(qxpcs, DW_VR_MII_PCS_DIG_CTRL1, USXG_RST_BIT_STATUS);
	if (ret < 0)
		return ret;

	switch (qxpcs->phy_interface) {
	case PHY_INTERFACE_MODE_USXGMII:
		qcom_xpcs_read(qxpcs, DW_SR_MII_MMD_STS);
		ret = qcom_xpcs_poll_bit_set(qxpcs, DW_SR_MII_MMD_STS, DW_SR_MII_STS_LINK_STS);
		if (ret < 0)
			return ret;
		fallthrough;
	case PHY_INTERFACE_MODE_10GBASER:
	case PHY_INTERFACE_MODE_5GBASER:
		qcom_xpcs_read(qxpcs, DW_SR_MII_PCS_STS1);
		ret = qcom_xpcs_poll_bit_set(qxpcs, DW_SR_MII_PCS_STS1, DW_SR_XS_PCS_STS1);
		if (ret < 0)
			return ret;
		break;
	default:
		break;
	}

	return ret;
}

/* ---- CL37 AN config (USXGMII) ------------------------------------------- */

static int xpcs_config_aneg_c37(struct dw_xpcs_qcom *qxpcs)
{
	int ret;

	ret = qcom_xpcs_read(qxpcs, DW_SR_MII_MMD_CTRL);
	if (ret < 0)
		return -EINVAL;

	ret |= AN_CL37_EN;
	qcom_xpcs_write(qxpcs, DW_SR_MII_MMD_CTRL, ret);

	ret = qcom_xpcs_read(qxpcs, DW_VR_MII_AN_CTRL);
	if (ret < 0)
		return -EINVAL;

	ret |= DW_VR_MII_TX_CONFIG_PHY_SIDE << DW_VR_MII_AN_CTRL_TX_CONFIG_SHIFT;
	ret |= DW_VR_MII_SGMII_LINK_STS;

	qcom_xpcs_write(qxpcs, DW_VR_MII_AN_CTRL, ret);

	return 0;
}

static int qcom_xpcs_do_config(struct dw_xpcs_qcom *qxpcs, phy_interface_t interface)
{
	const struct xpcs_compat *compat;
	int ret;

	compat = xpcs_find_compat(qxpcs->id, interface);
	if (!compat)
		return -ENODEV;

	switch (compat->an_mode) {
	case DW_AN_C37_USXGMII:
		qxpcs->needs_aneg = true;
		ret = xpcs_config_aneg_c37(qxpcs);
		if (ret < 0)
			return ret;
		break;
	case DW_10GBASER:
		break;
	default:
		XPCSERR("Incompatible Autonegotiation mode\n");
		return -EINVAL;
	}

	return 0;
}

/*
 * qcom_xpcs_select_mode - select PCS interface type and wake from LPM.
 *
 * Programs SR_XS_PCS_CTRL2 (interface type), SR_XS_PCS_CTRL1 (LPM toggle to
 * reset the PCS into the new mode), and VR_XS_PCS_DIG_CTRL1 (USXGMII_EN).
 * This is the critical sequence missing from the previous direct driver.
 */
static int qcom_xpcs_select_mode(struct dw_xpcs_qcom *qxpcs, phy_interface_t interface)
{
	int ret;

	if (interface != PHY_INTERFACE_MODE_USXGMII &&
	    interface != PHY_INTERFACE_MODE_10GBASER &&
	    interface != PHY_INTERFACE_MODE_5GBASER) {
		XPCSERR("Incompatible MII interface: %d\n", interface);
		return -EINVAL;
	}

	/* Step 1: set PCS type to BASE-R in SR_XS_PCS_CTRL2 */
	ret = qcom_xpcs_read(qxpcs, DW_SR_MII_PCS_CTRL2);
	if (ret < 0)
		goto out;

	ret &= ~PCS_TYPE_SEL_10GBR;
	qcom_xpcs_write(qxpcs, DW_SR_MII_PCS_CTRL2, ret);

	/* Step 2: LPM toggle on SR_XS_PCS_CTRL1 to reset PCS into new mode */
	ret = qcom_xpcs_read(qxpcs, DW_SR_MII_PCS_CTRL1);
	if (ret < 0)
		goto out;

	qcom_xpcs_write(qxpcs, DW_SR_MII_PCS_CTRL1, ret | LPM_EN);

	usleep_range(1, 20);

	ret = qcom_xpcs_read(qxpcs, DW_SR_MII_PCS_CTRL1);
	if (ret < 0)
		goto out;

	ret &= ~LPM_EN;
	qcom_xpcs_write(qxpcs, DW_SR_MII_PCS_CTRL1, ret);

	/* Step 3: enable USXGMII state machine */
	ret = qcom_xpcs_read(qxpcs, DW_VR_MII_PCS_DIG_CTRL1);
	if (ret < 0)
		goto out;

	qcom_xpcs_write(qxpcs, DW_VR_MII_PCS_DIG_CTRL1, ret | DW_USXGMII_EN);

	/* Step 4: speed mode selection in VR_MII_PCS_KR_CTRL */
	ret = qcom_xpcs_read(qxpcs, DW_VR_MII_PCS_KR_CTRL);
	if (ret < 0)
		goto out;

	ret &= ~USXG_MODE_SEL;
	if (interface == PHY_INTERFACE_MODE_5GBASER)
		ret |= USXGMII_5G;

	qcom_xpcs_write(qxpcs, DW_VR_MII_PCS_KR_CTRL, ret);

	return 0;

out:
	XPCSERR("Register read failed\n");
	return -EINVAL;
}

static int qcom_xpcs_config(struct phylink_pcs *pcs, unsigned int neg_mode,
			    phy_interface_t interface,
			    const unsigned long *advertising,
			    bool permit_pause_to_mac)
{
	struct dw_xpcs_qcom *qxpcs = phylink_pcs_to_xpcs(pcs);
	int ret;

	/*
	 * qcom_xpcs_select_mode() toggles LPM_EN on SR_XS_PCS_CTRL1 which
	 * resets PCS state — call only when interface actually changes.
	 */
	if (qxpcs->phy_interface != interface) {
		ret = qcom_xpcs_select_mode(qxpcs, interface);
		if (ret)
			return ret;
		qxpcs->phy_interface = interface;
	}

	ret = qcom_xpcs_do_config(qxpcs, interface);
	if (ret)
		return ret;

	return 0;
}

/* ---- link status -------------------------------------------------------- */

static int qcom_xpcs_get_link_status(struct dw_xpcs_qcom *qxpcs,
				     struct phylink_link_state *state)
{
	unsigned int retries = 32;
	unsigned int count = 0;
	int ret = -EFAULT;

	if (!qxpcs)
		goto failure;

recover:
	count++;

	if (count >= retries) {
		XPCSERR("Link recovery failed\n");
		goto failure;
	}

	/* VR_RST on VR_MII_PCS_DIG_CTRL1 to resync XPCS → SerDes alignment */
	ret = qcom_xpcs_read(qxpcs, DW_VR_MII_PCS_DIG_CTRL1);
	if (ret < 0)
		goto failure;

	qcom_xpcs_write(qxpcs, DW_VR_MII_PCS_DIG_CTRL1, ret | SOFT_RST);

	ret = qcom_xpcs_poll_reset(qxpcs, DW_VR_MII_PCS_DIG_CTRL1, SW_RST_BIT_STATUS);
	if (ret < 0) {
		XPCSERR("Poll reset failed\n");
		goto recover;
	}

	switch (qxpcs->phy_interface) {
	case PHY_INTERFACE_MODE_USXGMII:
		if (qxpcs->needs_aneg) {
			ret = qcom_xpcs_poll_bit_set(qxpcs, DW_VR_MII_AN_INTR_STS,
						     DW_VR_MII_ANCMPLT_INTR);
			if (ret < 0)
				goto recover;
			else
				XPCSDBG("XPCS AN completed\n");

			ret = qcom_xpcs_read(qxpcs, DW_VR_MII_AN_INTR_STS);
			ret &= ~DW_VR_MII_ANCMPLT_INTR;
			qcom_xpcs_write(qxpcs, DW_VR_MII_AN_INTR_STS, ret);
		}
		qcom_xpcs_read(qxpcs, DW_SR_MII_MMD_STS);
		ret = qcom_xpcs_poll_bit_set(qxpcs, DW_SR_MII_MMD_STS, DW_SR_MII_STS_LINK_STS);
		if (ret < 0)
			goto recover;
		fallthrough;
	case PHY_INTERFACE_MODE_10GBASER:
	case PHY_INTERFACE_MODE_5GBASER:
		qcom_xpcs_read(qxpcs, DW_SR_MII_PCS_STS1);
		ret = qcom_xpcs_poll_bit_set(qxpcs, DW_SR_MII_PCS_STS1, DW_SR_XS_PCS_STS1);
		if (ret < 0) {
			XPCSERR("Link is down, try to recover\n");
			goto recover;
		}
		break;
	default:
		break;
	}

	state->link = true;
	return 0;

failure:
	state->link = false;
	return ret;
}

static void qcom_xpcs_get_state(struct phylink_pcs *pcs,
				unsigned int neg_mode,
				struct phylink_link_state *state)
{
	struct dw_xpcs_qcom *qxpcs = phylink_pcs_to_xpcs(pcs);
	const struct xpcs_compat *compat;
	int ret;

	compat = xpcs_find_compat(qxpcs->id, state->interface);
	if (!compat)
		return;

	switch (compat->an_mode) {
	case DW_AN_C37_USXGMII:
		ret = qcom_xpcs_get_link_status(qxpcs, state);
		if (ret < 0)
			XPCSERR("Failed to get USXGMII state\n");
		break;
	case DW_10GBASER:
		ret = qcom_xpcs_get_link_status(qxpcs, state);
		if (ret < 0)
			XPCSERR("Failed to get BaseR state\n");
		break;
	default:
		return;
	}
}

/* ---- loopback helper ---------------------------------------------------- */

static void qcom_xpcs_loopback(struct dw_xpcs_qcom *qxpcs, bool on)
{
	int ret;

	ret = qcom_xpcs_read(qxpcs, DW_VR_MII_PCS_DIG_CTRL1);

	/* downstream always sets USXGMII_EN as a side effect of loopback() */
	qcom_xpcs_write(qxpcs, DW_VR_MII_PCS_DIG_CTRL1, ret |= DW_USXGMII_EN);

	ret = qcom_xpcs_read(qxpcs, DW_SR_MII_MMD_CTRL);
	if (on)
		ret |= DW_LBE;
	else
		ret &= ~DW_LBE;

	qcom_xpcs_write(qxpcs, DW_SR_MII_MMD_CTRL, ret);
}

static int qcom_xpcs_pre_init(struct phylink_pcs *pcs)
{
	struct dw_xpcs_qcom *qxpcs = phylink_pcs_to_xpcs(pcs);

	/* Loopback RX clock only if the MAC requires rxc_always_on.
	 * Nord uses the EMAC wrapper SGMII loopback for clk_rx_i; do NOT
	 * set rxc_always_on on Nord (it causes a TX fault loop with 10GBASE-R
	 * partners by leaving USXGMII-framed data on the SerDes TX lane).
	 */
	if (pcs->rxc_always_on)
		qcom_xpcs_loopback(qxpcs, true);

	return 0;
}

/* ---- link-up sequence --------------------------------------------------- */

/*
 * qcom_xpcs_link_up_usxgmii - downstream link-up sequence.
 *
 * Called for both 10GBASER (with USXGMII_EN already cleared) and USXGMII.
 * Arms the XPCS TX encoder and runs the USXGMII reset sequence that polls
 * for block-lock.  Without this the XPCS receives (block-lock OK) but
 * does not drive any TX signal.
 */
static int qcom_xpcs_link_up_usxgmii(struct dw_xpcs_qcom *qxpcs, int speed)
{
	int mmd_ctrl;
	int ret;

	/* Step 1: VR_RST in VR_MII_PCS_DIG_CTRL1 */
	ret = qcom_xpcs_read(qxpcs, DW_VR_MII_PCS_DIG_CTRL1);
	if (ret < 0)
		goto read_err;

	qcom_xpcs_write(qxpcs, DW_VR_MII_PCS_DIG_CTRL1, ret | SOFT_RST);

	mmd_ctrl = qcom_xpcs_poll_reset(qxpcs, DW_VR_MII_PCS_DIG_CTRL1, SW_RST_BIT_STATUS);
	if (mmd_ctrl < 0) {
		XPCSERR("Failed to perform soft reset\n");
		goto read_err;
	}

	/* Step 2: USXGMII AN config (USXGMII only) */
	if (qxpcs->needs_aneg) {
		ret = qcom_xpcs_read(qxpcs, DW_VR_MII_AN_CTRL);
		if (ret < 0)
			goto read_err;

		ret |= DW_VR_MII_TX_CONFIG_PHY_SIDE << DW_VR_MII_AN_CTRL_TX_CONFIG_SHIFT;
		ret |= DW_VR_MII_SGMII_LINK_STS;

		qcom_xpcs_write(qxpcs, DW_VR_MII_AN_CTRL, ret);
	}

	/* Step 3: set interface speed in SR_MII_MMD_CTRL */
	mmd_ctrl = qcom_xpcs_read(qxpcs, DW_SR_MII_MMD_CTRL);
	if (mmd_ctrl < 0)
		goto read_err;

	mmd_ctrl &= ~DW_USXGMII_SS_MASK;

	switch (speed) {
	case SPEED_10000:
		mmd_ctrl |= DW_USXGMII_10000;
		XPCSINFO("10Gbps-USXGMII enabled\n");
		break;
	case SPEED_5000:
		mmd_ctrl |= DW_USXGMII_5000;
		XPCSINFO("5Gbps-USXGMII enabled\n");
		break;
	case SPEED_2500:
		mmd_ctrl |= DW_GMII_2500;
		XPCSINFO("2.5Gbps-USXGMII enabled\n");
		break;
	case SPEED_1000:
		mmd_ctrl |= DW_GMII_1000;
		XPCSINFO("1Gbps-USXGMII enabled\n");
		break;
	case SPEED_100:
		mmd_ctrl |= DW_GMII_100;
		XPCSINFO("100Mbps-USXGMII enabled\n");
		break;
	case SPEED_10:
		XPCSINFO("10Mbps-USXGMII enabled\n");
		break;
	default:
		XPCSERR("Invalid speed mode selected\n");
		return -EINVAL;
	}

	qcom_xpcs_write(qxpcs, DW_SR_MII_MMD_CTRL, mmd_ctrl);

	/* Step 4: re-enable CL37 AN for USXGMII */
	if (qxpcs->needs_aneg) {
		ret = qcom_xpcs_read(qxpcs, DW_SR_MII_MMD_CTRL);
		if (ret < 0)
			goto read_err;

		qcom_xpcs_write(qxpcs, DW_SR_MII_MMD_CTRL, ret | AN_CL37_EN);
	}

	/* Step 5: USXGMII reset sequence (arms TX encoder, polls for block-lock) */
	mmd_ctrl = qcom_xpcs_reset_usxgmii(qxpcs);
	if (mmd_ctrl < 0)
		goto out;

	XPCSINFO("USXGMII link is up\n");
	return 0;

read_err:
	XPCSERR("Failed to read register\n");
	return -EIO;
out:
	XPCSERR("Failed to bring up USXGMII link\n");
	return -EAGAIN;
}

/*
 * qcom_xpcs_link_up — phylink pcs_link_up callback.
 *
 * For 10GBASER / 5GBASER: clears USXGMII_EN then runs link_up_usxgmii().
 * For USXGMII: leaves USXGMII_EN set and runs link_up_usxgmii().
 * SerDes reset is NOT performed here — handled by phy-qcom-usxgmii-eth.c.
 */
static void qcom_xpcs_link_up(struct phylink_pcs *pcs, unsigned int neg_mode,
			      phy_interface_t interface, int speed, int duplex)
{
	struct dw_xpcs_qcom *qxpcs = phylink_pcs_to_xpcs(pcs);
	struct phylink_link_state state;
	int recover_count = 0;
	int ret;

	/* Cache interface for qcom_xpcs_reset_usxgmii() switch */
	qxpcs->phy_interface = interface;

	/* Disable XPCS internal loopback (EMAC wrapper loopback supplies clk_rx_i) */
	qcom_xpcs_loopback(qxpcs, false);

	switch (interface) {
	case PHY_INTERFACE_MODE_10GBASER:
	case PHY_INTERFACE_MODE_5GBASER:
		/* Clear USXGMII_EN so PCS drives real 64B/66B BASE-R toward SerDes */
		ret = qcom_xpcs_read(qxpcs, DW_VR_MII_PCS_DIG_CTRL1);
		if (ret < 0) {
			XPCSERR("Failed to read register\n");
			break;
		}
		qcom_xpcs_write(qxpcs, DW_VR_MII_PCS_DIG_CTRL1, ret & ~DW_USXGMII_EN);
		fallthrough;
	case PHY_INTERFACE_MODE_USXGMII:
		ret = qcom_xpcs_link_up_usxgmii(qxpcs, speed);
		if (ret == -EAGAIN)
			goto recovery;
		else
			return;
		break;
	default:
		XPCSERR("Invalid MII mode: %s\n", phy_modes(interface));
		break;
	}

recovery:
	if (recover_count >= 10) {
		if (qxpcs->pcs.rxc_always_on)
			qcom_xpcs_loopback(qxpcs, true);
		XPCSERR("XPCS recovery failed after %d attempts\n", recover_count);
		return;
	}

	XPCSDBG("Recovery attempt %d\n", recover_count);
	recover_count++;

	/* VR_RST + link poll without SerDes reset (SerDes handled externally) */
	qcom_xpcs_get_link_status(qxpcs, &state);

	if (!state.link) {
		XPCSERR("Link still down after XPCS VR reset, retrying\n");
		goto recovery;
	}
}

/* ---- device ID detection ------------------------------------------------ */

static u32 xpcs_get_id(struct dw_xpcs_qcom *qxpcs)
{
	int ret;
	u32 id;

	ret = qcom_xpcs_read(qxpcs, DW_SR_MII_PCS_DEV_ID1);
	if (ret < 0)
		return 0xffffffff;

	id = ret << 16;

	ret = qcom_xpcs_read(qxpcs, DW_SR_MII_PCS_DEV_ID2);
	if (ret < 0)
		return 0xffffffff;

	if (id | ret)
		return id | ret;

	return 0xffffffff;
}

/* ---- compat / ID tables ------------------------------------------------- */

static const struct xpcs_compat synopsys_xpcs_compat[DW_XPCS_INTERFACE_MAX] = {
	[DW_XPCS_USXGMII] = {
		.supported	= xpcs_usxgmii_features,
		.interface	= xpcs_usxgmii_interfaces,
		.num_interfaces	= ARRAY_SIZE(xpcs_usxgmii_interfaces),
		.an_mode	= DW_AN_C37_USXGMII,
	},
	[DW_XPCS_10GBASER] = {
		.supported	= xpcs_10gbaser_features,
		.interface	= xpcs_10gbaser_interfaces,
		.num_interfaces	= ARRAY_SIZE(xpcs_10gbaser_interfaces),
		.an_mode	= DW_10GBASER,
	},
	[DW_XPCS_USX5G] = {
		.supported	= xpcs_usx5g_features,
		.interface	= xpcs_usx5g_interfaces,
		.num_interfaces	= ARRAY_SIZE(xpcs_usx5g_interfaces),
		.an_mode	= DW_10GBASER,
	},
};

static const struct xpcs_id xpcs_id_list[] = {
	{
		.id	= SYNOPSYS_XPCS_ID,
		.mask	= SYNOPSYS_XPCS_MASK,
		.compat	= synopsys_xpcs_compat,
	},
};

static const struct phylink_pcs_ops qcom_xpcs_phylink_ops = {
	.pcs_validate	= qcom_xpcs_validate,
	.pcs_config	= qcom_xpcs_config,
	.pcs_get_state	= qcom_xpcs_get_state,
	.pcs_link_up	= qcom_xpcs_link_up,
	.pcs_pre_init	= qcom_xpcs_pre_init,
};

/* ---- platform driver ---------------------------------------------------- */

/**
 * qcom_xpcs_direct_get_pcs - return the phylink_pcs for this XPCS device.
 *
 * Called from dwmac-qcom-ethqos.c after finding the XPCS platform device
 * via the pcs-handle DT phandle.
 */
struct phylink_pcs *qcom_xpcs_direct_get_pcs(struct platform_device *pdev)
{
	struct dw_xpcs_qcom *qxpcs = platform_get_drvdata(pdev);

	if (!qxpcs)
		return ERR_PTR(-EPROBE_DEFER);
	return &qxpcs->pcs;
}
EXPORT_SYMBOL_GPL(qcom_xpcs_direct_get_pcs);

static int qcom_xpcs_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct dw_xpcs_qcom *qxpcs;
	u32 xpcs_id;
	int i, ret;

	qxpcs = devm_kzalloc(dev, sizeof(*qxpcs), GFP_KERNEL);
	if (!qxpcs)
		return -ENOMEM;

	qxpcs->addr = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(qxpcs->addr))
		return PTR_ERR(qxpcs->addr);

	qxpcs->dev = dev;
	/* Sentinel so first pcs_config() call always runs select_mode() */
	qxpcs->phy_interface = PHY_INTERFACE_MODE_NA;

	/* Enable clocks before any hardware access — pclk is needed for register reads */
	qxpcs->n_clks = devm_clk_bulk_get_all(dev, &qxpcs->clks);
	if (qxpcs->n_clks < 0)
		return dev_err_probe(dev, qxpcs->n_clks, "failed to get clocks\n");

	if (qxpcs->n_clks > 0) {
		ret = clk_bulk_prepare_enable(qxpcs->n_clks, qxpcs->clks);
		if (ret)
			return dev_err_probe(dev, ret, "failed to enable clocks\n");
	}

	/* Identify the XPCS hardware */
	xpcs_id = xpcs_get_id(qxpcs);
	if (xpcs_id == 0xffffffff)
		return dev_err_probe(dev, -ENODEV, "Invalid XPCS Device ID\n");

	for (i = 0; i < ARRAY_SIZE(xpcs_id_list); i++) {
		if ((xpcs_id & xpcs_id_list[i].mask) == xpcs_id_list[i].id) {
			qxpcs->id = &xpcs_id_list[i];
			break;
		}
	}
	if (!qxpcs->id)
		return dev_err_probe(dev, -ENODEV,
				     "Unknown XPCS ID 0x%08x\n", xpcs_id);

	dev_info(dev, "XPCS ID 0x%08x matched\n", xpcs_id);

	/* Soft reset — programs SR_XS_PCS_CTRL1 (bit[15]=SOFT_RST) and
	 * VR_MII_DIG_CTRL1 to bring the PCS to a known initial state.
	 */
	ret = xpcs_soft_reset(qxpcs);
	if (ret)
		return dev_err_probe(dev, ret, "XPCS soft reset failed\n");

	qxpcs->pcs.ops  = &qcom_xpcs_phylink_ops;
	qxpcs->pcs.poll = true;
	__set_bit(PHY_INTERFACE_MODE_10GBASER, qxpcs->pcs.supported_interfaces);
	__set_bit(PHY_INTERFACE_MODE_USXGMII,  qxpcs->pcs.supported_interfaces);
	__set_bit(PHY_INTERFACE_MODE_5GBASER,  qxpcs->pcs.supported_interfaces);

	platform_set_drvdata(pdev, qxpcs);

	dev_info(dev, "QCOM XPCS probe complete (10GBASE-R + USXGMII)\n");
	return 0;
}

static void qcom_xpcs_remove(struct platform_device *pdev)
{
	struct dw_xpcs_qcom *qxpcs = platform_get_drvdata(pdev);

	if (qxpcs && qxpcs->n_clks > 0)
		clk_bulk_disable_unprepare(qxpcs->n_clks, qxpcs->clks);
}

static const struct of_device_id qcom_xpcs_match[] = {
	{ .compatible = "qcom,xpcs" },
	{ }
};
MODULE_DEVICE_TABLE(of, qcom_xpcs_match);

static struct platform_driver pcs_xpcs_qcom_driver = {
	.probe	= qcom_xpcs_probe,
	.remove	= qcom_xpcs_remove,
	.driver	= {
		.name		= "dwxpcs_qcom",
		.of_match_table	= of_match_ptr(qcom_xpcs_match),
	},
};
module_platform_driver(pcs_xpcs_qcom_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Qualcomm DW XPCS - adapted from downstream 6.6 driver");
