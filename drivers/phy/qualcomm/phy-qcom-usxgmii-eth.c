// SPDX-License-Identifier: GPL-2.0-only
/*
 * Qualcomm SA8797P (Nord) USXGMII 10G SerDes PHY driver.
 *
 * The SerDes VCO runs at a fixed 12.5 GHz for USXGMII.  Rate adaptation
 * for sub-10G speeds is handled by the XPCS and MAC layers, not here.
 * This driver mirrors the structure of phy-qcom-sgmii-eth.c.
 *
 * PHY bases (IPCAT nordschleife_2.0):
 *   SGMII_PHY_0: 0x088F8000  SGMII_PHY_1: 0x088FC000
 *
 * Block layout (offsets from phy base):
 *   COM: 0x000  TX0: 0x400  RX0: 0x600  PCS: 0xC00
 *
 * Register values from the GearVM Nord V2 10G sequence.
 *
 * QREF CXO routing registers (TCSR/TLMM) are iomapped directly at
 * probe time and programmed in power_on before SerDes calibration.
 *
 * Copyright (c) 2024 Qualcomm Technologies, Inc.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/ethtool.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>

#include "phy-qcom-qmp-qserdes-com-v7.h"
#include "phy-qcom-qmp-qserdes-txrx-v7.h"
#include "phy-qcom-qmp-pcs-sgmii.h"

/* Block base offsets from phy regmap base */
#define QSERDES_COM_BASE	0x000
#define QSERDES_TX0_BASE	0x400
#define QSERDES_RX0_BASE	0x600
#define QSERDES_PCS_BASE	0xC00

/* PCS registers absent from phy-qcom-qmp-pcs-sgmii.h.
 * Offsets verified against IPCAT nordschleife_2.0.
 */
#define QPHY_PCS_RETIME_BUFFER_EN		0x018
#define QPHY_PCS_TX_LARGE_AMP_POST_EMP_LVL	0x024
#define QPHY_PCS_TX_SMALL_AMP_POST_EMP_LVL	0x02C
#define QPHY_PCS_SGMII_MISC_CTRL7		0x114

/* TX_EMP_POST1_LVL offset (not yet in upstream v7 header) */
#define QSERDES_V7_TX_TX_EMP_POST1_LVL		0x00c

/* Peak load currents from Nord SerDes power requirements */
#define USXGMII_VDDA_0P9_UA	40210
#define USXGMII_VDDA_1P2_UA	12520

/* TCSR QREF CXO configuration — GearVM Nord V2 values, same for EMAC0 and EMAC1
 * Base and offsets verified against IPCAT nordschleife_2.0.
 */
#define TCSR_PHY_BASE			0x01FD5000UL
#define TCSR_PHY_SIZE			0x1000
#define TCSR_QREFS_CXO0_RPT0_CONFIG	0x00
#define TCSR_QREFS_CXO0_TX0_CONFIG	0x04
#define TCSR_QREFS_CXO0_RX3_CONFIG	0x08
#define TCSR_QREFS_CXO0_RX2_CONFIG	0x0c
#define TCSR_QREFS_CXO0_RPT2_CONFIG	0x10
#define TCSR_QREFS_CXO0_RX4_CONFIG	0x14
#define TCSR_QREFS_CXO0_RX0_CONFIG	0x18
#define TCSR_QREFS_CXO0_RPT1_CONFIG	0x1c
#define TCSR_QREFS_CXO0_RPT4_CONFIG	0x20
#define TCSR_QREFS_CXO0_RPT3_CONFIG	0x2c
#define TCSR_QREFS_CXO0_RPT5_CONFIG	0x30
#define TCSR_QREFS_CXO0_RPT6_CONFIG	0x34
#define TCSR_QREFS_CXO0_RX1_CONFIG	0x38
#define TCSR_QREFS_CXO0_RPT7_CONFIG	0x3c
#define TCSR_CXO_REFGEN_BIAS_SEL	0x40
#define TCSR_QREFS_CXO0_RX5_CONFIG	0x78
#define TCSR_QREFS_CXO0_TX1_CONFIG	0x7c

/* TLMM PHY0 QREF routing — base and offsets verified against IPCAT nordschleife_2.0 */
#define TLMM_PHY_BASE			0x0F1D8000UL
#define TLMM_PHY_SIZE			0x12000
#define TLMM_QREF_PHY_SEL_0		0x0000
#define TLMM_PHY0_QREF_TX_RPT_SEL	0x1000
#define TLMM_PHY0_QREF_RX_SEL		0x1004
#define TLMM_PHY0_QREF_ENABLE		0x1008

static const char * const qcom_dwmac_usxgmii_clk_names[] = {
	"sgmi_rx", "sgmi_tx", "sgmi_ref",
};

/* Mux source clocks (clk_regmap_mux) — switched to SerDes output after power-on */
static const char * const qcom_dwmac_usxgmii_mux_clk_names[] = {
	"sgmiiphy_rx_src", "sgmiiphy_tx_src",
	"mac_rclk_src",    "mac_tclk_src",
};

/* SerDes output fixed clocks — become the new parents of the mux above */
static const char * const qcom_dwmac_usxgmii_phy_clk_names[] = {
	"sgmiiphy_rclk",     "sgmiiphy_tclk",
	"sgmiiphy_mac_rclk", "sgmiiphy_mac_tclk",
};

struct qcom_dwmac_usxgmii_phy_data {
	struct regmap *regmap;
	struct clk_bulk_data clks[ARRAY_SIZE(qcom_dwmac_usxgmii_clk_names)];
	struct regulator *vdda_0p9;
	struct regulator *vdda_1p2;
	void __iomem *tcsr_base;
	void __iomem *tlmm_base;
	struct clk *mux_clks[ARRAY_SIZE(qcom_dwmac_usxgmii_mux_clk_names)];
	struct clk *phy_clks[ARRAY_SIZE(qcom_dwmac_usxgmii_phy_clk_names)];
};

#define com_w(rm, off, v)  regmap_write((rm), QSERDES_COM_BASE + (off), (v))
#define tx_w(rm, off, v)   regmap_write((rm), QSERDES_TX0_BASE + (off), (v))
#define rx_w(rm, off, v)   regmap_write((rm), QSERDES_RX0_BASE + (off), (v))
#define pcs_w(rm, off, v)  regmap_write((rm), QSERDES_PCS_BASE + (off), (v))

static void qcom_dwmac_usxgmii_phy_init_10g(struct regmap *rm)
{
	pcs_w(rm, QPHY_PCS_SW_RESET,            0x01);
	pcs_w(rm, QPHY_PCS_POWER_DOWN_CONTROL,  0x01);

	/* COM — 12.5 GHz VCO for USXGMII 10G */
	com_w(rm, QSERDES_V7_COM_BG_TIMER,                     0x0A);
	com_w(rm, QSERDES_V7_COM_PLL_IVCO,                     0x0F);
	com_w(rm, QSERDES_V7_COM_BIAS_EN_CLKBUFLR_EN,          0x07);
	com_w(rm, QSERDES_V7_COM_CLK_ENABLE1,                  0x0F);
	com_w(rm, QSERDES_V7_COM_CP_CTRL_MODE0,                0x08);
	com_w(rm, QSERDES_V7_COM_PLL_RCTRL_MODE0,              0x16);
	com_w(rm, QSERDES_V7_COM_PLL_CCTRL_MODE0,              0x36);
	com_w(rm, QSERDES_V7_COM_INTEGLOOP_GAIN0_MODE0,        0x1F);
	com_w(rm, QSERDES_V7_COM_INTEGLOOP_GAIN1_MODE0,        0x00);
	com_w(rm, QSERDES_V7_COM_PLL_EN,                       0x03);
	com_w(rm, QSERDES_V7_COM_SYSCLK_EN_SEL,                0x1A);
	com_w(rm, QSERDES_V7_COM_LOCK_CMP1_MODE0,              0x23);
	com_w(rm, QSERDES_V7_COM_LOCK_CMP2_MODE0,              0x43);
	com_w(rm, QSERDES_V7_COM_DEC_START_MODE0,              0x43);
	com_w(rm, QSERDES_V7_COM_DIV_FRAC_START1_MODE0,        0x00);
	com_w(rm, QSERDES_V7_COM_DIV_FRAC_START2_MODE0,        0x38);
	com_w(rm, QSERDES_V7_COM_DIV_FRAC_START3_MODE0,        0x02);
	com_w(rm, QSERDES_V7_COM_VCO_TUNE1_MODE0,              0xE6);
	com_w(rm, QSERDES_V7_COM_VCO_TUNE2_MODE0,              0x01);
	com_w(rm, QSERDES_V7_COM_VCO_TUNE_INITVAL2,            0x00);
	com_w(rm, QSERDES_V7_COM_HSCLK_SEL_1,                  0x00);
	com_w(rm, QSERDES_V7_COM_HSCLK_HS_SWITCH_SEL_1,        0x00);
	com_w(rm, QSERDES_V7_COM_CORECLK_DIV_MODE0,            0x04);
	com_w(rm, QSERDES_V7_COM_CORE_CLK_EN,                  0x30);
	com_w(rm, QSERDES_V7_COM_CMN_CONFIG_1,                 0x16);
	com_w(rm, QSERDES_V7_COM_BIN_VCOCAL_CMP_CODE1_MODE0,   0xD7);
	com_w(rm, QSERDES_V7_COM_BIN_VCOCAL_CMP_CODE2_MODE0,   0x0F);
	com_w(rm, QSERDES_V7_COM_BIN_VCOCAL_HSCLK_SEL_1,       0x11);

	/* TX0 */
	tx_w(rm, QSERDES_V7_TX_CLKBUF_ENABLE,                  0x0D);
	tx_w(rm, QSERDES_V7_TX_TX_BAND,                        0x04);
	tx_w(rm, QSERDES_V7_TX_SLEW_CNTL,                      0x08);
	tx_w(rm, QSERDES_V7_TX_RES_CODE_LANE_OFFSET_TX,        0x09);
	tx_w(rm, QSERDES_V7_TX_RES_CODE_LANE_OFFSET_RX,        0x09);
	tx_w(rm, QSERDES_V7_TX_LANE_MODE_1,                    0xF5);
	tx_w(rm, QSERDES_V7_TX_LANE_MODE_2,                    0x06);
	tx_w(rm, QSERDES_V7_TX_LANE_MODE_3,                    0x3F);
	tx_w(rm, QSERDES_V7_TX_LANE_MODE_4,                    0x3F);
	tx_w(rm, QSERDES_V7_TX_LANE_MODE_5,                    0x5F);
	tx_w(rm, QSERDES_V7_TX_RCV_DETECT_LVL_2,               0x12);
	tx_w(rm, QSERDES_V7_TX_TRAN_DRVR_EMP_EN,               0x0F);
	tx_w(rm, QSERDES_V7_TX_TX_EMP_POST1_LVL,               0x2B);

	/* RX0 */
	rx_w(rm, QSERDES_V7_RX_UCDR_FO_GAIN,                   0x0D);
	rx_w(rm, QSERDES_V7_RX_UCDR_SO_GAIN,                   0x03);
	rx_w(rm, QSERDES_V7_RX_UCDR_FASTLOCK_FO_GAIN,          0x0A);
	rx_w(rm, QSERDES_V7_RX_UCDR_SO_SATURATION_AND_ENABLE,  0x7F);
	rx_w(rm, QSERDES_V7_RX_UCDR_FASTLOCK_COUNT_LOW,        0x00);
	rx_w(rm, QSERDES_V7_RX_UCDR_FASTLOCK_COUNT_HIGH,       0x01);
	rx_w(rm, QSERDES_V7_RX_UCDR_PI_CONTROLS,               0x81);
	rx_w(rm, QSERDES_V7_RX_UCDR_PI_CTRL2,                  0x81);
	rx_w(rm, QSERDES_V7_RX_UCDR_SB2_THRESH1,               0x11);	/* V2 */
	rx_w(rm, QSERDES_V7_RX_UCDR_SB2_THRESH2,               0x22);	/* V2 */
	rx_w(rm, QSERDES_V7_RX_RX_TERM_BW,                     0x03);
	rx_w(rm, QSERDES_V7_RX_VGA_CAL_CNTRL2,                 0x08);
	rx_w(rm, QSERDES_V7_RX_GM_CAL,                         0x0F);
	rx_w(rm, QSERDES_V7_RX_RX_EQU_ADAPTOR_CNTRL1,          0x04);
	rx_w(rm, QSERDES_V7_RX_RX_EQU_ADAPTOR_CNTRL2,          0x00);
	rx_w(rm, QSERDES_V7_RX_RX_EQU_ADAPTOR_CNTRL3,          0x4A);
	rx_w(rm, QSERDES_V7_RX_RX_EQU_ADAPTOR_CNTRL4,          0x5A);
	rx_w(rm, QSERDES_V7_RX_RX_IDAC_TSETTLE_LOW,            0x80);
	rx_w(rm, QSERDES_V7_RX_RX_IDAC_TSETTLE_HIGH,           0x01);
	rx_w(rm, QSERDES_V7_RX_RX_IDAC_MEASURE_TIME,           0x20);
	rx_w(rm, QSERDES_V7_RX_RX_EQ_OFFSET_ADAPTOR_CNTRL1,    0x17);
	rx_w(rm, QSERDES_V7_RX_RX_OFFSET_ADAPTOR_CNTRL2,       0x00);
	rx_w(rm, QSERDES_V7_RX_SIGDET_CNTRL,                   0x0F);
	rx_w(rm, QSERDES_V7_RX_SIGDET_DEGLITCH_CNTRL,          0x1E);
	rx_w(rm, QSERDES_V7_RX_RX_BAND,                        0x18);
	rx_w(rm, QSERDES_V7_RX_RX_MODE_00_LOW,                 0x1F);
	rx_w(rm, QSERDES_V7_RX_RX_MODE_00_HIGH,                0xBF);
	rx_w(rm, QSERDES_V7_RX_RX_MODE_00_HIGH2,               0xFF);
	rx_w(rm, QSERDES_V7_RX_RX_MODE_00_HIGH3,               0xDF);
	rx_w(rm, QSERDES_V7_RX_RX_MODE_00_HIGH4,               0xEF);
	rx_w(rm, QSERDES_V7_RX_RX_MODE_01_LOW,                 0xE5);
	rx_w(rm, QSERDES_V7_RX_RX_MODE_01_HIGH,                0xC8);
	rx_w(rm, QSERDES_V7_RX_RX_MODE_01_HIGH2,               0xC8);
	rx_w(rm, QSERDES_V7_RX_RX_MODE_01_HIGH3,               0x14);
	rx_w(rm, QSERDES_V7_RX_RX_MODE_01_HIGH4,               0xB6);
	rx_w(rm, QSERDES_V7_RX_RX_MODE_10_LOW,                 0xE0);
	rx_w(rm, QSERDES_V7_RX_RX_MODE_10_HIGH,                0xC8);
	rx_w(rm, QSERDES_V7_RX_RX_MODE_10_HIGH2,               0xC8);
	rx_w(rm, QSERDES_V7_RX_RX_MODE_10_HIGH3,               0x3B);
	rx_w(rm, QSERDES_V7_RX_RX_MODE_10_HIGH4,               0xB7);
	rx_w(rm, QSERDES_V7_RX_DCC_CTRL1,                      0x0C);
	rx_w(rm, QSERDES_V7_RX_SIGDET_CAL_CTRL1,               0x00);
	rx_w(rm, QSERDES_V7_RX_SIGDET_CAL_CTRL2_AND_CDR_LOCK_EDGE, 0x00);

	/* PCS */
	pcs_w(rm, QPHY_PCS_LINE_RESET_TIME,             0x00);
	pcs_w(rm, QPHY_PCS_RETIME_BUFFER_EN,            0x01);
	pcs_w(rm, QPHY_PCS_TX_LARGE_AMP_DRV_LVL,       0x1A);
	pcs_w(rm, QPHY_PCS_TX_LARGE_AMP_POST_EMP_LVL,  0x0B);
	pcs_w(rm, QPHY_PCS_TX_SMALL_AMP_DRV_LVL,       0x03);
	pcs_w(rm, QPHY_PCS_TX_SMALL_AMP_POST_EMP_LVL,  0x00);
	pcs_w(rm, QPHY_PCS_SGMII_MISC_CTRL7,           0x00);
	pcs_w(rm, QPHY_PCS_SGMII_MISC_CTRL8,           0x14);
	pcs_w(rm, QPHY_PCS_TX_MID_TERM_CTRL1,          0x83);
	pcs_w(rm, QPHY_PCS_TX_MID_TERM_CTRL2,          0x08);
	pcs_w(rm, QPHY_PCS_SW_RESET,                   0x00);
	pcs_w(rm, QPHY_PCS_PHY_START,                  0x01);
	udelay(5);
	pcs_w(rm, QPHY_PCS_PHY_START,                  0x01);	/* SVE: write twice */
}

static int
qcom_dwmac_usxgmii_poll_status(struct regmap *rm, unsigned int reg,
				unsigned int bit)
{
	unsigned int val;

	return regmap_read_poll_timeout(rm, reg, val, val & bit, 1500, 750000);
}

static int qcom_dwmac_usxgmii_calibrate(struct phy *phy)
{
	struct qcom_dwmac_usxgmii_phy_data *data = phy_get_drvdata(phy);
	struct device *dev = phy->dev.parent;
	struct regmap *rm = data->regmap;

	qcom_dwmac_usxgmii_phy_init_10g(rm);

	if (qcom_dwmac_usxgmii_poll_status(rm,
					    QSERDES_COM_BASE + QSERDES_V7_COM_C_READY_STATUS,
					    BIT(0))) {
		dev_err(dev, "COM C_READY timed out\n");
		return -ETIMEDOUT;
	}

	if (qcom_dwmac_usxgmii_poll_status(rm,
					    QSERDES_PCS_BASE + QPHY_PCS_PCS_READY_STATUS,
					    BIT(0))) {
		dev_err(dev, "PCS_READY timed out\n");
		return -ETIMEDOUT;
	}

	if (qcom_dwmac_usxgmii_poll_status(rm,
					    QSERDES_PCS_BASE + QPHY_PCS_PCS_READY_STATUS,
					    BIT(7))) {
		dev_err(dev, "SGMIIPHY_READY timed out\n");
		return -ETIMEDOUT;
	}

	if (qcom_dwmac_usxgmii_poll_status(rm,
					    QSERDES_COM_BASE + QSERDES_V7_COM_CMN_STATUS,
					    BIT(1))) {
		dev_err(dev, "SerDes PLL lock timed out\n");
		return -ETIMEDOUT;
	}

	dev_info(dev, "SerDes calibration OK: C_READY PCS_READY SGMIIPHY_READY PLL_LOCKED\n");
	return 0;
}

static void qcom_dwmac_usxgmii_qref_init(struct device *dev,
					   struct qcom_dwmac_usxgmii_phy_data *data)
{
	void __iomem *tc = data->tcsr_base;
	void __iomem *tl = data->tlmm_base;

	/* Step 1: bias select + PHY0 QREF enable */
	writel(0x01,   tc + TCSR_CXO_REFGEN_BIAS_SEL);
	writel(0xFFFF, tl + TLMM_PHY0_QREF_TX_RPT_SEL);
	writel(0xFF,   tl + TLMM_PHY0_QREF_RX_SEL);
	writel(0x01,   tl + TLMM_PHY0_QREF_ENABLE);
	msleep(10);

	/* Step 2: QREFS CXO_0 consumer config — from GearVM emac_phy_tcsr_tlmm_enable */
	writel(0x3807, tc + TCSR_QREFS_CXO0_TX0_CONFIG);
	writel(0x2807, tc + TCSR_QREFS_CXO0_TX1_CONFIG);
	writel(0x43,   tc + TCSR_QREFS_CXO0_RX0_CONFIG);
	writel(0x01,   tc + TCSR_QREFS_CXO0_RX1_CONFIG);
	writel(0x01,   tc + TCSR_QREFS_CXO0_RX2_CONFIG);
	writel(0x01,   tc + TCSR_QREFS_CXO0_RX3_CONFIG);
	writel(0x01,   tc + TCSR_QREFS_CXO0_RX4_CONFIG);
	writel(0x01,   tc + TCSR_QREFS_CXO0_RX5_CONFIG);
	writel(0x03,   tc + TCSR_QREFS_CXO0_RPT0_CONFIG);
	writel(0x03,   tc + TCSR_QREFS_CXO0_RPT1_CONFIG);
	writel(0x03,   tc + TCSR_QREFS_CXO0_RPT2_CONFIG);
	writel(0x03,   tc + TCSR_QREFS_CXO0_RPT3_CONFIG);
	writel(0x03,   tc + TCSR_QREFS_CXO0_RPT4_CONFIG);
	msleep(10);

	/* Step 3: PHY0 routing select */
	writel(0x06, tl + TLMM_QREF_PHY_SEL_0);
	msleep(10);

	dev_info(dev, "TCSR BIAS_SEL=0x%x TX0=0x%x RX0=0x%x RPT0=0x%x\n",
		 readl(tc + TCSR_CXO_REFGEN_BIAS_SEL),
		 readl(tc + TCSR_QREFS_CXO0_TX0_CONFIG),
		 readl(tc + TCSR_QREFS_CXO0_RX0_CONFIG),
		 readl(tc + TCSR_QREFS_CXO0_RPT0_CONFIG));
	dev_info(dev, "TLMM PHY0 TX_RPT=0x%x RX=0x%x EN=0x%x SEL0=0x%x\n",
		 readl(tl + TLMM_PHY0_QREF_TX_RPT_SEL),
		 readl(tl + TLMM_PHY0_QREF_RX_SEL),
		 readl(tl + TLMM_PHY0_QREF_ENABLE),
		 readl(tl + TLMM_QREF_PHY_SEL_0));
}

static int qcom_dwmac_usxgmii_power_on(struct phy *phy)
{
	struct qcom_dwmac_usxgmii_phy_data *data = phy_get_drvdata(phy);
	int ret;

	ret = regulator_set_load(data->vdda_0p9, USXGMII_VDDA_0P9_UA);
	if (ret)
		return ret;
	ret = regulator_enable(data->vdda_0p9);
	if (ret)
		goto err_vdda_0p9;

	ret = regulator_set_load(data->vdda_1p2, USXGMII_VDDA_1P2_UA);
	if (ret)
		goto err_vdda_1p2_load;
	ret = regulator_enable(data->vdda_1p2);
	if (ret)
		goto err_vdda_1p2;

	ret = clk_bulk_prepare_enable(ARRAY_SIZE(data->clks), data->clks);
	if (ret)
		goto err_clk;

	qcom_dwmac_usxgmii_qref_init(phy->dev.parent, data);

	for (int i = 0; i < ARRAY_SIZE(data->mux_clks); i++) {
		ret = clk_set_parent(data->mux_clks[i], data->phy_clks[i]);
		if (ret)
			dev_warn(phy->dev.parent, "clk_set_parent %s failed: %d\n",
				 qcom_dwmac_usxgmii_mux_clk_names[i], ret);
	}

	usleep_range(2000, 4000);
	return 0;

err_clk:
	regulator_disable(data->vdda_1p2);err_vdda_1p2:
	regulator_set_load(data->vdda_1p2, 0);
err_vdda_1p2_load:
	regulator_disable(data->vdda_0p9);
err_vdda_0p9:
	regulator_set_load(data->vdda_0p9, 0);
	return ret;
}

static int qcom_dwmac_usxgmii_power_off(struct phy *phy)
{
	struct qcom_dwmac_usxgmii_phy_data *data = phy_get_drvdata(phy);
	struct regmap *rm = data->regmap;

	pcs_w(rm, QPHY_PCS_TX_MID_TERM_CTRL2, 0x08);
	pcs_w(rm, QPHY_PCS_SW_RESET,          0x01);

	clk_bulk_disable_unprepare(ARRAY_SIZE(data->clks), data->clks);

	regulator_disable(data->vdda_1p2);
	regulator_set_load(data->vdda_1p2, 0);
	regulator_disable(data->vdda_0p9);
	regulator_set_load(data->vdda_0p9, 0);

	return 0;
}

static int qcom_dwmac_usxgmii_set_speed(struct phy *phy, int speed)
{
	return qcom_dwmac_usxgmii_calibrate(phy);
}

static const struct phy_ops qcom_dwmac_usxgmii_ops = {
	.power_on	= qcom_dwmac_usxgmii_power_on,
	.power_off	= qcom_dwmac_usxgmii_power_off,
	.set_speed	= qcom_dwmac_usxgmii_set_speed,
	.calibrate	= qcom_dwmac_usxgmii_calibrate,
	.owner		= THIS_MODULE,
};

static const struct regmap_config qcom_dwmac_usxgmii_regmap_cfg = {
	.reg_bits		= 32,
	.val_bits		= 32,
	.reg_stride		= 4,
	.use_relaxed_mmio	= true,
	.disable_locking	= true,
};

static int qcom_dwmac_usxgmii_probe(struct platform_device *pdev)
{
	struct qcom_dwmac_usxgmii_phy_data *data;
	struct device *dev = &pdev->dev;
	struct phy_provider *provider;
	void __iomem *base;
	struct phy *phy;
	int ret;

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(base))
		return PTR_ERR(base);

	data->regmap = devm_regmap_init_mmio(dev, base,
					     &qcom_dwmac_usxgmii_regmap_cfg);
	if (IS_ERR(data->regmap))
		return PTR_ERR(data->regmap);

	data->tcsr_base = devm_ioremap(dev, TCSR_PHY_BASE, TCSR_PHY_SIZE);
	if (!data->tcsr_base)
		return dev_err_probe(dev, -ENOMEM, "failed to map TCSR\n");

	data->tlmm_base = devm_ioremap(dev, TLMM_PHY_BASE, TLMM_PHY_SIZE);
	if (!data->tlmm_base)
		return dev_err_probe(dev, -ENOMEM, "failed to map TLMM\n");

	phy = devm_phy_create(dev, NULL, &qcom_dwmac_usxgmii_ops);
	if (IS_ERR(phy))
		return PTR_ERR(phy);

	for (int i = 0; i < ARRAY_SIZE(data->clks); i++)
		data->clks[i].id = qcom_dwmac_usxgmii_clk_names[i];
	ret = devm_clk_bulk_get(dev, ARRAY_SIZE(data->clks), data->clks);
	if (ret)
		return ret;

	for (int i = 0; i < ARRAY_SIZE(data->mux_clks); i++) {
		data->mux_clks[i] = devm_clk_get(dev,
						  qcom_dwmac_usxgmii_mux_clk_names[i]);
		if (IS_ERR(data->mux_clks[i]))
			return dev_err_probe(dev, PTR_ERR(data->mux_clks[i]),
					     "failed to get mux clk %s\n",
					     qcom_dwmac_usxgmii_mux_clk_names[i]);
		data->phy_clks[i] = devm_clk_get(dev,
						  qcom_dwmac_usxgmii_phy_clk_names[i]);
		if (IS_ERR(data->phy_clks[i]))
			return dev_err_probe(dev, PTR_ERR(data->phy_clks[i]),
					     "failed to get phy clk %s\n",
					     qcom_dwmac_usxgmii_phy_clk_names[i]);
	}

	data->vdda_0p9 = devm_regulator_get(dev, "vdda-0p9");
	if (IS_ERR(data->vdda_0p9))
		return PTR_ERR(data->vdda_0p9);

	data->vdda_1p2 = devm_regulator_get(dev, "vdda-1p2");
	if (IS_ERR(data->vdda_1p2))
		return PTR_ERR(data->vdda_1p2);

	provider = devm_of_phy_provider_register(dev, of_phy_simple_xlate);
	if (IS_ERR(provider))
		return PTR_ERR(provider);

	phy_set_drvdata(phy, data);

	return 0;
}

static const struct of_device_id qcom_dwmac_usxgmii_of_match[] = {
	{ .compatible = "qcom,nord-dwmac-usxgmii-phy" },
	{ },
};
MODULE_DEVICE_TABLE(of, qcom_dwmac_usxgmii_of_match);

static struct platform_driver qcom_dwmac_usxgmii_driver = {
	.probe	= qcom_dwmac_usxgmii_probe,
	.driver = {
		.name		= "qcom-dwmac-usxgmii-phy",
		.of_match_table	= qcom_dwmac_usxgmii_of_match,
	},
};

module_platform_driver(qcom_dwmac_usxgmii_driver);

MODULE_DESCRIPTION("Qualcomm SA8797P USXGMII 10G SerDes PHY driver");
MODULE_LICENSE("GPL");
