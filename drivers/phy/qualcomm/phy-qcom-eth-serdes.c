// SPDX-License-Identifier: GPL-2.0-only
/*
 * Qualcomm DWMAC Ethernet SerDes PHY driver.
 *
 * Covers two SerDes variants used with the Synopsys DWMAC on Qualcomm
 * automotive SoCs:
 *
 *  - SGMII / 2500BASE-X (SA8775P and compatible): QMP V5 SerDes at
 *    1.25 Gbps or 3.125 Gbps.
 *
 *  - USXGMII 10G (SA8797P / Nord): QMP V7 SerDes at 12.5 Gbps.
 *    The VCO is fixed at 10G; sub-10G speeds are handled by the XPCS.
 *
 * Both variants share the same block layout:
 *   COM  0x000  TX  0x400  RX  0x600  PCS  0xC00
 *
 * Copyright (c) 2023, Linaro Limited
 * Copyright (c) 2024, Qualcomm Technologies, Inc.
 */

#include <linux/clk.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/phy.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

#include "phy-qcom-qmp-pcs-sgmii.h"
#include "phy-qcom-qmp-qserdes-com-v5.h"
#include "phy-qcom-qmp-qserdes-com-v7.h"
#include "phy-qcom-qmp-qserdes-txrx-v5.h"
#include "phy-qcom-qmp-qserdes-txrx-v7.h"
#include "phy-qcom-qserdes-nord-eth.h"

/* Block base offsets */
#define QSERDES_COM		0x000
#define QSERDES_TX		0x400
#define QSERDES_RX		0x600
#define QSERDES_PCS		0xc00

/* PCS status bits */
#define PCS_C_READY		BIT(0)
#define PCS_READY		BIT(0)
#define PCS_SGMIIPHY_READY	BIT(7)
#define COM_PLL_LOCKED		BIT(1)

/* Nord SA8797P USXGMII regulator peak current loads (µA) */
#define USXGMII_VDDA_0P9_UA		40210
#define USXGMII_VDDA_1P2_UA		12520

/* ------------------------------------------------------------------ */

static const struct regulator_bulk_data sgmii_vregs[] = {
	{ .supply = "vdda-0p9", .init_load_uA = 46000 },
	{ .supply = "vdda-1p2", .init_load_uA = 15000 },
};

static const struct regulator_bulk_data usxgmii_vregs[] = {
	{ .supply = "vdda-0p9", .init_load_uA = USXGMII_VDDA_0P9_UA },
	{ .supply = "vdda-1p2", .init_load_uA = USXGMII_VDDA_1P2_UA },
};

#define QCOM_SERDES_NUM_SUPPLIES	ARRAY_SIZE(sgmii_vregs)

struct qcom_serdes_cfg {
	bool usxgmii;
	const struct regulator_bulk_data *vregs;
};

struct qcom_serdes_data {
	struct regmap			*regmap;
	const struct qcom_serdes_cfg	*cfg;
	struct regulator_bulk_data	*vregs;
	struct clk			*refclk;

	/* SGMII only */
	phy_interface_t			interface;
};

/* ------------------------------------------------------------------ */
/* Shared helpers                                                       */
/* ------------------------------------------------------------------ */

static int serdes_poll(struct regmap *rm, unsigned int reg, u32 bit)
{
	unsigned int val;

	return regmap_read_poll_timeout(rm, reg, val, val & bit, 1500, 750000);
}

static int sgmii_poll_ready(struct regmap *rm)
{
	int ret;

	ret = serdes_poll(rm, QSERDES_COM + QSERDES_V5_COM_C_READY_STATUS,
			  PCS_C_READY);
	if (ret)
		return ret;

	ret = serdes_poll(rm, QSERDES_PCS + QPHY_PCS_PCS_READY_STATUS,
			  PCS_READY);
	if (ret)
		return ret;

	ret = serdes_poll(rm, QSERDES_PCS + QPHY_PCS_PCS_READY_STATUS,
			  PCS_SGMIIPHY_READY);
	if (ret)
		return ret;

	return serdes_poll(rm, QSERDES_COM + QSERDES_V5_COM_CMN_STATUS,
			   COM_PLL_LOCKED);
}

static int usxgmii_poll_ready(struct regmap *rm)
{
	int ret;

	ret = serdes_poll(rm, QSERDES_COM + QSERDES_V7_COM_C_READY_STATUS,
			  PCS_C_READY);
	if (ret)
		return ret;

	ret = serdes_poll(rm, QSERDES_PCS + QPHY_PCS_PCS_READY_STATUS,
			  PCS_READY);
	if (ret)
		return ret;

	ret = serdes_poll(rm, QSERDES_PCS + QPHY_PCS_PCS_READY_STATUS,
			  PCS_SGMIIPHY_READY);
	if (ret)
		return ret;

	return serdes_poll(rm, QSERDES_COM + QSERDES_V7_COM_CMN_STATUS,
			   COM_PLL_LOCKED);
}

/* ------------------------------------------------------------------ */
/* SGMII / 2500BASE-X init sequences (QMP V5)                         */
/* ------------------------------------------------------------------ */

static void sgmii_init_1g(struct regmap *rm)
{
	regmap_write(rm, QSERDES_PCS + QPHY_PCS_SW_RESET, 0x01);
	regmap_write(rm, QSERDES_PCS + QPHY_PCS_POWER_DOWN_CONTROL, 0x01);

	regmap_write(rm, QSERDES_COM + QSERDES_V5_COM_PLL_IVCO, 0x0F);
	regmap_write(rm, QSERDES_COM + QSERDES_V5_COM_CP_CTRL_MODE0, 0x06);
	regmap_write(rm, QSERDES_COM + QSERDES_V5_COM_PLL_RCTRL_MODE0, 0x16);
	regmap_write(rm, QSERDES_COM + QSERDES_V5_COM_PLL_CCTRL_MODE0, 0x36);
	regmap_write(rm, QSERDES_COM + QSERDES_V5_COM_SYSCLK_EN_SEL, 0x1A);
	regmap_write(rm, QSERDES_COM + QSERDES_V5_COM_LOCK_CMP1_MODE0, 0x0A);
	regmap_write(rm, QSERDES_COM + QSERDES_V5_COM_LOCK_CMP2_MODE0, 0x1A);
	regmap_write(rm, QSERDES_COM + QSERDES_V5_COM_DEC_START_MODE0, 0x82);
	regmap_write(rm, QSERDES_COM + QSERDES_V5_COM_DIV_FRAC_START1_MODE0, 0x55);
	regmap_write(rm, QSERDES_COM + QSERDES_V5_COM_DIV_FRAC_START2_MODE0, 0x55);
	regmap_write(rm, QSERDES_COM + QSERDES_V5_COM_DIV_FRAC_START3_MODE0, 0x03);
	regmap_write(rm, QSERDES_COM + QSERDES_V5_COM_VCO_TUNE1_MODE0, 0x24);
	regmap_write(rm, QSERDES_COM + QSERDES_V5_COM_VCO_TUNE2_MODE0, 0x02);
	regmap_write(rm, QSERDES_COM + QSERDES_V5_COM_VCO_TUNE_INITVAL2, 0x00);
	regmap_write(rm, QSERDES_COM + QSERDES_V5_COM_HSCLK_SEL, 0x04);
	regmap_write(rm, QSERDES_COM + QSERDES_V5_COM_HSCLK_HS_SWITCH_SEL, 0x00);
	regmap_write(rm, QSERDES_COM + QSERDES_V5_COM_CORECLK_DIV_MODE0, 0x0A);
	regmap_write(rm, QSERDES_COM + QSERDES_V5_COM_CORE_CLK_EN, 0x00);
	regmap_write(rm, QSERDES_COM + QSERDES_V5_COM_BIN_VCOCAL_CMP_CODE1_MODE0, 0xB9);
	regmap_write(rm, QSERDES_COM + QSERDES_V5_COM_BIN_VCOCAL_CMP_CODE2_MODE0, 0x1E);
	regmap_write(rm, QSERDES_COM + QSERDES_V5_COM_BIN_VCOCAL_HSCLK_SEL, 0x11);

	regmap_write(rm, QSERDES_TX + QSERDES_V5_TX_TX_BAND, 0x05);
	regmap_write(rm, QSERDES_TX + QSERDES_V5_TX_SLEW_CNTL, 0x0A);
	regmap_write(rm, QSERDES_TX + QSERDES_V5_TX_RES_CODE_LANE_OFFSET_TX, 0x09);
	regmap_write(rm, QSERDES_TX + QSERDES_V5_TX_RES_CODE_LANE_OFFSET_RX, 0x09);
	regmap_write(rm, QSERDES_TX + QSERDES_V5_TX_LANE_MODE_1, 0x05);
	regmap_write(rm, QSERDES_TX + QSERDES_V5_TX_LANE_MODE_3, 0x00);
	regmap_write(rm, QSERDES_TX + QSERDES_V5_TX_RCV_DETECT_LVL_2, 0x12);
	regmap_write(rm, QSERDES_TX + QSERDES_V5_TX_TRAN_DRVR_EMP_EN, 0x0C);

	regmap_write(rm, QSERDES_RX + QSERDES_V5_RX_UCDR_FO_GAIN, 0x0A);
	regmap_write(rm, QSERDES_RX + QSERDES_V5_RX_UCDR_SO_GAIN, 0x06);
	regmap_write(rm, QSERDES_RX + QSERDES_V5_RX_UCDR_FASTLOCK_FO_GAIN, 0x0A);
	regmap_write(rm, QSERDES_RX + QSERDES_V5_RX_UCDR_SO_SATURATION_AND_ENABLE, 0x7F);
	regmap_write(rm, QSERDES_RX + QSERDES_V5_RX_UCDR_FASTLOCK_COUNT_LOW, 0x00);
	regmap_write(rm, QSERDES_RX + QSERDES_V5_RX_UCDR_FASTLOCK_COUNT_HIGH, 0x01);
	regmap_write(rm, QSERDES_RX + QSERDES_V5_RX_UCDR_PI_CONTROLS, 0x81);
	regmap_write(rm, QSERDES_RX + QSERDES_V5_RX_UCDR_PI_CTRL2, 0x80);
	regmap_write(rm, QSERDES_RX + QSERDES_V5_RX_RX_TERM_BW, 0x04);
	regmap_write(rm, QSERDES_RX + QSERDES_V5_RX_VGA_CAL_CNTRL2, 0x08);
	regmap_write(rm, QSERDES_RX + QSERDES_V5_RX_GM_CAL, 0x0F);
	regmap_write(rm, QSERDES_RX + QSERDES_V5_RX_RX_EQU_ADAPTOR_CNTRL1, 0x04);
	regmap_write(rm, QSERDES_RX + QSERDES_V5_RX_RX_EQU_ADAPTOR_CNTRL2, 0x00);
	regmap_write(rm, QSERDES_RX + QSERDES_V5_RX_RX_EQU_ADAPTOR_CNTRL3, 0x4A);
	regmap_write(rm, QSERDES_RX + QSERDES_V5_RX_RX_EQU_ADAPTOR_CNTRL4, 0x0A);
	regmap_write(rm, QSERDES_RX + QSERDES_V5_RX_RX_IDAC_TSETTLE_LOW, 0x80);
	regmap_write(rm, QSERDES_RX + QSERDES_V5_RX_RX_IDAC_TSETTLE_HIGH, 0x01);
	regmap_write(rm, QSERDES_RX + QSERDES_V5_RX_RX_IDAC_MEASURE_TIME, 0x20);
	regmap_write(rm, QSERDES_RX + QSERDES_V5_RX_RX_EQ_OFFSET_ADAPTOR_CNTRL1, 0x17);
	regmap_write(rm, QSERDES_RX + QSERDES_V5_RX_RX_OFFSET_ADAPTOR_CNTRL2, 0x00);
	regmap_write(rm, QSERDES_RX + QSERDES_V5_RX_SIGDET_CNTRL, 0x0F);
	regmap_write(rm, QSERDES_RX + QSERDES_V5_RX_SIGDET_DEGLITCH_CNTRL, 0x1E);
	regmap_write(rm, QSERDES_RX + QSERDES_V5_RX_RX_BAND, 0x05);
	regmap_write(rm, QSERDES_RX + QSERDES_V5_RX_RX_MODE_00_LOW, 0xE0);
	regmap_write(rm, QSERDES_RX + QSERDES_V5_RX_RX_MODE_00_HIGH, 0xC8);
	regmap_write(rm, QSERDES_RX + QSERDES_V5_RX_RX_MODE_00_HIGH2, 0xC8);
	regmap_write(rm, QSERDES_RX + QSERDES_V5_RX_RX_MODE_00_HIGH3, 0x09);
	regmap_write(rm, QSERDES_RX + QSERDES_V5_RX_RX_MODE_00_HIGH4, 0xB1);
	regmap_write(rm, QSERDES_RX + QSERDES_V5_RX_RX_MODE_01_LOW, 0xE0);
	regmap_write(rm, QSERDES_RX + QSERDES_V5_RX_RX_MODE_01_HIGH, 0xC8);
	regmap_write(rm, QSERDES_RX + QSERDES_V5_RX_RX_MODE_01_HIGH2, 0xC8);
	regmap_write(rm, QSERDES_RX + QSERDES_V5_RX_RX_MODE_01_HIGH3, 0x09);
	regmap_write(rm, QSERDES_RX + QSERDES_V5_RX_RX_MODE_01_HIGH4, 0xB1);
	regmap_write(rm, QSERDES_RX + QSERDES_V5_RX_RX_MODE_10_LOW, 0xE0);
	regmap_write(rm, QSERDES_RX + QSERDES_V5_RX_RX_MODE_10_HIGH, 0xC8);
	regmap_write(rm, QSERDES_RX + QSERDES_V5_RX_RX_MODE_10_HIGH2, 0xC8);
	regmap_write(rm, QSERDES_RX + QSERDES_V5_RX_RX_MODE_10_HIGH3, 0x3B);
	regmap_write(rm, QSERDES_RX + QSERDES_V5_RX_RX_MODE_10_HIGH4, 0xB7);
	regmap_write(rm, QSERDES_RX + QSERDES_V5_RX_DCC_CTRL1, 0x0C);

	regmap_write(rm, QSERDES_PCS + QPHY_PCS_LINE_RESET_TIME, 0x0C);
	regmap_write(rm, QSERDES_PCS + QPHY_PCS_TX_LARGE_AMP_DRV_LVL, 0x1F);
	regmap_write(rm, QSERDES_PCS + QPHY_PCS_TX_SMALL_AMP_DRV_LVL, 0x03);
	regmap_write(rm, QSERDES_PCS + QPHY_PCS_TX_MID_TERM_CTRL1, 0x83);
	regmap_write(rm, QSERDES_PCS + QPHY_PCS_TX_MID_TERM_CTRL2, 0x08);
	regmap_write(rm, QSERDES_PCS + QPHY_PCS_SGMII_MISC_CTRL8, 0x0C);
	regmap_write(rm, QSERDES_PCS + QPHY_PCS_SW_RESET, 0x00);
	regmap_write(rm, QSERDES_PCS + QPHY_PCS_PHY_START, 0x01);
}

static void sgmii_init_2p5g(struct regmap *rm)
{
	regmap_write(rm, QSERDES_PCS + QPHY_PCS_SW_RESET, 0x01);
	regmap_write(rm, QSERDES_PCS + QPHY_PCS_POWER_DOWN_CONTROL, 0x01);

	regmap_write(rm, QSERDES_COM + QSERDES_V5_COM_PLL_IVCO, 0x0F);
	regmap_write(rm, QSERDES_COM + QSERDES_V5_COM_CP_CTRL_MODE0, 0x06);
	regmap_write(rm, QSERDES_COM + QSERDES_V5_COM_PLL_RCTRL_MODE0, 0x16);
	regmap_write(rm, QSERDES_COM + QSERDES_V5_COM_PLL_CCTRL_MODE0, 0x36);
	regmap_write(rm, QSERDES_COM + QSERDES_V5_COM_SYSCLK_EN_SEL, 0x1A);
	regmap_write(rm, QSERDES_COM + QSERDES_V5_COM_LOCK_CMP1_MODE0, 0x1A);
	regmap_write(rm, QSERDES_COM + QSERDES_V5_COM_LOCK_CMP2_MODE0, 0x41);
	regmap_write(rm, QSERDES_COM + QSERDES_V5_COM_DEC_START_MODE0, 0x7A);
	regmap_write(rm, QSERDES_COM + QSERDES_V5_COM_DIV_FRAC_START1_MODE0, 0x00);
	regmap_write(rm, QSERDES_COM + QSERDES_V5_COM_DIV_FRAC_START2_MODE0, 0x20);
	regmap_write(rm, QSERDES_COM + QSERDES_V5_COM_DIV_FRAC_START3_MODE0, 0x01);
	regmap_write(rm, QSERDES_COM + QSERDES_V5_COM_VCO_TUNE1_MODE0, 0xA1);
	regmap_write(rm, QSERDES_COM + QSERDES_V5_COM_VCO_TUNE2_MODE0, 0x02);
	regmap_write(rm, QSERDES_COM + QSERDES_V5_COM_VCO_TUNE_INITVAL2, 0x00);
	regmap_write(rm, QSERDES_COM + QSERDES_V5_COM_HSCLK_SEL, 0x03);
	regmap_write(rm, QSERDES_COM + QSERDES_V5_COM_HSCLK_HS_SWITCH_SEL, 0x00);
	regmap_write(rm, QSERDES_COM + QSERDES_V5_COM_CORECLK_DIV_MODE0, 0x05);
	regmap_write(rm, QSERDES_COM + QSERDES_V5_COM_CORE_CLK_EN, 0x00);
	regmap_write(rm, QSERDES_COM + QSERDES_V5_COM_BIN_VCOCAL_CMP_CODE1_MODE0, 0xCD);
	regmap_write(rm, QSERDES_COM + QSERDES_V5_COM_BIN_VCOCAL_CMP_CODE2_MODE0, 0x1C);
	regmap_write(rm, QSERDES_COM + QSERDES_V5_COM_BIN_VCOCAL_HSCLK_SEL, 0x11);

	regmap_write(rm, QSERDES_TX + QSERDES_V5_TX_TX_BAND, 0x04);
	regmap_write(rm, QSERDES_TX + QSERDES_V5_TX_SLEW_CNTL, 0x0A);
	regmap_write(rm, QSERDES_TX + QSERDES_V5_TX_RES_CODE_LANE_OFFSET_TX, 0x09);
	regmap_write(rm, QSERDES_TX + QSERDES_V5_TX_RES_CODE_LANE_OFFSET_RX, 0x02);
	regmap_write(rm, QSERDES_TX + QSERDES_V5_TX_LANE_MODE_1, 0x05);
	regmap_write(rm, QSERDES_TX + QSERDES_V5_TX_LANE_MODE_3, 0x00);
	regmap_write(rm, QSERDES_TX + QSERDES_V5_TX_RCV_DETECT_LVL_2, 0x12);
	regmap_write(rm, QSERDES_TX + QSERDES_V5_TX_TRAN_DRVR_EMP_EN, 0x0C);

	regmap_write(rm, QSERDES_RX + QSERDES_V5_RX_UCDR_FO_GAIN, 0x0A);
	regmap_write(rm, QSERDES_RX + QSERDES_V5_RX_UCDR_SO_GAIN, 0x06);
	regmap_write(rm, QSERDES_RX + QSERDES_V5_RX_UCDR_FASTLOCK_FO_GAIN, 0x0A);
	regmap_write(rm, QSERDES_RX + QSERDES_V5_RX_UCDR_SO_SATURATION_AND_ENABLE, 0x7F);
	regmap_write(rm, QSERDES_RX + QSERDES_V5_RX_UCDR_FASTLOCK_COUNT_LOW, 0x00);
	regmap_write(rm, QSERDES_RX + QSERDES_V5_RX_UCDR_FASTLOCK_COUNT_HIGH, 0x01);
	regmap_write(rm, QSERDES_RX + QSERDES_V5_RX_UCDR_PI_CONTROLS, 0x81);
	regmap_write(rm, QSERDES_RX + QSERDES_V5_RX_UCDR_PI_CTRL2, 0x80);
	regmap_write(rm, QSERDES_RX + QSERDES_V5_RX_RX_TERM_BW, 0x00);
	regmap_write(rm, QSERDES_RX + QSERDES_V5_RX_VGA_CAL_CNTRL2, 0x08);
	regmap_write(rm, QSERDES_RX + QSERDES_V5_RX_GM_CAL, 0x0F);
	regmap_write(rm, QSERDES_RX + QSERDES_V5_RX_RX_EQU_ADAPTOR_CNTRL1, 0x04);
	regmap_write(rm, QSERDES_RX + QSERDES_V5_RX_RX_EQU_ADAPTOR_CNTRL2, 0x00);
	regmap_write(rm, QSERDES_RX + QSERDES_V5_RX_RX_EQU_ADAPTOR_CNTRL3, 0x4A);
	regmap_write(rm, QSERDES_RX + QSERDES_V5_RX_RX_EQU_ADAPTOR_CNTRL4, 0x0A);
	regmap_write(rm, QSERDES_RX + QSERDES_V5_RX_RX_IDAC_TSETTLE_LOW, 0x80);
	regmap_write(rm, QSERDES_RX + QSERDES_V5_RX_RX_IDAC_TSETTLE_HIGH, 0x01);
	regmap_write(rm, QSERDES_RX + QSERDES_V5_RX_RX_IDAC_MEASURE_TIME, 0x20);
	regmap_write(rm, QSERDES_RX + QSERDES_V5_RX_RX_EQ_OFFSET_ADAPTOR_CNTRL1, 0x17);
	regmap_write(rm, QSERDES_RX + QSERDES_V5_RX_RX_OFFSET_ADAPTOR_CNTRL2, 0x00);
	regmap_write(rm, QSERDES_RX + QSERDES_V5_RX_SIGDET_CNTRL, 0x0F);
	regmap_write(rm, QSERDES_RX + QSERDES_V5_RX_SIGDET_DEGLITCH_CNTRL, 0x1E);
	regmap_write(rm, QSERDES_RX + QSERDES_V5_RX_RX_BAND, 0x18);
	regmap_write(rm, QSERDES_RX + QSERDES_V5_RX_RX_MODE_00_LOW, 0x18);
	regmap_write(rm, QSERDES_RX + QSERDES_V5_RX_RX_MODE_00_HIGH, 0xC8);
	regmap_write(rm, QSERDES_RX + QSERDES_V5_RX_RX_MODE_00_HIGH2, 0xC8);
	regmap_write(rm, QSERDES_RX + QSERDES_V5_RX_RX_MODE_00_HIGH3, 0x0C);
	regmap_write(rm, QSERDES_RX + QSERDES_V5_RX_RX_MODE_00_HIGH4, 0xB8);
	regmap_write(rm, QSERDES_RX + QSERDES_V5_RX_RX_MODE_01_LOW, 0xE0);
	regmap_write(rm, QSERDES_RX + QSERDES_V5_RX_RX_MODE_01_HIGH, 0xC8);
	regmap_write(rm, QSERDES_RX + QSERDES_V5_RX_RX_MODE_01_HIGH2, 0xC8);
	regmap_write(rm, QSERDES_RX + QSERDES_V5_RX_RX_MODE_01_HIGH3, 0x09);
	regmap_write(rm, QSERDES_RX + QSERDES_V5_RX_RX_MODE_01_HIGH4, 0xB1);
	regmap_write(rm, QSERDES_RX + QSERDES_V5_RX_RX_MODE_10_LOW, 0xE0);
	regmap_write(rm, QSERDES_RX + QSERDES_V5_RX_RX_MODE_10_HIGH, 0xC8);
	regmap_write(rm, QSERDES_RX + QSERDES_V5_RX_RX_MODE_10_HIGH2, 0xC8);
	regmap_write(rm, QSERDES_RX + QSERDES_V5_RX_RX_MODE_10_HIGH3, 0x3B);
	regmap_write(rm, QSERDES_RX + QSERDES_V5_RX_RX_MODE_10_HIGH4, 0xB7);
	regmap_write(rm, QSERDES_RX + QSERDES_V5_RX_DCC_CTRL1, 0x0C);

	regmap_write(rm, QSERDES_PCS + QPHY_PCS_LINE_RESET_TIME, 0x0C);
	regmap_write(rm, QSERDES_PCS + QPHY_PCS_TX_LARGE_AMP_DRV_LVL, 0x1F);
	regmap_write(rm, QSERDES_PCS + QPHY_PCS_TX_SMALL_AMP_DRV_LVL, 0x03);
	regmap_write(rm, QSERDES_PCS + QPHY_PCS_TX_MID_TERM_CTRL1, 0x83);
	regmap_write(rm, QSERDES_PCS + QPHY_PCS_TX_MID_TERM_CTRL2, 0x08);
	regmap_write(rm, QSERDES_PCS + QPHY_PCS_SGMII_MISC_CTRL8, 0x8C);
	regmap_write(rm, QSERDES_PCS + QPHY_PCS_SW_RESET, 0x00);
	regmap_write(rm, QSERDES_PCS + QPHY_PCS_PHY_START, 0x01);
}

/* ------------------------------------------------------------------ */
/* USXGMII 10G init sequence (QMP V7, Nord SA8797P)                   */
/* VCO: 12.5 GHz.  Firmware pre-configures QREF routing (TCSR/TLMM). */
/* ------------------------------------------------------------------ */

static void usxgmii_init_10g(struct regmap *rm)
{
	regmap_write(rm, QSERDES_PCS + QPHY_PCS_SW_RESET, 0x01);
	regmap_write(rm, QSERDES_PCS + QPHY_PCS_POWER_DOWN_CONTROL, 0x01);

	/* COM — 12.5 GHz VCO */
	regmap_write(rm, QSERDES_COM + QSERDES_V7_COM_BG_TIMER, 0x0A);
	regmap_write(rm, QSERDES_COM + QSERDES_V7_COM_PLL_IVCO, 0x0F);
	regmap_write(rm, QSERDES_COM + QSERDES_V7_COM_BIAS_EN_CLKBUFLR_EN, 0x07);
	regmap_write(rm, QSERDES_COM + QSERDES_V7_COM_CLK_ENABLE1, 0x0F);
	regmap_write(rm, QSERDES_COM + QSERDES_V7_COM_CP_CTRL_MODE0, 0x08);
	regmap_write(rm, QSERDES_COM + QSERDES_V7_COM_PLL_RCTRL_MODE0, 0x16);
	regmap_write(rm, QSERDES_COM + QSERDES_V7_COM_PLL_CCTRL_MODE0, 0x36);
	regmap_write(rm, QSERDES_COM + QSERDES_V7_COM_INTEGLOOP_GAIN0_MODE0, 0x1F);
	regmap_write(rm, QSERDES_COM + QSERDES_V7_COM_INTEGLOOP_GAIN1_MODE0, 0x00);
	regmap_write(rm, QSERDES_COM + QSERDES_V7_COM_PLL_EN, 0x03);
	regmap_write(rm, QSERDES_COM + QSERDES_V7_COM_SYSCLK_EN_SEL, 0x1A);
	regmap_write(rm, QSERDES_COM + QSERDES_V7_COM_LOCK_CMP1_MODE0, 0x23);
	regmap_write(rm, QSERDES_COM + QSERDES_V7_COM_LOCK_CMP2_MODE0, 0x43);
	regmap_write(rm, QSERDES_COM + QSERDES_V7_COM_DEC_START_MODE0, 0x43);
	regmap_write(rm, QSERDES_COM + QSERDES_V7_COM_DIV_FRAC_START1_MODE0, 0x00);
	regmap_write(rm, QSERDES_COM + QSERDES_V7_COM_DIV_FRAC_START2_MODE0, 0x38);
	regmap_write(rm, QSERDES_COM + QSERDES_V7_COM_DIV_FRAC_START3_MODE0, 0x02);
	regmap_write(rm, QSERDES_COM + QSERDES_V7_COM_VCO_TUNE1_MODE0, 0xE6);
	regmap_write(rm, QSERDES_COM + QSERDES_V7_COM_VCO_TUNE2_MODE0, 0x01);
	regmap_write(rm, QSERDES_COM + QSERDES_V7_COM_VCO_TUNE_INITVAL2, 0x00);
	regmap_write(rm, QSERDES_COM + QSERDES_V7_COM_HSCLK_SEL_1, 0x00);
	regmap_write(rm, QSERDES_COM + QSERDES_V7_COM_HSCLK_HS_SWITCH_SEL_1, 0x00);
	regmap_write(rm, QSERDES_COM + QSERDES_V7_COM_CORECLK_DIV_MODE0, 0x04);
	regmap_write(rm, QSERDES_COM + QSERDES_V7_COM_CORE_CLK_EN, 0x30);
	regmap_write(rm, QSERDES_COM + QSERDES_V7_COM_CMN_CONFIG_1, 0x16);
	regmap_write(rm, QSERDES_COM + QSERDES_V7_COM_BIN_VCOCAL_CMP_CODE1_MODE0, 0xD7);
	regmap_write(rm, QSERDES_COM + QSERDES_V7_COM_BIN_VCOCAL_CMP_CODE2_MODE0, 0x0F);
	regmap_write(rm, QSERDES_COM + QSERDES_V7_COM_BIN_VCOCAL_HSCLK_SEL_1, 0x11);

	/* TX */
	regmap_write(rm, QSERDES_TX + QSERDES_V7_TX_CLKBUF_ENABLE, 0x0D);
	regmap_write(rm, QSERDES_TX + QSERDES_V7_TX_TX_BAND, 0x04);
	regmap_write(rm, QSERDES_TX + QSERDES_V7_TX_SLEW_CNTL, 0x08);
	regmap_write(rm, QSERDES_TX + QSERDES_V7_TX_RES_CODE_LANE_OFFSET_TX, 0x09);
	regmap_write(rm, QSERDES_TX + QSERDES_V7_TX_RES_CODE_LANE_OFFSET_RX, 0x09);
	regmap_write(rm, QSERDES_TX + QSERDES_V7_TX_LANE_MODE_1, 0xF5);
	regmap_write(rm, QSERDES_TX + QSERDES_V7_TX_LANE_MODE_2, 0x06);
	regmap_write(rm, QSERDES_TX + QSERDES_V7_TX_LANE_MODE_3, 0x3F);
	regmap_write(rm, QSERDES_TX + QSERDES_V7_TX_LANE_MODE_4, 0x3F);
	regmap_write(rm, QSERDES_TX + QSERDES_V7_TX_LANE_MODE_5, 0x5F);
	regmap_write(rm, QSERDES_TX + QSERDES_V7_TX_RCV_DETECT_LVL_2, 0x12);
	regmap_write(rm, QSERDES_TX + QSERDES_V7_TX_TRAN_DRVR_EMP_EN, 0x0F);
	regmap_write(rm, QSERDES_TX + QSERDES_V7_TX_TX_EMP_POST1_LVL, 0x2B);

	/* RX */
	regmap_write(rm, QSERDES_RX + QSERDES_V7_RX_UCDR_FO_GAIN, 0x0D);
	regmap_write(rm, QSERDES_RX + QSERDES_V7_RX_UCDR_SO_GAIN, 0x03);
	regmap_write(rm, QSERDES_RX + QSERDES_V7_RX_UCDR_FASTLOCK_FO_GAIN, 0x0A);
	regmap_write(rm, QSERDES_RX + QSERDES_V7_RX_UCDR_SO_SATURATION_AND_ENABLE, 0x7F);
	regmap_write(rm, QSERDES_RX + QSERDES_V7_RX_UCDR_FASTLOCK_COUNT_LOW, 0x00);
	regmap_write(rm, QSERDES_RX + QSERDES_V7_RX_UCDR_FASTLOCK_COUNT_HIGH, 0x01);
	regmap_write(rm, QSERDES_RX + QSERDES_V7_RX_UCDR_PI_CONTROLS, 0x81);
	regmap_write(rm, QSERDES_RX + QSERDES_V7_RX_UCDR_PI_CTRL2, 0x81);
	regmap_write(rm, QSERDES_RX + QSERDES_V7_RX_UCDR_SB2_THRESH1, 0x11);
	regmap_write(rm, QSERDES_RX + QSERDES_V7_RX_UCDR_SB2_THRESH2, 0x22);
	regmap_write(rm, QSERDES_RX + QSERDES_V7_RX_RX_TERM_BW, 0x03);
	regmap_write(rm, QSERDES_RX + QSERDES_V7_RX_VGA_CAL_CNTRL2, 0x08);
	regmap_write(rm, QSERDES_RX + QSERDES_V7_RX_GM_CAL, 0x0F);
	regmap_write(rm, QSERDES_RX + QSERDES_V7_RX_RX_EQU_ADAPTOR_CNTRL1, 0x04);
	regmap_write(rm, QSERDES_RX + QSERDES_V7_RX_RX_EQU_ADAPTOR_CNTRL2, 0x00);
	regmap_write(rm, QSERDES_RX + QSERDES_V7_RX_RX_EQU_ADAPTOR_CNTRL3, 0x4A);
	regmap_write(rm, QSERDES_RX + QSERDES_V7_RX_RX_EQU_ADAPTOR_CNTRL4, 0x5A);
	regmap_write(rm, QSERDES_RX + QSERDES_V7_RX_RX_IDAC_TSETTLE_LOW, 0x80);
	regmap_write(rm, QSERDES_RX + QSERDES_V7_RX_RX_IDAC_TSETTLE_HIGH, 0x01);
	regmap_write(rm, QSERDES_RX + QSERDES_V7_RX_RX_IDAC_MEASURE_TIME, 0x20);
	regmap_write(rm, QSERDES_RX + QSERDES_V7_RX_RX_EQ_OFFSET_ADAPTOR_CNTRL1, 0x17);
	regmap_write(rm, QSERDES_RX + QSERDES_V7_RX_RX_OFFSET_ADAPTOR_CNTRL2, 0x00);
	regmap_write(rm, QSERDES_RX + QSERDES_V7_RX_SIGDET_CNTRL, 0x0F);
	regmap_write(rm, QSERDES_RX + QSERDES_V7_RX_SIGDET_DEGLITCH_CNTRL, 0x1E);
	regmap_write(rm, QSERDES_RX + QSERDES_V7_RX_RX_BAND, 0x18);
	regmap_write(rm, QSERDES_RX + QSERDES_V7_RX_RX_MODE_00_LOW, 0x1F);
	regmap_write(rm, QSERDES_RX + QSERDES_V7_RX_RX_MODE_00_HIGH, 0xBF);
	regmap_write(rm, QSERDES_RX + QSERDES_V7_RX_RX_MODE_00_HIGH2, 0xFF);
	regmap_write(rm, QSERDES_RX + QSERDES_V7_RX_RX_MODE_00_HIGH3, 0xDF);
	regmap_write(rm, QSERDES_RX + QSERDES_V7_RX_RX_MODE_00_HIGH4, 0xEF);
	regmap_write(rm, QSERDES_RX + QSERDES_V7_RX_RX_MODE_01_LOW, 0xE5);
	regmap_write(rm, QSERDES_RX + QSERDES_V7_RX_RX_MODE_01_HIGH, 0xC8);
	regmap_write(rm, QSERDES_RX + QSERDES_V7_RX_RX_MODE_01_HIGH2, 0xC8);
	regmap_write(rm, QSERDES_RX + QSERDES_V7_RX_RX_MODE_01_HIGH3, 0x14);
	regmap_write(rm, QSERDES_RX + QSERDES_V7_RX_RX_MODE_01_HIGH4, 0xB6);
	regmap_write(rm, QSERDES_RX + QSERDES_V7_RX_RX_MODE_10_LOW, 0xE0);
	regmap_write(rm, QSERDES_RX + QSERDES_V7_RX_RX_MODE_10_HIGH, 0xC8);
	regmap_write(rm, QSERDES_RX + QSERDES_V7_RX_RX_MODE_10_HIGH2, 0xC8);
	regmap_write(rm, QSERDES_RX + QSERDES_V7_RX_RX_MODE_10_HIGH3, 0x3B);
	regmap_write(rm, QSERDES_RX + QSERDES_V7_RX_RX_MODE_10_HIGH4, 0xB7);
	regmap_write(rm, QSERDES_RX + QSERDES_V7_RX_DCC_CTRL1, 0x0C);
	regmap_write(rm, QSERDES_RX + QSERDES_V7_RX_SIGDET_CAL_CTRL1, 0x00);
	regmap_write(rm, QSERDES_RX + QSERDES_V7_RX_SIGDET_CAL_CTRL2_AND_CDR_LOCK_EDGE, 0x00);

	/* PCS */
	regmap_write(rm, QSERDES_PCS + QPHY_PCS_LINE_RESET_TIME, 0x00);
	regmap_write(rm, QSERDES_PCS + QPHY_PCS_RETIME_BUFFER_EN, 0x01);
	regmap_write(rm, QSERDES_PCS + QPHY_PCS_TX_LARGE_AMP_DRV_LVL, 0x1A);
	regmap_write(rm, QSERDES_PCS + QPHY_PCS_TX_LARGE_AMP_POST_EMP_LVL, 0x0B);
	regmap_write(rm, QSERDES_PCS + QPHY_PCS_TX_SMALL_AMP_DRV_LVL, 0x03);
	regmap_write(rm, QSERDES_PCS + QPHY_PCS_TX_SMALL_AMP_POST_EMP_LVL, 0x00);
	regmap_write(rm, QSERDES_PCS + QPHY_PCS_SGMII_MISC_CTRL7, 0x00);
	regmap_write(rm, QSERDES_PCS + QPHY_PCS_SGMII_MISC_CTRL8, 0x14);
	regmap_write(rm, QSERDES_PCS + QPHY_PCS_TX_MID_TERM_CTRL1, 0x83);
	regmap_write(rm, QSERDES_PCS + QPHY_PCS_TX_MID_TERM_CTRL2, 0x08);
	regmap_write(rm, QSERDES_PCS + QPHY_PCS_SW_RESET, 0x00);
	regmap_write(rm, QSERDES_PCS + QPHY_PCS_PHY_START, 0x01);
	udelay(5);
	/* Second write required by databook to latch PHY_START */
	regmap_write(rm, QSERDES_PCS + QPHY_PCS_PHY_START, 0x01);
}

/* ------------------------------------------------------------------ */
/* SGMII phy_ops                                                       */
/* ------------------------------------------------------------------ */

static int sgmii_calibrate(struct phy *phy)
{
	struct qcom_serdes_data *data = phy_get_drvdata(phy);
	struct device *dev = &phy->dev;
	int ret;

	switch (data->interface) {
	case PHY_INTERFACE_MODE_SGMII:
	case PHY_INTERFACE_MODE_1000BASEX:
		sgmii_init_1g(data->regmap);
		break;
	case PHY_INTERFACE_MODE_2500BASEX:
		sgmii_init_2p5g(data->regmap);
		break;
	default:
		return -EINVAL;
	}

	ret = sgmii_poll_ready(data->regmap);
	if (ret)
		dev_err(dev, "SGMII SerDes calibration timed out\n");

	return ret;
}

static int sgmii_power_on(struct phy *phy)
{
	struct qcom_serdes_data *data = phy_get_drvdata(phy);
	int ret;

	ret = regulator_bulk_enable(QCOM_SERDES_NUM_SUPPLIES, data->vregs);
	if (ret)
		return ret;

	ret = clk_prepare_enable(data->refclk);
	if (ret)
		goto err_disable_regulators;

	ret = sgmii_calibrate(phy);
	if (ret)
		goto err_disable_clk;

	return 0;

err_disable_clk:
	clk_disable_unprepare(data->refclk);
err_disable_regulators:
	regulator_bulk_disable(QCOM_SERDES_NUM_SUPPLIES, data->vregs);
	return ret;
}

static int sgmii_power_off(struct phy *phy)
{
	struct qcom_serdes_data *data = phy_get_drvdata(phy);
	struct regmap *rm = data->regmap;

	regmap_write(rm, QSERDES_PCS + QPHY_PCS_TX_MID_TERM_CTRL2, 0x08);
	regmap_write(rm, QSERDES_PCS + QPHY_PCS_SW_RESET, 0x01);
	udelay(100);
	regmap_write(rm, QSERDES_PCS + QPHY_PCS_SW_RESET, 0x00);
	regmap_write(rm, QSERDES_PCS + QPHY_PCS_PHY_START, 0x01);

	clk_disable_unprepare(data->refclk);
	regulator_bulk_disable(QCOM_SERDES_NUM_SUPPLIES, data->vregs);

	return 0;
}

static int sgmii_set_mode(struct phy *phy, enum phy_mode mode, int submode)
{
	struct qcom_serdes_data *data = phy_get_drvdata(phy);

	if (mode != PHY_MODE_ETHERNET)
		return -EINVAL;

	switch (submode) {
	case PHY_INTERFACE_MODE_SGMII:
	case PHY_INTERFACE_MODE_1000BASEX:
	case PHY_INTERFACE_MODE_2500BASEX:
		break;
	default:
		return -EINVAL;
	}

	if (submode != data->interface)
		data->interface = submode;

	if (phy->power_count == 0)
		return 0;

	return sgmii_calibrate(phy);
}

static int sgmii_validate(struct phy *phy, enum phy_mode mode, int submode,
			   union phy_configure_opts *opts)
{
	if (mode != PHY_MODE_ETHERNET)
		return -EINVAL;
	if (submode == PHY_INTERFACE_MODE_SGMII ||
	    submode == PHY_INTERFACE_MODE_1000BASEX ||
	    submode == PHY_INTERFACE_MODE_2500BASEX)
		return 0;
	return -EINVAL;
}

static const struct phy_ops sgmii_ops = {
	.power_on	= sgmii_power_on,
	.power_off	= sgmii_power_off,
	.set_mode	= sgmii_set_mode,
	.validate	= sgmii_validate,
	.calibrate	= sgmii_calibrate,
	.owner		= THIS_MODULE,
};

/* ------------------------------------------------------------------ */
/* USXGMII phy_ops                                                     */
/* ------------------------------------------------------------------ */

static int usxgmii_calibrate(struct phy *phy)
{
	struct qcom_serdes_data *data = phy_get_drvdata(phy);
	struct device *dev = &phy->dev;
	int ret;

	usxgmii_init_10g(data->regmap);

	ret = usxgmii_poll_ready(data->regmap);
	if (ret)
		dev_err(dev, "USXGMII SerDes calibration timed out\n");

	return ret;
}

static int usxgmii_power_on(struct phy *phy)
{
	struct qcom_serdes_data *data = phy_get_drvdata(phy);
	int ret;

	ret = regulator_bulk_enable(QCOM_SERDES_NUM_SUPPLIES, data->vregs);
	if (ret)
		return ret;

	ret = clk_prepare_enable(data->refclk);
	if (ret)
		goto err_disable_regulators;

	ret = usxgmii_calibrate(phy);
	if (ret)
		goto err_disable_clk;

	return 0;

err_disable_clk:
	clk_disable_unprepare(data->refclk);
err_disable_regulators:
	regulator_bulk_disable(QCOM_SERDES_NUM_SUPPLIES, data->vregs);
	return ret;
}

static int usxgmii_power_off(struct phy *phy)
{
	struct qcom_serdes_data *data = phy_get_drvdata(phy);

	regmap_write(data->regmap, QSERDES_PCS + QPHY_PCS_TX_MID_TERM_CTRL2, 0x08);
	regmap_write(data->regmap, QSERDES_PCS + QPHY_PCS_SW_RESET, 0x01);

	clk_disable_unprepare(data->refclk);
	regulator_bulk_disable(QCOM_SERDES_NUM_SUPPLIES, data->vregs);

	return 0;
}

static const struct phy_ops usxgmii_ops = {
	.power_on	= usxgmii_power_on,
	.power_off	= usxgmii_power_off,
	.calibrate	= usxgmii_calibrate,
	.owner		= THIS_MODULE,
};

/* ------------------------------------------------------------------ */
/* Probe                                                               */
/* ------------------------------------------------------------------ */

static const struct regmap_config serdes_regmap_cfg = {
	.reg_bits		= 32,
	.val_bits		= 32,
	.reg_stride		= 4,
	.use_relaxed_mmio	= true,
	.disable_locking	= true,
};

static int qcom_serdes_eth_probe(struct platform_device *pdev)
{
	const struct qcom_serdes_cfg *cfg;
	struct qcom_serdes_data *data;
	struct device *dev = &pdev->dev;
	struct phy_provider *provider;
	const struct phy_ops *ops;
	void __iomem *base;
	struct phy *phy;
	int ret;

	cfg = of_device_get_match_data(dev);
	if (!cfg)
		return -EINVAL;

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->cfg = cfg;

	base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(base))
		return PTR_ERR(base);

	data->regmap = devm_regmap_init_mmio(dev, base, &serdes_regmap_cfg);
	if (IS_ERR(data->regmap))
		return PTR_ERR(data->regmap);

	ret = devm_regulator_bulk_get_const(dev, QCOM_SERDES_NUM_SUPPLIES,
					    cfg->vregs, &data->vregs);
	if (ret)
		return ret;

	data->refclk = devm_clk_get(dev, "sgmi_ref");
	if (IS_ERR(data->refclk))
		return dev_err_probe(dev, PTR_ERR(data->refclk),
				     "failed to get sgmi_ref clock\n");

	ops = cfg->usxgmii ? &usxgmii_ops : &sgmii_ops;

	if (!cfg->usxgmii)
		data->interface = PHY_INTERFACE_MODE_SGMII;

	phy = devm_phy_create(dev, NULL, ops);
	if (IS_ERR(phy))
		return PTR_ERR(phy);

	phy_set_drvdata(phy, data);

	provider = devm_of_phy_provider_register(dev, of_phy_simple_xlate);
	return PTR_ERR_OR_ZERO(provider);
}

/* ------------------------------------------------------------------ */
/* OF match table                                                      */
/* ------------------------------------------------------------------ */

static const struct qcom_serdes_cfg sgmii_cfg = {
	.usxgmii = false,
	.vregs   = sgmii_vregs,
};

static const struct qcom_serdes_cfg usxgmii_cfg = {
	.usxgmii = true,
	.vregs   = usxgmii_vregs,
};

static const struct of_device_id qcom_serdes_eth_of_match[] = {
	{ .compatible = "qcom,sa8775p-dwmac-sgmii-phy", .data = &sgmii_cfg   },
	{ .compatible = "qcom,nord-dwmac-usxgmii-phy",  .data = &usxgmii_cfg },
	{ }
};
MODULE_DEVICE_TABLE(of, qcom_serdes_eth_of_match);

static struct platform_driver qcom_serdes_eth_driver = {
	.probe	= qcom_serdes_eth_probe,
	.driver = {
		.name		= "qcom-dwmac-serdes",
		.of_match_table	= qcom_serdes_eth_of_match,
	},
};
module_platform_driver(qcom_serdes_eth_driver);

MODULE_DESCRIPTION("Qualcomm DWMAC Ethernet SerDes PHY driver");
MODULE_LICENSE("GPL");
