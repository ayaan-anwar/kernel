/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * SA8797P (Nord) QMP QSERDES V7 ethernet SerDes register additions.
 *
 * Supplements phy-qcom-qmp-qserdes-com-v7.h and
 * phy-qcom-qmp-qserdes-txrx-v7.h with registers absent from those
 * PCIe-oriented V7 headers.  Include after both V7 headers.
 *
 * Block bases relative to the SGMII_PHY regmap base:
 *   COM  0x000   TX0  0x400   RX0  0x600   PCS  0xC00
 *
 * All offsets verified against IPCAT nordschleife_2.0 (SGMII_PHY_0
 * at 0x088F8000, SGMII_PHY_1 at 0x088FC000).
 *
 * Copyright (c) 2024, Qualcomm Technologies, Inc.
 */

#ifndef __PHY_QCOM_QSERDES_NORD_ETH_H__
#define __PHY_QCOM_QSERDES_NORD_ETH_H__

/* COM — registers absent from phy-qcom-qmp-qserdes-com-v7.h */
#define QSERDES_V7_COM_BIAS_EN_CLKBUFLR_EN		0x0dc
#define QSERDES_V7_COM_PLL_EN				0x0ec
/* QSERDES_V7_COM_CORECLK_DIV_MODE0 aliases PLL_CORE_CLK_DIV_MODE0 */
#define QSERDES_V7_COM_CORECLK_DIV_MODE0		QSERDES_V7_COM_PLL_CORE_CLK_DIV_MODE0
#define QSERDES_V7_COM_BIN_VCOCAL_HSCLK_SEL_1		0x19c

/* TX — registers absent from phy-qcom-qmp-qserdes-txrx-v7.h */
#define QSERDES_V7_TX_TX_EMP_POST1_LVL			0x00c
#define QSERDES_V7_TX_SLEW_CNTL			0x028

/* RX — registers absent from phy-qcom-qmp-qserdes-txrx-v7.h */
#define QSERDES_V7_RX_UCDR_PI_CTRL2			0x048
#define QSERDES_V7_RX_RX_TERM_BW			0x080
#define QSERDES_V7_RX_RX_EQU_ADAPTOR_CNTRL1		0x0e8
#define QSERDES_V7_RX_RX_IDAC_MEASURE_TIME		0x100
#define QSERDES_V7_RX_RX_OFFSET_ADAPTOR_CNTRL2		0x114
#define QSERDES_V7_RX_RX_BAND				0x128
#define QSERDES_V7_RX_SIGDET_CAL_CTRL2_AND_CDR_LOCK_EDGE	0x1e8

/* PCS — registers absent from phy-qcom-qmp-pcs-sgmii.h */
#define QPHY_PCS_RETIME_BUFFER_EN			0x018
#define QPHY_PCS_TX_LARGE_AMP_POST_EMP_LVL		0x024
#define QPHY_PCS_TX_SMALL_AMP_POST_EMP_LVL		0x02c
#define QPHY_PCS_SGMII_MISC_CTRL7			0x114

#endif /* __PHY_QCOM_QSERDES_NORD_ETH_H__ */
