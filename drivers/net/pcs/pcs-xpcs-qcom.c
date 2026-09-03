// SPDX-License-Identifier: GPL-2.0
/*
 * Qualcomm XPCS platform driver.
 *
 * This driver bridges between the DW XPCS core (pcs-xpcs.c) and the
 * Qualcomm SA8797P (Nord) XPCS, which is memory-mapped over APB rather
 * than connected via MDIO.
 *
 * MMIO layout
 * -----------
 * The XPCS APB window is 0x5100 bytes, accessed with 32-bit reads/writes.
 * Registers are organised by MMD:
 *
 *   MDIO_MMD_PCS  (devad  3)
 *     SR XS PCS (standard):  offset = reg << 2           [0x0000–0x1FFC]
 *     VR XS PCS (vendor):    offset = 0x2000 + (reg & 0x7fff) << 2
 *
 *   MDIO_MMD_VEND2 (devad 31)
 *     SR MII (standard):     offset = 0x4000 + reg << 2
 *     VR MII (vendor):       offset = 0x5000 + (reg & 0x7fff) << 2
 *
 * The DW XPCS core uses BIT(15) in the register address to distinguish
 * vendor (VR) from standard (SR) registers within a given MMD; this
 * driver's regmap translates that convention to the MMIO layout above.
 *
 * Address encoding passed to the regmap: ((devad & 0x1f) << 16) | reg
 * (same encoding as devm_xpcs_regmap_register() expects).
 *
 * PHYSID interception
 * -------------------
 * The hardware reports the generic Synopsys ID (0x7996ced0), which would
 * select synopsys_xpcs_compat (DW_AN_C73) instead of qcom_xpcs_compat
 * (DW_AN_C37_USXGMII).  Reads of PHYSID1/2 on MDIO_MMD_PCS are therefore
 * intercepted to return QCOM_XPCS_ID, matching the qcom_xpcs_compat entry.
 *
 * Copyright (c) 2024, Qualcomm Technologies, Inc.
 */

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/io.h>
#include <linux/mii.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/pcs/pcs-xpcs.h>
#include <linux/pcs/pcs-xpcs-regmap.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

/* Total size of the XPCS APB window */
#define QCOM_XPCS_WINDOW_SIZE	0x5100

static ptrdiff_t qcom_xpcs_addr_to_offset(unsigned int addr)
{
	int devad = (addr >> 16) & 0x1f;
	int reg   = addr & 0xffff;
	ptrdiff_t offset;

	switch (devad) {
	case MDIO_MMD_PCS:
		if (reg & BIT(15))
			offset = 0x2000 + ((reg & 0x7fff) << 2);
		else
			offset = reg << 2;
		break;
	case MDIO_MMD_VEND2:
		if (reg & BIT(15))
			offset = 0x5000 + ((reg & 0x7fff) << 2);
		else
			offset = 0x4000 + (reg << 2);
		break;
	default:
		return -EINVAL;
	}

	if (offset >= QCOM_XPCS_WINDOW_SIZE)
		return -ERANGE;

	return offset;
}

static int qcom_xpcs_reg_read(void *ctx, unsigned int addr, unsigned int *val)
{
	void __iomem *base = ctx;
	int devad = (addr >> 16) & 0x1f;
	int reg   = addr & 0xffff;
	ptrdiff_t off;

	/*
	 * Return QCOM_XPCS_ID so xpcs_identify() selects qcom_xpcs_compat
	 * (DW_AN_C37_USXGMII for USXGMII) rather than synopsys_xpcs_compat
	 * (DW_AN_C73).  The hardware reports the generic Synopsys ID.
	 */
	if (devad == MDIO_MMD_PCS && reg == MII_PHYSID1) {
		*val = QCOM_XPCS_ID >> 16;
		return 0;
	}
	if (devad == MDIO_MMD_PCS && reg == MII_PHYSID2) {
		*val = QCOM_XPCS_ID & 0xffff;
		return 0;
	}

	off = qcom_xpcs_addr_to_offset(addr);
	if (off < 0) {
		/*
		 * -EINVAL: unsupported MMD (e.g. PMA/PMD probed by xpcs_read_ids).
		 * -ERANGE: offset outside the window — programming error.
		 */
		if (off == -ERANGE)
			pr_err_ratelimited("%s: addr 0x%08x maps outside window\n",
					   __func__, addr);
		*val = 0xffff;
		return 0;
	}

	*val = readl(base + off) & 0xffff;
	return 0;
}

static int qcom_xpcs_reg_write(void *ctx, unsigned int addr, unsigned int val)
{
	void __iomem *base = ctx;
	ptrdiff_t off = qcom_xpcs_addr_to_offset(addr);

	if (off < 0)
		return off;

	writel(val & 0xffff, base + off);
	return 0;
}

static const struct regmap_config qcom_xpcs_regmap_cfg = {
	.reg_bits  = 32,
	.val_bits  = 32,
	.reg_read  = qcom_xpcs_reg_read,
	.reg_write = qcom_xpcs_reg_write,
};

static int qcom_xpcs_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct dw_xpcs *xpcs;
	struct regmap *regmap;
	void __iomem *base;
	struct clk *clk;

	base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(base))
		return PTR_ERR(base);

	clk = devm_clk_get_optional_enabled(dev, "apb");
	if (IS_ERR(clk))
		return dev_err_probe(dev, PTR_ERR(clk),
				     "failed to enable apb clock\n");

	regmap = devm_regmap_init(dev, NULL, base, &qcom_xpcs_regmap_cfg);
	if (IS_ERR(regmap))
		return PTR_ERR(regmap);

	xpcs = devm_xpcs_regmap_register(dev, &(struct xpcs_regmap_config){
		.reg_indir = false,
		.regmap    = regmap,
	});
	if (IS_ERR(xpcs))
		return dev_err_probe(dev, PTR_ERR(xpcs),
				     "failed to register XPCS\n");

	platform_set_drvdata(pdev, xpcs);
	return 0;
}

static const struct of_device_id qcom_xpcs_of_match[] = {
	{ .compatible = "qcom,nord-xpcs" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, qcom_xpcs_of_match);

static struct platform_driver qcom_xpcs_driver = {
	.probe = qcom_xpcs_probe,
	.driver = {
		.name = "qcom-xpcs",
		.of_match_table = qcom_xpcs_of_match,
	},
};
module_platform_driver(qcom_xpcs_driver);

MODULE_DESCRIPTION("Qualcomm XPCS platform driver");
MODULE_LICENSE("GPL");
