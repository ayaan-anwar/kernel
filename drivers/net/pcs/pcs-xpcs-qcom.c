// SPDX-License-Identifier: GPL-2.0
/*
 * Qualcomm XPCS platform driver for Nord (SA8797P) dual 10G USXGMII.
 *
 * The XPCS APB window is a 0x5100-byte region accessible via 32-bit
 * reads/writes.  Registers are NOT laid out at the standard DW viewport
 * offset; instead each MMD is assigned a fixed MMIO base address:
 *
 *   MDIO_MMD_PCS  (devad  3)  SR XS PCS → reg << 2          (base 0x0000)
 *                              VR XS PCS → 0x2000 + (reg & 0x7fff) << 2
 *   MDIO_MMD_VEND2 (devad 31) SR MII    → 0x4000 + reg << 2
 *                              VR MII    → 0x5000 + (reg & 0x7fff) << 2
 *
 * devm_xpcs_regmap_register() bridges between pcs-xpcs.c and this regmap;
 * in direct mode (reg_indir=false) it passes (devad<<16)|reg as the regmap
 * address to our reg_read/reg_write callbacks, which apply the translation
 * above.
 *
 * MII_PHYSID1/2 reads on MDIO_MMD_PCS are intercepted to return
 * QCOM_NORD_XPCS_ID so that xpcs_identify() selects qcom_nord_xpcs_compat
 * (DW_AN_C37_USXGMII) rather than the generic Synopsys path (DW_AN_C73).
 * The hardware reports the generic Synopsys ID 0x7996CED0.
 *
 * struct dw_xpcs * is stored in platform drvdata.  stmmac retrieves it
 * via the plat_dat->pcs_init callback wired up in dwmac-qcom-ethqos.c.
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

#define QCOM_XPCS_REG_SIZE	0x5100

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

	if (offset < 0 || offset >= QCOM_XPCS_REG_SIZE)
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
	 * Return QCOM_NORD_XPCS_ID so xpcs_identify() in pcs-xpcs.c
	 * selects qcom_nord_xpcs_compat (DW_AN_C37_USXGMII for USXGMII)
	 * instead of synopsys_xpcs_compat (DW_AN_C73).
	 * Precedent: NXP SJA1105 uses distinct chip IDs; we fabricate one
	 * here because the hardware reports the generic Synopsys ID.
	 */
	if (devad == MDIO_MMD_PCS && reg == MII_PHYSID1) {
		*val = QCOM_NORD_XPCS_ID >> 16;
		return 0;
	}
	if (devad == MDIO_MMD_PCS && reg == MII_PHYSID2) {
		*val = QCOM_NORD_XPCS_ID & 0xffff;
		return 0;
	}

	off = qcom_xpcs_addr_to_offset(addr);
	if (off < 0) {
		/* -EINVAL: unsupported MMD (e.g. PMA/PMD probed by xpcs_read_ids).
		 * -ERANGE: translated offset outside the 0x5100 window — real bug. */
		if (off != -EINVAL)
			pr_err_ratelimited("%s: invalid addr=0x%08x (%pe)\n",
					   __func__, addr, ERR_PTR(off));
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
		return -EINVAL;

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
	void __iomem *base;
	struct regmap *regmap;
	struct dw_xpcs *xpcs;
	struct clk *clk;

	base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(base))
		return PTR_ERR(base);

	/* APB must be up before any register access. */
	clk = devm_clk_get_optional_enabled(dev, "apb");
	if (IS_ERR(clk))
		return dev_err_probe(dev, PTR_ERR(clk),
				     "failed to enable apb clock\n");

	/* RPCS Rx/Tx feed the PCS data path. */
	clk = devm_clk_get_optional_enabled(dev, "rpcs-rx");
	if (IS_ERR(clk))
		return dev_err_probe(dev, PTR_ERR(clk),
				     "failed to enable rpcs-rx clock\n");

	clk = devm_clk_get_optional_enabled(dev, "rpcs-tx");
	if (IS_ERR(clk))
		return dev_err_probe(dev, PTR_ERR(clk),
				     "failed to enable rpcs-tx clock\n");

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
	dev_info(dev, "registered Qualcomm Nord XPCS\n");
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
		.name = "qcom-nord-xpcs",
		.of_match_table = qcom_xpcs_of_match,
	},
};
module_platform_driver(qcom_xpcs_driver);

MODULE_DESCRIPTION("Qualcomm Nord SA8797P XPCS platform driver");
MODULE_LICENSE("GPL");
