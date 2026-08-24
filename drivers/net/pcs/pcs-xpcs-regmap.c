// SPDX-License-Identifier: GPL-2.0
/*
 * Support for accessing the DW XPCS via a regmap-backed MDIO bus.
 *
 * Copyright (C) 2024 Broadcom
 * Author: Alex Elder <elder@linaro.org>
 *
 * Based on pcs-xpcs-plat.c
 */

#include <linux/atomic.h>
#include <linux/device.h>
#include <linux/mdio.h>
#include <linux/mdio/mdio-regmap.h>
#include <linux/mii.h>
#include <linux/phy/phy.h>
#include <linux/pcs/pcs-xpcs-regmap.h>
#include <linux/regmap.h>

#include "pcs-xpcs.h"

#define DW_VR_CSR_VIEWPORT	0xff

struct dw_xpcs_regmap {
	struct device *dev;
	struct mii_bus *bus;
	struct regmap *regmap;
	bool reg_indir;
};

static ptrdiff_t xpcs_regmap_addr(int dev, int reg)
{
	return FIELD_PREP(0x1f0000, dev) | FIELD_PREP(0xffff, reg);
}

static int xpcs_regmap_read_direct(struct dw_xpcs_regmap *pxpcs,
				   int dev, int reg, u16 *val)
{
	unsigned int regval;
	int ret;

	ret = regmap_read(pxpcs->regmap, xpcs_regmap_addr(dev, reg), &regval);
	if (ret)
		return ret;

	*val = regval & 0xffff;

	return 0;
}

static int xpcs_regmap_write_direct(struct dw_xpcs_regmap *pxpcs,
				    int dev, int reg, u16 val)
{
	return regmap_write(pxpcs->regmap, xpcs_regmap_addr(dev, reg), val);
}

static int xpcs_regmap_read_indir(struct dw_xpcs_regmap *pxpcs,
				  int dev, int reg, u16 *val)
{
	u32 page = FIELD_GET(0x1fff00, xpcs_regmap_addr(dev, reg));
	u32 offset = FIELD_GET(0xff, xpcs_regmap_addr(dev, reg));
	unsigned int regval;
	int ret;

	ret = regmap_write(pxpcs->regmap,
			   xpcs_regmap_addr(dev, DW_VR_CSR_VIEWPORT), page);
	if (ret)
		return ret;

	ret = regmap_read(pxpcs->regmap, offset, &regval);
	if (ret)
		return ret;

	*val = regval & 0xffff;

	return 0;
}

static int xpcs_regmap_write_indir(struct dw_xpcs_regmap *pxpcs,
				   int dev, int reg, u16 val)
{
	u32 page = FIELD_GET(0x1fff00, xpcs_regmap_addr(dev, reg));
	u32 offset = FIELD_GET(0xff, xpcs_regmap_addr(dev, reg));
	int ret;

	ret = regmap_write(pxpcs->regmap,
			   xpcs_regmap_addr(dev, DW_VR_CSR_VIEWPORT), page);
	if (ret)
		return ret;

	return regmap_write(pxpcs->regmap, offset, val);
}

static int xpcs_regmap_mii_read(struct dw_xpcs_regmap *pxpcs,
				int dev, int reg)
{
	u16 val;
	int ret;

	if (pxpcs->reg_indir)
		ret = xpcs_regmap_read_indir(pxpcs, dev, reg, &val);
	else
		ret = xpcs_regmap_read_direct(pxpcs, dev, reg, &val);

	return ret < 0 ? ret : val;
}

static int xpcs_regmap_mii_write(struct dw_xpcs_regmap *pxpcs,
				 int dev, int reg, u16 val)
{
	if (pxpcs->reg_indir)
		return xpcs_regmap_write_indir(pxpcs, dev, reg, val);

	return xpcs_regmap_write_direct(pxpcs, dev, reg, val);
}

static int xpcs_regmap_read_c22(struct mii_bus *bus, int addr, int reg)
{
	struct dw_xpcs_regmap *pxpcs = bus->priv;

	if (addr != 0)
		return -ENODEV;

	return xpcs_regmap_mii_read(pxpcs, MDIO_MMD_VEND2, reg);
}

static int xpcs_regmap_write_c22(struct mii_bus *bus, int addr, int reg,
				 u16 val)
{
	struct dw_xpcs_regmap *pxpcs = bus->priv;

	if (addr != 0)
		return -ENODEV;

	return xpcs_regmap_mii_write(pxpcs, MDIO_MMD_VEND2, reg, val);
}

static int xpcs_regmap_read_c45(struct mii_bus *bus, int addr, int dev,
				int reg)
{
	struct dw_xpcs_regmap *pxpcs = bus->priv;

	if (addr != 0)
		return -ENODEV;

	return xpcs_regmap_mii_read(pxpcs, dev, reg);
}

static int xpcs_regmap_write_c45(struct mii_bus *bus, int addr, int dev,
				 int reg, u16 val)
{
	struct dw_xpcs_regmap *pxpcs = bus->priv;

	if (addr != 0)
		return -ENODEV;

	return xpcs_regmap_mii_write(pxpcs, dev, reg, val);
}

static void xpcs_regmap_unregister(void *data)
{
	xpcs_destroy(data);
}

static void xpcs_regmap_unregister_with_mdio(void *data)
{
	struct dw_xpcs *xpcs = data;

	mdio_device_remove(xpcs->mdiodev);
	xpcs_destroy(xpcs);
}

static atomic_t xpcs_regmap_id = ATOMIC_INIT(0);

/**
 * devm_xpcs_regmap_register - Register an XPCS using a regmap
 * @dev:    Device to register with
 * @config: XPCS regmap configuration
 *
 * Creates a virtual MDIO bus backed by the provided regmap and registers
 * a DW XPCS device on it.  Supports both direct and indirect register
 * access modes.
 *
 * Returns a pointer to the registered dw_xpcs, or an ERR_PTR on failure.
 */
struct dw_xpcs *devm_xpcs_regmap_register(struct device *dev,
					   const struct xpcs_regmap_config *config)
{
	struct dw_xpcs_regmap *pxpcs;
	struct dw_xpcs *xpcs;
	int id, ret;

	pxpcs = devm_kzalloc(dev, sizeof(*pxpcs), GFP_KERNEL);
	if (!pxpcs)
		return ERR_PTR(-ENOMEM);

	pxpcs->dev = dev;
	pxpcs->regmap = config->regmap;
	pxpcs->reg_indir = config->reg_indir;

	pxpcs->bus = devm_mdiobus_alloc(dev);
	if (!pxpcs->bus)
		return ERR_PTR(-ENOMEM);

	id = atomic_inc_return(&xpcs_regmap_id) - 1;
	snprintf(pxpcs->bus->id, MII_BUS_ID_SIZE, "%s", dev_name(dev));

	pxpcs->bus->name = "DW XPCS MCI/APB3";
	pxpcs->bus->read = xpcs_regmap_read_c22;
	pxpcs->bus->write = xpcs_regmap_write_c22;
	pxpcs->bus->read_c45 = xpcs_regmap_read_c45;
	pxpcs->bus->write_c45 = xpcs_regmap_write_c45;
	pxpcs->bus->phy_mask = ~0;	/* no PHY scanning — XPCS is not a PHY bus */
	pxpcs->bus->priv = pxpcs;
	pxpcs->bus->parent = dev;

	ret = devm_mdiobus_register(dev, pxpcs->bus);
	if (ret)
		return ERR_PTR(ret);

	xpcs = xpcs_create_mdiodev(pxpcs->bus, 0);
	if (IS_ERR(xpcs))
		return xpcs;

	/*
	 * Transfer the caller's firmware node to the mdio_device so that
	 * xpcs_create_fwnode() (called from stmmac_mdio.c when parsing
	 * "pcs-handle") can locate this instance by fwnode.
	 *
	 * Set fwnode BEFORE device_add() so bus_find_device_by_fwnode()
	 * sees the correct fwnode from the moment the device appears on the bus.
	 */
	device_set_node(&xpcs->mdiodev->dev, fwnode_handle_get(dev_fwnode(dev)));

	/*
	 * Register the mdiodev on the MDIO bus so that fwnode_mdio_find_device()
	 * can locate it.  mdio_device_create() only calls device_initialize();
	 * bus_find_device_by_fwnode() only iterates devices added via device_add().
	 */
	ret = mdio_device_register(xpcs->mdiodev);
	if (ret) {
		xpcs_destroy(xpcs);
		return ERR_PTR(ret);
	}

	dev_info(dev, "XPCS mdiodev registered (fwnode=%s)\n",
		 dev_name(&xpcs->mdiodev->dev));

	/*
	 * Use the combined cleanup function so mdio_device_remove() runs before
	 * xpcs_destroy() — device must be removed from the bus before the last
	 * mdiodev reference is dropped.
	 */
	ret = devm_add_action_or_reset(dev, xpcs_regmap_unregister_with_mdio, xpcs);
	if (ret)
		return ERR_PTR(ret);

	return xpcs;
}
EXPORT_SYMBOL_GPL(devm_xpcs_regmap_register);
