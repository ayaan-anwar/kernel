// SPDX-License-Identifier: GPL-2.0
/*
 * Regmap registration API for Synopsys DesignWare XPCS platform drivers.
 *
 * Wraps a caller-supplied regmap in a synthetic MDIO bus so that the DW XPCS
 * core (pcs-xpcs.c) can drive it via its standard MDIO C45 abstraction.
 *
 * Address encoding: the mii_bus read/write_c45 callbacks pass
 * ((devad & 0x1f) << 16) | (reg & 0xffff) as the regmap address.  The
 * caller's regmap read/write ops are responsible for translating this to
 * the hardware-specific MMIO layout.
 *
 * Copyright (C) 2024 Qualcomm Technologies, Inc.
 */

#include <linux/device.h>
#include <linux/fwnode.h>
#include <linux/mdio.h>
#include <linux/module.h>
#include <linux/pcs/pcs-xpcs.h>
#include <linux/pcs/pcs-xpcs-regmap.h>
#include <linux/property.h>
#include <linux/regmap.h>

#include "pcs-xpcs.h"

struct xpcs_regmap_bus {
	struct regmap *regmap;
};

static unsigned int xpcs_regmap_addr(int devad, int reg)
{
	return ((devad & 0x1f) << 16) | (reg & 0xffff);
}

static int xpcs_regmap_read_c22(struct mii_bus *bus, int addr, int reg)
{
	struct xpcs_regmap_bus *xrb = bus->priv;
	unsigned int val;
	int ret;

	if (addr != 0)
		return -ENODEV;

	ret = regmap_read(xrb->regmap,
			  xpcs_regmap_addr(MDIO_MMD_VEND2, reg), &val);
	return ret ? ret : (int)(val & 0xffff);
}

static int xpcs_regmap_write_c22(struct mii_bus *bus, int addr, int reg,
				  u16 val)
{
	struct xpcs_regmap_bus *xrb = bus->priv;

	if (addr != 0)
		return -ENODEV;

	return regmap_write(xrb->regmap,
			    xpcs_regmap_addr(MDIO_MMD_VEND2, reg), val);
}

static int xpcs_regmap_read_c45(struct mii_bus *bus, int addr, int devad,
				 int reg)
{
	struct xpcs_regmap_bus *xrb = bus->priv;
	unsigned int val;
	int ret;

	if (addr != 0)
		return -ENODEV;

	ret = regmap_read(xrb->regmap, xpcs_regmap_addr(devad, reg), &val);
	return ret ? ret : (int)(val & 0xffff);
}

static int xpcs_regmap_write_c45(struct mii_bus *bus, int addr, int devad,
				  int reg, u16 val)
{
	struct xpcs_regmap_bus *xrb = bus->priv;

	if (addr != 0)
		return -ENODEV;

	return regmap_write(xrb->regmap, xpcs_regmap_addr(devad, reg), val);
}

/**
 * devm_xpcs_regmap_register() - create a DW XPCS instance backed by a regmap
 * @dev: device to use for resource management and error reporting
 * @cfg: regmap configuration (regmap and indirect-access flag)
 *
 * Allocates a synthetic MDIO bus backed by the caller's regmap, creates an
 * MDIO device on it, and hands it to the DW XPCS core.  The returned pointer
 * is managed by @dev; the caller does not need to call xpcs_destroy().
 *
 * The regmap's read/write ops receive addresses encoded as:
 *   (devad << 16) | reg
 *
 * Return: pointer to dw_xpcs on success, ERR_PTR on failure.
 */
struct dw_xpcs *devm_xpcs_regmap_register(struct device *dev,
					   const struct xpcs_regmap_config *cfg)
{
	struct xpcs_regmap_bus *xrb;
	struct mdio_device *mdiodev;
	struct dw_xpcs *xpcs;
	struct mii_bus *bus;
	int ret;

	xrb = devm_kzalloc(dev, sizeof(*xrb), GFP_KERNEL);
	if (!xrb)
		return ERR_PTR(-ENOMEM);
	xrb->regmap = cfg->regmap;

	bus = devm_mdiobus_alloc_size(dev, 0);
	if (!bus)
		return ERR_PTR(-ENOMEM);

	bus->name	 = "DW XPCS regmap bus";
	bus->read	 = xpcs_regmap_read_c22;
	bus->write	 = xpcs_regmap_write_c22;
	bus->read_c45	 = xpcs_regmap_read_c45;
	bus->write_c45	 = xpcs_regmap_write_c45;
	bus->phy_mask	 = ~0;
	bus->parent	 = dev;
	bus->priv	 = xrb;
	snprintf(bus->id, MII_BUS_ID_SIZE, "xpcs-regmap-%s", dev_name(dev));

	ret = devm_mdiobus_register(dev, bus);
	if (ret)
		return ERR_PTR(ret);

	mdiodev = mdio_device_create(bus, 0);
	if (IS_ERR(mdiodev))
		return ERR_CAST(mdiodev);

	device_set_node(&mdiodev->dev, fwnode_handle_get(dev_fwnode(dev)));
	dev_set_of_node_reused(&mdiodev->dev);

	ret = mdio_device_register(mdiodev);
	if (ret) {
		mdio_device_free(mdiodev);
		return ERR_PTR(ret);
	}

	xpcs = xpcs_create_fwnode(dev_fwnode(dev));

	/*
	 * xpcs_create_fwnode() holds its own mdiodev reference.  Release the
	 * one we took above so mdio_device_put() in xpcs_destroy() is the
	 * sole owner.
	 */
	mdio_device_put(mdiodev);

	if (IS_ERR(xpcs))
		return xpcs;

	return devm_add_action_or_reset(dev, (void (*)(void *))xpcs_destroy,
					xpcs) ? ERR_PTR(-ENOMEM) : xpcs;
}
EXPORT_SYMBOL_GPL(devm_xpcs_regmap_register);
