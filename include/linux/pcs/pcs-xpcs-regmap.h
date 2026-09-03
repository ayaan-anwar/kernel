/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Regmap registration API for Synopsys DesignWare XPCS platform drivers.
 *
 * Platform drivers that access XPCS registers via MMIO (rather than MDIO)
 * may supply a regmap whose read/write ops translate MDIO C45 addresses to
 * hardware-specific MMIO offsets.  devm_xpcs_regmap_register() bridges the
 * regmap into the DW XPCS core, which uses the standard pcs-xpcs MDIO
 * abstraction layer internally.
 *
 * Address encoding passed to the regmap: ((devad & 0x1f) << 16) | (reg & 0xffff).
 *
 * Copyright (C) 2024 Qualcomm Technologies, Inc.
 */

#ifndef __LINUX_PCS_XPCS_REGMAP_H
#define __LINUX_PCS_XPCS_REGMAP_H

#include <linux/types.h>

struct device;
struct regmap;
struct dw_xpcs;

/**
 * struct xpcs_regmap_config - configuration for a regmap-backed XPCS
 * @regmap: regmap implementing the XPCS register space.  The read/write ops
 *          receive addresses encoded as ((devad & 0x1f) << 16) | reg.
 * @reg_indir: set true if the hardware requires indirect (viewport) access.
 *             Currently unused; reserved for future use.
 */
struct xpcs_regmap_config {
	struct regmap *regmap;
	bool reg_indir;
};

struct dw_xpcs *devm_xpcs_regmap_register(struct device *dev,
					   const struct xpcs_regmap_config *cfg);

#endif /* __LINUX_PCS_XPCS_REGMAP_H */
