/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2024 Broadcom
 */

#ifndef __LINUX_PCS_XPCS_REGMAP_H
#define __LINUX_PCS_XPCS_REGMAP_H

#include <linux/regmap.h>
#include <linux/pcs/pcs-xpcs.h>

/**
 * struct xpcs_regmap_config - Configuration for regmap-based XPCS
 * @regmap:   Regmap for the XPCS register space
 * @reg_indir: Use indirect (viewport) register access
 */
struct xpcs_regmap_config {
	struct regmap *regmap;
	bool reg_indir;
};

struct dw_xpcs *devm_xpcs_regmap_register(struct device *dev,
					   const struct xpcs_regmap_config *config);

#endif /* __LINUX_PCS_XPCS_REGMAP_H */
