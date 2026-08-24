/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _LINUX_PCS_XPCS_QCOM_DIRECT_H
#define _LINUX_PCS_XPCS_QCOM_DIRECT_H

#include <linux/phylink.h>

struct platform_device;

struct phylink_pcs *qcom_xpcs_direct_get_pcs(struct platform_device *pdev);

#endif /* _LINUX_PCS_XPCS_QCOM_DIRECT_H */
