/*
 * omap-pm-interface.c - OMAP power management interface
 *
 * This code implements the OMAP power management interface to
 * drivers, CPUIdle, CPUFreq, and DSP Bridge.
 *
 * Copyright (C) 2008-2011 Texas Instruments, Inc.
 * Copyright (C) 2008-2009 Nokia Corporation
 * Paul Walmsley
 *
 * Interface developed by (in alphabetical order):
 * Karthik Dasu, Tony Lindgren, Rajendra Nayak, Sakari Poussa, Veeramanikandan
 * Raju, Anand Sawant, Igor Stoppa, Paul Walmsley, Richard Woodruff
 */

#undef DEBUG

#include <linux/init.h>
#include <linux/cpufreq.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/io.h>
/* Interface documentation is in mach/omap-pm.h */
#include "omap_device.h"
#include "omap-pm.h"

#include "omap-pm-helper.h"
#include "prm44xx.h"
#include "soc.h"

bool off_mode_enabled;

static int dummy_context_loss_counter;

int omap_pm_get_dev_context_loss_count(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	int count;

	if (WARN_ON(!dev))
		return -ENODEV;

	if (dev->pm_domain == &omap_device_pm_domain) {
		count = omap_device_get_context_loss_count(pdev);
	} else {
		WARN_ONCE(omap_pm_get_off_mode(),
			"omap_pm: using dummy context loss counter; device %s "
			"should be converted to omap_device",
			  dev_name(dev));

		count = dummy_context_loss_counter;

		if (omap_pm_get_off_mode()) {
			count++;
			/*
			 * Context loss count has to be a non-negative value.
			 * Clear the sign bit to get a value range from 0 to
			 * INT_MAX.
			 */
			count &= INT_MAX;
			dummy_context_loss_counter = count;
		}
	}

	pr_debug("OMAP PM: context loss count for dev %s = %d\n",
		 dev_name(dev), count);

	return count;
}

int omap_pm_set_max_dev_wakeup_lat(struct device *req_dev, struct device *dev,
				   long t)
{
	int ret;
	if (!req_dev || !dev || t < -1) {
		WARN(1, "OMAP PM: %s: invalid parameter(s)", __func__);
		return -EINVAL;
	};

	if (t == -1)
		pr_debug("OMAP PM: remove max device latency constraint: "
			 "dev %s\n", dev_name(dev));
	else
		pr_debug("OMAP PM: add max device latency constraint: "
			 "dev %s, t = %ld usec\n", dev_name(dev), t);

	ret = omap_pm_set_max_dev_wakeup_lat_helper(req_dev, dev, t);

	return ret;
}

/* WARNING: Device drivers need to now use pm_qos directly.
int omap_pm_set_max_sdma_lat(struct pm_qos_request_list **qos_request, long t)
{
	WARN(1, "Deprecated %s: Driver should use pm_qos to add request\n",
	     __func__);

	return -EINVAL;
}*/

int omap_pm_set_min_clk_rate(struct device *dev, struct clk *c, long r)
{
	WARN(1, "Deprecated %s: Driver should use omap_device_scale/opp\n",
	     __func__);

	return -EINVAL;
}

/*
 * DSP Bridge-specific constraints
 * WARNING: Device drivers need to now use opp layer/omap_device_scale directly.
 */
const struct omap_opp *omap_pm_dsp_get_opp_table(void)
{
	WARN(1, "Deprecated %s: Driver should use omap_device_scale/opp\n",
	     __func__);

	return ERR_PTR(-EINVAL);
}

void omap_pm_dsp_set_min_opp(u8 opp_id)
{
	WARN(1, "Deprecated %s: Driver should use omap_device_scale/opp\n",
	     __func__);

	return;
}

int omap_pm_set_min_mpu_freq(struct device *dev, unsigned long f)
{
	WARN(1, "Deprecated %s: Driver should NOT use this function\n",
	     __func__);

	return -EINVAL;

}

EXPORT_SYMBOL(omap_pm_set_min_mpu_freq);

u8 omap_pm_dsp_get_opp(void)
{
	WARN(1, "Deprecated %s: Driver should use omap_device_scale/opp\n",
	     __func__);

	return 0;
}

/*
 * CPUFreq-originated constraint
 *
 * In the future, this should be handled by custom OPP clocktype
 * functions.
 */

struct cpufreq_frequency_table **omap_pm_cpu_get_freq_table(void)
{
	WARN(1, "Deprecated %s: Driver should use omap_device_scale/opp\n",
	     __func__);

	return ERR_PTR(-EINVAL);
}

void omap_pm_cpu_set_freq(unsigned long f)
{
	WARN(1, "Deprecated %s: Driver should use omap_device_scale/opp\n",
	     __func__);

	return;
}

unsigned long omap_pm_cpu_get_freq(void)
{
	WARN(1, "Deprecated %s: Driver should use omap_device_scale/opp\n",
	     __func__);

	return 0;
}

/**
 * omap_pm_enable_off_mode - notify OMAP PM that off-mode is enabled
 *
 * Intended for use only by OMAP PM core code to notify this layer
 * that off mode has been enabled.
 */
void omap_pm_enable_off_mode(void)
{
	off_mode_enabled = true;
}

/**
 * omap_pm_disable_off_mode - notify OMAP PM that off-mode is disabled
 *
 * Intended for use only by OMAP PM core code to notify this layer
 * that off mode has been disabled.
 */
void omap_pm_disable_off_mode(void)
{
	off_mode_enabled = false;
}

bool omap_pm_was_context_lost(struct device *dev)
{
	struct platform_device *pdev;
	struct omap_device *od;
	struct omap_hwmod *oh;

	if (!dev)
		goto save_ctx;

	pdev = container_of(dev, struct platform_device, dev);
	od = container_of(pdev, struct omap_device, pdev);
	oh = od->hwmods[0];

	if (!oh || !cpu_is_omap44xx())
		goto save_ctx;

#if 0
	if (oh->prcm.omap4.context_reg) {
		u32 context_reg_val = 0;

		/*Read what context was lost.*/
		context_reg_val = __raw_readl(oh->prcm.omap4.context_reg);

		/*clear context lost bits after read*/
		__raw_writel(context_reg_val, oh->prcm.omap4.context_reg);

		/* ABE special case, only report ctx lost when we loose
		 * mem, otherwise, constant firmware reload causes problems.
		 */
		if (oh->prcm.omap4.context_reg == OMAP4430_RM_ABE_AESS_CONTEXT)
			context_reg_val &= (1 << 8);

		return (context_reg_val != 0);
	}
#endif

save_ctx:
	/* by default return true so that driver will restore context*/
	return true;
}

/* Should be called before clk framework init */
int __init omap_pm_if_early_init(void)
{
	return 0;
}

/* Must be called after clock framework is initialized */
int __init omap_pm_if_init(void)
{
	return omap_pm_if_init_helper();
}

void omap_pm_if_exit(void)
{
	/* Deallocate CPUFreq frequency table here */
}
