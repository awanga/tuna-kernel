/*
 * omap-pm-interface.c - OMAP power management interface
 *
 * This code implements the OMAP power management interface to
 * drivers, CPUIdle, CPUFreq, and DSP Bridge.  It is strictly for
 * debug/demonstration use, as it does nothing but printk() whenever a
 * function is called (when DEBUG is defined, below)
 *
 * Copyright (C) 2008-2009 Texas Instruments, Inc.
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
#include <linux/slab.h>
#include <linux/list.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>

#include <asm/io.h>

#include "omap_device.h"
#include "omap-pm.h"
#include "common.h"
#include "iomap.h"
#include "clockdomain.h"
#include "powerdomain.h"
#include "prminst44xx.h"
#include "prm44xx.h"
#include "soc.h"

static char *mpu_clk_name;

struct omap_opp *dsp_opps;
struct omap_opp *mpu_opps;
struct omap_opp *l3_opps;

static DEFINE_MUTEX(bus_tput_mutex);
static DEFINE_MUTEX(mpu_tput_mutex);
static DEFINE_MUTEX(mpu_lat_mutex);

/* Used to model a Interconnect Throughput */
static struct interconnect_tput {
	/* Total no of users at any point of interconnect */
	u8 no_of_users;
	/* List of all the current users for interconnect */
	struct list_head users_list;
	struct list_head node;
	/* Protect interconnect throughput */
	struct mutex throughput_mutex;
	/* Target level for interconnect throughput */
	unsigned long target_level;

} *bus_tput;

/* Used to represent a user of a interconnect throughput */
struct users {
	/* Device pointer used to uniquely identify the user */
	struct device *dev;
	struct list_head node;
	/* Current level as requested for interconnect throughput by the user */
	u32 level;
};

/* Private/Internal Functions */

/**
 * user_lookup - look up a user by its device pointer, return a pointer
 * @dev: The device to be looked up
 *
 * Looks for a interconnect user by its device pointer. Returns a
 * pointer to
 * the struct users if found, else returns NULL.
 */
static struct users *user_lookup(struct device *dev)
{
	struct users *usr, *tmp_usr;

	usr = NULL;
	list_for_each_entry(tmp_usr, &bus_tput->users_list, node) {
		if (tmp_usr->dev == dev) {
			usr = tmp_usr;
			break;
		}
	}

	return usr;
}

/**
 * get_user - gets a new users_list struct dynamically
 *
 * This function allocates dynamcially the user node
 * Returns a pointer to users struct on success. On dynamic allocation
 * failure
 * returns a ERR_PTR(-ENOMEM).
 */
static struct users *get_user(void)
{
	struct users *user;

	user = kmalloc(sizeof(struct users), GFP_KERNEL);
	if (!user) {
		pr_err("%s FATAL ERROR: kmalloc failed\n", __func__);
		return ERR_PTR(-ENOMEM);
	}
	return user;
}

#ifdef CONFIG_PM_DEBUG
static int pm_dbg_show_tput(struct seq_file *s, void *unused)
{
	struct users *usr;

	mutex_lock(&bus_tput->throughput_mutex);
	list_for_each_entry(usr, &bus_tput->users_list, node)
		seq_printf(s, "%s:	%u\n", dev_name(usr->dev),
				usr->level);
	mutex_unlock(&bus_tput->throughput_mutex);

	return 0;
}

static int pm_dbg_open(struct inode *inode, struct file *file)
{
	return single_open(file, pm_dbg_show_tput,
			&inode->i_private);
}

static const struct file_operations tputdebugfs_fops = {
	.open           = pm_dbg_open,
	.read           = seq_read,
	.llseek         = seq_lseek,
	.release        = single_release,
};
#endif

/**
 * omap_bus_tput_init - Initializes the interconnect throughput
 * userlist
 * Allocates memory for global throughput variable dynamically.
 * Intializes Userlist, no. of users and throughput target level.
 * Returns 0 on sucess, else returns EINVAL if memory
 * allocation fails.
 */
static int __init omap_bus_tput_init(void)
{
	bus_tput = kmalloc(sizeof(struct interconnect_tput), GFP_KERNEL);
	if (!bus_tput) {
		pr_err("%s FATAL ERROR: kmalloc failed\n", __func__);
		return -EINVAL;
	}
	INIT_LIST_HEAD(&bus_tput->users_list);
	mutex_init(&bus_tput->throughput_mutex);
	bus_tput->no_of_users = 0;
	bus_tput->target_level = 0;

#ifdef CONFIG_PM_DEBUG
	(void) debugfs_create_file("tput", S_IRUGO,
		NULL, (void *)bus_tput, &tputdebugfs_fops);
#endif

	return 0;
}

/**
 * add_req_tput  - Request for a required level by a device
 * @dev: Uniquely identifes the caller
 * @level: The requested level for the interconnect bandwidth in KiB/s
 *
 * This function recomputes the target level of the interconnect
 * bandwidth
 * based on the level requested by all the users.
 * Multiple calls to this function by the same device will
 * replace the previous level requested
 * Returns the updated level of interconnect throughput.
 * In case of Invalid dev or user pointer, it returns 0.
 */
static unsigned long add_req_tput(struct device *dev, unsigned long level)
{
	int ret;
	struct users *user;

	if (!dev) {
		pr_err("Invalid dev pointer\n");
		ret = 0;
	}
	mutex_lock(&bus_tput->throughput_mutex);
	user = user_lookup(dev);
	if (user == NULL) {
		user = get_user();
		if (IS_ERR(user)) {
			pr_err("Couldn't get user from the list to"
			       "add new throughput constraint");
			ret = 0;
			goto unlock;
		}
		bus_tput->target_level += level;
		bus_tput->no_of_users++;
		user->dev = dev;
		list_add(&user->node, &bus_tput->users_list);
		user->level = level;
	} else {
		bus_tput->target_level -= user->level;
		bus_tput->target_level += level;
		user->level = level;
	}
	ret = bus_tput->target_level;
unlock:
	mutex_unlock(&bus_tput->throughput_mutex);
	return ret;
}

/**
 * remove_req_tput - Release a previously requested level of
 * a throughput level for interconnect
 * @dev: Device pointer to dev
 *
 * This function recomputes the target level of the interconnect
 * throughput after removing
 * the level requested by the user.
 * Returns 0, if the dev structure is invalid
 * else returns modified interconnect throughput rate.
 */
static unsigned long remove_req_tput(struct device *dev)
{
	struct users *user;
	int found = 0;
	int ret;

	mutex_lock(&bus_tput->throughput_mutex);
	list_for_each_entry(user, &bus_tput->users_list, node) {
		if (user->dev == dev) {
			found = 1;
			break;
		}
	}
	if (!found) {
		/* No such user exists */
		pr_err("Invalid Device Structure\n");
		ret = 0;
		goto unlock;
	}
	bus_tput->target_level -= user->level;
	bus_tput->no_of_users--;
	list_del(&user->node);
	kfree(user);
	ret = bus_tput->target_level;
unlock:
	mutex_unlock(&bus_tput->throughput_mutex);
	return ret;
}

/*
 * Device-driver-originated constraints (via board-*.c files)
 */

int omap_pm_set_max_mpu_wakeup_lat(struct device *dev, long t)
{
	struct platform_device *pdev;
	struct omap_device *odev;
	struct powerdomain *pwrdm_dev;
	int ret = 0;

	if (!dev || t < -1) {
		WARN(1, "OMAP PM: %s: invalid parameter(s)", __func__);
		return -EINVAL;
	}

	/* Look for the devices Power Domain */
	pdev = container_of(dev, struct platform_device, dev);

	/* Try to catch non platform devices. */
	if (pdev->name == NULL) {
		pr_err("OMAP-PM: Error: platform device not valid\n");
		return -EINVAL;
	}

	odev = to_omap_device(pdev);
	if (odev) {
		WARN(odev->hwmods_cnt == 0, "OMAP-PM: Error: could not find powerdomain for %s\n",
			pdev->name);
		pwrdm_dev = omap_hwmod_get_pwrdm(odev->hwmods[0]);
	} else {
		pr_err("OMAP-PM: Error: Could not find omap_device for %s\n",
		       pdev->name);
		return -EINVAL;
	}

	/* Catch devices with undefined powerdomains. */
	if (!pwrdm_dev) {
		pr_err("OMAP-PM: Error: could not find parent pwrdm for %s\n",
		       pdev->name);
		return -EINVAL;
	}

	/*
	 * For current Linux, this needs to map the MPU to a
	 * powerdomain, then go through the list of current max lat
	 * constraints on the MPU and find the smallest.  If
	 * the latency constraint has changed, the code should
	 * recompute the state to enter for the next powerdomain
	 * state.
	 *
	 * TI CDP code can call constraint_set here.
	 */
	 if (t == -1) {
		pr_debug("OMAP PM: remove max MPU wakeup latency constraint: dev %s\n",
			 dev_name(dev));
		ret = pwrdm_wakeuplat_release_constraint(pwrdm_dev, dev);
	} else {
		pr_debug("OMAP PM: add max MPU wakeup latency constraint: dev %s, t = %ld usec\n",
			 dev_name(dev), t);
		ret = pwrdm_wakeuplat_set_constraint(pwrdm_dev, dev, t);
	}

	return ret;
}

int omap_pm_set_min_bus_tput(struct device *dev, u8 agent_id, unsigned long r)
{
	struct platform_device *pdev;
	struct omap_device *odev;
	/*struct device *l3_dev;*/
	unsigned long target_level = 0;
	int ret = 0;

	if (!dev || (agent_id != OCP_INITIATOR_AGENT &&
	    agent_id != OCP_TARGET_AGENT)) {
		WARN(1, "OMAP PM: %s: invalid parameter(s)", __func__);
		return -EINVAL;
	}

	/* Look for the devices Power Domain */
	pdev = container_of(dev, struct platform_device, dev);

	/* Try to catch non platform devices. */
	if (pdev->name == NULL) {
		pr_err("OMAP-PM: Error: platform device not valid\n");
		return -EINVAL;
	}

	odev = to_omap_device(pdev);
	if (!odev) {
		pr_err("OMAP-PM: Error: Could not find omap_device for %s\n",
		       pdev->name);
		return -EINVAL;
	}

	mutex_lock(&bus_tput_mutex);

	/*
	l3_dev = omap2_get_l3_device();
	if (!l3_dev) {
		pr_err("Unable to get l3 device pointer");
		ret = -EINVAL;
		goto unlock;
	}*/

	/*
	 * This code should model the interconnect and compute the
	 * required clock frequency, convert that to a VDD2 OPP ID, then
	 * set the VDD2 OPP appropriately.
	 *
	 * TI CDP code can call constraint_set here on the VDD2 OPP.
	 */
	 if (r == 0) {
		pr_debug("OMAP PM: remove min bus tput constraint: dev %s for agent_id %d\n",
			 dev_name(dev), agent_id);
		target_level = remove_req_tput(dev);
	} else {
		pr_debug("OMAP PM: add min bus tput constraint: dev %s for agent_id %d: rate %ld KiB\n",
			 dev_name(dev), agent_id, r);
		target_level = add_req_tput(dev, r);
	}

	/* Convert the throughput(in KiB/s) into Hz. */
	target_level = (target_level * 1000) / 4;

#warning "Need to implement interconnect clock scaling/limiting"
	/*ret = omap_device_scale(&dummy_l3_dev, l3_dev, target_level);
	if (ret)
		pr_err("Failed: change interconnect bandwidth to %ld\n",
		     target_level);
unlock:*/
	mutex_unlock(&bus_tput_mutex);
	return ret;

}

int omap_pm_set_max_dev_wakeup_lat(struct device *req_dev, struct device *dev,
				   long t)
{
	struct omap_device *odev;
	struct powerdomain *pwrdm_dev;
	struct platform_device *pdev;
	int ret = 0;

	if (!req_dev || !dev || t < -1) {
		WARN(1, "OMAP PM: %s: invalid parameter(s)", __func__);
		return -EINVAL;
	}

	/* Look for the devices Power Domain */
	pdev = container_of(dev, struct platform_device, dev);

	/* Try to catch non platform devices. */
	if (pdev->name == NULL) {
		pr_err("OMAP-PM: Error: platform device not valid\n");
		return -EINVAL;
	}

	odev = to_omap_device(pdev);
	if (odev) {
		WARN(odev->hwmods_cnt == 0, "OMAP-PM: Error: could not find powerdomain for %s\n",
			pdev->name);
		pwrdm_dev = omap_hwmod_get_pwrdm(odev->hwmods[0]);
	} else {
		pr_err("OMAP-PM: Error: Could not find omap_device for %s\n",
		       pdev->name);
		return -EINVAL;
	}

	/* Catch devices with undefined powerdomains. */
	if (!pwrdm_dev) {
		pr_err("OMAP-PM: Error: could not find parent pwrdm for %s\n",
		       pdev->name);
		return -EINVAL;
	}

	/*
	 * For current Linux, this needs to map the device to a
	 * powerdomain, then go through the list of current max lat
	 * constraints on that powerdomain and find the smallest.  If
	 * the latency constraint has changed, the code should
	 * recompute the state to enter for the next powerdomain
	 * state.  Conceivably, this code should also determine
	 * whether to actually disable the device clocks or not,
	 * depending on how long it takes to re-enable the clocks.
	 *
	 * TI CDP code can call constraint_set here.
	 */
	if (t == -1) {
		pr_debug("OMAP PM: remove max device latency constraint: dev %s\n",
			 dev_name(dev));
		ret = pwrdm_wakeuplat_release_constraint(pwrdm_dev, req_dev);
	} else {
		pr_debug("OMAP PM: add max device latency constraint: dev %s, t = %ld usec\n",
			 dev_name(dev), t);
		ret = pwrdm_wakeuplat_set_constraint(pwrdm_dev, req_dev, t);
	}

	return ret;
}

int omap_pm_set_max_sdma_lat(struct device *dev, long t)
{
	if (!dev || t < -1) {
		WARN(1, "OMAP PM: %s: invalid parameter(s)", __func__);
		return -EINVAL;
	}

	if (t == -1)
		pr_debug("OMAP PM: remove max DMA latency constraint: dev %s\n",
			 dev_name(dev));
	else
		pr_debug("OMAP PM: add max DMA latency constraint: dev %s, t = %ld usec\n",
			 dev_name(dev), t);

	/*
	 * For current Linux PM QOS params, this code should scan the
	 * list of maximum CPU and DMA latencies and select the
	 * smallest, then set cpu_dma_latency pm_qos_param
	 * accordingly.
	 *
	 * For future Linux PM QOS params, with separate CPU and DMA
	 * latency params, this code should just set the dma_latency param.
	 *
	 * TI CDP code can call constraint_set here.
	 */

	return 0;
}

int omap_pm_set_min_clk_rate(struct device *dev, struct clk *c, long r)
{
	if (!dev || !c || r < 0) {
		WARN(1, "OMAP PM: %s: invalid parameter(s)", __func__);
		return -EINVAL;
	}

	if (r == 0)
		pr_debug("OMAP PM: remove min clk rate constraint: dev %s\n",
			 dev_name(dev));
	else
		pr_debug("OMAP PM: add min clk rate constraint: dev %s, rate = %ld Hz\n",
			 dev_name(dev), r);

	/*
	 * Code in a real implementation should keep track of these
	 * constraints on the clock, and determine the highest minimum
	 * clock rate.  It should iterate over each OPP and determine
	 * whether the OPP will result in a clock rate that would
	 * satisfy this constraint (and any other PM constraint in effect
	 * at that time).  Once it finds the lowest-voltage OPP that
	 * meets those conditions, it should switch to it, or return
	 * an error if the code is not capable of doing so.
	 */

	return 0;
}

/*
 * DSP Bridge-specific constraints
 */

const struct omap_opp *omap_pm_dsp_get_opp_table(void)
{
	pr_debug("OMAP PM: DSP request for OPP table\n");

	/*
	 * Return DSP frequency table here:  The final item in the
	 * array should have .rate = .opp_id = 0.
	 */

	return NULL;
}

void omap_pm_dsp_set_min_opp(u8 opp_id)
{
	if (opp_id == 0) {
		WARN_ON(1);
		return;
	}

	pr_debug("OMAP PM: DSP requests minimum VDD1 OPP to be %d\n", opp_id);

	/*
	 *
	 * For l-o dev tree, our VDD1 clk is keyed on OPP ID, so we
	 * can just test to see which is higher, the CPU's desired OPP
	 * ID or the DSP's desired OPP ID, and use whichever is
	 * highest.
	 *
	 * In CDP12.14+, the VDD1 OPP custom clock that controls the DSP
	 * rate is keyed on MPU speed, not the OPP ID.  So we need to
	 * map the OPP ID to the MPU speed for use with clk_set_rate()
	 * if it is higher than the current OPP clock rate.
	 *
	 */
}


u8 omap_pm_dsp_get_opp(void)
{
	pr_debug("OMAP PM: DSP requests current DSP OPP ID\n");

	/*
	 * For l-o dev tree, call clk_get_rate() on VDD1 OPP clock
	 *
	 * CDP12.14+:
	 * Call clk_get_rate() on the OPP custom clock, map that to an
	 * OPP ID using the tables defined in board-*.c/chip-*.c files.
	 */

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
	pr_debug("OMAP PM: CPUFreq request for frequency table\n");

	/*
	 * Return CPUFreq frequency table here: loop over
	 * all VDD1 clkrates, pull out the mpu_ck frequencies, build
	 * table
	 */

	return NULL;
}

void omap_pm_cpu_set_freq(unsigned long f)
{
	if (f == 0) {
		WARN_ON(1);
		return;
	}

	pr_debug("OMAP PM: CPUFreq requests CPU frequency to be set to %lu\n",
		 f);

	/*
	 * For l-o dev tree, determine whether MPU freq or DSP OPP id
	 * freq is higher.  Find the OPP ID corresponding to the
	 * higher frequency.  Call clk_round_rate() and clk_set_rate()
	 * on the OPP custom clock.
	 *
	 * CDP should just be able to set the VDD1 OPP clock rate here.
	 */
}

unsigned long omap_pm_cpu_get_freq(void)
{
	struct clk *mpu_ck;

	pr_debug("OMAP PM: CPUFreq requests current CPU frequency\n");

	/*
	 * Call clk_get_rate() on the mpu_ck.
	 */
	mpu_ck = clk_get(NULL, mpu_clk_name);
	if (WARN_ONCE(IS_ERR(mpu_ck), "Failed to get mpu_ck\n"))
		return 0;

	return clk_get_rate(mpu_ck);
}

/*
 * Device context loss tracking
 */

int omap_pm_get_dev_context_loss_count(struct device *dev)
{
	struct platform_device *pdev;
	struct omap_device *od;
	struct omap_hwmod *oh;
	int count = 0;

	if (WARN_ON(!dev))
		return -ENODEV;

	pdev = container_of(dev, struct platform_device, dev);
	od = container_of(pdev, struct omap_device, pdev);
	oh = od->hwmods[0];

	if (!oh || !cpu_is_omap44xx())
		goto save_ctx;

	if (oh->prcm.omap4.context_offs) {
		u32 context_reg_val = 0;

		/*Read what context was lost.*/
		context_reg_val = omap4_prminst_read_inst_reg(
			oh->clkdm->pwrdm.ptr->prcm_partition,
			oh->clkdm->pwrdm.ptr->prcm_offs,
			oh->prcm.omap4.context_offs);

		/*clear context lost bits after read*/
		omap4_prminst_write_inst_reg(context_reg_val,
			oh->clkdm->pwrdm.ptr->prcm_partition,
			oh->clkdm->pwrdm.ptr->prcm_offs,
			oh->prcm.omap4.context_offs);

		/* ABE special case, only report ctx lost when we loose
		 * mem, otherwise, constant firmware reload causes problems.
		 */
		/* OMAP4430_RM_ABE_AESS_CONTEXT */
		if ( !strcmp(oh->name, "aess") || !strcmp(oh->name, "l4_abe") )
			context_reg_val &= (1 << 8);

		count = context_reg_val;
	}

save_ctx:
	/* by default return true so that driver will restore context*/
	return count;
}

/* Should be called before clk framework init */
int __init omap_pm_if_early_init(void)
{
	return 0;
}

/* Must be called after clock framework is initialized */
int __init omap_pm_if_init(void)
{
	int ret;

	if (cpu_is_omap24xx())
		mpu_clk_name = "virt_prcm_set";
	else if (cpu_is_omap34xx())
		mpu_clk_name = "dpll1_ck";
	else if (cpu_is_omap443x())
		mpu_clk_name = "dpll_mpu_ck";
	else if (cpu_is_omap446x())
		mpu_clk_name = "virt_dpll_mpu_ck";
	else
		mpu_clk_name = "";

	ret = omap_bus_tput_init();
	if (ret)
		pr_err("Failed: init of interconnect bandwidth users list\n");
	return ret;
}

void omap_pm_if_exit(void)
{
	/* Deallocate CPUFreq frequency table here */
}

