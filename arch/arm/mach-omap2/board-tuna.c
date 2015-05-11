/*
 * Board support file for Samsung Tuna Board.
 *
 * Copyright (C) 2011 Google, Inc.
 * Copyright (C) 2010 Texas Instruments
 *
 * Based on mach-omap2/board-omap4panda.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/ion.h>
#include <linux/omap_ion.h>
#include <linux/leds.h>
#include <linux/gpio.h>
#include <linux/usb/otg.h>
#include <linux/i2c/twl.h>
#include <linux/mfd/twl6040.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>
#include <linux/ti_wilink_st.h>
#include <linux/reboot.h>
#include <linux/memblock.h>
#include <linux/sysfs.h>
#include <linux/uaccess.h>
#include <linux/proc_fs.h>
#include <linux/spi/spi.h>
#include <linux/spi/flash.h>
#include <linux/usb/musb.h>
#include <linux/usb/phy.h>
#include <linux/usb/nop-usb-xceiv.h>
#include <linux/wl12xx.h>
#include <linux/irqchip/arm-gic.h>
#include <linux/platform_data/omap-abe-twl6040.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/memory.h>
#include <asm/system_misc.h>

#include "omap4-sar-layout.h"
#include "common.h"
#include "soc.h"
#include "mmc.h"
#include "hsmmc.h"
#include "control.h"
#include "mux.h"
#include "common-board-devices.h"
#include "dss-common.h"
#include "board-tuna.h"
#include "resetreason.h"

#define PHYS_ADDR_SMC_SIZE			(SZ_1M * 3)
#define PHYS_ADDR_DUCATI_SIZE			(SZ_1M * 105)
#define OMAP_TUNA_ION_HEAP_SECURE_INPUT_SIZE	(SZ_1M * 90)
#define OMAP_TUNA_ION_HEAP_TILER_SIZE		(SZ_1M * 81)
#define OMAP_TUNA_ION_HEAP_NONSECURE_TILER_SIZE	(SZ_1M * 15)

#define PHYS_ADDR_SMC_MEM	(0x80000000 + SZ_1G - PHYS_ADDR_SMC_SIZE)
#define PHYS_ADDR_DUCATI_MEM	(PHYS_ADDR_SMC_MEM - \
				 PHYS_ADDR_DUCATI_SIZE - \
				 OMAP_TUNA_ION_HEAP_SECURE_INPUT_SIZE)

#define TUNA_RAMCONSOLE_START	(PHYS_OFFSET + SZ_512M)
#define TUNA_RAMCONSOLE_SIZE	SZ_2M

/* For LTE(CMC221) */
#define OMAP_GPIO_LTE_ACTIVE	47
#define OMAP_GPIO_CMC2AP_INT1	61

#define GPIO_AUD_PWRON		127
#define GPIO_AUD_PWRON_TORO_V1	20

/* GPS GPIO Setting */
#define GPIO_AP_AGPS_TSYNC	18
#define GPIO_GPS_nRST	136
#define GPIO_GPS_PWR_EN	137
#define GPIO_GPS_UART_SEL	164

#define GPIO_MHL_SCL_18V	99
#define GPIO_MHL_SDA_18V	98

#define REBOOT_FLAG_RECOVERY	0x52564352
#define REBOOT_FLAG_FASTBOOT	0x54534146
#define REBOOT_FLAG_NORMAL	0x4D524F4E
#define REBOOT_FLAG_POWER_OFF	0x46464F50

#define UART_NUM_FOR_GPS	0


/*
 * Tuna HW ID Routines
 */

static int tuna_hw_rev;

static struct gpio tuna_hw_rev_gpios[] = {
	{76, GPIOF_IN, "hw_rev0"},
	{75, GPIOF_IN, "hw_rev1"},
	{74, GPIOF_IN, "hw_rev2"},
	{73, GPIOF_IN, "hw_rev3"},
	{170, GPIOF_IN, "hw_rev4"},
};

static const char const *omap4_tuna_hw_name_maguro[] = {
	[0x00] = "Toro Lunchbox #1",
	[0x01] = "Maguro 1st Sample",
	[0x02] = "Maguro 2nd Sample",
	[0x03] = "Maguro 4th Sample",
	[0x05] = "Maguro 5th Sample",
	[0x07] = "Maguro 6th Sample",
	[0x08] = "Maguro 7th Sample",
	[0x09] = "Maguro 8th Sample",
};

static const char const *omap4_tuna_hw_name_toro[] = {
	[0x00] = "Toro Lunchbox #2",
	[0x01] = "Toro 1st Sample",
	[0x02] = "Toro 2nd Sample",
	[0x03] = "Toro 4th Sample",
	[0x05] = "Toro 5th Sample",
	[0x06] = "Toro 8th Sample",
	[0x08] = "Toro 8th Sample",
	[0x09] = "Toro 8-1th Sample",
};

int omap4_tuna_get_revision(void)
{
	return tuna_hw_rev & TUNA_REV_MASK;
}

int omap4_tuna_get_type(void)
{
	return tuna_hw_rev & TUNA_TYPE_MASK;
}


static const char *omap4_tuna_hw_rev_name(void) {
	const char *ret;
	const char **names;
	int num;
	int rev;

	if (omap4_tuna_get_type() == TUNA_TYPE_MAGURO) {
		names = omap4_tuna_hw_name_maguro;
		num = ARRAY_SIZE(omap4_tuna_hw_name_maguro);
		ret = "Maguro unknown";
	} else {
		names = omap4_tuna_hw_name_toro;
		num = ARRAY_SIZE(omap4_tuna_hw_name_toro);
		ret = "Toro unknown";
	}

	rev = omap4_tuna_get_revision();
	if (rev >= num || !names[rev])
		return ret;

	return names[rev];
}


/*
 * BT/Wifi/GPS
 */

/* wl127x BT, FM, GPS connectivity chip */
static struct ti_st_plat_data wilink_platform_data = {
	.nshutdown_gpio	= 46,
	.dev_name	= "/dev/ttyO1",
	.flow_cntrl	= 1,
	.baud_rate	= 3000000,
	.chip_enable	= NULL,
	.suspend	= NULL,
	.resume		= NULL,
};

static struct platform_device wl1271_device = {
	.name	= "kim",
	.id	= -1,
	.dev	= {
		.platform_data	= &wilink_platform_data,
	},
};


static struct platform_device btwilink_device = {
	.name	= "btwilink",
	.id	= -1,
};


/*
 * USB
 */

/* PHY device on HS USB Port 1 i.e. nop_usb_xceiv.1 */
static struct nop_usb_xceiv_platform_data hsusb1_phy_data = {
	/* FREF_CLK3 provides the 19.2 MHz reference clock to the PHY */
	.clk_rate = 19200000,
};

static struct usbhs_phy_data phy_data[] __initdata = {
	{
		.port = 1,
		//.reset_gpio = GPIO_HUB_NRESET,
		//.vcc_gpio = GPIO_HUB_POWER,
		.vcc_polarity = 1,
		.platform_data = &hsusb1_phy_data,
	},
};

static struct usbhs_omap_platform_data usbhs_bdata __initdata = {
	.port_mode[0] = OMAP_EHCI_PORT_MODE_PHY,
};

void __init omap4_ehci_init(void)
{
	int ret;

	/* FREF_CLK3 provides the 19.2 MHz reference clock to the PHY */
	ret = clk_add_alias("main_clk", "nop_usb_xceiv.1", "auxclk3_ck", NULL);
	if (ret)
		pr_err("Failed to add main_clk alias to auxclk3_ck\n");

	usbhs_init_phys(phy_data, ARRAY_SIZE(phy_data));
	usbhs_init(&usbhs_bdata);
}

static struct omap_musb_board_data musb_board_data = {
        .interface_type         = MUSB_INTERFACE_UTMI,
#ifdef CONFIG_USB_MUSB_OTG
        .mode                   = MUSB_OTG,
#else
        .mode                   = MUSB_PERIPHERAL,
#endif
        .power                  = 500,
};


/*
 * MMC
 */
#define HSMMC2_MUX	(OMAP_MUX_MODE1 | OMAP_PIN_INPUT_PULLUP)
#define HSMMC1_MUX	OMAP_PIN_INPUT_PULLUP

static struct omap2_hsmmc_info mmc[] = {
	{
		.mmc		= 1,
		.caps		= MMC_CAP_4_BIT_DATA | MMC_CAP_8_BIT_DATA,
		.ocr_mask	= MMC_VDD_165_195,
		.gpio_wp	= -EINVAL,
		.gpio_cd	= -EINVAL,
	},
	{
		.name		= "wl1271",
		.mmc		= 5,
		.caps		= MMC_CAP_4_BIT_DATA | MMC_CAP_POWER_OFF_CARD,
		.gpio_wp	= -EINVAL,
		.gpio_cd	= -EINVAL,
		.ocr_mask	= MMC_VDD_165_195 | MMC_VDD_20_21,
		.nonremovable	= true,
	},
	/*{
		.name		= "omap_wlan",
		.mmc		= 5,
		.caps		= MMC_CAP_4_BIT_DATA,
		.gpio_wp	= -EINVAL,
		.gpio_cd	= -EINVAL,
		.ocr_mask	= MMC_VDD_165_195 | MMC_VDD_20_21,
		.nonremovable	= false,
	},*/
	{}	/* Terminator */
};

static struct regulator_consumer_supply tuna_vmmc_supply[] = {
	{
		.supply = "vmmc",
		.dev_name = "omap_hsmmc.0",
	},
	{
		.supply = "vmmc",
		.dev_name = "omap_hsmmc.1",
	},
};


/*
 * WLAN
 */

#if 0
static struct regulator_consumer_supply omap4_panda_vmmc5_supply[] = {
	REGULATOR_SUPPLY("vmmc", "omap_hsmmc.4"),
};

static struct regulator_init_data panda_vmmc5 = {
	.constraints = {
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies = ARRAY_SIZE(omap4_panda_vmmc5_supply),
	.consumer_supplies = omap4_panda_vmmc5_supply,
};

static struct fixed_voltage_config panda_vwlan = {
	.supply_name = "vwl1271",
	.microvolts = 1800000, /* 1.8V */
	.gpio = GPIO_WIFI_PMENA,
	.startup_delay = 70000, /* 70msec */
	.enable_high = 1,
	.enabled_at_boot = 0,
	.init_data = &panda_vmmc5,
};

static struct platform_device omap_vwlan_device = {
	.name		= "reg-fixed-voltage",
	.id		= 1,
	.dev = {
		.platform_data = &panda_vwlan,
	},
};

static struct wl12xx_platform_data omap_panda_wlan_data  __initdata = {
	.board_ref_clock = WL12XX_REFCLOCK_38, /* 38.4 MHz */
};
#endif /* 0 */


/*
 * Audio
 */

static struct omap_abe_twl6040_data tuna_abe_audio_data = {
	/* Audio out */
	.has_hs		= ABE_TWL6040_LEFT | ABE_TWL6040_RIGHT,
	/* HandsFree through expansion connector */
	.has_hf		= ABE_TWL6040_LEFT | ABE_TWL6040_RIGHT,
	/* PandaBoard: FM TX, PandaBoardES: can be connected to audio out */
	.has_aux	= ABE_TWL6040_LEFT | ABE_TWL6040_RIGHT,
	/* PandaBoard: FM RX, PandaBoardES: audio in */
	.has_afm	= ABE_TWL6040_LEFT | ABE_TWL6040_RIGHT,
	/* No jack detection. */
	.jack_detection	= 0,
	/* MCLK input is 38.4MHz */
	.mclk_freq	= 38400000,

};

static struct platform_device tuna_abe_audio = {
	.name		= "omap-abe-twl6040",
	.id		= -1,
	.dev = {
		.platform_data = &tuna_abe_audio_data,
	},
};

static struct platform_device tuna_hdmi_audio_codec = {
	.name	= "hdmi-audio-codec",
	.id	= -1,
};

static struct twl4030_audio_data twl6040_data = {
	//.codec		= &twl6040_codec,
	.audpwron_gpio	= 127,
};

static struct twl4030_madc_platform_data twl6030_madc = {
	.irq_line = -1,
};

static struct platform_device twl6030_madc_device = {
	.name   = "twl6030_madc",
	.id = -1,
	.dev	= {
		.platform_data	= &twl6030_madc,
	},
};

static void tuna_audio_init(void)
{
	unsigned int aud_pwron;

	/* twl6040 naudint */
	omap_mux_init_signal("sys_nirq2.sys_nirq2", \
		OMAP_PIN_INPUT_PULLUP);

	/* aud_pwron */
	if (omap4_tuna_get_type() == TUNA_TYPE_TORO &&
	    omap4_tuna_get_revision() >= 1)
		aud_pwron = GPIO_AUD_PWRON_TORO_V1;
	else
		aud_pwron = GPIO_AUD_PWRON;
	omap_mux_init_gpio(aud_pwron, OMAP_PIN_OUTPUT);
	twl6040_data.audpwron_gpio = aud_pwron;

	omap_mux_init_signal("gpmc_a24.gpio_48", OMAP_PIN_OUTPUT | OMAP_MUX_MODE3);
	omap_mux_init_signal("kpd_col3.gpio_171", OMAP_PIN_OUTPUT | OMAP_MUX_MODE3);
}


/*
 * PMIC
 */

static struct regulator_init_data tuna_vaux1 = {
	.constraints = {
		.min_uV			= 3000000,
		.max_uV			= 3000000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 =	REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
		.always_on		= true,
	},
};

static struct regulator_init_data tuna_vaux2 = {
	.constraints = {
		.min_uV			= 1200000,
		.max_uV			= 2800000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
		.state_mem = {
			.disabled	= true,
		},
		.initial_state		= PM_SUSPEND_MEM,
	},
};

static struct regulator_consumer_supply tuna_vaux3_supplies[] = {
	{
		.supply = "vlcd",
	},
};

static struct regulator_init_data tuna_vaux3 = {
	.constraints = {
		.min_uV			= 3100000,
		.max_uV			= 3100000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies = ARRAY_SIZE(tuna_vaux3_supplies),
	.consumer_supplies = tuna_vaux3_supplies,
};

static struct regulator_init_data tuna_vmmc = {
	.constraints = {
		.min_uV			= 1800000,
		.max_uV			= 1800000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies = 2,
	.consumer_supplies = tuna_vmmc_supply,
};

static struct regulator_init_data tuna_vpp = {
	.constraints = {
		.min_uV			= 1800000,
		.max_uV			= 2500000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
		.state_mem = {
			.disabled	= true,
		},
		.initial_state		= PM_SUSPEND_MEM,
	},
};

static struct regulator_consumer_supply tuna_vusim_supplies[] = {
	{
		.supply = "vlcd-iovcc",
	},
};

static struct regulator_init_data tuna_vusim = {
	.constraints = {
		.min_uV			= 2200000,
		.max_uV 		= 2200000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 	= REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies = ARRAY_SIZE(tuna_vusim_supplies),
	.consumer_supplies = tuna_vusim_supplies,
};

static struct regulator_init_data tuna_vana = {
	.constraints = {
		.min_uV			= 2100000,
		.max_uV			= 2100000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
		.always_on	= true,
	},
};

static struct regulator_consumer_supply tuna_vcxio_supply[] = {
	REGULATOR_SUPPLY("vdds_dsi", "omapdss_dss"),
	REGULATOR_SUPPLY("vdds_dsi", "omapdss_dsi1"),
};

static struct regulator_init_data tuna_vcxio = {
	.constraints = {
		.min_uV			= 1800000,
		.max_uV			= 1800000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= ARRAY_SIZE(tuna_vcxio_supply),
	.consumer_supplies	= tuna_vcxio_supply,

};

static struct regulator_consumer_supply tuna_vdac_supply[] = {
	{
		.supply = "hdmi_vref",
	},
};

static struct regulator_init_data tuna_vdac = {
	.constraints = {
		.min_uV			= 1800000,
		.max_uV			= 1800000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= ARRAY_SIZE(tuna_vdac_supply),
	.consumer_supplies	= tuna_vdac_supply,
};

static struct regulator_consumer_supply tuna_vusb_supply[] = {
	REGULATOR_SUPPLY("vusb", "tuna_otg"),
};

static struct regulator_init_data tuna_vusb = {
	.constraints = {
		.min_uV			= 3300000,
		.max_uV			= 3300000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 =	REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
		.state_mem = {
			.disabled	= true,
		},
		.initial_state		= PM_SUSPEND_MEM,
	},
	.num_consumer_supplies	= ARRAY_SIZE(tuna_vusb_supply),
	.consumer_supplies	= tuna_vusb_supply,
};

/* clk32kg is a twl6030 32khz clock modeled as a regulator, used by GPS */
static struct regulator_init_data tuna_clk32kg = {
	.constraints = {
		.valid_ops_mask		= REGULATOR_CHANGE_STATUS,
		.always_on		= true,
	},
};

static struct regulator_consumer_supply tuna_clk32kaudio_supply[] = {
	{
		.supply = "clk32kaudio",
	},
	{
		.supply = "twl6040_clk32k",
	}
};

static struct regulator_init_data tuna_clk32kaudio = {
	.constraints = {
		.valid_ops_mask		= REGULATOR_CHANGE_STATUS,
		.boot_on                = true,
	},
	.num_consumer_supplies  = ARRAY_SIZE(tuna_clk32kaudio_supply),
	.consumer_supplies      = tuna_clk32kaudio_supply,
};


static struct regulator_init_data tuna_vdd3 = {
	.constraints = {
		.valid_ops_mask		= REGULATOR_CHANGE_STATUS,
		.state_mem = {
			.disabled	= true,
		},
		.initial_state		= PM_SUSPEND_MEM,
	},
};

/*
 * VMEM is unused. Register it to regulator framework and let it
 * be in disabled state.
 */
static struct regulator_init_data tuna_vmem = {
	.constraints = {
		.valid_ops_mask		= REGULATOR_CHANGE_STATUS,
		.state_mem = {
			.disabled	= true,
		},
		.initial_state		= PM_SUSPEND_MEM,
	},
};

static struct regulator_init_data tuna_v2v1 = {
	.constraints = {
		.min_uV			= 2100000,
		.max_uV			= 2100000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
		.always_on		= true,
	},
};

static struct twl4030_platform_data tuna_twldata = {
	/* Regulators */
	.vmmc		= &tuna_vmmc,
	.vpp		= &tuna_vpp,
	.vusim		= &tuna_vusim,
	.vana		= &tuna_vana,
	.vcxio		= &tuna_vcxio,
	.vdac		= &tuna_vdac,
	.vusb		= &tuna_vusb,
	.vaux1		= &tuna_vaux1,
	.vaux2		= &tuna_vaux2,
	.vaux3		= &tuna_vaux3,
	.clk32kg	= &tuna_clk32kg,

	/* children */
	.audio		= &twl6040_data,
	.madc		= &twl6030_madc,

	/* SMPS */
	.vdd3		= &tuna_vdd3,
	//.vmem		= &tuna_vmem,
	.v2v1		= &tuna_v2v1,
};


/*
 * I2C
 */

static struct i2c_board_info __initdata tuna_i2c1_boardinfo[] = {
	{
		I2C_BOARD_INFO("twl6030", 0x48),
		.flags = I2C_CLIENT_WAKE,
		//.irq = OMAP44XX_IRQ_SYS_1N,
		.platform_data = &tuna_twldata,
	},
};

static struct i2c_board_info __initdata tuna_i2c2_boardinfo[] = {
	{
		I2C_BOARD_INFO("ducati", 0x20),
		//.irq = OMAP44XX_IRQ_I2C2,
		//.ext_master = true,
	},
};

static struct i2c_board_info __initdata tuna_i2c4_boardinfo[] = {
	{
		I2C_BOARD_INFO("an30259a", 0x30),
	},
};

static int __init tuna_i2c_init(void)
{
	u32 r;

	/*omap_mux_init_signal("sys_nirq1", OMAP_PIN_INPUT_PULLUP |
						OMAP_WAKEUP_EN);
	omap_mux_init_signal("i2c1_scl.i2c1_scl", OMAP_PIN_INPUT_PULLUP);
	omap_mux_init_signal("i2c1_sda.i2c1_sda", OMAP_PIN_INPUT_PULLUP);*/

	/*
	 * This will allow unused regulator to be shutdown. This flag
	 * should be set in the board file. Before regulators are registered.
	 */
	regulator_has_full_constraints();

	/*
	 * Phoenix Audio IC needs I2C1 to
	 * start with 400 KHz or less
	 */
	/*omap_register_i2c_bus(1, 400, tuna_i2c1_boardinfo,
			      ARRAY_SIZE(tuna_i2c1_boardinfo));*/
	omap4_pmic_init("twl6030", &tuna_twldata, NULL, 0);

	omap_register_i2c_bus(2, 400, tuna_i2c2_boardinfo,
                              ARRAY_SIZE(tuna_i2c2_boardinfo));
	omap_register_i2c_bus(3, 400, NULL, 0);

	/* Disable internal pullup on i2c.4 line:
	 * as external 2.2K is already present
	 */
	r = omap4_ctrl_pad_readl(OMAP4_CTRL_MODULE_PAD_CORE_CONTROL_I2C_0);
	r |= (1 << OMAP4_I2C4_SDA_PULLUPRESX_SHIFT);
	omap4_ctrl_pad_writel(r, OMAP4_CTRL_MODULE_PAD_CORE_CONTROL_I2C_0);

	omap_register_i2c_bus(4, 400, NULL, 0);

	/*
	 * Drive MSECURE high for TWL6030 write access.
	 */
	omap_mux_init_signal("fref_clk0_out.gpio_wk6", OMAP_PIN_OUTPUT);
	gpio_request(6, "msecure");
	gpio_direction_output(6, 1);

	return 0;
}


/*
 * GPIO Mux Support
 */

#ifdef CONFIG_OMAP_MUX
static struct omap_board_mux board_mux[] __initdata = {
	/* WLAN IRQ - GPIO 53 */
	OMAP4_MUX(GPMC_NCS3, OMAP_MUX_MODE3 | OMAP_PIN_INPUT),
	/* WLAN POWER ENABLE - GPIO 43 */
	OMAP4_MUX(GPMC_A19, OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT),
	/* WLAN SDIO: MMC5 CMD */
	OMAP4_MUX(SDMMC5_CMD, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP),
	/* WLAN SDIO: MMC5 CLK */
	OMAP4_MUX(SDMMC5_CLK, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP),
	/* WLAN SDIO: MMC5 DAT[0-3] */
	OMAP4_MUX(SDMMC5_DAT0, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP),
	OMAP4_MUX(SDMMC5_DAT1, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP),
	OMAP4_MUX(SDMMC5_DAT2, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP),
	OMAP4_MUX(SDMMC5_DAT3, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP),
	/* gpio 0 - TFP410 PD */
	OMAP4_MUX(KPD_COL1, OMAP_PIN_OUTPUT | OMAP_MUX_MODE3),
	/* dispc2_data23 */
	OMAP4_MUX(USBB2_ULPITLL_STP, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data22 */
	OMAP4_MUX(USBB2_ULPITLL_DIR, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data21 */
	OMAP4_MUX(USBB2_ULPITLL_NXT, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data20 */
	OMAP4_MUX(USBB2_ULPITLL_DAT0, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data19 */
	OMAP4_MUX(USBB2_ULPITLL_DAT1, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data18 */
	OMAP4_MUX(USBB2_ULPITLL_DAT2, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data15 */
	OMAP4_MUX(USBB2_ULPITLL_DAT3, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data14 */
	OMAP4_MUX(USBB2_ULPITLL_DAT4, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data13 */
	OMAP4_MUX(USBB2_ULPITLL_DAT5, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data12 */
	OMAP4_MUX(USBB2_ULPITLL_DAT6, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data11 */
	OMAP4_MUX(USBB2_ULPITLL_DAT7, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data10 */
	OMAP4_MUX(DPM_EMU3, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data9 */
	OMAP4_MUX(DPM_EMU4, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data16 */
	OMAP4_MUX(DPM_EMU5, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data17 */
	OMAP4_MUX(DPM_EMU6, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_hsync */
	OMAP4_MUX(DPM_EMU7, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_pclk */
	OMAP4_MUX(DPM_EMU8, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_vsync */
	OMAP4_MUX(DPM_EMU9, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_de */
	OMAP4_MUX(DPM_EMU10, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data8 */
	OMAP4_MUX(DPM_EMU11, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data7 */
	OMAP4_MUX(DPM_EMU12, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data6 */
	OMAP4_MUX(DPM_EMU13, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data5 */
	OMAP4_MUX(DPM_EMU14, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data4 */
	OMAP4_MUX(DPM_EMU15, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data3 */
	OMAP4_MUX(DPM_EMU16, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data2 */
	OMAP4_MUX(DPM_EMU17, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data1 */
	OMAP4_MUX(DPM_EMU18, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data0 */
	OMAP4_MUX(DPM_EMU19, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* NIRQ2 for twl6040 */
	OMAP4_MUX(SYS_NIRQ2, OMAP_MUX_MODE0 |
		  OMAP_PIN_INPUT_PULLUP | OMAP_PIN_OFF_WAKEUPENABLE),
	/* GPIO_127 for twl6040 */
	OMAP4_MUX(HDQ_SIO, OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT),
	/* McPDM */
	OMAP4_MUX(ABE_PDM_UL_DATA, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLDOWN),
	OMAP4_MUX(ABE_PDM_DL_DATA, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLDOWN),
	OMAP4_MUX(ABE_PDM_FRAME, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP),
	OMAP4_MUX(ABE_PDM_LB_CLK, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLDOWN),
	OMAP4_MUX(ABE_CLKS, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLDOWN),
	/* McBSP1 */
	OMAP4_MUX(ABE_MCBSP1_CLKX, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
	OMAP4_MUX(ABE_MCBSP1_DR, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLDOWN),
	OMAP4_MUX(ABE_MCBSP1_DX, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT |
		  OMAP_PULL_ENA),
	OMAP4_MUX(ABE_MCBSP1_FSX, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),

	/* UART2 - BT/FM/GPS shared transport */
	OMAP4_MUX(UART2_CTS,	OMAP_PIN_INPUT	| OMAP_MUX_MODE0),
	OMAP4_MUX(UART2_RTS,	OMAP_PIN_OUTPUT	| OMAP_MUX_MODE0),
	OMAP4_MUX(UART2_RX,	OMAP_PIN_INPUT	| OMAP_MUX_MODE0),
	OMAP4_MUX(UART2_TX,	OMAP_PIN_OUTPUT	| OMAP_MUX_MODE0),

	{ .reg_offset = OMAP_MUX_TERMINATOR },
};

static struct omap_board_mux board_wkup_mux[] __initdata = {
	/* power button */
	OMAP4_MUX(SIM_CD, OMAP_MUX_MODE3 | OMAP_PIN_INPUT),
	{ .reg_offset = OMAP_MUX_TERMINATOR },
};

#else
#define board_mux	NULL
#define board_wkup_mux	NULL
#endif


#if 0
/*
 * UART
 */
 
/* sample4+ adds gps rts/cts lines */
static struct omap_device_pad tuna_uart1_pads_sample4[] __initdata = {
	{
		.name	= "mcspi1_cs3.uart1_rts",
		.enable	= OMAP_PIN_OUTPUT | OMAP_MUX_MODE1,
	},
	{
		.name	= "mcspi1_cs2.uart1_cts",
		.enable	= OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE1,
	},
	{
		.name   = "uart3_cts_rctx.uart1_tx",
		.enable = OMAP_PIN_OUTPUT | OMAP_MUX_MODE1,
	},
	{
		.name   = "mcspi1_cs1.uart1_rx",
		.flags  = OMAP_DEVICE_PAD_REMUX | OMAP_DEVICE_PAD_WAKEUP,
		.enable = OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE1,
		.idle   = OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE1,
	},
};

static struct omap_device_pad tuna_uart1_pads[] __initdata = {
	{
		.name   = "uart3_cts_rctx.uart1_tx",
		.enable = OMAP_PIN_OUTPUT | OMAP_MUX_MODE1,
	},
	{
		.name   = "mcspi1_cs1.uart1_rx",
		.flags  = OMAP_DEVICE_PAD_REMUX | OMAP_DEVICE_PAD_WAKEUP,
		.enable = OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE1,
		.idle   = OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE1,
	},
};

static struct omap_device_pad tuna_uart2_pads[] __initdata = {
	{
		.name	= "uart2_cts.uart2_cts",
		.enable	= OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE0,
	},
	{
		.name	= "uart2_rts.uart2_rts",
		.enable	= OMAP_PIN_OUTPUT | OMAP_MUX_MODE0,
	},
	{
		.name	= "uart2_tx.uart2_tx",
		.enable	= OMAP_PIN_OUTPUT | OMAP_MUX_MODE0,
	},
	{
		.name	= "uart2_rx.uart2_rx",
		.enable	= OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE0,
	},
};

static struct omap_device_pad tuna_uart3_pads[] __initdata = {
	{
		.name   = "uart3_tx_irtx.uart3_tx_irtx",
		.enable = OMAP_PIN_OUTPUT | OMAP_MUX_MODE0,
	},
	{
		.name   = "uart3_rx_irrx.uart3_rx_irrx",
		.flags  = OMAP_DEVICE_PAD_REMUX | OMAP_DEVICE_PAD_WAKEUP,
		.enable = OMAP_PIN_INPUT | OMAP_MUX_MODE0,
		.idle   = OMAP_PIN_INPUT | OMAP_MUX_MODE0,
	},
};

static struct omap_device_pad tuna_uart4_pads[] __initdata = {
	{
		.name	= "uart4_tx.uart4_tx",
		.enable	= OMAP_PIN_OUTPUT | OMAP_MUX_MODE0,
	},
	{
		.name	= "uart4_rx.uart4_rx",
		.enable	= OMAP_PIN_INPUT | OMAP_MUX_MODE0,
	},
};

static struct omap_uart_port_info tuna_uart2_info __initdata = {
	.use_dma	= 0,
/*	.dma_rx_buf_size = DEFAULT_RXDMA_BUFSIZE,
	.dma_rx_poll_rate = DEFAULT_RXDMA_POLLRATE,
	.dma_rx_timeout = DEFAULT_RXDMA_TIMEOUT,*/
	.auto_sus_timeout = 0,
	.wake_peer	= bcm_bt_lpm_exit_lpm_locked,
	.rts_mux_driver_control = 1,
};

static inline void __init board_serial_init(void)
{
	struct omap_device_pad *uart1_pads;
	int uart1_pads_sz;

	if (omap4_tuna_get_revision() >= TUNA_REV_SAMPLE_4) {
		uart1_pads = tuna_uart1_pads_sample4;
		uart1_pads_sz = ARRAY_SIZE(tuna_uart1_pads_sample4);
	} else {
		uart1_pads = tuna_uart1_pads;
		uart1_pads_sz = ARRAY_SIZE(tuna_uart1_pads);
	}

	omap_serial_init_port_pads(0, uart1_pads, uart1_pads_sz, NULL);
	omap_serial_init_port_pads(1, tuna_uart2_pads,
		ARRAY_SIZE(tuna_uart2_pads), &tuna_uart2_info);
	omap_serial_init_port_pads(3, tuna_uart4_pads,
				   ARRAY_SIZE(tuna_uart4_pads), NULL);
}

/* fiq_debugger initializes really early but OMAP resource mgmt
 * is not yet ready @ arch_init, so init the serial debugger later */
static int __init board_serial_debug_init(void)
{
	return omap_serial_debug_init(2, false, true,
			tuna_uart3_pads, ARRAY_SIZE(tuna_uart3_pads));
}
device_initcall(board_serial_debug_init);
#endif /* 0 */


/* 
 * Miscellaneous Routines
 */
struct class *sec_class;
EXPORT_SYMBOL(sec_class);

static int __init sec_common_init(void)
{
	sec_class = class_create(THIS_MODULE, "sec");
	if (IS_ERR(sec_class))
		pr_err("Failed to create class(sec)!\n");

	return 0;
}


/*
 * Low Level Bring-up Routines
 */

static struct resource ramconsole_resources[] = {
	{
		.flags  = IORESOURCE_MEM,
		.start	= TUNA_RAMCONSOLE_START,
		.end	= TUNA_RAMCONSOLE_START + TUNA_RAMCONSOLE_SIZE - 1,
	},
};

//static struct ram_console_platform_data ramconsole_pdata;

static struct platform_device ramconsole_device = {
	.name           = "ram_console",
	.id             = -1,
	.num_resources  = ARRAY_SIZE(ramconsole_resources),
	.resource       = ramconsole_resources,
	/*.dev		= {
		.platform_data = &ramconsole_pdata,
	},*/
};

static struct platform_device *tuna_devices[] __initdata = {
	&ramconsole_device,
	&wl1271_device,
	&twl6030_madc_device,
	&tuna_abe_audio,
	&tuna_hdmi_audio_codec,
	&btwilink_device,
};

/*
 * Store a handy board information string which we can use elsewhere like
 * like in panic situation
 */
static char omap4_tuna_bd_info_string[255];
static void omap4_tuna_init_hw_rev(void)
{
	int ret;
	int i;
	u32 r;

	/* Disable weak driver pulldown on usbb2_hsic_strobe */
	r = omap4_ctrl_pad_readl(OMAP4_CTRL_MODULE_PAD_CORE_CONTROL_USBB_HSIC);
	r &= ~OMAP4_USBB2_HSIC_STROBE_WD_MASK;
	omap4_ctrl_pad_writel(r, OMAP4_CTRL_MODULE_PAD_CORE_CONTROL_USBB_HSIC);

	ret = gpio_request_array(tuna_hw_rev_gpios,
		ARRAY_SIZE(tuna_hw_rev_gpios));

	BUG_ON(ret);

	for (i = 0; i < ARRAY_SIZE(tuna_hw_rev_gpios); i++)
		tuna_hw_rev |= gpio_get_value(tuna_hw_rev_gpios[i].gpio) << i;

	snprintf(omap4_tuna_bd_info_string,
		ARRAY_SIZE(omap4_tuna_bd_info_string),
		"Tuna HW revision: %02x (%s), cpu %s ES%d.%d ",
		tuna_hw_rev,
		omap4_tuna_hw_rev_name(),
		cpu_is_omap443x() ? "OMAP4430" : "OMAP4460",
		(GET_OMAP_REVISION() >> 4) & 0xf,
		GET_OMAP_REVISION() & 0xf);

	pr_info("%s\n", omap4_tuna_bd_info_string);
	mach_panic_string = omap4_tuna_bd_info_string;
}

/* always reboot into charger mode on "power-off". Let the bootloader
 * figure out if we should truly power-off or not.
 */
static void tuna_power_off(void)
{
	printk(KERN_EMERG "Rebooting into bootloader for power-off.\n");
	arm_pm_restart('c', NULL);
}

static int tuna_notifier_call(struct notifier_block *this,
					unsigned long code, void *_cmd)
{
	void __iomem *sar_base;
	unsigned int flag = REBOOT_FLAG_NORMAL;

	sar_base = omap4_get_sar_ram_base();

	if (!sar_base)
		return notifier_from_errno(-ENOMEM);

	if (code == SYS_RESTART) {
		if (_cmd) {
			if (!strcmp(_cmd, "recovery"))
				flag = REBOOT_FLAG_RECOVERY;
			else if (!strcmp(_cmd, "bootloader"))
				flag = REBOOT_FLAG_FASTBOOT;
		}
	} else if (code == SYS_POWER_OFF) {
		flag = REBOOT_FLAG_POWER_OFF;
	}

	/* The Samsung LOKE bootloader will look for the boot flag at a fixed
	 * offset from the end of the 1st SAR bank.
	 */
	writel(flag, sar_base + SAR_BANK2_OFFSET - 0xC);

#if defined(CONFIG_DSSCOMP) && defined(CONFIG_EARLYSUSPEND)
	/*
	 * HACK: Blank screen to avoid screen artifacts due to removal of
	 * DSS/panel drivers shutdown in reboot path.
	 */
	{
		extern void dsscomp_early_suspend(struct early_suspend *h);

		dsscomp_early_suspend(NULL);
	}
#endif

	return NOTIFY_DONE;
}

static struct notifier_block tuna_reboot_notifier = {
	.notifier_call = tuna_notifier_call,
};

static void __init tuna_init(void)
{
	int package = OMAP_PACKAGE_CBS;
	int ret;

	if (omap_rev() == OMAP4430_REV_ES1_0)
		package = OMAP_PACKAGE_CBL;
	omap4_mux_init(board_mux, board_wkup_mux, package);

	omap4_tuna_init_hw_rev();

	// FIXME: handled by DTS?
	//omap4_tuna_emif_init();

	pm_power_off = tuna_power_off;

	register_reboot_notifier(&tuna_reboot_notifier);

	/* hsmmc d0-d7 */
	omap_mux_init_signal("sdmmc1_dat0.sdmmc1_dat0", HSMMC1_MUX);
	omap_mux_init_signal("sdmmc1_dat1.sdmmc1_dat1", HSMMC1_MUX);
	omap_mux_init_signal("sdmmc1_dat2.sdmmc1_dat2", HSMMC1_MUX);
	omap_mux_init_signal("sdmmc1_dat3.sdmmc1_dat3", HSMMC1_MUX);
	omap_mux_init_signal("sdmmc1_dat4.sdmmc1_dat4", HSMMC1_MUX);
	omap_mux_init_signal("sdmmc1_dat5.sdmmc1_dat5", HSMMC1_MUX);
	omap_mux_init_signal("sdmmc1_dat6.sdmmc1_dat6", HSMMC1_MUX);
	omap_mux_init_signal("sdmmc1_dat7.sdmmc1_dat7", HSMMC1_MUX);
	/* hsmmc cmd */
	omap_mux_init_signal("sdmmc1_cmd.sdmmc1_cmd", HSMMC1_MUX);
	/* hsmmc clk */
	omap_mux_init_signal("sdmmc1_clk.sdmmc1_clk", HSMMC1_MUX);

	gpio_request(158, "emmc_en");
	gpio_direction_output(158, 1);
	omap_mux_init_gpio(158, OMAP_PIN_INPUT_PULLUP);

	omap_mux_init_gpio(GPIO_MHL_SDA_18V, OMAP_PIN_INPUT_PULLUP);
	omap_mux_init_gpio(GPIO_MHL_SCL_18V, OMAP_PIN_INPUT_PULLUP);

	sec_common_init();

	if (TUNA_TYPE_TORO == omap4_tuna_get_type()) {
		omap_mux_init_signal("gpmc_wait0",
				OMAP_MUX_MODE3 | OMAP_PIN_INPUT_PULLDOWN);
		gpio_request(OMAP_GPIO_CMC2AP_INT1, "gpio_61");
		gpio_direction_input(OMAP_GPIO_CMC2AP_INT1);

		omap_mux_init_signal("mcspi4_clk", OMAP_MUX_MODE0);
		omap_mux_init_signal("mcspi4_simo", OMAP_MUX_MODE0);
		omap_mux_init_signal("mcspi4_somi", OMAP_MUX_MODE0);
		omap_mux_init_signal("mcspi4_cs0", OMAP_MUX_MODE0);
	}

	/* ----------------
	omap_panda_wlan_data.irq = gpio_to_irq(GPIO_WIFI_IRQ);
	ret = wl12xx_set_platform_data(&omap_panda_wlan_data);
	if (ret)
		pr_err("error setting wl12xx data: %d\n", ret);*/

	tuna_i2c_init();
	platform_add_devices(tuna_devices, ARRAY_SIZE(tuna_devices));
	//platform_device_register(&omap_vwlan_device);
	omap_serial_init();
	omap_sdrc_init(NULL, NULL);
	omap4_twl6030_hsmmc_init(mmc);
	omap4_ehci_init();
	usb_bind_phy("musb-hdrc.2.auto", 0, "omap-usb2.3.auto");
	usb_musb_init(&musb_board_data);
	omap4_tuna_display_init();
}

static void __init tuna_reserve(void)
{
	int i;
	int ret;

	/* do the static reservations first */
	memblock_remove(TUNA_RAMCONSOLE_START, TUNA_RAMCONSOLE_SIZE);
	memblock_remove(PHYS_ADDR_SMC_MEM, PHYS_ADDR_SMC_SIZE);
	memblock_remove(PHYS_ADDR_DUCATI_MEM, PHYS_ADDR_DUCATI_SIZE);

#if 0
	for (i = 0; i < tuna_ion_data.nr; i++)
		if (tuna_ion_data.heaps[i].type == ION_HEAP_TYPE_CARVEOUT ||
		    tuna_ion_data.heaps[i].type == OMAP_ION_HEAP_TYPE_TILER) {
			ret = memblock_remove(tuna_ion_data.heaps[i].base,
					      tuna_ion_data.heaps[i].size);
			if (ret)
				pr_err("memblock remove of %x@%lx failed\n",
				       tuna_ion_data.heaps[i].size,
				       tuna_ion_data.heaps[i].base);
		}

	/* ipu needs to recognize secure input buffer area as well */
	omap_ipu_set_static_mempool(PHYS_ADDR_DUCATI_MEM, PHYS_ADDR_DUCATI_SIZE +
					OMAP_TUNA_ION_HEAP_SECURE_INPUT_SIZE);
#endif /* 0 */
	omap_reserve();
}

#define PHOENIX_LAST_TURNOFF_STS	0x22
static int __init tuna_print_last_turnon_sts(void)
{
	u8 turnon_sts;
	int err;

	err = twl_i2c_read_u8(TWL6030_MODULE_ID0, &turnon_sts,
			PHOENIX_LAST_TURNOFF_STS);

	if (!err) {
		pr_info("PHOENIX_LAST_TURNOFF_STS: 0x%02x\n", turnon_sts);
		twl_i2c_write_u8(TWL6030_MODULE_ID0, turnon_sts,
				 PHOENIX_LAST_TURNOFF_STS);
	}

	return 0;
}
device_initcall(tuna_print_last_turnon_sts);

MACHINE_START(TUNA, "Tuna")
	/* Maintainer: Google, Inc */
	.atag_offset	= 0x100,
	.smp		= smp_ops(omap4_smp_ops),
	.reserve	= tuna_reserve,
	.map_io		= omap4_map_io,
	.init_early	= omap4430_init_early,
	.init_irq	= gic_init_irq,
	.init_machine	= tuna_init,
	.init_late	= omap4430_init_late,
	.init_time	= omap4_local_timer_init,
	.restart	= omap44xx_restart,
MACHINE_END
