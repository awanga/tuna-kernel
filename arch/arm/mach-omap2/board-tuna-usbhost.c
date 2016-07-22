/* USB Host (EHCI) support for Samsung Tuna Board.
 *
 * Copyright (C) 2011 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/of_platform.h>

#include <linux/usb/nop-usb-xceiv.h>
#include "usb.h"

#include "board-tuna.h"
#include "mux.h"

#define GPIO_USB3333_RESETB     159

/* PHY device on HS USB Port 1 i.e. nop_usb_xceiv.1 */
static struct nop_usb_xceiv_platform_data hsusb1_phy_data = {
	/* FREF_CLK3 provides the 19.2 MHz reference clock to the PHY */
	.clk_rate = 19200000,
};

static struct usbhs_phy_data phy_data[] __initdata = {
	{
		.port = 1,
		.reset_gpio = GPIO_USB3333_RESETB,
		.platform_data = &hsusb1_phy_data,
	},
};

static struct usbhs_omap_platform_data usbhs_bdata __initdata = {
	.nports = 1,
	.port_mode[0] = OMAP_EHCI_PORT_MODE_PHY,
	.port_mode[1] = OMAP_USBHS_PORT_MODE_UNUSED,
	.port_mode[2] = OMAP_USBHS_PORT_MODE_UNUSED,
	.phy_reset  = false,
	.reset_gpio_port[0]  = -EINVAL,
	.reset_gpio_port[1]  = -EINVAL,
	.reset_gpio_port[2]  = -EINVAL
};

void __init omap4_ehci_init(void)
{
	int ret = 0;
	struct clk *phy_ref_clk;

	omap_mux_init_gpio(GPIO_USB3333_RESETB, OMAP_PIN_OUTPUT |
						OMAP_PIN_OFF_NONE);

	ret = gpio_request(GPIO_USB3333_RESETB, "usb3333_resetb");
	if (ret) {
		pr_err("omap: ehci: Cannot request GPIO %d",
					GPIO_USB3333_RESETB);
		return;
	}
	gpio_direction_output(GPIO_USB3333_RESETB, 0);
	gpio_set_value(GPIO_USB3333_RESETB, 0);

	/* FREF_CLK3 provides the 19.2 MHz reference clock to the PHY */
	omap_mux_init_signal("fref_clk3_out", OMAP_PIN_OUTPUT | OMAP_MUX_MODE0);

	phy_ref_clk = clk_get(NULL, "auxclk3_ck");
	if (IS_ERR(phy_ref_clk)) {
		pr_err("omap: ehci: Cannot request auxclk3");
		return;
	}
	ret = clk_set_rate(phy_ref_clk, 19200000);
	if (ret < 0) {
		pr_err("omap: ehci: Cannot clk_set_rate  auxclk3 err %d", ret);
		return;
	}
	ret = clk_enable(phy_ref_clk);
	if (ret < 0) {
		pr_err("omap: ehci: Cannot clk_enable auxclk3 err %d", ret);
		return;
	}

	ret = clk_add_alias("main_clk", "nop_usb_xceiv.1", "auxclk3_ck", NULL);
	if (ret)
		pr_err("Failed to add main_clk alias to auxclk3_ck\n");

	udelay(100);
	gpio_set_value(GPIO_USB3333_RESETB, 1);

	/* Everything went well with phy clock, pass it to ehci driver for
	* low power managment now
	*/
	/*usbhs_bdata.transceiver_clk[0] = phy_ref_clk;*/

	usbhs_init_phys(phy_data, ARRAY_SIZE(phy_data));
	usbhs_init(&usbhs_bdata);

	pr_info("usb:ehci initialized");
	return;
}
