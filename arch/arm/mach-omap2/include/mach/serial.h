/*
 * Copyright (C) 2009 Texas Instruments
 * Added OMAP4 support- Santosh Shilimkar <santosh.shilimkar@ti.com>
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include <asm/memory.h>

#define OMAP_UART_INFO		(PHYS_OFFSET + 0x3ffc)

/* OMAP2 serial ports */
#define OMAP2_UART1_BASE	0x4806a000
#define OMAP2_UART2_BASE	0x4806c000
#define OMAP2_UART3_BASE	0x4806e000

/* OMAP3 serial ports */
#define OMAP3_UART1_BASE	OMAP2_UART1_BASE
#define OMAP3_UART2_BASE	OMAP2_UART2_BASE
#define OMAP3_UART3_BASE	0x49020000
#define OMAP3_UART4_BASE	0x49042000	/* Only on 36xx */
#define OMAP3_UART4_AM35XX_BASE	0x4809E000	/* Only on AM35xx */

/* OMAP4 serial ports */
#define OMAP4_UART1_BASE	OMAP2_UART1_BASE
#define OMAP4_UART2_BASE	OMAP2_UART2_BASE
#define OMAP4_UART3_BASE	0x48020000
#define OMAP4_UART4_BASE	0x4806e000

/* TI81XX serial ports */
#define TI81XX_UART1_BASE	0x48020000
#define TI81XX_UART2_BASE	0x48022000
#define TI81XX_UART3_BASE	0x48024000

/* AM3505/3517 UART4 */
#define AM35XX_UART4_BASE	0x4809E000	/* Only on AM3505/3517 */

/* AM33XX serial port */
#define AM33XX_UART1_BASE	0x44E09000

/* OMAP5 serial ports */
#define OMAP5_UART1_BASE	OMAP2_UART1_BASE
#define OMAP5_UART2_BASE	OMAP2_UART2_BASE
#define OMAP5_UART3_BASE	OMAP4_UART3_BASE
#define OMAP5_UART4_BASE	OMAP4_UART4_BASE
#define OMAP5_UART5_BASE	0x48066000
#define OMAP5_UART6_BASE	0x48068000

/* External port on Zoom2/3 */
#define ZOOM_UART_BASE		0x10000000
#define ZOOM_UART_VIRT		0xfa400000

#define OMAP_PORT_SHIFT		2
#define ZOOM_PORT_SHIFT		1

#define OMAP24XX_BASE_BAUD	(48000000/16)

/*
 * DEBUG_LL port encoding stored into the UART1 scratchpad register by
 * decomp_setup in uncompress.h
 */
#define OMAP2UART1		21
#define OMAP2UART2		22
#define OMAP2UART3		23
#define OMAP3UART1		OMAP2UART1
#define OMAP3UART2		OMAP2UART2
#define OMAP3UART3		33
#define OMAP3UART4		34		/* Only on 36xx */
#define OMAP4UART1		OMAP2UART1
#define OMAP4UART2		OMAP2UART2
#define OMAP4UART3		43
#define OMAP4UART4		44
#define TI816XUART1		81
#define TI816XUART2		82
#define TI816XUART3		83
#define ZOOM_UART		95		/* Only on zoom2/3 */

#ifndef __ASSEMBLER__

struct omap_board_data;
struct omap_uart_port_info;

extern void omap_serial_init(void);
extern void omap_serial_board_init(struct omap_uart_port_info *platform_data);
extern void omap_serial_init_port(struct omap_board_data *bdata,
		struct omap_uart_port_info *platform_data);
extern int omap_serial_wake(int port_num);
extern int omap_serial_enable(int port_num);
extern int omap_serial_disable(int port_num);

extern void omap_rts_mux_write(u16 val, int num);
#endif
