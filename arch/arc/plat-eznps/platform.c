/*******************************************************************************

  EZNPS Platform support code
  Copyright(c) 2012 EZchip Technologies.

  This program is free software; you can redistribute it and/or modify it
  under the terms and conditions of the GNU General Public License,
  version 2, as published by the Free Software Foundation.

  This program is distributed in the hope it will be useful, but WITHOUT
  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
  more details.

  The full GNU General Public License is included in this distribution in
  the file called "COPYING".

*******************************************************************************/

#include <linux/types.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/console.h>
#include <linux/if_ether.h>
#include <linux/socket.h>
#include <asm/setup.h>
#include <asm/mach_desc.h>
#include <asm/clk.h>
#include <plat/irq.h>
#include <plat/memmap.h>
#include <plat/time.h>
#ifdef CONFIG_SMP
#include <plat/smp.h>
#endif

/*----------------------- Platform Devices -----------------------------*/

#ifdef CONFIG_SERIAL_8250
#include <linux/io.h>
#include <linux/serial_8250.h>

unsigned int serial8250_early_in(struct uart_port *port, int offset)
{
	return __raw_readl(port->membase + (offset << 2));
}

void serial8250_early_out(struct uart_port *port, int offset, int value)
{
	__raw_writel(value, port->membase + (offset << 2));
}

static struct plat_serial8250_port uart8250_data[] = {
	{
		.type = PORT_16550A,
		.irq = UART0_IRQ,
		.flags = (UPF_FIXED_TYPE | UPF_SKIP_TEST) ,
		.membase = (void __iomem *)UART0_BASE,
		.mapbase = (resource_size_t)UART0_BASE,
		.iotype = UPIO_MEM32,
		.regshift = 2,
		/* asm-generic/io.h always assume LE */
		.serial_in = serial8250_early_in,
		.serial_out = serial8250_early_out,
	},
	{ },
};

static struct platform_device arc_uart8250_dev = {
	.name = "serial8250",
	.id = PLAT8250_DEV_PLATFORM,
	.dev = {
		.platform_data = uart8250_data,
	}
};

#endif	/* CONFIG_SERIAL_8250 */

/*
 * Early Platform Initialization called from setup_arch()
 */
unsigned long eznps_he_version;
static void __init eznps_plat_early_init(void)
{
	pr_info("[plat-eznps]: registering early dev resources\n");

	if (running_on_hw) {
		unsigned long hw_ver, major_ver, minor_ver;

		hw_ver = __raw_readl(REG_CPU_HW_VER);
		major_ver = (hw_ver & HW_VER_MAJOR_MASK) >> HW_VER_MAJOR_SHIFT;
		minor_ver = (hw_ver & HW_VER_MINOR_MASK) >> HW_VER_MINOR_SHIFT;
		eznps_he_version = (major_ver > 1) ? 1 : 0;
		pr_info("[plat-eznps]: FPGA (HE%lu) version %02lx.%02lx\n",
			 eznps_he_version, major_ver, minor_ver);

		if (eznps_he_version == 1)
			arc_set_core_freq(__raw_readl(REG_CPU_HW_CLK));
	}

#ifdef CONFIG_SERIAL_8250
	/*
	 * Now that we set core frequency,
	 * BASE_BAUD is calculated properly.
	 * parse_early_param() already called once so
	 * calling parse_early_options() directly.
	 */
	{
		static __initdata char tmp_cmdline[COMMAND_LINE_SIZE];
		sprintf(tmp_cmdline, "earlycon=uart8250,mmio32,0x%X,115200n8",
						(unsigned int)UART0_BASE);
		parse_early_options(tmp_cmdline);
	}
#endif

#ifdef CONFIG_SMP
	eznps_init_early_smp();
#endif
}

static void __init eznps_plat_cmdline_init(char *cmdline)
{
       /* Special case for getting command line from nSIM */
        if (!running_on_hw)
                strlcat(cmdline, CMDLINE_BASE, COMMAND_LINE_SIZE);
}

static struct platform_device *arc_devs[] = {
#ifdef CONFIG_SERIAL_8250
	&arc_uart8250_dev,
#endif
};

static void __init eznps_plat_init(void)
{
	pr_info("[plat-eznps]: registering device resources\n");

	uart8250_data[0].uartclk = arc_get_core_freq();

	platform_add_devices(arc_devs, ARRAY_SIZE(arc_devs));
}

/*----------------------- Machine Descriptions ------------------------------
 *
 * Machine description is simply a set of platform/board specific callbacks
 * This is not directly related to DeviceTree based dynamic device creation,
 * however as part of early device tree scan, we also select the right
 * callback set, by matching the DT compatible name.
 */

static const char *eznps_compat[] __initdata = {
	"ezchip,arc-nps",
	NULL,
};

MACHINE_START(NPS, "nps")
	.dt_compat	= eznps_compat,
	.init_cmdline	= eznps_plat_cmdline_init,
	.init_early	= eznps_plat_early_init,
	.init_machine	= eznps_plat_init,
#ifdef CONFIG_SMP
	.init_time	= eznps_counter_setup,
	.init_smp	= eznps_smp_init_cpu,  /* for each CPU */
#endif
MACHINE_END

#ifdef CONFIG_EZNPS_NET
/*
 * this MAC address is defined in network driver
 * and used in platform in order to initialize
 * it with value given from command line
 */
extern struct sockaddr mac_addr;

static int __init mac_addr_setup(char *mac)
{
	int i, h, l;

	for (i = 0; i < ETH_ALEN; i++) {
		if (i != ETH_ALEN-1 && *(mac + 2) != ':')
			return 1;

		h = hex_to_bin(*mac++);
		if (h == -1)
			return 1;

		l = hex_to_bin(*mac++);
		if (l == -1)
			return 1;

		mac++;
		mac_addr.sa_data[i] = (h << 4) + l;
	}

	return 0;
}

__setup("mac=", mac_addr_setup);

static int  __init add_eznet(void)
{
	struct platform_device *pd;

	pd = platform_device_register_simple("eth", 0, NULL, 0);
	if (IS_ERR(pd))
		pr_err("Fail\n");

	return IS_ERR(pd) ? PTR_ERR(pd) : 0;
}
device_initcall(add_eznet);
#endif /* CONFIG_EZNPS_NET */
