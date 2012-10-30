/*
 * Copyright (C) 2004, 2007-2010, 2011-2012 Synopsys, Inc. (www.synopsys.com)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * vineetg: Feb 2009
 *  -For AA4 board, System Memory Map for Peripherals etc
 */

#ifndef __PLAT_MEMMAP_H
#define __PLAT_MEMMAP_H

#define UART0_BASE              0xC0FC1000
#define UART1_BASE              0xC0FC1100

#define IDE_CONTROLLER_BASE     0xC0FC9000

#define AHB_PCI_HOST_BRG_BASE   0xC0FD0000

#define PGU_BASEADDR            0xC0FC8000
#define VLCK_ADDR               0xC0FCF028

#define BVCI_LAT_UNIT_BASE      0xC0FED000

#define PS2_BASE_ADDR		0xC0FCC000

#if 0
#include <linux/init.h>

#define printk_init(fmt, ...)			\
({							\
	static const __initconst char __fmt[] = fmt;	\
	pr_info(__fmt, ##__VA_ARGS__);				\
})

#else
#define printk_init(fmt, ...)   pr_info(fmt, ##__VA_ARGS__)
#endif

#endif
