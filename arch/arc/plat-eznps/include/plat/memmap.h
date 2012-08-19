/*******************************************************************************

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

#ifndef __PLAT_MEMMAP_H
#define __PLAT_MEMMAP_H

#define UART0_BASE			0xC0000000
#define CPU_REGS_BASE			0xC0002000

#define REG_CPU_HW_VER     		(unsigned int *)(CPU_REGS_BASE + 0x3FC)
#define HW_VER_MAJOR_SHIFT		8
#define HW_VER_MINOR_SHIFT		0
#define HW_VER_MAJOR_MASK		0xFF00
#define HW_VER_MINOR_MASK		0x00FF

#define REG_CPU_HW_CLK			(unsigned int *)(CPU_REGS_BASE + 0x3F8)

/* Simulator stuff */
#define CMDLINE_BASE			(char *)0xC0004000
#define ACTIVE_CPUS_BASE		(char *)0xC0005000
#define SIMULATED_CPUS_BASE		(char *)0xC0005100
#define CLOCK_SOURCE_BASE		(unsigned int *)(CPU_REGS_BASE + 0x24)

#define CPU_SEC_ENTRY_POINT             (unsigned int *)(0xFFFFFFFC)
#define CPU_REGS_IPI_BASE               (unsigned int *)(CPU_REGS_BASE + 0xC0)
#define REGS_CPU_IPI(cpu)               (unsigned int *)(CPU_REGS_IPI_BASE + (cpu * 4))
#define REG_CPU_RST_CTL                 (unsigned int *)(CPU_REGS_BASE + 0x14)
#define REG_CPU_HALT_CTL                (unsigned int *)(CPU_REGS_BASE + 0x18)

extern unsigned long eznps_he_version;

#endif /* __PLAT_MEMMAP_H */
