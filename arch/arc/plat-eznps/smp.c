/*******************************************************************************

  ARC700 Extensions for SMP
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

#include <linux/smp.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <asm/irq.h>
#include <asm/setup.h>
#include <plat/smp.h>
#include <plat/memmap.h>
#include <plat/ctop.h>

static char smp_cpuinfo_buf[128];

/* Initialize Number of Active Threads */
static void init_nat(int cpu)
{
	char *__cpu_active_map  = ACTIVE_CPUS_BASE;
	int core = (cpu >> 4) & 0xff;
	int nat = __cpu_active_map[core];
	int log_nat = 0;
	long tmp;

	switch (nat) {
	case 1:
		log_nat = 0;
		break;
	case 2:
		log_nat = 1;
		break;
	case 4:
		log_nat = 2;
		break;
	case 8:
		log_nat = 3;
		break;
	case 16:
		log_nat = 4;
		break;
	default:
		break;
	}

	tmp = read_aux_reg(CTOP_AUX_UDMC);
	tmp &= ~CTOP_UDMC_NAT_MASK;
	tmp |= log_nat << CTOP_UDMC_NAT_OFFSET;
	write_aux_reg(CTOP_AUX_UDMC,tmp);
}

/*
 * Any SMP specific init any CPU does when it comes up.
 * Here we setup the CPU to enable Inter-Processor-Interrupts
 * Called for each CPU
 * -Master      : init_IRQ()
 * -Other(s)    : start_kernel_secondary()
 */
void eznps_smp_init_cpu(unsigned int cpu)
{
	if (eznps_he_version == 1 || !running_on_hw) {
		unsigned int rc;

		rc = smp_ipi_irq_setup(cpu, IPI_IRQ);
		if (rc)
			panic("IPI IRQ %d reg failed on BOOT cpu\n", IPI_IRQ);

		if (!running_on_hw)
			init_nat(cpu);

	} else {
		int irq __maybe_unused;
		unsigned int tmp, rc;
		/* Attach the arch-common IPI ISR to our IPI IRQ */
		for (irq = 0; irq < num_possible_cpus(); irq++) {
			/* using edge */
			tmp = read_aux_reg(AUX_ITRIGGER);
			write_aux_reg(AUX_ITRIGGER,
					tmp | (1 << (IPI_IRQS_BASE + irq)));


			rc = smp_ipi_irq_setup(cpu, IPI_IRQS_BASE + irq);
			if (rc)
				panic("IPI IRQ %d reg failed on BOOT cpu\n",
					  IPI_IRQS_BASE + irq);
		}
	}
}

/*
 * Master kick starting another CPU
 */
static void eznps_smp_wakeup_cpu(int cpu, unsigned long pc)
{
	unsigned int halt_ctrl;

	/* override pc */
	pc = (unsigned long)ez_first_lines_of_secondary;

	/* setup the start PC */
	__raw_writel(pc, CPU_SEC_ENTRY_POINT);

	/* Take the cpu out of Halt */
	if (!running_on_hw) {
		halt_ctrl = 0;
	}
	else {
		halt_ctrl = __raw_readl(REG_CPU_HALT_CTL);
		halt_ctrl |= 1 << cpu;
	}
	__raw_writel(halt_ctrl, REG_CPU_HALT_CTL);
}

static void he1_eznps_smp_wakeup_cpu(int cpu, unsigned long pc)
{
	unsigned long cpu_cfg_value = 0;
	unsigned int *cpu_cfg_reg = REG_CPU_CONFIG(cpu);

	/* override pc */
	pc = (unsigned long)ez_first_lines_of_secondary;

	/* check pc alignment */
	if (pc & NPS_CPU_MEM_ADDRESS_MASK)
		panic("pc is not properly aligned:%lx", pc);

	/* set pc */
	cpu_cfg_value = pc;

	/* set dmsid */
	cpu_cfg_value |= NPS_DEFAULT_MSID << NPS_DEFAULT_MSID_SHIFT;

	/* cpu start */
	cpu_cfg_value |= 1;
	__raw_writel(cpu_cfg_value, cpu_cfg_reg);
}

static void eznps_ipi_send(void *p)
{
	const struct cpumask *callmap = p;
	unsigned int cpu, this_cpu = smp_processor_id();

	for_each_cpu(cpu, callmap) {
		__raw_writel((1 << cpu), REGS_CPU_IPI(this_cpu));
		__raw_writel(0, REGS_CPU_IPI(this_cpu));
	}
}

static void he1_eznps_ipi_send(void *p)
{
	const struct cpumask *callmap = p;
	unsigned int cpu, reg_value = 0;
	for_each_cpu(cpu, callmap) {
		reg_value  = (NPS_CPU_TO_THREAD_NUM(cpu));
		reg_value |= (NPS_CPU_TO_CORE_NUM(cpu) << 8);
		reg_value |= (NPS_CPU_TO_CLUSTER(cpu) << 16);
		reg_value |= (IPI_IRQ << 24);

		__asm__ __volatile__(
				"mov r3, %0\n"
				".word 0x3b56003e #asri 0, r3\n"
				:
				: "r"(reg_value)
				: "r3"
		);
	}
}

static void eznps_ipi_clear(int cpu, int irq)
{
	write_aux_reg(AUX_IPULSE, (1 << irq));
}

static void eznps_map_cpus(int max_cpus)
{
	int i;
	char *__cpu_simulated_map  = SIMULATED_CPUS_BASE;
	int thread, core;

        for (i = 0; i < NR_CPUS; i++) {
		thread = i & 0xf;
		core = (i >> 4) & 0xff;
		if (thread < __cpu_simulated_map[core]) {
			if (max_cpus)
				set_cpu_present(i, true);
			else
				set_cpu_possible(i, true);
		}
	}
}

void eznps_init_early_smp(void)
{
	sprintf(smp_cpuinfo_buf, "Extn [EZNPS-SMP]\t: On\n");
	plat_smp_ops.info = smp_cpuinfo_buf;
	if (!running_on_hw) {
		plat_smp_ops.cpu_kick = eznps_smp_wakeup_cpu;
		plat_smp_ops.map_cpus = eznps_map_cpus;
		plat_smp_ops.ipi_send = he1_eznps_ipi_send;
		plat_smp_ops.ipi_clear = NULL;
	} else if (eznps_he_version == 0) {
		plat_smp_ops.cpu_kick = eznps_smp_wakeup_cpu;
		plat_smp_ops.map_cpus = eznps_map_cpus;
		plat_smp_ops.ipi_send = eznps_ipi_send;
		plat_smp_ops.ipi_clear = eznps_ipi_clear;
	} else {
		plat_smp_ops.cpu_kick = he1_eznps_smp_wakeup_cpu;
		plat_smp_ops.map_cpus = NULL;
		plat_smp_ops.ipi_send = he1_eznps_ipi_send;
		plat_smp_ops.ipi_clear = NULL;
	}
}
