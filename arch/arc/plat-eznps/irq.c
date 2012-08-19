/*******************************************************************************

  EZNPS Platform IRQ hookups
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

#include <linux/interrupt.h>
#include <linux/of.h>
#include <linux/irqdomain.h>
#include "../../drivers/irqchip/irqchip.h"
#include <asm/setup.h>
#include <plat/irq.h>
#include <plat/memmap.h>
#include <plat/ctop.h>
#ifdef CONFIG_SMP
#include <plat/smp.h>
#endif

static void arc_irq_mask(struct irq_data *data)
{
	arch_mask_irq(data->irq);
}

static void arc_irq_unmask(struct irq_data *data)
{
	arch_unmask_irq(data->irq);
}

static void arc_irq_ack(struct irq_data *data)
{
	write_aux_reg(CTOP_AUX_IACK, 1 << data->irq);
}

#ifndef CONFIG_SMP
static void arc_irq_mask_ack(struct irq_data *data)
{
	arch_mask_irq(data->irq);
	arc_irq_ack(data);
}
#endif /* CONFIG_SMP */

#ifdef CONFIG_SMP
static unsigned int arc_irq_startup(struct irq_data *data)
{
	smp_send_ienable(data->irq, true);

	return 0;
}

static void arc_irq_shutdown(struct irq_data *data)
{
	smp_send_ienable(data->irq, false);
}
#endif

static struct irq_chip extchip_intc = {
	.name		= "ARC Ext-core Intc",
	.irq_mask	= arc_irq_mask,
	.irq_unmask	= arc_irq_unmask,
#ifdef CONFIG_SMP
	.irq_startup	= arc_irq_startup,
	.irq_shutdown	= arc_irq_shutdown,
#endif /* CONFIG_SMP */

/* This is the addition for the external interrupt controller */
#ifdef CONFIG_SMP
	.irq_ack	= arc_irq_ack,
#else
	.irq_mask_ack	= arc_irq_mask_ack,
#endif /* CONFIG_SMP */
};

static struct irq_chip onchip_intc = {
	.name		= "ARC In-core Intc",
	.irq_mask	= arc_irq_mask,
	.irq_unmask	= arc_irq_unmask,
#ifdef CONFIG_SMP
	.irq_startup	= arc_irq_startup,
	.irq_shutdown	= arc_irq_shutdown,
#endif /* CONFIG_SMP */
};

static int arc_intc_domain_map(struct irq_domain *d, unsigned int irq,
                                irq_hw_number_t hw)
{
	switch (irq) {
		case TIMER0_IRQ:
			if (eznps_he_version == 1)
				irq_set_chip_and_handler(irq,
					&extchip_intc, handle_percpu_irq);
			else
				irq_set_chip_and_handler(irq,
					&onchip_intc, handle_percpu_irq);
		break;
#ifdef CONFIG_SMP
		case IPI_IRQ:
			if (eznps_he_version == 1 || !running_on_hw)
				irq_set_chip_and_handler(irq,
					&extchip_intc, handle_percpu_irq);
			else
				irq_set_chip_and_handler(irq,
					&onchip_intc, handle_level_irq);
		break;
		case IPI_IRQS_BASE:
		case IPI_IRQS_BASE + 1:
		case IPI_IRQS_BASE + 2:
		case IPI_IRQS_BASE + 3:
			if (eznps_he_version == 0)
				irq_set_chip_and_handler(irq,
					&onchip_intc, handle_percpu_irq);
			else
				irq_set_chip_and_handler(irq,
					&onchip_intc, handle_level_irq);
		break;
#endif /* CONFIG_SMP */
		default:
			if (eznps_he_version == 1)
				irq_set_chip_and_handler(irq,
					&extchip_intc, handle_level_irq);
			else
				irq_set_chip_and_handler(irq,
					&onchip_intc, handle_level_irq);
		break;
	}

        return 0;
}

static const struct irq_domain_ops arc_intc_domain_ops = {
        .xlate = irq_domain_xlate_onecell,
        .map = arc_intc_domain_map,
};

static struct irq_domain *root_domain;

static int __init
init_onchip_IRQ(struct device_node *intc, struct device_node *parent)
{
        if (parent)
                panic("DeviceTree incore intc not a root irq controller\n");

        root_domain = irq_domain_add_legacy(intc, NR_CPU_IRQS, 0, 0,
                                            &arc_intc_domain_ops, NULL);

        if (!root_domain)
                panic("root irq domain not avail\n");

        /* with this we don't need to export root_domain */
        irq_set_default_host(root_domain);

        return 0;
}

IRQCHIP_DECLARE(arc_intc, "eznps,arc700-intc", init_onchip_IRQ);
