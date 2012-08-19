/*******************************************************************************

  EZNPS Platform time
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

#include <linux/clocksource.h>
#include <asm/arcregs.h>
#include <asm/clk.h>
#include <linux/smp.h>
#include <asm/setup.h>
#include <plat/smp.h>
#include <plat/memmap.h>

static unsigned long long rtc64(unsigned int cpu)
{
	union RTC_64 rtc_64;
	unsigned long cmp_value;

	/* get correct clk value */
	do {
		/* MSW */
		rtc_64.clk_value[0] = __raw_readl(REG_CPU_RTC_HI(cpu));
		/* LSW */
		rtc_64.clk_value[1] = __raw_readl(REG_CPU_RTC_LO(cpu));
		cmp_value = __raw_readl(REG_CPU_RTC_HI(cpu));
	} while (rtc_64.clk_value[0] != cmp_value);

	return rtc_64.ret_value;
}

static cycle_t he1_eznps_counter_read(struct clocksource *cs)
{
	return (cycle_t) rtc64(smp_processor_id());
}
static cycle_t sim_eznps_counter_read(struct clocksource *cs)
{
	return (cycle_t) (*CLOCK_SOURCE_BASE);
}

static cycle_t he0_eznps_counter_read(struct clocksource *cs)
{
	unsigned long cyc;

	 __asm__ __volatile__("rtsc %0,0 \r\n" : "=r" (cyc));

	return (cycle_t) cyc;
}

static struct clocksource eznps_counter = {
	.name   = "EZNPS Counter",
	.rating = 301,
	.mask   = CLOCKSOURCE_MASK(32),
	.flags  = CLOCK_SOURCE_IS_CONTINUOUS,
};

void eznps_counter_setup(void)
{
	if (!running_on_hw)
		eznps_counter.read = sim_eznps_counter_read;
	else {
		if (eznps_he_version == 1) {
			eznps_counter.read = he1_eznps_counter_read;
			eznps_counter.mask = CLOCKSOURCE_MASK(64);
		}
		else
			eznps_counter.read = he0_eznps_counter_read;
	}

	clocksource_register_hz(&eznps_counter, arc_get_core_freq());
}
