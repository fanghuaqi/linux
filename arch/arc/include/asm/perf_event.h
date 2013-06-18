/*
 * Linux performance counter support for ARC
 *
 * Copyright (C) 2011-2013 Synopsys, Inc. (www.synopsys.com)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#ifndef __ASM_PERF_EVENT_H
#define __ASM_PERF_EVENT_H

/* real maximum varies per CPU, this is the maximum supported by the driver */
#define ARC_PMU_MAX_HWEVENTS	64

#define ARC_REG_CC_BUILD	0xF6
#define ARC_REG_CC_INDEX	0x240
#define ARC_REG_CC_NAME0	0x241
#define ARC_REG_CC_NAME1	0x242

#define ARC_REG_PCT_BUILD	0xF5
#define ARC_REG_PCT_COUNTL	0x250
#define ARC_REG_PCT_COUNTH	0x251
#define ARC_REG_PCT_SNAPL	0x252
#define ARC_REG_PCT_SNAPH	0x253
#define ARC_REG_PCT_CONFIG	0x254
#define ARC_REG_PCT_CONTROL	0x255
#define ARC_REG_PCT_INDEX	0x256

#define ARC_REG_PCT_CONTROL_CC	(1 << 16)	/* clear counts */
#define ARC_REG_PCT_CONTROL_SN	(1 << 17)	/* snapshot */

struct arc_reg_pct_build {
#ifdef CONFIG_CPU_BIG_ENDIAN
	unsigned int m:8, c:8, r:6, s:2, v:8;
#else
	unsigned int v:8, s:2, r:6, c:8, m:8;
#endif
};

struct arc_reg_cc_build {
#ifdef CONFIG_CPU_BIG_ENDIAN
	unsigned int c:16, r:8, v:8;
#else
	unsigned int v:8, r:8, c:16;
#endif
};

/* The "generalized" performance events seem to really be a copy
   of the available events on x86 processors; the mapping to ARC
   events is not always possible 1-to-1. Fortunately, there doesn't
   seem to be an exact definition for these events, so we can cheat
   a bit where necessary */
static const char * const arc_pmu_ev_hw_map[] = {
	/* cycles not in halted state */
	[PERF_COUNT_HW_CPU_CYCLES] = "crun",
	/* reference cycles not in halted state; also "crun" as we don't
	   do Dynamic Voltage/Frequency Scaling (yet) */
	[PERF_COUNT_HW_REF_CPU_CYCLES] = "crun",
	/* Unclear what this means, Intel uses 0x013c, which according to
	   their datasheet means "unhalted reference cycles". Don't ask
	   me what the difference is with PERF_COUNT_HW_REF_CPU_CYCLES */
	[PERF_COUNT_HW_BUS_CYCLES] = "crun",
	[PERF_COUNT_HW_INSTRUCTIONS] = "iall",
	[PERF_COUNT_HW_BRANCH_MISSES] = "bpfail",
	[PERF_COUNT_HW_BRANCH_INSTRUCTIONS] = "ijmp",
	/* ARC can either measure stalls per pipeline stage, or all stalls
	   combined; for now we assign all stalls to STALLED_CYCLES_BACKEND
	   and all pipeline flushes (e.g. caused by mispredicts, etc.) to
	   STALLED_CYCLES_FRONTEND.

	   We could start multiple performance counters and combine everything
	   afterwards, but that makes it complicated, and users can always
	   use PERF_TYPE_RAW.

	   Note that I$ cache misses aren't counted by either of the two! */
	[PERF_COUNT_HW_STALLED_CYCLES_FRONTEND] = "bflush",
	[PERF_COUNT_HW_STALLED_CYCLES_BACKEND] = "bstall",
};

#endif /* __ASM_PERF_EVENT_H */
