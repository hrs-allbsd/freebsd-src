/*-
 * Copyright (c) 2015 Ruslan Bukin <br@bsdpad.com>
 * All rights reserved.
 *
 * This software was developed by the University of Cambridge Computer
 * Laboratory with support from ARM Ltd.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#ifndef _DEV_HWPMC_LOONGARCH_H_
#define	_DEV_HWPMC_LOONGARCH_H_

#define	LOONGARCH_PMC_CAPS	(PMC_CAP_INTERRUPT | PMC_CAP_USER | \
    PMC_CAP_SYSTEM | PMC_CAP_READ | PMC_CAP_WRITE)

/* LoongArch v1, PMCFG: privilege enables, interrupt enable, 10-bit event. */
#define	LOONGARCH_PMC_EVENT_MASK	0x3ffU
#define	LOONGARCH_PMC_PLV0	(1U << 16)
#define	LOONGARCH_PMC_PLV3	(1U << 19)
#define	LOONGARCH_PMC_IE		(1U << 20)
#define	LOONGARCH_PMC_OVERFLOW	(1ULL << 63)
#define	LOONGARCH_PMC_MAX_PERIOD	(LOONGARCH_PMC_OVERFLOW - 1)
#define	LOONGARCH_PMC_MAX	16

/* PCOV is level asserted while PMCNT[63] == 1, not on 64-bit wrap. */
#define	LOONGARCH_RELOAD_COUNT_TO_PERFCTR_VALUE(R) \
    (LOONGARCH_PMC_OVERFLOW - (R))
#define	LOONGARCH_PERFCTR_VALUE_TO_RELOAD_COUNT(P) \
    (((P) & LOONGARCH_PMC_OVERFLOW) ? 0 : (LOONGARCH_PMC_OVERFLOW - (P)))

#ifdef _KERNEL
/* MD extension for 'struct pmc' */
struct pmc_md_loongarch_pmc {
	uint32_t	pm_loongarch_evsel;
};
#endif /* _KERNEL */
#endif /* _DEV_HWPMC_LOONGARCH_H_ */
