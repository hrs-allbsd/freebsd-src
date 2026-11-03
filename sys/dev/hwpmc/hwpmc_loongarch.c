/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 */

/* LoongArch PMU v1. See pmc.loongarch(3) for the supported interface. */
#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/pmc.h>
#include <sys/pmckern.h>
#include <machine/cpu.h>
#include <machine/cpufunc.h>
#include <machine/frame.h>
#include <machine/intr.h>
#include <machine/pmc_mdep.h>

static struct pmc_hw **la_pcpu;
static uint32_t la_cpucfg;
static unsigned int la_npmc;

static void
la_pcov_mask(bool enable)
{
	uint32_t val, mask;

	mask = 1U << IRQ_PCOV;
	val = enable ? mask : 1;
	__asm __volatile("csrxchg %0, %1, 0x4" : "+r" (val) :
	    "r" (mask) : "memory");
}

/* CSR numbers are instruction immediates: never use a variable CSR operand. */
#define LA_ROWS(M) \
    M(0) M(1) M(2) M(3) M(4) M(5) M(6) M(7) \
    M(8) M(9) M(10) M(11) M(12) M(13) M(14) M(15)

static uint64_t
la_read_counter(unsigned int ri)
{

	switch (ri) {
#define R(n) case n: return (csr_read64(0x201 + 2 * n));
	LA_ROWS(R)
#undef R
	default: panic("hwpmc: bad LoongArch counter %u", ri);
	}
}

static void
la_write_counter(unsigned int ri, uint64_t v)
{

	switch (ri) {
#define W(n) case n: csr_write64(v, 0x201 + 2 * n); break;
	LA_ROWS(W)
#undef W
	default: panic("hwpmc: bad LoongArch counter %u", ri);
	}
}

static uint32_t
la_read_control(unsigned int ri)
{

	switch (ri) {
#define R(n) case n: return (csr_read32(0x200 + 2 * n));
	LA_ROWS(R)
#undef R
	default: panic("hwpmc: bad LoongArch control %u", ri);
	}
}

static void
la_write_control(unsigned int ri, uint32_t v)
{

	switch (ri) {
#define W(n) case n: csr_write32(v, 0x200 + 2 * n); break;
	LA_ROWS(W)
#undef W
	default: panic("hwpmc: bad LoongArch control %u", ri);
	}
}

/* Named events use the Loongson event map; raw event codes remain explicit. */
static const uint16_t la_events[] = {
	0x00, 0x01, 0x02, 0x03, 0x06, 0x07, 0x08, 0x09,
	0x0c, 0x0d, 0x3b, 0x3c
};

static int
la_allocate(int cpu __unused, int ri __unused, struct pmc *pm,
    const struct pmc_op_pmcallocate *a)
{
	uint64_t event;
	uint32_t config, priv;

	if (a->pm_class != PMC_CLASS_LOONGARCH)
		return (EINVAL);
	if ((a->pm_flags & PMC_F_EV_PMU) != 0) {
		event = a->pm_md.pm_md_config;
		if (event > LOONGARCH_PMC_EVENT_MASK)
			return (EINVAL);
	} else {
		if (a->pm_ev < PMC_EV_LOONGARCH_FIRST ||
		    a->pm_ev >= PMC_EV_LOONGARCH_RAW)
			return (EINVAL);
		event = la_events[a->pm_ev - PMC_EV_LOONGARCH_FIRST];
	}
	if (PMC_IS_SAMPLING_MODE(PMC_TO_MODE(pm)) &&
	    pm->pm_sc.pm_reloadcount > LOONGARCH_PMC_MAX_PERIOD)
		return (EINVAL);
	config = event;
	priv = a->pm_caps & (PMC_CAP_USER | PMC_CAP_SYSTEM);
	if (priv == 0 || (priv & PMC_CAP_SYSTEM) != 0)
		config |= LOONGARCH_PMC_PLV0;
	if (priv == 0 || (priv & PMC_CAP_USER) != 0)
		config |= LOONGARCH_PMC_PLV3;
	if (PMC_IS_SAMPLING_MODE(PMC_TO_MODE(pm)))
		config |= LOONGARCH_PMC_IE;
	pm->pm_md.pm_loongarch.pm_loongarch_evsel = config;
	return (0);
}

static int
la_read(int cpu __unused, int ri, struct pmc *pm, pmc_value_t *v)
{
	uint64_t raw;

	raw = la_read_counter(ri);
	*v = PMC_IS_SAMPLING_MODE(PMC_TO_MODE(pm)) ?
	    LOONGARCH_PERFCTR_VALUE_TO_RELOAD_COUNT(raw) : raw;
	return (0);
}

static int
la_write(int cpu __unused, int ri, struct pmc *pm, pmc_value_t v)
{

	if (PMC_IS_SAMPLING_MODE(PMC_TO_MODE(pm))) {
		if (v == 0 || v > LOONGARCH_PMC_MAX_PERIOD)
			return (EINVAL);
		v = LOONGARCH_RELOAD_COUNT_TO_PERFCTR_VALUE(v);
	}
	la_write_counter(ri, v);
	return (0);
}

static int
la_config(int cpu, int ri, struct pmc *pm)
{
	struct pmc_hw *phw;

	phw = &la_pcpu[cpu][ri];
	KASSERT(pm == NULL || phw->phw_pmc == NULL,
	    ("hwpmc: LoongArch counter already configured"));
	phw->phw_pmc = pm;
	return (0);
}

static int
la_start(int cpu __unused, int ri, struct pmc *pm)
{

	la_write_control(ri, pm->pm_md.pm_loongarch.pm_loongarch_evsel);
	return (0);
}

static int
la_stop(int cpu __unused, int ri, struct pmc *pm __unused)
{

	/* Disables privilege counting and PMIEn together. */
	la_write_control(ri, 0);
	return (0);
}

static int
la_release(int cpu, int ri, struct pmc *pm __unused)
{

	KASSERT(la_pcpu[cpu][ri].phw_pmc == NULL,
	    ("hwpmc: releasing a configured LoongArch counter"));
	return (0);
}

static int
la_intr(struct trapframe *tf)
{
	struct pmc *pm;
	uint64_t raw;
	uint32_t config;
	int cpu, handled, error;
	unsigned int ri;

	cpu = curcpu;
	handled = 0;
	if (la_pcpu[cpu] == NULL)
		goto done;
	for (ri = 0; ri < la_npmc; ri++) {
		config = la_read_control(ri);
		if ((config & LOONGARCH_PMC_IE) == 0)
			continue;
		raw = la_read_counter(ri);
		if ((raw & LOONGARCH_PMC_OVERFLOW) == 0)
			continue;
		handled = 1;
		/* Quiesce before reading the owner or calling the MI sampler. */
		la_write_control(ri, 0);
		la_write_counter(ri, raw & LOONGARCH_PMC_MAX_PERIOD);
		pm = la_pcpu[cpu][ri].phw_pmc;
		if (pm == NULL || pm->pm_state != PMC_STATE_RUNNING ||
		    !PMC_IS_SAMPLING_MODE(PMC_TO_MODE(pm)))
			continue;
		error = pmc_process_interrupt(PMC_HR, pm, tf);
		if (la_write(cpu, ri, pm, pm->pm_sc.pm_reloadcount) != 0)
			continue;
		/* The MI layer marks a full sample buffer stalled and restarts it. */
		if (error == 0)
			la_start(cpu, ri, pm);
	}
done:
	if (handled)
		counter_u64_add(pmc_stats.pm_intr_processed, 1);
	else
		counter_u64_add(pmc_stats.pm_intr_ignored, 1);
	return (handled);
}

static int
la_describe(int cpu, int ri, struct pmc_info *pi, struct pmc **ppm)
{
	struct pmc_hw *phw;

	phw = &la_pcpu[cpu][ri];
	snprintf(pi->pm_name, sizeof(pi->pm_name), "LOONGARCH-%d", ri);
	pi->pm_class = PMC_CLASS_LOONGARCH;
	pi->pm_enabled = (phw->phw_state & PMC_PHW_FLAG_IS_ENABLED) != 0;
	*ppm = pi->pm_enabled ? phw->phw_pmc : NULL;
	return (0);
}

static int
la_get_config(int cpu, int ri, struct pmc **ppm)
{

	*ppm = la_pcpu[cpu][ri].phw_pmc;
	return (0);
}

static int
la_pcpu_init(struct pmc_mdep *md, int cpu)
{
	struct pmc_hw *phw;
	unsigned int ri;
	int base;

	/* Never touch unimplemented CSRs on a heterogeneous or emulated CPU. */
	if ((read_cpucfg(LOONGARCH_CPUCFG6) & 0x3fff) != la_cpucfg)
		return (ENXIO);
	la_pcpu[cpu] = malloc(la_npmc * sizeof(*phw), M_PMC,
	    M_WAITOK | M_ZERO);
	base = md->pmd_classdep[PMC_MDEP_CLASS_INDEX_LOONGARCH].pcd_ri;
	for (ri = 0; ri < la_npmc; ri++) {
		la_write_control(ri, 0);
		la_write_counter(ri, 0);
		phw = &la_pcpu[cpu][ri];
		phw->phw_state = PMC_PHW_FLAG_IS_ENABLED |
		    PMC_PHW_CPU_TO_STATE(cpu) | PMC_PHW_INDEX_TO_STATE(ri);
		pmc_pcpu[cpu]->pc_hwpmcs[base + ri] = phw;
	}
	/* IRQ_PCOV dispatch is supplied by the kernel's CPU-local controller. */
	la_pcov_mask(true);
	return (0);
}

static int
la_pcpu_fini(struct pmc_mdep *md __unused, int cpu)
{
	unsigned int ri;

	if (la_pcpu[cpu] == NULL)
		return (0);
	la_pcov_mask(false);
	for (ri = 0; ri < la_npmc; ri++) {
		la_write_control(ri, 0);
		la_write_counter(ri, 0);
	}
	free(la_pcpu[cpu], M_PMC);
	la_pcpu[cpu] = NULL;
	return (0);
}

struct pmc_mdep *
pmc_md_initialize(void)
{
	struct pmc_binding pb;
	struct pmc_mdep *md;
	struct pmc_classdep *pcd;
	uint32_t cfg;
	int cpu;
	bool compatible;

	cfg = read_cpucfg(LOONGARCH_CPUCFG6) & 0x3fff;
	/* v1, 64-bit counters only; an absent PMU gets MI software PMCs. */
	if ((cfg & CPUCFG6_PMP) == 0 || ((cfg >> 1) & 7) != 1 ||
	    ((cfg >> 8) & 63) != 63)
		return (NULL);
	/* Check every active CPU before allocating state or touching any PMC. */
	compatible = true;
	pmc_save_cpu_binding(&pb);
	for (cpu = 0; cpu < pmc_cpu_max(); cpu++) {
		if (!pmc_cpu_is_active(cpu))
			continue;
		pmc_select_cpu(cpu);
		if ((read_cpucfg(LOONGARCH_CPUCFG6) & 0x3fff) != cfg) {
			compatible = false;
			break;
		}
	}
	pmc_restore_cpu_binding(&pb);
	if (!compatible)
		return (NULL);
	la_cpucfg = cfg;
	la_npmc = ((cfg >> 4) & 15) + 1;
	la_pcpu = malloc(pmc_cpu_max() * sizeof(*la_pcpu), M_PMC,
	    M_WAITOK | M_ZERO);
	md = pmc_mdep_alloc(1);
	md->pmd_cputype = PMC_CPU_LOONGARCH;
	pcd = &md->pmd_classdep[PMC_MDEP_CLASS_INDEX_LOONGARCH];
	pcd->pcd_caps = LOONGARCH_PMC_CAPS;
	pcd->pcd_class = PMC_CLASS_LOONGARCH;
	pcd->pcd_num = la_npmc;
	pcd->pcd_ri = md->pmd_npmc;
	pcd->pcd_width = 64;
	pcd->pcd_allocate_pmc = la_allocate;
	pcd->pcd_config_pmc = la_config;
	pcd->pcd_pcpu_init = la_pcpu_init;
	pcd->pcd_pcpu_fini = la_pcpu_fini;
	pcd->pcd_describe = la_describe;
	pcd->pcd_get_config = la_get_config;
	pcd->pcd_read_pmc = la_read;
	pcd->pcd_release_pmc = la_release;
	pcd->pcd_start_pmc = la_start;
	pcd->pcd_stop_pmc = la_stop;
	pcd->pcd_write_pmc = la_write;
	md->pmd_intr = la_intr;
	md->pmd_npmc += la_npmc;
	snprintf(pmc_cpuid, sizeof(pmc_cpuid), "loongarch-%08x",
	    (uint32_t)read_cpucfg(LOONGARCH_CPUCFG0));
	return (md);
}

void
pmc_md_finalize(struct pmc_mdep *md __unused)
{

	free(la_pcpu, M_PMC);
	la_pcpu = NULL;
}

/* Report the interrupted instruction; do not invent a frame-pointer ABI. */
int
pmc_save_kernel_callchain(uintptr_t *cc, int maxsamples, struct trapframe *tf)
{

	if (maxsamples <= 0)
		return (0);
	cc[0] = tf->tf_era;
	return (1);
}

int
pmc_save_user_callchain(uintptr_t *cc, int maxsamples, struct trapframe *tf)
{

	if (maxsamples <= 0)
		return (0);
	cc[0] = tf->tf_era;
	return (1);
}
