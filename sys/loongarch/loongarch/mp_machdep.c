/*-
 * Copyright (c) 2015 The FreeBSD Foundation
 * Copyright (c) 2016 Ruslan Bukin <br@bsdpad.com>
 * All rights reserved.
 *
 * Portions of this software were developed by Andrew Turner under
 * sponsorship from the FreeBSD Foundation.
 *
 * Portions of this software were developed by SRI International and the
 * University of Cambridge Computer Laboratory under DARPA/AFRL contract
 * FA8750-10-C-0237 ("CTSRD"), as part of the DARPA CRASH research programme.
 *
 * Portions of this software were developed by the University of Cambridge
 * Computer Laboratory as part of the CTSRD Project, with support from the
 * UK Higher Education Innovation Fund (HEIF).
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

#include "opt_kstack_pages.h"
#include "opt_platform.h"
#include "opt_acpi.h"

#include <sys/cdefs.h>
#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/cpu.h>
#include <sys/cpuset.h>
#include <sys/kernel.h>
#include <sys/ktr.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/proc.h>
#include <sys/kdb.h>
#include <sys/sched.h>
#include <sys/smp.h>
#include <sys/sysctl.h>

#include <vm/vm.h>
#include <vm/pmap.h>
#include <vm/vm_extern.h>
#include <vm/vm_kern.h>
#include <vm/vm_map.h>

#include <machine/intr.h>
#include <machine/machdep.h>
#include <machine/cpufunc.h>
#include <machine/smp.h>
#include <machine/tlb.h>
#include <machine/vmparam.h>

#ifdef FDT
#include <dev/ofw/openfirm.h>
#include <dev/ofw/ofw_cpu.h>
#endif

#define	MP_BOOTSTACK_SIZE	(kstack_pages * PAGE_SIZE)

uint32_t __loongarch_boot_ap[1024];
static u_int cpu_hwid[MAXCPU];
static u_int cpu_acpi_id[MAXCPU];

static u_long smp_disabled_hwtids;
SYSCTL_ULONG(_machdep, OID_AUTO, smp_disabled_hwtids,
    CTLFLAG_RDTUN | CTLFLAG_NOFETCH, &smp_disabled_hwtids, 0,
    "Disabled hardware CPU IDs 0 through 63; the boot CPU is retained");

static int
sysctl_smp_hwtids(SYSCTL_HANDLER_ARGS)
{

	return (SYSCTL_OUT(req, cpu_hwid, mp_ncpus * sizeof(cpu_hwid[0])));
}
SYSCTL_PROC(_machdep, OID_AUTO, smp_hwtids,
    CTLTYPE_UINT | CTLFLAG_RD | CTLFLAG_MPSAFE, NULL, 0,
    sysctl_smp_hwtids, "IU", "Hardware CPU IDs in logical CPU order");

static int ipi_handler(void *);

extern uint32_t boot_cpu;
extern cpuset_t all_cpus;
extern void cpu_exception_handler(void);

void mpentry(u_long cpuid);
void init_secondary(uint32_t);

static struct mtx ap_boot_mtx;

/* Stacks for AP initialization, discarded once idle threads are started. */
void *bootstack;
static void *bootstacks[MAXCPU];

/* Count of started APs, used to synchronize access to bootstack. */
static volatile int aps_started;

/* Set to 1 once we're ready to let the APs out of the pen. */
static volatile int aps_ready;

/*
 * One synchronous request per initiating CPU.  A pending bit is cleared by
 * its target only after a full local invalidation.  No request retains a
 * pmap pointer, and no handler takes locks held by another initiator.
 */
static cpuset_t tlb_pending[MAXCPU];
static cpuset_t tlb_requesters;
static u_long tlb_shootdowns;
static u_long tlb_acknowledgements;
SYSCTL_ULONG(_machdep, OID_AUTO, tlb_shootdowns, CTLFLAG_RD,
    &tlb_shootdowns, 0, "Synchronous TLB shootdown requests");
SYSCTL_ULONG(_machdep, OID_AUTO, tlb_acknowledgements, CTLFLAG_RD,
    &tlb_acknowledgements, 0, "Remote TLB shootdown acknowledgements");

static void la_invlop_handler(void);

/* Temporary variables for init_secondary()  */
void *dpcpu[MAXCPU - 1];

static void
release_aps(void *dummy __unused)
{
	int i;

	if (mp_ncpus == 1)
		return;

	/* Setup the IPI handler */
	cpuic_setup_ipi(ipi_handler);

	/* Enable all IPI vectors in the IOCSR IPI controller. */
	iocsr_write32(0xffffffff, LOONGARCH_IOCSR_IPI_EN);

	atomic_store_rel_int(&aps_ready, 1);

	/*
	 * APs busy-wait on aps_ready in init_secondary(), so no IPI is needed
	 * to release them.  (They can't take an IPI yet anyway: mpentry leaves
	 * CRMD.IE=0 and their IPI isn't armed until later in init_secondary.)
	 */
	printf("Release APs\n");

	for (i = 0; i < 2000; i++) {
		if (atomic_load_acq_int(&smp_started))
			return;
		DELAY(1000);
	}

	panic("Secondary CPUs did not become ready");
}
SYSINIT(start_aps, SI_SUB_SMP, SI_ORDER_FIRST, release_aps, NULL);

void
init_secondary(uint32_t cpuid)
{
	struct pcpu *pcpup;
	u_int hwid;

	hwid = cpuid;
	for (cpuid = 0; cpuid < mp_ncpus; cpuid++) {
		if (cpu_hwid[cpuid] == hwid)
			break;
	}
	KASSERT(cpuid < mp_ncpus, ("Unknown hardware CPU %u", hwid));

	/* Setup the pcpu pointer */
	pcpup = &__pcpu[cpuid];
	__asm __volatile("move $r21, %0" :: "r"(pcpup));

	csr_write64((uint64_t)pcpup, PERCPU_BASE_KS);

	/* cpu identify */
	identify_cpu(cpuid);
	/* Configure per-CPU CSRs: ASID and page table walker. */
	asid_init(cpuid);
	pt_walker_init(cpuid);

	/* eentry */
	csr_write64((uint64_t)cpu_exception_handler, LOONGARCH_CSR_EENTRY);

	/* No printf can be called before while loop exit
		pc_curthread is NULL !!!
	 */

	/* Signal the BSP and spin until it has released all APs.  Busy-wait */
	atomic_add_rel_int(&aps_started, 1);
	while (!atomic_load_acq_int(&aps_ready))
		cpu_spinwait();

	/* Clear the IPI. */
	ipi_read_clear(cpuid);

	/* Initialize curthread */
	KASSERT(PCPU_GET(idlethread) != NULL, ("no idle thread"));
	pcpup->pc_curthread = pcpup->pc_idlethread;
	sched_init_ap();

	/*
	 * Enable IOCSR IPI delivery; ECFG unmasking of the IRQ_IPI line is
	 * done by cpuic_init_secondary() below (it enables any IRQ with a
	 * registered handler, and cpuic_setup_ipi registered the filter).
	 */
	iocsr_write32(0xffffffff, LOONGARCH_IOCSR_IPI_EN);

	/* Initialize per-CPU EIOINTC routing */
	eiointc_init_cpu();

	/*
	 * Initialize per-CPU interrupt state for the root PIC.
	 * This re-enables any interrupts that were set up on the BSP.
	 * Serialize the PIC's shared interrupt-source CPU masks.
	 */
	mtx_lock_spin(&ap_boot_mtx);
	intr_pic_init_secondary();
	mtx_unlock_spin(&ap_boot_mtx);

#ifndef EARLY_AP_STARTUP
	/* Start per-CPU event timers. */
	cpu_initclocks_ap();
#endif

	/* Activate this cpu in the kernel pmap. */
	CPU_SET_ATOMIC(cpuid, &kernel_pmap->pm_active);

	/* Activate process 0's pmap. */
	pmap_activate_boot(vmspace_pmap(proc0.p_vmspace));

	mtx_lock_spin(&ap_boot_mtx);

	atomic_add_rel_32(&smp_cpus, 1);

	if (smp_cpus == mp_ncpus) {
		/* enable IPI's, tlb shootdown, freezes etc */
		atomic_store_rel_int(&smp_started, 1);
	}

	mtx_unlock_spin(&ap_boot_mtx);

	if (bootverbose)
		printf("Secondary CPU %u fully online\n", cpuid);

	sched_ap_entry();

	panic("scheduler returned us to init_secondary");
	/* NOTREACHED */
}

static void
smp_after_idle_runnable(void *arg __unused)
{
	int cpu;

	if (mp_ncpus == 1)
		return;

	KASSERT(smp_started != 0, ("%s: SMP not started yet", __func__));

	/*
	 * Wait for all APs to handle an interrupt.  After that, we know that
	 * the APs have entered the scheduler at least once, so the boot stacks
	 * are safe to free.
	 */
	smp_rendezvous(smp_no_rendezvous_barrier, NULL,
	    smp_no_rendezvous_barrier, NULL);

	for (cpu = 1; cpu <= mp_maxid; cpu++) {
		if (bootstacks[cpu] != NULL)
			kmem_free(bootstacks[cpu], MP_BOOTSTACK_SIZE);
	}
}
SYSINIT(smp_after_idle_runnable, SI_SUB_SMP, SI_ORDER_ANY,
    smp_after_idle_runnable, NULL);

static void
la_tlb_pending_load(u_int cpu, cpuset_t *mask)
{
	u_int i;

	for (i = 0; i < nitems(mask->__bits); i++)
		mask->__bits[i] = atomic_load_acq_long(&tlb_pending[cpu].__bits[i]);
}

static void
la_invlop_handler(void)
{
	cpuset_t mask;
	u_int cpu, self;

	self = PCPU_GET(cpuid);
	CPU_FOREACH(cpu) {
		if (cpu == self)
			continue;
		la_tlb_pending_load(cpu, &mask);
		if (!CPU_ISSET(self, &mask))
			continue;
		invtlb_all(INVTLB_CURRENT_ALL);
		flush_icache();
		atomic_add_long(&tlb_acknowledgements, 1);
		atomic_thread_fence_rel();
		CPU_CLR_ATOMIC(self, &tlb_pending[cpu]);
	}
}

/* Spin-lock and rendezvous waiters must acknowledge TLB requests too. */
void
loongarch_cpu_spinwait(void)
{
	register_t intr;
	u_int i;

	__asm __volatile("" ::: "memory");
	if (!smp_started)
		return;
	for (i = 0; i < nitems(tlb_requesters.__bits); i++) {
		if (atomic_load_acq_long(&tlb_requesters.__bits[i]) != 0)
			break;
	}
	if (i == nitems(tlb_requesters.__bits))
		return;
	critical_enter();
	intr = intr_disable();
	la_invlop_handler();
	intr_restore(intr);
	critical_exit();
}

void
smp_invalidate_tlb(void)
{
	cpuset_t mask;
	register_t intr;
	u_int self;

	critical_enter();
	intr = intr_disable();
	if (kdb_active || KERNEL_PANICKED() || !smp_started) {
		invtlb_all(INVTLB_CURRENT_ALL);
		flush_icache();
		goto out;
	}
	self = PCPU_GET(cpuid);
	mask = all_cpus;
	CPU_CLR(self, &mask);
	/*
	 * Invalidate on every CPU, including inactive ASIDs.  This also
	 * covers a CPU switching into a pmap during a page-table update.
	 * Publish the page-table stores before making the request visible.
	 */
	CPU_COPY_STORE_REL(&mask, &tlb_pending[self]);
	CPU_SET_ATOMIC(self, &tlb_requesters);
	atomic_add_long(&tlb_shootdowns, 1);
	ipi_selected(mask, IPI_TLB);
	invtlb_all(INVTLB_CURRENT_ALL);
	flush_icache();
	for (;;) {
		la_tlb_pending_load(self, &mask);
		if (CPU_EMPTY(&mask))
			break;
		/*
		 * Service concurrent initiators even with interrupts disabled.
		 * Waiting for a hardware IPI here would deadlock CPUs that
		 * both entered with interrupts masked by a spin lock.
		 */
		cpu_spinwait();
	}
	CPU_CLR_ATOMIC(self, &tlb_requesters);
out:
	intr_restore(intr);
	critical_exit();
}

static int
ipi_handler(void *arg)
{
	u_int ipi_bitmap;
	u_int cpu, ipi;
	int bit;

	cpu = PCPU_GET(cpuid);

	for (;;) {
		mb();

		ipi_bitmap = atomic_readandclear_int(PCPU_PTR(pending_ipis));

		if (ipi_bitmap == 0) {
			/*
			 * Clear the hardware IPI status.  Without this
			 * the interrupt line stays asserted and the CPU
			 * would re-enter the handler immediately on ertn.
			 */
			ipi_read_clear(cpu);
			/*
			 * Re-check after clearing: an IPI may have arrived
			 * between the readandclear and the ipi_read_clear.
			 * Its hardware status was just cleared so the
			 * interrupt will not re-fire — we must process it
			 * here to avoid losing it.
			 */
			if (atomic_load_int(PCPU_PTR(pending_ipis)) == 0)
				return (FILTER_HANDLED);
			continue;
		}

		while ((bit = ffs(ipi_bitmap))) {
			bit = (bit - 1);
			ipi = (1 << bit);
			ipi_bitmap &= ~ipi;

			mb();

			switch (ipi) {
			case IPI_AST:
				CTR0(KTR_SMP, "IPI_AST");
				break;
			case IPI_PREEMPT:
				CTR1(KTR_SMP, "%s: IPI_PREEMPT", __func__);
				sched_preempt(curthread);
				break;
			case IPI_RENDEZVOUS:
				CTR0(KTR_SMP, "IPI_RENDEZVOUS");
				smp_rendezvous_action();
				break;
			case IPI_STOP:
			case IPI_STOP_HARD:
				CTR0(KTR_SMP, (ipi == IPI_STOP) ? "IPI_STOP" :
				    "IPI_STOP_HARD");
				savectx(&stoppcbs[cpu]);

				/* Indicate we are stopped */
				CPU_SET_ATOMIC(cpu, &stopped_cpus);

				/* Wait for restart */
				while (!CPU_ISSET(cpu, &started_cpus))
					cpu_spinwait();

				CPU_CLR_ATOMIC(cpu, &started_cpus);
				CPU_CLR_ATOMIC(cpu, &stopped_cpus);
				CTR0(KTR_SMP, "IPI_STOP (restart)");

				/*
				 * The kernel debugger might have set a
				 * breakpoint, so flush the instruction cache.
				 */
				flush_icache();
				break;
			case IPI_HARDCLOCK:
				CTR1(KTR_SMP, "%s: IPI_HARDCLOCK", __func__);
				hardclockintr();
				break;
			case IPI_TLB:
				CTR1(KTR_SMP, "%s: IPI_TLB", __func__);
				la_invlop_handler();
				break;
			default:
				panic("Unknown IPI %#0x on cpu %d",
				    ipi, curcpu);
			}
		}

		/*
		 * Clear hardware IPI status.  An IPI that arrived
		 * during processing set pc_pending_ipis but its
		 * hardware status is cleared here.  Re-check to avoid
		 * losing it.
		 */
		ipi_read_clear(cpu);

		if (atomic_load_int(PCPU_PTR(pending_ipis)) == 0)
			return (FILTER_HANDLED);
	}
}

struct cpu_group *
cpu_topo(void)
{

	return (smp_topo_none());
}

/* Determine if we running MP machine */
int
cpu_mp_probe(void)
{

	return (mp_ncpus > 1);
}

static bool
cpu_add(u_int hwid)
{
	u_int cpu;

	if (hwid >= nitems(__loongarch_boot_ap))
		return (true);
	if (hwid != boot_cpu && hwid < sizeof(smp_disabled_hwtids) * NBBY &&
	    (smp_disabled_hwtids & (1UL << hwid)) != 0)
		return (true);
	for (cpu = 0; cpu < mp_ncpus; cpu++) {
		if (cpu_hwid[cpu] == hwid)
			return (true);
	}
	if (mp_ncpus <= mp_maxid)
		cpu_hwid[mp_ncpus++] = hwid;
	return (true);
}

#ifdef DEV_ACPI
static bool
cpu_init_acpi(u_int hwid, u_int uid)
{
	u_int cpu;

	cpu_add(hwid);
	for (cpu = 0; cpu < mp_ncpus; cpu++) {
		if (cpu_hwid[cpu] == hwid) {
			cpu_acpi_id[cpu] = uid;
			break;
		}
	}
	return (true);
}
#endif

#ifdef FDT
static bool
cpu_init_fdt(u_int id __unused, phandle_t node __unused, u_int addr_size,
    pcell_t *reg)
{
	uint64_t hwid;

	if (addr_size != 1 && addr_size != 2)
		return (true);
	hwid = reg[0];
	if (addr_size == 2)
		hwid = (hwid << 32) | reg[1];
	if (hwid < nitems(__loongarch_boot_ap))
		cpu_add(hwid);
	return (true);
}
#endif

/* Initialize and fire up non-boot processors. */
void
cpu_mp_start(void)
{
	struct pcpu *pcpup;
	vm_paddr_t entry;
	u_int cpu, hwid;
	int naps, i;

	mtx_init(&ap_boot_mtx, "ap boot", NULL, MTX_SPIN);
	CPU_SET(0, &all_cpus);
	PCPU_SET(hwid, boot_cpu);
	PCPU_SET(acpi_id, cpu_acpi_id[0]);
	iocsr_write32(0xffffffff, LOONGARCH_IOCSR_IPI_EN);
	entry = pmap_extract(kernel_pmap, (vm_offset_t)mpentry);
	for (cpu = 1; cpu <= mp_maxid; cpu++) {
		hwid = cpu_hwid[cpu];
		pcpup = &__pcpu[cpu];
		pcpu_init(pcpup, cpu, sizeof(*pcpup));
		pcpup->pc_hwid = hwid;
		pcpup->pc_acpi_id = cpu_acpi_id[cpu];
		dpcpu[cpu - 1] = kmem_malloc(DPCPU_SIZE, M_WAITOK | M_ZERO);
		dpcpu_init(dpcpu[cpu - 1], cpu);
		bootstacks[cpu] = kmem_malloc(MP_BOOTSTACK_SIZE,
		    M_WAITOK | M_ZERO);
		bootstack = (char *)bootstacks[cpu] + MP_BOOTSTACK_SIZE;
		naps = atomic_load_acq_int(&aps_started);
		if (bootverbose)
			printf("Starting CPU %u (hardware ID %u)\n", cpu, hwid);
		/* Publish the stack and CPU mapping before the AP reads them. */
		atomic_store_rel_32(&__loongarch_boot_ap[hwid], 1);
		csr_mail_send(entry, hwid, 0);
		ipi_write_action(hwid, 1);
		for (i = 0; i < 5000; i++) {
			if (atomic_load_acq_int(&aps_started) != naps)
				break;
			DELAY(1000);
		}
		if (i == 5000)
			panic("CPU %u (hardware ID %u) did not start", cpu, hwid);
		CPU_SET(cpu, &all_cpus);
	}
}

/* Introduce rest of cores to the world */
void
cpu_mp_announce(void)
{
	u_int cpu;

	CPU_FOREACH(cpu) {
		/* Already announced. */
		if (cpu == 0)
			continue;

		printcpuinfo(cpu);
	}
}

void
cpu_mp_setmaxid(void)
{
	int limit, ncpu;

	limit = MAXCPU;
	TUNABLE_INT_FETCH("kern.maxcpus", &limit);
	limit = MAX(1, MIN(limit, MAXCPU));
	if (TUNABLE_INT_FETCH("hw.ncpu", &ncpu) && ncpu > 0)
		limit = MIN(limit, ncpu);
	mp_maxid = limit - 1;
	TUNABLE_ULONG_FETCH("machdep.smp_disabled_hwtids", &smp_disabled_hwtids);
	mp_ncpus = 1;
	for (ncpu = 0; ncpu < MAXCPU; ncpu++)
		cpu_acpi_id[ncpu] = 0xffffffff;
	cpu_hwid[0] = boot_cpu;
#ifdef DEV_ACPI
	if (loongarch_efi_acpi_rsdp() != 0 &&
	    loongarch_acpi_madt_cpu_count() > 0) {
		loongarch_acpi_madt_foreach_cpu(cpu_init_acpi);
	} else
#endif
#ifdef FDT
	{
		ofw_cpu_early_foreach(cpu_init_fdt, true);
	}
#else
	{
	}
#endif
	mp_maxid = mp_ncpus - 1;
	if (bootverbose)
		printf("SMP: %d CPUs selected, boot hardware ID %u\n",
		    mp_ncpus, boot_cpu);
}
