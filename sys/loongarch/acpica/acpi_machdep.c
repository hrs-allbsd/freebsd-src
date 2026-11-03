/*-
 * Copyright (c) 2026 Hiroki Sato <hrs@FreeBSD.org>
 * Copyright (c) 2001 Mitsuru IWASAKI
 * Copyright (c) 2015 The FreeBSD Foundation
 * All rights reserved.
 *
 * This software was developed by Andrew Turner under
 * sponsorship from the FreeBSD Foundation.
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

#include <sys/param.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/pcpu.h>
#include <machine/cpufunc.h>
#include <machine/loongarchreg.h>

#include <vm/vm.h>
#include <vm/pmap.h>

#include <machine/machdep.h>

#include <contrib/dev/acpica/include/acpi.h>
#include <contrib/dev/acpica/include/accommon.h>
#include <contrib/dev/acpica/include/actables.h>

#include <dev/acpica/acpivar.h>

extern struct bus_space memmap_bus;
int (*apei_nmi)(void);

static void acpi_set_boot_cpu_id(void);

int
acpi_machdep_init(device_t dev)
{

	acpi_set_boot_cpu_id();
	return (0);
}

int
acpi_machdep_quirks(int *quirks)
{

	return (0);
}

static void *
map_table(vm_paddr_t pa, const char *sig)
{
	ACPI_TABLE_HEADER *header;
	vm_size_t length;
	void *table;

	header = pmap_mapbios(pa, sizeof(ACPI_TABLE_HEADER));
	if (header == NULL)
		return (NULL);
	if (strncmp(header->Signature, sig, ACPI_NAMESEG_SIZE) != 0) {
		pmap_unmapbios(header, sizeof(ACPI_TABLE_HEADER));
		return (NULL);
	}
	length = header->Length;
	pmap_unmapbios(header, sizeof(ACPI_TABLE_HEADER));

	if (length < sizeof(ACPI_TABLE_HEADER) || length > 1024 * 1024)
		return (NULL);
	table = pmap_mapbios(pa, length);
	if (table == NULL)
		return (NULL);
	if (AcpiUtChecksum(table, length) != 0) {
		if (bootverbose)
			printf("ACPI: Failed checksum for table %s\n", sig);
		pmap_unmapbios(table, length);
		return (NULL);
	}
	return (table);
}

/*
 * See if a given ACPI table is the requested table.  Returns the
 * length of the table if it matches or zero on failure.
 */
static int
probe_table(vm_paddr_t address, const char *sig)
{
	ACPI_TABLE_HEADER *table;

	table = pmap_mapbios(address, sizeof(ACPI_TABLE_HEADER));
	if (table == NULL) {
		if (bootverbose)
			printf("ACPI: Failed to map table at 0x%jx\n",
			    (uintmax_t)address);
		return (0);
	}

	if (strncmp(table->Signature, sig, ACPI_NAMESEG_SIZE) != 0) {
		pmap_unmapbios(table, sizeof(ACPI_TABLE_HEADER));
		return (0);
	}
	pmap_unmapbios(table, sizeof(ACPI_TABLE_HEADER));
	return (1);
}

/* Unmap a table previously mapped via acpi_map_table(). */
void
acpi_unmap_table(void *table)
{
	ACPI_TABLE_HEADER *header;

	header = (ACPI_TABLE_HEADER *)table;
	pmap_unmapbios(table, header->Length);
}

/*
 * Try to map a table at a given physical address previously returned
 * by acpi_find_table().
 */
void *
acpi_map_table(vm_paddr_t pa, const char *sig)
{

	return (map_table(pa, sig));
}

/*
 * Return the physical address of the requested table or zero if one
 * is not found.
 */
vm_paddr_t
acpi_find_table(const char *sig)
{
	ACPI_PHYSICAL_ADDRESS rsdp_ptr;
	ACPI_TABLE_RSDP *rsdp;
	ACPI_TABLE_XSDT *xsdt;
	ACPI_TABLE_HEADER *table;
	vm_paddr_t addr;
	int i, count;

	if (resource_disabled("acpi", 0))
		return (0);

	/*
	 * Map in the RSDP.  Since ACPI uses AcpiOsMapMemory() which in turn
	 * calls pmap_mapbios() to find the RSDP, we assume that we can use
	 * pmap_mapbios() to map the RSDP.
	 */
	if ((rsdp_ptr = AcpiOsGetRootPointer()) == 0)
		return (0);
	rsdp = pmap_mapbios(rsdp_ptr, sizeof(ACPI_TABLE_RSDP));
	if (rsdp == NULL) {
		printf("ACPI: Failed to map RSDP\n");
		return (0);
	}

	if (memcmp(rsdp->Signature, ACPI_SIG_RSDP, 8) != 0 ||
	    AcpiUtChecksum((UINT8 *)rsdp, ACPI_RSDP_CHECKSUM_LENGTH) != 0) {
		pmap_unmapbios(rsdp, sizeof(*rsdp));
		return (0);
	}
	addr = 0;
	if (rsdp->Revision >= 2 && rsdp->XsdtPhysicalAddress != 0) {
		/*
		 * AcpiOsGetRootPointer only verifies the checksum for
		 * the version 1.0 portion of the RSDP.  Version 2.0 has
		 * an additional checksum that we verify first.
		 */
		if (AcpiUtChecksum((UINT8 *)rsdp, ACPI_RSDP_XCHECKSUM_LENGTH)) {
			printf("ACPI: RSDP failed extended checksum\n");
			pmap_unmapbios(rsdp, sizeof(ACPI_TABLE_RSDP));
			return (0);
		}
		xsdt = map_table(rsdp->XsdtPhysicalAddress, ACPI_SIG_XSDT);
		if (xsdt == NULL) {
			printf("ACPI: Failed to map XSDT\n");
			pmap_unmapbios(rsdp, sizeof(ACPI_TABLE_RSDP));
			return (0);
		}
		count = (xsdt->Header.Length - sizeof(ACPI_TABLE_HEADER)) /
		    sizeof(UINT64);
		for (i = 0; i < count; i++)
			if (probe_table(xsdt->TableOffsetEntry[i], sig)) {
				addr = xsdt->TableOffsetEntry[i];
				break;
			}
		acpi_unmap_table(xsdt);
	} else {
		printf("ACPI: Unsupported RSDP version %d and XSDT %#lx\n",
		    rsdp->Revision, rsdp->XsdtPhysicalAddress);
	}
	pmap_unmapbios(rsdp, sizeof(ACPI_TABLE_RSDP));

	if (addr == 0)
		return (0);

	/*
	 * Verify that we can map the full table and that its checksum is
	 * correct, etc.
	 */
	table = map_table(addr, sig);
	if (table == NULL)
		return (0);
	acpi_unmap_table(table);

	return (addr);
}

int
acpi_map_addr(struct acpi_generic_address *addr, bus_space_tag_t *tag,
    bus_space_handle_t *handle, bus_size_t size)
{
	bus_addr_t phys;

	/* Check if the device is Memory mapped */
	if (addr->SpaceId != 0)
		return (ENXIO);

	phys = addr->Address;
	*tag = &memmap_bus;

	return (bus_space_map(*tag, phys, size, 0, handle));
}


/* Walk only validated, bounded MADT subtables, including before ACPICA attach. */
static ACPI_TABLE_MADT *
loongarch_madt(void)
{
	ACPI_TABLE_MADT *madt;
	vm_paddr_t pa;

	pa = acpi_find_table(ACPI_SIG_MADT);
	if (pa == 0)
		return (NULL);
	madt = acpi_map_table(pa, ACPI_SIG_MADT);
	if (madt != NULL && madt->Header.Length < sizeof(*madt)) {
		acpi_unmap_table(madt);
		return (NULL);
	}
	return (madt);
}

static ACPI_SUBTABLE_HEADER *
madt_next(ACPI_TABLE_MADT *madt, ACPI_SUBTABLE_HEADER *entry)
{
	char *p, *end;

	p = entry == NULL ? (char *)(madt + 1) :
	    (char *)entry + entry->Length;
	end = (char *)madt + madt->Header.Length;
	if (p > end || (size_t)(end - p) < sizeof(*entry))
		return (NULL);
	entry = (ACPI_SUBTABLE_HEADER *)p;
	if (entry->Length < sizeof(*entry) || entry->Length > end - p)
		return (NULL);
	return (entry);
}

static void
acpi_set_boot_cpu_id(void)
{
	ACPI_TABLE_MADT *madt;
	ACPI_SUBTABLE_HEADER *entry;
	ACPI_MADT_CORE_PIC *cpu;
	u_int core;

	PCPU_SET(acpi_id, 0xffffffff);
	core = csr_read32(LOONGARCH_CSR_CPUID) & 0x3ff;
	madt = loongarch_madt();
	if (madt == NULL)
		return;
	for (entry = NULL; (entry = madt_next(madt, entry)) != NULL;) {
		if (entry->Type != ACPI_MADT_TYPE_CORE_PIC ||
		    entry->Length < sizeof(*cpu))
			continue;
		cpu = (ACPI_MADT_CORE_PIC *)entry;
		if ((cpu->Flags & ACPI_MADT_ENABLED) != 0 && cpu->CoreId == core) {
			PCPU_SET(acpi_id, cpu->ProcessorId);
			printf("ACPI: boot core %u processor UID %u\n", core,
			    cpu->ProcessorId);
			break;
		}
	}
	acpi_unmap_table(madt);
}

int
loongarch_acpi_madt_cpu_count(void)
{
	ACPI_TABLE_MADT *madt;
	ACPI_SUBTABLE_HEADER *entry;
	ACPI_MADT_CORE_PIC *cpu;
	int count = 0;

	if ((madt = loongarch_madt()) == NULL)
		return (0);
	for (entry = NULL; (entry = madt_next(madt, entry)) != NULL;) {
		if (entry->Type == ACPI_MADT_TYPE_CORE_PIC &&
		    entry->Length >= sizeof(*cpu)) {
			cpu = (ACPI_MADT_CORE_PIC *)entry;
			if ((cpu->Flags & ACPI_MADT_ENABLED) != 0)
				count++;
		}
	}
	acpi_unmap_table(madt);
	return (count);
}

void
loongarch_acpi_madt_foreach_cpu(bool (*callback)(u_int, u_int))
{
	ACPI_TABLE_MADT *madt;
	ACPI_SUBTABLE_HEADER *entry;
	ACPI_MADT_CORE_PIC *cpu;

	if ((madt = loongarch_madt()) == NULL)
		return;
	for (entry = NULL; (entry = madt_next(madt, entry)) != NULL;) {
		if (entry->Type != ACPI_MADT_TYPE_CORE_PIC ||
		    entry->Length < sizeof(*cpu))
			continue;
		cpu = (ACPI_MADT_CORE_PIC *)entry;
		if ((cpu->Flags & ACPI_MADT_ENABLED) != 0 && !callback(cpu->CoreId, cpu->ProcessorId))
			break;
	}
	acpi_unmap_table(madt);
}

bool
loongarch_acpi_pch_pic_info(vm_paddr_t *addr, u_int *size, u_int *gsi_base)
{
	ACPI_TABLE_MADT *madt;
	ACPI_SUBTABLE_HEADER *entry;
	ACPI_MADT_BIO_PIC *pic;
	bool found = false;

	if ((madt = loongarch_madt()) == NULL)
		return (false);
	for (entry = NULL; (entry = madt_next(madt, entry)) != NULL;) {
		if (entry->Type != ACPI_MADT_TYPE_BIO_PIC ||
		    entry->Length < sizeof(*pic))
			continue;
		pic = (ACPI_MADT_BIO_PIC *)entry;
		*addr = pic->Address;
		*size = pic->Size;
		*gsi_base = pic->GsiBase;
		found = *addr != 0 && *size >= 0x400;
		break;
	}
	acpi_unmap_table(madt);
	return (found);
}

bool
loongarch_acpi_pch_msi_info(vm_paddr_t *doorbell, u_int *start, u_int *count)
{
	ACPI_TABLE_MADT *madt;
	ACPI_SUBTABLE_HEADER *entry;
	ACPI_MADT_MSI_PIC *pic;
	bool found = false;

	if ((madt = loongarch_madt()) == NULL)
		return (false);
	for (entry = NULL; (entry = madt_next(madt, entry)) != NULL;) {
		if (entry->Type != ACPI_MADT_TYPE_MSI_PIC ||
		    entry->Length < sizeof(*pic))
			continue;
		pic = (ACPI_MADT_MSI_PIC *)entry;
		*doorbell = pic->MsgAddress;
		*start = pic->Start;
		*count = pic->Count;
		found = *doorbell != 0 && *count != 0 &&
		    *start < 256 && *count <= 256 - *start;
		break;
	}
	acpi_unmap_table(madt);
	return (found);
}
