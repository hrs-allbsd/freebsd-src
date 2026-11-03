/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Hiroki Sato <hrs@FreeBSD.org>
 */

#include <sys/param.h>
#include <machine/cpufunc.h>
#include <machine/loongarchreg.h>
#include <machine/vmparam.h>
#include <contrib/dev/acpica/include/acpi.h>

/*
 * ACPICA accesses full-hardware FADT registers with its hardware spinlock
 * held.  Mapping each access through kva_alloc/pmap_mapdev can sleep.  DMW0
 * already provides an uncached, allocation-free physical I/O window.
 */
static volatile void *
acpi_memory_address(ACPI_PHYSICAL_ADDRESS address, UINT32 width)
{
	if (width != 8 && width != 16 && width != 32 && width != 64)
		return (NULL);
	if ((address & (width / 8 - 1)) != 0 ||
	    address >= (1ULL << DMW_PABITS) ||
	    width / 8 > (1ULL << DMW_PABITS) - address)
		return (NULL);
	return ((volatile void *)(CSR_DMW0_BASE | address));
}

ACPI_STATUS
AcpiOsReadMemory(ACPI_PHYSICAL_ADDRESS address, UINT64 *value, UINT32 width)
{
	volatile void *p;

	if (value == NULL || (p = acpi_memory_address(address, width)) == NULL)
		return (AE_BAD_PARAMETER);
	mb();
	switch (width) {
	case 8: *value = *(volatile uint8_t *)p; break;
	case 16: *value = *(volatile uint16_t *)p; break;
	case 32: *value = *(volatile uint32_t *)p; break;
	case 64: *value = *(volatile uint64_t *)p; break;
	}
	mb();
	return (AE_OK);
}

ACPI_STATUS
AcpiOsWriteMemory(ACPI_PHYSICAL_ADDRESS address, UINT64 value, UINT32 width)
{
	volatile void *p;

	if ((p = acpi_memory_address(address, width)) == NULL)
		return (AE_BAD_PARAMETER);
	mb();
	switch (width) {
	case 8: *(volatile uint8_t *)p = value; break;
	case 16: *(volatile uint16_t *)p = value; break;
	case 32: *(volatile uint32_t *)p = value; break;
	case 64: *(volatile uint64_t *)p = value; break;
	}
	mb();
	return (AE_OK);
}
