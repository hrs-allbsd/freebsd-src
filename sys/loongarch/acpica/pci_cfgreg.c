/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Hiroki Sato <hrs@FreeBSD.org>
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <vm/vm.h>
#include <vm/pmap.h>
#include <machine/bus.h>
#include <machine/pci_cfgreg.h>
#include <contrib/dev/acpica/include/acpi.h>

extern struct bus_space memmap_bus;

/* ACPICA PCI_Config operation regions use the same MCFG ECAM as pcib(4). */
static vm_paddr_t
pci_cfgreg_address(int domain, int bus, int slot, int func, int reg, int bytes)
{
	ACPI_TABLE_MCFG *table;
	ACPI_MCFG_ALLOCATION *entry;
	vm_paddr_t pa, address = 0;
	size_t offset;

	if (domain < 0 || domain > 65535 || bus < 0 || bus > 255 ||
	    slot < 0 || slot > 31 || func < 0 || func > 7 || reg < 0 ||
	    (bytes != 1 && bytes != 2 && bytes != 4) ||
	    reg > 4096 - bytes || (reg & (bytes - 1)) != 0)
		return (0);
	pa = acpi_find_table(ACPI_SIG_MCFG);
	if (pa == 0)
		return (0);
	table = acpi_map_table(pa, ACPI_SIG_MCFG);
	if (table == NULL)
		return (0);
	if (table->Header.Length < sizeof(*table)) {
		acpi_unmap_table(table);
		return (0);
	}
	for (offset = sizeof(*table); offset + sizeof(*entry) <= table->Header.Length;
	    offset += sizeof(*entry)) {
		entry = (void *)((char *)table + offset);
		if (entry->PciSegment != domain || bus < entry->StartBusNumber ||
		    bus > entry->EndBusNumber)
			continue;
		if (memcmp(table->Header.OemId, "LOONGS", 6) == 0 &&
		    memcmp(table->Header.OemTableId, "LOONGSON", 8) == 0) {
			uint32_t offset = loongson_pci_cfg_offset(bus,
			    entry->StartBusNumber, slot, func, reg);
			if (offset != 0xffffffff)
				address = entry->Address + offset;
		} else
			address = entry->Address + ((uint64_t)bus << 20) +
			    (slot << 15) + (func << 12) + reg;
		break;
	}
	acpi_unmap_table(table);
	return (address);
}

int
pci_cfgregopen(void)
{
	return (acpi_find_table(ACPI_SIG_MCFG) != 0);
}

uint32_t
pci_cfgregread(int domain, int bus, int slot, int func, int reg, int bytes)
{
	vm_paddr_t pa;
	bus_space_handle_t handle;
	uint32_t value;

	pa = pci_cfgreg_address(domain, bus, slot, func, reg, bytes);
	if (pa == 0 || bus_space_map(&memmap_bus, pa, bytes, 0, &handle) != 0)
		return (0xffffffff);
	switch (bytes) {
	case 1: value = bus_space_read_1(&memmap_bus, handle, 0); break;
	case 2: value = bus_space_read_2(&memmap_bus, handle, 0); break;
	default: value = bus_space_read_4(&memmap_bus, handle, 0); break;
	}
	bus_space_unmap(&memmap_bus, handle, bytes);
	return (value);
}

void
pci_cfgregwrite(int domain, int bus, int slot, int func, int reg, uint32_t value,
    int bytes)
{
	vm_paddr_t pa;
	bus_space_handle_t handle;

	pa = pci_cfgreg_address(domain, bus, slot, func, reg, bytes);
	if (pa == 0 || bus_space_map(&memmap_bus, pa, bytes, 0, &handle) != 0)
		return;
	switch (bytes) {
	case 1: bus_space_write_1(&memmap_bus, handle, 0, value); break;
	case 2: bus_space_write_2(&memmap_bus, handle, 0, value); break;
	default: bus_space_write_4(&memmap_bus, handle, 0, value); break;
	}
	bus_space_barrier(&memmap_bus, handle, 0, bytes, BUS_SPACE_BARRIER_WRITE);
	bus_space_unmap(&memmap_bus, handle, bytes);
}
