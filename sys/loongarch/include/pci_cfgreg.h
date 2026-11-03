/*
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2024 Xiaoqiang Zhao <zhaoxiaoqiang007@gmail.com>
 */

#ifndef _MACHINE_PCI_CFGREG_H
#define	_MACHINE_PCI_CFGREG_H

int pci_cfgregopen(void);
uint32_t pci_cfgregread(int, int, int, int, int, int);
void pci_cfgregwrite(int, int, int, int, int, uint32_t, int);

/* LS7A CFG1: type 1 at bit 28, extended register byte at bits 27:24. */
static __inline uint32_t
loongson_pci_cfg_offset(u_int bus, u_int root, u_int slot, u_int func, u_int reg)
{
	uint32_t offset;

	/* These root-bridge functions do not exist and alias function zero. */
	if (bus == root && slot >= 9 && slot <= 20 && func != 0)
		return (0xffffffff);
	offset = bus == root ? 0 : 0x10000000 | (bus << 16);
	return (offset | (slot << 11) | (func << 8) | (reg & 0xff) |
	    ((reg & 0xf00) << 16));
}

#endif /* !_MACHINE_PCI_CFGREG_H */
