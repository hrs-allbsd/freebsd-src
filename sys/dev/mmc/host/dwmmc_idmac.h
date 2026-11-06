/*-
 * SPDX-License-Identifier: BSD-2-Clause
 * Copyright (c) 2026 Hiroki Sato <hrs@FreeBSD.org>
 */
#ifndef _DWMMC_IDMAC_H_
#define _DWMMC_IDMAC_H_

/* Validate a word-aligned segment for the 32-bit IDMAC descriptor format. */
static inline int
dwmmc_idmac_segment_valid(unsigned long long address, unsigned long long length)
{
	return (length != 0 && length <= 2048 && (length & 3) == 0 &&
	    (address & 3) == 0 && address <= 0xffffffffULL &&
	    length - 1 <= 0xffffffffULL - address);
}

static inline int
dwmmc_idmac_ready(int command, int data, int dma, int auto_stop, int stop)
{
	return (command && data && dma && (!auto_stop || stop));
}

#endif
