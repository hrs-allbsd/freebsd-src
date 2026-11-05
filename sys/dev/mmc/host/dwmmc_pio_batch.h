/*-
 * SPDX-License-Identifier: BSD-2-Clause
 * Copyright (c) 2026 Hiroki Sato <hrs@FreeBSD.org>
 */
#ifndef _DWMMC_PIO_BATCH_H_
#define _DWMMC_PIO_BATCH_H_

/*
 * Read no more words than are already present in the FIFO.  The caller must
 * own the controller mutex and have no competing FIFO consumer or DMA.
 * Zero requests the legacy single-word path, not an unbounded retry.
 */
static inline unsigned int
dwmmc_pio_batch_words(unsigned int status, unsigned int remaining,
    unsigned int depth)
{
	unsigned int available;

	available = (status >> 17) & 0x1fff;
	if (remaining == 0 || depth == 0 || (status & (1U << 2)) != 0 ||
	    available == 0 || available > depth)
		return (0);
	return (available < remaining ? available : remaining);
}

#endif
