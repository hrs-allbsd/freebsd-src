/*-
 * SPDX-License-Identifier: BSD-2-Clause
 * Copyright (c) 2026 Hiroki Sato <hrs@FreeBSD.org>
 */
#ifndef _DWMMC_CLOCK_H_
#define _DWMMC_CLOCK_H_

/* Input clock is after any SoC CLKGEN divider; decode slot zero. */
static inline uint64_t
dwmmc_card_clock(uint64_t input, uint32_t divs, uint32_t src, uint32_t ena)
{
	unsigned int div;

	div = (divs >> (8 * (src & 3))) & 255;

	if ((ena & 1) == 0)
		return (0);
	return (div != 0 ? input / (2 * div) : input);
}

#endif
