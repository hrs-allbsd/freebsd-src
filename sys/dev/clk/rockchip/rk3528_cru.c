/*-
 * SPDX-License-Identifier: BSD-2-Clause
 * Copyright (c) 2026 Hiroki Sato <hrs@FreeBSD.org>
 */

/*
 * Partial RK3528 clock/reset provider for private FreeBSD compatibles.
 * Register fields follow FriendlyElec's RK3528 clock implementations.
 * PLL rates come from firmware configuration; no rate tables are supplied.
 * Clock IDs use the imported binding; reset IDs are raw bank/bit indices.
 */
#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/rman.h>
#include <machine/bus.h>
#include <dev/fdt/simplebus.h>
#include <dev/ofw/ofw_bus_subr.h>
#include <dev/clk/rockchip/rk_cru.h>
#include <contrib/device-tree/include/dt-bindings/clock/rockchip,rk3528-cru.h>

#include "hwreset_if.h"

#define CRU_CLKSEL_CON(n) (0x300 + (n) * 4)
#define CRU_CLKGATE_CON(n) (0x800 + (n) * 4)

PLIST(pll_parent) = { "xin24m" };
PLIST(sdmmc_parents) = { "gpll", "cpll", "xin24m" };
PLIST(vo_parents) = { "clk_150m_src", "clk_100m_src", "xin24m" };

static struct rk_clk_pll_def cpll = {
	.clkdef = { .id = PLL_CPLL, .name = "cpll",
	    .parent_names = pll_parent, .parent_cnt = nitems(pll_parent) },
	.base_offset = 0x20,
	.mode_reg = 0x280,
	.mode_shift = 2,
};
static struct rk_clk_pll_def gpll = {
	.clkdef = { .id = PLL_GPLL, .name = "gpll",
	    .parent_names = pll_parent, .parent_cnt = nitems(pll_parent) },
	.base_offset = 0x60,
	.mode_reg = 0x280,
	.mode_shift = 4,
};

static struct rk_clk rk3528_clks[] = {
	FRATE(0, "xin24m", 24000000),
	{ .type = RK3328_CLK_PLL, .clk.pll = &cpll },
	{ .type = RK3328_CLK_PLL, .clk.pll = &gpll },
	CDIV(0, "clk_100m_div", "cpll", 0, 0, 7, 5),
	CDIV(0, "clk_150m_div", "gpll", 0, 1, 0, 5),
	MUX(0, "hclk_vo_mux", vo_parents, 0, 83, 2, 2),
	COMP(0, "sdmmc0_div", sdmmc_parents, 0, 85, 0, 6, 6, 2),
};
static struct rk_cru_gate rk3528_gates[] = {
	GATE(CLK_MATRIX_100M_SRC, "clk_100m_src", "clk_100m_div", 0, 2),
	GATE(CLK_MATRIX_150M_SRC, "clk_150m_src", "clk_150m_div", 0, 3),
	GATE(HCLK_VO_ROOT, "hclk_vo_root", "hclk_vo_mux", 39, 1),
	GATE(CCLK_SRC_SDMMC0, "cclk_src_sdmmc0", "sdmmc0_div", 42, 8),
	GATE(HCLK_SDMMC0, "hclk_sdmmc0", "hclk_vo_root", 42, 9),
};

/* The partial provider does not define a reset map. */
static int
rk3528_cru_reset_assert(device_t dev, intptr_t id, bool reset)
{
	return (ENXIO);
}

static int
rk3528_cru_reset_is_asserted(device_t dev, intptr_t id, bool *reset)
{
	return (ENXIO);
}

static int
rk3528_cru_probe(device_t dev)
{
	if (!ofw_bus_status_okay(dev) ||
	    !ofw_bus_is_compatible(dev, "freebsd,rk3528-sdmmc-cru"))
		return (ENXIO);
	device_set_desc(dev, "Rockchip RK3528 SDMMC0 clocks (initial support)");
	return (BUS_PROBE_DEFAULT);
}

static int
rk3528_cru_attach(device_t dev)
{
	struct rk_cru_softc *sc = device_get_softc(dev);

	sc->clks = rk3528_clks;
	sc->nclks = nitems(rk3528_clks);
	sc->gates = rk3528_gates;
	sc->ngates = nitems(rk3528_gates);
	return (rk_cru_attach(dev));
}

static device_method_t rk3528_cru_methods[] = {
	DEVMETHOD(device_probe, rk3528_cru_probe),
	DEVMETHOD(device_attach, rk3528_cru_attach),
	DEVMETHOD(hwreset_assert, rk3528_cru_reset_assert),
	DEVMETHOD(hwreset_is_asserted, rk3528_cru_reset_is_asserted),
	DEVMETHOD_END
};
DEFINE_CLASS_1(rk3528_cru, rk3528_cru_driver, rk3528_cru_methods,
    sizeof(struct rk_cru_softc), rk_cru_driver);
EARLY_DRIVER_MODULE(rk3528_cru, simplebus, rk3528_cru_driver, 0, 0,
    BUS_PASS_BUS + BUS_PASS_ORDER_MIDDLE);
