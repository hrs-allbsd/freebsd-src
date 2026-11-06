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
#include <sys/malloc.h>
#include <sys/lock.h>
#include <sys/mutex.h>
#include <sys/rman.h>
#include <machine/bus.h>
#include <dev/fdt/simplebus.h>
#include <dev/ofw/ofw_bus_subr.h>
#include <dev/clk/rockchip/rk_cru.h>
#include <contrib/device-tree/include/dt-bindings/clock/rockchip,rk3528-cru.h>

#include "hwreset_if.h"

/* pclk_vo_root supplies the CRU's own register interface. Keep a lifetime
 * reference in profiles that expose consumers of this critical root.
 */
struct rk3528_cru_softc {
	struct rk_cru_softc base;
	clk_t critical_pclk_vo;
};

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

/* Additional clocks require the larger GMAC bring-up register mapping. */
PLIST(pclk_parents) = { "clk_100m_src", "clk_50m_src", "xin24m" };
PLIST(hclk_vpu_parents) = { "clk_200m_src", "clk_100m_src", "clk_50m_src", "xin24m" };
PLIST(aclk_parents) = { "clk_200m_src", "clk_100m_src", "xin24m" };
static struct rk_clk_pll_def ppll = {
	.clkdef = { .id = PLL_PPLL, .name = "ppll",
	    .parent_names = pll_parent, .parent_cnt = nitems(pll_parent) },
	.base_offset = 0x20080,
};
static struct rk_clk rk3528_net_clks[] = {
	{ .type = RK3328_CLK_PLL, .clk.pll = &ppll },
	CDIV(0, "clk_50m_div", "cpll", 0, 0, 2, 5),
	CDIV(0, "clk_200m_div", "gpll", 0, 1, 5, 5),
	MUX(0, "pclk_vpu_mux", pclk_parents, 0, 61, 4, 2),
	MUX(0, "aclk_vpu_l_mux", aclk_parents, 0, 60, 0, 2),
	CDIV(0, "gmac1_125m_div", "ppll", 0, 60, 10, 5),
	CDIV(0, "gmac1_25m_div", "ppll", 0, 60, 2, 8),
	MUX(0, "hclk_vpu_mux", hclk_vpu_parents, 0, 61, 2, 2),
	MUX(0, "pclk_rkvenc_mux", pclk_parents, 0, 79, 4, 2),
};
static struct rk_cru_gate rk3528_net_gates[] = {
	GATE(CLK_MATRIX_50M_SRC, "clk_50m_src", "clk_50m_div", 0, 1),
	GATE(CLK_MATRIX_200M_SRC, "clk_200m_src", "clk_200m_div", 0, 4),
	GATE(PCLK_VPU_ROOT, "pclk_vpu_root", "pclk_vpu_mux", 25, 5),
	GATE(ACLK_VPU_L_ROOT, "aclk_vpu_l_root", "aclk_vpu_l_mux", 25, 0),
	GATE(PCLK_MAC_VPU, "pclk_gmac1", "pclk_vpu_root", 28, 6),
	GATE(ACLK_MAC_VPU, "aclk_gmac1", "aclk_vpu_l_root", 28, 5),
	GATE(CLK_GMAC1_SRC_VPU, "clk_gmac1_125m", "gmac1_125m_div", 25, 2),
	GATE(CLK_GMAC1_VPU_25M, "clk_gmac1_25m", "gmac1_25m_div", 25, 1),
	GATE(HCLK_VPU_ROOT, "hclk_vpu_root", "hclk_vpu_mux", 25, 4),
	GATE(PCLK_RKVENC_ROOT, "pclk_rkvenc_root", "pclk_rkvenc_mux", 36, 2),
	GATE(PCLK_GPIO4, "pclk_gpio4", "pclk_rkvenc_root", 37, 8),
};

/* UART0 is opt-in so the established console profiles stay unchanged. */
PLIST(uart0_parents) = { "clk_uart0_src", "clk_uart0_frac", "xin24m" };
static struct rk_clk rk3528_uart_clks[] = {
	CDIV(0, "uart0_src_div", "gpll", 0, 4, 5, 5),
	FRACT(0, "uart0_frac_div", "clk_uart0_src", 0, 5),
	MUX(CLK_UART0, "clk_uart0", uart0_parents, 0, 6, 0, 2),
	MUX(0, "pclk_bus_mux", pclk_parents, 0, 43, 10, 2),
};
static struct rk_cru_gate rk3528_uart_gates[] = {
	GATE(CLK_UART0_SRC, "clk_uart0_src", "uart0_src_div", 0, 12),
	GATE(CLK_UART0_FRAC, "clk_uart0_frac", "uart0_frac_div", 0, 13),
	GATE(SCLK_UART0, "sclk_uart0", "clk_uart0", 0, 14),
	GATE(PCLK_BUS_ROOT, "pclk_bus_root", "pclk_bus_mux", 8, 6),
	GATE(PCLK_UART0, "pclk_uart0", "pclk_bus_root", 10, 7),
};

/* I2C1 clocks for the board RTC; no other I2C controllers are exposed. */
static struct rk_clk rk3528_rtc_clks[] = {
	MUX(0, "i2c1_mux", hclk_vpu_parents, 0, 79, 9, 2),
};
static struct rk_cru_gate rk3528_rtc_gates[] = {
	GATE(CLK_I2C1, "clk_i2c1", "i2c1_mux", 36, 12),
	GATE(PCLK_I2C1, "pclk_i2c1", "pclk_rkvenc_root", 36, 11),
};

/* TSADC clocks for the opt-in monitor profile. */
static struct rk_clk rk3528_thermal_clks[] = {
	CDIV(0, "tsadc_div", "xin24m", 0, 74, 3, 5),
	CDIV(0, "tsadc_tsen_div", "xin24m", 0, 74, 8, 5),
};
static struct rk_cru_gate rk3528_thermal_gates[] = {
	GATE(PCLK_TSADC, "pclk_tsadc", "pclk_vpu_root", 32, 14),
	GATE(CLK_TSADC, "clk_tsadc", "tsadc_div", 32, 15),
	GATE(CLK_TSADC_TSEN, "clk_tsadc_tsen", "tsadc_tsen_div", 33, 0),
};

/* Only the non-secure read controller used for the TSADC trim word. */
static struct rk_clk rk3528_trim_clks[] = {
	MUX(0, "pclk_vo_mux", pclk_parents, 0, 83, 4, 2),
	FFACT(CLK_USER_OTPC_NS, "clk_user_otpc_ns", "clk_sbpi_otpc_ns", 1, 2),
};
static struct rk_cru_gate rk3528_trim_gates[] = {
	GATE(PCLK_VO_ROOT, "pclk_vo_root", "pclk_vo_mux", 39, 2),
	GATE(PCLK_OTPC_NS, "pclk_otpc_ns", "pclk_vo_root", 42, 11),
	GATE(CLK_SBPI_OTPC_NS, "clk_sbpi_otpc_ns", "xin24m", 42, 12),
};

/* Non-secure watchdog: 24MHz counter and APB register clock. */
static struct rk_cru_gate rk3528_wdt_gates[] = {
	GATE(PCLK_WDT_NS, "pclk_wdt_ns", "pclk_bus_root", 9, 15),
	GATE(TCLK_WDT_NS, "tclk_wdt_ns", "xin24m", 10, 0),
};

/* USB2 OTG clocks; the combo PHY stays assigned to PCIe. */
static struct rk_cru_gate rk3528_usb_gates[] = {
	GATE(PCLK_USBPHY, "pclk_usbphy", "pclk_vo_root", 43, 13),
	GATE(CLK_REF_USBPHY, "clk_ref_usbphy", "xin24m", 43, 14),
	GATE(CLK_REF_USB3OTG, "clk_ref_usb3otg", "xin24m", 33, 2),
	GATE(CLK_SUSPEND_USB3OTG, "clk_suspend_usb3otg", "xin24m", 33, 4),
	GATE(ACLK_USB3OTG, "aclk_usb3otg", "aclk_vpu_l_root", 33, 1),
};

/* Select the board's 24MHz path without changing its running console rate. */
static int
rk3528_uart0_console_clock(device_t dev)
{
	clk_t mux, oscillator;
	uint64_t frequency;
	int error;

	if ((error = clk_get_by_name(dev, "clk_uart0", &mux)) != 0)
		return (error);
	if ((error = clk_get_freq(mux, &frequency)) != 0 ||
	    frequency != 24000000) {
		device_printf(dev, "UART0 handoff clock is not 24MHz\n");
		clk_release(mux);
		return (error != 0 ? error : EINVAL);
	}
	error = clk_get_by_name(dev, "xin24m", &oscillator);
	if (error == 0) {
		error = clk_set_parent_by_clk(mux, oscillator);
		clk_release(oscillator);
	}
	if (error == 0)
		error = clk_get_freq(mux, &frequency);
	clk_release(mux);
	if (error != 0 || frequency != 24000000)
		return (error != 0 ? error : EINVAL);
	device_printf(dev, "UART0 console clock: 24000000 Hz from xin24m\n");
	return (0);
}

/* Raw reset indices are private to the FreeBSD prototype compatibles. */
static bool
rk3528_reset_valid(struct rk_cru_softc *sc, intptr_t id)
{
	if (sc->type >= 8 && id == 529)
		return (true);
	if (sc->type >= 6 && (id == 159 || id == 160))
		return (true);
	if (sc->type >= 5 && (id == 526 || id == 527))
		return (true);
	if (sc->type >= 1 && id == 453)
		return (true);
	return (false);
}

static int
rk3528_cru_reset_assert(device_t dev, intptr_t id, bool reset)
{
	struct rk_cru_softc *sc = device_get_softc(dev);

	if (!rk3528_reset_valid(sc, id))
		return (ENXIO);
	mtx_lock(&sc->mtx);
	bus_write_4(sc->res, 0xa00 + (id / 16) * 4,
	    (1u << (16 + id % 16)) | (reset ? (1u << (id % 16)) : 0));
	mtx_unlock(&sc->mtx);
	return (0);
}

static int
rk3528_cru_reset_is_asserted(device_t dev, intptr_t id, bool *reset)
{
	struct rk_cru_softc *sc = device_get_softc(dev);

	if (!rk3528_reset_valid(sc, id))
		return (ENXIO);
	mtx_lock(&sc->mtx);
	*reset = (bus_read_4(sc->res, 0xa00 + (id / 16) * 4) &
	    (1u << (id % 16))) != 0;
	mtx_unlock(&sc->mtx);
	return (0);
}

static int
rk3528_cru_probe(device_t dev)
{
	if (!ofw_bus_status_okay(dev) ||
	    (!ofw_bus_is_compatible(dev, "freebsd,rk3528-sdmmc-cru") &&
	    !ofw_bus_is_compatible(dev, "freebsd,rk3528-sdmmc-gmac-cru") &&
	    !ofw_bus_is_compatible(dev, "freebsd,rk3528-r28s-uart-cru") &&
	    !ofw_bus_is_compatible(dev, "freebsd,rk3528-r28s-rtc-cru") &&
	    !ofw_bus_is_compatible(dev, "freebsd,rk3528-r28s-thermal-cru") &&
	    !ofw_bus_is_compatible(dev, "freebsd,rk3528-r28s-trim-cru") &&
	    !ofw_bus_is_compatible(dev, "freebsd,rk3528-r28s-watchdog-cru") &&
	    !ofw_bus_is_compatible(dev, "freebsd,rk3528-r28s-usb-cru")))
		return (ENXIO);
	device_set_desc(dev, "Rockchip RK3528 SDMMC0 clocks (initial support)");
	return (BUS_PROBE_DEFAULT);
}

static int
rk3528_cru_attach(device_t dev)
{
	struct rk3528_cru_softc *rsc = device_get_softc(dev);
	struct rk_cru_softc *sc = &rsc->base;
	bool uart, rtc, thermal, trim, wdt, usb;
	size_t clock_bytes, gate_bytes;
	int error;

	usb = ofw_bus_is_compatible(dev, "freebsd,rk3528-r28s-usb-cru");
	wdt = usb || ofw_bus_is_compatible(dev, "freebsd,rk3528-r28s-watchdog-cru");
	trim = wdt || ofw_bus_is_compatible(dev, "freebsd,rk3528-r28s-trim-cru");
	thermal = trim || ofw_bus_is_compatible(dev, "freebsd,rk3528-r28s-thermal-cru");
	rtc = thermal || ofw_bus_is_compatible(dev, "freebsd,rk3528-r28s-rtc-cru");
	uart = rtc || ofw_bus_is_compatible(dev, "freebsd,rk3528-r28s-uart-cru");

	sc->clks = rk3528_clks;
	sc->nclks = nitems(rk3528_clks);
	sc->gates = rk3528_gates;
	sc->ngates = nitems(rk3528_gates);
	if (ofw_bus_is_compatible(dev, "freebsd,rk3528-sdmmc-gmac-cru") || uart) {
		sc->type = 1;
		sc->nclks += nitems(rk3528_net_clks);
		clock_bytes = sizeof(rk3528_clks) + sizeof(rk3528_net_clks);
		if (uart)
			clock_bytes += sizeof(rk3528_uart_clks);
		if (rtc)
			clock_bytes += sizeof(rk3528_rtc_clks);
		if (thermal)
			clock_bytes += sizeof(rk3528_thermal_clks);
		if (trim)
			clock_bytes += sizeof(rk3528_trim_clks);
		sc->clks = malloc(clock_bytes, M_DEVBUF, M_WAITOK);
		memcpy(sc->clks, rk3528_clks, sizeof(rk3528_clks));
		memcpy(sc->clks + nitems(rk3528_clks), rk3528_net_clks,
		    sizeof(rk3528_net_clks));
		sc->ngates += nitems(rk3528_net_gates);
		gate_bytes = sizeof(rk3528_gates) + sizeof(rk3528_net_gates);
		if (uart)
			gate_bytes += sizeof(rk3528_uart_gates);
		if (rtc)
			gate_bytes += sizeof(rk3528_rtc_gates);
		if (thermal)
			gate_bytes += sizeof(rk3528_thermal_gates);
		if (trim)
			gate_bytes += sizeof(rk3528_trim_gates);
		if (wdt)
			gate_bytes += sizeof(rk3528_wdt_gates);
		if (usb)
			gate_bytes += sizeof(rk3528_usb_gates);
		sc->gates = malloc(gate_bytes, M_DEVBUF, M_WAITOK);
		memcpy(sc->gates, rk3528_gates, sizeof(rk3528_gates));
		memcpy(sc->gates + nitems(rk3528_gates), rk3528_net_gates,
		    sizeof(rk3528_net_gates));
		device_set_desc(dev, "Rockchip RK3528 SDMMC0/GMAC1 clocks");
	}
	if (uart) {
		memcpy(sc->clks + sc->nclks, rk3528_uart_clks,
		    sizeof(rk3528_uart_clks));
		sc->nclks += nitems(rk3528_uart_clks);
		memcpy(sc->gates + sc->ngates, rk3528_uart_gates,
		    sizeof(rk3528_uart_gates));
		sc->ngates += nitems(rk3528_uart_gates);
		sc->type = 3;
		device_set_desc(dev, "Rockchip RK3528 SDMMC0/GMAC1/UART0 clocks");
	}
	if (rtc) {
		memcpy(sc->clks + sc->nclks, rk3528_rtc_clks,
		    sizeof(rk3528_rtc_clks));
		sc->nclks += nitems(rk3528_rtc_clks);
		memcpy(sc->gates + sc->ngates, rk3528_rtc_gates,
		    sizeof(rk3528_rtc_gates));
		sc->ngates += nitems(rk3528_rtc_gates);
		sc->type = 4;
	}
	if (thermal) {
		memcpy(sc->clks + sc->nclks, rk3528_thermal_clks,
		    sizeof(rk3528_thermal_clks));
		sc->nclks += nitems(rk3528_thermal_clks);
		memcpy(sc->gates + sc->ngates, rk3528_thermal_gates,
		    sizeof(rk3528_thermal_gates));
		sc->ngates += nitems(rk3528_thermal_gates);
		sc->type = 5;
	}
	if (trim) {
		memcpy(sc->clks + sc->nclks, rk3528_trim_clks,
		    sizeof(rk3528_trim_clks));
		sc->nclks += nitems(rk3528_trim_clks);
		memcpy(sc->gates + sc->ngates, rk3528_trim_gates,
		    sizeof(rk3528_trim_gates));
		sc->ngates += nitems(rk3528_trim_gates);
	}
	if (wdt) {
		memcpy(sc->gates + sc->ngates, rk3528_wdt_gates,
		    sizeof(rk3528_wdt_gates));
		sc->ngates += nitems(rk3528_wdt_gates);
		sc->type = 6;
	}
	if (usb) {
		memcpy(sc->gates + sc->ngates, rk3528_usb_gates,
		    sizeof(rk3528_usb_gates));
		sc->ngates += nitems(rk3528_usb_gates);
		sc->type = 8;
	}
	error = rk_cru_attach(dev);
	if (error == 0 && trim) {
		error = clk_get_by_name(dev, "pclk_vo_root", &rsc->critical_pclk_vo);
		if (error == 0) {
			error = clk_enable(rsc->critical_pclk_vo);
			if (error != 0) {
				clk_release(rsc->critical_pclk_vo);
				rsc->critical_pclk_vo = NULL;
			}
		}
		if (error == 0)
			device_printf(dev,
			    "Retained critical pclk_vo_root for CRU register access\n");
	}

	if (error == 0 && uart)
		error = rk3528_uart0_console_clock(dev);
	return (error);
}

static device_method_t rk3528_cru_methods[] = {
	DEVMETHOD(device_probe, rk3528_cru_probe),
	DEVMETHOD(device_attach, rk3528_cru_attach),
	DEVMETHOD(hwreset_assert, rk3528_cru_reset_assert),
	DEVMETHOD(hwreset_is_asserted, rk3528_cru_reset_is_asserted),
	DEVMETHOD_END
};
DEFINE_CLASS_1(rk3528_cru, rk3528_cru_driver, rk3528_cru_methods,
    sizeof(struct rk3528_cru_softc), rk_cru_driver);
EARLY_DRIVER_MODULE(rk3528_cru, simplebus, rk3528_cru_driver, 0, 0,
    BUS_PASS_BUS + BUS_PASS_ORDER_MIDDLE);
