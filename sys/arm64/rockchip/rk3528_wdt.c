/*-
 * SPDX-License-Identifier: BSD-2-Clause
 * Copyright (c) 2026 Hiroki Sato <hrs@FreeBSD.org>
 */
/* RK3528 non-secure DesignWare watchdog, reset-only mode. */
#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/eventhandler.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/rman.h>
#include <sys/sbuf.h>
#include <sys/sysctl.h>
#include <sys/time.h>
#include <sys/watchdog.h>

#include <machine/bus.h>

#include <dev/clk/clk.h>
#include <dev/hwreset/hwreset.h>
#include <dev/ofw/ofw_bus_subr.h>

#define WDT_CR	      0x00
#define WDT_CR_EN     (1u << 0)
#define WDT_CR_RMOD   (1u << 1)
#define WDT_TORR      0x04
#define WDT_CCVR      0x08
#define WDT_CRR	      0x0c
#define WDT_PARAMS    0xf4
#define WDT_FIXED_TOP (1u << 6)
#define WDT_HC_TOP (1u << 5)
#define WDT_HC_RMOD (1u << 3)
#define WDT_DUAL_TOP (1u << 2)
#define WDT_DEFAULT_RMOD (1u << 1)
#define WDT_ALWAYS_EN (1u << 0)

struct rk3528_wdt_softc {
	device_t dev;
	struct resource *res;
	clk_t clocks[2];
	bool clocks_on[2];
	hwreset_array_t resets;
	struct mtx lock;
	eventhandler_tag tag;
	uint64_t frequency, feeds, stops;
	uint32_t params;
	sbintime_t actual;
};

/* Reject synthesis options not supported by this reset-only profile. */
static bool
rk3528_wdt_supported(uint32_t params)
{
	return (((params >> 24) & 0x1f) == 16 &&
	    ((params >> 8) & 3) == 2 &&
	    (params & WDT_ALWAYS_EN) == 0 &&
	    (params & WDT_FIXED_TOP) != 0 &&
	    (params & WDT_HC_TOP) == 0 &&
	    ((params & WDT_HC_RMOD) == 0 ||
	    (params & WDT_DEFAULT_RMOD) == 0));
}

/* A real peripheral reset is required: CR.EN cannot be cleared by a write. */
static int
rk3528_wdt_stop(struct rk3528_wdt_softc *sc)
{
	int error;
	error = hwreset_array_assert(sc->resets);
	if (error != 0)
		return (error);
	DELAY(5);
	error = hwreset_array_deassert(sc->resets);
	DELAY(5);
	if (error == 0 && (bus_read_4(sc->res, WDT_CR) & WDT_CR_EN) != 0)
		error = EIO;
	if (error == 0) {
		sc->actual = 0;
		sc->stops++;
	}
	return (error);
}

static void
rk3528_wdt_pat(void *arg, sbintime_t requested, sbintime_t *excess, int *error)
{
	struct rk3528_wdt_softc *sc = arg;
	uint64_t period;
	unsigned top;
	uint32_t torr, control;
	int result;

	/* Reject unsupported periods without changing a running watchdog. */
	if (requested < 0)
		return;
	period = 0;
	for (top = 0; requested != 0 && top < 16; top++) {
		period = ((1ULL << (16 + top)) * SBT_1S + sc->frequency - 1) /
		    sc->frequency;
		if (period >= (uint64_t)requested)
			break;
	}
	if (requested != 0 && top == 16)
		return;
	mtx_lock(&sc->lock);
	if (requested == 0) {
		result = rk3528_wdt_stop(sc);
	} else {
		/* TOP_INIT exists only when synthesized with DUAL_TOP. */
		torr = top;
		if ((sc->params & WDT_DUAL_TOP) != 0)
			torr |= top << 4;
		bus_write_4(sc->res, WDT_TORR, torr);
		bus_write_4(sc->res, WDT_CRR, 0x76);
		/* Preserve reset pulse length; select direct reset and enable. */
		control = bus_read_4(sc->res, WDT_CR);
		bus_write_4(sc->res, WDT_CR, (control & ~WDT_CR_RMOD) | WDT_CR_EN);
		sc->actual = period;
		sc->feeds++;
		result = 0;
	}
	if (result == 0) {
		*error = 0;
		*excess = sc->actual - requested;
	} else
		*error = result;
	mtx_unlock(&sc->lock);
}

static int
rk3528_wdt_snapshot(SYSCTL_HANDLER_ARGS)
{
	struct rk3528_wdt_softc *sc = arg1;
	struct sbuf *sb;
	int error;
	sb = sbuf_new_for_sysctl(NULL, NULL, 256, req);
	if (sb == NULL)
		return (ENOMEM);
	mtx_lock(&sc->lock);
	sbuf_printf(sb,
	    "params=%#x cr=%#x torr=%#x counter=%u clock_hz=%ju "
	    "timeout_sbt=%jd feeds=%ju stops=%ju",
	    sc->params, bus_read_4(sc->res, WDT_CR),
	    bus_read_4(sc->res, WDT_TORR),
	    bus_read_4(sc->res, WDT_CCVR), (uintmax_t)sc->frequency,
	    (intmax_t)sc->actual, (uintmax_t)sc->feeds, (uintmax_t)sc->stops);
	mtx_unlock(&sc->lock);
	error = sbuf_finish(sb);
	sbuf_delete(sb);
	return (error);
}



static int
rk3528_wdt_probe(device_t dev)
{
	if (!ofw_bus_status_okay(dev) ||
	    !ofw_bus_is_compatible(dev, "freebsd,rk3528-r28s-wdt"))
		return (ENXIO);
	device_set_desc(dev, "Rockchip RK3528 non-secure watchdog");
	return (BUS_PROBE_DEFAULT);
}

static int
rk3528_wdt_attach(device_t dev)
{
	struct rk3528_wdt_softc *sc = device_get_softc(dev);
	static const char *names[] = { "tclk", "pclk" };
	int error, rid = 0;
	sc->dev = dev;
	if (OF_hasprop(ofw_bus_get_node(dev), "freebsd,boot-watchdog-test"))
		return (EPERM);
	mtx_init(&sc->lock, device_get_nameunit(dev), NULL, MTX_DEF);
	sc->res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid, RF_ACTIVE);
	if (sc->res == NULL) {
		error = ENXIO;
		goto fail;
	}
	if (rman_get_size(sc->res) < WDT_PARAMS + sizeof(uint32_t)) {
		error = ENXIO;
		goto fail;
	}
	for (unsigned i = 0; i < nitems(names); i++) {
		error = clk_get_by_ofw_name(dev, 0, names[i], &sc->clocks[i]);
		if (error != 0)
			goto fail;
		error = clk_enable(sc->clocks[i]);
		if (error != 0)
			goto fail;
		sc->clocks_on[i] = true;
	}
	error = hwreset_array_get_ofw(dev, 0, &sc->resets);
	if (error != 0)
		goto fail;
	error = clk_get_freq(sc->clocks[0], &sc->frequency);
	if (error != 0)
		goto fail;
	sc->params = bus_read_4(sc->res, WDT_PARAMS);
	if (sc->frequency != 24000000 ||
	    !rk3528_wdt_supported(sc->params)) {
		device_printf(dev,
		    "unsupported counter clock or synthesis parameters\n");
		error = ENXIO;
		goto fail;
	}
	error = rk3528_wdt_stop(sc);
	if (error != 0)
		goto fail;
	sc->tag = EVENTHANDLER_REGISTER(watchdog_sbt_list, rk3528_wdt_pat, sc,
	    0);
	if (sc->tag == NULL) {
		error = ENOMEM;
		goto fail;
	}
	SYSCTL_ADD_PROC(device_get_sysctl_ctx(dev),
	    SYSCTL_CHILDREN(device_get_sysctl_tree(dev)), OID_AUTO, "snapshot",
	    CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_MPSAFE, sc, 0,
	    rk3528_wdt_snapshot, "A",
	    "Watchdog state (no side-effect registers)");
	device_printf(dev,
	    "24000000 Hz, reset-only, maximum 89.478s; disarmed\n");
	return (0);
fail:
	if (sc->resets != NULL)
		hwreset_array_release(sc->resets);
	for (unsigned i = 0; i < nitems(names); i++) {
		if (sc->clocks_on[i])
			clk_disable(sc->clocks[i]);
		if (sc->clocks[i] != NULL)
			clk_release(sc->clocks[i]);
	}
	if (sc->res != NULL)
		bus_release_resource(dev, SYS_RES_MEMORY, 0, sc->res);
	mtx_destroy(&sc->lock);
	return (error);
}

static int
rk3528_wdt_shutdown(device_t dev)
{
	struct rk3528_wdt_softc *sc = device_get_softc(dev);
	int error;
	mtx_lock(&sc->lock);
	error = rk3528_wdt_stop(sc);
	mtx_unlock(&sc->lock);
	return (error);
}
static int
rk3528_wdt_detach(device_t dev)
{
	return (EBUSY);
}
static device_method_t rk3528_wdt_methods[] = {
	DEVMETHOD(device_probe, rk3528_wdt_probe),
	DEVMETHOD(device_attach, rk3528_wdt_attach),
	DEVMETHOD(device_detach, rk3528_wdt_detach),
	DEVMETHOD(device_shutdown, rk3528_wdt_shutdown),
	DEVMETHOD_END
};
static driver_t rk3528_wdt_driver = {
	"rk3528_wdt", rk3528_wdt_methods, sizeof(struct rk3528_wdt_softc)
};
DRIVER_MODULE(rk3528_wdt, simplebus, rk3528_wdt_driver, 0, 0);
MODULE_DEPEND(rk3528_wdt, ofw_bus, 1, 1, 1);
