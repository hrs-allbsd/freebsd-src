/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Hiroki Sato <hrs@FreeBSD.org>
 */

#include "opt_acpi.h"
#include "opt_platform.h"

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/clock.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/rman.h>
#include <machine/bus.h>
#ifdef DEV_ACPI
#include <contrib/dev/acpica/include/acpi.h>
#include <dev/acpica/acpivar.h>
#endif
#ifdef FDT
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>
#endif
#include "clock_if.h"

#define TOY_WRITE0	0x24
#define TOY_WRITE1	0x28
#define TOY_READ0	0x2c
#define TOY_READ1	0x30
#define RTC_CTRL	0x40
#define CTRL_TS		(1U << 0)
#define CTRL_OSC_EN	(1U << 8)
#define CTRL_TOY_EN	(1U << 11)
#define TOY_YEAR_MASK	0x3fff
#define TOY_YEAR_BASE	1900

struct ls7a_rtc_softc {
	struct resource *mem;
	struct mtx lock;
	int rid;
};

static int
ls7a_rtc_probe(device_t dev)
{
#ifdef DEV_ACPI
	static char *ids[] = { "LOON0001", NULL };

	if (strcmp(device_get_name(device_get_parent(dev)), "acpi") == 0) {
		if (acpi_disabled("ls7a_rtc") ||
		    ACPI_ID_PROBE(device_get_parent(dev), dev, ids, NULL) > 0)
			return (ENXIO);
	} else
#endif
	{
#ifdef FDT
		if (!ofw_bus_status_okay(dev) ||
		    !ofw_bus_is_compatible(dev, "loongson,ls7a-rtc"))
			return (ENXIO);
#else
		return (ENXIO);
#endif
	}
	device_set_desc(dev, "Loongson LS7A real-time clock");
	return (BUS_PROBE_DEFAULT);
}

/* Wait at most 10 ms for a calendar write to reach the clock domain. */
static int
ls7a_rtc_wait(struct ls7a_rtc_softc *sc)
{
	int i;

	for (i = 0; i < 1000; i++) {
		if ((bus_read_4(sc->mem, RTC_CTRL) & CTRL_TS) == 0)
			return (0);
		DELAY(10);
	}
	return (ETIMEDOUT);
}

static int
ls7a_rtc_gettime(device_t dev, struct timespec *ts)
{
	struct ls7a_rtc_softc *sc = device_get_softc(dev);
	struct clocktime ct;
	uint32_t year, low, after;
	int error, retry;

	mtx_lock(&sc->lock);
	error = ls7a_rtc_wait(sc);
	if (error == 0) {
		/* The packed calendar is atomic; bracket it for year rollover. */
		for (retry = 0; retry < 4; retry++) {
			year = bus_read_4(sc->mem, TOY_READ1);
			low = bus_read_4(sc->mem, TOY_READ0);
			after = bus_read_4(sc->mem, TOY_READ1);
			if (year == after)
				break;
		}
		if (retry == 4)
			error = EAGAIN;
	}
	mtx_unlock(&sc->lock);
	if (error != 0)
		return (error);
	bzero(&ct, sizeof(ct));
	ct.year = (year & TOY_YEAR_MASK) + TOY_YEAR_BASE;
	ct.mon = (low >> 26) & 0x3f;
	ct.day = (low >> 21) & 0x1f;
	ct.hour = (low >> 16) & 0x1f;
	ct.min = (low >> 10) & 0x3f;
	ct.sec = (low >> 4) & 0x3f;
	if ((low & 0xf) > 9)
		return (EINVAL);
	ct.nsec = (low & 0xf) * 100000000;
	clock_dbgprint_ct(dev, CLOCK_DBG_READ, &ct);
	return (clock_ct_to_ts(&ct, ts));
}

static int
ls7a_rtc_settime(device_t dev, struct timespec *ts)
{
	struct ls7a_rtc_softc *sc = device_get_softc(dev);
	struct clocktime ct;
	uint32_t low;
	int error;

	clock_ts_to_ct(ts, &ct);
	clock_dbgprint_ct(dev, CLOCK_DBG_WRITE, &ct);
	if (ct.year < TOY_YEAR_BASE ||
	    ct.year > TOY_YEAR_BASE + TOY_YEAR_MASK)
		return (EINVAL);
	low = (ct.mon << 26) | (ct.day << 21) | (ct.hour << 16) |
	    (ct.min << 10) | (ct.sec << 4) | (ct.nsec / 100000000);
	mtx_lock(&sc->lock);
	error = ls7a_rtc_wait(sc);
	if (error == 0) {
		bus_write_4(sc->mem, TOY_WRITE1, ct.year - TOY_YEAR_BASE);
		error = ls7a_rtc_wait(sc);
	}
	if (error == 0) {
		bus_write_4(sc->mem, TOY_WRITE0, low);
		error = ls7a_rtc_wait(sc);
	}
	mtx_unlock(&sc->lock);
	return (error);
}

static int
ls7a_rtc_attach(device_t dev)
{
	struct ls7a_rtc_softc *sc = device_get_softc(dev);
	uint32_t ctrl;

	sc->mem = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &sc->rid,
	    RF_ACTIVE);
	if (sc->mem == NULL)
		return (ENXIO);
	if (rman_get_size(sc->mem) < RTC_CTRL + 4) {
		bus_release_resource(dev, SYS_RES_MEMORY, sc->rid, sc->mem);
		return (ENXIO);
	}
	mtx_init(&sc->lock, device_get_nameunit(dev), NULL, MTX_DEF);
	ctrl = bus_read_4(sc->mem, RTC_CTRL);
	if ((ctrl & (CTRL_OSC_EN | CTRL_TOY_EN)) !=
	    (CTRL_OSC_EN | CTRL_TOY_EN))
		bus_write_4(sc->mem, RTC_CTRL, ctrl | CTRL_OSC_EN | CTRL_TOY_EN);
	/* Preserve the calendar, oscillator trim, and all alarm registers. */
	clock_register(dev, 100000);
	return (0);
}

static int
ls7a_rtc_detach(device_t dev)
{
	struct ls7a_rtc_softc *sc = device_get_softc(dev);

	clock_unregister(dev);
	mtx_destroy(&sc->lock);
	bus_release_resource(dev, SYS_RES_MEMORY, sc->rid, sc->mem);
	return (0);
}

static device_method_t ls7a_rtc_methods[] = {
	DEVMETHOD(device_probe,		ls7a_rtc_probe),
	DEVMETHOD(device_attach,	ls7a_rtc_attach),
	DEVMETHOD(device_detach,	ls7a_rtc_detach),
	DEVMETHOD(clock_gettime,	ls7a_rtc_gettime),
	DEVMETHOD(clock_settime,	ls7a_rtc_settime),
	DEVMETHOD_END
};

static driver_t ls7a_rtc_driver = {
	"ls7a_rtc", ls7a_rtc_methods, sizeof(struct ls7a_rtc_softc)
};
#ifdef DEV_ACPI
DRIVER_MODULE(ls7a_rtc, acpi, ls7a_rtc_driver, 0, 0);
MODULE_DEPEND(ls7a_rtc, acpi, 1, 1, 1);
#endif
#ifdef FDT
DRIVER_MODULE(ls7a_rtc, simplebus, ls7a_rtc_driver, 0, 0);
#endif
MODULE_VERSION(ls7a_rtc, 1);
