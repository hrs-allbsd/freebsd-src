/*-
 * SPDX-License-Identifier: BSD-2-Clause
 * Copyright (c) 2026 Hiroki Sato <hrs@FreeBSD.org>
 */
/* Read non-secure RKRNG DRNG output initialized and seeded by firmware. */
#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/rman.h>
#include <sys/random.h>
#include <sys/sysctl.h>
#include <sys/time.h>
#include <machine/bus.h>
#include <dev/clk/clk.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>
#include <dev/random/randomdev.h>

#include <dev/random/rkrng_reg.h>

struct rkrng_softc {
	device_t dev;
	struct resource *res;
	clk_t clock;
	struct mtx lock;
	bool pending, have_last, failed;
	sbintime_t deadline;
	uint32_t last[8];
	uint64_t blocks, bytes, timeouts, health_failures, hardware_failures;
	uint32_t last_error_state;
};
static struct rkrng_softc *source_sc;
static u_int rkrng_read(void *, u_int);
static const struct random_source rkrng_source = {
	.rs_ident = "Rockchip RKRNG firmware-seeded DRNG",
	.rs_source = RANDOM_PURE_ROCKCHIP,
	.rs_read = rkrng_read,
};

/* Stop only the NS DRNG request; configuration belongs to secure firmware. */
static int
rkrng_check_state(struct rkrng_softc *sc, uint32_t state, uint32_t data[8])
{
	if ((state & RKRNG_STATE_ERRORS) == 0)
		return (0);
	sc->last_error_state = state & RKRNG_STATE_ERRORS;
	sc->hardware_failures++;
	sc->failed = true;
	sc->pending = false;
	bus_write_4(sc->res, RKRNG_CTRL, RKRNG_CTRL_DRNG_WE);
	explicit_bzero(data, RKRNG_BYTES);
	return (EIO);
}

/* EAGAIN preserves an outstanding request for the next nonblocking read. */
static int
rkrng_block(struct rkrng_softc *sc, uint32_t data[8], bool wait)
{
	uint32_t state, any = 0, all = UINT32_MAX;

	mtx_assert(&sc->lock, MA_OWNED);
	explicit_bzero(data, RKRNG_BYTES);
	if (sc->failed)
		return (EIO);
	if (!sc->pending) {
		state = bus_read_4(sc->res, RKRNG_STATE);
		if (rkrng_check_state(sc, state, data) != 0)
			return (EIO);
		if (state & RKRNG_STATE_DRNG_ACK)
			bus_write_4(sc->res, RKRNG_STATE, RKRNG_STATE_DRNG_ACK);
		bus_write_4(sc->res, RKRNG_CTRL,
		    RKRNG_CTRL_DRNG_WE | RKRNG_CTRL_DRNG_REQ);
		sc->pending = true;
		sc->deadline = sbinuptime() + 50 * SBT_1MS;
	}
	for (;;) {
		state = bus_read_4(sc->res, RKRNG_STATE);
		if (rkrng_check_state(sc, state, data) != 0)
			return (EIO);
		if (state & RKRNG_STATE_DRNG_ACK)
			break;
		if (sbinuptime() >= sc->deadline) {
			bus_write_4(sc->res, RKRNG_CTRL, RKRNG_CTRL_DRNG_WE);
			sc->pending = false;
			sc->timeouts++;
			sc->failed = true;
			return (ETIMEDOUT);
		}
		if (!wait)
			return (EAGAIN);
		DELAY(100);
	}
	bus_read_region_4(sc->res, RKRNG_DRNG_DATA0, data, 8);
	state = bus_read_4(sc->res, RKRNG_STATE);
	if (rkrng_check_state(sc, state, data) != 0)
		return (EIO);
	bus_write_4(sc->res, RKRNG_STATE, RKRNG_STATE_DRNG_ACK);
	bus_write_4(sc->res, RKRNG_CTRL, RKRNG_CTRL_DRNG_WE);
	sc->pending = false;
	for (unsigned i = 0; i < 8; i++) {
		any |= data[i];
		all &= data[i];
	}
	if (any == 0 || all == UINT32_MAX ||
	    (sc->have_last && memcmp(data, sc->last, RKRNG_BYTES) == 0)) {
		sc->health_failures++;
		sc->failed = true;
		explicit_bzero(data, RKRNG_BYTES);
		return (EIO);
	}
	memcpy(sc->last, data, RKRNG_BYTES);
	sc->have_last = true;
	sc->blocks++;
	return (0);
}

/* Called inside the entropy source epoch: never wait for a lock or hardware. */
static u_int
rkrng_read(void *buf, u_int size)
{
	struct rkrng_softc *sc = source_sc;
	uint32_t data[8];
	u_int count = 0;

	if (sc == NULL || size == 0 || !mtx_trylock(&sc->lock))
		return (0);
	if (rkrng_block(sc, data, false) == 0) {
		count = MIN(size, RKRNG_BYTES);
		memcpy(buf, data, count);
		sc->bytes += count;
	}
	explicit_bzero(data, sizeof(data));
	mtx_unlock(&sc->lock);
	return (count);
}

static int
rkrng_probe(device_t dev)
{
	if (!ofw_bus_status_okay(dev) ||
	    (!ofw_bus_is_compatible(dev, "rockchip,rkrng") &&
	    !ofw_bus_is_compatible(dev, "rockchip,rk3528-rkrng")))
		return (ENXIO);
	device_set_desc(dev, "Rockchip RKRNG firmware-initialized DRNG");
	return (BUS_PROBE_DEFAULT);
}

static int
rkrng_attach(device_t dev)
{
	struct rkrng_softc *sc = device_get_softc(dev);
	uint32_t data[8];
	int rid = 0, error;

	sc->dev = dev;
	mtx_init(&sc->lock, "rk3528 rng", NULL, MTX_DEF);
	if ((error = clk_get_by_ofw_name(dev, 0, "hclk_trng", &sc->clock)) != 0)
		goto fail;
	if ((error = clk_enable(sc->clock)) != 0)
		goto fail;
	sc->res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid, RF_ACTIVE);
	if (sc->res == NULL) {
		error = ENXIO;
		goto fail_clock;
	}
	if (rman_get_size(sc->res) < RKRNG_DRNG_DATA0 + RKRNG_BYTES) {
		error = ENXIO;
		goto fail_res;
	}
	/* Leave secure configuration and error status intact. */
	bus_write_4(sc->res, RKRNG_CTRL, RKRNG_CTRL_DRNG_WE);
	mtx_lock(&sc->lock);
	for (unsigned i = 0; i < 16; i++) {
		error = rkrng_block(sc, data, true);
		if (error != 0)
			break;
	}
	explicit_bzero(data, sizeof(data));
	mtx_unlock(&sc->lock);
	if (error != 0) {
		device_printf(dev, "Initial sample check failed: %d\n", error);
		goto fail_res;
	}
#define RNG_COUNTER(name) \
	SYSCTL_ADD_U64(device_get_sysctl_ctx(dev), \
	    SYSCTL_CHILDREN(device_get_sysctl_tree(dev)), OID_AUTO, #name, \
	    CTLFLAG_RD, &sc->name, 0, "RKRNG " #name)
	RNG_COUNTER(blocks);
	RNG_COUNTER(bytes);
	RNG_COUNTER(timeouts);
	RNG_COUNTER(health_failures);
	RNG_COUNTER(hardware_failures);
#undef RNG_COUNTER
	SYSCTL_ADD_BOOL(device_get_sysctl_ctx(dev),
	    SYSCTL_CHILDREN(device_get_sysctl_tree(dev)), OID_AUTO, "failed",
	    CTLFLAG_RD, &sc->failed, 0, "Source failed closed");
	SYSCTL_ADD_UINT(device_get_sysctl_ctx(dev),
	    SYSCTL_CHILDREN(device_get_sysctl_tree(dev)), OID_AUTO,
	    "last_error_state", CTLFLAG_RD, &sc->last_error_state, 0,
	    "Latched RKRNG hardware error bits");
	if (OF_hasprop(ofw_bus_get_node(dev), "freebsd,entropy-source")) {
		if (source_sc != NULL) {
			error = EBUSY;
			goto fail_res;
		}
		source_sc = sc;
		random_source_register(&rkrng_source);
		device_printf(dev, "Registered firmware-seeded DRNG entropy source\n");
	} else
		device_printf(dev, "Diagnostic mode; entropy source not registered\n");
	return (0);
fail_res:
	bus_release_resource(dev, SYS_RES_MEMORY, rid, sc->res);
fail_clock:
	clk_disable(sc->clock);
fail:
	if (sc->clock != NULL)
		clk_release(sc->clock);
	mtx_destroy(&sc->lock);
	return (error);
}

static int
rkrng_detach(device_t dev)
{
	/* The entropy callback's singleton remains valid for the kernel lifetime. */
	return (EBUSY);
}
static device_method_t rkrng_methods[] = {
	DEVMETHOD(device_probe, rkrng_probe),
	DEVMETHOD(device_attach, rkrng_attach),
	DEVMETHOD(device_detach, rkrng_detach),
	DEVMETHOD_END
};
static driver_t rkrng_driver = {
	"rkrng", rkrng_methods, sizeof(struct rkrng_softc)
};
DRIVER_MODULE(rkrng, simplebus, rkrng_driver, 0, 0);
MODULE_DEPEND(rkrng, random_harvestq, 1, 1, 1);
