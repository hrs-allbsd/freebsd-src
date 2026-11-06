/*-
 * SPDX-License-Identifier: BSD-2-Clause
 * Copyright (c) 2026 Hiroki Sato <hrs@FreeBSD.org>
 */
/* R28S nominal temperature monitoring; no trim or shutdown policy. */
#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/rman.h>
#include <sys/sysctl.h>

#include <machine/bus.h>

#include <dev/clk/clk.h>
#include <dev/hwreset/hwreset.h>
#include <dev/ofw/ofw_bus_subr.h>
#include <dev/syscon/syscon.h>

#include "syscon_if.h"

#define AUTO_CON      0x04
#define AUTO_SRC      0x0c
#define HT_INT_EN     0x14
#define SHUT_GPIO_EN  0x18
#define SHUT_CRU_EN   0x1c
#define DATA0	      0x2c
#define INT_DEBOUNCE  0x14c
#define SHUT_DEBOUNCE 0x150
#define PERIOD	      0x154
#define PERIOD_HT     0x158
#define Q_MAX	      0x210
#define GRF_TSADC_CON 0x40030

/* Manufacturer Board Support Package rk3528_code_table, excluding out-of-range sentinels. */
static const struct {
	uint16_t code;
	int temp;
} calibration[] = {
	{ 1410, -40000 },
	{ 1419, -35000 },
	{ 1428, -30000 },
	{ 1436, -25000 },
	{ 1445, -20000 },
	{ 1454, -15000 },
	{ 1463, -10000 },
	{ 1471, -5000 },
	{ 1480, 0 },
	{ 1489, 5000 },
	{ 1498, 10000 },
	{ 1506, 15000 },
	{ 1515, 20000 },
	{ 1524, 25000 },
	{ 1533, 30000 },
	{ 1541, 35000 },
	{ 1550, 40000 },
	{ 1558, 45000 },
	{ 1567, 50000 },
	{ 1575, 55000 },
	{ 1584, 60000 },
	{ 1593, 65000 },
	{ 1602, 70000 },
	{ 1610, 75000 },
	{ 1619, 80000 },
	{ 1628, 85000 },
	{ 1637, 90000 },
	{ 1646, 95000 },
	{ 1654, 100000 },
	{ 1663, 105000 },
	{ 1672, 110000 },
	{ 1680, 115000 },
	{ 1689, 120000 },
	{ 1697, 125000 },
};
struct rk3528_tsadc_softc {
	device_t dev;
	struct resource *res;
	struct syscon *grf;
	clk_t clocks[3];
	bool enabled[3], active;
	hwreset_array_t resets;
	struct mtx lock;
	uint64_t reads, invalid;

};
static const char *clock_names[] = { "apb_pclk", "tsadc", "tsadc_tsen" };

static int
rk3528_tsadc_sample(struct rk3528_tsadc_softc *sc, uint32_t *raw, int *temp)
{
	u_int i;

	*raw = bus_read_4(sc->res, DATA0);
	sc->reads++;
	if (*raw < calibration[0].code ||
	    *raw > calibration[nitems(calibration) - 1].code) {
		sc->invalid++;
		return (EAGAIN);
	}
	for (i = 1; i < nitems(calibration); i++) {
		if (*raw <= calibration[i].code) {
			*temp = calibration[i - 1].temp +
			    ((int)*raw - calibration[i - 1].code) *
				(calibration[i].temp -
				    calibration[i - 1].temp) /
				(calibration[i].code - calibration[i - 1].code);
			return (0);
		}
	}
	return (ERANGE);
}
static int
rk3528_tsadc_temperature(SYSCTL_HANDLER_ARGS)
{
	struct rk3528_tsadc_softc *sc = arg1;
	uint32_t raw;
	int temp, error;

	mtx_lock(&sc->lock);
	error = rk3528_tsadc_sample(sc, &raw, &temp);
	mtx_unlock(&sc->lock);
	if (error != 0)
		return (error);
	if (arg2 == 1)
		temp = (temp + 273150) / 100;
	return (sysctl_handle_int(oidp, &temp, 0, req));
}
static int
rk3528_tsadc_snapshot(SYSCTL_HANDLER_ARGS)
{
	struct rk3528_tsadc_softc *sc = arg1;
	uint32_t raw;
	int temp, error;
	char text[192];

	mtx_lock(&sc->lock);
	error = rk3528_tsadc_sample(sc, &raw, &temp);
	snprintf(text, sizeof(text),
	    "raw=%u valid=%d millicelsius=%d auto=%x qmax=%u reads=%ju invalid=%ju",
	    raw, error == 0, error == 0 ? temp : 0,
	    bus_read_4(sc->res, AUTO_CON), bus_read_4(sc->res, Q_MAX),
	    (uintmax_t)sc->reads, (uintmax_t)sc->invalid);
	mtx_unlock(&sc->lock);
	return (sysctl_handle_string(oidp, text, 0, req));
}
static int
rk3528_tsadc_probe(device_t dev)
{
	if (!ofw_bus_status_okay(dev) ||
	    !ofw_bus_is_compatible(dev, "freebsd,rk3528-r28s-tsadc"))
		return (ENXIO);
	device_set_desc(dev, "RK3528 temperature monitor");
	return (BUS_PROBE_DEFAULT);
}
static void
rk3528_tsadc_release(struct rk3528_tsadc_softc *sc)
{
	if (sc->active) {
		bus_write_4(sc->res, SHUT_CRU_EN, 1u << 16);
		bus_write_4(sc->res, AUTO_CON, 1u << 16);
	}
	for (int i = 2; i >= 0; i--) {
		if (sc->enabled[i])
			clk_disable(sc->clocks[i]);
		if (sc->clocks[i] != NULL)
			clk_release(sc->clocks[i]);
	}
	if (sc->resets != NULL)
		hwreset_array_release(sc->resets);
	if (sc->res != NULL)
		bus_release_resource(sc->dev, SYS_RES_MEMORY, 0, sc->res);
	mtx_destroy(&sc->lock);
}
static int
rk3528_tsadc_attach(device_t dev)
{
	struct rk3528_tsadc_softc *sc = device_get_softc(dev);
	struct sysctl_ctx_list *ctx = device_get_sysctl_ctx(dev);
	struct sysctl_oid_list *tree = SYSCTL_CHILDREN(
	    device_get_sysctl_tree(dev));
	uint64_t frequency;
	uint32_t raw;
	int error, rid = 0, temp;

	sc->dev = dev;
	/* Do not silently ignore calibration or protection requests. */
	if (OF_hasprop(ofw_bus_get_node(dev), "nvmem-cells") ||
	    OF_hasprop(ofw_bus_get_node(dev), "freebsd,require-otp-trim") ||
	    OF_hasprop(ofw_bus_get_node(dev), "rockchip,hw-tshut-temp") ||
	    OF_hasprop(ofw_bus_get_node(dev), "freebsd,boot-tshut-test"))
		return (EOPNOTSUPP);
	mtx_init(&sc->lock, "r28s_tsadc", NULL, MTX_DEF);
	sc->res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid, RF_ACTIVE);
	if (sc->res == NULL) {
		error = ENXIO;
		goto fail;
	}
	if (rman_get_size(sc->res) < Q_MAX + sizeof(uint32_t)) {
		error = ENXIO;
		goto fail;
	}
	error = syscon_get_by_ofw_property(dev, ofw_bus_get_node(dev),
	    "rockchip,grf", &sc->grf);
	if (error != 0)
		goto fail;
	error = hwreset_array_get_ofw(dev, 0, &sc->resets);
	if (error != 0)
		goto fail;
	for (int i = 0; i < 3; i++) {
		error = clk_get_by_ofw_name(dev, 0, clock_names[i],
		    &sc->clocks[i]);
		if (error != 0)
			goto fail;
	}
	error = hwreset_array_assert(sc->resets);
	if (error != 0)
		goto fail;
	for (int i = 1; i < 3; i++) {
		frequency = i == 1 ? 1200000 : 12000000;
		error = clk_set_freq(sc->clocks[i], frequency, 0);
		if (error != 0)
			goto fail;
		error = clk_get_freq(sc->clocks[i], &frequency);
		if (error != 0 || frequency != (i == 1 ? 1200000 : 12000000)) {
			error = EINVAL;
			goto fail;
		}
	}
	for (int i = 0; i < 3; i++) {
		error = clk_enable(sc->clocks[i]);
		if (error != 0)
			goto fail;
		sc->enabled[i] = true;
	}
	DELAY(10);
	error = hwreset_array_deassert(sc->resets);
	if (error != 0)
		goto fail;
	/* Board Support Package rk_tsadcv11_initialize: 2.5 ms period at 1.2 MHz. */
	bus_write_4(sc->res, PERIOD, 3000);
	bus_write_4(sc->res, PERIOD_HT, 3000);
	bus_write_4(sc->res, INT_DEBOUNCE, 4);
	bus_write_4(sc->res, SHUT_DEBOUNCE, 4);
	bus_write_4(sc->res, Q_MAX, 2047);
	bus_write_4(sc->res, AUTO_CON, (1u << 17) | (1u << 1));
	bus_write_4(sc->res, AUTO_CON, 1u << 24);
	SYSCON_WRITE_4(sc->grf, GRF_TSADC_CON, 0x10001u << 8);
	DELAY(15);
	SYSCON_WRITE_4(sc->grf, GRF_TSADC_CON, 0x10001u << 0);
	SYSCON_WRITE_4(sc->grf, GRF_TSADC_CON, 0x10001u << 1);
	SYSCON_WRITE_4(sc->grf, GRF_TSADC_CON, 0x10001u << 2);
	DELAY(100);
	/* Monitor only: no alarm interrupt or CRU/GPIO shutdown route. */
	bus_write_4(sc->res, HT_INT_EN, 1u << 16);
	bus_write_4(sc->res, SHUT_GPIO_EN, 1u << 16);
	bus_write_4(sc->res, SHUT_CRU_EN, 1u << 16);
	bus_write_4(sc->res, AUTO_SRC, 0x10001);
	bus_write_4(sc->res, AUTO_CON, 0x10001);
	sc->active = true;
	for (int i = 0; i < 100; i++) {
		error = rk3528_tsadc_sample(sc, &raw, &temp);
		if (error == 0)
			break;
		DELAY(1000);
	}
	if (error != 0)
		goto fail;
	SYSCTL_ADD_PROC(ctx, tree, OID_AUTO, "temperature",
	    CTLTYPE_INT | CTLFLAG_RD | CTLFLAG_MPSAFE, sc, 1,
	    rk3528_tsadc_temperature, "IK", "CPU temperature");
	SYSCTL_ADD_PROC(ctx, tree, OID_AUTO, "temperature_millicelsius",
	    CTLTYPE_INT | CTLFLAG_RD | CTLFLAG_MPSAFE, sc, 0,
	    rk3528_tsadc_temperature, "I", "CPU temperature (mC)");
	SYSCTL_ADD_PROC(ctx, tree, OID_AUTO, "snapshot",
	    CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_MPSAFE, sc, 0,
	    rk3528_tsadc_snapshot, "A",
	    "Nominal TSADC monitor state");
	device_printf(dev, "raw=%u nominal temperature=%d mC; monitor only\n",
	    raw, temp);
	return (0);
fail:
	device_printf(dev, "attach failed: %d\n", error);
	rk3528_tsadc_release(sc);
	return (error);
}
static int
rk3528_tsadc_detach(device_t dev)
{
	return (EBUSY);
}
static device_method_t rk3528_tsadc_methods[] = {
	DEVMETHOD(device_probe, rk3528_tsadc_probe),
	DEVMETHOD(device_attach, rk3528_tsadc_attach),
	DEVMETHOD(device_detach, rk3528_tsadc_detach),
	DEVMETHOD_END
};
static driver_t rk3528_tsadc_driver = {
	"rk3528_tsadc", rk3528_tsadc_methods,
	sizeof(struct rk3528_tsadc_softc)
};
DRIVER_MODULE(rk3528_tsadc, simplebus, rk3528_tsadc_driver, 0, 0);
MODULE_VERSION(rk3528_tsadc, 1);
