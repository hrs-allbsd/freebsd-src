/*-
 * SPDX-License-Identifier: BSD-2-Clause
 * Copyright (c) 2026 Hiroki Sato <hrs@FreeBSD.org>
 */
/* R28S temperature monitor with optional OTP trim and emergency TSHUT. */
#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/callout.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/rman.h>
#include <sys/sysctl.h>

#include <machine/bus.h>

#include <dev/clk/clk.h>
#include <dev/hwreset/hwreset.h>
#include <dev/nvmem/nvmem.h>
#include <dev/ofw/ofw_bus_subr.h>
#include <dev/syscon/syscon.h>

#include "syscon_if.h"

#define AUTO_CON      0x04
#define AUTO_SRC      0x0c
#define HT_INT_EN     0x14
#define SHUT_GPIO_EN  0x18
#define SHUT_CRU_EN   0x1c
#define COMP_SHUT0    0x10c
#define HSHUT_PD      0x28
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
	bool calibrated;
	int trim_millicelsius;
	u_int trim_word;
	bool protection, warning_active, monitor_started;
	int warning_millicelsius, shutdown_millicelsius;
	u_int shutdown_code, warnings;
	struct callout monitor;
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
			*temp -= sc->trim_millicelsius;
			return (0);
		}
	}
	return (ERANGE);
}
/* Round upwards like Board Support Package temp_to_code, after adding the factory offset. */
static int
rk3528_tsadc_trip_code(struct rk3528_tsadc_softc *sc, int corrected,
    u_int *code)
{
	int nominal = corrected + sc->trim_millicelsius;

	if (nominal < calibration[0].temp ||
	    nominal > calibration[nitems(calibration) - 1].temp)
		return (ERANGE);
	for (u_int i = 1; i < nitems(calibration); i++) {
		if (nominal <= calibration[i].temp) {
			int span = calibration[i].temp -
			    calibration[i - 1].temp;
			*code = calibration[i - 1].code +
			    howmany((nominal - calibration[i - 1].temp) *
				    (calibration[i].code -
					calibration[i - 1].code),
				span);
			return (0);
		}
	}
	return (ERANGE);
}

/* Called with the sensor lock held. Warning policy does not change CPU rate. */
static void
rk3528_tsadc_monitor(void *arg)
{
	struct rk3528_tsadc_softc *sc = arg;
	uint32_t raw;
	int temp;

	mtx_assert(&sc->lock, MA_OWNED);
	if (!sc->monitor_started)
		return;
	if (rk3528_tsadc_sample(sc, &raw, &temp) == 0) {
		if (!sc->warning_active && temp >= sc->warning_millicelsius) {
			sc->warning_active = true;
			sc->warnings++;
			device_printf(sc->dev,
			    "Temperature warning: %d mC >= %d mC\n", temp,
			    sc->warning_millicelsius);
		} else if (sc->warning_active &&
		    temp < sc->warning_millicelsius - 2000) {
			sc->warning_active = false;
			device_printf(sc->dev,
			    "Temperature warning cleared: %d mC\n", temp);
		}
	}
	callout_reset(&sc->monitor, hz, rk3528_tsadc_monitor, sc);
}

static int
rk3528_tsadc_warning_limit(SYSCTL_HANDLER_ARGS)
{
	struct rk3528_tsadc_softc *sc = arg1;
	int value, error;

	mtx_lock(&sc->lock);
	value = sc->warning_millicelsius;
	mtx_unlock(&sc->lock);
	error = sysctl_handle_int(oidp, &value, 0, req);
	if (error != 0 || req->newptr == NULL)
		return (error);
	if (value < 40000 || value > 110000 ||
	    value > sc->shutdown_millicelsius - 2000)
		return (EINVAL);
	mtx_lock(&sc->lock);
	sc->warning_millicelsius = value;
	mtx_unlock(&sc->lock);
	return (0);
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
	else if (arg2 == 2)
		temp += sc->trim_millicelsius;
	return (sysctl_handle_int(oidp, &temp, 0, req));
}
static int
rk3528_tsadc_snapshot(SYSCTL_HANDLER_ARGS)
{
	struct rk3528_tsadc_softc *sc = arg1;
	uint32_t raw;
	int temp, error;
	char text[384];

	mtx_lock(&sc->lock);
	error = rk3528_tsadc_sample(sc, &raw, &temp);
	snprintf(text, sizeof(text),
	    "raw=%u valid=%d millicelsius=%d nominal=%d calibrated=%d trim_word=%u correction=%d auto=%x qmax=%u reads=%ju invalid=%ju protection=%d tshut_code=%u cru=%x gpio=%x source=%x",
	    raw, error == 0, error == 0 ? temp : 0,
	    error == 0 ? temp + sc->trim_millicelsius : 0, sc->calibrated,
	    sc->trim_word, sc->trim_millicelsius, bus_read_4(sc->res, AUTO_CON),
	    bus_read_4(sc->res, Q_MAX), (uintmax_t)sc->reads,
	    (uintmax_t)sc->invalid, sc->protection,
	    bus_read_4(sc->res, COMP_SHUT0), bus_read_4(sc->res, SHUT_CRU_EN),
	    bus_read_4(sc->res, SHUT_GPIO_EN), bus_read_4(sc->res, AUTO_SRC));
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
	if (sc->monitor_started) {
		mtx_lock(&sc->lock);
		sc->monitor_started = false;
		mtx_unlock(&sc->lock);
		callout_drain(&sc->monitor);
	}
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
/* Board Support Package defaults: trim base 30 C, fractional part zero.
 * Positive correction is subtracted from the nominal table temperature.
 */
static int
rk3528_tsadc_load_trim(struct rk3528_tsadc_softc *sc)
{
	phandle_t node = ofw_bus_get_node(sc->dev);
	uint8_t low, high;
	int error;

	if (!OF_hasprop(node, "nvmem-cells"))
		return (
		    OF_hasprop(node, "freebsd,require-otp-trim") ? ENOENT : 0);
	if (nvmem_get_cell_len(node, "trim_l") != 1 ||
	    nvmem_get_cell_len(node, "trim_h") != 1)
		return (EINVAL);
	error = nvmem_read_cell_by_name(node, "trim_l", &low, 1);
	if (error != 0)
		return (error);
	error = nvmem_read_cell_by_name(node, "trim_h", &high, 1);
	if (error != 0)
		return (error);
	sc->trim_word = ((u_int)high << 8) | low;
	if (sc->trim_word == 0 || sc->trim_word > 2047)
		return (EINVAL);
	if (2047 - sc->trim_word < calibration[0].code ||
	    2047 - sc->trim_word > calibration[nitems(calibration) - 1].code)
		return (ERANGE);
	sc->trim_millicelsius = 574 * (2047 - (int)sc->trim_word - 1533);
	sc->calibrated = true;
	return (0);
}

static int
rk3528_tsadc_protect(struct rk3528_tsadc_softc *sc)
{
	phandle_t node = ofw_bus_get_node(sc->dev);
	pcell_t setting;
	int error;

	if (!OF_hasprop(node, "rockchip,hw-tshut-temp"))
		return (0);
	if (!sc->calibrated ||
	    OF_getencprop(node, "rockchip,hw-tshut-temp", &setting,
		sizeof(setting)) != sizeof(setting) ||
	    setting < 110000 || setting > 120000)
		return (EINVAL);
	sc->shutdown_millicelsius = setting;
	sc->warning_millicelsius = 95000;
	if (OF_getencprop(node, "rockchip,hw-tshut-mode", &setting,
		sizeof(setting)) != sizeof(setting) ||
	    setting != 0)
		return (EINVAL);
	if (OF_getencprop(node, "rockchip,hw-tshut-polarity", &setting,
		sizeof(setting)) != sizeof(setting) ||
	    setting != 0)
		return (EINVAL);
	error = rk3528_tsadc_trip_code(sc, sc->shutdown_millicelsius,
	    &sc->shutdown_code);
	if (error != 0)
		return (error);
	bus_write_4(sc->res, COMP_SHUT0, sc->shutdown_code);
	bus_write_4(sc->res, HSHUT_PD, 1);
	bus_write_4(sc->res, SHUT_CRU_EN, 0x10001);
	if (bus_read_4(sc->res, COMP_SHUT0) != sc->shutdown_code ||
	    (bus_read_4(sc->res, SHUT_CRU_EN) & 1) == 0 ||
	    (bus_read_4(sc->res, SHUT_GPIO_EN) & 1) != 0 ||
	    (bus_read_4(sc->res, AUTO_SRC) & 1) == 0)
		return (EIO);
	sc->protection = true;
	return (0);
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
	if (OF_hasprop(ofw_bus_get_node(dev), "freebsd,boot-tshut-test"))
		return (EPERM);
	error = rk3528_tsadc_load_trim(sc);
	if (error != 0) {
		device_printf(dev, "OTP trim unavailable: %d\n", error);
		return (error);
	}
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
	error = rk3528_tsadc_protect(sc);
	if (error != 0)
		goto fail;
	if (sc->protection) {
		SYSCTL_ADD_PROC(ctx, tree, OID_AUTO, "warning_millicelsius",
		    CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_MPSAFE, sc, 0,
		    rk3528_tsadc_warning_limit, "I",
		    "Software warning threshold; 2 C hysteresis");
		SYSCTL_ADD_BOOL(ctx, tree, OID_AUTO, "warning_active",
		    CTLFLAG_RD, &sc->warning_active, 0,
		    "Software temperature warning active");
		SYSCTL_ADD_UINT(ctx, tree, OID_AUTO, "warnings", CTLFLAG_RD,
		    &sc->warnings, 0, "Software warning transitions");
		SYSCTL_ADD_INT(ctx, tree, OID_AUTO, "shutdown_millicelsius",
		    CTLFLAG_RD, &sc->shutdown_millicelsius, 0,
		    "Hardware emergency CRU reset threshold");
		SYSCTL_ADD_UINT(ctx, tree, OID_AUTO, "shutdown_code",
		    CTLFLAG_RD, &sc->shutdown_code, 0,
		    "Factory-corrected hardware reset comparator code");
		callout_init_mtx(&sc->monitor, &sc->lock, 0);
		sc->monitor_started = true;
		callout_reset(&sc->monitor, hz, rk3528_tsadc_monitor, sc);
		device_printf(dev,
		    "Hardware CRU TSHUT=%d mC code=%u; software warning=%d mC\n",
		    sc->shutdown_millicelsius, sc->shutdown_code,
		    sc->warning_millicelsius);
	}
	SYSCTL_ADD_PROC(ctx, tree, OID_AUTO, "temperature",
	    CTLTYPE_INT | CTLFLAG_RD | CTLFLAG_MPSAFE, sc, 1,
	    rk3528_tsadc_temperature, "IK", "CPU temperature");
	SYSCTL_ADD_PROC(ctx, tree, OID_AUTO, "temperature_millicelsius",
	    CTLTYPE_INT | CTLFLAG_RD | CTLFLAG_MPSAFE, sc, 0,
	    rk3528_tsadc_temperature, "I", "CPU temperature (mC)");
	SYSCTL_ADD_PROC(ctx, tree, OID_AUTO, "snapshot",
	    CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_MPSAFE, sc, 0,
	    rk3528_tsadc_snapshot, "A",
	    "TSADC monitor state and applied OTP correction");
	SYSCTL_ADD_PROC(ctx, tree, OID_AUTO, "nominal_temperature_millicelsius",
	    CTLTYPE_INT | CTLFLAG_RD | CTLFLAG_MPSAFE, sc, 2,
	    rk3528_tsadc_temperature, "I",
	    "Temperature before OTP correction (mC)");
	SYSCTL_ADD_BOOL(ctx, tree, OID_AUTO, "calibrated", CTLFLAG_RD,
	    &sc->calibrated, 0, "Individual OTP trim applied");
	SYSCTL_ADD_INT(ctx, tree, OID_AUTO, "trim_millicelsius", CTLFLAG_RD,
	    &sc->trim_millicelsius, 0,
	    "OTP correction subtracted from nominal temperature");
	SYSCTL_ADD_UINT(ctx, tree, OID_AUTO, "trim_word", CTLFLAG_RD,
	    &sc->trim_word, 0, "Individual OTP trim word");
	device_printf(dev,
	    "raw=%u temperature=%d mC; OTP calibrated=%d correction=%d mC; protection=%d\n",
	    raw, temp, sc->calibrated, sc->trim_millicelsius, sc->protection);
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
