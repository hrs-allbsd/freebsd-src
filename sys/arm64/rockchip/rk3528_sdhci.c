/*-
 * SPDX-License-Identifier: BSD-2-Clause
 * Copyright (c) 2026 Hiroki Sato <hrs@FreeBSD.org>
 */
/* RK3528 eMMC: legacy/HS SDR/DDR PIO/SDMA through the standard MMC stack. */
#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/rman.h>
#include <sys/sysctl.h>
#include <sys/taskqueue.h>

#include <machine/bus.h>

#include <dev/clk/clk.h>
#include <dev/fdt/fdt_pinctrl.h>
#include <dev/hwreset/hwreset.h>
#include <dev/mmc/bridge.h>
#include <dev/mmc/mmcbrvar.h>
#include <dev/ofw/ofw_bus_subr.h>
#include <dev/phy/phy.h>
#include <dev/sdhci/sdhci.h>
#include <dev/sdhci/sdhci_fdt.h>
#include <dev/syscon/syscon.h>

#include "mmcbr_if.h"
#include "sdhci_if.h"
#include "syscon_if.h"

/* Qualified fixed delay in DLL bypass; do not derive taps from an unlocked DLL. */
#define RK3528_DDR_TXCLK ((1U << 27) | (1U << 26) | (1U << 29) | (64U << 16))

struct rk3528_sdhci_softc {
	/* Inherited register and MMC bridge methods expect this first. */
	struct sdhci_fdt_softc fdt;
	clk_t clocks[5];
	hwreset_t resets[5];
	unsigned enabled;
	uint32_t vendor;
	uint64_t core_hz;
	bool use_sdma, use_hs, use_ddr;
	unsigned fast_hz;
	uint64_t dma_requests, pio_requests, dma_bytes, dma_addresses;
	uint64_t dma_address_errors, stop_commands;
};

static void
rk3528_sdhci_bypass(struct rk3528_sdhci_softc *sc)
{
	struct resource *r = sc->fdt.mem_res[0];
	uint32_t v;

	v = bus_read_4(r, sc->vendor + 8);
	bus_write_4(r, sc->vendor + 8, (v & ~1U) | (1U << 4));
	bus_write_4(r, 0x800, 0);
	bus_write_4(r, 0x800, (1U << 24) | 1U);
	bus_write_4(r, 0x804, 1U << 31);
	bus_write_4(r, 0x808,
	    sc->fdt.slots[0].host.ios.timing == bus_timing_mmc_ddr52 ?
	    RK3528_DDR_TXCLK : 0);
	bus_write_4(r, 0x80c,
	    sc->fdt.slots[0].host.ios.timing == bus_timing_mmc_ddr52 ?
	    (1U << 27) | (1U << 26) | (10U << 16) : 0);
	bus_write_4(r, 0x810, 0);
}

static void
rk3528_sdhci_reset(device_t dev, struct sdhci_slot *slot, uint8_t mask)
{
	struct rk3528_sdhci_softc *sc = device_get_softc(dev);
	struct resource *r = sc->fdt.mem_res[0];

	sdhci_generic_reset(dev, slot, mask);
	if ((bus_read_1(r, SDHCI_SOFTWARE_RESET) & mask) != 0)
		return;
	/* Software reset clears the RK3528 internal clock enable. */
	bus_write_4(r, 0x81c, bus_read_4(r, 0x81c) | (1U << 1));
	/* A command-line reset after a response timeout can latch Command
	 * Complete on RK3528.  It belongs to the aborted command, not the
	 * next request.  Retire command status only after reset completes;
	 * leave data, card and power status for their normal handlers.
	 */
	if (mask & SDHCI_RESET_CMD) {
		bus_write_4(r, SDHCI_INT_STATUS, SDHCI_INT_CMD_MASK);
		(void)bus_read_4(r, SDHCI_INT_STATUS);
	}
	if (mask & SDHCI_RESET_ALL) {
		rk3528_sdhci_bypass(sc);
		bus_write_2(r, SDHCI_HOST_CONTROL2, SDHCI_CTRL2_S18_ENABLE);
	}
}

static uint32_t
rk3528_sdhci_read_4(device_t dev, struct sdhci_slot *slot, bus_size_t off)
{
	struct rk3528_sdhci_softc *sc = device_get_softc(dev);
	uint32_t v = bus_read_4(sc->fdt.mem_res[slot->num], off);

	if (off == SDHCI_CAPABILITIES)
		v &= ~(SDHCI_CLOCK_V3_BASE_MASK |
		    SDHCI_CAN_DO_ADMA2 | SDHCI_CAN_DO_8BITBUS);
	if (off == SDHCI_CAPABILITIES && !sc->use_hs)
		v &= ~SDHCI_CAN_DO_HISPD;
	if (off == SDHCI_CAPABILITIES && !sc->use_sdma)
		v &= ~SDHCI_CAN_DO_DMA;
	if (off == SDHCI_CAPABILITIES2)
		v = 0;
	return (v);
}

static void
rk3528_sdhci_write_2(device_t dev, struct sdhci_slot *slot, bus_size_t off,
    uint16_t value)
{
	struct rk3528_sdhci_softc *sc = device_get_softc(dev);

	/* In the low-speed DLL bypass path, the CRU leaf determines the
	 * effective rate.  Keep the verified nonzero SDR controller encoding;
	 * changing this field alone did not change SDR throughput on RK3528.
	 * DDR follows the BSP zero-divider encoding.
	 */
	if (off == SDHCI_CLOCK_CONTROL && (value & SDHCI_CLOCK_INT_EN)) {
		value &= ~((SDHCI_DIVIDER_MASK << SDHCI_DIVIDER_SHIFT) |
		    (SDHCI_DIVIDER_HI_MASK << SDHCI_DIVIDER_HI_SHIFT));
		if (!sc->use_ddr ||
		    slot->host.ios.timing != bus_timing_mmc_ddr52)
			value |= 1U << SDHCI_DIVIDER_SHIFT;
	}
	if (off == SDHCI_TRANSFER_MODE && slot->curcmd != NULL &&
	    slot->curcmd->data != NULL) {
		if (value & SDHCI_TRNS_DMA) {
			sc->dma_requests++;
			sc->dma_bytes += slot->curcmd->data->len;
		} else
			sc->pio_requests++;
	}
	if (off == SDHCI_COMMAND_FLAGS && (value >> 8) == MMC_STOP_TRANSMISSION)
		sc->stop_commands++;
	/* NanoPi R28S has fixed 3.3V VCC and 1.8V VCCQ rails. */
	if (off == SDHCI_HOST_CONTROL2)
		value |= SDHCI_CTRL2_S18_ENABLE;
	bus_write_2(sc->fdt.mem_res[slot->num], off, value);
}

static void
rk3528_sdhci_write_4(device_t dev, struct sdhci_slot *slot,
    bus_size_t off, uint32_t value)
{
	struct rk3528_sdhci_softc *sc = device_get_softc(dev);

	if (off == SDHCI_DMA_ADDRESS) {
		sc->dma_addresses++;
		if (value != slot->paddr)
			sc->dma_address_errors++;
	}
	bus_write_4(sc->fdt.mem_res[slot->num], off, value);
}

/* Qualified SDR fast rate, otherwise exact xin24m divisors. */
static unsigned
rk3528_sdhci_rate(uintptr_t requested, unsigned fast_hz)
{
	unsigned rate;

	if (requested == 0)
		return (0);
	if (fast_hz > 24000000 && requested >= fast_hz)
		return (fast_hz);
	for (rate = 24000000; rate > 375000 && rate > requested; rate /= 2)
		;
	return (rate);
}

/* Hardware uses integer divisors; the clock API reports truncated Hz. */
static unsigned
rk3528_sdhci_fast_rate(uint64_t parent_hz, unsigned ceiling)
{
	unsigned div;

	for (div = 1; div <= 64; div++) {
		if (parent_hz <= (uint64_t)MIN(ceiling, 52000000) * div &&
		    parent_hz / div > 24000000)
			return (parent_hz / div);
	}
	return (0);
}

static int
rk3528_sdhci_status(SYSCTL_HANDLER_ARGS)
{
	struct rk3528_sdhci_softc *sc = arg1;
	struct sdhci_slot *slot = &sc->fdt.slots[0];
	char text[512];
	bool dma, bounds;

	mtx_lock(&slot->mtx);
	dma = (slot->opt & SDHCI_HAVE_DMA) != 0;
	bounds = dma && slot->paddr != 0 && slot->paddr <= UINT32_MAX &&
	    slot->sdma_bbufsz <= UINT32_MAX - slot->paddr + 1;
	snprintf(text, sizeof(text),
	    "core=%ju ios=%u clock=%#x sdma=%u buf=%u bounds=%u "
	    "dma_requests=%ju pio_requests=%ju dma_bytes=%ju "
	    "dma_addresses=%ju address_errors=%ju active=%u "
	    "timing=%u vccq=%u host2=%#x emmc=%#x tx=%#x stops=%ju",
	    (uintmax_t)sc->core_hz, slot->host.ios.clock,
	    bus_read_2(sc->fdt.mem_res[0], SDHCI_CLOCK_CONTROL), dma,
	    slot->sdma_bbufsz, bounds, (uintmax_t)sc->dma_requests,
	    (uintmax_t)sc->pio_requests, (uintmax_t)sc->dma_bytes,
	    (uintmax_t)sc->dma_addresses, (uintmax_t)sc->dma_address_errors,
	    slot->req != NULL, slot->host.ios.timing, slot->host.ios.vccq,
	    bus_read_2(sc->fdt.mem_res[0], SDHCI_HOST_CONTROL2),
	    bus_read_2(sc->fdt.mem_res[0], sc->vendor + 0x2c),
	    bus_read_4(sc->fdt.mem_res[0], 0x808),
	    (uintmax_t)sc->stop_commands);
	mtx_unlock(&slot->mtx);
	return (sysctl_handle_string(oidp, text, 0, req));
}

/* Called with the SDHCI mutex held: register access only, no clock locks. */
static int
rk3528_sdhci_set_clock(device_t dev, struct sdhci_slot *slot, int clock)
{
	struct rk3528_sdhci_softc *sc = device_get_softc(dev);

	if (clock == 0)
		return (0);
	rk3528_sdhci_bypass(sc);
	return (MIN((unsigned)clock, sc->core_hz));
}

/* This board's VCCQ is physically fixed at 1.8V; no rail switch occurs. */
static int
rk3528_sdhci_switch_vccq(device_t dev, device_t child)
{
	struct rk3528_sdhci_softc *sc = device_get_softc(dev);
	struct sdhci_slot *slot = device_get_ivars(child);

	if (!sc->use_ddr)
		return (0); /* Preserve the legacy profile's logical requests. */
	return (slot->host.ios.vccq == vccq_180 ? 0 : EOPNOTSUPP);
}

/* The mode follows timing, including DDR at identification/transition rates. */
static void
rk3528_sdhci_set_uhs_timing(device_t dev, struct sdhci_slot *slot)
{
	struct rk3528_sdhci_softc *sc = device_get_softc(dev);
	struct resource *r = sc->fdt.mem_res[slot->num];
	uint16_t clock, mode;

	if (!sc->use_ddr)
		return;
	mtx_assert(&slot->mtx, MA_OWNED);
	/* Use the standard explicit STOP path, including error recovery. */
	slot->quirks |= SDHCI_QUIRK_BROKEN_AUTO_STOP;
	bus_write_2(r, sc->vendor + 0x2c,
	    (bus_read_2(r, sc->vendor + 0x2c) & ~2U) | 1U);
	clock = bus_read_2(r, SDHCI_CLOCK_CONTROL);
	bus_write_2(r, SDHCI_CLOCK_CONTROL, clock & ~SDHCI_CLOCK_CARD_EN);
	mode = bus_read_2(r, SDHCI_HOST_CONTROL2) & ~SDHCI_CTRL2_UHS_MASK;
	if (slot->host.ios.timing == bus_timing_mmc_ddr52)
		mode |= SDHCI_CTRL2_UHS_DDR50;
	else if (slot->host.ios.timing == bus_timing_hs)
		mode |= SDHCI_CTRL2_UHS_SDR25;
	rk3528_sdhci_write_2(dev, slot, SDHCI_HOST_CONTROL2, mode);
	rk3528_sdhci_write_2(dev, slot, SDHCI_CLOCK_CONTROL, clock);
}

static int
rk3528_sdhci_update_ios(device_t dev, device_t child)
{
	struct rk3528_sdhci_softc *sc = device_get_softc(dev);
	struct sdhci_slot *slot = device_get_ivars(child);
	uint64_t hz, actual;
	int error;

	/* MMC host ownership serializes IOS changes with requests.  Clock
	 * topology uses a sleepable lock, so never call it under slot->mtx.
	 */
	hz = slot->host.ios.clock;
	if (slot->host.ios.clock != 0 && hz != sc->core_hz) {
		mtx_lock(&slot->mtx);
		bus_write_2(sc->fdt.mem_res[slot->num], SDHCI_CLOCK_CONTROL, 0);
		slot->clock = 0;
		sc->core_hz = 0;
		mtx_unlock(&slot->mtx);
		error = clk_set_freq(sc->clocks[0], hz, 0);
		if (error != 0)
			return (error);
		if (clk_get_freq(sc->clocks[0], &actual) != 0 || actual != hz)
			return (ERANGE);
		mtx_lock(&slot->mtx);
		sc->core_hz = hz;
		slot->max_clk = hz;
		mtx_unlock(&slot->mtx);
	}
	return (sdhci_generic_update_ios(dev, child));
}

/* The generic ivar setter assumes a fixed SDHCI divider input.  This
 * frontend selects a supported CRU leaf rate before update_ios instead.
 */
static int
rk3528_sdhci_write_ivar(device_t dev, device_t child, int which,
    uintptr_t value)
{
	struct sdhci_slot *slot = device_get_ivars(child);
	struct rk3528_sdhci_softc *sc = device_get_softc(dev);

	if (which != MMCBR_IVAR_CLOCK)
		return (sdhci_generic_write_ivar(dev, child, which, value));
	slot->host.ios.clock = rk3528_sdhci_rate(value, sc->fast_hz);
	return (0);
}

/* Avoid SDMA boundary restarts and exact boundary endings on this host.
 * Upper layers split larger I/O into half-buffer transactions.
 */
static int
rk3528_sdhci_read_ivar(device_t dev, device_t child, int which,
    uintptr_t *result)
{
	struct sdhci_slot *slot = device_get_ivars(child);

	if (which == MMCBR_IVAR_MAX_DATA &&
	    (slot->opt & SDHCI_HAVE_DMA) != 0) {
		*result = MIN(65535, slot->sdma_bbufsz / 2 / 512);
		return (0);
	}
	return (sdhci_generic_read_ivar(dev, child, which, result));
}

static uint32_t
rk3528_sdhci_min_freq(device_t dev, struct sdhci_slot *slot)
{
	return (375000);
}

static void
rk3528_sdhci_intr(void *arg)
{
	struct rk3528_sdhci_softc *sc = arg;

	sdhci_generic_intr(&sc->fdt.slots[0]);
}

static void
rk3528_sdhci_release(device_t dev)
{
	struct rk3528_sdhci_softc *sc = device_get_softc(dev);
	struct sdhci_fdt_softc *f = &sc->fdt;

	if (f->num_slots != 0) {
		bus_write_4(f->mem_res[0], SDHCI_SIGNAL_ENABLE, 0);
		bus_write_4(f->mem_res[0], SDHCI_INT_ENABLE, 0);
	}
	if (f->intrhand != NULL)
		bus_teardown_intr(dev, f->irq_res, f->intrhand);
	if (f->num_slots != 0)
		sdhci_cleanup_slot(&f->slots[0]);
	if (f->irq_res != NULL)
		bus_release_resource(dev, SYS_RES_IRQ, 0, f->irq_res);
	if (f->mem_res[0] != NULL)
		bus_release_resource(dev, SYS_RES_MEMORY, 0, f->mem_res[0]);
	for (unsigned i = 0; i < 5; i++)
		if (sc->resets[i] != NULL)
			hwreset_release(sc->resets[i]);
	while (sc->enabled != 0)
		clk_disable(sc->clocks[--sc->enabled]);
	for (unsigned i = 0; i < 5; i++)
		if (sc->clocks[i] != NULL)
			clk_release(sc->clocks[i]);
}

static int
rk3528_sdhci_probe(device_t dev)
{
	phandle_t node = ofw_bus_get_node(dev);

	if (!ofw_bus_status_okay(dev) ||
	    !ofw_bus_is_compatible(dev, "rockchip,rk3528-dwcmshc") ||
	    OF_hasprop(node, "freebsd,empty-socket-test") ||
	    OF_hasprop(node, "freebsd,read-only-identify") ||
	    !ofw_bus_node_is_compatible(OF_finddevice("/"),
		"friendlyelec,nanopi-r28s"))
		return (ENXIO);
	device_set_desc(dev, "RK3528 eMMC SDHCI (legacy PIO/SDMA)");
	return (BUS_PROBE_DEFAULT);
}

static int
rk3528_sdhci_attach(device_t dev)
{
	static char *names[] = { "core", "bus", "axi", "block", "timer" };
	struct rk3528_sdhci_softc *sc = device_get_softc(dev);
	struct sdhci_fdt_softc *f = &sc->fdt;
	struct sdhci_slot *slot = &f->slots[0];
	clk_t divider = NULL, oscillator = NULL, fast_parent = NULL;
	uint64_t parent_hz;
	struct syscon *ioc;
	int error, rid;
	uint32_t bus_width = 1, max_frequency = 24000000;

	if (OF_hasprop(ofw_bus_get_node(dev), "max-frequency") &&
	    OF_getencprop(ofw_bus_get_node(dev), "max-frequency", &max_frequency,
	    sizeof(max_frequency)) != sizeof(max_frequency))
		return (EINVAL);
	if (max_frequency < 375000 || max_frequency > 52000000)
		return (EINVAL);
	/* This is the wired maximum; MMC verifies the usable width. */
	if (OF_hasprop(ofw_bus_get_node(dev), "bus-width") &&
	    OF_getencprop(ofw_bus_get_node(dev), "bus-width", &bus_width,
	    sizeof(bus_width)) != sizeof(bus_width))
		return (EINVAL);
	if (bus_width != 1 && bus_width != 4 && bus_width != 8)
		return (EINVAL);
	sc->use_hs = OF_hasprop(ofw_bus_get_node(dev), "freebsd,use-hs-sdr");
	if (max_frequency > 24000000 && !sc->use_hs)
		return (EINVAL);
	sc->use_ddr = OF_hasprop(ofw_bus_get_node(dev), "freebsd,use-ddr52");
	sc->use_sdma = OF_hasprop(ofw_bus_get_node(dev), "freebsd,use-sdma");
	if (sc->use_ddr && (!sc->use_hs || !sc->use_sdma ||
	    bus_width != 8 || max_frequency > 49500000))
		return (EINVAL);
	/* Large eight-bit PIO transfers were not reliable on this board. */
	if (bus_width == 8 && !sc->use_sdma)
		return (EOPNOTSUPP);
	f->dev = dev;
	f->wp_disabled = true;
	rid = 0;
	f->mem_res[0] = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid,
	    RF_ACTIVE);
	if (f->mem_res[0] == NULL)
		return (ENXIO);
	for (unsigned i = 0; i < 5; i++) {
		error = clk_get_by_ofw_name(dev, 0, names[i], &sc->clocks[i]);
		if (error != 0)
			goto fail;
		if ((error = clk_enable(sc->clocks[i])) != 0)
			goto fail;
		sc->enabled++;
		if ((error = hwreset_get_by_ofw_name(dev, 0, names[i],
			 &sc->resets[i])) != 0)
			goto fail;
	}
	if ((error = clk_get_by_name(dev, "emmc_div", &divider)) != 0 ||
	    (error = clk_get_by_name(dev, "xin24m", &oscillator)) != 0 ||
	    (error = clk_set_parent_by_clk(divider, oscillator)) != 0 ||
	    (error = clk_set_freq(sc->clocks[0], 375000, 0)) != 0)
		goto fail;
	if (sc->use_hs && max_frequency > 24000000) {
		if ((error = clk_get_by_name(dev, "gpll", &fast_parent)) != 0 ||
		    (error = clk_get_freq(fast_parent, &parent_hz)) != 0)
			goto fail;
		/* Select a leaf divisor only; never set the shared PLL rate. */
		sc->fast_hz = rk3528_sdhci_fast_rate(parent_hz, max_frequency);
		if (sc->fast_hz == 0) {
			device_printf(dev, "unsupported GPLL rate %ju\n",
			    (uintmax_t)parent_hz);
			error = ERANGE;
			goto fail;
		}
	}
	for (unsigned i = 0; i < 5; i++)
		if ((error = hwreset_assert(sc->resets[i])) != 0)
			goto fail;
	DELAY(10);
	for (int i = 4; i >= 0; i--)
		if ((error = hwreset_deassert(sc->resets[i])) != 0)
			goto fail;
	if ((error = syscon_get_by_ofw_property(dev, ofw_bus_get_node(dev),
		 "rockchip,ioc", &ioc)) != 0 ||
	    (error = fdt_pinctrl_configure_by_name(dev, "default")) != 0)
		goto fail;
	if ((SYSCON_READ_4(ioc, 0x20034) & 0xffff) != 0x1111 ||
	    (SYSCON_READ_4(ioc, 0x20038) & 0xffff) != 0x1111 ||
	    (SYSCON_READ_4(ioc, 0x2003c) & 0xf0ff) != 0x1011) {
		error = EIO;
		goto fail;
	}
	sc->vendor = bus_read_4(f->mem_res[0], 0xe8) & 0xfff;
	if (sc->vendor != 0x500) {
		error = ENXIO;
		goto fail;
	}
	if (bus_width == 8 &&
	    (bus_read_4(f->mem_res[0], SDHCI_CAPABILITIES) &
	    SDHCI_CAN_DO_8BITBUS) == 0) {
		error = EOPNOTSUPP;
		goto fail;
	}
	rid = 0;
	f->irq_res = bus_alloc_resource_any(dev, SYS_RES_IRQ, &rid, RF_ACTIVE);
	if (f->irq_res == NULL) {
		error = ENXIO;
		goto fail;
	}
	slot->quirks = SDHCI_QUIRK_BROKEN_TIMEOUT_VAL |
	    SDHCI_QUIRK_ALL_SLOTS_NON_REMOVABLE;
	if (sc->use_sdma)
		/* Keep identification, bus tests and sector-size I/O on PIO. */
		slot->quirks |= SDHCI_QUIRK_BROKEN_TIMINGS |
		    SDHCI_QUIRK_32BIT_DMA_SIZE;
	else
		slot->quirks |= SDHCI_QUIRK_BROKEN_DMA;
	sc->core_hz = 375000;
	slot->max_clk = 375000;
	if ((error = sdhci_init_slot(dev, slot, 0)) != 0)
		goto fail;
	f->num_slots = 1;
	if (sc->use_sdma && (slot->opt & SDHCI_HAVE_DMA) == 0) {
		error = EOPNOTSUPP;
		goto fail;
	}
	/* Override the core default with the board wiring limit. */
	slot->host.caps &= ~(MMC_CAP_4_BIT_DATA | MMC_CAP_8_BIT_DATA);
	if (bus_width >= 4)
		slot->host.caps |= MMC_CAP_4_BIT_DATA;
	if (bus_width == 8)
		slot->host.caps |= MMC_CAP_8_BIT_DATA;
	if (sc->use_hs &&
	    (slot->host.caps & MMC_CAP_HSPEED) == 0) {
		error = EOPNOTSUPP;
		goto fail;
	}
	if (sc->use_ddr) {
		/* The core requires the aggregate DDR52 mode mask.
		 * Actual signaling support is restricted separately to 1.8V. */
		slot->host.caps |= MMC_CAP_MMC_DDR52 | MMC_CAP_SIGNALING_180;
		slot->host.caps &= ~(MMC_CAP_SIGNALING_120 | MMC_CAP_SIGNALING_330);
	}
	slot->host.f_max = rk3528_sdhci_rate(max_frequency, sc->fast_hz);
	if ((error = bus_setup_intr(dev, f->irq_res,
		 INTR_TYPE_MISC | INTR_MPSAFE, NULL, rk3528_sdhci_intr, sc,
		 &f->intrhand)) != 0)
		goto fail;
	clk_release(divider);
	clk_release(oscillator);
	if (fast_parent != NULL)
		clk_release(fast_parent);
	SYSCTL_ADD_PROC(device_get_sysctl_ctx(dev),
	    SYSCTL_CHILDREN(device_get_sysctl_tree(dev)), OID_AUTO, "status",
	    CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_MPSAFE, sc, 0,
	    rk3528_sdhci_status, "A", "Cached I/O counters and host state");
	sdhci_start_slot(slot);
	return (0);
fail:
	if (divider != NULL)
		clk_release(divider);
	if (oscillator != NULL)
		clk_release(oscillator);
	if (fast_parent != NULL)
		clk_release(fast_parent);
	rk3528_sdhci_release(dev);
	return (error);
}

static int
rk3528_sdhci_detach(device_t dev)
{
	int error;

	if ((error = bus_generic_detach(dev)) != 0)
		return (error);
	rk3528_sdhci_release(dev);
	return (0);
}

static device_method_t rk3528_sdhci_methods[] = {
	DEVMETHOD(device_probe, rk3528_sdhci_probe),
	DEVMETHOD(device_attach, rk3528_sdhci_attach),
	DEVMETHOD(device_detach, rk3528_sdhci_detach),
	DEVMETHOD(bus_write_ivar, rk3528_sdhci_write_ivar),
	DEVMETHOD(bus_read_ivar, rk3528_sdhci_read_ivar),
	DEVMETHOD(mmcbr_update_ios, rk3528_sdhci_update_ios),
	DEVMETHOD(sdhci_read_4, rk3528_sdhci_read_4),
	DEVMETHOD(sdhci_write_4, rk3528_sdhci_write_4),
	DEVMETHOD(sdhci_write_2, rk3528_sdhci_write_2),
	DEVMETHOD(mmcbr_switch_vccq, rk3528_sdhci_switch_vccq),
	DEVMETHOD(sdhci_set_uhs_timing, rk3528_sdhci_set_uhs_timing),
	DEVMETHOD(sdhci_set_clock, rk3528_sdhci_set_clock),
	DEVMETHOD(sdhci_min_freq, rk3528_sdhci_min_freq),
	DEVMETHOD(sdhci_reset, rk3528_sdhci_reset),
	DEVMETHOD_END
};
extern driver_t sdhci_fdt_driver;
DEFINE_CLASS_1(rk3528_sdhci, rk3528_sdhci_driver, rk3528_sdhci_methods,
    sizeof(struct rk3528_sdhci_softc), sdhci_fdt_driver);
DRIVER_MODULE(rk3528_sdhci, simplebus, rk3528_sdhci_driver, NULL, NULL);
MODULE_DEPEND(rk3528_sdhci, sdhci, 1, 1, 1);
