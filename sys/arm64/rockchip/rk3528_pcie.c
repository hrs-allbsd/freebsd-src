/*-
 * SPDX-License-Identifier: BSD-2-Clause
 * Copyright (c) 2026 Hiroki Sato <hrs@FreeBSD.org>
 */

/*
 * R28S single-lane host with integrated Naneng PCIe PHY setup.
 * The 3.3V rail and PLL programming are inherited from firmware.
 * The shared PHY is dedicated to the onboard RTL8111H, not USB3.
 * One endpoint uses the aggregate legacy INTA interrupt; MSI is not exposed.
 */
#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/rman.h>
#include <sys/gpio.h>
#include <dev/gpio/gpiobusvar.h>
#include <machine/bus.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>
#include <dev/ofw/ofw_pci.h>
#include <dev/ofw/ofwpci.h>
#include <dev/pci/pcivar.h>
#include <dev/pci/pcireg.h>
#include <dev/pci/pci_dw.h>
#include <dev/clk/clk.h>
#include <dev/hwreset/hwreset.h>
#include <dev/syscon/syscon.h>
#include "pcib_if.h"
#include "syscon_if.h"

#define	PCIE_CON		0x000
#define	PCIE_INTX_MASK		0x01c
#define	PCIE_HOT_RESET		0x180
#define	PCIE_LTSSM		0x300
#define	GRF_PHY_CON(n)		(0x48000 + (n) * 4)

static const char *clock_names[] = {
	"gpio", "aclk", "hclk_slv", "hclk_dbi", "pclk_cru", "aux",
	"pclk", "phy_apb", "pipe_grf", "refclk"
};
static char *reset_names[] = { "periph", "power_up", "phy_apb", "phy" };

struct rk3528_pcie_softc {
	struct pci_dw_softc dw;
	struct resource *res[3];
	struct syscon *grf;
	int clocks_enabled;
	bool apb_ready;
	bool dw_ready;
	gpio_pin_t reset_gpio;
	clk_t clocks[nitems(clock_names)];
	hwreset_t resets[nitems(reset_names)];
};
static struct resource_spec rk3528_pcie_spec[] = {
	{ SYS_RES_MEMORY, 0, RF_ACTIVE }, /* DBI */
	{ SYS_RES_MEMORY, 1, RF_ACTIVE }, /* APB */
	{ SYS_RES_MEMORY, 3, RF_ACTIVE }, /* PHY */
	{ -1, 0 }
};

static int
rk3528_pcie_link(device_t dev, bool *up)
{
	struct rk3528_pcie_softc *sc;
	uint32_t v;

	sc = device_get_softc(dev);
	v = bus_read_4(sc->res[1], PCIE_LTSSM);

	*up = (v & 0x3003f) == 0x30011;
	return (0);
}

static void
rk3528_phy_modify(struct rk3528_pcie_softc *sc, unsigned reg,
    uint32_t mask, uint32_t val)
{
	uint32_t old;

	old = bus_read_4(sc->res[2], reg);

	bus_write_4(sc->res[2], reg, (old & ~mask) | val);
}

/* Only used before children attach; the endpoint remains in reset. */
static void
rk3528_pcie_release(struct rk3528_pcie_softc *sc)
{
	int i;

	if (sc->apb_ready) {
		bus_write_4(sc->res[1], PCIE_INTX_MASK, 0xffff00ff);
		bus_write_4(sc->res[1], PCIE_CON, 0x000c0008);
	}
	if (sc->reset_gpio != NULL)
		gpio_pin_set_active(sc->reset_gpio, true);
	if (sc->dw_ready) {
		device_delete_children(sc->dw.dev);
		pci_dw_fini(sc->dw.dev);
	}
	for (i = 0; i < nitems(reset_names); i++) {
		if (sc->resets[i] != NULL) {
			hwreset_assert(sc->resets[i]);
			hwreset_release(sc->resets[i]);
		}
	}
	for (i = sc->clocks_enabled - 1; i >= 0; i--)
		clk_disable(sc->clocks[i]);
	for (i = 0; i < nitems(clock_names); i++) {
		if (sc->clocks[i] != NULL)
			clk_release(sc->clocks[i]);
	}
	if (sc->reset_gpio != NULL)
		gpio_pin_release(sc->reset_gpio);
	bus_release_resources(sc->dw.dev, rk3528_pcie_spec, sc->res);
}

static int
rk3528_pcie_attach(device_t dev)
{
	struct rk3528_pcie_softc *sc;
	phandle_t node;
	uint64_t rate;
	uint32_t val;
	bool up;
	int error, i;

	sc = device_get_softc(dev);
	node = ofw_bus_get_node(dev);
	sc->dw.dev = dev;
	if (bus_alloc_resources(dev, rk3528_pcie_spec, sc->res))
		return (ENXIO);
	sc->dw.dbi_res = sc->res[0];
	if ((error = syscon_get_by_ofw_property(dev, node,
	    "rockchip,grf", &sc->grf)) != 0)
		goto fail;
	if ((error = clk_set_assigned(dev, node)) != 0)
		goto fail;
	for (i = 0; i < nitems(clock_names); i++) {
		if ((error = clk_get_by_ofw_name(dev, 0, clock_names[i],
		    &sc->clocks[i])) != 0 ||
		    (error = clk_enable(sc->clocks[i])) != 0) {
			device_printf(dev, "Cannot enable %s clock: %d\n",
			    clock_names[i], error);
			goto fail;
		}
		sc->clocks_enabled++;
	}
	if (clk_get_freq(sc->clocks[nitems(clock_names) - 1], &rate) ||
	    rate != 100000000) {
		device_printf(dev, "PHY requires a 100MHz reference\n");
		error = ENXIO;
		goto fail;
	}
	/* Preload the asserted level before driving the reset pin. */
	if ((error = gpio_pin_get_by_ofw_property(dev, node,
	    "reset-gpios", &sc->reset_gpio)) != 0 ||
	    (error = gpio_pin_set_active(sc->reset_gpio, true)) != 0 ||
	    (error = gpio_pin_setflags(sc->reset_gpio, GPIO_PIN_OUTPUT)) != 0)
		goto fail;
	for (i = 0; i < nitems(reset_names); i++) {
		if ((error = hwreset_get_by_ofw_name(dev, 0, reset_names[i],
		    &sc->resets[i])) != 0 ||
		    (error = hwreset_assert(sc->resets[i])) != 0)
			goto fail;
	}
	DELAY(10);
	if ((error = hwreset_deassert(sc->resets[2])) != 0)
		goto fail;
	/* PCIe mode, internal 100MHz reference, SSC downward and PHY tuning. */
	rk3528_phy_modify(sc, 0x18, 0x30, 0x10);
	SYSCON_WRITE_4(sc->grf, GRF_PHY_CON(0), 0xffff0110);
	SYSCON_WRITE_4(sc->grf, GRF_PHY_CON(1), 0xffff0000);
	SYSCON_WRITE_4(sc->grf, GRF_PHY_CON(2), 0xffff0101);
	SYSCON_WRITE_4(sc->grf, GRF_PHY_CON(3), 0xffff0200);
	SYSCON_WRITE_4(sc->grf, GRF_PHY_CON(1), 0x60004000);
	rk3528_phy_modify(sc, 0x14, 0, 1 << 3);
	rk3528_phy_modify(sc, 0x18, 0x1c00, 0x0800);
	bus_write_4(sc->res[2], 0x108, 0x570804f0);
	if ((error = hwreset_deassert(sc->resets[3])) != 0)
		goto fail;
	DELAY(1000);
	if ((error = hwreset_deassert(sc->resets[0])) != 0 ||
	    (error = hwreset_deassert(sc->resets[1])) != 0)
		goto fail;
	sc->apb_ready = true;
	DELAY(10);
	bus_write_4(sc->res[1], PCIE_CON, 0x000c0008); /* LTSSM off */
	bus_write_4(sc->res[1], PCIE_HOT_RESET, 0x00100010);
	bus_write_4(sc->res[1], PCIE_CON, 0x00f00040); /* root complex */
	bus_write_4(sc->res[1], PCIE_INTX_MASK, 0xffff00ff);
	/* Start at Gen1, the speed used by this board's RTL8168H under Linux. */
	bus_write_4(sc->res[0], DW_MISC_CONTROL_1,
	    bus_read_4(sc->res[0], DW_MISC_CONTROL_1) | DBI_RO_WR_EN);
	val = bus_read_4(sc->res[0], 0x7c);
	bus_write_4(sc->res[0], 0x7c, (val & ~0xf) | 1);
	bus_write_2(sc->res[0], 0xa0,
	    (bus_read_2(sc->res[0], 0xa0) & ~0xf) | 1);
	bus_write_4(sc->res[0], DW_MISC_CONTROL_1,
	    bus_read_4(sc->res[0], DW_MISC_CONTROL_1) & ~DBI_RO_WR_EN);
	bus_write_4(sc->res[0], DW_MSI_INTR0_ENABLE, 0);
	bus_write_4(sc->res[0], DW_MSI_INTR0_MASK, 0xffffffff);
	if ((error = pci_dw_init(dev)) != 0)
		goto fail;
	sc->dw_ready = true;
	/* RK3528 root-port BAR masks must be disabled, as in vendor host init. */
	bus_write_4(sc->res[0], DW_MISC_CONTROL_1,
	    bus_read_4(sc->res[0], DW_MISC_CONTROL_1) | DBI_RO_WR_EN);
	bus_write_4(sc->res[0], 0x100010, 0);
	bus_write_4(sc->res[0], 0x100014, 0);
	bus_write_4(sc->res[0], DW_MISC_CONTROL_1,
	    bus_read_4(sc->res[0], DW_MISC_CONTROL_1) & ~DBI_RO_WR_EN);
	bus_write_4(sc->res[1], PCIE_CON, 0x000c000c);
	DELAY(200000); /* power/refclk stable before releasing PERST# */
	if ((error = gpio_pin_set_active(sc->reset_gpio, false)) != 0)
		goto fail;
	for (i = 0; i < 200; i++) {
		DELAY(10000);
		rk3528_pcie_link(dev, &up);
		if (up)
			break;
	}
	if (!up) {
		device_printf(dev, "PCIe link timeout, LTSSM=%#x\n",
		    bus_read_4(sc->res[1], PCIE_LTSSM));
		error = ETIMEDOUT;
		goto fail;
	}
	device_printf(dev, "PCIe link up, LTSSM=%#x; legacy INTA only\n",
	    bus_read_4(sc->res[1], PCIE_LTSSM));
	/* INTA assertion/deassertion only; all other aggregate sources masked. */
	bus_write_4(sc->res[1], PCIE_INTX_MASK, 0xffff00ee);
	bus_attach_children(dev);
	return (0);

fail:
	rk3528_pcie_release(sc);
	return (error);
}

static int
rk3528_pcie_no_msi(device_t dev, device_t child, int count, int maxcount,
    int *irqs)
{
	return (ENXIO);
}

static int
rk3528_pcie_no_msix(device_t dev, device_t child, int *irq)
{
	return (ENXIO);
}

static int
rk3528_pcie_probe(device_t dev)
{
	if (!ofw_bus_status_okay(dev) ||
	    !ofw_bus_is_compatible(dev, "freebsd,rk3528-r28s-pcie"))
		return (ENXIO);
	device_set_desc(dev, "Rockchip RK3528 R28S PCIe x1 host");
	return (BUS_PROBE_DEFAULT);
}

static int
rk3528_pcie_detach(device_t dev)
{
	/* The onboard endpoint and shared PHY cannot be hot-detached. */
	return (EBUSY);
}

static device_method_t rk3528_pcie_methods[] = {
	DEVMETHOD(device_probe, rk3528_pcie_probe),
	DEVMETHOD(device_attach, rk3528_pcie_attach),
	DEVMETHOD(device_detach, rk3528_pcie_detach),
	DEVMETHOD(pci_dw_get_link, rk3528_pcie_link),
	DEVMETHOD(pcib_alloc_msi, rk3528_pcie_no_msi),
	DEVMETHOD(pcib_alloc_msix, rk3528_pcie_no_msix),
	DEVMETHOD_END
};
DEFINE_CLASS_1(pcib, rk3528_pcie_driver, rk3528_pcie_methods,
    sizeof(struct rk3528_pcie_softc), pci_dw_driver);
DRIVER_MODULE(rk3528_pcie, simplebus, rk3528_pcie_driver, 0, 0);
