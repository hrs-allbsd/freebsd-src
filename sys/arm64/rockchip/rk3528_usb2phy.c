/*-
 * SPDX-License-Identifier: BSD-2-Clause
 * Copyright (c) 2026 Hiroki Sato <hrs@FreeBSD.org>
 */

/* Fixed USB2 host support for the R28S OTG port. */
#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/rman.h>
#include <machine/bus.h>
#include <dev/ofw/ofw_bus_subr.h>
#include <dev/clk/clk.h>
#include <dev/phy/phy_usb.h>
#include <dev/syscon/syscon.h>
#include "syscon_if.h"

struct rk3528_usb2phy_softc {
	device_t dev;
	struct resource *mem;
	struct syscon *grf;
	clk_t ref, apb;
	int mode;
};

static int
rk3528_usb2phy_enable(struct phynode *phy, bool enable)
{
	struct rk3528_usb2phy_softc *sc;
	uint32_t v;

	sc = device_get_softc(phynode_get_device(phy));
	if (enable) {
		/* PHY-local registers use ordinary RMW, not GRF high-word masks. */
		v = bus_read_4(sc->mem, 0x30);
		bus_write_4(sc->mem, 0x30, v & ~0x74u);
		v = bus_read_4(sc->mem, 0x94);
		bus_write_4(sc->mem, 0x94, (v & ~0x78u) | 0x18u);
		v = bus_read_4(sc->mem, 0x41c);
		bus_write_4(sc->mem, 0x41c, (v & ~0xfcu) | (0x27u << 2));
		DELAY(1300);
		/* Force host ID without enabling a VBUS source. */
		SYSCON_WRITE_4(sc->grf, 0x6004c, 0x06000200);
	}
	/* OTG UTMI suspend control: active=0, suspend=0x1d1 (bits 8:0). */
	SYSCON_WRITE_4(sc->grf, 0x6004c,
	    0x01ff0000 | (enable ? 0 : 0x1d1));
	DELAY(2000);
	device_printf(sc->dev, "USB2 OTG host PHY %s, control=%#x\n",
	    enable ? "enabled" : "suspended", SYSCON_READ_4(sc->grf, 0x6004c));
	return (0);
}

static int
rk3528_usb2phy_get_mode(struct phynode *phy, int *mode)
{
	struct rk3528_usb2phy_softc *sc;

	sc = device_get_softc(phynode_get_device(phy));
	*mode = sc->mode;
	return (0);
}

static int
rk3528_usb2phy_set_mode(struct phynode *phy, int mode)
{
	struct rk3528_usb2phy_softc *sc;

	sc = device_get_softc(phynode_get_device(phy));
	/* The boot DT owns the fixed role; do not switch a live PHY. */
	return (mode == sc->mode ? 0 : EOPNOTSUPP);
}

static phynode_method_t rk3528_usb2phy_phynode_methods[] = {
	PHYNODEMETHOD(phynode_enable, rk3528_usb2phy_enable),
	PHYNODEMETHOD(phynode_usb_get_mode, rk3528_usb2phy_get_mode),
	PHYNODEMETHOD(phynode_usb_set_mode, rk3528_usb2phy_set_mode),
	PHYNODEMETHOD_END
};
DEFINE_CLASS_1(rk3528_usb2phy_phynode, rk3528_usb2phy_phynode_class,
    rk3528_usb2phy_phynode_methods, sizeof(struct phynode_usb_sc),
    phynode_usb_class);

static int
rk3528_usb2phy_probe(device_t dev)
{
	if (!ofw_bus_status_okay(dev) || !ofw_bus_is_compatible(dev,
	    "freebsd,rk3528-r28s-usb2phy"))
		return (ENXIO);
	device_set_desc(dev, "RK3528 R28S USB2 OTG PHY");
	return (BUS_PROBE_DEFAULT);
}

static int
rk3528_usb2phy_attach(device_t dev)
{
	struct rk3528_usb2phy_softc *sc = device_get_softc(dev);
	struct phynode_init_def init = { 0 };
	struct phynode *phy;
	phandle_t node, port;
	char mode[16] = {0};
	bool ref_on = false, apb_on = false;
	int rid = 0, error, len;

	sc->dev = dev;
	node = ofw_bus_get_node(dev);
	sc->mode = PHY_USB_MODE_HOST;
	len = OF_getprop(node, "dr_mode", mode, sizeof(mode));
	if (len > 0 && (len != sizeof("host") ||
	    memcmp(mode, "host", sizeof("host")) != 0))
		return (EINVAL);
	port = ofw_bus_find_child(node, "otg-port");
	if (port <= 0 || !ofw_bus_node_status_okay(port))
		return (ENXIO);
	error = syscon_get_by_ofw_property(dev, node, "rockchip,usbgrf", &sc->grf);
	if (error != 0)
		return (error);
	sc->mem = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid, RF_ACTIVE);
	if (sc->mem == NULL)
		return (ENXIO);
	if (rman_get_size(sc->mem) < 0x420) {
		error = ENXIO;
		goto fail;
	}
	error = clk_get_by_ofw_name(dev, node, "phyclk", &sc->ref);
	if (error != 0)
		goto fail;
	error = clk_get_by_ofw_name(dev, node, "apb_pclk", &sc->apb);
	if (error != 0)
		goto fail;
	error = clk_enable(sc->ref);
	if (error != 0)
		goto fail;
	ref_on = true;
	error = clk_enable(sc->apb);
	if (error != 0)
		goto fail;
	apb_on = true;
	init.id = 0;
	init.ofw_node = port;
	phy = phynode_create(dev, &rk3528_usb2phy_phynode_class, &init);
	if (phy == NULL || phynode_register(phy) == NULL) {
		error = ENXIO;
		goto fail;
	}
	OF_device_register_xref(OF_xref_from_node(port), dev);
	return (0);

fail:
	if (apb_on)
		clk_disable(sc->apb);
	if (ref_on)
		clk_disable(sc->ref);
	if (sc->apb != NULL) {
		clk_release(sc->apb);
		sc->apb = NULL;
	}
	if (sc->ref != NULL) {
		clk_release(sc->ref);
		sc->ref = NULL;
	}
	bus_release_resource(dev, SYS_RES_MEMORY, rid, sc->mem);
	sc->mem = NULL;
	return (error);
}

static device_method_t rk3528_usb2phy_methods[] = {
	DEVMETHOD(device_probe, rk3528_usb2phy_probe),
	DEVMETHOD(device_attach, rk3528_usb2phy_attach),
	DEVMETHOD_END
};
static driver_t rk3528_usb2phy_driver = {
	"rk3528_usb2phy", rk3528_usb2phy_methods, sizeof(struct rk3528_usb2phy_softc)
};
EARLY_DRIVER_MODULE(rk3528_usb2phy, simplebus, rk3528_usb2phy_driver, 0, 0,
    BUS_PASS_SUPPORTDEV + BUS_PASS_ORDER_MIDDLE);
