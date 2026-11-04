/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Hiroki Sato <hrs@FreeBSD.org>
 */

#include "opt_acpi.h"

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/module.h>
#include <sys/mutex.h>

#include <machine/bus.h>
#include <machine/intr.h>
#include <machine/machdep.h>

#include <dev/ofw/openfirm.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>
#include <dev/ofw/ofw_subr.h>

#include "msi_if.h"
#include "eioic.h"

struct pch_msi_softc {
	struct eioic_softc *eioic;
	struct mtx lock;
	uint64_t doorbell;
	u_int start;
	u_int count;
	device_t owner[EIOINTC_MAX_IRQS];
	u_int block[EIOINTC_MAX_IRQS];
};

static bool
pch_msi_has_node(phandle_t node)
{

	return (node != 0 && node != (phandle_t)-1);
}

static int
pch_msi_probe(device_t dev)
{

	if (pch_msi_has_node(ofw_bus_get_node(dev))) {
		if (loongarch_efi_acpi_rsdp() != 0 ||
		    !ofw_bus_status_okay(dev) ||
		    !ofw_bus_is_compatible(dev, "loongson,pch-msi-1.0"))
			return (ENXIO);
	} else if (strcmp(device_get_name(dev), "pch_msi") != 0)
		return (ENXIO);
	device_set_desc(dev, "Loongson PCH MSI controller");
	return (BUS_PROBE_DEFAULT);
}

static int
pch_msi_attach(device_t dev)
{
	struct pch_msi_softc *sc;
	bus_addr_t addr;
	bus_size_t size;
	phandle_t node;
	pcell_t start, count;
	intptr_t xref;
	int error;

	sc = device_get_softc(dev);
	sc->eioic = eioic_get_softc();
	if (sc->eioic == NULL || !device_is_attached(sc->eioic->dev))
		return (ENXIO);
	node = ofw_bus_get_node(dev);
	if (pch_msi_has_node(node)) {
		if (ofw_reg_to_paddr(node, 0, &addr, &size, NULL) != 0 ||
		    size < sizeof(uint32_t) ||
		    OF_getencprop(node, "loongson,msi-base-vec", &start,
		    sizeof(start)) != sizeof(start) ||
		    OF_getencprop(node, "loongson,msi-num-vecs", &count,
		    sizeof(count)) != sizeof(count))
			return (ENXIO);
		sc->doorbell = addr;
		sc->start = start;
		sc->count = count;
		xref = OF_xref_from_node(node);
	} else {
#ifdef DEV_ACPI
		if (!loongarch_acpi_pch_msi_info(&addr, &sc->start,
		    &sc->count))
			return (ENXIO);
		sc->doorbell = addr;
		xref = ACPI_MSI_XREF;
#else
		return (ENXIO);
#endif
	}
	if (sc->doorbell == 0 || (sc->doorbell & 3) != 0 ||
	    sc->count == 0 || sc->start >= sc->eioic->vec_count ||
	    sc->count > sc->eioic->vec_count - sc->start)
		return (EINVAL);
	mtx_init(&sc->lock, "pch_msi", NULL, MTX_SPIN);
	error = intr_msi_register(dev, xref);
	if (error != 0) {
		mtx_destroy(&sc->lock);
		return (error);
	}
	if (pch_msi_has_node(node))
		OF_device_register_xref(xref, dev);
	device_printf(dev, "vectors %u-%u, address %#jx\n", sc->start,
	    sc->start + sc->count - 1, (uintmax_t)sc->doorbell);
	return (0);
}

static int
pch_msi_alloc_msi(device_t dev, device_t child, int count, int maxcount,
    device_t *pic, struct intr_irqsrc **srcs)
{
	struct pch_msi_softc *sc;
	u_int vec, end, i;

	if (count <= 0 || maxcount < count || !powerof2(count) ||
	    !powerof2(maxcount) || maxcount > 32)
		return (EINVAL);
	sc = device_get_softc(dev);
	end = sc->start + sc->count;
	mtx_lock_spin(&sc->lock);
	for (vec = roundup2(sc->start, maxcount); vec < end;
	    vec += maxcount) {
		if ((u_int)count > end - vec)
			break;
		for (i = 0; i < (u_int)count; i++) {
			if (sc->owner[vec + i] != NULL ||
			    sc->eioic->isrcs[vec + i].child != NULL)
				break;
		}
		if (i == (u_int)count)
			break;
	}
	if (vec >= end || (u_int)count > end - vec) {
		mtx_unlock_spin(&sc->lock);
		return (ENOSPC);
	}
	for (i = 0; i < (u_int)count; i++) {
		sc->owner[vec + i] = child;
		srcs[i] = &sc->eioic->isrcs[vec + i].isrc;
	}
	sc->block[vec] = count;
	mtx_unlock_spin(&sc->lock);
	*pic = sc->eioic->dev;
	return (0);
}

/* Reject foreign or stale sources before interpreting their vector. */
static int
pch_msi_vector(struct pch_msi_softc *sc, struct intr_irqsrc *src)
{
	u_int vec;

	for (vec = sc->start; vec < sc->start + sc->count; vec++)
		if (src == &sc->eioic->isrcs[vec].isrc)
			return (vec);
	return (-1);
}

static int
pch_msi_release_msi(device_t dev, device_t child, int count,
    struct intr_irqsrc **srcs)
{
	struct pch_msi_softc *sc;
	int vec, i;

	sc = device_get_softc(dev);
	if (count <= 0 || count > 32)
		return (EINVAL);
	vec = pch_msi_vector(sc, srcs[0]);
	if (vec < 0 || (u_int)count > sc->start + sc->count - vec)
		return (EINVAL);
	mtx_lock_spin(&sc->lock);
	if (sc->block[vec] != (u_int)count)
		goto invalid;
	for (i = 0; i < count; i++) {
		if (sc->owner[vec + i] != child ||
		    srcs[i] != &sc->eioic->isrcs[vec + i].isrc)
			goto invalid;
	}
	for (i = 0; i < count; i++)
		sc->owner[vec + i] = NULL;
	sc->block[vec] = 0;
	mtx_unlock_spin(&sc->lock);
	return (0);
invalid:
	mtx_unlock_spin(&sc->lock);
	return (EINVAL);
}

static int
pch_msi_alloc_msix(device_t dev, device_t child, device_t *pic,
    struct intr_irqsrc **src)
{

	return (pch_msi_alloc_msi(dev, child, 1, 1, pic, src));
}

static int
pch_msi_release_msix(device_t dev, device_t child, struct intr_irqsrc *src)
{

	return (pch_msi_release_msi(dev, child, 1, &src));
}

static int
pch_msi_map_msi(device_t dev, device_t child, struct intr_irqsrc *src,
    uint64_t *addr, uint32_t *data)
{
	struct pch_msi_softc *sc;
	int vec;

	sc = device_get_softc(dev);
	vec = pch_msi_vector(sc, src);
	if (vec < 0)
		return (EINVAL);
	mtx_lock_spin(&sc->lock);
	if (sc->owner[vec] != child) {
		mtx_unlock_spin(&sc->lock);
		return (EINVAL);
	}
	/* The message data is the absolute HT/EIOINTC vector number. */
	*addr = sc->doorbell;
	*data = vec;
	mtx_unlock_spin(&sc->lock);
	return (0);
}

static device_method_t pch_msi_methods[] = {
	DEVMETHOD(device_probe,		pch_msi_probe),
	DEVMETHOD(device_attach,		pch_msi_attach),
	DEVMETHOD(msi_alloc_msi,		pch_msi_alloc_msi),
	DEVMETHOD(msi_release_msi,	pch_msi_release_msi),
	DEVMETHOD(msi_alloc_msix,		pch_msi_alloc_msix),
	DEVMETHOD(msi_release_msix,	pch_msi_release_msix),
	DEVMETHOD(msi_map_msi,		pch_msi_map_msi),
	DEVMETHOD_END
};

static driver_t pch_msi_driver = {
	"pch_msi", pch_msi_methods, sizeof(struct pch_msi_softc)
};

EARLY_DRIVER_MODULE(pch_msi, simplebus, pch_msi_driver, 0, 0,
    BUS_PASS_INTERRUPT + BUS_PASS_ORDER_LAST);
EARLY_DRIVER_MODULE(pch_msi, nexus, pch_msi_driver, 0, 0,
    BUS_PASS_INTERRUPT + BUS_PASS_ORDER_LAST);
