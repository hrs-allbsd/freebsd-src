/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Hiroki Sato <hrs@FreeBSD.org>
 *
 * Loongson LS7A2000 integrated Ethernet, single-channel DWMAC 3.73.
 * New iflib implementation. Register definitions are shared with FreeBSD's
 * BSD-licensed DWC driver.
 */
#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/endian.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/mbuf.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/rman.h>
#include <sys/socket.h>
#include <sys/sockio.h>
#include <sys/sysctl.h>

#include <net/if.h>
#include <net/if_var.h>
#include <net/if_media.h>
#include <net/ethernet.h>
#include <net/iflib.h>
#include <dev/pci/pcireg.h>
#include <dev/pci/pcivar.h>
#include <dev/mii/mii.h>
#include <dev/mii/miivar.h>
#include <dev/dwc/dwc1000_reg.h>

#include "ifdi_if.h"
#include "miibus_if.h"

/* DWMAC enhanced descriptors, with 32-bit buffer addresses. ATDS stays off. */
struct gnet_desc {
	uint32_t status, control, buffer, next;
};
#define GN_OWN	     (1U << 31)
#define GN_TX_INT    (1U << 30)
#define GN_TX_LAST   (1U << 29)
#define GN_TX_FIRST  (1U << 28)
#define GN_TX_END    (1U << 21)
#define GN_RX_END    (1U << 15)
#define GN_ERROR     (1U << 15)
#define GN_RX_FIRST  (1U << 9)
#define GN_RX_LAST   (1U << 8)
#define GN_FRAME_MAX 1522
#define GN_IRQ_MASK                                                       \
	(INT_EN_NIE | INT_EN_AIE | INT_EN_FBE | INT_EN_UNE | INT_EN_RIE | \
	    INT_EN_TIE | INT_EN_RUE)

struct gnet_softc {
	if_ctx_t ctx;
	device_t dev, miibus;
	struct resource *mem;
	int rid;
	struct mtx mdio_lock;
	struct gnet_desc *tx, *rx;
	bus_addr_t txaddr, rxaddr;
	qidx_t ntx, nrx, txprod, txclean, txpublish;
	bool running;
	int link;
	uint64_t irq_count, tx_errors, rx_errors, resets;
	uint32_t last_status, last_tx, last_rx;
	uint8_t mac[ETHER_ADDR_LEN];
	qidx_t rx_ids[1024];
};
static uint32_t
rd(struct gnet_softc *sc, bus_size_t off)
{
	return (bus_read_4(sc->mem, off));
}
static void
wr(struct gnet_softc *sc, bus_size_t off, uint32_t val)
{
	bus_write_4(sc->mem, off, val);
	bus_barrier(sc->mem, off, 4,
	    BUS_SPACE_BARRIER_READ | BUS_SPACE_BARRIER_WRITE);
}
static int
wait_clear(struct gnet_softc *sc, bus_size_t off, uint32_t bits)
{
	for (int i = 0; i < 10000; i++) {
		if ((rd(sc, off) & bits) == 0)
			return (0);
		DELAY(10);
	}
	return (ETIMEDOUT);
}
/* The LS7A DMA reset can take up to two seconds; MDIO remains bounded
 * by the shorter wait_clear() timeout above. Preserve other bus-mode bits.
 */
static int
dma_reset(struct gnet_softc *sc)
{
	wr(sc, BUS_MODE, rd(sc, BUS_MODE) | BUS_MODE_SWR);
	for (int i = 0; i < 2000; i++) {
		DELAY(1000);
		if ((rd(sc, BUS_MODE) & BUS_MODE_SWR) == 0)
			return (0);
	}
	device_printf(sc->dev, "DMA reset timeout cfg=%#x bus=%#x\n",
	    rd(sc, MAC_CONFIGURATION), rd(sc, BUS_MODE));
	return (ETIMEDOUT);
}
static struct gnet_softc *
fromdev(device_t dev)
{
	return (iflib_get_softc(device_get_softc(dev)));
}
static int
mdio_read(device_t dev, int phy, int reg)
{
	struct gnet_softc *sc = fromdev(dev);
	int val = 0xffff;
	mtx_lock(&sc->mdio_lock);
	if (wait_clear(sc, GMII_ADDRESS, 1) == 0) {
		wr(sc, GMII_ADDRESS, (phy << 11) | (reg << 6) | (1 << 2) | 1);
		if (wait_clear(sc, GMII_ADDRESS, 1) == 0)
			val = rd(sc, GMII_DATA) & 0xffff;
	}
	mtx_unlock(&sc->mdio_lock);
	return (val);
}
static int
mdio_write(device_t dev, int phy, int reg, int val)
{
	struct gnet_softc *sc = fromdev(dev);
	int error;
	mtx_lock(&sc->mdio_lock);
	error = wait_clear(sc, GMII_ADDRESS, 1);
	if (error == 0) {
		wr(sc, GMII_DATA, val & 0xffff);
		wr(sc, GMII_ADDRESS, (phy << 11) | (reg << 6) | (1 << 2) | 3);
		error = wait_clear(sc, GMII_ADDRESS, 1);
	}
	mtx_unlock(&sc->mdio_lock);
	return (error);
}
static void
link_update(device_t dev)
{
	struct gnet_softc *sc = fromdev(dev);
	struct mii_data *mii;
	uint32_t cfg, old;
	uint64_t speed = 0;
	int link = LINK_STATE_DOWN;
	/* MII polling can run before iflib has prepared the RX buffers. */
	if (sc->miibus == NULL || !sc->running)
		return;
	mii = device_get_softc(sc->miibus);
	old = rd(sc, MAC_CONFIGURATION);
	cfg = old & ~(CONF_PS | CONF_FES | CONF_DM);
	if ((mii->mii_media_status & IFM_ACTIVE) != 0 &&
	    (mii->mii_media_active & IFM_FDX) != 0) {
		switch (IFM_SUBTYPE(mii->mii_media_active)) {
		case IFM_1000_T:
			speed = IF_Gbps(1);
			break;
		case IFM_100_TX:
			cfg |= CONF_PS | CONF_FES;
			speed = IF_Mbps(100);
			break;
		case IFM_10_T:
			cfg |= CONF_PS;
			speed = IF_Mbps(10);
			break;
		}
		if (speed != 0) {
			cfg |= CONF_DM;
			link = LINK_STATE_UP;
		}
	}
	wr(sc, MAC_CONFIGURATION, cfg);
	/* Integrated PHY needs a second negotiation after MII -> GMII. */
	if (speed == IF_Gbps(1) && (old & CONF_PS) != 0) {
		int bmcr = mdio_read(dev, 2, MII_BMCR);
		if (bmcr != 0xffff)
			mdio_write(dev, 2, MII_BMCR, bmcr | BMCR_STARTNEG);
		link = LINK_STATE_DOWN;
	}
	if (link != sc->link) {
		sc->link = link;
		iflib_link_state_change(sc->ctx, link, speed);
	}
}
static int
media_change(if_t ifp)
{
	struct gnet_softc *sc = iflib_get_softc(if_getsoftc(ifp));
	return (mii_mediachg(device_get_softc(sc->miibus)));
}
static void
media_status(if_t ifp, struct ifmediareq *req)
{
	struct gnet_softc *sc = iflib_get_softc(if_getsoftc(ifp));
	struct mii_data *mii = device_get_softc(sc->miibus);
	mii_pollstat(mii);
	req->ifm_status = mii->mii_media_status;
	req->ifm_active = mii->mii_media_active;
}
static void
gnet_intr_disable(if_ctx_t ctx)
{
	struct gnet_softc *sc = iflib_get_softc(ctx);
	if (sc->mem != NULL)
		wr(sc, INTERRUPT_ENABLE, 0);
}
static void
gnet_intr_enable(if_ctx_t ctx)
{
	struct gnet_softc *sc = iflib_get_softc(ctx);
	if (sc->running) {
		wr(sc, MAC_CONFIGURATION,
		    rd(sc, MAC_CONFIGURATION) | CONF_TE | CONF_RE);
		wr(sc, OPERATION_MODE, MODE_TSF | MODE_RSF | MODE_ST | MODE_SR);
		wr(sc, RECEIVE_POLL_DEMAND, 1);
		wr(sc, INTERRUPT_ENABLE, GN_IRQ_MASK);
	}
}
static int
legacy_intr(void *arg)
{
	struct gnet_softc *sc = arg;
	uint32_t status = rd(sc, DMA_STATUS);
	if ((status & GN_IRQ_MASK) == 0)
		return (FILTER_STRAY);
	wr(sc, INTERRUPT_ENABLE, 0);
	wr(sc, DMA_STATUS, status & DMA_STATUS_INTR_MASK);
	sc->irq_count++;
	sc->last_status = status;
	if (status & DMA_STATUS_FBI)
		iflib_request_reset(sc->ctx);
	return (FILTER_SCHEDULE_THREAD);
}
static void
filter_set(if_ctx_t ctx)
{
	struct gnet_softc *sc = iflib_get_softc(ctx);
	int flags = if_getflags(iflib_get_ifp(ctx));
	/* Accept multicast without promising a hardware hash filter. */
	wr(sc, MAC_FRAME_FILTER,
	    FRAME_FILTER_PM | ((flags & IFF_PROMISC) ? FRAME_FILTER_PR : 0));
}
static int
promisc_set(if_ctx_t ctx, int flags)
{
	filter_set(ctx);
	return (0);
}
static void
stop(if_ctx_t ctx)
{
	struct gnet_softc *sc = iflib_get_softc(ctx);
	if (sc->mem == NULL)
		return;
	sc->running = false;
	gnet_intr_disable(ctx);
	wr(sc, OPERATION_MODE, rd(sc, OPERATION_MODE) & ~(MODE_ST | MODE_SR));
	wr(sc, MAC_CONFIGURATION,
	    rd(sc, MAC_CONFIGURATION) & ~(CONF_TE | CONF_RE));
	/* Reset fences DMA before iflib may release packet mappings. */
	/* Keep the negotiated PHY interface clock selection during reset. */
	if (dma_reset(sc) != 0) {
		pci_disable_busmaster(sc->dev);
		device_printf(sc->dev,
		    "DMA reset timed out; bus mastering disabled\n");
	}
}
static void
init(if_ctx_t ctx)
{
	struct gnet_softc *sc = iflib_get_softc(ctx);
	pci_enable_busmaster(sc->dev);
	/* Keep the negotiated PHY interface clock selection during reset. */
	if (dma_reset(sc) != 0) {
		pci_disable_busmaster(sc->dev);
		iflib_init_failed(ctx);
		return;
	}
	sc->resets++;
	sc->txprod = sc->txclean = sc->txpublish = 0;
	bzero(sc->tx, sizeof(*sc->tx) * sc->ntx);
	/* RX buffers are populated by iflib after this callback. */
	wr(sc, BUS_MODE,
	    BUS_MODE_USP | (32 << BUS_MODE_PBL_SHIFT) |
		(32 << BUS_MODE_RPBL_SHIFT));
	wr(sc, RX_DESCR_LIST_ADDR, sc->rxaddr);
	wr(sc, TX_DESCR_LIST_ADDR, sc->txaddr);
	wr(sc, MAC_ADDRESS_HIGH(0), sc->mac[4] | (sc->mac[5] << 8));
	wr(sc, MAC_ADDRESS_LOW(0), le32dec(sc->mac));
	wr(sc, INTERRUPT_MASK, 0xffffffff);
	wr(sc, MMC_RECEIVE_INTERRUPT_MASK, 0xffffffff);
	wr(sc, MMC_TRANSMIT_INTERRUPT_MASK, 0xffffffff);
	wr(sc, MMC_IPC_RECEIVE_INT_MASK, 0xffffffff);
	wr(sc, FLOW_CONTROL, 0);
	wr(sc, GMAC_VLAN_TAG, 0);
	wr(sc, MAC_CONFIGURATION, CONF_DM);
	filter_set(ctx);
	wr(sc, DMA_STATUS, DMA_STATUS_INTR_MASK);
	wr(sc, OPERATION_MODE, MODE_TSF | MODE_RSF);
	sc->running = true;
	mii_mediachg(device_get_softc(sc->miibus));
}
static int
tx_alloc(if_ctx_t ctx, caddr_t *v, uint64_t *p, int nrings, int nsets)
{
	struct gnet_softc *sc = iflib_get_softc(ctx);
	if (nrings != 1 || nsets != 1 || p[0] > UINT32_MAX)
		return (EINVAL);
	sc->tx = (void *)v[0];
	sc->txaddr = p[0];
	sc->ntx = iflib_get_softc_ctx(ctx)->isc_ntxd[0];
	return (0);
}
static int
rx_alloc(if_ctx_t ctx, caddr_t *v, uint64_t *p, int nrings, int nsets)
{
	struct gnet_softc *sc = iflib_get_softc(ctx);
	if (nrings != 1 || nsets != 1 || p[0] > UINT32_MAX)
		return (EINVAL);
	sc->rx = (void *)v[0];
	sc->rxaddr = p[0];
	sc->nrx = iflib_get_softc_ctx(ctx)->isc_nrxd[0];
	return (0);
}
static void
queues_free(if_ctx_t ctx)
{
	struct gnet_softc *sc = iflib_get_softc(ctx);
	sc->tx = sc->rx = NULL;
}
static int
tx_encap(void *arg, if_pkt_info_t pi)
{
	struct gnet_softc *sc = arg;
	qidx_t idx = pi->ipi_pidx;
	if (pi->ipi_nsegs < 1 || pi->ipi_nsegs > 16 ||
	    pi->ipi_len > GN_FRAME_MAX)
		return (EFBIG);
	for (unsigned j = 0; j < pi->ipi_nsegs; j++)
		if (pi->ipi_segs[j].ds_addr > UINT32_MAX)
			return (EFBIG);
	for (unsigned j = 0; j < pi->ipi_nsegs; j++) {
		struct gnet_desc *d = &sc->tx[idx];
		d->buffer = htole32(pi->ipi_segs[j].ds_addr);
		d->next = 0;
		d->control = htole32(pi->ipi_segs[j].ds_len);
		d->status = htole32((j == 0 ? GN_TX_FIRST : 0) |
		    (j == pi->ipi_nsegs - 1 ? GN_TX_LAST | GN_TX_INT : 0) |
		    (idx == sc->ntx - 1 ? GN_TX_END : 0));
		idx = (idx + 1) % sc->ntx;
	}
	/* Index-based doorbell: nonzero ipi_ndescs means a count to iflib. */
	pi->ipi_ndescs = 0;
	pi->ipi_new_pidx = idx;
	sc->txprod = idx;
	return (0);
}
static void
tx_flush(void *arg, uint16_t q, qidx_t pidx)
{
	struct gnet_softc *sc = arg;
	qidx_t i = pidx;
	/* Packet PREWRITE is complete. Publish tails before packet heads. */
	while (i != sc->txpublish) {
		i = (i + sc->ntx - 1) % sc->ntx;
		atomic_thread_fence_rel();
		sc->tx[i].status |= htole32(GN_OWN);
	}
	sc->txpublish = pidx;
	atomic_thread_fence_seq_cst();
	wr(sc, TRANSMIT_POLL_DEMAND, 1);
}
static int
tx_credits(void *arg, uint16_t q, bool clear)
{
	struct gnet_softc *sc = arg;
	qidx_t i = sc->txclean;
	int n = 0;
	while (i != sc->txpublish) {
		uint32_t status = le32toh(sc->tx[i].status);
		if (status & GN_OWN)
			break;
		if (!clear)
			return (1);
		if (status & GN_ERROR) {
			if (sc->tx_errors == 0)
				device_printf(sc->dev,
				    "TX error descriptor=%#x\n", status);
			sc->tx_errors++;
		}
		sc->last_tx = status;
		i = (i + 1) % sc->ntx;
		n++;
	}
	if (clear)
		sc->txclean = i;
	return (n);
}
static int
rx_available(void *arg, uint16_t q, qidx_t idx, qidx_t budget)
{
	struct gnet_softc *sc = arg;
	int n = 0;
	while (n < budget && n < sc->nrx &&
	    (le32toh(sc->rx[idx].status) & GN_OWN) == 0 &&
	    sc->rx[idx].buffer != 0) {
		n++;
		idx = (idx + 1) % sc->nrx;
	}
	return (n);
}
static int
rx_get(void *arg, if_rxd_info_t ri)
{
	struct gnet_softc *sc = arg;
	qidx_t i = ri->iri_cidx;
	uint32_t status = le32toh(sc->rx[i].status);
	sc->last_rx = status;
	unsigned len = (status >> 16) & 0x3fff;
	if ((status & (GN_OWN | GN_ERROR | GN_RX_FIRST | GN_RX_LAST)) !=
		(GN_RX_FIRST | GN_RX_LAST) ||
	    len < ETHER_CRC_LEN || len > GN_FRAME_MAX) {
		sc->rx_errors++;
		return (EBADMSG);
	}
	ri->iri_len = len - ETHER_CRC_LEN;
	ri->iri_nfrags = 1;
	ri->iri_frags[0].irf_flid = 0;
	ri->iri_frags[0].irf_idx = sc->rx_ids[i];
	ri->iri_frags[0].irf_len = ri->iri_len;
	ri->iri_cidx = (i + 1) % sc->nrx;
	return (0);
}
static void
rx_refill(void *arg, if_rxd_update_t u)
{
	struct gnet_softc *sc = arg;
	for (unsigned j = 0; j < u->iru_count; j++) {
		qidx_t i = (u->iru_pidx + j) % sc->nrx;
		struct gnet_desc *d = &sc->rx[i];
		sc->rx_ids[i] = u->iru_idxs[j];
		KASSERT(u->iru_paddrs[j] <= UINT32_MAX, ("gnet RX address"));
		d->buffer = htole32(u->iru_paddrs[j]);
		d->next = 0;
		d->control = htole32(MIN(u->iru_buf_size, 2047) |
		    (i == sc->nrx - 1 ? GN_RX_END : 0));
		atomic_thread_fence_rel();
		d->status = htole32(GN_OWN);
	}
}
static void
rx_flush(void *arg, uint16_t q, uint8_t fl, qidx_t pidx)
{
	struct gnet_softc *sc = arg;
	wr(sc, RECEIVE_POLL_DEMAND, 1);
}
static struct if_txrx txrx = {
	.ift_txd_encap = tx_encap,
	.ift_txd_flush = tx_flush,
	.ift_txd_credits_update = tx_credits,
	.ift_rxd_available = rx_available,
	.ift_rxd_pkt_get = rx_get,
	.ift_rxd_refill = rx_refill,
	.ift_rxd_flush = rx_flush,
	.ift_legacy_intr = legacy_intr,
};
static int
detach(if_ctx_t ctx)
{
	struct gnet_softc *sc = iflib_get_softc(ctx);
	stop(ctx);
	if (sc->miibus != NULL) {
		device_delete_child(sc->dev, sc->miibus);
		sc->miibus = NULL;
	}
	if (sc->mem != NULL) {
		bus_release_resource(sc->dev, SYS_RES_MEMORY, sc->rid, sc->mem);
		sc->mem = NULL;
	}
	if (mtx_initialized(&sc->mdio_lock))
		mtx_destroy(&sc->mdio_lock);
	return (0);
}
static int
attach_pre(if_ctx_t ctx)
{
	struct gnet_softc *sc = iflib_get_softc(ctx);
	if_softc_ctx_t c = iflib_get_softc_ctx(ctx);
	uint32_t low, high, version;
	int error;
	sc->ctx = ctx;
	sc->dev = iflib_get_dev(ctx);
	sc->link = LINK_STATE_UNKNOWN;
	mtx_init(&sc->mdio_lock, "gnet MDIO", NULL, MTX_DEF);
	sc->rid = PCIR_BAR(0);
	sc->mem = bus_alloc_resource_any(sc->dev, SYS_RES_MEMORY, &sc->rid,
	    RF_ACTIVE);
	if (sc->mem == NULL) {
		error = ENXIO;
		goto fail;
	}
	version = rd(sc, VERSION);
	device_printf(sc->dev, "GMAC version=%#x features=%#x BAR0=%#jx\n",
	    version, rd(sc, HW_FEATURE), (uintmax_t)rman_get_start(sc->mem));
	/* 0x10/0x12 are different multichannel IPs, intentionally unsupported.
	 */
	if ((version & 0xff) != 0x37) {
		error = ENXIO;
		goto fail;
	}
	low = rd(sc, MAC_ADDRESS_LOW(0));
	high = rd(sc, MAC_ADDRESS_HIGH(0));
	le32enc(sc->mac, low);
	le16enc(sc->mac + 4, high);
	if (ETHER_IS_MULTICAST(sc->mac) || ETHER_IS_ZERO(sc->mac)) {
		device_printf(sc->dev, "invalid firmware MAC address\n");
		error = ENXIO;
		goto fail;
	}
	gnet_intr_disable(ctx);
	pci_enable_busmaster(sc->dev);
	c->isc_txrx = &txrx;
	c->isc_tx_nsegments = 16;
	c->isc_txqsizes[0] = c->isc_ntxd[0] * sizeof(struct gnet_desc);
	c->isc_rxqsizes[0] = c->isc_nrxd[0] * sizeof(struct gnet_desc);
	c->isc_ntxqsets = c->isc_nrxqsets = 1;
	c->isc_ntxqsets_max = c->isc_nrxqsets_max = 1;
	c->isc_msix_bar =
	    0; /* INTx via ACPI; MSI is not implemented on this port. */
	c->isc_dma_width = 32;
	c->isc_max_frame_size = GN_FRAME_MAX;
	c->isc_capabilities = c->isc_capenable = IFCAP_VLAN_MTU;
	iflib_set_mac(ctx, sc->mac);
	error = mii_attach(sc->dev, &sc->miibus, iflib_get_ifp(ctx),
	    media_change, media_status, BMSR_DEFCAPMASK, 2, MII_OFFSET_ANY, 0);
	if (error != 0)
		goto fail;
	c->isc_media =
	    &((struct mii_data *)device_get_softc(sc->miibus))->mii_media;
	return (0);
fail:
	detach(ctx);
	return (error);
}
static int
reg_sysctl(SYSCTL_HANDLER_ARGS)
{
	struct gnet_softc *sc = arg1;
	uint32_t val = rd(sc, arg2);
	return (sysctl_handle_32(oidp, &val, 0, req));
}
static int
attach_post(if_ctx_t ctx)
{
	struct gnet_softc *sc = iflib_get_softc(ctx);
	struct sysctl_ctx_list *s = device_get_sysctl_ctx(sc->dev);
	struct sysctl_oid_list *l = SYSCTL_CHILDREN(
	    device_get_sysctl_tree(sc->dev));
	SYSCTL_ADD_U64(s, l, OID_AUTO, "interrupts", CTLFLAG_RD, &sc->irq_count,
	    0, "INTx events");
	SYSCTL_ADD_U64(s, l, OID_AUTO, "tx_errors", CTLFLAG_RD, &sc->tx_errors,
	    0, "TX errors");
	SYSCTL_ADD_U64(s, l, OID_AUTO, "rx_errors", CTLFLAG_RD, &sc->rx_errors,
	    0, "RX errors");
	SYSCTL_ADD_U64(s, l, OID_AUTO, "resets", CTLFLAG_RD, &sc->resets, 0,
	    "DMA initializations");
	SYSCTL_ADD_U32(s, l, OID_AUTO, "last_status", CTLFLAG_RD,
	    &sc->last_status, 0, "Last DMA IRQ status");
	SYSCTL_ADD_PROC(s, l, OID_AUTO, "mac_config",
	    CTLTYPE_UINT | CTLFLAG_RD | CTLFLAG_MPSAFE, sc, MAC_CONFIGURATION,
	    reg_sysctl, "IU", "Hardware register");
	SYSCTL_ADD_PROC(s, l, OID_AUTO, "dma_bus_mode",
	    CTLTYPE_UINT | CTLFLAG_RD | CTLFLAG_MPSAFE, sc, BUS_MODE,
	    reg_sysctl, "IU", "Hardware register");
	SYSCTL_ADD_PROC(s, l, OID_AUTO, "dma_status",
	    CTLTYPE_UINT | CTLFLAG_RD | CTLFLAG_MPSAFE, sc, DMA_STATUS,
	    reg_sysctl, "IU", "Hardware register");
	SYSCTL_ADD_PROC(s, l, OID_AUTO, "dma_opmode",
	    CTLTYPE_UINT | CTLFLAG_RD | CTLFLAG_MPSAFE, sc, OPERATION_MODE,
	    reg_sysctl, "IU", "Hardware register");
	SYSCTL_ADD_U32(s, l, OID_AUTO, "last_tx", CTLFLAG_RD, &sc->last_tx, 0,
	    "Last TX descriptor");
	SYSCTL_ADD_U32(s, l, OID_AUTO, "last_rx", CTLFLAG_RD, &sc->last_rx, 0,
	    "Last RX descriptor");
	return (0);
}
static void
admin_update(if_ctx_t ctx)
{
	struct gnet_softc *sc = iflib_get_softc(ctx);
	if (sc->running) {
		mii_tick(device_get_softc(sc->miibus));
		mii_pollstat(device_get_softc(sc->miibus));
		link_update(sc->dev);
	}
}
static void
timer(if_ctx_t ctx, uint16_t q)
{
	iflib_admin_intr_deferred(ctx);
}
static int
mtu_set(if_ctx_t ctx, uint32_t mtu)
{
	return (mtu >= ETHERMIN && mtu <= ETHERMTU ? 0 : EINVAL);
}
static uint64_t
get_counter(if_ctx_t ctx, ift_counter c)
{
	struct gnet_softc *sc = iflib_get_softc(ctx);
	if (c == IFCOUNTER_IERRORS)
		return (sc->rx_errors);
	if (c == IFCOUNTER_OERRORS)
		return (sc->tx_errors);
	return (if_get_counter_default(iflib_get_ifp(ctx), c));
}
static device_method_t if_methods[] = { DEVMETHOD(ifdi_attach_pre, attach_pre),
	DEVMETHOD(ifdi_attach_post, attach_post),
	DEVMETHOD(ifdi_detach, detach), DEVMETHOD(ifdi_init, init),
	DEVMETHOD(ifdi_stop, stop),
	DEVMETHOD(ifdi_intr_enable, gnet_intr_enable),
	DEVMETHOD(ifdi_intr_disable, gnet_intr_disable),
	DEVMETHOD(ifdi_tx_queues_alloc, tx_alloc),
	DEVMETHOD(ifdi_rx_queues_alloc, rx_alloc),
	DEVMETHOD(ifdi_queues_free, queues_free),
	DEVMETHOD(ifdi_multi_set, filter_set),
	DEVMETHOD(ifdi_promisc_set, promisc_set),
	DEVMETHOD(ifdi_mtu_set, mtu_set), DEVMETHOD(ifdi_timer, timer),
	DEVMETHOD(ifdi_update_admin_status, admin_update),
	DEVMETHOD(ifdi_get_counter, get_counter), DEVMETHOD_END };
static driver_t if_driver = { "gnet_if", if_methods,
	sizeof(struct gnet_softc) };
static const pci_vendor_info_t vendors[] = {
	PVID(0x0014, 0x7a13, "Loongson LS7A2000 Gigabit Ethernet"), PVID_END
};
static struct if_shared_ctx shared = {
	.isc_magic = IFLIB_MAGIC,
	.isc_driver = &if_driver,
	.isc_q_align = 4096,
	.isc_tx_maxsize = GN_FRAME_MAX,
	.isc_tx_maxsegsize = GN_FRAME_MAX,
	.isc_rx_maxsize = 2048,
	.isc_rx_maxsegsize = 2048,
	.isc_rx_nsegments = 1,
	.isc_nfl = 1,
	.isc_nrxqs = 1,
	.isc_ntxqs = 1,
	.isc_vendor_info = vendors,
	.isc_driver_version = "0.1",
	.isc_flags = IFLIB_DRIVER_MEDIA,
	.isc_ntxd_min = { 64 },
	.isc_ntxd_default = { 256 },
	.isc_ntxd_max = { 1024 },
	.isc_nrxd_min = { 64 },
	.isc_nrxd_default = { 256 },
	.isc_nrxd_max = { 1024 },
};
static void *
gnet_register(device_t dev)
{
	return (&shared);
}
static device_method_t methods[] = { DEVMETHOD(device_register, gnet_register),
	DEVMETHOD(device_probe, iflib_device_probe),
	DEVMETHOD(device_attach, iflib_device_attach),
	DEVMETHOD(device_detach, iflib_device_detach),
	DEVMETHOD(device_shutdown, iflib_device_shutdown),
	DEVMETHOD(miibus_readreg, mdio_read),
	DEVMETHOD(miibus_writereg, mdio_write),
	DEVMETHOD(miibus_statchg, link_update),
	DEVMETHOD(miibus_linkchg, link_update), DEVMETHOD_END };
static driver_t driver = { "gnet", methods, sizeof(struct gnet_softc) };
DRIVER_MODULE(miibus, gnet, miibus_driver, 0, 0);
DRIVER_MODULE(gnet, pci, driver, 0, 0);
MODULE_DEPEND(gnet, pci, 1, 1, 1);
MODULE_DEPEND(gnet, ether, 1, 1, 1);
MODULE_DEPEND(gnet, iflib, 1, 1, 1);
MODULE_DEPEND(gnet, miibus, 1, 1, 1);
MODULE_VERSION(gnet, 1);
IFLIB_PNP_INFO(pci, gnet, vendors);
