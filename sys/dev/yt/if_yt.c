/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Hiroki Sato <hrs@FreeBSD.org>
 *
 * Motorcomm YT6801 PCI Ethernet, single-queue DWMAC4 iflib driver.
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
struct dwc_softc;
#include <dev/dwc/dwc4_reg.h>
#include <dev/dwc/dwc4_mtl.h>
#include <dev/dwc/dwc4_dma.h>

#include "ifdi_if.h"
#include "miibus_if.h"

/* DWMAC4 descriptor layout from BSD dwc4_dma.c (no context/offloads). */
struct yt_desc {
	uint32_t buffer, high, control, status;
};
#define YT_OWN	     (1U << 31)
#define YT_TX_INT    (1U << 31) /* word2 */
#define YT_FIRST     (1U << 29)
#define YT_LAST	     (1U << 28)
#define YT_RX_INT    (1U << 30)
#define YT_RX_VALID  (1U << 24)
#define YT_ERROR     (1U << 15)
#define YT_FRAME_MAX 1522
#define YT_IRQ_MASK                                               \
	(DMACIER_NIE | DMACIER_AIE | DMACIER_FBEE | DMACIER_RIE | \
	    DMACIER_TIE | DMACIER_RBUE)
/* Hardware facts for the Motorcomm wrapper, outside DWMAC's +0x2000 block. */
#define YT_PHY	    0x1004
#define YT_OOB	    0x1010
#define YT_HOST_IRQ 0x1100
#define YT_MODERATE 0x1108
#define YT_RSS_CTRL 0x1048
#define YT_MAC_BASE 0x2000

struct yt_softc {
	if_ctx_t ctx;
	device_t dev, miibus;
	struct resource *mem;
	int rid;
	struct mtx mdio_lock, irq_lock;
	struct yt_desc *tx, *rx;
	bus_addr_t txaddr, rxaddr;
	qidx_t ntx, nrx, txprod, txclean, txpublish;
	bool running;
	int link;
	uint64_t irq_count, tx_errors, rx_errors, resets, rearm_pending;
	uint32_t last_status, last_tx, last_rx, last_host_irq;
	uint8_t mac[ETHER_ADDR_LEN];
	qidx_t rx_ids[1024];
	uint8_t rx_posted[1024];
};
static uint32_t
rd(struct yt_softc *sc, bus_size_t off)
{
	return (bus_read_4(sc->mem, YT_MAC_BASE + off));
}
static void
wr(struct yt_softc *sc, bus_size_t off, uint32_t val)
{
	bus_write_4(sc->mem, YT_MAC_BASE + off, val);
	bus_barrier(sc->mem, YT_MAC_BASE + off, 4,
	    BUS_SPACE_BARRIER_READ | BUS_SPACE_BARRIER_WRITE);
}
static void
wrapper_write(struct yt_softc *sc, bus_size_t off, uint32_t val)
{
	bus_write_4(sc->mem, off, val);
	bus_barrier(sc->mem, off, 4,
	    BUS_SPACE_BARRIER_READ | BUS_SPACE_BARRIER_WRITE);
}
static int
wait_clear(struct yt_softc *sc, bus_size_t off, uint32_t bits)
{
	for (int i = 0; i < 10000; i++) {
		if ((rd(sc, off) & bits) == 0)
			return (0);
		DELAY(10);
	}
	return (ETIMEDOUT);
}
static struct yt_softc *
fromdev(device_t dev)
{
	return (iflib_get_softc(device_get_softc(dev)));
}
static int
mdio_read(device_t dev, int phy, int reg)
{
	struct yt_softc *sc = fromdev(dev);
	int val = 0xffff;
	mtx_lock(&sc->mdio_lock);
	if (wait_clear(sc, ETH_MACMDIOAR, 1) == 0) {
		wr(sc, ETH_MACMDIOAR,
		    (phy << MACMDIOAR_PA_S) | (reg << MACMDIOAR_RDA_S) |
			(2 << MACMDIOAR_CR_S) | MACMDIOAR_READ | MACMDIOAR_GB);
		if (wait_clear(sc, ETH_MACMDIOAR, 1) == 0)
			val = rd(sc, ETH_MACMDIODR) & 0xffff;
	}
	mtx_unlock(&sc->mdio_lock);
	return (val);
}
static int
mdio_write(device_t dev, int phy, int reg, int val)
{
	struct yt_softc *sc = fromdev(dev);
	int error;
	mtx_lock(&sc->mdio_lock);
	error = wait_clear(sc, ETH_MACMDIOAR, 1);
	if (error == 0) {
		wr(sc, ETH_MACMDIODR, val & 0xffff);
		wr(sc, ETH_MACMDIOAR,
		    (phy << MACMDIOAR_PA_S) | (reg << MACMDIOAR_RDA_S) |
			(2 << MACMDIOAR_CR_S) | MACMDIOAR_WRITE | MACMDIOAR_GB);
		error = wait_clear(sc, ETH_MACMDIOAR, 1);
	}
	mtx_unlock(&sc->mdio_lock);
	return (error);
}
static void
link_update(device_t dev)
{
	struct yt_softc *sc = fromdev(dev);
	struct mii_data *mii;
	uint32_t cfg, old;
	uint64_t speed = 0;
	int link = LINK_STATE_DOWN;
	if (sc->miibus == NULL)
		return;
	mii = device_get_softc(sc->miibus);
	old = rd(sc, ETH_MACCR);
	cfg = old & ~(MACCR_PS | MACCR_FES | MACCR_DM);
	if ((mii->mii_media_status & IFM_ACTIVE) != 0 &&
	    (mii->mii_media_active & IFM_FDX) != 0) {
		switch (IFM_SUBTYPE(mii->mii_media_active)) {
		case IFM_1000_T:
			speed = IF_Gbps(1);
			break;
		case IFM_100_TX:
			cfg |= MACCR_PS | MACCR_FES;
			speed = IF_Mbps(100);
			break;
		case IFM_10_T:
			cfg |= MACCR_PS;
			speed = IF_Mbps(10);
			break;
		}
		if (speed != 0) {
			cfg |= MACCR_DM;
			link = LINK_STATE_UP;
		}
	}
	wr(sc, ETH_MACCR, cfg);
	if (link != sc->link) {
		sc->link = link;
		iflib_link_state_change(sc->ctx, link, speed);
	}
}
static int
media_change(if_t ifp)
{
	struct yt_softc *sc = iflib_get_softc(if_getsoftc(ifp));
	return (mii_mediachg(device_get_softc(sc->miibus)));
}
static void
media_status(if_t ifp, struct ifmediareq *req)
{
	struct yt_softc *sc = iflib_get_softc(if_getsoftc(ifp));
	struct mii_data *mii = device_get_softc(sc->miibus);
	mii_pollstat(mii);
	req->ifm_status = mii->mii_media_status;
	req->ifm_active = mii->mii_media_active;
}
static void
yt_intr_disable_locked(struct yt_softc *sc)
{
	mtx_assert(&sc->irq_lock, MA_OWNED);
	if (sc->mem != NULL) {
		wrapper_write(sc, YT_HOST_IRQ, 0xffff0000);
		wr(sc, ETH_DMACIER(0), 0);
	}
}
static void
yt_intr_disable(if_ctx_t ctx)
{
	struct yt_softc *sc = iflib_get_softc(ctx);

	mtx_lock_spin(&sc->irq_lock);
	yt_intr_disable_locked(sc);
	mtx_unlock_spin(&sc->irq_lock);
}
static void
yt_intr_enable(if_ctx_t ctx)
{
	struct yt_softc *sc = iflib_get_softc(ctx);
	uint32_t pending = 0;

	/* Legacy RX and TX tasks may rearm this shared interrupt on different
	 * CPUs. Serialize them with the filter's mask/ack sequence. */
	mtx_lock_spin(&sc->irq_lock);
	if (sc->running) {
		wr(sc, ETH_MACCR, rd(sc, ETH_MACCR) | MACCR_TE | MACCR_RE);
		wr(sc, ETH_DMACTXCR(0), rd(sc, ETH_DMACTXCR(0)) | DMACTXCR_ST);
		wr(sc, ETH_DMACRXCR(0), rd(sc, ETH_DMACRXCR(0)) | DMACRXCR_SR);
		/* Arm the host path before a pending DMA status can assert
		 * INTx. */
		wrapper_write(sc, YT_HOST_IRQ, 0xffee0000); /* RX0 + TX0 only */
		wr(sc, ETH_DMACIER(0), YT_IRQ_MASK);
		/* A completion while masked may have consumed the host edge.
		 * Recheck the DMA condition after arming, then let iflib drain
		 * it.
		 */
		pending = rd(sc, ETH_DMACSR(0));
		if (pending & (DMACSR_RI | DMACSR_TI | DMACSR_FBE)) {
			yt_intr_disable_locked(sc);
			wr(sc, ETH_DMACSR(0), pending & 0xffff);
			(void)bus_read_4(sc->mem, YT_HOST_IRQ);
			sc->rearm_pending++;
		}
	}
	mtx_unlock_spin(&sc->irq_lock);
	if (pending & (DMACSR_RI | DMACSR_TI | DMACSR_FBE)) {
		if (pending & DMACSR_FBE)
			iflib_request_reset(ctx);
		iflib_rx_intr_deferred(ctx, 0);
		iflib_tx_intr_deferred(ctx, 0);
	}
}
static int
legacy_intr(void *arg)
{
	struct yt_softc *sc = arg;
	uint32_t host, status;

	mtx_lock_spin(&sc->irq_lock);
	/* Reading the wrapper status acknowledges its latched host interrupt.
	 */
	host = bus_read_4(sc->mem, YT_HOST_IRQ);
	status = rd(sc, ETH_DMACSR(0));
	if ((status & YT_IRQ_MASK) == 0 && (host & 0x11) == 0) {
		mtx_unlock_spin(&sc->irq_lock);
		return (FILTER_STRAY);
	}
	yt_intr_disable_locked(sc);
	wr(sc, ETH_DMACSR(0), status & 0xffff);
	/* Release the host latch after its DMA interrupt condition is clear. */
	(void)bus_read_4(sc->mem, YT_HOST_IRQ);
	sc->irq_count++;
	sc->last_status = status;
	sc->last_host_irq = host;
	mtx_unlock_spin(&sc->irq_lock);
	if (status & DMACSR_FBE)
		iflib_request_reset(sc->ctx);
	return (FILTER_SCHEDULE_THREAD);
}
static void
filter_set(if_ctx_t ctx)
{
	struct yt_softc *sc = iflib_get_softc(ctx);
	int flags = if_getflags(iflib_get_ifp(ctx));
	/* Accept multicast without promising a hardware hash filter. */
	wr(sc, ETH_MACPFR, MACPFR_PM | ((flags & IFF_PROMISC) ? MACPFR_PR : 0));
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
	struct yt_softc *sc = iflib_get_softc(ctx);
	if (sc->mem == NULL)
		return;
	mtx_lock_spin(&sc->irq_lock);
	sc->running = false;
	yt_intr_disable_locked(sc);
	mtx_unlock_spin(&sc->irq_lock);
	wrapper_write(sc, YT_HOST_IRQ, 0xffff0000);
	wr(sc, ETH_DMACTXCR(0), rd(sc, ETH_DMACTXCR(0)) & ~DMACTXCR_ST);
	wr(sc, ETH_DMACRXCR(0), rd(sc, ETH_DMACRXCR(0)) & ~DMACRXCR_SR);
	wr(sc, ETH_MACCR, rd(sc, ETH_MACCR) & ~(MACCR_TE | MACCR_RE));
	wr(sc, ETH_DMAMR, DMAMR_SWR);
	if (wait_clear(sc, ETH_DMAMR, DMAMR_SWR) != 0) {
		pci_disable_busmaster(sc->dev);
		device_printf(sc->dev,
		    "DMA reset timeout; bus mastering disabled\n");
	}
}
static void
init(if_ctx_t ctx)
{
	struct yt_softc *sc = iflib_get_softc(ctx);
	uint32_t feat, txfifo, rxfifo;
	pci_enable_busmaster(sc->dev);
	wr(sc, ETH_DMAMR, DMAMR_SWR);
	if (wait_clear(sc, ETH_DMAMR, DMAMR_SWR) != 0) {
		pci_disable_busmaster(sc->dev);
		iflib_init_failed(ctx);
		return;
	}
	sc->resets++;
	sc->txprod = sc->txclean = sc->txpublish = 0;
	bzero(sc->tx, sizeof(*sc->tx) * sc->ntx);
	bzero(sc->rx_posted, sizeof(sc->rx_posted));
	wrapper_write(sc, YT_OOB, 1); /* Normal host DMA interrupt delivery */
	/* The wrapper has separate RSS steering which survives MAC reset. */
	wrapper_write(sc, YT_RSS_CTRL, 0);
	wrapper_write(sc, YT_MODERATE,
	    0); /* No interrupt moderation initially */
	wrapper_write(sc, YT_HOST_IRQ, 0xffff0000);
	wr(sc, ETH_DMASBMR,
	    DMASBMR_BLEN4 | DMASBMR_BLEN8 | DMASBMR_BLEN16 | DMASBMR_EAME);
	wr(sc, ETH_DMACCR(0), 0);
	wr(sc, ETH_DMACTXCR(0), (32 << DMACTXCR_TXPBL_S) | DMACTXCR_OSF);
	wr(sc, ETH_DMACRXCR(0),
	    (32 << DMACRXCR_RXPBL_S) | (2048 << DMACRXCR_RBSZ_S));
	wr(sc, ETH_DMACTXDLAR_HI(0), 0);
	wr(sc, ETH_DMACTXDLAR(0), sc->txaddr);
	wr(sc, ETH_DMACRXDLAR_HI(0), 0);
	wr(sc, ETH_DMACRXDLAR(0), sc->rxaddr);
	wr(sc, ETH_DMACTXRLR(0), sc->ntx - 1);
	wr(sc, ETH_DMACRXRLR(0), sc->nrx - 1);
	wr(sc, ETH_DMACTXDTPR(0), sc->txaddr);
	wr(sc, ETH_DMACRXDTPR(0), sc->rxaddr);
	/* Give all FIFO storage to the single enabled queue. */
	feat = rd(sc, ETH_MACHWF1R);
	if ((feat & 31) == 0 || ((feat >> 6) & 31) == 0) {
		iflib_init_failed(ctx);
		return;
	}
	rxfifo = (1U << MIN(feat & 31, 5U)) - 1;
	txfifo = (1U << MIN((feat >> 6) & 31, 5U)) - 1;
	rxfifo = (rxfifo + 1) / 2 - 1;
	txfifo = (txfifo + 1) / 2 - 1;
	wr(sc, ETH_MTLTXQOMR(0),
	    MTLTXQOMR_TSF | MTLTXQOMR_EN | (txfifo << MTLTXQOMR_TQS_S));
	wr(sc, ETH_MTLRXQOMR(0), MTLRXQOMR_RSF | (rxfifo << MTLRXQOMR_RQS_S));
	wr(sc, ETH_MTLRXQDMAMR, 0); /* RX queue0 -> channel0 */
	wr(sc, ETH_MACRXQC0R(0), MACRXQC0R_RXQ0EN_GEN);
	wr(sc, ETH_MACRXQC2R, 0xff); /* All VLAN priorities -> queue0 */
	wr(sc, 0x0c80, 0);	     /* DWMAC4 RSS control, disabled */
	wr(sc, ETH_MACA0HR, sc->mac[4] | (sc->mac[5] << 8));
	wr(sc, ETH_MACA0LR, le32dec(sc->mac));
	wr(sc, ETH_MACIER, 0);
	wr(sc, ETH_MMC_RX_INTERRUPT_MASK, 0xffffffff);
	wr(sc, ETH_MMC_TX_INTERRUPT_MASK, 0xffffffff);
	wr(sc, 0x800, 0xffffffff); /* MMC IPC interrupt mask */
	wr(sc, ETH_MACQ0TXFCR, 0);
	wr(sc, ETH_MACRXFCR, 0);
	wr(sc, ETH_MACCR,
	    MACCR_DM | MACCR_ACS | (1U << 21)); /* Strip CRC on all frames */
	filter_set(ctx);
	wr(sc, ETH_DMACSR(0), 0xffff);
	mtx_lock_spin(&sc->irq_lock);
	sc->running = true; /* iflib fills RX before enabling DMA/interrupts */
	mtx_unlock_spin(&sc->irq_lock);
	mii_mediachg(device_get_softc(sc->miibus));
}
static int
tx_alloc(if_ctx_t ctx, caddr_t *v, uint64_t *p, int nrings, int nsets)
{
	struct yt_softc *sc = iflib_get_softc(ctx);
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
	struct yt_softc *sc = iflib_get_softc(ctx);
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
	struct yt_softc *sc = iflib_get_softc(ctx);
	sc->tx = sc->rx = NULL;
}
static int
tx_encap(void *arg, if_pkt_info_t pi)
{
	struct yt_softc *sc = arg;
	qidx_t idx = pi->ipi_pidx;
	if (pi->ipi_nsegs < 1 || pi->ipi_nsegs > 16 ||
	    pi->ipi_len > YT_FRAME_MAX)
		return (EFBIG);
	for (unsigned j = 0; j < pi->ipi_nsegs; j++)
		if (pi->ipi_segs[j].ds_addr > UINT32_MAX)
			return (EFBIG);
	for (unsigned j = 0; j < pi->ipi_nsegs; j++) {
		struct yt_desc *d = &sc->tx[idx];
		d->buffer = htole32(pi->ipi_segs[j].ds_addr);
		d->high = 0;
		d->control = htole32(pi->ipi_segs[j].ds_len |
		    (j == pi->ipi_nsegs - 1 ? YT_TX_INT : 0));
		d->status = htole32((j == 0 ? YT_FIRST | pi->ipi_len : 0) |
		    (j == pi->ipi_nsegs - 1 ? YT_LAST : 0));
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
	struct yt_softc *sc = arg;
	qidx_t i = pidx;
	/* Packet PREWRITE is complete. Publish tails before packet heads. */
	while (i != sc->txpublish) {
		i = (i + sc->ntx - 1) % sc->ntx;
		atomic_thread_fence_rel();
		sc->tx[i].status |= htole32(YT_OWN);
	}
	sc->txpublish = pidx;
	atomic_thread_fence_seq_cst();
	wr(sc, ETH_DMACTXDTPR(0), sc->txaddr + pidx * sizeof(struct yt_desc));
}
static int
tx_credits(void *arg, uint16_t q, bool clear)
{
	struct yt_softc *sc = arg;
	qidx_t i = sc->txclean;
	int n = 0;
	while (i != sc->txpublish) {
		uint32_t status = le32toh(sc->tx[i].status);
		if (status & YT_OWN)
			break;
		if (!clear)
			return (1);
		if (status & YT_ERROR) {
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
	struct yt_softc *sc = arg;
	int n = 0;
	while (n < budget && n < sc->nrx &&
	    (le32toh(sc->rx[idx].status) & YT_OWN) == 0 &&
	    sc->rx_posted[idx] != 0) {
		n++;
		idx = (idx + 1) % sc->nrx;
	}
	return (n);
}
static int
rx_get(void *arg, if_rxd_info_t ri)
{
	struct yt_softc *sc = arg;
	qidx_t i = ri->iri_cidx;
	uint32_t status = le32toh(sc->rx[i].status);
	sc->last_rx = status;
	unsigned len = status & 0x7fff;
	if ((status & (YT_OWN | (1U << 15) | YT_FIRST | YT_LAST)) !=
		(YT_FIRST | YT_LAST) ||
	    len < ETHER_HDR_LEN || len > YT_FRAME_MAX) {
		sc->rx_errors++;
		return (EBADMSG);
	}
	sc->rx_posted[i] = 0;
	ri->iri_len = len;
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
	struct yt_softc *sc = arg;
	for (unsigned j = 0; j < u->iru_count; j++) {
		qidx_t i = (u->iru_pidx + j) % sc->nrx;
		struct yt_desc *d = &sc->rx[i];
		sc->rx_ids[i] = u->iru_idxs[j];
		KASSERT(u->iru_paddrs[j] <= UINT32_MAX, ("yt RX address"));
		d->buffer = htole32(u->iru_paddrs[j]);
		d->high = 0;
		d->control = 0;
		atomic_thread_fence_rel();
		sc->rx_posted[i] = 1;
		d->status = htole32(YT_OWN | YT_RX_INT | YT_RX_VALID);
	}
}
static void
rx_flush(void *arg, uint16_t q, uint8_t fl, qidx_t pidx)
{
	struct yt_softc *sc = arg;
	wr(sc, ETH_DMACRXDTPR(0), sc->rxaddr + pidx * sizeof(struct yt_desc));
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
	struct yt_softc *sc = iflib_get_softc(ctx);
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
	if (mtx_initialized(&sc->irq_lock))
		mtx_destroy(&sc->irq_lock);
	return (0);
}
static int
attach_pre(if_ctx_t ctx)
{
	struct yt_softc *sc = iflib_get_softc(ctx);
	if_softc_ctx_t c = iflib_get_softc_ctx(ctx);
	uint32_t low, high, version;
	int error;
	sc->ctx = ctx;
	sc->dev = iflib_get_dev(ctx);
	sc->link = LINK_STATE_UNKNOWN;
	mtx_init(&sc->mdio_lock, "yt MDIO", NULL, MTX_DEF);
	mtx_init(&sc->irq_lock, "yt interrupt", NULL, MTX_SPIN);
	sc->rid = PCIR_BAR(0);
	sc->mem = bus_alloc_resource_any(sc->dev, SYS_RES_MEMORY, &sc->rid,
	    RF_ACTIVE);
	if (sc->mem == NULL) {
		error = ENXIO;
		goto fail;
	}
	wrapper_write(sc, YT_PHY, bus_read_4(sc->mem, YT_PHY) | 1);
	wrapper_write(sc, YT_OOB, 1);
	version = rd(sc, ETH_MACVR);
	device_printf(sc->dev, "GMAC version=%#x features=%#x BAR0=%#jx\n",
	    version, rd(sc, ETH_MACHWF0R), (uintmax_t)rman_get_start(sc->mem));
	/* YT6801 uses DWMAC4; refuse an unknown register layout. */
	if ((version & 0xff) < 0x40 || (version & 0xff) > 0x5f) {
		error = ENXIO;
		goto fail;
	}
	low = rd(sc, ETH_MACA0LR);
	high = rd(sc, ETH_MACA0HR);
	le32enc(sc->mac, low);
	le16enc(sc->mac + 4, high);
	if (ETHER_IS_MULTICAST(sc->mac) || ETHER_IS_ZERO(sc->mac)) {
		device_printf(sc->dev, "invalid firmware MAC address\n");
		error = ENXIO;
		goto fail;
	}
	yt_intr_disable(ctx);
	pci_enable_busmaster(sc->dev);
	c->isc_txrx = &txrx;
	c->isc_tx_nsegments = 16;
	c->isc_txqsizes[0] = c->isc_ntxd[0] * sizeof(struct yt_desc);
	c->isc_rxqsizes[0] = c->isc_nrxd[0] * sizeof(struct yt_desc);
	c->isc_ntxqsets = c->isc_nrxqsets = 1;
	c->isc_ntxqsets_max = c->isc_nrxqsets_max = 1;
	c->isc_msix_bar =
	    0; /* INTx via ACPI; MSI is not implemented on this port. */
	c->isc_dma_width = 32;
	c->isc_max_frame_size = YT_FRAME_MAX;
	c->isc_capabilities = c->isc_capenable = IFCAP_VLAN_MTU;
	iflib_set_mac(ctx, sc->mac);
	error = mii_attach(sc->dev, &sc->miibus, iflib_get_ifp(ctx),
	    media_change, media_status, BMSR_DEFCAPMASK, MII_PHY_ANY,
	    MII_OFFSET_ANY, 0);
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
	struct yt_softc *sc = arg1;
	uint32_t val = rd(sc, arg2);
	return (sysctl_handle_32(oidp, &val, 0, req));
}
static int
attach_post(if_ctx_t ctx)
{
	struct yt_softc *sc = iflib_get_softc(ctx);
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
	SYSCTL_ADD_U64(s, l, OID_AUTO, "rearm_pending", CTLFLAG_RD,
	    &sc->rearm_pending, 0, "DMA completions found while rearming");
	SYSCTL_ADD_U32(s, l, OID_AUTO, "last_status", CTLFLAG_RD,
	    &sc->last_status, 0, "Last DMA IRQ status");
	SYSCTL_ADD_PROC(s, l, OID_AUTO, "mac_config",
	    CTLTYPE_UINT | CTLFLAG_RD | CTLFLAG_MPSAFE, sc, ETH_MACCR,
	    reg_sysctl, "IU", "Hardware register");
	SYSCTL_ADD_PROC(s, l, OID_AUTO, "dma_bus_mode",
	    CTLTYPE_UINT | CTLFLAG_RD | CTLFLAG_MPSAFE, sc, ETH_DMAMR,
	    reg_sysctl, "IU", "Hardware register");
	SYSCTL_ADD_PROC(s, l, OID_AUTO, "dma_status",
	    CTLTYPE_UINT | CTLFLAG_RD | CTLFLAG_MPSAFE, sc, ETH_DMACSR(0),
	    reg_sysctl, "IU", "Hardware register");
	SYSCTL_ADD_PROC(s, l, OID_AUTO, "dma_irq_enable",
	    CTLTYPE_UINT | CTLFLAG_RD | CTLFLAG_MPSAFE, sc, ETH_DMACIER(0),
	    reg_sysctl, "IU", "DMA interrupt enable register");
	SYSCTL_ADD_PROC(s, l, OID_AUTO, "dma_opmode",
	    CTLTYPE_UINT | CTLFLAG_RD | CTLFLAG_MPSAFE, sc, ETH_DMACRXCR(0),
	    reg_sysctl, "IU", "Hardware register");
	SYSCTL_ADD_U32(s, l, OID_AUTO, "last_tx", CTLFLAG_RD, &sc->last_tx, 0,
	    "Last TX descriptor");
	SYSCTL_ADD_U32(s, l, OID_AUTO, "last_rx", CTLFLAG_RD, &sc->last_rx, 0,
	    "Last RX descriptor");
	SYSCTL_ADD_U32(s, l, OID_AUTO, "last_host_irq", CTLFLAG_RD,
	    &sc->last_host_irq, 0, "Last wrapper interrupt status");
	return (0);
}
static void
admin_update(if_ctx_t ctx)
{
	struct yt_softc *sc = iflib_get_softc(ctx);
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
	struct yt_softc *sc = iflib_get_softc(ctx);
	if (c == IFCOUNTER_IERRORS)
		return (sc->rx_errors);
	if (c == IFCOUNTER_OERRORS)
		return (sc->tx_errors);
	return (if_get_counter_default(iflib_get_ifp(ctx), c));
}
static device_method_t if_methods[] = { DEVMETHOD(ifdi_attach_pre, attach_pre),
	DEVMETHOD(ifdi_attach_post, attach_post),
	DEVMETHOD(ifdi_detach, detach), DEVMETHOD(ifdi_init, init),
	DEVMETHOD(ifdi_stop, stop), DEVMETHOD(ifdi_intr_enable, yt_intr_enable),
	DEVMETHOD(ifdi_intr_disable, yt_intr_disable),
	DEVMETHOD(ifdi_tx_queues_alloc, tx_alloc),
	DEVMETHOD(ifdi_rx_queues_alloc, rx_alloc),
	DEVMETHOD(ifdi_queues_free, queues_free),
	DEVMETHOD(ifdi_multi_set, filter_set),
	DEVMETHOD(ifdi_promisc_set, promisc_set),
	DEVMETHOD(ifdi_mtu_set, mtu_set), DEVMETHOD(ifdi_timer, timer),
	DEVMETHOD(ifdi_update_admin_status, admin_update),
	DEVMETHOD(ifdi_get_counter, get_counter), DEVMETHOD_END };
static driver_t if_driver = { "yt_if", if_methods, sizeof(struct yt_softc) };
static const pci_vendor_info_t vendors[] = {
	PVID(0x1f0a, 0x6801, "Motorcomm YT6801 Gigabit Ethernet"), PVID_END
};
static struct if_shared_ctx shared = {
	.isc_magic = IFLIB_MAGIC,
	.isc_driver = &if_driver,
	.isc_q_align = 4096,
	.isc_tx_maxsize = YT_FRAME_MAX,
	.isc_tx_maxsegsize = YT_FRAME_MAX,
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
yt_register(device_t dev)
{
	return (&shared);
}
static device_method_t methods[] = { DEVMETHOD(device_register, yt_register),
	DEVMETHOD(device_probe, iflib_device_probe),
	DEVMETHOD(device_attach, iflib_device_attach),
	DEVMETHOD(device_detach, iflib_device_detach),
	DEVMETHOD(device_shutdown, iflib_device_shutdown),
	DEVMETHOD(miibus_readreg, mdio_read),
	DEVMETHOD(miibus_writereg, mdio_write),
	DEVMETHOD(miibus_statchg, link_update),
	DEVMETHOD(miibus_linkchg, link_update), DEVMETHOD_END };
static driver_t driver = { "yt", methods, sizeof(struct yt_softc) };
DRIVER_MODULE(miibus, yt, miibus_driver, 0, 0);
DRIVER_MODULE(yt, pci, driver, 0, 0);
MODULE_DEPEND(yt, pci, 1, 1, 1);
MODULE_DEPEND(yt, ether, 1, 1, 1);
MODULE_DEPEND(yt, iflib, 1, 1, 1);
MODULE_DEPEND(yt, miibus, 1, 1, 1);
MODULE_VERSION(yt, 1);
IFLIB_PNP_INFO(pci, yt, vendors);
