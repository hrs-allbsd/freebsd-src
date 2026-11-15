/*-
 * SPDX-License-Identifier: BSD-2-Clause
 * Copyright (c) 2026 Hiroki Sato <hrs@FreeBSD.org>
 */
/* RK3528 crypto-v4: private DMA buffers, one serialized asynchronous queue.
 * Register facts: Rockchip's crypto-v3/v4 BSP (references/rk3528-crypto).
 * No caller-owned memory is ever mapped for DMA.
 */
#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/condvar.h>
#include <sys/endian.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/rman.h>
#include <sys/sysctl.h>
#include <sys/taskqueue.h>
#include <machine/bus.h>
#include <dev/clk/clk.h>
#include <dev/hwreset/hwreset.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>
#include <opencrypto/cryptodev.h>
#include <crypto/sha2/sha256.h>
#include "cryptodev_if.h"

#define RKCR_CHUNK	65536
#define RKCR_HASH_MAX	262144
#define RKCR_RESET	0x004
#define RKCR_INT_EN	0x008
#define RKCR_INT_ST	0x00c
#define RKCR_DMA_CTL	0x010
#define RKCR_LLI_ADDR	0x014
#define RKCR_DMA_ST	0x018
#define RKCR_FIFO	0x040
#define RKCR_BC		0x044
#define RKCR_HASH	0x048
#define RKCR_CIPHER_ST	0x04c
#define RKCR_IV		0x100
#define RKCR_IV_LEN	0x300
#define RKCR_KEY	0x180
#define RKCR_HASH_OUT	0x3a0
#define RKCR_HASH_VALID	0x3e4
#define RKCR_MID_VALID	0x3e8
#define RKCR_MID_SWITCH	0x630
#define RKCR_AES_VERSION 0x680
#define RKCR_HASH_VERSION 0x68c
#define RKCR_VERSION	0x6f0
#define RKCR_HASH_MID	0x700
#define RKCR_MASK	0xffff0000U
#define RKCR_DST_DONE	(1U << 1)
#define RKCR_SRC_DONE	(1U << 2)
#define RKCR_ERRORS	((0xfU << 3) | (1U << 9))
#define RKCR_LAST	1U
#define RKCR_PAUSE	2U
#define RKCR_DESC_DST	(1U << 9)
#define RKCR_DESC_SRC	(1U << 10)
#define RKCR_USER_FIRST	3U
#define RKCR_USER_LAST	4U

struct rkcrypto_desc {
	uint32_t src, slen, dst, dlen, user, reserved, control, next;
};
struct rkcrypto_dma {
	struct rkcrypto_desc desc;
	uint8_t pad[32];
	uint8_t in[RKCR_HASH_MAX];
	uint8_t out[RKCR_CHUNK];
};
CTASSERT(sizeof(struct rkcrypto_desc) == 32);
CTASSERT(offsetof(struct rkcrypto_dma, in) == 64);

struct rkcrypto_session { uint8_t key[32]; };
struct rkcrypto_softc {
	device_t dev;
	struct resource *mem, *irq;
	void *cookie;
	clk_t clocks[3];
	unsigned enabled;
	hwreset_t reset;
	struct mtx lock;
	struct cv cv;
	struct taskqueue *tq;
	struct task task;
	struct cryptopq queue;
	unsigned queued, sessions;
	bool active, working, detaching, fault, pinned, selftesting;
	uint32_t status, version, aes_version, hash_version;
	int32_t cid;
	bus_dma_tag_t tag;
	bus_dmamap_t map;
	struct rkcrypto_dma *dma;
	bus_addr_t address;
	int map_error;
	bool mapped;
	uint64_t requests, completed, errors, interrupts, dma_ops;
	uint64_t cipher_bytes, hash_bytes, timeouts, reset_failures, selftests;
	uint64_t hash_software_requests, hash_software_bytes;
};
#define RD(sc,r) bus_read_4((sc)->mem,(r))
#define WR(sc,r,v) bus_write_4((sc)->mem,(r),(v))

/* Publish failure to concurrent session/process entry points. */
static void
rkcrypto_disable(struct rkcrypto_softc *sc)
{
	mtx_lock(&sc->lock);
	sc->fault = true;
	mtx_unlock(&sc->lock);
}

/* The shared engine reset excludes the separate RKRNG device. */
static int
rkcrypto_reset(struct rkcrypto_softc *sc)
{
	bool asserted;

	WR(sc, RKCR_INT_EN, 0);
	WR(sc, RKCR_RESET, 0x00010001);
	for (unsigned i = 0; i < 1000; i++) {
		if ((RD(sc, RKCR_RESET) & 1) == 0 &&
		    (RD(sc, RKCR_DMA_ST) & 1) == 0) {
			WR(sc, RKCR_BC, RKCR_MASK);
			WR(sc, RKCR_HASH, RKCR_MASK);
			WR(sc, RKCR_INT_ST, RD(sc, RKCR_INT_ST));
			return (0);
		}
		DELAY(1);
	}
	sc->reset_failures++;
	rkcrypto_disable(sc);
	/* An independently verified CRU reset is the final DMA fence. */
	sc->pinned = true;
	if (hwreset_assert(sc->reset) == 0 &&
	    hwreset_is_asserted(sc->reset, &asserted) == 0 && asserted)
		sc->pinned = false;
	device_printf(sc->dev, "reset failed; disabled, DMA buffers %s\n",
	    sc->pinned ? "pinned" : "stopped by CRU reset");
	return (EIO);
}

static void
rkcrypto_interrupt(void *arg)
{
	struct rkcrypto_softc *sc = arg;
	uint32_t status;

	mtx_lock(&sc->lock);
	status = RD(sc, RKCR_INT_ST);
	WR(sc, RKCR_INT_ST, status);
	if (status != 0) {
		sc->interrupts++;
		if (sc->active) {
			sc->status |= status;
			cv_signal(&sc->cv);
		}
	}
	mtx_unlock(&sc->lock);
}

/* Hardware consumes little-endian descriptors, big-endian key/IV words. */
static void
rkcrypto_words(struct rkcrypto_softc *sc, unsigned base,
    const uint8_t *data, unsigned length)
{
	for (unsigned i = 0; i < length; i += 4)
		WR(sc, base + i, be32dec(data + i));
}

/* One complete DMA list; partial hash continuation is deliberately unused. */
static int
rkcrypto_dma_run(struct rkcrypto_softc *sc, unsigned length, bool hash)
{
	struct rkcrypto_desc *d = &sc->dma->desc;
	uint32_t done = hash ? RKCR_SRC_DONE : RKCR_DST_DONE;
	uint32_t start = 1U;
	int error = 0;
	sbintime_t deadline;

	*d = (struct rkcrypto_desc) {
		.src = htole32(sc->address + offsetof(struct rkcrypto_dma, in)),
		.slen = htole32(length),
		.dst = hash ? 0 : htole32(sc->address +
		    offsetof(struct rkcrypto_dma, out)),
		.dlen = hash ? 0 : htole32(length),
		.user = htole32(RKCR_USER_FIRST | RKCR_USER_LAST),
		.control = htole32(RKCR_LAST |
		    (hash ? RKCR_DESC_SRC : RKCR_DESC_DST)),
	};
	bus_dmamap_sync(sc->tag, sc->map,
	    BUS_DMASYNC_PREWRITE | BUS_DMASYNC_PREREAD);
	mtx_lock(&sc->lock);
	sc->status = 0;
	sc->active = true;
	WR(sc, RKCR_INT_ST, RD(sc, RKCR_INT_ST));
	WR(sc, RKCR_INT_EN, done | RKCR_ERRORS);
	WR(sc, RKCR_LLI_ADDR, sc->address);
	bus_barrier(sc->mem, 0, 0x800, BUS_SPACE_BARRIER_WRITE);
	WR(sc, RKCR_DMA_CTL, (start << 16) | start);
	sc->dma_ops++;
	deadline = sbinuptime() + SBT_1S;
	while ((sc->status & (done | RKCR_ERRORS)) == 0) {
		if (sc->selftesting) {
			sc->status |= RD(sc, RKCR_INT_ST);
			if (sc->status & (done | RKCR_ERRORS))
				break;
			if (sbinuptime() < deadline) {
				DELAY(10);
				continue;
			}
			error = ETIMEDOUT;
			sc->timeouts++;
			break;
		}
		if (cv_timedwait_sbt(&sc->cv, &sc->lock, deadline, 0,
		    C_ABSOLUTE) == EWOULDBLOCK) {
			error = ETIMEDOUT;
			sc->timeouts++;
			break;
		}
	}
	/* Errors win over a simultaneously observed completion. */
	if (sc->status & RKCR_ERRORS)
		error = EIO;
	WR(sc, RKCR_INT_EN, 0);
	WR(sc, RKCR_INT_ST, RD(sc, RKCR_INT_ST));
	sc->active = false;
	mtx_unlock(&sc->lock);
	if (error == 0) {
		unsigned i;

		for (i = 0; i < 10000; i++) {
			if ((RD(sc, RKCR_DMA_ST) & 1) == 0)
				break;
			DELAY(1);
		}
		if (i == 10000) {
			error = ETIMEDOUT;
			sc->timeouts++;
		}
	}
	if (error != 0) {
		rkcrypto_disable(sc);
		(void)rkcrypto_reset(sc);
	}
	if (!sc->pinned)
		bus_dmamap_sync(sc->tag, sc->map,
		    BUS_DMASYNC_POSTREAD | BUS_DMASYNC_POSTWRITE);
	return (error);
}

static int
rkcrypto_cipher_chunk(struct rkcrypto_softc *sc, int alg, const uint8_t *key,
    unsigned klen, const uint8_t iv[16], unsigned length, bool encrypt)
{
	uint32_t control;
	int error;

	if ((error = rkcrypto_reset(sc)) != 0)
		return (error);
	control = (alg == CRYPTO_AES_CBC ? 0x10U : 0x30U) |
	    (klen == 16 ? 0 : klen == 24 ? 4U : 8U) | 1U;
	if (!encrypt && alg == CRYPTO_AES_CBC)
		control |= 2U;
	rkcrypto_words(sc, RKCR_KEY, key, klen);
	rkcrypto_words(sc, RKCR_IV, iv, 16);
	WR(sc, RKCR_IV_LEN, 16);
	WR(sc, RKCR_FIFO, 0x00030003);
	WR(sc, RKCR_BC, RKCR_MASK | control);
	error = rkcrypto_dma_run(sc, length, false);
	if (error == 0)
		error = rkcrypto_reset(sc);
	return (error);
}

static void
rkcrypto_counter_add(uint8_t iv[16], unsigned blocks)
{
	unsigned carry = blocks;

	for (int i = 15; i >= 0 && carry != 0; i--) {
		carry += iv[i];
		iv[i] = carry & 255;
		carry >>= 8;
	}
}

static int
rkcrypto_hash_init(struct rkcrypto_softc *sc)
{
	int error;

	if ((error = rkcrypto_reset(sc)) != 0)
		return (error);
	for (unsigned i = 0; i < 16; i++)
		WR(sc, RKCR_HASH_OUT + i * 4, 0);
	WR(sc, RKCR_FIFO, 0x00030003);
	WR(sc, RKCR_HASH, RKCR_MASK | 0x24U); /* SHA256, HW padding */
	WR(sc, RKCR_MID_SWITCH, 0x00010001);
	WR(sc, RKCR_MID_VALID, 0x00020002);
	for (unsigned i = 0; i < 26; i++)
		WR(sc, RKCR_HASH_MID + i * 4, 0);
	WR(sc, RKCR_HASH, 0x00010001);
	return (0);
}

static int
rkcrypto_hash_result(struct rkcrypto_softc *sc, uint8_t digest[32])
{
	unsigned waited;

	/* SRC_DONE fences the DMA input, not the hash compression pipeline. */
	for (waited = 0; waited < 10000; waited++) {
		if ((RD(sc, RKCR_CIPHER_ST) & 2) == 0)
			break;
		DELAY(1);
	}
	if (waited == 10000) {
		sc->timeouts++;
		rkcrypto_disable(sc);
		(void)rkcrypto_reset(sc);
		return (ETIMEDOUT);
	}
	for (unsigned i = 0; i < 8; i++)
		be32enc(digest + i * 4, RD(sc, RKCR_HASH_OUT + i * 4));
	WR(sc, RKCR_HASH_VALID, 1);
	return (0);
}

static int
rkcrypto_sha_update(void *arg, const void *data, u_int length)
{
	SHA256_Update(arg, data, length);
	return (0);
}

static int
rkcrypto_execute(struct rkcrypto_softc *sc, struct cryptop *crp)
{
	static const uint8_t empty_sha256[32] = {
		0xe3,0xb0,0xc4,0x42,0x98,0xfc,0x1c,0x14,
		0x9a,0xfb,0xf4,0xc8,0x99,0x6f,0xb9,0x24,
		0x27,0xae,0x41,0xe4,0x64,0x9b,0x93,0x4c,
		0xa4,0x95,0x99,0x1b,0x78,0x52,0xb8,0x55 };
	const struct crypto_session_params *csp = crypto_get_params(crp->crp_session);
	struct rkcrypto_session *session = crypto_get_driver_session(crp->crp_session);
	uint8_t iv[16] = {}, key[32] = {}, digest[32], expected[32];
	SHA256_CTX hash_context;
	unsigned offset, amount, rounded, length, mlen;
	bool hash = csp->csp_mode == CSP_MODE_DIGEST;
	int error = 0, output;
	size_t inlen = crypto_buffer_len(&crp->crp_buf);
	size_t outlen = CRYPTO_HAS_OUTPUT_BUFFER(crp) ?
	    crypto_buffer_len(&crp->crp_obuf) : inlen;

	if (sc->fault)
		return (EIO);
	if (crp->crp_payload_length < 0 || crp->crp_payload_start < 0 ||
	    (size_t)crp->crp_payload_start > crypto_buffer_len(&crp->crp_buf) ||
	    (size_t)crp->crp_payload_length > crypto_buffer_len(&crp->crp_buf) -
	    crp->crp_payload_start)
		return (EINVAL);
	length = crp->crp_payload_length;
	output = CRYPTO_HAS_OUTPUT_BUFFER(crp) ? crp->crp_payload_output_start :
	    crp->crp_payload_start;
	mlen = csp->csp_auth_mlen != 0 ? csp->csp_auth_mlen : 32;
	if (!hash) {
		if (output < 0 || (size_t)output > outlen || length > outlen - output ||
		    (csp->csp_cipher_key == NULL && crp->crp_cipher_key == NULL) ||
		    (!(crp->crp_flags & CRYPTO_F_IV_SEPARATE) &&
		    (crp->crp_iv_start < 0 || (size_t)crp->crp_iv_start > inlen ||
		    16 > inlen - crp->crp_iv_start)))
			return (EINVAL);
		if (csp->csp_cipher_alg == CRYPTO_AES_CBC && (length % 16) != 0)
			return (EINVAL);
		crypto_read_iv(crp, iv);
		memcpy(key, crp->crp_cipher_key != NULL ? crp->crp_cipher_key :
		    session->key, csp->csp_cipher_klen);
	} else {
		size_t limit = (crp->crp_op & CRYPTO_OP_VERIFY_DIGEST) ? inlen : outlen;

		if (crp->crp_digest_start < 0 || (size_t)crp->crp_digest_start > limit ||
		    mlen > limit - crp->crp_digest_start)
			return (EINVAL);
	}
	if (hash) {
		if (length == 0) {
			memcpy(digest, empty_sha256, 32);
		} else if (length > RKCR_HASH_MAX) {
			/* OCF has no per-request-size probe. Keep large kernel hashes
			 * correct without unbounded contiguous DMA allocations.
			 * /dev/crypto's maximum request is below this threshold.
			 */
			SHA256_Init(&hash_context);
			error = crypto_apply(crp, crp->crp_payload_start, length,
			    rkcrypto_sha_update, &hash_context);
			if (error != 0) goto out;
			SHA256_Final(digest, &hash_context);
			sc->hash_software_requests++;
			sc->hash_software_bytes += length;
		} else {
			crypto_copydata(crp, crp->crp_payload_start, length, sc->dma->in);
			if ((error = rkcrypto_hash_init(sc)) != 0) goto out;
			if ((error = rkcrypto_dma_run(sc, length, true)) != 0) goto out;
			if ((error = rkcrypto_hash_result(sc, digest)) != 0) goto out;
			if ((error = rkcrypto_reset(sc)) != 0) goto out;
			sc->hash_bytes += length;
		}
	} else for (offset = 0; offset < length; offset += amount) {
		amount = MIN(RKCR_CHUNK, length - offset);
		rounded = roundup2(amount, 16);
		memset(sc->dma->in, 0, rounded);
		crypto_copydata(crp, crp->crp_payload_start + offset, amount, sc->dma->in);
		error = rkcrypto_cipher_chunk(sc, csp->csp_cipher_alg,
		    key, csp->csp_cipher_klen, iv, rounded,
		    CRYPTO_OP_IS_ENCRYPT(crp->crp_op));
		if (error != 0) goto out;
		if (csp->csp_cipher_alg == CRYPTO_AES_CBC)
			memcpy(iv, CRYPTO_OP_IS_ENCRYPT(crp->crp_op) ?
			    sc->dma->out + amount - 16 : sc->dma->in + amount - 16, 16);
		else
			rkcrypto_counter_add(iv, rounded / 16);
		crypto_copyback(crp, output + offset, amount, sc->dma->out);
		sc->cipher_bytes += amount;
	}
	if (hash) {
		if (crp->crp_op & CRYPTO_OP_VERIFY_DIGEST) {
			crypto_copydata(crp, crp->crp_digest_start, mlen, expected);
			if (timingsafe_bcmp(digest, expected, mlen) != 0)
				error = EBADMSG;
		} else {
			crypto_copyback(crp, crp->crp_digest_start, mlen, digest);
		}
	}
out:
	/* Never CPU-touch memory whose DMA stop could not be established. */
	if (!sc->pinned) {
		explicit_bzero(sc->dma, sizeof(*sc->dma));
		if (!sc->fault) {
			for (unsigned i = 0; i < 8; i++)
				WR(sc, RKCR_KEY + i * 4, 0);
			for (unsigned i = 0; i < 4; i++)
				WR(sc, RKCR_IV + i * 4, 0);
		}
	}
	explicit_bzero(iv, sizeof(iv));
	explicit_bzero(key, sizeof(key));
	explicit_bzero(digest, sizeof(digest));
	explicit_bzero(expected, sizeof(expected));
	explicit_bzero(&hash_context, sizeof(hash_context));
	return (error);
}

static void
rkcrypto_worker(void *arg, int pending __unused)
{
	struct rkcrypto_softc *sc = arg;
	struct cryptop *crp;

	mtx_lock(&sc->lock);
	sc->working = true;
	while ((crp = TAILQ_FIRST(&sc->queue)) != NULL) {
		TAILQ_REMOVE(&sc->queue, crp, crp_next);
		sc->queued--;
		mtx_unlock(&sc->lock);
		crp->crp_etype = rkcrypto_execute(sc, crp);
		if (crp->crp_etype != 0)
			sc->errors++;
		else
			sc->completed++;
		crypto_done(crp);
		mtx_lock(&sc->lock);
	}
	sc->working = false;
	mtx_unlock(&sc->lock);
}

static int
rkcrypto_probesession(device_t dev, const struct crypto_session_params *csp)
{
	struct rkcrypto_softc *sc = device_get_softc(dev);

	if (sc->fault || sc->detaching ||
	    (csp->csp_flags & ~CSP_F_SEPARATE_OUTPUT) != 0)
		return (EINVAL);
	if (csp->csp_mode == CSP_MODE_CIPHER) {
		if ((csp->csp_cipher_alg != CRYPTO_AES_CBC &&
		    csp->csp_cipher_alg != CRYPTO_AES_ICM) ||
		    csp->csp_ivlen != 16 || (csp->csp_cipher_klen != 16 &&
		    csp->csp_cipher_klen != 24 && csp->csp_cipher_klen != 32))
			return (EINVAL);
		if ((sc->aes_version & (csp->csp_cipher_alg == CRYPTO_AES_CBC ?
		    (1U << 1) : (1U << 3))) == 0)
			return (EINVAL);
		if ((sc->aes_version & (1U <<
		    (16 + (csp->csp_cipher_klen - 16) / 8))) == 0)
			return (EINVAL);
	} else if (csp->csp_mode == CSP_MODE_DIGEST) {
		if (csp->csp_auth_alg != CRYPTO_SHA2_256 ||
		    csp->csp_auth_klen != 0 || csp->csp_auth_mlen < 0 ||
		    csp->csp_auth_mlen > 32 || (sc->hash_version & 4) == 0)
			return (EINVAL);
	} else {
		return (EINVAL);
	}
	return (CRYPTODEV_PROBE_HARDWARE);
}

static int
rkcrypto_newsession(device_t dev, crypto_session_t cses,
    const struct crypto_session_params *csp)
{
	struct rkcrypto_softc *sc = device_get_softc(dev);
	struct rkcrypto_session *session = crypto_get_driver_session(cses);

	mtx_lock(&sc->lock);
	if (sc->fault || sc->detaching) {
		mtx_unlock(&sc->lock);
		return (ENXIO);
	}
	if (csp->csp_mode == CSP_MODE_CIPHER && csp->csp_cipher_key != NULL)
		memcpy(session->key, csp->csp_cipher_key, csp->csp_cipher_klen);
	sc->sessions++;
	mtx_unlock(&sc->lock);
	return (0);
}

static void
rkcrypto_freesession(device_t dev, crypto_session_t cses)
{
	struct rkcrypto_softc *sc = device_get_softc(dev);

	explicit_bzero(crypto_get_driver_session(cses), sizeof(struct rkcrypto_session));
	mtx_lock(&sc->lock);
	sc->sessions--;
	mtx_unlock(&sc->lock);
}

static int
rkcrypto_process(device_t dev, struct cryptop *crp, int hint __unused)
{
	struct rkcrypto_softc *sc = device_get_softc(dev);

	mtx_lock(&sc->lock);
	if (sc->fault || sc->detaching || sc->queued >= 64) {
		crp->crp_etype = sc->fault ? EIO : ENOMEM;
		mtx_unlock(&sc->lock);
		crypto_done(crp);
		return (0);
	}
	sc->requests++;
	TAILQ_INSERT_TAIL(&sc->queue, crp, crp_next);
	sc->queued++;
	mtx_unlock(&sc->lock);
	taskqueue_enqueue(sc->tq, &sc->task);
	return (0);
}

static void
rkcrypto_map(void *arg, bus_dma_segment_t *segs, int nsegs, int error)
{
	struct rkcrypto_softc *sc = arg;

	sc->map_error = error;
	if (error != 0)
		return;
	sc->mapped = true;
	if (nsegs != 1 || (segs[0].ds_addr & 63) != 0 ||
	    segs[0].ds_len != sizeof(*sc->dma) ||
	    segs[0].ds_addr > UINT32_MAX - (sizeof(*sc->dma) - 1)) {
		sc->map_error = EINVAL;
		return;
	}
	sc->address = segs[0].ds_addr;
}

static int
rkcrypto_probe(device_t dev)
{
	if (!ofw_bus_status_okay(dev) ||
	    !ofw_bus_is_compatible(dev, "rockchip,crypto-v4") ||
	    !ofw_bus_node_is_compatible(OF_finddevice("/"), "friendlyelec,nanopi-r28s"))
		return (ENXIO);
	device_set_desc(dev, "RK3528 crypto-v4 AES-CBC/CTR and SHA256");
	return (BUS_PROBE_DEFAULT);
}

static int
rkcrypto_detach(device_t dev)
{
	struct rkcrypto_softc *sc = device_get_softc(dev);
	int rid;

	mtx_lock(&sc->lock);
	if (sc->sessions != 0 || sc->working || sc->queued != 0 || sc->pinned) {
		mtx_unlock(&sc->lock);
		return (EBUSY);
	}
	sc->detaching = true;
	mtx_unlock(&sc->lock);
	if (sc->cid >= 0)
		crypto_unregister_all(sc->cid);
	if (sc->tq != NULL) {
		taskqueue_drain(sc->tq, &sc->task);
		taskqueue_free(sc->tq);
	}
	if (sc->cookie != NULL)
		bus_teardown_intr(dev, sc->irq, sc->cookie);
	if (sc->dma != NULL) {
		explicit_bzero(sc->dma, sizeof(*sc->dma));
		if (sc->mapped)
			bus_dmamap_unload(sc->tag, sc->map);
		bus_dmamem_free(sc->tag, sc->dma, sc->map);
	}
	if (sc->tag != NULL)
		bus_dma_tag_destroy(sc->tag);
	if (sc->reset != NULL)
		hwreset_release(sc->reset);
	while (sc->enabled != 0)
		clk_disable(sc->clocks[--sc->enabled]);
	for (unsigned i = 0; i < 3; i++)
		if (sc->clocks[i] != NULL)
			clk_release(sc->clocks[i]);
	if (sc->irq != NULL) {
		rid = 0;
		bus_release_resource(dev, SYS_RES_IRQ, rid, sc->irq);
	}
	if (sc->mem != NULL) {
		rid = 0;
		bus_release_resource(dev, SYS_RES_MEMORY, rid, sc->mem);
	}
	cv_destroy(&sc->cv);
	mtx_destroy(&sc->lock);
	return (0);
}

static int
rkcrypto_selftest(struct rkcrypto_softc *sc)
{
	static const uint8_t plain[16] = {
		0x00,0x11,0x22,0x33,0x44,0x55,0x66,0x77,
		0x88,0x99,0xaa,0xbb,0xcc,0xdd,0xee,0xff };
	static const uint8_t cipher[3][16] = {
		{0x69,0xc4,0xe0,0xd8,0x6a,0x7b,0x04,0x30,0xd8,0xcd,0xb7,0x80,0x70,0xb4,0xc5,0x5a},
		{0xdd,0xa9,0x7c,0xa4,0x86,0x4c,0xdf,0xe0,0x6e,0xaf,0x70,0xa0,0xec,0x0d,0x71,0x91},
		{0x8e,0xa2,0xb7,0xca,0x51,0x67,0x45,0xbf,0xea,0xfc,0x49,0x90,0x4b,0x49,0x60,0x89} };
	static const uint8_t abc_hash[32] = {
		0xba,0x78,0x16,0xbf,0x8f,0x01,0xcf,0xea,0x41,0x41,0x40,0xde,0x5d,0xae,0x22,0x23,
		0xb0,0x03,0x61,0xa3,0x96,0x17,0x7a,0x9c,0xb4,0x10,0xff,0x61,0xf2,0x00,0x15,0xad };
	static const uint8_t long_hash[32] = {
		0x78,0xcb,0xba,0xc2,0xe0,0x6e,0xd5,0xb9,0x22,0xe3,0x9a,0x3a,0x24,0xbf,0xe9,0xee,
		0x9e,0x27,0x21,0xdd,0x2a,0x38,0x01,0x67,0xa9,0xa4,0x85,0x4d,0xec,0x73,0x60,0x4c };
	uint8_t key[32], zero[16] = {}, hash[32];
	int error = 0;

	sc->selftesting = true;
	for (unsigned i = 0; i < sizeof(key); i++) key[i] = i;
	for (unsigned n = 0; n < 3; n++) {
		memcpy(sc->dma->in, plain, 16);
		error = rkcrypto_cipher_chunk(sc, CRYPTO_AES_CBC, key, 16 + n * 8, zero, 16, true);
		if (error != 0 || timingsafe_bcmp(sc->dma->out, cipher[n], 16) != 0) goto done;
		memcpy(sc->dma->in, cipher[n], 16);
		error = rkcrypto_cipher_chunk(sc, CRYPTO_AES_CBC, key, 16 + n * 8, zero, 16, false);
		if (error != 0 || timingsafe_bcmp(sc->dma->out, plain, 16) != 0) goto done;
		memset(sc->dma->in, 0, 16);
		error = rkcrypto_cipher_chunk(sc, CRYPTO_AES_ICM, key, 16 + n * 8, plain, 16, true);
		if (error != 0 || timingsafe_bcmp(sc->dma->out, cipher[n], 16) != 0) goto done;
		sc->selftests += 3;
	}
	if ((error = rkcrypto_hash_init(sc)) != 0) goto done;
	memcpy(sc->dma->in, "abc", 3);
	if ((error = rkcrypto_dma_run(sc, 3, true)) != 0) goto done;
	if ((error = rkcrypto_hash_result(sc, hash)) != 0) goto done;
	if (timingsafe_bcmp(hash, abc_hash, 32) != 0) goto done;
	if ((error = rkcrypto_reset(sc)) != 0) goto done;
	sc->selftests++;
	for (unsigned k = 0; k < 131089; k++)
		sc->dma->in[k] = (k * 31 + k / 251 + 7) & 255;
	if ((error = rkcrypto_hash_init(sc)) != 0) goto done;
	if ((error = rkcrypto_dma_run(sc, 131089, true)) != 0) goto done;
	if ((error = rkcrypto_hash_result(sc, hash)) != 0) goto done;
	if (timingsafe_bcmp(hash, long_hash, 32) != 0) goto done;
	if ((error = rkcrypto_reset(sc)) != 0) goto done;
	sc->selftests++;
	sc->selftesting = false;
	explicit_bzero(sc->dma, sizeof(*sc->dma));
	for (unsigned i = 0; i < 8; i++) WR(sc, RKCR_KEY + i * 4, 0);
	explicit_bzero(key, sizeof(key));
	device_printf(sc->dev, "AES-128/192/256 CBC encrypt/decrypt, CTR and SHA256 (including 131089-byte input) self-tests passed\n");
	return (0);
done:
	sc->selftesting = false;
	rkcrypto_disable(sc);
	if (!sc->pinned) (void)rkcrypto_reset(sc);
	explicit_bzero(key, sizeof(key));
	device_printf(sc->dev, "known-answer self-test failed after %ju tests, error=%d\n",
	    (uintmax_t)sc->selftests, error);
	return (error != 0 ? error : EIO);
}

static int
rkcrypto_attach(device_t dev)
{
	static const char *names[] = { "aclk", "hclk", "sclk" };
	struct rkcrypto_softc *sc = device_get_softc(dev);
	int rid, error;

	sc->dev = dev;
	sc->cid = -1;
	mtx_init(&sc->lock, "rkcrypto queue", NULL, MTX_DEF);
	cv_init(&sc->cv, "rkcrypto DMA");
	TAILQ_INIT(&sc->queue);
	TASK_INIT(&sc->task, 0, rkcrypto_worker, sc);
	rid = 0;
	sc->mem = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid, RF_ACTIVE);
	if (sc->mem == NULL) { error = ENXIO; goto fail; }
	if (rman_get_start(sc->mem) != 0xffc40000 || rman_get_size(sc->mem) != 0x2000) {
		error = ENXIO; goto fail;
	}
	for (unsigned i = 0; i < 3; i++) {
		error = clk_get_by_ofw_name(dev, 0, names[i], &sc->clocks[i]);
		if (error != 0) goto fail;
		error = clk_enable(sc->clocks[i]);
		if (error != 0) goto fail;
		sc->enabled++;
	}
	error = hwreset_get_by_ofw_name(dev, 0, "crypto-rst", &sc->reset);
	if (error != 0) goto fail;
	error = hwreset_deassert(sc->reset);
	if (error != 0) goto fail;
	sc->version = RD(sc, RKCR_VERSION);
	sc->aes_version = RD(sc, RKCR_AES_VERSION);
	sc->hash_version = RD(sc, RKCR_HASH_VERSION);
	device_printf(dev, "version=%#x AES=%#x HASH=%#x\n", sc->version,
	    sc->aes_version, sc->hash_version);
	if ((sc->version & 0x0f000000) != 0x04000000) {
		error = ENXIO; goto fail;
	}
	if ((error = rkcrypto_reset(sc)) != 0) goto fail;
	error = bus_dma_tag_create(bus_get_dma_tag(dev), 64, 0,
	    BUS_SPACE_MAXADDR_32BIT, BUS_SPACE_MAXADDR, NULL, NULL,
	    sizeof(*sc->dma), 1, sizeof(*sc->dma), 0, NULL, NULL, &sc->tag);
	if (error != 0) goto fail;
	error = bus_dmamem_alloc(sc->tag, (void **)&sc->dma,
	    BUS_DMA_COHERENT | BUS_DMA_ZERO | BUS_DMA_WAITOK, &sc->map);
	if (error != 0) goto fail;
	error = bus_dmamap_load(sc->tag, sc->map, sc->dma, sizeof(*sc->dma),
	    rkcrypto_map, sc, BUS_DMA_NOWAIT);
	if (error == 0) error = sc->map_error;
	if (error != 0) goto fail;
	rid = 0;
	sc->irq = bus_alloc_resource_any(dev, SYS_RES_IRQ, &rid, RF_ACTIVE);
	if (sc->irq == NULL) { error = ENXIO; goto fail; }
	error = bus_setup_intr(dev, sc->irq, INTR_TYPE_MISC | INTR_MPSAFE,
	    NULL, rkcrypto_interrupt, sc, &sc->cookie);
	if (error != 0) goto fail;
	sc->tq = taskqueue_create("rkcrypto", M_WAITOK, taskqueue_thread_enqueue, &sc->tq);
	error = taskqueue_start_threads(&sc->tq, 1, PI_DISK, "rkcrypto");
	if (error != 0) goto fail;
	if ((error = rkcrypto_selftest(sc)) != 0) goto fail;
	sc->cid = crypto_get_driverid(dev, sizeof(struct rkcrypto_session), CRYPTOCAP_F_HARDWARE);
	if (sc->cid < 0) { error = ENOMEM; goto fail; }
#define COUNTER(n) SYSCTL_ADD_U64(device_get_sysctl_ctx(dev), \
	SYSCTL_CHILDREN(device_get_sysctl_tree(dev)), OID_AUTO, #n, CTLFLAG_RD, &sc->n, 0, #n)
	COUNTER(requests); COUNTER(completed); COUNTER(errors); COUNTER(interrupts);
	COUNTER(dma_ops); COUNTER(cipher_bytes); COUNTER(hash_bytes);
	COUNTER(timeouts); COUNTER(reset_failures); COUNTER(selftests);
	COUNTER(hash_software_requests); COUNTER(hash_software_bytes);
#undef COUNTER
	SYSCTL_ADD_BOOL(device_get_sysctl_ctx(dev), SYSCTL_CHILDREN(device_get_sysctl_tree(dev)),
	    OID_AUTO, "fault", CTLFLAG_RD, &sc->fault, 0, "Latched DMA fault");
	return (0);
fail:
	device_printf(dev, "attach failed: %d\n", error);
	if (!sc->pinned) (void)rkcrypto_detach(dev);
	return (error);
}

static device_method_t rkcrypto_methods[] = {
	DEVMETHOD(device_probe, rkcrypto_probe),
	DEVMETHOD(device_attach, rkcrypto_attach),
	DEVMETHOD(device_detach, rkcrypto_detach),
	DEVMETHOD(cryptodev_probesession, rkcrypto_probesession),
	DEVMETHOD(cryptodev_newsession, rkcrypto_newsession),
	DEVMETHOD(cryptodev_freesession, rkcrypto_freesession),
	DEVMETHOD(cryptodev_process, rkcrypto_process),
	DEVMETHOD_END
};
static driver_t rkcrypto_driver = {
	"rkcrypto", rkcrypto_methods, sizeof(struct rkcrypto_softc)
};
DRIVER_MODULE(rkcrypto, simplebus, rkcrypto_driver, 0, 0);
MODULE_DEPEND(rkcrypto, crypto, 1, 1, 1);
MODULE_VERSION(rkcrypto, 1);
