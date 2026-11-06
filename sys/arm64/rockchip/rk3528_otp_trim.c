/*-
 * SPDX-License-Identifier: BSD-2-Clause
 * Copyright (c) 2026 Hiroki Sato <hrs@FreeBSD.org>
 */
/* Read/cache TSADC trim and opt-in public chip ID; no OTP programming. */
#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/rman.h>
#include <sys/sysctl.h>

#include <machine/atomic.h>
#include <machine/bus.h>

#include <dev/clk/clk.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <crypto/sha2/sha256.h>

#include "nvmem_if.h"

#define SBPI_CTRL      0x20
#define CMD_VALID      0x24
#define LOCK_CTRL      0x50
#define USER_CTRL      0x100
#define USER_ADDR      0x104
#define USER_ENABLE    0x108
#define USER_QP	       0x120
#define USER_Q	       0x124
#define INT_STATUS     0x304
#define CMD0	       0x1000
#define CMD1	       0x1004
#define TRIM_OFFSET    0x44
#define CHIP_ID_OFFSET 0x0a
#define CHIP_ID_BYTES  16

struct rk3528_otp_trim_softc {
	uint8_t trim[2], chip_id[CHIP_ID_BYTES];
	char serial[38], gmac[18], usb_mac[13], pcie_mac[18];
	u_int identity_reads, identity_valid;
	u_int word, hardware_reads, cache_reads;
};
/* Ten milliseconds per operation, matching the Board Support Package read timeout. */
static int
rk3528_otp_trim_wait(struct resource *res, uint32_t bit)
{
	for (int i = 0; i < 10000; i++) {
		if ((bus_read_4(res, INT_STATUS) & bit) != 0) {
			bus_write_4(res, INT_STATUS, bit);
			return (0);
		}
		DELAY(1);
	}
	return (ETIMEDOUT);
}
static int
rk3528_otp_trim_read(device_t dev, uint32_t offset, uint32_t size, uint8_t *buf)
{
	struct rk3528_otp_trim_softc *sc = device_get_softc(dev);

	if (size == 0 || offset < TRIM_OFFSET || offset >= TRIM_OFFSET + 2 ||
	    size > TRIM_OFFSET + 2 - offset)
		return (ERANGE);
	memcpy(buf, sc->trim + (offset - TRIM_OFFSET), size);
	atomic_add_int(&sc->cache_reads, 1);
	return (0);
}
/* Only the public ID range declared in the vendor DT and the two trim
 * bytes can be requested. No caller supplies an arbitrary OTP address.
 */
static int
rk3528_otp_read_word(struct resource *res, uint32_t offset, uint16_t *word)
{
	uint32_t status;
	int error;

	if ((offset & 1) != 0 ||
	    (offset != TRIM_OFFSET &&
		(offset < CHIP_ID_OFFSET ||
		    offset >= CHIP_ID_OFFSET + CHIP_ID_BYTES)))
		return (ERANGE);
	bus_write_4(res, INT_STATUS, 4);
	bus_write_4(res, USER_ADDR, 0xffff0000 | (offset / 2));
	bus_write_4(res, USER_ENABLE, 0x10001);
	error = rk3528_otp_trim_wait(res, 4);
	if (error != 0)
		return (error);
	status = bus_read_4(res, USER_QP);
	if ((status & 0xc0) == 0xc0 || (status & 0x20) != 0)
		return (EIO);
	*word = bus_read_4(res, USER_Q) & 0xffff;
	return (0);
}

/* Stable, locally administered unicast identities, domain-separated by
 * purpose. This is identity derivation, not key generation or entropy.
 */
static int
rk3528_otp_identity(const uint8_t id[CHIP_ID_BYTES], char serial[38],
    uint8_t gmac[6], uint8_t usb[6], uint8_t pcie[6])
{
	static const char hex[] = "0123456789abcdef";
	static const char *domains[] = { "nanopi-r28s/gmac1/v1",
		"nanopi-r28s/usb-ecm/v1", "nanopi-r28s/pcie-re0/v1" };
	SHA256_CTX ctx;
	uint8_t digest[SHA256_DIGEST_LENGTH];
	unsigned zero = 0, ones = 0;

	for (unsigned i = 0; i < CHIP_ID_BYTES; i++) {
		zero += id[i] == 0;
		ones += id[i] == 0xff;
	}
	if (zero == CHIP_ID_BYTES || ones == CHIP_ID_BYTES)
		return (EINVAL);
	memcpy(serial, "R28S-", 5);
	for (unsigned i = 0; i < CHIP_ID_BYTES; i++) {
		serial[5 + i * 2] = hex[id[i] >> 4];
		serial[6 + i * 2] = hex[id[i] & 15];
	}
	serial[37] = 0;
	for (unsigned i = 0; i < nitems(domains); i++) {
		SHA256_Init(&ctx);
		SHA256_Update(&ctx, domains[i], strlen(domains[i]) + 1);
		SHA256_Update(&ctx, id, CHIP_ID_BYTES);
		SHA256_Final(digest, &ctx);
		digest[0] = (digest[0] & 0xfc) | 2;
		memcpy(i == 0 ? gmac : i == 1 ? usb : pcie, digest, 6);
	}
	explicit_bzero(digest, sizeof(digest));
	return (0);
}

/* Derive diagnostic identities without configuring any consumer. */
static int
rk3528_otp_prepare_identity(device_t dev)
{
	struct rk3528_otp_trim_softc *sc = device_get_softc(dev);
	uint8_t gmac[6], usb[6], pcie[6];
	int error;

	error = rk3528_otp_identity(sc->chip_id, sc->serial, gmac, usb, pcie);
	if (error != 0)
		return (error);
	snprintf(sc->gmac, sizeof(sc->gmac), "%02x:%02x:%02x:%02x:%02x:%02x",
	    gmac[0], gmac[1], gmac[2], gmac[3], gmac[4], gmac[5]);
	snprintf(sc->usb_mac, sizeof(sc->usb_mac), "%02X%02X%02X%02X%02X%02X",
	    usb[0], usb[1], usb[2], usb[3], usb[4], usb[5]);
	snprintf(sc->pcie_mac, sizeof(sc->pcie_mac),
	    "%02x:%02x:%02x:%02x:%02x:%02x", pcie[0], pcie[1], pcie[2],
	    pcie[3], pcie[4], pcie[5]);
	sc->identity_valid = 1;
	return (0);
}

static int
rk3528_otp_trim_probe(device_t dev)
{
	if (!ofw_bus_status_okay(dev) ||
	    !ofw_bus_is_compatible(dev, "freebsd,rk3528-r28s-otp-trim"))
		return (ENXIO);
	device_set_desc(dev, "RK3528 read-only TSADC OTP trim cache");
	return (BUS_PROBE_DEFAULT);
}
static int
rk3528_otp_trim_attach(device_t dev)
{
	static const char *names[] = { "apb", "sbpi", "usr" };
	struct rk3528_otp_trim_softc *sc = device_get_softc(dev);
	struct resource *res;
	clk_t clocks[3] = { };
	bool enabled[3] = { };
	uint16_t word;
	bool identity;
	uint64_t frequency;
	int error = ENXIO, rid = 0;

	identity = OF_hasprop(ofw_bus_get_node(dev), "freebsd,board-identity");
	/* No arbitrary address, size or programming controls. */
	if (!ofw_bus_node_is_compatible(OF_finddevice("/"),
		"friendlyelec,nanopi-r28s"))
		return (ENXIO);
	res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid, RF_ACTIVE);
	if (res == NULL)
		return (ENXIO);
	if (rman_get_start(res) != 0xffce0000 || rman_get_size(res) != 0x4000)
		goto out;
	for (int i = 0; i < 3; i++) {
		error = clk_get_by_ofw_name(dev, 0, names[i], &clocks[i]);
		if (error != 0)
			goto out;
		if (i != 0) {
			error = clk_get_freq(clocks[i], &frequency);
			if (error != 0 ||
			    frequency != (i == 1 ? 24000000 : 12000000)) {
				error = EINVAL;
				goto out;
			}
		}
		error = clk_enable(clocks[i]);
		if (error != 0)
			goto out;
		enabled[i] = true;
	}
	device_printf(dev, "Read clocks enabled; requesting trim word 0x22\n");
	/* Clear stale completions, then configure the controller's ECC read
	 * path. 0xfa is DAP WRF/ECC-register configuration, not a fuse-program
	 * command. The sequence follows the manufacturer's rk3568_otp_read used
	 * by RK3528.
	 */
	bus_write_4(res, INT_STATUS, 6);
	bus_write_4(res, SBPI_CTRL, 0xff000000 | (2 << 8));
	bus_write_4(res, CMD_VALID, 0xffff0001);
	bus_write_4(res, CMD0, 0xc0 | 0x3a);
	bus_write_4(res, CMD1, 0); /* ECC enabled */
	bus_write_4(res, SBPI_CTRL, 0x10001);
	error = rk3528_otp_trim_wait(res, 2);
	if (error != 0)
		goto out;
	bus_write_4(res, USER_CTRL, 0x10001);
	DELAY(5);
	bus_write_4(res, LOCK_CTRL, 0x10001);
	error = rk3528_otp_read_word(res, TRIM_OFFSET, &word);
	if (error == 0) {
		sc->trim[0] = word & 0xff;
		sc->trim[1] = word >> 8;
		sc->word = word;
		sc->hardware_reads = 1;
	}
	if (error == 0 && identity) {
		for (unsigned i = 0; i < CHIP_ID_BYTES; i += 2) {
			error = rk3528_otp_read_word(res, CHIP_ID_OFFSET + i,
			    &word);
			if (error != 0)
				break;
			sc->chip_id[i] = word & 0xff;
			sc->chip_id[i + 1] = word >> 8;
			sc->identity_reads++;
		}
	}

	/* Restore the read-mode/lock controls even after a read timeout/ECC
	 * error. */
	bus_write_4(res, LOCK_CTRL, 0x10000);
	bus_write_4(res, USER_CTRL, 0x10000);
out:
	if (sc->hardware_reads == 1)
		device_printf(dev,
		    "Trim word read; read controls restored, releasing leaf clocks\n");
	for (int i = 2; i >= 0; i--) {
		if (enabled[i])
			clk_disable(clocks[i]);
		if (clocks[i] != NULL)
			clk_release(clocks[i]);
	}
	bus_release_resource(dev, SYS_RES_MEMORY, rid, res);
	if (error != 0) {
		device_printf(dev, "trim read failed: %d\n", error);
		return (error);
	}
	if (identity) {
		error = rk3528_otp_prepare_identity(dev);
		if (error != 0) {
			device_printf(dev,
			    "board identity preparation failed: %d\n", error);
			return (error);
		}
	}
	SYSCTL_ADD_UINT(device_get_sysctl_ctx(dev),
	    SYSCTL_CHILDREN(device_get_sysctl_tree(dev)), OID_AUTO,
	    "identity_valid", CTLFLAG_RD, &sc->identity_valid, 0,
	    "Chip-ID identities derived successfully");
	SYSCTL_ADD_UINT(device_get_sysctl_ctx(dev),
	    SYSCTL_CHILDREN(device_get_sysctl_tree(dev)), OID_AUTO,
	    "identity_reads", CTLFLAG_RD, &sc->identity_reads, 0,
	    "Read-only public chip-ID words read at attach");
	SYSCTL_ADD_STRING(device_get_sysctl_ctx(dev),
	    SYSCTL_CHILDREN(device_get_sysctl_tree(dev)), OID_AUTO, "serial",
	    CTLFLAG_RD, sc->serial, sizeof(sc->serial),
	    "Stable public per-chip USB serial");
	SYSCTL_ADD_STRING(device_get_sysctl_ctx(dev),
	    SYSCTL_CHILDREN(device_get_sysctl_tree(dev)), OID_AUTO, "gmac",
	    CTLFLAG_RD, sc->gmac, sizeof(sc->gmac),
	    "Derived GMAC1 address; not installed");
	SYSCTL_ADD_STRING(device_get_sysctl_ctx(dev),
	    SYSCTL_CHILDREN(device_get_sysctl_tree(dev)), OID_AUTO, "usb_mac",
	    CTLFLAG_RD, sc->usb_mac, sizeof(sc->usb_mac),
	    "Derived ECM host MAC; not installed");
	SYSCTL_ADD_STRING(device_get_sysctl_ctx(dev),
	    SYSCTL_CHILDREN(device_get_sysctl_tree(dev)), OID_AUTO, "pcie_mac",
	    CTLFLAG_RD, sc->pcie_mac, sizeof(sc->pcie_mac),
	    "Derived onboard re0 MAC; not installed");

	OF_device_register_xref(OF_xref_from_node(ofw_bus_get_node(dev)), dev);
	SYSCTL_ADD_UINT(device_get_sysctl_ctx(dev),
	    SYSCTL_CHILDREN(device_get_sysctl_tree(dev)), OID_AUTO, "trim_word",
	    CTLFLAG_RD, &sc->word, 0,
	    "Cached TSADC OTP trim word at byte offset 0x44");
	SYSCTL_ADD_UINT(device_get_sysctl_ctx(dev),
	    SYSCTL_CHILDREN(device_get_sysctl_tree(dev)), OID_AUTO,
	    "hardware_reads", CTLFLAG_RD, &sc->hardware_reads, 0,
	    "16-bit OTP hardware reads at attach");
	SYSCTL_ADD_UINT(device_get_sysctl_ctx(dev),
	    SYSCTL_CHILDREN(device_get_sysctl_tree(dev)), OID_AUTO,
	    "cache_reads", CTLFLAG_RD, &sc->cache_reads, 0,
	    "NVMEM requests served from the trim cache");
	device_printf(dev,
	    "trim@44=%02x %02x; one 16-bit read, controller released\n",
	    sc->trim[0], sc->trim[1]);
	return (0);
}
static int
rk3528_otp_trim_detach(device_t dev)
{
	/* Keep the registered NVMEM provider and its cache alive. */
	return (EBUSY);
}
static device_method_t rk3528_otp_trim_methods[] = {
	DEVMETHOD(device_probe, rk3528_otp_trim_probe),
	DEVMETHOD(device_attach, rk3528_otp_trim_attach),
	DEVMETHOD(device_detach, rk3528_otp_trim_detach),
	DEVMETHOD(nvmem_read, rk3528_otp_trim_read),
	/* nvmem_write defaults to ENXIO. */
	DEVMETHOD_END
};
static driver_t rk3528_otp_trim_driver = {
	"rk3528_otp_trim", rk3528_otp_trim_methods,
	sizeof(struct rk3528_otp_trim_softc)
};
EARLY_DRIVER_MODULE(rk3528_otp_trim, simplebus, rk3528_otp_trim_driver, 0, 0,
    BUS_PASS_SUPPORTDEV + BUS_PASS_ORDER_FIRST);
MODULE_VERSION(rk3528_otp_trim, 1);
