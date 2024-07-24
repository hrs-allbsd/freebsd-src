/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2019-2024 Hiroki Sato <hrs@FreeBSD.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 */
#include <sys/param.h>
#include <dev/pci/pcireg.h>
#include <dev/usb/controller/xhcireg.h>

#include <bootstrap.h>
#include <efi.h>
#include <efilib.h>

#include "xhci_dbc_cons.h"
#include "xhci_dbc_pci.h"
#include "xhci_dbc_dma.h"	/* udb_init_dma() */
#include <dev/usb/controller/xhci_pci.h>
#include <dev/usb/controller/xhci_private.h>
#include <dev/usb/controller/xhci_dbc.h>
#include <dev/usb/controller/xhci_dbc_private.h>

static void udbc_probe(struct console *);
static int udbc_getc(void);
static void udbc_putc(int);
static int udbc_ischar(void);
static int udbc_init(int);
struct console udb_console = {
	.c_name = "udbc",
	.c_desc = "USB DbC serial port",
	.c_flags = 0,
	.c_probe = udbc_probe,
	.c_init = udbc_init,
	.c_out = udbc_putc,
	.c_in = udbc_getc,
	.c_ready = udbc_ischar
};

struct xhci_debug_softc *udb_sc;
const char *udb_hostname;
const char *udb_serial;
const char *udb_gdb;

static void
udbc_probe(struct console *cons)
{
	struct xhci_debug_softc *sc;
	EFI_STATUS status;
	EFI_HANDLE *h0, *h;
	EFI_PCI_IO_PROTOCOL *pciio;
	UINTN hlen;
	char buf[32], *p;
	int i, error;
	device_t dev;

	/* XXX: There kenv should be in c_init() function. */
	p = getenv("hw.usb.xhci.dbc.enable");
	if (p != NULL && p[0] == '0')
		return;
	udb_hostname = getenv("hw.usb.xhci.dbc.hostname");
	udb_serial = getenv("hw.usb.xhci.dbc.serial");
	udb_gdb = getenv("hw.usb.xhci.dbc.gdb");

	h0 = h = NULL;
	sc = NULL;
	if (udb_sc != NULL)
		return;

	/* Get PCI I/O handle. */
	hlen = 0;
	status = BS->LocateHandleBuffer(
	    ByProtocol,
	    &pciio_guid,
	    NULL,
	    &hlen,
	    &h0);
	if (EFI_ERROR(status))
		goto error;

	h = NULL;
	for (UINTN i = 0; i < hlen; i++) {
		status = BS->HandleProtocol(
		    h0[i],
		    &pciio_guid,
		    (VOID **)&pciio);
		if (EFI_ERROR(status))
			continue;
		status = pciio->Pci.Read(
		    pciio,
		    EfiPciIoWidthUint8,
		    0,
		    sizeof(dev),
		    &dev);
		if (EFI_ERROR(status))
			continue;
		if (pci_get_headertype(dev) != PCIM_HDRTYPE_NORMAL &&
		    pci_get_headertype(dev) != PCIM_MFDEV)
			continue;
		if (xhci_pci_match(dev) != NULL) {
			h = h0[i];
			break;
		}
	}
	if (h == NULL) {	/* Not found. */
		DEBUG_PRINTF(1, "%s: Compatible xHC not found.\n", __func__);
		goto error;
	}

	/* h and pciio will be stored in the softc. */
	sc = udb_sc_malloc(sizeof(*sc), h, pciio);
	if (sc == NULL)
		goto error;
	udb_sc = sc;
	sc->sc_cons = cons;

	/* sc_efi_pciio must be valid before this. */
	if (!xhci_debug_probe(sc))
		goto error;

	xhci_debug_disable(sc);
	cons->c_flags = C_PRESENTIN | C_PRESENTOUT;

	xhci_debug_update_state(sc);
	if (sc->sc_init == false) {
		error = xhci_debug_init_dma(sc);
		if (error) {
			DEBUG_PRINTF(1, "USB DbC DMA configuration error\n");
			goto error;
		}
		error = xhci_debug_init_ring(sc);
		if (error) {
			DEBUG_PRINTF(1, "USB DbC TRB ring configuration error\n");
			goto error;
		}
		sc->sc_init = true;
		sc->sc_cookie = XHCI_DC_COOKIE;
	}
	xhci_debug_enable(sc);

	return;
error:
	(void) BS->FreePool(h0);
#if 0
	/* XXX: sc cannot be free using free().  Use udb_free_dma() instead. */
	free(sc);
#endif
	sc = NULL;
	return;
}

static int
udbc_init(int arg)
{
	struct xhci_debug_softc *sc = udb_sc;
	int error;

	if (sc == NULL) {		/* allocated in c_probe */
		DEBUG_PRINTF(1, "USB DbC not found\n");
		return (CMD_ERROR);
	}
	if (sc->sc_dbc_off == 0) {	/* set in c_probe */
		DEBUG_PRINTF(1, "USB DbC register not found\n");
		return (CMD_ERROR);
	}
	xhci_debug_enable(sc);
	xhci_debug_event_dequeue(sc);

	return (CMD_OK);
}

static void
udbc_putc(int c0)
{
	struct xhci_debug_softc *sc = udb_sc;
	u_char c = (0xff & c0);

	if (sc == NULL)
		return;
	if (work_enqueue(&sc->udb_oring, &c, sizeof(c)) != sizeof(c))
		return;

	xhci_debug_bulk_transfer(sc);
	/*
	 * This dequeue just after the transfer is important.
	 * Do not remove this.
	 */
	xhci_debug_event_dequeue(sc);
}

/* udbc_getc() is called periodically. */
static int
udbc_getc(void)
{
	struct xhci_debug_softc *sc = udb_sc;

	if (sc == NULL)
		return (-1);

	/* Use udbc_ischar() to receive data */
	return (udbc_ischar() ? work_dequeue(&sc->udb_iring) : -1);
}

static int
udbc_ischar(void)
{
	struct xhci_debug_softc *sc = udb_sc;
	struct xhci_debug_ring *iring;

	if (sc == NULL)
		return (0);
	iring = &sc->udb_iring;
	if (!DC_WORK_RING_EMPTY(iring))
		return (1);

	xhci_debug_bulk_transfer(sc);
	xhci_debug_event_dequeue(sc);

	return (!DC_WORK_RING_EMPTY(iring));
}
