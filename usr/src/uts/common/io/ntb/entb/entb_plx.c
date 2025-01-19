/*
 * This file and its contents are supplied under the terms of the
 * Common Development and Distribution License ("CDDL"), version 1.0.
 * You may only use this file in accordance with the terms of version
 * 1.0 of the CDDL.
 *
 * A full copy of the text of the CDDL should have accompanied this
 * source.  A copy of the CDDL is also available via the Internet at
 * http://www.illumos.org/license/CDDL.
 */

/*
 * Copyright 2025 Tintri by DDN, Inc.  All rights reserved.
 */

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/cmn_err.h>
#include <sys/strsun.h>
#include <sys/cmn_err.h>
#include <sys/conf.h>
#include <sys/ddi.h>
#include <sys/ddidmareq.h>

#include "../ntb.h"
#include "entb_impl.h"
#include "entb_plx.h"

#define	ENTB_PLX_FRAME_MAX_LENGTH 5000000
#define	ENTB_PLX_FRAME_IDENTIFIER 0xDEADFEED

typedef struct entb_plx_frame_header_s {
	uint32_t	epfh_header_id;
	int32_t		epfh_transmit_frame_len;
	int32_t		epfh_txrx_packet_len;
	int32_t		epfh_padding;
} entb_plx_frame_header_t;

typedef struct entb_plx_ringbuf_info_s {
	uint64_t	epri_ringbuf_write_marker;
} entb_plx_ringbuf_info_t;

static void entb_plx_dma_free(entb_dma_info_t *info);
static void entb_plx_dma_write_free(entb_plx_t *entb_plx_info);
static void entb_plx_dma_read_free(entb_plx_t *entb_plx_info);
static int entb_plx_alloc_dma_read(entb_plx_t *entb_plx_info, dev_info_t *dip);
static int entb_plx_alloc_dma_write(entb_plx_t *entb_plx_info, dev_info_t *dip);
static void entb_plx_doorbell_start_read(void *arg, int doorbell);
static void entb_plx_doorbell_peer_read_done(void *arg, int doorbell);
static void entb_plx_peer_state_change(void *arg, int doorbell);
static int entb_plx_init_doorbells(entb_plx_t *entb_plx_info);
static int entb_plx_destroy_doorbells(entb_plx_t *entb_plx_info);
static void entb_plx_init_state(entb_plx_t *entb_plx_info);
static void entb_plx_destroy_state(entb_plx_t *entb_plx_info);
static void entb_plx_transmit_ringbuf_info(entb_plx_t *entb_plx_info);
static entb_plx_frame_header_t *entb_plx_read_and_verify_hdr(entb_t *entb_ss,
    int32_t offset);
static int entb_plx_read_data(entb_t *entb_ss, int32_t offset, int32_t len,
    boolean_t do_ack);
static void entb_plx_ack_peer(entb_t *entb_ss, int32_t count);

int
entb_plx_init(entb_t *entb_ss, dev_info_t *dip)
{
	entb_plx_t	*entb_plx_info = NULL;
	ntb_svc_t	*hdl = NULL;
	uint_t		up, down;
	int32_t		doorbell, seg, nsegments;
	boolean_t	state_init = B_FALSE;

	if (ntb_get_dma_segments_by_dev(dip, &seg, &nsegments) != DDI_SUCCESS)
		/* No segments, so don't init plx structs. But this is ok */
		return (DDI_SUCCESS);

	entb_ss->entb_plx_info = kmem_zalloc(sizeof (entb_plx_t), KM_SLEEP);

	if (ntb_allocate_svc(dip, &hdl) != DDI_SUCCESS) {
		cmn_err(CE_WARN, "ntb_allocate_svc failed");
		goto failed;
	}

	if (ntb_get_status_doorbells(hdl, &up, &down) != DDI_SUCCESS) {
		cmn_err(CE_WARN, "ntb_get_status_doorbells failed");
		goto failed;
	}

	entb_plx_info = entb_ss->entb_plx_info;
	entb_plx_info->dip = dip;
	entb_plx_info->ntb_hdl = hdl;
	entb_plx_info->up = up;
	entb_plx_info->down = down;
	entb_plx_info->entb_plx_ss = entb_ss;

	entb_plx_info->entb_ringbuf_nelems = ENTB_MAX_BUFS_PER_SEGMENT *
	    nsegments - 1;

	entb_plx_init_state(entb_plx_info);
	state_init = B_TRUE;

	entb_tx_init(entb_ss);
	entb_rx_init(entb_ss);

	if ((doorbell = ddi_prop_get_int(DDI_DEV_T_ANY, dip, DDI_PROP_DONTPASS,
	    "doorbell-no", -1)) == -1) {
		goto failed;
	}

	entb_plx_info->ep_rx_doorbell_id = doorbell;

	if (entb_plx_alloc_dma_read(entb_plx_info, dip) == DDI_FAILURE)
		goto failed;

	if (entb_plx_alloc_dma_write(entb_plx_info, dip) == DDI_FAILURE)
		goto failed;

	if (entb_plx_init_doorbells(entb_plx_info) == DDI_SUCCESS)
		return (DDI_SUCCESS);

failed:
	if (hdl)
		(void) ntb_free_svc(hdl);

	entb_tx_destroy(entb_ss);
	entb_rx_destroy(entb_ss);

	entb_plx_dma_read_free(entb_plx_info);
	entb_plx_dma_write_free(entb_plx_info);

	if (state_init)
		entb_plx_destroy_state(entb_plx_info);

	kmem_free(entb_plx_info, sizeof (entb_plx_t));
	entb_ss->entb_plx_info = NULL;

	return (DDI_FAILURE);
}

/* ARGSUSED */
int
entb_plx_destroy(entb_t *entb_ss, dev_info_t *dip)
{
	entb_plx_t	*entb_plx_info;

	entb_tx_destroy(entb_ss);
	entb_rx_destroy(entb_ss);

	entb_plx_info = entb_ss->entb_plx_info;
	entb_plx_destroy_state(entb_plx_info);

	(void) ntb_free_svc(entb_plx_info->ntb_hdl);

	entb_plx_dma_read_free(entb_plx_info);
	entb_plx_dma_write_free(entb_plx_info);
	(void) entb_plx_destroy_doorbells(entb_plx_info);

	kmem_free(entb_ss->entb_plx_info, sizeof (entb_plx_t));
	entb_ss->entb_plx_info = NULL;

	return (DDI_SUCCESS);
}

/*
 * Not mp safe.
 */
int
entb_plx_send(entb_plx_t *entb_plx_info, mblk_t *mp_elem)
{
	entb_dma_info_t	*write_info;
	caddr_t		bufp;
	mblk_t		*nmp;
	entb_plx_frame_header_t	entb_hdr;
	int32_t		pktsize;
	int		ret;
	int32_t		hdr_size;
	int32_t		driver_doorbell_no;
	int32_t		frame_len = 0;
	int32_t		offset;

	hdr_size = sizeof (entb_plx_frame_header_t);

	write_info = &entb_plx_info->w_info;

	/*
	 * We use 2 variables, 'entb_ringbuf_write_marker' and
	 * 'entb_ringbuf_peer_read_marker' which act as markers for the ring
	 * buffer's start and end respectively.
	 */
	offset = ENTB_DATA_DMA_FIXED_OFFSET;

	/*
	 * There can be a race b/w the tx thread setting
	 * entb_ringbuf_write_marker and the entb_plx_peer_state_change()
	 * re-setting the value. Add an ASSERT() to catch this.
	 */
	ASSERT(entb_plx_info->entb_ringbuf_write_marker > 0);
	offset += ((entb_plx_info->entb_ringbuf_write_marker - 1) %
	    entb_plx_info->entb_ringbuf_nelems) * ENTB_BUF_SIZE;
	bufp = write_info->edi_addr + offset;

	/*
	 * Determine the total MTU size
	 */
	pktsize = 0;
	for (nmp = mp_elem; nmp != NULL; nmp = nmp->b_cont) {
		pktsize += MBLKL(nmp);
	}
	frame_len += pktsize + hdr_size;

	if (frame_len > ENTB_PLX_FRAME_MAX_LENGTH) {
		/*
		 * We should NEVER reach here. Drop the packet as of now.
		 * TODO: Add an ASSERT() here.
		 */
		cmn_err(CE_WARN, "!entb: entb_plx_send: failed! Packet size %d "
		    "bytes greater than Max permitted size %d bytes",
		    pktsize, ENTB_PLX_FRAME_MAX_LENGTH);
		return (DDI_FAILURE);
	}

	/*
	 * Inititalize our header.
	 */
	entb_hdr.epfh_header_id = ENTB_PLX_FRAME_IDENTIFIER;
	entb_hdr.epfh_txrx_packet_len = pktsize;
	entb_hdr.epfh_transmit_frame_len = frame_len;

	bcopy(&entb_hdr, bufp, hdr_size);
	bufp += hdr_size;

	for (nmp = mp_elem; nmp != NULL; nmp = nmp->b_cont) {
		pktsize = MBLKL(nmp);
		bcopy(nmp->b_rptr, bufp, pktsize);
		bufp += pktsize;
	}
	(void) ntb_put(entb_plx_info->ntb_hdl, write_info->edi_hdl, offset,
	    frame_len);
	entb_plx_transmit_ringbuf_info(entb_plx_info);

	/*
	 * Now send a doorbell to the other side
	 */
	driver_doorbell_no = entb_plx_info->ep_rx_doorbell_id;
	ret = ntb_send_doorbell(entb_plx_info->ntb_hdl, driver_doorbell_no);
	if (ret != DDI_SUCCESS) {
		cmn_err(CE_WARN, "!entb: Error!: Setting doorbell (%d) failed."
		    "Peer not intimated on write complete.",
		    driver_doorbell_no);
	}

	return (DDI_SUCCESS);
}

boolean_t
entb_plx_peer_ok(entb_plx_t *entb_plx_info)
{
	struct entb_s	*entb_ss;

	if (entb_plx_info == NULL || (entb_ss = entb_plx_info->entb_plx_ss) ==
	    NULL) {
		return (B_FALSE);
	}

	return (entb_ss->entb_peer_state == ENTB_PEER_STATE_UP);
}

static int
entb_plx_alloc_dma_write(entb_plx_t *entb_plx_info, dev_info_t *dip)
{
	entb_dma_info_t	*write_info;
	ddi_dma_attr_t	w_attr;
	ddi_device_acc_attr_t acc_attr;
	size_t		len;

	if (!entb_plx_info) {
		return (DDI_FAILURE);
	}

	if (ntb_get_dma_attr(entb_plx_info->ntb_hdl, NTB_PUT, &w_attr,
	    &acc_attr, NULL) != DDI_SUCCESS) {
		return (DDI_FAILURE);
	}

	write_info = &(entb_plx_info->w_info);

	if (ddi_dma_alloc_handle(dip, &w_attr, DDI_DMA_SLEEP, NULL,
	    &write_info->edi_hdl) != DDI_SUCCESS) {
		cmn_err(CE_WARN, "!entb: entb_plx_alloc_dma_write: failed to "
		    "alloc write handle");
		return (DDI_FAILURE);
	}

	if (ddi_dma_mem_alloc(write_info->edi_hdl, ENTB_PLX_FRAME_MAX_LENGTH,
	    &acc_attr, DDI_DMA_CONSISTENT, DDI_DMA_SLEEP, NULL,
	    &write_info->edi_addr, &len, &write_info->edi_acc_hdl) !=
	    DDI_SUCCESS) {
		cmn_err(CE_WARN, "!entb: entb_plx_alloc_dma_write: failed to "
		    "alloc write mem");
		entb_plx_dma_write_free(entb_plx_info);
		return (DDI_FAILURE);
	}

	if (ddi_dma_addr_bind_handle(write_info->edi_hdl, NULL,
	    write_info->edi_addr, len, DDI_DMA_WRITE | DDI_DMA_CONSISTENT,
	    DDI_DMA_SLEEP, NULL, &write_info->edi_cookie,
	    &write_info->edi_cookie_cnt) != DDI_DMA_MAPPED) {
		cmn_err(CE_WARN, "!entb: entb_plx_alloc_dma_write: failed to "
		    "bind write mem");
		entb_plx_dma_write_free(entb_plx_info);
		return (DDI_FAILURE);
	}

	return (DDI_SUCCESS);
}

static int
entb_plx_alloc_dma_read(entb_plx_t *entb_plx_info, dev_info_t *dip)
{
	ddi_dma_attr_t  attr;
	ddi_device_acc_attr_t acc_attr;
	entb_dma_info_t	*read_info;
	size_t		len, dma_size;

	if (!entb_plx_info) {
		return (DDI_FAILURE);
	}

	if (ntb_get_dma_attr(entb_plx_info->ntb_hdl, NTB_GET, &attr,
	    &acc_attr, &dma_size) != DDI_SUCCESS) {
		cmn_err(CE_WARN, "!entb_plx_alloc_dma_read(): "
		    "ntb_get_dma_attr() failed.");
		return (DDI_FAILURE);
	}

	read_info = &(entb_plx_info->r_info);

	if (ddi_dma_alloc_handle(dip, &attr, DDI_DMA_SLEEP, NULL,
	    &read_info->edi_hdl) != DDI_SUCCESS) {
		cmn_err(CE_WARN, "!entb: entb_plx_alloc_dma_read(): failed to "
		    "alloc read handle");
		return (DDI_FAILURE);
	}

	if (ddi_dma_mem_alloc(read_info->edi_hdl, dma_size, &acc_attr,
	    DDI_DMA_CONSISTENT, DDI_DMA_SLEEP, NULL, &read_info->edi_addr,
	    &len, &read_info->edi_acc_hdl) != DDI_SUCCESS) {
		cmn_err(CE_WARN, "!entb: entb_plx_alloc_dma_read(): failed to "
		    "alloc read mem");
		entb_plx_dma_read_free(entb_plx_info);
		return (DDI_FAILURE);
	}

	if (ddi_dma_addr_bind_handle(read_info->edi_hdl, NULL,
	    read_info->edi_addr, len, DDI_DMA_READ | DDI_DMA_CONSISTENT,
	    DDI_DMA_SLEEP, NULL, &read_info->edi_cookie,
	    &read_info->edi_cookie_cnt) != DDI_DMA_MAPPED) {
		cmn_err(CE_WARN, "!entb: entb_plx_alloc_dma_read(): failed to "
		    "bind read mem");
		entb_plx_dma_read_free(entb_plx_info);
		return (DDI_FAILURE);
	}

	return (DDI_SUCCESS);
}

static void
entb_plx_dma_free(entb_dma_info_t *info)
{
	if (info->edi_addr) {
		(void) ddi_dma_unbind_handle(info->edi_hdl);
	}

	if (info->edi_acc_hdl) {
		ddi_dma_mem_free(&info->edi_acc_hdl);
	}

	if (info->edi_hdl) {
		ddi_dma_free_handle(&info->edi_hdl);
	}
}

/* ARGSUSED */
static void
entb_plx_doorbell_start_read(void *arg, int doorbell)
{
	entb_plx_frame_header_t	*entb_hdr;
	int		ret;
	int		i;
	int32_t		hdr_size;
	int32_t		pkt_len;
	int32_t		count;
	entb_plx_ringbuf_info_t *epri = NULL;
	int32_t		epri_pktsize;
	int32_t		offset;
	int32_t		offset_debug;
	uint64_t	marker;
	entb_plx_t *entb_plx_info;
	entb_t		*entb_ss;
	entb_dma_info_t *read_info;

	entb_plx_info = (entb_plx_t *)arg;
	if (entb_plx_info == NULL) {
		return;
	}

	hdr_size = sizeof (entb_plx_frame_header_t);
	epri_pktsize = sizeof (entb_plx_ringbuf_info_t);
	entb_ss = entb_plx_info->entb_plx_ss;

	read_info = &(entb_plx_info->r_info);

	if (entb_ss->entb_link_state != LINK_STATE_UP) {
		return;
	}

	entb_hdr = entb_plx_read_and_verify_hdr(entb_ss,
	    entb_plx_info->entb_ringbuf_offset);
	if (entb_hdr == NULL) {
		cmn_err(CE_WARN, "!entb: Reading Ringbuf info: "
		    "entb_plx_read_and_verify_hdr() failed.");
		return;
	}

	(void) ntb_get(entb_plx_info->ntb_hdl, read_info->edi_hdl,
	    (entb_plx_info->entb_ringbuf_offset + hdr_size), epri_pktsize);

	epri = (entb_plx_ringbuf_info_t *)(uintptr_t)(read_info->edi_addr +
	    entb_plx_info->entb_ringbuf_offset + hdr_size);

	if (epri->epri_ringbuf_write_marker <
	    entb_plx_info->entb_ringbuf_local_read_marker) {
		/*
		 * This can happen if the local PLX chip has reset and the peer
		 * node has re-started sending the packets.
		 */
		entb_plx_info->entb_ringbuf_local_read_marker =
		    epri->epri_ringbuf_write_marker;
	} else {
		count = epri->epri_ringbuf_write_marker -
		    entb_plx_info->entb_ringbuf_local_read_marker;

		marker = entb_plx_info->entb_ringbuf_local_read_marker;
		for (i = 0; i < count; i++) {
			entb_plx_info->entb_ringbuf_local_read_marker++;
			offset = (((marker + i) %
			    entb_plx_info->entb_ringbuf_nelems) * ENTB_BUF_SIZE)
			    + ENTB_DATA_DMA_FIXED_OFFSET;
			offset_debug = (marker + i) %
			    entb_plx_info->entb_ringbuf_nelems;
			entb_hdr = entb_plx_read_and_verify_hdr(entb_ss,
			    offset);
			if (entb_hdr == NULL) {
				cmn_err(CE_WARN, "!entb: "
				    "entb_plx_read_and_verify_hdr failed. "
				    "Marker = %"PRIu64". Count = %d Offset "
				    "= %d Instance - %d", marker, count,
				    offset_debug, i);
				continue;
			}

			pkt_len = entb_hdr->epfh_txrx_packet_len;
			ret = entb_plx_read_data(entb_ss, offset + hdr_size,
			    pkt_len, B_FALSE);
			if (ret != DDI_SUCCESS) {
				cmn_err(CE_WARN, "!entb: entb_plx_read_data "
				    "failed.");
				continue;
			}
		}
	}

	entb_plx_ack_peer(entb_ss, 1);
}

static void
entb_plx_init_state(entb_plx_t *entb_plx_info)
{
	entb_plx_info->entb_plx_ss->entb_peer_state = ENTB_PEER_STATE_UNKNOWN;

	(void) ntb_set_doorbell(entb_plx_info->ntb_hdl, entb_plx_info->up,
	    entb_plx_peer_state_change, entb_plx_info);
	(void) ntb_set_doorbell(entb_plx_info->ntb_hdl, entb_plx_info->down,
	    entb_plx_peer_state_change, entb_plx_info);
}

static void
entb_plx_destroy_state(entb_plx_t *entb_plx_info)
{
	(void) ntb_set_doorbell(entb_plx_info->ntb_hdl, entb_plx_info->up,
	    NULL, NULL);
	(void) ntb_set_doorbell(entb_plx_info->ntb_hdl, entb_plx_info->down,
	    NULL, NULL);

	entb_plx_info->entb_plx_ss->entb_peer_state = ENTB_PEER_STATE_UNKNOWN;
}

static int
entb_plx_init_doorbells(entb_plx_t *entb_plx_info)
{
	int ret;
	int32_t	driver_doorbell_no;

	if (!entb_plx_info) {
		return (DDI_FAILURE);
	}

	driver_doorbell_no = entb_plx_info->ep_rx_doorbell_id;

	ret = ntb_set_doorbell(entb_plx_info->ntb_hdl, driver_doorbell_no,
	    entb_plx_doorbell_start_read, entb_plx_info);
	if (ret != DDI_SUCCESS) {
		cmn_err(CE_WARN, "!entb: plx_init_doorbells: ntb_set_doorbell "
		    "error: Doorbell(%d)", driver_doorbell_no);
		return (ret);
	}

	ret = ntb_set_doorbell(entb_plx_info->ntb_hdl, driver_doorbell_no + 1,
	    entb_plx_doorbell_peer_read_done, entb_plx_info);
	if (ret != DDI_SUCCESS) {
		cmn_err(CE_WARN, "!entb: plx_init_doorbells: ntb_set_doorbell "
		    "error: Doorbell(%d)", driver_doorbell_no + 1);
	}
	return (ret);
}

static int
entb_plx_destroy_doorbells(entb_plx_t *entb_plx_info)
{
	int ret;
	int32_t	driver_doorbell_no;

	if (!entb_plx_info) {
		return (DDI_FAILURE);
	}

	driver_doorbell_no = entb_plx_info->ep_rx_doorbell_id;

	ret = ntb_set_doorbell(entb_plx_info->ntb_hdl, driver_doorbell_no,
	    NULL, NULL);
	if (ret != DDI_SUCCESS) {
		cmn_err(CE_WARN, "!entb: plx_destroy_doorbells: "
		    "ntb_set_doorbell " "error: Doorbell(%d)",
		    driver_doorbell_no);
		return (ret);
	}

	ret = ntb_set_doorbell(entb_plx_info->ntb_hdl, driver_doorbell_no + 1,
	    NULL, NULL);
	if (ret != DDI_SUCCESS) {
		cmn_err(CE_WARN, "!entb: entb_plx_destroy_doorbells: "
		    "ntb_set_doorbell error: Doorbell(%d)",
		    driver_doorbell_no + 1);
	}
	return (ret);
}

static void
entb_plx_peer_state_change(void *arg, int doorbell)
{
	entb_plx_t *entb_plx_info = (entb_plx_t *)arg;
	entb_t		*entb_ss;
	link_state_t	new_state;

	if (!entb_plx_info) {
		cmn_err(CE_WARN, "!entb: Doorbell(%d) : NULL plx info. Exit!",
		    doorbell);
		return;
	}

	if (doorbell == entb_plx_info->up) {
		cmn_err(CE_WARN, "!entb: Doorbell(%d) : Recieved a state "
		    "change doorbell from Peer. State is now UP", doorbell);
	} else {
		cmn_err(CE_WARN, "!entb: Doorbell(%d) : Recieved a state "
		    "change doorbell from Peer. State is now DOWN", doorbell);
	}

	entb_ss = entb_plx_info->entb_plx_ss;
	if (entb_ss == NULL) {
		cmn_err(CE_WARN, "!entb: Doorbell(%d) : NULL entb info. Exit!",
		    doorbell);
		return;
	}

	if (doorbell == entb_plx_info->up &&
	    entb_ss->entb_peer_state == ENTB_PEER_STATE_UP) {
		return;
	} else if (doorbell == entb_plx_info->down &&
	    entb_ss->entb_peer_state == ENTB_PEER_STATE_DOWN) {
		return;
	}

	mutex_enter(&entb_ss->entb_link_lock);
	entb_ss->entb_peer_state = (doorbell == entb_plx_info->up) ?
	    ENTB_PEER_STATE_UP : ENTB_PEER_STATE_DOWN;

	if (doorbell == entb_plx_info->up) {
		entb_choose_backend(entb_ss);
		new_state = LINK_STATE_UP;
		entb_plx_info->entb_ringbuf_write_marker = 0;
		entb_plx_info->entb_ringbuf_peer_read_marker = 0;
	} else {
		new_state = LINK_STATE_DOWN;
	}

	if (entb_ss->entb_backend == plx_transport) {
		mac_link_update(entb_ss->entb_mac_hdl, new_state);
		entb_ss->entb_link_state = new_state;
	}

	mutex_exit(&entb_ss->entb_link_lock);
}

static void
entb_plx_dma_write_free(entb_plx_t *entb_plx_info)
{
	entb_dma_info_t	*write_info;

	if (!entb_plx_info) {
		return;
	}

	write_info = &(entb_plx_info->w_info);
	entb_plx_dma_free(write_info);
}

static void
entb_plx_dma_read_free(entb_plx_t *entb_plx_info)
{
	entb_dma_info_t	*read_info;

	if (!entb_plx_info) {
		return;
	}

	read_info = &(entb_plx_info->r_info);
	entb_plx_dma_free(read_info);
}

static void
entb_plx_transmit_ringbuf_info(entb_plx_t *entb_plx_info)
{
	entb_dma_info_t	*write_info;
	caddr_t		bufp;
	entb_plx_frame_header_t	entb_hdr;
	int32_t		pktsize;
	int32_t		hdr_size;
	int32_t		frame_len;
	entb_plx_ringbuf_info_t	epri;

	hdr_size = sizeof (entb_plx_frame_header_t);
	pktsize = sizeof (entb_plx_ringbuf_info_t);

	write_info = &(entb_plx_info->w_info);
	bufp = (write_info->edi_addr + entb_plx_info->entb_ringbuf_offset);

	frame_len = (hdr_size + pktsize);
	entb_hdr.epfh_header_id = ENTB_PLX_FRAME_IDENTIFIER;
	entb_hdr.epfh_txrx_packet_len = pktsize;
	entb_hdr.epfh_transmit_frame_len = frame_len;

	bcopy(&entb_hdr, bufp, hdr_size);
	bufp += hdr_size;

	epri.epri_ringbuf_write_marker =
	    entb_plx_info->entb_ringbuf_write_marker;
	bcopy(&epri, bufp, pktsize);

	(void) ntb_put(entb_plx_info->ntb_hdl, write_info->edi_hdl,
	    entb_plx_info->entb_ringbuf_offset, frame_len);
}

static entb_plx_frame_header_t *
entb_plx_read_and_verify_hdr(entb_t *entb_ss, int32_t offset)
{
	entb_plx_t *entb_plx_info;
	entb_plx_frame_header_t	*entb_hdr;
	int32_t		hdr_size = 0;
	entb_dma_info_t *read_info = NULL;

	hdr_size = sizeof (entb_plx_frame_header_t);
	entb_plx_info = entb_ss->entb_plx_info;
	read_info = &(entb_plx_info->r_info);
	(void) ntb_get(entb_plx_info->ntb_hdl, read_info->edi_hdl, offset,
	    hdr_size);

	entb_hdr = (entb_plx_frame_header_t *)(uintptr_t)(read_info->edi_addr +
	    offset);

	if (entb_hdr->epfh_header_id !=  ENTB_PLX_FRAME_IDENTIFIER ||
	    entb_hdr->epfh_transmit_frame_len <= 0 ||
	    entb_hdr->epfh_transmit_frame_len > ENTB_PLX_FRAME_MAX_LENGTH) {
		cmn_err(CE_WARN, "!received packet header=0x%x, framesize=%d",
		    entb_hdr->epfh_header_id,
		    entb_hdr->epfh_transmit_frame_len);
		return (NULL);
	}

	return (entb_hdr);
}

static int
entb_plx_read_data(entb_t *entb_ss, int32_t offset, int32_t len,
    boolean_t do_ack)
{
	entb_plx_t *entb_plx_info;
	caddr_t		bufp;
	mblk_t		*mp;
	entb_dma_info_t *read_info;
	int32_t		ret;

	entb_plx_info = entb_ss->entb_plx_info;
	read_info = &entb_plx_info->r_info;

	(void) ntb_get(entb_plx_info->ntb_hdl, read_info->edi_hdl, offset, len);
	bufp = read_info->edi_addr + offset;

	if (entb_ss->entb_link_state == LINK_STATE_UP) {
		mp = allocb(len, BPRI_HI);
		if (mp == NULL) {
			cmn_err(CE_WARN, "!failed to allocate mp");
			return (DDI_FAILURE);
		}
		mp->b_next = NULL;
		mp->b_cont = NULL;
		mp->b_wptr = mp->b_rptr + len;
		bcopy(bufp, mp->b_rptr, len);

		if (mp) {
			entb_post_rx_queue(entb_ss, mp);
		}
	}

	if (do_ack) {
		ret = ntb_send_doorbell(entb_plx_info->ntb_hdl,
		    entb_plx_info->ep_rx_doorbell_id + 1);
		if (ret != DDI_SUCCESS) {
			cmn_err(CE_WARN, "!entb: ERROR = %d: Could not update "
			    "peer on read complete! Ring buffers will not be "
			    "updated!", ret);
		}
	}

	return (DDI_SUCCESS);
}

/* ARGSUSED */
static void
entb_plx_doorbell_peer_read_done(void *arg, int doorbell)
{
	entb_plx_t	*entb_plx_info;
	boolean_t	was_ringbuf_full;

	entb_plx_info = (entb_plx_t *)arg;
	if (entb_plx_info == NULL) {
		return;
	}

	/*
	 * The peer node has indicated that it has processed a ring buf. Free
	 * the buf.
	 */
	was_ringbuf_full = (entb_plx_info->entb_ringbuf_write_marker -
	    entb_plx_info->entb_ringbuf_peer_read_marker) >=
	    entb_plx_info->entb_ringbuf_nelems ? B_TRUE : B_FALSE;

	entb_plx_info->entb_ringbuf_peer_read_marker =
	    entb_plx_info->entb_ringbuf_write_marker;

	if (was_ringbuf_full || entb_plx_info->entb_ringbuf_full) {
		cv_signal(&entb_plx_info->entb_tx_queue_cv);
		entb_plx_info->entb_ringbuf_full = B_FALSE;
	}
}

static void
entb_plx_ack_peer(entb_t *entb_ss, int32_t count)
{
	entb_plx_t *entb_plx_info;
	int32_t	ret;
	int32_t	i;

	entb_plx_info = entb_ss->entb_plx_info;
	for (i = 0; i < count; i++) {
		ret = ntb_send_doorbell(entb_plx_info->ntb_hdl,
		    entb_plx_info->ep_rx_doorbell_id + 1);
		if (ret != DDI_SUCCESS) {
			cmn_err(CE_WARN, "!error = %d: could not update "
			    "peer on read complete, ring buffers will not be "
			    "updated", ret);
		}
	}
}
