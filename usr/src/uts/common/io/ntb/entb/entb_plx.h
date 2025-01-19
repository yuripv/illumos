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
 * Copyright 2025 Tintri by DDN, Inc. All rights reserved.
 */

#ifndef _ENTB_PLX_H
#define	_ENTB_PLX_H

#include "../ntb.h"

#define	PLX_DEVICE_SEGMENT_SIZE			(256 * 1024)
#define	ENTB_BUF_SIZE				(64 * 1024)
#define	ENTB_MAX_BUFS_PER_SEGMENT		\
	(PLX_DEVICE_SEGMENT_SIZE / ENTB_BUF_SIZE)
#define	ENTB_DATA_DMA_FIXED_OFFSET		ENTB_BUF_SIZE

typedef struct entb_dma_info_s {
	caddr_t		edi_addr;
	ddi_dma_handle_t edi_hdl;
	ddi_acc_handle_t edi_acc_hdl;
	ddi_dma_cookie_t edi_cookie;
	uint_t		edi_cookie_cnt;
	size_t		edi_size;
} entb_dma_info_t;

typedef struct entb_txrx_packet_s {
	struct entb_txrx_packet_s	*etp_next;
	int32_t				etp_type;
	mblk_t				*etp_mp;
} entb_txrx_packet_t;

typedef struct entb_plx_s {
	dev_info_t		*dip;
	struct entb_s		*entb_plx_ss;
	ntb_svc_t		*ntb_hdl;
	entb_dma_info_t		w_info;
	entb_dma_info_t		r_info;
	uint_t			up;
	uint_t			down;
	int			inst;
	int32_t			ep_current_frame_len;
	int32_t			ep_rx_doorbell_id;

	boolean_t		entb_ringbuf_full;
	int32_t			entb_ringbuf_offset;
	int32_t			entb_ringbuf_nelems;
	uint64_t		entb_ringbuf_write_marker;
	uint64_t		entb_ringbuf_peer_read_marker;
	uint64_t		entb_ringbuf_local_read_marker;

	kcondvar_t		entb_rx_queue_cv;
	kmutex_t		entb_rx_queue_lock;
	kt_did_t		entb_rx_thread;
	entb_txrx_packet_t	*entb_rx_queue;

	kcondvar_t		entb_tx_queue_cv;
	kmutex_t		entb_tx_queue_lock;
	kt_did_t		entb_tx_thread;
	entb_txrx_packet_t	*entb_tx_queue;
} entb_plx_t;

extern void entb_post_tx_queue(entb_t *entb_ss, mblk_t *mp);
extern void entb_choose_backend(entb_t *entb_ss);

extern void entb_tx_init(entb_t *entb_ss);
extern void entb_tx_destroy(entb_t *entb_ss);
extern void entb_post_tx_queue(entb_t *entb_ss, mblk_t *mp);
extern void entb_rx_init(entb_t *entb_ss);
extern void entb_rx_destroy(entb_t *entb_ss);
extern void entb_post_rx_queue(entb_t *entb_ss, mblk_t *mp);

extern int entb_plx_init(entb_t *entb_ss, dev_info_t *dip);
extern int entb_plx_destroy(entb_t *entb_ss, dev_info_t *dip);
extern int entb_plx_send(entb_plx_t *entb_plx_info, mblk_t *mp);
extern boolean_t entb_plx_peer_ok(entb_plx_t *entb_plx_info);

#endif
