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

#ifndef	_NTB_TRANSPORT_H
#define	_NTB_TRANSPORT_H

#include <sys/types.h>
#include <sys/dcopy.h>
#include <sys/disp.h>
#include <sys/list.h>
#include <sys/sunddi.h>

#include "ntb.h"

typedef struct ntb_sge {
	caddr_t	local;		/* virtual address at local end */
	size_t	bytes;		/* bytes to transfer */
	uint64_t remote;	/* location at remote end */
	int	result;		/* errno, 0 == success */
} ntb_sge_t;

typedef struct ntb_handle ntb_handle_t;
typedef void (*ntb_cb_fn_t)(ntb_handle_t *, ntb_sge_t *, void *);

typedef enum { TRANSPORT_CONNECTED, TRANSPORT_DISCONNECTED } link_status_t;

typedef enum {
	NTB_NETWORK = 0,	/* network traffic, Eg entb */
	NTB_BULK,		/* bulk data, Eg nvrd */
	NTB_CLASS_CNT		/* always last */
} ntb_class_t;

/*
 * Each "endpoint" driver needs to define one of these.
 * remote_to_kva() is called by the ntb_transport so the endpoint can
 * convert the "remote" from the originators ntb_sge_t to a valid
 * kernel virtual address. It should return the kva and DDI_SUCCESS if the
 * combination of remote and byte count arguments are valid.
 *
 * remote_flushed() is called by the ntb_transport when data represented
 * by a sge has been flushed to endpoints private buffer. The routine
 * should set the "result" element of the ntb_sge_t to indicate it has
 * successfully (or not) dealt with the data. This is fed back to the
 * originators ntb_sge_t.
 */
typedef struct ntb_endpoint_ops {
	uint_t		id;		/* endpoint's unique id */
	ntb_class_t	class;		/* endpoint's data class */
	caddr_t	(*remote_to_kva)(void *, ntb_sge_t *);
	void	(*remote_flushed)(void *, ntb_sge_t *);
	void	(*link_status)(void *, link_status_t);
} ntb_endpoint_ops_t;

extern int	ntb_name_to_endpoint_id(const char *);
extern int	ntb_register_endpoint(ntb_endpoint_ops_t *, void *);
extern int	ntb_unregister_endpoint(ntb_endpoint_ops_t *);

extern ntb_handle_t	*ntb_alloc_handle(uint_t, ntb_class_t);
extern void		ntb_release_handle(ntb_handle_t *);
extern int		ntb_store(ntb_handle_t *, ntb_sge_t *);
extern int		ntb_async_store(ntb_handle_t *, ntb_sge_t *,
			    ntb_cb_fn_t, void *);
extern int		ntb_sync(ntb_handle_t *);
extern int		ntb_async_fetch(ntb_handle_t *, ntb_sge_t *,
			    ntb_cb_fn_t, void *);
extern int		ntb_fetch(ntb_handle_t *, ntb_sge_t *);
extern size_t		ntb_transfer_max(ntb_handle_t *);

#define	HIGHEST_ENDPOINT_ID	256

#define	DCOPY_POLL_LIMIT	5
#define	DCOPY_TIMEOUT_US	1400000		/* 1400 millisecs in microsecs */
#define	ALIVE_INTERVAL_US	500000		/* 1/2 second in microsecs */
#define	MAX_SGE			(256 * 1024)	/* max a single sge can xfer */

#define	MAX_SIZET	((size_t)(~(size_t)0)-2)

typedef enum { NTB_FETCH, NTB_STORE } ntb_dir_t;
typedef enum {
	VERSION1 = 1,
	CURRENT_VERSION
} ntb_version_t;

typedef struct ntb_trans ntb_trans_t;

/*
 * tracks the ring buffer for the real data
 */
typedef struct data_hdr {
	caddr_t		head;		/* beginning of ring */
	uint_t		head_off;	/* offset from dma start to ring */
	size_t		size;		/* size of ring */
	uint64_t	prod;		/* prod % size = idx of nxt free byte */
	uint64_t	cons;		/* cons % size = idx of 1st used byte */
} data_hdr_t;

typedef struct ring_entry {
	uint64_t	id;		/* unique id for each sgl entry */
	caddr_t		src;		/* source physical address */
	size_t		size;		/* number of bytes */
	uint64_t	offset;		/* offset into destinations buffer */
	uint64_t	start;		/* start of data in data_hdr_t */
	ntb_dir_t	dir;		/* sending or fetching? */
	int		endpoint_id;	/* id of endpoint in peer */
	int		err;		/* returned error state (errno) */
} ring_entry_t;

/*
 * tracks the ring buffer of sgl entries
 */
typedef struct sgl_hdr {
	ring_entry_t	*head;		/* head of sgl ring */
	uint_t		head_off;	/* offset from dma start to ring */
	uint_t		size;		/* num of ring_entry in ring */
	volatile uint64_t next_prod;	/* next prod to be processed */
	volatile uint64_t prod;		/* prod % size = idx of nxt free sgl */
	volatile uint64_t cons;		/* cons % size = idx of 1st used sgl */
} sgl_hdr_t;

typedef struct dma_msg {
	uint64_t	msg_id;		/* unique identifier for msg */
	int		msg;		/* the msg code */
} dma_msg_t;

typedef enum {
	I_AM_ALIVE = 0,
	I_AM_DYING,
	RESET_RINGS,
	HEADERS_SENT,
	HEADERS_ACK,
	MSG_CNT		/* always last */
} internal_msg_t;

typedef struct dma_ring {
	sgl_hdr_t	sgl_ring;	/* ring of sgl info */
	data_hdr_t	data_ring;	/* ring of data referenced by sgl */
	int		s_id;		/* current session id */

					/* msg_id and msg must always be */
					/* last in struct. */
	uint64_t	msg_id;		/* id of last message sent */

					/* there is a "mailbox" for each */
					/* internal_msg_t */
	dma_msg_t	msg[MSG_CNT];	/* to send internal messages */
} dma_ring_t;

typedef struct dma_header {
	dma_ring_t	*send;		/* send rings */
	offset_t	send_off;	/* offset with DMA buffer */
	dma_ring_t	*recv;		/* recv progress and status */
	offset_t	recv_off;	/* offset with DMA buffer */
	uint_t		header_sz;
} dma_header_t;

/*
 * The first 4K of the the NTB memory segment is used to track
 * the state of the rings which needs to be shared by the peers
 */
#define	SGL_OFFSET	0x1000
#define	MIN_RING_SIZE	(2 * SGL_OFFSET)	/* < this causes panic */
#define	DATA_MSG_SIZE	512			/* a guess .... */

typedef struct dma_info {
	caddr_t		vaddr;
	size_t		size;
	ddi_dma_handle_t dma_hdl;
	ddi_acc_handle_t dma_acc_hdl;
	ddi_dma_cookie_t *cookies;	/* cookies for NTB segments */
	int		ccnt;
} dma_info_t;

typedef struct ntb_ep_hdl {
	void			*user_arg;	/* caller supplied arg */
	ntb_endpoint_ops_t	*ops;		/* caller's ops vector */
	volatile uint_t		ref;		/* reference counter */
	list_node_t		link;
} ntb_ep_hdl_t;

/*
 * Provides a link between the sge provided by the user and the
 * entry used in the ring buffers
 */
typedef struct sge_req {
	ntb_sge_t	*sge;		/* originators sge */
	uint64_t	id;		/* uinique id assigned to request */
	ntb_handle_t	*hdl;
	ntb_cb_fn_t	complete_fn;	/* sge completion callback */
	void		*complete_arg;	/* private argument for complete_fn */
	uint_t		ref;		/* reference count */
	boolean_t	ready;		/* this req is ready to send  */
	boolean_t	blocked;	/* this req is blocking */
	boolean_t	notified;	/* sge notified req is finished */
	int		idx;		/* index in array */
	dcopy_cmd_t	b_cmd;		/* for data and sgls to bridge */
	dcopy_cmd_t	k_cmd;		/* for data and sgls to kernel */
	list_node_t	link;
	list_node_t	p_link;		/* post queue link */
} sge_req_t;

struct ntb_handle {
	ntb_trans_t	*ntb;		/* bridge the handle belongs to */
	uint_t		endpoint_id;	/* endpoint this handle sends to */
	list_t		req_list;	/* list of pending/active requests */
	int		waiting;	/* ntb_sync()'s waiting */
	int		incomplete;	/* no. requests incomplete */
	kmutex_t	hdl_lock;
	kcondvar_t	hdl_cv;
	list_node_t	link;
};

typedef struct ntb_node {
	dev_info_t	*dip;
	int		inst;
	ntb_svc_t	*ntb_hdl;	/* handle for ntb services */
	ntb_kstat_t	*ntb_kdata;	/* ntb kstat data */
	uint_t		up;		/* up doorbell */
	uint_t		down;		/* down doorbell */
	uint_t		ack;		/* ack doorbell */
	uint_t		data;		/* data doorbell */
} ntb_node_t;

struct ntb_trans {
	ntb_node_t	*node;
	list_node_t	c_link;		/* list of ntb_trans of same class */
	int		id;		/* transport id */

	int		state;
	ntb_class_t	class;		/* for now interactive or bulk */
	ntb_version_t	version;	/* active version */

	int		seg;		/* first segment for this ntb_trans */
	int		cnt;		/* and number */

	dma_info_t	r_info;		/* dma for "in" hdrs, sgls */
	dma_info_t	rd_info;	/* dma for "in" data */
	dma_info_t	w_info;		/* dma for "out" hdrs, sgls */
	dma_info_t	wd_info;	/* dma for "out" data */
	dma_info_t	*rd_infop;	/* dma_info of read data dma */
	dma_info_t	*wr_infop;	/* dma_info of write data dma */
	dma_header_t	out;		/* outbound ring buffers */
	dma_header_t	in;		/* inbound rings */

	sge_req_t	**callers;	/* sgl requests of callers */
	ring_entry_t	**reap;		/* sgl's which need reaping */
	ntb_sge_t	*endpoint_sge;	/* sge's passed to endpoints */
	int		caller_sz;	/* size of array */
	size_t		sge_xfer_max;	/* max size of a single sge request */

	dcopy_handle_t	k_dcopy_hdl;	/* for dcopy to kernel */
	dcopy_handle_t	b_dcopy_hdl;	/* for dcopy to bridge */
	boolean_t	dcopy_ena;	/* dcopy() is enabled */
	size_t		dcopy_threshold; /* size to start using dcopy */

	uint64_t	last_cons;	/* last cons ack'ed */
	uint64_t	last_prod;	/* last processed sgl */
	boolean_t	data_pending;	/* data doorbell is pending */
	boolean_t	ack_pending;	/* ack doorbell is pending */
	int		pending;	/* count of pending sends */

	list_t		hdl_list;	/* list of ntb handles allocated */
	uint_t		hdl_count;	/* count of ntb handles */
	list_t		ack_req;	/* list of req being ack'ed */
	list_t		post_q;		/* ordered q of sge_req_t to be sent */
	boolean_t	alive_sending;	/* alive thread is sending */
	boolean_t	in_reset;	/* in critical reset_rings path */
	ddi_taskq_t	*status_tq;	/* taskq for status change callback */

	kmutex_t	rupt_lock;
	kmutex_t	xmit_lock;
	kmutex_t	ring_lock;
	kcondvar_t	ring_cv;
	kcondvar_t	data_cv;
	kcondvar_t	ack_cv;
	kcondvar_t	post_cv;
	kcondvar_t	alive_cv;
	kcondvar_t	reset_cv;

	int		waiting;	/* threads waiting for ring space */
	kt_did_t	data_thr;
	kt_did_t	ack_thr;
	kt_did_t	post_thr;
	kt_did_t	alive_thr;

	uint64_t	last_msg_id;	/* id of last message */
	dma_msg_t	last_msg[MSG_CNT]; /* last internal msgs from peer */
	void		(*msg_fn[MSG_CNT])(ntb_trans_t *, dma_msg_t *);
};

#define	STATE_NTB_UP	0x01
#define	STATE_DRIVER_UP	0x02
#define	STATE_RESETING	0x04
#define	STATE_ENDING	0x08
#define	STATE_CANCEL	0x10		/* cancel ack and data threads */
#define	STATE_INITED	0x20		/* ntb_trans struct has been init'ed */

#define	STATE_UP	(STATE_NTB_UP | STATE_DRIVER_UP)

#define	peer_down(ntb)	(((ntb)->state & (STATE_UP | STATE_RESETING)) != \
			    STATE_UP)
#define	peer_up(ntb)	(!peer_down(ntb))

typedef struct ep_reg {
	ntb_endpoint_ops_t	*ops;
	list_t			link;
} ep_reg_t;

extern void	ntb_names_init(void);
extern void	ntb_names_fini(void);

#endif
