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

#include <sys/types.h>
#include <sys/callb.h>
#include <sys/cmn_err.h>
#include <sys/stat.h>

#include "entb_impl.h"
#include "entb_plx.h"

#define	ENTB_TX_POST		"entb_tx_process_thread"
#define	ENTB_TX_SHUTDOWN_REQ	0
#define	ENTB_TX_PROCESS_REQ	1

extern pri_t minclsyspri;

static void entb_do_post_tx_queue(entb_plx_t *entb_plx_info,
    entb_txrx_packet_t *pkt);
static void entb_stop_tx_thread(entb_plx_t *entb_plx_info);
static void entb_send_data(entb_plx_t *entb_plx_info);
static void entb_init_tx_lock(entb_plx_t *entb_plx_info);
static void entb_destroy_tx_lock(entb_plx_t *entb_plx_info);

void
entb_tx_init(entb_t *entb_ss)
{
	entb_plx_t	*entb_plx_info = entb_ss->entb_plx_info;
	kthread_t	*entb_kt;

	if (!entb_ss) {
		return;
	}

	entb_init_tx_lock(entb_plx_info);

	entb_kt = thread_create(NULL, 0, entb_send_data, entb_plx_info, 0,
	    &p0, TS_RUN, minclsyspri);

	entb_plx_info->entb_tx_thread = entb_kt->t_did;

	entb_plx_info->entb_ringbuf_write_marker = 0;
	entb_plx_info->entb_ringbuf_peer_read_marker = 0;
}

void
entb_tx_destroy(entb_t *entb_ss)
{
	if (!entb_ss->entb_plx_info) {
		return;
	}

	if (entb_ss->entb_plx_info->entb_tx_thread) {
		entb_stop_tx_thread(entb_ss->entb_plx_info);
		entb_ss->entb_plx_info->entb_tx_thread = 0;
	}

	entb_destroy_tx_lock(entb_ss->entb_plx_info);
}

void
entb_post_tx_queue(entb_t *entb_ss, mblk_t *mp)
{
	entb_txrx_packet_t *elem;

	elem = kmem_zalloc(sizeof (entb_txrx_packet_t), KM_SLEEP);
	elem->etp_next = NULL;
	elem->etp_type = ENTB_TX_PROCESS_REQ;
	elem->etp_mp = mp;

	entb_do_post_tx_queue(entb_ss->entb_plx_info, elem);
}

static void
entb_init_tx_lock(entb_plx_t *entb_plx_info)
{
	mutex_init(&entb_plx_info->entb_tx_queue_lock, NULL, MUTEX_DEFAULT,
	    NULL);
	cv_init(&entb_plx_info->entb_tx_queue_cv, NULL, CV_DRIVER, NULL);
}

static void
entb_destroy_tx_lock(entb_plx_t *entb_plx_info)
{
	mutex_destroy(&entb_plx_info->entb_tx_queue_lock);
}

static void
entb_do_post_tx_queue(entb_plx_t *entb_plx_info, entb_txrx_packet_t *pkt)
{
	entb_txrx_packet_t *elem;
	entb_txrx_packet_t *tail = NULL;

	mutex_enter(&entb_plx_info->entb_tx_queue_lock);
	if (entb_plx_info->entb_tx_queue) {
		if (entb_plx_info->entb_tx_queue->etp_type ==
		    ENTB_TX_SHUTDOWN_REQ) {
			/*
			 * Shutdown in progress. Don't queue anymore packets.
			 */
			mutex_exit(&entb_plx_info->entb_tx_queue_lock);
			kmem_free(pkt, sizeof (entb_txrx_packet_t));
			cmn_err(CE_WARN, "!entb: entb_do_post_tx_queue: "
			    "shutdown in progress");
			return;
		}
	}

	if (pkt->etp_type == ENTB_TX_SHUTDOWN_REQ) {
		/*
		 * Shutdown request should be added to the beginning of
		 * the queue.
		 */
		pkt->etp_next = entb_plx_info->entb_tx_queue;
		entb_plx_info->entb_tx_queue = pkt;
		cmn_err(CE_WARN, "!entb: entb_do_post_tx_queue: shutdown "
		    "request added to the queue");
	} else {
		for (elem = entb_plx_info->entb_tx_queue; elem;
		    elem = elem->etp_next) {
			tail = elem;
		}
		if (tail) {
			tail->etp_next = pkt;
		} else {
			entb_plx_info->entb_tx_queue = pkt;
		}
	}
	cv_signal(&entb_plx_info->entb_tx_queue_cv);

	mutex_exit(&entb_plx_info->entb_tx_queue_lock);
}

static void
entb_send_data(entb_plx_t *entb_plx_info)
{
	entb_txrx_packet_t	*entb_tx_pkt;
	entb_txrx_packet_t	*entb_tx_elem;
	kmutex_t	ci_lock;
	callb_cpr_t	ci;
	int32_t		pkt_type;
	boolean_t	do_sleep = B_FALSE;

	mutex_init(&ci_lock, NULL, MUTEX_DRIVER, NULL);
	CALLB_CPR_INIT(&ci, &ci_lock, callb_generic_cpr, ENTB_TX_POST);

wait_for_tx:
	mutex_enter(&entb_plx_info->entb_tx_queue_lock);
	while ((entb_tx_elem = entb_plx_info->entb_tx_queue) == NULL ||
	    do_sleep) {
		mutex_enter(&ci_lock);
		CALLB_CPR_SAFE_BEGIN(&ci);
		mutex_exit(&ci_lock);

		cv_wait(&entb_plx_info->entb_tx_queue_cv,
		    &entb_plx_info->entb_tx_queue_lock);
		entb_tx_elem = entb_plx_info->entb_tx_queue;
		do_sleep = B_FALSE;

		mutex_enter(&ci_lock);
		CALLB_CPR_SAFE_END(&ci, &ci_lock);
		mutex_exit(&ci_lock);
	}

	if (entb_tx_elem->etp_type != ENTB_TX_SHUTDOWN_REQ) {
		/*
		 * Calculate the availability of free ring buffers here itself.
		 */
		ASSERT(entb_plx_info->entb_ringbuf_write_marker >=
		    entb_plx_info->entb_ringbuf_peer_read_marker);
		if ((entb_plx_info->entb_ringbuf_write_marker -
		    entb_plx_info->entb_ringbuf_peer_read_marker) >=
		    entb_plx_info->entb_ringbuf_nelems) {
			/*
			 * We are out of buffers. Wait for buffers to be freed.
			 * TODO: Hmmmm, can we rely wholly on the peer device to
			 * update the ring buffer status? Maybe not. Its
			 * important to have a scavenger thread to free up ring
			 * buffers and it should be activated from here.
			 */
			entb_plx_info->entb_ringbuf_full = B_TRUE;
			cmn_err(CE_WARN, "!entb: entb: Ring buffer is full.");
			do_sleep = B_TRUE;
			mutex_exit(&entb_plx_info->entb_tx_queue_lock);
			goto wait_for_tx;
		}
	}

	/*
	 * Take the first request from the queue
	 */
	entb_plx_info->entb_tx_queue = entb_tx_elem->etp_next;
	entb_tx_elem->etp_next = NULL;
	pkt_type = entb_tx_elem->etp_type;

	/*
	 * Process the packet.
	 */
	switch (pkt_type) {
	case ENTB_TX_SHUTDOWN_REQ:
		while ((entb_tx_pkt = entb_plx_info->entb_tx_queue) != NULL) {
			entb_plx_info->entb_tx_queue = entb_tx_pkt->etp_next;
			entb_tx_pkt->etp_next = NULL;

			freemsg(entb_tx_pkt->etp_mp);
			kmem_free(entb_tx_pkt, sizeof (entb_txrx_packet_t));
		}
		mutex_exit(&entb_plx_info->entb_tx_queue_lock);
		break;
	case ENTB_TX_PROCESS_REQ:
		mutex_exit(&entb_plx_info->entb_tx_queue_lock);
		if (entb_plx_peer_ok(entb_plx_info)) {
			entb_plx_info->entb_ringbuf_write_marker++;
			(void) entb_plx_send(entb_plx_info,
			    entb_tx_elem->etp_mp);
		}

		break;
	default:
		cmn_err(CE_WARN, "!entb: entb_send_data: Illegal Request.");
		break;
	}

	if (entb_tx_elem) {
		freemsg(entb_tx_elem->etp_mp);
		kmem_free(entb_tx_elem, sizeof (entb_txrx_packet_t));
	}

	if (pkt_type == ENTB_TX_SHUTDOWN_REQ) {
		mutex_enter(&ci_lock);
		CALLB_CPR_EXIT(&ci);
		mutex_destroy(&ci_lock);

		return;
	}
	goto wait_for_tx;
}

static void
entb_stop_tx_thread(entb_plx_t *entb_plx_info)
{
	entb_txrx_packet_t *elem;

	elem = kmem_zalloc(sizeof (entb_txrx_packet_t), KM_SLEEP);
	elem->etp_next = NULL;
	elem->etp_type = ENTB_TX_SHUTDOWN_REQ;
	elem->etp_mp = NULL;

	entb_do_post_tx_queue(entb_plx_info, elem);

	thread_join(entb_plx_info->entb_tx_thread);
}
