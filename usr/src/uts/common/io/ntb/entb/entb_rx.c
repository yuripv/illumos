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
#include <sys/stat.h>
#include <sys/cmn_err.h>
#include <sys/callb.h>

#include "entb_impl.h"
#include "entb_plx.h"

#define	ENTB_RX_POST "entb_rx_process_thread"
#define	ENTB_RX_SHUTDOWN_REQ	0
#define	ENTB_RX_PROCESS_REQ	1

extern pri_t minclsyspri;
static void entb_recv_data(entb_plx_t *entb_plx_info);
static void entb_do_post_rx_queue(entb_plx_t *entb_plx_info,
    entb_txrx_packet_t *pkt);
static void entb_stop_rx_thread(entb_plx_t *entb_plx_info);
static void entb_init_rx_lock(entb_plx_t *entb_plx_info);
static void entb_destroy_rx_lock(entb_plx_t *entb_plx_info);

void
entb_post_rx_queue(entb_t *entb_ss, mblk_t *mp)
{
	entb_txrx_packet_t *elem;

	elem = kmem_zalloc(sizeof (entb_txrx_packet_t), KM_SLEEP);
	elem->etp_next = NULL;
	elem->etp_type = ENTB_RX_PROCESS_REQ;
	elem->etp_mp = mp;

	entb_do_post_rx_queue(entb_ss->entb_plx_info, elem);
}

void
entb_rx_init(entb_t *entb_ss)
{
	entb_plx_t	*entb_plx_info = entb_ss->entb_plx_info;
	kthread_t	*entb_kt;

	if (!entb_plx_info) {
		return;
	}

	entb_init_rx_lock(entb_plx_info);

	entb_kt = thread_create(NULL, 0, entb_recv_data, entb_plx_info, 0,
	    &p0, TS_RUN, minclsyspri);
	entb_plx_info->entb_rx_thread = entb_kt->t_did;

	entb_plx_info->entb_ringbuf_local_read_marker = 0;
}

void
entb_rx_destroy(entb_t *entb_ss)
{
	if (!entb_ss->entb_plx_info) {
		return;
	}

	if (entb_ss->entb_plx_info->entb_rx_thread) {
		entb_stop_rx_thread(entb_ss->entb_plx_info);
		entb_ss->entb_plx_info->entb_rx_thread = 0;
	}
	entb_destroy_rx_lock(entb_ss->entb_plx_info);
}

static void
entb_recv_data(entb_plx_t *entb_plx_info)
{
	entb_txrx_packet_t *entb_rx_pkt;
	entb_txrx_packet_t *entb_elem;
	kmutex_t ci_lock;
	callb_cpr_t ci;
	int32_t	pkt_type;

	mutex_init(&ci_lock, NULL, MUTEX_DRIVER, NULL);
	CALLB_CPR_INIT(&ci, &ci_lock, callb_generic_cpr, ENTB_RX_POST);

wait_for_recv:
	mutex_enter(&entb_plx_info->entb_rx_queue_lock);
	while ((entb_elem = entb_plx_info->entb_rx_queue) == NULL) {
		mutex_enter(&ci_lock);
		CALLB_CPR_SAFE_BEGIN(&ci);
		mutex_exit(&ci_lock);

		cv_wait(&entb_plx_info->entb_rx_queue_cv,
		    &entb_plx_info->entb_rx_queue_lock);
		entb_elem = entb_plx_info->entb_rx_queue;

		mutex_enter(&ci_lock);
		CALLB_CPR_SAFE_END(&ci, &ci_lock);
		mutex_exit(&ci_lock);
	}

	/*
	 * Take the first request from the queue
	 */
	entb_plx_info->entb_rx_queue = entb_elem->etp_next;
	entb_elem->etp_next = NULL;
	pkt_type = entb_elem->etp_type;

	/*
	 * Process the packet.
	 */
	switch (pkt_type) {
	case ENTB_RX_SHUTDOWN_REQ:
		while ((entb_rx_pkt = entb_plx_info->entb_rx_queue) != NULL) {
			entb_plx_info->entb_rx_queue = entb_rx_pkt->etp_next;
			entb_rx_pkt->etp_next = NULL;
			kmem_free(entb_rx_pkt, sizeof (entb_txrx_packet_t));
		}
		mutex_exit(&entb_plx_info->entb_rx_queue_lock);
		break;
	case ENTB_RX_PROCESS_REQ:
		mutex_exit(&entb_plx_info->entb_rx_queue_lock);
		if (entb_elem->etp_mp) {
			mac_rx(entb_plx_info->entb_plx_ss->entb_mac_hdl,
			    NULL, entb_elem->etp_mp);
		}
		break;
	default:
		cmn_err(CE_WARN, "!entb: entb_recv_data: Illegal Request.");
		break;
	}

	if (entb_elem) {
		kmem_free(entb_elem, sizeof (entb_txrx_packet_t));
	}

	if (pkt_type == ENTB_RX_SHUTDOWN_REQ) {
		mutex_enter(&ci_lock);
		CALLB_CPR_EXIT(&ci);
		mutex_destroy(&ci_lock);

		return;
	}

	goto wait_for_recv;
}

static void
entb_do_post_rx_queue(entb_plx_t *entb_plx_info, entb_txrx_packet_t *pkt)
{
	entb_txrx_packet_t *elem;
	entb_txrx_packet_t *tail = NULL;

	if (!entb_plx_info) {
		return;
	}

	mutex_enter(&entb_plx_info->entb_rx_queue_lock);
	if (entb_plx_info->entb_rx_queue) {
		if (entb_plx_info->entb_rx_queue->etp_type ==
		    ENTB_RX_SHUTDOWN_REQ) {

			/*
			 * Shutdown in progress. Don't queue anymore packets.
			 */
			mutex_exit(&entb_plx_info->entb_rx_queue_lock);
			kmem_free(pkt, sizeof (entb_txrx_packet_t));
			cmn_err(CE_WARN, "!entb: entb_do_post_rx_queue: "
			    "shutdown in progress");
			return;
		}
	}

	if (pkt->etp_type == ENTB_RX_SHUTDOWN_REQ) {
		/*
		 * Shutdown request should be added to the beginning of
		 * the queue.
		 */
		pkt->etp_next = entb_plx_info->entb_rx_queue;
		entb_plx_info->entb_rx_queue = pkt;
	} else {
		for (elem = entb_plx_info->entb_rx_queue; elem;
		    elem = elem->etp_next) {
			tail = elem;
		}
		if (tail) {
			tail->etp_next = pkt;
		} else {
			entb_plx_info->entb_rx_queue = pkt;
		}
	}
	cv_signal(&entb_plx_info->entb_rx_queue_cv);

	mutex_exit(&entb_plx_info->entb_rx_queue_lock);
}

static void
entb_stop_rx_thread(entb_plx_t *entb_plx_info)
{
	entb_txrx_packet_t *elem;

	elem = kmem_zalloc(sizeof (entb_txrx_packet_t), KM_SLEEP);
	elem->etp_next = NULL;
	elem->etp_type = ENTB_RX_SHUTDOWN_REQ;
	elem->etp_mp = NULL;

	entb_do_post_rx_queue(entb_plx_info, elem);

	thread_join(entb_plx_info->entb_rx_thread);
}

static void
entb_init_rx_lock(entb_plx_t *entb_plx_info)
{
	mutex_init(&entb_plx_info->entb_rx_queue_lock, NULL, MUTEX_DEFAULT,
	    NULL);
	cv_init(&entb_plx_info->entb_rx_queue_cv, NULL, CV_DRIVER, NULL);
}

static void
entb_destroy_rx_lock(entb_plx_t *entb_plx_info)
{
	mutex_destroy(&entb_plx_info->entb_rx_queue_lock);
}
