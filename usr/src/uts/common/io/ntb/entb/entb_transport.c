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
#include <sys/cmn_err.h>
#include <sys/ddi.h>
#include <sys/disp.h>
#include <sys/list.h>
#include <sys/sdt.h>
#include <sys/strsun.h>
#include <sys/sysmacros.h>

#include "../ntb_transport.h"
#include "entb_impl.h"
#include "entb_ntb.h"

/*
 * Transport mblks across the NTB.
 *
 * mblks are posted on the tx queue. The entb_sender() thread is responsible
 * for pulling mblks off the tx queue and storing them in the ring
 * on the _other side_ of the NTB. Esch mblk is prefixed with a packet
 * header to describe its size and provide the ability to do some data
 * integrity checks.
 * When a message arrives on the other side it is translated back into
 * a mblk which is posted on the rx queue. The entb_receiver() thread
 * takes mblks off the rx queue and posts them back up the network stack.
 *
 *                     |
 *      SIDE A         |               SIDE B
 *                     |
 *       mblk          |
 *        |            |
 *        V            |
 *    entb_sender()    |
 *        |            +
 *        +--- ntb_transport ---> ring buffer ---> endpoint_flushed()
 *                     +                                 |
 *                     |                                mblk
 *                     |                                 |
 *                     |                                 V
 *                     |                           entb_receiver()
 *                     |                                 |
 *                     |                                 V
 *                     |                             ip stack
 *
 * Although the ring only exists as kernel memory on the remote peer, its
 * producer and consumer indices are maintained locally. The entb header and
 * corresponding data are posted sequentially into the ring.
 * The endpoint_flushed() function is called in the peer for each discrete
 * ntb_sge sent, using this feature, sending the entb header as a separate
 * ntb_sge allows us to determine the boundaries of each mblk's data and
 * reconstruct a mblk to send back up the network stack.
 */

#define	ENTB_FRAME_IDENTIFIER	0xDEADFEED

typedef struct entb_ntb_frame_header_s  {
	uint32_t	enfh_header_id;
	uint32_t	enfh_transmit_frame_len;
	uint32_t	enfh_txrx_packet_len;
	uint32_t	enfh_padding;
} entb_ntb_frame_header_t;

/*
 * Tunables. Increasing entb_rx_threads may benefit throughput for
 *	     workloads with smaller messages.
 *	     Increasing entb_ring_size is likely to have no effect, the
 *	     gating factor is the rate at which the ioat driver can eat
 *	     up messages.
 */
uint32_t	entb_rx_threads = 1;
uint32_t	entb_ring_size = 16 * 1024 * 1024;

/*
 * Describes an entry in the ring buffer
 */
typedef struct ring_desc {
	int		flag;
	ntb_sge_t	sge;
} ring_desc_t;

#define	DESC_IN_USE	0x01

struct entb_conn {
	entb_ntb_t	*conn_ntb_info;
	ntb_handle_t	*conn_hdl;	/* hdl to peer */
	kt_did_t	conn_tx_thrid;	/* thread for pushing data */
	kt_did_t	*conn_rx_thrid;	/* threads for receiving data */
	volatile boolean_t was_down;
	boolean_t	shutdown;	/* B_TRUE to terminate thread */
	boolean_t	conn_waiting;	/* waiting for space in ring */
	int		conn_ep;	/* peer endpount id */
	caddr_t		conn_buf;	/* buffer to receive into */
	size_t		conn_buf_sz;	/* size of the buffer */
	offset_t	conn_prod;	/* current producer index */
	offset_t	conn_cons;	/* current consumer index */
	ring_desc_t	*conn_desc;	/* descriptor ring */
	uint32_t	conn_desc_sz;
	uint64_t	conn_desc_prod;	/* current desc producer */
	uint64_t	conn_desc_cons; /* current desc consumer */
	mblk_t		**conn_mp;	/* one per desc */

	mblk_t		*rx_mp;		/* chain of rx mblks */
	int		rx_length;	/* bytes to receive */

	kmutex_t	conn_lock;
	kcondvar_t	conn_cv;
};

typedef struct mp_entry {
	mblk_t	*mp;
	list_node_t link;
} mp_entry_t;

static kmem_cache_t *mp_cache;

static void	entb_sender(void *);
static void	entb_receiver(void *);
static void	entb_ntb_peer_state_change(void *, link_status_t);

void
entb_transport_init(void)
{
	mp_cache = kmem_cache_create("entb_mp_cache", sizeof (mp_entry_t),
	    0, NULL, NULL, NULL, NULL, NULL, 0);
}

void
entb_transport_fini(void)
{
	kmem_cache_destroy(mp_cache);
}

/*
 * Called to convert the dst field within the sge to a valid kernel virtual
 * address, into which the data represented by the sge will be flushed.
 */
static caddr_t
endpoint_dst_to_kva(void *arg, ntb_sge_t *sge)
{
	entb_conn_t	*peer = arg;

	return (peer->conn_buf + sge->remote);
}

/*
 * Called when we have data in our buffer. Convert it into an mblk, and
 * when the chain of mblks is complete post it to the rx queue.
 */
static void
endpoint_flushed(void *arg, ntb_sge_t *sge)
{
	entb_conn_t	*peer = arg;
	entb_ntb_t	*entb_ntb_info;
	entb_t		*entb_ss;
	mblk_t		*mp = NULL;
	mp_entry_t	*mpe;
	entb_ntb_frame_header_t fr_hdr;

	entb_ntb_info = peer->conn_ntb_info;
	entb_ss = entb_ntb_info->entb_ntb_ss;

	if (!entb_ntb_peer_ok(entb_ss)) {
		/* the peer will probably never see this ... */
		sge->result = EIO;
		return;
	}

	if (peer->was_down) {
		peer->was_down = B_FALSE;
		peer->rx_length = 0;
		if (peer->rx_mp != NULL) {
			freemsg(peer->rx_mp);
			peer->rx_mp = NULL;
		}
	}

	if (peer->rx_length == 0) {
		/*
		 * We are expecting a frame header
		 */
		if (sge->bytes != sizeof (fr_hdr)) {
			sge->result = EINVAL;
			return;
		}

		bcopy(peer->conn_buf + sge->remote, &fr_hdr, sge->bytes);

		if (fr_hdr.enfh_header_id != ENTB_FRAME_IDENTIFIER ||
		    fr_hdr.enfh_txrx_packet_len == 0 ||
		    fr_hdr.enfh_txrx_packet_len > entb_ss->entb_props->ep_mtu) {
			DTRACE_PROBE2(badhdr, entb_conn_t *, peer, ntb_sge_t *,
			    sge);
			sge->result = EINVAL;
			return;
		}

		/*
		 * Allocate a mblk big enough to accomodate the whole
		 * frame
		 */
		if ((mp = allocb(fr_hdr.enfh_txrx_packet_len, BPRI_HI)) ==
		    NULL) {
			sge->result = ENOMEM;
			return;
		}

		peer->rx_mp = mp;
		peer->rx_length = fr_hdr.enfh_txrx_packet_len;

		sge->result = 0;
		return;
	}

	peer->rx_length -= sge->bytes;
	if (peer->rx_length < 0) {
		DTRACE_PROBE2(badrx, entb_conn_t *, peer, mblk_t *,
		    peer->rx_mp);
		freemsg(peer->rx_mp);
		peer->rx_mp = NULL;
		peer->rx_length = 0;

		sge->result = EINVAL;
		return;
	}

	/* copy the data out of the ring into the mblk */
	bcopy(peer->conn_buf + sge->remote, peer->rx_mp->b_wptr, sge->bytes);
	peer->rx_mp->b_wptr += sge->bytes;

	sge->result = 0;

	if (peer->rx_length == 0) {
		/*
		 * The mblk is complete, post it onto the rx queue
		 */
		mpe = kmem_cache_alloc(mp_cache, KM_NOSLEEP);
		if (mpe == NULL) {
			return;
		}

		mpe->mp = peer->rx_mp;
		peer->rx_mp = NULL;
		DTRACE_PROBE2(rx, entb_conn_t *, peer, mblk_t *, mpe->mp);

		mutex_enter(&entb_ntb_info->rx_lock);
		list_insert_head(&entb_ntb_info->rx_list, mpe);
		cv_broadcast(&entb_ntb_info->rx_cv);
		mutex_exit(&entb_ntb_info->rx_lock);
	}
}

int
entb_ntb_init(entb_t *entb_ss, dev_info_t *dip)
{
	entb_ntb_t	*entb_ntb_info;
	entb_conn_t	*ntb_peer;
	ntb_handle_t	*ntb_hdl;
	kthread_t	*thr;
	char		ep_name[32];
	caddr_t		ring;
	uint32_t	mtu;
	size_t		max_xfer;
	int		ep, i;

	entb_ss->entb_ntb_info = kmem_zalloc(sizeof (entb_ntb_t), KM_SLEEP);

	entb_ntb_info = entb_ss->entb_ntb_info;
	entb_ntb_info->dip = dip;
	entb_ntb_info->entb_ntb_ss = entb_ss;

	(void) snprintf(ep_name, sizeof (ep_name), "%s%d", ddi_driver_name(dip),
	    entb_ss->entb_instance);
	if ((ep = ntb_name_to_endpoint_id(ep_name)) < 0) {
		cmn_err(CE_WARN, "!failed to get endpoint id for %s from "
		    "ntb_transport. Using default %d", ep_name,
		    entb_ss->entb_instance);
		ep = entb_ss->entb_instance;
	}

	entb_ntb_info->ep_ops.id = ep;
	entb_ntb_info->ep_ops.class = NTB_NETWORK;
	entb_ntb_info->ep_ops.remote_to_kva = endpoint_dst_to_kva;
	entb_ntb_info->ep_ops.remote_flushed = endpoint_flushed;
	entb_ntb_info->ep_ops.link_status = entb_ntb_peer_state_change;

	if ((ntb_hdl = ntb_alloc_handle(ep, NTB_NETWORK)) == NULL) {
		cmn_err(CE_WARN, "!Failed to allocate handle for ntb "
		    "transport");
		kmem_free(entb_ss->entb_ntb_info, sizeof (entb_ntb_t));
		entb_ss->entb_ntb_info = NULL;

		return (DDI_FAILURE);
	}

	/*
	 * Make sure our max mtu is no larger than the max xfer size
	 * support by a single ntb_sge_t
	 */
	max_xfer = ntb_transfer_max(ntb_hdl);
	if (max_xfer < entb_ss->entb_props->ep_mtu) {
		mtu = (uint32_t)max_xfer - (sizeof (struct ether_header) +
		    VLAN_TAGSZ);
		(void) entb_m_setprop(entb_ss, NULL, MAC_PROP_MTU,
		    sizeof (mtu), &mtu);
		entb_ss->entb_props->ep_mtu = (uint32_t)max_xfer;
		cmn_err(CE_WARN, "!Reduced mtu to %u due to ntb transport "
		    "message size restrictions", mtu);
	}

	ntb_peer = kmem_zalloc(sizeof (entb_conn_t), KM_SLEEP);
	entb_ntb_info->ntb_peer = ntb_peer;

	ring = kmem_alloc(entb_ring_size, KM_SLEEP);

	ntb_peer->conn_ntb_info = entb_ntb_info;
	ntb_peer->conn_ep = ep;
	ntb_peer->conn_buf_sz = entb_ring_size;
	ntb_peer->conn_buf = ring;
	ntb_peer->conn_hdl = ntb_hdl;
	/*
	 * A guess at the number of descriptors, assume on average a packet
	 * is 1/2 the mtu
	 */
	ntb_peer->conn_desc_sz = ntb_peer->conn_buf_sz /
	    (entb_ss->entb_props->ep_mtu / 2);
	ntb_peer->conn_desc = kmem_zalloc(sizeof (ring_desc_t) *
	    ntb_peer->conn_desc_sz, KM_SLEEP);
	ntb_peer->conn_mp = kmem_zalloc(sizeof (mblk_t *) *
	    ntb_peer->conn_desc_sz, KM_SLEEP);
	mutex_init(&ntb_peer->conn_lock, NULL, MUTEX_DEFAULT, NULL);
	cv_init(&ntb_peer->conn_cv, NULL, CV_DEFAULT, NULL);

	entb_ss->entb_peer_state = ENTB_PEER_STATE_UNKNOWN;

	if (ntb_register_endpoint(&entb_ntb_info->ep_ops, ntb_peer) !=
	    DDI_SUCCESS) {
		ntb_release_handle(ntb_hdl);
		kmem_free(ntb_peer->conn_desc, sizeof (ring_desc_t) *
		    ntb_peer->conn_desc_sz);
		kmem_free(ntb_peer->conn_mp, sizeof (mblk_t *) *
		    ntb_peer->conn_desc_sz);
		mutex_destroy(&ntb_peer->conn_lock);
		cv_destroy(&ntb_peer->conn_cv);

		kmem_free(ring, ntb_peer->conn_buf_sz);
		kmem_free(ntb_peer, sizeof (entb_conn_t));
		kmem_free(entb_ntb_info, sizeof (entb_ntb_t));
		entb_ss->entb_ntb_info = NULL;

		return (DDI_FAILURE);
	}


	list_create(&entb_ntb_info->tx_list, sizeof (mp_entry_t),
	    offsetof(mp_entry_t, link));
	mutex_init(&entb_ntb_info->tx_lock, NULL, MUTEX_DEFAULT, NULL);
	cv_init(&entb_ntb_info->tx_cv, NULL, CV_DEFAULT, NULL);

	list_create(&entb_ntb_info->rx_list, sizeof (mp_entry_t),
	    offsetof(mp_entry_t, link));
	mutex_init(&entb_ntb_info->rx_lock, NULL, MUTEX_DEFAULT, NULL);
	cv_init(&entb_ntb_info->rx_cv, NULL, CV_DEFAULT, NULL);

	/*
	 * entb_sender() pushes the mblks across the NTB
	 */
	thr = thread_create(NULL, 0, entb_sender, ntb_peer, 0, &p0,
	    TS_RUN, maxclsyspri);
	ntb_peer->conn_tx_thrid = thr->t_did;

	ntb_peer->conn_rx_thrid = kmem_alloc(sizeof (kt_did_t) *
	    entb_rx_threads, KM_SLEEP);
	for (i = 0; i < entb_rx_threads; i++) {
		/*
		 * entb_receiver() threads push mblks back into the ip stack
		 */
		thr = thread_create(NULL, 0, entb_receiver, ntb_peer, 0, &p0,
		    TS_RUN, maxclsyspri);
		ntb_peer->conn_rx_thrid[i] = thr->t_did;
	}

	return (DDI_SUCCESS);
}

/* ARGSUSED */
int
entb_ntb_destroy(entb_t *entb_ss, dev_info_t *dip)
{
	entb_ntb_t	*entb_ntb_info = entb_ss->entb_ntb_info;
	entb_conn_t	*ntb_peer = entb_ntb_info->ntb_peer;
	mp_entry_t	*mpe;
	int		i;

	/*
	 * First shutdown the threads
	 */
	ntb_peer->shutdown = B_TRUE;

	cv_broadcast(&entb_ntb_info->tx_cv);
	cv_broadcast(&entb_ntb_info->rx_cv);

	thread_join(ntb_peer->conn_tx_thrid);
	for (i = 0; i < entb_rx_threads; i++)
		thread_join(ntb_peer->conn_rx_thrid[i]);

	kmem_free(ntb_peer->conn_rx_thrid, sizeof (kt_did_t) * entb_rx_threads);

	/*
	 * Remove and free anything queued but not processed
	 */
	while ((mpe = list_remove_head(&entb_ntb_info->tx_list)) != NULL) {
		freemsg(mpe->mp);
		kmem_cache_free(mp_cache, mpe);
	}

	while ((mpe = list_remove_head(&entb_ntb_info->rx_list)) != NULL) {
		freemsg(mpe->mp);
		kmem_cache_free(mp_cache, mpe);
	}

	ntb_release_handle(ntb_peer->conn_hdl);
	(void) ntb_unregister_endpoint(&entb_ntb_info->ep_ops);

	kmem_free(ntb_peer->conn_desc, sizeof (ring_desc_t) *
	    ntb_peer->conn_desc_sz);
	kmem_free(ntb_peer->conn_mp, sizeof (mblk_t *) *
	    ntb_peer->conn_desc_sz);
	mutex_destroy(&ntb_peer->conn_lock);
	cv_destroy(&ntb_peer->conn_cv);

	kmem_free(ntb_peer->conn_buf, ntb_peer->conn_buf_sz);
	kmem_free(ntb_peer, sizeof (entb_conn_t));

	list_destroy(&entb_ntb_info->tx_list);
	mutex_destroy(&entb_ntb_info->tx_lock);
	cv_destroy(&entb_ntb_info->tx_cv);

	list_destroy(&entb_ntb_info->rx_list);
	mutex_destroy(&entb_ntb_info->rx_lock);
	cv_destroy(&entb_ntb_info->rx_cv);

	kmem_free(entb_ntb_info, sizeof (entb_ntb_t));

	entb_ss->entb_ntb_info = NULL;

	return (DDI_SUCCESS);
}

/*
 * Called as each sge is completed.
 * Update the consumer indicators in the ring buffers.
 */
/* ARGSUSED */
static void
entb_sge_done(ntb_handle_t *hdl, ntb_sge_t *sge, void *arg)
{
	entb_conn_t	*peer = arg;
	ring_desc_t	*desc;
	uint64_t	cons, gen, gen2;
	int		i;

	desc = (ring_desc_t *)((uintptr_t)sge - offsetof(ring_desc_t, sge));

	/* index of ring_desc in array */
	i = desc - peer->conn_desc;
	ASSERT(i >= 0 && i < peer->conn_desc_sz);

	/*
	 * There is no special processing if it failed, code higher
	 * up the stack will work it out.
	 */
	freeb(peer->conn_mp[i]);
	peer->conn_mp[i] = NULL;

	mutex_enter(&peer->conn_lock);

	desc->flag &= ~DESC_IN_USE;
	/*
	 * Run from the consumer index to the producer, freeing up
	 * ring space until we encounter a ring descriptor which is
	 * in use
	 */
	for (cons = peer->conn_desc_cons; cons < peer->conn_desc_prod; cons++) {
		i = cons % peer->conn_desc_sz;
		if (peer->conn_desc[i].flag & DESC_IN_USE)
			break;

		sge = &peer->conn_desc[i].sge;

		peer->conn_desc_cons++;

		/*
		 * sge's cannot wrap the ring buffer, check if it would and then
		 * reset the consumer to the start of the ring. The producer
		 * loop does the equivalent.
		 */
		gen = peer->conn_cons / peer->conn_buf_sz;
		gen2 = (peer->conn_cons + sge->bytes) / peer->conn_buf_sz;
		if (gen2 > gen)
			peer->conn_cons = gen2 * peer->conn_buf_sz;

		peer->conn_cons += sge->bytes;
	}

	if (peer->conn_waiting || peer->shutdown)
		cv_signal(&peer->conn_cv);

	mutex_exit(&peer->conn_lock);
}

/*
 * Send the data represented by the mblks across the NTB.
 * Each mblk is sent as a separate sge.
 */
static void
send_mblks(entb_conn_t *peer, mblk_t *hdr)
{
	mblk_t	*mp;
	int	i;
	uint64_t len, gen, gen2;
	uint64_t sge_avail, buf_avail;
	uint64_t start;

	while (hdr != NULL) {
		mp = hdr;
		hdr = hdr->b_cont;

		len = MBLKL(mp);
		if (len == 0) {
			/* don't send empty messages */
			freeb(mp);
			continue;
		}

		while (1 /* CONSTCOND */) {
			/*
			 * Whilst processing the producer/consumer
			 * variables we do *not* need to hold any locks
			 * the producer variables are only modified
			 * here and the consumer in entb_sge_done().
			 * The worst thing that can happen is the
			 * available space maybe _under_ calculated and
			 * we will wait an extra iteration.
			 */
			sge_avail = peer->conn_desc_sz -
			    (peer->conn_desc_prod - peer->conn_desc_cons);
			buf_avail = peer->conn_buf_sz -
			    (peer->conn_prod - peer->conn_cons);

			/*
			 * We don't want the data to wrap the buffer, check
			 * for that and if it will, force it to the start
			 * of the ring
			 */
			if (sge_avail > 0 && buf_avail >= len) {
				/*
				 * work out the current "generation" of the
				 * ring and compare it to what it would be
				 * after using space for this mblk
				 */
				gen = peer->conn_prod / peer->conn_buf_sz;
				gen2 = (peer->conn_prod + len) /
				    peer->conn_buf_sz;
				if (gen2 > gen) {
					/*
					 * It will overflow, so skip to
					 * the beginning of the ring
					 */
					peer->conn_prod = gen2 *
					    peer->conn_buf_sz;

					buf_avail = peer->conn_buf_sz -
					    (peer->conn_prod - peer->conn_cons);

					if (buf_avail >= len)
						/* still enough space */
						break;
				}

				break;
			}

			/* not enough space, wait */
			mutex_enter(&peer->conn_lock);
			peer->conn_waiting = B_TRUE;
			cv_wait(&peer->conn_cv, &peer->conn_lock);
			peer->conn_waiting = B_FALSE;
			mutex_exit(&peer->conn_lock);
		}

		i = peer->conn_desc_prod % peer->conn_desc_sz;
		start = peer->conn_prod % peer->conn_buf_sz;

		peer->conn_mp[i] = mp;
		peer->conn_desc[i].flag = DESC_IN_USE;
		peer->conn_desc[i].sge.local = (caddr_t)mp->b_rptr;
		peer->conn_desc[i].sge.bytes = (uint32_t)len;
		peer->conn_desc[i].sge.remote = start;

		/*
		 * We need to ensure the above reach global visibility
		 * before we increment the producers.
		 */
		membar_producer();

		peer->conn_desc_prod++;
		peer->conn_prod += len;

		(void) ntb_async_store(peer->conn_hdl, &peer->conn_desc[i].sge,
		    entb_sge_done, peer);
	}
}

/*
 * Takes mblks off the tx queue, and posts them into the ring buffer
 * across the ntb.
 */
static void
entb_sender(void *arg)
{
	entb_conn_t	*peer = arg;
	entb_ntb_t	*entb_ntb_info = peer->conn_ntb_info;
	entb_t		*entb_ss = entb_ntb_info->entb_ntb_ss;
	mp_entry_t	*mpe = NULL;
	mblk_t		*hdr, *mp;
	int		pktsize;
	entb_ntb_frame_header_t *fr_hdr;

	while (1 /* CONSTCOND */) {
		mutex_enter(&entb_ntb_info->tx_lock);
		while (!peer->shutdown &&
		    (mpe = list_remove_tail(&entb_ntb_info->tx_list)) == NULL) {
			cv_wait(&entb_ntb_info->tx_cv, &entb_ntb_info->tx_lock);
		}
		mutex_exit(&entb_ntb_info->tx_lock);

		if (peer->shutdown)
			break;

		hdr = allocb(sizeof (entb_ntb_frame_header_t), BPRI_HI);
		if (hdr == NULL) {
			freemsg(mpe->mp);
			kmem_cache_free(mp_cache, mpe);
			continue;
		}

		/* determine packet size */
		for (pktsize = 0, mp = mpe->mp; mp != NULL; mp = mp->b_cont)
			pktsize += MBLKL(mp);

		if (pktsize > entb_ss->entb_props->ep_mtu) {
			freemsg(mpe->mp);
			kmem_cache_free(mp_cache, mpe);
			continue;
		}

		/*
		 * Add a header mblk to be sent along with the packet.
		 * Each mblk is sent as a seperate ntb_sge.
		 */
		fr_hdr = (entb_ntb_frame_header_t *)(uintptr_t)hdr->b_wptr;
		hdr->b_wptr += sizeof (entb_ntb_frame_header_t);

		fr_hdr->enfh_header_id = ENTB_FRAME_IDENTIFIER;
		fr_hdr->enfh_txrx_packet_len = pktsize;
		fr_hdr->enfh_transmit_frame_len = pktsize + sizeof (*fr_hdr);

		linkb(hdr, mpe->mp);

		kmem_cache_free(mp_cache, mpe);

		send_mblks(peer, hdr);
	}

	/*
	 * Wait for anything outstanding to drain
	 */
	mutex_enter(&peer->conn_lock);
	while (peer->conn_desc_cons < peer->conn_desc_prod)
		cv_wait(&peer->conn_cv, &peer->conn_lock);
	mutex_exit(&peer->conn_lock);
}

/*
 * Post an mblk chain on the queue to be processed by the NTB
 * sender thread.
 */
void
entb_ntb_send(entb_t *entb_ss, mblk_t *mp)
{
	entb_ntb_t	*entb_ntb_info = entb_ss->entb_ntb_info;
	mp_entry_t	*mpe;

	if (!entb_ntb_peer_ok(entb_ss)) {
		freemsg(mp);
		return;
	}

	mpe = kmem_cache_alloc(mp_cache, KM_NOSLEEP);
	if (mpe == NULL) {
		freemsg(mp);
		return;
	}

	mpe->mp = mp;

	mutex_enter(&entb_ntb_info->tx_lock);
	list_insert_head(&entb_ntb_info->tx_list, mpe);
	cv_broadcast(&entb_ntb_info->tx_cv);
	mutex_exit(&entb_ntb_info->tx_lock);
}

/*
 * Takes mblks posted onto the rx queue and passes them up the network
 * stack.
 */
static void
entb_receiver(void *arg)
{
	entb_conn_t	*peer = arg;
	entb_ntb_t	*entb_ntb_info = peer->conn_ntb_info;
	entb_t		*entb_ss = entb_ntb_info->entb_ntb_ss;
	mp_entry_t	*mpe = NULL;

	while (1 /* CONSTCOND */) {
		mutex_enter(&entb_ntb_info->rx_lock);
		while (!peer->shutdown &&
		    (mpe = list_remove_tail(&entb_ntb_info->rx_list)) == NULL) {
			cv_wait(&entb_ntb_info->rx_cv, &entb_ntb_info->rx_lock);
		}
		mutex_exit(&entb_ntb_info->rx_lock);

		if (peer->shutdown)
			break;

		DTRACE_PROBE2(rx, entb_conn_t *, peer, mblk_t *, mpe->mp);
		mac_rx(entb_ss->entb_mac_hdl, NULL, mpe->mp);

		kmem_cache_free(mp_cache, mpe);
	}
}

/*
 * Called when there is a link state change.
 * Updates the mac layer as appropriate.
 */
static void
entb_ntb_peer_state_change(void *arg, link_status_t status)
{
	entb_conn_t	*ntb_peer = arg;
	entb_ntb_t	*entb_ntb_info = ntb_peer->conn_ntb_info;
	entb_t		*entb_ss = entb_ntb_info->entb_ntb_ss;
	link_state_t	new_state;

	if (status == TRANSPORT_CONNECTED &&
	    entb_ss->entb_peer_state == ENTB_PEER_STATE_UP)
		return;

	if (status == TRANSPORT_DISCONNECTED &&
	    entb_ss->entb_peer_state == ENTB_PEER_STATE_DOWN)
		return;

	mutex_enter(&entb_ss->entb_link_lock);
	entb_ss->entb_peer_state = status == TRANSPORT_CONNECTED ?
	    ENTB_PEER_STATE_UP : ENTB_PEER_STATE_DOWN;

	if (entb_ss->entb_peer_state == ENTB_PEER_STATE_DOWN) {
		new_state = LINK_STATE_DOWN;
		/* Now flag all connections, so they can do individual resets */
		entb_ntb_info->ntb_peer->was_down = B_TRUE;
	} else {
		entb_choose_backend(entb_ss);
		new_state = LINK_STATE_UP;
	}

	if (entb_ss->entb_backend == ntb_transport) {
		mac_link_update(entb_ss->entb_mac_hdl, new_state);
		entb_ss->entb_link_state = new_state;
	}
	mutex_exit(&entb_ss->entb_link_lock);

	cmn_err(CE_WARN, "!ntb_transport peer is %s",
	    entb_ss->entb_peer_state == ENTB_PEER_STATE_UP ? "up" :
	    "down");
}

boolean_t
entb_ntb_peer_ok(entb_t *entb_ss)
{
	if (entb_ss == NULL)
		return (B_FALSE);

	return (entb_ss->entb_peer_state == ENTB_PEER_STATE_UP);
}
