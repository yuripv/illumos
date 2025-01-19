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
#include <sys/sysmacros.h>
#include <sys/sunddi.h>
#include <sys/sunndi.h>
#include <sys/autoconf.h>
#include <sys/modctl.h>
#include <sys/devops.h>
#include <sys/conf.h>
#include <sys/lgrp.h>
#include <sys/atomic.h>
#include <sys/spl.h>
#include <sys/sdt.h>
#include <sys/ddi.h>
#include <sys/dcopy.h>

#include "ntb.h"
#include "ntb_transport.h"

/*
 * There are two interfaces into the driver. There is one for a client to
 * exchange data with an endpoint at the peer, and one for the endpoint driver
 * to register.
 *
 * Each endpoint driver has a unique endpoint_id. The configuration file
 * /etc/ntb_transport/names_to_endpoint maps a meaningful name to an
 * endpoint_id and also provides a central management point for allocation
 * of endpoint_ids.
 *
 * Client API
 * ==========
 * int ntb_name_to_endpoint_id(const char *name)
 *	Given the name of an endpoint, looks it up in
 *	/etc/ntb_transport/names_to_endpoint and returns the endpoint_id.
 *	Will return -1 if an entry for name is not found.
 *	The endpoint_id returned should be used in the subsequent call(s)
 *	to ntb_alloc_handle(), and stored in the id field of ntb_endpoint_ops_t
 *	structure used in ntb_register_endpoint().
 *
 * ntb_handle_t *ntb_alloc_handle(int endpoint_id, ntb_class_t class))
 *	Allocates and returns a handle for data exchange with endpoint_id.
 *	class is either NTB_NETWORK or NTB_BULK and provides a mechanism for
 *	separating these traffic types. E.g. entb will use network and
 *	nvrd use bulk.
 *
 * void ntb_release_handle(ntb_handle_t *hdl)
 *	Frees the handle and resources used. It is up to the caller
 *	to ensure there are not outstanding requests, a call to ntb_sync()
 *	will do this.
 *
 * int ntb_async_store(ntb_handle_t *hdl, ntb_sge_t *sge, ntb_cb_fn_t cb,
 *	    void *arg)
 *	Asynchronously store data describe by sge to the endpoint
 *	described by hdl. The sge has the local source address of the data,
 *	its size and a destination which is meaningful to the endpoint.
 *	Typically the destination would be an offset within an address
 *	range.
 *	cb(hdl, sge, arg) is called when the sge has been acked, any error
 *	status has been set to indicate success/failure. It can be NULL, then
 *	ntb_sync() needs to be called to wait for completion of any store
 *	requests.
 *	A return of DDI_FAILURE indicates the sge was not sent, an errno
 *	can be found in sge->result.
 *	The sge used as an argument should not be re-used or freed until
 *	after a ntb_sync(), or the callback function has been called for it.
 *	At which point it will have status about the individual sge request.
 *
 * int ntb_async_fetch(ntb_handle_t *hdl, ntb_sge_t *sge, ntb_cb_fn_t cb,
 *	    void *arg)
 *	Asynchronously fetch data described by sge from the endpoint
 *	designated in hdl. local in the sge is where the data will be
 *	copied into, remote in the sge is the source location at the peer.
 *	cb(hdl, sge, arg) is called when the sge has been acked, any error
 *	status has been set to indicate success/failure. It can be NULL, then
 *	ntb_sync() needs to be called to wait for completion of any fetch
 *	requests.
 *	A return of DDI_FAILURE indicates the sge was not sent, an errno
 *	can be found in sge->result.
 *	The sge used as an argument should not be re-used or freed until
 *	after a ntb_sync(), at which point it will have status about the
 *	individual sge request.
 *
 * int ntb_sync(ntb_handle_t *hdl)
 *	Waits for all outstanding ntb_async_store() and ntb_async_fetch()
 *	requests to complete.
 *	On success, the previously used sgls will have valid status from
 *	the endpoint.
 *	A return of DDI_FAILURE indicates a failure in synchronising with
 *	the peer. It could be indicative the peer is down. Individual
 *	status is still provided in the sgls from the previous ntb_async_stores.
 *
 * int ntb_store(ntb_handle_t *hdl, ntb_sge_t *sge)
 *	Stores data across the NTB. It will block until the sge has been
 *	completed, failed or timed out.
 *
 * int ntb_fetch(ntb_handle_t *hdl, ntb_sge_t *sge)
 *	Fetch data from across the NTB. It will block until the sge has been
 *	completed, failed or timed out.
 *
 * size_t ntb_transfer_max(ntb_handle_t *hdl)
 *	Return the maximum number of bytes which can be transfered in a
 *	single ntb_sge_t.
 *
 * Endpoint API
 * ============
 * An endpoint driver needs to register with the ntb_transport driver,
 * the registration provides on ops vector used by the transport when
 * it has data for the endpoint:
 *
 *	typedef struct ntb_endpoint_ops {
 *		int	id;
 *		caddr_t	(*remote_to_kva)(void *, ntb_sge_t *);
 *		void	(*remote_flushed)(void *, ntb_sge_t *);
 *		void	(*link_status)(void *, link_status_t);
 *	} ntb_endpoint_ops_t;
 *
 *	id		Is the endpoints pre-allocated endpoint_id.
 *			This is the id used in ntb_alloc_handle() by the
 *			client.
 *	remote_to_kva	Called to convert the remote location in the ntb_sge_t
 *			into a valid KVA which real address of
 *			the data. The first argument is the private argument
 *			given in the call to ntb_register_endpoint.
 *	remote_flushed	Called after the data has been copied into the
 *			address provided by remote_to_kva(). Again, the
 *			first argument is the private argument from
 *			ntb_register_endpoint.
 *	link_status	Called when the transport detects a change in the
 *			status of the connection to the peer. It maybe
 *			called consecutively with the same status as the
 *			status of each ntb_transport device changes.
 *
 * int ntb_register_endpoint(ntb_endpoint_ops_t *ops, void *private)
 *	Registers the endpoint with the ntb_transport. ops is the
 *	ops vector previously described. An endpoint_id can only be registered
 *	once. private is passed as an argument to the ops vector functions.
 *
 * int ntb_unregister_endpoint(ntb_endpoint_ops_t *)
 *	Unregister an endpoint. A previously registered endpoint *must*
 *	call this when it unloads/detaches.
 */

static struct modlinkage modlinkage;

static ntb_node_t	*ntb_node;

/*
 * Lists of bridges indexed by ntb_class_t
 */
static list_t		bridges[NTB_CLASS_CNT];

/*
 * endpoints registered, indexed by their id
 */
static ntb_ep_hdl_t	*ntb_ep[HIGHEST_ENDPOINT_ID];
static krwlock_t	ep_lock;

/*
 * Struct passed to taskq to invoke state change callback function.
 */
typedef struct {
	void		(*fn)(void *, link_status_t);
	void		*fn_arg;
	link_status_t	fn_status;
} status_tq_arg_t;

/*
 * For tracking class level info.
 */
typedef struct {
	int		ep_count;
	int		peer_count;
	int		peer_up_count;
	list_t		ep_registered;
	char		*name;
} class_t;

static class_t classes[NTB_CLASS_CNT] = {
	{ .name = "network" },
	{ .name = "bulk" }
};

typedef struct ring_fns {
	int	(*alloc)(ntb_trans_t *);
	void	(*free)(ntb_trans_t *);
} ring_fns_t;

static int allocate_v1_rings(ntb_trans_t *);
static void free_v1_rings(ntb_trans_t *);
static int allocate_rings(ntb_trans_t *);
static void free_rings(ntb_trans_t *);

static boolean_t completed(dcopy_cmd_t *cmd);

static ring_fns_t ring_funcs[CURRENT_VERSION] = {
	{ allocate_v1_rings, free_v1_rings },
	{ allocate_rings, free_rings }
};

/*
 * This is seeded with gethrtime() at module load and then incremented for
 * each internal message. Seeding it prevents a collision in ids when a
 * peer restarts
 */
static uint64_t		last_id;

/*
 * Cache for sge_req_t
 */
static kmem_cache_t	*req_cache;

static ntb_drv_ops_t ntb_trans_ops;

static void	receive_data_thr(void *);
static void	receive_ack_thr(void *);
static void	free_dma(ntb_trans_t *, dma_info_t *);
static void	send_doorbell(ntb_trans_t *, int);
static int	dma_to_bridge(ntb_trans_t *, uint64_t, size_t);
static int	dcopy_data(dcopy_handle_t, dcopy_cmd_t *, uint64_t, caddr_t,
		    size_t, ddi_dma_cookie_t *, int, boolean_t);
static int	copy_sgls_to_bridge(ntb_trans_t *, sgl_hdr_t *, int, int);
static int	send_data(ntb_trans_t *, ring_entry_t *, dcopy_cmd_t *,
		    boolean_t);
static int	recv_data(ntb_trans_t *, ring_entry_t *, caddr_t, dcopy_cmd_t *,
		    boolean_t);
static void	fail_sge(ntb_trans_t *, ntb_sge_t *, sge_req_t *, int);
static void	fail_handle_sges(ntb_trans_t *, ntb_handle_t *, int);
static void	reap_sgls(ntb_trans_t *);
static void	dequeue_sge_request(sge_req_t *);
static void	remove_sge_request(sge_req_t *);
static void	free_sge_request(sge_req_t *);
static void	send_headers_sent_message(ntb_trans_t *);
static void	cleanup_bridges(void);
static int	init_ntb_trans(ntb_trans_t *);
static void	fini_ntb_trans(ntb_trans_t *);

static int
allocate_dma(ntb_trans_t *ntb, dma_info_t *inf, ddi_dma_attr_t *attr,
    ddi_device_acc_attr_t *acc, int cache_attr, int rw, size_t size,
    boolean_t get_cookies)
{
	dev_info_t	*dip = ntb->node->dip;
	ddi_dma_cookie_t cookie;
	uint_t		cookie_cnt, alloc_flags;
	size_t		len;

	if (ddi_dma_alloc_handle(dip, attr, DDI_DMA_SLEEP, NULL,
	    &inf->dma_hdl) != DDI_SUCCESS) {
		cmn_err(CE_WARN, "!ntb_trans: failed to alloc dma handle");
		return (DDI_FAILURE);
	}

	inf->vaddr = NULL;
	inf->dma_acc_hdl = NULL;

	alloc_flags = DDI_DMA_CONSISTENT | cache_attr;

	if (ntb_dma_mem_alloc(ntb->node->ntb_hdl, inf->dma_hdl, size,
	    acc, alloc_flags, DDI_DMA_SLEEP, NULL, &inf->vaddr, &len,
	    (void **)&inf->dma_acc_hdl) != DDI_SUCCESS) {
		cmn_err(CE_WARN, "!ntb_trans: failed to alloc ntb mem");
		goto failed;
	}

	if (ddi_dma_addr_bind_handle(inf->dma_hdl, NULL, inf->vaddr, len,
	    rw | DDI_DMA_CONSISTENT, DDI_DMA_SLEEP, NULL,
	    &cookie, &cookie_cnt) != DDI_DMA_MAPPED) {
		cmn_err(CE_WARN, "!ntb_trans: failed to bind dma mem");
		goto failed;
	}

	inf->size = size;

	if (!get_cookies) {
		inf->cookies = NULL;
		inf->ccnt = 0;
	} else if (ntb_get_link_cookies(ntb->node->ntb_hdl, inf->dma_hdl,
	    &inf->cookies, &inf->ccnt) != DDI_SUCCESS) {
		cmn_err(CE_WARN, "!ntb_trans: failed to get NTB cookies");
		goto failed;
	}

	return (DDI_SUCCESS);

failed:
	free_dma(ntb, inf);
	return (DDI_FAILURE);
}

static void
free_dma(ntb_trans_t *ntb, dma_info_t *inf)
{
	if (inf->vaddr)
		(void) ddi_dma_unbind_handle(inf->dma_hdl);

	if (inf->dma_acc_hdl) {
		ntb_dma_mem_free(ntb->node->ntb_hdl,
		    (void **)&inf->dma_acc_hdl);
	}

	if (inf->dma_hdl)
		ddi_dma_free_handle(&inf->dma_hdl);

	if (inf->cookies)
		kmem_free(inf->cookies, sizeof (*inf->cookies) * inf->ccnt);

	inf->vaddr = NULL;
	inf->dma_acc_hdl = NULL;
	inf->dma_hdl = NULL;
	inf->cookies = NULL;
}

static int
allocate_read_dma(ntb_trans_t *ntb, dma_info_t *inf, int cache_attr)
{
	ddi_dma_attr_t	attr;
	ddi_device_acc_attr_t acc;
	size_t		len;

	/*
	 * For any dma memory which is to be mapped to a ntb segment, this
	 * call _must_ be used to get the ddi_dma_attr with the correct
	 * attributes and flags
	 */
	if (ntb_get_dma_attr(ntb->node->ntb_hdl, NTB_GET, &attr, &acc, &len) !=
	    DDI_SUCCESS) {
		cmn_err(CE_WARN, "!ntb_trans: failed to get dma attr");
		return (DDI_FAILURE);
	}

	return (allocate_dma(ntb, inf, &attr, &acc, cache_attr, DDI_DMA_READ,
	    len, B_TRUE));
}

/*
 * Changes the ring version.
 * To do this it needs to stop any threads which maybe be currently
 * using the rings, free the old version and allocate the new version.
 */
static int
change_ring_version(ntb_trans_t *ntb, int version)
{
	kthread_t *thr;

	/*
	 * cancel the receive_ack_thr and the receive_data_thr iff this
	 * function is not called by that thread.
	 */
	mutex_enter(&ntb->rupt_lock);
	ntb->state |= STATE_CANCEL;
	cv_signal(&ntb->ack_cv);

	/*
	 * We need to release the ring_lock to allow the data and ack
	 * threads to terminate.
	 */
	mutex_exit(&ntb->ring_lock);

	if (curthread->t_did != ntb->data_thr) {
		cv_signal(&ntb->data_cv);
		mutex_exit(&ntb->rupt_lock);
		thread_join(ntb->data_thr);
	} else {
		mutex_exit(&ntb->rupt_lock);
	}

	thread_join(ntb->ack_thr);

	mutex_enter(&ntb->ring_lock);

	ring_funcs[ntb->version - 1].free(ntb);
	if (ring_funcs[version - 1].alloc(ntb) != DDI_SUCCESS) {
		cmn_err(CE_WARN, "!Failed to allocate DMA for ring version %d."
		    " Transport will be non-functional", version);
		return (DDI_FAILURE);
	}

	ntb->version = version;

	ntb->state &= ~STATE_CANCEL;

	if (curthread->t_did != ntb->data_thr) {
		thr = thread_create(NULL, 0, receive_data_thr, ntb, 0, &p0,
		    TS_RUN, maxclsyspri - ntb->class);
		ntb->data_thr = thr->t_did;
	}

	thr = thread_create(NULL, 0, receive_ack_thr, ntb, 0, &p0, TS_RUN,
	    maxclsyspri - ntb->class);
	ntb->ack_thr = thr->t_did;

	return (DDI_SUCCESS);
}

/*
 * Called either when we detect the peer node goes down and when
 * we receive a message from the peer telling us to reset the rings.
 * The peer will send the message when the driver is loaded, this
 * mechanism ensures the the ring variables stay in sync across driver
 * loads and unloads
 */
static void
reset_rings(ntb_trans_t *ntb, int version, boolean_t peer_alive)
{
	ntb_handle_t	*hdl;

	ASSERT(mutex_owned(&ntb->xmit_lock));
	ASSERT(mutex_owned(&ntb->ring_lock));

	/*
	 * Execution through this function must be serialised.
	 */
	while (ntb->in_reset && !(ntb->state & STATE_ENDING)) {
		mutex_exit(&ntb->ring_lock);
		cv_wait(&ntb->reset_cv, &ntb->xmit_lock);
		mutex_enter(&ntb->ring_lock);
	}

	if (ntb->state & STATE_ENDING)
		return;

	ntb->in_reset = B_TRUE;
	ntb->state |= STATE_RESETING;
	/*
	 * increment the session id so we can identify any stale messages
	 */
	ntb->out.send->s_id++;

	/*
	 * If anything is blocked waiting for ring space, give it a poke
	 * and they will fail
	 */
	if (ntb->waiting)
		cv_broadcast(&ntb->ring_cv);

	for (hdl = list_head(&ntb->hdl_list); hdl;
	    hdl = list_next(&ntb->hdl_list, hdl)) {
		/*
		 * Fail and wake up any pending completions.
		 */
		fail_handle_sges(ntb, hdl, EIO);
	}

	/*
	 * Give the post_thr a kick so it will terminate anything in its
	 * queue
	 */
	cv_signal(&ntb->post_cv);

	/*
	 * Now wait for everything to terminate before restoring link
	 * state.
	 *
	 * ntb->pending and ntb->waiting are protected by ring_lock,
	 * so that must be the lock used in the cv_wait. After cv_wait
	 * the ring_lock is acquired out of order, but because everything
	 * is now quiesced it is safe to re-acquire in the correct order.
	 */
	mutex_exit(&ntb->xmit_lock);
	while (ntb->pending > 0 || ntb->waiting > 0)
		cv_wait(&ntb->ring_cv, &ntb->ring_lock);

	/*
	 * Now all ntb activity is quiesced, and any attempts to start
	 * any new transactions will fail.
	 */

	mutex_exit(&ntb->ring_lock);
	mutex_enter(&ntb->xmit_lock);
	mutex_enter(&ntb->ring_lock);

	/*
	 * Each peer sends its CURRENT_VERSION, that way both peers
	 * will end up using the highest common ring version.
	 */
	if (version != ntb->version && version <= CURRENT_VERSION) {
		/*
		 * The ring version is different to the one active and
		 * it is a supported version.
		 * Re-allocate our rings to the same version.
		 */
		if (change_ring_version(ntb, version) != DDI_SUCCESS)
			/* Failed. Leave the ring down. */
			goto out;
	}

	ntb->out.send->sgl_ring.cons = 0;
	ntb->out.send->sgl_ring.prod = 0;
	ntb->out.send->sgl_ring.next_prod = 0;

	ntb->out.recv->sgl_ring.cons = 0;
	ntb->out.recv->sgl_ring.prod = 0;

	ntb->out.send->data_ring.cons = 0;
	ntb->out.send->data_ring.prod = 0;
	ntb->out.recv->data_ring.cons = 0;
	ntb->out.recv->data_ring.prod = 0;

	ntb->last_cons = 0;
	ntb->last_prod = 0;

	bzero(ntb->reap, sizeof (ring_entry_t *) * ntb->caller_sz);

	/*
	 * Send the reset headers to the peer
	 */
	if (peer_alive) {
		(void) dma_to_bridge(ntb, ntb->out.recv_off,
		    ntb->out.header_sz);

		(void) dma_to_bridge(ntb, ntb->out.send_off,
		    ntb->out.header_sz);

		send_doorbell(ntb, ntb->node->ack);

		send_headers_sent_message(ntb);
	} else {
		ntb->state &= ~STATE_RESETING;
	}

out:
	ntb->in_reset = B_FALSE;
	cv_broadcast(&ntb->reset_cv);
}

/*
 * For now sends the single word in "msg" to the peer.
 * It could be extended to send a buffer too, if required
 */
static void
send_message(ntb_trans_t *ntb, internal_msg_t msg_id)
{
	uint64_t	off;
	int		rv;

	/* offset of the specific message to be sent */
	off = offsetof(dma_ring_t, msg[msg_id]);

	/*
	 * First send the message
	 */
	rv = dma_to_bridge(ntb, ntb->out.send_off + off, sizeof (dma_msg_t));

	if (rv != DDI_SUCCESS)
		return;

	/*
	 * Send the msg id after the message, then we know
	 * when the msg_id has changed in the peer, the peer will know
	 * any messages received are valid.
	 */
	ntb->out.send->msg_id = ntb->out.send->msg[msg_id].msg_id;

	off = offsetof(dma_ring_t, msg_id);

	rv = dma_to_bridge(ntb, ntb->out.send_off + off, sizeof (uint64_t));

	if (rv != DDI_FAILURE) {
		send_doorbell(ntb, ntb->node->data);
	}
}

static uint64_t
get_next_msg_id(void)
{
	return (atomic_inc_64_nv(&last_id));
}

/*
 * ping - I'm alive
 */
static void
send_alive_message(ntb_trans_t *ntb)
{
	ntb->out.send->msg[I_AM_ALIVE].msg_id = get_next_msg_id();
	ntb->out.send->msg[I_AM_ALIVE].msg = CURRENT_VERSION;

	send_message(ntb, I_AM_ALIVE);
}

/*
 * Sent when the device is being detached, gives the peer a chance to
 * clean up and abort any active transactions.
 */
static void
send_dying_message(ntb_trans_t *ntb)
{
	ntb->out.send->msg[I_AM_DYING].msg_id = get_next_msg_id();
	ntb->out.send->msg[I_AM_DYING].msg = 1;	/* anything non-zero */

	send_message(ntb, I_AM_DYING);
}

/*
 * Tell the peer to reset its rings
 */
static void
send_reset_message(ntb_trans_t *ntb)
{
	ntb->out.send->msg[RESET_RINGS].msg_id = get_next_msg_id();
	ntb->out.send->msg[RESET_RINGS].msg = CURRENT_VERSION;

	send_message(ntb, RESET_RINGS);
}

/*
 * A "headers sent" is sent after the rings have been reset.
 * The peer will respond with a "headers ack" at which point both sides
 * rings are in sync.
 */
static void
send_headers_sent_message(ntb_trans_t *ntb)
{
	ntb->out.send->msg[HEADERS_SENT].msg_id = get_next_msg_id();
	ntb->out.send->msg[HEADERS_SENT].msg = 1; /* anything non-zero */

	send_message(ntb, HEADERS_SENT);
}

static void
send_headers_ack_message(ntb_trans_t *ntb)
{
	ntb->out.send->msg[HEADERS_ACK].msg_id = get_next_msg_id();
	ntb->out.send->msg[HEADERS_ACK].msg = 1; /* anything non-zero */

	send_message(ntb, HEADERS_ACK);
}

/*
 * Called to handle any inter-driver messages.
 */
static void
process_msg(ntb_trans_t *ntb)
{
	dma_msg_t	msg[MSG_CNT];
	int		i;

	/*
	 * If the message id hasn't changed, there are no new messages
	 */
	if (ntb->last_msg_id == ntb->in.send->msg_id)
		return;

	ntb->last_msg_id = ntb->in.send->msg_id;

	bcopy(ntb->in.send->msg, msg, sizeof (msg));

	for (i = 0; i < MSG_CNT; i++) {
		if (msg[i].msg == 0)
			/* no message */
			continue;

		if (msg[i].msg_id == ntb->last_msg[i].msg_id)
			/* already handled */
			continue;

		DTRACE_PROBE2(recv_msg, uint64_t, msg[i].msg_id, int,
		    msg[i].msg);

		ntb->last_msg[i] = msg[i];

		ntb->msg_fn[i](ntb, &msg[i]);
	}
}

static void
run_link_status(void *arg)
{
	status_tq_arg_t	*tq_arg = arg;

	tq_arg->fn(tq_arg->fn_arg, tq_arg->fn_status);

	kmem_free(tq_arg, sizeof (status_tq_arg_t));
}

static void
schedule_link_status(ntb_trans_t *ntb, void (*fn)(void *, link_status_t),
    void *fn_arg, link_status_t fn_status)
{
	status_tq_arg_t	*tq_arg;

	tq_arg = kmem_alloc(sizeof (status_tq_arg_t), KM_SLEEP);

	tq_arg->fn = fn;
	tq_arg->fn_arg = fn_arg;
	tq_arg->fn_status = fn_status;

	(void) ddi_taskq_dispatch(ntb->status_tq, run_link_status, tq_arg,
	    DDI_SLEEP);
}

static void
run_link_status_routines(link_status_t status, ntb_trans_t *ntb)
{
	ntb_ep_hdl_t	*ep_hdl;
	class_t		*cp = &classes[ntb->class];
#if 0
	nvlist_t	*erp;
	char		event_modname[64];
#endif

	rw_enter(&ep_lock, RW_WRITER);
	cp->peer_up_count += status == TRANSPORT_CONNECTED ? 1 : -1;

	ASSERT(cp->peer_up_count >= 0);

	cmn_err(CE_NOTE, "!ntb_trans(%d): peer is %s", ntb->id,
	    status == TRANSPORT_CONNECTED ? "up" : "down");

	if (status == TRANSPORT_CONNECTED && cp->peer_up_count <
	    cp->peer_count) {
		rw_exit(&ep_lock);
		return;
	}

	rw_downgrade(&ep_lock);

	for (ep_hdl = list_head(&cp->ep_registered); ep_hdl;
	    ep_hdl = list_next(&cp->ep_registered, ep_hdl)) {
		if (!ep_hdl->ops->link_status)
			continue;

		schedule_link_status(ntb, ep_hdl->ops->link_status,
		    ep_hdl->user_arg, status);
	}

	rw_exit(&ep_lock);
#if 0
	(void) snprintf(event_modname, sizeof (event_modname), "%s.%s",
	    ZEBI_EV_MOD_NTB, ddi_driver_name(ntb->node->dip));

	erp = zebi_sys_monitor_ereport_create(event_modname,
	    status == TRANSPORT_CONNECTED ? ZEBI_EV_LINK_UP : ZEBI_EV_LINK_DOWN,
	    B_FALSE, 0);

	if (erp != NULL) {
		(void) nvlist_add_string(erp, "traffic-class",
		    ntb->class == NTB_NETWORK ? "network" : "bulk");

		zebi_sys_monitor_ereport_post(erp, B_FALSE);
	}
#endif
}

/*
 * Doorbell handler invoked when the ntb driver detects a state change
 * in the connection with the peer. i.e. a transition from up->down or
 * down->up
 */
/* ARGSUSED */
static void
state_change(void *arg, int doorbell)
{
	ntb_trans_t	*ntb = arg;
	ntb_node_t	*node = ntb->node;
	boolean_t	down = B_FALSE;
	boolean_t	up = B_FALSE;

	/*
	 * We don't want this to be updated whilst we're reseting the
	 * rings
	 */
	mutex_enter(&ntb->xmit_lock);

	if (doorbell == node->up && !(ntb->state & STATE_NTB_UP)) {
		mutex_enter(&ntb->ring_lock);
		ntb->state |= STATE_NTB_UP;
		up = (ntb->state & STATE_UP) == STATE_UP;
		mutex_exit(&ntb->ring_lock);
	} else if (doorbell == node->down && (ntb->state & STATE_NTB_UP)) {
		down = (ntb->state & STATE_DRIVER_UP) != 0;

		mutex_enter(&ntb->ring_lock);

		ntb->state &= ~(STATE_NTB_UP | STATE_DRIVER_UP |
		    STATE_RESETING);

		reset_rings(ntb, CURRENT_VERSION, B_FALSE);

		mutex_exit(&ntb->ring_lock);

		ntb->alive_sending = B_TRUE;
		cv_signal(&ntb->alive_cv);
	}

	mutex_exit(&ntb->xmit_lock);

	if (up)
		run_link_status_routines(TRANSPORT_CONNECTED, ntb);
	else if (down)
		run_link_status_routines(TRANSPORT_DISCONNECTED, ntb);
}

/* ARGSUSED */
static void
handle_reset_msg(ntb_trans_t *ntb, dma_msg_t *msg)
{
	boolean_t	up;

	if (msg->msg <= 0 || msg->msg > CURRENT_VERSION)
		return;

	mutex_enter(&ntb->xmit_lock);
	mutex_enter(&ntb->ring_lock);

	reset_rings(ntb, msg->msg, B_TRUE);

	if (ntb->alive_sending)
		send_reset_message(ntb);

	ntb->state |= STATE_DRIVER_UP;
	ntb->alive_sending = B_FALSE;

	up = (ntb->state & STATE_UP) == STATE_UP;

	mutex_exit(&ntb->ring_lock);
	mutex_exit(&ntb->xmit_lock);

	if (up)
		run_link_status_routines(TRANSPORT_CONNECTED, ntb);
}

/*
 * Tell the alive_thread that the peer is alive.
 * It will let messages flow over the transport
 */
/* ARGSUSED */
static void
handle_alive_msg(ntb_trans_t *ntb, dma_msg_t *msg)
{
	if (msg->msg <= 0 || msg->msg > CURRENT_VERSION)
		return;

	mutex_enter(&ntb->xmit_lock);
	mutex_enter(&ntb->ring_lock);

	/*
	 * We received an alive message from the peer.
	 * This implies the peer is not in a normal state. If we're here
	 * then we both need to reset the rings.
	 *
	 * As originally written an alive message was only responded to if
	 * this node was in an alive_sending state as well. This seems to
	 * be incorrect in some circumstances where the peer NTB port is reset.
	 * IF311-4514
	 */
	ntb->alive_sending = B_FALSE;
	reset_rings(ntb, msg->msg, B_TRUE);
	send_reset_message(ntb);

	mutex_exit(&ntb->ring_lock);
	mutex_exit(&ntb->xmit_lock);
}

/*
 * Received when the peer is doing a clean detach.
 * It will reset the rings, cause any pending messages to fail, and
 * prevent further messages from being sent.
 * It will also wake up the alive_thread which will start sending
 * I'm alive messages
 */
/* ARGSUSED */
static void
handle_dying_msg(ntb_trans_t *ntb, dma_msg_t *msg)
{
	boolean_t	down;

	mutex_enter(&ntb->xmit_lock);

	mutex_enter(&ntb->ring_lock);
	down = (ntb->state & STATE_DRIVER_UP) != 0;
	ntb->alive_sending = B_TRUE;
	ntb->state &= ~(STATE_DRIVER_UP | STATE_RESETING);

	reset_rings(ntb, CURRENT_VERSION, B_FALSE);

	mutex_exit(&ntb->ring_lock);

	cv_signal(&ntb->alive_cv);

	mutex_exit(&ntb->xmit_lock);

	if (down)
		run_link_status_routines(TRANSPORT_DISCONNECTED, ntb);
}

/*
 * When we get the "headers sent" message all we have to do is send the
 * ack, as we know we have received sgl headers with the rings reset
 */
/* ARGSUSED */
static void
handle_headers_sent_msg(ntb_trans_t *ntb, dma_msg_t *msg)
{
	send_headers_ack_message(ntb);
}

/*
 * When we get the "headers ack" mark the rings as reset
 */
/* ARGSUSED */
static void
handle_headers_ack_msg(ntb_trans_t *ntb, dma_msg_t *msg)
{
	mutex_enter(&ntb->ring_lock);
	ntb->state &= ~STATE_RESETING;
	mutex_exit(&ntb->ring_lock);
}

/*
 * "I'm alive thread"
 * I'm alive messages are sent when:
 *	- the driver is first loaded.
 *	- the peer driver sends I'm dying message.
 *	- NTB bridge going down - probably the peer has been shutdown..
 *
 * I'm alive are no longer sent when it has been determined the peer is
 * up. The criteria are:
 *	1. Received an I'm alive message from the peer. A "reset rings"
 *	   message is then sent.
 *	2. Received a "reset rings" from the peer. At which point normal
 *	   data traffic will commence.
 *
 * Any attempts to send data traffic before the reset rings will fail with
 * EIO
 */
static void
alive_thr(void *arg)
{
	ntb_trans_t	*ntb = arg;

	mutex_enter(&ntb->xmit_lock);

	ntb->alive_sending = B_TRUE;
	while (/* CONSTCOND */ 1) {
		if (ntb->alive_sending)
			send_alive_message(ntb);

		(void) cv_reltimedwait(&ntb->alive_cv, &ntb->xmit_lock,
		    drv_usectohz(ALIVE_INTERVAL_US), TR_CLOCK_TICK);

		mutex_enter(&ntb->ring_lock);
		if (ntb->state & STATE_ENDING) {
			mutex_exit(&ntb->ring_lock);
			break;
		}
		mutex_exit(&ntb->ring_lock);
	}

	mutex_exit(&ntb->xmit_lock);
}

/*
 * The following three functions are used for moving data between
 * the NTB and kernel using dcopy and will be used when dcopy devices
 * have been detected and dcopy has not been explicitly disabled.
 * dcopy will operate on blocks greater than the dcopy-threshold
 * attribute.
 */

/*
 * If dcopy is enabled for all transfers use dcopy to copy
 * from the DMA memory into a KVA.
 */
static int
data_to_kernel(ntb_trans_t *ntb, dcopy_cmd_t *cmd, uint64_t dma_offset,
    caddr_t dst, size_t size, boolean_t use_dma, boolean_t last)
{
	if (use_dma) {
		return (dcopy_data(ntb->k_dcopy_hdl, cmd, dma_offset, dst,
		    size, ntb->rd_infop->cookies, DDI_DMA_SYNC_FORKERNEL,
		    last));
	} else {
		bcopy(ntb->rd_infop->vaddr + dma_offset, dst, size);
		return (DDI_SUCCESS);
	}
}

/*
 * If dcopy is enabled and block size is above threshold use dcopy to copy
 * from a KVA directly into the NTB's PCI memory.
 * Dcopy uses the cookies (w_info.cookies) of the PCI memory to directly
 * access the physical memory.
 *
 * The data will then be sent across the NTB.
 */
static int
data_to_bridge(ntb_trans_t *ntb, data_hdr_t *data_ring, dcopy_cmd_t *cmd,
    uint64_t dma_offset, caddr_t src, size_t size, boolean_t use_dma,
    boolean_t last)
{

	if (use_dma) {
		ntb_kstat_accumulate_put(ntb->node->ntb_kdata, size);
		return (dcopy_data(ntb->b_dcopy_hdl, cmd,
		    data_ring->head_off + dma_offset, src, size,
		    ntb->wr_infop->cookies, DDI_DMA_SYNC_FORDEV, last));
	} else {
		/*
		 * Copy the data into dma memory
		 */
		bcopy(src, data_ring->head + dma_offset, size);

		/*
		 * Now move it across the NTB
		 */
		return (ntb_put(ntb->node->ntb_hdl, ntb->wr_infop->dma_hdl,
		    data_ring->head_off + dma_offset, size));
	}
}

/*
 * Use ntb services to copy information (Eg sgls and ring info) from the
 * memory mapped DMA into the NTB's PCI memory.
 * The data will then be sent across the NTB.
 */
static int
dma_to_bridge(ntb_trans_t *ntb, uint64_t dma_offset, size_t size)
{
	return (ntb_put(ntb->node->ntb_hdl, ntb->w_info.dma_hdl, dma_offset,
	    size));
}

static void
send_doorbell(ntb_trans_t *ntb, int db)
{
	(void) ntb_send_doorbell(ntb->node->ntb_hdl, db);
}

/*
 * Copies data between dma (addresses in cookies) at dma_offset and
 * virtual address vaddr. dir indicates direction
 */
static int
dcopy_data(dcopy_handle_t dc_hdl, dcopy_cmd_t *cmdp, uint64_t dma_offset,
    caddr_t vaddr, size_t size, ddi_dma_cookie_t *cookies, int dir,
    boolean_t do_intr)
{
	dcopy_cmd_t	prevcmd = *cmdp;
	dcopy_cmd_t	cmd = *cmdp;
	ddi_dma_cookie_t *cp;
	size_t		c_max, pg_max, to_copy;
	offset_t	c_offset, pg_offset;
	uint64_t	pg_addr, c_addr;
	int		rv, idx, flags;

	flags = DCOPY_SLEEP;
	if (prevcmd)
		flags |= DCOPY_ALLOC_LINK;

	/*
	 * split the copies up so they don't cross segment or page boundaries.
	 */
	while (size > 0) {
		/*
		 * All cookies provided by the ntb driver are guaranteed
		 * to be the same size, this enables a quick lookup to the
		 * correct cookie for the dma_offset
		 */
		idx = dma_offset / cookies->dmac_size;
		cp = &cookies[idx];
		ASSERT(cp->dmac_size == cookies->dmac_size);

		c_offset = dma_offset % cp->dmac_size;
		c_addr = cp->dmac_laddress + c_offset;
		c_max = c_offset + size > cp->dmac_size ?
		    cp->dmac_size - c_offset : size;

		pg_offset = (uint64_t)(uintptr_t)vaddr & PAGEOFFSET;
		pg_addr = (uint64_t)ptob(kvtoppid(vaddr)) + pg_offset;
		pg_max = pg_offset + size > PAGESIZE ?
		    PAGESIZE - pg_offset : size;

		to_copy = MIN(c_max, pg_max);

		if (dcopy_cmd_alloc(dc_hdl, flags, &cmd) != DCOPY_SUCCESS) {
			if (prevcmd)
				dcopy_cmd_free(&prevcmd);
			*cmdp = NULL;
			return (DDI_FAILURE);
		}

		flags |= DCOPY_ALLOC_LINK;
		prevcmd = cmd;

		cmd->dp_cmd = DCOPY_CMD_COPY;

		/*
		 * Each cmd is within a single physical page, so dcopy()
		 * doesn't need to check this.
		 */
		//cmd->dp_flags = DCOPY_CMD_CONTIG_MEM;
		if (to_copy == size && do_intr)
			cmd->dp_flags |= DCOPY_CMD_INTR;

		cmd->dp.copy.cc_size = to_copy;
		if (dir == DDI_DMA_SYNC_FORDEV) {
			cmd->dp.copy.cc_source = pg_addr;
			cmd->dp.copy.cc_dest = c_addr;
		} else {
			cmd->dp.copy.cc_source = c_addr;
			cmd->dp.copy.cc_dest = pg_addr;
		}

		DTRACE_PROBE3(dcopy_addrs, uint64_t, cmd->dp.copy.cc_source,
		    uint64_t, cmd->dp.copy.cc_dest, size_t, to_copy);

		do {
			rv = dcopy_cmd_post(cmd);
			/*
			 * This will happen when the ioat ring is full.
			 * It is rare and very transient.
			 */
		} while (rv == DCOPY_NORESOURCES);

		if (rv != DCOPY_SUCCESS) {
			dcopy_cmd_free(&cmd);
			*cmdp = NULL;
			return (DDI_FAILURE);
		}

		dma_offset += to_copy;
		vaddr += to_copy;
		size -= to_copy;
	}

	*cmdp = cmd;
	return (DDI_SUCCESS);
}

static int
sync_sgls_forkernel(ntb_trans_t *ntb, ddi_dma_handle_t hdl, sgl_hdr_t *sgl_ring,
    int idx, uint64_t count)
{
	if ((idx + count) > sgl_ring->size) {
		/*
		 * The number wraps the sgl ring, so do two "copies"
		 */
		if (ntb_get(ntb->node->ntb_hdl, hdl, sgl_ring->head_off +
		    idx * sizeof (ring_entry_t),
		    (sgl_ring->size - idx) * sizeof (ring_entry_t)) !=
		    DDI_SUCCESS)
			return (DDI_FAILURE);

		count -= sgl_ring->size - idx;
		idx = 0;
	}

	return (ntb_get(ntb->node->ntb_hdl, hdl, sgl_ring->head_off +
	    idx * sizeof (ring_entry_t), count * sizeof (ring_entry_t)));
}

/*
 * Doorbell callback invoked when the peer wants to "ack" receipt and
 * processing of data. Simply wakes up the main handler thread.
 */
/* ARGSUSED */
static void
receive_ack(void *arg, int doorbell)
{
	ntb_trans_t	*ntb = arg;

	mutex_enter(&ntb->rupt_lock);
	ntb->ack_pending = B_TRUE;
	cv_signal(&ntb->ack_cv);
	mutex_exit(&ntb->rupt_lock);
}

static inline void
invoke_completion_fn(sge_req_t *req, int err)
{
	boolean_t	already_notified;

	/*
	 * Check whether the sge owner has been notified already.
	 */
	already_notified = (boolean_t)atomic_swap_uint(
	    (uint_t *)&req->notified, (uint_t)B_TRUE);

	DTRACE_PROBE2(notify, sge_req_t *, req, boolean_t, already_notified);

	if (!already_notified) {
		req->sge->result = err;

		if (req->complete_fn != NULL)
			req->complete_fn(req->hdl, req->sge, req->complete_arg);
	}
}

/*
 * ntb->ack_req holds a list of sge_req_t which need to have their completion
 * status posted back to the originating driver. Post back status but make
 * sure any NTB_FETCH's have completed.
 */
static void
run_completion_fns(ntb_trans_t *ntb, ring_entry_t *sgl_head)
{
	ring_entry_t	*sgl;
	sge_req_t	*req, *next;

	for (req = list_head(&ntb->ack_req); req; req = next) {
		next = list_next(&ntb->ack_req, req);

		ASSERT(req->ref > 0);

		sgl = &sgl_head[req->idx];

		if (sgl->dir == NTB_FETCH && sgl->err == 0) {
			if (!completed(&req->k_cmd))
				sgl->err = EFAULT;
		}

		list_remove(&ntb->ack_req, req);

		invoke_completion_fn(req, sgl->err);

		free_sge_request(req);
	}

	ASSERT(list_is_empty(&ntb->ack_req));
}

/*
 * The function flow involved in handling a request across the ntb is:
 *
 *         PEER A				    PEER B
 *
 *   ntb_async_store()
 *	     +------------ sgl and data  ------>  process_data()
 *	  returns				     +--> remote_to_kva()
 *						     +--> remote_flushed()
 *      process_ack()  <-- sgl with result ----------+
 *           |
 *           v
 *   completion callback
 *
 *   ntb_async_fetch()
 *	     +------------ sgl ---------------->  process_data()
 *	  returns				     +--> remote_to_kva()
 *                                                   |
 *      process_ack()  <-- sgl with result and data -+
 *           |
 *           v
 *   completion callback
 *
 * Handles an incoming ack.
 * Looks at which sgls have not been processed, feeds back completion
 * status into the callers originating sgl. If the caller is waiting
 * for notification, wake them when all their pending sgls are complete.
 */
static boolean_t
process_ack(ntb_trans_t *ntb)
{
	sgl_hdr_t	*o_sgl_ring = &ntb->out.send->sgl_ring;
	data_hdr_t	*s_data_ring = &ntb->out.send->data_ring;
	data_hdr_t	*r_data_ring = &ntb->out.recv->data_ring;
	sgl_hdr_t	*r_sgl_ring = &ntb->in.recv->sgl_ring;
	sge_req_t	*req, *next;
	ring_entry_t	*sgl, *sgl_head;
	uint64_t	recv_cons, last_cons, saved_last_cons;
	uint64_t	count, processed_sgls;
	int		fetch_end, store_end;
	int		i, rv;

	mutex_enter(&ntb->ring_lock);
	/* Take a snapshot of the values we want out of the dma buf */
	recv_cons = r_sgl_ring->cons;

	DTRACE_PROBE2(ack_in, uint64_t, recv_cons, uint64_t, ntb->last_cons);

	if (peer_down(ntb)) {
		mutex_exit(&ntb->ring_lock);
		return (B_FALSE);
	}

	if (ntb->in.recv->s_id != ntb->out.send->s_id) {
		mutex_exit(&ntb->ring_lock);
		/* Stale message - ignore it */
		return (B_FALSE);
	}

	if (o_sgl_ring->cons >= recv_cons) {
		/*
		 * The consumer of the out ring has progressed passed
		 * that processed by the peer. This is indicative sgls
		 * have been failed locally. So, ignore
		 */
		DTRACE_PROBE3(over, uint64_t, ntb->last_cons, uint64_t,
		    o_sgl_ring->cons, uint64_t, recv_cons);

		mutex_exit(&ntb->ring_lock);
		return (B_FALSE);
	}

	if (recv_cons <= ntb->last_cons) {
		mutex_exit(&ntb->ring_lock);
		/*
		 * These have already been processed
		 */
		DTRACE_PROBE(no_ack);
		return (B_FALSE);
	}

	/*
	 * Some sanity checks on the incoming data - both sides must
	 * have symmetric DMA setups
	 */
	if (r_sgl_ring->size != o_sgl_ring->size) {
		mutex_exit(&ntb->ring_lock);
		return (B_FALSE);
	}

	/*
	 * This will make a ring reset wait until we have processed this
	 * batch of acks
	 */
	ntb->pending++;

	/*
	 * This contains an un-processed ack from the peer
	 */
	saved_last_cons = ntb->last_cons;
	ntb->last_cons = recv_cons;
	last_cons = o_sgl_ring->cons;
	mutex_exit(&ntb->ring_lock);

	processed_sgls = recv_cons - last_cons;


	i = last_cons % o_sgl_ring->size;
	if (sync_sgls_forkernel(ntb, ntb->r_info.dma_hdl, r_sgl_ring, i,
	    processed_sgls) != DDI_SUCCESS) {
		ntb->last_cons = saved_last_cons;
		return (B_FALSE);
	}

	/*
	 * Now place the completion/error status back into the
	 * originators private ntb sge
	 */
	sgl_head = (ring_entry_t *)(uintptr_t)(ntb->r_info.vaddr +
	    r_sgl_ring->head_off);

	ASSERT(list_is_empty(&ntb->ack_req));

	/*
	 * Pull all the requests to be ack'ed out of the callers array.
	 * We can then process them without holding the ring_lock.
	 */
	mutex_enter(&ntb->ring_lock);
	for (count = processed_sgls; count; count--) {
		if (ntb->callers[i]) {
			dequeue_sge_request(ntb->callers[i]);
			list_insert_tail(&ntb->ack_req, ntb->callers[i]);

			ntb->callers[i] = NULL;
		}

		ntb->reap[i] = NULL;	/* reaped ... */

		if (++i == ntb->caller_sz)
			i = 0;
	}
	mutex_exit(&ntb->ring_lock);

	/*
	 * Iterate through the list of requests to be ack'ed.
	 * First, initiate asynchronous copies of data for NTB_FETCH's
	 * from the bridge into kernel
	 */
	fetch_end = store_end = -1;
	for (req = list_head(&ntb->ack_req); req; req = next) {
		next = list_next(&ntb->ack_req, req);

		ASSERT(req->ref > 0);

		sgl = &sgl_head[req->idx];

		if (sgl->dir == NTB_STORE) {
			store_end = req->idx;
			continue;
		}

		fetch_end = req->idx;

		if (sgl->err != 0)
			continue;

		rv = recv_data(ntb, sgl, sgl->src, &req->k_cmd, B_TRUE);

		sgl->err = rv == DDI_SUCCESS ? 0 : EFAULT;
	}

	/*
	 * Now send back completion status
	 */
	run_completion_fns(ntb, sgl_head);

	mutex_enter(&ntb->ring_lock);

	/*
	 * Free up the sgl entries which have been processed by the peer
	 */
	o_sgl_ring->cons = recv_cons;

	/*
	 * Release space in the data rings
	 */
	if (store_end >= 0) {
		/* Index of the last consumed sgl for NTB_STORE */
		sgl = &o_sgl_ring->head[store_end];
		s_data_ring->cons = sgl->start + sgl->size;
	}

	if (fetch_end >= 0) {
		/* Index of the last consumed sgl for NTB_FETCH */
		sgl = &o_sgl_ring->head[fetch_end];
		r_data_ring->cons = sgl->start + sgl->size;
	}

	/* clean up any holes in the ring from prior failures */
	reap_sgls(ntb);

	/*
	 * Wake up anything that is blocked because the rings
	 * are full or if the rings are being reset, let the reset
	 * complete.
	 */
	ntb->pending--;
	if (ntb->waiting || (ntb->state & STATE_RESETING))
		cv_broadcast(&ntb->ring_cv);

	mutex_exit(&ntb->ring_lock);

	return (B_TRUE);
}

/*
 * Thread for processing acks. Once it gets woken by a doorbell, it will
 * try and process continuously whilst there are unprocessed acks in the
 * ring
 */
static void
receive_ack_thr(void *arg)
{
	ntb_trans_t	*ntb = arg;
	ntb_node_t	*node = ntb->node;
	sgl_hdr_t	*sgl_ring = &ntb->in.recv->sgl_ring;

	//lgrp_move_curthread(get_numa_lgrp_prop(node->dip));

	for (;;) {
		mutex_enter(&ntb->rupt_lock);
		while (!ntb->ack_pending && !(ntb->state & STATE_CANCEL)) {
			cv_wait(&ntb->ack_cv, &ntb->rupt_lock);
		}
		ntb->ack_pending = B_FALSE;
		mutex_exit(&ntb->rupt_lock);

		if (ntb->state & STATE_CANCEL)
			return;

		/*
		 * Get the peer's header so we can determine what has changed
		 */
		if (ntb_get(node->ntb_hdl, ntb->r_info.dma_hdl,
		    ntb->in.recv_off, sizeof (dma_ring_t)) != DDI_SUCCESS)
			continue;

		do {
			if (!process_ack(ntb))
				break;

			if (ntb_get(node->ntb_hdl, ntb->r_info.dma_hdl,
			    ntb->in.recv_off, sizeof (dma_ring_t)) !=
			    DDI_SUCCESS)
				break;

		} while (sgl_ring->cons > ntb->last_cons);
	}
}

/*
 * Copy data out of the NTB into kernel memory.
 * Data is taken out of the ring buffer at offset fr_offset and copied
 * to kernel address at dst.
 */
static int
copy_data_to_kernel(ntb_trans_t *ntb, data_hdr_t *data_ring, dcopy_cmd_t *cmd,
    uint64_t fr_offset, caddr_t dst, size_t to_copy, boolean_t use_dma,
    boolean_t last)
{
	/*
	 * This copies it into the ring buffer (maybe a no-op depending
	 * on which specific NTB hardware we are using)
	 */
	if (ntb_get(ntb->node->ntb_hdl, ntb->rd_infop->dma_hdl,
	    data_ring->head_off + fr_offset, to_copy) != DDI_SUCCESS)
		return (DDI_FAILURE);

	return (data_to_kernel(ntb, cmd, data_ring->head_off + fr_offset,
	    dst, to_copy, use_dma, last));
}

/*
 * When dcopy was used for the transaction  wait for a set of dcopy
 * commands to complete. If dcopy was not used, there is nothing to
 * wait for. dcopy will set a dcopy_cmd if used. If not the cmd pointer
 * is null.
 * Return B_TRUE if all completed, B_FALSE otherwise
 */
static boolean_t
completed(dcopy_cmd_t *cmd)
{
	int	rv;

	if (*cmd == NULL)
		/* not a dcopy transaction so nothing to wait for */
		return (B_TRUE);

	rv = dcopy_cmd_poll(*cmd, DCOPY_POLL_BLOCK);
	ASSERT(rv != DCOPY_PENDING);

	dcopy_cmd_free(cmd);

	return (rv == DCOPY_COMPLETED);
}

/*
 * Copies the data out of the ntb into the the "end-point" buffer.
 * Calls the remote_to_kva end-point ops function which the end-point
 * provides to convert the "remote" field in the sgl to a valid KVA
 */
static int
copy_sgl_data_to_kernel(ntb_trans_t *ntb, ring_entry_t *sgl, ntb_sge_t *sge,
    dcopy_cmd_t *cmd, ntb_endpoint_ops_t *ops, void *ep_arg, boolean_t last)
{
	caddr_t		kva;
	int		rv;

	/*
	 * convert remote in the sgl to a kernel virtual address
	 */
	kva = ops->remote_to_kva(ep_arg, sge);
	if (kva == NULL)
		return (DDI_FAILURE);

	/*
	 * Now copy the data out of the data ring into the address
	 * provided by the endpoint
	 */
	rv = recv_data(ntb, sgl, kva, cmd, last);

	sge->result = rv == DDI_FAILURE ? ENXIO : 0;

	return (rv);
}

static int
copy_sgl_data_to_bridge(ntb_trans_t *ntb, ring_entry_t *sgl, ntb_sge_t *sge,
    dcopy_cmd_t *cmd, ntb_endpoint_ops_t *ops, void *ep_arg, boolean_t last)
{
	caddr_t		kva;
	ring_entry_t	tmp_sgl;
	int		rv;

	/*
	 * convert remote in the sgl to a kernel virtual address
	 */
	kva = ops->remote_to_kva(ep_arg, sge);
	if (kva == NULL)
		return (DDI_FAILURE);

	tmp_sgl.src = kva;
	tmp_sgl.start = sgl->start;
	tmp_sgl.size = sgl->size;
	tmp_sgl.dir = sgl->dir;

	rv = send_data(ntb, &tmp_sgl, cmd, last);

	sge->result = rv == DDI_FAILURE ? ENXIO : 0;

	return (rv);
}

/*
 * Doorbell callback called when there is data on the bridge which needs
 * to be processed
 */
/* ARGSUSED */
static void
receive_data(void *arg, int doorbell)
{
	ntb_trans_t	*ntb = arg;

	mutex_enter(&ntb->rupt_lock);
	ntb->data_pending = B_TRUE;
	cv_signal(&ntb->data_cv);
	mutex_exit(&ntb->rupt_lock);
}

/*
 * Handle oustanding sgls from the "in" ring.
 * Each sgl is passed to the end-point to get an address to copy the data to,
 * the data is copied to the end-point's buffer, the end-point is notified
 * that the data has been flushed to its buffer.
 * Finally, the "recv" ring is updated, and the peer is sent an ack.
 */
static boolean_t
process_data(ntb_trans_t *ntb, uint64_t in_prod)
{
	sgl_hdr_t	*sgl_ring = &ntb->in.send->sgl_ring;
	sgl_hdr_t	*o_sgl_ring = &ntb->out.recv->sgl_ring;
	ring_entry_t	*sgl, *sgl_head;
	ntb_sge_t	*sge;
	ntb_ep_hdl_t	*ep;
	dcopy_cmd_t	b_cmd = NULL;	/* for copy to bridge */
	dcopy_cmd_t	k_cmd = NULL;	/* for copy to kernel */
	uint64_t	sgls, count, last_prod, store_count, fetch_count;
	uint64_t	next_id;
	int		i;
	boolean_t	store_rv, fetch_rv;

	DTRACE_PROBE2(data_in, uint64_t, in_prod, uint64_t, ntb->last_prod);

	/*
	 * Some sanity checks on the incoming data - both sides must
	 * have symmetric DMA setups
	 */
	if (sgl_ring->head_off != ntb->out.send->sgl_ring.head_off ||
	    sgl_ring->size != ntb->out.send->sgl_ring.size)
		return (B_FALSE);

	sgl_head = (ring_entry_t *)(uintptr_t)(ntb->r_info.vaddr +
	    sgl_ring->head_off);

	mutex_enter(&ntb->ring_lock);

	if (peer_down(ntb)) {
		mutex_exit(&ntb->ring_lock);
		return (B_FALSE);
	}

	if (in_prod <= ntb->last_prod) {
		mutex_exit(&ntb->ring_lock);
		/*
		 * We've already handled everything outstanding
		 */
		DTRACE_PROBE(no_data);
		return (B_FALSE);
	}

	last_prod = ntb->last_prod;
	sgls = in_prod - last_prod;
	ntb->last_prod = in_prod;

	mutex_exit(&ntb->ring_lock);

	/*
	 * This should not happen, if it does there is something wrong with
	 * the data
	 */
	if (sgls > sgl_ring->size)
		return (B_FALSE);

	/*
	 * take the buffers from "cons" up to "prod"
	 * the data represented by the sgl's has to be copied from
	 * dma memory into kernel pages.
	 * When it is available we use dcopy (IOAT).
	 */

	i = last_prod % sgl_ring->size;
	if (sync_sgls_forkernel(ntb, ntb->r_info.dma_hdl, sgl_ring, i, sgls) !=
	    DDI_SUCCESS) {
		ntb->last_prod = last_prod;
		return (B_FALSE);
	}

	/*
	 * Store and fetch are assigned to different dcopy channels
	 * so we need to count how many there are of each in order
	 * to correctly determine the "last" argument passed to
	 * copy_sgl_data_to_bridge/kernel().
	 */
	store_count = fetch_count = 0;
	next_id = last_prod;
	for (count = sgls; count; count--) {
		sgl = &sgl_head[i];

		if (sgl->id == next_id++ && sgl->err == 0 &&
		    sgl->endpoint_id < HIGHEST_ENDPOINT_ID &&
		    ntb_ep[sgl->endpoint_id] != NULL) {
			if (sgl->dir == NTB_STORE)
				store_count++;
			else
				fetch_count++;
		}

		if (++i == sgl_ring->size)
			i = 0;
	}

	/*
	 * Run through all the outstanding sgls and initiate asynchronous
	 * copies of the data from the ring buffer to the endpoint's
	 * provided kva
	 */
	i = last_prod % sgl_ring->size;
	next_id = last_prod;
	for (count = sgls; count; count--) {
		sgl = &sgl_head[i];
		sge = &ntb->endpoint_sge[i];

		/*
		 * When the sgl id does not match the expected id or
		 * the sgl indicates an error, then this is an sgl
		 * that the peer allocated but didn't use because of other
		 * errors. Skip it
		 */
		if (sgl->id != next_id++ || sgl->err != 0) {
			DTRACE_PROBE2(id_err, uint64_t, sgl->id, uint64_t,
			    next_id - 1);
			sge->result = EINVAL;
			goto cont;
		}

		sge->local = sgl->src;
		sge->remote = sgl->offset;
		sge->bytes = sgl->size;

		rw_enter(&ep_lock, RW_READER);
		if (sgl->endpoint_id >= HIGHEST_ENDPOINT_ID ||
		    ntb_ep[sgl->endpoint_id] == NULL) {
			sge->result = EINVAL;
		} else {
			ep = ntb_ep[sgl->endpoint_id];

			if (sgl->dir == NTB_STORE) {
				ASSERT(store_count != 0);
				store_count--;
				(void) copy_sgl_data_to_kernel(ntb, sgl, sge,
				    &k_cmd, ep->ops, ep->user_arg,
				    store_count == 0);
			} else {
				ASSERT(fetch_count != 0);
				fetch_count--;
				(void) copy_sgl_data_to_bridge(ntb, sgl, sge,
				    &b_cmd, ep->ops, ep->user_arg,
				    fetch_count == 0);
			}

			if (sge->result == 0)
				atomic_inc_32(&ep->ref);
		}
		rw_exit(&ep_lock);

cont:
		if (++i == sgl_ring->size)
			i = 0;
	}

	/*
	 * Make sure the data has been copied into the endpoint's buffer,
	 * and any stores have gone into the bridge.
	 */
	store_rv = completed(&k_cmd);
	fetch_rv = completed(&b_cmd);

	if (!store_rv || !fetch_rv) {
		/*
		 * Feedback error status, which will get returned later
		 */
		i = last_prod % sgl_ring->size;
		for (count = sgls; count; count--) {
			sgl = &sgl_head[i];
			sge = &ntb->endpoint_sge[i];
			ep = ntb_ep[sgl->endpoint_id];
			if (sge->result == 0)
				atomic_dec_32(&ep->ref);

			if (sgl->dir == NTB_STORE) {
				if (!store_rv)
					sge->result = EIO;
			} else {
				if (!fetch_rv)
					sge->result = EIO;
			}

			if (++i == sgl_ring->size)
				i = 0;
		}
	}

	/*
	 * Now let the endpoint know the data has been flushed and
	 * give it the opportunity to update the sge
	 */
	i = last_prod % sgl_ring->size;
	for (count = sgls; count; count--) {
		sgl = &sgl_head[i];
		sge = &ntb->endpoint_sge[i];

		if (sge->result == 0) {
			ep = ntb_ep[sgl->endpoint_id];
			if (sgl->dir == NTB_STORE)
				ep->ops->remote_flushed(ep->user_arg, sge);
			atomic_dec_32(&ep->ref);
		}

		sgl->err = sge->result;
		o_sgl_ring->head[i] = *sgl;

		if (++i == sgl_ring->size)
			i = 0;
	}

	i = last_prod % sgl_ring->size;
	if (copy_sgls_to_bridge(ntb, o_sgl_ring, i, sgls) != DDI_SUCCESS)
		return (B_FALSE);

	/*
	 * This gets fed back to the peer, so they can update ring
	 * counters.
	 */
	ntb->out.recv->sgl_ring.cons = in_prod;
	ntb->out.recv->s_id = ntb->in.send->s_id;

	/*
	 * Send back what we have processed - this is the "ack"
	 * Any failures in the operations are pretty much fatal. The safest
	 * way to handle them is not send the doorbell and let them
	 * timeout.
	 */
	if (dma_to_bridge(ntb, ntb->out.recv_off, ntb->out.header_sz) !=
	    DDI_SUCCESS) {
		return (B_FALSE);
	}

	send_doorbell(ntb, ntb->node->ack);

	return (B_TRUE);
}

/*
 * Thread for processing data. Once it gets woken by a doorbell, it will
 * try and process continuously whilst there is unprocessed data in the
 * ring
 */
static void
receive_data_thr(void *arg)
{
	ntb_trans_t	*ntb = arg;
	ntb_node_t	*node = ntb->node;
	uint64_t	in_prod;

	//lgrp_move_curthread(get_numa_lgrp_prop(node->dip));

	for (;;) {
		mutex_enter(&ntb->rupt_lock);
		while (!ntb->data_pending && !(ntb->state & STATE_CANCEL)) {
			cv_wait(&ntb->data_cv, &ntb->rupt_lock);
		}
		ntb->data_pending = B_FALSE;
		mutex_exit(&ntb->rupt_lock);

		if (ntb->state & STATE_CANCEL)
			return;

		if (ntb_get(node->ntb_hdl, ntb->r_info.dma_hdl,
		    ntb->in.send_off, sizeof (dma_ring_t)) != DDI_SUCCESS)
			continue;

		in_prod = ntb->in.send->sgl_ring.next_prod;

		do {
			process_msg(ntb);

			if (!process_data(ntb, in_prod))
				break;

			if (ntb_get(node->ntb_hdl, ntb->r_info.dma_hdl,
			    ntb->in.send_off, sizeof (dma_ring_t)) !=
			    DDI_SUCCESS)
				break;

			in_prod = ntb->in.send->sgl_ring.next_prod;

		} while (in_prod > ntb->last_prod);
	}
}

static void
sync_wakeup(ntb_handle_t *hdl)
{
	ASSERT(mutex_owned(&hdl->hdl_lock));

	if (hdl->waiting > 0 && hdl->incomplete == 0) {
		DTRACE_PROBE1(signal_hdl, ntb_handle_t *, hdl);
		cv_broadcast(&hdl->hdl_cv);
	}
}

/*
 * This value needs to be more than any possible timeout from the DMA (IOAT)
 * engine. If it does timeout we need to be certain that no further DMA might
 * occur and corrupt the destnation once the caller thinks things are done.
 * This is particularly important if that destination is on the stack.
 * The corresponding timeout for dcopy/ioat would be
 * ioat_max_poll_delay*ioat_pollwintr_mult.
 */
static volatile int ntb_dcopy_timeout = DCOPY_TIMEOUT_US;
/*
 * called by ntb_sync(). Waits for all pending sgls on a handle to complete.
 * Can be woken by a timeout or peer detected as down.
 * returns -1 for timeout, 0 for success, 1 for other error
 */
static int
wait_for(ntb_trans_t *ntb, ntb_handle_t *hdl, boolean_t (*test_fn)(void *),
    void *test_arg)
{
	int	rv = 0;

	ASSERT(mutex_owned(&ntb->ring_lock));
	ASSERT(mutex_owned(&hdl->hdl_lock));

	ntb->pending++;

	while (!peer_down(ntb)) {
		if (!test_fn(test_arg)) {
			rv = 0;
			break;
		}
		if (rv == -1)
			break;
		mutex_exit(&ntb->ring_lock);

		rv = (int)cv_reltimedwait(&hdl->hdl_cv, &hdl->hdl_lock,
		    drv_usectohz(ntb_dcopy_timeout), TR_CLOCK_TICK);

		DTRACE_PROBE2(cvret, ntb_handle_t *, hdl, int, rv);
		if (!mutex_tryenter(&ntb->ring_lock)) {
			/* We are trying to get locks out of order */
			mutex_exit(&hdl->hdl_lock);
			mutex_enter(&ntb->ring_lock);
			mutex_enter(&hdl->hdl_lock);
		}
	}

	/*
	 * If the ring is being reset, and this is the last transaction
	 * to terminate, wake up the reset so it can complete
	 */
	if (--ntb->pending == 0 && ntb->state & STATE_RESETING)
		cv_broadcast(&ntb->ring_cv);

	if (peer_down(ntb))
		rv = 1;
	else
		rv = rv < 0 ? -1 : 0;

	return (rv);
}

static boolean_t
hdl_complete(void *arg)
{
	ntb_handle_t *hdl = arg;

	return (hdl->incomplete > 0);
}

static int
wait_for_all(ntb_trans_t *ntb, ntb_handle_t *hdl)
{
	return (wait_for(ntb, hdl, hdl_complete, hdl));
}

static boolean_t
req_complete(void *arg)
{
	sge_req_t *req = arg;

	return (req->blocked);
}

static int
wait_for_one(ntb_trans_t *ntb, sge_req_t *req)
{
	return (wait_for(ntb, req->hdl, req_complete, req));
}

/*
 * Copy the sgls starting at index sgl_idx for a count of num into the NTB.
 * If dcopy() is enabled, it will use dcopy() to move the data directly
 * into the PCI address of the bridge, which will let the data move
 * asynchronously and stop us from blocking here and enable us
 * to block at a single point and wait for all the dcopy() commands.
 */
static int
copy_sgls_to_bridge(ntb_trans_t *ntb, sgl_hdr_t *sgl_ring, int sgl_idx,
    int num)
{
	int	rv;

	if ((sgl_idx + num) > sgl_ring->size) {
		int	part1 = sgl_ring->size - sgl_idx;

		/*
		 * Will wrap the ring, so split into two copies.
		 */
		rv = dma_to_bridge(ntb,
		    sgl_ring->head_off + sgl_idx * sizeof (ring_entry_t),
		    part1 * sizeof (ring_entry_t));

		if (rv != DDI_SUCCESS)
			return (rv);

		/*
		 * Adjust for part2
		 */
		sgl_idx = 0;
		num -= part1;
	}

	return (dma_to_bridge(ntb,
	    sgl_ring->head_off + sgl_idx * sizeof (ring_entry_t),
	    num * sizeof (ring_entry_t)));
}

/*
 * Send a block of data described by the sgl across the NTB.
 * When dcopy() is enabled there is at least one less memory copy
 * and the copies do not block here. Which allows us to complete
 * further processing asynchronously and block at a single point waiting
 * for all to complete
 */
static int
send_data(ntb_trans_t *ntb, ring_entry_t *sgl, dcopy_cmd_t *cmd, boolean_t last)
{
	data_hdr_t	*data_ring;
	uint64_t	to_copy, size;
	uint64_t	start;
	caddr_t		src;
	int		rv;
	boolean_t	use_dma;

	data_ring = sgl->dir == NTB_STORE ? &ntb->out.send->data_ring :
	    &ntb->out.recv->data_ring;

	start = sgl->start % data_ring->size;
	src = sgl->src;
	size = sgl->size;
	use_dma = (size >= ntb->dcopy_threshold);

	if ((start + size) > data_ring->size) {
		to_copy = data_ring->size - start;

		rv = data_to_bridge(ntb, data_ring, cmd, start, src,
		    to_copy, use_dma, B_FALSE);

		if (rv != DDI_SUCCESS)
			return (rv);

		start = 0;
		src += to_copy;
		size -= to_copy;
	}

	return (data_to_bridge(ntb, data_ring, cmd, start, src, size,
	    use_dma, last));
}

static int
recv_data(ntb_trans_t *ntb, ring_entry_t *sgl, caddr_t kva, dcopy_cmd_t *cmd,
    boolean_t last)
{
	data_hdr_t	*data_ring;
	uint64_t	start, to_copy, size;
	int		rv;
	boolean_t	use_dma;

	data_ring = sgl->dir == NTB_STORE ? &ntb->in.send->data_ring :
	    &ntb->in.recv->data_ring;

	start = sgl->start % data_ring->size;
	size = sgl->size;
	use_dma = (ntb->dcopy_threshold == 0);

	if ((start + size) > data_ring->size) {
		to_copy = data_ring->size - start;

		rv = copy_data_to_kernel(ntb, data_ring, cmd, start, kva,
		    to_copy, use_dma, B_FALSE);

		if (rv != DDI_SUCCESS)
			return (rv);

		start = 0;
		kva += to_copy;
		size -= to_copy;
	}

	return (copy_data_to_kernel(ntb, data_ring, cmd, start, kva, size,
	    use_dma, last));
}

/*
 * Create and add a request for the specific handle
 */
static void
add_sge_request(sge_req_t **reqp, ntb_handle_t *hdl, ntb_sge_t *sge, int ref,
    ntb_cb_fn_t fn, void *arg, boolean_t blocks)
{
	sge_req_t	*req;

	if (*reqp == NULL)
		*reqp = kmem_cache_alloc(req_cache, KM_SLEEP);

	req = *reqp;
	req->sge = sge;
	req->hdl = hdl;
	req->ref = ref;
	req->b_cmd = NULL;
	req->k_cmd = NULL;
	req->ready = B_FALSE;
	req->blocked = blocks;
	req->notified = B_FALSE;
	req->complete_fn = fn;
	req->complete_arg = arg;

	mutex_enter(&hdl->hdl_lock);
	list_insert_tail(&hdl->req_list, req);
	hdl->incomplete++;
	mutex_exit(&hdl->hdl_lock);
}

/*
 * Take a completed request off the handle's list.
 */
static void
dequeue_sge_request(sge_req_t *req)
{
	mutex_enter(&req->hdl->hdl_lock);
	list_remove(&req->hdl->req_list, req);
	mutex_exit(&req->hdl->hdl_lock);
}

static void
release_sge_request(sge_req_t *req)
{
	uint_t nv;

	ASSERT(req->ref > 0);

	nv = atomic_dec_uint_nv(&req->ref);

	if (nv == 0) {
		if (req->b_cmd)
			dcopy_cmd_free(&req->b_cmd);
		if (req->k_cmd)
			dcopy_cmd_free(&req->k_cmd);

		kmem_cache_free(req_cache, req);
	}
}

static void
remove_sge_request(sge_req_t *req)
{
	mutex_enter(&req->hdl->hdl_lock);
	list_remove(&req->hdl->req_list, req);
	ASSERT(req->hdl->incomplete > 0);
	req->hdl->incomplete--;
	sync_wakeup(req->hdl);
	mutex_exit(&req->hdl->hdl_lock);

	release_sge_request(req);
}

static void
free_sge_request(sge_req_t *req)
{
	mutex_enter(&req->hdl->hdl_lock);
	ASSERT(req->hdl->incomplete > 0);
	req->hdl->incomplete--;
	sync_wakeup(req->hdl);
	mutex_exit(&req->hdl->hdl_lock);

	release_sge_request(req);
}

/*
 * Block until there is space for an sgl and the data, and then reserve it.
 * Returns with xmit_lock held
 */
static boolean_t
reserve_ring_space(ntb_trans_t *ntb, ntb_dir_t dir, ntb_sge_t *sge)
{
	sgl_hdr_t	*sgl_ring;
	data_hdr_t	*data_ring;
	uint64_t	sgl_avail, data_avail;
	boolean_t	ok;

	mutex_enter(&ntb->xmit_lock);

	sgl_ring = &ntb->out.send->sgl_ring;
	data_ring = dir == NTB_STORE ? &ntb->out.send->data_ring :
	    &ntb->out.recv->data_ring;

	mutex_enter(&ntb->ring_lock);
	sgl_avail = sgl_ring->size - (sgl_ring->prod - sgl_ring->cons);
	data_avail = data_ring->size - (data_ring->prod - data_ring->cons);

	ok = peer_up(ntb);

	ntb->waiting++;
	while ((sgl_avail == 0 || data_avail < sge->bytes) && ok) {

		mutex_exit(&ntb->ring_lock);
		cv_wait(&ntb->ring_cv, &ntb->xmit_lock);
		mutex_enter(&ntb->ring_lock);

		ok = peer_up(ntb);
		if (ok) {
			sgl_avail = sgl_ring->size - (sgl_ring->prod -
			    sgl_ring->cons);
			data_avail = data_ring->size - (data_ring->prod -
			    data_ring->cons);
		}
	}
	ntb->waiting--;

	if (ok) {
		data_ring->prod += sge->bytes;
		sgl_ring->prod++;
	} else if ((ntb->state & STATE_RESETING)) {
		cv_broadcast(&ntb->ring_cv);
	}

	mutex_exit(&ntb->ring_lock);

	return (ok);
}

/*
 * Allocates space in the dma rings for the sgl and data and initiates
 * the copy of it across the bridge.
 * To maintain ring integrity this function has to be single threaded per
 * ntb_trans instance.
 */
static int
sge_send(ntb_handle_t *hdl, ntb_sge_t *sge, ntb_cb_fn_t fn, void *fn_arg,
    boolean_t blocks, sge_req_t **reqp, ntb_dir_t dir)
{
	ntb_trans_t	*ntb = hdl->ntb;
	sgl_hdr_t	*sgl_ring;
	data_hdr_t	*data_ring;
	sge_req_t	*req;
	int		i, rv;

	/*
	 * First check there is space in the ring, and reserve what we
	 * will use.
	 * Returns with xmit_lock held
	 */
	if (!reserve_ring_space(ntb, dir, sge)) {
		mutex_exit(&ntb->xmit_lock);
		sge->result = EIO;

		if (fn)
			fn(hdl, sge, fn_arg);

		*reqp = NULL;

		return (DDI_FAILURE);
	}

	sgl_ring = &ntb->out.send->sgl_ring;
	data_ring = dir == NTB_STORE ? &ntb->out.send->data_ring :
	    &ntb->out.recv->data_ring;

	i = (sgl_ring->prod - 1) % sgl_ring->size;

	/*
	 * Add the sge request to the req_list and set the initial reference
	 * count. There is a reference for each of:
	 *
	 * 1. Allocation and enqueuing on request list for the handle.
	 * 2. Enqueuing on the post_q list for the asynchronous posting of
	 *    the request.
	 * 3. 1 and 2, above, can be freed asynchronously when a peer goes
	 *    down, so we need an extra reference through the critical
	 *    path in this function.
	 * 4. If the request is synchronous a hold is placed on the request
	 *    only to be released when the synchronous request completes
	 *    end to end.
	 */
	add_sge_request(reqp, hdl, sge, blocks ? 4 : 3, fn, fn_arg, blocks);
	req = *reqp;

	sge->result = -1;
	req->id = sgl_ring->prod - 1;
	req->idx = i;
	ntb->callers[i] = req;

	sgl_ring->head[i].id = req->id;
	sgl_ring->head[i].src = sge->local;
	sgl_ring->head[i].size = sge->bytes;
	sgl_ring->head[i].offset = sge->remote;
	sgl_ring->head[i].start = data_ring->prod - sge->bytes;
	sgl_ring->head[i].endpoint_id = hdl->endpoint_id;
	sgl_ring->head[i].dir = dir;
	sgl_ring->head[i].err = 0;

	/*
	 * There is a required ordering between when the sgls are stored in the
	 * header and the data is available in the ring buffer. This guarantees
	 * all memory stores are globally visibly and the SGLs and data
	 * will be in sync on all CPUs.
	 */
	membar_producer();

	list_insert_tail(&ntb->post_q, req);

	mutex_exit(&ntb->xmit_lock);

	rv = dir == NTB_STORE ? send_data(ntb, &sgl_ring->head[i], &req->b_cmd,
	    B_TRUE) : DDI_SUCCESS;

	if (rv != DDI_FAILURE)
		rv = completed(&req->b_cmd) ?
		    copy_sgls_to_bridge(ntb, sgl_ring, i, 1) : DDI_FAILURE;

	mutex_enter(&ntb->xmit_lock);
	if (rv != DDI_SUCCESS) {
		if (blocks) {
			/* Drop an extra reference requested by the caller. */
			release_sge_request(req);
		}

		/* Check whether the request is still on the send queue. */
		if (list_link_active(&req->p_link)) {
			/* It is on the send queue. We simply remove it. */
			list_remove(&ntb->post_q, req);
		} else {
			/*
			 * It is not on the send queue anymore. This means
			 * that the post thread has already called fail_sge()
			 * and release_sge_request() because the NTB peer is
			 * down or the ring is shutting down.
			 *
			 * We drop the reference held for the scope of this
			 * function early and prevent any further processing
			 * of the request in the code below.
			 */
			release_sge_request(req);
			req = NULL;
		}
	} else {
		req->ready = B_TRUE;
		cv_signal(&ntb->post_cv);
	}
	mutex_exit(&ntb->xmit_lock);

	if (req != NULL) {
		if (rv != DDI_SUCCESS) {
			fail_sge(ntb, sge, req, EIO);
			/*
			 * We originally acquired three or four references.
			 * The optional fourth reference for the benefit of the
			 * caller has already been dropped above. The call to
			 * fail_sge() dropped one more reference which brings
			 * us down to two. We drop another reference here
			 * because this request has been abandoned. The last
			 * reference which is tied to the scope of this
			 * function will be dropped unconditionally below.
			 */
			release_sge_request(req);
		}

		/* Drop the reference held for use by this function */
		release_sge_request(req);
	}

	return (rv);
}

/*
 * Run through the outgoing list of sgls, from the last one used, looking
 * for sgls that are in error and can be reaped. To maintain ring integrity
 * only single or a consecutive group of failed sgls can be reaped.
 */
static void
reap_sgls(ntb_trans_t *ntb)
{
	sgl_hdr_t	*sgl_ring = &ntb->out.send->sgl_ring;
	ring_entry_t	*sgl;
	uint64_t	u;
	int		i, last_send, last_fetch;

	ASSERT(mutex_owned(&ntb->ring_lock));

	last_send = last_fetch = -1;
	i = sgl_ring->cons % sgl_ring->size;
	for (u = sgl_ring->cons; u < sgl_ring->prod; u++) {
		if (ntb->reap[i] == NULL)
			break;

		if (ntb->reap[i]->dir == NTB_STORE)
			last_send = i;
		else
			last_fetch = i;

		ntb->reap[i] = NULL;

		if (++i == sgl_ring->size)
			i = 0;
	}

	if (last_send >= 0) {
		sgl = &sgl_ring->head[last_send];
		/* reap data */
		ntb->out.send->data_ring.cons = sgl->start + sgl->size;
	}

	if (last_fetch >= 0) {
		sgl = &sgl_ring->head[last_fetch];
		/* reap data */
		ntb->out.recv->data_ring.cons = sgl->start + sgl->size;
	}

	/* reap sgls, if any */
	DTRACE_PROBE2(reaper, uint64_t, sgl_ring->cons, uint64_t, u);

	sgl_ring->cons = u;
}

/*
 * Mark an sgl entry as failed
 */
static void
fail_sge(ntb_trans_t *ntb, ntb_sge_t *sge, sge_req_t *req, int err)
{
	sgl_hdr_t	*sgl_ring = &ntb->out.send->sgl_ring;
	int		i;

	mutex_enter(&ntb->ring_lock);

	/*
	 * If we are called with the sge explicitly passed as an argument,
	 * then it is safe to set result.
	 */
	if (sge != NULL)
		sge->result = err;

	if (req != NULL) {
		ASSERT(sge == NULL || req->sge == sge);

		i = req->id % sgl_ring->size;
		sgl_ring->head[i].err = err;

		if (ntb->callers[i] != NULL) {
			ntb->callers[i] = NULL;
			ntb->reap[i] = &sgl_ring->head[i];

			invoke_completion_fn(req, err);

			remove_sge_request(req);
		}
	}

	reap_sgls(ntb);

	mutex_exit(&ntb->ring_lock);
}

/*
 * Fail all sgl entries for a given handle
 */
static void
fail_handle_sges(ntb_trans_t *ntb, ntb_handle_t *hdl, int err)
{
	sgl_hdr_t	*sgl_ring = &ntb->out.send->sgl_ring;
	sge_req_t	*req;
	int		i;

	ASSERT(mutex_owned(&ntb->ring_lock));

	mutex_enter(&hdl->hdl_lock);
	while ((req = list_remove_head(&hdl->req_list)) != NULL) {
		mutex_exit(&hdl->hdl_lock);

		i = req->id % sgl_ring->size;
		sgl_ring->head[i].err = err;
		ntb->callers[i] = NULL;
		ntb->reap[i] = &sgl_ring->head[i];

		DTRACE_PROBE2(req, sge_req_t *, req, uint64_t, req->id);

		ASSERT(req->hdl == hdl);

		invoke_completion_fn(req, err);

		release_sge_request(req);

		mutex_enter(&hdl->hdl_lock);
		ASSERT(hdl->incomplete > 0);
		hdl->incomplete--;
		sync_wakeup(hdl);
	}
	mutex_exit(&hdl->hdl_lock);
}

/*
 * Find the device with the least number of open handles
 */
static ntb_trans_t *
find_best_transport(ntb_class_t class)
{
	ntb_trans_t	*ntb;
	ntb_trans_t	*best = NULL;
	uint_t		count = UINT_MAX;
	list_t		*br = &bridges[class];

	for (ntb = list_head(br); ntb != NULL; ntb = list_next(br, ntb)) {
		if ((ntb->state & STATE_INITED) == 0)
			continue;

		if (ntb->hdl_count < count) {
			best = ntb;
			count = ntb->hdl_count;
		}
	}

	return (best);
}

/*
 * Allocate a handle for transmission to a given endpoint id over a specific
 * class of device
 */
ntb_handle_t *
ntb_alloc_handle(uint_t endpoint_id, ntb_class_t class)
{
	ntb_handle_t	*hdl;
	ntb_trans_t	*ntb;
	ddi_dma_attr_t	attr;
	ddi_device_acc_attr_t acc;

	if (endpoint_id >= HIGHEST_ENDPOINT_ID)
		return (NULL);

	if (class >= NTB_CLASS_CNT)
		return (NULL);

	/*
	 * From the list of transports, pick the one with the lowest reference
	 * count
	 */
	if ((ntb = find_best_transport(class)) == NULL) {
		/*
		 * This shouldn't happen, so we'll print a message.
		 */
		cmn_err(CE_WARN, "!ntb_trans: failed to find initialised "
		    "bridge for class: %s, endpoint %d", classes[class].name,
		    endpoint_id);
		return (NULL);
	}

	if (ntb_get_dma_attr(ntb->node->ntb_hdl, NTB_PUT, &attr, &acc, NULL) !=
	    DDI_SUCCESS) {
		return (NULL);
	}

	hdl = kmem_alloc(sizeof (ntb_handle_t), KM_SLEEP);

	hdl->ntb = ntb;
	hdl->endpoint_id = endpoint_id;
	hdl->waiting = 0;
	hdl->incomplete = 0;

	list_create(&hdl->req_list, sizeof (sge_req_t),
	    offsetof(sge_req_t, link));

	mutex_init(&hdl->hdl_lock, NULL, MUTEX_DEFAULT, NULL);
	cv_init(&hdl->hdl_cv, NULL, CV_DRIVER, NULL);

	mutex_enter(&ntb->ring_lock);
	list_insert_tail(&ntb->hdl_list, hdl);
	ntb->hdl_count++;
	mutex_exit(&ntb->ring_lock);

	return (hdl);
}

void
ntb_release_handle(ntb_handle_t *hdl)
{
	ntb_trans_t	*ntb = hdl->ntb;

	mutex_enter(&ntb->ring_lock);
	list_remove(&ntb->hdl_list, hdl);
	ntb->hdl_count--;
	mutex_exit(&ntb->ring_lock);

	list_destroy(&hdl->req_list);
	mutex_destroy(&hdl->hdl_lock);
	cv_destroy(&hdl->hdl_cv);
	kmem_free(hdl, sizeof (*hdl));
}

/*
 * This thread takes sge_req_t off the post_q and posts them into the
 * NTB, requests are sent strictly in order.
 * A request only ever appears on the post_q once its sgl and data (if
 * NTB_STORE) have been placed into NTB's address space.
 * PCI message ordering guarantees the sgl and data will arrive in the ring
 * buffers at the peer before the request posted here.
 */
static void
post_thr(void *arg)
{
	ntb_trans_t	*ntb = arg;
	ntb_node_t	*node = ntb->node;
	list_t		to_send;
	sge_req_t	*req, *next;
	int		rv;

	//lgrp_move_curthread(get_numa_lgrp_prop(node->dip));

	list_create(&to_send, sizeof (sge_req_t), offsetof(sge_req_t, p_link));

	for (;;) {
		mutex_enter(&ntb->xmit_lock);
		do {
			/*
			 * We wait until the sge_req_t at the head of the
			 * queue is ready. It is critical notifications of
			 * requests are sent in order.
			 */
			req = list_head(&ntb->post_q);
			if ((req == NULL || (peer_up(ntb) && !req->ready)) &&
			    !(ntb->state & STATE_ENDING)) {
				cv_wait(&ntb->post_cv, &ntb->xmit_lock);
			} else {
				break;
			}
		} while (/* CONSTCOND */ 1);

		if (peer_down(ntb) || (ntb->state & STATE_ENDING)) {
			/* remove and fail everything */
			while ((req = list_head(&ntb->post_q)) != NULL) {
				list_remove(&ntb->post_q, req);
				fail_sge(ntb, NULL, req, EIO);
				release_sge_request(req);
			}

			if (ntb->state & STATE_ENDING) {
				mutex_exit(&ntb->xmit_lock);
				break;
			}

			mutex_exit(&ntb->xmit_lock);
			continue;
		}

		/*
		 * Find the request with the highest id, we actually only
		 * need to post the highest id. process_data() will process
		 * all un-handled sgls.
		 */
		next = req;
		do {
			req = next;
			next = list_next(&ntb->post_q, req);

			list_remove(&ntb->post_q, req);
			/* add to a local list for later clean up */
			list_insert_tail(&to_send, req);

		} while (next != NULL && next->ready);

		mutex_exit(&ntb->xmit_lock);

		ASSERT(req);
		ASSERT(req->ref > 0);

		/* The next_prod is always one higher than the request id */
		ntb->out.send->sgl_ring.next_prod = req->id + 1;

		/* and post it across the ntb */
		rv = dma_to_bridge(ntb, ntb->out.send_off, ntb->out.header_sz);

		if (rv == DDI_SUCCESS)
			send_doorbell(ntb, node->data);

		while ((req = list_head(&to_send)) != NULL) {
			list_remove(&to_send, req);
			if (rv != DDI_SUCCESS) {
				fail_sge(ntb, NULL, req, EIO);
			}
			release_sge_request(req);
		}
	}

	list_destroy(&to_send);
}

size_t
ntb_transfer_max(ntb_handle_t *hdl)
{
	return (hdl->ntb->sge_xfer_max);
}

/*
 * Send an sgl (and data). It does not wait for the peer to acknowledge
 * and process the data.
 * If any part of the send fails, it will return DDI_FAILURE, the sgl
 * entry will have specific errno.
 */
static int
async_store_fetch_common(ntb_handle_t *hdl, ntb_sge_t *sge, ntb_cb_fn_t fn,
    void *fn_arg, ntb_dir_t dir)
{
	ntb_trans_t	*ntb;
	sge_req_t	*req = NULL;
	int		err = 0;

	if (hdl == NULL || (ntb = hdl->ntb) == NULL) {
		return (DDI_FAILURE);
	}

	if (peer_down(ntb))
		err = EIO;
	else if (sge->bytes > ntb->sge_xfer_max)
		err = E2BIG;

	if (err != 0) {
		sge->result = err;
		if (fn)
			fn(hdl, sge, fn_arg);
		return (DDI_FAILURE);
	}

	return (sge_send(hdl, sge, fn, fn_arg, B_FALSE, &req, dir));
}

int
ntb_async_store(ntb_handle_t *hdl, ntb_sge_t *sge, ntb_cb_fn_t fn, void *fn_arg)
{
	return (async_store_fetch_common(hdl, sge, fn, fn_arg, NTB_STORE));
}

/*
 * Waits for all pending ntb_async_store/fetch() to complete.
 * When it returns, the sgls for all the requests will have completion
 * status.
 * Even when this return DDI_SUCCESS, the status of any individual sgl
 * entries should be check as this reflect success or failure at the peer.
 */
int
ntb_sync(ntb_handle_t *hdl)
{
	ntb_trans_t	*ntb = hdl->ntb;
	int		rv;

	mutex_enter(&ntb->ring_lock);
	mutex_enter(&hdl->hdl_lock);
	hdl->waiting++;
	if ((rv = wait_for_all(ntb, hdl)) != 0) {
		/* timed out or peer went down */
		hdl->waiting--;
		mutex_exit(&hdl->hdl_lock);
		fail_handle_sges(ntb, hdl, rv < 0 ? ETIMEDOUT : EIO);
		reap_sgls(ntb);
		mutex_exit(&ntb->ring_lock);

		return (DDI_FAILURE);
	}

	hdl->waiting--;
	mutex_exit(&hdl->hdl_lock);
	mutex_exit(&ntb->ring_lock);

	return (DDI_SUCCESS);
}

/* ARGSUSED */
static void
wakeup_req(ntb_handle_t *hdl, ntb_sge_t *sge, void *arg)
{
	sge_req_t *req = arg;

	if (req == NULL)
		return;

	mutex_enter(&hdl->hdl_lock);
	req->blocked = B_FALSE;
	cv_broadcast(&hdl->hdl_cv);
	mutex_exit(&hdl->hdl_lock);
}

/*
 * common code for synchronous sending or fetching from the peer
 */
static int
store_fetch_common(ntb_handle_t *hdl, ntb_sge_t *sge, ntb_dir_t dir)
{
	ntb_trans_t	*ntb = hdl->ntb;
	sge_req_t	*req;
	int		rv, wait_rv;

	if (peer_down(ntb)) {
		sge->result = EIO;
		return (DDI_FAILURE);
	}

	if (sge->bytes > ntb->sge_xfer_max) {
		sge->result = E2BIG;
		return (DDI_FAILURE);
	}

	/*
	 * Pre-allocate the req. wakeup_req needs the req as an argument,
	 * but the argument is stored in the req itself. And the contents
	 * of the req need to be set before any data pertaining to it
	 * are sent across the NTB to avoid any race with the completion
	 * processing.
	 */
	req = kmem_cache_alloc(req_cache, KM_SLEEP);

	rv = sge_send(hdl, sge, wakeup_req, req, B_TRUE, &req, dir);
	if (rv != DDI_SUCCESS)
		return (DDI_FAILURE);

	mutex_enter(&ntb->ring_lock);
	mutex_enter(&hdl->hdl_lock);

	if ((wait_rv = wait_for_one(ntb, req)) != 0) {
		/* timed out or peer went down */
		rv = DDI_FAILURE;
	}

	mutex_exit(&ntb->ring_lock);
	mutex_exit(&hdl->hdl_lock);

	if (req->blocked) {
		/* fail_sge() can't have been called. Do it now. */
		fail_sge(ntb, sge, req, wait_rv < 0 ? ETIMEDOUT : EIO);
	}

	release_sge_request(req);

	return (rv);
}

/*
 * send an individual sgl and wait for its completion
 */
int
ntb_store(ntb_handle_t *hdl, ntb_sge_t *sge)
{
	return (store_fetch_common(hdl, sge, NTB_STORE));
}

/*
 * send a request to fetch data from the peer don't wait for it to complete
 */
int
ntb_async_fetch(ntb_handle_t *hdl, ntb_sge_t *sge, ntb_cb_fn_t fn, void *fn_arg)
{
	return (async_store_fetch_common(hdl, sge, fn, fn_arg, NTB_FETCH));
}

/*
 * send an individual fetch sgl and wait for its completion
 */
int
ntb_fetch(ntb_handle_t *hdl, ntb_sge_t *sge)
{
	return (store_fetch_common(hdl, sge, NTB_FETCH));
}

/*
 * Allocate the arrays which are dependent on the size of the rings.
 */
static void
allocate_arrays(ntb_trans_t *ntb)
{
	ntb->caller_sz = ntb->out.send->sgl_ring.size;
	ntb->callers = kmem_zalloc(sizeof (sge_req_t *) * ntb->caller_sz,
	    KM_SLEEP);
	ntb->reap = kmem_zalloc(sizeof (ring_entry_t *) * ntb->caller_sz,
	    KM_SLEEP);
	ntb->endpoint_sge = kmem_zalloc(sizeof (ntb_sge_t) * ntb->caller_sz,
	    KM_SLEEP);
}

static void
free_arrays(ntb_trans_t *ntb)
{
	if (ntb->callers != NULL)
		kmem_free(ntb->callers, sizeof (sge_req_t *) * ntb->caller_sz);
	if (ntb->reap != NULL)
		kmem_free(ntb->reap, sizeof (ring_entry_t *) * ntb->caller_sz);
	if (ntb->endpoint_sge != NULL)
		kmem_free(ntb->endpoint_sge, sizeof (ntb_sge_t) *
		    ntb->caller_sz);

	ntb->callers = NULL;
	ntb->reap = NULL;
	ntb->endpoint_sge = NULL;
}

/*
 * This illustrates the layout of the DMA data buffers. There are two DMA
 * buffers allocated with different CPU caching attributes. One buffer,
 * equivalent to one NTB segment, is used for headers and sgl rings. The
 * other buffer, the remainder of the NTB segments allocated to the device,
 * is used for the data ring.
 * The header and sgl buffer uses normal caching attributes as they are
 * accessed using ddi_get() routines. The data buffer has caching disabled
 * and is accessed using dcopy/ioat, the ioat device bypasses the CPU
 * cache so there is not performance penalty. It seems the hardware ioat
 * devices can hang when CPU caching is enabled for the DMA memory when
 * the NTB is writing into the memory and ioat is reading.
 *
 *           +== Headers and SGL DMA =+
 *           |  Send hdr              |
 *           |  Send sgl ring ptr  >--+----+
 *           |  Send data ring ptr >--+----+-+
 *           |  Send msg buffers      |    | |
 *           +------------------------+    | |
 *           |  Receive hdr           |    | |
 *      +----+< Receive sgl ring ptr  |    | |
 *    +-+----+< Receive data ring ptr |    | |
 *    | |    |  Receive msg buffers   |    | |
 *    | |    +========= 4K ===========+<---+ |
 *    | |    |                        |      |
 *    | |    |  Send sgl ring         |      |
 *    | |    |                        |      |
 *    | +--->+------------------------+      |
 *    |      |                        |      |
 *    |      |  Receive sgl ring      |      |
 *    |      |                        |      |
 *    |      +========================+      |
 *    |                                      |
 *    |      +======= Data DMA =======+<-----+
 *    |      |                        |
 *    |      |  Send data ring        |
 *    |      |                        |
 *    +----->+------------------------+
 *           |                        |
 *           |  Receive data ring     |
 *           |                        |
 *           +========================+
 *
 * Version 1 ring format differs in that the DMA is allocated as
 * a continuous single piece of DMA and the Data portion follows directly
 * after the Receive sgl ring. In both version 1 and the current (version 2)
 * the headers in the 1st 4K have the same layout.
 *
 * Since the ntb_transport device is bi-directional a map is created for
 * transactions in the "out" direction, and "in" when the transactions
 * terminate in the peer.
 *
 *
 * Data flow between peers goes like this when sending from A to B:
 *           A                                    B
 *    SGL of out, send    ------->         SGL in, send      -------+
 *    Data of out         ------->         Data in                  ! status
 *    SGL of in, recv     <-------         SGL out, recv     <------+
 *
 * There are two buffers one for outgoing data (out) and one for incoming
 * data. The data in "out" will end up in the "in" of the peer.
 * Within each buffer there are two logical rings, a "send" ring and a
 * "recv" ring. The "send" ring tracks what has been sent (or "produced'),
 * and the "recv" ring is what the peer has received and processed (or
 * consumed).
 * Each "send" and "recv" ring is an array of scatter gather entries. In the
 * "send" ring is the primary definition of source, destination and size
 * of data, in the "recv" ring the scatter gather entry provides feedback
 * as to success or failure of the request.
 * The data for each SGL is also managed in a ring, there is a single "data"
 * ring in the "out" buffer which maps to a "data" ring in the "in" buffer.
 */
static void
map_headers_and_sgls(dma_info_t *dma, dma_header_t *hdr, int ring_entries,
    int roundup)
{
	caddr_t	data;
	int	i;

	hdr->header_sz = offsetof(dma_ring_t, msg_id);

	/*
	 * The "send" and "recv" dma_ring_t are before SGL_OFFSET.
	 * At SGL_OFFSET are the "send" sgl entries,
	 * on the next 64 byte alignemnt after this are the "recv" sgl
	 * entries which will return error status for each sgl.
	 * Then on a 4K boundary after this is the main data ring.
	 */

	hdr->send = (dma_ring_t *)(uintptr_t)dma->vaddr;
	hdr->send_off = 0;
	/* align the recv on 64 byte */
	hdr->recv = (dma_ring_t *)P2ROUNDUP((uintptr_t)(hdr->send + 1), 0x40);
	hdr->recv_off = (uintptr_t)hdr->recv - (uintptr_t)dma->vaddr;

	hdr->send->s_id = 0;
	hdr->send->sgl_ring.head = (ring_entry_t *)(uintptr_t)(dma->vaddr +
	    SGL_OFFSET);
	hdr->send->sgl_ring.head_off = SGL_OFFSET;
	hdr->send->sgl_ring.prod = 0;
	hdr->send->sgl_ring.next_prod = 0;
	hdr->send->sgl_ring.size = (uint_t)ring_entries;
	hdr->send->sgl_ring.cons = 0;
	bzero(hdr->send->sgl_ring.head, sizeof (ring_entry_t) * ring_entries);
	hdr->send->msg_id = 0;
	for (i = 0; i < MSG_CNT; i++) {
		hdr->send->msg[i].msg_id = 0;
		hdr->send->msg[i].msg = 0;
	}

	hdr->recv->s_id = 0;
	data = (caddr_t)(&hdr->send->sgl_ring.head[ring_entries]);
	data = (caddr_t)P2ROUNDUP((uintptr_t)data, roundup);
	hdr->recv->sgl_ring.head = (ring_entry_t *)(uintptr_t)data;
	hdr->recv->sgl_ring.head_off = (uint_t)((uintptr_t)data -
	    (uintptr_t)dma->vaddr);
	hdr->recv->sgl_ring.prod = 0;
	hdr->recv->sgl_ring.next_prod = 0;
	hdr->recv->sgl_ring.size = (uint_t)ring_entries;
	hdr->recv->sgl_ring.cons = 0;
	bzero(hdr->recv->sgl_ring.head, sizeof (ring_entry_t) * ring_entries);
	hdr->recv->msg_id = 0;
	for (i = 0; i < MSG_CNT; i++) {
		hdr->recv->msg[i].msg_id = 0;
		hdr->recv->msg[i].msg = 0;
	}
}

static void
map_data_rings(dma_info_t *dma, dma_header_t *hdr, caddr_t start, size_t size)
{
	/*
	 * The send data ring starts at the beginning of the DMA mem.
	 */
	hdr->send->data_ring.head = start;
	hdr->send->data_ring.head_off = (uint_t)((uintptr_t)start -
	    (uintptr_t)dma->vaddr);
	size >>= 1;
	hdr->send->data_ring.size = size;
	hdr->send->data_ring.prod = 0;
	hdr->send->data_ring.cons = 0;

	/*
	 * and the recv data ring is after the send data ring
	 */
	start = hdr->send->data_ring.head + size;
	hdr->recv->data_ring.head = start;
	hdr->recv->data_ring.head_off = (uint_t)((uintptr_t)start -
	    (uintptr_t)dma->vaddr);
	hdr->recv->data_ring.size = size;
	hdr->recv->data_ring.prod = 0;
	hdr->recv->data_ring.cons = 0;
}

static void
map_v1_rings(dma_info_t *dma, dma_header_t *hdr, size_t ring_size)
{
	caddr_t		data;
	size_t		ring_entries, data_size;

	/*
	 * Take a guess at how many ring entries there might base on an
	 * arbitrary guess at the average size of a data message
	 */
	ring_entries = (ring_size - SGL_OFFSET) / (((sizeof (ring_entry_t) +
	    DATA_MSG_SIZE) * 2));

	map_headers_and_sgls(dma, hdr, ring_entries, 0x40);

	/*
	 * The send data ring starts at the next 4K boundary beyond the
	 * sgl ring
	 */
	data = (caddr_t)(&hdr->recv->sgl_ring.head[ring_entries]);
	data = (caddr_t)P2ROUNDUP((uintptr_t)data, 0x1000);
	data_size = ring_size - (size_t)((uintptr_t)data -
	    (uintptr_t)dma->vaddr);

	map_data_rings(dma, hdr, data, data_size);
}

static int
allocate_v1_rings(ntb_trans_t *ntb)
{
	ntb_node_t	*node = ntb->node;
	size_t		wr_size;
	ddi_dma_attr_t	attr;
	ddi_device_acc_attr_t acc;

	/*
	 * Set the bind limits to the segments for this transport.
	 */
	if (ntb_set_bind_limits(node->ntb_hdl, ntb->seg, ntb->cnt) !=
	    DDI_SUCCESS)
		return (DDI_FAILURE);

	/*
	 * Allocate and map dma for the read side of the ring buffers
	 */
	if (allocate_read_dma(ntb, &ntb->r_info, 0) == DDI_FAILURE)
		goto failed;

	if (ntb->r_info.size < MIN_RING_SIZE) {
		cmn_err(CE_WARN, "!Insufficient space for transport rings. "
		    "%lu allocated, at least %d needed", ntb->r_info.size,
		    MIN_RING_SIZE);
		goto failed;
	}

	map_v1_rings(&ntb->r_info, &ntb->in, ntb->r_info.size);

	if (ntb_get_dma_attr(node->ntb_hdl, NTB_PUT, &attr, &acc,
	    &wr_size) == DDI_FAILURE)
		goto failed;

	/*
	 * When ioat/dcopy is enabled, and it is used for all ntb_transport
	 * transactions, the data ring only exists within the bridges PCI
	 * memory, so we don't need to allocate dma mem for it.
	 */
	if (ntb->dcopy_threshold == 0)
		wr_size = ntb->in.send->data_ring.head_off;

	if (allocate_dma(ntb, &ntb->w_info, &attr, &acc, 0, DDI_DMA_WRITE,
	    wr_size, B_TRUE) == DDI_FAILURE)
		goto failed;

	map_v1_rings(&ntb->w_info, &ntb->out, ntb->r_info.size);

	ntb->rd_infop = &ntb->r_info;
	ntb->wr_infop = &ntb->w_info;

	allocate_arrays(ntb);

	ntb->sge_xfer_max = MIN(MAX_SGE, ntb->out.send->data_ring.size >> 3);

	(void) ntb_reset_bind_limits(node->ntb_hdl);

	return (DDI_SUCCESS);

failed:
	free_dma(ntb, &ntb->r_info);
	(void) ntb_reset_bind_limits(node->ntb_hdl);

	return (DDI_FAILURE);
}

static void
free_v1_rings(ntb_trans_t *ntb)
{
	free_dma(ntb, &ntb->r_info);
	free_dma(ntb, &ntb->w_info);

	free_arrays(ntb);
}

static void
map_sgl_rings(dma_info_t *dma, dma_header_t *hdr, size_t ring_size)
{
	size_t	ring_entries;

	/*
	 * Calculate how many ring entries there are beyond the space
	 * used for headers.
	 * The difference between this and a V1 map is the number of
	 * entries is restricted to a single NTB segment.
	 */
	ring_entries = (ring_size - SGL_OFFSET) / (2 * sizeof (ring_entry_t));

	/*
	 * 0x4 roundup will make sure the send and recv sgl arrays are
	 * next to each other without any wasted space, otherwise the
	 * ring_entries may overflow the DMA.
	 */
	map_headers_and_sgls(dma, hdr, ring_entries, 0x4);
}

static int
allocate_rings(ntb_trans_t *ntb)
{
	ntb_node_t	*node = ntb->node;
	size_t		wr_size;
	int		seg, cnt, uncached;
	ddi_dma_attr_t	attr;
	ddi_device_acc_attr_t acc;


	if (ntb->cnt < 2) {
		cmn_err(CE_WARN, "!Insufficient NTB segments (%d). At least "
		    "2 are required", ntb->cnt);
		return (DDI_FAILURE);
	}

	seg = ntb->seg;
	cnt = ntb->cnt;

	/*
	 * Restrict the bind to the first NTB segment allocated to this
	 * device and allocate headers and sgl rings.
	 */
	if (ntb_set_bind_limits(node->ntb_hdl, seg, 1) != DDI_SUCCESS ||
	    allocate_read_dma(ntb, &ntb->r_info, 0) != DDI_SUCCESS)
		return (DDI_FAILURE);

	if (ntb->r_info.size < MIN_RING_SIZE) {
		cmn_err(CE_WARN, "!Insufficient space for transport rings. "
		    "%lu allocated, at least %d needed", ntb->r_info.size,
		    MIN_RING_SIZE);
		goto free;
	}

	map_sgl_rings(&ntb->r_info, &ntb->in, ntb->r_info.size);

	if (ntb_get_dma_attr(node->ntb_hdl, NTB_PUT, &attr, &acc, NULL) ==
	    DDI_FAILURE) {
		goto free;
	}

	if (allocate_dma(ntb, &ntb->w_info, &attr, &acc, 0, DDI_DMA_WRITE,
	    ntb->r_info.size, B_TRUE) == DDI_FAILURE) {
		goto free;
	}

	map_sgl_rings(&ntb->w_info, &ntb->out, ntb->w_info.size);

	/*
	 * Allocate another DMA buffer. This starts at the segment after
	 * the headers and uses up the rest of the NTB segments allocated
	 * to this transport.
	 */
	if (ntb->dcopy_threshold == 0)
		/* dcopy use in data_to_kernel() */
		uncached = IOMEM_DATA_UNCACHED;
	else
		uncached = 0;

	if (ntb_set_bind_limits(node->ntb_hdl, seg + 1, cnt - 1) !=
	    DDI_SUCCESS || allocate_read_dma(ntb, &ntb->rd_info, uncached) !=
	    DDI_SUCCESS) {
		goto free;
	}

	/*
	 * The data rings are at the start of the DMA
	 */
	map_data_rings(&ntb->rd_info, &ntb->in, ntb->rd_info.vaddr,
	    ntb->rd_info.size);

	if (ntb->dcopy_threshold == 0) {
		/*
		 * When ioat/dcopy is enabled, the data ring only exists within
		 * the bridges PCI memory.
		 * If we are only using dcopy, allocate a page of DMA so we
		 * can get the cookies for the NTB segments. If both dcopy
		 * and non-dcopy are used, allocate all the required memory.
		 */
		wr_size = PAGESIZE;
	} else {
		wr_size = ntb->rd_info.size;
	}

	if (allocate_dma(ntb, &ntb->wd_info, &attr, &acc, 0,
	    DDI_DMA_WRITE, wr_size, B_TRUE) != DDI_SUCCESS) {
		goto free;
	}

	map_data_rings(&ntb->wd_info, &ntb->out, ntb->wd_info.vaddr,
	    ntb->rd_info.size);

	(void) ntb_reset_bind_limits(node->ntb_hdl);

	allocate_arrays(ntb);

	ntb->rd_infop = &ntb->rd_info;
	ntb->wr_infop = &ntb->wd_info;

	ntb->sge_xfer_max = MIN(MAX_SGE, ntb->out.send->data_ring.size >> 3);

	return (DDI_SUCCESS);

free:
	free_dma(ntb, &ntb->r_info);
	free_dma(ntb, &ntb->rd_info);
	free_dma(ntb, &ntb->w_info);

	return (DDI_FAILURE);
}

static void
free_rings(ntb_trans_t *ntb)
{
	free_dma(ntb, &ntb->r_info);
	free_dma(ntb, &ntb->rd_info);
	free_dma(ntb, &ntb->w_info);
	free_dma(ntb, &ntb->wd_info);

	free_arrays(ntb);
}

/*
 * Allocate the ntb_trans structures based on definitions in
 * ntb_transport.conf.
 * The properties "ntb-dma-segment" and "ntb-dma-segment-count" are
 * required by the parent NTB driver, these define the starting
 * NTB segment to be reserved, and how many segments for ntb_transport.
 * This range is then sub-divided between instantiations of each ring with
 * instantiations starting from 0 and separate indices maintained for
 * "network" and "bulk".
 * E.g. If ntb-dma-segment=2 ntb-dma-segment-count=10, then a 2 segment ring
 * for network traffic and 2 x 4 segment rings for bulk traffic, could
 * be defined using:
 *	network0-segment=2 network0-segment-cnt=2
 *	bulk0-segment=4 bulk0-segment-cnt=4
 *	bulk1-segment=8 bulk1-segment-cnt=4
 *
 * The segment numbers referenced in these statements are NTB segment
 * numbers and should be in the range:
 *	[ ntb-dma-segment, ntb-dma-segment + ntb-dma-segment-count).
 *
 */
static int
create_bridge_classes(ntb_node_t *node)
{
	ntb_trans_t	*ntb;
	int		id, seg, cnt, segs_left;
	int		i, j, c_seg, c_cnt;
	char		name[32];

	if (ntb_get_dma_segments(node->ntb_hdl, &seg, &cnt) != DDI_SUCCESS) {
		cmn_err(CE_WARN, "!ntb_trans: failed to get assigned ntb "
		    "segments");
		return (DDI_FAILURE);
	}

	id = 0;
	segs_left = cnt;
	for (i = 0; i < NTB_CLASS_CNT; i++) {
		for (j = 0; ; j++) {
			(void) snprintf(name, sizeof (name), "%s%d-segment",
			    classes[i].name, j);
			c_seg = ddi_prop_get_int(DDI_DEV_T_ANY, node->dip,
			    DDI_PROP_DONTPASS, name, -1);

			if (c_seg == -1)
				break;

			(void) snprintf(name, sizeof (name), "%s%d-segment-cnt",
			    classes[i].name, j);
			c_cnt = ddi_prop_get_int(DDI_DEV_T_ANY, node->dip,
			    DDI_PROP_DONTPASS, name, -1);

			if (c_cnt == -1)
				break;

			/*
			 * Some rudimentary sanity checks.
			 * Later on when we try and use these segments
			 * to create the rings, if there is any overlap
			 * the higher up driver will return an error.
			 */
			if (c_seg < seg || (c_seg + c_cnt) > (seg + cnt))
				break;

			if (c_cnt > segs_left)
				break;

			segs_left -= c_cnt;

			ntb = kmem_zalloc(sizeof (ntb_trans_t), KM_SLEEP);
			ntb->node = node;
			ntb->id = id++;
			ntb->class = i;
			ntb->seg = c_seg;
			ntb->cnt = c_cnt;

			list_insert_tail(&bridges[i], ntb);
		}
	}

	return (DDI_SUCCESS);
}

static void
destroy_bridges(void)
{
	ntb_trans_t	*ntb;
	int		i;

	for (i = 0; i < NTB_CLASS_CNT; i++)
		while ((ntb = list_remove_head(&bridges[i])) != NULL)
			kmem_free(ntb, sizeof (ntb_trans_t));
}

/*
 * After the bare structures have been created, initialize each ring
 * based on what was parsed out of ntb_transport.conf.
 */
static int
setup_bridges(void)
{
	ntb_trans_t	*ntb;
	int		i;

	for (i = 0; i < NTB_CLASS_CNT; i++) {
		for (ntb = list_head(&bridges[i]); ntb != NULL;
		    ntb = list_next(&bridges[i], ntb)) {
			if (init_ntb_trans(ntb) != DDI_SUCCESS) {
				cleanup_bridges();
				return (DDI_FAILURE);
			}
		}
	}

	return (DDI_SUCCESS);
}

static void
cleanup_bridges(void)
{
	ntb_trans_t	*ntb;
	int		i;

	for (i = 0; i < NTB_CLASS_CNT; i++) {
		for (ntb = list_head(&bridges[i]); ntb != NULL;
		    ntb = list_next(&bridges[i], ntb)) {
			fini_ntb_trans(ntb);
		}
	}
}

/*
 * NTB doorbell callback.
 * When a doorbell is received it is passed on to each ring.
 */
/* ARGSUSED */
static void
bridges_callback(void *arg, int db)
{
	db_func_t	func = (db_func_t)arg;
	ntb_trans_t	*ntb;
	int		i;

	for (i = 0; i < NTB_CLASS_CNT; i++) {
		for (ntb = list_head(&bridges[i]); ntb != NULL;
		    ntb = list_next(&bridges[i], ntb)) {
			if ((ntb->state & STATE_INITED) == 0)
				continue;

			func(ntb, db);
		}
	}
}

/*
 * Initialize an individual ring.
 * This routine is not thread safe, and it is assumed each ring is
 * initialiized in sequence.
 */
static int
init_ntb_trans(ntb_trans_t *ntb)
{
	ntb_node_t	*node = ntb->node;
#if 0
	lgrp_id_t	lgrp;
#endif
	uint_t		disable_dcopy __unused; /* XXX */
	kthread_t	*thr;
	dcopy_query_t	dquery;
	char		tq_name[16];

	(void) snprintf(tq_name, sizeof (tq_name), "status_%d", ntb->id);
	if ((ntb->status_tq = ddi_taskq_create(node->dip, tq_name, 1,
	    TASKQ_DEFAULTPRI, 0)) == NULL) {
		cmn_err(CE_WARN, "!ntb_trans: failed to create taskq %s",
		    tq_name);
		goto failed;
	}

	/*
	 * if dcopy-disable=1 then disable dcopy and use programmed i/o
	 * if enabled, use dcopy-threshold to determine when to dcopy
	 *		transfer size < dcopy-threshold - use programmed i/o
	 *		transfer size >= dcopy-threshold - use dcopy
	 * if dcopy-threshold is not set/set to 0 and dcopy enabled,
	 * always use dcopy
	 * if dcopy is disabled, set the threshold to MAX_SIZET
	 */
	disable_dcopy = ddi_prop_get_int(DDI_DEV_T_ANY, node->dip,
	    DDI_PROP_DONTPASS, "disable-dcopy", 0);

	ntb->dcopy_threshold = ddi_prop_get_int(DDI_DEV_T_ANY, node->dip,
	    DDI_PROP_DONTPASS, "dcopy-threshold", 0);

	dcopy_query(&dquery);
#if 0
	ntb->dcopy_ena = B_TRUE;
	if (dquery.dq_num_channels > 0 && !disable_dcopy) {
		//lgrp = get_numa_lgrp_prop(node->dip);

		/*
		 * Allocate a dcopy channel from the same lgrp, if
		 * that fails, resort to letting the ntb driver do the
		 * copying of data.
		 */
		if (dcopy_alloc(DCOPY_SLEEP, lgrp, &ntb->b_dcopy_hdl) !=
		    DCOPY_SUCCESS) {
			/*
			 * let the ntb driver handle the sync'ing to peer
			 */
			ntb->dcopy_ena = B_FALSE;
		}
		if (ntb->dcopy_ena && dcopy_alloc(DCOPY_SLEEP, lgrp,
		    &ntb->k_dcopy_hdl) != DCOPY_SUCCESS) {
			dcopy_free(&ntb->b_dcopy_hdl);
			ntb->dcopy_ena = B_FALSE;
		}
	} else {
		ntb->dcopy_ena = B_FALSE;
	}
#else
	ntb->dcopy_ena = B_FALSE;
#endif

	if (ntb->dcopy_ena)
		cmn_err(CE_NOTE, "!ntb_trans: ntb_transport will use "
		    "dcopy/IOAT for blocks >= %d bytes",
		    (int)ntb->dcopy_threshold);
	else
		ntb->dcopy_threshold = MAX_SIZET;

	if (allocate_rings(ntb) != DDI_SUCCESS)
		goto failed;

	/*
	 * This lock is used in the "fast callbacks" which run at interrupt
	 * priority.
	 */
	mutex_init(&ntb->rupt_lock, NULL, MUTEX_ADAPTIVE,
	    (void *)ipltospl(LOCK_LEVEL));
	mutex_init(&ntb->ring_lock, NULL, MUTEX_DEFAULT, NULL);
	mutex_init(&ntb->xmit_lock, NULL, MUTEX_DEFAULT, NULL);
	cv_init(&ntb->ring_cv, NULL, CV_DRIVER, NULL);
	cv_init(&ntb->data_cv, NULL, CV_DRIVER, NULL);
	cv_init(&ntb->ack_cv, NULL, CV_DRIVER, NULL);
	cv_init(&ntb->post_cv, NULL, CV_DRIVER, NULL);
	cv_init(&ntb->alive_cv, NULL, CV_DRIVER, NULL);
	cv_init(&ntb->reset_cv, NULL, CV_DRIVER, NULL);

	list_create(&ntb->hdl_list, sizeof (ntb_handle_t),
	    offsetof(ntb_handle_t, link));
	list_create(&ntb->ack_req, sizeof (sge_req_t),
	    offsetof(sge_req_t, link));
	list_create(&ntb->post_q, sizeof (sge_req_t),
	    offsetof(sge_req_t, p_link));

	rw_enter(&ep_lock, RW_WRITER);
	classes[ntb->class].peer_count++;
	rw_exit(&ep_lock);

	ntb->version = CURRENT_VERSION;
	ntb->msg_fn[I_AM_ALIVE] = handle_alive_msg;
	ntb->msg_fn[I_AM_DYING] = handle_dying_msg;
	ntb->msg_fn[RESET_RINGS] = handle_reset_msg;
	ntb->msg_fn[HEADERS_SENT] = handle_headers_sent_msg;
	ntb->msg_fn[HEADERS_ACK] = handle_headers_ack_msg;

	/*
	 * Set thread priorities based on "class". Network is higher than
	 * bulk.
	 */
	thr = thread_create(NULL, 0, receive_data_thr, ntb, 0, &p0, TS_RUN,
	    maxclsyspri - ntb->class);
	ntb->data_thr = thr->t_did;

	thr = thread_create(NULL, 0, receive_ack_thr, ntb, 0, &p0, TS_RUN,
	    maxclsyspri - ntb->class);
	ntb->ack_thr = thr->t_did;

	thr = thread_create(NULL, 0, post_thr, ntb, 0, &p0, TS_RUN,
	    maxclsyspri - ntb->class);
	ntb->post_thr = thr->t_did;

	/*
	 * Mark ourselves as initialised now so we are able to act on
	 * messages as a response to the alive_thr.
	 */
	ntb->state |= STATE_INITED;

	/*
	 * This is v. low activity thread - keep it the same for all.
	 */
	thr = thread_create(NULL, 0, alive_thr, ntb, 0, &p0, TS_RUN,
	    maxclsyspri);
	ntb->alive_thr = thr->t_did;

	return (DDI_SUCCESS);

failed:
	if (ntb->status_tq)
		ddi_taskq_destroy(ntb->status_tq);
	if (ntb->b_dcopy_hdl)
		dcopy_free(&ntb->b_dcopy_hdl);
	if (ntb->k_dcopy_hdl)
		dcopy_free(&ntb->k_dcopy_hdl);

	return (DDI_FAILURE);
}

static int
ntb_trans_attach(dev_info_t *dip, ddi_attach_cmd_t cmd)
{
	int		inst;
	ntb_svc_t	*hdl = NULL;
	uint_t		base_db;
	uint_t		lo, hi, up, down;

	switch (cmd) {
	case DDI_RESUME:
		return (DDI_SUCCESS);

	case DDI_ATTACH:
		break;

	default:
		return (DDI_FAILURE);
	}

	if (ntb_node != NULL) {
		cmn_err(CE_WARN, "!ntb_trans: only one device is permitted");
		return (DDI_FAILURE);
	}

	inst = ddi_get_instance(dip);

	/*
	 * Allocate a handle as a NTB client.
	 */
	if (ntb_allocate_svc(dip, &hdl) != DDI_SUCCESS) {
		cmn_err(CE_WARN, "!ntb_trans: failed to allocate ntb svc hdl");
		goto failed;
	}

	if (ntb_get_doorbell_range(hdl, &lo, &hi) != DDI_SUCCESS ||
	    ntb_get_status_doorbells(hdl, &up, &down) != DDI_SUCCESS) {
		cmn_err(CE_WARN, "!ntb_trans: failed to get doorbell info");
		goto failed;
	}

	if ((base_db = ddi_prop_get_int(DDI_DEV_T_ANY, dip, DDI_PROP_DONTPASS,
	    "doorbell-base", -1)) == -1U || base_db > hi) {
		cmn_err(CE_WARN, "!ntb_trans: \"doorbell-base\" is not "
		    "supplied or out of range");
		goto failed;
	}

	ntb_node = kmem_zalloc(sizeof (ntb_node_t), KM_SLEEP);

	ntb_node->dip = dip;
	ntb_node->inst = inst;
	ntb_node->ntb_hdl = hdl;
	ntb_node->up = up;
	ntb_node->down = down;
	ntb_node->data = base_db;
	ntb_node->ack = base_db == hi ? lo : base_db + 1;

	/*
	 * Register ourselves as a NTB nexus, we will forward any
	 * NTB requests up to the parent device.
	 */
	if (ntb_register(dip, &ntb_trans_ops, ntb_node) != DDI_SUCCESS) {
		cmn_err(CE_WARN, "!ntb_trans: failed to register as ntb "
		    "nexus");
		goto failed;
	}

	if (ntb_register_kstat(dip, &ntb_node->ntb_kdata) != DDI_SUCCESS) {
		cmn_err(CE_WARN, "!ntb_trans: failed to register ntb kstat");
		goto failed;
	}

	if (create_bridge_classes(ntb_node) != DDI_SUCCESS) {
		cmn_err(CE_WARN, "!ntb_trans: failed to create bridge classes");
		goto failed;
	}

	ntb_node->ntb_hdl = hdl;

	/*
	 * Setup the "ack" and "data" doorbells before the individual
	 * bridge nodes are initialised to enable immediate message
	 * passing.
	 */
	(void) ntb_set_fast_doorbell(hdl, ntb_node->ack, bridges_callback,
	    (void *)receive_ack);
	(void) ntb_set_fast_doorbell(hdl, ntb_node->data, bridges_callback,
	    (void *)receive_data);

	if (setup_bridges() != DDI_SUCCESS) {
		cmn_err(CE_WARN, "!ntb_trans: failed to setup transports");
		goto failed;
	}

	/*
	 * Only set the "up" and "down" callbacks after the bridge nodes
	 * have been initialised to ensure the initial state is propagated
	 * across all nodes.
	 */
	(void) ntb_set_doorbell(hdl, ntb_node->up, bridges_callback,
	    (void *)state_change);
	(void) ntb_set_doorbell(hdl, ntb_node->down, bridges_callback,
	    (void *)state_change);

	ddi_set_driver_private(dip, ntb_node);

	ddi_report_dev(dip);

	return (DDI_SUCCESS);

failed:
	if (ntb_node != NULL && ntb_node->ntb_hdl != NULL) {
		(void) ntb_set_doorbell(ntb_node->ntb_hdl, ntb_node->up, NULL,
		    NULL);
		(void) ntb_set_doorbell(ntb_node->ntb_hdl, ntb_node->down, NULL,
		    NULL);
		(void) ntb_set_doorbell(ntb_node->ntb_hdl, ntb_node->ack, NULL,
		    NULL);
		(void) ntb_set_doorbell(ntb_node->ntb_hdl, ntb_node->data, NULL,
		    NULL);
	}

	(void) ntb_unregister(dip);

	cleanup_bridges();
	destroy_bridges();

	if (hdl != NULL)
		(void) ntb_free_svc(hdl);

	if (ntb_node != NULL) {
		kmem_free(ntb_node, sizeof (ntb_node_t));
		ntb_node = NULL;
	}

	return (DDI_FAILURE);
}

/*
 * Shutdown a ring.
 * Like init_ntb_trans() this routine is not thread safe and it is
 * assumed each ring will be shutdown sequentially.
 */
static void
fini_ntb_trans(ntb_trans_t *ntb)
{
	if ((ntb->state & STATE_INITED) == 0)
		return;

	send_dying_message(ntb);

	rw_enter(&ep_lock, RW_WRITER);
	classes[ntb->class].peer_count--;
	rw_exit(&ep_lock);

	mutex_enter(&ntb->xmit_lock);
	mutex_enter(&ntb->ring_lock);
	ntb->state |= STATE_ENDING | STATE_CANCEL;
	mutex_exit(&ntb->ring_lock);
	cv_signal(&ntb->alive_cv);
	cv_signal(&ntb->post_cv);
	cv_signal(&ntb->reset_cv);
	mutex_exit(&ntb->xmit_lock);

	mutex_enter(&ntb->rupt_lock);
	cv_signal(&ntb->data_cv);
	cv_signal(&ntb->ack_cv);
	mutex_exit(&ntb->rupt_lock);

	thread_join(ntb->data_thr);
	thread_join(ntb->ack_thr);
	thread_join(ntb->post_thr);
	thread_join(ntb->alive_thr);

	ddi_taskq_wait(ntb->status_tq);
	ddi_taskq_destroy(ntb->status_tq);

	ring_funcs[ntb->version - 1].free(ntb);

	if (ntb->b_dcopy_hdl)
		dcopy_free(&ntb->b_dcopy_hdl);
	if (ntb->k_dcopy_hdl)
		dcopy_free(&ntb->k_dcopy_hdl);

	list_destroy(&ntb->hdl_list);
	list_destroy(&ntb->ack_req);
	list_destroy(&ntb->post_q);

	mutex_destroy(&ntb->rupt_lock);
	mutex_destroy(&ntb->ring_lock);
	mutex_destroy(&ntb->xmit_lock);
	cv_destroy(&ntb->ring_cv);
	cv_destroy(&ntb->data_cv);
	cv_destroy(&ntb->ack_cv);
	cv_destroy(&ntb->post_cv);
	cv_destroy(&ntb->alive_cv);
	cv_destroy(&ntb->reset_cv);
}

static int
ntb_trans_detach(dev_info_t *dip, ddi_detach_cmd_t cmd)
{
	ntb_node_t	*node;
	ntb_trans_t	*ntb;
	int		class;

	node = ddi_get_driver_private(dip);
	ASSERT(node == ntb_node);

	switch (cmd) {
	case DDI_DETACH:
		break;

	case DDI_SUSPEND:
		return (DDI_SUCCESS);

	default:
		return (DDI_FAILURE);
	}

	/*
	 * Can't detach if we have any end points registered.
	 * Or, if any of the transports have handles allocated.
	 */
	for (class = 0; class < NTB_CLASS_CNT; class++) {
		if (classes[class].ep_count != 0)
			return (DDI_FAILURE);

		for (ntb = list_head(&bridges[class]); ntb != NULL;
		    ntb = list_next(&bridges[class], ntb))
			if (!list_is_empty(&ntb->hdl_list))
				return (DDI_FAILURE);
	}

	(void) ntb_set_doorbell(node->ntb_hdl, node->up, NULL, NULL);
	(void) ntb_set_doorbell(node->ntb_hdl, node->down, NULL, NULL);
	(void) ntb_set_doorbell(node->ntb_hdl, node->ack, NULL, NULL);
	(void) ntb_set_doorbell(node->ntb_hdl, node->data, NULL, NULL);

	(void) ntb_unregister(dip);

	cleanup_bridges();

	destroy_bridges();

	(void) ntb_free_svc(node->ntb_hdl);
	ddi_set_driver_private(node->dip, NULL);
	kmem_free(ntb_node, sizeof (ntb_node_t));
	ntb_node = NULL;

	return (DDI_SUCCESS);
}

/* ARGSUSED */
static int
ntb_info(dev_info_t *dip, ddi_info_cmd_t cmd, void *arg, void **result)
{
	dev_t	dev = (dev_t)arg;
	minor_t	inst = getminor(dev);

	switch (cmd) {
	case DDI_INFO_DEVT2DEVINFO:
		*result = (void *)ntb_node->dip;
		return (DDI_SUCCESS);

	case DDI_INFO_DEVT2INSTANCE:
		*result = (void *)(uintptr_t)inst;
		return (DDI_SUCCESS);

	default:
		return (DDI_FAILURE);
	}
}

/*
 * Register an endpoint's ops vector.
 * An endpoint can only register itself once.
 */
int
ntb_register_endpoint(ntb_endpoint_ops_t *ops, void *arg)
{
	class_t	*cp;

	if (ops->id >= HIGHEST_ENDPOINT_ID || ops->class >= NTB_CLASS_CNT)
		return (DDI_FAILURE);

	rw_enter(&ep_lock, RW_WRITER);

	if (ntb_ep[ops->id] != NULL) {
		rw_exit(&ep_lock);
		return (DDI_FAILURE);
	}

	/*
	 * keep the endpoints in an array by id for fast lookups
	 */
	ntb_ep[ops->id] = kmem_alloc(sizeof (ntb_ep_hdl_t), KM_SLEEP);
	ntb_ep[ops->id]->ops = ops;
	ntb_ep[ops->id]->user_arg = arg;
	ntb_ep[ops->id]->ref = 0;

	cp = &classes[ops->class];
	list_insert_tail(&cp->ep_registered, ntb_ep[ops->id]);

	cp->ep_count++;

	rw_downgrade(&ep_lock);

	/*
	 * If there is a link_status function, call it immediately so the
	 * caller can get the current status of the transport
	 */
	if (ops->link_status) {
		/*
		 * If any of the ntb devices are down, then effectively
		 * the ntb is down or on its way down
		 */
		ops->link_status(arg, cp->peer_count == 0 ||
		    cp->peer_up_count < cp->peer_count ?
		    TRANSPORT_DISCONNECTED : TRANSPORT_CONNECTED);
	}

	rw_exit(&ep_lock);

	return (DDI_SUCCESS);
}

int
ntb_unregister_endpoint(ntb_endpoint_ops_t *ops)
{
	ntb_ep_hdl_t	*ep;

	if (ops->id >= HIGHEST_ENDPOINT_ID || ops->class >= NTB_CLASS_CNT)
		return (DDI_FAILURE);

	rw_enter(&ep_lock, RW_WRITER);

	if ((ep = ntb_ep[ops->id]) == NULL || ep->ref) {
		rw_exit(&ep_lock);
		return (DDI_FAILURE);
	}

	list_remove(&classes[ops->class].ep_registered, ntb_ep[ops->id]);
	kmem_free(ntb_ep[ops->id], sizeof (ntb_ep_hdl_t));
	ntb_ep[ops->id] = NULL;

	classes[ops->class].ep_count--;

	rw_exit(&ep_lock);

	return (DDI_SUCCESS);
}

static int
ntb_trans_busmap(dev_info_t *dip, dev_info_t *rdip, ddi_map_req_t *mp,
    off_t offset, off_t len, caddr_t *vaddrp)
{
	dev_info_t	*pdip = (dev_info_t *)DEVI(dip)->devi_parent;

	return ((DEVI(pdip)->devi_ops->devo_bus_ops->bus_map)(pdip, rdip, mp,
	    offset, len, vaddrp));
}

static int
ntb_trans_ctlops(dev_info_t *dip, dev_info_t *rdip, ddi_ctl_enum_t ctlop,
    void *arg, void *result)
{
	if (ctlop == DDI_CTLOPS_REPORTDEV) {
		if (rdip == NULL)
			return (DDI_FAILURE);

		cmn_err(CE_CONT, "?ntb_transport child: %s@%s, %s%d\n",
		    ddi_node_name(rdip), ddi_get_name_addr(rdip),
		    ddi_driver_name(rdip), ddi_get_instance(rdip));

		return (DDI_SUCCESS);
	}

	return (ddi_ctlops(dip, rdip, ctlop, arg, result));
}

/*
 * We are also a ntb_svc nexus.
 * Any ntb_svc operations are passed up to the parent.
 */
static int
ntb_trans_get_dma_attr(void *pvt, dev_info_t *dip, xfer_dir_t dir,
    ddi_dma_attr_t *attr, ddi_device_acc_attr_t *acc_attr, size_t *len)
{
	ntb_svc_t	*svc = ((ntb_node_t *)pvt)->ntb_hdl;

	return (ntb_svc_get_dma_attr(svc, dip, dir, attr, acc_attr, len));
}

static int
ntb_trans_dma_mem_alloc(void *pvt, ddi_dma_handle_t handle, size_t length,
    ddi_device_acc_attr_t *accattrp, uint_t flags, int (*waitfp)(caddr_t),
    caddr_t arg, caddr_t *kaddrp, size_t *real_length, void **handlep)
{
	ntb_svc_t	*svc = ((ntb_node_t *)pvt)->ntb_hdl;

	return (ntb_svc_dma_mem_alloc(svc, handle, length, accattrp, flags,
	    waitfp, arg, kaddrp, real_length, handlep));
}

static void
ntb_trans_dma_mem_free(void *pvt, void **handlep)
{
	ntb_svc_t	*svc = ((ntb_node_t *)pvt)->ntb_hdl;

	ntb_svc_dma_mem_free(svc, handlep);
}

static int
ntb_trans_get_link_cookies(void *pvt, ddi_dma_handle_t hdl,
    ddi_dma_cookie_t **cpp, int *ccnt)
{
	ntb_svc_t	*svc = ((ntb_node_t *)pvt)->ntb_hdl;

	return (ntb_svc_get_link_cookies(svc, hdl, cpp, ccnt));
}

static int
ntb_trans_set_segment(void *pvt, ddi_dma_handle_t hdl, int seg, uint64_t base)
{
	ntb_svc_t	*svc = ((ntb_node_t *)pvt)->ntb_hdl;

	return (ntb_svc_set_segment(svc, hdl, seg, base));
}

static int
ntb_trans_set_window(void *pvt, dev_info_t *cdip, int window,
    ddi_dma_cookie_t *cp)
{
	ntb_svc_t	*svc = ((ntb_node_t *)pvt)->ntb_hdl;

	return (ntb_svc_set_window(svc, cdip, window, cp));
}

static int
ntb_trans_put(void *pvt, dev_info_t *cdip, ddi_dma_handle_t hdl, offset_t off,
    size_t len)
{
	ntb_svc_t	*svc = ((ntb_node_t *)pvt)->ntb_hdl;

	return (ntb_svc_put(svc, cdip, hdl, off, len));
}

static int
ntb_trans_vput(void *pvt, dev_info_t *cdip, ddi_dma_handle_t hdl,
    caddr_t vaddr, offset_t off, size_t len)
{
	ntb_svc_t	*svc = ((ntb_node_t *)pvt)->ntb_hdl;

	return (ntb_svc_vput(svc, cdip, hdl, vaddr, off, len));
}

static int
ntb_trans_get(void *pvt, dev_info_t *cdip, ddi_dma_handle_t hdl, offset_t off,
    size_t len)
{
	ntb_svc_t	*svc = ((ntb_node_t *)pvt)->ntb_hdl;

	return (ntb_svc_get(svc, cdip, hdl, off, len));
}

static int
ntb_trans_doorbell_ctl(void *pvt, dev_info_t *dip, db_ctl_t ctl, uint_t num,
    void *arg1, void *arg2)
{
	ntb_svc_t	*svc = ((ntb_node_t *)pvt)->ntb_hdl;

	return (ntb_svc_doorbell_ctl(svc, dip, ctl, num, arg1, arg2));
}

static ntb_drv_ops_t ntb_trans_ops = {
	ntb_trans_get_dma_attr,
	ntb_trans_dma_mem_alloc,
	ntb_trans_dma_mem_free,
	ntb_trans_get_link_cookies,
	ntb_trans_set_segment,
	ntb_trans_set_window,
	ntb_trans_put,
	ntb_trans_vput,
	ntb_trans_get,
	ntb_trans_doorbell_ctl
};

static struct bus_ops ntb_bus_ops = {
	BUSO_REV,
	ntb_trans_busmap,		/* bus_map */
	NULL,				/* bus_get_intrspec */
	NULL,				/* bus_add_intrspec */
	NULL,				/* bus_remove_intrspec */
	i_ddi_map_fault,		/* bus_map_fault */
	NULL,				/* bus_dma_map */
	ddi_dma_allochdl,		/* bus_dma_allochdl */
	ddi_dma_freehdl,		/* bus_dma_freehdl */
	ddi_dma_bindhdl,		/* bus_dma_bindhdl */
	ddi_dma_unbindhdl,		/* bus_unbindhdl */
	ddi_dma_flush,			/* bus_dma_flush */
	ddi_dma_win,			/* bus_dma_win */
	ddi_dma_mctl,			/* bus_dma_ctl */
	ntb_trans_ctlops,		/* bus_ctl */
	ddi_bus_prop_op,		/* bus_prop_op */
	NULL,				/* bus_get_eventcookie */
	NULL,				/* bus_add_eventcall */
	NULL,				/* bus_remove_eventcall */
	NULL,				/* bus_post_event */
	0,				/* bus_intr_ctl 	*/
	NULL,				/* bus_config		*/
	NULL,				/* bus_unconfig		*/
	0,				/* bus_fm_init		*/
	0,				/* bus_fm_fini		*/
	0,				/* bus_fm_access_enter	*/
	0,				/* bus_fm_access_exit	*/
	0,				/* bus_power		*/
	i_ddi_intr_ops			/* bus_intr_op		*/
};

struct cb_ops ntb_cb_ops = {
	nodev,			/* cb_open */
	nodev,			/* cb_close */
	nodev,			/* cb_strategy */
	nodev,			/* cb_print */
	nodev,			/* cb_dump */
	nodev,			/* cb_read */
	nodev,			/* cb_write */
	nodev,			/* cb_ioctl */
	nodev,			/* cb_devmap */
	nodev,			/* cb_mmap */
	nodev,			/* cb_segmap */
	nochpoll,		/* cb_chpoll */
	ddi_prop_op,		/* cb_prop_op */
	NULL,			/* cb_stream */
	D_MP,			/* cb_flag */
	CB_REV,			/* cb_rev */
	nodev,			/* cb_aread */
	nodev			/* cb_awrite */
};

static struct dev_ops ntb_dev_ops = {
	DEVO_REV,	/* devo_rev */
	0,		/* devo_refcnt */
	ntb_info,		/* devo_getinfo */
	nulldev,	/* devo_identify */
	nulldev,	/* devo_probe */
	ntb_trans_attach,	/* devo_attach */
	ntb_trans_detach,	/* devo_detach */
	nodev,		/* devo_reset */
	&ntb_cb_ops,	/* devo_cb_ops */
	&ntb_bus_ops,	/* devo_bus_ops */
	NULL,		/* devo_power */
	ddi_quiesce_not_needed	/* devo_quiesce */
};

static struct modldrv ntb_modldrv = {
	&mod_driverops,
	"NTB transport",
	&ntb_dev_ops
};

static struct modlinkage modlinkage = {
	MODREV_1, (void *)&ntb_modldrv, NULL
};

int
_info(struct modinfo *modinfop)
{
	return (mod_info(&modlinkage, modinfop));
}

int
_init(void)
{
	int	status, i;

	status = mod_install(&modlinkage);
	if (status == 0) {
		last_id = (uint64_t)gethrtime();
		if ((req_cache = kmem_cache_create("ntb_requests",
		    sizeof (sge_req_t), 8, NULL, NULL, NULL, NULL,
		    NULL, 0)) == NULL) {
			return (1);
		}
		rw_init(&ep_lock, NULL, RW_DRIVER, NULL);
		for (i = NTB_NETWORK; i < NTB_CLASS_CNT; i++) {
			list_create(&classes[i].ep_registered,
			    sizeof (ntb_ep_hdl_t),
			    offsetof(ntb_ep_hdl_t, link));
			list_create(&bridges[i], sizeof (ntb_trans_t),
			    offsetof(ntb_trans_t, c_link));
		}
		ntb_names_init();
	}

	return (status);
}

int
_fini(void)
{
	int status, i;

	status = mod_remove(&modlinkage);
	if (status == 0) {
		ntb_names_fini();
		kmem_cache_destroy(req_cache);
		rw_destroy(&ep_lock);
		for (i = NTB_NETWORK; i < NTB_CLASS_CNT; i++) {
			list_destroy(&classes[i].ep_registered);
			list_destroy(&bridges[i]);
		}
	}

	return (status);
}
