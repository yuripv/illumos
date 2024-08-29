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
 * Copyright 2024 Tintri by DDN, Inc. All rights reserved.
 */

#include "bnxt.h"

/* Doorbell functions */
static inline void
bnxt_db_write(bnxt_t *bp, uintptr_t off, uint32_t val)
{
	uintptr_t addr;

	addr = (uintptr_t)bp->db_base + off;
	ddi_put32(bp->db_hdl, (void *)addr, val);
}

static inline void
bnxt_db_cp(bnxt_ring_t *rp, bool intr)
{
	bnxt_db_write(rp->bp, rp->db, CMPL_DOORBELL_KEY_CMPL |
	    (rp->cp_cidx == UINT32_MAX ? 0 :
	    (rp->cp_cidx | CMPL_DOORBELL_IDX_VALID)) |
	    (intr ? 0 : CMPL_DOORBELL_MASK));
}

static inline void
bnxt_db_rx(bnxt_ring_t *rxr)
{
	bnxt_db_write(rxr->bp, rxr->db, RX_DOORBELL_KEY_RX | rxr->pidx);
}

static inline void
bnxt_db_tx(bnxt_ring_t *txr)
{
	bnxt_db_write(txr->bp, txr->db, TX_DOORBELL_KEY_TX | txr->pidx);
}

void
bnxt_cp_reset(bnxt_ring_t *rp)
{
	struct cmpl_base *cmp = (void *)rp->cp_descs.va;
	int i;

	for (i = 0; i < rp->cp_ndesc; i++)
		cmp[i].info3_v = !rp->cp_v_bit;
}

/* Update CP consumer index keeping interrupt state */
static inline void
bnxt_cp_update(bnxt_ring_t *rp)
{
	bnxt_db_cp(rp, rp->cp_intr);
}

inline void
bnxt_ring_intr_disable(bnxt_ring_t *rp)
{
	rp->cp_intr = false;
	bnxt_db_cp(rp, false);
}

inline void
bnxt_ring_intr_enable(bnxt_ring_t *rp)
{
	rp->cp_intr = true;
	bnxt_db_cp(rp, true);
}

static void
bnxt_rx_buf_refill(bnxt_ring_t *rxr)
{
	bool update = false;
	int pidx = rxr->pidx;
	int need;

	/* Unconditionaly update CP ring before adding any new buffers */
	bnxt_cp_update(rxr);

	if (rxr->ndesc_free == rxr->ndesc)
		need = rxr->ndesc - 1;
	else
		need = rxr->ndesc_free;

	for (int i = 0; i < need; i++) {
		bnxt_buf_t *buf;
		rx_prod_pkt_bd_t *rbd;

		buf = list_remove_head(&rxr->bufs_free);
		if (buf == NULL) {
			rxr->rx.stats.nobufs.value.ui64++;
			break;
		}
		rbd = &((rx_prod_pkt_bd_t *)rxr->descs.va)[pidx];
		rbd->flags_type = RX_PROD_PKT_BD_TYPE_RX_PROD_PKT;
		rbd->opaque = buf->opaque;
		rbd->addr = LE_64(buf->dma.pa);
		rbd->len = LE_16(buf->dma.len);
		rxr->ndesc_free--;
		update = true;
		pidx = RING_NEXT_PIDX(rxr, pidx);
	}
	/* Ring doorbell if any buffers were added to the ring */
	if (update) {
		rxr->pidx = pidx;
		BNXT_DMA_SYNC(rxr->descs, DDI_DMA_SYNC_FORDEV);
		bnxt_db_rx(rxr);
	}
}

static void
bnxt_rx_buf_return(bnxt_buf_t *buf)
{
	bnxt_ring_t *rxr = buf->rp;

	/* If buffer is not loaned to MAC we are tearing down the ring */
	if (!buf->rx.loaned)
		return;
	/* Mark the buffer as no longer loaned */
	buf->rx.loaned = false;
	/* Recreate the mblk */
	buf->mp = desballoc((unsigned char *)buf->dma.va, buf->dma.len, 0,
	    &buf->rx.frtn);
	/* Move the buffer to free list */
	mutex_enter(&rxr->lock);
	VERIFY0(list_link_active(&buf->node));
	list_insert_tail(&rxr->bufs_free, buf);
	mutex_exit(&rxr->lock);
}

void
bnxt_buf_ring_alloc(bnxt_ring_t *rp)
{
	bnxt_t *bp = rp->bp;
	size_t buf_size;

	VERIFY(rp->type == BNXT_RING_RX || rp->type == BNXT_RING_TX);

	switch (rp->type) {
	case BNXT_RING_RX:
		buf_size = bp->rx_buf_size;
		break;
	case BNXT_RING_TX:
		buf_size = bp->tx_buf_size;
		break;
	default:
		/*
		 * The default case should not be needed, but gcc is not smart
		 * enough.
		 */
		panic("!invalid ring type %d", rp->type);
	}

	rp->bufs = kmem_zalloc(rp->nbuf * sizeof (bnxt_buf_t *), KM_SLEEP);
	list_create(&rp->bufs_free, sizeof (bnxt_buf_t),
	    offsetof (bnxt_buf_t, node));

	for (int i = 0; i < rp->nbuf; i++) {
		bnxt_buf_t *buf;

		buf = kmem_zalloc(sizeof (*buf), KM_SLEEP);
		VERIFY0(bnxt_dma_alloc(bp, &buf->dma, &bnxt_acc_attr,
		    buf_size));
		buf->rp = rp;
		/* Encode parent ring type and index in the opaque field */
		buf->opaque = rp->type << 24 | rp->idx << 16 | i;

		switch (rp->type) {
		case BNXT_RING_RX:
			buf->rx.loaned = false;
			buf->rx.frtn.free_func = bnxt_rx_buf_return;
			buf->rx.frtn.free_arg = (void *)buf;
			buf->mp = desballoc((unsigned char *)buf->dma.va,
			    buf->dma.len, 0, &buf->rx.frtn);
			break;
		case BNXT_RING_TX:
			buf->tx.len = 0;
			buf->tx.next = NULL;
			break;
		}

		rp->bufs[i] = buf;
		list_insert_tail(&rp->bufs_free, buf);
	}

	switch (rp->type) {
	case BNXT_RING_RX:
		bnxt_rx_buf_refill(rp);
		break;
	case BNXT_RING_TX:
		bnxt_db_tx(rp);
		break;
	}
}

void
bnxt_buf_ring_free(bnxt_ring_t *rp)
{
	bnxt_buf_t *buf;

	while ((buf = list_remove_tail(&rp->bufs_free)) != NULL)
		;
	list_destroy(&rp->bufs_free);
	for (int i = 0; i < rp->nbuf; i++) {
		buf = rp->bufs[i];

		switch (rp->type) {
		case BNXT_RING_RX:
			freemsg(buf->mp);
			break;
		case BNXT_RING_TX:
			break;
		}

		bnxt_dma_free(&buf->dma);
		kmem_free(buf, sizeof (*buf));
	}
	kmem_free(rp->bufs, rp->nbuf * sizeof (bnxt_buf_t *));
}

static inline bnxt_buf_t *
bnxt_buf_find(bnxt_t *bp, uint32_t opaque)
{
	int rtype = (opaque >> 24) & 0xff;
	int ridx = (opaque >> 16) & 0xff;
	int idx = opaque & 0xffff;

	VERIFY(rtype == BNXT_RING_RX || rtype == BNXT_RING_TX);

	switch (rtype) {
	case BNXT_RING_RX:
		return (bp->rxr[ridx].bufs[idx]);
	case BNXT_RING_TX:
		return (bp->txr[ridx].bufs[idx]);
	default:
		/*
		 * The default case should not be needed, but gcc is not smart
		 * enough.
		 */
		panic("!invalid opaque %08x", opaque);
	}
}

static void
bnxt_async_cmpl(bnxt_t *bp, cmpl_base_t *cmpl)
{
	hwrm_async_event_cmpl_t *ae = (void *)cmpl;
	uint16_t async_id = LE_16(ae->event_id);

	switch (async_id) {
	case HWRM_ASYNC_EVENT_CMPL_EVENT_ID_LINK_STATUS_CHANGE:
	case HWRM_ASYNC_EVENT_CMPL_EVENT_ID_LINK_SPEED_CHANGE:
	case HWRM_ASYNC_EVENT_CMPL_EVENT_ID_LINK_SPEED_CFG_CHANGE:
		bnxt_link_update(bp);
		break;
	default:
		bnxt_error(bp, "!unhandled async completion type %u", async_id);
		break;
	}
}

static mblk_t *
bnxt_rx_cmpl_l2(bnxt_ring_t *rxr, int poll_bytes)
{
	bnxt_t *bp = rxr->bp;
	mblk_t *mp_head = NULL;
	mblk_t *mp_tail = NULL;
	uint16_t ndesc = 0;
	int total_bytes = 0;
	uint32_t cidx = rxr->cp_cidx;
	bool v_bit = rxr->cp_v_bit;

	for (;;) {
		rx_pkt_cmpl_t *cmpl;
		rx_pkt_cmpl_hi_t *cmpl_hi;
		bnxt_buf_t *buf;
		uint32_t errors;
		uint32_t cksum_flags = 0;
		uint32_t flags2;

		/* First BD */
		cmpl = &((rx_pkt_cmpl_t *)rxr->cp_descs.va)[rxr->cp_cidx];
		if (!CMPL_VALID(cmpl, rxr->cp_v_bit) ||
		    (LE_16(cmpl->flags_type) & CMPL_BASE_TYPE_MASK) !=
		    CMPL_BASE_TYPE_RX_L2) {
			rxr->cp_cidx = cidx;
			rxr->cp_v_bit = v_bit;
			break;
		}
		/* Check if new packet fits in the poll limit */
		if (poll_bytes != 0 && total_bytes + cmpl->len > poll_bytes) {
			rxr->cp_cidx = cidx;
			rxr->cp_v_bit = v_bit;
			break;
		}

		buf = bnxt_buf_find(bp, cmpl->opaque);
		VERIFY3P(buf, !=, NULL);

		/* Second BD */
		RING_NEXT_CIDX_V(rxr);
		cmpl_hi = &((rx_pkt_cmpl_hi_t *)rxr->cp_descs.va)[rxr->cp_cidx];
		errors = (LE_16(cmpl_hi->errors_v2) &
		    RX_PKT_CMPL_ERRORS_MASK) >> RX_PKT_CMPL_ERRORS_SFT;
		flags2 = LE_32(cmpl_hi->flags2);

		if (errors != 0) {
			/* TODO update error counters */
			bnxt_error(bp, "!rxr%d errors %u", rxr->idx, errors);
			list_insert_tail(&rxr->bufs_free, buf);
			goto next;
		}

		/* Get checksum flags */
		if (flags2 & RX_PKT_CMPL_FLAGS2_IP_CS_CALC)
			cksum_flags |= HCK_IPV4_HDRCKSUM_OK;
		if (flags2 & (RX_PKT_CMPL_FLAGS2_L4_CS_CALC |
		    RX_PKT_CMPL_FLAGS2_T_L4_CS_CALC))
			cksum_flags |= HCK_FULLCKSUM_OK;
		mac_hcksum_set(buf->mp, 0, 0, 0, 0, cksum_flags);

		if (mp_head == NULL)
			mp_head = buf->mp;
		/* Chain up with previous one */
		if (mp_tail != NULL)
			mp_tail->b_next = buf->mp;
		mp_tail = buf->mp;

		/* Mark buffer as loaned to MAC */
		buf->rx.loaned = true;
		/* Set the read data length */
		buf->mp->b_wptr = buf->mp->b_rptr + cmpl->len;

		total_bytes += cmpl->len;
		rxr->rx.stats.rbytes.value.ui64 += cmpl->len;
next:
		ndesc++;
		rxr->rx.stats.ipackets.value.ui64++;

		/* Do not overflow the RX ring */
		if (rxr->ndesc_free + ndesc >= rxr->ndesc - 1)
			break;
		/* If handling interrupt limit number of processed buffers */
		if (poll_bytes == 0 && ndesc >= rxr->ndesc / 4)
			break;

		cidx = rxr->cp_cidx;
		v_bit = rxr->cp_v_bit;
		RING_NEXT_CIDX_V(rxr);
	}

	rxr->ndesc_free += ndesc;

	return (mp_head);
}

/* Also handles async notifications delivered to RX CP ring 0 */
static mblk_t *
bnxt_rx_cmpl(bnxt_ring_t *rxr, int pb)
{
	bnxt_t *bp = rxr->bp;
	cmpl_base_t *cmpl;
	mblk_t *mp = NULL;
	uint32_t cidx;
	bool v_bit;
	uint16_t type;

again:
	BNXT_DMA_SYNC(rxr->cp_descs, DDI_DMA_SYNC_FORKERNEL);
	cidx = rxr->cp_cidx;
	v_bit = rxr->cp_v_bit;
	RING_NEXT_CIDX_V(rxr);
	cmpl = &((cmpl_base_t *)rxr->cp_descs.va)[rxr->cp_cidx];

	if (!CMPL_VALID(cmpl, rxr->cp_v_bit)) {
		rxr->cp_cidx = cidx;
		rxr->cp_v_bit = v_bit;
		return (NULL);
	}

	type = LE_16(cmpl->type) & CMPL_BASE_TYPE_MASK;
	switch (type) {
	case CMPL_BASE_TYPE_NO_OP:
		/* Seen intermittently, nothing to do */
		goto again;
	case CMPL_BASE_TYPE_HWRM_ASYNC_EVENT:
		bnxt_async_cmpl(bp, cmpl);
		goto again;
	case CMPL_BASE_TYPE_RX_L2:
		mp = bnxt_rx_cmpl_l2(rxr, pb);
		break;
	default:
		bnxt_error(bp, "!unexpected rx completion %u", type);
		break;
	}

	bnxt_rx_buf_refill(rxr);
	return (mp);
}

uint_t
bnxt_rx_intr(caddr_t arg1, caddr_t arg2)
{
	bnxt_ring_t *rxr = (bnxt_ring_t *)arg1;
	bnxt_t *bp = rxr->bp;
	mblk_t *mp;

	mutex_enter(&rxr->lock);
	bnxt_ring_intr_disable(rxr);
	mp = bnxt_rx_cmpl(rxr, 0);
	bnxt_ring_intr_enable(rxr);
	mutex_exit(&rxr->lock);

	if (mp != NULL)
		mac_rx_ring(bp->mh, rxr->rh, mp, rxr->rx.gen);

	return (DDI_INTR_CLAIMED);
}

mblk_t *
bnxt_ring_rx(void *arg, int poll_bytes)
{
	bnxt_ring_t *rxr = (bnxt_ring_t *)arg;
	mblk_t *mp;

	mutex_enter(&rxr->lock);
	mp = bnxt_rx_cmpl(rxr, poll_bytes);
	mutex_exit(&rxr->lock);

	return (mp);
}

static void
bnxt_tx_cmpl(bnxt_ring_t *txr)
{
	bnxt_t *bp = txr->bp;
	list_t to_free;
	uint16_t ndesc = 0;
	uint32_t cidx;
	bool v_bit;
	bool notify = false;

	mutex_enter(&txr->lock);
	if (txr->cp_busy) {
		mutex_exit(&txr->lock);
		return;
	}
	txr->cp_busy = true;
	mutex_exit(&txr->lock);

	list_create(&to_free, sizeof (bnxt_buf_t), offsetof(bnxt_buf_t, node));
	BNXT_DMA_SYNC(txr->cp_descs, DDI_DMA_SYNC_FORKERNEL);

	for (;;) {
		bnxt_buf_t *buf;
		tx_cmpl_t *cmpl;
		uint16_t type;
		uint16_t errors;

		cidx = txr->cp_cidx;
		v_bit = txr->cp_v_bit;

		RING_NEXT_CIDX_V(txr);
		cmpl = &((tx_cmpl_t *)txr->cp_descs.va)[txr->cp_cidx];

		if (!CMPL_VALID(cmpl, txr->cp_v_bit)) {
			txr->cp_cidx = cidx;
			txr->cp_v_bit = v_bit;
			break;
		}

		type = LE_16(cmpl->flags_type) & CMPL_BASE_TYPE_MASK;
		switch (type) {
		case CMPL_BASE_TYPE_TX_L2:
			errors = (LE_16(cmpl->errors_v) &
			    TX_CMPL_ERRORS_BUFFER_ERROR_MASK) >>
			    TX_CMPL_ERRORS_BUFFER_ERROR_SFT;
			if (errors != 0) {
				/* TODO update error counters */
				bnxt_error(bp, "!txr%d errors %u",
				    txr->idx, errors);
			}
			/* Get first buffer */
			buf = bnxt_buf_find(bp, cmpl->opaque);
			VERIFY3P(buf, !=, NULL);
			/* Account for tx_bd_long_hi */
			ndesc++;
			while (buf != NULL) {
				bnxt_buf_t *next_buf = buf->tx.next;

				VERIFY0(list_link_active(&buf->node));
				ndesc++;
				buf->tx.len = 0;
				buf->tx.next = NULL;
				list_insert_tail(&to_free, buf);
				buf = next_buf;
			}
			break;
		default:
			bnxt_error(bp, "!unexpected tx completion %u", type);
			continue;
		}
	}

	mutex_enter(&txr->lock);
	bnxt_cp_update(txr);
	txr->ndesc_free += ndesc;
	list_move_tail(&txr->bufs_free, &to_free);
	/* Unblock the ring if it has enough free descriptors */
	if (txr->tx.blocked && txr->ndesc_free > bp->tx_notify_thresh) {
		txr->tx.blocked = false;
		notify = true;
	}
	txr->cp_busy = false;
	mutex_exit(&txr->lock);

	if (notify)
		mac_tx_ring_update(txr->bp->mh, txr->rh);

	list_destroy(&to_free);
}

uint_t
bnxt_tx_intr(caddr_t arg1, caddr_t arg2)
{
	bnxt_ring_t *txr = (bnxt_ring_t *)arg1;

	mutex_enter(&txr->lock);
	bnxt_ring_intr_disable(txr);
	mutex_exit(&txr->lock);

	bnxt_tx_cmpl(txr);

	mutex_enter(&txr->lock);
	bnxt_ring_intr_enable(txr);
	mutex_exit(&txr->lock);

	return (DDI_INTR_CLAIMED);
}

static const uint16_t bnxt_tx_lhint[] = {
	TX_BD_LONG_FLAGS_LHINT_LT512,
	TX_BD_LONG_FLAGS_LHINT_LT1K,
	TX_BD_LONG_FLAGS_LHINT_LT2K,
	TX_BD_LONG_FLAGS_LHINT_LT2K,
	TX_BD_LONG_FLAGS_LHINT_GTE2K,
};

mblk_t *
bnxt_ring_tx(void *arg, mblk_t *mp)
{
	bnxt_ring_t *txr = arg;
	bnxt_t *bp = txr->bp;
	tx_bd_long_t *first_desc, *cur_desc;
	tx_bd_long_hi_t *first_desc_hi;
	bnxt_buf_t *cur_buf;
	mblk_t *cur_mp;
	list_t to_write;
	size_t ndesc;
	size_t total_len;
	size_t buf_rem;
	uint16_t first_pidx, first_pidx_hi, cur_pidx;

	ASSERT3P(mp->b_next, ==, NULL);

	if (txr->ndesc_free < bp->tx_recycle_thresh)
		bnxt_tx_cmpl(txr);

	mutex_enter(&txr->lock);
	if (txr->ndesc_free < bp->tx_notify_thresh) {
		txr->tx.stats.nodescs.value.ui64++;
		goto blocked;
	}
	mutex_exit(&txr->lock);

	/* List of buffers to write */
	list_create(&to_write, sizeof (bnxt_buf_t), offsetof(bnxt_buf_t, node));
	/* Account for tx_bd_long_hi */
	ndesc = 1;
	/* Process the message chain and fill up data buffers */
	cur_buf = NULL;
	buf_rem = 0;
	total_len = 0;
	for (cur_mp = mp; cur_mp != NULL; cur_mp = cur_mp->b_cont) {
		size_t len = MBLKL(cur_mp);
		size_t off = 0;

		while (len > 0) {
			const void *src;
			void *dest;
			size_t to_copy;

			if (cur_buf == NULL || buf_rem == 0) {
				bnxt_buf_t *new_buf;

				mutex_enter(&txr->lock);
				new_buf = list_remove_head(&txr->bufs_free);
				if (new_buf == NULL) {
					txr->tx.stats.nobufs.value.ui64++;
					mutex_exit(&txr->lock);
					goto rollback;
				}
				mutex_exit(&txr->lock);
				/* Chain up buffers */
				if (cur_buf != NULL)
					cur_buf->tx.next = new_buf;
				cur_buf = new_buf;
				cur_buf->tx.len = 0;
				list_insert_tail(&to_write, cur_buf);
				buf_rem = cur_buf->dma.len;
				ndesc++;
			}

			to_copy = min(len, buf_rem);
			src = cur_mp->b_rptr + off;
			dest = cur_buf->dma.va + cur_buf->tx.len;
			memcpy(dest, src, to_copy);

			/* Total length used for setting the size hint */
			total_len += to_copy;

			cur_buf->tx.len += to_copy;
			buf_rem -= to_copy;
			len -= to_copy;
			off += to_copy;
		}
	}

	VERIFY0(list_is_empty(&to_write));

	/* Write descriptors */
	mutex_enter(&txr->lock);
	if (ndesc + bp->tx_gap > txr->ndesc_free) {
		mutex_exit(&txr->lock);
		bnxt_tx_cmpl(txr);
		mutex_enter(&txr->lock);
		if (ndesc + bp->tx_gap > txr->ndesc_free) {
			txr->tx.stats.nodescs.value.ui64++;
			mutex_exit(&txr->lock);
			goto rollback;
		}
	}

	/* Starting tx_bd_long descriptor (also includes first data buffer) */
	first_pidx = txr->pidx;
	first_desc = &((tx_bd_long_t *)txr->descs.va)[first_pidx];
	first_desc->flags_type = LE_32(TX_BD_LONG_TYPE_TX_BD_LONG);
	/* Number of descriptors */
	first_desc->flags_type |= LE_32((ndesc << TX_BD_LONG_FLAGS_BD_CNT_SFT) &
	    TX_BD_LONG_FLAGS_BD_CNT_MASK);
	/* Size hint */
	if (total_len >= 2048)
		first_desc->flags_type |= TX_BD_LONG_FLAGS_LHINT_GTE2K;
	else
		first_desc->flags_type |= bnxt_tx_lhint[total_len >> 9];
	/* Starting tx_bd_long_hi descriptor (does not include data buffer) */
	first_pidx_hi = RING_NEXT_PIDX(txr, first_pidx);
	first_desc_hi = &((tx_bd_long_hi_t *)txr->descs.va)[first_pidx_hi];
	/* TODO actually fill the descriptor (TSO, checksums) */
	memset(first_desc_hi, 0, sizeof (*first_desc_hi));

	cur_desc = first_desc;	/* silence gcc */
	cur_pidx = txr->pidx;
	while ((cur_buf = list_remove_head(&to_write)) != NULL) {
		cur_desc = &((tx_bd_long_t *)txr->descs.va)[cur_pidx];

		if (cur_desc == first_desc) {
			/*
			 * Set the starting buffer opaque in the first
			 * descriptor as we will only get single completion
			 * record.  All the buffers for the message are already
			 * chained up above so the starting buffer opaque should
			 * be enough to clean up.
			 */
			cur_desc->opaque = cur_buf->opaque;
			/* Skip the tx_bd_long_hi descriptor */
			cur_pidx = RING_NEXT_PIDX(txr, cur_pidx);
		} else {
			cur_desc->flags_type =
			    LE_32(TX_BD_SHORT_TYPE_TX_BD_SHORT);
		}

		cur_desc->addr = LE_64(cur_buf->dma.pa);
		cur_desc->len = LE_16(cur_buf->tx.len);

		txr->tx.stats.obytes.value.ui64 += cur_buf->tx.len;
		txr->tx.stats.opackets.value.ui64++;

		cur_pidx = RING_NEXT_PIDX(txr, cur_pidx);
	}
	/* Mark last descriptor */
	cur_desc->flags_type |= LE_32(TX_BD_LONG_FLAGS_PACKET_END);
	BNXT_DMA_SYNC(txr->descs, DDI_DMA_SYNC_FORDEV);

	txr->ndesc_free -= ndesc;
	txr->pidx = cur_pidx;
	bnxt_db_tx(txr);
	mutex_exit(&txr->lock);

	list_destroy(&to_write);
	freemsgchain(mp);
	return (NULL);

rollback:
	/* We ran out of buffers or descriptors, clean up and return mp */
	for (bnxt_buf_t *buf = list_head(&to_write); buf != NULL;
	    buf = list_next(&to_write, buf)) {
		buf->tx.len = 0;
		buf->tx.next = NULL;
	}
	mutex_enter(&txr->lock);
	list_move_tail(&txr->bufs_free, &to_write);
	list_destroy(&to_write);
blocked:
	txr->tx.blocked = true;
	mutex_exit(&txr->lock);
	return (mp);
}
