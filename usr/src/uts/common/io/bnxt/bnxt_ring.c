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

static void
bnxt_rx_buf_return(bnxt_buf_t *buf)
{
	bnxt_ring_t *rp = buf->rp;

	mutex_enter(&rp->lock);
	VERIFY0(list_link_active(&buf->node));
	list_insert_tail(&rp->bufs_free, buf);
	mutex_exit(&rp->lock);
}

void
bnxt_buf_ring_alloc(bnxt_ring_t *rp)
{
	bnxt_t *bp = rp->bp;
	size_t buf_size;

	switch (rp->type) {
	case BNXT_RING_RX:
		buf_size = bp->rx_buf_size;
		break;
	case BNXT_RING_TX:
		buf_size = bp->tx_buf_size;
		break;
	default:
		panic("unhandled ring type %d", rp->type);
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
		buf->opaque = rp->type << 24 | rp->idx << 16 | i;
		buf->rx_frtn.free_func = bnxt_rx_buf_return;
		buf->rx_frtn.free_arg = (void *)buf;
		rp->bufs[i] = buf;
		list_insert_tail(&rp->bufs_free, buf);
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
		bnxt_dma_free(&rp->bufs[i]->dma);
		kmem_free(rp->bufs[i], sizeof (*buf));
	}
	kmem_free(rp->bufs, rp->nbuf * sizeof (bnxt_buf_t *));
}

static inline bnxt_buf_t *
bnxt_buf_find(bnxt_t *bp, uint32_t opaque)
{
	int rtype = (opaque >> 24) & 0xff;
	int ridx = (opaque >> 16) & 0xff;
	int idx = opaque & 0xffff;

	switch (rtype) {
	case BNXT_RING_RX:
		return (bp->rxr[ridx].bufs[idx]);
	case BNXT_RING_TX:
		return (bp->txr[ridx].bufs[idx]);
	default:
		panic("invalid opaque %08x", opaque);
	}
}

/* Doorbell functions */
static void
bnxt_db_write(bnxt_t *bp, uintptr_t off, uint32_t val)
{
	uintptr_t addr;

	addr = (uintptr_t)bp->db_base + off;
	ddi_put32(bp->db_hdl, (void *)addr, val);
}

inline void
bnxt_db_cp(bnxt_ring_t *rp, bool enable_irq)
{
	bnxt_db_write(rp->bp, rp->db, CMPL_DOORBELL_KEY_CMPL |
	    (rp->cp_cidx == UINT32_MAX ? 0 :
	    (rp->cp_cidx | CMPL_DOORBELL_IDX_VALID)) |
	    (enable_irq ? 0 : CMPL_DOORBELL_MASK));
}

inline void
bnxt_db_rx(bnxt_ring_t *rxr)
{
	bnxt_db_write(rxr->bp, rxr->db, RX_DOORBELL_KEY_RX | rxr->pidx);
}

inline void
bnxt_db_tx(bnxt_ring_t *txr)
{
	bnxt_db_write(txr->bp, txr->db, TX_DOORBELL_KEY_TX | txr->pidx);
}

void
bnxt_cp_mark_invalid(bnxt_ring_t *rp)
{
	struct cmpl_base *cmp = (void *)rp->cp_descs.va;
	int i;

	for (i = 0; i < rp->cp_ndesc; i++)
		cmp[i].info3_v = !rp->cp_v_bit;
}

/*
 * RX path.
 *
 * TODO:
 * - investigate TPA (bnxt's HW LRO)
 * - investigate checksums provided by hardware
 * - implement counters and/or check if stats provided by hardware are enough
 */

void
bnxt_rx_refill(bnxt_ring_t *rxr)
{
	bool update = false;
	int pidx;
	int need;

	if (!rxr->active) {
		mutex_exit(&rxr->lock);
		return;
	}
	pidx = rxr->pidx;
	need = min(rxr->ndesc_free, rxr->ndesc / 8);

	for (int i = 0; i < need; i++) {
		bnxt_buf_t *buf;
		rx_prod_pkt_bd_t *rbd;

		buf = list_remove_head(&rxr->bufs_free);
		if (buf == NULL)
			break;
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
		bnxt_db_cp(rxr, false);
		rxr->pidx = pidx;
		BNXT_DMA_SYNC(rxr->descs, DDI_DMA_SYNC_FORDEV);
		bnxt_db_rx(rxr);
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
bnxt_rx_cmpl_l2(bnxt_ring_t *rxr, int pb)
{
	bnxt_t *bp = rxr->bp;
	mblk_t *mp = NULL;
	mblk_t *tail = NULL;
	uint32_t cidx = rxr->cp_cidx;
	bool v_bit = rxr->cp_v_bit;
	int total = 0;
	int count = 0;

	/* Consume as many RX completions as possible */
	for (;;) {
		rx_pkt_cmpl_t *rcp;
		rx_pkt_cmpl_hi_t *rcph;
		bnxt_buf_t *buf;
		mblk_t *nmp;
		uint32_t errors;
		uint32_t cksum_flags = 0;
		uint32_t flags2;

		/* First BD */
		rcp = &((rx_pkt_cmpl_t *)rxr->cp_descs.va)[rxr->cp_cidx];

		if (!CMPL_VALID(rcp, rxr->cp_v_bit) ||
		    (LE_16(rcp->flags_type) & CMPL_BASE_TYPE_MASK) !=
		    CMPL_BASE_TYPE_RX_L2) {
			rxr->cp_cidx = cidx;
			rxr->cp_v_bit = v_bit;
			break;
		}

		/* Check if new packet fits in the poll limit */
		if (pb != 0 && total + rcp->len > pb) {
			rxr->cp_cidx = cidx;
			rxr->cp_v_bit = v_bit;
			break;
		}

		buf = bnxt_buf_find(bp, rcp->opaque);
		VERIFY3P(buf, !=, NULL);
		count++;

		/* Second BD */
		RING_NEXT_CIDX_V(rxr);
		rcph = &((rx_pkt_cmpl_hi_t *)rxr->cp_descs.va)[rxr->cp_cidx];
		errors = (LE_16(rcph->errors_v2) & RX_PKT_CMPL_ERRORS_MASK) >>
		    RX_PKT_CMPL_ERRORS_SFT;
		flags2 = LE_32(rcph->flags2);

		if (errors != 0) {
			/* TODO update error counters */
			bnxt_error(bp, "!rxr%d errors %u", rxr->idx, errors);
			list_insert_tail(&rxr->bufs_free, buf);
			goto next;
		}

		/* Get new mblk */
		nmp = desballoc((unsigned char *)buf->dma.va, rcp->len, 0,
		    &buf->rx_frtn);
		nmp->b_wptr = nmp->b_rptr + rcp->len;
		/* Chain up with previous one */
		if (mp == NULL) {
			mp = nmp;
			tail = mp;
		} else {
			tail->b_next = nmp;
			tail = nmp;
		}
		total += rcp->len;

		/* Get checksum flags */
		if (flags2 & RX_PKT_CMPL_FLAGS2_IP_CS_CALC)
			cksum_flags |= HCK_IPV4_HDRCKSUM_OK;
		if (flags2 & (RX_PKT_CMPL_FLAGS2_L4_CS_CALC |
		    RX_PKT_CMPL_FLAGS2_T_L4_CS_CALC))
			cksum_flags |= HCK_FULLCKSUM_OK;
		mac_hcksum_set(mp, 0, 0, 0, 0, cksum_flags);
next:
		cidx = rxr->cp_cidx;
		v_bit = rxr->cp_v_bit;
		RING_NEXT_CIDX_V(rxr);
	}

	rxr->ndesc_free += count;

	return (mp);
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

	BNXT_DMA_SYNC(rxr->descs, DDI_DMA_SYNC_FORKERNEL);
again:
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
		bnxt_rx_refill(rxr);
		break;
	default:
		bnxt_error(bp, "!%s: unexpected completion type %u",
		    __func__, type);
		break;
	}

	return (mp);
}

uint_t
bnxt_rx_intr(caddr_t arg1, caddr_t arg2 __unused)
{
	bnxt_ring_t *rxr = (bnxt_ring_t *)arg1;
	bnxt_t *bp = rxr->bp;
	mblk_t *mp;

	mutex_enter(&rxr->lock);
	if (!rxr->active) {
		mutex_exit(&rxr->lock);
		return (DDI_INTR_UNCLAIMED);
	}
	bnxt_db_cp(rxr, false);
	mp = bnxt_rx_cmpl(rxr, 0);
	bnxt_db_cp(rxr, true);
	mutex_exit(&rxr->lock);

	if (mp != NULL)
		mac_rx_ring(bp->mh, rxr->rh, mp, rxr->rx_gen);

	return (DDI_INTR_CLAIMED);
}

mblk_t *
bnxt_ring_rx(void *arg, int pb)
{
	bnxt_ring_t *rxr = (bnxt_ring_t *)arg;
	mblk_t *mp;

	mutex_enter(&rxr->lock);
	mp = bnxt_rx_cmpl(rxr, pb);
	mutex_exit(&rxr->lock);

	return (mp);
}

/*
 * TX path modeled after the igc driver with the exception that we do not use
 * interrupt notifications and rather poll the TX CP ring.  This also allows for
 * more RX/TX rings when MSI-X vectors are limited -- cannot share the vector
 * between the two as we need to disable the interrupts for RX polling that MAC
 * does and there is no way to mask just the RX path.
 *
 * TODO:
 * - dma binding instead of copying if over the threshold (512?)
 * - turn the ring blocking defines below into tunables
 * - turn the poll interval into tunable (?)
 * - investigate TSO
 * - investigate HW checksumming
 * - implement counters and/or check if stats provided by hardware are enough
 */

/* These are taken from igc which took them from other Intel drivers */
#define	BNXT_TX_NOTIFY_MIN	8
#define	BNXT_TX_FREE_MIN	32
#define	BNXT_TX_GAP		4

bnxt_buf_t *
bnxt_tx_buf_loan(bnxt_ring_t *txr)
{
	bnxt_buf_t *buf;

	mutex_enter(&txr->lock);
	buf = list_remove_head(&txr->bufs_free);
	if (buf == NULL) {
		/* TODO update counters */
	}
	mutex_exit(&txr->lock);

	return (buf);
}

static void
bnxt_tx_buf_reset(bnxt_buf_t *buf)
{
	if (buf->tx_bind)
		(void) ddi_dma_unbind_handle(buf->tx_bind_hdl);
	buf->tx_bind = false;
	buf->tx_len = 0;
	buf->tx_mp = NULL;
}

void
bnxt_tx_recycle(void *arg)
{
	bnxt_ring_t *txr = arg;
	bnxt_t *bp = txr->bp;
	list_t to_free;
	size_t ndesc = 0;
	uint32_t cidx;
	bool v_bit;
	bool notify = false;

	list_create(&to_free, sizeof (bnxt_buf_t), offsetof(bnxt_buf_t, node));

	for (;;) {
		mutex_enter(&txr->lock);
		if (!txr->active) {
			mutex_exit(&txr->lock);
			break;
		}
		/*
		 * XXX timeout is pretty arbitrary, check if bigger one (so that
		 * there is less load while idling) behaves the same.
		 */
		(void) cv_reltimedwait(&txr->tx_recycle, &txr->lock,
		    drv_usectohz(1000), TR_MILLISEC);

		BNXT_DMA_SYNC(txr->cp_descs, DDI_DMA_SYNC_FORKERNEL);
		for (;;) {
			bnxt_buf_t *buf;
			tx_cmpl_t *tcp;
			uint16_t type;
			uint16_t errors;

			cidx = txr->cp_cidx;
			v_bit = txr->cp_v_bit;

			RING_NEXT_CIDX_V(txr);
			tcp = &((tx_cmpl_t *)txr->cp_descs.va)[txr->cp_cidx];

			if (!CMPL_VALID(tcp, txr->cp_v_bit)) {
				txr->cp_cidx = cidx;
				txr->cp_v_bit = v_bit;
				break;
			}

			type = LE_16(tcp->flags_type) & CMPL_BASE_TYPE_MASK;
			switch (type) {
			case CMPL_BASE_TYPE_HWRM_DONE:
				/*
				 * Intermittently seen on rings startup, nothing
				 * to do.
				 */
				break;
			case CMPL_BASE_TYPE_TX_L2:
				errors = (LE_16(tcp->errors_v) &
				    TX_CMPL_ERRORS_BUFFER_ERROR_MASK) >>
				    TX_CMPL_ERRORS_BUFFER_ERROR_SFT;
				if (errors != 0) {
					/* TODO update error counters */
					bnxt_error(bp, "!txr%d errors %u",
					    txr->idx, errors);
				}
				/* Get first buffer */
				buf = bnxt_buf_find(bp, tcp->opaque);
				VERIFY3P(buf, !=, NULL);
				if (buf->tx_mp != NULL)
					freemsgchain(buf->tx_mp);
				for (; buf != NULL; buf = buf->tx_next) {
					VERIFY0(list_link_active(&buf->node));
					ndesc++;
					bnxt_tx_buf_reset(buf);
					list_insert_tail(&to_free, buf);
				}
				break;
			default:
				bnxt_error(bp,
				    "!%s: unexpected completion type %u",
				    __func__, type);
				continue;
			}
		}

		txr->ndesc_free += ndesc;
		list_move_tail(&txr->bufs_free, &to_free);
		/* Unblock the ring if it has enough free descriptors */
		if (txr->tx_blocked &&
		    txr->ndesc_free > BNXT_TX_NOTIFY_MIN) {
			txr->tx_blocked = false;
			notify = true;
		}
		/* Update consumer index */
		bnxt_db_cp(txr, false);
		mutex_exit(&txr->lock);

		if (notify)
			mac_tx_ring_update(txr->bp->mh, txr->rh);
	}

	list_destroy(&to_free);
}

static const uint16_t bnxt_tx_lhint[] = {
	TX_BD_SHORT_FLAGS_LHINT_LT512,
	TX_BD_SHORT_FLAGS_LHINT_LT1K,
	TX_BD_SHORT_FLAGS_LHINT_LT2K,
	TX_BD_SHORT_FLAGS_LHINT_LT2K,
	TX_BD_SHORT_FLAGS_LHINT_GTE2K,
};

typedef struct bnxt_tx {
	list_t		bufs;
	bnxt_buf_t	*cur_buf;
	size_t		buf_rem;
	uint8_t		ndesc;
	size_t		len;
} bnxt_tx_t;

int
bnxt_tx_copy(bnxt_ring_t *txr, bnxt_tx_t *tx, mblk_t *mp)
{
	bnxt_buf_t *cur_buf;
	size_t len = MBLKL(mp);
	size_t off = 0;

	while (len > 0) {
		const void *src;
		void *dest;
		size_t to_copy;

		cur_buf = tx->cur_buf;
		if (tx->cur_buf != NULL &&
		    (tx->cur_buf->tx_bind || tx->buf_rem == 0)) {
			tx->cur_buf = NULL;
			tx->buf_rem = 0;
		}
		if (cur_buf == NULL) {
			tx->cur_buf = bnxt_tx_buf_loan(txr);
			if (tx->cur_buf == NULL)
				return (1);
			if (cur_buf != NULL) {
				/* Chain up buffers */
				cur_buf->tx_next = tx->cur_buf;
			} else {
				/* Store mblk pointer in first buffer */
				tx->cur_buf->tx_mp = mp;
			}
			list_insert_tail(&tx->bufs, tx->cur_buf);
			tx->buf_rem = tx->cur_buf->dma.len;
			tx->ndesc++;
			tx->cur_buf->tx_bind = false;
		}

		to_copy = min(len, tx->buf_rem);
		src = mp->b_rptr + off;
		dest = tx->cur_buf->dma.va + tx->cur_buf->tx_len;
		memcpy(dest, src, to_copy);

		/* Calculate packet length used for setting the size hint */
		tx->len += to_copy;

		tx->buf_rem -= to_copy;
		tx->cur_buf->tx_len += to_copy;
		len -= to_copy;
		off += to_copy;
	}

	return (0);
}

static int
bnxt_tx_write(bnxt_ring_t *txr, bnxt_tx_t *tx)
{
	bnxt_buf_t *cur_buf;
	tx_bd_long_t *first_desc, *cur_desc;
	tx_bd_long_hi_t *first_desc_hi;
	uint16_t pidx;

	mutex_enter(&txr->lock);
	if (tx->ndesc + BNXT_TX_GAP > txr->ndesc_free) {
		cv_signal(&txr->tx_recycle);
		if (tx->ndesc + BNXT_TX_GAP > txr->ndesc_free) {
			mutex_exit(&txr->lock);
			return (1);
		}
	}

	txr->ndesc_free -= tx->ndesc;

	pidx = txr->pidx;
	/*
	 * Setting cur_desc here to silence gcc which otherwise warns about
	 * cur_desc possibly being uninitialized below.  This is false positive
	 * as would not even get here if we do not have any buffers to process.
	 */
	first_desc = cur_desc = &((tx_bd_long_t *)txr->descs.va)[pidx];
	while ((cur_buf = list_remove_head(&tx->bufs)) != NULL) {
		cur_desc = &((tx_bd_long_t *)txr->descs.va)[pidx];

		if (cur_desc == first_desc) {
			cur_desc->flags_type =
			    LE_32(TX_BD_LONG_TYPE_TX_BD_LONG);
			cur_desc->flags_type |= LE_32((tx->ndesc <<
			    TX_BD_LONG_FLAGS_BD_CNT_SFT) &
			    TX_BD_LONG_FLAGS_BD_CNT_MASK);
			/* Size hint */
			if (tx->len >= 2048) {
				cur_desc->flags_type |=
				    TX_BD_LONG_FLAGS_LHINT_GTE2K;
			} else {
				cur_desc->flags_type |=
				    bnxt_tx_lhint[tx->len >> 9];
			}
			cur_desc->opaque = cur_buf->opaque;

			/* Fill tx_bd_long_hi descriptor */
			pidx = RING_NEXT_PIDX(txr, pidx);
			first_desc_hi =
			    &((tx_bd_long_hi_t *)txr->descs.va)[pidx];
			/* FIXME actually fill the descriptor */
			memset(first_desc_hi, 0, sizeof (*first_desc_hi));
		} else {
			cur_desc->flags_type =
			    LE_32(TX_BD_SHORT_TYPE_TX_BD_SHORT);
		}

		cur_desc->addr = LE_64(cur_buf->dma.pa);
		cur_desc->len = LE_16(cur_buf->tx_len);

		pidx = RING_NEXT_PIDX(txr, pidx);
	}
	/* Mark last descriptor */
	cur_desc->flags_type |= LE_32(TX_BD_LONG_FLAGS_PACKET_END);
	txr->pidx = pidx;
	BNXT_DMA_SYNC(txr->descs, DDI_DMA_SYNC_FORDEV);
	bnxt_db_tx(txr);
	mutex_exit(&txr->lock);

	return (0);
}

mblk_t *
bnxt_ring_tx(void *arg, mblk_t *mp)
{
	bnxt_ring_t *txr = arg;
	bnxt_tx_t tx = { 0 };
	mblk_t *cur_mp;

	ASSERT3P(mp->b_next, ==, NULL);

	mutex_enter(&txr->lock);
	if (!txr->active) {
		mutex_exit(&txr->lock);
		return (mp);
	}

	/* Recycle the ring if needed */
	if (txr->ndesc_free < BNXT_TX_FREE_MIN)
		cv_signal(&txr->tx_recycle);
	if (txr->ndesc_free < BNXT_TX_NOTIFY_MIN) {
		/* TODO ring full counter */
		txr->tx_blocked = true;
		mutex_exit(&txr->lock);
		return (mp);
	}
	mutex_exit(&txr->lock);

	list_create(&tx.bufs, sizeof (bnxt_buf_t), offsetof(bnxt_buf_t, node));
	/* Account for tx_bd_long_hi */
	tx.ndesc = 1;
	for (cur_mp = mp; cur_mp != NULL; cur_mp = cur_mp->b_cont) {
		size_t len = MBLKL(cur_mp);

		if (len == 0)
			continue;

		/* TODO check bind threshold */
		if (bnxt_tx_copy(txr, &tx, cur_mp) != 0)
			goto fail;
	}

	if (bnxt_tx_write(txr, &tx) != 0)
		goto fail;

	list_destroy(&tx.bufs);
	return (NULL);
fail:
	/* We ran out of buffers or descriptors, clean up and return mp */
	for (bnxt_buf_t *buf = list_head(&tx.bufs); buf != NULL;
	    buf = list_next(&tx.bufs, buf))
		bnxt_tx_buf_reset(buf);

	mutex_enter(&txr->lock);
	list_move_tail(&txr->bufs_free, &tx.bufs);
	txr->tx_blocked = true;
	mutex_exit(&txr->lock);
	list_destroy(&tx.bufs);
	return (mp);
}
