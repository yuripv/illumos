/*
 * Copyright 2023 Tintri by DDN, Inc. All rights reserved.
 */

#include <sys/types.h>
#include <sys/cmn_err.h>
#include <sys/sysmacros.h>
#include <sys/sunddi.h>
#include <sys/ntb_svc.h>
#include "psxntb_impl.h"

ddi_device_acc_attr_t	pntb_acc_attr = {
	DDI_DEVICE_ATTR_V1,
	DDI_STRUCTURE_LE_ACC,
	DDI_STRICTORDER_ACC,
	DDI_DEFAULT_ACC
};

ddi_dma_attr_t pntb_attr_tmpl = {
	DMA_ATTR_V0,		/* version number */
	0x0,			/* low dma address */
	0xffffffffffffffffull,	/* high dma address (64 bit max) */
	0xffffffffffffffffull,	/* count max */
	0,			/* alignment */
	0xffffffff,		/* burst sizes */
	0,			/* min xfer */
	0xffffffffffffull,	/* max xfer */
	0,			/* segment boundary */
	0x7fffffff,		/* effectively no scatter gather limits */
	1,			/* granularity */
	0			/* flags */
};

typedef void (*rep8_t)(ddi_acc_handle_t, uint8_t *, uint8_t *, size_t, uint_t);
typedef void (*rep64_t)(ddi_acc_handle_t, uint64_t *, uint64_t *, size_t,
    uint_t);

/*
 * Get the current ntb-dma-window and return it. If it is not
 * set, get the current segment range and return the number of
 * the segmented window.
 */
static psx_bar_t *
psxntb_get_window(psxntb_t *pntb, dev_info_t *dip, int *segp, int *cntp)
{
	psx_bar_t	*win = NULL;
	int		window = -1;

	if (ntb_get_dma_window_by_dev(dip, &window) == DDI_SUCCESS &&
	    window >= pntb->pntb_wcnt)
		return (NULL);

	if (window >= 0)
		return (pntb->pntb_window[pntb->pntb_win_base + window]);

	if (ntb_get_bind_limits_by_dev(dip, segp, cntp) == DDI_SUCCESS ||
	    ntb_get_dma_segments_by_dev(dip, segp, cntp) == DDI_SUCCESS)
		win = pntb->pntb_seg_window;

	return (win);
}

/*
 * get_dma_attr function of ntb_drvr_ops.
 * Returns the required dma_attr and acc_attr to be used to
 * allocate a DMA object to be used for operations in the direction
 * of "dir".
 */
static int
psxntb_get_dma_attr(void *pvt, dev_info_t *dip, xfer_dir_t dir,
    ddi_dma_attr_t *attr, ddi_device_acc_attr_t *acc_attr, size_t *len)
{
	psxntb_t	*pntb = pvt;
	psx_bar_t	*win;
	int		seg, cnt;

	if (psxntb_peer_is_faulted(pntb))
		return (DDI_FAILURE);

	cnt = 1;
	win = psxntb_get_window(pntb, dip, &seg, &cnt);

	if (win == NULL)
		return (DDI_FAILURE);

	if (win->pb_owner != pntb->pntb_dip && win->pb_owner != dip)
		return (DDI_FAILURE);

	/*
	 * NTB_GET memory will be subject to ddi_dma_sync() calls
	 * which will check error status, so for those use attributes
	 * configured for FMA.
	 */
	if (attr != NULL) {
		if (dir == NTB_GET) {
			*attr = pntb->pntb_dma_attr;
			attr->dma_attr_seg = win->pb_seg_size - 1;
			attr->dma_attr_align = win->pb_seg_size;
			attr->dma_attr_minxfer = 1U << PNTB_MIN_SIZE;
			attr->dma_attr_sgllen = cnt;
			attr->dma_attr_flags |= PNTB_DMA_PEER;
		} else {
			*attr = pntb_attr_tmpl;
			attr->dma_attr_seg = PAGEOFFSET;
			attr->dma_attr_align = PAGESIZE;
			attr->dma_attr_minxfer = PAGESIZE;
		}
	}

	if (acc_attr != NULL)
		*acc_attr = dir == NTB_GET ? pntb->pntb_acc_attr :
		    pntb_acc_attr;

	if (len != NULL)
		*len = win->pb_seg_size * cnt;

	return (DDI_SUCCESS);
}

/*
 * dma_mem_alloc function of ntb_drvr_ops.
 */
static int
psxntb_dma_mem_alloc(void *pvt, ddi_dma_handle_t hdl, size_t length,
    ddi_device_acc_attr_t *accattrp, uint_t flags, int (*waitfp)(caddr_t),
    caddr_t arg, caddr_t *kaddrp, size_t *real_length, void **handlep)
{
	_NOTE(ARGUNUSED(pvt));

	return (ddi_dma_mem_alloc(hdl, length, accattrp, flags, waitfp, arg,
	    kaddrp, real_length, (ddi_acc_handle_t *)handlep));
}

/*
 * dma_mem_free function of ntb_drvr_ops.
 */
static void
psxntb_dma_mem_free(void *pvt, void **handlep)
{
	_NOTE(ARGUNUSED(pvt));

	ddi_dma_mem_free((ddi_acc_handle_t *)handlep);
}

/*
 * get_link_cookies function of ntb_drvr_ops.
 */
static int
psxntb_get_link_cookies(void *pvt, ddi_dma_handle_t hdl, ddi_dma_cookie_t **cpp,
    int *ccnt)
{
	psxntb_t		*pntb = pvt;
	ddi_dma_impl_t		*i_hdl = (ddi_dma_impl_t *)hdl;
	ddi_dma_attr_t		*dma_attr = &i_hdl->dmai_attr;
	psx_dma_handle_t	*pntb_h = i_hdl->dmai_driver_private;
	psx_bar_t		*win = pntb_h->pdh_window;
	size_t			size, seg_size;
	uint64_t		start;
	int			i, seg;

	if (psxntb_peer_is_faulted(pntb))
		return (DDI_FAILURE);

	if ((dma_attr->dma_attr_flags & PNTB_DMA_PEER) != 0) {
		/*
		 * This is a handle for DMA on the peer side,
		 * all the cookies have been saved - just return those.
		 */
		size = sizeof (ddi_dma_cookie_t) * pntb_h->pdh_lut_cookie_cnt;

		*cpp = kmem_alloc(size, KM_SLEEP);
		*ccnt = pntb_h->pdh_lut_cookie_cnt;

		bcopy(pntb_h->pdh_lut_cookies, *cpp, size);

		return (DDI_SUCCESS);
	}

	/*
	 * A local handle. Return one cookie per segment.
	 * The windows which are not segmented will only have one segment.
	 */
	size = sizeof (ddi_dma_cookie_t) * pntb_h->pdh_seg_cnt;
	*cpp = kmem_alloc(size, KM_SLEEP);
	*ccnt = pntb_h->pdh_seg_cnt;

	seg_size = pntb_h->pdh_seg_size;
	seg = win->pb_lut_child + pntb_h->pdh_seg;
	start = pntb_h->pdh_window->pb_paddr + seg_size * seg;

	for (i = 0; i < pntb_h->pdh_seg_cnt; i++) {
		(*cpp)[i].dmac_laddress = start + i * seg_size;
		(*cpp)[i].dmac_size = seg_size;
		(*cpp)[i].dmac_type = 0;
	}

	return (DDI_SUCCESS);
}

/*
 * set_segment function of ntb_drvr_ops.
 */
static int
psxntb_set_segment(void *pvt, ddi_dma_handle_t hdl, int seg, uint64_t base)
{
	psxntb_t		*pntb = pvt;
	ddi_dma_impl_t		*i_hdl = (ddi_dma_impl_t *)hdl;
	ddi_dma_attr_t		*dma_attr = &i_hdl->dmai_attr;
	psx_dma_handle_t	*pntb_h = i_hdl->dmai_driver_private;
	psx_bar_t		*win = pntb_h->pdh_window;
	uint_t			lut;

	if (psxntb_peer_is_faulted(pntb))
		return (DDI_FAILURE);

	/* Operation is only applicable to peer DMA. */
	if ((dma_attr->dma_attr_flags & PNTB_DMA_PEER) == 0)
		return (DDI_FAILURE);

	/* Verify alignment. */
	if ((base & (pntb_h->pdh_seg_size - 1)) != 0)
		return (DDI_FAILURE);

	/* Verify segment is in range. */
	if (seg < pntb_h->pdh_seg ||
	    seg >= (pntb_h->pdh_seg + pntb_h->pdh_seg_cnt))
		return (DDI_FAILURE);

	/* Work out the actual entry in the LUT. */
	lut = win->pb_lut_child + seg;

	return (psxntb_set_lut_window(pntb, lut, pntb->pntb_partition, base));
}

/*
 * set_window function of ntb_drvr_ops.
 * Set the bar setup register to the physical address given in
 * the dma cookie, then any data coming across the NTB via this window
 * will end up in the memory described by the cookie.
 */
static int
psxntb_set_window(void *pvt, dev_info_t *cdip, int window, ddi_dma_cookie_t *cp)
{
	psxntb_t	*pntb = pvt;
	psx_bar_t	*win;
	uint64_t	paddr;
	int		rv;

	if (psxntb_peer_is_faulted(pntb))
		return (DDI_FAILURE);

	if (window < 0 || window >= pntb->pntb_wcnt)
		return (DDI_FAILURE);

	win = pntb->pntb_window[pntb->pntb_win_base + window];

	if (cp == NULL) {
		/*
		 * A NULL cp means we should remove the translation
		 * and close the window.
		 */
		psxntb_release_window(pntb, win);

		return (psxntb_clear_direct_window(pntb, win,
		    pntb->pntb_partition));
	}

	if (!psxntb_claim_window(pntb, win, cdip))
		return (DDI_FAILURE);

	/*
	 * The translations rely on the address being aligned to the size.
	 */
	paddr = cp->dmac_laddress & ~(win->pb_size - 1);

	rv = psxntb_set_direct_window(pntb, win, pntb->pntb_partition, paddr);
	if (rv != DDI_SUCCESS)
		psxntb_release_window(pntb, win);

	return (rv);
}

/*
 * Convert a DMA copy operation which covers an address range into a
 * sequence of ddi_rep_* operations to copy the data as efficiently as
 * possible. The operation happens in three steps:
 *	1. Use ddi_rep8 operations to align to 64 bits.
 *	2. Use ddi_rep64 operations as far as possible.
 *	3. Use ddi_rep8 to mop up from the last 64 bit alignment to the end.
 *
 * rep8 and rep64 arguments are the "rep" functions to use and are
 * either ddi_rep_getxx or ddi_rep_putxx.
 */
static int
psxntb_repcopy(psxntb_t *pntb, ddi_acc_handle_t handle, caddr_t host_addr,
    caddr_t dev_addr, off_t offset, size_t len, rep8_t rep8, rep64_t rep64)
{
	_NOTE(ARGUNUSED(pntb));
	size_t	count8, count64;

	/*
	 * 8 bit copies to align to 64 bit.
	 */
	count8 = sizeof (uint64_t) - (offset & (sizeof (uint64_t) - 1));
	if (count8 < sizeof (uint64_t)) {
		if (count8 > len)
			count8 = len;

		/* The copy .. */
		rep8(handle, (uint8_t *)(host_addr + offset),
		    (uint8_t *)(dev_addr + offset), count8,
		    DDI_DEV_AUTOINCR);

		offset += count8;
		len -= count8;
	}

	if (len == 0)
		goto done;

	/*
	 * Now 64 bit access ....
	 */
	count64 = len / sizeof (uint64_t);
	if (count64 > 0) {
		rep64(handle, (uint64_t *)(uintptr_t)(host_addr + offset),
		    (uint64_t *)(uintptr_t)(dev_addr + offset), count64,
		    DDI_DEV_AUTOINCR);

		offset += count64 * sizeof (uint64_t);
		len -= count64 * sizeof (uint64_t);
	}

	/*
	 * ... and any leftover unaligned bytes using 8 bit copies.
	 */
	if (len > 0)
		rep8(handle, (uint8_t *)(host_addr + offset),
		    (uint8_t *)(dev_addr + offset), len, DDI_DEV_AUTOINCR);

done:
	/*
	 * This is an SFENCE instruction, which will ensure all the
	 * write-combining buffers are flushed.
	 */
	membar_producer();

	return (psxntb_check_acc_handle(pntb, handle));
}

/*
 * put function of ntb_drvr_ops.
 * Moves data onto the bridge for "hdl".
 */
static int
psxntb_dma_put(void *pvt, dev_info_t *cdip, ddi_dma_handle_t hdl, offset_t off,
    size_t len)
{
	_NOTE(ARGUNUSED(cdip));
	psxntb_t		*pntb = pvt;
	ddi_dma_impl_t		*i_hdl = (ddi_dma_impl_t *)hdl;
	ddi_dma_attr_t		*dma_attr = &i_hdl->dmai_attr;
	psx_dma_handle_t	*pntb_h = i_hdl->dmai_driver_private;
	psx_bar_t		*win = pntb_h->pdh_window;
	size_t			segs_size, len_max;
	int			seg;

	if (psxntb_peer_is_faulted(pntb) || !psxntb_peer_is_up(pntb))
		return (DDI_FAILURE);

	/*
	 * We should only ever do a "put" to the primary side DMA.
	 */
	if ((dma_attr->dma_attr_flags & PNTB_DMA_PEER) != 0)
		return (DDI_FAILURE);

	/*
	 * off == 0 and len == 0 means put everything. But if just len
	 * is zero, then there is nothing to do.
	 */
	if (off != 0 && len == 0)
		return (DDI_SUCCESS);

	/*
	 * The maximum value len can be is the smallest of the DMA
	 * allocated (idh_dma_len), and the segments reserved in the window.
	 */
	segs_size = pntb_h->pdh_seg_size * pntb_h->pdh_seg_cnt;
	len_max = MIN(pntb_h->pdh_dma_len, segs_size);
	seg = win->pb_lut_child + pntb_h->pdh_seg;

	/*
	 * Some sanity checks. If "off" is beyond the length of either the
	 * source or destination, then there is nothing to do.
	 */
	if (off >= len_max)
		return (DDI_SUCCESS);

	/*
	 * truncate [off .. off + len) to fit in len_max.
	 */
	if (len == 0 || (len + off) > len_max)
		len = len_max - off;

	ntb_kstat_accumulate_put(pntb->pntb_kdata, len);

	return (psxntb_repcopy(pntb, win->pb_hdl, pntb_h->pdh_dma_addr,
	    win->pb_vaddr + seg * pntb_h->pdh_seg_size,
	    off, len, ddi_rep_put8, ddi_rep_put64));
}

/*
 * vput function of ntb_drvr_ops.
 * Move data onto the bridge from a "vaddr".
 * Uses non-temporal copy.
 */
static int
psxntb_vput(void *pvt, dev_info_t *cdip, ddi_dma_handle_t hdl, caddr_t vaddr,
    offset_t off, size_t len)
{
	_NOTE(ARGUNUSED(cdip));
	psxntb_t		*pntb = pvt;
	ddi_dma_impl_t		*i_hdl = (ddi_dma_impl_t *)hdl;
	ddi_dma_attr_t		*dma_attr = &i_hdl->dmai_attr;
	psx_dma_handle_t	*pntb_h = i_hdl->dmai_driver_private;
	psx_bar_t		*win = pntb_h->pdh_window;
	size_t			segs_size;
	int			seg;

	if (psxntb_peer_is_faulted(pntb) || !psxntb_peer_is_up(pntb))
		return (DDI_FAILURE);

	/*
	 * We should only ever do a "put" to the primary side DMA.
	 */
	if ((dma_attr->dma_attr_flags & PNTB_DMA_PEER) != 0)
		return (DDI_FAILURE);

	if (len == 0)
		return (DDI_SUCCESS);

	segs_size = pntb_h->pdh_seg_size * pntb_h->pdh_seg_cnt;
	if ((off + len) > segs_size)
		len = segs_size - off;

	seg = win->pb_lut_child + pntb_h->pdh_seg;

	if (kcopy_nta(vaddr, win->pb_vaddr +
	    seg * pntb_h->pdh_seg_size + off, len, 0) != 0)
		return (DDI_FAILURE);

	/*
	 * This is an SFENCE instruction, which will ensure all the
	 * write-combining buffers are flushed.
	 */
	membar_producer();

	ntb_kstat_accumulate_put(pntb->pntb_kdata, len);

	return (psxntb_check_win_acc_handle(pntb, win));
}

/*
 * Get function of ntb_drvr_ops.
 * Move data from the peer side of the bridge into DMA described * by "hdl".
 */
static int
psxntb_dma_get(void *pvt, dev_info_t *cdip, ddi_dma_handle_t hdl, offset_t off,
    size_t len)
{
	_NOTE(ARGUNUSED(cdip));
	psxntb_t		*pntb = pvt;
	ddi_dma_impl_t		*i_hdl = (ddi_dma_impl_t *)hdl;
	ddi_dma_attr_t		*dma_attr = &i_hdl->dmai_attr;
	psx_dma_handle_t	*pntb_h = i_hdl->dmai_driver_private;
	psx_bar_t		*win;
	int			seg;

	if (psxntb_peer_is_faulted(pntb))
		return (DDI_FAILURE);

	/*
	 * We should only ever do a "get" from the secondary side DMA.
	 */
	if ((dma_attr->dma_attr_flags & PNTB_DMA_PEER) == 0)
		return (DDI_FAILURE);

	/*
	 * This is a handle for DMA on the peer side,
	 * the client has direct access to the vaddr of the
	 * memory mapped. So no copy is required,
	 * all we may need to do is ddi_dma_sync().
	 */

	win = pntb_h->pdh_window;

	/*
	 * Adjust the offset based on which range of segments
	 * are assigned to this handle. When the handle is
	 * for a whole window (WINDOW_HDL), then there would
	 * only be one segment starting at zero. Also take into
	 * consideration that the user data may be offset from the
	 * window's base address.
	 */
	seg = win->pb_lut_child + pntb_h->pdh_seg;
	off += seg * pntb_h->pdh_seg_size;

	if (ddi_dma_sync(hdl, off, len, DDI_DMA_SYNC_FORKERNEL) !=
	    DDI_SUCCESS)
		return (DDI_FAILURE);

	return (psxntb_check_dma_handle(pntb, hdl));
}

/*
 * doorbell_ctl function of ntb_drvr_ops.
 */
static int
psxntb_doorbell_ctl(void *pvt, dev_info_t *dip, db_ctl_t ctl, uint_t num,
    void *arg1, void *arg2)
{
	psxntb_t	*pntb = pvt;

	if (psxntb_peer_is_faulted(pntb))
		return (DDI_FAILURE);

	switch (ctl) {
	case SEND_DOORBELL:
		if (num > pntb->pntb_db_high)
			return (DDI_FAILURE);

		return (psxntb_set_peer_doorbell(pntb, num));

	case SET_DOORBELL:
	case SET_FAST_DOORBELL:
		/*
		 * We don't differentiate between fast and normal doorbells.
		 * They're all handled in the interrupt context directly.
		 */
		if (arg1 == NULL)
			/* NULL function pointer */
			return (psxntb_clear_db_callback(pntb, dip, num));

		return (psxntb_set_db_callback(pntb, dip, num, (db_func_t)arg1,
		    arg2));

	case GET_DOORBELL_LIMITS:
		*(uint_t *)arg1 = 0;
		*(uint_t *)arg2 = pntb->pntb_db_high;
		return (DDI_SUCCESS);

	case GET_STATUS_DOORBELLS:
		*(uint_t *)arg1 = pntb->pntb_db_up;
		*(uint_t *)arg2 = pntb->pntb_db_down;
		return (DDI_SUCCESS);

	default:
		return (DDI_FAILURE);
	}
}

ntb_drvr_ops_t pntb_drvr_ops = {
	psxntb_get_dma_attr,
	psxntb_dma_mem_alloc,
	psxntb_dma_mem_free,
	psxntb_get_link_cookies,
	psxntb_set_segment,
	psxntb_set_window,
	psxntb_dma_put,
	psxntb_vput,
	psxntb_dma_get,
	psxntb_doorbell_ctl
};
