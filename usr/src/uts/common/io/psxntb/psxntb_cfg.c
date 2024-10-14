/*
 * Copyright 2020 Tintri by DDN. All rights reserved.
 */

#include <sys/types.h>
#include <sys/ddi.h>
#include <sys/sunddi.h>
#include <sys/sunndi.h>
#include <sys/sysmacros.h>
#include <sys/sdt.h>
#include "psxntb_impl.h"

/*
 * Move the operation control state to "state" with operation "opc".
 */
static int
psxntb_config_change_state(psxntb_t *pntb, uint_t ctl_off, uint16_t opc,
    uint16_t state)
{
	uint16_t	new_state;
	int		retries = 0;

	psxntb_put16(pntb, ctl_off + PNTB_NTB_CNTL_OPC, opc);
	do {
		/* use busy wait so we can be called during quiesce(). */
		drv_usecwait(10000);	/* 10ms */

		new_state = psxntb_get16(pntb, ctl_off +
		    PNTB_NTB_CNTL_STATUS);
	} while (new_state != state && ++retries < 50);

	if (new_state == state)
		return (DDI_SUCCESS);

	/*
	 * Timed out. Reset the OPC which will set the state back to
	 * "normal".
	 */
	psxntb_put16(pntb, ctl_off + PNTB_NTB_CNTL_OPC,
	    PNTB_NTB_CNTL_OPC_RESET);

	return (DDI_FAILURE);
}

/*
 * To configure the address look up tables or the requester id tables
 * the following steps are needed:
 * 1. Lock the partition.
 * 2. Perform the configuration operation.
 * 3. Configure the partition, and it transitions to unlocked.
 *
 * These set of states are described in Device Specification manual,
 * Section 17.9.2 under the tile "NT Partition Operation Register".
 */
static int
psxntb_config_update_nolock(psxntb_t *pntb, uint_t ctl_off,
    int (*cfg_fn)(psxntb_t *, void *), void *cfg_arg)
{
	int		rv;
	uint16_t	sts;

	/*
	 * Make sure the partition is in the expected state - "normal".
	 */
	sts = psxntb_get16(pntb, ctl_off + PNTB_NTB_CNTL_STATUS);
	if (sts != PNTB_NTB_CNTl_STATUS_NORMAL) {
		psx_warn(pntb, "!Partition in unexpected state: 0x%x", sts);
		return (DDI_FAILURE);
	}

	/*
	 * "lock" the config so we can operate on it.
	 * It must return as "locked".
	 */
	if (psxntb_config_change_state(pntb, ctl_off, PNTB_NTB_CNTL_OPC_LOCK,
	    PNTB_NTB_CNTl_STATUS_LOCKED) != DDI_SUCCESS) {
		psx_warn(pntb, "!Failed to lock partition");
		return (DDI_FAILURE);
	}

	/*
	 * Our operation ....
	 */
	rv = cfg_fn(pntb, cfg_arg);

	/*
	 * Perform the "configure" and the state should transition back
	 * to "normal".
	 */
	if (psxntb_config_change_state(pntb, ctl_off,
	    PNTB_NTB_CNTL_OPC_CONFIG, PNTB_NTB_CNTl_STATUS_NORMAL) !=
	    DDI_SUCCESS) {
		psx_warn(pntb, "!Failed to update partition configuration");
		return (DDI_FAILURE);
	}

	return (rv);
}

static int
psxntb_config_update(psxntb_t *pntb, uint_t ctl_off,
    int (*cfg_fn)(psxntb_t *, void *), void *cfg_arg)
{
	int	rv;

	/*
	 * This should be the only place where we try and lock a partition.
	 * So, if we serialise access through this function we should
	 * never encounter a locked partition. If we do so it is an
	 * error.
	 */
	sema_p(&pntb->pntb_sema);

	rv = psxntb_config_update_nolock(pntb, ctl_off, cfg_fn, cfg_arg);

	if (rv == DDI_SUCCESS)
		rv = psxntb_check_gas_acc_handle(pntb);

	sema_v(&pntb->pntb_sema);

	return (rv);
}

typedef struct {
	psx_bar_t 	*win;
	int		target;
	uint64_t	addr;
} set_direct_args_t;

/*
 * Set up a window (BAR) for direct address translation (ie no LUT).
 */
static int
psxntb_set_direct_window_cb(psxntb_t *pntb, void *arg)
{
	set_direct_args_t *args = arg;
	int		bar = args->win->pb_bar;
	ulong_t		size = args->win->pb_size;
	uint_t		ctl_off, bar_off;
	uint32_t	reg;
	int		i;

	ctl_off = args->target == pntb->pntb_partition ?
	    pntb->pntb_peer_ctl_off : pntb->pntb_ctl_off;

	bar_off = PNTB_NTB_CNTL_BAR_SETUP(bar);

	reg = psxntb_get32(pntb, ctl_off + bar_off);
	if ((reg & PNTB_NTB_CNTL_BAR_VALID) == 0)
		return (DDI_FAILURE);

	if (args->addr == -1) {
		/*
		 * Clear the settings for the window.
		 */
		for (i = 0; i < 16; i += 4)
			psxntb_put32(pntb, ctl_off + bar_off + i, 0);

		return (DDI_SUCCESS);
	}

	reg &= ~PNTB_NTB_CNTL_BAR_LUT;
	reg |= PNTB_NTB_CNTL_BAR_DIRECT;
	psxntb_put32(pntb, ctl_off + bar_off, reg);

	reg = PNTB_NTB_CNTL_BAR_DIR_POS(highbit(size) - 1) |
	    PNTB_NTB_CNTL_BAR_DIR_WIN_SIZE(size);
	psxntb_put32(pntb, ctl_off + bar_off + 4, reg);

	reg = PNTB_NTB_CNTL_BAR_DIR_TPART(args->target) |
	    PNTB_NTB_CNTL_BAR_DIR_ADDRL(args->addr);
	psxntb_put32(pntb, ctl_off + bar_off + 8, reg);

	reg = PNTB_NTB_CNTL_BAR_DIR_ADDRH(args->addr);
	psxntb_put32(pntb, ctl_off + bar_off + 12, reg);

	reg = PNTB_NTB_CNTL_BAR_EXT_WIN_SIZE(size);
	psxntb_put32(pntb, ctl_off + PNTB_NTB_CNTL_BAR_EXT(bar), reg);

	return (DDI_SUCCESS);
}

int
psxntb_set_direct_window(psxntb_t *pntb, psx_bar_t *win, int target,
    uint64_t addr)
{
	uint_t			ctl_off;
	set_direct_args_t	args;

	ctl_off = target == pntb->pntb_partition ?
	    pntb->pntb_peer_ctl_off : pntb->pntb_ctl_off;

	args.target = target;
	args.win = win;
	args.addr = addr;

	return (psxntb_config_update(pntb, ctl_off, psxntb_set_direct_window_cb,
	    &args));
}

int
psxntb_clear_direct_window(psxntb_t *pntb, psx_bar_t *win, int target)
{
	return (psxntb_set_direct_window(pntb, win, target, -1));
}

typedef struct {
	int 		index;
	int		target;
	uint64_t	addr;
} set_lut_args_t;

/*
 * Configure an address translation entry in the LUT.
 */
static int
psxntb_set_lut_window_cb(psxntb_t *pntb, void *arg)
{
	set_lut_args_t	*args = arg;
	uint_t		ctl_off, lut_off;
	uint32_t	reg;

	ctl_off = args->target == pntb->pntb_partition ?
	    pntb->pntb_peer_ctl_off : pntb->pntb_ctl_off;

	lut_off = PNTB_NTB_CNTL_LUT(args->index);

	if (args->addr == -1) {
		/* Clear the LUT entry. */
		psxntb_put32(pntb, ctl_off + lut_off, 0);
		psxntb_put32(pntb, ctl_off + lut_off + 4, 0);

		return (DDI_SUCCESS);
	}

	reg = PNTB_NTB_CNTL_LUT_ENABLE | PNTB_NTB_CNTL_LUT_DPART(args->target) |
	    PNTB_NTB_CNTL_LUT_ADDRL(args->addr);
	psxntb_put32(pntb, ctl_off + lut_off, reg);

	reg = PNTB_NTB_CNTL_LUT_ADDRH(args->addr);
	psxntb_put32(pntb, ctl_off + lut_off + 4, reg);

	return (DDI_SUCCESS);
}

int
psxntb_set_lut_window(psxntb_t *pntb, int index, int target, uint64_t addr)
{
	uint_t		ctl_off;
	set_lut_args_t	args;

	ctl_off = target == pntb->pntb_partition ?
	    pntb->pntb_peer_ctl_off : pntb->pntb_ctl_off;

	args.target = target;
	args.index = index;
	args.addr = addr;

	return (psxntb_config_update(pntb, ctl_off, psxntb_set_lut_window_cb,
	    &args));
}

int
psxntb_clear_lut_window(psxntb_t *pntb, int index, int target)
{
	return (psxntb_set_lut_window(pntb, index, target, -1));
}

/*
 * Program a range of LUT entries in the peer partition with the set
 * of addresses in the ddi_dma_cookie array.
 * This is for translation from the private PCI bus to local memory.
 */
static int
psxntb_set_lut_peer_cb(psxntb_t *pntb, void *arg)
{
	psx_dma_handle_t	*pntb_h = arg;
	psx_bar_t		*win = pntb->pntb_seg_window;
	ddi_dma_cookie_t	*cp;
	uint_t			lut_off;
	uint32_t		reg;
	int			cnt;

	for (cnt = 0; cnt < pntb_h->pdh_lut_cookie_cnt; cnt++) {
		cp = &pntb_h->pdh_lut_cookies[cnt];

		lut_off = PNTB_NTB_CNTL_LUT(win->pb_lut_child +
		    pntb_h->pdh_seg + cnt);

		reg = PNTB_NTB_CNTL_LUT_ENABLE |
		    PNTB_NTB_CNTL_LUT_DPART(pntb->pntb_partition) |
		    PNTB_NTB_CNTL_LUT_ADDRL(cp->dmac_laddress);
		psxntb_put32(pntb, pntb->pntb_peer_ctl_off + lut_off, reg);

		reg = PNTB_NTB_CNTL_LUT_ADDRH(cp->dmac_laddress);
		psxntb_put32(pntb, pntb->pntb_peer_ctl_off + lut_off + 4, reg);
	}

	return (DDI_SUCCESS);
}

/*
 * Set the LUT entries for the segments described in the psx_dma_handle.
 */
int
psxntb_set_lut_peer(psxntb_t *pntb, psx_dma_handle_t *pntb_h)
{
	ddi_dma_impl_t	*i_hdl = (ddi_dma_impl_t *)pntb_h->pdh_handle;
	dev_info_t	*dip;
	ddi_dma_cookie_t *cp;
	size_t		entries;
	uint64_t	addr;
	uint64_t	final_size;
	int		alut, cnt, i;

	/*
	 * For an internal handle, the real handle will be NULL.
	 */
	dip = i_hdl == NULL ? pntb->pntb_dip : i_hdl->dmai_rdip;

	/*
	 * pdh_cookies are the cookies returned from the bind.
	 * They should all have the correct alignment, but it is
	 * possible a single cookie could span multiple LUT entries.
	 * Convert the bind cookies into lut size cookies.
	 */
	ASSERT(pntb_h->pdh_lut_cookies == NULL);
	pntb_h->pdh_lut_cookies = kmem_alloc(sizeof (ddi_dma_cookie_t) *
	    pntb_h->pdh_seg_cnt, KM_SLEEP);

	alut = 0;
	cp = &pntb_h->pdh_lut_cookies[0];

	for (cnt = 0; cnt < pntb_h->pdh_cookie_cnt; cnt++) {
		entries = (pntb_h->pdh_cookies[cnt].dmac_size +
		    pntb_h->pdh_seg_size - 1) / pntb_h->pdh_seg_size;

		final_size = pntb_h->pdh_cookies[cnt].dmac_size %
		    pntb_h->pdh_seg_size;

		addr = pntb_h->pdh_cookies[cnt].dmac_laddress;
		for (i = 0; i < entries; i++) {
			if (alut == pntb_h->pdh_seg_cnt) {
				dev_err(dip, CE_WARN, "!Too many cookies");
				goto failed;
			}

			cp->dmac_laddress = addr + i * pntb_h->pdh_seg_size;
			cp->dmac_size = pntb_h->pdh_seg_size;
			cp->dmac_type = 0;

			cp++;
			alut++;
		}

		if (final_size > 0)
			(--cp)->dmac_size = final_size;
	}

	if (alut != pntb_h->pdh_seg_cnt) {
		dev_err(dip, CE_WARN, "!Not enought cookies to span segments");
		goto failed;
	}

	pntb_h->pdh_lut_cookie_cnt = alut;

	if (psxntb_config_update(pntb, pntb->pntb_peer_ctl_off,
	    psxntb_set_lut_peer_cb, pntb_h) == DDI_SUCCESS)
		return (DDI_SUCCESS);

failed:
	kmem_free(pntb_h->pdh_lut_cookies, sizeof (ddi_dma_cookie_t) *
	    pntb_h->pdh_seg_cnt);
	pntb_h->pdh_lut_cookies = NULL;

	return (DDI_FAILURE);
}

static int
psxntb_clear_lut_range(psxntb_t *pntb, uint_t ctl_off, int seg, int cnt)
{
	psx_bar_t	*win = pntb->pntb_seg_window;
	uint_t		lut_off;
	int		i;

	for (i = 0; i < cnt; i++) {
		lut_off = PNTB_NTB_CNTL_LUT(win->pb_lut_child + seg + i);
		psxntb_put32(pntb, ctl_off + lut_off, 0);
		psxntb_put32(pntb, ctl_off + lut_off + 4, 0);
	}

	return (DDI_SUCCESS);
}

static int
psxntb_clear_lut_peer_cb(psxntb_t *pntb, void *arg)
{
	psx_dma_handle_t	*pntb_h = arg;

	return (psxntb_clear_lut_range(pntb, pntb->pntb_peer_ctl_off,
	    pntb_h->pdh_seg, pntb_h->pdh_lut_cookie_cnt));
}

int
psxntb_clear_lut_peer(psxntb_t *pntb, psx_dma_handle_t *pntb_h)
{
	ASSERT(pntb_h->pdh_lut_cookies != NULL);

	if (psxntb_config_update(pntb, pntb->pntb_peer_ctl_off,
	    psxntb_clear_lut_peer_cb, pntb_h) != DDI_SUCCESS)
		return (DDI_FAILURE);

	kmem_free(pntb_h->pdh_lut_cookies, sizeof (ddi_dma_cookie_t) *
	    pntb_h->pdh_lut_cookie_cnt);

	pntb_h->pdh_lut_cookies = NULL;
	pntb_h->pdh_lut_cookie_cnt = 0;

	return (DDI_SUCCESS);
}

/*
 * Configure a range of segments in local LUT to translate addresses to
 * go on the private PCI bus.
 */
static int
psxntb_set_lut_local_cb(psxntb_t *pntb, void *arg)
{
	psx_dma_handle_t	*pntb_h = arg;
	psx_bar_t		*win = pntb->pntb_seg_window;
	uint64_t		addr;
	uint_t			lut_off;
	uint32_t		reg;
	int			i, cnt;

	for (cnt = 0; cnt < pntb_h->pdh_seg_cnt; cnt++) {
		i = win->pb_lut_child + pntb_h->pdh_seg + cnt;

		lut_off = PNTB_NTB_CNTL_LUT(i);

		addr = win->pb_peer_paddr + i * win->pb_seg_size;

		reg = PNTB_NTB_CNTL_LUT_ENABLE |
		    PNTB_NTB_CNTL_LUT_DPART(pntb->pntb_peer_partition) |
		    PNTB_NTB_CNTL_LUT_ADDRL(addr);
		psxntb_put32(pntb, pntb->pntb_ctl_off + lut_off, reg);

		reg = PNTB_NTB_CNTL_LUT_ADDRH(addr);
		psxntb_put32(pntb, pntb->pntb_ctl_off + lut_off + 4, reg);
	}

	return (DDI_SUCCESS);
}

int
psxntb_set_lut_local(psxntb_t *pntb, psx_dma_handle_t *pntb_h)
{
	return (psxntb_config_update(pntb, pntb->pntb_ctl_off,
	    psxntb_set_lut_local_cb, pntb_h));
}

static int
psxntb_clear_lut_local_cb(psxntb_t *pntb, void *arg)
{
	psx_dma_handle_t	*pntb_h = arg;

	return (psxntb_clear_lut_range(pntb, pntb->pntb_ctl_off,
	    pntb_h->pdh_seg, pntb_h->pdh_seg_cnt));
}

int
psxntb_clear_lut_local(psxntb_t *pntb, psx_dma_handle_t *pntb_h)
{
	return (psxntb_config_update(pntb, pntb->pntb_ctl_off,
	    psxntb_clear_lut_local_cb, pntb_h));
}

typedef struct {
	psx_bar_t	*win;
	int		partition;
} ctl_lut_args_t;

/*
 * Setup a window (BAR) for LUT, rather than direct address translation.
 */
static int
psxntb_setup_lut_cb(psxntb_t *pntb, void *arg)
{
	ctl_lut_args_t	*args = arg;
	int		bar = args->win->pb_bar;
	int		base = args->win->pb_lut_base;
	int		entries = args->win->pb_seg_count;
	ulong_t		seg_size = args->win->pb_seg_size;
	uint_t		ctl_off, bar_off;
	uint32_t	reg;
	int		i;

	ctl_off = args->partition == pntb->pntb_partition ?
	    pntb->pntb_ctl_off : pntb->pntb_peer_ctl_off;

	bar_off = PNTB_NTB_CNTL_BAR_SETUP(bar);

	reg = psxntb_get32(pntb, ctl_off + bar_off);
	if ((reg & PNTB_NTB_CNTL_BAR_VALID) == 0)
		return (DDI_FAILURE);

	if (entries == 0) {
		reg = 0;
	} else {
		reg &= ~PNTB_NTB_CNTL_BAR_DIRECT;
		reg |= PNTB_NTB_CNTL_BAR_LUT |
		    PNTB_NTB_CNTL_BAR_LUT_POS(highbit(seg_size) - 1) |
		    PNTB_NTB_CNTL_BAR_LUT_SUBWIN(entries - 1) |
		    PNTB_NTB_CNTL_BAR_LUT_SUBWIN_BASE(base);
	}

	psxntb_put32(pntb, ctl_off + bar_off, reg);
	for (i = 4; i < 16; i += 4)
		psxntb_put32(pntb, ctl_off + bar_off + i, 0);

	return (DDI_SUCCESS);
}

int
psxntb_setup_lut(psxntb_t *pntb, psx_bar_t *win, int partition)
{
	ctl_lut_args_t	args;
	uint_t		ctl_off;

	ctl_off = partition == pntb->pntb_partition ?
	    pntb->pntb_ctl_off : pntb->pntb_peer_ctl_off;

	args.win = win;
	args.partition = partition;

	return (psxntb_config_update(pntb, ctl_off, psxntb_setup_lut_cb,
	    &args));
}

static int
psxntb_clear_lut_cb(psxntb_t *pntb, void *arg)
{
	ctl_lut_args_t	*args = arg;
	int		bar = args->win->pb_bar;
	uint_t		ctl_off, bar_off;
	int		i;

	ctl_off = args->partition == pntb->pntb_partition ?
	    pntb->pntb_ctl_off : pntb->pntb_peer_ctl_off;

	bar_off = PNTB_NTB_CNTL_BAR_SETUP(bar);

	for (i = 0; i < 16; i += 4)
		psxntb_put32(pntb, ctl_off + bar_off + i, 0);

	return (DDI_SUCCESS);
}

int
psxntb_clear_lut(psxntb_t *pntb, psx_bar_t *win, int partition)
{
	ctl_lut_args_t	args;
	uint_t		ctl_off;

	ctl_off = partition == pntb->pntb_partition ?
	    pntb->pntb_ctl_off : pntb->pntb_peer_ctl_off;

	args.win = win;
	args.partition = partition;

	return (psxntb_config_update(pntb, ctl_off, psxntb_clear_lut_cb,
	    &args));
}

typedef struct {
	uint_t		partition;
	uint16_t	*rids;
	uint_t		cnt;
} set_rids_args_t;

/*
 * Program the allowed requester Ids.
 */
static int
psxntb_set_rids_cb(psxntb_t *pntb, void *arg)
{
	set_rids_args_t	*args = arg;
	uint_t		ctl_off;
	int		i;

	ctl_off = args->partition == pntb->pntb_partition ?
	    pntb->pntb_ctl_off : pntb->pntb_peer_ctl_off;

	for (i = 0; i < args->cnt; i++) {
		psxntb_put32(pntb, ctl_off + PNTB_NTB_CNTL_RID(i),
		    PNTB_NTB_CNTL_RID_ID(args->rids[i]) |
		    PNTB_NTB_CNTL_RID_ENABLE);
	}

	return (DDI_SUCCESS);
}

int
psxntb_set_rids(psxntb_t *pntb, int partition, uint16_t *rids, uint_t cnt)
{
	set_rids_args_t	args;
	uint_t		ctl_off;

	ctl_off = partition == pntb->pntb_partition ?
	    pntb->pntb_ctl_off : pntb->pntb_peer_ctl_off;

	args.partition = partition;
	args.rids = rids;
	args.cnt = cnt;

	return (psxntb_config_update(pntb, ctl_off, psxntb_set_rids_cb, &args));
}

static int
psxntb_clear_rids_cb(psxntb_t *pntb, void *arg)
{
	int		partition = (int)(uintptr_t)arg;
	uint_t		ctl_off;
	uint16_t	rid_limit;
	int		i;

	ctl_off = partition == pntb->pntb_partition ?
	    pntb->pntb_ctl_off : pntb->pntb_peer_ctl_off;

	rid_limit = psxntb_get16(pntb, ctl_off + PNTB_NTB_CNTL_RID_LIMIT);
	if (rid_limit == 0)
		return (DDI_SUCCESS);

	for (i = 0; i < rid_limit; i++)
		psxntb_put32(pntb, ctl_off + PNTB_NTB_CNTL_RID(i), 0);

	return (DDI_SUCCESS);
}

int
psxntb_clear_rids(psxntb_t *pntb, int partition)
{
	uint_t	ctl_off;

	ctl_off = partition == pntb->pntb_partition ?
	    pntb->pntb_ctl_off : pntb->pntb_peer_ctl_off;

	return (psxntb_config_update(pntb, ctl_off, psxntb_clear_rids_cb,
	    (void *)(uintptr_t)partition));
}

/*
 * disable rid usage
 */
void
psxntb_rid_disable(psxntb_t *pntb)
{
	uint32_t reg, reg_peer;

	reg = psxntb_get32(pntb, pntb->pntb_ctl_off + PNTB_NTB_PART_CNTRL);
	reg_peer = psxntb_get32(pntb, pntb->pntb_peer_ctl_off +
	    PNTB_NTB_PART_CNTRL);

	reg |= PNTB_ID_PROTECTION_DISABLE;
	reg_peer |= PNTB_ID_PROTECTION_DISABLE;

	psxntb_put32(pntb, pntb->pntb_ctl_off + PNTB_NTB_PART_CNTRL, reg);
	psxntb_put32(pntb, pntb->pntb_peer_ctl_off + PNTB_NTB_PART_CNTRL,
	    reg_peer);

#ifdef DEBUG
	cmn_err(CE_WARN, "psxntb_rid_disable: NTB Partition Control set to "
	    "0x%x/0x%x", reg, reg_peer);
#endif
}

/*
 * Called when the driver is quiesced.
 * Clear out all translations.
 */
static int
psxntb_quiesce_translation_cb(psxntb_t *pntb, void *arg)
{
	int		partition = (int)(uintptr_t)arg;
	int		bar, i;
	uint32_t	reg;
	uint_t		ctl_off, bar_off, lut_off;

	ctl_off = partition == pntb->pntb_partition ?
	    pntb->pntb_ctl_off : pntb->pntb_peer_ctl_off;

	for (bar = 0; bar < PCI_BASE_NUM; bar++) {
		bar_off = PNTB_NTB_CNTL_BAR_SETUP(bar);

		reg = psxntb_get32(pntb, ctl_off + bar_off);
		if ((reg & PNTB_NTB_CNTL_BAR_VALID) == 0)
			continue;

		for (i = 0; i < 16; i += 4)
			psxntb_put32(pntb, ctl_off + bar_off + i, 0);
	}

	for (i = 0; i < PNTB_NTB_LUT_ENTRIES; i++) {
		lut_off = PNTB_NTB_CNTL_LUT(i);

		psxntb_put32(pntb, ctl_off + lut_off, 0);
		psxntb_put32(pntb, ctl_off + lut_off + 4, 0);
	}

	return (DDI_SUCCESS);
}

/*
 * Called from the ddi_quiesce() routine. Must not sleep.
 */
int
psxntb_quiesce_translation(psxntb_t *pntb)
{
	/*
	 * Disable doorbells and messages.
	 */
	psxntb_inter_ntb_cleanup(pntb);

	/*
	 * Disable all translations in outbound partition.
	 */
	if (psxntb_config_update_nolock(pntb, pntb->pntb_ctl_off,
	    psxntb_quiesce_translation_cb,
	    (void *)(uintptr_t)pntb->pntb_partition) != DDI_SUCCESS)
		return (DDI_FAILURE);

	/*
	 * Disable all translations in inbound partition.
	 */
	if (psxntb_config_update_nolock(pntb, pntb->pntb_peer_ctl_off,
	    psxntb_quiesce_translation_cb,
	    (void *)(uintptr_t)pntb->pntb_peer_partition) != DDI_SUCCESS)
		return (DDI_FAILURE);

	/*
	 * Clear requester ids in outbound partition.
	 */
	if (psxntb_config_update_nolock(pntb, pntb->pntb_ctl_off,
	    psxntb_clear_rids_cb,
	    (void *)(uintptr_t)pntb->pntb_partition) != DDI_SUCCESS)
		return (DDI_FAILURE);

	/*
	 * Clear requester ids in inbound partition.
	 */
	if (psxntb_config_update_nolock(pntb, pntb->pntb_peer_ctl_off,
	    psxntb_clear_rids_cb,
	    (void *)(uintptr_t)pntb->pntb_peer_partition) != DDI_SUCCESS)
		return (DDI_FAILURE);

	return (DDI_SUCCESS);
}
