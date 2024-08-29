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
 * Copyright (c) 2017 Joyent, Inc.
 * Copyright 2024 Tintri by DDN, Inc. All rights reserved.
 */

/*
 * Interface with the Hardware Resource Manager (HWRM)
 */

#include "bnxt.h"

/*
 * Allocate a single page of DMA memory and size several of the attributes based
 * on that.
 */
#define	BNXT_HWRM_BUFFER_SIZE	PAGESIZE

/* The offset from the start of the BAR to the HWRM doorbell */
#define	BNXT_HWRM_DB_OFF	0x100

/* Time in milliseconds to wait between checks (one second, by default) */
#define	BNXT_HWRM_DELAY_MS	1
#define	BNXT_HWRM_DEFAULT_TIMEOUT	1000

/* Function ID for the requesting function */
#define	BNXT_HWRM_FID_SELF	0xffff

static inline void
bnxt_hwrm_write(bnxt_t *bp, uintptr_t off, uint32_t val)
{
	uintptr_t addr;

	addr = (uintptr_t)bp->dev_base + off;
	ddi_put32(bp->dev_hdl, (void *)addr, val);
}

static void
bnxt_hwrm_init_header(bnxt_t *bp, void *req, uint16_t rtype)
{
	struct input *in = req;

	/* The seq_id is initialized when sending the request */
	in->req_type = LE_16(rtype);
	in->cmpl_ring = LE_16(UINT16_MAX);
	in->target_id = LE_16(UINT16_MAX);
	in->resp_addr = LE_64(bp->hwrm_reply.pa);
}

/*
 * Send a message to the HWRM. We do this in a few steps:
 *
 * 1. Assign a sequence identifier.
 * 2. Make sure that the output buffer has been zeroed and synced.
 * 3. Write all bytes to the hwrm channel via PIO.
 * 4. Write zeroes to the hwrm channel via PIO to get the max buffer size.
 * 5. Ring the doorbell.
 * 6. Wait the timeout for the update length to transition to being non-zero.
 * 7. Wait the timeout for the last byte to be set to the valid bit.
 */
static int
bnxt_hwrm_send_message(bnxt_t *bp, void *req, size_t len, uint_t timeout)
{
	hwrm_err_output_t *resp = (void *)bp->hwrm_reply.va;
	struct input *in = req;
	uint32_t *req32;
	uint8_t *valid;
	uint16_t resplen, rtype, err;
	uint_t maxdelay;
	size_t i;

	if ((len % 4) != 0) {
		bnxt_error(bp, "!HWRM request must be 4-byte aligned, id %zd",
		    len % 4);
		return (1);
	}

	if (len > bp->hwrm_max_req) {
		bnxt_error(bp,
		    "!HWRM request too long (%zd bytes), max of %d bytes",
		    len, bp->hwrm_max_req);
		return (1);
	}

	in->seq_id = bp->hwrm_seqid;
	bp->hwrm_seqid++;
	rtype = LE_16(in->req_type);

	/* Make sure that our output buffer is clean */
	memset(bp->hwrm_reply.va, 0, bp->hwrm_reply.len);
	BNXT_DMA_SYNC(bp->hwrm_reply, DDI_DMA_SYNC_FORDEV);

	/* All requests are supposed to be 4-byte aligned */
	req32 = req;
	for (i = 0; i < len; i += 4, req32++)
		bnxt_hwrm_write(bp, i, *req32);

	for (; i < bp->hwrm_max_req; i += 4)
		bnxt_hwrm_write(bp, i, 0);

	/*
	 * Note, the doorbell for the HWRM is still off of the main device
	 * handle. It is not off of the doorbell handle.
	 */
	bnxt_hwrm_write(bp, BNXT_HWRM_DB_OFF, 1);

	if (timeout == 0)
		timeout = bp->hwrm_timeout;
	maxdelay = timeout / BNXT_HWRM_DELAY_MS;

	for (i = 0; i < maxdelay; i++) {
		BNXT_DMA_SYNC(bp->hwrm_reply, DDI_DMA_SYNC_FORKERNEL);
		resplen = LE_16(resp->resp_len);
		if (resplen != 0 && resplen <= BNXT_HWRM_BUFFER_SIZE)
			break;
		delay(drv_usectohz(BNXT_HWRM_DELAY_MS * MILLISEC));
	}

	if (i >= maxdelay) {
		bnxt_error(bp,
		    "!timed out sending command %s waiting for length",
		    GET_HWRM_REQ_TYPE(rtype));
		return (1);
	}

	valid = (uint8_t *)(void *)((uintptr_t)resp + resplen - 1);
	for (i = 0; i < maxdelay; i++) {
		BNXT_DMA_SYNC(bp->hwrm_reply, DDI_DMA_SYNC_FORKERNEL);
		if (*valid == HWRM_RESP_VALID_KEY)
			break;
		delay(drv_usectohz(BNXT_HWRM_DELAY_MS * MILLISEC));
	}

	if (i >= maxdelay) {
		bnxt_error(bp,
		    "!timed out sending command %s waiting for valid byte",
		    GET_HWRM_REQ_TYPE(rtype));
		return (1);
	}

	err = LE_16(resp->error_code);
	if (err != HWRM_ERR_CODE_SUCCESS) {
		bnxt_error(bp, "!command %s failed with %s (%d)",
		    GET_HWRM_REQ_TYPE(rtype), GET_HWRM_ERROR_CODE(err), err);
		return (1);
	}

	return (0);
}

/*
 * This command is used to get basic information about the device. It is run
 * before the device has been fully set up, so we can only assume some basic
 * aspects of the device.
 */
int
bnxt_hwrm_version_get(bnxt_t *bp)
{
	hwrm_ver_get_input_t req = { 0 };
	const hwrm_ver_get_output_t *resp = (void *)bp->hwrm_reply.va;

	bnxt_hwrm_init_header(bp, &req, HWRM_VER_GET);

	req.hwrm_intf_maj = HWRM_VERSION_MAJOR;
	req.hwrm_intf_min = HWRM_VERSION_MINOR;
	req.hwrm_intf_upd = HWRM_VERSION_UPDATE;

	mutex_enter(&bp->hwrm_lock);
	if (bnxt_hwrm_send_message(bp, &req, sizeof (req), 0) != 0) {
		mutex_exit(&bp->hwrm_lock);
		return (1);
	}
	/*
	 * Snapshot the version output and go through and make sure all fields
	 * that aren't part of the response header are in native endian.
	 */
	bcopy(resp, &bp->ver, sizeof (bp->ver));
	bp->ver.dev_caps_cfg = LE_32(bp->ver.dev_caps_cfg);
	bp->ver.chip_num = LE_16(bp->ver.chip_num);
	bp->ver.max_req_win_len = LE_16(bp->ver.max_req_win_len);
	bp->ver.max_resp_len = LE_16(bp->ver.max_resp_len);
	bp->ver.def_req_timeout = LE_16(bp->ver.def_req_timeout);
	mutex_exit(&bp->hwrm_lock);

	return (0);
}

/* Obtain and save basic NVM information */
int
bnxt_hwrm_nvm_info_get(bnxt_t *bp)
{
	hwrm_nvm_get_dev_info_input_t req = { 0 };
	const hwrm_nvm_get_dev_info_output_t *resp =
	    (void *)bp->hwrm_reply.va;

	bnxt_hwrm_init_header(bp, &req, HWRM_NVM_GET_DEV_INFO);

	mutex_enter(&bp->hwrm_lock);
	/*
	 * While NVM operations may be slower, we still use the default timeout
	 * when simply getting information.
	 */
	if (bnxt_hwrm_send_message(bp, &req, sizeof (req), 0) != 0) {
		mutex_exit(&bp->hwrm_lock);
		return (1);
	}
	bp->nvm.manufacturer_id = LE_16(resp->manufacturer_id);
	bp->nvm.device_id = LE_16(resp->device_id);
	bp->nvm.sector_size = LE_32(resp->sector_size);
	bp->nvm.nvram_size = LE_32(resp->nvram_size);
	bp->nvm.reserved_size = LE_32(resp->reserved_size);
	bp->nvm.available_size = LE_32(resp->available_size);
	mutex_exit(&bp->hwrm_lock);

	return (0);
}

int
bnxt_hwrm_func_reset(bnxt_t *bp)
{
	hwrm_func_reset_input_t req = { 0 };
	int ret;

	bnxt_hwrm_init_header(bp, &req, HWRM_FUNC_RESET);

	req.enables = 0;

	mutex_enter(&bp->hwrm_lock);
	ret = bnxt_hwrm_send_message(bp, &req, sizeof (req),
	    (uint_t)bp->hwrm_timeout * 4);
	mutex_exit(&bp->hwrm_lock);

	return (ret);
}

int
bnxt_hwrm_func_qcaps(bnxt_t *bp)
{
	hwrm_func_qcaps_input_t req = { 0 };
	const hwrm_func_qcaps_output_t *resp = (void *)bp->hwrm_reply.va;

	bnxt_hwrm_init_header(bp, &req, HWRM_FUNC_QCAPS);

	req.fid = LE_16(BNXT_HWRM_FID_SELF);

	mutex_enter(&bp->hwrm_lock);
	if (bnxt_hwrm_send_message(bp, &req, sizeof (req), 0) != 0) {
		mutex_exit(&bp->hwrm_lock);
		return (1);
	}
	bp->fid = LE_16(resp->fid);
	bp->port_id = LE_16(resp->port_id);
	bp->qcap_flags = LE_32(resp->flags);
	memcpy(bp->macaddr, resp->mac_address, ETHERADDRL);
	bp->max_rsscos_ctx = LE_16(resp->max_rsscos_ctx);
	bp->max_cp_rings = LE_16(resp->max_cmpl_rings);
	bp->max_tx_rings = LE_16(resp->max_tx_rings);
	bp->max_rx_rings = LE_16(resp->max_rx_rings);
	bp->max_l2_ctxs = LE_16(resp->max_l2_ctxs);
	bp->max_vnics = LE_16(resp->max_vnics);
	bp->max_stat_ctx = LE_16(resp->max_stat_ctx);
	bp->max_rx_em_flows = LE_32(resp->max_rx_em_flows);
	bp->max_rx_wm_flows = LE_32(resp->max_rx_wm_flows);
	bp->max_mcast_filters = LE_32(resp->max_mcast_filters);
	bp->max_flow_id = LE_32(resp->max_flow_id);
	bp->max_hw_ring_grps = LE_32(resp->max_hw_ring_grps);
	mutex_exit(&bp->hwrm_lock);

	return (0);
}

int
bnxt_hwrm_func_qcfg(bnxt_t *bp)
{
	hwrm_func_qcfg_input_t req = { 0 };
	const hwrm_func_qcfg_output_t *resp = (void *)bp->hwrm_reply.va;

	bnxt_hwrm_init_header(bp, &req, HWRM_FUNC_QCFG);

	req.fid = LE_16(BNXT_HWRM_FID_SELF);

	mutex_enter(&bp->hwrm_lock);
	if (bnxt_hwrm_send_message(bp, &req, sizeof (req), 0) != 0) {
		mutex_exit(&bp->hwrm_lock);
		return (1);
	}
	bp->alloc_cp_rings = LE_16(resp->alloc_cmpl_rings);
	bp->alloc_tx_rings = LE_16(resp->alloc_tx_rings);
	bp->alloc_rx_rings = LE_16(resp->alloc_rx_rings);
	bp->alloc_vnics = LE_16(resp->alloc_vnics);
	bp->alloc_mcast_filters = LE_32(resp->alloc_mcast_filters);
	bp->alloc_hw_ring_grps = LE_32(resp->alloc_hw_ring_grps);
	mutex_exit(&bp->hwrm_lock);

	return (0);
}

int
bnxt_hwrm_queue_qportcfg(bnxt_t *bp)
{
	hwrm_queue_qportcfg_input_t req = { 0 };
	const hwrm_queue_qportcfg_output_t *resp = (void *)bp->hwrm_reply.va;

	bnxt_hwrm_init_header(bp, &req, HWRM_QUEUE_QPORTCFG);

	mutex_enter(&bp->hwrm_lock);
	if (bnxt_hwrm_send_message(bp, &req, sizeof (req), 0) != 0) {
		mutex_exit(&bp->hwrm_lock);
		return (1);
	}
	bcopy(resp, &bp->qportcfg, sizeof (*resp));
	mutex_exit(&bp->hwrm_lock);

	return (0);
}

int
bnxt_hwrm_port_phy_qcfg(bnxt_t *bp)
{
	hwrm_port_phy_qcfg_input_t req = {0};
	const hwrm_port_phy_qcfg_output_t *resp = (void *)bp->hwrm_reply.va;

	bnxt_hwrm_init_header(bp, &req, HWRM_PORT_PHY_QCFG);

	mutex_enter(&bp->hwrm_lock);
	if (bnxt_hwrm_send_message(bp, &req, sizeof (req), 0) != 0) {
		mutex_exit(&bp->hwrm_lock);
		return (1);
	}
	bp->link.autoneg = resp->auto_mode ==
	    HWRM_PORT_PHY_QCFG_OUTPUT_AUTO_MODE_NONE ? 0 : 1;
	bp->link.duplex = bnxt_duplex_to_gld(resp->duplex_cfg);
#ifdef HAVE_MAC_MEDIA
	bp->link.media = bnxt_media_to_gld(resp->phy_type);
#endif
	bp->link.state = bnxt_state_to_gld(resp->link);
	bp->link.speed = LE_16(resp->link_speed) * 100000000ULL;
	bp->link.support_speeds = LE_16(resp->support_speeds);
	mutex_exit(&bp->hwrm_lock);

	return (0);
}

void
bnxt_hwrm_temp_monitor_query(bnxt_t *bp)
{
	hwrm_temp_monitor_query_input_t req = {0};
	const hwrm_temp_monitor_query_output_t *resp =
	    (void *)bp->hwrm_reply.va;

	bp->temp_valid = false;
	bp->temp_phy_valid = false;
	bp->temp_om_valid = false;

	bnxt_hwrm_init_header(bp, &req, HWRM_TEMP_MONITOR_QUERY);

	mutex_enter(&bp->hwrm_lock);
	if (bnxt_hwrm_send_message(bp, &req, sizeof (req), 0) != 0) {
		mutex_exit(&bp->hwrm_lock);
		return;
	}
	if ((resp->flags &
	    HWRM_TEMP_MONITOR_QUERY_OUTPUT_FLAGS_TEMP_NOT_AVAILABLE) == 0 &&
	    resp->temp != 0) {
		bp->temp = resp->temp;
		bp->temp_valid = true;
	}
	if ((resp->flags &
	    HWRM_TEMP_MONITOR_QUERY_OUTPUT_FLAGS_PHY_TEMP_NOT_AVAILABLE) == 0 &&
	    resp->phy_temp != 0) {
		bp->temp_phy = resp->phy_temp;
		bp->temp_phy_valid = true;
	}
	if ((resp->flags &
	    HWRM_TEMP_MONITOR_QUERY_OUTPUT_FLAGS_OM_NOT_PRESENT) == 0 &&
	    (resp->flags &
	    HWRM_TEMP_MONITOR_QUERY_OUTPUT_FLAGS_OM_TEMP_NOT_AVAILABLE) == 0 &&
	    resp->om_temp != 0) {
		bp->temp_om = resp->om_temp;
		bp->temp_om_valid = true;
	}
	mutex_exit(&bp->hwrm_lock);
}

int
bnxt_hwrm_host_register(bnxt_t *bp)
{
	hwrm_func_drv_rgtr_input_t req = { 0 };
	ulong_t *bitmap;
	uint_t i;
	int ret;

	bnxt_hwrm_init_header(bp, &req, HWRM_FUNC_DRV_RGTR);

	req.ver_maj = HWRM_VERSION_MAJOR;
	req.ver_min = HWRM_VERSION_MINOR;
	req.ver_upd = HWRM_VERSION_UPDATE;

	req.enables = LE_32(HWRM_FUNC_DRV_RGTR_INPUT_ENABLES_VER |
	    HWRM_FUNC_DRV_RGTR_INPUT_ENABLES_OS_TYPE |
	    HWRM_FUNC_DRV_RGTR_INPUT_ENABLES_ASYNC_EVENT_FWD);
	req.flags = LE_32(HWRM_FUNC_DRV_RGTR_INPUT_FLAGS_16BIT_VER_MODE);
	req.os_type = LE_16(HWRM_FUNC_DRV_RGTR_INPUT_OS_TYPE_OTHER);

	bitmap = (ulong_t *)req.async_event_fwd;
	BT_SET(bitmap, HWRM_ASYNC_EVENT_CMPL_EVENT_ID_LINK_MTU_CHANGE);
	BT_SET(bitmap, HWRM_ASYNC_EVENT_CMPL_EVENT_ID_LINK_SPEED_CFG_CHANGE);
	BT_SET(bitmap, HWRM_ASYNC_EVENT_CMPL_EVENT_ID_LINK_SPEED_CHANGE);
	BT_SET(bitmap, HWRM_ASYNC_EVENT_CMPL_EVENT_ID_LINK_STATUS_CHANGE);
	BT_SET(bitmap, HWRM_ASYNC_EVENT_CMPL_EVENT_ID_PORT_CONN_NOT_ALLOWED);
	BT_SET(bitmap, HWRM_ASYNC_EVENT_CMPL_EVENT_ID_PORT_PHY_CFG_CHANGE);

	for (i = 0; i < sizeof (req.async_event_fwd) / sizeof (ulong_t); i++)
		bitmap[i] = LE_64(bitmap[i]);

	mutex_enter(&bp->hwrm_lock);
	ret = bnxt_hwrm_send_message(bp, &req, sizeof (req), 0);
	mutex_exit(&bp->hwrm_lock);

	return (ret);
}

int
bnxt_hwrm_host_unregister(bnxt_t *bp)
{
	hwrm_func_drv_rgtr_input_t req = { 0 };
	int ret;

	bnxt_hwrm_init_header(bp, &req, HWRM_FUNC_DRV_UNRGTR);

	mutex_enter(&bp->hwrm_lock);
	ret = bnxt_hwrm_send_message(bp, &req, sizeof (req), 0);
	mutex_exit(&bp->hwrm_lock);

	return (ret);
}

int
bnxt_hwrm_events_cfg_cr(bnxt_ring_t *rp)
{
	bnxt_t *bp = rp->bp;
	hwrm_func_cfg_input_t req = { 0 };
	int ret;

	bnxt_hwrm_init_header(bp, &req, HWRM_FUNC_CFG);

	req.fid = LE_16(BNXT_HWRM_FID_SELF);
	req.enables = LE_32(HWRM_FUNC_CFG_INPUT_ENABLES_ASYNC_EVENT_CR);
	req.async_event_cr = rp->cp_phys_id;

	mutex_enter(&bp->hwrm_lock);
	ret = bnxt_hwrm_send_message(bp, &req, sizeof (req), 0);
	mutex_exit(&bp->hwrm_lock);

	return (ret);
}

int
bnxt_hwrm_stat_ctx_alloc(bnxt_ring_t *rp)
{
	bnxt_t *bp = rp->bp;
	hwrm_stat_ctx_alloc_input_t req = { 0 };
	hwrm_stat_ctx_alloc_output_t *resp = (void *)bp->hwrm_reply.va;
	int ret;

	VERIFY3U(rp->cp_stat_ctx_id, ==, HWRM_NA_SIGNATURE);

	bnxt_hwrm_init_header(bp, &req, HWRM_STAT_CTX_ALLOC);

	req.update_period_ms = LE_32(1000);
	req.stats_dma_addr = LE_64(rp->cp_stats.pa);
	req.stats_dma_length = LE_16(sizeof (ctx_hw_stats_t));

	mutex_enter(&bp->hwrm_lock);
	ret = bnxt_hwrm_send_message(bp, &req, sizeof (req), 0);
	if (ret == 0)
		rp->cp_stat_ctx_id = LE_32(resp->stat_ctx_id);
	mutex_exit(&bp->hwrm_lock);

	return (ret);
}

int
bnxt_hwrm_stat_ctx_free(bnxt_ring_t *rp)
{
	bnxt_t *bp = rp->bp;
	hwrm_stat_ctx_free_input_t req = { 0 };
	int ret;

	if (rp->cp_stat_ctx_id == HWRM_NA_SIGNATURE)
		return (0);

	bnxt_hwrm_init_header(bp, &req, HWRM_STAT_CTX_FREE);

	req.stat_ctx_id = LE_32(rp->cp_stat_ctx_id);

	mutex_enter(&bp->hwrm_lock);
	ret = bnxt_hwrm_send_message(bp, &req, sizeof (req), 0);
	if (ret == 0)
		rp->cp_stat_ctx_id = HWRM_NA_SIGNATURE;
	mutex_exit(&bp->hwrm_lock);

	return (ret);
}

int
bnxt_hwrm_ring_alloc(bnxt_ring_t *rp, uint8_t type)
{
	bnxt_t *bp = rp->bp;
	hwrm_ring_alloc_input_t req = { 0 };
	const hwrm_ring_alloc_output_t *resp = (void *)bp->hwrm_reply.va;
	uint16_t *phys_id = &rp->phys_id;
	int ret;

	bnxt_hwrm_init_header(bp, &req, HWRM_RING_ALLOC);

	req.enables = LE_32(0);
	req.fbo = LE_32(0);
	req.logical_id = LE_16(rp->id);
	req.ring_type = type;

	switch (type) {
	case HWRM_RING_ALLOC_INPUT_RING_TYPE_L2_CMPL:
		req.int_mode = HWRM_RING_ALLOC_INPUT_INT_MODE_MSIX;
		req.page_tbl_addr = LE_64(rp->cp_descs.pa);
		req.length = LE_32(rp->cp_ndesc);
		phys_id = &rp->cp_phys_id;
		break;
	case HWRM_RING_ALLOC_INPUT_RING_TYPE_RX:
		req.page_tbl_addr = LE_64(rp->descs.pa);
		req.length = LE_32(rp->ndesc);
		break;
	case HWRM_RING_ALLOC_INPUT_RING_TYPE_TX:
		req.page_tbl_addr = LE_64(rp->descs.pa);
		req.length = LE_32(rp->ndesc);
		req.cmpl_ring_id = LE_16(rp->cp_phys_id);
		req.queue_id = bp->qportcfg.queue_id0;
		req.stat_ctx_id = LE_32(rp->cp_stat_ctx_id);
		req.enables |=
		    LE_32(HWRM_RING_ALLOC_INPUT_ENABLES_STAT_CTX_ID_VALID);
		break;
	}

	VERIFY3U(*phys_id, ==, (uint16_t)HWRM_NA_SIGNATURE);

	mutex_enter(&bp->hwrm_lock);
	ret = bnxt_hwrm_send_message(bp, &req, sizeof (req), 0);
	if (ret == 0)
		*phys_id = resp->ring_id;
	mutex_exit(&bp->hwrm_lock);

	return (ret);
}

int
bnxt_hwrm_ring_free(bnxt_ring_t *rp, uint8_t type)
{
	bnxt_t *bp = rp->bp;
	hwrm_ring_free_input_t req = { 0 };
	uint16_t *phys_id = &rp->phys_id;
	int ret;

	bnxt_hwrm_init_header(bp, &req, HWRM_RING_FREE);

	req.ring_type = type;
	switch (type) {
	case HWRM_RING_FREE_INPUT_RING_TYPE_L2_CMPL:
		req.cmpl_ring = (uint16_t)HWRM_NA_SIGNATURE;
		phys_id = &rp->cp_phys_id;
		break;
	case HWRM_RING_FREE_INPUT_RING_TYPE_RX:
	case HWRM_RING_FREE_INPUT_RING_TYPE_TX:
		req.cmpl_ring = rp->cp_phys_id;
		break;
	}

	if (*phys_id == (uint16_t)HWRM_NA_SIGNATURE)
		return (0);

	req.ring_id = *phys_id;

	mutex_enter(&bp->hwrm_lock);
	ret = bnxt_hwrm_send_message(bp, &req, sizeof (req), 0);
	if (ret == 0)
		*phys_id = (uint16_t)HWRM_NA_SIGNATURE;
	mutex_exit(&bp->hwrm_lock);

	return (ret);
}

void
bnxt_hwrm_ring_reset(bnxt_ring_t *rp)
{
	bnxt_t *bp = rp->bp;
	hwrm_ring_reset_input_t req = { 0 };

	if (rp->phys_id == (uint16_t)HWRM_NA_SIGNATURE)
		return;

	bnxt_hwrm_init_header(bp, &req, HWRM_RING_RESET);

	req.ring_id = LE_16(rp->phys_id);
	req.ring_type = LE_32(rp->type);

	mutex_enter(&bp->hwrm_lock);
	(void) bnxt_hwrm_send_message(bp, &req, sizeof (req), 0);
	mutex_exit(&bp->hwrm_lock);
}

int
bnxt_hwrm_ring_grp_alloc(bnxt_t *bp, bnxt_group_t *grp)
{
	hwrm_ring_grp_alloc_input_t req = { 0 };
	hwrm_ring_grp_alloc_output_t *resp = (void *)bp->hwrm_reply.va;
	int ret;

	VERIFY3U(grp->grp_id, ==, (uint16_t)HWRM_NA_SIGNATURE);

	bnxt_hwrm_init_header(bp, &req, HWRM_RING_GRP_ALLOC);

	req.sc = LE_16(grp->stat_ctx_id);
	req.cr = LE_16(grp->cpr_id);
	req.rr = LE_16(grp->rxr_id);
	req.ar = LE_16(grp->agr_id);

	mutex_enter(&bp->hwrm_lock);
	ret = bnxt_hwrm_send_message(bp, &req, sizeof (req), 0);
	if (ret == 0)
		grp->grp_id = LE_32(resp->ring_group_id);
	mutex_exit(&bp->hwrm_lock);

	return (ret);
}

int
bnxt_hwrm_ring_grp_free(bnxt_t *bp, bnxt_group_t *grp)
{
	hwrm_ring_grp_free_input_t req = { 0 };
	int ret;

	if (grp->grp_id == (uint16_t)HWRM_NA_SIGNATURE)
		return (0);

	bnxt_hwrm_init_header(bp, &req, HWRM_RING_GRP_FREE);

	req.ring_group_id = LE_32(grp->grp_id);

	mutex_enter(&bp->hwrm_lock);
	ret = bnxt_hwrm_send_message(bp, &req, sizeof (req), 0);
	if (ret == 0) {
		grp->grp_id = (uint16_t)HWRM_NA_SIGNATURE;
		grp->stat_ctx_id = (uint16_t)HWRM_NA_SIGNATURE;
		grp->cpr_id = (uint16_t)HWRM_NA_SIGNATURE;
		grp->rxr_id = (uint16_t)HWRM_NA_SIGNATURE;
		grp->agr_id = (uint16_t)HWRM_NA_SIGNATURE;
	}
	mutex_exit(&bp->hwrm_lock);

	return (ret);
}

int
bnxt_hwrm_vnic_ctx_alloc(bnxt_t *bp)
{
	hwrm_vnic_rss_cos_lb_ctx_alloc_input_t req = { 0 };
	hwrm_vnic_rss_cos_lb_ctx_alloc_output_t *resp =
	    (void *)bp->hwrm_reply.va;
	int ret;

	VERIFY3U(bp->vnic.rss_id, ==, (uint16_t)HWRM_NA_SIGNATURE);

	bnxt_hwrm_init_header(bp, &req, HWRM_VNIC_RSS_COS_LB_CTX_ALLOC);

	mutex_enter(&bp->hwrm_lock);
	ret = bnxt_hwrm_send_message(bp, &req, sizeof (req), 0);
	if (ret == 0)
		bp->vnic.rss_id = LE_16(resp->rss_cos_lb_ctx_id);
	mutex_exit(&bp->hwrm_lock);

	return (ret);
}

int
bnxt_hwrm_vnic_ctx_free(bnxt_t *bp)
{
	hwrm_vnic_rss_cos_lb_ctx_free_input_t req = { 0 };
	int ret;

	if (bp->vnic.rss_id == (uint16_t)HWRM_NA_SIGNATURE)
		return (0);

	bnxt_hwrm_init_header(bp, &req, HWRM_VNIC_RSS_COS_LB_CTX_FREE);

	req.rss_cos_lb_ctx_id = LE_16(bp->vnic.rss_id);

	mutex_enter(&bp->hwrm_lock);
	ret = bnxt_hwrm_send_message(bp, &req, sizeof (req), 0);
	if (ret == 0)
		bp->vnic.rss_id = (uint16_t)HWRM_NA_SIGNATURE;
	mutex_exit(&bp->hwrm_lock);

	return (ret);
}

int
bnxt_hwrm_vnic_alloc(bnxt_t *bp)
{
	hwrm_vnic_alloc_input_t req = { 0 };
	const hwrm_vnic_alloc_output_t *resp = (void *)bp->hwrm_reply.va;
	int ret;

	VERIFY(bp->vnic.id == (uint16_t)HWRM_NA_SIGNATURE);

	bnxt_hwrm_init_header(bp, &req, HWRM_VNIC_ALLOC);

	req.flags = LE_32(HWRM_VNIC_ALLOC_INPUT_FLAGS_DEFAULT);

	mutex_enter(&bp->hwrm_lock);
	ret = bnxt_hwrm_send_message(bp, &req, sizeof (req), 0);
	if (ret == 0)
		bp->vnic.id = LE_32(resp->vnic_id);
	mutex_exit(&bp->hwrm_lock);

	return (ret);
}

int
bnxt_hwrm_vnic_free(bnxt_t *bp)
{
	hwrm_vnic_free_input_t req = {0};
	int ret;

	if (bp->vnic.id == (uint16_t)HWRM_NA_SIGNATURE)
		return (0);

	bnxt_hwrm_init_header(bp, &req, HWRM_VNIC_FREE);

	req.vnic_id = LE_32(bp->vnic.id);

	mutex_enter(&bp->hwrm_lock);
	ret = bnxt_hwrm_send_message(bp, &req, sizeof (req), 0);
	if (ret == 0)
		bp->vnic.id = (uint16_t)HWRM_NA_SIGNATURE;
	mutex_exit(&bp->hwrm_lock);

	return (ret);
}

int
bnxt_hwrm_vnic_cfg(bnxt_t *bp)
{
	hwrm_vnic_cfg_input_t req = { 0 };
	int ret;

	bnxt_hwrm_init_header(bp, &req, HWRM_VNIC_CFG);

	req.flags = LE_32(HWRM_VNIC_CFG_INPUT_FLAGS_DEFAULT);

	req.vnic_id = LE_16(bp->vnic.id);
	req.dflt_ring_grp = LE_16(bp->vnic.def_grp);
#if 0
	req.cos_rule = LE_16(bp->vnic.cos_rule);
	req.lb_rule = LE_16(bp->vnic.lb_rule);
#else
	req.cos_rule = (uint16_t)HWRM_NA_SIGNATURE;
	req.lb_rule = (uint16_t)HWRM_NA_SIGNATURE;
#endif
	req.rss_rule = LE_16(bp->vnic.rss_id);
	req.mru = LE_16(bp->vnic.mru);
	req.enables = LE_32(HWRM_VNIC_CFG_INPUT_ENABLES_DFLT_RING_GRP |
	    HWRM_VNIC_CFG_INPUT_ENABLES_MRU |
	    HWRM_VNIC_CFG_INPUT_ENABLES_RSS_RULE);

	mutex_enter(&bp->hwrm_lock);
	ret = bnxt_hwrm_send_message(bp, &req, sizeof (req), 0);
	mutex_exit(&bp->hwrm_lock);

	return (ret);
}

int
bnxt_hwrm_vnic_set_rx_mask(bnxt_t *bp)
{
	hwrm_cfa_l2_set_rx_mask_input_t req = { 0 };
	int ret;

	bnxt_hwrm_init_header(bp, &req, HWRM_CFA_L2_SET_RX_MASK);

	req.vnic_id = LE_32(bp->vnic.id);
	req.mask = LE_32(bp->vnic.rx_mask);
	req.mc_tbl_addr = LE_64(bp->vnic.mcast_tbl.pa);
	req.num_mc_entries = LE_32(bp->vnic.num_mcast);
	req.vlan_tag_tbl_addr = LE_64(bp->vnic.vlan_tbl.pa);
	req.num_vlan_tags = LE_32(bp->vnic.num_vlan);

	mutex_enter(&bp->hwrm_lock);
	ret = bnxt_hwrm_send_message(bp, &req, sizeof (req), 0);
	mutex_exit(&bp->hwrm_lock);

	return (ret);
}

int
bnxt_hwrm_vnic_filter_alloc(bnxt_t *bp, const uint8_t *mac, uint64_t *filter)
{
	hwrm_cfa_l2_filter_alloc_input_t req = { 0 };
	const hwrm_cfa_l2_filter_alloc_output_t *resp =
	    (void *)bp->hwrm_reply.va;
	int ret;

	bnxt_hwrm_init_header(bp, &req, HWRM_CFA_L2_FILTER_ALLOC);

	req.flags = LE_32(HWRM_CFA_L2_FILTER_ALLOC_INPUT_FLAGS_PATH_RX);

	req.dst_id = LE_16(bp->vnic.id);
	memcpy(req.l2_addr, mac, ETHERADDRL);
	memset(req.l2_addr_mask, 0xff, sizeof (req.l2_addr_mask));
	req.enables = LE_32(HWRM_CFA_L2_FILTER_ALLOC_INPUT_ENABLES_DST_ID |
	    HWRM_CFA_L2_FILTER_ALLOC_INPUT_ENABLES_L2_ADDR |
	    HWRM_CFA_L2_FILTER_ALLOC_INPUT_ENABLES_L2_ADDR_MASK);

	mutex_enter(&bp->hwrm_lock);
	ret = bnxt_hwrm_send_message(bp, &req, sizeof (req), 0);
	if (ret == 0)
		*filter = LE_64(resp->l2_filter_id);
	mutex_exit(&bp->hwrm_lock);

	return (ret);
}

int
bnxt_hwrm_vnic_filter_free(bnxt_t *bp, uint64_t filter)
{
	hwrm_cfa_l2_filter_free_input_t req = { 0 };
	int ret;

	bnxt_hwrm_init_header(bp, &req, HWRM_CFA_L2_FILTER_FREE);

	req.l2_filter_id = LE_64(filter);

	mutex_enter(&bp->hwrm_lock);
	ret = bnxt_hwrm_send_message(bp, &req, sizeof (req), 0);
	mutex_exit(&bp->hwrm_lock);

	return (ret);
}

int
bnxt_hwrm_vnic_rss_cfg(bnxt_t *bp)
{
	hwrm_vnic_rss_cfg_input_t req = { 0 };
	int ret;

	bnxt_hwrm_init_header(bp, &req, HWRM_VNIC_RSS_CFG);

	req.hash_mode_flags = HWRM_FUNC_SPD_CFG_INPUT_HASH_MODE_FLAGS_DEFAULT;
	req.hash_type = LE_32(bp->vnic.rss_hash_type);
	req.ring_grp_tbl_addr = LE_64(bp->vnic.rss_grp_tbl.pa);
	req.hash_key_tbl_addr = LE_64(bp->vnic.rss_hash_key_tbl.pa);
	req.rss_ctx_idx = LE_16(bp->vnic.rss_id);

	mutex_enter(&bp->hwrm_lock);
	ret = bnxt_hwrm_send_message(bp, &req, sizeof (req), 0);
	mutex_exit(&bp->hwrm_lock);

	return (ret);
}

int
bnxt_hwrm_init(bnxt_t *bp)
{
	if (bnxt_dma_alloc(bp, &bp->hwrm_reply,
	    &bnxt_acc_attr, BNXT_HWRM_BUFFER_SIZE) != 0)
		return (1);

	mutex_init(&bp->hwrm_lock, NULL, MUTEX_DRIVER, NULL);
	/*
	 * We need to assign a default timeout time.  For the moment, we
	 * basically opt to wait for 1 second in 10ms ticks.
	 */
	bp->hwrm_timeout = BNXT_HWRM_DEFAULT_TIMEOUT;
	bp->hwrm_max_req = HWRM_MAX_REQ_LEN;
	return (0);
}

void
bnxt_hwrm_fini(bnxt_t *bp)
{
	bnxt_dma_free(&bp->hwrm_reply);
	mutex_destroy(&bp->hwrm_lock);
}
