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

#include "bnxt.h"

ddi_device_acc_attr_t bnxt_acc_attr = {
	DDI_DEVICE_ATTR_V1,
	DDI_NEVERSWAP_ACC,
	DDI_STRICTORDER_ACC
};

ddi_device_acc_attr_t bnxt_db_acc_attr = {
	DDI_DEVICE_ATTR_V1,
	DDI_STRUCTURE_LE_ACC,
	DDI_STRICTORDER_ACC
};

/* Used for ring descriptors and data buffers */
ddi_dma_attr_t bnxt_dma_attr = {
	.dma_attr_version = DMA_ATTR_V0,
	.dma_attr_addr_lo = 0,
	.dma_attr_addr_hi = UINT64_MAX,
	.dma_attr_count_max = UINT32_MAX,
	.dma_attr_align = 0x10,
	.dma_attr_burstsizes = 0xfff,
	.dma_attr_seg = UINT32_MAX,
	.dma_attr_minxfer = 0x1,
	.dma_attr_maxxfer = UINT32_MAX,
	.dma_attr_sgllen = 1,
	.dma_attr_granular = 1,
	.dma_attr_flags = 0
};

void
bnxt_log(bnxt_t *bp, const char *fmt, ...)
{
	va_list ap;

	va_start(ap, fmt);
	vdev_err(bp->dip, CE_NOTE, fmt, ap);
	va_end(ap);
}

void
bnxt_error(bnxt_t *bp, const char *fmt, ...)
{
	va_list ap;

	va_start(ap, fmt);
	vdev_err(bp->dip, CE_WARN, fmt, ap);
	va_end(ap);
}

void
bnxt_dma_free(bnxt_dma_buf_t *bdb)
{
	if (bdb->nck > 0) {
		VERIFY3P(bdb->dma_hdl, !=, NULL);
		(void) ddi_dma_unbind_handle(bdb->dma_hdl);
		bzero(&bdb->ck, sizeof (ddi_dma_cookie_t));
		bdb->len = 0;
		bdb->nck = 0;
	}

	if (bdb->acc_hdl != NULL) {
		ddi_dma_mem_free(&bdb->acc_hdl);
		bdb->acc_hdl = NULL;
		bdb->va = NULL;
	}

	if (bdb->dma_hdl != NULL) {
		ddi_dma_free_handle(&bdb->dma_hdl);
		bdb->dma_hdl = NULL;
	}

	ASSERT3P(bdb->va, ==, NULL);
	ASSERT3U(bdb->nck, ==, 0);
	ASSERT3U(bdb->len, ==, 0);
}

int
bnxt_dma_alloc(bnxt_t *bp, bnxt_dma_buf_t *bdb, ddi_device_acc_attr_t *accp,
    size_t size)
{
	size_t len;

	if (ddi_dma_alloc_handle(bp->dip, &bnxt_dma_attr, DDI_DMA_SLEEP, NULL,
	    &bdb->dma_hdl) != 0) {
		bnxt_error(bp, "!failed to allocate dma handle");
		bdb->dma_hdl = NULL;
		return (1);
	}
	if (ddi_dma_mem_alloc(bdb->dma_hdl, size, accp,
	    DDI_DMA_CONSISTENT, DDI_DMA_SLEEP, NULL, &bdb->va,
	    &len, &bdb->acc_hdl) != DDI_SUCCESS) {
		bnxt_error(bp, "!failed to allocate dma memory");
		bdb->va = NULL;
		bdb->acc_hdl = NULL;
		bnxt_dma_free(bdb);
		return (1);
	}
	memset(bdb->va, 0, len);
	if (ddi_dma_addr_bind_handle(bdb->dma_hdl, NULL, bdb->va, len,
	    DDI_DMA_RDWR | DDI_DMA_CONSISTENT, DDI_DMA_SLEEP, NULL,
	    &bdb->ck, &bdb->nck) != 0) {
		bdb->nck = 0;
		memset(&bdb->ck, 0, sizeof (ddi_dma_cookie_t));
		bnxt_error(bp, "!failed to bind dma memory");
		bnxt_dma_free(bdb);
		return (1);
	}
	/* We want single segment for ring descriptors */
	VERIFY3U(bdb->nck, ==, 1);

	bdb->len = size;
	bdb->pa = bdb->ck.dmac_laddress;

	return (0);
}

static int
bnxt_mac_compare(const void *arg1, const void *arg2)
{
	const bnxt_mac_t *mac1 = arg1;
	const bnxt_mac_t *mac2 = arg2;
	int diff;

	diff = memcmp(mac1->macaddr, mac2->macaddr, ETHERADDRL);
	if (diff > 0)
		return (1);
	if (diff < 0)
		return (-1);
	return (0);
}

static int
bnxt_vlan_compare(const void *arg1, const void *arg2)
{
	const bnxt_vlan_t *vlan1 = arg1;
	const bnxt_vlan_t *vlan2 = arg2;

	if (vlan1->tag > vlan2->tag)
		return (1);
	if (vlan1->tag < vlan2->tag)
		return (-1);
	return (0);
}

#ifdef HAVE_KSENSOR
static int
bnxt_temp_read(void *arg, sensor_ioctl_scalar_t *sis)
{
	bnxt_t *bp = arg;

	bnxt_hwrm_temp_monitor_query(bp);

	if (bp->temp_valid) {
		sis->sis_unit = SENSOR_UNIT_CELSIUS;
		sis->sis_gran = 1;
		sis->sis_prec = 0;
		sis->sis_value = bp->temp;
		return (0);
	}
	return (EIO);
}

static int
bnxt_temp_om_read(void *arg, sensor_ioctl_scalar_t *sis)
{
	bnxt_t *bp = arg;

	bnxt_hwrm_temp_monitor_query(bp);

	if (bp->temp_om_valid) {
		sis->sis_unit = SENSOR_UNIT_CELSIUS;
		sis->sis_gran = 1;
		sis->sis_prec = 0;
		sis->sis_value = bp->temp_om;
		return (0);
	}
	return (EIO);
}

static int
bnxt_temp_phy_read(void *arg, sensor_ioctl_scalar_t *sis)
{
	bnxt_t *bp = arg;

	bnxt_hwrm_temp_monitor_query(bp);

	if (bp->temp_phy_valid) {
		sis->sis_unit = SENSOR_UNIT_CELSIUS;
		sis->sis_gran = 1;
		sis->sis_prec = 0;
		sis->sis_value = bp->temp_phy;
		return (0);
	}
	return (EIO);
}

static const ksensor_ops_t bnxt_temp_ops = {
	.kso_kind = ksensor_kind_temperature,
	.kso_scalar = bnxt_temp_read,
};

static const ksensor_ops_t bnxt_temp_om_ops = {
	.kso_kind = ksensor_kind_temperature,
	.kso_scalar = bnxt_temp_om_read,
};

static const ksensor_ops_t bnxt_temp_phy_ops = {
	.kso_kind = ksensor_kind_temperature,
	.kso_scalar = bnxt_temp_phy_read,
};
#endif

static void
bnxt_teardown(bnxt_t *bp)
{
	dev_info_t *dip = bp->dip;
	bnxt_mac_t *mac;
	bnxt_vlan_t *vlan;
	void *ck;
	int i;

	/* This should take care of all allocated HWRM resources */
	(void) bnxt_hwrm_func_reset(bp);
	(void) bnxt_hwrm_host_unregister(bp);
	bnxt_hwrm_fini(bp);

#ifdef HAVE_KSENSOR
	/* Destroy sensors */
	(void) ksensor_remove(bp->dip, KSENSOR_ALL_IDS);
#endif

	/* Unregister from MAC */
	bnxt_mac_unregister(bp);

	/* VNIC related resources */
	mutex_destroy(&bp->vnic.lock);
	/* RSS */
	if (bp->vnic.rss_hash_key_tbl.va != NULL)
		bnxt_dma_free(&bp->vnic.rss_hash_key_tbl);
	if (bp->vnic.rss_grp_tbl.va != NULL)
		bnxt_dma_free(&bp->vnic.rss_grp_tbl);
	/* Unicast */
	ck = NULL;
	while ((mac = avl_destroy_nodes(&bp->vnic.ucast, &ck)) != NULL)
		kmem_free(mac, sizeof (*mac));
	avl_destroy(&bp->vnic.ucast);
	/* Multicast */
	ck = NULL;
	while ((mac = avl_destroy_nodes(&bp->vnic.mcast, &ck)) != NULL)
		kmem_free(mac, sizeof (*mac));
	avl_destroy(&bp->vnic.mcast);
	if (bp->vnic.mcast_tbl.va != NULL)
		bnxt_dma_free(&bp->vnic.mcast_tbl);
	/* VLANs */
	ck = NULL;
	while ((vlan = avl_destroy_nodes(&bp->vnic.vlan, &ck)) != NULL)
		kmem_free(vlan, sizeof (*vlan));
	avl_destroy(&bp->vnic.vlan);
	if (bp->vnic.vlan_tbl.va != NULL)
		bnxt_dma_free(&bp->vnic.vlan_tbl);

	/* Teardown interrupts */
	if (bp->ihdl != NULL) {
		for (i = 0; i < bp->nintr; i++) {
			(void) ddi_intr_disable(bp->ihdl[i]);
			(void) ddi_intr_remove_handler(bp->ihdl[i]);
			(void) ddi_intr_free(bp->ihdl[i]);
		}
		kmem_free(bp->ihdl, bp->nintr_req * sizeof (ddi_intr_handle_t));
	}

	/* Teardown rings */
	for (int i = 0; i < bp->nrxq; i++) {
		bnxt_ring_t *rxr = &bp->rxr[i];

		if (rxr->kstat != NULL)
			kstat_delete(rxr->kstat);
		if (rxr->cp_stats.va != NULL)
			bnxt_dma_free(&rxr->cp_stats);
		if (rxr->cp_descs.va != NULL)
			bnxt_dma_free(&rxr->cp_descs);
		if (rxr->descs.va != NULL) {
			bnxt_dma_free(&rxr->descs);
			mutex_destroy(&rxr->lock);
		}
	}
	for (i = 0; i < bp->ntxq; i++) {
		bnxt_ring_t *txr = &bp->txr[i];

		if (txr->kstat != NULL)
			kstat_delete(txr->kstat);
		if (txr->cp_stats.va != NULL)
			bnxt_dma_free(&txr->cp_stats);
		if (txr->cp_descs.va != NULL)
			bnxt_dma_free(&txr->cp_descs);
		if (txr->descs.va != NULL) {
			bnxt_dma_free(&txr->descs);
			mutex_destroy(&txr->lock);
		}
	}
	if (bp->rxr != NULL)
		kmem_free(bp->rxr, bp->nrxq * sizeof (bnxt_ring_t));
	if (bp->grp != NULL)
		kmem_free(bp->grp, bp->nrxq * sizeof (bnxt_group_t));
	if (bp->txr != NULL)
		kmem_free(bp->txr, bp->ntxq * sizeof (bnxt_ring_t));

	if (&bp->dev_hdl != NULL)
		ddi_regs_map_free(&bp->dev_hdl);
	if (&bp->db_hdl != NULL)
		ddi_regs_map_free(&bp->db_hdl);
	if (bp->pci_hdl != NULL)
		pci_config_teardown(&bp->pci_hdl);

	kmem_free(bp, sizeof (bnxt_t));
	ddi_set_driver_private(dip, NULL);
}

static int
bnxt_attach(dev_info_t *dip, ddi_attach_cmd_t cmd)
{
	bnxt_t *bp;
	off_t devsz, dbsz;
	int itypes;

	if (cmd != DDI_ATTACH)
		return (DDI_FAILURE);

	bp = kmem_zalloc(sizeof (bnxt_t), KM_SLEEP);
	bp->dip = dip;
	ddi_set_driver_private(dip, bp);

	bp->started = false;

	if (pci_config_setup(dip, &bp->pci_hdl) != DDI_SUCCESS) {
		bnxt_error(bp, "!failed to map PCI configurations");
		goto cleanup;
	}

	if (ddi_dev_regsize(bp->dip, BNXT_BAR_DEV, &devsz) != DDI_SUCCESS ||
	    ddi_dev_regsize(bp->dip, BNXT_BAR_DB, &dbsz) != DDI_SUCCESS ||
	    ddi_regs_map_setup(bp->dip, BNXT_BAR_DEV, &bp->dev_base, 0,
	    devsz, &bnxt_acc_attr, &bp->dev_hdl) != DDI_SUCCESS ||
	    ddi_regs_map_setup(bp->dip, BNXT_BAR_DB, &bp->db_base, 0,
	    dbsz, &bnxt_db_acc_attr, &bp->db_hdl) != DDI_SUCCESS) {
		bnxt_error(bp, "!failed to map device registers");
		goto cleanup;
	}

	if (bnxt_hwrm_init(bp) != 0) {
		bnxt_error(bp, "!failed to allocate HWRM resources");
		goto cleanup;
	}

	if (bnxt_hwrm_version_get(bp) != 0) {
		bnxt_error(bp, "!failed to get HWRM version");
		goto cleanup;
	}

	/*
	 * First check the firmware rev.  If older firmware is in place, then we
	 * need to error out.
	 */
	bnxt_log(bp, "!HWRM interface %d.%d.%d.%d",
	    bp->ver.hwrm_intf_major, bp->ver.hwrm_intf_minor,
	    bp->ver.hwrm_intf_build, bp->ver.hwrm_intf_patch);
	if (bp->ver.hwrm_intf_major < 1) {
		bnxt_error(bp,
		    "!unsupported HWRM major version %d (required 1)",
		    bp->ver.hwrm_intf_major);
		goto cleanup;
	}

	/*
	 * The HWRM offers a secondary short command mode that is not
	 * implemented at the moment.  If hardware marks that it requires it in
	 * the version structures, fail attach.
	 */
	if ((bp->ver.dev_caps_cfg &
	    HWRM_VER_GET_OUTPUT_DEV_CAPS_CFG_SHORT_CMD_SUPPORTED) != 0 &&
	    (bp->ver.dev_caps_cfg &
	    HWRM_VER_GET_OUTPUT_DEV_CAPS_CFG_SHORT_CMD_REQUIRED) != 0) {
		bnxt_error(bp,
		    "!HWRM requires unsupported short command mode");
		goto cleanup;
	}

	/* Update default timeout */
	if (bp->ver.def_req_timeout != 0)
		bp->hwrm_timeout = bp->ver.def_req_timeout;
	if (bp->ver.max_req_win_len != 0)
		bp->hwrm_max_req = bp->ver.max_req_win_len;

	/*
	 * Reset the function into a known state to configure all needed
	 * resources.  Treat failure as not fatal.
	 */
	if (bnxt_hwrm_func_reset(bp) != 0)
		bnxt_error(bp, "!failed to reset PCI function");

	if (bnxt_hwrm_nvm_info_get(bp) != 0) {
		bnxt_error(bp, "!failed to get nvm info");
		goto cleanup;
	}

	/* Register with the device */
	if (bnxt_hwrm_host_register(bp) != 0) {
		bnxt_error(bp, "!failed to register host driver");
		goto cleanup;
	}

	/* XXX We should come back and gather LED caps */
	if (bnxt_hwrm_func_qcaps(bp) != 0) {
		bnxt_error(bp, "!failed to get PF queue caps");
		goto cleanup;
	}

	if (bnxt_hwrm_func_qcfg(bp) != 0) {
		bnxt_error(bp, "!failed to get PF queue config");
		goto cleanup;
	}

	if (bnxt_hwrm_queue_qportcfg(bp) != 0) {
		bnxt_error(bp, "!failed to get PF port queue config");
		goto cleanup;
	}

	/* Setup interrupts */
	if (ddi_intr_get_supported_types(bp->dip, &itypes) != DDI_SUCCESS ||
	    (itypes & DDI_INTR_TYPE_MSIX) == 0) {
		bnxt_error(bp, "!only MSI-X interrupts are supported");
		goto cleanup;
	}
	if (ddi_intr_get_navail(bp->dip, DDI_INTR_TYPE_MSIX,
	    &bp->nintr_req) != DDI_SUCCESS || bp->nintr_req == 0) {
		bnxt_error(bp, "!failed to get available interrupts");
		goto cleanup;
	}
	bp->ihdl = kmem_zalloc(bp->nintr_req *
	    sizeof (ddi_intr_handle_t), KM_SLEEP);
	if (ddi_intr_alloc(bp->dip, bp->ihdl, DDI_INTR_TYPE_MSIX, 0,
	    bp->nintr_req, &bp->nintr,
	    DDI_INTR_ALLOC_NORMAL) != DDI_SUCCESS) {
		bnxt_error(bp, "!failed to allocate interrupt handles");
		bp->nintr = 0;
		goto cleanup;
	}
	if (ddi_intr_get_cap(bp->ihdl[0], &bp->intr_caps) != DDI_SUCCESS ||
	    ddi_intr_get_pri(bp->ihdl[0], &bp->intr_pri) != DDI_SUCCESS) {
		bnxt_error(bp, "!failed to get interrupt cap/pri");
		goto cleanup;
	}

	bp->nrxq = bp->nintr / 2 + bp->nintr % 2;
	bp->ntxq = bp->nintr - bp->nrxq;
	bnxt_log(bp, "!using %d MSI-X vectors; %d/%d RX/TX queues", bp->nintr,
	    bp->nrxq, bp->ntxq);

	/* RX */
	bp->rxr = kmem_zalloc(bp->nrxq * sizeof (bnxt_ring_t), KM_SLEEP);
	bp->grp = kmem_zalloc(bp->nrxq * sizeof (bnxt_group_t), KM_SLEEP);
	for (int i = 0; i < bp->nrxq; i++) {
		bnxt_ring_t *rxr = &bp->rxr[i];
		bnxt_group_t *grp = &bp->grp[i];
		bnxt_rx_stats_t *stats = &rxr->rx.stats;
		kstat_t *ksp;
		char name[32];

		mutex_init(&rxr->lock, NULL, MUTEX_DRIVER, NULL);
		/* Ring stats */
		(void) snprintf(name, sizeof (name), "rxr%u", i);
		ksp = kstat_create(BNXT_MOD_NAME, ddi_get_instance(dip),
		    name, "net", KSTAT_TYPE_NAMED, sizeof (bnxt_rx_stats_t) /
		    sizeof (kstat_named_t), KSTAT_FLAG_VIRTUAL);
		if (ksp == NULL) {
			bnxt_error(bp, "!failed to create rxr%d kstats", i);
			goto cleanup;
		}
		rxr->kstat = ksp;
		ksp->ks_data = stats;
		/* Store ring pointer for debugging purposes */
		kstat_named_init(&stats->addr, "addr", KSTAT_DATA_UINT64);
		stats->addr.value.ui64 = (uintptr_t)rxr;
		kstat_named_init(&stats->rbytes, "rbytes", KSTAT_DATA_UINT64);
		kstat_named_init(&stats->ipackets, "ipackets",
		    KSTAT_DATA_UINT64);
		kstat_named_init(&stats->nobufs, "nobufs", KSTAT_DATA_UINT64);
		kstat_install(ksp);
		/* Ring group */
		grp->grp_id = (uint16_t)HWRM_NA_SIGNATURE;
		grp->stat_ctx_id = (uint16_t)HWRM_NA_SIGNATURE;
		grp->cpr_id = (uint16_t)HWRM_NA_SIGNATURE;
		grp->rxr_id = (uint16_t)HWRM_NA_SIGNATURE;
		grp->agr_id = (uint16_t)HWRM_NA_SIGNATURE;
		/* RX ring */
		rxr->bp = bp;
		rxr->type = BNXT_RING_RX;
		rxr->idx = i;
		rxr->id = i;
		rxr->phys_id = (uint16_t)HWRM_NA_SIGNATURE;
		rxr->db = rxr->id * 0x80;
		rxr->ndesc = BNXT_RX_RING_LEN;
		rxr->nbuf = rxr->ndesc * 8;
		if (bnxt_dma_alloc(bp, &rxr->descs, &bnxt_acc_attr,
		    rxr->ndesc * sizeof (rx_prod_pkt_bd_t)) != 0) {
			bnxt_error(bp, "!failed to create rx ring %d", i);
			goto cleanup;
		}
		/* RX CP ring */
		rxr->cp_phys_id = (uint16_t)HWRM_NA_SIGNATURE;
		rxr->cp_stat_ctx_id = HWRM_NA_SIGNATURE;
		rxr->cp_ndesc = BNXT_RX_CP_RING_LEN;
		if (bnxt_dma_alloc(bp, &rxr->cp_stats, &bnxt_acc_attr,
		    sizeof (ctx_hw_stats_t)) != 0 ||
		    bnxt_dma_alloc(bp, &rxr->cp_descs, &bnxt_acc_attr,
		    rxr->cp_ndesc * sizeof (cmpl_base_t)) != 0) {
			bnxt_error(bp, "!failed to create rx cp ring %d", i);
			goto cleanup;
		}
		/* RX CP intr handler */
		rxr->cp_ihdl = bp->ihdl[i];
		if (ddi_intr_add_handler(rxr->cp_ihdl, bnxt_rx_intr,
		    rxr, NULL) != DDI_SUCCESS ||
		    ddi_intr_enable(rxr->cp_ihdl) != DDI_SUCCESS) {
			bnxt_error(bp,
			    "!failed to setup intr handler for rx ring %d", i);
			goto cleanup;
		}
	}

	/* TX */
	bp->tx_recycle_thresh = BNXT_TX_RECYCLE_THRESHOLD;
	bp->tx_notify_thresh = BNXT_TX_NOTIFY_THRESHOLD;
	bp->tx_gap = BNXT_TX_GAP;

	bp->txr = kmem_zalloc(bp->ntxq * sizeof (bnxt_ring_t), KM_SLEEP);
	for (int i = 0; i < bp->ntxq; i++) {
		bnxt_ring_t *txr = &bp->txr[i];
		bnxt_tx_stats_t *stats = &txr->tx.stats;
		kstat_t *ksp;
		char name[32];

		mutex_init(&txr->lock, NULL, MUTEX_DRIVER, NULL);
		/* Ring stats */
		(void) snprintf(name, sizeof (name), "txr%u", i);
		ksp = kstat_create(BNXT_MOD_NAME, ddi_get_instance(dip),
		    name, "net", KSTAT_TYPE_NAMED, sizeof (bnxt_tx_stats_t) /
		    sizeof (kstat_named_t), KSTAT_FLAG_VIRTUAL);
		if (ksp == NULL) {
			bnxt_error(bp, "!failed to create txr%d kstats", i);
			goto cleanup;
		}
		txr->kstat = ksp;
		ksp->ks_data = stats;
		/* Store ring pointer for debugging purposes */
		kstat_named_init(&stats->addr, "addr", KSTAT_DATA_UINT64);
		stats->addr.value.ui64 = (uintptr_t)txr;
		kstat_named_init(&stats->obytes, "obytes", KSTAT_DATA_UINT64);
		kstat_named_init(&stats->opackets, "opackets",
		    KSTAT_DATA_UINT64);
		kstat_named_init(&stats->nobufs, "nobufs", KSTAT_DATA_UINT64);
		kstat_named_init(&stats->nodescs, "nodescs", KSTAT_DATA_UINT64);
		kstat_install(ksp);
		/* TX ring */
		txr->bp = bp;
		txr->idx = i;
		txr->type = BNXT_RING_TX;
		txr->id = bp->nrxq + i;
		txr->db = txr->id * 0x80;
		txr->phys_id = (uint16_t)HWRM_NA_SIGNATURE;
		txr->ndesc = BNXT_TX_RING_LEN;
		txr->nbuf = txr->ndesc;
		if (bnxt_dma_alloc(bp, &txr->descs, &bnxt_acc_attr,
		    txr->ndesc * sizeof (tx_bd_short_t)) != 0) {
			bnxt_error(bp, "!failed to create tx ring %d", i);
			goto cleanup;
		}
		/* TX CP ring */
		txr->cp_phys_id = (uint16_t)HWRM_NA_SIGNATURE;
		txr->cp_stat_ctx_id = HWRM_NA_SIGNATURE;
		txr->cp_ndesc = BNXT_TX_CP_RING_LEN;
		if (bnxt_dma_alloc(bp, &txr->cp_stats, &bnxt_acc_attr,
		    sizeof (ctx_hw_stats_t)) != 0 ||
		    bnxt_dma_alloc(bp, &txr->cp_descs, &bnxt_acc_attr,
		    txr->cp_ndesc * sizeof (cmpl_base_t)) != 0) {
			bnxt_error(bp, "!failed to create tx cp ring %d", i);
			goto cleanup;
		}
		/* TX CP intr handler */
		txr->cp_ihdl = bp->ihdl[i + bp->nrxq];
		if (ddi_intr_add_handler(txr->cp_ihdl, bnxt_tx_intr,
		    txr, NULL) != DDI_SUCCESS ||
		    ddi_intr_enable(txr->cp_ihdl) != DDI_SUCCESS) {
			bnxt_error(bp,
			    "!failed to setup intr handler for tx ring %d", i);
			goto cleanup;
		}
	}

	/* VNIC */
	mutex_init(&bp->vnic.lock, NULL, MUTEX_DRIVER, NULL);
	/* Default MRU */
	bp->vnic.mru = BNXT_MTU_DEF + sizeof (struct ether_vlan_header) +
	    ETHERFCSL;
	bp->vnic.id = (uint16_t)HWRM_NA_SIGNATURE;
	bp->vnic.cos_rule = (uint16_t)HWRM_NA_SIGNATURE;
	bp->vnic.lb_rule = (uint16_t)HWRM_NA_SIGNATURE;
	bp->vnic.rss_id = (uint16_t)HWRM_NA_SIGNATURE;
	bp->vnic.rss_hash_type =
	    HWRM_VNIC_RSS_CFG_INPUT_HASH_TYPE_IPV4 |
	    HWRM_VNIC_RSS_CFG_INPUT_HASH_TYPE_TCP_IPV4 |
	    HWRM_VNIC_RSS_CFG_INPUT_HASH_TYPE_UDP_IPV4 |
	    HWRM_VNIC_RSS_CFG_INPUT_HASH_TYPE_IPV6 |
	    HWRM_VNIC_RSS_CFG_INPUT_HASH_TYPE_TCP_IPV6 |
	    HWRM_VNIC_RSS_CFG_INPUT_HASH_TYPE_UDP_IPV6;

	/* The VNIC RSS Hash Key */
 	if (bnxt_dma_alloc(bp, &bp->vnic.rss_hash_key_tbl, &bnxt_acc_attr,
	    HW_HASH_KEY_SIZE) != 0)
		goto cleanup;
	(void) random_get_pseudo_bytes((uint8_t *)bp->vnic.rss_hash_key_tbl.va,
	    HW_HASH_KEY_SIZE);
	BNXT_DMA_SYNC(bp->vnic.rss_hash_key_tbl, DDI_DMA_SYNC_FORDEV);
	/* Allocate the RSS tables */
	if (bnxt_dma_alloc(bp, &bp->vnic.rss_grp_tbl, &bnxt_acc_attr,
	    HW_HASH_INDEX_SIZE * sizeof (uint16_t)) != 0)
		goto cleanup;
	memset(bp->vnic.rss_grp_tbl.va, 0xff, bp->vnic.rss_grp_tbl.len);
	BNXT_DMA_SYNC(bp->vnic.rss_grp_tbl, DDI_DMA_SYNC_FORDEV);

	/* Unicast */
	avl_create(&bp->vnic.ucast, bnxt_mac_compare, sizeof (bnxt_mac_t),
	    offsetof(bnxt_mac_t, avl));
	/* Multicast */
	if (bnxt_dma_alloc(bp, &bp->vnic.mcast_tbl, &bnxt_acc_attr,
	    BNXT_MAX_MC_ADDRS * ETHERADDRL) != 0)
		goto cleanup;
	avl_create(&bp->vnic.mcast, bnxt_mac_compare, sizeof (bnxt_mac_t),
	    offsetof(bnxt_mac_t, avl));
	/* VLANs */
	if (bnxt_dma_alloc(bp, &bp->vnic.vlan_tbl, &bnxt_acc_attr,
	    BNXT_MAX_VLAN_TAGS * VLAN_TAGSZ) != 0)
		goto cleanup;
	avl_create(&bp->vnic.vlan, bnxt_vlan_compare, sizeof (bnxt_vlan_t),
	    offsetof(bnxt_vlan_t, avl));
	/* Default MTU */
	bp->mtu = BNXT_MTU_DEF;

	/* Finally register with MAC */
	if (bnxt_mac_register(bp) != 0) {
		bnxt_error(bp, "!failed to register with MAC");
		goto cleanup;
	}

#ifdef HAVE_KSENSOR
	/* Get initial temperature readings to check which sensors to create */
	bnxt_hwrm_temp_monitor_query(bp);
	/* Create temperature sensors */
	if (bp->temp_valid) {
		(void) ksensor_create_scalar_pcidev(bp->dip,
		    SENSOR_KIND_TEMPERATURE, &bnxt_temp_ops, bp,
		    "temp", &bp->id_temp);
	}
	if (bp->temp_om_valid) {
		(void) ksensor_create_scalar_pcidev(bp->dip,
		    SENSOR_KIND_TEMPERATURE, &bnxt_temp_om_ops, bp,
		    "temp_om", &bp->id_temp_om);
	}
	if (bp->temp_phy_valid) {
		(void) ksensor_create_scalar_pcidev(bp->dip,
		    SENSOR_KIND_TEMPERATURE, &bnxt_temp_phy_ops, bp,
		    "temp_phy", &bp->id_temp_phy);
	}
#endif

	/* Get initial link state */
	bnxt_link_update(bp);

	return (DDI_SUCCESS);

cleanup:
	bnxt_teardown(bp);

	return (DDI_FAILURE);
}

static int
bnxt_detach(dev_info_t *dip, ddi_detach_cmd_t cmd)
{
	bnxt_t *bp;

	if (cmd != DDI_DETACH)
		return (DDI_FAILURE);

	bp = ddi_get_driver_private(dip);
	if (bp == NULL) {
		dev_err(dip, CE_NOTE,
		    "!detach called with NULL pointer");
		return (DDI_FAILURE);
	}

	bnxt_teardown(bp);
	return (DDI_SUCCESS);
}

static struct cb_ops bnxt_cb_ops = {
	.cb_open = nulldev,
	.cb_close = nulldev,
	.cb_strategy = nodev,
	.cb_print = nodev,
	.cb_dump = nodev,
	.cb_read = nodev,
	.cb_write = nodev,
	.cb_ioctl = nodev,
	.cb_devmap = nodev,
	.cb_mmap = nodev,
	.cb_segmap = nodev,
	.cb_chpoll = nochpoll,
	.cb_prop_op = ddi_prop_op,
	.cb_str = NULL,
	.cb_flag = D_MP | D_HOTPLUG,
	.cb_rev = CB_REV,
	.cb_aread = nodev,
	.cb_awrite = nodev,
};

static struct dev_ops bnxt_dev_ops = {
	.devo_rev = DEVO_REV,
	.devo_refcnt = 0,
	.devo_getinfo = NULL,
	.devo_identify = nulldev,
	.devo_probe = nulldev,
	.devo_attach = bnxt_attach,
	.devo_detach = bnxt_detach,
	.devo_reset = nodev,
	.devo_cb_ops = &bnxt_cb_ops,
	.devo_bus_ops = NULL,
	.devo_power = ddi_power,
	.devo_quiesce = ddi_quiesce_not_supported,
};

static struct modldrv bnxt_modldrv = {
	.drv_modops = &mod_driverops,
	.drv_linkinfo = "Broadcom NetXtreme-C/E network driver",
	.drv_dev_ops = &bnxt_dev_ops,
};

static struct modlinkage bnxt_modlinkage = {
	.ml_rev = MODREV_1,
	.ml_linkage = &bnxt_modldrv,
};

int
_init(void)
{
	int status;

	mac_init_ops(&bnxt_dev_ops, BNXT_MOD_NAME);
	status = mod_install(&bnxt_modlinkage);
	if (status != DDI_SUCCESS)
		mac_fini_ops(&bnxt_dev_ops);
	return (status);
}

int
_info(struct modinfo *modinfop)
{
	return (mod_info(&bnxt_modlinkage, modinfop));
}

int
_fini(void)
{
	int status;

	status = mod_remove(&bnxt_modlinkage);
	if (status == DDI_SUCCESS)
		mac_fini_ops(&bnxt_dev_ops);
	return (status);
}
