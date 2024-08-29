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

/* Note: knows about bp */
#define	_SPD(speed)	((bp->link.support_speeds & (speed)) != 0)

link_duplex_t
bnxt_duplex_to_gld(uint8_t duplex_cfg)
{
	switch (duplex_cfg) {
	case HWRM_PORT_PHY_QCFG_OUTPUT_DUPLEX_CFG_HALF:
		return (LINK_DUPLEX_HALF);
	case HWRM_PORT_PHY_QCFG_OUTPUT_DUPLEX_CFG_FULL:
		return (LINK_DUPLEX_FULL);
	default:
		return (LINK_DUPLEX_UNKNOWN);
	}
}

#ifdef HAVE_MAC_MEDIA
mac_ether_media_t
bnxt_media_to_gld(uint8_t phy_type)
{
	switch (phy_type) {
	case HWRM_PORT_PHY_QCFG_OUTPUT_PHY_TYPE_BASECR:
		return (ETHER_MEDIA_10GBASE_CR);
	case HWRM_PORT_PHY_QCFG_OUTPUT_PHY_TYPE_BASEKR:
	case HWRM_PORT_PHY_QCFG_OUTPUT_PHY_TYPE_BASEKR2:
	case HWRM_PORT_PHY_QCFG_OUTPUT_PHY_TYPE_BASEKR4:
	case HWRM_PORT_PHY_QCFG_OUTPUT_PHY_TYPE_BASEKX:
		return (ETHER_MEDIA_10GBASE_KR);
	case HWRM_PORT_PHY_QCFG_OUTPUT_PHY_TYPE_BASELR:
		return (ETHER_MEDIA_10GBASE_LR);
	case HWRM_PORT_PHY_QCFG_OUTPUT_PHY_TYPE_BASESR:
		return (ETHER_MEDIA_10GBASE_SR);
	case HWRM_PORT_PHY_QCFG_OUTPUT_PHY_TYPE_BASET:
	case HWRM_PORT_PHY_QCFG_OUTPUT_PHY_TYPE_BASETE:
		return (ETHER_MEDIA_10GBASE_T);
	case HWRM_PORT_PHY_QCFG_OUTPUT_PHY_TYPE_SGMIIEXTPHY:
		return (ETHER_MEDIA_1000_SGMII);
	case HWRM_PORT_PHY_QCFG_OUTPUT_PHY_TYPE_1G_BASET:
		return (ETHER_MEDIA_1000BASE_T);
	case HWRM_PORT_PHY_QCFG_OUTPUT_PHY_TYPE_1G_BASESX:
		return (ETHER_MEDIA_1000BASE_SX);
	case HWRM_PORT_PHY_QCFG_OUTPUT_PHY_TYPE_1G_BASECX:
		return (ETHER_MEDIA_1000BASE_CX);
	case HWRM_PORT_PHY_QCFG_OUTPUT_PHY_TYPE_25G_BASECR_CA_L:
	case HWRM_PORT_PHY_QCFG_OUTPUT_PHY_TYPE_25G_BASECR_CA_S:
	case HWRM_PORT_PHY_QCFG_OUTPUT_PHY_TYPE_25G_BASECR_CA_N:
		return (ETHER_MEDIA_25GBASE_CR);
	case HWRM_PORT_PHY_QCFG_OUTPUT_PHY_TYPE_25G_BASESR:
		return (ETHER_MEDIA_25GBASE_SR);
	case HWRM_PORT_PHY_QCFG_OUTPUT_PHY_TYPE_40G_BASECR4:
		return (ETHER_MEDIA_40GBASE_CR4);
	case HWRM_PORT_PHY_QCFG_OUTPUT_PHY_TYPE_40G_BASESR4:
		return (ETHER_MEDIA_40GBASE_SR4);
	case HWRM_PORT_PHY_QCFG_OUTPUT_PHY_TYPE_40G_BASELR4:
		return (ETHER_MEDIA_40GBASE_LR4);
	case HWRM_PORT_PHY_QCFG_OUTPUT_PHY_TYPE_40G_BASEER4:
		return (ETHER_MEDIA_40GBASE_ER4);
	case HWRM_PORT_PHY_QCFG_OUTPUT_PHY_TYPE_40G_ACTIVE_CABLE:
		return (ETHER_MEDIA_UNKNOWN);
	case HWRM_PORT_PHY_QCFG_OUTPUT_PHY_TYPE_50G_BASECR:
		return (ETHER_MEDIA_50GBASE_CR);
	case HWRM_PORT_PHY_QCFG_OUTPUT_PHY_TYPE_50G_BASEER:
		return (ETHER_MEDIA_50GBASE_ER);
	case HWRM_PORT_PHY_QCFG_OUTPUT_PHY_TYPE_50G_BASELR:
		return (ETHER_MEDIA_50GBASE_LR);
	case HWRM_PORT_PHY_QCFG_OUTPUT_PHY_TYPE_50G_BASESR:
		return (ETHER_MEDIA_50GBASE_SR);
	case HWRM_PORT_PHY_QCFG_OUTPUT_PHY_TYPE_100G_BASECR2:
		return (ETHER_MEDIA_100GBASE_CR2);
	case HWRM_PORT_PHY_QCFG_OUTPUT_PHY_TYPE_100G_BASEER2:
		return (ETHER_MEDIA_UNKNOWN);
	case HWRM_PORT_PHY_QCFG_OUTPUT_PHY_TYPE_100G_BASELR2:
		return (ETHER_MEDIA_UNKNOWN);
	case HWRM_PORT_PHY_QCFG_OUTPUT_PHY_TYPE_100G_BASESR2:
		return (ETHER_MEDIA_100GBASE_SR2);
	case HWRM_PORT_PHY_QCFG_OUTPUT_PHY_TYPE_100G_BASECR4:
		return (ETHER_MEDIA_100GBASE_CR4);
	case HWRM_PORT_PHY_QCFG_OUTPUT_PHY_TYPE_100G_BASEER4:
		return (ETHER_MEDIA_UNKNOWN);
	case HWRM_PORT_PHY_QCFG_OUTPUT_PHY_TYPE_100G_BASELR4:
		return (ETHER_MEDIA_100GBASE_LR4);
	case HWRM_PORT_PHY_QCFG_OUTPUT_PHY_TYPE_100G_BASESR4:
		return (ETHER_MEDIA_100GBASE_SR4);
	case HWRM_PORT_PHY_QCFG_OUTPUT_PHY_TYPE_100G_BASESR10:
		return (ETHER_MEDIA_100GBASE_SR10);
	case HWRM_PORT_PHY_QCFG_OUTPUT_PHY_TYPE_200G_BASECR4:
		return (ETHER_MEDIA_200GBASE_CR4);
	case HWRM_PORT_PHY_QCFG_OUTPUT_PHY_TYPE_200G_BASEER4:
		return (ETHER_MEDIA_200GBASE_ER4);
	case HWRM_PORT_PHY_QCFG_OUTPUT_PHY_TYPE_200G_BASELR4:
		return (ETHER_MEDIA_200GBASE_LR4);
	case HWRM_PORT_PHY_QCFG_OUTPUT_PHY_TYPE_200G_BASESR4:
		return (ETHER_MEDIA_200GBASE_SR4);
	case HWRM_PORT_PHY_QCFG_OUTPUT_PHY_TYPE_UNKNOWN:
	default:
		return (ETHER_MEDIA_UNKNOWN);
	}
}
#endif

link_state_t
bnxt_state_to_gld(uint8_t state)
{
	switch (state) {
	case HWRM_PORT_PHY_QCFG_OUTPUT_LINK_NO_LINK:
	case HWRM_PORT_PHY_QCFG_OUTPUT_LINK_SIGNAL:
		return (LINK_STATE_DOWN);
	case HWRM_PORT_PHY_QCFG_OUTPUT_LINK_LINK:
		return (LINK_STATE_UP);
	default:
		return (LINK_STATE_UNKNOWN);
	}
}

void
bnxt_link_update(bnxt_t *bp)
{
	(void) bnxt_hwrm_port_phy_qcfg(bp);

	mac_link_update(bp->mh, bp->link.state);
}

static int
bnxt_m_stat(void *arg, uint_t stat, uint64_t *val)
{
	bnxt_t *bp = arg;

	switch (stat) {
	case MAC_STAT_IFSPEED:
		*val = bp->link.speed;
		break;
	case ETHER_STAT_LINK_DUPLEX:
		*val = bp->link.duplex;
		break;

	case ETHER_STAT_CAP_10HDX:
		*val = _SPD(HWRM_PORT_PHY_QCFG_OUTPUT_SUPPORT_SPEEDS_10MBHD);
		break;
	case ETHER_STAT_CAP_10FDX:
		*val = _SPD(HWRM_PORT_PHY_QCFG_OUTPUT_SUPPORT_SPEEDS_10MB);
		break;
	case ETHER_STAT_CAP_100HDX:
		*val = _SPD(HWRM_PORT_PHY_QCFG_OUTPUT_SUPPORT_SPEEDS_100MBHD);
		break;
	case ETHER_STAT_CAP_100FDX:
		*val = _SPD(HWRM_PORT_PHY_QCFG_OUTPUT_SUPPORT_SPEEDS_100MB);
		break;
	case ETHER_STAT_CAP_1000HDX:
		*val = _SPD(HWRM_PORT_PHY_QCFG_OUTPUT_SUPPORT_SPEEDS_1GBHD);
		break;
	case ETHER_STAT_CAP_1000FDX:
		*val = _SPD(HWRM_PORT_PHY_QCFG_OUTPUT_SUPPORT_SPEEDS_1GB);
		break;
	case ETHER_STAT_CAP_2500FDX:
		*val = _SPD(HWRM_PORT_PHY_QCFG_OUTPUT_SUPPORT_SPEEDS_2_5GB);
		break;
	case ETHER_STAT_CAP_10GFDX:
		*val = _SPD(HWRM_PORT_PHY_QCFG_OUTPUT_SUPPORT_SPEEDS_10GB);
		break;
	case ETHER_STAT_CAP_25GFDX:
		*val = _SPD(HWRM_PORT_PHY_QCFG_OUTPUT_SUPPORT_SPEEDS_25GB);
		break;
	case ETHER_STAT_CAP_40GFDX:
		*val = _SPD(HWRM_PORT_PHY_QCFG_OUTPUT_SUPPORT_SPEEDS_40GB);
		break;
	case ETHER_STAT_CAP_50GFDX:
		*val = _SPD(HWRM_PORT_PHY_QCFG_OUTPUT_SUPPORT_SPEEDS_50GB);
		break;
	case ETHER_STAT_CAP_100GFDX:
		*val = _SPD(HWRM_PORT_PHY_QCFG_OUTPUT_SUPPORT_SPEEDS_100GB);
		break;

	default:
		return (ENOTSUP);
	}
	return (0);
}

static int
bnxt_ring_rx_start(mac_ring_driver_t arg, uint64_t gen)
{
	bnxt_ring_t *rxr = (bnxt_ring_t *)arg;
	bnxt_t *bp = rxr->bp;
	bnxt_group_t *grp = &bp->grp[rxr->idx];

	/* Allocate RX ring */
	if (bnxt_hwrm_stat_ctx_alloc(rxr) != 0 ||
	    bnxt_hwrm_ring_alloc(rxr,
	    HWRM_RING_ALLOC_INPUT_RING_TYPE_L2_CMPL) != 0 ||
	    bnxt_hwrm_ring_alloc(rxr,
	    HWRM_RING_ALLOC_INPUT_RING_TYPE_RX) != 0) {
		bnxt_error(bp, "!failed to allocate rx ring %d", rxr->idx);
		return (EIO);
	}

	rxr->ndesc_free = rxr->ndesc;
	rxr->pidx = 0;
	rxr->rx.gen = gen;
	/* Reset completion ring */
	rxr->cp_cidx = UINT32_MAX;
	rxr->cp_v_bit = true;
	bnxt_cp_reset(rxr);
	/* Allocate ring group */
	grp->stat_ctx_id = rxr->cp_stat_ctx_id;
	grp->cpr_id = rxr->cp_phys_id;
	grp->rxr_id = rxr->phys_id;
	grp->agr_id = (uint16_t)HWRM_NA_SIGNATURE;
	if (bnxt_hwrm_ring_grp_alloc(bp, grp) != 0) {
		bnxt_error(bp, "!failed to allocate ring group %d", rxr->idx);
		return (EIO);
	}

	bnxt_buf_ring_alloc(rxr);
	bnxt_ring_intr_enable(rxr);

	/*
	 * XXX This is better done in group_start_post() if one existed.
	 * For the moment, do postaction after allocating last RX ring.
	 */
	if (rxr->idx == bp->nrxq - 1) {
		uint16_t *rgt;
		int i, j;

		/* Configure async notification on RX CP ring 0 */
		if (bnxt_hwrm_events_cfg_cr(&bp->rxr[0]) != 0) {
			bnxt_error(bp, "!failed to set async cp ring");
			return (EIO);
		}

		/* Configure VNIC */
		bp->vnic.def_grp = grp->grp_id;
		if (bnxt_hwrm_vnic_cfg(bp) != 0) {
			bnxt_error(bp, "!failed to configure vnic");
			return (EIO);
		}

		/* Initialize RSS group table */
		rgt = (uint16_t *)bp->vnic.rss_grp_tbl.va;
		for (i = 0, j = 0; i < HW_HASH_INDEX_SIZE; i++) {
			rgt[i] = LE_16(bp->grp[j].grp_id);
			if (++j == bp->nrxq)
				j = 0;
		}
		if (bnxt_hwrm_vnic_rss_cfg(bp) != 0) {
			bnxt_error(bp, "!failed to initialize rss");
			return (EIO);
		}

		/* Set initial RX mask */
		bp->vnic.rx_mask =
		    HWRM_CFA_L2_SET_RX_MASK_INPUT_MASK_BCAST |
		    HWRM_CFA_L2_SET_RX_MASK_INPUT_MASK_ANYVLAN_NONVLAN;
		if (bnxt_hwrm_vnic_set_rx_mask(bp) != 0) {
			bnxt_error(bp, "!failed to set vnic rx mask");
			return (EIO);
		}
	}

	return (0);
}

static void
bnxt_ring_rx_stop(mac_ring_driver_t arg)
{
	bnxt_ring_t *rxr = (bnxt_ring_t *)arg;
	bnxt_t *bp = rxr->bp;
	bnxt_group_t *grp = &bp->grp[rxr->idx];

	/* Free RX ring */
	mutex_enter(&rxr->lock);
	bnxt_ring_intr_disable(rxr);
	(void) bnxt_hwrm_ring_free(rxr, HWRM_RING_FREE_INPUT_RING_TYPE_RX);
	(void) bnxt_hwrm_ring_free(rxr, HWRM_RING_FREE_INPUT_RING_TYPE_L2_CMPL);
	(void) bnxt_hwrm_stat_ctx_free(rxr);
	bnxt_buf_ring_free(rxr);
	mutex_exit(&rxr->lock);
	/* Free ring group */
	(void) bnxt_hwrm_ring_grp_free(bp, grp);
}

static int
bnxt_ring_rx_intr_disable(mac_intr_handle_t arg)
{
	bnxt_ring_t *rxr = (bnxt_ring_t *)arg;

	mutex_enter(&rxr->lock);
	bnxt_ring_intr_disable(rxr);
	mutex_exit(&rxr->lock);

	return (0);
}

static int
bnxt_ring_rx_intr_enable(mac_intr_handle_t arg)
{
	bnxt_ring_t *rxr = (bnxt_ring_t *)arg;

	mutex_enter(&rxr->lock);
	bnxt_ring_intr_enable(rxr);
	mutex_exit(&rxr->lock);

	return (0);
}

int
bnxt_ring_rx_stat(mac_ring_driver_t rh, uint_t stat, uint64_t *val)
{
	/* TODO implement */
	return (ENOTSUP);
}

static void
bnxt_rx_info(void *arg, mac_ring_type_t rtype, const int gidx,
    const int ridx, mac_ring_info_t *infop, mac_ring_handle_t rh)
{
	bnxt_t *bp = arg;
	bnxt_ring_t *rxr;

	VERIFY3S(rtype, ==, MAC_RING_TYPE_RX);
	VERIFY3S(ridx, <, bp->nrxq);
	rxr = &bp->rxr[ridx];
	rxr->rh = rh;
	infop->mri_driver = (mac_ring_driver_t)rxr;
	infop->mri_start = bnxt_ring_rx_start;
	infop->mri_stop = bnxt_ring_rx_stop;
	infop->mri_intr.mi_handle = (mac_intr_handle_t)rxr;
	infop->mri_intr.mi_enable = bnxt_ring_rx_intr_enable;
	infop->mri_intr.mi_disable = bnxt_ring_rx_intr_disable;
	infop->mri_intr.mi_ddi_handle = rxr->cp_ihdl;
	infop->mri_poll = bnxt_ring_rx;
	infop->mri_stat = bnxt_ring_rx_stat;
}

static int
bnxt_rx_mac_add(void *arg, const uint8_t *mac)
{
	bnxt_t *bp = arg;
	bnxt_mac_t smac;
	bnxt_mac_t *emac;
	avl_index_t ai;
	uint64_t filter;

	if (ETHER_IS_MULTICAST(mac))
		return (EINVAL);

	mutex_enter(&bp->vnic.lock);
	/* Check if address already exists */
	memcpy(smac.macaddr, mac, ETHERADDRL);
	emac = avl_find(&bp->vnic.ucast, &smac, &ai);
	if (emac != NULL) {
		mutex_exit(&bp->vnic.lock);
		return (0);
	}
	/* Try allocating L2 filter first */
	if (bnxt_hwrm_vnic_filter_alloc(bp, mac, &filter) != 0) {
		bnxt_error(bp, "!failed to allocate l2 filter");
		mutex_exit(&bp->vnic.lock);
		return (EIO);
	}
	/* Create address node and add it to the tree */
	emac = kmem_zalloc(sizeof (*emac), KM_SLEEP);
	memcpy(emac->macaddr, mac, ETHERADDRL);
	emac->filter = filter;
	avl_insert(&bp->vnic.ucast, emac, ai);
	mutex_exit(&bp->vnic.lock);

	return (0);
}

static int
bnxt_rx_mac_rem(void *arg, const uint8_t *mac)
{
	bnxt_t *bp = arg;
	bnxt_mac_t smac;
	bnxt_mac_t *emac;

	if (ETHER_IS_MULTICAST(mac))
		return (EINVAL);

	mutex_enter(&bp->vnic.lock);
	/* Check if address exists */
	memcpy(smac.macaddr, mac, ETHERADDRL);
	emac = avl_find(&bp->vnic.ucast, &smac, NULL);
	if (emac == NULL) {
		mutex_exit(&bp->vnic.lock);
		return (ENOENT);
	}
	/* Free the L2 filter */
	(void) bnxt_hwrm_vnic_filter_free(bp, emac->filter);
	avl_remove(&bp->vnic.ucast, emac);
	kmem_free(emac, sizeof (*emac));
	mutex_exit(&bp->vnic.lock);

	return (0);
}

#if 0
/* FIXME needs work, does not pass traffic with vlan tag table populated */
static int
bnxt_rx_vlan(mac_group_driver_t arg, bool add, uint16_t vlan)
{
	bnxt_t *bp = (bnxt_t *)arg;
	bnxt_vlan_t svlan;
	bnxt_vlan_t *evlan;
	avl_index_t ai;
	uint32_t *vtp;

	if (vlan == 0xffff)
		return (0);

	mutex_enter(&bp->vnic.lock);
	/* Check if vlan already exists */
	svlan.tag = vlan;
	evlan = avl_find(&bp->vnic.vlans, &svlan, &ai);

	if (add) {
		if (evlan != NULL) {
			mutex_exit(&bp->vnic.lock);
			return (0);
		}
		evlan = kmem_zalloc(sizeof (*evlan), KM_SLEEP);
		evlan->tag = vlan;
		avl_insert(&bp->vnic.vlans, evlan, ai);
		bp->vnic.num_vlan++;
	} else {
		if (evlan == NULL) {
			mutex_exit(&bp->vnic.lock);
			return (ENOENT);
		}
		avl_remove(&bp->vnic.vlans, evlan);
		kmem_free(evlan, sizeof (*evlan));
		bp->vnic.num_vlan--;
	}

	if (bp->vnic.num_vlan > BNXT_MAX_VLAN_TAGS) {
		bp->vnic.rx_mask &=
		    ~HWRM_CFA_L2_SET_RX_MASK_INPUT_MASK_VLAN_NONVLAN;
		bp->vnic.rx_mask |=
		    HWRM_CFA_L2_SET_RX_MASK_INPUT_MASK_ANYVLAN_NONVLAN;
	} else {
		bp->vnic.rx_mask &=
		    ~HWRM_CFA_L2_SET_RX_MASK_INPUT_MASK_ANYVLAN_NONVLAN;
		bp->vnic.rx_mask |=
		    HWRM_CFA_L2_SET_RX_MASK_INPUT_MASK_VLAN_NONVLAN;
		vtp = (uint32_t *)bp->vnic.vlan_tbl.va;
		memset(vtp, 0, bp->vnic.vlan_tbl.len);
		for (evlan = avl_first(&bp->vnic.vlans); evlan != NULL;
		    evlan = AVL_NEXT(&bp->vnic.vlans, evlan)) {
			*vtp = htonl(VLAN_TPID << 16 | evlan->tag);
			vtp++;
		}
	}
	BNXT_DMA_SYNC(bp->vnic.vlan_tbl, DDI_DMA_SYNC_FORDEV);

	if (bnxt_hwrm_cfa_l2_set_rx_mask(bp) != 0) {
		mutex_exit(&bp->vnic.lock);
		return (EIO);
	}
	mutex_exit(&bp->vnic.lock);

	return (0);
}
#endif

#ifdef HAVE_MAC_VLAN
static int
bnxt_rx_vlan_add(mac_group_driver_t arg, uint16_t vlan)
{
	return (0);
}

static int
bnxt_rx_vlan_rem(mac_group_driver_t arg, uint16_t vlan)
{
	return (0);
}
#endif

static int
bnxt_rx_grp_start(mac_group_driver_t arg)
{
	bnxt_t *bp = (bnxt_t *)arg;

	/* Allocate VNIC */
	if (bnxt_hwrm_vnic_alloc(bp) != 0 ||
	    bnxt_hwrm_vnic_ctx_alloc(bp) != 0) {
		bnxt_error(bp, "!failed to allocate vnic");
		return (EIO);
	}

	return (0);
}

static void
bnxt_rx_grp_stop(mac_group_driver_t arg)
{
	bnxt_t *bp = (bnxt_t *)arg;
	bnxt_mac_t *mac;
	bnxt_vlan_t *vlan;
	void *ck;

	/* Free any stray filters */
	ck = NULL;
	while ((mac = avl_destroy_nodes(&bp->vnic.ucast, &ck)) != NULL) {
		(void) bnxt_hwrm_vnic_filter_free(bp, mac->filter);
		kmem_free(mac, sizeof (*mac));
	}
	ck = NULL;
	while ((mac = avl_destroy_nodes(&bp->vnic.mcast, &ck)) != NULL) {
		kmem_free(mac, sizeof (*mac));
		bp->vnic.num_mcast--;
	}
	ck = NULL;
	while ((vlan = avl_destroy_nodes(&bp->vnic.vlan, &ck)) != NULL) {
		kmem_free(vlan, sizeof (*vlan));
		bp->vnic.num_vlan--;
	}
	(void) bnxt_hwrm_vnic_free(bp);
	(void) bnxt_hwrm_vnic_ctx_free(bp);
}

static void
bnxt_rx_grp_info(void *arg, mac_ring_type_t rtype, const int gidx,
    mac_group_info_t *infop, mac_group_handle_t gh)
{
	bnxt_t *bp = arg;

	VERIFY3S(rtype, ==, MAC_RING_TYPE_RX);
	VERIFY3S(gidx, ==, 0);
	infop->mgi_driver = (mac_group_driver_t)bp;
	infop->mgi_count = bp->nrxq;
	infop->mgi_start = bnxt_rx_grp_start;
	infop->mgi_stop = bnxt_rx_grp_stop;
	infop->mgi_addmac = bnxt_rx_mac_add;
	infop->mgi_remmac = bnxt_rx_mac_rem;
#ifdef HAVE_MAC_VLAN
	infop->mgi_addvlan = bnxt_rx_vlan_add;
	infop->mgi_remvlan = bnxt_rx_vlan_rem;
#endif
}

static int
bnxt_ring_tx_start(mac_ring_driver_t arg, uint64_t gen)
{
	bnxt_ring_t *txr = (bnxt_ring_t *)arg;
	bnxt_t *bp = txr->bp;

	/* Allocate hardware rings */
	if (bnxt_hwrm_stat_ctx_alloc(txr) != 0 ||
	    bnxt_hwrm_ring_alloc(txr,
	    HWRM_RING_ALLOC_INPUT_RING_TYPE_L2_CMPL) != 0 ||
	    bnxt_hwrm_ring_alloc(txr,
	    HWRM_RING_ALLOC_INPUT_RING_TYPE_TX) != 0) {
		bnxt_error(bp, "!failed to allocate tx ring %d", txr->idx);
		return (EIO);
	}
	txr->ndesc_free = txr->ndesc;
	txr->pidx = 0;
	/* Reset completion ring */
	txr->cp_cidx = UINT32_MAX;
	txr->cp_v_bit = true;
	bnxt_cp_reset(txr);

	bnxt_buf_ring_alloc(txr);
	bnxt_ring_intr_enable(txr);

	return (0);
}

static void
bnxt_ring_tx_stop(mac_ring_driver_t arg)
{
	bnxt_ring_t *txr = (bnxt_ring_t *)arg;

	/* Free TX ring */
	mutex_enter(&txr->lock);
	bnxt_ring_intr_disable(txr);
	(void) bnxt_hwrm_ring_free(txr, HWRM_RING_FREE_INPUT_RING_TYPE_TX);
	(void) bnxt_hwrm_ring_free(txr, HWRM_RING_FREE_INPUT_RING_TYPE_L2_CMPL);
	(void) bnxt_hwrm_stat_ctx_free(txr);
	bnxt_buf_ring_free(txr);
	mutex_exit(&txr->lock);
}

int
bnxt_ring_stat_tx(mac_ring_driver_t rh, uint_t stat, uint64_t *val)
{
	/* TODO implement */
	return (ENOTSUP);
}

void
bnxt_tx_info(void *arg, mac_ring_type_t rtype, const int gidx,
    const int ridx, mac_ring_info_t *infop, mac_ring_handle_t rh)
{
	bnxt_t *bp = arg;
	bnxt_ring_t *txr;

	VERIFY3S(rtype, ==, MAC_RING_TYPE_TX);
	VERIFY3S(ridx, <, bp->ntxq);
	txr = &bp->txr[ridx];
	txr->rh = rh;
	infop->mri_driver = (mac_ring_driver_t)txr;
	infop->mri_start = bnxt_ring_tx_start;
	infop->mri_stop = bnxt_ring_tx_stop;
	infop->mri_tx = bnxt_ring_tx;
	infop->mri_stat = bnxt_ring_stat_tx;
}

static int
bnxt_m_start(void *arg)
{
	bnxt_t *bp = arg;

	(void) bnxt_hwrm_func_reset(bp);

	/* Set data buffers sizes based on current MTU */
	bp->rx_buf_size = P2ROUNDUP(bp->vnic.mru, PAGESIZE);
	bp->tx_buf_size = P2ROUNDUP(bp->mtu +
	    sizeof (struct ether_vlan_header) + ETHERFCSL, PAGESIZE);

	bp->started = true;

	return (0);
}

static void
bnxt_m_stop(void *arg)
{
	bnxt_t *bp = arg;

	bp->started = false;
}

static int
bnxt_m_setpromisc(void *arg, boolean_t on)
{
	bnxt_t *bp = arg;
	int ret = 0;

	mutex_enter(&bp->vnic.lock);
	if (on) {
		bp->vnic.rx_mask |=
		    HWRM_CFA_L2_SET_RX_MASK_INPUT_MASK_PROMISCUOUS;
	} else {
		bp->vnic.rx_mask &=
		    ~HWRM_CFA_L2_SET_RX_MASK_INPUT_MASK_PROMISCUOUS;
	}
	if (bnxt_hwrm_vnic_set_rx_mask(bp) != 0)
		ret = EIO;
	mutex_exit(&bp->vnic.lock);

	return (ret);
}

static int
bnxt_m_multicst(void *arg, boolean_t add, const uint8_t *mac)
{
	bnxt_t *bp = arg;
	bnxt_mac_t *emac;
	bnxt_mac_t smac;
	uint8_t *mcp;
	avl_index_t ai;

	if (!ETHER_IS_MULTICAST(mac))
		return (EINVAL);

	mutex_enter(&bp->vnic.lock);
	/* Check if address exists */
	memcpy(smac.macaddr, mac, ETHERADDRL);
	emac = avl_find(&bp->vnic.mcast, &smac, &ai);

	if (add) {
		if (emac != NULL) {
			mutex_exit(&bp->vnic.lock);
			return (0);
		}
		emac = kmem_zalloc(sizeof (*emac), KM_SLEEP);
		memcpy(emac->macaddr, mac, ETHERADDRL);
		avl_insert(&bp->vnic.mcast, emac, ai);
		bp->vnic.num_mcast++;
	} else {
		if (emac == NULL) {
			mutex_exit(&bp->vnic.lock);
			return (ENOENT);
		}
		avl_remove(&bp->vnic.mcast, emac);
		kmem_free(emac, sizeof (*emac));
		bp->vnic.num_mcast--;
	}

	mcp = (uint8_t *)bp->vnic.mcast_tbl.va;
	memset(mcp, 0, bp->vnic.mcast_tbl.len);
	if (bp->vnic.num_mcast > BNXT_MAX_MC_ADDRS) {
		bp->vnic.rx_mask &= ~HWRM_CFA_L2_SET_RX_MASK_INPUT_MASK_MCAST;
		bp->vnic.rx_mask |=
		    HWRM_CFA_L2_SET_RX_MASK_INPUT_MASK_ALL_MCAST;
	} else {
		bp->vnic.rx_mask &=
		    ~HWRM_CFA_L2_SET_RX_MASK_INPUT_MASK_ALL_MCAST;
		bp->vnic.rx_mask |= HWRM_CFA_L2_SET_RX_MASK_INPUT_MASK_MCAST;
		for (emac = avl_first(&bp->vnic.mcast); emac != NULL;
		    emac = AVL_NEXT(&bp->vnic.mcast, emac)) {
			memcpy(mcp, &emac->macaddr, ETHERADDRL);
			mcp += ETHERADDRL;
		}
	}
	BNXT_DMA_SYNC(bp->vnic.mcast_tbl, DDI_DMA_SYNC_FORDEV);

	if (bnxt_hwrm_vnic_set_rx_mask(bp) != 0) {
		mutex_exit(&bp->vnic.lock);
		return (EIO);
	}
	mutex_exit(&bp->vnic.lock);

	return (0);
}

static boolean_t
bnxt_m_getcapab(void *arg, mac_capab_t cap, void *cap_data)
{
	bnxt_t *bp = arg;
	mac_capab_rings_t *cap_rings;

	switch (cap) {
	case MAC_CAPAB_RINGS:
		cap_rings = cap_data;
		cap_rings->mr_group_type = MAC_GROUP_TYPE_STATIC;
		cap_rings->mr_gaddring = NULL;
		cap_rings->mr_gremring = NULL;

		switch (cap_rings->mr_type) {
		case MAC_RING_TYPE_RX:
			cap_rings->mr_gnum = 1;
			cap_rings->mr_rnum = bp->nrxq;
			cap_rings->mr_rget = bnxt_rx_info;
			cap_rings->mr_gget = bnxt_rx_grp_info;
			return (B_TRUE);
		case MAC_RING_TYPE_TX:
			cap_rings->mr_gnum = 0;
			cap_rings->mr_rnum = bp->ntxq;
			cap_rings->mr_rget = bnxt_tx_info;
			return (B_TRUE);
		default:
			return (B_FALSE);
		}
	default:
		return (B_FALSE);
	}
}

static int
bnxt_m_setprop(void *arg, const char *pr_name, mac_prop_id_t pr_num,
    uint_t pr_valsize, const void *pr_val)
{
	bnxt_t *bp = arg;
	uint32_t new_mtu;
	int ret = 0;

	mutex_enter(&bp->vnic.lock);
	switch (pr_num) {
	case MAC_PROP_MTU:
		memcpy(&new_mtu, pr_val, sizeof (new_mtu));
		if (new_mtu == bp->mtu)
			break;
		if (new_mtu < BNXT_MTU_MIN || new_mtu > BNXT_MTU_MAX) {
			ret = EINVAL;
			break;
		}
		if (bp->started) {
			ret = EBUSY;
			break;
		}
		ret = mac_maxsdu_update(bp->mh, new_mtu);
		if (ret == 0) {
			bp->mtu = new_mtu;
			bp->vnic.mru = new_mtu +
			    sizeof (struct ether_vlan_header) + ETHERFCSL;
		}
		break;
	default:
		ret = ENOTSUP;
		break;
	}
	mutex_exit(&bp->vnic.lock);

	return (ret);
}

static int
bnxt_m_getprop(void *arg, const char *pr_name, mac_prop_id_t pr_num,
    uint_t pr_valsize, void *pr_val)
{
	bnxt_t *bp = arg;
	int ret = 0;

	switch (pr_num) {
	case MAC_PROP_AUTONEG:
		*(uint8_t *)pr_val = bp->link.autoneg;
		break;
	case MAC_PROP_DUPLEX:
		*(link_duplex_t *)pr_val = bp->link.duplex;
		break;
#ifdef HAVE_MAC_MEDIA
	case MAC_PROP_MEDIA:
		*(mac_ether_media_t *)pr_val = bp->link.media;
		break;
#endif
	case MAC_PROP_SPEED:
		*(uint64_t *)pr_val = bp->link.speed;
		break;
	case MAC_PROP_STATUS:
		*(link_state_t *)pr_val = bp->link.state;
		break;

	case MAC_PROP_ADV_10HDX_CAP:
	case MAC_PROP_EN_10HDX_CAP:
		*(uint8_t *)pr_val =
		    _SPD(HWRM_PORT_PHY_QCFG_OUTPUT_SUPPORT_SPEEDS_10MBHD);
		break;
	case MAC_PROP_ADV_10FDX_CAP:
	case MAC_PROP_EN_10FDX_CAP:
		*(uint8_t *)pr_val =
		    _SPD(HWRM_PORT_PHY_QCFG_OUTPUT_SUPPORT_SPEEDS_10MB);
		break;
	case MAC_PROP_ADV_100HDX_CAP:
	case MAC_PROP_EN_100HDX_CAP:
		*(uint8_t *)pr_val =
		    _SPD(HWRM_PORT_PHY_QCFG_OUTPUT_SUPPORT_SPEEDS_100MBHD);
		break;
	case MAC_PROP_ADV_100FDX_CAP:
	case MAC_PROP_EN_100FDX_CAP:
		*(uint8_t *)pr_val =
		    _SPD(HWRM_PORT_PHY_QCFG_OUTPUT_SUPPORT_SPEEDS_100MB);
		break;
	case MAC_PROP_ADV_1000HDX_CAP:
	case MAC_PROP_EN_1000HDX_CAP:
		*(uint8_t *)pr_val =
		    _SPD(HWRM_PORT_PHY_QCFG_OUTPUT_SUPPORT_SPEEDS_1GBHD);
		break;
	case MAC_PROP_ADV_1000FDX_CAP:
	case MAC_PROP_EN_1000FDX_CAP:
		*(uint8_t *)pr_val =
		    _SPD(HWRM_PORT_PHY_QCFG_OUTPUT_SUPPORT_SPEEDS_1GB);
		break;
	case MAC_PROP_ADV_2500FDX_CAP:
	case MAC_PROP_EN_2500FDX_CAP:
		*(uint8_t *)pr_val =
		    _SPD(HWRM_PORT_PHY_QCFG_OUTPUT_SUPPORT_SPEEDS_2_5GB);
		break;
	case MAC_PROP_ADV_10GFDX_CAP:
	case MAC_PROP_EN_10GFDX_CAP:
		*(uint8_t *)pr_val =
		    _SPD(HWRM_PORT_PHY_QCFG_OUTPUT_SUPPORT_SPEEDS_10GB);
		break;
	case MAC_PROP_ADV_25GFDX_CAP:
	case MAC_PROP_EN_25GFDX_CAP:
		*(uint8_t *)pr_val =
		    _SPD(HWRM_PORT_PHY_QCFG_OUTPUT_SUPPORT_SPEEDS_25GB);
		break;
	case MAC_PROP_ADV_40GFDX_CAP:
	case MAC_PROP_EN_40GFDX_CAP:
		*(uint8_t *)pr_val =
		    _SPD(HWRM_PORT_PHY_QCFG_OUTPUT_SUPPORT_SPEEDS_40GB);
		break;
	case MAC_PROP_ADV_50GFDX_CAP:
	case MAC_PROP_EN_50GFDX_CAP:
		*(uint8_t *)pr_val =
		    _SPD(HWRM_PORT_PHY_QCFG_OUTPUT_SUPPORT_SPEEDS_50GB);
		break;
	case MAC_PROP_ADV_100GFDX_CAP:
	case MAC_PROP_EN_100GFDX_CAP:
		*(uint8_t *)pr_val =
		    _SPD(HWRM_PORT_PHY_QCFG_OUTPUT_SUPPORT_SPEEDS_100GB);
		break;

	default:
		ret = ENOTSUP;
		break;
	}

	return (ret);
}

static void
bnxt_m_propinfo(void *arg, const char *pr_name, mac_prop_id_t pr_num,
    mac_prop_info_handle_t prh)
{
	switch (pr_num) {
	case MAC_PROP_MTU:
		mac_prop_info_set_range_uint32(prh, BNXT_MTU_MIN, BNXT_MTU_MAX);
		break;
	default:
		mac_prop_info_set_perm(prh, MAC_PROP_PERM_READ);
		break;
	}
}

static mac_callbacks_t bnxt_m_callbacks = {
	.mc_callbacks = MC_GETCAPAB | MC_PROPERTIES,
	.mc_getstat = bnxt_m_stat,
	.mc_start = bnxt_m_start,
	.mc_stop = bnxt_m_stop,
	.mc_setpromisc = bnxt_m_setpromisc,
	.mc_multicst = bnxt_m_multicst,
	.mc_getcapab = bnxt_m_getcapab,
	.mc_setprop = bnxt_m_setprop,
	.mc_getprop = bnxt_m_getprop,
	.mc_propinfo = bnxt_m_propinfo,
};

int
bnxt_mac_register(bnxt_t *bp)
{
	mac_register_t *mac;
	int ret;

	mac = mac_alloc(MAC_VERSION);
	if (mac == NULL) {
		bnxt_error(bp, "!failed to allocate MAC handle");
		return (1);
	}

	mac->m_type_ident = MAC_PLUGIN_IDENT_ETHER;
	mac->m_driver = bp;
	mac->m_dip = bp->dip;
	mac->m_src_addr = bp->macaddr;
	mac->m_dst_addr = NULL;
	mac->m_callbacks = &bnxt_m_callbacks;
	mac->m_min_sdu = 0;
	mac->m_max_sdu = bp->mtu;
	mac->m_margin = VLAN_TAGSZ;
	mac->m_pdata = NULL;
	mac->m_pdata_size = 0;
	mac->m_priv_props = NULL;
	mac->m_v12n = MAC_VIRT_LEVEL1;

	ret = mac_register(mac, &bp->mh);
	mac_free(mac);
	if (ret != 0) {
		bnxt_error(bp, "!failed to register with MAC: %d", ret);
		return (1);
	}

	return (0);
}

void
bnxt_mac_unregister(bnxt_t *bp)
{
	if (bp->mh != NULL)
		(void) mac_unregister(bp->mh);
}
