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
 * Copyright (c) 2017, Joyent, Inc.
 * Copyright 2024 Tintri by DDN, Inc. All rights reserved.
 */

#ifndef _BNXT_H
#define	_BNXT_H

#include <sys/types.h>
#include <sys/avl.h>
#include <sys/conf.h>
#include <sys/ddi.h>
#include <sys/devops.h>
#include <sys/disp.h>
#include <sys/errno.h>
#include <sys/mac_provider.h>
#include <sys/mac_ether.h>
#include <sys/modctl.h>
#include <sys/param.h>
#include <sys/pattr.h>
#include <sys/pci.h>
#include <sys/random.h>
#ifdef HAVE_KSENSOR
#include <sys/sensors.h>
#endif
#include <sys/stdbool.h>
#include <sys/strsubr.h>
#include <sys/strsun.h>
#include <sys/sunddi.h>
#include <sys/sysmacros.h>
#include <sys/vlan.h>

#include "hsi_struct_def.h"

#define	BNXT_MOD_NAME	"bnxt"

#define	BNXT_DRV_MAJOR	1
#define	BNXT_DRV_MINOR	0
#define	BNXT_DRV_UPD	0

/*
 * These values indicate the regs array entry for the BAR (PCI base address
 * registers) to be used for the different sets of registers.  BAR 0
 * contains the main device registers while BAR 1 contains the doorbell
 * registers.  The first REGS property always refers to the start of config
 * space and then the BARs follow in subsequent values.
 */
#define	BNXT_BAR_DEV	1
#define	BNXT_BAR_DB	2

/*
 * Macros to synchronize DMA memory.  Because we never specify an offset or
 * a length, ddi_dma_sync is supposed to always return successfully.  Thus we
 * VERIFY this on debug and ignore it on non-debug.
 */
#ifdef	DEBUG
#define	BNXT_DMA_SYNC(dma, flag)	\
	VERIFY0(ddi_dma_sync((dma).dma_hdl, 0, 0, (flag)))
#else
#define	BNXT_DMA_SYNC(dma, flag)	\
	((void) ddi_dma_sync((dma).dma_hdl, 0, 0, (flag)))
#endif

extern ddi_dma_attr_t bnxt_dma_attr;
extern ddi_device_acc_attr_t bnxt_acc_attr;
extern ddi_device_acc_attr_t bnxt_db_acc_attr;

/* Data payload size */
#define	BNXT_MTU_MIN		ETHERMTU
#define	BNXT_MTU_MAX		9600
#define	BNXT_MTU_DEF		ETHERMTU

#define	BNXT_MAX_MC_ADDRS	16
#define	BNXT_MAX_VLAN_TAGS	16

/* CP ring values recomended by Broadcom documentation */
#define	BNXT_RX_RING_LEN	256
#define	BNXT_RX_CP_RING_LEN	BNXT_RX_RING_LEN * 4
#define	BNXT_TX_RING_LEN	256
#define	BNXT_TX_CP_RING_LEN	BNXT_TX_RING_LEN * 2

/* These are modeled after the igc driver */
#define	BNXT_TX_RECYCLE_THRESHOLD	32
#define	BNXT_TX_NOTIFY_THRESHOLD	4
#define	BNXT_TX_GAP			2

typedef struct bnxt_dma_buffer {
	caddr_t			va;
	uint64_t		pa;
	size_t			len;
	ddi_acc_handle_t	acc_hdl;
	ddi_dma_handle_t	dma_hdl;
	ddi_dma_cookie_t	ck;
	uint_t			nck;
} bnxt_dma_buf_t;

typedef struct bnxt bnxt_t;
typedef struct bnxt_buf bnxt_buf_t;
typedef struct bnxt_ring bnxt_ring_t;

struct bnxt_buf {
	bnxt_ring_t	*rp;
	uint32_t	opaque;
	list_node_t	node;
	bnxt_dma_buf_t	dma;
	mblk_t		*mp;

	union {
		struct {
			bool		loaned;
			frtn_t		frtn;
		} rx;
		struct {
			bnxt_buf_t	*next;
			size_t		len;
		} tx;
	};
};

typedef enum {
	BNXT_RING_RX,
	BNXT_RING_TX,
} bnxt_ring_type_t;

typedef struct bnxt_rx_stats {
	kstat_named_t	addr;
	kstat_named_t	rbytes;
	kstat_named_t	ipackets;
	kstat_named_t	nobufs;
} bnxt_rx_stats_t;

typedef struct bnxt_tx_stats {
	kstat_named_t	addr;
	kstat_named_t	obytes;
	kstat_named_t	opackets;
	kstat_named_t	nobufs;
	kstat_named_t	nodescs;
} bnxt_tx_stats_t;

struct bnxt_ring {
	bnxt_t		*bp;
	kmutex_t	lock;
	kstat_t		*kstat;
	mac_ring_handle_t rh;

	bnxt_ring_type_t type;
	int		idx;

	uint16_t	id;		/* logical ID */
	uint16_t	phys_id;	/* hardware ID */
	uintptr_t	db;		/* doorbell offset */

	bnxt_dma_buf_t	descs;		/* DMA buffer for descriptors */
	uint32_t	pidx;		/* producer index */
	uint16_t	ndesc;		/* number of descriptors */
	uint16_t	ndesc_free;	/* number of free descriptors */

	bnxt_buf_t	**bufs;		/* all buffers assigned to ring */
	uint16_t	nbuf;		/* number of buffers */
	list_t		bufs_free;	/* idle ring buffers */

	uint16_t	cp_phys_id;
	bnxt_dma_buf_t	cp_descs;	/* DMA buffer for CP descriptors */
	size_t		cp_ndesc;	/* number of CP descriptors */
	uint32_t	cp_cidx;	/* consumer index */
	bool		cp_v_bit;	/* consumer valid bit */
	uint32_t	cp_stat_ctx_id;	/* stats hardware ID */
	bnxt_dma_buf_t	cp_stats;	/* stats DMA buffer */
	ddi_intr_handle_t cp_ihdl;	/* CP ring interrupt handle */
	bool		cp_intr;	/* CP ring interrupt state */
	bool		cp_busy;	/* CP ring recycle */

	union {
		struct {
			uint64_t	gen;
			bnxt_rx_stats_t	stats;
		} rx;
		struct {
			bool		blocked;
			bnxt_tx_stats_t	stats;
		} tx;
	};
};

typedef struct bnxt_group {
	uint16_t	stat_ctx_id;
	uint16_t	cpr_id;
	uint16_t	rxr_id;
	uint16_t	agr_id;
	uint16_t	grp_id;
} bnxt_group_t;

typedef struct bnxt_mac {
	avl_node_t	avl;
	uint8_t		macaddr[ETHERADDRL];
	uint64_t	filter;
} bnxt_mac_t;

typedef struct bnxt_vlan {
	avl_node_t	avl;
	uint32_t	tag;
	uint64_t	filter;
} bnxt_vlan_t;

typedef struct bnxt_vnic {
	uint16_t	id;
	kmutex_t	lock;

	uint16_t	mru;
	uint16_t	def_grp;
	uint16_t	cos_rule;
	uint16_t	lb_rule;
	uint32_t	rx_mask;

	uint16_t	rss_id;
	uint32_t	rss_hash_type;
	bnxt_dma_buf_t	rss_hash_key_tbl;
	bnxt_dma_buf_t	rss_grp_tbl;

	/* Unicast */
	avl_tree_t	ucast;
	/* Multicast */
	avl_tree_t	mcast;
	uint32_t	num_mcast;
	bnxt_dma_buf_t	mcast_tbl;
	/* VLANs */
	avl_tree_t	vlan;
	uint32_t	num_vlan;
	bnxt_dma_buf_t	vlan_tbl;
} bnxt_vnic_t;

typedef struct bnxt_link {
	uint8_t		autoneg;
	link_duplex_t	duplex;
	link_state_t	state;
#ifdef HAVE_MAC_MEDIA
	mac_ether_media_t media;
#endif
	uint64_t	speed;
	uint16_t	support_speeds;
} bnxt_link_t;

struct bnxt {
	dev_info_t		*dip;
	bool			started;

	/* PCI and register access */
	ddi_acc_handle_t	pci_hdl;
	ddi_acc_handle_t	dev_hdl;
	ddi_acc_handle_t	db_hdl;
	caddr_t			dev_base;
	caddr_t			db_base;

	/* GLDv3 data */
	mac_handle_t		mh;
	uint8_t			macaddr[ETHERADDRL];
	uint16_t		mtu;
	bnxt_link_t		link;

	/* Interrupt data */
	int			nintr_req;
	int			nintr;
	uint_t			intr_pri;
	int			intr_caps;
	ddi_intr_handle_t	*ihdl;

	/* HWRM resources */
	kmutex_t		hwrm_lock;
	bnxt_dma_buf_t		hwrm_reply;
	uint16_t		hwrm_seqid;
	uint16_t		hwrm_timeout;
	uint16_t		hwrm_max_req;

	/* Device, Firmware, and NVM information */
	hwrm_ver_get_output_t	ver;
	hwrm_nvm_get_dev_info_output_t nvm;
	uint16_t		fid;
	uint16_t		port_id;
	uint32_t		qcap_flags;
	uint16_t		max_rsscos_ctx;
	uint16_t		max_cp_rings;
	uint16_t		max_tx_rings;
	uint16_t		max_rx_rings;
	uint16_t		max_l2_ctxs;
	uint16_t		max_vnics;
	uint32_t		max_stat_ctx;
	uint32_t		max_rx_em_flows;
	uint32_t		max_rx_wm_flows;
	uint32_t		max_mcast_filters;
	uint32_t		max_flow_id;
	uint32_t		max_hw_ring_grps;

	/* Current allocations of resources and configuration */
	uint16_t		alloc_cp_rings;
	uint16_t		alloc_tx_rings;
	uint16_t		alloc_rx_rings;
	uint16_t		alloc_vnics;
	uint32_t		alloc_mcast_filters;
	uint32_t		alloc_hw_ring_grps;
	hwrm_queue_qportcfg_output_t qportcfg;

	/* VNIC */
	bnxt_vnic_t		vnic;
	/* RX */
	int			nrxq;
	bnxt_ring_t		*rxr;
	bnxt_group_t		*grp;
	size_t			rx_buf_size;
	/* TX */
	int			ntxq;
	bnxt_ring_t		*txr;
	uint32_t		tx_recycle_thresh;
	uint32_t		tx_notify_thresh;
	uint32_t		tx_gap;
	size_t			tx_buf_size;

	/* Temperature sensors */
	id_t			id_temp;
	uint8_t			temp;
	bool			temp_valid;
	id_t			id_temp_om;
	uint8_t			temp_om;
	bool			temp_om_valid;
	id_t			id_temp_phy;
	uint8_t			temp_phy;
	bool			temp_phy_valid;
};

extern void bnxt_log(bnxt_t *, const char *, ...) __PRINTFLIKE(2);
extern void bnxt_error(bnxt_t *, const char *, ...) __PRINTFLIKE(2);

extern int bnxt_buf_compare(const void *, const void *);
extern void bnxt_buf_ring_alloc(bnxt_ring_t *);
extern void bnxt_buf_ring_free(bnxt_ring_t *);

/* DMA */
extern int bnxt_dma_alloc(bnxt_t *, bnxt_dma_buf_t *, ddi_device_acc_attr_t *,
    size_t);
extern void bnxt_dma_free(bnxt_dma_buf_t *);

/* GLD */
extern int bnxt_mac_register(bnxt_t *);
extern void bnxt_mac_unregister(bnxt_t *);
extern void bnxt_link_update(bnxt_t *);
extern link_duplex_t bnxt_duplex_to_gld(uint8_t);
#ifdef HAVE_MAC_MEDIA
extern mac_ether_media_t bnxt_media_to_gld(uint8_t);
#endif
extern link_state_t bnxt_state_to_gld(uint8_t);

/* HWRM */
extern int bnxt_hwrm_init(bnxt_t *);
extern void bnxt_hwrm_fini(bnxt_t *);
extern void bnxt_hwrm_resource_free(bnxt_t *);

extern int bnxt_hwrm_func_qcaps(bnxt_t *);
extern int bnxt_hwrm_func_qcfg(bnxt_t *);
extern int bnxt_hwrm_func_reset(bnxt_t *);

extern int bnxt_hwrm_host_register(bnxt_t *);
extern int bnxt_hwrm_host_unregister(bnxt_t *);

extern int bnxt_hwrm_version_get(bnxt_t *);
extern int bnxt_hwrm_nvm_info_get(bnxt_t *);
extern int bnxt_hwrm_port_phy_qcfg(bnxt_t *);
extern int bnxt_hwrm_queue_qportcfg(bnxt_t *);

extern int bnxt_hwrm_ring_alloc(bnxt_ring_t *, uint8_t);
extern int bnxt_hwrm_ring_free(bnxt_ring_t *, uint8_t);
extern void bnxt_hwrm_ring_reset(bnxt_ring_t *);
extern int bnxt_hwrm_ring_grp_alloc(bnxt_t *, bnxt_group_t *);
extern int bnxt_hwrm_ring_grp_free(bnxt_t *, bnxt_group_t *);

extern int bnxt_hwrm_stat_ctx_alloc(bnxt_ring_t *);
extern int bnxt_hwrm_stat_ctx_free(bnxt_ring_t *);

extern int bnxt_hwrm_vnic_ctx_alloc(bnxt_t *);
extern int bnxt_hwrm_vnic_ctx_free(bnxt_t *);
extern int bnxt_hwrm_vnic_alloc(bnxt_t *);
extern int bnxt_hwrm_vnic_free(bnxt_t *);
extern int bnxt_hwrm_vnic_cfg(bnxt_t *);
extern int bnxt_hwrm_vnic_filter_alloc(bnxt_t *, const uint8_t *, uint64_t *);
extern int bnxt_hwrm_vnic_filter_free(bnxt_t *, uint64_t);
extern int bnxt_hwrm_vnic_rss_cfg(bnxt_t *);
extern int bnxt_hwrm_vnic_set_rx_mask(bnxt_t *);

extern int bnxt_hwrm_events_cfg_cr(bnxt_ring_t *);
extern void bnxt_hwrm_temp_monitor_query(bnxt_t *);

/* Ring */
extern void bnxt_cp_reset(bnxt_ring_t *);
extern uint_t bnxt_rx_intr(caddr_t, caddr_t);
extern uint_t bnxt_tx_intr(caddr_t, caddr_t);
extern void bnxt_ring_intr_disable(bnxt_ring_t *);
extern void bnxt_ring_intr_enable(bnxt_ring_t *);
extern mblk_t *bnxt_ring_rx(void *, int);
extern mblk_t *bnxt_ring_tx(void *, mblk_t *);

#ifndef ETHER_IS_MULTICAST
#define	ETHER_IS_MULTICAST(addr)	(((addr)[0] & 0x01) != 0)
#endif

#define CMPL_VALID(cmpl, v_bit)						\
	((!!(((cmpl_base_t *)(cmpl))->info3_v & LE_32(CMPL_BASE_V)))	\
	    == !!(v_bit))

#define RING_NEXT_CIDX_V(rp) do {					\
	if (++(rp)->cp_cidx == (rp)->cp_ndesc)				\
	    ((rp)->cp_cidx = 0, (rp)->cp_v_bit = !(rp)->cp_v_bit);	\
} while (0)

#define	RING_NEXT_PIDX(rp, pidx)					\
	((pidx) + 1 == (rp)->ndesc ? 0 : (pidx) + 1)

#endif /* _BNXT_H */
