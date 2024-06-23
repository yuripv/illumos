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
#include <sys/sensors.h>
#include <sys/stdbool.h>
#include <sys/strsubr.h>
#include <sys/strsun.h>
#include <sys/sunddi.h>
#include <sys/sysmacros.h>
#include <sys/vlan.h>

#include "hsi_struct_def.h"

/*
 * Version of the driver.
 */
#define	BNXT_DRV_MAJOR	0
#define	BNXT_DRV_MINOR	2
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
 * DMA alignment requirements for various structures.  This comes from the HSI
 * specification.
 */
#define	BNXT_DMA_ALIGN	16

/*
 * DMA SGL lengths for different types of transfers.  The various rings and
 * other data structures do not allow for more than one entry.  However, when
 * putting in data for transmitting packets, we may use more SGL entries.  This
 * restriction comes from the fact that firmware only allows for a single SGL
 * entry.
 */
#define	BNXT_DMA_SGLLEN	1

/*
 * The bnxt device driver supports a full 64-bit range for performing I/O.
 */
#define	BNXT_DMA_ADDR_LO	0x0
#define	BNXT_DMA_ADDR_HI	UINT64_MAX

/*
 * Default values for the burstsizes, count_max, and segment boundaries.  Since
 * there seems to be no restrictions from a HW side, we try to set this to a
 * reasonable value.
 */
#define	BNXT_DMA_COUNT_MAX	UINT32_MAX
#define	BNXT_DMA_BURSTSIZES	0xfff
#define	BNXT_DMA_SEGMENT	UINT32_MAX

/*
 * Minimum and maximum transfer sizes.  Since there isn't an obvious maximum
 * transfer size we set this to UINT32_MAX.
 */
#define	BNXT_DMA_MINXFER	1
#define	BNXT_DMA_MAXXFER	UINT32_MAX

/*
 * The DMA granularity for the bnxt driver is always one byte because the PCI
 * express bus is byte addressable.
 */
#define	BNXT_DMA_GRANULARITY	1

/*
 * Macros to synchronize DMA memory.  Because we never specify an offset or
 * a length, ddi_dma_sync is supposed to always return successfully.  Thus we
 * VERIFY this on debug and ignore it on non-debug.
 */
#ifdef	DEBUG
#define	BNXT_DMA_SYNC(dma, flag)	VERIFY0(ddi_dma_sync( \
					    (dma).dma_handle, 0, 0, (flag)))
#else
#define	BNXT_DMA_SYNC(dma, flag)	((void) ddi_dma_sync( \
					    (dma).dma_handle, 0, 0, (flag)))
#endif

extern ddi_dma_attr_t bnxt_dma_attr;
extern ddi_device_acc_attr_t bnxt_acc_attr;
extern ddi_device_acc_attr_t bnxt_db_acc_attr;

/* Data payload size */
#define	BNXT_MTU_MIN		ETHERMIN
#define	BNXT_MTU_MAX		9600
#define	BNXT_MTU_DEF		ETHERMTU

/* XXX Why these limits? */
#define	BNXT_MAX_MC_ADDRS	16
#define	BNXT_MAX_VLAN_TAGS	16

typedef struct bnxt_dma_buffer {
	caddr_t			va;		/* Buffer VA */
	uint64_t		pa;		/* Buffer PA */
	size_t			len;		/* Buffer logical len */
	ddi_acc_handle_t	acc_handle;	/* Access Handle */
	ddi_dma_handle_t	dma_handle;	/* DMA Handle */
	uint_t			ncookies;	/* Actual cookies */
	ddi_dma_cookie_t	cookie;
} bnxt_dma_buf_t;

typedef struct bnxt bnxt_t;
typedef struct bnxt_buf bnxt_buf_t;
typedef struct bnxt_ring bnxt_ring_t;

struct bnxt_buf {
	bnxt_ring_t	*rp;
	uint32_t	opaque;
	list_node_t	node;
	bnxt_dma_buf_t	dma;

	frtn_t		rx_frtn;

	bnxt_buf_t	*tx_next;
	mblk_t		*tx_mp;
	ddi_dma_handle_t tx_bind_hdl;
	bool		tx_bind;
	size_t		tx_len;
};

typedef struct bnxt_full_tpa_start {
	rx_tpa_start_cmpl_t lo;
	rx_tpa_start_cmpl_hi_t hi;
} bnxt_tpa_t;

typedef struct bnxt_group {
	uint16_t	stat_ctx_id;
	uint16_t	cpr_id;
	uint16_t	rxr_id;
	uint16_t	agr_id;
	uint16_t	grp_id;
} bnxt_group_t;

#define	BNXT_RX_RING_LEN	512
#define	BNXT_RX_CP_RING_LEN	BNXT_RX_RING_LEN * 8
#define	BNXT_TX_RING_LEN	512
#define	BNXT_TX_CP_RING_LEN	BNXT_TX_RING_LEN * 2

typedef enum {
	BNXT_RING_CP,
	BNXT_RING_RX,
	BNXT_RING_TX,
} bnxt_ring_type_t;

struct bnxt_ring {
	bnxt_t		*bp;
	bnxt_ring_type_t type;
	int		idx;

	bool		active;
	kmutex_t	lock;

	mac_ring_handle_t rh;

	uint16_t	id;		/* logical ID */
	uint16_t	phys_id;	/* hardware ID */
	uintptr_t	db;		/* doorbell offset */

	bnxt_dma_buf_t	descs;		/* DMA buffer for descriptors */
	uint32_t	pidx;		/* producer index */
	uint32_t	ndesc;		/* number of descriptors */
	uint16_t	ndesc_free;	/* number of free descriptors */

	bnxt_buf_t	**bufs;		/* all buffers assigned to ring */
	uint16_t	nbuf;
	list_t		bufs_free;	/* idle ring buffers */

	uint16_t	cp_phys_id;
	bnxt_dma_buf_t	cp_descs;
	size_t		cp_ndesc;
	uint32_t	cp_cidx;	/* consumer index */
	bool		cp_v_bit;
	uint32_t	cp_stat_ctx_id;
	bnxt_dma_buf_t	cp_stats;	/* stats DMA buffer */
	ddi_intr_handle_t cp_ihdl;

	uint64_t	rx_gen;

	bool		tx_blocked;
	kcondvar_t	tx_recycle;
};

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
	mac_ether_media_t media;
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
	size_t			rx_buf_size;
	bnxt_group_t		*grp;
	/* TX */
	int			ntxq;
	bnxt_ring_t		*txr;
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

extern void bnxt_cp_mark_invalid(bnxt_ring_t *);
extern void bnxt_db_cp(bnxt_ring_t *, bool);
extern void bnxt_db_rx(bnxt_ring_t *);
extern void bnxt_db_tx(bnxt_ring_t *);

/* DMA */
extern int bnxt_dma_alloc(bnxt_t *, bnxt_dma_buf_t *, ddi_device_acc_attr_t *,
    size_t);
extern void bnxt_dma_free(bnxt_dma_buf_t *);

/* GLD */
extern int bnxt_mac_register(bnxt_t *);
extern void bnxt_mac_unregister(bnxt_t *);
extern void bnxt_link_update(bnxt_t *);
extern link_duplex_t bnxt_duplex_to_gld(uint8_t);
extern mac_ether_media_t bnxt_media_to_gld(uint8_t);
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

extern int bnxt_hwrm_ring_alloc(bnxt_ring_t *, bnxt_ring_type_t);
extern int bnxt_hwrm_ring_free(bnxt_ring_t *, bnxt_ring_type_t);
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
extern void bnxt_rx_refill(bnxt_ring_t *);
extern uint_t bnxt_rx_intr(caddr_t, caddr_t);
extern mblk_t *bnxt_ring_rx(void *, int);
extern void bnxt_tx_recycle(void *);
extern mblk_t *bnxt_ring_tx(void *, mblk_t *);

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
