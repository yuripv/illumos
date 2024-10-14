/*
 * Copyright 2023 Tintri by DDN, Inc. All rights reserved.
 */

#ifndef _PSXNTB_IMPL_H
#define	_PSXNTB_IMPL_H

#include <sys/types.h>
#include <sys/cmn_err.h>
#include <sys/dditypes.h>
#include <sys/cyclic.h>
#include <sys/pci.h>
#include <sys/list.h>
#include <sys/ntb_svc.h>

#define	psx_warn(pntb, ...)	dev_err((pntb)->pntb_dip, CE_WARN, __VA_ARGS__)
#define	psx_note(pntb, ...)	dev_err((pntb)->pntb_dip, CE_NOTE, __VA_ARGS__)
#define	psx_cont(pntb, ...)	dev_err((pntb)->pntb_dip, CE_CONT, __VA_ARGS__)

#define	psxntb_peer_is_faulted(pntb)	((pntb->pntb_flags & PNTB_FAULTED) != 0)
#define	psxntb_peer_is_up(pntb)		((pntb->pntb_flags & PNTB_PEER_UP) != 0)

#ifdef DEBUG

extern int	psxntb_debug;
#define	ALWAYS		0x01
#define	EARLY		0x02
#define	XLINK		0x04

#define	psx_debug(pntb, flags, ...)				\
	do {							\
		if ((((flags) | ALWAYS) & psxntb_debug) != 0)	\
			psx_warn(pntb, __VA_ARGS__);		\
	_NOTE(CONSTCOND)					\
	} while (0)
#else
#define	psx_debug(pntb, flags, ...)
#endif

#define	PNTB_PING_INTERVAL	100000000	/* ns (1/10 sec) */
#define	PNTB_PING_TIMEOUT	2000000000	/* ns 2 sec */
#define	PNTB_PING_TIMEOUT_CNT	(PNTB_PING_TIMEOUT / PNTB_PING_INTERVAL)

#define	PNTB_NTB_LUT_ENTRIES	512
#define	PNTB_SEGMENTS		PNTB_NTB_LUT_ENTRIES

#define	PNTB_MESSAGES		4
#define	PNTB_TOTAL_DOORBELLS	(sizeof (uint64_t) * NBBY)
#define	PNTB_RUPT_LIMIT		8

#define	PNTB_PING_MESSAGE	0
enum {
	PNTB_MSGS_UP = 1,
	PNTB_MSGS_UP_ACK = 2
};

#define	PNTB_MIN_SIZE		12

/*
 * Underlying MicroChip SDK version
 */
#define	PSX_FW_BETA	0x1060041	/* 01.06.0.041 */
#define	PSX_FW_MR0P0	0x1070056	/* 01.07.0.056 */
#define	PSX_FW_MR1P0	0x1080058	/* 01.08.0.058 */
#define	PSX_FW_MR2P0	0x1090063	/* 01.09.0.063 */
#define	PSX_FW_MR3P0	0x10A0073	/* 01.0a.0.073 */
#define	PSX_FW_MR3P1	0x10A0074	/* 01.0a.0.074 */
#define	PSX_FW_MR4P0	0x10B0088	/* 01.0b.0.088 */
#define	PSX_FW_MR4P1	0x10B0089	/* 01.0b.0.089 */
#define	PSX_FW_MR4P2	0x10B008A	/* 01.0b.0.08a */
#define	PSX_FW_MR4P3	0x10B008B	/* 01.0b.0.08b */
#define	PSX_FW_MR4P4	0x10B008C	/* 01.0b.0.08c */

typedef struct psxntb psxntb_t;

typedef void (*msg_func_t)(psxntb_t *, uint_t, uint32_t);

/*
 * A BAR (or window).
 */
typedef struct psx_bar {
	dev_info_t	*pb_owner;	/* window owned by this dip */
	int		pb_ref;		/* reference counter */
	ddi_acc_handle_t pb_hdl;	/* access handle */
	caddr_t		pb_vaddr;	/* vaddr from ddi_regs_map_setup */
	uint64_t	pb_paddr;	/* phys addr assigned by BIOS */
	uint64_t	pb_peer_paddr;
	uint64_t	pb_size;	/* window size */
	int		pb_type;	/* 32 or 64 bit */
	int		pb_bar;
	int		pb_win_num;
	int		pb_rnum;

	uint_t		pb_lut_base;	/* window's 1st entry in LUT */
	uint_t		pb_lut_child;	/* 1st entry in LUT for child drivers */
	uint64_t	pb_seg_size;	/* size of segment ... */
	int		pb_seg_count;	/* ... and no. of segments in window */
} psx_bar_t;

typedef enum {
	PASS_THRU_HDL, SEGMENT_HDL, WINDOW_HDL,
} pntb_hdl_t;

typedef struct psx_dma_handle {
	pntb_hdl_t		pdh_type;
	ddi_dma_handle_t	pdh_handle;	/* real handle */
	int			pdh_seg;	/* segment number assigned */
	int			pdh_seg_cnt;	/* and number of segments */
	size_t			pdh_seg_size;	/* size of a segment */
	caddr_t			pdh_dma_addr;	/* client's vaddr */
	size_t			pdh_dma_len;	/* size of client's DMA */
	ddi_dma_cookie_t	*pdh_cookies;
	uint_t			pdh_cookie_cnt;
	ddi_dma_cookie_t	*pdh_lut_cookies;
	uint_t			pdh_lut_cookie_cnt;
	psx_bar_t		*pdh_window;	/* this handle's window */
	list_node_t		pdh_link;
} psx_dma_handle_t;

struct psxntb {
	uint_t		pntb_flags;
	int		pntb_inst;
	dev_info_t	*pntb_dip;
	ddi_acc_handle_t pntb_cfg_hdl;

	int		pntb_partition;
	int		pntb_peer_partition;
	int		pntb_partition_cnt;
	uint_t		pntb_ntb_off;		/* offset to ntb */
	uint_t		pntb_ctl_off;		/* offset to part's ctl regs */
	uint_t		pntb_pcfg_off;		/* offset to partition's cfg */
	uint_t		pntb_db_msg_off;	/* offset to part's db & msg */
	uint_t		pntb_peer_pcfg_off;	/* offset to peer's cfg */
	uint_t		pntb_peer_ctl_off;	/* offset to peer's ctl regs */
	uint_t		pntb_peer_db_msg_off;	/* offset to peer's db & msg */
	uint_t		pntb_peer_cfg_spc_off;	/* offset to peer's cfg space */

	uint16_t	pntb_vid;		/* PCI vendor id */
	uint16_t	pntb_did;		/* PCI device id */

	uint8_t		pntb_bus;		/* PCI bus */
	uint8_t		pntb_dev;		/* PCI dev */
	uint8_t		pntb_fn;		/* PCI fn */

	uint16_t	pntb_speed;		/* GEN1, 2, 3 */
	uint16_t	pntb_width;		/* lanes */

	ddi_intr_handle_t *pntb_rupts;
	uint_t		pntb_rupt_pri;
	int		pntb_rupt_size;
	int		pntb_rupt_count;
	int		pntb_rupt_db_count;
	uint8_t		pntb_rupt_event;
	uint8_t		pntb_rupt_message;
	uint8_t		*pntb_rupt_doorbell;
	uint64_t	*pntb_rupt_masks;

	int		pntb_db_count;		/* doorbells available */
	int		pntb_db_high;		/* highest db number */
	uint64_t	pntb_db_mask;		/* active doorbell mask */
	uint64_t	pntb_db_omask;		/* out doorbell mask */
	int		pntb_db_shift;		/* shift for our doorbells */
	int		pntb_db_peer_shift;	/* shift for peer's doorbells */
	int		pntb_db_up;		/* Peer is up doorbell */
	int		pntb_db_down;		/* and peer is down */

	uint32_t	pntb_msg_map;		/* msg -> partition map */
	msg_func_t	pntb_msg_callback[PNTB_MESSAGES];

	cyclic_id_t	pntb_pinger;
	uint32_t	pntb_ping_sent;
	uint32_t	pntb_ping_recv;
	uint32_t	pntb_ping_recv_latest;
	uint_t		pntb_ping_misses;

	kmutex_t	pntb_lock;
	ksema_t		pntb_sema;		/* serialise config ops. */

	uint32_t	pntb_alloc_map[PNTB_SEGMENTS / 32];	/* 256 bits */
	int		pntb_wcnt;		/* number of windows */
	int		pntb_win_base;		/* 1st win num for child drv */
	psx_bar_t	*pntb_seg_window;	/* the segmented win */
	psx_bar_t	*pntb_window[PCI_BASE_NUM];	/* map window -> bar */
							/* win0 == bar0 */
	psx_bar_t	pntb_bars[PCI_BASE_NUM];	/* all BARs */

	kt_did_t	pntb_db_thr;		/* for local doorbells */
	kcondvar_t	pntb_db_cv;
	krwlock_t	pntb_db_lock;
	uint64_t	pntb_db_set;		/* local doorbells set */
	list_t		*pntb_dbs;

	int			pntb_fm_cap;	/* fma capabilities */
	ddi_iblock_cookie_t	pntb_fm_ibc;
	ddi_device_acc_attr_t	pntb_acc_attr;
	ddi_dma_attr_t		pntb_dma_attr;

	ddi_dma_handle_t	pntb_pvt_handle;
	ddi_acc_handle_t	pntb_pvt_acc_handle;
	caddr_t			pntb_pvt_vaddr;		/* vaddr of pvt DMA */
	uintptr_t		pntb_pvt_paddr;		/* paddr of pvt DMA */
	size_t			pntb_pvt_size;
	offset_t		pntb_pvt_offset;
	caddr_t			pntb_pvt_xaddr;		/* xfer addr */

	psx_dma_handle_t	pntb_psx_handle;	/* fake handle */

	ntb_kstat_t		*pntb_kdata;		/* ntb kstats */
};

#define	PNTB_SHUTDOWN		0x0001
#define	PNTB_PEER_UP		0x0002
#define	PNTB_FAULTED		0x0004
#define	PNTB_FAULT_HANDLED	0x0008
#define	PNTB_MR4SAFE_LOCAL	0x0010
#define	PNTB_MR4SAFE_PEER	0x0020

/*
 * Private dma_attr_flag. DMA memory is tied to the peer partition's LUT.
 */
#define	PNTB_DMA_PEER		0x01000000

typedef struct pntb_acc_handle {
	ddi_acc_handle_t	iah_acc_handle;
} pntb_acc_handle_t;

typedef struct psx_db_callback {
	db_func_t	pdc_fn;		/* function to call */
	void		*pdc_arg;	/* argument to pdc_fn */
	uint_t		pdc_db;		/* the doorbell number */
	dev_info_t	*pdc_dip;	/* dip the doorbell belongs to */
	boolean_t	pdc_local;	/* only posted locally, not by peer */
	list_node_t	pdc_link;
} psx_db_callback_t;

#define	PNTB_GAS_CFG_SIZE		0x00400
#define	PNTB_GAS_CFG_SPACE0		0x04000
#define	PNTB_GAS_CFG_SPACE(p)		(PNTB_GAS_CFG_SPACE0 + (p) * \
					    PNTB_GAS_CFG_SIZE)
/*
 * Registers and offsets within partition's configuration region.
 */
#define	PNTB_CFG_VEP_PFF_INST_ID	0x14
#define	PNTB_CFG_EVENT_VECTOR		0x0100

#define	PNTB_SYSINFO_OFFSET		0x2000
#define	PNTB_SYSINFO_FWVER_OFFSET	0x8
#define	PNTB_SYSINFO_SCRATCHPAD_OFFSET	0x3FC0
#define	PNTB_SYSINFO_DRIVERVER_OFFSET	0x0
#define	PNTB_SYSINFO_HEARTBEAT_OFFSET	0x8

#define	PNTB_GAS_NTB_OFFSET		0x10000
/*
 * Registers and offsets within the NTB region of GAS.
 */
#define	PNTB_NTB_PARTITION_NUMBER	0x00
#define	PNTB_NTB_PARTITION_ID		0x01
#define	PNTB_NTB_ENDPOINT_MAP		0x04
#define	PNTB_NTB_REQUESTER_ID		0x0c

#define	PNTB_NTB_PART_INFO_SIZE		0x10
#define	PNTB_NTB_PART_INFO0		0x20
#define	PNTB_NTB_PART_INFO(p)		(PNTB_NTB_PART_INFO0 + (p) * \
					    PNTB_NTB_PART_INFO_SIZE)
#define	PNTB_NTB_PART_INFO_X_ENABLED	0x0
#define	PNTB_NTB_PART_INFO_LOW		0x4
#define	PNTB_NTB_PART_INFO_HIGH		0x8

#define	PNTB_NTB_CNTL_SIZE		0x2000
#define	PNTB_NTB_CNTL_SPACE0		0x4000
#define	PNTB_NTB_CNTL_SPACE(p)		(PNTB_NTB_CNTL_SPACE0 + (p) * \
					    PNTB_NTB_CNTL_SIZE)
#define	PNTB_NTB_CNTL_STATUS		0x00
#define	PNTB_NTB_CNTl_STATUS_NORMAL	0x01
#define	PNTB_NTB_CNTl_STATUS_LOCKED	0x02
#define	PNTB_NTB_CNTl_STATUS_LIP	0x03	/* Lock in progress */
#define	PNTB_NTB_CNTl_STATUS_CIP	0x04	/* Config. in progress */
#define	PNTB_NTB_CNTl_STATUS_RIP	0x05	/* Reset. in progress */

#define	PNTB_NTB_CNTL_OPC		0x04
#define	PNTB_NTB_CNTL_OPC_LOCK		0x01
#define	PNTB_NTB_CNTL_OPC_CONFIG	0x02
#define	PNTB_NTB_CNTL_OPC_RESET		0x03

#define	PNTB_NTB_CNTL_BAR_LIMIT		0x0c
#define	PNTB_NTB_CNTL_LUT_LIMIT		0x14
#define	PNTB_NTB_CNTL_RID_LIMIT		0x1c

/* Definitions for BAR translation setup */
#define	PNTB_NTB_CNTL_BAR_SETUP0	0x40
#define	PNTB_NTB_CNTL_BAR_SETUP(i)	(PNTB_NTB_CNTL_BAR_SETUP0 + (i) * 0x10)
/* Word 0 bits */
#define	PNTB_NTB_CNTL_BAR_VALID		(1 << 0)
#define	PNTB_NTB_CNTL_BAR_TYPE(t)	(((t) & 3) << 1)
#define	PNTB_NTB_CNTL_BAR_DIRECT	(1 << 4)
#define	PNTB_NTB_CNTL_BAR_LUT		(1 << 5)
#define	PNTB_NTB_CNTL_BAR_LUT_POS(n)	(((n) & 0x3f) << 8)
#define	PNTB_NTB_CNTL_BAR_LUT_SUBWIN(n)	(((n) & 0x1ff) << 14)
#define	PNTB_NTB_CNTL_BAR_LUT_SUBWIN_BASE(n)	\
					(((n) & 0x1ff) << 23)
/* Word 1 bits */
#define	PNTB_NTB_CNTL_BAR_DIR_POS(n)	((n) & 0x3f)
#define	PNTB_NTB_CNTL_BAR_DIR_WIN_SIZE(n)	\
					((uint32_t)(uintptr_t)(n) & 0xfffff000u)
/* Word 2 bits */
#define	PNTB_NTB_CNTL_BAR_DIR_TPART(n)	((n) & 0x3f)
#define	PNTB_NTB_CNTL_BAR_DIR_ADDRL(n)	((uint32_t)(uintptr_t)(n) & 0xfffff000u)
/* Word 3 bits */
#define	PNTB_NTB_CNTL_BAR_DIR_ADDRH(n)	((uint32_t)((uint64_t)(n) >> 32))

/* NT Partition Control Register offset and ID Protection Disable */
#define	PNTB_NTB_PART_CNTRL		0x08
#define	PNTB_ID_PROTECTION_DISABLE	0x01

/* Bar translation extension register */
#define	PNTB_NTB_CNTL_BAR_EXT0		0xa0
#define	PNTB_NTB_CNTL_BAR_EXT(i)	(PNTB_NTB_CNTL_BAR_EXT0 + (i) * 0x10)
#define	PNTB_NTB_CNTL_BAR_EXT_WIN_SIZE(n)	\
					((uint32_t)((uint64_t)(n) >> 32))

#define	PNTB_NTB_CNTL_RID0		0x0400
#define	PNTB_NTB_CNTL_RID(i)		(PNTB_NTB_CNTL_RID0 + (i) * 4)
#define	PNTB_NTB_CNTL_RID_ENABLE	0x01
#define	PNTB_NTB_CNTL_RID_PROXY_ID(n)	(((n) >> 1) & 0xff)
#define	PNTB_NTB_CNTL_RID_ID(n)		((uint32_t)(n) << 16)

#define	PNTB_NTB_CNTL_LUT0		0x1000
#define	PNTB_NTB_CNTL_LUT(i)		(PNTB_NTB_CNTL_LUT0 + (i) * 8)
/* Word 0 bits */
#define	PNTB_NTB_CNTL_LUT_ENABLE	(1 << 0)
#define	PNTB_NTB_CNTL_LUT_DPART(n)	(((n) & 0x3f) << 1)
#define	PNTB_NTB_CNTL_LUT_ADDRL(n)	((uint32_t)(uintptr_t)(n) & 0xfffff000u)
/* Word 1 bits */
#define	PNTB_NTB_CNTL_LUT_ADDRH(n)	((uint32_t)((uint64_t)(n) >> 32))

#define	PNTB_NTB_DB_MSG_SIZE		0x4000
#define	PNTB_NTB_DB_MSG_SPACE0		0x64000
#define	PNTB_NTB_DB_MSG_SPACE(p)	(PNTB_NTB_DB_MSG_SPACE0 + (p) * \
					    PNTB_NTB_DB_MSG_SIZE)

/* Relative to PNTB_NTB_DB_MSG_SPACE(p) */
#define	PNTB_NTB_ODB			0x1000
#define	PNTB_NTB_ODB_MASK		0x1008
#define	PNTB_NTB_IDB			0x1010
#define	PNTB_NTB_IDB_MASK		0x1018
#define	PNTB_NTB_IDB_MAP		0x1020
#define	PNTB_NTB_IDB_MAP_SIZE		PNTB_TOTAL_DOORBELLS
#define	PNTB_NTB_MSG_MAP		0x1060
#define	PNTB_NTB_OMSG0			0x1068
#define	PNTB_NTB_OMSG(i)		(PNTB_NTB_OMSG0 + (i) * 8)
#define	PNTB_NTB_OMSG_FULL		(1ull << 32)
#define	PNTB_NTB_IMSG0			0x1088
#define	PNTB_NTB_IMSG(i)		(PNTB_NTB_IMSG0 + (i) * 8)
#define	PNTB_NTB_IMSG_FULL		(1ull << 32)
#define	PNTB_NTB_IMSG_IMASK		(1ull << 40)

#define	PNTB_NTB_CONFIG_SPACE_SIZE	0x1000
#define	PNTB_NTB_CONFIG_SPACE0		0x134000
#define	PNTB_NTB_CONFIG_SPACE(n)	(PNTB_NTB_CONFIG_SPACE0 + (n) * \
					    PNTB_NTB_CONFIG_SPACE_SIZE)

/*
 * Offsets in PCI config space.
 */
#define	PNTB_NTB_CONFIG_LNK_STS		0x76
#define	PNTB_NTB_CONFIG_LNK_SPEED(s)	((s) & 0xf)
#define	PNTB_NTB_CONFIG_LNK_WIDTH(s)	(((s) >> 4) & 0x3f)

extern void	psxntb_put8(psxntb_t *, uint_t, uint8_t);
extern void	psxntb_put16(psxntb_t *, uint_t, uint16_t);
extern void	psxntb_put32(psxntb_t *, uint_t, uint32_t);
extern void	psxntb_put64(psxntb_t *, uint_t, uint64_t);
extern uint8_t	psxntb_get8(psxntb_t *, uint_t);
extern uint16_t	psxntb_get16(psxntb_t *, uint_t);
extern uint32_t	psxntb_get32(psxntb_t *, uint_t);
extern uint64_t	psxntb_get64(psxntb_t *, uint_t);

extern void	psxntb_putpeer64(psxntb_t *, uint_t, uint64_t);
extern uint32_t	psxntb_getpeer32(psxntb_t *, uint_t);
extern uint64_t	psxntb_getpeer64(psxntb_t *, uint_t);

extern boolean_t psxntb_claim_window(psxntb_t *, psx_bar_t *, dev_info_t *);
extern void	psxntb_release_window(psxntb_t *, psx_bar_t *);

extern int	psxntb_start_pinging(psxntb_t *);
extern void	psxntb_stop_pinging(psxntb_t *);

extern void	psxntb_inter_ntb_cleanup(psxntb_t *);
extern int	psxntb_intr_init(psxntb_t *);
extern void	psxntb_intr_fini(psxntb_t *);
extern int	psxntb_intr_enable(psxntb_t *);
extern void	psxntb_intr_disable(psxntb_t *);

extern void	psxntb_get_peer_link_info(psxntb_t *);

extern int	psxntb_doorbells_init(psxntb_t *);
extern void	psxntb_doorbells_fini(psxntb_t *);
extern int	psxntb_set_peer_doorbell(psxntb_t *, uint_t);
extern int	psxntb_set_db_callback(psxntb_t *, dev_info_t *, uint_t,
		    db_func_t, void *);
extern int	psxntb_clear_db_callback(psxntb_t *, dev_info_t *, uint_t);

extern int	psxntb_set_msg_callback(psxntb_t *, msg_func_t, uint_t);
extern int	psxntb_clear_msg_callback(psxntb_t *, uint_t);
extern int	psxntb_send_message(psxntb_t *, uint_t, uint32_t);

extern int	psxntb_check_acc_handle(psxntb_t *, ddi_acc_handle_t);
extern int	psxntb_check_gas_acc_handle(psxntb_t *);
extern int	psxntb_check_bar_acc_handle(psxntb_t *, int);
extern int	psxntb_check_win_acc_handle(psxntb_t *, psx_bar_t *);
extern int	psxntb_check_dma_handle(psxntb_t *, ddi_dma_handle_t);

extern int	psxntb_set_direct_window(psxntb_t *, psx_bar_t *, int,
		    uint64_t);
extern int	psxntb_clear_direct_window(psxntb_t *, psx_bar_t *, int);

extern int	psxntb_set_lut_window(psxntb_t *, int, int, uint64_t);
extern int	psxntb_clear_lut_window(psxntb_t *, int, int);
extern int	psxntb_set_lut_peer(psxntb_t *, psx_dma_handle_t *);
extern int	psxntb_clear_lut_peer(psxntb_t *, psx_dma_handle_t *);
extern int	psxntb_set_lut_local(psxntb_t *, psx_dma_handle_t *);
extern int	psxntb_clear_lut_local(psxntb_t *, psx_dma_handle_t *);
extern int	psxntb_setup_lut(psxntb_t *, psx_bar_t *, int);
extern int	psxntb_clear_lut(psxntb_t *, psx_bar_t *, int);

extern int	psxntb_set_rids(psxntb_t *, int, uint16_t *, uint_t);
extern int	psxntb_clear_rids(psxntb_t *, int);
extern void	psxntb_rid_disable(psxntb_t *pntb);
extern int	psxntb_quiesce_translation(psxntb_t *);

extern void	psxntb_get_sdk_fw_version(psxntb_t *, uint32_t *, uint32_t *);

#endif /* _PSXNTB_IMPL_H */
