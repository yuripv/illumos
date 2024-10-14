/*
 * Copyright 2024 Tintri by DDN, Inc. All rights reserved.
 */

#ifndef _PEXNTB_IMPL_H
#define	_PEXNTB_IMPL_H

#include <sys/types.h>
#include <sys/cmn_err.h>
#include <sys/cyclic.h>
#include <sys/dditypes.h>
#include <sys/list.h>
#include <sys/pci.h>
#include <sys/stdbool.h>

#include <sys/ntb_svc.h>

#define	pexntb_peer_is_faulted(pntb)	((pntb->pntb_flags & PNTB_FAULTED) != 0)
#define	pexntb_peer_is_up(pntb)		((pntb->pntb_flags & PNTB_PEER_UP) != 0)

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

typedef struct pexntb pexntb_t;

typedef void (*msg_func_t)(pexntb_t *, uint_t, uint32_t);

/* A BAR (or window) */
typedef struct {
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
	uint64_t	pb_seg_size;	/* size of segment */
	int		pb_seg_count;	/* number of segments in window */
} pex_bar_t;

typedef enum {
	PASS_THRU_HDL,
	SEGMENT_HDL,
	WINDOW_HDL,
} pntb_hdl_t;

typedef struct {
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
	pex_bar_t		*pdh_window;	/* this handle's window */
	list_node_t		pdh_link;
} pex_dma_handle_t;

#define	PNTB_SHUTDOWN		0x0001
#define	PNTB_PEER_UP		0x0002
#define	PNTB_FAULTED		0x0004
#define	PNTB_FAULT_HANDLED	0x0008
#define	PNTB_MR4SAFE_LOCAL	0x0010
#define	PNTB_MR4SAFE_PEER	0x0020

/* Private dma_attr_flag.  DMA memory is tied to the peer partition's LUT. */
#define	PNTB_DMA_PEER		0x01000000

typedef struct pntb_acc_handle {
	ddi_acc_handle_t	iah_acc_handle;
} pntb_acc_handle_t;

typedef struct {
	db_func_t	pdc_fn;		/* function to call */
	void		*pdc_arg;	/* argument to pdc_fn */
	uint_t		pdc_db;		/* the doorbell number */
	dev_info_t	*pdc_dip;	/* dip the doorbell belongs to */
	bool		pdc_local;	/* only posted locally, not by peer */
	list_node_t	pdc_link;
} pex_db_cb_t;

#define	PNTB_GAS_CFG_SIZE		0x00400
#define	PNTB_GAS_CFG_SPACE0		0x04000
#define	PNTB_GAS_CFG_SPACE(p)		\
	(PNTB_GAS_CFG_SPACE0 + (p) * PNTB_GAS_CFG_SIZE)

/* Registers and offsets within partition's configuration region */
#define	PNTB_CFG_VEP_PFF_INST_ID	0x14
#define	PNTB_CFG_EVENT_VECTOR		0x0100

#define	PNTB_SYSINFO_OFFSET		0x2000
#define	PNTB_SYSINFO_FWVER_OFFSET	0x8
#define	PNTB_SYSINFO_SCRATCHPAD_OFFSET	0x3FC0
#define	PNTB_SYSINFO_DRIVERVER_OFFSET	0x0
#define	PNTB_SYSINFO_HEARTBEAT_OFFSET	0x8

#define	PNTB_GAS_NTB_OFFSET		0x10000
/* Registers and offsets within the NTB region of GAS */
#define	PNTB_NTB_PARTITION_NUMBER	0x00
#define	PNTB_NTB_PARTITION_ID		0x01
#define	PNTB_NTB_ENDPOINT_MAP		0x04
#define	PNTB_NTB_REQUESTER_ID		0x0c

#define	PNTB_NTB_PART_INFO_SIZE		0x10
#define	PNTB_NTB_PART_INFO0		0x20
#define	PNTB_NTB_PART_INFO(p)		\
	(PNTB_NTB_PART_INFO0 + (p) * PNTB_NTB_PART_INFO_SIZE)
#define	PNTB_NTB_PART_INFO_X_ENABLED	0x0
#define	PNTB_NTB_PART_INFO_LOW		0x4
#define	PNTB_NTB_PART_INFO_HIGH		0x8

#define	PNTB_NTB_CNTL_SIZE		0x2000
#define	PNTB_NTB_CNTL_SPACE0		0x4000
#define	PNTB_NTB_CNTL_SPACE(p)		\
	(PNTB_NTB_CNTL_SPACE0 + (p) * PNTB_NTB_CNTL_SIZE)
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
#define	PNTB_NTB_CNTL_BAR_DIR_WIN_SIZE(n) \
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
#define	PNTB_NTB_CNTL_BAR_EXT_WIN_SIZE(n) \
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
#define	PNTB_NTB_DB_MSG_SPACE(p)	\
	(PNTB_NTB_DB_MSG_SPACE0 + (p) * PNTB_NTB_DB_MSG_SIZE)

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
#define	PNTB_NTB_CONFIG_SPACE(n)	\
	(PNTB_NTB_CONFIG_SPACE0 + (n) * PNTB_NTB_CONFIG_SPACE_SIZE)

/* Offsets in PCI config space */
#define	PNTB_NTB_CONFIG_LNK_STS		0x76
#define	PNTB_NTB_CONFIG_LNK_SPEED(s)	((s) & 0xf)
#define	PNTB_NTB_CONFIG_LNK_WIDTH(s)	(((s) >> 4) & 0x3f)

#if 0
extern void pex_log(pexntb_t *, const char *, ...) __PRINTFLIKE(2);
extern void pex_error(pexntb_t *, const char *, ...) __PRINTFLIKE(2);

extern void pexntb_lput8(pexntb_t *, uint_t, uint8_t);
extern void pexntb_lput16(pexntb_t *, uint_t, uint16_t);
extern void pexntb_lput32(pexntb_t *, uint_t, uint32_t);
extern void pexntb_lput64(pexntb_t *, uint_t, uint64_t);
extern void pexntb_pput64(pexntb_t *, uint_t, uint64_t);
extern uint8_t pexntb_lget8(pexntb_t *, uint_t);
extern uint16_t pexntb_lget16(pexntb_t *, uint_t);
extern uint32_t pexntb_lget32(pexntb_t *, uint_t);
extern uint32_t	pexntb_pget32(pexntb_t *, uint_t);
extern uint64_t pexntb_lget64(pexntb_t *, uint_t);
extern uint64_t	pexntb_pget64(pexntb_t *, uint_t);
#endif

extern bool pexntb_claim_window(pexntb_t *, pex_bar_t *, dev_info_t *);
extern void pexntb_release_window(pexntb_t *, pex_bar_t *);

extern int pexntb_ping_start(pexntb_t *);
extern void pexntb_ping_stop(pexntb_t *);

extern void pexntb_inter_ntb_cleanup(pexntb_t *);
extern int pexntb_intr_init(pexntb_t *);
extern void pexntb_intr_fini(pexntb_t *);
extern int pexntb_intr_enable(pexntb_t *);
extern void pexntb_intr_disable(pexntb_t *);

extern void pexntb_get_peer_link_info(pexntb_t *);

extern int pexntb_doorbells_init(pexntb_t *);
extern void pexntb_doorbells_fini(pexntb_t *);
extern int pexntb_set_peer_doorbell(pexntb_t *, uint_t);
extern int pexntb_set_db_callback(pexntb_t *, dev_info_t *, uint_t, db_func_t,
    void *);
extern int pexntb_clear_db_callback(pexntb_t *, dev_info_t *, uint_t);

extern int pexntb_set_msg_callback(pexntb_t *, msg_func_t, uint_t);
extern int pexntb_clear_msg_callback(pexntb_t *, uint_t);
extern int pexntb_send_message(pexntb_t *, uint_t, uint32_t);

extern int pexntb_check_acc_handle(pexntb_t *, ddi_acc_handle_t);
extern int pexntb_check_gas_acc_handle(pexntb_t *);
extern int pexntb_check_bar_acc_handle(pexntb_t *, int);
extern int pexntb_check_win_acc_handle(pexntb_t *, pex_bar_t *);
extern int pexntb_check_dma_handle(pexntb_t *, ddi_dma_handle_t);

extern int pexntb_set_direct_window(pexntb_t *, pex_bar_t *, int, uint64_t);
extern int pexntb_clear_direct_window(pexntb_t *, pex_bar_t *, int);

extern int pexntb_set_lut_window(pexntb_t *, int, int, uint64_t);
extern int pexntb_clear_lut_window(pexntb_t *, int, int);
extern int pexntb_set_lut_peer(pexntb_t *, pex_dma_handle_t *);
extern int pexntb_clear_lut_peer(pexntb_t *, pex_dma_handle_t *);
extern int pexntb_set_lut_local(pexntb_t *, pex_dma_handle_t *);
extern int pexntb_clear_lut_local(pexntb_t *, pex_dma_handle_t *);
extern int pexntb_setup_lut(pexntb_t *, pex_bar_t *, int);
extern int pexntb_clear_lut(pexntb_t *, pex_bar_t *, int);

extern int pexntb_set_rids(pexntb_t *, int, uint16_t *, uint_t);
extern int pexntb_clear_rids(pexntb_t *, int);
extern void pexntb_rid_disable(pexntb_t *pntb);
extern int pexntb_quiesce_translation(pexntb_t *);

extern void pexntb_get_sdk_fw_version(pexntb_t *, uint32_t *, uint32_t *);

#endif /* _PEXNTB_IMPL_H */
