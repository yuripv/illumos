/*
 * Copyright 2024 Tintri by DDN, Inc. All rights reserved.
 */

#include <sys/types.h>
#include <sys/devops.h>
#include <sys/pcie.h>
#include <sys/pcie_impl.h>
#include <sys/pci_cap.h>
#include <sys/pci_cfgacc.h>
#include <sys/sunddi.h>
#include <sys/sunndi.h>
#include <sys/sysmacros.h>

#include "pexntb_impl.h"

#define	KBYTE	(1024)
#define	MBYTE	(KBYTE * 1024)
#define	GBYTE	(MBYTE * 1024)

static void *statep;

#define	MCPU_HDR_VERSION	1
#define	MCPU_MAX_PKT_SIZE_DW	16
#define	MCPU_MSG_ADDR_TYPE_NT	1
#define	MCPU_MSG_FLAGS_LASTPKT	0x1
#define	MCPU_TYPE_API		0

#define	PEX_RX_ADDR_LOW		0x18
#define	PEX_RX_ADDR_HI		0x1c
#define	PEX_RX_BUF_SIZE		0x20
#define	PEX_RX_FIFO_PTR		0x24
#define	PEX_TX_ADDR_LOW		0x28
#define	PEX_TX_ADDR_HI		0x2c
#define	PEX_TX_BUF_SIZE		0x30
#define	PEX_TX_FIFO_PTR		0x34

#define	HEAD_IDX(idx)		((idx >> 16) & 0xffff)
#define	TAIL_IDX(idx)		((idx) & 0xffff)
#define	TO_QPTRS(h, t)		(((h) << 16) | ((t) & 0xffff))

#define	HAM_GET_FW_VERSION	0x1
#define	HAM_GET_LINK_STATUS	0x4
#define	HAM_GET_ERR_CNT		0x6

#define	BYTES_TO_DW(x)		((x) >> 2)
#define	DW_TO_BYTES(x)		((x) << 2)

#define	PEX_BAR_WIN		2

#define	PEX_DOMAIN_BUS_REG	0x118

#define	PEX_NT_VSEC_ID		0x4
#define	PEX_SYNTH_NT_VSEC_ID	0x500

#define	PEX_OFFSET_GID_LOCAL	0x8
#define	PEX_OFFSET_GID_PEER	0x10

typedef struct {
	uint8_t		addr_dn;
	uint8_t		domain;
	uint8_t		reserved;
	uint8_t		port;
} ham_cmd_t;

typedef struct {
	union {
		/* Switch ID */
		uint16_t id;                       /* Switch ID */
		struct {
			/* Chip number within domain (0-based) */
			uint8_t		number;
			/* Fabric domain number */
			uint8_t		domain;
		} dn;
	};
} pmg_switch_id_t;

typedef struct {
	/* PCIe defined port type (PCIE_PORT_TYPE enum) */
	uint8_t		port_type;
	/* Internal port number */
	uint8_t		port_number;
	/* Negotiated link width */
	uint8_t		link_width;
	/* Max link width device is capable of */
	uint8_t		max_link_width;
	/* Negotiated link speed */
	uint8_t		link_speed;
	/* Max link speed device is capable of */
	uint8_t		max_link_speed;
	/* Max read request size allowed */
	uint16_t	max_read_req_size;
	/* Max payload size setting */
	uint16_t	max_payload_size;
	/* Max payload size supported by device */
	uint16_t	max_payload_suported;
} pcie_port_prop_t;

typedef struct {
	/* Switch Id the Port belongs to */
	pmg_switch_id_t	switch_id;
	/* Reserved */
	uint8_t		reserved;
	/* Clock Mode for Port */
	uint8_t		clock_mode;
	/* Port Number whose attributes are requested */
	uint8_t		port_num;
	/* Station port is in */
	uint8_t		station;
	/* Port number in station */
	uint8_t		station_port;
	/* Port type */
	uint8_t		type;
	/* Unique Global ID of port */
	uint32_t	gid;
	/* PCIe properties of port */
	pcie_port_prop_t pcie;
	/* Type specific attributes */
	union {
		/* For Invalid Port Types or Disabled Ports */
		uint64_t	type_specific;
		/* DS Port Attributes */
		struct {
			/* Global DS Secondary Bus */
			uint8_t		glob_bus_sec;
			/* Global DS Subordinate Bus */
			uint8_t		glob_bus_sub;
			/* Number of Downstream devices present */
			uint16_t	ds_dev_count;
			/* Host port number assigned to, if any */
			uint8_t		host_port_num;
			/* Reserved */
			uint8_t		reserved[3];
		} downstream;
		/* Host Port Attributes */
		struct {
			/* Assigned Global Bus */
			uint8_t		global_pci_bus;
			/* Number of Virtual Slots Assigned */
			uint8_t		virt_slot_cnt;
			/* Host Flags: TWC, MPT, GDMA, etc */
			uint16_t	host_flags;
			/* Reserved */
			uint8_t		reserved[4];
		} host;
		/* Fabric Port Attributes */
		struct {
			/* Is Valid Connection */
			uint8_t		valid;
			/* Peer switch connected port number */
			uint8_t		peer_port;
			/* Switch ID of connected peer switch */
			pmg_switch_id_t	peer_id;
			/* Reserved */
			uint8_t		reserved[4];
		} fabric;
	} attr;
} sm_port_attr_t;

typedef struct {
	/* Switch Id the Port belongs to */
	pmg_switch_id_t	switch_id;
	/* Station number of port device is connected to */
	uint8_t		station;
	/* Station port number of port device is connected to */
	uint8_t		station_port;
	/* Port Number in switch whose ecs are requested */
	uint8_t		port_num;
	/* Reserved */
	uint8_t		reserved[3];
	/* Unique Global ID of port */
	uint32_t	gid;
	/* Port Receiver Errors */
	uint32_t	receiver_error;
	/* Bad TLP Errors */
	uint32_t	bad_tlp;
	/* Bad TDLLP Errors */
	uint32_t	bad_dllp;
	/* Recovery Diag Errors */
	uint32_t	recovery_diag;
} sm_port_err_cntrs_t;

typedef struct {
	/* Length of this packet in dwords */
	uint32_t	msg_len_dw:5;
	uint32_t	hdr_ver:3;
	uint32_t	src_domain:8;
	/* Source ID of the packet.  Value defined by src_addr_type. */
	uint32_t	src_id:8;
	/* Source address type of the packet */
	uint32_t	src_addr_type:3;
	/* Source address type specific information */
	uint32_t	src_type_specific:5;

	uint32_t	msg_type:4;
	uint32_t	msg_flags:4;
	uint32_t	dest_domain:8;
	/* Destination ID of the packet.  Value defined by dest_addr_type. */
	uint32_t	dst_id:8;
	/* Destination address type of the packet */
	uint32_t	dst_addr_type:3;
	/* Destination address type specific information */
	uint32_t	dst_type_specific:5;

	uint32_t	pkt_num:8;
	uint32_t	req_tag:8;
	uint32_t	msg_type_specific:16;
} mcpu_hdr_t;

typedef struct {
	mcpu_hdr_t	hdr;
	union {
		struct {
			/*
			 * Total size (in dword) of the request (can span
			 * multiple tlps).
			 */
			uint32_t	size_dw;
			uint32_t	payload[12];
		} first;
		struct {
			uint32_t	payload[13];
		} next;
	};
} mcpu_tlp_cmd_t;
CTASSERT(BYTES_TO_DW(sizeof (mcpu_tlp_cmd_t)) == MCPU_MAX_PKT_SIZE_DW);

typedef struct {
	mcpu_hdr_t	hdr;
	union {
		struct {
			uint32_t	size_dw;
			uint32_t	status;
			uint32_t	payload[11];
		} first;
		struct {
			uint32_t	payload[13];
		} next;
	};
} mcpu_tlp_rsp_t;
CTASSERT(BYTES_TO_DW(sizeof (mcpu_tlp_rsp_t)) == MCPU_MAX_PKT_SIZE_DW);

#if 0
static uint32_t gid_map[][2] = {
	{ 0x00040, 0x10040 },
	{ 0x10040, 0x00040 },
	{ 0x00020, 0x10020 },
	{ 0x10020, 0x00020 },
	{ 0xfffff, 0xfffff }
};
#endif

/* Ingress data */
typedef union {
	struct {
		uint32_t	target_nt2_bus:8;
		uint32_t	target_nt2_domain:8;
		uint32_t	target_egress_alut:9;
		uint32_t	reserved:6;
		uint32_t	entry_valid:1;
	} __packed;
	uint32_t	data;
} ingress_alut_t;

/* Egress data */
typedef union {
	struct {
		uint32_t	egress_seg_size:6;
		uint32_t	write_enable:1;
		uint32_t	read_enable:1;
		uint32_t	source_id_check:1;
		uint32_t	clear_no_snoop:1;
		uint32_t	reserved:2;
		uint32_t	address_remap:20;
	} __packed;
	uint32_t	data;
} egress_alut_t;

struct pexntb {
	dev_info_t	*dip;
	int		inst;
	uint_t		flags;

	kmutex_t	lock;
	ksema_t		sema;		/* serialise config ops. */

	uint16_t	vid;		/* PCI vendor id */
	uint16_t	did;		/* PCI device id */

	uint8_t		bus;		/* PCI bus */
	uint8_t		dev;		/* PCI dev */
	uint8_t		fn;		/* PCI fn */

	ddi_acc_handle_t	cfg_hdl;
	ddi_acc_handle_t        win_hdl;
	caddr_t			win_base;
	off_t			win_sz;

	uint32_t		hw_ver;
	uint32_t		eep_ver;

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

	uint32_t	pntb_alloc_map[PNTB_SEGMENTS / 32];	/* 256 bits */
	int		pntb_wcnt;		/* number of windows */
	int		pntb_win_base;		/* 1st win num for child drv */
	pex_bar_t	*pntb_seg_window;	/* the segmented win */
	pex_bar_t	*pntb_window[PCI_BASE_NUM];	/* map window -> bar */
							/* win0 == bar0 */
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
	uint64_t		pntb_pvt_paddr;		/* paddr of pvt DMA */
	size_t			pntb_pvt_size;
	offset_t		pntb_pvt_offset;
	caddr_t			pntb_pvt_xaddr;		/* xfer addr */

	pex_dma_handle_t	pntb_pex_handle;	/* fake handle */

	ntb_kstat_t		*pntb_kdata;		/* ntb kstats */

	/* XXX */
	uint32_t		scratchpad[8];
	/*
	 * Must be right after scratchpad as remote will update remote
	 * scratchpad + index while updating its local scratchpad so that
	 * rem_spad_copy is in sync.
	 */
	uint32_t		rem_spad_copy[8];
	uint32_t		gid;

	uint16_t		vs_off;

	uint8_t			ham_rx_q_sz;
	uint8_t			ham_tx_q_sz;
	//duck_pcie_domain_bus_t  domain_bus;
	//duck_pcie_domain_bus_t  mcpu_domain_bus;
	volatile uint32_t	*peer_scratchpad;
	/* Pad of 4k to make the following two queues 4k-aligned */
	uint8_t			pad[0x1000];
	uint8_t			ham_rx_buffer[0x1000];
	uint8_t			ham_tx_buffer[0x1000];
	mcpu_tlp_rsp_t		*ham_rx;
	mcpu_tlp_cmd_t		*ham_tx;
	//aer_cor_errors_t        fabric_aer;
	//aer_cor_errors_t        host_aer; /* host port AER */

	uint8_t			port;
};

ddi_device_acc_attr_t pex_dev_acc_attr = {
	.devacc_attr_version = DDI_DEVICE_ATTR_V1,
	.devacc_attr_endian_flags = DDI_STRUCTURE_LE_ACC,
	.devacc_attr_dataorder = DDI_STRICTORDER_ACC,
	.devacc_attr_access = DDI_DEFAULT_ACC
};

ddi_device_acc_attr_t pex_win_acc_attr = {
	.devacc_attr_version = DDI_DEVICE_ATTR_V1,
	.devacc_attr_endian_flags = DDI_STRUCTURE_LE_ACC,
	.devacc_attr_dataorder = DDI_MERGING_OK_ACC,
	.devacc_attr_access = DDI_DEFAULT_ACC
};

void
pex_log(pexntb_t *pntb, const char *fmt, ...)
{
	va_list ap;

	va_start(ap, fmt);
	vdev_err(pntb->dip, CE_NOTE, fmt, ap);
	va_end(ap);
}

void
pex_error(pexntb_t *pntb, const char *fmt, ...)
{
	va_list ap;

	va_start(ap, fmt);
	vdev_err(pntb->dip, CE_WARN, fmt, ap);
	va_end(ap);
}

int pexntb_debug;
#define	ALWAYS	1
#define	EARLY	2
#define	XLINK	4

void
pex_debug(pexntb_t *pntb, int flags, const char *fmt, ...)
{
	va_list ap;

	if (((flags | ALWAYS) & pexntb_debug) == 0)
		return;
	va_start(ap, fmt);
	vdev_err(pntb->dip, CE_WARN, fmt, ap);
	va_end(ap);
}

#if 0
/*
 * A set of general service routines for accessing registers of
 * different sizes in the main MMIO (BAR0) register space.
 */
uint8_t
pexntb_lget8(pexntb_t *pntb, uint_t off)
{
	uint8_t val;

	//ASSERT(off < pntb->dev_sz);
	if (off >= pntb->dev_sz)
		pex_error(pntb, "bad");
	val = ddi_get8(pntb->dev_hdl,
	    (uint8_t *)(uintptr_t)(pntb->dev_base + off));
	pex_debug(pntb, EARLY, "get8 0x%x = 0x%x02", off, val);
	return (val);
}

uint16_t
pexntb_lget16(pexntb_t *pntb, uint_t off)
{
	uint16_t val;

	ASSERT(off < pntb->dev_sz);
	val = ddi_get16(pntb->dev_hdl,
	    (uint16_t *)(uintptr_t)(pntb->dev_base + off));
	pex_debug(pntb, EARLY, "get16 0x%x = 0x%x04", off, val);
	return (val);
}

uint32_t
pexntb_lget32(pexntb_t *pntb, uint_t off)
{
	uint32_t val;

	ASSERT(off < pntb->dev_sz);
	val = ddi_get32(pntb->dev_hdl,
	    (uint32_t *)(uintptr_t)(pntb->dev_base + off));
	pex_debug(pntb, EARLY, "get32 0x%x = 0x%x08", off, val);
	return (val);
}

#if 0
uint32_t
pexntb_pget32(pexntb_t *pntb, uint_t off)
{
	pex_bar_t *bar = &pntb->pntb_bars[4];
	uint32_t val;

	ASSERT(off < bar->pb_size);

	val = ddi_get32(bar->pb_hdl,
	    (uint32_t *)(uintptr_t)(bar->pb_vaddr + off));
	pex_debug(pntb, EARLY, "peer_get32 0x%x = 0x%x08", off, val);
	return (val);
}
#endif

uint64_t
pexntb_lget64(pexntb_t *pntb, uint_t off)
{
	uint64_t val;

	ASSERT(off < pntb->dev_sz);
	val = ddi_get64(pntb->dev_hdl,
	    (uint64_t *)(uintptr_t)(pntb->dev_base + off));
	pex_debug(pntb, EARLY, "get64 0x%x = 0x%016" PRIx64, off, val);
	return (val);
}

#if 0
uint64_t
pexntb_pget64(pexntb_t *pntb, uint_t off)
{
	pex_bar_t *bar = &pntb->pntb_bars[4];
	uint64_t val;

	ASSERT(off < bar->pb_size);
	val = ddi_get64(bar->pb_hdl,
	    (uint64_t *)(uintptr_t)(bar->pb_vaddr + off));
	pex_debug(pntb, EARLY, "peer_get64 0x%x = 0x%016" PRIx64, off, val);
	return (val);
}
#endif

void
pexntb_lput8(pexntb_t *pntb, uint_t off, uint8_t val)
{
	ASSERT(off < pntb->dev_sz);
	pex_debug(pntb, EARLY, "put8 0x%x = 0x%02x", off, val);
	ddi_put8(pntb->dev_hdl,
	    (uint8_t *)(uintptr_t)(pntb->dev_base + off), val);
}

void
pexntb_lput16(pexntb_t *pntb, uint_t off, uint16_t val)
{
	ASSERT(off < pntb->dev_sz);
	pex_debug(pntb, EARLY, "put16 0x%x = 0x%04x", off, val);
	ddi_put16(pntb->dev_hdl,
	    (uint16_t *)(uintptr_t)(pntb->dev_base + off), val);
}

void
pexntb_lput32(pexntb_t *pntb, uint_t off, uint32_t val)
{
	ASSERT(off < pntb->dev_sz);
	pex_debug(pntb, EARLY, "put32 0x%x = 0x%08x", off, val);
	ddi_put32(pntb->dev_hdl,
	    (uint32_t *)(uintptr_t)(pntb->dev_base + off), val);
}

void
pexntb_lput64(pexntb_t *pntb, uint_t off, uint64_t val)
{
	ASSERT(off < pntb->dev_sz);
	pex_debug(pntb, EARLY, "put64 0x%x = 0x%016" PRIx64, off, val);
	ddi_put64(pntb->dev_hdl,
	    (uint64_t *)(uintptr_t)(pntb->dev_base + off), val);
}

void
pexntb_pput64(pexntb_t *pntb, uint_t off, uint64_t val)
{
	pex_bar_t *bar = &pntb->pntb_bars[4];

	ASSERT(off < bar->pb_size);
	pex_debug(pntb, EARLY, "peer_put64 0x%x = 0x%016" PRIx64, off, val);
	ddi_put64(bar->pb_hdl,
	    (uint64_t *)(uintptr_t)(bar->pb_vaddr + off), val);
}
#endif

#if 0
void
pexntb_ham_post(pexntb_t *pntb, uint32_t msg_type, void *msg, uint32_t msg_len)
{
	mcpu_tlp_cmd_t *cmd;
	uint32_t qptrs;
	uint16_t tx_head;

	qptrs = pci_config_get32(pntb->cfg_hdl,
	    pntb->vs_off + PEX_TX_FIFO_PTR);
	pex_log(pntb, "qptrs %x", qptrs);

	return;
	tx_head = HEAD_IDX(qptrs);

	cmd = &pntb->ham_tx[tx_head];
	memset(&cmd->hdr, 0, sizeof (cmd->hdr));
	cmd->hdr.msg_len_dw = BYTES_TO_DW(sizeof (mcpu_hdr_t) +
	    sizeof (cmd->first.size_dw) + msg_len);
	cmd->hdr.hdr_ver = MCPU_HDR_VERSION;
	//cmd->hdr.src_domain = ntb->atlas.domain_bus.domain;
	cmd->hdr.src_addr_type = MCPU_MSG_ADDR_TYPE_NT;
	cmd->hdr.msg_type = MCPU_TYPE_API;
	//cmd->hdr.dest_domain = ntb->atlas.domain_bus.domain;
	cmd->hdr.req_tag = msg_type; /* Tag 0 is invalid */
	cmd->hdr.msg_type_specific = msg_type;
	cmd->hdr.msg_flags |= MCPU_MSG_FLAGS_LASTPKT;

	cmd->first.size_dw = BYTES_TO_DW(msg_len + sizeof(cmd->first.size_dw));
	memcpy(cmd->first.payload, msg, msg_len);

	tx_head = (tx_head + 1) & (pntb->ham_tx_q_sz -1);
	qptrs = TO_QPTRS(tx_head, 0);
	pci_config_put32(pntb->cfg_hdl, pntb->vs_off + PEX_TX_FIFO_PTR,
	    qptrs);
}

int
pexntb_ham_recv(pexntb_t *pntb, void *buf, uint32_t buf_len)
{
	uint32_t qptrs;
	uint16_t rx_head, rx_tail;
	uint8_t *data;
	uint32_t i = 0;
	uint32_t pkt_len_bytes;
	uint32_t rsp_len_bytes = 0xff;
	uint32_t offset = 0;
	uint32_t copy_bytes;
	mcpu_tlp_rsp_t *rsp;

	/*
	 * At first we assume a non-zero value of rsp_len_bytes to enter this
	 * loop.  Then we set the value correctly based on the first packet
	 * of response.  Note that there can be multiple packets.
	 */
	while (rsp_len_bytes != 0) {
		do {
			qptrs = pexntb_lget32(pntb,
			    pntb->vs_off + PEX_RX_FIFO_PTR);
			rx_head = HEAD_IDX(qptrs);
			rx_tail = TAIL_IDX(qptrs);
			delay(5);
			if (i++ >= 20) {
				/* Timeout */
				return (1);
			}
		} while (rx_head == rx_tail);
		rsp = &pntb->ham_rx[rx_tail];
		if (rsp->hdr.pkt_num == 0) {
			data = (void *)rsp->first.payload;
			if (rsp->first.status != 0)
				return (1);
			/*
			 * Useful response length across ALL packets - subtract
			 * uint32_t for size field and status field.
			 */
			rsp_len_bytes = DW_TO_BYTES(rsp->first.size_dw) -
			    sizeof(rsp->first.size_dw) -
			    sizeof(rsp->first.status);
			/*
			 * Useful response data in this packet does not include
			 * the header info, the size info or the status.  The
			 * rest of the data makes up the response struct.
			 */
			pkt_len_bytes = DW_TO_BYTES(rsp->hdr.msg_len_dw) -
			    sizeof(rsp->hdr) - sizeof(rsp->first.size_dw) -
			    sizeof(rsp->first.status);
		} else {
			data = (void *)rsp->next.payload;
			/*
			 * Useful response data in this packet does not include
			 * the header info.  Since this is not the first packet,
			 * it does not have additional info like the total
			 * response length or the status.
			 */
			pkt_len_bytes = DW_TO_BYTES(rsp->hdr.msg_len_dw) -
			    sizeof(rsp->hdr);
		}

		rsp_len_bytes -= pkt_len_bytes;
		copy_bytes = MIN(pkt_len_bytes, buf_len);
		memcpy(buf + offset, data, copy_bytes);
		offset += copy_bytes;
		buf_len -= copy_bytes;

		rx_tail = (rx_tail + 1) & (pntb->ham_rx_q_sz - 1);
		qptrs = TO_QPTRS(0, rx_tail);
		pexntb_lput32(pntb, pntb->vs_off + PEX_RX_FIFO_PTR, qptrs);
	}

	return (0);
}
#endif

/*
 * The next set of routines are for checking for errors on
 * any previous ddi_[get|put] calls.
 */
int
pexntb_check_acc_handle(pexntb_t *pntb, ddi_acc_handle_t hdl)
{
	ddi_fm_error_t	de;

	de.fme_status = DDI_FM_OK;
	ddi_fm_acc_err_get(hdl, &de, DDI_FME_VERSION);
	if (de.fme_status != DDI_FM_OK) {
		ddi_fm_service_impact(pntb->dip, DDI_SERVICE_DEGRADED);
		return (DDI_FAILURE);
	}

	return (DDI_SUCCESS);
}

int
pexntb_check_win_acc_handle(pexntb_t *pntb, pex_bar_t *win)
{
	return (pexntb_check_acc_handle(pntb, win->pb_hdl));
}

int
pexntb_check_dma_handle(pexntb_t *pntb, ddi_dma_handle_t hdl)
{
	ddi_fm_error_t de;

	de.fme_status = DDI_FM_OK;
	ddi_fm_dma_err_get(hdl, &de, DDI_FME_VERSION);
	if (de.fme_status != DDI_FM_OK) {
		ddi_fm_service_impact(pntb->dip, DDI_SERVICE_DEGRADED);
		return (DDI_FAILURE);
	}

	return (DDI_SUCCESS);
}

void
pexntb_get_peer_link_info(pexntb_t *pntb)
{
}

int
pexntb_pair_hosts(pexntb_t *pntb)
{
	uint32_t val;

	val = PCI_XCAP_GET32(pntb->cfg_hdl, 0, pntb->vs_off, PEX_NT_VSEC_ID);
	if ((val & 0xffff) != PEX_SYNTH_NT_VSEC_ID) {
		pex_error(pntb, "!pairing failed %x", val);
		return (1);
	}

	return (0);
}

static int
pexntb_attach(dev_info_t *dip, ddi_attach_cmd_t cmd)
{
	pexntb_t *pntb;
	pci_regspec_t *regp;
	pcie_req_id_t bdf __unused;
	uint_t len;
	uint16_t reg;
	int inst;

	switch (cmd) {
	case DDI_RESUME:
		return (DDI_FAILURE);
	case DDI_ATTACH:
		break;
	default:
		return (DDI_FAILURE);
	}

	inst = ddi_get_instance(dip);
	if (ddi_soft_state_zalloc(statep, inst) != DDI_SUCCESS) {
		dev_err(dip, CE_WARN,
		    "!failed to allocate softstate for instance %d", inst);
		return (DDI_FAILURE);
	}

	pntb = ddi_get_soft_state(statep, inst);
	ASSERT(pntb != NULL);

	pntb->dip = dip;
	pntb->inst = inst;
	mutex_init(&pntb->lock, NULL, MUTEX_DEFAULT, NULL);
	sema_init(&pntb->sema, 1, "pexntb_sema", SEMA_DRIVER, NULL);

	if (pci_config_setup(dip, &pntb->cfg_hdl) != DDI_SUCCESS) {
		pex_error(pntb, "!failed to map PCI configurations");
		goto failure;
	}

	/* Enable Bus Master and Memory Space access */
	reg = pci_config_get16(pntb->cfg_hdl, PCI_CONF_COMM);
	reg |= PCI_COMM_MAE | PCI_COMM_ME | PCI_COMM_INTX_DISABLE;
	pci_config_put16(pntb->cfg_hdl, PCI_CONF_COMM, reg);

	if (pexntb_check_acc_handle(pntb, pntb->cfg_hdl) != DDI_SUCCESS)
		goto failure;

	/* The PCI address of the device */
	if (ddi_prop_lookup_int_array(DDI_DEV_T_ANY, pntb->dip,
	    DDI_PROP_DONTPASS, "reg", (int **)&regp, &len) !=
	    DDI_PROP_SUCCESS) {
		pex_error(pntb, "!failed to get 'reg' property");
		return (DDI_FAILURE);
	}
	pntb->bus = PCI_REG_BUS_G(regp[0].pci_phys_hi);
	pntb->dev = PCI_REG_DEV_G(regp[0].pci_phys_hi);
	pntb->fn = PCI_REG_FUNC_G(regp[0].pci_phys_hi);
	bdf = PCI_GETBDF(pntb->bus, pntb->dev, pntb->fn);

	/* Program BAR2 size */
	pex_log(pntb, "0x18 = %x", pci_cfgacc_get32(dip, bdf, 0x18));
	pex_log(pntb, "0x1c = %x", pci_cfgacc_get32(dip, bdf, 0x1c));
	pex_log(pntb, "0x2c = %x", pci_cfgacc_get32(dip, bdf, 0x2c));
	//pci_cfgacc_put32(dip, bdf, 0xe8, 0x0000000c);
	//pci_cfgacc_put32(dip, bdf, 0xec, 0xfffffffc);
	//pex_log(pntb, "0x18 = %x", pci_cfgacc_get32(dip, bdf, 0x18));
	//pex_log(pntb, "0x1c = %x", pci_cfgacc_get32(dip, bdf, 0x1c));

	/* Map BARs */
	if (ddi_dev_regsize(dip, PEX_BAR_WIN, &pntb->win_sz) != DDI_SUCCESS ||
	    ddi_regs_map_setup(dip, PEX_BAR_WIN, &pntb->win_base, 0,
	    pntb->win_sz, &pex_win_acc_attr, &pntb->win_hdl) != DDI_SUCCESS) {
		pex_error(pntb, "!failed to map device registers");
		goto failure;
	}
	pex_log(pntb, "bar2 %p size %x", pntb->win_base, pntb->win_sz);

	pntb->vid = pci_config_get16(pntb->cfg_hdl, PCI_CONF_VENID);
	pntb->did = pci_config_get16(pntb->cfg_hdl, PCI_CONF_DEVID);

	/* Get the vendor-specific cap offset */
	if (PCI_CAP_LOCATE(pntb->cfg_hdl, PCI_CAP_XCFG_SPC(PCIE_EXT_CAP_ID_VS),
	    &pntb->vs_off) == DDI_FAILURE)
		goto failure;
	/* Populate NT port number */
	pntb->port = (PCIE_CAP_GET(32, PCIE_DIP2BUS(dip), PCIE_LINKCAP) >>
	    PCIE_LINKCAP_PORT_NUMBER_SHIFT) & PCIE_LINKCAP_PORT_NUMBER_MASK;
	/* Populate GID */
	pntb->gid = PCI_XCAP_GET32(pntb->cfg_hdl, 0, pntb->vs_off,
	    PEX_OFFSET_GID_LOCAL);

	//pex_log(pntb, "pci_off_cap_vs %x %x", pntb->vs_off, pntb->gid);

	/* Pair hosts */
	if (pexntb_pair_hosts(pntb) != 0) {
		goto failure;
	}


#if 0
	struct domain_bus {
		uint8_t bus;
		uint8_t domain;
		uint8_t mcpu_bus;
		uint8_t mcpu_domain;
	} __packed;
	struct domain_bus db;
	db = pci_config_get32(pntb->cfg_hdl, PEX_DOMAIN_BUS_REG);
#endif


	goto failure;

#if 0
	if (ntb_register(dip, &pntb_drvr_ops, pntb) != DDI_SUCCESS) {
		pex_error(pntb, "!failed to register with ntb");
		goto failure;
	}

	if (ntb_register_kstat(dip, &pntb->pntb_kdata) != DDI_SUCCESS) {
		pex_error(pntb, "!failed to register kstat with ntb");
		goto failure;
	}
#endif

	ddi_set_driver_private(dip, pntb);
	ddi_report_dev(dip);

	return (DDI_SUCCESS);

failure:
	//(void) ntb_unregister(pntb->dip);

	mutex_destroy(&pntb->lock);
	sema_destroy(&pntb->sema);

	if (pntb->cfg_hdl != NULL)
		pci_config_teardown(&pntb->cfg_hdl);
	ddi_set_driver_private(pntb->dip, NULL);
	ddi_soft_state_free(statep, inst);

	return (DDI_FAILURE);
}

static int
pexntb_detach(dev_info_t *dip, ddi_detach_cmd_t cmd)
{
	pexntb_t	*pntb;

	switch (cmd) {
	case DDI_SUSPEND:
		return (DDI_SUCCESS);
	case DDI_DETACH:
		break;
	default:
		return (DDI_FAILURE);
	}

	pntb = ddi_get_driver_private(dip);

	if (ntb_unregister(pntb->dip) != DDI_SUCCESS) {
		/* Probably has a client driver attached */
		return (DDI_FAILURE);
	}

	mutex_destroy(&pntb->lock);
	sema_destroy(&pntb->sema);

	pci_config_teardown(&pntb->cfg_hdl);
	ddi_set_driver_private(pntb->dip, NULL);
	ddi_soft_state_free(statep, pntb->inst);

	return (DDI_SUCCESS);
}

static int
pexntb_getinfo(dev_info_t *dip, ddi_info_cmd_t cmd, void *arg, void **result)
{
	dev_t		dev = (dev_t)arg;
	minor_t		inst = getminor(dev);
	pexntb_t	*pntb;

	switch (cmd) {
	case DDI_INFO_DEVT2DEVINFO:
		if ((pntb = ddi_get_soft_state(statep, inst)) == NULL)
			return (DDI_FAILURE);
		*result = (void *)pntb->dip;
		return (DDI_SUCCESS);
	case DDI_INFO_DEVT2INSTANCE:
		*result = (void *)(uintptr_t)inst;
		return (DDI_SUCCESS);
	default:
		return (DDI_FAILURE);
	}
}

/*
 * Quiesce the driver - disable all translations to prevent spurious
 * data arriving when the system comes back up.
 */
static int
pexntb_quiesce(dev_info_t *dip)
{
	return (DDI_FAILURE);
}

static int
pexntb_bus_map(dev_info_t *dip, dev_info_t *rdip, ddi_map_req_t *mp,
    off_t offset, off_t len, caddr_t *vaddrp)
{
	dev_info_t *pdip = (dev_info_t *)DEVI(dip)->devi_parent;

	return ((DEVI(pdip)->devi_ops->devo_bus_ops->bus_map)(pdip, rdip, mp,
	    offset, len, vaddrp));
}

static int
pexntb_dma_allochdl(dev_info_t *dip, dev_info_t *rdip, ddi_dma_attr_t *attr,
    int (*waitfp)(caddr_t), caddr_t arg, ddi_dma_handle_t *handlep)
{
	return (DDI_FAILURE);
}

static int
pexntb_dma_freehdl(dev_info_t *dip, dev_info_t *rdip, ddi_dma_handle_t handle)
{
	return (DDI_FAILURE);
}

static int
pexntb_dma_bindhdl(dev_info_t *dip, dev_info_t *rdip,
    ddi_dma_handle_t handle, struct ddi_dma_req *dmareq,
    ddi_dma_cookie_t *cp, uint_t *ccountp)
{
	return (DDI_FAILURE);
}

static int
pexntb_dma_unbindhdl(dev_info_t *dip, dev_info_t *rdip, ddi_dma_handle_t handle)
{
	return (DDI_FAILURE);
}

static int
pexntb_ctl_initchild(dev_info_t *child)
{
	char name[32];
	int inst;

	/* pntb child must have "instance" property */
	inst = ddi_prop_get_int(DDI_DEV_T_ANY, child, DDI_PROP_DONTPASS,
	    "instance", -1);
	if (inst == -1) {
		dev_err(child, CE_WARN, "!'instance' property is required");
		return (DDI_FAILURE);
	}

	(void) snprintf(name, sizeof (name), "%x", inst);
	ddi_set_name_addr(child, name);

	return (DDI_SUCCESS);
}

static int
pexntb_fm_init_child(dev_info_t *dip, dev_info_t *cdip, int cap,
    ddi_iblock_cookie_t *ibc_p)
{
	pexntb_t *pntb = ddi_get_driver_private(dip);

	*ibc_p = pntb->pntb_fm_ibc;

	return (pntb->pntb_fm_cap);
}

static int
pexntb_ctlops(dev_info_t *dip, dev_info_t *rdip, ddi_ctl_enum_t ctlop,
    void *arg, void *result)
{
	switch (ctlop) {
	case DDI_CTLOPS_REPORTDEV:
		if (rdip == NULL)
			return (DDI_FAILURE);
		dev_err(dip, CE_CONT, "!child: %s@%s, %s%d\n",
		    ddi_node_name(rdip), ddi_get_name_addr(rdip),
		    ddi_driver_name(rdip), ddi_get_instance(rdip));
		return (DDI_SUCCESS);
	case DDI_CTLOPS_INITCHILD:
		return (pexntb_ctl_initchild(arg));
	case DDI_CTLOPS_UNINITCHILD:
		ddi_set_name_addr(arg, NULL);
		return (DDI_SUCCESS);
	case DDI_CTLOPS_POWER:
		return (DDI_SUCCESS);
	default:
		return (ddi_ctlops(dip, rdip, ctlop, arg, result));
	}
}

static struct bus_ops pexntb_bus_ops = {
	.busops_rev = BUSO_REV,
	.bus_add_eventcall = ndi_busop_add_eventcall,
	.bus_ctl = pexntb_ctlops,
	.bus_dma_allochdl = pexntb_dma_allochdl,
	.bus_dma_bindhdl = pexntb_dma_bindhdl,
	.bus_dma_ctl = ddi_dma_mctl,
	.bus_dma_flush = ddi_dma_flush,
	.bus_dma_freehdl = pexntb_dma_freehdl,
	.bus_dma_unbindhdl = pexntb_dma_unbindhdl,
	.bus_dma_win = ddi_dma_win,
	.bus_fm_init = pexntb_fm_init_child,
	.bus_get_eventcookie = ndi_busop_get_eventcookie,
	.bus_intr_op = i_ddi_intr_ops,
	.bus_map = pexntb_bus_map,
	.bus_map_fault = i_ddi_map_fault,
	.bus_post_event = ndi_post_event,
	.bus_prop_op = ddi_bus_prop_op,
	.bus_remove_eventcall = ndi_busop_remove_eventcall,
};

static struct dev_ops pexntb_dev_ops = {
	.devo_rev = DEVO_REV,
	.devo_attach = pexntb_attach,
	.devo_bus_ops = &pexntb_bus_ops,
	.devo_cb_ops = NULL,
	.devo_detach = pexntb_detach,
	.devo_getinfo = pexntb_getinfo,
	.devo_identify = nulldev,
	.devo_power = ddi_power,
	.devo_probe = nulldev,
	.devo_quiesce = pexntb_quiesce,
	.devo_refcnt = 0,
	.devo_reset = nodev,
};

static struct modldrv pexntb_modldrv = {
	.drv_modops = &mod_driverops,
	.drv_linkinfo = "Broadcom PEX880xx driver",
	.drv_dev_ops = &pexntb_dev_ops,
};

static struct modlinkage pexntb_modlinkage = {
	.ml_rev = MODREV_1,
	.ml_linkage = (void *)&pexntb_modldrv,
};

int
_info(struct modinfo *modinfop)
{
	return (mod_info(&pexntb_modlinkage, modinfop));
}

int
_init(void)
{
	int ret;

	ret = ddi_soft_state_init(&statep, sizeof (pexntb_t), 1);
	if (ret == 0)
		ret = mod_install(&pexntb_modlinkage);

	return (ret);
}

int
_fini(void)
{
	int ret;

	ret = mod_remove(&pexntb_modlinkage);
	if (ret == 0)
		ddi_soft_state_fini(&statep);

	return (ret);
}
