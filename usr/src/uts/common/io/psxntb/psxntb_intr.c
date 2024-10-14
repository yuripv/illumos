/*
 * Copyright 2024 Tintri by DDN, Inc. All rights reserved.
 */

#include <sys/types.h>
#include <sys/devops.h>
#include <sys/sunddi.h>
#include <sys/sunndi.h>
#include <sys/sysmacros.h>
#include <sys/sdt.h>
#include <sys/cpuvar.h>
#include "psxntb_impl.h"

static uint_t	psxntb_event_intr(caddr_t, caddr_t);
static uint_t	psxntb_doorbell_intr(caddr_t, caddr_t);
static uint_t	psxntb_message_intr(caddr_t, caddr_t);
static void	psxntb_run_db_callbacks(psxntb_t *, uint64_t, boolean_t);
static void	psxntb_set_local_doorbell(psxntb_t *, uint_t);
static int	psxntb_set_internal_db_callback(psxntb_t *, uint_t, db_func_t);
static void	psxntb_clear_internal_db_callback(psxntb_t *, uint_t,
		    db_func_t);
static void	psxntb_mark_peer_down(psxntb_t *);

#ifdef DEBUG
static uint32_t prev_flags = 0;
#endif
static uint32_t prev_peer_fw = 0;

/*
 * Mode for SDK release
 */
#define	LEGACY_MODE	0
#define	SAFE_MODE	1

typedef struct psxsdk {
	uint32_t id;
	char *name;
	uint_t mode;
} psxsdk_t;

psxsdk_t psxsdk_ver[] = {
	{PSX_FW_BETA,	"BETA",		LEGACY_MODE},
	{PSX_FW_MR0P0,	"MR0",		LEGACY_MODE},
	{PSX_FW_MR1P0,	"MR1",		LEGACY_MODE},
	{PSX_FW_MR2P0,	"MR2",		LEGACY_MODE},
	{PSX_FW_MR3P0,	"MR3",		LEGACY_MODE},
	{PSX_FW_MR3P1,	"MR3p1",	LEGACY_MODE},
	{PSX_FW_MR4P0,	"MR4",		SAFE_MODE},
	{PSX_FW_MR4P1,	"MR4p1",	SAFE_MODE},
	{PSX_FW_MR4P2,	"MR4p2",	SAFE_MODE},
	{PSX_FW_MR4P3,	"MR4p3",	SAFE_MODE},
	{PSX_FW_MR4P4,	"MR4p4",	SAFE_MODE},
	{0,		"Unknown",	LEGACY_MODE}
};

static void
sdk_version(uint32_t fwver, int *mode, char **name)
{
	psxsdk_t *sdkptr = psxsdk_ver;

	while(sdkptr->id != 0) {
		if (sdkptr->id == fwver)
			break;
		sdkptr++;
	}
	if (name != NULL)
		*name = sdkptr->name;
	if (mode != NULL)
		*mode = sdkptr->mode;
}

/*
 * Send a doorbell message across the private PCI bus.
 */
static void
psxntb_peer_dbmsg_put32(psxntb_t *pntb, uint_t off, uint32_t val)
{
	psx_bar_t	*win = pntb->pntb_seg_window;

	/*
	 * pntb_seg_window uses a LUT to direct the message to the BAR0
	 * of the NTB endpoint across the private PCI bus.
	 */

	psx_debug(pntb, EARLY | XLINK, "dbmsg_put32 0x%x = 0x%08x", off, val);
	ddi_put32(win->pb_hdl, (uint32_t *)(uintptr_t)(win->pb_vaddr +
	    pntb->pntb_peer_db_msg_off + off), val);
}

static void
psxntb_peer_dbmsg_put64(psxntb_t *pntb, uint_t off, uint64_t val)
{
	psx_bar_t	*win = pntb->pntb_seg_window;

	psx_debug(pntb, EARLY | XLINK, "dbmsg_put64 0x%x = 0x%016" PRIx64, off,
	    val);
	ddi_put64(win->pb_hdl, (uint64_t *)(uintptr_t)(win->pb_vaddr +
	    pntb->pntb_peer_db_msg_off + off), val);
}

/*
 * Setup registers to enable communications between peer ntbs.
 * That is doorbells and messages (aka scratchpad).
 */
static int
psxntb_inter_ntb_setup(psxntb_t *pntb)
{
	int	i, vec;

	if (pntb->pntb_rupt_count < 3) {
		psx_warn(pntb, "!%d is not enough interrupt vectors. At least "
		    "3 are needed", pntb->pntb_rupt_count);
		return (DDI_FAILURE);
	}

	/*
	 * 1. Get the vectors and plumb in the register settings to
	 *    tie them up to specific services: events, doorbells and
	 *    messages.
	 */

	/* The event vector */
	pntb->pntb_rupt_event = psxntb_get8(pntb, pntb->pntb_pcfg_off +
	    PNTB_CFG_EVENT_VECTOR);

	/* Assign a vector for messages */
	pntb->pntb_rupt_message = (pntb->pntb_rupt_event + 1) %
	    pntb->pntb_rupt_count;

	pntb->pntb_rupt_db_count = pntb->pntb_rupt_count - 2;
	ASSERT(pntb->pntb_rupt_db_count < PNTB_RUPT_LIMIT);

	/* Save the remaining vectors for the doorbells */
	for (vec = 0, i = 0; vec < pntb->pntb_rupt_count; vec++) {

		if (vec == pntb->pntb_rupt_event ||
		    vec == pntb->pntb_rupt_message)
			continue;

		ASSERT(i < pntb->pntb_rupt_db_count);
		psx_debug(pntb, ALWAYS, "!pntb->pntb_rupt_count=%d "
		    "pntb->pntb_rupt_db_count=%d vec=%d i=%d",
		    pntb->pntb_rupt_count, pntb->pntb_rupt_db_count, vec, i);
		pntb->pntb_rupt_doorbell[i++] = vec;
	}

	/*
	 * Now update the incoming doorbell and message map with their
	 * respective vectors. The last 4 "doorbells" are actually used to
	 * signal incoming messages, the rest are for pure doorbells.
	 */
	for (i = 0; i < PNTB_NTB_IDB_MAP_SIZE - PNTB_MESSAGES; i++) {
		/*
		 * Spread the doorbells across the doorbell vectors.
		 */
		vec = pntb->pntb_rupt_doorbell[i % pntb->pntb_rupt_db_count];
		pntb->pntb_rupt_masks[vec] |= 1ull << i;
		psxntb_put8(pntb, pntb->pntb_db_msg_off + PNTB_NTB_IDB_MAP + i,
		    vec);
	}

	for (; i < PNTB_NTB_IDB_MAP_SIZE; i++)
		psxntb_put8(pntb, pntb->pntb_db_msg_off + PNTB_NTB_IDB_MAP + i,
		    pntb->pntb_rupt_message);

	/*
	 * 2. Set up the doorbell mask. There are a total of 64 doorbells,
	 *    but the top 4 doorbells are reserved for messages, leaving
	 *    60 for general use.
	 *    In the cross-link configuration, which this driver supports,
	 *    all 60 are available to both sides. Should we ever need to
	 *    extend the driver to support direct mode, then the doorbells
	 *    would need to be split between partitions.
	 */

	pntb->pntb_db_count = PNTB_TOTAL_DOORBELLS - PNTB_MESSAGES;
	pntb->pntb_db_shift = 0;
	pntb->pntb_db_peer_shift = 0;

	/* Mask all inbound doorbells including those for messages */
	psxntb_put64(pntb, pntb->pntb_db_msg_off + PNTB_NTB_IDB_MASK, 0);

	/* Unmask outgoing doorbells to the peer */
	pntb->pntb_db_omask = ((1ull << pntb->pntb_db_count) - 1) <<
	    pntb->pntb_db_peer_shift;

	/*
	 * 3. Set the outbound message map and initialize the inbound
	 *    message registers.
	 */

	/*
	 * The outbound message map is 4 bytes long, one byte per message.
	 * Within in each byte, bits 0-1 are the message number (so 0 -> 3),
	 * bits 2-7 are the destination partition.
	 */
	pntb->pntb_msg_map = 0;
	for (i = 0; i < PNTB_MESSAGES; i++) {
		pntb->pntb_msg_map |= (((pntb->pntb_partition & 0x3f) << 2) |
		    (i & 3)) << i * 8;
	}

	/*
	 * Unmask the inbound message and clear any potentially left over
	 * status.
	 */
	for (i = 0; i < PNTB_MESSAGES; i++)
		psxntb_put64(pntb, pntb->pntb_db_msg_off + PNTB_NTB_IMSG(i),
		    PNTB_NTB_IMSG_FULL | PNTB_NTB_IMSG_IMASK);

	return (psxntb_check_gas_acc_handle(pntb));
}

void
psxntb_inter_ntb_cleanup(psxntb_t *pntb)
{
	int	i;

	/* Clear the vector map */
	for (i = 0; i < PNTB_NTB_IDB_MAP_SIZE; i++)
		psxntb_put8(pntb, pntb->pntb_db_msg_off + PNTB_NTB_IDB_MAP + i,
		    0);

	/* Mask all inbound doorbells including those for messages */
	psxntb_put64(pntb, pntb->pntb_db_msg_off + PNTB_NTB_IDB_MASK, 0);

	/* Mask inbound messages */
	for (i = 0; i < PNTB_MESSAGES; i++)
		psxntb_put64(pntb, pntb->pntb_db_msg_off + PNTB_NTB_IMSG(i), 0);
}

/*
 * Interrupt initialization.
 * Interrupts are generated for doorbells, messages and "events".
 */
int
psxntb_intr_init(psxntb_t *pntb)
{
	int	i, nintrs, count;
	uint_t	(*hdlr)(caddr_t, caddr_t);

	if (ddi_intr_get_nintrs(pntb->pntb_dip, DDI_INTR_TYPE_MSIX, &nintrs) !=
	    DDI_SUCCESS || nintrs == 0) {
		psx_warn(pntb, "!Failed to determine number of MSI-X "
		    "interrupts");
		return (DDI_FAILURE);
	}

	pntb->pntb_rupt_size = sizeof (*pntb->pntb_rupts) * nintrs;
	pntb->pntb_rupts = kmem_zalloc(pntb->pntb_rupt_size, KM_SLEEP);

	if (ddi_intr_alloc(pntb->pntb_dip, pntb->pntb_rupts, DDI_INTR_TYPE_MSIX,
	    0, nintrs, &count, DDI_INTR_ALLOC_NORMAL) != DDI_SUCCESS ||
	    count == 0) {
		psx_warn(pntb, "!Failed to allocated %d interrupts", nintrs);
		goto failed;
	}
	pntb->pntb_rupt_count = count;

	pntb->pntb_rupt_doorbell = kmem_zalloc(
	    sizeof (*pntb->pntb_rupt_doorbell) * pntb->pntb_rupt_count,
	    KM_SLEEP);
	pntb->pntb_rupt_masks = kmem_zalloc(
	    sizeof (*pntb->pntb_rupt_masks) * pntb->pntb_rupt_count,
	    KM_SLEEP);

	/*
	 * Now we know how many interrupt vectors we have, we can
	 * set up the registers to enable doorbells and messages across the
	 * PCI bus between the two systems.
	 */
	if (psxntb_inter_ntb_setup(pntb) != DDI_SUCCESS) {
		psx_warn(pntb, "!PCI error setting up inter NTB registers");
		goto failed;
	}

	if (ddi_intr_get_pri(pntb->pntb_rupts[0], &pntb->pntb_rupt_pri) !=
	    DDI_SUCCESS) {
		psx_warn(pntb, "!Failed to get interrupt priority");
		goto failed;
	}

	for (i = 0; i < count; i++) {
		if (i == pntb->pntb_rupt_event)
			hdlr = psxntb_event_intr;
		else if (i == pntb->pntb_rupt_message)
			hdlr = psxntb_message_intr;
		else
			hdlr = psxntb_doorbell_intr;

		if (ddi_intr_add_handler(pntb->pntb_rupts[i], hdlr,
		    (caddr_t)pntb, (caddr_t)(uintptr_t)i) != DDI_SUCCESS) {
			psx_warn(pntb, "!Failed to add interrupt %d", i);
			goto failed;
		}
	}

	return (DDI_SUCCESS);

failed:
	psxntb_intr_fini(pntb);
	return (DDI_FAILURE);
}

/*
 * Cleanup up after (a possibly partial) psxntb_intr_init().
 */
void
psxntb_intr_fini(psxntb_t *pntb)
{
	int	i;

	psxntb_inter_ntb_cleanup(pntb);

	if (pntb->pntb_rupt_doorbell != NULL)
		kmem_free(pntb->pntb_rupt_doorbell,
		    sizeof (*pntb->pntb_rupt_doorbell) * pntb->pntb_rupt_count);

	if (pntb->pntb_rupt_masks != NULL)
		kmem_free(pntb->pntb_rupt_masks,
		    sizeof (*pntb->pntb_rupt_masks) * pntb->pntb_rupt_count);

	for (i = 0; i < pntb->pntb_rupt_count; i++) {
		(void) ddi_intr_remove_handler(pntb->pntb_rupts[i]);

		(void) ddi_intr_free(pntb->pntb_rupts[i]);
	}

	kmem_free(pntb->pntb_rupts, pntb->pntb_rupt_size);
}

/*
 * Let the interrupts rip!
 */
int
psxntb_intr_enable(psxntb_t *pntb)
{
	int	i;

	for (i = 0; i < pntb->pntb_rupt_count; i++) {
		if (ddi_intr_enable(pntb->pntb_rupts[i]) != DDI_SUCCESS) {
			psx_warn(pntb, "!Failed to enable interrupt %d", i);
			break;
		}
	}

	if (i == pntb->pntb_rupt_count)
		return (DDI_SUCCESS);

	for (i--; i >= 0; i--)
		(void) ddi_intr_disable(pntb->pntb_rupts[i]);

	return (DDI_FAILURE);
}

void
psxntb_intr_disable(psxntb_t *pntb)
{
	int	i;

	for (i = 0; i < pntb->pntb_rupt_count; i++)
		(void) ddi_intr_disable(pntb->pntb_rupts[i]);
}

/*
 * Event interrupt handler.
 * The link enabled fields on the peer NTB endpoint are not
 * used to track the real state of the link. Instead we try and
 * read the config space of the peer on the other side of the
 * interconnect to determine whether the link is up or not.
 */
static uint_t
psxntb_event_intr(caddr_t arg1, caddr_t arg2)
{
	_NOTE(ARGUNUSED(arg2));
	psxntb_t	*pntb = (psxntb_t *)(uintptr_t)arg1;
	psx_bar_t	*win = pntb->pntb_seg_window;
	boolean_t	set_db = B_FALSE;
	uint16_t	vid;

	vid = ddi_get16(win->pb_hdl, (uint16_t *)(uintptr_t)(win->pb_vaddr +
	    pntb->pntb_peer_cfg_spc_off));

	mutex_enter(&pntb->pntb_lock);

	if (vid != pntb->pntb_vid) {
		/* only send a doorbell if we were up */
		set_db = psxntb_peer_is_up(pntb);
		psxntb_mark_peer_down(pntb);
	}

	mutex_exit(&pntb->pntb_lock);

	/*
	 * We only send the down doorbell. We wait for the "ping" to
	 * detect the peer driver is up before sending an up doorbell.
	 */
	if (set_db) {
		psxntb_set_local_doorbell(pntb, pntb->pntb_db_down);
		psx_note(pntb, "!link down");
		ntb_kstat_link_down(pntb->pntb_kdata);
	}

	return (DDI_INTR_CLAIMED);
}

/*
 * Doorbell interrupt handler.
 * Invoke any doorbell callback routines.
 */
static uint_t
psxntb_doorbell_intr(caddr_t arg1, caddr_t arg2)
{
	psxntb_t	*pntb = (psxntb_t *)(uintptr_t)arg1;
	uint_t		vec = (uint_t)(uintptr_t)arg2;
	uint64_t	set, mask;

	mask = pntb->pntb_rupt_masks[vec];

	do {
		set = psxntb_get64(pntb, pntb->pntb_db_msg_off + PNTB_NTB_IDB);
		set &= mask;

		if (set == 0)
			break;

		DTRACE_PROBE3(doorbell, psxntb_t *, pntb, uint_t, vec,
		    uint64_t, set);

		/*
		 * Clear the doorbells just read.
		 */
		psxntb_put64(pntb, pntb->pntb_db_msg_off + PNTB_NTB_IDB, set);

		/*
		 * ... and run any callbacks registered.
		 */
		psxntb_run_db_callbacks(pntb, set, B_FALSE);
	} while (/* CONSTCOND */ 1);

	if (psxntb_check_gas_acc_handle(pntb)) {
		mutex_enter(&pntb->pntb_lock);
		pntb->pntb_flags |= PNTB_FAULTED;
		mutex_exit(&pntb->pntb_lock);
	}

	return (DDI_INTR_CLAIMED);
}

/*
 * Message interrupt handler.
 * Invoke any message callback routines rtegistered.
 */
static uint_t
psxntb_message_intr(caddr_t arg1, caddr_t arg2)
{
	_NOTE(ARGUNUSED(arg2));
	psxntb_t	*pntb = (psxntb_t *)(uintptr_t)arg1;
	uint64_t	sts;
	uint32_t	msg;
	int		i, set;

	do {
		set = 0;
		for (i = 0; i < PNTB_MESSAGES; i++) {
			sts = psxntb_get64(pntb, pntb->pntb_db_msg_off +
			    PNTB_NTB_IMSG(i));
			if ((sts & PNTB_NTB_IMSG_FULL) == 0)
				/* the message register is "empty" */
				continue;

			set++;
			msg = (uint32_t)sts;

			DTRACE_PROBE3(message, psxntb_t *, pntb, int, i,
			    uint32_t, msg);

			/* Clear it. */
			psxntb_put64(pntb, pntb->pntb_db_msg_off +
			    PNTB_NTB_IMSG(i), sts);

			rw_enter(&pntb->pntb_db_lock, RW_READER);
			if (pntb->pntb_msg_callback[i] != NULL) {
				pntb->pntb_msg_callback[i](pntb, i, msg);
			}
			rw_exit(&pntb->pntb_db_lock);
		}
	} while (set > 0);

	if (psxntb_check_gas_acc_handle(pntb)) {
		mutex_enter(&pntb->pntb_lock);
		pntb->pntb_flags |= PNTB_FAULTED;
		mutex_exit(&pntb->pntb_lock);
	}

	return (DDI_INTR_CLAIMED);
}

static void
psxntb_mark_peer_up(psxntb_t *pntb)
{
	ASSERT(mutex_owned(&pntb->pntb_lock));
	pntb->pntb_flags |= PNTB_PEER_UP;
}

static void
psxntb_mark_peer_down(psxntb_t *pntb)
{
	ASSERT(mutex_owned(&pntb->pntb_lock));
	pntb->pntb_flags &= ~PNTB_PEER_UP;
	pntb->pntb_ping_misses = 0;
}

/*
 * We received a down doorbell from the peer. The peer will send
 * a down doorbell when it is shutdown cleanly.
 */
static void
psxntb_received_peer_down(void *arg, int db)
{
	_NOTE(ARGUNUSED(db));
	psxntb_t	*pntb = arg;
	boolean_t	set_db;

	mutex_enter(&pntb->pntb_lock);

	/*
	 * When we get an explicit down message, update the ping
	 * counters too.
	 */
	if (pntb->pntb_flags & PNTB_MR4SAFE_LOCAL) {
		pntb->pntb_ping_recv_latest =
		    (uint32_t)psxntb_get64(pntb,
		    PNTB_SYSINFO_SCRATCHPAD_OFFSET +
		    PNTB_SYSINFO_HEARTBEAT_OFFSET);
	} else {
		if (ddi_dma_sync(pntb->pntb_pvt_handle, 0, sizeof (uint32_t),
		    DDI_DMA_SYNC_FORKERNEL) != DDI_SUCCESS) {
			pntb->pntb_flags |= PNTB_FAULTED;
			pntb->pntb_ping_recv_latest = pntb->pntb_ping_recv;
		} else {
			pntb->pntb_ping_recv_latest =
			    *(uint32_t *)(uintptr_t)pntb->pntb_pvt_vaddr;
		}
	}

	pntb->pntb_ping_recv = pntb->pntb_ping_recv_latest;

	if ((set_db = psxntb_peer_is_up(pntb)))
		psxntb_mark_peer_down(pntb);

	mutex_exit(&pntb->pntb_lock);

	if (set_db) {
		psxntb_set_local_doorbell(pntb, pntb->pntb_db_down);
		psx_note(pntb, "!link down");
		ntb_kstat_link_down(pntb->pntb_kdata);
	}
}

/*
 * Ping the peer driver by writing incrementing values in a reserved
 * scratchpad register. At the same time verify we are receiving
 * regular updates in the scratchpad register on this side.
 */
static void
psxntb_ping_peer(void *arg)
{
	psxntb_t  	*pntb = arg;
	psx_bar_t	*win = pntb->pntb_seg_window;
	boolean_t	set_db = B_FALSE;
	int		db;
	uint32_t	fwver, peer_fwver;
	char		*local_name, *peer_name;
	int		local_mode, peer_mode;

	mutex_enter(&pntb->pntb_lock);

	if ((pntb->pntb_flags & PNTB_SHUTDOWN) != 0) {
		mutex_exit(&pntb->pntb_lock);
		return;
	}

	/*
	 * check the SDK FW version on both canisters.
	 * - if anyone is running an MR4 we must use the MR4-safe mode
	 * - else use the legacy mode
	 * We can tell instantly if the chip is MR4.
	 * MR2 does not expose the scratchpad registers, so they
	 * cannot be used with MR2.
	 * MR4 must use the scratchpad registers as writes to the
	 * mapped page of RAM are not blocked.
	 * Upgrades:
	 *   1) both canisters must have the psxntb driver upgraded before
	 *	the SES FW is upgraded. If the SES FW is upgraded before the
	 *	driver, memory corruption will happen.
	 *   2) Use the correct mode on a per-canister basis:
	 *   2a) If a canister is running MR2, use legacy mode
	 *   2b) If a canister is running MR4, or the version is not readable,
	 *	 use the MR4-safe mode.
	 * Upgrades must upgrade the OS on both canisters and have both
	 * canisters rebooted before upgrading the SES FW.
	 *   legacy mode: uses a 4k page of memory.
	 *   MR4 safe mode: uses the scratchpad registers.
	 */
	psxntb_get_sdk_fw_version(pntb, &fwver, &peer_fwver);
	sdk_version(fwver, &local_mode, &local_name);
	sdk_version(peer_fwver, &peer_mode, &peer_name);
	if (peer_fwver != 0xFFFFFFFF) {
		if (prev_peer_fw != peer_fwver) {
			cmn_err(CE_NOTE, "!psxntb: SDK versions: local(%s) "
			    "peer(%s)", local_name, peer_name);
			prev_peer_fw = peer_fwver;
			if (strcmp(local_name, "Unknown") == 0)
				cmn_err(CE_NOTE, "!psxntb: Unknown Local SDK "
				    "version: 0x%8x", fwver);
			if (strcmp(peer_name, "Unknown") == 0)
				cmn_err(CE_NOTE, "!psxntb: Unknown Peer SDK "
				    "version: 0x%8x", peer_fwver);
		}
	}

	if (peer_mode == LEGACY_MODE)
		pntb->pntb_flags &= ~PNTB_MR4SAFE_PEER;
	else
		pntb->pntb_flags |= PNTB_MR4SAFE_PEER;

	if (local_mode == LEGACY_MODE)
		pntb->pntb_flags &= ~PNTB_MR4SAFE_LOCAL;
	else
		pntb->pntb_flags |= PNTB_MR4SAFE_LOCAL;

	/*
	 * 1. Try and send regular ping hearbeats, and retrieve any
	 *    sent our way.
	 */

	if (psxntb_peer_is_faulted(pntb)) {
		pntb->pntb_ping_recv_latest = pntb->pntb_ping_recv;
	} else {
		if (pntb->pntb_flags & PNTB_MR4SAFE_PEER) {
			psxntb_putpeer64(pntb, PNTB_SYSINFO_SCRATCHPAD_OFFSET +
			    PNTB_SYSINFO_HEARTBEAT_OFFSET,
			    ++pntb->pntb_ping_sent);

			if (pntb->pntb_flags & PNTB_MR4SAFE_LOCAL) {
				pntb->pntb_ping_recv_latest =
				    (uint32_t)psxntb_get64(pntb,
				    PNTB_SYSINFO_SCRATCHPAD_OFFSET +
				    PNTB_SYSINFO_HEARTBEAT_OFFSET);
			} else {
				pntb->pntb_ping_recv_latest =
				    *(uint32_t *)(uintptr_t)
				    pntb->pntb_pvt_vaddr;
			}
		} else {
			ddi_put32(win->pb_hdl,
			    (uint32_t *)(uintptr_t)pntb->pntb_pvt_xaddr,
			    ++pntb->pntb_ping_sent);

			if (psxntb_check_win_acc_handle(pntb, win) !=
			    DDI_SUCCESS ||
			    ddi_dma_sync(pntb->pntb_pvt_handle, 0,
			    sizeof (uint32_t), DDI_DMA_SYNC_FORKERNEL) !=
			    DDI_SUCCESS) {
				pntb->pntb_flags |= PNTB_FAULTED;
				pntb->pntb_ping_recv_latest =
				    pntb->pntb_ping_recv;
			} else {
				if (pntb->pntb_flags & PNTB_MR4SAFE_LOCAL) {
					pntb->pntb_ping_recv_latest =
					    (uint32_t)psxntb_get64(pntb,
					    PNTB_SYSINFO_SCRATCHPAD_OFFSET +
					    PNTB_SYSINFO_HEARTBEAT_OFFSET);
				} else {
					pntb->pntb_ping_recv_latest =
					    *(uint32_t *)(uintptr_t)
					    pntb->pntb_pvt_vaddr;
				}
			}
		}
	}

	/*
	 * 2. Verify we have been receiving regular heartbeats.
	 */

	if (pntb->pntb_ping_recv_latest == pntb->pntb_ping_recv) {
		/*
		 * Looks like the peer may have stopped sending.
		 */
		if (psxntb_peer_is_up(pntb)) {
			if (psxntb_peer_is_faulted(pntb) &&
			    (pntb->pntb_flags & PNTB_FAULT_HANDLED) == 0) {
				pntb->pntb_flags |= PNTB_FAULT_HANDLED;
				set_db = B_TRUE;
			} else {
				set_db = ++pntb->pntb_ping_misses >
				    PNTB_PING_TIMEOUT_CNT;
			}
		}

		if (set_db) {
			psxntb_mark_peer_down(pntb);
			db = pntb->pntb_db_down;
		}
	} else {
		pntb->pntb_ping_recv = pntb->pntb_ping_recv_latest;
		pntb->pntb_ping_misses = 0;

		if (!psxntb_peer_is_up(pntb)) {
			/*
			 * The peer DMA is available.
			 * We have to reset the doorbell mask and message
			 * map across the interconnect.
			 */
			psxntb_peer_dbmsg_put64(pntb, PNTB_NTB_ODB_MASK,
			    pntb->pntb_db_omask);

			psxntb_peer_dbmsg_put32(pntb, PNTB_NTB_MSG_MAP,
			    pntb->pntb_msg_map);

			/*
			 * We can only be sure the above masks are active
			 * when we get the ack to this message.
			 * Repeatedly set the masks and send this message
			 * until we get the ack.
			 */
			(void) psxntb_send_message(pntb, PNTB_PING_MESSAGE,
			    PNTB_MSGS_UP);
		}
	}

	mutex_exit(&pntb->pntb_lock);

	if (set_db) {
		psxntb_set_local_doorbell(pntb, db);
		if (db == pntb->pntb_db_up) {
			psx_note(pntb, "!link up");
			psxntb_get_peer_link_info(pntb);
			ntb_kstat_link_up(pntb->pntb_kdata, pntb->pntb_width,
			    pntb->pntb_speed);
		} else {
			psx_note(pntb, "!link down");
			ntb_kstat_link_down(pntb->pntb_kdata);
		}
	}

#ifdef DEBUG
	if (pntb->pntb_flags != prev_flags) {
		cmn_err(CE_WARN, "!psxntb_ping_peer: exit: flags now 0x%x",
		    pntb->pntb_flags);
		prev_flags = pntb->pntb_flags;
	}
#endif
}

static void
psxntb_peer_message_cb(psxntb_t *pntb, uint_t num, uint32_t msg)
{
	if (msg == PNTB_MSGS_UP) {
		(void) psxntb_send_message(pntb, num, PNTB_MSGS_UP_ACK);
		return;
	}

	ASSERT(msg == PNTB_MSGS_UP_ACK);

	mutex_enter(&pntb->pntb_lock);
	if (psxntb_peer_is_up(pntb)) {
		mutex_exit(&pntb->pntb_lock);
		return;
	}

	psxntb_mark_peer_up(pntb);
	mutex_exit(&pntb->pntb_lock);

	psxntb_set_local_doorbell(pntb, pntb->pntb_db_up);

	psx_note(pntb, "!link up");
	psxntb_get_peer_link_info(pntb);

	ntb_kstat_link_up(pntb->pntb_kdata, pntb->pntb_width, pntb->pntb_speed);
}

/*
 * Set up a cyclic to "ping" the peer to check whether it is up or down.
 */
int
psxntb_start_pinging(psxntb_t *pntb)
{
	cyc_handler_t	cyc_hdlr;
	cyc_time_t	cyc_when;

	if (psxntb_set_msg_callback(pntb, psxntb_peer_message_cb,
	    PNTB_PING_MESSAGE) != DDI_SUCCESS)
		return (DDI_FAILURE);
	*(uint32_t *)(uintptr_t)pntb->pntb_pvt_vaddr = 0;
	if (pntb->pntb_flags & PNTB_MR4SAFE_LOCAL) {
		psxntb_put64(pntb, PNTB_SYSINFO_SCRATCHPAD_OFFSET +
		    PNTB_SYSINFO_HEARTBEAT_OFFSET, 0);
	}

	cyc_hdlr.cyh_level = CY_LOCK_LEVEL;
	cyc_hdlr.cyh_func = psxntb_ping_peer;
	cyc_hdlr.cyh_arg = pntb;

	cyc_when.cyt_when = 0;
	cyc_when.cyt_interval = PNTB_PING_INTERVAL;

	mutex_enter(&cpu_lock);
	pntb->pntb_pinger = cyclic_add(&cyc_hdlr, &cyc_when);
	mutex_exit(&cpu_lock);

	return (DDI_SUCCESS);
}

void
psxntb_stop_pinging(psxntb_t *pntb)
{
	if (pntb->pntb_pinger == CYCLIC_NONE)
		return;

	mutex_enter(&cpu_lock);
	cyclic_remove(pntb->pntb_pinger);
	mutex_exit(&cpu_lock);

	(void) psxntb_clear_msg_callback(pntb, PNTB_PING_MESSAGE);
}

/*
 * Thread which executes doorbell callback functions when a doorbell is
 * asserted locally rather than on the peer.
 */
static void
psxntb_db_thread(void *arg)
{
	psxntb_t	*pntb = arg;
	uint64_t	set;

	//lgrp_move_curthread(get_numa_lgrp_prop(pntb->pntb_dip));

	do {
		mutex_enter(&pntb->pntb_lock);
		while (pntb->pntb_db_set == 0 &&
		    (pntb->pntb_flags & PNTB_SHUTDOWN) == 0)
			cv_wait(&pntb->pntb_db_cv, &pntb->pntb_lock);

		set = pntb->pntb_db_set << pntb->pntb_db_shift;
		pntb->pntb_db_set = 0;
		mutex_exit(&pntb->pntb_lock);

		if ((pntb->pntb_flags & PNTB_SHUTDOWN) != 0)
			break;

		psxntb_run_db_callbacks(pntb, set, B_TRUE);
	} while (/* CONSTCOND */ 1);
}

/*
 * For a given set of doorbells, run all doorbell
 * callback routines registered.
 */
static void
psxntb_run_db_callbacks(psxntb_t *pntb, uint64_t set, boolean_t local)
{
	psx_db_callback_t	*entry;
	int			db;

	rw_enter(&pntb->pntb_db_lock, RW_READER);

	set >>= pntb->pntb_db_shift;
	for (db = 0; db < pntb->pntb_db_count && set != 0;
	    db++, set >>= 1) {
		if ((set & 1) == 0)
			continue;

		for (entry = list_head(&pntb->pntb_dbs[db]); entry != NULL;
		    entry = list_next(&pntb->pntb_dbs[db], entry)) {
			ASSERT(entry->pdc_db == db);
			if (entry->pdc_local == local)
				entry->pdc_fn(entry->pdc_arg, db);
		}
	}

	rw_exit(&pntb->pntb_db_lock);
}

int
psxntb_doorbells_init(psxntb_t *pntb)
{
	kthread_t	*thr;
	int		i;

	/*
	 * One callback list for each doorbell. Reserve 2 for up and down
	 * doorbells.
	 */
	pntb->pntb_db_up = pntb->pntb_db_count - 1;
	pntb->pntb_db_down = pntb->pntb_db_up - 1;
	pntb->pntb_db_high = pntb->pntb_db_down - 1;

	pntb->pntb_dbs = kmem_alloc(sizeof (list_t) * pntb->pntb_db_count,
	    KM_SLEEP);

	for (i = 0; i < pntb->pntb_db_count; i++)
		list_create(&pntb->pntb_dbs[i], sizeof (psx_db_callback_t),
		    offsetof(psx_db_callback_t, pdc_link));

	rw_init(&pntb->pntb_db_lock, NULL, RW_DRIVER,
	    DDI_INTR_PRI(pntb->pntb_rupt_pri));
	cv_init(&pntb->pntb_db_cv, NULL, CV_DRIVER, NULL);

	/*
	 * For processing doorbells posted to ourselves.
	 */
	thr = thread_create(NULL, 0, psxntb_db_thread, pntb, 0, &p0, TS_RUN,
	    maxclsyspri);
	pntb->pntb_db_thr = thr->t_did;

	/*
	 * An internal callback to handle peer going down doorbell.
	 */
	if (psxntb_set_internal_db_callback(pntb, pntb->pntb_db_down,
	    psxntb_received_peer_down) != DDI_SUCCESS) {
		psx_warn(pntb, "!Failed to set callback for internal function "
		    "\"psxntb_received_peer_down\"");
		pntb->pntb_flags |= PNTB_SHUTDOWN;
		psxntb_doorbells_fini(pntb);
		return (DDI_FAILURE);
	}

	return (DDI_SUCCESS);
}

void
psxntb_doorbells_fini(psxntb_t *pntb)
{
	int	i;

	if (pntb->pntb_dbs == NULL)
		return;

	ASSERT((pntb->pntb_flags & PNTB_SHUTDOWN) != 0);

	cv_signal(&pntb->pntb_db_cv);
	thread_join(pntb->pntb_db_thr);

	psxntb_clear_internal_db_callback(pntb, pntb->pntb_db_down,
	    psxntb_received_peer_down);

	cv_destroy(&pntb->pntb_db_cv);
	rw_destroy(&pntb->pntb_db_lock);

	for (i = 0; i < pntb->pntb_db_count; i++)
		list_destroy(&pntb->pntb_dbs[i]);

	kmem_free(pntb->pntb_dbs, sizeof (list_t) * pntb->pntb_db_count);
	pntb->pntb_dbs = NULL;
}

/*
 * Add an internal doorbell callback and set the bit in the doorbell mask.
 */
static int
psxntb_set_internal_db_callback(psxntb_t *pntb, uint_t db, db_func_t fn)
{
	psx_db_callback_t	*db_cb;

	db_cb = kmem_alloc(sizeof (*db_cb), KM_SLEEP);
	db_cb->pdc_fn = fn;
	db_cb->pdc_arg = pntb;
	db_cb->pdc_db = db;
	db_cb->pdc_dip = NULL;
	db_cb->pdc_local = B_FALSE;

	rw_enter(&pntb->pntb_db_lock, RW_WRITER);

	list_insert_head(&pntb->pntb_dbs[db], db_cb);
	pntb->pntb_db_mask |= 1ull << (db + pntb->pntb_db_shift);
	psxntb_put64(pntb, pntb->pntb_db_msg_off + PNTB_NTB_IDB_MASK,
	    pntb->pntb_db_mask);

	rw_exit(&pntb->pntb_db_lock);

	return (psxntb_check_gas_acc_handle(pntb));
}

static void
psxntb_clear_internal_db_callback(psxntb_t *pntb, uint_t db, db_func_t fn)
{
	psx_db_callback_t	*db_cb, *del_cb;
	int			real_dbs;

	rw_enter(&pntb->pntb_db_lock, RW_WRITER);

	/*
	 * Find the doorbell and function combination.
	 */
	del_cb = NULL;
	real_dbs = 0;
	for (db_cb = list_head(&pntb->pntb_dbs[db]); db_cb != NULL;
	    db_cb = list_next(&pntb->pntb_dbs[db], db_cb)) {
		if (db_cb->pdc_fn == fn) {
			ASSERT(del_cb == NULL);
			del_cb = db_cb;
		} else if (!db_cb->pdc_local) {
			real_dbs++;
		}
	}

	/*
	 * Remove it if we found one.
	 */
	if (del_cb != NULL)
		list_remove(&pntb->pntb_dbs[db], del_cb);

	if (real_dbs == 0) {
		/*
		 * No more callback functions for this doorbell,
		 * mask it off.
		 */
		pntb->pntb_db_mask &= ~(1ull << (db + pntb->pntb_db_shift));
		psxntb_put64(pntb, pntb->pntb_db_msg_off + PNTB_NTB_IDB_MASK,
		    pntb->pntb_db_mask);
	}

	rw_exit(&pntb->pntb_db_lock);

	if (del_cb != NULL)
		kmem_free(del_cb, sizeof (*del_cb));
}

/*
 * Set a callback funtion for a doorbell for a child device "dip".
 */
int
psxntb_set_db_callback(psxntb_t *pntb, dev_info_t *dip, uint_t db, db_func_t fn,
    void *arg)
{
	psx_db_callback_t	*db_cb, *entry;
	boolean_t		up;

	if (db >= pntb->pntb_db_count)
		return (DDI_FAILURE);

	db_cb = kmem_zalloc(sizeof (*db_cb), KM_SLEEP);
	db_cb->pdc_fn = fn;
	db_cb->pdc_arg = arg;
	db_cb->pdc_db = db;
	db_cb->pdc_dip = dip;

	rw_enter(&pntb->pntb_db_lock, RW_WRITER);

	for (entry = list_head(&pntb->pntb_dbs[db]); entry != NULL;
	    entry = list_next(&pntb->pntb_dbs[db], entry)) {
		/*
		 * Each dip can only have one callback per doorbell.
		 */
		if (entry->pdc_dip == dip)
			break;
	}

	if (entry != NULL)
		goto cleanup;

	list_insert_tail(&pntb->pntb_dbs[db], db_cb);

	/*
	 * up and down doorbells are not used across the NTB. Keep
	 * them masked.
	 */
	db_cb->pdc_local = db == pntb->pntb_db_up || db == pntb->pntb_db_down;
	if (!db_cb->pdc_local) {
		pntb->pntb_db_mask |= 1ull << (db + pntb->pntb_db_shift);
		psxntb_put64(pntb, pntb->pntb_db_msg_off + PNTB_NTB_IDB_MASK,
		    pntb->pntb_db_mask);

		if (psxntb_check_gas_acc_handle(pntb) != DDI_SUCCESS)
			goto cleanup;
	}

	rw_exit(&pntb->pntb_db_lock);

	up = psxntb_peer_is_up(pntb);
	if (up && db == pntb->pntb_db_up)
		psxntb_set_local_doorbell(pntb, db);
	else if (!up && db == pntb->pntb_db_down)
		psxntb_set_local_doorbell(pntb, db);

	return (DDI_SUCCESS);

cleanup:
	if (list_link_active(&db_cb->pdc_link))
		list_remove(&pntb->pntb_dbs[db], db_cb);

	rw_exit(&pntb->pntb_db_lock);
	kmem_free(db_cb, sizeof (*db_cb));

	return (DDI_FAILURE);
}

/*
 * Remove a doorbell callback registered by "dip".
 */
int
psxntb_clear_db_callback(psxntb_t *pntb, dev_info_t *dip, uint_t db)
{
	psx_db_callback_t	*db_cb, *del_cb;
	int			real_dbs;

	if (db >= pntb->pntb_db_count)
		return (DDI_FAILURE);

	rw_enter(&pntb->pntb_db_lock, RW_WRITER);

	del_cb = NULL;
	real_dbs = 0;
	for (db_cb = list_head(&pntb->pntb_dbs[db]); db_cb != NULL;
	    db_cb = list_next(&pntb->pntb_dbs[db], db_cb)) {
		if (db_cb->pdc_dip == dip) {
			ASSERT(del_cb == NULL);
			del_cb = db_cb;
		} else if (!db_cb->pdc_local) {
			real_dbs++;
		}
	}

	if (del_cb != NULL)
		list_remove(&pntb->pntb_dbs[db], del_cb);

	if (real_dbs == 0) {
		/*
		 * There are no more callbacks for this doorbell.
		 * Mask the doorbell.
		 */
		pntb->pntb_db_mask &= ~(1ull << (db + pntb->pntb_db_shift));
		psxntb_put64(pntb, pntb->pntb_db_msg_off + PNTB_NTB_IDB_MASK,
		    pntb->pntb_db_mask);

		if (psxntb_check_gas_acc_handle(pntb) != DDI_SUCCESS) {
			rw_exit(&pntb->pntb_db_lock);
			return (DDI_FAILURE);
		}
	}

	rw_exit(&pntb->pntb_db_lock);

	if (del_cb != NULL)
		kmem_free(del_cb, sizeof (*del_cb));

	return (DDI_SUCCESS);
}

/*
 * Post a doorbell at the peer.
 */
int
psxntb_set_peer_doorbell(psxntb_t *pntb, uint_t db)
{
	ASSERT(db < pntb->pntb_db_count);

	mutex_enter(&pntb->pntb_lock);
	psxntb_peer_dbmsg_put64(pntb, PNTB_NTB_ODB,
	    1ull << (db + pntb->pntb_db_peer_shift));

	/* sfence to flush the write-combining cache. */
	membar_producer();
	mutex_exit(&pntb->pntb_lock);

	return (psxntb_check_win_acc_handle(pntb, pntb->pntb_seg_window));
}

/*
 * Post a local doorbell.
 * Simply set the bit in the structure and wake up the thread.
 */
static void
psxntb_set_local_doorbell(psxntb_t *pntb, uint_t db)
{
	ASSERT(db < pntb->pntb_db_count);

	mutex_enter(&pntb->pntb_lock);
	pntb->pntb_db_set |= 1ull << db;
	cv_signal(&pntb->pntb_db_cv);
	mutex_exit(&pntb->pntb_lock);
}

int
psxntb_set_msg_callback(psxntb_t *pntb, msg_func_t fn, uint_t msg)
{
	ASSERT(msg < PNTB_MESSAGES);

	rw_enter(&pntb->pntb_db_lock, RW_WRITER);

	if (pntb->pntb_msg_callback[msg] != NULL) {
		rw_exit(&pntb->pntb_db_lock);
		return (DDI_FAILURE);
	}

	pntb->pntb_msg_callback[msg] = fn;
	pntb->pntb_db_mask |= 1ull << (msg + PNTB_TOTAL_DOORBELLS -
	    PNTB_MESSAGES);

	psxntb_put64(pntb, pntb->pntb_db_msg_off + PNTB_NTB_IDB_MASK,
	    pntb->pntb_db_mask);

	rw_exit(&pntb->pntb_db_lock);

	return (psxntb_check_gas_acc_handle(pntb));
}


int
psxntb_clear_msg_callback(psxntb_t *pntb, uint_t msg)
{
	ASSERT(msg < PNTB_MESSAGES);

	rw_enter(&pntb->pntb_db_lock, RW_WRITER);

	pntb->pntb_msg_callback[msg] = NULL;
	pntb->pntb_db_mask &= (~1ull << (msg + PNTB_TOTAL_DOORBELLS -
	    PNTB_MESSAGES));

	psxntb_put64(pntb, pntb->pntb_db_msg_off + PNTB_NTB_IDB_MASK,
	    pntb->pntb_db_mask);

	rw_exit(&pntb->pntb_db_lock);

	return (psxntb_check_gas_acc_handle(pntb));
}

int
psxntb_send_message(psxntb_t *pntb, uint_t num, uint32_t msg)
{
	ASSERT(num < PNTB_MESSAGES);

	psxntb_peer_dbmsg_put32(pntb, PNTB_NTB_OMSG(num), msg);

	return (psxntb_check_win_acc_handle(pntb, pntb->pntb_seg_window));
}
