/*
 * Copyright 2023 Tintri by DDN, Inc. All rights reserved.
 */
#include <sys/types.h>
#include <sys/devops.h>
#include <sys/sunddi.h>
#include <sys/sunndi.h>
#include <sys/sysmacros.h>
#include "psxntb_impl.h"

#ifdef DEBUG
int	psxntb_debug;
#endif

#define	PSXNTB_VERSION	"Switchtec PSX NTB Driver v0.1"

/*
 * References:
 * 1. PM85xx Switchtec PSX 96/80/64/48/32/24xG3 PCIe Gen3 Storage Switch
 *    Device Specification.
 * 2. Microsemi Switchtec Non-Transparent Bridging (NTB) Address and ID
 *    Translation Application Note.
 * 3. Microsemi Switchtec Cache Mirror Interface Guide.
 *
 * This driver supports one specific configuration and it is that
 * described in "Microsemi Switchtec Cache Mirror Interface Guide".
 * In other NTB drivers this is referred to as "back-to-back". Which is,
 * there are two NTBs one in each controller and between the two NTBs
 * there is a PCI bus that is not directly visable to either host OS.
 *
 * The switchtec device itself supports several device types, one of
 * which is the NTB. Devices within the Switchtec are each assigned a
 * partition number and configuration involves establishing links
 * between partitions. The NTB has two partitions, one partition on the
 * host side and the other on the private side. Within the driver the
 * host side parition is referenced as "pntb_partition" and the private
 * as "pntb_peer_partition".
 *
 * Address translation occurs when data is written to a suitably configured
 * host BAR. The translation configuration requires defining how the
 * address is translated and the destination partition after translation.
 * The NTB supports two type of address translation. A Direct translation
 * BAR has a single base address and mask which determines the final
 * address, calculated by using the offset of the input address from
 * the BAR's base and applying the mask. A Look Up Table (LUT) BAR is split
 * into equal size segments, each assigned an index from 0.
 * Address translation is two fold:
 * 	1. The offset from the BAR will fall within a segment. The index
 *	   of the segment is used to look up the entry in the LUT, the
 *	   LUT entry has a base address to be used for translation.
 *	2. The offset within the *segment* is used in conjunction with
 *	   base in the LUT to generate the translated address.
 *
 * The driver exposes both types of translation. The LUT method provides
 * better granularity and sharing of the NTB (as used by ntb_transport),
 * and the direct window allows us to map a large contiguous chunck
 * of memory (eg the NVDIMM as used by ppram_copy).
 */

#define	MBYTE (1024*1024)
#define	GBYTE (MBYTE*1024)

static void	*statep;

static void psxntb_release_bars(psxntb_t *);
static void psxntb_clear_mappings(psxntb_t *);
static int psxntb_update_allocations_map(psxntb_t *, dev_info_t *, int, int);
static void psxntb_release_segments(psxntb_t *, int, int);

static int psxntb_set_pvt_translations(psxntb_t *);
static void psxntb_clear_pvt_translations(psxntb_t *);

extern ddi_device_acc_attr_t	pntb_acc_attr;
extern ddi_dma_attr_t		pntb_attr_tmpl;
extern ntb_drvr_ops_t		pntb_drvr_ops;

/*
 * A set of general service routines for accessing registers of
 * different sizes in the main MMIO (BAR0) register space.
 */
void
psxntb_put8(psxntb_t *pntb, uint_t off, uint8_t val)
{
	psx_bar_t	*bar0 = &pntb->pntb_bars[0];

	ASSERT(off < bar0->pb_size);
	psx_debug(pntb, EARLY, "put8 0x%x = 0x%02x", off, val);
	ddi_put8(bar0->pb_hdl,
	    (uint8_t *)(uintptr_t)(bar0->pb_vaddr + off), val);
}

void
psxntb_put16(psxntb_t *pntb, uint_t off, uint16_t val)
{
	psx_bar_t	*bar0 = &pntb->pntb_bars[0];

	ASSERT(off < bar0->pb_size);
	psx_debug(pntb, EARLY, "put16 0x%x = 0x%04x", off, val);
	ddi_put16(bar0->pb_hdl,
	    (uint16_t *)(uintptr_t)(bar0->pb_vaddr + off), val);
}

void
psxntb_put32(psxntb_t *pntb, uint_t off, uint32_t val)
{
	psx_bar_t	*bar0 = &pntb->pntb_bars[0];

	ASSERT(off < bar0->pb_size);
	psx_debug(pntb, EARLY, "put32 0x%x = 0x%08x", off, val);
	ddi_put32(bar0->pb_hdl,
	    (uint32_t *)(uintptr_t)(bar0->pb_vaddr + off), val);
}

void
psxntb_put64(psxntb_t *pntb, uint_t off, uint64_t val)
{
	psx_bar_t	*bar0 = &pntb->pntb_bars[0];

	ASSERT(off < bar0->pb_size);
	psx_debug(pntb, EARLY, "put64 0x%x = 0x%016" PRIx64, off, val);
	ddi_put64(bar0->pb_hdl,
	    (uint64_t *)(uintptr_t)(bar0->pb_vaddr + off), val);
}

void
psxntb_putpeer64(psxntb_t *pntb, uint_t off, uint64_t val)
{
	psx_bar_t	*bar = &pntb->pntb_bars[4];

	ASSERT(off < bar->pb_size);
	psx_debug(pntb, EARLY, "putpeer64 0x%x = 0x%016" PRIx64, off, val);
	ddi_put64(bar->pb_hdl,
	    (uint64_t *)(uintptr_t)(bar->pb_vaddr + off), val);
#ifdef DEBUG
	uint64_t	rdval;

	rdval = ddi_get64(bar->pb_hdl,
	    (uint64_t *)(uintptr_t)(bar->pb_vaddr + off));
	if (rdval != val) {
		cmn_err(CE_WARN, "psxntb: psxntb_putpeer64: write failed (%"
		    PRIu64"/%"PRIu64"", val, rdval);
	}
#endif
}

uint8_t
psxntb_get8(psxntb_t *pntb, uint_t off)
{
	psx_bar_t	*bar0 = &pntb->pntb_bars[0];
#ifdef DEBUG
	uint8_t		val;

	ASSERT(off < bar0->pb_size);
	val = ddi_get8(bar0->pb_hdl,
	    (uint8_t *)(uintptr_t)(bar0->pb_vaddr + off));

	psx_debug(pntb, EARLY, "get8 0x%x = 0x%x02", off, val);

	return (val);
#else
	return (ddi_get8(bar0->pb_hdl,
	    (uint8_t *)(uintptr_t)(bar0->pb_vaddr + off)));
#endif
}

uint16_t
psxntb_get16(psxntb_t *pntb, uint_t off)
{
	psx_bar_t	*bar0 = &pntb->pntb_bars[0];
#ifdef DEBUG
	uint16_t	val;

	ASSERT(off < bar0->pb_size);
	val = ddi_get16(bar0->pb_hdl,
	    (uint16_t *)(uintptr_t)(bar0->pb_vaddr + off));

	psx_debug(pntb, EARLY, "get16 0x%x = 0x%x04", off, val);

	return (val);
#else
	return (ddi_get16(bar0->pb_hdl,
	    (uint16_t *)(uintptr_t)(bar0->pb_vaddr + off)));
#endif
}

uint32_t
psxntb_get32(psxntb_t *pntb, uint_t off)
{
	psx_bar_t	*bar0 = &pntb->pntb_bars[0];
#ifdef DEBUG
	uint32_t	val;

	ASSERT(off < bar0->pb_size);
	val = ddi_get32(bar0->pb_hdl,
	    (uint32_t *)(uintptr_t)(bar0->pb_vaddr + off));

	psx_debug(pntb, EARLY, "get32 0x%x = 0x%x08", off, val);

	return (val);
#else
	return (ddi_get32(bar0->pb_hdl,
	    (uint32_t *)(uintptr_t)(bar0->pb_vaddr + off)));
#endif
}

uint64_t
psxntb_get64(psxntb_t *pntb, uint_t off)
{
	psx_bar_t	*bar0 = &pntb->pntb_bars[0];
#ifdef DEBUG
	uint64_t	val;

	ASSERT(off < bar0->pb_size);
	val = ddi_get64(bar0->pb_hdl,
	    (uint64_t *)(uintptr_t)(bar0->pb_vaddr + off));

	psx_debug(pntb, EARLY, "get64 0x%x = 0x%016" PRIx64, off, val);

	return (val);
#else
	return (ddi_get64(bar0->pb_hdl,
	    (uint64_t *)(uintptr_t)(bar0->pb_vaddr + off)));
#endif
}

uint32_t
psxntb_getpeer32(psxntb_t *pntb, uint_t off)
{
	psx_bar_t	*bar = &pntb->pntb_bars[4];
#ifdef DEBUG
	uint32_t	val;

	ASSERT(off < bar->pb_size);
	val = ddi_get32(bar->pb_hdl,
	    (uint32_t *)(uintptr_t)(bar->pb_vaddr + off));

	psx_debug(pntb, EARLY, "getpeer32 0x%x = 0x%x08", off, val);

	return (val);
#else
	return (ddi_get32(bar->pb_hdl,
	    (uint32_t *)(uintptr_t)(bar->pb_vaddr + off)));
#endif
}

uint64_t
psxntb_getpeer64(psxntb_t *pntb, uint_t off)
{
	psx_bar_t	*bar = &pntb->pntb_bars[4];
#ifdef DEBUG
	uint64_t	val;

	ASSERT(off < bar->pb_size);
	val = ddi_get64(bar->pb_hdl,
	    (uint64_t *)(uintptr_t)(bar->pb_vaddr + off));

	psx_debug(pntb, EARLY, "getpeer64 0x%x = 0x%016" PRIx64, off, val);

	return (val);
#else
	return (ddi_get64(bar->pb_hdl,
	    (uint64_t *)(uintptr_t)(bar->pb_vaddr + off)));
#endif
}
/*
 * Claim or reference a window. A window can only be claimed by one
 * driver, but that driver can reference it as often as it needs.
 */
boolean_t
psxntb_claim_window(psxntb_t *pntb, psx_bar_t *win, dev_info_t *dip)
{
	mutex_enter(&pntb->pntb_lock);
	if (win->pb_owner != NULL && win->pb_owner != dip) {
		mutex_exit(&pntb->pntb_lock);
		return (B_FALSE);
	}

	win->pb_owner = dip;
	win->pb_ref++;
	mutex_exit(&pntb->pntb_lock);

	return (B_TRUE);
}

/*
 * Release a window, but only release it on last reference.
 */
void
psxntb_release_window(psxntb_t *pntb, psx_bar_t *win)
{
	mutex_enter(&pntb->pntb_lock);
	ASSERT(win->pb_ref > 0);
	if (--win->pb_ref == 0)
		win->pb_owner = NULL;
	mutex_exit(&pntb->pntb_lock);
}

/*
 * Gather everything we know about the BARs and set up addressability to
 * them.
 */
static int
psxntb_get_bars(psxntb_t *pntb)
{
	ddi_device_acc_attr_t	acc;
	pci_regspec_t		*regp;
	psx_bar_t		*win;
	uint_t			type, len;
	int			i, cnt, bar, offset, size;
	char 			*units;

	/*
	 * Read the "reg" property to get each BAR's type and size.
	 */
	if (ddi_prop_lookup_int_array(DDI_DEV_T_ANY, pntb->pntb_dip,
	    DDI_PROP_DONTPASS, "reg", (int **)&regp, &len) !=
	    DDI_PROP_SUCCESS) {
		psx_warn(pntb, "!Failed to get \"reg\" property");
		return (DDI_FAILURE);
	}

	/*
	 * The PCI address of the device.
	 */
	pntb->pntb_bus = PCI_REG_BUS_G(regp[0].pci_phys_hi);
	pntb->pntb_dev = PCI_REG_DEV_G(regp[0].pci_phys_hi);
	pntb->pntb_fn = PCI_REG_FUNC_G(regp[0].pci_phys_hi);

	cnt = (len * sizeof (int)) / sizeof (pci_regspec_t);
	for (i = 0; i < cnt; i++) {
		type = regp[i].pci_phys_hi & PCI_ADDR_MASK;
		offset = PCI_REG_REG_G(regp[i].pci_phys_hi);

		switch (type) {
		case PCI_ADDR_MEM32:
			win = &pntb->pntb_bars[(offset - PCI_CONF_BASE0) >> 2];
			win->pb_rnum = i;
			win->pb_type = type;
			win->pb_size = regp[i].pci_size_low;
			break;

		case PCI_ADDR_MEM64:
			win = &pntb->pntb_bars[(offset - PCI_CONF_BASE0) >> 2];
			win->pb_rnum = i;
			win->pb_type = type;
			win->pb_size = regp[i].pci_size_low |
			    (uint64_t)regp[i].pci_size_hi << 32;
			break;

		case PCI_ADDR_CONFIG:
		case PCI_ADDR_IO:
			continue;

		default:
			psx_warn(pntb, "Unrecognised BAR type: 0x%x", type);
			continue;
		}

		/*
		 * Until we know better, each BAR is a single direct window.
		 */
		win->pb_seg_size = win->pb_size;
		win->pb_seg_count = 1;
	}

	ddi_prop_free(regp);

	/*
	 * Now lookup the address information from "assigned-addresses"
	 * for each BAR.
	 */
	if (ddi_prop_lookup_int_array(DDI_DEV_T_ANY, pntb->pntb_dip,
	    DDI_PROP_DONTPASS, "assigned-addresses", (int **)&regp,
	    &len) == DDI_PROP_SUCCESS)
		cnt = (len * sizeof (int)) / sizeof (pci_regspec_t);
	else
		cnt = 0;

	/*
	 * window 0 is special, so we start inserting in slot 1 unless there
	 * is a window that is suitable for window 0. As a minimum we need
	 * to identify two windows, one of which qualifies as window 0
	 */
	pntb->pntb_wcnt = 1;
	for (i = 0; i < cnt; i++) {
		offset = PCI_REG_REG_G(regp[i].pci_phys_hi);
		type = regp[i].pci_phys_hi & PCI_ADDR_MASK;

		switch (type) {
		case PCI_ADDR_MEM32:
			bar = (offset - PCI_CONF_BASE0) >> 2;
			win = &pntb->pntb_bars[bar];
			win->pb_paddr = regp[i].pci_phys_low;
			break;

		case PCI_ADDR_MEM64:
			bar = (offset - PCI_CONF_BASE0) >> 2;
			win = &pntb->pntb_bars[bar];
			win->pb_paddr = regp[i].pci_phys_low |
			    (uint64_t)regp[i].pci_phys_mid << 32;
			break;

		default:
			continue;
		}

		/*
		 * BAR2 and above are "windows" across the NTB.
		 * The mapping of BAR to "window", in particular window 0,
		 * varies depending on system configuration. Current Athenas
		 * have 2 windows BAR2 and above
		 * 	HGST Athena has 64MB BAR2, 128GB BAR4
		 * 	DDN Athena has 256GB BAR2, 64MB BAR4
		 * Select window 0 so it's the first window that we can allocte
		 * for DMA. Currently selecting a widow size < 1GB. 64MB or
		 * higher is needed for Athena.
		 */
		if (bar >= 2) {
			if (pntb->pntb_window[0] == NULL &&
			    win->pb_size < GBYTE) {
				pntb->pntb_window[0] = win;
				psx_debug(pntb, ALWAYS, "!bar=%d pntb_wcnt=%d",
				    bar, 0);
			} else {
				win->pb_win_num = pntb->pntb_wcnt;
				pntb->pntb_window[pntb->pntb_wcnt++] = win;
				psx_debug(pntb, ALWAYS, "!bar=%d pntb_wcnt=%d",
				    bar, pntb->pntb_wcnt);
			}
		}
	}

	if (cnt > 0)
		ddi_prop_free(regp);

	if (psxntb_check_acc_handle(pntb, pntb->pntb_cfg_hdl) != DDI_SUCCESS)
		return (DDI_FAILURE);

	if (pntb->pntb_window[0] == NULL || pntb->pntb_wcnt == 1) {
		psx_warn(pntb, "!There are no appropriate Window BARs "
		    "configured. Aborting initialisation.");
		return (DDI_FAILURE);
	}

	/*
	 * Map the BARs we have to a vaddr.
	 */
	acc = pntb->pntb_acc_attr;
	for (bar = 0; bar < PCI_BASE_NUM; bar++) {
		win = &pntb->pntb_bars[bar];

		win->pb_bar = bar;
		if (win->pb_size == 0)
			continue;

		/*
		 * Use write combining for BARs other than BAR0.
		 * We use SFENCE appropriately to ensure caches are
		 * flushed.
		 */
		acc.devacc_attr_dataorder = bar == 0 ?
		    pntb->pntb_acc_attr.devacc_attr_dataorder :
		    DDI_MERGING_OK_ACC;

		if (ddi_regs_map_setup(pntb->pntb_dip, win->pb_rnum,
		    &win->pb_vaddr, 0, 0, &acc, &win->pb_hdl) != DDI_SUCCESS) {
			psx_warn(pntb, "!Failed to map BAR %d", bar);
			psxntb_release_bars(pntb);
			return (DDI_FAILURE);
		}
		if (win->pb_size >= GBYTE) {
			size = win->pb_size/GBYTE;
			units = "GB";
		} else {
			size = win->pb_size/MBYTE;
			units = "MB";
		}
		psx_note(pntb, "!%d bit BAR%d [0x%" PRIx64 " - 0x%" PRIx64 "] "
		    "%d%s",
		    win->pb_type == PCI_ADDR_MEM32 ? 32 : 64, bar,
		    win->pb_paddr, win->pb_paddr + win->pb_size - 1,
		    size, units);
	}

	return (DDI_SUCCESS);
}

/*
 * If mapped, unmap each BAR.
 */
static void
psxntb_release_bars(psxntb_t *pntb)
{
	int	i;

	for (i = 0; i < PCI_BASE_NUM; i++)
		if (pntb->pntb_bars[i].pb_hdl)
			ddi_regs_map_free(&pntb->pntb_bars[i].pb_hdl);
}

/*
 * The next set of routines are for checking for errors on
 * any previous ddi_[get|put] calls.
 */
int
psxntb_check_acc_handle(psxntb_t *pntb, ddi_acc_handle_t hdl)
{
	ddi_fm_error_t	de;

	de.fme_status = DDI_FM_OK;
	ddi_fm_acc_err_get(hdl, &de, DDI_FME_VERSION);
	if (de.fme_status != DDI_FM_OK) {
		ddi_fm_service_impact(pntb->pntb_dip, DDI_SERVICE_DEGRADED);
		return (DDI_FAILURE);
	}

	return (DDI_SUCCESS);
}

int
psxntb_check_gas_acc_handle(psxntb_t *pntb)
{
	return (psxntb_check_bar_acc_handle(pntb, 0));
}

int
psxntb_check_bar_acc_handle(psxntb_t *pntb, int bar)
{
	psx_bar_t	*ib = &pntb->pntb_bars[bar];

	return (psxntb_check_acc_handle(pntb, ib->pb_hdl));
}

int
psxntb_check_win_acc_handle(psxntb_t *pntb, psx_bar_t *win)
{
	return (psxntb_check_acc_handle(pntb, win->pb_hdl));
}

int
psxntb_check_dma_handle(psxntb_t *pntb, ddi_dma_handle_t hdl)
{
	ddi_fm_error_t	de;

	de.fme_status = DDI_FM_OK;
	ddi_fm_dma_err_get(hdl, &de, DDI_FME_VERSION);
	if (de.fme_status != DDI_FM_OK) {
		ddi_fm_service_impact(pntb->pntb_dip, DDI_SERVICE_DEGRADED);
		return (DDI_FAILURE);
	}

	return (DDI_SUCCESS);
}

/*
 * When we get told there is an I/O fault, set the device to faulted.
 * This state will be detected by the ping routine which will initiate
 * any state change callbacks.
 */
static int
psxntb_fm_err_cb(dev_info_t *dip, ddi_fm_error_t *err, const void *arg)
{
	psxntb_t	*pntb = (psxntb_t *)arg;

	pci_ereport_post(dip, err, NULL);

	mutex_enter(&pntb->pntb_lock);

	if (err->fme_status == DDI_FM_FATAL)
		pntb->pntb_flags |= PNTB_FAULTED;

	mutex_exit(&pntb->pntb_lock);

	return (err->fme_status);
}

/*
 * Enable fault checking if not overridden in properties.
 */
static void
psxntb_fm_init(psxntb_t *pntb)
{
	/*
	 * Keep per device attribute sets.
	 */
	pntb->pntb_acc_attr = pntb_acc_attr;
	pntb->pntb_dma_attr = pntb_attr_tmpl;

	/*
	 * Get any capabilities, if set, from intb.conf.
	 */
	pntb->pntb_fm_cap = ddi_prop_get_int(DDI_DEV_T_ANY, pntb->pntb_dip,
	    DDI_PROP_DONTPASS, "fm-capable", DDI_FM_EREPORT_CAPABLE |
	    DDI_FM_ACCCHK_CAPABLE | DDI_FM_DMACHK_CAPABLE |
	    DDI_FM_ERRCB_CAPABLE);

	if (DDI_FM_DEFAULT_CAP(pntb->pntb_fm_cap))
		/* None set. */
		return;

	ddi_fm_init(pntb->pntb_dip, &pntb->pntb_fm_cap, &pntb->pntb_fm_ibc);

	if (DDI_FM_ACC_ERR_CAP(pntb->pntb_fm_cap))
		pntb->pntb_acc_attr.devacc_attr_access = DDI_FLAGERR_ACC;

	if (DDI_FM_DMA_ERR_CAP(pntb->pntb_fm_cap))
		pntb->pntb_dma_attr.dma_attr_flags = DDI_DMA_FLAGERR;

	if (DDI_FM_EREPORT_CAP(pntb->pntb_fm_cap) ||
	    DDI_FM_ERRCB_CAP(pntb->pntb_fm_cap))
		pci_ereport_setup(pntb->pntb_dip);

	if (DDI_FM_ERRCB_CAP(pntb->pntb_fm_cap))
		ddi_fm_handler_register(pntb->pntb_dip, psxntb_fm_err_cb, pntb);
}

static void
psxntb_fm_fini(psxntb_t *pntb)
{
	if (DDI_FM_DEFAULT_CAP(pntb->pntb_fm_cap))
		/* None set. */
		return;

	if (DDI_FM_EREPORT_CAP(pntb->pntb_fm_cap) ||
	    DDI_FM_ERRCB_CAP(pntb->pntb_fm_cap))
		pci_ereport_teardown(pntb->pntb_dip);

	if (DDI_FM_ERRCB_CAP(pntb->pntb_fm_cap))
		ddi_fm_handler_unregister(pntb->pntb_dip);

	ddi_fm_fini(pntb->pntb_dip);
}

static int
psxntb_fm_init_child(dev_info_t *dip, dev_info_t *cdip, int cap,
    ddi_iblock_cookie_t *ibc_p)
{
	_NOTE(ARGUNUSED(cdip));
	_NOTE(ARGUNUSED(cap));
	psxntb_t	*pntb = ddi_get_driver_private(dip);

	*ibc_p = pntb->pntb_fm_ibc;

	return (pntb->pntb_fm_cap);
}

/*
 * The requester id mapping should hold entries for the root complex (0.0.0)
 * and the host bridge.
 * "requester-ids" property is an array of additional ids for other device,
 * eg ioat.
 */
static int
psxntb_configure_rid_mappings(psxntb_t *pntb)
{
	uint16_t	host_bridge;
	int		rid_limit, peer_rid_limit;
	uint16_t	*rids;
	int		*ridprop;
	int		i, rv;
	uint_t		cnt = 0;
	uint16_t	proxy_id;

	/*
	 * Get the requester id limit from this and the peer partition.
	 * Set the limit to the smallest.
	 * This limit is the maximum number of requester ids we can
	 * map.
	 */
	rid_limit = psxntb_get16(pntb, pntb->pntb_ctl_off +
	    PNTB_NTB_CNTL_RID_LIMIT);
	peer_rid_limit = psxntb_get16(pntb, pntb->pntb_peer_ctl_off +
	    PNTB_NTB_CNTL_RID_LIMIT);

	rid_limit = MIN(rid_limit, peer_rid_limit);

	if (rid_limit == 0) {
		psx_warn(pntb, "!Requester Id limit is zero!");
		return (DDI_FAILURE);
	}

	host_bridge = psxntb_get16(pntb, pntb->pntb_ntb_off +
	    PNTB_NTB_REQUESTER_ID);

	rids = kmem_alloc(sizeof (*rids) * rid_limit, KM_SLEEP);

	/*
	 * The first two requester ids are the root complex and the
	 * host bridge.
	 */
	rids[0] = 0;
	rids[1] = host_bridge;

	/*
	 * Gather any other requester ids specified in the .conf file.
	 */
	if (ddi_prop_lookup_int_array(DDI_DEV_T_ANY, pntb->pntb_dip,
	    DDI_PROP_DONTPASS, "requester_ids", &ridprop, &cnt) ==
	    DDI_SUCCESS) {
		cnt = MIN(cnt, rid_limit - 2);

		for (i = 0; i < cnt; i++)
			rids[i + 2] = (uint16_t)ridprop[i];

		ddi_prop_free(ridprop);
	}

	/*
	 * Program the requester ids.
	 */
	cnt += 2;
	rv = psxntb_set_rids(pntb, pntb->pntb_partition, rids, cnt);
	if (rv != DDI_SUCCESS) {
		psx_warn(pntb, "!Failed to set Request Ids for partition %d",
		    pntb->pntb_partition);
		goto done;
	}

	/*
	 * We also need to set RID mappings for the peer (incoming)
	 * partition. The requester IDs in the incoming partition are
	 * those which have been translated. The ids they are translated
	 * to is provided by the proxy field of the RID translation
	 * register. Read the proxy ids from the mappings just set
	 * and add translations for those in the peer.
	 */
	for (i = 0; i < cnt; i++) {
		proxy_id = psxntb_get16(pntb, pntb->pntb_ctl_off +
		    PNTB_NTB_CNTL_RID(i));

		rids[i] = PNTB_NTB_CNTL_RID_PROXY_ID(proxy_id);
	}

	/*
	 * Program the requester ids for the peer partition.
	 */
	rv = psxntb_set_rids(pntb, pntb->pntb_peer_partition, rids, cnt);
	if (rv != DDI_SUCCESS)
		psx_warn(pntb, "!Failed to set Request Ids for partition %d",
		    pntb->pntb_peer_partition);

	/*
	 * check if we want to disable the use of rid mappings
	 */
	if (ddi_prop_get_int(DDI_DEV_T_ANY, pntb->pntb_dip,
	    DDI_PROP_DONTPASS, "disable-rid-mapping", 0) == 1)
		psxntb_rid_disable(pntb);

done:
	kmem_free(rids, sizeof (*rids) * rid_limit);

	return (rv);
}

#define	BAR_BASE64	(1ull << 56)
#define	BAR_BASE32	(1u << 24)

/*
 * Between the two systems there is a private PCI bus which belongs
 * to the peer partition. The BAR registers are not enumerated (except
 * by us), so we need to assign address to each BAR on the private
 * PCI bus.
 */
static int
psxntb_init_peer_config_space(psxntb_t *pntb)
{
	psx_bar_t	*bar;
	int		i;

	for (i = 0; i < PCI_BASE_NUM; i++) {
		bar = &pntb->pntb_bars[i];

		/*
		 * If the BAR on the host side is not configured, there
		 * is not point in configuring the peer's BAR.
		 */
		if (bar->pb_size == 0)
			continue;

		if (bar->pb_type == PCI_ADDR_MEM64)
			psxntb_put64(pntb, pntb->pntb_peer_cfg_spc_off +
			    PCI_CONF_BASE0 + bar->pb_bar * 4,
			    BAR_BASE64 << i);
		else
			psxntb_put32(pntb, pntb->pntb_peer_cfg_spc_off +
			    PCI_CONF_BASE0 + bar->pb_bar * 4,
			    BAR_BASE32 << i);
	}

	/*
	 * Enable memory access and bus mastering on the private PCI bus.
	 */
	psxntb_put16(pntb, pntb->pntb_peer_cfg_spc_off + PCI_CONF_COMM,
	    PCI_COMM_MAE | PCI_COMM_ME);

	return (psxntb_check_gas_acc_handle(pntb));
}

/*
 * This reads the BAR address out of the config space in the peer partition.
 */
static uint64_t
psxntb_get_peer_bar(psxntb_t *pntb, int bar)
{
	uint64_t	addr;

	addr = psxntb_get32(pntb, pntb->pntb_peer_cfg_spc_off +
	    PCI_CONF_BASE0 + bar * 4);
	if ((addr & PCI_BASE_TYPE_ALL) != 0) {
		/* 64 bit address */
		addr &= PCI_BASE_M_ADDR_M;
		addr |= (uint64_t)psxntb_get32(pntb,
		    pntb->pntb_peer_cfg_spc_off + PCI_CONF_BASE0 +
		    bar * 4 + 4) << 32;
	} else {
		addr &= PCI_BASE_M_ADDR_M;
	}

	return (addr);
}

static int
psxntb_identify_peer_partition(psxntb_t *pntb)
{
	uint64_t	map;
	uint32_t	info_offset = PNTB_NTB_PART_INFO(pntb->pntb_partition);
	int		part;

	/*
	 * Get the partition map out of the NTB information registers
	 * specific to pntb_partition.
	 */
	map = psxntb_get32(pntb, pntb->pntb_ntb_off + info_offset +
	    PNTB_NTB_PART_INFO_LOW);
	map |= (uint64_t)psxntb_get32(pntb, pntb->pntb_ntb_off + info_offset +
	    PNTB_NTB_PART_INFO_HIGH) << 32;

	part = ddi_ffs((long)map);
	if (part != 0) {
		/*
		 * There is a partition map. The bit set will be for
		 * the peer partition. Use it and check whether cross
		 * link is enabled.
		 */

		if (part != highbit(map)) {
			psx_warn(pntb, "!There is more than one mapping");
			return (DDI_FAILURE);
		}

		pntb->pntb_peer_partition = part - 1;

		info_offset = PNTB_NTB_PART_INFO(pntb->pntb_peer_partition);
		if (psxntb_get8(pntb, pntb->pntb_ntb_off + info_offset +
		    PNTB_NTB_PART_INFO_X_ENABLED) == 0) {
			psx_warn(pntb, "!Cross link mode is not enabled");
			return (DDI_FAILURE);
		}

		return (DDI_SUCCESS);
	}

	/*
	 * The specific partition map for pntb_partition was empty.
	 * Attempt to get the peer partition from the global endpoint map.
	 */
	map = psxntb_get64(pntb, pntb->pntb_ntb_off + PNTB_NTB_ENDPOINT_MAP);
	map &= ~(1ull << pntb->pntb_partition);
	pntb->pntb_peer_partition = ddi_ffs((long)map) - 1;
	if (pntb->pntb_peer_partition < 0) {
		psx_warn(pntb, "!Not enough partitions");
		return (DDI_FAILURE);
	}

	return (DDI_SUCCESS);
}

/*
 * The GAS (Global Address Space) holds registers for all the supported
 * partitions, determine our and the peer's partition id and calculate
 * commonly needed offsets within the GAS.
 * Also, set up initial NTB mappings to enable access across to the
 * peer partition on the *other side* of PCI bus between the two controllers.
 * This will enable the use of doorbells and message registers.
 */
static int
psxntb_init_offsets_and_mappings(psxntb_t *pntb)
{
	psx_bar_t	*win;
	uint64_t	bar;
	uint64_t	bar_size, seg_size, seg_count;
	uint32_t	vep_inst_id;
	int		i;
	uint16_t	peer_count;

	/* NTB registers. */
	pntb->pntb_ntb_off = PNTB_GAS_NTB_OFFSET;

	pntb->pntb_partition_cnt = psxntb_get8(pntb, pntb->pntb_ntb_off +
	    PNTB_NTB_PARTITION_NUMBER);
	pntb->pntb_partition = psxntb_get8(pntb, pntb->pntb_ntb_off +
	    PNTB_NTB_PARTITION_ID);

	if (psxntb_identify_peer_partition(pntb) != DDI_SUCCESS)
		return (DDI_FAILURE);

	psx_note(pntb, "!Local partition: %d, Peer: %d", pntb->pntb_partition,
	    pntb->pntb_peer_partition);

	/* This partition's configuration area. */
	pntb->pntb_pcfg_off = PNTB_GAS_CFG_SPACE(pntb->pntb_partition);

	/* The peer partition's configuration area. */
	pntb->pntb_peer_pcfg_off =
	    PNTB_GAS_CFG_SPACE(pntb->pntb_peer_partition);

	/* This partition's control registers. */
	pntb->pntb_ctl_off = pntb->pntb_ntb_off +
	    PNTB_NTB_CNTL_SPACE(pntb->pntb_partition);

	/* The peer partition's control registers. */
	pntb->pntb_peer_ctl_off = pntb->pntb_ntb_off +
	    PNTB_NTB_CNTL_SPACE(pntb->pntb_peer_partition);

	/* This partition's doorbell and messages area. */
	pntb->pntb_db_msg_off = pntb->pntb_ntb_off +
	    PNTB_NTB_DB_MSG_SPACE(pntb->pntb_partition);

	/* The peer partition's doorbell and messages area. */
	pntb->pntb_peer_db_msg_off = pntb->pntb_ntb_off +
	    PNTB_NTB_DB_MSG_SPACE(pntb->pntb_peer_partition);

	/* The peers partition's configuration space. */
	vep_inst_id = psxntb_get32(pntb, pntb->pntb_peer_pcfg_off +
	    PNTB_CFG_VEP_PFF_INST_ID);
	if (vep_inst_id == -1u) {
		psx_warn(pntb, "!Peer partition is not valid");
		return (DDI_FAILURE);
	}

	pntb->pntb_peer_cfg_spc_off = PNTB_NTB_CONFIG_SPACE(vep_inst_id);

	/*
	 * Window zero will be segmented. Some of the segments are used
	 * by this driver to read and write registers across the private
	 * PCI bus, the rest of the segments are made available to child
	 * drivers.
	 */
	pntb->pntb_seg_window = pntb->pntb_window[0];
	win = pntb->pntb_seg_window;

	/*
	 * Set the base for child driver windows beyond the segment
	 * window, and reduce the available window count accordingly.
	 */
	pntb->pntb_win_base = 1;
	pntb->pntb_wcnt--;

	/*
	 * Enumerate the BARs in the peer's configuration space.
	 */
	if (psxntb_init_peer_config_space(pntb) != DDI_SUCCESS) {
		psx_warn(pntb, "!Failed to initialize BARs in peer's "
		    "PCI config space");
		return (DDI_FAILURE);
	}

	/*
	 * Get the address of the remote peer's BAR2. This is used
	 * to set LUT entries beyond those used for BAR0 to map
	 * to the peer's BAR2 LUT.
	 * We rely on our BAR2 (and BAR4) in the peer partition to be
	 * the same on either side of the private PCi bus. Since we
	 * explicitly program these addresses ourself it is a safe
	 * assumption.
	 */
	win->pb_peer_paddr = psxntb_get_peer_bar(pntb, win->pb_bar);

	/*
	 * The LUT is potentially shared by all windows (BARs). The
	 * LUT base is the starting point in the LUT for a given
	 * window. At the moment we only let one window (the segment
	 * window) access the LUT and that starts at offset 0.
	 */
	win->pb_lut_base = 0;

	/*
	 * Determine how many LUT entries are available to us. We cannot
	 * use more than the smaller of the local and peer partition.
	 */
	win->pb_seg_count = psxntb_get16(pntb, pntb->pntb_ctl_off +
	    PNTB_NTB_CNTL_LUT_LIMIT);
	peer_count = psxntb_get16(pntb, pntb->pntb_peer_ctl_off +
	    PNTB_NTB_CNTL_LUT_LIMIT);

	win->pb_seg_count = MIN(peer_count, win->pb_seg_count);

	if (win->pb_seg_count == 0) {
		psx_warn(pntb, "!LUT in either partition is zero!");
		return (DDI_FAILURE);
	}

	/*
	 * The actual segment size (size each LUT entry covers) is
	 * determined by the size of the BAR itself and how many
	 * segments are configured. It is then rounded down to the
	 * previous power of 2.
	 */
	win->pb_seg_size = win->pb_size / win->pb_seg_count;
	win->pb_seg_size = 1ULL << (highbit((ulong_t)win->pb_seg_size) - 1);

	/*
	 * Initialize the BAR setup of the segmented window for LUT in
	 * both partitions.
	 */
	if (psxntb_setup_lut(pntb, win, pntb->pntb_partition) != DDI_SUCCESS) {
		psx_warn(pntb, "!Failed to set up lut for BAR%d, partition %d",
		    win->pb_bar, pntb->pntb_partition);
		return (DDI_FAILURE);
	}

	if (psxntb_setup_lut(pntb, win, pntb->pntb_peer_partition) !=
	    DDI_SUCCESS) {
		psx_warn(pntb, "!Failed to set up lut for BAR%d, peer "
		    "partition %d", win->pb_bar, pntb->pntb_peer_partition);
		goto failed;
	}

	/*
	 * Determine how many segments (LUT entries) are needed to cover
	 * BAR0.
	 */
	seg_size = win->pb_seg_size;
	bar_size = pntb->pntb_bars[0].pb_size + seg_size - 1;
	seg_count = bar_size / seg_size;
	if (seg_count > win->pb_seg_count) {
		psx_warn(pntb, "!%" PRId64 "LUT entries are required to map "
		    "into remote GAS. Only %d are available", seg_count,
		    win->pb_seg_count);
		goto failed;
	}

	/*
	 * Get the address of the peer's BAR0.
	 * We will map this into the BAR2 LUT starting from index 0.
	 * We are assuming symmetrical configurations, so when we write to
	 * the beginning of *our* BAR2, the message will be translated and
	 * arrive in the peer partition's BAR0 in the *remote* system.
	 */
	bar = psxntb_get_peer_bar(pntb, 0);
	pntb->pntb_bars[0].pb_peer_paddr = bar;

	/*
	 * Set the translation addresses from entry 0 in the LUT for
	 * BAR0.
	 */
	for (i = 0; i < seg_count; i++) {
		/*
		 * Map from "bar" in pb_seg_size increments into LUT.
		 */
		if (psxntb_set_lut_window(pntb, win->pb_lut_base + i,
		    pntb->pntb_peer_partition, bar + i * seg_size) !=
		    DDI_SUCCESS) {
			psx_warn(pntb, "!Failed to set LUT %d to 0x%" PRIx64,
			    i, bar + i * seg_size);
			goto failed;
		}
	}

	/*
	 * Entries in the LUT for child drivers begin after those used
	 * internally.
	 */
	win->pb_lut_child = win->pb_lut_base + (uint_t)seg_count;

	/*
	 * Mark these segments as used in the allocations map.
	 */
	if (psxntb_update_allocations_map(pntb, pntb->pntb_dip, 0, seg_count) !=
	    DDI_SUCCESS) {
		psx_warn(pntb, "!Failed to reserve LUT entries for internal "
		    "mappings");
		goto failed;
	}

	/*
	 * Finally, update the requester id mappings.
	 */
	if (psxntb_configure_rid_mappings(pntb) == DDI_SUCCESS)
		return (DDI_SUCCESS);

failed:
	psxntb_clear_mappings(pntb);

	return (DDI_FAILURE);
}

/*
 * Clear out all LUT and requester id mappings.
 */
static void
psxntb_clear_mappings(psxntb_t *pntb)
{
	uint64_t	seg_size, bar_size, seg_count;
	int		i;

	if (pntb->pntb_seg_window == NULL)
		return;

	(void) psxntb_clear_lut(pntb, pntb->pntb_seg_window,
	    pntb->pntb_partition);
	(void) psxntb_clear_lut(pntb, pntb->pntb_seg_window,
	    pntb->pntb_peer_partition);

	seg_size = pntb->pntb_seg_window->pb_seg_size;
	bar_size = pntb->pntb_bars[0].pb_size + seg_size - 1;
	seg_count = MIN(bar_size / seg_size,
	    pntb->pntb_seg_window->pb_seg_count);

	for (i = 0; i < seg_count; i++) {
		(void) psxntb_clear_lut_window(pntb,
		    pntb->pntb_seg_window->pb_lut_base + i,
		    pntb->pntb_peer_partition);
	}

	psxntb_release_segments(pntb, 0, seg_count);

	(void) psxntb_clear_rids(pntb, pntb->pntb_partition);
	(void) psxntb_clear_rids(pntb, pntb->pntb_peer_partition);
}

void
psxntb_get_peer_link_info(psxntb_t *pntb)
{
	uint16_t	reg;

	reg = psxntb_get16(pntb, pntb->pntb_peer_cfg_spc_off +
	    PNTB_NTB_CONFIG_LNK_STS);

	pntb->pntb_speed = PNTB_NTB_CONFIG_LNK_SPEED(reg);
	pntb->pntb_width = PNTB_NTB_CONFIG_LNK_WIDTH(reg);

	psx_note(pntb, "!Link %d x GEN%d", pntb->pntb_width, pntb->pntb_speed);
}

void
psxntb_get_sdk_fw_version(psxntb_t *pntb, uint32_t *fwver,
    uint32_t *peer_fwver)
{
	*fwver = psxntb_get32(pntb, PNTB_SYSINFO_OFFSET +
	    PNTB_SYSINFO_FWVER_OFFSET);
	*peer_fwver = psxntb_getpeer32(pntb, PNTB_SYSINFO_OFFSET +
	    PNTB_SYSINFO_FWVER_OFFSET);
}

static int
psxntb_attach(dev_info_t *dip, ddi_attach_cmd_t cmd)
{
	ddi_acc_handle_t cfg_hdl;
	psxntb_t	*pntb;
	int		inst;
	uint16_t	reg;
	boolean_t	own = B_FALSE;

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
		dev_err(dip, CE_WARN, "!Failed to allocate softstate for "
		    "instance: %d", inst);
		return (DDI_FAILURE);
	}

	pntb = ddi_get_soft_state(statep, inst);
	ASSERT(pntb != NULL);

	pntb->pntb_dip = dip;
	pntb->pntb_inst = inst;
	mutex_init(&pntb->pntb_lock, NULL, MUTEX_DEFAULT, NULL);
	sema_init(&pntb->pntb_sema, 1, "psxntb_sema", SEMA_DRIVER, NULL);

	psxntb_fm_init(pntb);

	if (pci_config_setup(dip, &cfg_hdl) != DDI_SUCCESS) {
		psx_warn(pntb, "!pci_config_setup() failed for instance: %d",
		    inst);
		goto failure;
	}
	pntb->pntb_cfg_hdl = cfg_hdl;

	pntb->pntb_vid = pci_config_get16(cfg_hdl, PCI_CONF_VENID);
	pntb->pntb_did = pci_config_get16(cfg_hdl, PCI_CONF_DEVID);
	psx_warn(pntb, "!PCI Vendor: 0x%04x Device: 0x%04x", pntb->pntb_vid,
	    pntb->pntb_did);

	/*
	 * Enable Bus Master and Memory Space access.
	 */
	reg = pci_config_get16(pntb->pntb_cfg_hdl, PCI_CONF_COMM);
	reg |= PCI_COMM_MAE | PCI_COMM_ME;
	pci_config_put16(pntb->pntb_cfg_hdl, PCI_CONF_COMM, reg);

	if (psxntb_check_acc_handle(pntb, pntb->pntb_cfg_hdl) != DDI_SUCCESS)
		goto failure;

	if (psxntb_get_bars(pntb) != DDI_SUCCESS)
		goto failure;

	/*
	 * Initialize pertinent offsets within the GAS.
	 */
	if (psxntb_init_offsets_and_mappings(pntb) != DDI_SUCCESS)
		goto failure;

	/*
	 * Initialize interrupts and setup doorbell and message mappings
	 * between local and peer partition.
	 */
	if (psxntb_intr_init(pntb) != DDI_SUCCESS)
		goto failure;

	/*
	 * Start the doorbell processing thread.
	 */
	if (psxntb_doorbells_init(pntb) != DDI_SUCCESS)
		goto failure;

	if (psxntb_intr_enable(pntb) != DDI_SUCCESS)
		goto failure;

	psxntb_get_peer_link_info(pntb);

	/*
	 * Claim the segmented window for this driver. Any segmented mappings
	 * must come though us.
	 */
	own = psxntb_claim_window(pntb, pntb->pntb_seg_window, pntb->pntb_dip);
	if (!own) {
		psx_warn(pntb, "!Failed to claim segment window");
		goto failure;
	}

	/*
	 * Initialise private translations to the peers config space and
	 * BAR0.
	 */
	if (psxntb_set_pvt_translations(pntb) != DDI_SUCCESS)
		goto failure;

#ifdef DEBUG
	psxntb_debug &= ~EARLY;
#endif

	if (psxntb_start_pinging(pntb) != DDI_SUCCESS)
		goto failure;

	if (ntb_register(dip, &pntb_drvr_ops, pntb) != DDI_SUCCESS) {
		psx_warn(pntb, "!Failed to register with ntb_svc");
		goto failure;
	}

	if (ntb_register_kstat(dip, &pntb->pntb_kdata) != DDI_SUCCESS) {
		psx_warn(pntb, "!Failed to register kstat with ntb_svc");
		goto failure;
	}

	ddi_set_driver_private(dip, pntb);
	ddi_report_dev(dip);

	return (DDI_SUCCESS);

failure:
	(void) ntb_unregister(pntb->pntb_dip);

	mutex_enter(&pntb->pntb_lock);
	pntb->pntb_flags |= PNTB_SHUTDOWN;
	mutex_exit(&pntb->pntb_lock);

	psxntb_stop_pinging(pntb);

	psxntb_doorbells_fini(pntb);

	if (pntb->pntb_rupt_count > 0) {
		psxntb_intr_disable(pntb);
		psxntb_intr_fini(pntb);
	}

	psxntb_clear_pvt_translations(pntb);

	psxntb_clear_mappings(pntb);
	if (own)
		psxntb_release_window(pntb, pntb->pntb_seg_window);

	psxntb_fm_fini(pntb);
	psxntb_release_bars(pntb);
	mutex_destroy(&pntb->pntb_lock);
	sema_destroy(&pntb->pntb_sema);

	if (pntb->pntb_cfg_hdl != NULL)
		pci_config_teardown(&pntb->pntb_cfg_hdl);

	ddi_set_driver_private(pntb->pntb_dip, NULL);
	ddi_soft_state_free(statep, inst);

	return (DDI_FAILURE);
}


static int
psxntb_detach(dev_info_t *dip, ddi_detach_cmd_t cmd)
{
	psxntb_t	*pntb;

	switch (cmd) {
	case DDI_SUSPEND:
		return (DDI_SUCCESS);
	case DDI_DETACH:
		break;
	default:
		return (DDI_FAILURE);
	}

	pntb = ddi_get_driver_private(dip);

	if (ntb_unregister(pntb->pntb_dip) != DDI_SUCCESS)
		/* probably has a client driver attached */
		return (DDI_FAILURE);

	mutex_enter(&pntb->pntb_lock);
	pntb->pntb_flags |= PNTB_SHUTDOWN;
	mutex_exit(&pntb->pntb_lock);

	psxntb_stop_pinging(pntb);

	/* Let the peer know we are going away */
	(void) psxntb_set_peer_doorbell(pntb, pntb->pntb_db_down);

	psxntb_doorbells_fini(pntb);

	psxntb_clear_pvt_translations(pntb);

	psxntb_clear_mappings(pntb);
	psxntb_release_window(pntb, pntb->pntb_seg_window);

	psxntb_fm_fini(pntb);

	psxntb_intr_disable(pntb);
	psxntb_intr_fini(pntb);

	psxntb_release_bars(pntb);

	mutex_destroy(&pntb->pntb_lock);
	sema_destroy(&pntb->pntb_sema);

	pci_config_teardown(&pntb->pntb_cfg_hdl);
	ddi_set_driver_private(pntb->pntb_dip, NULL);

	ddi_soft_state_free(statep, pntb->pntb_inst);

	return (DDI_SUCCESS);
}

static int
psxntb_getinfo(dev_info_t *dip, ddi_info_cmd_t cmd, void *arg, void **result)
{
	_NOTE(ARGUNUSED(dip));
	dev_t		dev = (dev_t)arg;
	minor_t		inst = getminor(dev);
	psxntb_t	*pntb;

	switch (cmd) {
	case DDI_INFO_DEVT2DEVINFO:
		if ((pntb = ddi_get_soft_state(statep, inst)) == NULL)
			return (DDI_FAILURE);

		*result = (void *)pntb->pntb_dip;
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
psxntb_quiesce(dev_info_t *dip)
{
	psxntb_t	*pntb = ddi_get_driver_private(dip);

	/* Let the peer know we are going away */
	(void) psxntb_set_peer_doorbell(pntb, pntb->pntb_db_down);

	return (psxntb_quiesce_translation(pntb));
}

static int
psxntb_bus_map(dev_info_t *dip, dev_info_t *rdip, ddi_map_req_t *mp,
    off_t offset, off_t len, caddr_t *vaddrp)
{
	dev_info_t *pdip = (dev_info_t *)DEVI(dip)->devi_parent;

	return ((DEVI(pdip)->devi_ops->devo_bus_ops->bus_map)(pdip, rdip, mp,
	    offset, len, vaddrp));
}

/*
 * Allocate a DMA handle. As well as the standard ddi_dma_handle we
 * allocate a shadow handle (psx_dma_handle) to keep data we need.
 * We keep this in a driver private pointer in ddi_dma_handle.
 */
static int
psxntb_dma_allochdl(dev_info_t *dip, dev_info_t *rdip, ddi_dma_attr_t *attr,
    int (*waitfp)(caddr_t), caddr_t arg, ddi_dma_handle_t *handlep)
{
	psxntb_t		*pntb = ddi_get_driver_private(dip);
	psx_dma_handle_t	*pntb_h;
	ddi_dma_impl_t		*i_hdl;
	psx_bar_t		*win;
	pntb_hdl_t		seg_type;
	size_t			seg_size;
	int			rv, window, seg, seg_cnt;

	if (dip == rdip)
		return (ddi_dma_allochdl(dip, rdip, attr, waitfp, arg,
		    handlep));

	if (ntb_get_dma_window_by_dev(rdip, &window) == DDI_SUCCESS) {
		/*
		 * The "ntb-dma-window" property is set. It holds the window
		 * rdip wants to use.
		 */
		if (window >= pntb->pntb_wcnt) {
			psx_warn(pntb, "!Window %d is out of range: "
			    "[0, %d]", window, pntb->pntb_wcnt - 1);
			return (DDI_FAILURE);
		}

		win = pntb->pntb_window[pntb->pntb_win_base + window];
		if (!psxntb_claim_window(pntb, win, rdip)) {
			psx_warn(pntb, "!Window %d is already in use.",
			    window);
			return (DDI_FAILURE);
		}

		/*
		 * Get the address in the BAR on the private PCI bus.
		 */
		win->pb_peer_paddr = psxntb_get_peer_bar(pntb, win->pb_bar);

		seg = 0;
		seg_cnt = win->pb_seg_count;
		seg_size = win->pb_seg_size;
		seg_type = WINDOW_HDL;
	} else if (ntb_get_dma_segments_by_dev(rdip, &seg, &seg_cnt) ==
	    DDI_SUCCESS) {
		/*
		 * "ntb-dma-segment" and "ntb-dma-segment-count" properties
		 * are set. rdip is requesting a range of segments out
		 * of the segmented window.
		 */
		win = pntb->pntb_seg_window;
		seg_size = win->pb_seg_size;
		seg_type = SEGMENT_HDL;
	} else {
		/*
		 * A normal ddi_dma_alloc_handle() call.
		 */
		win = NULL;
		seg = 0;
		seg_cnt = 0;
		seg_size = 0;
		seg_type = PASS_THRU_HDL;
	}

	if ((rv = ddi_dma_allochdl(dip, rdip, attr, waitfp, arg, handlep)) !=
	    DDI_SUCCESS) {
		if (seg_type == WINDOW_HDL)
			psxntb_release_window(pntb, win);

		return (rv);
	}

	/*
	 * Create a private handle to hold our data.
	 */
	pntb_h = kmem_zalloc(sizeof (*pntb_h), KM_SLEEP);
	pntb_h->pdh_type = seg_type;
	pntb_h->pdh_seg = seg;
	pntb_h->pdh_seg_cnt = seg_cnt;
	pntb_h->pdh_seg_size = seg_size;
	pntb_h->pdh_window = win;
	pntb_h->pdh_handle = *handlep;
	i_hdl = (ddi_dma_impl_t *)(*handlep);
	i_hdl->dmai_driver_private = pntb_h;

	return (DDI_SUCCESS);
}

static int
psxntb_dma_freehdl(dev_info_t *dip, dev_info_t *rdip, ddi_dma_handle_t handle)
{
	psxntb_t		*pntb = ddi_get_driver_private(dip);
	ddi_dma_impl_t		*i_hdl = (ddi_dma_impl_t *)handle;
	psx_dma_handle_t	*pntb_h = i_hdl->dmai_driver_private;

	if (pntb_h->pdh_type == WINDOW_HDL)
		psxntb_release_window(pntb, pntb_h->pdh_window);

	kmem_free(pntb_h, sizeof (*pntb_h));

	return (ddi_dma_freehdl(dip, rdip, handle));
}

/*
 * The allocation map has 1 bit per segment. When bit 'n' is set then
 * segment 'n' is in use.
 */
static int
psxntb_update_allocations_map(psxntb_t *pntb, dev_info_t *rdip, int seg,
    int cnt)
{
	int		i, first_bit, bit_cnt;
	uint32_t	mask;

	mutex_enter(&pntb->pntb_lock);

	/*
	 * Reserve segments in 2 phases: first check all the requested segments
	 * are free, then update the bit map.
	 */
	first_bit = seg % 32;
	bit_cnt = cnt;
	for (i = seg / 32; i < PNTB_SEGMENTS / 32 && bit_cnt > 0; i++) {
		mask = bit_cnt >= 32 ? -1u : ((1u << bit_cnt) - 1);

		if (pntb->pntb_alloc_map[i] & (mask << first_bit))
			break;

		bit_cnt -= 32 - first_bit;
		first_bit = 0;
	}

	if (bit_cnt > 0) {
		mutex_exit(&pntb->pntb_lock);
		dev_err(rdip, CE_WARN, "!Failed to find %d segments starting "
		    "at %d", cnt, seg);
		return (DDI_FAILURE);
	}

	/*
	 * Now update the bitmap
	 */
	first_bit = seg % 32;
	bit_cnt = cnt;
	for (i = seg / 32; i < PNTB_SEGMENTS / 32 && bit_cnt > 0; i++) {
		mask = bit_cnt >= 32 ? -1u : ((1u << bit_cnt) - 1);

		pntb->pntb_alloc_map[i] |= mask << first_bit;

		bit_cnt -= 32 - first_bit;
		first_bit = 0;
	}

	mutex_exit(&pntb->pntb_lock);

	return (DDI_SUCCESS);
}

/*
 * Reserve segments either defined by the "ntb-bind-segment_..." or
 * "ntb-dma-segment-..." properties. "ntb-bind-segment_..." is always
 * a subset of "ntb-dma-segment-..." and when not specified the full
 * range in "ntb-dma-segment-..." is reserved.
 */
static int
psxntb_reserve_segments(psxntb_t *pntb, dev_info_t *rdip, int *segp, int *cntp)
{
	psx_bar_t	*win = pntb->pntb_seg_window;
	int		seg;

	if (ntb_get_bind_limits_by_dev(rdip, segp, cntp) != DDI_SUCCESS &&
	    ntb_get_dma_segments_by_dev(rdip, segp, cntp) != DDI_SUCCESS)
		return (DDI_FAILURE);

	seg = win->pb_lut_child - win->pb_lut_base + *segp;

	return (psxntb_update_allocations_map(pntb, rdip, seg, *cntp));
}

/*
 * Release a range of segments previously reserved.
 */
static void
psxntb_release_segments(psxntb_t *pntb, int seg, int cnt)
{
	int		i, first_bit;
	uint32_t	mask;

	seg += pntb->pntb_seg_window->pb_lut_child -
	    pntb->pntb_seg_window->pb_lut_base;

	mutex_enter(&pntb->pntb_lock);

	first_bit = seg % 32;
	for (i = seg / 32; i < PNTB_SEGMENTS / 32 && cnt > 0; i++) {
		mask = cnt >= 32 ? -1u : ((1u << cnt) - 1);

		pntb->pntb_alloc_map[i] &= ~(mask << first_bit);

		cnt -= 32 - first_bit;
		first_bit = 0;
	}

	mutex_exit(&pntb->pntb_lock);
}

/*
 * Bind processing for a handle which has a full window assigned.
 * The dma_cookie provided is used to update the NTB registers to
 * complete the mappings.
 * The mapping established in this function is the translation of the
 * address in the TLP packet as it is traverses the private PCI bus from
 * the other controller and ingresses into the NTB in this controller.
 * It translates the address to target the physical memory in this
 * controller.
 */
static int
psxntb_bind_peer_window(dev_info_t *dip, dev_info_t *rdip,
    psx_dma_handle_t *pntb_h, struct ddi_dma_req *dmareq,
    ddi_dma_cookie_t *cp, uint_t *ccountp)
{
	psxntb_t		*pntb = ddi_get_driver_private(dip);
	ddi_dma_handle_t 	handle = pntb_h->pdh_handle;
	ddi_dma_impl_t		*i_hdl = (ddi_dma_impl_t *)handle;
	ddi_dma_attr_t		*dma_attr = &i_hdl->dmai_attr;
	psx_bar_t		*win = pntb_h->pdh_window;
	uint64_t		paddr;

	if ((dma_attr->dma_attr_align % win->pb_size) != 0) {
		dev_err(rdip, CE_WARN, "!DMA request must be 0x%lx "
		    "aligned", win->pb_size);
		return (DDI_FAILURE);
	}

	if (ddi_dma_bindhdl(dip, rdip, handle, dmareq, cp, ccountp) !=
	    DDI_SUCCESS)
		return (DDI_FAILURE);

	ASSERT(*ccountp == 1);

	/*
	 * Take a copy of the cookie for future reference.
	 */
	pntb_h->pdh_cookies = kmem_alloc(sizeof (ddi_dma_cookie_t), KM_SLEEP);
	pntb_h->pdh_cookies[0] = *cp;
	pntb_h->pdh_cookie_cnt = 1;

	/*
	 * We (pntb_partition) are the target for the configuration update.
	 * Update the BAR setup in the peer to translate to the address
	 * in the ddi_dma_cookie.
	 */
	paddr = cp->dmac_laddress & ~(win->pb_size - 1);
	if (psxntb_set_direct_window(pntb, win, pntb->pntb_partition,
	    paddr) != DDI_SUCCESS) {
		dev_err(rdip, CE_WARN, "!Failed to set direct window in "
		    "peer partition");
		(void) ddi_dma_unbindhdl(dip, rdip, handle);
		return (DDI_FAILURE);
	}

	return (DDI_SUCCESS);
}

static int
psxntb_unbind_peer_window(dev_info_t *dip, dev_info_t *rdip,
    psx_dma_handle_t *pntb_h)
{
	psxntb_t	*pntb = ddi_get_driver_private(dip);
	psx_bar_t	*win = pntb_h->pdh_window;

	/*
	 * Remove the translation and close the window.
	 */
	if (psxntb_clear_direct_window(pntb, win, pntb->pntb_partition) !=
	    DDI_SUCCESS) {
		dev_err(rdip, CE_WARN, "!Failed to clear direct window in "
		    "peer partition");
		return (DDI_FAILURE);
	}

	kmem_free(pntb_h->pdh_cookies, pntb_h->pdh_cookie_cnt *
	    sizeof (ddi_dma_cookie_t));

	return (ddi_dma_unbindhdl(dip, rdip, pntb_h->pdh_handle));
}

/*
 * Establish a mapping such that TLP packets with addresses targeted at the
 * window's BAR are translated to an address on the private PCI bus, this
 * is the address of the corresponding BAR in the other controller.
 * When the TLP packet ingresses the other NTB at that address it will undergo
 * the translation as setup in psxntb_bind_peer_window() to target a physical
 * addreass in the that controller.
 */
static int
psxntb_bind_local_window(dev_info_t *dip, dev_info_t *rdip,
    psx_dma_handle_t *pntb_h, struct ddi_dma_req *dmareq,
    ddi_dma_cookie_t *cp, uint_t *ccountp)
{
	psxntb_t		*pntb = ddi_get_driver_private(dip);
	ddi_dma_handle_t	handle = pntb_h->pdh_handle;
	psx_bar_t		*win = pntb_h->pdh_window;

	if (ddi_dma_bindhdl(dip, rdip, handle, dmareq, cp, ccountp) !=
	    DDI_SUCCESS)
		return (DDI_FAILURE);

	/*
	 * The peer partition is the target for the configuration update.
	 * Update the BAR setup in the local partition to translate to an
	 * addresses targetted at the BAR to an address on the private
	 * PCI bus (pb_peer_paddr).
	 */
	if (psxntb_set_direct_window(pntb, win, pntb->pntb_peer_partition,
	    win->pb_peer_paddr) != DDI_SUCCESS) {
		dev_err(rdip, CE_WARN, "!Failed to set direct window in "
		    "local partition");
		(void) ddi_dma_unbindhdl(dip, rdip, handle);
		return (DDI_FAILURE);
	}

	pntb_h->pdh_dma_addr = dmareq->dmar_object.dmao_obj.virt_obj.v_addr;
	pntb_h->pdh_dma_len = dmareq->dmar_object.dmao_size;

	return (DDI_SUCCESS);
}

static int
psxntb_unbind_local_window(dev_info_t *dip, dev_info_t *rdip,
    psx_dma_handle_t *pntb_h)
{
	psxntb_t		*pntb = ddi_get_driver_private(dip);
	ddi_dma_handle_t	handle = pntb_h->pdh_handle;
	psx_bar_t		*win = pntb_h->pdh_window;

	if (psxntb_clear_direct_window(pntb, win, pntb->pntb_peer_partition) !=
	    DDI_SUCCESS) {
		dev_err(rdip, CE_WARN, "!Failed to clear direct window in "
		    "local partition");
		return (DDI_FAILURE);
	}

	pntb_h->pdh_dma_addr = NULL;
	pntb_h->pdh_dma_len = 0;

	return (ddi_dma_unbindhdl(dip, rdip, handle));
}

/*
 * Same as the previous group of functions which are for a complete window,
 * these perform the equivalent, accept on a range of segments within the
 * segment window. So rather than configuring the BAR as a whole, we
 * set translations in appropriate LUT entries.
 */
static int
psxntb_bind_peer_segments(dev_info_t *dip, dev_info_t *rdip,
    psx_dma_handle_t *pntb_h, struct ddi_dma_req *dmareq,
    ddi_dma_cookie_t *cp, uint_t *ccountp)
{
	psxntb_t		*pntb = ddi_get_driver_private(dip);
	ddi_dma_handle_t	handle = pntb_h->pdh_handle;
	ddi_dma_impl_t		*i_hdl = (ddi_dma_impl_t *)handle;
	ddi_dma_attr_t		*dma_attr = &i_hdl->dmai_attr;
	ddi_dma_cookie_t	*cookie0;
	int			seg, cnt, i;

	if ((dma_attr->dma_attr_align % pntb_h->pdh_seg_size) != 0) {
		dev_err(rdip, CE_WARN, "!DMA request must be 0x%lx "
		    "aligned", pntb_h->pdh_seg_size);
		return (DDI_FAILURE);
	}

	/*
	 * Get the segment range assigned to rdip, and attempt to reserve
	 * them.
	 */
	if (psxntb_reserve_segments(pntb, rdip, &seg, &cnt) != DDI_SUCCESS) {
		dev_err(rdip, CE_WARN, "!Not able to reserve segments");
		return (DDI_FAILURE);
	}

	pntb_h->pdh_seg = seg;
	pntb_h->pdh_seg_cnt = cnt;

	if (ddi_dma_bindhdl(dip, rdip, handle, dmareq, cp, ccountp) !=
	    DDI_SUCCESS)
		goto release;

	/* copy the cookies for our use. */
	pntb_h->pdh_cookies = kmem_alloc(sizeof (ddi_dma_cookie_t) *
	    *ccountp, KM_SLEEP);
	pntb_h->pdh_cookie_cnt = *ccountp;

	cookie0 = i_hdl->dmai_cookie;
	pntb_h->pdh_cookies[0] = *cp;
	for (i = 1; i < *ccountp; i++)
		ddi_dma_nextcookie(handle, &pntb_h->pdh_cookies[i]);
	i_hdl->dmai_cookie = cookie0;

	if (psxntb_set_lut_peer(pntb, pntb_h) != DDI_FAILURE)
		return (DDI_SUCCESS);

	/* Failed. */
	kmem_free(pntb_h->pdh_cookies, sizeof (ddi_dma_cookie_t) *
	    pntb_h->pdh_cookie_cnt);

	(void) ddi_dma_unbindhdl(dip, rdip, handle);

release:
	psxntb_release_segments(pntb, seg, cnt);

	pntb_h->pdh_seg = 0;
	pntb_h->pdh_seg_cnt = 0;

	return (DDI_FAILURE);
}

static int
psxntb_unbind_peer_segments(dev_info_t *dip, dev_info_t *rdip,
    psx_dma_handle_t *pntb_h)
{
	psxntb_t		*pntb = ddi_get_driver_private(dip);
	ddi_dma_handle_t	handle = pntb_h->pdh_handle;
	int			rv;

	if (psxntb_clear_lut_peer(pntb, pntb_h) != DDI_SUCCESS) {
		dev_err(rdip, CE_WARN, "!Failed to clear peer LUT");
		return (DDI_FAILURE);
	}

	kmem_free(pntb_h->pdh_cookies, sizeof (ddi_dma_cookie_t) *
	    pntb_h->pdh_cookie_cnt);

	pntb_h->pdh_cookies = NULL;
	pntb_h->pdh_cookie_cnt = 0;

	rv = ddi_dma_unbindhdl(dip, rdip, handle);

	psxntb_release_segments(pntb, pntb_h->pdh_seg, pntb_h->pdh_seg_cnt);

	pntb_h->pdh_seg = 0;
	pntb_h->pdh_seg_cnt = 0;

	return (rv);
}

static int
psxntb_bind_local_segments(dev_info_t *dip, dev_info_t *rdip,
    psx_dma_handle_t *pntb_h, struct ddi_dma_req *dmareq,
    ddi_dma_cookie_t *cp, uint_t *ccountp)
{
	psxntb_t		*pntb = ddi_get_driver_private(dip);
	ddi_dma_handle_t	handle = pntb_h->pdh_handle;
	int			seg, cnt;

	if (ntb_get_bind_limits_by_dev(rdip, &seg, &cnt) == DDI_SUCCESS) {
		/* Apply any bind limits via properties. */
		pntb_h->pdh_seg = seg;
		pntb_h->pdh_seg_cnt = cnt;
	}

	if (ddi_dma_bindhdl(dip, rdip, handle, dmareq, cp, ccountp) !=
	    DDI_SUCCESS)
		return (DDI_FAILURE);

	if (psxntb_set_lut_local(pntb, pntb_h) != DDI_SUCCESS) {
		(void) ddi_dma_unbindhdl(dip, rdip, handle);
		return (DDI_FAILURE);
	}

	pntb_h->pdh_dma_addr = dmareq->dmar_object.dmao_obj.virt_obj.v_addr;
	pntb_h->pdh_dma_len = dmareq->dmar_object.dmao_size;

	return (DDI_SUCCESS);
}

static int
psxntb_unbind_local_segments(dev_info_t *dip, dev_info_t *rdip,
    psx_dma_handle_t *pntb_h)
{
	psxntb_t		*pntb = ddi_get_driver_private(dip);
	ddi_dma_handle_t	handle = pntb_h->pdh_handle;

	if (psxntb_clear_lut_local(pntb, pntb_h) != DDI_SUCCESS) {
		dev_err(rdip, CE_WARN, "!Failed to clear local LUT");
		return (DDI_FAILURE);
	}

	pntb_h->pdh_dma_addr = NULL;
	pntb_h->pdh_dma_len = 0;

	return (ddi_dma_unbindhdl(dip, rdip, handle));
}

/*
 * Bind bus_ops function.
 * Different paths when the handle is tied to a full window, segmented
 * window or is non-NTB related.
 */
static int
psxntb_dma_bindhdl(dev_info_t *dip, dev_info_t *rdip,
    ddi_dma_handle_t handle, struct ddi_dma_req *dmareq,
    ddi_dma_cookie_t *cp, uint_t *ccountp)
{
	ddi_dma_impl_t		*i_hdl = (ddi_dma_impl_t *)handle;
	ddi_dma_attr_t		*dma_attr = &i_hdl->dmai_attr;
	psx_dma_handle_t	*pntb_h = i_hdl->dmai_driver_private;
	boolean_t		peer_bind;
	int			rv;

	peer_bind = (dma_attr->dma_attr_flags & PNTB_DMA_PEER) != 0;

	switch (pntb_h->pdh_type) {
	case SEGMENT_HDL:
		rv = (peer_bind ? psxntb_bind_peer_segments :
		    psxntb_bind_local_segments)(dip, rdip, pntb_h, dmareq,
		    cp, ccountp);
		break;

	case WINDOW_HDL:
		rv = (peer_bind ? psxntb_bind_peer_window :
		    psxntb_bind_local_window)(dip, rdip, pntb_h, dmareq,
		    cp, ccountp);
		break;

	case PASS_THRU_HDL:
		rv = ddi_dma_bindhdl(dip, rdip, handle, dmareq, cp,
		    ccountp);
		break;

	default:
		rv = DDI_FAILURE;
		break;
	}

	return (rv);
}

/*
 * Unbind bus_ops.
 */
static int
psxntb_dma_unbindhdl(dev_info_t *dip, dev_info_t *rdip, ddi_dma_handle_t handle)
{
	ddi_dma_impl_t		*i_hdl = (ddi_dma_impl_t *)handle;
	ddi_dma_attr_t		*dma_attr = &i_hdl->dmai_attr;
	psx_dma_handle_t	*pntb_h = i_hdl->dmai_driver_private;
	boolean_t		peer_bind;
	int			rv;

	peer_bind = (dma_attr->dma_attr_flags & PNTB_DMA_PEER) != 0;

	switch (pntb_h->pdh_type) {
	case SEGMENT_HDL:
		rv = (peer_bind ? psxntb_unbind_peer_segments :
		    psxntb_unbind_local_segments)(dip, rdip, pntb_h);
		break;

	case WINDOW_HDL:
		rv = (peer_bind ? psxntb_unbind_peer_window :
		    psxntb_unbind_local_window)(dip, rdip, pntb_h);
		break;

	case PASS_THRU_HDL:
		rv = ddi_dma_unbindhdl(dip, rdip, handle);
		break;

	default:
		rv = DDI_FAILURE;
		break;
	}

	return (rv);
}

/*
 * Sets up the name to be used in the device tree.
 */
static int
psxntb_ctl_initchild(dev_info_t *child)
{
	char	name[32];
	int	inst;

	/*
	 * pntb child must have "instance" property.
	 */
	inst = ddi_prop_get_int(DDI_DEV_T_ANY, child, DDI_PROP_DONTPASS,
	    "instance", -1);

	if (inst == -1) {
		dev_err(child, CE_WARN, "!\"instance\" property is required");
		return (DDI_FAILURE);
	}

	(void) snprintf(name, sizeof (name), "%x", inst);
	ddi_set_name_addr(child, name);

	return (DDI_SUCCESS);
}

/*
 * bus_ctl bus_op.
 */
static int
psxntb_ctlops(dev_info_t *dip, dev_info_t *rdip, ddi_ctl_enum_t ctlop,
    void *arg, void *result)
{
	psxntb_t	*pntb = ddi_get_driver_private(dip);

	switch (ctlop) {
	case DDI_CTLOPS_REPORTDEV:
		if (rdip == NULL)
			return (DDI_FAILURE);

		psx_cont(pntb, "?Child: %s@%s, %s%d\n",
		    ddi_node_name(rdip), ddi_get_name_addr(rdip),
		    ddi_driver_name(rdip), ddi_get_instance(rdip));

		return (DDI_SUCCESS);

	case DDI_CTLOPS_INITCHILD:
		return (psxntb_ctl_initchild(arg));

	case DDI_CTLOPS_UNINITCHILD:
		ddi_set_name_addr(arg, NULL);
		return (DDI_SUCCESS);

	case DDI_CTLOPS_POWER:
		return (DDI_SUCCESS);

	default:
		return (ddi_ctlops(dip, rdip, ctlop, arg, result));
	}
}

static struct bus_ops psxntb_bus_ops = {
	.busops_rev = BUSO_REV,
	.bus_map = psxntb_bus_map,
	.bus_map_fault = i_ddi_map_fault,
	.bus_dma_map = NULL,
	.bus_dma_allochdl = psxntb_dma_allochdl,
	.bus_dma_freehdl = psxntb_dma_freehdl,
	.bus_dma_bindhdl = psxntb_dma_bindhdl,
	.bus_dma_unbindhdl = psxntb_dma_unbindhdl,
	.bus_dma_flush = ddi_dma_flush,
	.bus_dma_win = ddi_dma_win,
	.bus_dma_ctl = ddi_dma_mctl,
	.bus_ctl = psxntb_ctlops,
	.bus_prop_op = ddi_bus_prop_op,
	.bus_get_eventcookie = ndi_busop_get_eventcookie,
	.bus_add_eventcall = ndi_busop_add_eventcall,
	.bus_remove_eventcall = ndi_busop_remove_eventcall,
	.bus_post_event = ndi_post_event,
	.bus_fm_init = psxntb_fm_init_child,
	.bus_intr_op = i_ddi_intr_ops
};


static struct dev_ops psxntb_dev_ops = {
	DEVO_REV,		/* devo_rev */
	0,			/* devo_refcnt */
	psxntb_getinfo,		/* devo_getinfo */
	nulldev,		/* devo_identify */
	nulldev,		/* devo_probe */
	psxntb_attach,		/* devo_attach */
	psxntb_detach,		/* devo_detach */
	nodev,			/* devo_reset */
	NULL,			/* devo_cb_ops */
	&psxntb_bus_ops,	/* devo_bus_ops */
	ddi_power,		/* devo_power */
	psxntb_quiesce		/* devo_quiesce */
};

static struct modldrv psxntb_modldrv = {
	&mod_driverops,
	PSXNTB_VERSION,
	&psxntb_dev_ops
};

static struct modlinkage psxntb_modlinkage = {
	MODREV_1,
	(void *)&psxntb_modldrv,
	NULL
};

int
_info(struct modinfo *modinfop)
{
	return (mod_info(&psxntb_modlinkage, modinfop));
}

int
_init(void)
{
	int	rv;

	rv = ddi_soft_state_init(&statep, sizeof (psxntb_t), 1);
	if (rv == 0)
		rv = mod_install(&psxntb_modlinkage);

	return (rv);
}

int
_fini(void)
{
	int	rv;

	rv = mod_remove(&psxntb_modlinkage);
	if (rv == 0)
		ddi_soft_state_fini(&statep);

	return (rv);
}

static void
psxntb_free_pvt_dma(psxntb_t *pntb)
{
	if (pntb->pntb_pvt_acc_handle != NULL)
		ddi_dma_mem_free(&pntb->pntb_pvt_acc_handle);

	if (pntb->pntb_pvt_paddr != (uintptr_t)NULL)
		(void) ddi_dma_unbind_handle(pntb->pntb_pvt_handle);

	if (pntb->pntb_pvt_handle != NULL)
		ddi_dma_free_handle(&pntb->pntb_pvt_handle);

	pntb->pntb_pvt_handle = NULL;
	pntb->pntb_pvt_acc_handle = NULL;
	pntb->pntb_pvt_vaddr = NULL;
	pntb->pntb_pvt_paddr = (uintptr_t)NULL;
}

/*
 * Allocate a page of DMA to exchange private information across the
 * interconnect. Currently these are only alive counters.
 */
static int
psxntb_allocate_pvt_dma(psxntb_t *pntb)
{
	ddi_dma_attr_t		attr;
	ddi_dma_cookie_t	cookie;
	uint_t			cookie_cnt;
	size_t			len;

	attr = pntb->pntb_dma_attr;
	attr.dma_attr_seg = pntb->pntb_seg_window->pb_seg_size - 1;
	attr.dma_attr_align = pntb->pntb_seg_window->pb_seg_size;
	attr.dma_attr_sgllen = 1;
	attr.dma_attr_minxfer = 1U << PNTB_MIN_SIZE;

	psx_debug(pntb, EARLY, "!attr.dma_attr_seg=0x%" PRIx64,
	    pntb->pntb_seg_window->pb_seg_size);
	if (ddi_dma_alloc_handle(pntb->pntb_dip, &attr, DDI_DMA_SLEEP, NULL,
	    &pntb->pntb_pvt_handle) != DDI_SUCCESS) {
		psx_warn(pntb, "!Failed ddi_dma_alloc_handle");
		return (DDI_FAILURE);
	}
	if (ddi_dma_mem_alloc(pntb->pntb_pvt_handle, PAGESIZE,
	    &pntb->pntb_acc_attr, DDI_DMA_STREAMING, DDI_DMA_SLEEP, NULL,
	    &pntb->pntb_pvt_vaddr, &len, &pntb->pntb_pvt_acc_handle) !=
	    DDI_SUCCESS) {
		psx_warn(pntb, "!Failed ddi_dma_mem_alloc");
		goto failed;
	}

	if (ddi_dma_addr_bind_handle(pntb->pntb_pvt_handle, NULL,
	    pntb->pntb_pvt_vaddr, len, DDI_DMA_STREAMING,
	    DDI_DMA_SLEEP, NULL, &cookie, &cookie_cnt) != DDI_DMA_MAPPED) {
		psx_warn(pntb, "!Failed ddi_dma_addr_bind_handle");
		goto failed;
	}

	pntb->pntb_pvt_paddr = cookie.dmac_laddress;
	pntb->pntb_pvt_size = len;

	return (DDI_SUCCESS);

failed:
	psxntb_free_pvt_dma(pntb);

	return (DDI_FAILURE);
}

static int
psxntb_set_pvt_translations(psxntb_t *pntb)
{
	psx_bar_t	*win = pntb->pntb_seg_window;

	if (psxntb_allocate_pvt_dma(pntb) != DDI_SUCCESS) {
		psx_warn(pntb, "!Failed to allocate private DMA");
		return (DDI_FAILURE);
	}

	/*
	 * Fake up a psx handle.
	 */
	pntb->pntb_psx_handle.pdh_seg = 0;
	pntb->pntb_psx_handle.pdh_seg_cnt = 1;
	pntb->pntb_psx_handle.pdh_seg_size = win->pb_seg_size;
	pntb->pntb_psx_handle.pdh_dma_addr = pntb->pntb_pvt_vaddr;
	pntb->pntb_psx_handle.pdh_dma_len = pntb->pntb_pvt_size;

	pntb->pntb_psx_handle.pdh_cookies = kmem_alloc(
	    sizeof (ddi_dma_cookie_t), KM_SLEEP);
	pntb->pntb_psx_handle.pdh_cookie_cnt = 1;
	pntb->pntb_psx_handle.pdh_cookies[0].dmac_laddress =
	    pntb->pntb_pvt_paddr;
	pntb->pntb_psx_handle.pdh_cookies[0].dmac_size = pntb->pntb_pvt_size;
	pntb->pntb_psx_handle.pdh_cookies[0].dmac_type = 0;

	pntb->pntb_psx_handle.pdh_window = win;

	if (psxntb_set_lut_local(pntb, &pntb->pntb_psx_handle) != DDI_SUCCESS) {
		psx_warn(pntb, "!Failed to set local lut for private use");
		goto failed;
	}

	if (psxntb_set_lut_peer(pntb, &pntb->pntb_psx_handle) != DDI_SUCCESS) {
		psx_warn(pntb, "!Failed to set peer lut for private use");
		goto failed;
	}

	if (psxntb_update_allocations_map(pntb, pntb->pntb_dip,
	    win->pb_lut_child, 1) != DDI_SUCCESS) {
		psx_warn(pntb, "!Failed to reserve private segment");
		goto failed;
	}

	pntb->pntb_pvt_offset = win->pb_seg_size *
	    (win->pb_lut_child - win->pb_lut_base);

	pntb->pntb_pvt_xaddr = win->pb_vaddr + pntb->pntb_pvt_offset;

	win->pb_lut_child++;

	return (DDI_SUCCESS);

failed:
	(void) psxntb_clear_lut_peer(pntb, &pntb->pntb_psx_handle);
	(void) psxntb_clear_lut_local(pntb, &pntb->pntb_psx_handle);

	psxntb_free_pvt_dma(pntb);

	kmem_free(pntb->pntb_psx_handle.pdh_cookies, sizeof (ddi_dma_cookie_t));

	return (DDI_FAILURE);
}

static void
psxntb_clear_pvt_translations(psxntb_t *pntb)
{
	if (pntb->pntb_pvt_vaddr == NULL)
		return;

	pntb->pntb_seg_window->pb_lut_child--;

	psxntb_release_segments(pntb, 0, 1);

	(void) psxntb_clear_lut_peer(pntb, &pntb->pntb_psx_handle);
	(void) psxntb_clear_lut_local(pntb, &pntb->pntb_psx_handle);

	psxntb_free_pvt_dma(pntb);

	if (pntb->pntb_psx_handle.pdh_cookies != NULL) {
		kmem_free(pntb->pntb_psx_handle.pdh_cookies,
		    sizeof (ddi_dma_cookie_t));
		pntb->pntb_psx_handle.pdh_cookies = NULL;
	}
}
