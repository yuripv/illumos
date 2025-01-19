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
 * Copyright 2025 Tintri by DDN, Inc. All rights reserved.
 */

#include <sys/types.h>
#include <sys/devops.h>
#include <sys/sunddi.h>
#include <sys/sunndi.h>
#include <sys/sysmacros.h>

#include "ntb_hw_psx.h"

#ifdef DEBUG
int	ntb_psx_debug;
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

static void ntb_psx_release_bars(ntb_psx_t *);
static void ntb_psx_clear_mappings(ntb_psx_t *);
static int ntb_psx_update_allocations_map(ntb_psx_t *, dev_info_t *, int, int);
static void ntb_psx_release_segments(ntb_psx_t *, int, int);

static int ntb_psx_set_pvt_translations(ntb_psx_t *);
static void ntb_psx_clear_pvt_translations(ntb_psx_t *);

extern ddi_device_acc_attr_t	ntb_psx_acc_attr;
extern ddi_dma_attr_t		pntb_attr_tmpl;
extern ntb_drv_ops_t		ntb_psx_ops;

/*
 * A set of general service routines for accessing registers of
 * different sizes in the main MMIO (BAR0) register space.
 */
void
ntb_psx_put8(ntb_psx_t *pntb, uint_t off, uint8_t val)
{
	psx_bar_t	*bar0 = &pntb->pntb_bars[0];

	ASSERT(off < bar0->pb_size);
	psx_debug(pntb, EARLY, "put8 0x%x = 0x%02x", off, val);
	ddi_put8(bar0->pb_hdl,
	    (uint8_t *)(uintptr_t)(bar0->pb_vaddr + off), val);
}

void
ntb_psx_put16(ntb_psx_t *pntb, uint_t off, uint16_t val)
{
	psx_bar_t	*bar0 = &pntb->pntb_bars[0];

	ASSERT(off < bar0->pb_size);
	psx_debug(pntb, EARLY, "put16 0x%x = 0x%04x", off, val);
	ddi_put16(bar0->pb_hdl,
	    (uint16_t *)(uintptr_t)(bar0->pb_vaddr + off), val);
}

void
ntb_psx_put32(ntb_psx_t *pntb, uint_t off, uint32_t val)
{
	psx_bar_t	*bar0 = &pntb->pntb_bars[0];

	ASSERT(off < bar0->pb_size);
	psx_debug(pntb, EARLY, "put32 0x%x = 0x%08x", off, val);
	ddi_put32(bar0->pb_hdl,
	    (uint32_t *)(uintptr_t)(bar0->pb_vaddr + off), val);
}

void
ntb_psx_put64(ntb_psx_t *pntb, uint_t off, uint64_t val)
{
	psx_bar_t	*bar0 = &pntb->pntb_bars[0];

	ASSERT(off < bar0->pb_size);
	psx_debug(pntb, EARLY, "put64 0x%x = 0x%016" PRIx64, off, val);
	ddi_put64(bar0->pb_hdl,
	    (uint64_t *)(uintptr_t)(bar0->pb_vaddr + off), val);
}

void
ntb_psx_putpeer64(ntb_psx_t *pntb, uint_t off, uint64_t val)
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
		cmn_err(CE_WARN, "ntb_psx: ntb_psx_putpeer64: write failed (%"
		    PRIu64"/%"PRIu64"", val, rdval);
	}
#endif
}

uint8_t
ntb_psx_get8(ntb_psx_t *pntb, uint_t off)
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
ntb_psx_get16(ntb_psx_t *pntb, uint_t off)
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
ntb_psx_get32(ntb_psx_t *pntb, uint_t off)
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
ntb_psx_get64(ntb_psx_t *pntb, uint_t off)
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
ntb_psx_getpeer32(ntb_psx_t *pntb, uint_t off)
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
ntb_psx_getpeer64(ntb_psx_t *pntb, uint_t off)
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
ntb_psx_claim_window(ntb_psx_t *pntb, psx_bar_t *win, dev_info_t *dip)
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
ntb_psx_release_window(ntb_psx_t *pntb, psx_bar_t *win)
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
ntb_psx_get_bars(ntb_psx_t *pntb)
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

	if (ntb_psx_check_acc_handle(pntb, pntb->pntb_cfg_hdl) != DDI_SUCCESS)
		return (DDI_FAILURE);

	if (pntb->pntb_window[0] == NULL || pntb->pntb_wcnt == 1) {
		psx_warn(pntb, "!There are no appropriate Window BARs "
		    "configured. Aborting initialisation.");
		return (DDI_FAILURE);
	}

	/* Map the BARs we have to a vaddr */
	acc = pntb->acc_attr;
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
		    pntb->acc_attr.devacc_attr_dataorder :
		    DDI_MERGING_OK_ACC;

		if (ddi_regs_map_setup(pntb->pntb_dip, win->pb_rnum,
		    &win->pb_vaddr, 0, 0, &acc, &win->pb_hdl) != DDI_SUCCESS) {
			psx_warn(pntb, "!Failed to map BAR %d", bar);
			ntb_psx_release_bars(pntb);
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
ntb_psx_release_bars(ntb_psx_t *pntb)
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
ntb_psx_check_acc_handle(ntb_psx_t *pntb, ddi_acc_handle_t hdl)
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
ntb_psx_check_gas_acc_handle(ntb_psx_t *pntb)
{
	return (ntb_psx_check_bar_acc_handle(pntb, 0));
}

int
ntb_psx_check_bar_acc_handle(ntb_psx_t *pntb, int bar)
{
	psx_bar_t	*ib = &pntb->pntb_bars[bar];

	return (ntb_psx_check_acc_handle(pntb, ib->pb_hdl));
}

int
ntb_psx_check_win_acc_handle(ntb_psx_t *pntb, psx_bar_t *win)
{
	return (ntb_psx_check_acc_handle(pntb, win->pb_hdl));
}

int
ntb_psx_check_dma_handle(ntb_psx_t *pntb, ddi_dma_handle_t hdl)
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
ntb_psx_fm_err_cb(dev_info_t *dip, ddi_fm_error_t *err, const void *arg)
{
	ntb_psx_t	*pntb = (ntb_psx_t *)arg;

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
ntb_psx_fm_init(ntb_psx_t *pntb)
{
	/*
	 * Keep per device attribute sets.
	 */
	pntb->acc_attr = ntb_psx_acc_attr;
	pntb->dma_attr = pntb_attr_tmpl;

	/*
	 * Get any capabilities, if set, from intb.conf.
	 */
	pntb->fm_cap = ddi_prop_get_int(DDI_DEV_T_ANY, pntb->pntb_dip,
	    DDI_PROP_DONTPASS, "fm-capable", DDI_FM_EREPORT_CAPABLE |
	    DDI_FM_ACCCHK_CAPABLE | DDI_FM_DMACHK_CAPABLE |
	    DDI_FM_ERRCB_CAPABLE);

	if (DDI_FM_DEFAULT_CAP(pntb->fm_cap))
		/* None set. */
		return;

	ddi_fm_init(pntb->pntb_dip, &pntb->fm_cap, &pntb->fm_ibc);

	if (DDI_FM_ACC_ERR_CAP(pntb->fm_cap))
		pntb->acc_attr.devacc_attr_access = DDI_FLAGERR_ACC;

	if (DDI_FM_DMA_ERR_CAP(pntb->fm_cap))
		pntb->dma_attr.dma_attr_flags = DDI_DMA_FLAGERR;

	if (DDI_FM_EREPORT_CAP(pntb->fm_cap) ||
	    DDI_FM_ERRCB_CAP(pntb->fm_cap))
		pci_ereport_setup(pntb->pntb_dip);

	if (DDI_FM_ERRCB_CAP(pntb->fm_cap))
		ddi_fm_handler_register(pntb->pntb_dip, ntb_psx_fm_err_cb, pntb);
}

static void
ntb_psx_fm_fini(ntb_psx_t *pntb)
{
	if (DDI_FM_DEFAULT_CAP(pntb->fm_cap))
		/* None set. */
		return;

	if (DDI_FM_EREPORT_CAP(pntb->fm_cap) ||
	    DDI_FM_ERRCB_CAP(pntb->fm_cap))
		pci_ereport_teardown(pntb->pntb_dip);

	if (DDI_FM_ERRCB_CAP(pntb->fm_cap))
		ddi_fm_handler_unregister(pntb->pntb_dip);

	ddi_fm_fini(pntb->pntb_dip);
}

static int
ntb_psx_fm_init_child(dev_info_t *dip, dev_info_t *cdip, int cap,
    ddi_iblock_cookie_t *ibc_p)
{
	ntb_psx_t	*pntb = ddi_get_driver_private(dip);

	*ibc_p = pntb->fm_ibc;

	return (pntb->fm_cap);
}

/*
 * The requester id mapping should hold entries for the root complex (0.0.0)
 * and the host bridge.
 * "requester-ids" property is an array of additional ids for other device,
 * eg ioat.
 */
static int
ntb_psx_configure_rid_mappings(ntb_psx_t *pntb)
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
	rid_limit = ntb_psx_get16(pntb, pntb->pntb_ctl_off +
	    PNTB_NTB_CNTL_RID_LIMIT);
	peer_rid_limit = ntb_psx_get16(pntb, pntb->pntb_peer_ctl_off +
	    PNTB_NTB_CNTL_RID_LIMIT);

	rid_limit = MIN(rid_limit, peer_rid_limit);

	if (rid_limit == 0) {
		psx_warn(pntb, "!Requester Id limit is zero!");
		return (DDI_FAILURE);
	}

	host_bridge = ntb_psx_get16(pntb, pntb->pntb_ntb_off +
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
	rv = ntb_psx_set_rids(pntb, pntb->pntb_partition, rids, cnt);
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
		proxy_id = ntb_psx_get16(pntb, pntb->pntb_ctl_off +
		    PNTB_NTB_CNTL_RID(i));

		rids[i] = PNTB_NTB_CNTL_RID_PROXY_ID(proxy_id);
	}

	/*
	 * Program the requester ids for the peer partition.
	 */
	rv = ntb_psx_set_rids(pntb, pntb->pntb_peer_partition, rids, cnt);
	if (rv != DDI_SUCCESS)
		psx_warn(pntb, "!Failed to set Request Ids for partition %d",
		    pntb->pntb_peer_partition);

	/*
	 * check if we want to disable the use of rid mappings
	 */
	if (ddi_prop_get_int(DDI_DEV_T_ANY, pntb->pntb_dip,
	    DDI_PROP_DONTPASS, "disable-rid-mapping", 0) == 1)
		ntb_psx_rid_disable(pntb);

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
ntb_psx_init_peer_config_space(ntb_psx_t *pntb)
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
			ntb_psx_put64(pntb, pntb->pntb_peer_cfg_spc_off +
			    PCI_CONF_BASE0 + bar->pb_bar * 4,
			    BAR_BASE64 << i);
		else
			ntb_psx_put32(pntb, pntb->pntb_peer_cfg_spc_off +
			    PCI_CONF_BASE0 + bar->pb_bar * 4,
			    BAR_BASE32 << i);
	}

	/*
	 * Enable memory access and bus mastering on the private PCI bus.
	 */
	ntb_psx_put16(pntb, pntb->pntb_peer_cfg_spc_off + PCI_CONF_COMM,
	    PCI_COMM_MAE | PCI_COMM_ME);

	return (ntb_psx_check_gas_acc_handle(pntb));
}

/*
 * This reads the BAR address out of the config space in the peer partition.
 */
static uint64_t
ntb_psx_get_peer_bar(ntb_psx_t *pntb, int bar)
{
	uint64_t	addr;

	addr = ntb_psx_get32(pntb, pntb->pntb_peer_cfg_spc_off +
	    PCI_CONF_BASE0 + bar * 4);
	if ((addr & PCI_BASE_TYPE_ALL) != 0) {
		/* 64 bit address */
		addr &= PCI_BASE_M_ADDR_M;
		addr |= (uint64_t)ntb_psx_get32(pntb,
		    pntb->pntb_peer_cfg_spc_off + PCI_CONF_BASE0 +
		    bar * 4 + 4) << 32;
	} else {
		addr &= PCI_BASE_M_ADDR_M;
	}

	return (addr);
}

static int
ntb_psx_identify_peer_partition(ntb_psx_t *pntb)
{
	uint64_t	map;
	uint32_t	info_offset = PNTB_NTB_PART_INFO(pntb->pntb_partition);
	int		part;

	/*
	 * Get the partition map out of the NTB information registers
	 * specific to pntb_partition.
	 */
	map = ntb_psx_get32(pntb, pntb->pntb_ntb_off + info_offset +
	    PNTB_NTB_PART_INFO_LOW);
	map |= (uint64_t)ntb_psx_get32(pntb, pntb->pntb_ntb_off + info_offset +
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
		if (ntb_psx_get8(pntb, pntb->pntb_ntb_off + info_offset +
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
	map = ntb_psx_get64(pntb, pntb->pntb_ntb_off + PNTB_NTB_ENDPOINT_MAP);
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
ntb_psx_init_offsets_and_mappings(ntb_psx_t *pntb)
{
	psx_bar_t	*win;
	uint64_t	bar;
	uint64_t	bar_size, seg_size, seg_count;
	uint32_t	vep_inst_id;
	int		i;
	uint16_t	peer_count;

	/* NTB registers. */
	pntb->pntb_ntb_off = PNTB_GAS_NTB_OFFSET;

	pntb->pntb_partition_cnt = ntb_psx_get8(pntb, pntb->pntb_ntb_off +
	    PNTB_NTB_PARTITION_NUMBER);
	pntb->pntb_partition = ntb_psx_get8(pntb, pntb->pntb_ntb_off +
	    PNTB_NTB_PARTITION_ID);

	if (ntb_psx_identify_peer_partition(pntb) != DDI_SUCCESS)
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
	vep_inst_id = ntb_psx_get32(pntb, pntb->pntb_peer_pcfg_off +
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
	if (ntb_psx_init_peer_config_space(pntb) != DDI_SUCCESS) {
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
	win->pb_peer_paddr = ntb_psx_get_peer_bar(pntb, win->pb_bar);

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
	win->pb_seg_count = ntb_psx_get16(pntb, pntb->pntb_ctl_off +
	    PNTB_NTB_CNTL_LUT_LIMIT);
	peer_count = ntb_psx_get16(pntb, pntb->pntb_peer_ctl_off +
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
	if (ntb_psx_setup_lut(pntb, win, pntb->pntb_partition) != DDI_SUCCESS) {
		psx_warn(pntb, "!Failed to set up lut for BAR%d, partition %d",
		    win->pb_bar, pntb->pntb_partition);
		return (DDI_FAILURE);
	}

	if (ntb_psx_setup_lut(pntb, win, pntb->pntb_peer_partition) !=
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
	bar = ntb_psx_get_peer_bar(pntb, 0);
	pntb->pntb_bars[0].pb_peer_paddr = bar;

	/*
	 * Set the translation addresses from entry 0 in the LUT for
	 * BAR0.
	 */
	for (i = 0; i < seg_count; i++) {
		/*
		 * Map from "bar" in pb_seg_size increments into LUT.
		 */
		if (ntb_psx_set_lut_window(pntb, win->pb_lut_base + i,
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
	if (ntb_psx_update_allocations_map(pntb, pntb->pntb_dip, 0, seg_count) !=
	    DDI_SUCCESS) {
		psx_warn(pntb, "!Failed to reserve LUT entries for internal "
		    "mappings");
		goto failed;
	}

	/*
	 * Finally, update the requester id mappings.
	 */
	if (ntb_psx_configure_rid_mappings(pntb) == DDI_SUCCESS)
		return (DDI_SUCCESS);

failed:
	ntb_psx_clear_mappings(pntb);

	return (DDI_FAILURE);
}

/*
 * Clear out all LUT and requester id mappings.
 */
static void
ntb_psx_clear_mappings(ntb_psx_t *pntb)
{
	uint64_t	seg_size, bar_size, seg_count;
	int		i;

	if (pntb->pntb_seg_window == NULL)
		return;

	(void) ntb_psx_clear_lut(pntb, pntb->pntb_seg_window,
	    pntb->pntb_partition);
	(void) ntb_psx_clear_lut(pntb, pntb->pntb_seg_window,
	    pntb->pntb_peer_partition);

	seg_size = pntb->pntb_seg_window->pb_seg_size;
	bar_size = pntb->pntb_bars[0].pb_size + seg_size - 1;
	seg_count = MIN(bar_size / seg_size,
	    pntb->pntb_seg_window->pb_seg_count);

	for (i = 0; i < seg_count; i++) {
		(void) ntb_psx_clear_lut_window(pntb,
		    pntb->pntb_seg_window->pb_lut_base + i,
		    pntb->pntb_peer_partition);
	}

	ntb_psx_release_segments(pntb, 0, seg_count);

	(void) ntb_psx_clear_rids(pntb, pntb->pntb_partition);
	(void) ntb_psx_clear_rids(pntb, pntb->pntb_peer_partition);
}

void
ntb_psx_get_peer_link_info(ntb_psx_t *pntb)
{
	uint16_t	reg;

	reg = ntb_psx_get16(pntb, pntb->pntb_peer_cfg_spc_off +
	    PNTB_NTB_CONFIG_LNK_STS);

	pntb->pntb_speed = PNTB_NTB_CONFIG_LNK_SPEED(reg);
	pntb->pntb_width = PNTB_NTB_CONFIG_LNK_WIDTH(reg);

	psx_note(pntb, "!Link %d x GEN%d", pntb->pntb_width, pntb->pntb_speed);
}

void
ntb_psx_get_sdk_fw_version(ntb_psx_t *pntb, uint32_t *fwver,
    uint32_t *peer_fwver)
{
	*fwver = ntb_psx_get32(pntb, PNTB_SYSINFO_OFFSET +
	    PNTB_SYSINFO_FWVER_OFFSET);
	*peer_fwver = ntb_psx_getpeer32(pntb, PNTB_SYSINFO_OFFSET +
	    PNTB_SYSINFO_FWVER_OFFSET);
}

static int
ntb_psx_attach(dev_info_t *dip, ddi_attach_cmd_t cmd)
{
	ddi_acc_handle_t cfg_hdl;
	ntb_psx_t	*pntb;
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
	sema_init(&pntb->pntb_sema, 1, "ntb_psx_sema", SEMA_DRIVER, NULL);

	ntb_psx_fm_init(pntb);

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

	if (ntb_psx_check_acc_handle(pntb, pntb->pntb_cfg_hdl) != DDI_SUCCESS)
		goto failure;

	if (ntb_psx_get_bars(pntb) != DDI_SUCCESS)
		goto failure;

	/*
	 * Initialize pertinent offsets within the GAS.
	 */
	if (ntb_psx_init_offsets_and_mappings(pntb) != DDI_SUCCESS)
		goto failure;

	/*
	 * Initialize interrupts and setup doorbell and message mappings
	 * between local and peer partition.
	 */
	if (ntb_psx_intr_init(pntb) != DDI_SUCCESS)
		goto failure;

	/*
	 * Start the doorbell processing thread.
	 */
	if (ntb_psx_doorbells_init(pntb) != DDI_SUCCESS)
		goto failure;

	if (ntb_psx_intr_enable(pntb) != DDI_SUCCESS)
		goto failure;

	ntb_psx_get_peer_link_info(pntb);

	/*
	 * Claim the segmented window for this driver. Any segmented mappings
	 * must come though us.
	 */
	own = ntb_psx_claim_window(pntb, pntb->pntb_seg_window, pntb->pntb_dip);
	if (!own) {
		psx_warn(pntb, "!Failed to claim segment window");
		goto failure;
	}

	/*
	 * Initialise private translations to the peers config space and
	 * BAR0.
	 */
	if (ntb_psx_set_pvt_translations(pntb) != DDI_SUCCESS)
		goto failure;

#ifdef DEBUG
	ntb_psx_debug &= ~EARLY;
#endif

	if (ntb_psx_start_pinging(pntb) != DDI_SUCCESS)
		goto failure;

	if (ntb_register(dip, &ntb_psx_ops, pntb) != DDI_SUCCESS) {
		psx_warn(pntb, "!Failed to register with ntb_svc");
		goto failure;
	}

	if (ntb_register_kstat(dip, &pntb->kdata) != DDI_SUCCESS) {
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

	ntb_psx_stop_pinging(pntb);

	ntb_psx_doorbells_fini(pntb);

	if (pntb->pntb_rupt_count > 0) {
		ntb_psx_intr_disable(pntb);
		ntb_psx_intr_fini(pntb);
	}

	ntb_psx_clear_pvt_translations(pntb);

	ntb_psx_clear_mappings(pntb);
	if (own)
		ntb_psx_release_window(pntb, pntb->pntb_seg_window);

	ntb_psx_fm_fini(pntb);
	ntb_psx_release_bars(pntb);
	mutex_destroy(&pntb->pntb_lock);
	sema_destroy(&pntb->pntb_sema);

	if (pntb->pntb_cfg_hdl != NULL)
		pci_config_teardown(&pntb->pntb_cfg_hdl);

	ddi_set_driver_private(pntb->pntb_dip, NULL);
	ddi_soft_state_free(statep, inst);

	return (DDI_FAILURE);
}


static int
ntb_psx_detach(dev_info_t *dip, ddi_detach_cmd_t cmd)
{
	ntb_psx_t	*pntb;

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

	ntb_psx_stop_pinging(pntb);

	/* Let the peer know we are going away */
	(void) ntb_psx_set_peer_doorbell(pntb, pntb->pntb_db_down);

	ntb_psx_doorbells_fini(pntb);

	ntb_psx_clear_pvt_translations(pntb);

	ntb_psx_clear_mappings(pntb);
	ntb_psx_release_window(pntb, pntb->pntb_seg_window);

	ntb_psx_fm_fini(pntb);

	ntb_psx_intr_disable(pntb);
	ntb_psx_intr_fini(pntb);

	ntb_psx_release_bars(pntb);

	mutex_destroy(&pntb->pntb_lock);
	sema_destroy(&pntb->pntb_sema);

	pci_config_teardown(&pntb->pntb_cfg_hdl);
	ddi_set_driver_private(pntb->pntb_dip, NULL);

	ddi_soft_state_free(statep, pntb->pntb_inst);

	return (DDI_SUCCESS);
}

static int
ntb_psx_getinfo(dev_info_t *dip, ddi_info_cmd_t cmd, void *arg, void **result)
{
	dev_t		dev = (dev_t)arg;
	minor_t		inst = getminor(dev);
	ntb_psx_t	*pntb;

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
ntb_psx_quiesce(dev_info_t *dip)
{
	ntb_psx_t	*pntb = ddi_get_driver_private(dip);

	/* Let the peer know we are going away */
	(void) ntb_psx_set_peer_doorbell(pntb, pntb->pntb_db_down);

	return (ntb_psx_quiesce_translation(pntb));
}

static int
ntb_psx_bus_map(dev_info_t *dip, dev_info_t *rdip, ddi_map_req_t *mp,
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
ntb_psx_dma_allochdl(dev_info_t *dip, dev_info_t *rdip, ddi_dma_attr_t *attr,
    int (*waitfp)(caddr_t), caddr_t arg, ddi_dma_handle_t *handlep)
{
	ntb_psx_t		*pntb = ddi_get_driver_private(dip);
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
		if (!ntb_psx_claim_window(pntb, win, rdip)) {
			psx_warn(pntb, "!Window %d is already in use.",
			    window);
			return (DDI_FAILURE);
		}

		/*
		 * Get the address in the BAR on the private PCI bus.
		 */
		win->pb_peer_paddr = ntb_psx_get_peer_bar(pntb, win->pb_bar);

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
			ntb_psx_release_window(pntb, win);

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
ntb_psx_dma_freehdl(dev_info_t *dip, dev_info_t *rdip, ddi_dma_handle_t handle)
{
	ntb_psx_t		*pntb = ddi_get_driver_private(dip);
	ddi_dma_impl_t		*i_hdl = (ddi_dma_impl_t *)handle;
	psx_dma_handle_t	*pntb_h = i_hdl->dmai_driver_private;

	if (pntb_h->pdh_type == WINDOW_HDL)
		ntb_psx_release_window(pntb, pntb_h->pdh_window);

	kmem_free(pntb_h, sizeof (*pntb_h));

	return (ddi_dma_freehdl(dip, rdip, handle));
}

/*
 * The allocation map has 1 bit per segment. When bit 'n' is set then
 * segment 'n' is in use.
 */
static int
ntb_psx_update_allocations_map(ntb_psx_t *pntb, dev_info_t *rdip, int seg,
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
ntb_psx_reserve_segments(ntb_psx_t *pntb, dev_info_t *rdip, int *segp, int *cntp)
{
	psx_bar_t	*win = pntb->pntb_seg_window;
	int		seg;

	if (ntb_get_bind_limits_by_dev(rdip, segp, cntp) != DDI_SUCCESS &&
	    ntb_get_dma_segments_by_dev(rdip, segp, cntp) != DDI_SUCCESS)
		return (DDI_FAILURE);

	seg = win->pb_lut_child - win->pb_lut_base + *segp;

	return (ntb_psx_update_allocations_map(pntb, rdip, seg, *cntp));
}

/*
 * Release a range of segments previously reserved.
 */
static void
ntb_psx_release_segments(ntb_psx_t *pntb, int seg, int cnt)
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
ntb_psx_bind_peer_window(dev_info_t *dip, dev_info_t *rdip,
    psx_dma_handle_t *pntb_h, struct ddi_dma_req *dmareq,
    ddi_dma_cookie_t *cp, uint_t *ccountp)
{
	ntb_psx_t		*pntb = ddi_get_driver_private(dip);
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
	if (ntb_psx_set_direct_window(pntb, win, pntb->pntb_partition,
	    paddr) != DDI_SUCCESS) {
		dev_err(rdip, CE_WARN, "!Failed to set direct window in "
		    "peer partition");
		(void) ddi_dma_unbindhdl(dip, rdip, handle);
		return (DDI_FAILURE);
	}

	return (DDI_SUCCESS);
}

static int
ntb_psx_unbind_peer_window(dev_info_t *dip, dev_info_t *rdip,
    psx_dma_handle_t *pntb_h)
{
	ntb_psx_t	*pntb = ddi_get_driver_private(dip);
	psx_bar_t	*win = pntb_h->pdh_window;

	/*
	 * Remove the translation and close the window.
	 */
	if (ntb_psx_clear_direct_window(pntb, win, pntb->pntb_partition) !=
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
 * the translation as setup in ntb_psx_bind_peer_window() to target a physical
 * addreass in the that controller.
 */
static int
ntb_psx_bind_local_window(dev_info_t *dip, dev_info_t *rdip,
    psx_dma_handle_t *pntb_h, struct ddi_dma_req *dmareq,
    ddi_dma_cookie_t *cp, uint_t *ccountp)
{
	ntb_psx_t		*pntb = ddi_get_driver_private(dip);
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
	if (ntb_psx_set_direct_window(pntb, win, pntb->pntb_peer_partition,
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
ntb_psx_unbind_local_window(dev_info_t *dip, dev_info_t *rdip,
    psx_dma_handle_t *pntb_h)
{
	ntb_psx_t		*pntb = ddi_get_driver_private(dip);
	ddi_dma_handle_t	handle = pntb_h->pdh_handle;
	psx_bar_t		*win = pntb_h->pdh_window;

	if (ntb_psx_clear_direct_window(pntb, win, pntb->pntb_peer_partition) !=
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
ntb_psx_bind_peer_segments(dev_info_t *dip, dev_info_t *rdip,
    psx_dma_handle_t *pntb_h, struct ddi_dma_req *dmareq,
    ddi_dma_cookie_t *cp, uint_t *ccountp)
{
	ntb_psx_t		*pntb = ddi_get_driver_private(dip);
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
	if (ntb_psx_reserve_segments(pntb, rdip, &seg, &cnt) != DDI_SUCCESS) {
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

	if (ntb_psx_set_lut_peer(pntb, pntb_h) != DDI_FAILURE)
		return (DDI_SUCCESS);

	/* Failed. */
	kmem_free(pntb_h->pdh_cookies, sizeof (ddi_dma_cookie_t) *
	    pntb_h->pdh_cookie_cnt);

	(void) ddi_dma_unbindhdl(dip, rdip, handle);

release:
	ntb_psx_release_segments(pntb, seg, cnt);

	pntb_h->pdh_seg = 0;
	pntb_h->pdh_seg_cnt = 0;

	return (DDI_FAILURE);
}

static int
ntb_psx_unbind_peer_segments(dev_info_t *dip, dev_info_t *rdip,
    psx_dma_handle_t *pntb_h)
{
	ntb_psx_t		*pntb = ddi_get_driver_private(dip);
	ddi_dma_handle_t	handle = pntb_h->pdh_handle;
	int			rv;

	if (ntb_psx_clear_lut_peer(pntb, pntb_h) != DDI_SUCCESS) {
		dev_err(rdip, CE_WARN, "!Failed to clear peer LUT");
		return (DDI_FAILURE);
	}

	kmem_free(pntb_h->pdh_cookies, sizeof (ddi_dma_cookie_t) *
	    pntb_h->pdh_cookie_cnt);

	pntb_h->pdh_cookies = NULL;
	pntb_h->pdh_cookie_cnt = 0;

	rv = ddi_dma_unbindhdl(dip, rdip, handle);

	ntb_psx_release_segments(pntb, pntb_h->pdh_seg, pntb_h->pdh_seg_cnt);

	pntb_h->pdh_seg = 0;
	pntb_h->pdh_seg_cnt = 0;

	return (rv);
}

static int
ntb_psx_bind_local_segments(dev_info_t *dip, dev_info_t *rdip,
    psx_dma_handle_t *pntb_h, struct ddi_dma_req *dmareq,
    ddi_dma_cookie_t *cp, uint_t *ccountp)
{
	ntb_psx_t		*pntb = ddi_get_driver_private(dip);
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

	if (ntb_psx_set_lut_local(pntb, pntb_h) != DDI_SUCCESS) {
		(void) ddi_dma_unbindhdl(dip, rdip, handle);
		return (DDI_FAILURE);
	}

	pntb_h->pdh_dma_addr = dmareq->dmar_object.dmao_obj.virt_obj.v_addr;
	pntb_h->pdh_dma_len = dmareq->dmar_object.dmao_size;

	return (DDI_SUCCESS);
}

static int
ntb_psx_unbind_local_segments(dev_info_t *dip, dev_info_t *rdip,
    psx_dma_handle_t *pntb_h)
{
	ntb_psx_t		*pntb = ddi_get_driver_private(dip);
	ddi_dma_handle_t	handle = pntb_h->pdh_handle;

	if (ntb_psx_clear_lut_local(pntb, pntb_h) != DDI_SUCCESS) {
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
ntb_psx_dma_bindhdl(dev_info_t *dip, dev_info_t *rdip,
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
		rv = (peer_bind ? ntb_psx_bind_peer_segments :
		    ntb_psx_bind_local_segments)(dip, rdip, pntb_h, dmareq,
		    cp, ccountp);
		break;

	case WINDOW_HDL:
		rv = (peer_bind ? ntb_psx_bind_peer_window :
		    ntb_psx_bind_local_window)(dip, rdip, pntb_h, dmareq,
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
ntb_psx_dma_unbindhdl(dev_info_t *dip, dev_info_t *rdip, ddi_dma_handle_t handle)
{
	ddi_dma_impl_t		*i_hdl = (ddi_dma_impl_t *)handle;
	ddi_dma_attr_t		*dma_attr = &i_hdl->dmai_attr;
	psx_dma_handle_t	*pntb_h = i_hdl->dmai_driver_private;
	boolean_t		peer_bind;
	int			rv;

	peer_bind = (dma_attr->dma_attr_flags & PNTB_DMA_PEER) != 0;

	switch (pntb_h->pdh_type) {
	case SEGMENT_HDL:
		rv = (peer_bind ? ntb_psx_unbind_peer_segments :
		    ntb_psx_unbind_local_segments)(dip, rdip, pntb_h);
		break;

	case WINDOW_HDL:
		rv = (peer_bind ? ntb_psx_unbind_peer_window :
		    ntb_psx_unbind_local_window)(dip, rdip, pntb_h);
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
ntb_psx_ctl_initchild(dev_info_t *child)
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
ntb_psx_ctlops(dev_info_t *dip, dev_info_t *rdip, ddi_ctl_enum_t ctlop,
    void *arg, void *result)
{
	ntb_psx_t	*pntb = ddi_get_driver_private(dip);

	switch (ctlop) {
	case DDI_CTLOPS_REPORTDEV:
		if (rdip == NULL)
			return (DDI_FAILURE);

		psx_cont(pntb, "?Child: %s@%s, %s%d\n",
		    ddi_node_name(rdip), ddi_get_name_addr(rdip),
		    ddi_driver_name(rdip), ddi_get_instance(rdip));

		return (DDI_SUCCESS);

	case DDI_CTLOPS_INITCHILD:
		return (ntb_psx_ctl_initchild(arg));

	case DDI_CTLOPS_UNINITCHILD:
		ddi_set_name_addr(arg, NULL);
		return (DDI_SUCCESS);

	case DDI_CTLOPS_POWER:
		return (DDI_SUCCESS);

	default:
		return (ddi_ctlops(dip, rdip, ctlop, arg, result));
	}
}

static struct bus_ops ntb_psx_bus_ops = {
	.busops_rev = BUSO_REV,
	.bus_map = ntb_psx_bus_map,
	.bus_map_fault = i_ddi_map_fault,
	.bus_dma_allochdl = ntb_psx_dma_allochdl,
	.bus_dma_freehdl = ntb_psx_dma_freehdl,
	.bus_dma_bindhdl = ntb_psx_dma_bindhdl,
	.bus_dma_unbindhdl = ntb_psx_dma_unbindhdl,
	.bus_dma_flush = ddi_dma_flush,
	.bus_dma_win = ddi_dma_win,
	.bus_dma_ctl = ddi_dma_mctl,
	.bus_ctl = ntb_psx_ctlops,
	.bus_prop_op = ddi_bus_prop_op,
	.bus_get_eventcookie = ndi_busop_get_eventcookie,
	.bus_add_eventcall = ndi_busop_add_eventcall,
	.bus_remove_eventcall = ndi_busop_remove_eventcall,
	.bus_post_event = ndi_post_event,
	.bus_fm_init = ntb_psx_fm_init_child,
	.bus_intr_op = i_ddi_intr_ops
};


static struct dev_ops ntb_psx_dev_ops = {
	DEVO_REV,		/* devo_rev */
	0,			/* devo_refcnt */
	ntb_psx_getinfo,		/* devo_getinfo */
	nulldev,		/* devo_identify */
	nulldev,		/* devo_probe */
	ntb_psx_attach,		/* devo_attach */
	ntb_psx_detach,		/* devo_detach */
	nodev,			/* devo_reset */
	NULL,			/* devo_cb_ops */
	&ntb_psx_bus_ops,	/* devo_bus_ops */
	ddi_power,		/* devo_power */
	ntb_psx_quiesce		/* devo_quiesce */
};

static struct modldrv ntb_psx_modldrv = {
	&mod_driverops,
	PSXNTB_VERSION,
	&ntb_psx_dev_ops
};

static struct modlinkage ntb_psx_modlinkage = {
	MODREV_1,
	(void *)&ntb_psx_modldrv,
	NULL
};

int
_info(struct modinfo *modinfop)
{
	return (mod_info(&ntb_psx_modlinkage, modinfop));
}

int
_init(void)
{
	int	rv;

	rv = ddi_soft_state_init(&statep, sizeof (ntb_psx_t), 1);
	if (rv == 0)
		rv = mod_install(&ntb_psx_modlinkage);

	return (rv);
}

int
_fini(void)
{
	int	rv;

	rv = mod_remove(&ntb_psx_modlinkage);
	if (rv == 0)
		ddi_soft_state_fini(&statep);

	return (rv);
}

static void
ntb_psx_free_pvt_dma(ntb_psx_t *pntb)
{
	if (pntb->pvt_acc_handle != NULL)
		ddi_dma_mem_free(&pntb->pvt_acc_handle);

	if (pntb->pvt_paddr != (uintptr_t)NULL)
		(void) ddi_dma_unbind_handle(pntb->pvt_handle);

	if (pntb->pvt_handle != NULL)
		ddi_dma_free_handle(&pntb->pvt_handle);

	pntb->pvt_handle = NULL;
	pntb->pvt_acc_handle = NULL;
	pntb->pvt_vaddr = NULL;
	pntb->pvt_paddr = 0;
}

/*
 * Allocate a page of DMA to exchange private information across the
 * interconnect. Currently these are only alive counters.
 */
static int
ntb_psx_allocate_pvt_dma(ntb_psx_t *pntb)
{
	ddi_dma_attr_t		attr;
	ddi_dma_cookie_t	cookie;
	uint_t			cookie_cnt;
	size_t			len;

	attr = pntb->dma_attr;
	attr.dma_attr_seg = pntb->pntb_seg_window->pb_seg_size - 1;
	attr.dma_attr_align = pntb->pntb_seg_window->pb_seg_size;
	attr.dma_attr_sgllen = 1;
	attr.dma_attr_minxfer = 1U << PNTB_MIN_SIZE;

	psx_debug(pntb, EARLY, "!attr.dma_attr_seg=0x%" PRIx64,
	    pntb->pntb_seg_window->pb_seg_size);
	if (ddi_dma_alloc_handle(pntb->pntb_dip, &attr, DDI_DMA_SLEEP, NULL,
	    &pntb->pvt_handle) != DDI_SUCCESS) {
		psx_warn(pntb, "!Failed ddi_dma_alloc_handle");
		return (DDI_FAILURE);
	}
	if (ddi_dma_mem_alloc(pntb->pvt_handle, PAGESIZE,
	    &pntb->acc_attr, DDI_DMA_STREAMING, DDI_DMA_SLEEP, NULL,
	    &pntb->pvt_vaddr, &len, &pntb->pvt_acc_handle) !=
	    DDI_SUCCESS) {
		psx_warn(pntb, "!Failed ddi_dma_mem_alloc");
		goto failed;
	}

	if (ddi_dma_addr_bind_handle(pntb->pvt_handle, NULL,
	    pntb->pvt_vaddr, len, DDI_DMA_STREAMING,
	    DDI_DMA_SLEEP, NULL, &cookie, &cookie_cnt) != DDI_DMA_MAPPED) {
		psx_warn(pntb, "!Failed ddi_dma_addr_bind_handle");
		goto failed;
	}

	pntb->pvt_paddr = cookie.dmac_laddress;
	pntb->pvt_size = len;

	return (DDI_SUCCESS);

failed:
	ntb_psx_free_pvt_dma(pntb);

	return (DDI_FAILURE);
}

static int
ntb_psx_set_pvt_translations(ntb_psx_t *pntb)
{
	psx_bar_t	*win = pntb->pntb_seg_window;

	if (ntb_psx_allocate_pvt_dma(pntb) != DDI_SUCCESS) {
		psx_warn(pntb, "!Failed to allocate private DMA");
		return (DDI_FAILURE);
	}

	/*
	 * Fake up a psx handle.
	 */
	pntb->psx_handle.pdh_seg = 0;
	pntb->psx_handle.pdh_seg_cnt = 1;
	pntb->psx_handle.pdh_seg_size = win->pb_seg_size;
	pntb->psx_handle.pdh_dma_addr = pntb->pvt_vaddr;
	pntb->psx_handle.pdh_dma_len = pntb->pvt_size;

	pntb->psx_handle.pdh_cookies = kmem_alloc(
	    sizeof (ddi_dma_cookie_t), KM_SLEEP);
	pntb->psx_handle.pdh_cookie_cnt = 1;
	pntb->psx_handle.pdh_cookies[0].dmac_laddress =
	    pntb->pvt_paddr;
	pntb->psx_handle.pdh_cookies[0].dmac_size = pntb->pvt_size;
	pntb->psx_handle.pdh_cookies[0].dmac_type = 0;

	pntb->psx_handle.pdh_window = win;

	if (ntb_psx_set_lut_local(pntb, &pntb->psx_handle) != DDI_SUCCESS) {
		psx_warn(pntb, "!Failed to set local lut for private use");
		goto failed;
	}

	if (ntb_psx_set_lut_peer(pntb, &pntb->psx_handle) != DDI_SUCCESS) {
		psx_warn(pntb, "!Failed to set peer lut for private use");
		goto failed;
	}

	if (ntb_psx_update_allocations_map(pntb, pntb->pntb_dip,
	    win->pb_lut_child, 1) != DDI_SUCCESS) {
		psx_warn(pntb, "!Failed to reserve private segment");
		goto failed;
	}

	pntb->pvt_offset = win->pb_seg_size *
	    (win->pb_lut_child - win->pb_lut_base);

	pntb->pvt_xaddr = win->pb_vaddr + pntb->pvt_offset;

	win->pb_lut_child++;

	return (DDI_SUCCESS);

failed:
	(void) ntb_psx_clear_lut_peer(pntb, &pntb->psx_handle);
	(void) ntb_psx_clear_lut_local(pntb, &pntb->psx_handle);

	ntb_psx_free_pvt_dma(pntb);

	kmem_free(pntb->psx_handle.pdh_cookies, sizeof (ddi_dma_cookie_t));

	return (DDI_FAILURE);
}

static void
ntb_psx_clear_pvt_translations(ntb_psx_t *pntb)
{
	if (pntb->pvt_vaddr == NULL)
		return;

	pntb->pntb_seg_window->pb_lut_child--;

	ntb_psx_release_segments(pntb, 0, 1);

	(void) ntb_psx_clear_lut_peer(pntb, &pntb->psx_handle);
	(void) ntb_psx_clear_lut_local(pntb, &pntb->psx_handle);

	ntb_psx_free_pvt_dma(pntb);

	if (pntb->psx_handle.pdh_cookies != NULL) {
		kmem_free(pntb->psx_handle.pdh_cookies,
		    sizeof (ddi_dma_cookie_t));
		pntb->psx_handle.pdh_cookies = NULL;
	}
}

/*
 * Move the operation control state to "state" with operation "opc".
 */
static int
ntb_psx_config_change_state(ntb_psx_t *pntb, uint_t ctl_off, uint16_t opc,
    uint16_t state)
{
	uint16_t	new_state;
	int		retries = 0;

	ntb_psx_put16(pntb, ctl_off + PNTB_NTB_CNTL_OPC, opc);
	do {
		/* use busy wait so we can be called during quiesce(). */
		drv_usecwait(10000);	/* 10ms */

		new_state = ntb_psx_get16(pntb, ctl_off +
		    PNTB_NTB_CNTL_STATUS);
	} while (new_state != state && ++retries < 50);

	if (new_state == state)
		return (DDI_SUCCESS);

	/*
	 * Timed out. Reset the OPC which will set the state back to
	 * "normal".
	 */
	ntb_psx_put16(pntb, ctl_off + PNTB_NTB_CNTL_OPC,
	    PNTB_NTB_CNTL_OPC_RESET);

	return (DDI_FAILURE);
}

/*
 * To configure the address look up tables or the requester id tables
 * the following steps are needed:
 * 1. Lock the partition.
 * 2. Perform the configuration operation.
 * 3. Configure the partition, and it transitions to unlocked.
 *
 * These set of states are described in Device Specification manual,
 * Section 17.9.2 under the tile "NT Partition Operation Register".
 */
static int
ntb_psx_config_update_nolock(ntb_psx_t *pntb, uint_t ctl_off,
    int (*cfg_fn)(ntb_psx_t *, void *), void *cfg_arg)
{
	int		rv;
	uint16_t	sts;

	/*
	 * Make sure the partition is in the expected state - "normal".
	 */
	sts = ntb_psx_get16(pntb, ctl_off + PNTB_NTB_CNTL_STATUS);
	if (sts != PNTB_NTB_CNTl_STATUS_NORMAL) {
		psx_warn(pntb, "!Partition in unexpected state: 0x%x", sts);
		return (DDI_FAILURE);
	}

	/*
	 * "lock" the config so we can operate on it.
	 * It must return as "locked".
	 */
	if (ntb_psx_config_change_state(pntb, ctl_off, PNTB_NTB_CNTL_OPC_LOCK,
	    PNTB_NTB_CNTl_STATUS_LOCKED) != DDI_SUCCESS) {
		psx_warn(pntb, "!Failed to lock partition");
		return (DDI_FAILURE);
	}

	/*
	 * Our operation ....
	 */
	rv = cfg_fn(pntb, cfg_arg);

	/*
	 * Perform the "configure" and the state should transition back
	 * to "normal".
	 */
	if (ntb_psx_config_change_state(pntb, ctl_off,
	    PNTB_NTB_CNTL_OPC_CONFIG, PNTB_NTB_CNTl_STATUS_NORMAL) !=
	    DDI_SUCCESS) {
		psx_warn(pntb, "!Failed to update partition configuration");
		return (DDI_FAILURE);
	}

	return (rv);
}

static int
ntb_psx_config_update(ntb_psx_t *pntb, uint_t ctl_off,
    int (*cfg_fn)(ntb_psx_t *, void *), void *cfg_arg)
{
	int	rv;

	/*
	 * This should be the only place where we try and lock a partition.
	 * So, if we serialise access through this function we should
	 * never encounter a locked partition. If we do so it is an
	 * error.
	 */
	sema_p(&pntb->pntb_sema);

	rv = ntb_psx_config_update_nolock(pntb, ctl_off, cfg_fn, cfg_arg);

	if (rv == DDI_SUCCESS)
		rv = ntb_psx_check_gas_acc_handle(pntb);

	sema_v(&pntb->pntb_sema);

	return (rv);
}

typedef struct {
	psx_bar_t 	*win;
	int		target;
	uint64_t	addr;
} set_direct_args_t;

/*
 * Set up a window (BAR) for direct address translation (ie no LUT).
 */
static int
ntb_psx_set_direct_window_cb(ntb_psx_t *pntb, void *arg)
{
	set_direct_args_t *args = arg;
	int		bar = args->win->pb_bar;
	ulong_t		size = args->win->pb_size;
	uint_t		ctl_off, bar_off;
	uint32_t	reg;
	int		i;

	ctl_off = args->target == pntb->pntb_partition ?
	    pntb->pntb_peer_ctl_off : pntb->pntb_ctl_off;

	bar_off = PNTB_NTB_CNTL_BAR_SETUP(bar);

	reg = ntb_psx_get32(pntb, ctl_off + bar_off);
	if ((reg & PNTB_NTB_CNTL_BAR_VALID) == 0)
		return (DDI_FAILURE);

	if (args->addr == -1) {
		/*
		 * Clear the settings for the window.
		 */
		for (i = 0; i < 16; i += 4)
			ntb_psx_put32(pntb, ctl_off + bar_off + i, 0);

		return (DDI_SUCCESS);
	}

	reg &= ~PNTB_NTB_CNTL_BAR_LUT;
	reg |= PNTB_NTB_CNTL_BAR_DIRECT;
	ntb_psx_put32(pntb, ctl_off + bar_off, reg);

	reg = PNTB_NTB_CNTL_BAR_DIR_POS(highbit(size) - 1) |
	    PNTB_NTB_CNTL_BAR_DIR_WIN_SIZE(size);
	ntb_psx_put32(pntb, ctl_off + bar_off + 4, reg);

	reg = PNTB_NTB_CNTL_BAR_DIR_TPART(args->target) |
	    PNTB_NTB_CNTL_BAR_DIR_ADDRL(args->addr);
	ntb_psx_put32(pntb, ctl_off + bar_off + 8, reg);

	reg = PNTB_NTB_CNTL_BAR_DIR_ADDRH(args->addr);
	ntb_psx_put32(pntb, ctl_off + bar_off + 12, reg);

	reg = PNTB_NTB_CNTL_BAR_EXT_WIN_SIZE(size);
	ntb_psx_put32(pntb, ctl_off + PNTB_NTB_CNTL_BAR_EXT(bar), reg);

	return (DDI_SUCCESS);
}

int
ntb_psx_set_direct_window(ntb_psx_t *pntb, psx_bar_t *win, int target,
    uint64_t addr)
{
	uint_t			ctl_off;
	set_direct_args_t	args;

	ctl_off = target == pntb->pntb_partition ?
	    pntb->pntb_peer_ctl_off : pntb->pntb_ctl_off;

	args.target = target;
	args.win = win;
	args.addr = addr;

	return (ntb_psx_config_update(pntb, ctl_off, ntb_psx_set_direct_window_cb,
	    &args));
}

int
ntb_psx_clear_direct_window(ntb_psx_t *pntb, psx_bar_t *win, int target)
{
	return (ntb_psx_set_direct_window(pntb, win, target, -1));
}

typedef struct {
	int 		index;
	int		target;
	uint64_t	addr;
} set_lut_args_t;

/*
 * Configure an address translation entry in the LUT.
 */
static int
ntb_psx_set_lut_window_cb(ntb_psx_t *pntb, void *arg)
{
	set_lut_args_t	*args = arg;
	uint_t		ctl_off, lut_off;
	uint32_t	reg;

	ctl_off = args->target == pntb->pntb_partition ?
	    pntb->pntb_peer_ctl_off : pntb->pntb_ctl_off;

	lut_off = PNTB_NTB_CNTL_LUT(args->index);

	if (args->addr == -1) {
		/* Clear the LUT entry. */
		ntb_psx_put32(pntb, ctl_off + lut_off, 0);
		ntb_psx_put32(pntb, ctl_off + lut_off + 4, 0);

		return (DDI_SUCCESS);
	}

	reg = PNTB_NTB_CNTL_LUT_ENABLE | PNTB_NTB_CNTL_LUT_DPART(args->target) |
	    PNTB_NTB_CNTL_LUT_ADDRL(args->addr);
	ntb_psx_put32(pntb, ctl_off + lut_off, reg);

	reg = PNTB_NTB_CNTL_LUT_ADDRH(args->addr);
	ntb_psx_put32(pntb, ctl_off + lut_off + 4, reg);

	return (DDI_SUCCESS);
}

int
ntb_psx_set_lut_window(ntb_psx_t *pntb, int index, int target, uint64_t addr)
{
	uint_t		ctl_off;
	set_lut_args_t	args;

	ctl_off = target == pntb->pntb_partition ?
	    pntb->pntb_peer_ctl_off : pntb->pntb_ctl_off;

	args.target = target;
	args.index = index;
	args.addr = addr;

	return (ntb_psx_config_update(pntb, ctl_off, ntb_psx_set_lut_window_cb,
	    &args));
}

int
ntb_psx_clear_lut_window(ntb_psx_t *pntb, int index, int target)
{
	return (ntb_psx_set_lut_window(pntb, index, target, -1));
}

/*
 * Program a range of LUT entries in the peer partition with the set
 * of addresses in the ddi_dma_cookie array.
 * This is for translation from the private PCI bus to local memory.
 */
static int
ntb_psx_set_lut_peer_cb(ntb_psx_t *pntb, void *arg)
{
	psx_dma_handle_t	*pntb_h = arg;
	psx_bar_t		*win = pntb->pntb_seg_window;
	ddi_dma_cookie_t	*cp;
	uint_t			lut_off;
	uint32_t		reg;
	int			cnt;

	for (cnt = 0; cnt < pntb_h->pdh_lut_cookie_cnt; cnt++) {
		cp = &pntb_h->pdh_lut_cookies[cnt];

		lut_off = PNTB_NTB_CNTL_LUT(win->pb_lut_child +
		    pntb_h->pdh_seg + cnt);

		reg = PNTB_NTB_CNTL_LUT_ENABLE |
		    PNTB_NTB_CNTL_LUT_DPART(pntb->pntb_partition) |
		    PNTB_NTB_CNTL_LUT_ADDRL(cp->dmac_laddress);
		ntb_psx_put32(pntb, pntb->pntb_peer_ctl_off + lut_off, reg);

		reg = PNTB_NTB_CNTL_LUT_ADDRH(cp->dmac_laddress);
		ntb_psx_put32(pntb, pntb->pntb_peer_ctl_off + lut_off + 4, reg);
	}

	return (DDI_SUCCESS);
}

/*
 * Set the LUT entries for the segments described in the psx_dma_handle.
 */
int
ntb_psx_set_lut_peer(ntb_psx_t *pntb, psx_dma_handle_t *pntb_h)
{
	ddi_dma_impl_t	*i_hdl = (ddi_dma_impl_t *)pntb_h->pdh_handle;
	dev_info_t	*dip;
	ddi_dma_cookie_t *cp;
	size_t		entries;
	uint64_t	addr;
	uint64_t	final_size;
	int		alut, cnt, i;

	/*
	 * For an internal handle, the real handle will be NULL.
	 */
	dip = i_hdl == NULL ? pntb->pntb_dip : i_hdl->dmai_rdip;

	/*
	 * pdh_cookies are the cookies returned from the bind.
	 * They should all have the correct alignment, but it is
	 * possible a single cookie could span multiple LUT entries.
	 * Convert the bind cookies into lut size cookies.
	 */
	ASSERT(pntb_h->pdh_lut_cookies == NULL);
	pntb_h->pdh_lut_cookies = kmem_alloc(sizeof (ddi_dma_cookie_t) *
	    pntb_h->pdh_seg_cnt, KM_SLEEP);

	alut = 0;
	cp = &pntb_h->pdh_lut_cookies[0];

	for (cnt = 0; cnt < pntb_h->pdh_cookie_cnt; cnt++) {
		entries = (pntb_h->pdh_cookies[cnt].dmac_size +
		    pntb_h->pdh_seg_size - 1) / pntb_h->pdh_seg_size;

		final_size = pntb_h->pdh_cookies[cnt].dmac_size %
		    pntb_h->pdh_seg_size;

		addr = pntb_h->pdh_cookies[cnt].dmac_laddress;
		for (i = 0; i < entries; i++) {
			if (alut == pntb_h->pdh_seg_cnt) {
				dev_err(dip, CE_WARN, "!Too many cookies");
				goto failed;
			}

			cp->dmac_laddress = addr + i * pntb_h->pdh_seg_size;
			cp->dmac_size = pntb_h->pdh_seg_size;
			cp->dmac_type = 0;

			cp++;
			alut++;
		}

		if (final_size > 0)
			(--cp)->dmac_size = final_size;
	}

	if (alut != pntb_h->pdh_seg_cnt) {
		dev_err(dip, CE_WARN, "!Not enought cookies to span segments");
		goto failed;
	}

	pntb_h->pdh_lut_cookie_cnt = alut;

	if (ntb_psx_config_update(pntb, pntb->pntb_peer_ctl_off,
	    ntb_psx_set_lut_peer_cb, pntb_h) == DDI_SUCCESS)
		return (DDI_SUCCESS);

failed:
	kmem_free(pntb_h->pdh_lut_cookies, sizeof (ddi_dma_cookie_t) *
	    pntb_h->pdh_seg_cnt);
	pntb_h->pdh_lut_cookies = NULL;

	return (DDI_FAILURE);
}

static int
ntb_psx_clear_lut_range(ntb_psx_t *pntb, uint_t ctl_off, int seg, int cnt)
{
	psx_bar_t	*win = pntb->pntb_seg_window;
	uint_t		lut_off;
	int		i;

	for (i = 0; i < cnt; i++) {
		lut_off = PNTB_NTB_CNTL_LUT(win->pb_lut_child + seg + i);
		ntb_psx_put32(pntb, ctl_off + lut_off, 0);
		ntb_psx_put32(pntb, ctl_off + lut_off + 4, 0);
	}

	return (DDI_SUCCESS);
}

static int
ntb_psx_clear_lut_peer_cb(ntb_psx_t *pntb, void *arg)
{
	psx_dma_handle_t	*pntb_h = arg;

	return (ntb_psx_clear_lut_range(pntb, pntb->pntb_peer_ctl_off,
	    pntb_h->pdh_seg, pntb_h->pdh_lut_cookie_cnt));
}

int
ntb_psx_clear_lut_peer(ntb_psx_t *pntb, psx_dma_handle_t *pntb_h)
{
	ASSERT(pntb_h->pdh_lut_cookies != NULL);

	if (ntb_psx_config_update(pntb, pntb->pntb_peer_ctl_off,
	    ntb_psx_clear_lut_peer_cb, pntb_h) != DDI_SUCCESS)
		return (DDI_FAILURE);

	kmem_free(pntb_h->pdh_lut_cookies, sizeof (ddi_dma_cookie_t) *
	    pntb_h->pdh_lut_cookie_cnt);

	pntb_h->pdh_lut_cookies = NULL;
	pntb_h->pdh_lut_cookie_cnt = 0;

	return (DDI_SUCCESS);
}

/*
 * Configure a range of segments in local LUT to translate addresses to
 * go on the private PCI bus.
 */
static int
ntb_psx_set_lut_local_cb(ntb_psx_t *pntb, void *arg)
{
	psx_dma_handle_t	*pntb_h = arg;
	psx_bar_t		*win = pntb->pntb_seg_window;
	uint64_t		addr;
	uint_t			lut_off;
	uint32_t		reg;
	int			i, cnt;

	for (cnt = 0; cnt < pntb_h->pdh_seg_cnt; cnt++) {
		i = win->pb_lut_child + pntb_h->pdh_seg + cnt;

		lut_off = PNTB_NTB_CNTL_LUT(i);

		addr = win->pb_peer_paddr + i * win->pb_seg_size;

		reg = PNTB_NTB_CNTL_LUT_ENABLE |
		    PNTB_NTB_CNTL_LUT_DPART(pntb->pntb_peer_partition) |
		    PNTB_NTB_CNTL_LUT_ADDRL(addr);
		ntb_psx_put32(pntb, pntb->pntb_ctl_off + lut_off, reg);

		reg = PNTB_NTB_CNTL_LUT_ADDRH(addr);
		ntb_psx_put32(pntb, pntb->pntb_ctl_off + lut_off + 4, reg);
	}

	return (DDI_SUCCESS);
}

int
ntb_psx_set_lut_local(ntb_psx_t *pntb, psx_dma_handle_t *pntb_h)
{
	return (ntb_psx_config_update(pntb, pntb->pntb_ctl_off,
	    ntb_psx_set_lut_local_cb, pntb_h));
}

static int
ntb_psx_clear_lut_local_cb(ntb_psx_t *pntb, void *arg)
{
	psx_dma_handle_t	*pntb_h = arg;

	return (ntb_psx_clear_lut_range(pntb, pntb->pntb_ctl_off,
	    pntb_h->pdh_seg, pntb_h->pdh_seg_cnt));
}

int
ntb_psx_clear_lut_local(ntb_psx_t *pntb, psx_dma_handle_t *pntb_h)
{
	return (ntb_psx_config_update(pntb, pntb->pntb_ctl_off,
	    ntb_psx_clear_lut_local_cb, pntb_h));
}

typedef struct {
	psx_bar_t	*win;
	int		partition;
} ctl_lut_args_t;

/*
 * Setup a window (BAR) for LUT, rather than direct address translation.
 */
static int
ntb_psx_setup_lut_cb(ntb_psx_t *pntb, void *arg)
{
	ctl_lut_args_t	*args = arg;
	int		bar = args->win->pb_bar;
	int		base = args->win->pb_lut_base;
	int		entries = args->win->pb_seg_count;
	ulong_t		seg_size = args->win->pb_seg_size;
	uint_t		ctl_off, bar_off;
	uint32_t	reg;
	int		i;

	ctl_off = args->partition == pntb->pntb_partition ?
	    pntb->pntb_ctl_off : pntb->pntb_peer_ctl_off;

	bar_off = PNTB_NTB_CNTL_BAR_SETUP(bar);

	reg = ntb_psx_get32(pntb, ctl_off + bar_off);
	if ((reg & PNTB_NTB_CNTL_BAR_VALID) == 0)
		return (DDI_FAILURE);

	if (entries == 0) {
		reg = 0;
	} else {
		reg &= ~PNTB_NTB_CNTL_BAR_DIRECT;
		reg |= PNTB_NTB_CNTL_BAR_LUT |
		    PNTB_NTB_CNTL_BAR_LUT_POS(highbit(seg_size) - 1) |
		    PNTB_NTB_CNTL_BAR_LUT_SUBWIN(entries - 1) |
		    PNTB_NTB_CNTL_BAR_LUT_SUBWIN_BASE(base);
	}

	ntb_psx_put32(pntb, ctl_off + bar_off, reg);
	for (i = 4; i < 16; i += 4)
		ntb_psx_put32(pntb, ctl_off + bar_off + i, 0);

	return (DDI_SUCCESS);
}

int
ntb_psx_setup_lut(ntb_psx_t *pntb, psx_bar_t *win, int partition)
{
	ctl_lut_args_t	args;
	uint_t		ctl_off;

	ctl_off = partition == pntb->pntb_partition ?
	    pntb->pntb_ctl_off : pntb->pntb_peer_ctl_off;

	args.win = win;
	args.partition = partition;

	return (ntb_psx_config_update(pntb, ctl_off, ntb_psx_setup_lut_cb,
	    &args));
}

static int
ntb_psx_clear_lut_cb(ntb_psx_t *pntb, void *arg)
{
	ctl_lut_args_t	*args = arg;
	int		bar = args->win->pb_bar;
	uint_t		ctl_off, bar_off;
	int		i;

	ctl_off = args->partition == pntb->pntb_partition ?
	    pntb->pntb_ctl_off : pntb->pntb_peer_ctl_off;

	bar_off = PNTB_NTB_CNTL_BAR_SETUP(bar);

	for (i = 0; i < 16; i += 4)
		ntb_psx_put32(pntb, ctl_off + bar_off + i, 0);

	return (DDI_SUCCESS);
}

int
ntb_psx_clear_lut(ntb_psx_t *pntb, psx_bar_t *win, int partition)
{
	ctl_lut_args_t	args;
	uint_t		ctl_off;

	ctl_off = partition == pntb->pntb_partition ?
	    pntb->pntb_ctl_off : pntb->pntb_peer_ctl_off;

	args.win = win;
	args.partition = partition;

	return (ntb_psx_config_update(pntb, ctl_off, ntb_psx_clear_lut_cb,
	    &args));
}

typedef struct {
	uint_t		partition;
	uint16_t	*rids;
	uint_t		cnt;
} set_rids_args_t;

/*
 * Program the allowed requester Ids.
 */
static int
ntb_psx_set_rids_cb(ntb_psx_t *pntb, void *arg)
{
	set_rids_args_t	*args = arg;
	uint_t		ctl_off;
	int		i;

	ctl_off = args->partition == pntb->pntb_partition ?
	    pntb->pntb_ctl_off : pntb->pntb_peer_ctl_off;

	for (i = 0; i < args->cnt; i++) {
		ntb_psx_put32(pntb, ctl_off + PNTB_NTB_CNTL_RID(i),
		    PNTB_NTB_CNTL_RID_ID(args->rids[i]) |
		    PNTB_NTB_CNTL_RID_ENABLE);
	}

	return (DDI_SUCCESS);
}

int
ntb_psx_set_rids(ntb_psx_t *pntb, int partition, uint16_t *rids, uint_t cnt)
{
	set_rids_args_t	args;
	uint_t		ctl_off;

	ctl_off = partition == pntb->pntb_partition ?
	    pntb->pntb_ctl_off : pntb->pntb_peer_ctl_off;

	args.partition = partition;
	args.rids = rids;
	args.cnt = cnt;

	return (ntb_psx_config_update(pntb, ctl_off, ntb_psx_set_rids_cb, &args));
}

static int
ntb_psx_clear_rids_cb(ntb_psx_t *pntb, void *arg)
{
	int		partition = (int)(uintptr_t)arg;
	uint_t		ctl_off;
	uint16_t	rid_limit;
	int		i;

	ctl_off = partition == pntb->pntb_partition ?
	    pntb->pntb_ctl_off : pntb->pntb_peer_ctl_off;

	rid_limit = ntb_psx_get16(pntb, ctl_off + PNTB_NTB_CNTL_RID_LIMIT);
	if (rid_limit == 0)
		return (DDI_SUCCESS);

	for (i = 0; i < rid_limit; i++)
		ntb_psx_put32(pntb, ctl_off + PNTB_NTB_CNTL_RID(i), 0);

	return (DDI_SUCCESS);
}

int
ntb_psx_clear_rids(ntb_psx_t *pntb, int partition)
{
	uint_t	ctl_off;

	ctl_off = partition == pntb->pntb_partition ?
	    pntb->pntb_ctl_off : pntb->pntb_peer_ctl_off;

	return (ntb_psx_config_update(pntb, ctl_off, ntb_psx_clear_rids_cb,
	    (void *)(uintptr_t)partition));
}

/*
 * disable rid usage
 */
void
ntb_psx_rid_disable(ntb_psx_t *pntb)
{
	uint32_t reg, reg_peer;

	reg = ntb_psx_get32(pntb, pntb->pntb_ctl_off + PNTB_NTB_PART_CNTRL);
	reg_peer = ntb_psx_get32(pntb, pntb->pntb_peer_ctl_off +
	    PNTB_NTB_PART_CNTRL);

	reg |= PNTB_ID_PROTECTION_DISABLE;
	reg_peer |= PNTB_ID_PROTECTION_DISABLE;

	ntb_psx_put32(pntb, pntb->pntb_ctl_off + PNTB_NTB_PART_CNTRL, reg);
	ntb_psx_put32(pntb, pntb->pntb_peer_ctl_off + PNTB_NTB_PART_CNTRL,
	    reg_peer);

#ifdef DEBUG
	cmn_err(CE_WARN, "ntb_psx_rid_disable: NTB Partition Control set to "
	    "0x%x/0x%x", reg, reg_peer);
#endif
}

/*
 * Called when the driver is quiesced.
 * Clear out all translations.
 */
static int
ntb_psx_quiesce_translation_cb(ntb_psx_t *pntb, void *arg)
{
	int		partition = (int)(uintptr_t)arg;
	int		bar, i;
	uint32_t	reg;
	uint_t		ctl_off, bar_off, lut_off;

	ctl_off = partition == pntb->pntb_partition ?
	    pntb->pntb_ctl_off : pntb->pntb_peer_ctl_off;

	for (bar = 0; bar < PCI_BASE_NUM; bar++) {
		bar_off = PNTB_NTB_CNTL_BAR_SETUP(bar);

		reg = ntb_psx_get32(pntb, ctl_off + bar_off);
		if ((reg & PNTB_NTB_CNTL_BAR_VALID) == 0)
			continue;

		for (i = 0; i < 16; i += 4)
			ntb_psx_put32(pntb, ctl_off + bar_off + i, 0);
	}

	for (i = 0; i < PNTB_NTB_LUT_ENTRIES; i++) {
		lut_off = PNTB_NTB_CNTL_LUT(i);

		ntb_psx_put32(pntb, ctl_off + lut_off, 0);
		ntb_psx_put32(pntb, ctl_off + lut_off + 4, 0);
	}

	return (DDI_SUCCESS);
}

/*
 * Called from the ddi_quiesce() routine. Must not sleep.
 */
int
ntb_psx_quiesce_translation(ntb_psx_t *pntb)
{
	/*
	 * Disable doorbells and messages.
	 */
	ntb_psx_inter_ntb_cleanup(pntb);

	/*
	 * Disable all translations in outbound partition.
	 */
	if (ntb_psx_config_update_nolock(pntb, pntb->pntb_ctl_off,
	    ntb_psx_quiesce_translation_cb,
	    (void *)(uintptr_t)pntb->pntb_partition) != DDI_SUCCESS)
		return (DDI_FAILURE);

	/*
	 * Disable all translations in inbound partition.
	 */
	if (ntb_psx_config_update_nolock(pntb, pntb->pntb_peer_ctl_off,
	    ntb_psx_quiesce_translation_cb,
	    (void *)(uintptr_t)pntb->pntb_peer_partition) != DDI_SUCCESS)
		return (DDI_FAILURE);

	/*
	 * Clear requester ids in outbound partition.
	 */
	if (ntb_psx_config_update_nolock(pntb, pntb->pntb_ctl_off,
	    ntb_psx_clear_rids_cb,
	    (void *)(uintptr_t)pntb->pntb_partition) != DDI_SUCCESS)
		return (DDI_FAILURE);

	/*
	 * Clear requester ids in inbound partition.
	 */
	if (ntb_psx_config_update_nolock(pntb, pntb->pntb_peer_ctl_off,
	    ntb_psx_clear_rids_cb,
	    (void *)(uintptr_t)pntb->pntb_peer_partition) != DDI_SUCCESS)
		return (DDI_FAILURE);

	return (DDI_SUCCESS);
}

static uint_t	ntb_psx_event_intr(caddr_t, caddr_t);
static uint_t	ntb_psx_doorbell_intr(caddr_t, caddr_t);
static uint_t	ntb_psx_message_intr(caddr_t, caddr_t);
static void	ntb_psx_run_db_callbacks(ntb_psx_t *, uint64_t, boolean_t);
static void	ntb_psx_set_local_doorbell(ntb_psx_t *, uint_t);
static int	ntb_psx_set_internal_db_callback(ntb_psx_t *, uint_t, db_func_t);
static void	ntb_psx_clear_internal_db_callback(ntb_psx_t *, uint_t,
		    db_func_t);
#if 0
static void	ntb_psx_ereport(ntb_psx_t *, char *);
#endif
static void	ntb_psx_mark_peer_down(ntb_psx_t *);

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
ntb_psx_peer_dbmsg_put32(ntb_psx_t *pntb, uint_t off, uint32_t val)
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
ntb_psx_peer_dbmsg_put64(ntb_psx_t *pntb, uint_t off, uint64_t val)
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
ntb_psx_inter_ntb_setup(ntb_psx_t *pntb)
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
	pntb->pntb_rupt_event = ntb_psx_get8(pntb, pntb->pntb_pcfg_off +
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
		ntb_psx_put8(pntb, pntb->pntb_db_msg_off + PNTB_NTB_IDB_MAP + i,
		    vec);
	}

	for (; i < PNTB_NTB_IDB_MAP_SIZE; i++)
		ntb_psx_put8(pntb, pntb->pntb_db_msg_off + PNTB_NTB_IDB_MAP + i,
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
	ntb_psx_put64(pntb, pntb->pntb_db_msg_off + PNTB_NTB_IDB_MASK, 0);

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
		ntb_psx_put64(pntb, pntb->pntb_db_msg_off + PNTB_NTB_IMSG(i),
		    PNTB_NTB_IMSG_FULL | PNTB_NTB_IMSG_IMASK);

	return (ntb_psx_check_gas_acc_handle(pntb));
}

void
ntb_psx_inter_ntb_cleanup(ntb_psx_t *pntb)
{
	int	i;

	/* Clear the vector map */
	for (i = 0; i < PNTB_NTB_IDB_MAP_SIZE; i++)
		ntb_psx_put8(pntb, pntb->pntb_db_msg_off + PNTB_NTB_IDB_MAP + i,
		    0);

	/* Mask all inbound doorbells including those for messages */
	ntb_psx_put64(pntb, pntb->pntb_db_msg_off + PNTB_NTB_IDB_MASK, 0);

	/* Mask inbound messages */
	for (i = 0; i < PNTB_MESSAGES; i++)
		ntb_psx_put64(pntb, pntb->pntb_db_msg_off + PNTB_NTB_IMSG(i), 0);
}

/*
 * Interrupt initialization.
 * Interrupts are generated for doorbells, messages and "events".
 */
int
ntb_psx_intr_init(ntb_psx_t *pntb)
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
	if (ntb_psx_inter_ntb_setup(pntb) != DDI_SUCCESS) {
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
			hdlr = ntb_psx_event_intr;
		else if (i == pntb->pntb_rupt_message)
			hdlr = ntb_psx_message_intr;
		else
			hdlr = ntb_psx_doorbell_intr;

		if (ddi_intr_add_handler(pntb->pntb_rupts[i], hdlr,
		    (caddr_t)pntb, (caddr_t)(uintptr_t)i) != DDI_SUCCESS) {
			psx_warn(pntb, "!Failed to add interrupt %d", i);
			goto failed;
		}
	}

	return (DDI_SUCCESS);

failed:
	ntb_psx_intr_fini(pntb);
	return (DDI_FAILURE);
}

/*
 * Cleanup up after (a possibly partial) ntb_psx_intr_init().
 */
void
ntb_psx_intr_fini(ntb_psx_t *pntb)
{
	int	i;

	ntb_psx_inter_ntb_cleanup(pntb);

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
ntb_psx_intr_enable(ntb_psx_t *pntb)
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
ntb_psx_intr_disable(ntb_psx_t *pntb)
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
ntb_psx_event_intr(caddr_t arg1, caddr_t arg2)
{
	ntb_psx_t	*pntb = (ntb_psx_t *)(uintptr_t)arg1;
	psx_bar_t	*win = pntb->pntb_seg_window;
	boolean_t	set_db = B_FALSE;
	uint16_t	vid;

	vid = ddi_get16(win->pb_hdl, (uint16_t *)(uintptr_t)(win->pb_vaddr +
	    pntb->pntb_peer_cfg_spc_off));

	mutex_enter(&pntb->pntb_lock);

	if (vid != pntb->pntb_vid) {
		/* only send a doorbell if we were up */
		set_db = ntb_psx_peer_is_up(pntb);
		ntb_psx_mark_peer_down(pntb);
	}

	mutex_exit(&pntb->pntb_lock);

	/*
	 * We only send the down doorbell. We wait for the "ping" to
	 * detect the peer driver is up before sending an up doorbell.
	 */
	if (set_db) {
		ntb_psx_set_local_doorbell(pntb, pntb->pntb_db_down);
#if 0
		ntb_psx_ereport(pntb, ZEBI_EV_LINK_DOWN);
#endif
		psx_note(pntb, "!Link is down");
		ntb_kstat_link_down(pntb->kdata);
	}

	return (DDI_INTR_CLAIMED);
}

/*
 * Doorbell interrupt handler.
 * Invoke any doorbell callback routines.
 */
static uint_t
ntb_psx_doorbell_intr(caddr_t arg1, caddr_t arg2)
{
	ntb_psx_t	*pntb = (ntb_psx_t *)(uintptr_t)arg1;
	uint_t		vec = (uint_t)(uintptr_t)arg2;
	uint64_t	set, mask;

	mask = pntb->pntb_rupt_masks[vec];

	do {
		set = ntb_psx_get64(pntb, pntb->pntb_db_msg_off + PNTB_NTB_IDB);
		set &= mask;

		if (set == 0)
			break;

		DTRACE_PROBE3(doorbell, ntb_psx_t *, pntb, uint_t, vec,
		    uint64_t, set);

		/*
		 * Clear the doorbells just read.
		 */
		ntb_psx_put64(pntb, pntb->pntb_db_msg_off + PNTB_NTB_IDB, set);

		/*
		 * ... and run any callbacks registered.
		 */
		ntb_psx_run_db_callbacks(pntb, set, B_FALSE);
	} while (/* CONSTCOND */ 1);

	if (ntb_psx_check_gas_acc_handle(pntb)) {
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
ntb_psx_message_intr(caddr_t arg1, caddr_t arg2)
{
	ntb_psx_t	*pntb = (ntb_psx_t *)(uintptr_t)arg1;
	uint64_t	sts;
	uint32_t	msg;
	int		i, set;

	do {
		set = 0;
		for (i = 0; i < PNTB_MESSAGES; i++) {
			sts = ntb_psx_get64(pntb, pntb->pntb_db_msg_off +
			    PNTB_NTB_IMSG(i));
			if ((sts & PNTB_NTB_IMSG_FULL) == 0)
				/* the message register is "empty" */
				continue;

			set++;
			msg = (uint32_t)sts;

			DTRACE_PROBE3(message, ntb_psx_t *, pntb, int, i,
			    uint32_t, msg);

			/* Clear it. */
			ntb_psx_put64(pntb, pntb->pntb_db_msg_off +
			    PNTB_NTB_IMSG(i), sts);

			rw_enter(&pntb->pntb_db_lock, RW_READER);
			if (pntb->pntb_msg_callback[i] != NULL) {
				pntb->pntb_msg_callback[i](pntb, i, msg);
			}
			rw_exit(&pntb->pntb_db_lock);
		}
	} while (set > 0);

	if (ntb_psx_check_gas_acc_handle(pntb)) {
		mutex_enter(&pntb->pntb_lock);
		pntb->pntb_flags |= PNTB_FAULTED;
		mutex_exit(&pntb->pntb_lock);
	}

	return (DDI_INTR_CLAIMED);
}

static void
ntb_psx_mark_peer_up(ntb_psx_t *pntb)
{
	ASSERT(mutex_owned(&pntb->pntb_lock));
	pntb->pntb_flags |= PNTB_PEER_UP;
}

static void
ntb_psx_mark_peer_down(ntb_psx_t *pntb)
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
ntb_psx_received_peer_down(void *arg, int db)
{
	ntb_psx_t	*pntb = arg;
	boolean_t	set_db;

	mutex_enter(&pntb->pntb_lock);

	/*
	 * When we get an explicit down message, update the ping
	 * counters too.
	 */
	if (pntb->pntb_flags & PNTB_MR4SAFE_LOCAL) {
		pntb->pntb_ping_recv_latest =
		    (uint32_t)ntb_psx_get64(pntb,
		    PNTB_SYSINFO_SCRATCHPAD_OFFSET +
		    PNTB_SYSINFO_HEARTBEAT_OFFSET);
	} else {
		if (ddi_dma_sync(pntb->pvt_handle, 0, sizeof (uint32_t),
		    DDI_DMA_SYNC_FORKERNEL) != DDI_SUCCESS) {
			pntb->pntb_flags |= PNTB_FAULTED;
			pntb->pntb_ping_recv_latest = pntb->pntb_ping_recv;
		} else {
			pntb->pntb_ping_recv_latest =
			    *(uint32_t *)(uintptr_t)pntb->pvt_vaddr;
		}
	}

	pntb->pntb_ping_recv = pntb->pntb_ping_recv_latest;

	if ((set_db = ntb_psx_peer_is_up(pntb)))
		ntb_psx_mark_peer_down(pntb);

	mutex_exit(&pntb->pntb_lock);

	if (set_db) {
		ntb_psx_set_local_doorbell(pntb, pntb->pntb_db_down);
#if 0
		ntb_psx_ereport(pntb, ZEBI_EV_LINK_DOWN);
#endif
		psx_note(pntb, "!Link is down");
		ntb_kstat_link_down(pntb->kdata);
	}
}

/*
 * Ping the peer driver by writing incrementing values in a reserved
 * scratchpad register. At the same time verify we are receiving
 * regular updates in the scratchpad register on this side.
 */
static void
ntb_psx_ping_peer(void *arg)
{
	ntb_psx_t  	*pntb = arg;
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
	 *   1) both canisters must have the ntb_psx driver upgraded before
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
	ntb_psx_get_sdk_fw_version(pntb, &fwver, &peer_fwver);
	sdk_version(fwver, &local_mode, &local_name);
	sdk_version(peer_fwver, &peer_mode, &peer_name);
	if (peer_fwver != 0xFFFFFFFF) {
		if (prev_peer_fw != peer_fwver) {
			cmn_err(CE_NOTE, "!ntb_psx: SDK versions: local(%s) "
			    "peer(%s)", local_name, peer_name);
			prev_peer_fw = peer_fwver;
			if (strcmp(local_name, "Unknown") == 0)
				cmn_err(CE_NOTE, "!ntb_psx: Unknown Local SDK "
				    "version: 0x%8x", fwver);
			if (strcmp(peer_name, "Unknown") == 0)
				cmn_err(CE_NOTE, "!ntb_psx: Unknown Peer SDK "
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

	if (ntb_psx_peer_is_faulted(pntb)) {
		pntb->pntb_ping_recv_latest = pntb->pntb_ping_recv;
	} else {
		if (pntb->pntb_flags & PNTB_MR4SAFE_PEER) {
			ntb_psx_putpeer64(pntb, PNTB_SYSINFO_SCRATCHPAD_OFFSET +
			    PNTB_SYSINFO_HEARTBEAT_OFFSET,
			    ++pntb->pntb_ping_sent);

			if (pntb->pntb_flags & PNTB_MR4SAFE_LOCAL) {
				pntb->pntb_ping_recv_latest =
				    (uint32_t)ntb_psx_get64(pntb,
				    PNTB_SYSINFO_SCRATCHPAD_OFFSET +
				    PNTB_SYSINFO_HEARTBEAT_OFFSET);
			} else {
				pntb->pntb_ping_recv_latest =
				    *(uint32_t *)(uintptr_t)
				    pntb->pvt_vaddr;
			}
		} else {
			ddi_put32(win->pb_hdl,
			    (uint32_t *)(uintptr_t)pntb->pvt_xaddr,
			    ++pntb->pntb_ping_sent);

			if (ntb_psx_check_win_acc_handle(pntb, win) !=
			    DDI_SUCCESS ||
			    ddi_dma_sync(pntb->pvt_handle, 0,
			    sizeof (uint32_t), DDI_DMA_SYNC_FORKERNEL) !=
			    DDI_SUCCESS) {
				pntb->pntb_flags |= PNTB_FAULTED;
				pntb->pntb_ping_recv_latest =
				    pntb->pntb_ping_recv;
			} else {
				if (pntb->pntb_flags & PNTB_MR4SAFE_LOCAL) {
					pntb->pntb_ping_recv_latest =
					    (uint32_t)ntb_psx_get64(pntb,
					    PNTB_SYSINFO_SCRATCHPAD_OFFSET +
					    PNTB_SYSINFO_HEARTBEAT_OFFSET);
				} else {
					pntb->pntb_ping_recv_latest =
					    *(uint32_t *)(uintptr_t)
					    pntb->pvt_vaddr;
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
		if (ntb_psx_peer_is_up(pntb)) {
			if (ntb_psx_peer_is_faulted(pntb) &&
			    (pntb->pntb_flags & PNTB_FAULT_HANDLED) == 0) {
				pntb->pntb_flags |= PNTB_FAULT_HANDLED;
				set_db = B_TRUE;
			} else {
				set_db = ++pntb->pntb_ping_misses >
				    PNTB_PING_TIMEOUT_CNT;
			}
		}

		if (set_db) {
			ntb_psx_mark_peer_down(pntb);
			db = pntb->pntb_db_down;
		}
	} else {
		pntb->pntb_ping_recv = pntb->pntb_ping_recv_latest;
		pntb->pntb_ping_misses = 0;

		if (!ntb_psx_peer_is_up(pntb)) {
			/*
			 * The peer DMA is available.
			 * We have to reset the doorbell mask and message
			 * map across the interconnect.
			 */
			ntb_psx_peer_dbmsg_put64(pntb, PNTB_NTB_ODB_MASK,
			    pntb->pntb_db_omask);

			ntb_psx_peer_dbmsg_put32(pntb, PNTB_NTB_MSG_MAP,
			    pntb->pntb_msg_map);

			/*
			 * We can only be sure the above masks are active
			 * when we get the ack to this message.
			 * Repeatedly set the masks and send this message
			 * until we get the ack.
			 */
			(void) ntb_psx_send_message(pntb, PNTB_PING_MESSAGE,
			    PNTB_MSGS_UP);
		}
	}

	mutex_exit(&pntb->pntb_lock);

	if (set_db) {
		ntb_psx_set_local_doorbell(pntb, db);
		if (db == pntb->pntb_db_up) {
#if 0
			ntb_psx_ereport(pntb, ZEBI_EV_LINK_UP);
#endif
			psx_note(pntb, "!Link is up");
			ntb_psx_get_peer_link_info(pntb);
			ntb_kstat_link_up(pntb->kdata, pntb->pntb_width,
			    pntb->pntb_speed);
		} else {
#if 0
			ntb_psx_ereport(pntb, ZEBI_EV_LINK_DOWN);
#endif
			psx_note(pntb, "!Link is down");
			ntb_kstat_link_down(pntb->kdata);
		}
	}

#ifdef DEBUG
	if (pntb->pntb_flags != prev_flags) {
		cmn_err(CE_WARN, "!ntb_psx_ping_peer: exit: flags now 0x%x",
		    pntb->pntb_flags);
		prev_flags = pntb->pntb_flags;
	}
#endif

}

static void
ntb_psx_peer_message_cb(ntb_psx_t *pntb, uint_t num, uint32_t msg)
{
	if (msg == PNTB_MSGS_UP) {
		(void) ntb_psx_send_message(pntb, num, PNTB_MSGS_UP_ACK);
		return;
	}

	ASSERT(msg == PNTB_MSGS_UP_ACK);

	mutex_enter(&pntb->pntb_lock);
	if (ntb_psx_peer_is_up(pntb)) {
		mutex_exit(&pntb->pntb_lock);
		return;
	}

	ntb_psx_mark_peer_up(pntb);
	mutex_exit(&pntb->pntb_lock);

	ntb_psx_set_local_doorbell(pntb, pntb->pntb_db_up);

	psx_note(pntb, "!Link is up");
	ntb_psx_get_peer_link_info(pntb);
#if 0
	ntb_psx_ereport(pntb, ZEBI_EV_LINK_UP);
#endif
	ntb_kstat_link_up(pntb->kdata, pntb->pntb_width, pntb->pntb_speed);
}

/*
 * Set up a cyclic to "ping" the peer to check whether it is up or down.
 */
int
ntb_psx_start_pinging(ntb_psx_t *pntb)
{
	cyc_handler_t	cyc_hdlr;
	cyc_time_t	cyc_when;

	if (ntb_psx_set_msg_callback(pntb, ntb_psx_peer_message_cb,
	    PNTB_PING_MESSAGE) != DDI_SUCCESS)
		return (DDI_FAILURE);
	*(uint32_t *)(uintptr_t)pntb->pvt_vaddr = 0;
	if (pntb->pntb_flags & PNTB_MR4SAFE_LOCAL) {
		ntb_psx_put64(pntb, PNTB_SYSINFO_SCRATCHPAD_OFFSET +
		    PNTB_SYSINFO_HEARTBEAT_OFFSET, 0);
	}

	cyc_hdlr.cyh_level = CY_LOCK_LEVEL;
	cyc_hdlr.cyh_func = ntb_psx_ping_peer;
	cyc_hdlr.cyh_arg = pntb;

	cyc_when.cyt_when = 0;
	cyc_when.cyt_interval = PNTB_PING_INTERVAL;

	mutex_enter(&cpu_lock);
	pntb->pntb_pinger = cyclic_add(&cyc_hdlr, &cyc_when);
	mutex_exit(&cpu_lock);

	return (DDI_SUCCESS);
}

void
ntb_psx_stop_pinging(ntb_psx_t *pntb)
{
	if (pntb->pntb_pinger == CYCLIC_NONE)
		return;

	mutex_enter(&cpu_lock);
	cyclic_remove(pntb->pntb_pinger);
	mutex_exit(&cpu_lock);

	(void) ntb_psx_clear_msg_callback(pntb, PNTB_PING_MESSAGE);
}

/*
 * Thread which executes doorbell callback functions when a doorbell is
 * asserted locally rather than on the peer.
 */
static void
ntb_psx_db_thread(void *arg)
{
	ntb_psx_t	*pntb = arg;
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

		ntb_psx_run_db_callbacks(pntb, set, B_TRUE);
	} while (/* CONSTCOND */ 1);
}

/*
 * For a given set of doorbells, run all doorbell
 * callback routines registered.
 */
static void
ntb_psx_run_db_callbacks(ntb_psx_t *pntb, uint64_t set, boolean_t local)
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
ntb_psx_doorbells_init(ntb_psx_t *pntb)
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
	thr = thread_create(NULL, 0, ntb_psx_db_thread, pntb, 0, &p0, TS_RUN,
	    maxclsyspri);
	pntb->pntb_db_thr = thr->t_did;

	/*
	 * An internal callback to handle peer going down doorbell.
	 */
	if (ntb_psx_set_internal_db_callback(pntb, pntb->pntb_db_down,
	    ntb_psx_received_peer_down) != DDI_SUCCESS) {
		psx_warn(pntb, "!Failed to set callback for internal function "
		    "\"ntb_psx_received_peer_down\"");
		pntb->pntb_flags |= PNTB_SHUTDOWN;
		ntb_psx_doorbells_fini(pntb);
		return (DDI_FAILURE);
	}

	return (DDI_SUCCESS);
}

void
ntb_psx_doorbells_fini(ntb_psx_t *pntb)
{
	int	i;

	if (pntb->pntb_dbs == NULL)
		return;

	ASSERT((pntb->pntb_flags & PNTB_SHUTDOWN) != 0);

	cv_signal(&pntb->pntb_db_cv);
	thread_join(pntb->pntb_db_thr);

	ntb_psx_clear_internal_db_callback(pntb, pntb->pntb_db_down,
	    ntb_psx_received_peer_down);

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
ntb_psx_set_internal_db_callback(ntb_psx_t *pntb, uint_t db, db_func_t fn)
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
	ntb_psx_put64(pntb, pntb->pntb_db_msg_off + PNTB_NTB_IDB_MASK,
	    pntb->pntb_db_mask);

	rw_exit(&pntb->pntb_db_lock);

	return (ntb_psx_check_gas_acc_handle(pntb));
}

static void
ntb_psx_clear_internal_db_callback(ntb_psx_t *pntb, uint_t db, db_func_t fn)
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
		ntb_psx_put64(pntb, pntb->pntb_db_msg_off + PNTB_NTB_IDB_MASK,
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
ntb_psx_set_db_callback(ntb_psx_t *pntb, dev_info_t *dip, uint_t db, db_func_t fn,
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
		ntb_psx_put64(pntb, pntb->pntb_db_msg_off + PNTB_NTB_IDB_MASK,
		    pntb->pntb_db_mask);

		if (ntb_psx_check_gas_acc_handle(pntb) != DDI_SUCCESS)
			goto cleanup;
	}

	rw_exit(&pntb->pntb_db_lock);

	up = ntb_psx_peer_is_up(pntb);
	if (up && db == pntb->pntb_db_up)
		ntb_psx_set_local_doorbell(pntb, db);
	else if (!up && db == pntb->pntb_db_down)
		ntb_psx_set_local_doorbell(pntb, db);

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
ntb_psx_clear_db_callback(ntb_psx_t *pntb, dev_info_t *dip, uint_t db)
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
		ntb_psx_put64(pntb, pntb->pntb_db_msg_off + PNTB_NTB_IDB_MASK,
		    pntb->pntb_db_mask);

		if (ntb_psx_check_gas_acc_handle(pntb) != DDI_SUCCESS) {
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
ntb_psx_set_peer_doorbell(ntb_psx_t *pntb, uint_t db)
{
	ASSERT(db < pntb->pntb_db_count);

	mutex_enter(&pntb->pntb_lock);
	ntb_psx_peer_dbmsg_put64(pntb, PNTB_NTB_ODB,
	    1ull << (db + pntb->pntb_db_peer_shift));

	/* sfence to flush the write-combining cache. */
	membar_producer();
	mutex_exit(&pntb->pntb_lock);

	return (ntb_psx_check_win_acc_handle(pntb, pntb->pntb_seg_window));
}

/*
 * Post a local doorbell.
 * Simply set the bit in the structure and wake up the thread.
 */
static void
ntb_psx_set_local_doorbell(ntb_psx_t *pntb, uint_t db)
{
	ASSERT(db < pntb->pntb_db_count);

	mutex_enter(&pntb->pntb_lock);
	pntb->pntb_db_set |= 1ull << db;
	cv_signal(&pntb->pntb_db_cv);
	mutex_exit(&pntb->pntb_lock);
}

int
ntb_psx_set_msg_callback(ntb_psx_t *pntb, msg_func_t fn, uint_t msg)
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

	ntb_psx_put64(pntb, pntb->pntb_db_msg_off + PNTB_NTB_IDB_MASK,
	    pntb->pntb_db_mask);

	rw_exit(&pntb->pntb_db_lock);

	return (ntb_psx_check_gas_acc_handle(pntb));
}


int
ntb_psx_clear_msg_callback(ntb_psx_t *pntb, uint_t msg)
{
	ASSERT(msg < PNTB_MESSAGES);

	rw_enter(&pntb->pntb_db_lock, RW_WRITER);

	pntb->pntb_msg_callback[msg] = NULL;
	pntb->pntb_db_mask &= (~1ull << (msg + PNTB_TOTAL_DOORBELLS -
	    PNTB_MESSAGES));

	ntb_psx_put64(pntb, pntb->pntb_db_msg_off + PNTB_NTB_IDB_MASK,
	    pntb->pntb_db_mask);

	rw_exit(&pntb->pntb_db_lock);

	return (ntb_psx_check_gas_acc_handle(pntb));
}

int
ntb_psx_send_message(ntb_psx_t *pntb, uint_t num, uint32_t msg)
{
	ASSERT(num < PNTB_MESSAGES);

	ntb_psx_peer_dbmsg_put32(pntb, PNTB_NTB_OMSG(num), msg);

	return (ntb_psx_check_win_acc_handle(pntb, pntb->pntb_seg_window));
}

#if 0
static void
ntb_psx_ereport(ntb_psx_t *pntb, char *str)
{
	nvlist_t	*erp;

	if ((erp = zebi_sys_monitor_ereport_create(ZEBI_EV_MOD_NTB, str,
	    B_FALSE, 0)) == NULL)
		return;

	(void) nvlist_add_string(erp, "driver-name",
	    ddi_driver_name(pntb->pntb_dip));
	(void) nvlist_add_uint32(erp, "pci-bus", pntb->pntb_bus);
	(void) nvlist_add_uint32(erp, "pci-dev", pntb->pntb_dev);
	(void) nvlist_add_uint32(erp, "pci-fn", pntb->pntb_fn);
	if (ntb_psx_peer_is_up(pntb)) {
		(void) nvlist_add_uint32(erp, "pci-negotiated-link-width",
		    pntb->pntb_width);
		(void) nvlist_add_uint32(erp, "pci-negotiated-link-speed",
		    pntb->pntb_speed);
	}

	zebi_sys_monitor_ereport_post(erp, B_FALSE);
}
#endif

ddi_device_acc_attr_t ntb_psx_acc_attr = {
	DDI_DEVICE_ATTR_V1,
	DDI_STRUCTURE_LE_ACC,
	DDI_STRICTORDER_ACC,
	DDI_DEFAULT_ACC
};

ddi_dma_attr_t pntb_attr_tmpl = {
	DMA_ATTR_V0,		/* version number */
	0x0,			/* low dma address */
	0xffffffffffffffffull,	/* high dma address (64 bit max) */
	0xffffffffffffffffull,	/* count max */
	0,			/* alignment */
	0xffffffff,		/* burst sizes */
	0,			/* min xfer */
	0xffffffffffffull,	/* max xfer */
	0,			/* segment boundary */
	0x7fffffff,		/* effectively no scatter gather limits */
	1,			/* granularity */
	0			/* flags */
};

typedef void (*rep8_t)(ddi_acc_handle_t, uint8_t *, uint8_t *, size_t, uint_t);
typedef void (*rep64_t)(ddi_acc_handle_t, uint64_t *, uint64_t *, size_t,
    uint_t);

/*
 * Get the current ntb-dma-window and return it. If it is not
 * set, get the current segment range and return the number of
 * the segmented window.
 */
static psx_bar_t *
ntb_psx_get_window(ntb_psx_t *pntb, dev_info_t *dip, int *segp, int *cntp)
{
	psx_bar_t	*win = NULL;
	int		window = -1;

	if (ntb_get_dma_window_by_dev(dip, &window) == DDI_SUCCESS &&
	    window >= pntb->pntb_wcnt)
		return (NULL);

	if (window >= 0)
		return (pntb->pntb_window[pntb->pntb_win_base + window]);

	if (ntb_get_bind_limits_by_dev(dip, segp, cntp) == DDI_SUCCESS ||
	    ntb_get_dma_segments_by_dev(dip, segp, cntp) == DDI_SUCCESS)
		win = pntb->pntb_seg_window;

	return (win);
}

/*
 * get_dma_attr function of ntb_drvr_ops.
 * Returns the required dma_attr and acc_attr to be used to
 * allocate a DMA object to be used for operations in the direction
 * of "dir".
 */
static int
ntb_psx_get_dma_attr(void *pvt, dev_info_t *dip, xfer_dir_t dir,
    ddi_dma_attr_t *attr, ddi_device_acc_attr_t *acc_attr, size_t *len)
{
	ntb_psx_t	*pntb = pvt;
	psx_bar_t	*win;
	int		seg, cnt;

	if (ntb_psx_peer_is_faulted(pntb))
		return (DDI_FAILURE);

	cnt = 1;
	win = ntb_psx_get_window(pntb, dip, &seg, &cnt);

	if (win == NULL)
		return (DDI_FAILURE);

	if (win->pb_owner != pntb->pntb_dip && win->pb_owner != dip)
		return (DDI_FAILURE);

	/*
	 * NTB_GET memory will be subject to ddi_dma_sync() calls
	 * which will check error status, so for those use attributes
	 * configured for FMA.
	 */
	if (attr != NULL) {
		if (dir == NTB_GET) {
			*attr = pntb->dma_attr;
			attr->dma_attr_seg = win->pb_seg_size - 1;
			attr->dma_attr_align = win->pb_seg_size;
			attr->dma_attr_minxfer = 1U << PNTB_MIN_SIZE;
			attr->dma_attr_sgllen = cnt;
			attr->dma_attr_flags |= PNTB_DMA_PEER;
		} else {
			*attr = pntb_attr_tmpl;
			attr->dma_attr_seg = PAGEOFFSET;
			attr->dma_attr_align = PAGESIZE;
			attr->dma_attr_minxfer = PAGESIZE;
		}
	}

	if (acc_attr != NULL)
		*acc_attr = dir == NTB_GET ? pntb->acc_attr :
		    ntb_psx_acc_attr;

	if (len != NULL)
		*len = win->pb_seg_size * cnt;

	return (DDI_SUCCESS);
}

/*
 * dma_mem_alloc function of ntb_drvr_ops.
 */
static int
ntb_psx_dma_mem_alloc(void *pvt, ddi_dma_handle_t hdl, size_t length,
    ddi_device_acc_attr_t *accattrp, uint_t flags, int (*waitfp)(caddr_t),
    caddr_t arg, caddr_t *kaddrp, size_t *real_length, void **handlep)
{
	return (ddi_dma_mem_alloc(hdl, length, accattrp, flags, waitfp, arg,
	    kaddrp, real_length, (ddi_acc_handle_t *)handlep));
}

/*
 * dma_mem_free function of ntb_drvr_ops.
 */
static void
ntb_psx_dma_mem_free(void *pvt, void **handlep)
{
	ddi_dma_mem_free((ddi_acc_handle_t *)handlep);
}

/*
 * get_link_cookies function of ntb_drvr_ops.
 */
static int
ntb_psx_get_link_cookies(void *pvt, ddi_dma_handle_t hdl, ddi_dma_cookie_t **cpp,
    int *ccnt)
{
	ntb_psx_t		*pntb = pvt;
	ddi_dma_impl_t		*i_hdl = (ddi_dma_impl_t *)hdl;
	ddi_dma_attr_t		*dma_attr = &i_hdl->dmai_attr;
	psx_dma_handle_t	*pntb_h = i_hdl->dmai_driver_private;
	psx_bar_t		*win = pntb_h->pdh_window;
	size_t			size, seg_size;
	uint64_t		start;
	int			i, seg;

	if (ntb_psx_peer_is_faulted(pntb))
		return (DDI_FAILURE);

	if ((dma_attr->dma_attr_flags & PNTB_DMA_PEER) != 0) {
		/*
		 * This is a handle for DMA on the peer side,
		 * all the cookies have been saved - just return those.
		 */
		size = sizeof (ddi_dma_cookie_t) * pntb_h->pdh_lut_cookie_cnt;

		*cpp = kmem_alloc(size, KM_SLEEP);
		*ccnt = pntb_h->pdh_lut_cookie_cnt;

		bcopy(pntb_h->pdh_lut_cookies, *cpp, size);

		return (DDI_SUCCESS);
	}

	/*
	 * A local handle. Return one cookie per segment.
	 * The windows which are not segmented will only have one segment.
	 */
	size = sizeof (ddi_dma_cookie_t) * pntb_h->pdh_seg_cnt;
	*cpp = kmem_alloc(size, KM_SLEEP);
	*ccnt = pntb_h->pdh_seg_cnt;

	seg_size = pntb_h->pdh_seg_size;
	seg = win->pb_lut_child + pntb_h->pdh_seg;
	start = pntb_h->pdh_window->pb_paddr + seg_size * seg;

	for (i = 0; i < pntb_h->pdh_seg_cnt; i++) {
		(*cpp)[i].dmac_laddress = start + i * seg_size;
		(*cpp)[i].dmac_size = seg_size;
		(*cpp)[i].dmac_type = 0;
	}

	return (DDI_SUCCESS);
}

/*
 * set_segment function of ntb_drvr_ops.
 */
static int
ntb_psx_set_segment(void *pvt, ddi_dma_handle_t hdl, int seg, uint64_t base)
{
	ntb_psx_t		*pntb = pvt;
	ddi_dma_impl_t		*i_hdl = (ddi_dma_impl_t *)hdl;
	ddi_dma_attr_t		*dma_attr = &i_hdl->dmai_attr;
	psx_dma_handle_t	*pntb_h = i_hdl->dmai_driver_private;
	psx_bar_t		*win = pntb_h->pdh_window;
	uint_t			lut;

	if (ntb_psx_peer_is_faulted(pntb))
		return (DDI_FAILURE);

	/* Operation is only applicable to peer DMA. */
	if ((dma_attr->dma_attr_flags & PNTB_DMA_PEER) == 0)
		return (DDI_FAILURE);

	/* Verify alignment. */
	if ((base & (pntb_h->pdh_seg_size - 1)) != 0)
		return (DDI_FAILURE);

	/* Verify segment is in range. */
	if (seg < pntb_h->pdh_seg ||
	    seg >= (pntb_h->pdh_seg + pntb_h->pdh_seg_cnt))
		return (DDI_FAILURE);

	/* Work out the actual entry in the LUT. */
	lut = win->pb_lut_child + seg;

	return (ntb_psx_set_lut_window(pntb, lut, pntb->pntb_partition, base));
}

/*
 * set_window function of ntb_drvr_ops.
 * Set the bar setup register to the physical address given in
 * the dma cookie, then any data coming across the NTB via this window
 * will end up in the memory described by the cookie.
 */
static int
ntb_psx_set_window(void *pvt, dev_info_t *cdip, int window, ddi_dma_cookie_t *cp)
{
	ntb_psx_t	*pntb = pvt;
	psx_bar_t	*win;
	uint64_t	paddr;
	int		rv;

	if (ntb_psx_peer_is_faulted(pntb))
		return (DDI_FAILURE);

	if (window < 0 || window >= pntb->pntb_wcnt)
		return (DDI_FAILURE);

	win = pntb->pntb_window[pntb->pntb_win_base + window];

	if (cp == NULL) {
		/*
		 * A NULL cp means we should remove the translation
		 * and close the window.
		 */
		ntb_psx_release_window(pntb, win);

		return (ntb_psx_clear_direct_window(pntb, win,
		    pntb->pntb_partition));
	}

	if (!ntb_psx_claim_window(pntb, win, cdip))
		return (DDI_FAILURE);

	/*
	 * The translations rely on the address being aligned to the size.
	 */
	paddr = cp->dmac_laddress & ~(win->pb_size - 1);

	rv = ntb_psx_set_direct_window(pntb, win, pntb->pntb_partition, paddr);
	if (rv != DDI_SUCCESS)
		ntb_psx_release_window(pntb, win);

	return (rv);
}

/*
 * Convert a DMA copy operation which covers an address range into a
 * sequence of ddi_rep_* operations to copy the data as efficiently as
 * possible. The operation happens in three steps:
 *	1. Use ddi_rep8 operations to align to 64 bits.
 *	2. Use ddi_rep64 operations as far as possible.
 *	3. Use ddi_rep8 to mop up from the last 64 bit alignment to the end.
 *
 * rep8 and rep64 arguments are the "rep" functions to use and are
 * either ddi_rep_getxx or ddi_rep_putxx.
 */
static int
ntb_psx_repcopy(ntb_psx_t *pntb, ddi_acc_handle_t handle, caddr_t host_addr,
    caddr_t dev_addr, off_t offset, size_t len, rep8_t rep8, rep64_t rep64)
{
	size_t	count8, count64;

	/*
	 * 8 bit copies to align to 64 bit.
	 */
	count8 = sizeof (uint64_t) - (offset & (sizeof (uint64_t) - 1));
	if (count8 < sizeof (uint64_t)) {
		if (count8 > len)
			count8 = len;

		/* The copy .. */
		rep8(handle, (uint8_t *)(host_addr + offset),
		    (uint8_t *)(dev_addr + offset), count8,
		    DDI_DEV_AUTOINCR);

		offset += count8;
		len -= count8;
	}

	if (len == 0)
		goto done;

	/*
	 * Now 64 bit access ....
	 */
	count64 = len / sizeof (uint64_t);
	if (count64 > 0) {
		rep64(handle, (uint64_t *)(uintptr_t)(host_addr + offset),
		    (uint64_t *)(uintptr_t)(dev_addr + offset), count64,
		    DDI_DEV_AUTOINCR);

		offset += count64 * sizeof (uint64_t);
		len -= count64 * sizeof (uint64_t);
	}

	/*
	 * ... and any leftover unaligned bytes using 8 bit copies.
	 */
	if (len > 0)
		rep8(handle, (uint8_t *)(host_addr + offset),
		    (uint8_t *)(dev_addr + offset), len, DDI_DEV_AUTOINCR);

done:
	/*
	 * This is an SFENCE instruction, which will ensure all the
	 * write-combining buffers are flushed.
	 */
	membar_producer();

	return (ntb_psx_check_acc_handle(pntb, handle));
}

/*
 * put function of ntb_drvr_ops.
 * Moves data onto the bridge for "hdl".
 */
static int
ntb_psx_dma_put(void *pvt, dev_info_t *cdip, ddi_dma_handle_t hdl, offset_t off,
    size_t len)
{
	ntb_psx_t		*pntb = pvt;
	ddi_dma_impl_t		*i_hdl = (ddi_dma_impl_t *)hdl;
	ddi_dma_attr_t		*dma_attr = &i_hdl->dmai_attr;
	psx_dma_handle_t	*pntb_h = i_hdl->dmai_driver_private;
	psx_bar_t		*win = pntb_h->pdh_window;
	size_t			segs_size, len_max;
	int			seg;

	if (ntb_psx_peer_is_faulted(pntb) || !ntb_psx_peer_is_up(pntb))
		return (DDI_FAILURE);

	/*
	 * We should only ever do a "put" to the primary side DMA.
	 */
	if ((dma_attr->dma_attr_flags & PNTB_DMA_PEER) != 0)
		return (DDI_FAILURE);

	/*
	 * off == 0 and len == 0 means put everything. But if just len
	 * is zero, then there is nothing to do.
	 */
	if (off != 0 && len == 0)
		return (DDI_SUCCESS);

	/*
	 * The maximum value len can be is the smallest of the DMA
	 * allocated (idh_dma_len), and the segments reserved in the window.
	 */
	segs_size = pntb_h->pdh_seg_size * pntb_h->pdh_seg_cnt;
	len_max = MIN(pntb_h->pdh_dma_len, segs_size);
	seg = win->pb_lut_child + pntb_h->pdh_seg;

	/*
	 * Some sanity checks. If "off" is beyond the length of either the
	 * source or destination, then there is nothing to do.
	 */
	if (off >= len_max)
		return (DDI_SUCCESS);

	/*
	 * truncate [off .. off + len) to fit in len_max.
	 */
	if (len == 0 || (len + off) > len_max)
		len = len_max - off;

	ntb_kstat_accumulate_put(pntb->kdata, len);

	return (ntb_psx_repcopy(pntb, win->pb_hdl, pntb_h->pdh_dma_addr,
	    win->pb_vaddr + seg * pntb_h->pdh_seg_size,
	    off, len, ddi_rep_put8, ddi_rep_put64));
}

/*
 * vput function of ntb_drvr_ops.
 * Move data onto the bridge from a "vaddr".
 * Uses non-temporal copy.
 */
static int
ntb_psx_vput(void *pvt, dev_info_t *cdip, ddi_dma_handle_t hdl, caddr_t vaddr,
    offset_t off, size_t len)
{
	ntb_psx_t		*pntb = pvt;
	ddi_dma_impl_t		*i_hdl = (ddi_dma_impl_t *)hdl;
	ddi_dma_attr_t		*dma_attr = &i_hdl->dmai_attr;
	psx_dma_handle_t	*pntb_h = i_hdl->dmai_driver_private;
	psx_bar_t		*win = pntb_h->pdh_window;
	size_t			segs_size;
	int			seg;

	if (ntb_psx_peer_is_faulted(pntb) || !ntb_psx_peer_is_up(pntb))
		return (DDI_FAILURE);

	/*
	 * We should only ever do a "put" to the primary side DMA.
	 */
	if ((dma_attr->dma_attr_flags & PNTB_DMA_PEER) != 0)
		return (DDI_FAILURE);

	if (len == 0)
		return (DDI_SUCCESS);

	segs_size = pntb_h->pdh_seg_size * pntb_h->pdh_seg_cnt;
	if ((off + len) > segs_size)
		len = segs_size - off;

	seg = win->pb_lut_child + pntb_h->pdh_seg;

	if (kcopy_nta(vaddr, win->pb_vaddr +
	    seg * pntb_h->pdh_seg_size + off, len, 0) != 0)
		return (DDI_FAILURE);

	/*
	 * This is an SFENCE instruction, which will ensure all the
	 * write-combining buffers are flushed.
	 */
	membar_producer();

	ntb_kstat_accumulate_put(pntb->kdata, len);

	return (ntb_psx_check_win_acc_handle(pntb, win));
}

/*
 * Get function of ntb_drvr_ops.
 * Move data from the peer side of the bridge into DMA described * by "hdl".
 */
static int
ntb_psx_dma_get(void *pvt, dev_info_t *cdip, ddi_dma_handle_t hdl, offset_t off,
    size_t len)
{
	ntb_psx_t		*pntb = pvt;
	ddi_dma_impl_t		*i_hdl = (ddi_dma_impl_t *)hdl;
	ddi_dma_attr_t		*dma_attr = &i_hdl->dmai_attr;
	psx_dma_handle_t	*pntb_h = i_hdl->dmai_driver_private;
	psx_bar_t		*win;
	int			seg;

	if (ntb_psx_peer_is_faulted(pntb))
		return (DDI_FAILURE);

	/*
	 * We should only ever do a "get" from the secondary side DMA.
	 */
	if ((dma_attr->dma_attr_flags & PNTB_DMA_PEER) == 0)
		return (DDI_FAILURE);

	/*
	 * This is a handle for DMA on the peer side,
	 * the client has direct access to the vaddr of the
	 * memory mapped. So no copy is required,
	 * all we may need to do is ddi_dma_sync().
	 */

	win = pntb_h->pdh_window;

	/*
	 * Adjust the offset based on which range of segments
	 * are assigned to this handle. When the handle is
	 * for a whole window (WINDOW_HDL), then there would
	 * only be one segment starting at zero. Also take into
	 * consideration that the user data may be offset from the
	 * window's base address.
	 */
	seg = win->pb_lut_child + pntb_h->pdh_seg;
	off += seg * pntb_h->pdh_seg_size;

	if (ddi_dma_sync(hdl, off, len, DDI_DMA_SYNC_FORKERNEL) !=
	    DDI_SUCCESS)
		return (DDI_FAILURE);

	return (ntb_psx_check_dma_handle(pntb, hdl));
}

/*
 * doorbell_ctl function of ntb_drvr_ops.
 */
static int
ntb_psx_doorbell_ctl(void *pvt, dev_info_t *dip, db_ctl_t ctl, uint_t num,
    void *arg1, void *arg2)
{
	ntb_psx_t	*pntb = pvt;

	if (ntb_psx_peer_is_faulted(pntb))
		return (DDI_FAILURE);

	switch (ctl) {
	case SEND_DOORBELL:
		if (num > pntb->pntb_db_high)
			return (DDI_FAILURE);

		return (ntb_psx_set_peer_doorbell(pntb, num));

	case SET_DOORBELL:
	case SET_FAST_DOORBELL:
		/*
		 * We don't differentiate between fast and normal doorbells.
		 * They're all handled in the interrupt context directly.
		 */
		if (arg1 == NULL)
			/* NULL function pointer */
			return (ntb_psx_clear_db_callback(pntb, dip, num));

		return (ntb_psx_set_db_callback(pntb, dip, num, (db_func_t)arg1,
		    arg2));

	case GET_DOORBELL_LIMITS:
		*(uint_t *)arg1 = 0;
		*(uint_t *)arg2 = pntb->pntb_db_high;
		return (DDI_SUCCESS);

	case GET_STATUS_DOORBELLS:
		*(uint_t *)arg1 = pntb->pntb_db_up;
		*(uint_t *)arg2 = pntb->pntb_db_down;
		return (DDI_SUCCESS);

	default:
		return (DDI_FAILURE);
	}
}

ntb_drv_ops_t ntb_psx_ops = {
	ntb_psx_get_dma_attr,
	ntb_psx_dma_mem_alloc,
	ntb_psx_dma_mem_free,
	ntb_psx_get_link_cookies,
	ntb_psx_set_segment,
	ntb_psx_set_window,
	ntb_psx_dma_put,
	ntb_psx_vput,
	ntb_psx_dma_get,
	ntb_psx_doorbell_ctl
};
