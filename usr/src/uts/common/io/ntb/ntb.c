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
#include <sys/conf.h>
#include <sys/ddi.h>
#include <sys/kmem.h>
#include <sys/list.h>
#include <sys/modctl.h>
#include <sys/sunddi.h>
#include <sys/sysmacros.h>

#include "ntb.h"

static ntb_kstat_t ntb_kstat_template = {
	{ "ntb_state",			KSTAT_DATA_UINT32 },
	{ "ntb_pci_speed",		KSTAT_DATA_UINT32 },
	{ "ntb_pci_width",		KSTAT_DATA_UINT32 },
	{ "ntb_put_count",		KSTAT_DATA_UINT64 },
	{ "ntb_put_bytes",		KSTAT_DATA_UINT64 }
};

static kmutex_t	list_lock;
static list_t registered;

/*
 * The NTB drivers are kept in a linear linked list. There are expected to
 * be very few devices registered and very limited searching of the list.
 * Searches are only likely at driver load and unload.
 */
int
ntb_register(dev_info_t *dip, ntb_drv_ops_t *ops, void *pvt)
{
	ntb_drv_t	*ntb;

	mutex_enter(&list_lock);

	for (ntb = list_head(&registered); ntb != NULL;
	    ntb = list_next(&registered, ntb)) {
		if (ntb->dip == dip) {
			mutex_exit(&list_lock);
			return (DDI_FAILURE);
		}
	}

	ntb = kmem_zalloc(sizeof (*ntb), KM_SLEEP);
	ntb->dip = dip;
	ntb->pvt = pvt;
	ntb->ops = ops;

	list_insert_tail(&registered, ntb);

	mutex_exit(&list_lock);

	return (DDI_SUCCESS);
}

int
ntb_unregister(dev_info_t *dip)
{
	ntb_drv_t	*ntb;

	mutex_enter(&list_lock);

	for (ntb = list_head(&registered); ntb != NULL;
	    ntb = list_next(&registered, ntb)) {
		if (ntb->dip == dip) {
			break;
		}
	}

	if (ntb == NULL || ntb->ref > 0) {
		mutex_exit(&list_lock);
		return (DDI_FAILURE);
	}

	list_remove(&registered, ntb);

	mutex_exit(&list_lock);

	if (ntb->kstat != NULL) {
		kstat_delete(ntb->kstat);
		kmem_free(ntb->kdata, sizeof (*ntb->kdata));
	}

	kmem_free(ntb, sizeof (*ntb));

	return (DDI_SUCCESS);
}

static ntb_drv_t *
ntb_find(dev_info_t *dip)
{
	ntb_drv_t	*ntb;

	mutex_enter(&list_lock);

	for (ntb = list_head(&registered); ntb != NULL;
	    ntb = list_next(&registered, ntb)) {
		if (ntb->dip == dip) {
			break;
		}
	}

	mutex_exit(&list_lock);

	return (ntb);
}

int
ntb_register_kstat(dev_info_t *dip, ntb_kstat_t **kstatp)
{
	ntb_drv_t	*ntb;

	if ((ntb = ntb_find(dip)) == NULL)
		return (DDI_FAILURE);

	ntb->kstat = kstat_create(ddi_driver_name(dip), ddi_get_instance(dip),
	    "counters", "misc", KSTAT_TYPE_NAMED, sizeof (ntb_kstat_t) /
	    sizeof (kstat_named_t), KSTAT_FLAG_VIRTUAL);

	if (ntb->kstat == NULL)
		return (DDI_FAILURE);

	ntb->kdata = kmem_alloc(sizeof (*ntb->kdata), KM_SLEEP);
	bcopy(&ntb_kstat_template, ntb->kdata, sizeof (*ntb->kdata));
	ntb->kstat->ks_data = ntb->kdata;
	kstat_install(ntb->kstat);

	*kstatp = ntb->kdata;

	return (DDI_SUCCESS);
}

void
ntb_kstat_link_up(ntb_kstat_t *kdata, uint_t speed, uint width)
{
	kdata->state.value.ui32 = 1;
	kdata->speed.value.ui32 = speed;
	kdata->width.value.ui32 = width;
}

void
ntb_kstat_link_down(ntb_kstat_t *kdata)
{
	kdata->state.value.ui32 = 0;
	kdata->speed.value.ui32 = -1u;
	kdata->width.value.ui32 = -1u;
}

/*
 * Allocate a ntb_svc handle by finding the register ntb driver which is the
 * clients direct parent
 */
int
ntb_allocate_svc(dev_info_t *cdip, ntb_svc_t **svcp)
{
	dev_info_t	*pdip = ddi_get_parent(cdip);
	ntb_drv_t	*ntb;

	mutex_enter(&list_lock);

	for (ntb = list_head(&registered); ntb != NULL;
	    ntb = list_next(&registered, ntb)) {
		if (ntb->dip == pdip) {
			break;
		}
	}

	if (ntb == NULL) {
		mutex_exit(&list_lock);
		return (DDI_FAILURE);
	}

	ntb->ref++;

	mutex_exit(&list_lock);

	*svcp = kmem_zalloc(sizeof (ntb_svc_t), KM_SLEEP);
	(*svcp)->dip = cdip;
	(*svcp)->drv = ntb;
	(*svcp)->drv_pvt = ntb->pvt;
	(*svcp)->drv_ops = ntb->ops;

	return (DDI_SUCCESS);
}

int
ntb_free_svc(ntb_svc_t *svc)
{
	mutex_enter(&list_lock);
	svc->drv->ref--;
	mutex_exit(&list_lock);

	kmem_free(svc, sizeof (*svc));

	return (DDI_SUCCESS);
}

/*
 * For the given dip, return the dma window assigned to the device.
 */
int
ntb_get_dma_window_by_dev(dev_info_t *dip, int *window)
{
	if ((*window = ddi_prop_get_int(DDI_DEV_T_ANY, dip,
	    DDI_PROP_DONTPASS, "ntb-dma-window", -1)) == -1)
		return (DDI_FAILURE);

	return (DDI_SUCCESS);
}

/*
 * For the given ntb_svc handle, return the dma window assigned to it.
 */
int
ntb_get_dma_window(ntb_svc_t *svc, int *window)
{
	return (ntb_get_dma_window_by_dev(svc->dip, window));
}

/*
 * Set the DMA window for future allocations.
 */
int
ntb_set_dma_window(ntb_svc_t *svc, int window)
{
	if (window < 0) {
		(void) ddi_prop_remove(DDI_DEV_T_NONE, svc->dip,
		    "ntb-dma-window");
		return (DDI_SUCCESS);
	}

	if (ddi_prop_update_int(DDI_DEV_T_NONE, svc->dip,
	    "ntb-dma-window", window) != DDI_SUCCESS) {
		return (DDI_FAILURE);
	}

	return (DDI_SUCCESS);
}

static int
get_number_and_cnt_props(dev_info_t *dip, char *seg_prop, int *seg,
    char *cnt_prop, int *cnt)
{
	if (seg) {
		if ((*seg = ddi_prop_get_int(DDI_DEV_T_ANY, dip,
		    DDI_PROP_DONTPASS, seg_prop, -1)) == -1) {
			return (DDI_FAILURE);
		}
	}

	if (cnt) {
		*cnt = ddi_prop_get_int(DDI_DEV_T_ANY, dip, DDI_PROP_DONTPASS,
		    cnt_prop, 1);
	}

	return (DDI_SUCCESS);
}

/*
 * For the given dip, return the dma segments assigned to the device.
 */
int
ntb_get_dma_segments_by_dev(dev_info_t *dip, int *seg, int *cnt)
{
	return (get_number_and_cnt_props(dip, "ntb-dma-segment", seg,
	    "ntb-dma-segment-count", cnt));
}

/*
 * For the given ntb_svc handle, return the dma segments assigned to it.
 */
int
ntb_get_dma_segments(ntb_svc_t *svc, int *seg, int *cnt)
{
	return (ntb_get_dma_segments_by_dev(svc->dip, seg, cnt));
}

/*
 * Given a dip, return the current bind limits.
 * The bind limits are taken note of by the NTB driver (plx) when
 * assigning segments for a bind request. Segments are assigned from
 * the "ntb-bind-segment" for a maximum count of "ntb-bind-segment-count".
 * Should these properties not be present, then segment allocation
 * is based on "ntb-dma-segment".
 */
int
ntb_get_bind_limits_by_dev(dev_info_t *dip, int *seg, int *cnt)
{
	return (get_number_and_cnt_props(dip, "ntb-bind-segment", seg,
	    "ntb-bind-segment-count", cnt));
}

/*
 * For the given ntb_svc handle, return the current bind limits.
 */
int
ntb_get_bind_limits(ntb_svc_t *svc, int *seg, int *cnt)
{
	return (ntb_get_bind_limits_by_dev(svc->dip, seg, cnt));
}

/*
 * Set the bind limits. The bind limits must fall within the range of
 * segments assigned to a device.
 */
int
ntb_set_bind_limits(ntb_svc_t *svc, int seg, int cnt)
{
	int	dev_seg, dev_cnt;

	if (ntb_get_dma_segments_by_dev(svc->dip, &dev_seg, &dev_cnt) !=
	    DDI_SUCCESS)
		return (DDI_FAILURE);

	if (seg < dev_seg)
		return (DDI_FAILURE);

	if (cnt == 0)
		cnt = dev_cnt - (seg - dev_seg);

	if (cnt <= 0 || (seg + cnt) > (dev_seg + dev_cnt))
		return (DDI_FAILURE);

	if (ddi_prop_update_int(DDI_DEV_T_NONE, svc->dip,
	    "ntb-bind-segment", seg) != DDI_SUCCESS) {
		return (DDI_FAILURE);
	}

	if (ddi_prop_update_int(DDI_DEV_T_NONE, svc->dip,
	    "ntb-bind-segment-count", cnt) != DDI_SUCCESS) {
		(void) ddi_prop_remove(DDI_DEV_T_NONE, svc->dip,
		    "ntb-bind-segment");
		return (DDI_FAILURE);
	}

	return (DDI_SUCCESS);
}

/*
 * Remove the "bind" properties to remove bind limits
 */
int
ntb_reset_bind_limits(ntb_svc_t *svc)
{
	if (ddi_prop_remove(DDI_DEV_T_NONE, svc->dip, "ntb-bind-segment") !=
	    DDI_SUCCESS)
		return (DDI_FAILURE);

	return (ddi_prop_remove(DDI_DEV_T_NONE, svc->dip,
	    "ntb-bind-segment-count"));
}

/*
 * Returns the DMA attributes which are required by the client driver
 * to allocate DMA memory for use across the NTB
 */
int
ntb_get_dma_attr(ntb_svc_t *svc, xfer_dir_t dir, ddi_dma_attr_t *attr,
    ddi_device_acc_attr_t *acc_attr, size_t *size)
{
	return (svc->drv_ops->get_dma_attr(svc->drv_pvt, svc->dip, dir, attr,
	    acc_attr, size));
}

/*
 * Allocates memory address range suitable for use on the target (aka
 * secondary) side of the NTB.
 */
int
ntb_dma_mem_alloc(ntb_svc_t *svc, ddi_dma_handle_t handle, size_t length,
    ddi_device_acc_attr_t *accattrp, uint_t flags, int (*waitfp)(caddr_t),
    caddr_t arg, caddr_t *kaddrp, size_t *real_length, void **handlep)
{
	return (svc->drv_ops->dma_mem_alloc(svc->drv_pvt, handle, length,
	    accattrp, flags, waitfp, arg, kaddrp, real_length, handlep));
}

/*
 * Free the memory previously allocated by a call to ntb_dma_mem_alloc().
*/
void
ntb_dma_mem_free(ntb_svc_t *svc, void **handlep)
{
	svc->drv_ops->dma_mem_free(svc->drv_pvt, handlep);
}

/*
 * Return a set of cookies which can be used by the client driver to directly
 * transfer data into the NTB's memory space such that the data is transported
 * across the NTB.
 */
int
ntb_get_link_cookies(ntb_svc_t *svc, ddi_dma_handle_t hdl,
    ddi_dma_cookie_t **cookiep, int *ccnt)
{
	return (svc->drv_ops->get_link_cookies(svc->drv_pvt, hdl, cookiep,
	    ccnt));
}

/*
 * Set the base address of seg to the physical address given in addr.
 * The caller should ensure the segment is not being accessed.
 */
int
ntb_set_segment(ntb_svc_t *svc, ddi_dma_handle_t hdl, int seg, uint64_t addr)
{
	return (svc->drv_ops->set_segment(svc->drv_pvt, hdl, seg, addr));
}

/*
 * Set the address range of "window" to:
 * 	[cp->dmac_laddress .. cp->dmac_laddress + cp->dmac_size)
 * If the cp is NULL then the window is disabled.
 */
int
ntb_set_window(ntb_svc_t *svc, int window, ddi_dma_cookie_t *cp)
{
	return (svc->drv_ops->set_window(svc->drv_pvt, svc->dip, window, cp));
}

/*
 * Get data from the NTB's PCI address space. Used to retreive data which
 * has been sent (via ntb_put()) from the peer.
 *    off is the offset in the PCI address space and offset into DMA
 *	where the data will end up.
 *    len is the number of bytes to transfer.
 */
int
ntb_get(ntb_svc_t *svc, ddi_dma_handle_t hdl, offset_t off, size_t len)
{
	return (svc->drv_ops->get(svc->drv_pvt, svc->dip, hdl, off, len));
}

/*
 * Put data into the the NTB's PCI address space. The consequence is the
 * data will traverse the bridge.
 *    off is the offset from DMA and will be the offset in the PCI address
 *	space to which the data is copied.
 *    len is the number of bytes to transfer.
 */
int
ntb_put(ntb_svc_t *svc, ddi_dma_handle_t hdl, offset_t off, size_t len)
{
	return (svc->drv_ops->put(svc->drv_pvt, svc->dip, hdl, off, len));
}

/*
 * Similar to ntb_put() except rather than copying data from the DMA
 * linked to hdl, it takes the data from the kernel virtual address
 * given in vaddr.
 */
int
ntb_vput(ntb_svc_t *svc, ddi_dma_handle_t hdl, caddr_t vaddr, offset_t off,
    size_t len)
{
	return (svc->drv_ops->vput(svc->drv_pvt, svc->dip, hdl, vaddr, off,
	    len));
}

int
ntb_send_doorbell(ntb_svc_t *svc, uint_t num)
{
	return (svc->drv_ops->doorbell_ctl(svc->drv_pvt, svc->dip,
	    SEND_DOORBELL, num, NULL, NULL));
}

/*
 * Set the callback for doorbell "num" to function "fn" passing argument
 * "arg". When fn is NULL the doorbell callback is cleared.
 */
int
ntb_set_doorbell(ntb_svc_t *svc, uint_t num, db_func_t fn, void *arg)
{
	return (svc->drv_ops->doorbell_ctl(svc->drv_pvt, svc->dip,
	    SET_DOORBELL, num, (void *)fn, arg));
}

/*
 * Same as above, except the callback fn is called directly from the
 * interrupt thread rather than a service thread.
 */
int
ntb_set_fast_doorbell(ntb_svc_t *svc, uint_t num, db_func_t fn, void *arg)
{
	return (svc->drv_ops->doorbell_ctl(svc->drv_pvt, svc->dip,
	    SET_FAST_DOORBELL, num, (void *)fn, arg));
}

/*
 * Returns the lowest and highest doorbell numbers supported.
 */
int
ntb_get_doorbell_range(ntb_svc_t *svc, uint_t *lo, uint_t *hi)
{
	return (svc->drv_ops->doorbell_ctl(svc->drv_pvt, svc->dip,
	    GET_DOORBELL_LIMITS, 0, lo, hi));
}

/*
 * Returns the doorbells which are used to signal when the connection
 * to peer NTB is up or down
 */
int
ntb_get_status_doorbells(ntb_svc_t *svc, uint_t *up, uint_t *down)
{
	return (svc->drv_ops->doorbell_ctl(svc->drv_pvt, svc->dip,
	    GET_STATUS_DOORBELLS, 0, up, down));
}

/*
 * A set of default ntb_drv_ops which simply pass the call
 * through to the ntb_svc nexus one level up the device tree.
 */
int
ntb_svc_get_dma_attr(ntb_svc_t *svc, dev_info_t *dip, xfer_dir_t dir,
    ddi_dma_attr_t *attr, ddi_device_acc_attr_t *acc_attr, size_t *len)
{
	return (svc->drv_ops->get_dma_attr(svc->drv_pvt, dip, dir, attr,
	    acc_attr, len));
}

int
ntb_svc_dma_mem_alloc(ntb_svc_t *svc, ddi_dma_handle_t handle, size_t length,
    ddi_device_acc_attr_t *accattrp, uint_t flags, int (*waitfp)(caddr_t),
    caddr_t arg, caddr_t *kaddrp, size_t *real_length, void **handlep)
{
	return (svc->drv_ops->dma_mem_alloc(svc->drv_pvt, handle, length,
	    accattrp, flags, waitfp, arg, kaddrp, real_length, handlep));
}

void
ntb_svc_dma_mem_free(ntb_svc_t *svc, void **handlep)
{
	svc->drv_ops->dma_mem_free(svc->drv_pvt, handlep);
}

int
ntb_svc_get_link_cookies(ntb_svc_t *svc, ddi_dma_handle_t hdl,
    ddi_dma_cookie_t **cpp, int *ccnt)
{
	return (svc->drv_ops->get_link_cookies(svc->drv_pvt, hdl, cpp, ccnt));
}

int
ntb_svc_set_segment(ntb_svc_t *svc, ddi_dma_handle_t hdl, int seg,
    uint64_t base)
{
	return (svc->drv_ops->set_segment(svc->drv_pvt, hdl, seg, base));
}

int
ntb_svc_set_window(ntb_svc_t *svc, dev_info_t *cdip, int window,
    ddi_dma_cookie_t *cp)
{
	return (svc->drv_ops->set_window(svc->drv_pvt, cdip, window, cp));
}

int
ntb_svc_put(ntb_svc_t *svc, dev_info_t *cdip, ddi_dma_handle_t hdl,
    offset_t off, size_t len)
{
	return (svc->drv_ops->put(svc->drv_pvt, cdip, hdl, off, len));
}

int
ntb_svc_vput(ntb_svc_t *svc, dev_info_t *cdip, ddi_dma_handle_t hdl,
    caddr_t vaddr, offset_t off, size_t len)
{
	return (svc->drv_ops->vput(svc->drv_pvt, cdip, hdl, vaddr, off, len));
}

int
ntb_svc_get(ntb_svc_t *svc, dev_info_t *cdip, ddi_dma_handle_t hdl,
    offset_t off, size_t len)
{
	return (svc->drv_ops->get(svc->drv_pvt, cdip, hdl, off, len));
}

int
ntb_svc_doorbell_ctl(ntb_svc_t *svc, dev_info_t *dip, db_ctl_t ctl, uint_t num,
    void *arg1, void *arg2)
{
	return (svc->drv_ops->doorbell_ctl(svc->drv_pvt, dip, ctl, num, arg1,
	    arg2));
}

static struct modldrv modldrv = {
	&mod_miscops,
	"NTB Service module",
	NULL,
};

static struct modlinkage modlinkage = {
	MODREV_1,
	&modldrv,
	0
};

int
_init(void)
{
	int error;

	if ((error = mod_install(&modlinkage)) == 0) {
		mutex_init(&list_lock, NULL, MUTEX_DRIVER, NULL);
		list_create(&registered, sizeof (ntb_drv_t),
		    offsetof(ntb_drv_t, link));
	}

	return (error);
}

int
_fini(void)
{
	int error;

	if ((error = mod_remove(&modlinkage)) == 0) {
		mutex_destroy(&list_lock);
		list_destroy(&registered);
	}

	return (error);
}

int
_info(struct modinfo *modinfop)
{
	return (mod_info(&modlinkage, modinfop));
}
