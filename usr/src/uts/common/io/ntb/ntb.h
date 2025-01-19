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

#ifndef	_NTB_H
#define	_NTB_H

#include <sys/atomic.h>
#include <sys/ddidmareq.h>
#include <sys/kstat.h>

typedef void (*db_func_t)(void *, int);

typedef enum { NTB_PUT, NTB_GET } xfer_dir_t;

typedef struct ntb_drv ntb_drv_t;
typedef struct ntb_drv_ops ntb_drv_ops_t;

typedef struct ntb {
	dev_info_t	*dip;
	ntb_drv_t	*drv;
	void		*drv_pvt;
	ntb_drv_ops_t	*drv_ops;
} ntb_svc_t;

extern int	ntb_allocate_svc(dev_info_t *, ntb_svc_t **);
extern int	ntb_free_svc(ntb_svc_t *);
extern int	ntb_get_dma_window(ntb_svc_t *, int *);
extern int	ntb_set_dma_window(ntb_svc_t *, int);
extern int	ntb_get_dma_segments(ntb_svc_t *, int *, int *);
extern int	ntb_get_bind_limits(ntb_svc_t *, int *, int *);
extern int	ntb_set_bind_limits(ntb_svc_t *, int, int);
extern int	ntb_reset_bind_limits(ntb_svc_t *);
extern int	ntb_get_dma_attr(ntb_svc_t *, xfer_dir_t, ddi_dma_attr_t *,
		    ddi_device_acc_attr_t *, size_t *);
extern int	ntb_dma_mem_alloc(ntb_svc_t *, ddi_dma_handle_t, size_t,
		    ddi_device_acc_attr_t *, uint_t, int (*)(caddr_t),
		    caddr_t, caddr_t *, size_t *, void **);
extern void	ntb_dma_mem_free(ntb_svc_t *, void **);
extern int	ntb_get_link_cookies(ntb_svc_t *, ddi_dma_handle_t,
		    ddi_dma_cookie_t **, int *);
extern int	ntb_set_segment(ntb_svc_t *, ddi_dma_handle_t, int, uint64_t);
extern int	ntb_set_window(ntb_svc_t *, int, ddi_dma_cookie_t *);
extern int	ntb_put(ntb_svc_t *, ddi_dma_handle_t, offset_t, size_t);
extern int	ntb_vput(ntb_svc_t *, ddi_dma_handle_t, caddr_t, offset_t,
		    size_t);
extern int	ntb_get(ntb_svc_t *, ddi_dma_handle_t, offset_t, size_t);
extern int	ntb_send_doorbell(ntb_svc_t *, uint_t);
extern int	ntb_set_doorbell(ntb_svc_t *, uint_t, db_func_t, void *);
extern int	ntb_set_fast_doorbell(ntb_svc_t *, uint_t, db_func_t, void *);
extern int	ntb_get_doorbell_range(ntb_svc_t *, uint_t *, uint_t *);
extern int	ntb_get_status_doorbells(ntb_svc_t *, uint_t *, uint_t *);

/*
 * Parent driver related functions and structures
 */
typedef enum {
	SEND_DOORBELL,
	SET_DOORBELL,
	SET_FAST_DOORBELL,
	GET_DOORBELL_LIMITS,
	GET_STATUS_DOORBELLS
} db_ctl_t;

typedef struct ntb_kstat {
	kstat_named_t	state;		/* 0 == down, 1 == up */
	kstat_named_t	speed;		/* pci speed */
	kstat_named_t	width;		/* pci width */
	kstat_named_t	put_count;	/* count of ntb_[v]put() */
	kstat_named_t	put_bytes;	/* accum. of ntb_[v]put() bytes */
} ntb_kstat_t;

struct ntb_drv {
	dev_info_t	*dip;		/* parent's dev_info */
	int		ref;		/* number of children registered */
	void		*pvt;		/* parent private */
	ntb_drv_ops_t	*ops;
	kstat_t		*kstat;
	ntb_kstat_t	*kdata;
	list_node_t	link;
};

struct ntb_drv_ops {
	int	(*get_dma_attr)(void *, dev_info_t *, xfer_dir_t,
		    ddi_dma_attr_t *, ddi_device_acc_attr_t *, size_t *);
	int	(*dma_mem_alloc)(void *, ddi_dma_handle_t, size_t,
		    ddi_device_acc_attr_t *, uint_t, int (*)(caddr_t),
		    caddr_t, caddr_t *, size_t *, void **);
	void	(*dma_mem_free)(void *, void **);
	int	(*get_link_cookies)(void *, ddi_dma_handle_t,
		    ddi_dma_cookie_t **, int *);
	int	(*set_segment)(void *, ddi_dma_handle_t, int, uint64_t);
	int	(*set_window)(void *, dev_info_t *, int, ddi_dma_cookie_t *);
	int	(*put)(void *, dev_info_t *, ddi_dma_handle_t, offset_t,
		    size_t);
	int	(*vput)(void *, dev_info_t *, ddi_dma_handle_t, caddr_t,
		    offset_t, size_t);
	int	(*get)(void *, dev_info_t *, ddi_dma_handle_t, offset_t,
		    size_t);
	int	(*doorbell_ctl)(void *, dev_info_t *, db_ctl_t, uint_t, void *,
		    void *);
};

extern int	ntb_register(dev_info_t *, ntb_drv_ops_t *, void *);
extern int	ntb_unregister(dev_info_t *);
extern int	ntb_register_kstat(dev_info_t *, ntb_kstat_t **);
extern void	ntb_kstat_link_up(ntb_kstat_t *, uint_t, uint_t);
extern void	ntb_kstat_link_down(ntb_kstat_t *);
extern int	ntb_get_dma_window_by_dev(dev_info_t *, int *);
extern int	ntb_get_dma_segments_by_dev(dev_info_t *, int *, int *);
extern int	ntb_get_bind_limits_by_dev(dev_info_t *, int *, int *);

static inline void
ntb_kstat_accumulate_put(ntb_kstat_t *kdata, size_t len)
{
	atomic_inc_64(&kdata->put_count.value.ui64);
	atomic_add_64(&kdata->put_bytes.value.ui64, len);
}

/*
 * Pass through ntb_drv_ops routines.
 */
extern int	ntb_svc_get_dma_attr(ntb_svc_t *, dev_info_t *, xfer_dir_t,
		    ddi_dma_attr_t *, ddi_device_acc_attr_t *, size_t *);
extern int	ntb_svc_dma_mem_alloc(ntb_svc_t *, ddi_dma_handle_t, size_t,
		    ddi_device_acc_attr_t *, uint_t, int (*)(caddr_t),
		    caddr_t, caddr_t *, size_t *, void **);
extern void	ntb_svc_dma_mem_free(ntb_svc_t *, void **);
extern int	ntb_svc_get_link_cookies(ntb_svc_t *, ddi_dma_handle_t,
		    ddi_dma_cookie_t **, int *);
extern int	ntb_svc_set_segment(ntb_svc_t *, ddi_dma_handle_t, int,
		    uint64_t);
extern int	ntb_svc_set_window(ntb_svc_t *, dev_info_t *, int,
		    ddi_dma_cookie_t *);
extern int	ntb_svc_put(ntb_svc_t *, dev_info_t *, ddi_dma_handle_t,
		    offset_t, size_t);
extern int	ntb_svc_vput(ntb_svc_t *, dev_info_t *, ddi_dma_handle_t,
		    caddr_t, offset_t, size_t);
extern int	ntb_svc_get(ntb_svc_t *, dev_info_t *, ddi_dma_handle_t,
		    offset_t, size_t);
extern int	ntb_svc_doorbell_ctl(ntb_svc_t *, dev_info_t *, db_ctl_t,
		    uint_t, void *, void *);
#endif
