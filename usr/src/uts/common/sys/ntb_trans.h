/*
 * Copyright 2015 Tegile Systems, Inc.  All rights reserved.
 * Use is subject to license terms.
 */

#ifndef	_NTB_TRANS_H
#define	_NTB_TRANS_H

typedef struct ntb_sge {
	caddr_t	local;		/* virtual address at local end */
	size_t	bytes;		/* bytes to transfer */
	uint64_t remote;	/* location at remote end */
	int	result;		/* errno, 0 == success */
} ntb_sge_t;

typedef struct ntb_handle ntb_handle_t;
typedef void (*ntb_cb_fn_t)(ntb_handle_t *, ntb_sge_t *, void *);

typedef enum { TRANSPORT_CONNECTED, TRANSPORT_DISCONNECTED } link_status_t;

typedef enum {
	NTB_NETWORK = 0,	/* network traffic, Eg entb */
	NTB_BULK,		/* bulk data, Eg nvrd */
	NTB_CLASS_CNT		/* always last */
} ntb_class_t;

/*
 * Each "endpoint" driver needs to define one of these.
 * remote_to_kva() is called by the ntb_transport so the endpoint can
 * convert the "remote" from the originators ntb_sge_t to a valid
 * kernel virtual address. It should return the kva and DDI_SUCCESS if the
 * combination of remote and byte count arguments are valid.
 *
 * remote_flushed() is called by the ntb_transport when data represented
 * by a sge has been flushed to endpoints private buffer. The routine
 * should set the "result" element of the ntb_sge_t to indicate it has
 * successfully (or not) dealt with the data. This is fed back to the
 * originators ntb_sge_t.
 */
typedef struct ntb_endpoint_ops {
	uint_t		id;		/* endpoint's unique id */
	ntb_class_t	class;		/* endpoint's data class */
	caddr_t	(*remote_to_kva)(void *, ntb_sge_t *);
	void	(*remote_flushed)(void *, ntb_sge_t *);
	void	(*link_status)(void *, link_status_t);
} ntb_endpoint_ops_t;

extern int	ntb_name_to_endpoint_id(const char *);
extern int	ntb_register_endpoint(ntb_endpoint_ops_t *, void *);
extern int	ntb_unregister_endpoint(ntb_endpoint_ops_t *);

extern ntb_handle_t	*ntb_alloc_handle(uint_t, ntb_class_t);
extern void		ntb_release_handle(ntb_handle_t *);
extern int		ntb_store(ntb_handle_t *, ntb_sge_t *);
extern int		ntb_async_store(ntb_handle_t *, ntb_sge_t *,
			    ntb_cb_fn_t, void *);
extern int		ntb_sync(ntb_handle_t *);
extern int		ntb_async_fetch(ntb_handle_t *, ntb_sge_t *,
			    ntb_cb_fn_t, void *);
extern int		ntb_fetch(ntb_handle_t *, ntb_sge_t *);
extern size_t		ntb_transfer_max(ntb_handle_t *);

#endif
