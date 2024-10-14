/*
 * Copyright (c) 2018, Tegile Systems, Inc. All rights reserved.
 */
#ifndef _NTB_SVC_IMPL_H
#define	_NTB_SVC_IMPL_H

#include <sys/ntb_svc.h>

struct ntb_driver {
	dev_info_t	*dip;	/* parent's dev_info */
	int		ref;	/* number of children registered */
	void		*pvt;	/* parent private */
	ntb_drvr_ops_t	*ops;
	kstat_t		*kstat;
	ntb_kstat_t	*kdata;
	list_node_t	link;
};

#endif
