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

#ifndef _ENTB_NTB_H
#define	_ENTB_NTB_H

typedef struct entb_conn entb_conn_t;

typedef struct entb_ntb_s {
	dev_info_t	*dip;
	struct entb_s  *entb_ntb_ss;
	entb_conn_t	*ntb_peer;
	list_t		tx_list;	/* list of mblks for tx */
	kmutex_t	tx_lock;
	kcondvar_t	tx_cv;
	list_t		rx_list;	/* list of mblks for rx */
	kmutex_t	rx_lock;
	kcondvar_t	rx_cv;
	ntb_endpoint_ops_t ep_ops;
} entb_ntb_t;

#endif
