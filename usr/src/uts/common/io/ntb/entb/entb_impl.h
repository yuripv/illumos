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
 * Copyright 2025 Tintri by DDN, Inc.  All rights reserved.
 */

#ifndef _ENTB_IMPL_H
#define	_ENTB_IMPL_H

#include <sys/types.h>
#include <sys/mac_provider.h>
#include <sys/mac_ether.h>
#include <sys/vlan.h>

#define	ENTB_DEFAULT_MTU	(32 * 1024)
#define	ENTB_MAX_MTU		(4 * ENTB_DEFAULT_MTU)

typedef enum {
	ENTB_PEER_STATE_UNKNOWN = 0,
	ENTB_PEER_STATE_UP,
	ENTB_PEER_STATE_DOWN
} peer_state_t;

typedef struct entb_props_s {
	uint64_t		ep_ifspeed;
	uint32_t		ep_mtu;
} entb_props_t;

typedef enum { ntb_transport, plx_transport } entb_trans_t;

typedef struct entb_s {
	mac_handle_t		entb_mac_hdl;
	uint8_t			*entb_mac_addr;
	dev_info_t		*entb_dip;
	uint_t			entb_instance;
	entb_trans_t		entb_backend;
	entb_props_t		*entb_props;

	struct entb_ntb_s	*entb_ntb_info;
	struct entb_plx_s	*entb_plx_info;

	kmutex_t		entb_link_lock;
	link_state_t		entb_link_state;
	peer_state_t		entb_peer_state;
} entb_t;

extern void entb_choose_backend(entb_t *entb_ss);
extern int entb_m_setprop(void *, const char *, mac_prop_id_t, uint_t,
    const void *);

extern void entb_transport_init(void);
extern void entb_transport_fini(void);
extern int entb_ntb_init(entb_t *entb_ss, dev_info_t *dip);
extern int entb_ntb_destroy(entb_t *entb_ss, dev_info_t *dip);
extern void entb_ntb_send(entb_t *entb_ss, mblk_t *mp);
extern boolean_t entb_ntb_peer_ok(entb_t *entb_ss);

extern int entb_plx_init(entb_t *entb_ss, dev_info_t *dip);
extern int entb_plx_destroy(entb_t *entb_ss, dev_info_t *dip);
extern void entb_post_tx_queue(entb_t *entb_ss, mblk_t *mp);

#endif
