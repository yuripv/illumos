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

/*
 * Ethernet over PCI-X bridge.
 */

#include <sys/types.h>
#include <sys/cmn_err.h>
#include <sys/conf.h>
#include <sys/ddi.h>
#include <sys/devops.h>
#include <sys/kmem.h>
#include <sys/modctl.h>
#include <sys/random.h>
#include <sys/strsubr.h>

#include "entb_impl.h"

#define	ENTB_DRV_NAME	"entb"
#define	ENTB_SUCCESS	0
#define	ENTB_FAILURE	1

/*
 * Both of the following default mac addresses will be
 * re-generated before being registered with the MAC layer.
 * Look at the routine entb_get_random_mac_addr().
 */
uint8_t entb_default_mac0[] = {
	0x02, 0x00, 0x00, 0x00, 0x00, 0x02
};

uint8_t entb_default_mac1[] = {
	0x06, 0x00, 0x00, 0x00, 0x00, 0x02
};

/* Driver entry point declarations */
static int entb_attach(dev_info_t *, ddi_attach_cmd_t);
static int entb_detach(dev_info_t *, ddi_detach_cmd_t);

/* MAC callbacks */
static int entb_m_stat(void *, uint_t, uint64_t *);
static int entb_m_start(void *);
static void entb_m_stop(void *);
static int entb_m_promisc(void *, boolean_t);
static int entb_m_multicast(void *, boolean_t, const uint8_t *);
static int entb_m_unicast(void *, const uint8_t *);
static mblk_t *entb_m_tx(void *, mblk_t *);
static boolean_t entb_m_getcapab(void *, mac_capab_t, void *);
static int entb_m_getprop(void *, const char *, mac_prop_id_t, uint_t, void *);
static void entb_m_propinfo(void *, const char *, mac_prop_id_t,
    mac_prop_info_handle_t);

DDI_DEFINE_STREAM_OPS(entb_ops, nulldev, nulldev, entb_attach, entb_detach,
    nodev, NULL, D_MP, NULL, ddi_quiesce_not_needed);

static struct modldrv entb_modldrv = {
	&mod_driverops,		/* Driver module */
	"EoNTB driver",		/* Driver name and version */
	&entb_ops,		/* Driver ops */
};

static struct modlinkage entb_modlinkage = {
	MODREV_1, (void *)&entb_modldrv, NULL
};

#define	ENTB_M_CALLBACK_FLAGS	\
	(MC_GETCAPAB | MC_SETPROP | MC_GETPROP | MC_PROPINFO)
static mac_callbacks_t entb_m_callbacks = {
	ENTB_M_CALLBACK_FLAGS,
	entb_m_stat,
	entb_m_start,
	entb_m_stop,
	entb_m_promisc,
	entb_m_multicast,
	entb_m_unicast,
	entb_m_tx,
	NULL,
	NULL,
	entb_m_getcapab,
	NULL,
	NULL,
	entb_m_setprop,
	entb_m_getprop,
	entb_m_propinfo
};

void *entb_state;

/*
 * static functions
 */
static void entb_update_prop(entb_t *entb_ss);
static void entb_do_tx(entb_t *, mblk_t *);
static void entb_init_link_lock(entb_t *entb_ss);
static void entb_destroy_link_lock(entb_t *entb_ss);
static void entb_get_random_mac_addr(uint8_t *mac_addr);

int
_init(void)
{
	int ret;

	if (ddi_name_to_major(ENTB_DRV_NAME) == (major_t)-1) {
		cmn_err(CE_WARN, "!cannot get major number");
		return (ENODEV);
	}

	ret = ddi_soft_state_init(&entb_state, sizeof (entb_t), 0);
	if (ret != 0) {
		cmn_err(CE_WARN, "!cannot retrieve soft state");
		return (ret);
	}

	entb_transport_init();
	mac_init_ops(&entb_ops, ENTB_DRV_NAME);
	ret = mod_install(&entb_modlinkage);
	if (ret != 0) {
		entb_transport_fini();
		mac_fini_ops(&entb_ops);
		ddi_soft_state_fini(&entb_state);
		cmn_err(CE_WARN, "!mod install failed");
		return (ret);
	}

	return (ret);
}

int
_info(struct modinfo *modinfop)
{
	return (mod_info(&entb_modlinkage, modinfop));
}

int
_fini(void)
{
	int ret;

	ret = mod_remove(&entb_modlinkage);
	if (ret != 0)
		return (ret);

	entb_transport_fini();
	mac_fini_ops(&entb_ops);
	ddi_soft_state_fini(&entb_state);

	return (0);
}

static int
entb_m_stat(void *arg, uint_t stat, uint64_t *val)
{
	entb_t *entb_ss = (entb_t *)arg;
	switch (stat) {
		case MAC_STAT_IFSPEED:
			*val = entb_ss->entb_props->ep_ifspeed;
			break;
		case ETHER_STAT_LINK_DUPLEX:
			*val = LINK_DUPLEX_FULL;
			break;
		default:
			*val = 0;
			return (ENOTSUP);
	}
	return (0);
}

static int
entb_m_start(void *arg)
{
	entb_t *entb_ss = (entb_t *)arg;

	if (entb_ss->entb_link_state == LINK_STATE_UP) {
		return (0);
	}

	mutex_enter(&entb_ss->entb_link_lock);
	if (entb_ss->entb_peer_state == ENTB_PEER_STATE_UP) {
		mac_link_update(entb_ss->entb_mac_hdl, LINK_STATE_UP);
		entb_ss->entb_link_state = LINK_STATE_UP;
	}
	mutex_exit(&entb_ss->entb_link_lock);

	return (0);
}

static void
entb_m_stop(void *arg)
{
	entb_t *entb_ss = arg;

	if (entb_ss->entb_link_state == LINK_STATE_DOWN) {
		return;
	}

	mutex_enter(&entb_ss->entb_link_lock);
	mac_link_update(entb_ss->entb_mac_hdl, LINK_STATE_DOWN);
	entb_ss->entb_link_state = LINK_STATE_DOWN;
	mutex_exit(&entb_ss->entb_link_lock);
}

static int
entb_m_promisc(void *arg, boolean_t flag)
{
	return (0);
}

static int
entb_m_multicast(void *arg, boolean_t add, const uint8_t *mcast_mac)
{
	return (EINVAL);
}

static int
entb_m_unicast(void *arg, const uint8_t *macaddr)
{
	return (EINVAL);
}

static mblk_t *
entb_m_tx(void *arg, mblk_t *mp)
{
	entb_t *entb_ss = arg;

	if (entb_ss->entb_link_state != LINK_STATE_UP) {
		freemsgchain(mp);
		return (NULL);
	}

	entb_do_tx(entb_ss, mp);

	return (NULL);
}

static int
entb_led_set(void *arg, mac_led_mode_t mode, uint_t flags)
{
	return (mode == MAC_LED_DEFAULT ? 0 : ENOTSUP);
}

static boolean_t
entb_m_getcapab(void *arg, mac_capab_t cap, void *cap_data)
{
	mac_capab_led_t *cap_led;

	switch (cap) {
	case MAC_CAPAB_LED:
		cap_led = cap_data;
		cap_led->mcl_set = entb_led_set;
		cap_led->mcl_flags = 0;
		cap_led->mcl_modes = MAC_LED_DEFAULT;
		return (B_TRUE);

	default:
		return (B_FALSE);
	}
}

int
entb_m_setprop(void *arg, const char *pr_name, mac_prop_id_t pr_num,
    uint_t pr_valsize, const void *pr_val)
{
	entb_t *entb_ss = arg;
	uint32_t max_sdu, mtu;

	switch (pr_num) {
	case MAC_PROP_MTU:
		bcopy(pr_val, &mtu, sizeof (mtu));
		max_sdu = entb_ss->entb_props->ep_mtu -
		    (sizeof (struct ether_header) + VLAN_TAGSZ);
		if (mtu > ETHERMIN && mtu <= max_sdu)
			return (mac_maxsdu_update(entb_ss->entb_mac_hdl, mtu));

		return (EINVAL);
	default:
		break;
	}

	return (ENOTSUP);
}

static int
entb_m_getprop(void *arg, const char *pr_name, mac_prop_id_t pr_num,
    uint_t pr_valsize, void *pr_val)
{
	entb_t *entb_ss;
	link_duplex_t duplex;
	uint64_t speed;
	int err = 0;

	entb_ss = arg;
	duplex = LINK_DUPLEX_FULL;
	speed = entb_ss->entb_props->ep_ifspeed;

	switch (pr_num) {
	case MAC_PROP_DUPLEX:
		ASSERT(pr_valsize >= sizeof (link_duplex_t));
		bcopy(&duplex, pr_val, sizeof (link_duplex_t));
		break;

	case MAC_PROP_SPEED:
		ASSERT(pr_valsize >= sizeof (uint64_t));
		bcopy(&speed, pr_val, sizeof (speed));
		break;
	default:
		err = ENOTSUP;
		break;
	}

	return (err);
}

static void
entb_m_propinfo(void *arg, const char *pr_name, mac_prop_id_t pr_num,
    mac_prop_info_handle_t prh)
{
}

static int
entb_mac_register(entb_t *entb_ss, dev_info_t *dip)
{
	int instance;
	mac_register_t *macp;
	int ret;

	if ((macp = mac_alloc(MAC_VERSION)) == NULL) {
		cmn_err(CE_WARN, "!entb: mac alloc FAILED!");
		return (ENTB_FAILURE);
	}

	instance = ddi_get_instance(dip);
	macp->m_type_ident = MAC_PLUGIN_IDENT_ETHER;
	macp->m_driver = entb_ss;
	macp->m_dip = dip;
	if (instance % 2) {
		entb_get_random_mac_addr(entb_default_mac0);
		macp->m_src_addr = entb_default_mac0;
		entb_ss->entb_mac_addr = entb_default_mac0;
	} else {
		entb_get_random_mac_addr(entb_default_mac1);
		macp->m_src_addr = entb_default_mac1;
		entb_ss->entb_mac_addr = entb_default_mac1;
	}
	macp->m_callbacks = &entb_m_callbacks;
	macp->m_min_sdu = 0;
	macp->m_max_sdu = entb_ss->entb_props->ep_mtu -
	    (sizeof (struct ether_header) + VLAN_TAGSZ);
	macp->m_margin = VLAN_TAGSZ;
	macp->m_priv_props = NULL;

	ret = mac_register(macp, &entb_ss->entb_mac_hdl);
	mac_free(macp);

	if (ret != 0) {
		cmn_err(CE_WARN, "!entb: mac register FAILED!");
		return (ENTB_FAILURE);
	}

	return (ENTB_SUCCESS);
}

static int
entb_attach(dev_info_t *dip, ddi_attach_cmd_t cmd)
{
	entb_t		*entb_ss = NULL;
	int		instance;
	int32_t		ret;

	if (cmd != DDI_ATTACH) {
		cmn_err(CE_WARN, "!entb: Attach FAILED! wrong command!");
		return (DDI_FAILURE);
	}

	/*
	 * Allocate softstate for this instance.
	 */
	instance = ddi_get_instance(dip);
	if (ddi_soft_state_zalloc(entb_state, instance) == DDI_FAILURE) {
		cmn_err(CE_WARN, "!entb: zalloc FAILED!");
		return (DDI_FAILURE);
	}

	entb_ss = ddi_get_soft_state(entb_state, instance);
	if (entb_ss == NULL) {
		cmn_err(CE_WARN, "!entb: Get soft state FAILED!");
		ddi_soft_state_free(entb_state, instance);
		return (DDI_FAILURE);
	}

	entb_ss->entb_dip = dip;
	entb_ss->entb_instance = (uint_t)instance;
	entb_ss->entb_backend = ntb_transport;

	entb_update_prop(entb_ss);

	/*
	 * Inititialize the link state lock and the default link state.
	 */
	entb_ss->entb_link_state = LINK_STATE_UNKNOWN;
	entb_init_link_lock(entb_ss);

	/*
	 * Register with the mac driver.
	 */
	if (entb_mac_register(entb_ss, dip) != ENTB_SUCCESS) {
		cmn_err(CE_WARN, "!entb: Registering with the mac driver "
		    "FAILED!");
		goto attach_fail;
	}

	ret = entb_ntb_init(entb_ss, dip);
	if (ret != 0) {
		cmn_err(CE_WARN, "!entb: ntb initialization FAILED!");
		goto attach_fail;
	}

	ret = entb_plx_init(entb_ss, dip);
	if (ret != 0) {
		cmn_err(CE_WARN, "!entb: plx initialization FAILED!");
		goto attach_fail;
	}

	ddi_set_driver_private(dip, entb_ss);
	ddi_report_dev(dip);
	return (DDI_SUCCESS);

attach_fail:
	if (entb_ss != NULL) {
		if (entb_ss->entb_ntb_info)
			(void) entb_ntb_destroy(entb_ss, dip);

		if (entb_ss->entb_plx_info)
			(void) entb_plx_destroy(entb_ss, dip);

		if (entb_ss->entb_mac_hdl)
			(void) mac_unregister(entb_ss->entb_mac_hdl);

		ddi_soft_state_free(entb_state, instance);
	}

	return (DDI_FAILURE);
}

static int
entb_detach(dev_info_t *dip, ddi_detach_cmd_t cmd)
{
	entb_t *entb_ss;
	int instance;

	if (cmd != DDI_DETACH) {
		cmn_err(CE_WARN, "!entb: Detach FAILED: wrong command!");
		return (DDI_FAILURE);
	}

	instance = ddi_get_instance(dip);
	entb_ss = ddi_get_soft_state(entb_state, instance);

	if (entb_ss->entb_props) {
		kmem_free(entb_ss->entb_props, sizeof (entb_props_t));
		entb_ss->entb_props = NULL;
	}

	entb_destroy_link_lock(entb_ss);
	if (mac_unregister(entb_ss->entb_mac_hdl) != 0) {
		cmn_err(CE_WARN, "!entb: Unregister with mac layer FAILED!");
		return (DDI_FAILURE);
	}
	entb_ss->entb_mac_hdl = NULL;

	(void) entb_ntb_destroy(entb_ss, dip);

	if (entb_ss->entb_plx_info)
		(void) entb_plx_destroy(entb_ss, dip);

	ddi_set_driver_private(dip, NULL);
	ddi_soft_state_free(entb_state, instance);

	return (DDI_SUCCESS);
}

static void
entb_update_prop(entb_t *entb_ss)
{
	if (!entb_ss->entb_props) {
		entb_ss->entb_props = kmem_zalloc(sizeof (entb_props_t),
		    KM_SLEEP);
	}

	entb_ss->entb_props->ep_ifspeed = 10000000000;

	entb_ss->entb_props->ep_mtu = ddi_prop_get_int(DDI_DEV_T_ANY,
	    entb_ss->entb_dip, DDI_PROP_DONTPASS, "max_mtu", ENTB_DEFAULT_MTU);
	if (entb_ss->entb_props->ep_mtu < ETHERMTU ||
	    entb_ss->entb_props->ep_mtu > ENTB_MAX_MTU) {
		cmn_err(CE_WARN, "!entb: \"max_mtu\" property out of range. "
		    "Using default mtu %d", ENTB_DEFAULT_MTU);
		entb_ss->entb_props->ep_mtu = ENTB_DEFAULT_MTU;
	}
}

static void
entb_do_tx(entb_t *entb_ss, mblk_t *mp)
{
	mblk_t *mp_elem;

	while (mp) {
		mp_elem = mp->b_next;
		mp->b_next = NULL;

		if (entb_ss->entb_backend == ntb_transport)
			entb_ntb_send(entb_ss, mp);
		else
			entb_post_tx_queue(entb_ss, mp);

		mp = mp_elem;
	}
}

/*
 * The criteria (and priorities) for choosing which backend to use are:
 * 1. If there is no plx device enable (ie ntb-dma-segment is not set in
 *    the entb.conf file, then always use ntb_transport
 * 2. If the ntb_transport has not been initialised then fall back to plx.
 *    This should never happen - but just in case ...
 * 3. When both the plx and ntb_transport are both initialised, always
 *    use the ntb_transport when it is up.
 *
 * This routine is called when either the ntb_transport or the plx interface
 * is notified that a connection has been established with its peer, at which
 * point it is decided which backend to use. The notification each transport
 * receives about its peer is independent of each other and there is no
 * ordering guaranteed, so it is possible for the plx backend to be
 * enabled for a short (I do mean short!) while before the ntb_transport
 * is notified. As soon as the ntb_transport is notified its peer is
 * available traffic will transverse the ntb_transport.
 *
 * When available the ntb_transport is the preferred path.
 */
void
entb_choose_backend(entb_t *entb_ss)
{
	ASSERT(mutex_owner(&entb_ss->entb_link_lock));

	if (entb_ss->entb_plx_info == NULL)
		entb_ss->entb_backend = ntb_transport;
	else if (entb_ss->entb_ntb_info == NULL)
		entb_ss->entb_backend = plx_transport;
	else if (entb_ntb_peer_ok(entb_ss))
		entb_ss->entb_backend = ntb_transport;
	else
		entb_ss->entb_backend = plx_transport;
}

static void
entb_init_link_lock(entb_t *entb_ss)
{
	mutex_init(&entb_ss->entb_link_lock, NULL, MUTEX_DEFAULT, NULL);
}

static void
entb_destroy_link_lock(entb_t *entb_ss)
{
	mutex_destroy(&entb_ss->entb_link_lock);
}

static void
entb_get_random_mac_addr(uint8_t *mac_addr)
{
	/*
	 * We will not give users a provision to set the
	 * mac address. Use random number generator to generate
	 * the address. Note that the first byte and the last
	 * byte of the addr will be fixed. The 4 bytes in b/w
	 * will be randomly generated.
	 */
	(void) random_get_pseudo_bytes((uint8_t *)&mac_addr[1], 4);
}
