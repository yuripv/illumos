#
# This file and its contents are supplied under the terms of the
# Common Development and Distribution License ("CDDL"), version 1.0.
# You may only use this file in accordance with the terms of version
# 1.0 of the CDDL.
#
# A full copy of the text of the CDDL should have accompanied this
# source.  A copy of the CDDL is also available via the Internet at
# http://www.illumos.org/license/CDDL.
#

#
# Copyright (c) 2017, Joyent, Inc.
#

UTSBASE=	../..

MODULE=		bnxt
OBJECTS=	$(BNXT_OBJS:%=$(OBJS_DIR)/%)
ROOTMODULE=	$(ROOT_DRV_DIR)/$(MODULE)
CONF_SRCDIR=	$(UTSBASE)/common/io/bnxt

include		$(UTSBASE)/intel/Makefile.intel

CERRWARN +=	-_gcc=-Wno-type-limits
CPPFLAGS +=	-I$(UTSBASE)/common/io/bnxt
CPPFLAGS +=	-DHAVE_KSENSOR -DHAVE_MAC_MEDIA -DHAVE_MAC_VLAN

ALL_TARGET=	$(BINARY) $(CONFMOD)
INSTALL_TARGET=	$(BINARY) $(ROOTMODULE) $(ROOT_CONFFILE)

LDFLAGS +=	-Nmisc/mac

MAPFILES +=	ddi ksensor mac random

.KEEP_STATE:

def:		$(DEF_DEPS)

all:		$(ALL_DEPS)

clean:		$(CLEAN_DEPS)

clobber:	$(CLOBBER_DEPS)

install:	$(INSTALL_DEPS)

#include		$(UTSBASE)/Makefile.mapfile
include		$(UTSBASE)/intel/Makefile.targ
