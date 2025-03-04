#
# CDDL HEADER START
#
# The contents of this file are subject to the terms of the
# Common Development and Distribution License (the "License").
# You may not use this file except in compliance with the License.
#
# You can obtain a copy of the license at usr/src/OPENSOLARIS.LICENSE
# or http://www.opensolaris.org/os/licensing.
# See the License for the specific language governing permissions
# and limitations under the License.
#
# When distributing Covered Code, include this CDDL HEADER in each
# file and include the License file at usr/src/OPENSOLARIS.LICENSE.
# If applicable, add the following below this CDDL HEADER, with the
# fields enclosed by brackets "[]" replaced with your own identifying
# information: Portions Copyright [yyyy] [name of copyright owner]
#
# CDDL HEADER END
#
#
# Copyright (c) 2009, 2010, Oracle and/or its affiliates. All rights reserved.
# Copyright 2018 OmniOS Community Edition (OmniOSce) Association.
# Copyright (c) 2018, Joyent, Inc.
#

LIBRARY =	ioctl.a
VERS =
OBJECTS =	ioctl.o

PYSRCS=		__init__.py util.py dataset.py \
		allow.py unallow.py \
		userspace.py groupspace.py holds.py table.py

include ../../Makefile.lib

LIBLINKS =
SRCDIR =	../common
ROOTLIBDIR=	$(ROOT)/usr/lib/python$(PYVER)/vendor-packages/zfs
ROOTLIBDIR64=	$(ROOTLIBDIR)/64
PYOBJS=		$(PYSRCS:%.py=$(SRCDIR)/%.pyc)
PYFILES=	$(PYSRCS) $(PYSRCS:%.py=%.pyc)
ROOTPYZFSFILES= $(PYFILES:%=$(ROOTLIBDIR)/%)

CSTD=        $(CSTD_GNU99)

LIBS =		$(DYNLIB)
LDLIBS +=	-lc -lnvpair -lpython$(PYVER)$(PYSUFFIX) -lzfs
NATIVE_LIBS +=	libpython$(PYVER)$(PYSUFFIX).so
CFLAGS +=	$(CCVERBOSE)
CERRWARN +=	-_gcc=-Wno-unused-variable
CPPFLAGS +=	\
	-I$(ADJUNCT_PROTO)/usr/include/python$(PYVER)$(PYSUFFIX)
CPPFLAGS +=	-I../../../uts/common/fs/zfs
CPPFLAGS +=	-I../../../common/zfs

# needs work
SMOFF += all_func_returns

# Some python headers from the build system have indentation that triggers a
# smatch warning.
pics/ioctl.o := SMOFF += indenting

.KEEP_STATE:

all:

$(ROOTLIBDIR)/%: %
	$(INS.pyfile)

$(ROOTLIBDIR)/%: ../common/%
	$(INS.pyfile)

$(ROOTLIBDIR64)/%: %
	$(INS.pyfile)

$(ROOTLIBDIR64)/%: ../common/%
	$(INS.pyfile)

include ../../Makefile.targ
