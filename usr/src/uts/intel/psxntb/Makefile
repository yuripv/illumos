UTSBASE=	../..

MODULE=		psxntb
OBJECTS=	$(PSXNTB_OBJS:%=$(OBJS_DIR)/%)
ROOTMODULE=	$(ROOT_DRV_DIR)/$(MODULE)
CONF_SRCDIR=	$(UTSBASE)/common/io/psxntb

include		$(UTSBASE)/intel/Makefile.intel

ALL_TARGET= $(BINARY) $(CONFMOD)
INSTALL_TARGET	= $(BINARY) $(ROOTMODULE) $(ROOT_CONFFILE)

CFLAGS +=	-I$(UTSBASE)/common/io/psxntb

LDFLAGS +=	-Nmisc/pcie -Nmisc/ntb_svc

.KEEP_STATE:

def:		$(DEF_DEPS)

all:		$(ALL_DEPS)

clean:		$(CLEAN_DEPS)

clobber:	$(CLOBBER_DEPS)

install:	$(INSTALL_DEPS)

include		$(UTSBASE)/intel/Makefile.targ
