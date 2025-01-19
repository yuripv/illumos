UTSBASE=	../..

MODULE=		ntb_hw_psx
OBJECTS=	$(NTB_HW_PSX_OBJS:%=$(OBJS_DIR)/%)
ROOTMODULE=	$(ROOT_DRV_DIR)/$(MODULE)
CONF_SRCDIR=	$(UTSBASE)/common/io/ntb/ntb_hw

include		$(UTSBASE)/intel/Makefile.intel

ALL_TARGET=	$(BINARY) $(CONFMOD)
INSTALL_TARGET=	$(BINARY) $(ROOTMODULE) $(ROOT_CONFFILE)

LDFLAGS +=	-Nmisc/ntb -Nmisc/pcie

.KEEP_STATE:

def:		$(DEF_DEPS)

all:		$(ALL_DEPS)

clean:		$(CLEAN_DEPS)

clobber:	$(CLOBBER_DEPS)

install:	$(INSTALL_DEPS)

include		$(UTSBASE)/intel/Makefile.targ
