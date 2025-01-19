UTSBASE=	../..

MODULE=		entb
OBJECTS=	$(ENTB_OBJS:%=$(OBJS_DIR)/%)
ROOTMODULE=	$(ROOT_DRV_DIR)/$(MODULE)
CONF_SRCDIR=	$(UTSBASE)/common/io/ntb/entb

include		$(UTSBASE)/intel/Makefile.intel

ALL_TARGET=	$(BINARY) $(SRC_CONFILE)
INSTALL_TARGET=	$(BINARY) $(ROOTMODULE) $(ROOT_CONFFILE)

LDFLAGS +=	-Nmisc/mac -Nmisc/ntb -Ndrv/ntb_transport

.KEEP_STATE:

def:		$(DEF_DEPS)

all:		$(ALL_DEPS)

clean:		$(CLEAN_DEPS)

clobber:	$(CLOBBER_DEPS)

install:	$(INSTALL_DEPS)

include		$(UTSBASE)/intel/Makefile.targ
