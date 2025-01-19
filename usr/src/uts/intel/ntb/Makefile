UTSBASE=	../..

MODULE=		ntb
OBJECTS=	$(NTB_OBJS:%=$(OBJS_DIR)/%)
ROOTMODULE=	$(ROOT_MISC_DIR)/$(MODULE)

include		$(UTSBASE)/intel/Makefile.intel

ALL_TARGET=	$(BINARY)
INSTALL_TARGET=	$(BINARY) $(ROOTMODULE)

.KEEP_STATE:

def:		$(DEF_DEPS)

all:		$(ALL_DEPS)

clean:		$(CLEAN_DEPS)

clobber:	$(CLOBBER_DEPS)

install:	$(INSTALL_DEPS)

include		$(UTSBASE)/intel/Makefile.targ
