UTSBASE=	../..

SRCDIR=		$(UTSBASE)/common/io/ntb

MODULE=		ntb_transport
OBJECTS=	$(NTB_OBJS:%=$(OBJS_DIR)/%)
ROOTMODULE=	$(ROOT_DRV_DIR)/$(MODULE)
CONF_SRCDIR=	$(SRCDIR)

NTBFILES	= names_to_endpoint

ROOTETCNTB=	$(ROOT)/etc/ntb_transport
ROOTNTBFILES=	$(NTBFILES:%=$(ROOTETCNTB)/%)

$(ROOTNTBFILES)	:= FILEMODE= $(CFILEMODE)

include		$(UTSBASE)/intel/Makefile.intel

CFLAGS +=	-I$(UTSBASE)/common/io/ntb

ALL_TARGET=	$(BINARY) $(CONFMOD)
INSTALL_TARGET=	$(BINARY) $(ROOTMODULE) $(ROOT_CONFFILE)

LDFLAGS	+=	-Nmisc/dcopy -Nmisc/ntb_svc

.KEEP_STATE:

def:		$(DEF_DEPS)

all:		$(ALL_DEPS)

clean:		$(CLEAN_DEPS)

clobber:	$(CLOBBER_DEPS)

install:	$(INSTALL_DEPS) $(ROOTNTBFILES)

$(ROOTETCNTB):
		$(INS.dir)

$(ROOTETCNTB)/%: $(ROOTETCNTB) $(SRCDIR)/%
		$(INS.file)

include		$(UTSBASE)/intel/Makefile.targ
