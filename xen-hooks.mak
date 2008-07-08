CPPFLAGS+= -I$(XEN_ROOT)/tools/libxc
CPPFLAGS+= -I$(XEN_ROOT)/tools/xenstore
CPPFLAGS+= -I$(XEN_ROOT)/tools/include
CPPFLAGS+= -I$(XEN_ROOT)/tools/blktap/lib

SSE2 := $(call cc-option,$(CC),-msse2,)
ifeq ($(SSE2),-msse2)
CFLAGS += -DUSE_SSE2=1 -msse2
endif

QEMU_PROG=qemu-dm

CFLAGS += -Wno-unused -Wno-declaration-after-statement

ifeq (,$(shell $(CC) -Wno-pointer-sign -E - </dev/null >/dev/null || echo x))
CFLAGS += -Wno-pointer-sign
endif 

CFLAGS += $(CMDLINE_CFLAGS)

LIBS += -L$(XEN_ROOT)/tools/libxc -lxenctrl -lxenguest
LIBS += -L$(XEN_ROOT)/tools/xenstore -lxenstore
LIBS += -L$(XEN_ROOT)/tools/blktap/lib -lblktap

LDFLAGS := $(CFLAGS) $(LDFLAGS)

OBJS += piix4acpi.o
OBJS += xenstore.o
OBJS += xen_platform.o
OBJS += xen_machine_fv.o
OBJS += xen_machine_pv.o
OBJS += xenfb.o
OBJS += xen_console.o
OBJS += xen_machine_fv.o
OBJS += xen_blktap.o
OBJS += exec-dm.o
OBJS += pci_emulation.o

ifdef CONFIG_STUBDOM
CONFIG_PASSTHROUGH=1
OBJS += xenfbfront.o
endif

ifdef CONFIG_PASSTHROUGH
OBJS+= pass-through.o
endif

BAD_OBJS += gdbstub.o acpi.o apic.o
BAD_OBJS += vmmouse.o vmport.o tcg* helper.o

OBJS := $(filter-out $(BAD_OBJS), $(OBJS))

EXESUF=-xen

datadir := $(subst qemu,xen/qemu,$(datadir))
docdir :=  $(subst qemu,xen/qemu,$(docdir))
mandir :=  $(subst share/man,share/xen/man,$(mandir))

configdir := /etc/xen
