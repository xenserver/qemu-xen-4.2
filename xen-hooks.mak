CPPFLAGS+= -I$(XEN_ROOT)/tools/libxc
CPPFLAGS+= -I$(XEN_ROOT)/tools/xenstore
CPPFLAGS+= -I$(XEN_ROOT)/tools/include

SSE2 := $(call cc-option,$(CC),-msse2,)
ifeq ($(SSE2),-msse2)
CFLAGS += -DUSE_SSE2=1 -msse2
endif

QEMU_PROG=qemu-dm

CFLAGS += -Wno-unused -Wno-declaration-after-statement \
 -Wno-pointer-sign

LIBS += -L$(XEN_ROOT)/tools/libxc -lxenctrl -lxenguest
LIBS += -L$(XEN_ROOT)/tools/xenstore -lxenstore

LDFLAGS := $(CFLAGS) $(LDFLAGS)

OBJS += piix4acpi.o
OBJS += xenstore.o
OBJS += xen_platform.o
OBJS += xen_machine_fv.o
OBJS += xen_machine_pv.o
OBJS += xenfb.o
OBJS += xen_console.o
OBJS += xen_machine_fv.o

ifdef CONFIG_PASSTHROUGH
OBJS+= pass-through.o
endif

BAD_OBJS += loader.o monitor.o gdbstub.o acpi.o apic.o
BAD_OBJS += vmmouse.o vmport.o tcg* helper.o disas.o

OBJS := $(filter-out $(BAD_OBJS), $(OBJS))
