CPPFLAGS += -DHAS_AUDIO
QEMU_PROG=qemu-dm

OBJS += xen_blktap.o

include ../xen-hooks.mak
