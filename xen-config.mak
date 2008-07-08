QEMU_ROOT ?= ..
XEN_ROOT ?= $(QEMU_ROOT)/../xen-unstable.hg

include ../config-host.mak

TARGET_ARCH=i386
CONFIG_SOFTMMU=yes

CFLAGS += -I$(QEMU_ROOT)/hw

bindir = ${prefix}/${LIBLEAFDIR}/xen/bin
