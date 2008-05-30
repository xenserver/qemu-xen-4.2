QEMU_ROOT ?= .
XEN_ROOT ?= $(QEMU_ROOT)/../xen-unstable.hg
include $(XEN_ROOT)/tools/Rules.mk

TARGET_DIRS=i386-dm

include xen-hooks.mak
