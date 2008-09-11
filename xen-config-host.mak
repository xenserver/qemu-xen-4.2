QEMU_ROOT ?= .
XEN_ROOT ?= $(QEMU_ROOT)/../xen-unstable.hg
include $(XEN_ROOT)/tools/Rules.mk

ifdef CONFIG_STUBDOM
TARGET_DIRS=i386-stubdom
else
TARGET_DIRS=i386-dm
endif

-include $(QEMU_ROOT)/xen-hooks.mak
