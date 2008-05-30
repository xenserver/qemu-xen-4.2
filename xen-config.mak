include ../config-host.mak
XEN_ROOT ?= ../../xen-unstable.hg
include $(XEN_ROOT)/tools/Rules.mk

TARGET_ARCH=i386
CONFIG_SOFTMMU=yes
