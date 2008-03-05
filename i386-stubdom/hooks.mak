include ../xen-hooks.mak

OBJS += block-vbd.o
OBJS += tpm_tis.o

QEMU_STUBDOM= libqemu.a

PROGS=$(QEMU_STUBDOM)

$(QEMU_STUBDOM): $(OBJS)
	$(AR) rcs $@ $^
