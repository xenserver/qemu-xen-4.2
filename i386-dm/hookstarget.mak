IOEMU_OS=$(shell uname -s)

install-hook:
	$(INSTALL_DIR) "$(DESTDIR)/$(bindir)"
	$(INSTALL_DIR) "$(DESTDIR)/$(configdir)"
	$(INSTALL_PROG) qemu-ifup-$(IOEMU_OS) "$(DESTDIR)/$(configdir)/qemu-ifup"
