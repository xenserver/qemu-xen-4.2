
install-hook:
	$(INSTALL_DIR) "$(DESTDIR)/$(bindir)"
	$(INSTALL_DIR) "$(DESTDIR)/$(configdir)"
	$(INSTALL_PROG) qemu-ifup "$(DESTDIR)/$(configdir)/"
