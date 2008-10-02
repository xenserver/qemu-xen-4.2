#ifndef QEMU_XEN_H
#define QEMU_XEN_H

/* xen_machine_fv.c */

#if (defined(__i386__) || defined(__x86_64__)) && !defined(QEMU_TOOL)
#define MAPCACHE
uint8_t *qemu_map_cache(target_phys_addr_t phys_addr);
void     qemu_invalidate_map_cache(void);
#else 
#define qemu_invalidate_map_cache() ((void)0)
#endif

#define mapcache_lock()   ((void)0)
#define mapcache_unlock() ((void)0)

/* helper2.c */
extern long time_offset;
void timeoffset_get(void);

/* xen_platform.c */
#ifndef QEMU_TOOL
void pci_xen_platform_init(PCIBus *bus);
void xen_vga_stolen_vram_addr(uint64_t vram_addr);
void xen_vga_populate_vram(uint64_t vram_addr);
void xen_vga_vram_map(uint64_t vram_addr, int copy);
#endif


void destroy_hvm_domain(void);

#ifdef __ia64__
static inline void xc_domain_shutdown_hook(int xc_handle, uint32_t domid)
{
        xc_ia64_save_to_nvram(xc_handle, domid);
}
void handle_buffered_pio(void);
#else
#define xc_domain_shutdown_hook(xc_handle, domid)       do {} while (0)
#define handle_buffered_pio()                           do {} while (0)
#endif

/* xenstore.c */
void xenstore_parse_domain_config(int domid);
int xenstore_fd(void);
void xenstore_process_event(void *opaque);
void xenstore_record_dm(char *subpath, char *state);
void xenstore_record_dm_state(char *state);
void xenstore_check_new_media_present(int timeout);
void xenstore_write_vncport(int vnc_display);
void xenstore_read_vncpasswd(int domid, char *pwbuf, size_t pwbuflen);
void xenstore_write_vslots(char *vslots);

int xenstore_domain_has_devtype_danger(struct xs_handle *handle,
                                const char *devtype);
char **xenstore_domain_get_devices_danger(struct xs_handle *handle,
                                   const char *devtype, unsigned int *num);
char *xenstore_read_hotplug_status(struct xs_handle *handle,
                                   const char *devtype,
				   const char *inst_danger);
char *xenstore_backend_read_variable(struct xs_handle *,
                                     const char *devtype,
				     const char *inst_danger,
                                     const char *var);
int xenstore_subscribe_to_hotplug_status(struct xs_handle *handle,
                                         const char *devtype,
                                         const char *inst,
                                         const char *token);
int xenstore_unsubscribe_from_hotplug_status(struct xs_handle *handle,
                                             const char *devtype,
                                             const char *inst,
                                             const char *token);

 /* `danger' means that this parameter, variable or function refers to
  * an area of xenstore which is writeable by the guest and thus must
  * not be trusted by qemu code.  For variables containing xenstore
  * paths, `danger' can mean that both the path refers to a
  * guest-writeable area (so that data read from there is dangerous
  * too) and that path's value was read from somewhere controlled by
  * the guest (so writing to this path is not safe either).
  */
 /* When we're stubdom we don't mind doing as our domain tells us to,
  * at least when it comes to running our own frontends
  */

int xenstore_vm_write(int domid, char *key, char *val);
char *xenstore_vm_read(int domid, char *key, unsigned int *len);

/* xenfbfront.c */
int xenfb_pv_display_init(DisplayState *ds);
int xenfb_pv_display_start(void *vram_start);
int xenfb_connect_vkbd(const char *path);
int xenfb_connect_vfb(const char *path);

int has_tpm_device_danger(void);

#endif /*QEMU_XEN_H*/
