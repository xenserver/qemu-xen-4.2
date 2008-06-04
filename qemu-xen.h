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
#endif

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

#endif /*QEMU_XEN_H*/
