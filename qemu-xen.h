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

#endif /*QEMU_XEN_H*/
