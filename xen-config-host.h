#ifndef XEN_CONFIG_HOST_H
#define XEN_CONFIG_HOST_H

#ifdef CONFIG_STUBDOM
#undef CONFIG_AIO
#define NO_UNIX_SOCKETS 1
#define NO_BLUETOOTH_PASSTHROUGH 1
#endif

#define CONFIG_DM
#define CONFIG_XEN

extern char domain_name[64];
extern int domid, domid_backend;

#include <errno.h>
#include <stdbool.h>

#include "xenctrl.h"
#include "xs.h"
#ifndef CONFIG_STUBDOM
#include "blktaplib.h"
#endif

#define BIOS_SIZE ((256 + 64) * 1024)

#undef CONFIG_GDBSTUB

void main_loop_prepare(void);

extern int xc_handle;
extern int xen_pause_requested;
extern int vcpus;

#define DEFAULT_NETWORK_SCRIPT "/etc/xen/qemu-ifup"
#define DEFAULT_NETWORK_DOWN_SCRIPT "/etc/xen/qemu-ifdown"

#ifdef CONFIG_STUBDOM
#define bdrv_host_device bdrv_raw
#endif
struct CharDriverState;
void xenstore_store_serial_port_info(int i, struct CharDriverState *chr,
				     const char *devname);

extern unsigned long *logdirty_bitmap;
extern unsigned long logdirty_bitmap_size;

#ifdef CONFIG_STUBDOM
#undef HAVE_IOVEC
#endif

#endif /*XEN_CONFIG_HOST_H*/
