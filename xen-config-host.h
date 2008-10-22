#ifdef __MINIOS__
#define CONFIG_STUBDOM
#define NO_AIO 1
#define NO_UNIX_SOCKETS 1
#endif

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

void xenstore_parse_domain_config(int domid);
void xenstore_read_vncpasswd(int domid, char *pwbuf, size_t pwbuflen);
#ifdef CONFIG_STUBDOM
extern struct BlockDriver bdrv_vbd;
#endif
struct CharDriverState;
void xenstore_store_serial_port_info(int i, struct CharDriverState *chr,
				     const char *devname);

extern unsigned long *logdirty_bitmap;
extern unsigned long logdirty_bitmap_size;
